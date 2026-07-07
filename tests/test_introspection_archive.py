# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
from pathlib import Path

import pytest

from motion_spec.introspection.archive import (
    ArchiveError,
    create_archive_manifest,
    sha256_file,
    verify_manifest,
)
from motion_spec.introspection.artifacts import prov_uri
from motion_spec.introspection.frame_layout_spec import fields_with_offsets, frame_struct
from motion_spec.introspection import replay
from motion_spec.introspection.replay import MAGIC, HEADER, decode_frames, summarize, validate_header
from motion_spec.introspection.runtime_graph import write_runtime_ttl


def _hash_doc(doc: dict) -> str:
    return __import__("hashlib").sha256(json.dumps(doc, sort_keys=True).encode()).hexdigest()[:16]


def _schema() -> dict:
    schema = {
        "schema_version": 1,
        "frame_layout_version": 1,
        "runtime_rdf_contract_version": 1,
        "generated_by": "test",
        "ir_path": "ir.json",
        "graph": "model.jsonld",
        "context": {},
        "pools": {"constraints": 1, "monitors": 1, "quantities": 1, "triggers": 2},
        "timing": {"nominal_period_ns": 1_000_000},
        "fsm": {
            "states": [{"index": 0, "id": "S_START", "uri": "https://example.test/S_START"}],
            "events": [],
            "end": 0,
        },
        "by_state": {},
        "quantities": [],
        "provenance_contexts": [{"id": "prov", "source": "src/metamodels/prov.json"}],
        "runtime_provenance": {
            "activity_id": "activity:controller_execution",
            "producer_agent_id": "agent:controller_process",
            "runtime_agent_id": "agent:runtime:mujoco",
        },
    }
    schema["schema_hash"] = _hash_doc(schema)
    return schema


def _layout(schema: dict) -> dict:
    fields, size = fields_with_offsets(schema["pools"])
    layout = {
        "frame_layout_version": 1,
        "schema_version": 1,
        "runtime_rdf_contract_version": 1,
        "pools": schema["pools"],
        "field_bytes": 8,
        "frame_size_bytes": size,
        "schema_hash": schema["schema_hash"],
        "runtime_provenance": schema["runtime_provenance"],
        "fields": fields,
    }
    layout["frame_layout_hash"] = _hash_doc(layout)
    return layout


def _provenance() -> dict:
    return {
        "@context": {
            "prov": "http://www.w3.org/ns/prov#",
            "Entity": "prov:Entity",
            "Activity": "prov:Activity",
            "Agent": "prov:Agent",
            "SoftwareAgent": "prov:SoftwareAgent",
            "used": {"@id": "prov:used", "@type": "@id"},
            "wasGeneratedBy": {"@id": "prov:wasGeneratedBy", "@type": "@id"},
            "wasAssociatedWith": {"@id": "prov:wasAssociatedWith", "@type": "@id"},
        },
        "@graph": [
            {"@id": "https://example.test/entity/schema", "@type": "Entity"},
            {
                "@id": "https://example.test/entity/run_bin",
                "@type": "Entity",
                "wasGeneratedBy": "https://example.test/activity/run",
            },
            {
                "@id": "https://example.test/activity/run",
                "@type": "Activity",
                "used": "https://example.test/entity/schema",
                "wasAssociatedWith": "https://example.test/agent/producer",
            },
            {
                "@id": "https://example.test/agent/producer",
                "@type": ["SoftwareAgent", "Agent"],
            },
        ],
    }


def _write_frame_log(path: Path, schema: dict, layout: dict) -> None:
    st, names = frame_struct(schema["pools"])
    flat = {name: 0 for name in names}
    flat.update(
        {
            "t": 1.25,
            "step": 7,
            "fsm_state": 0,
            "active_motion": -1,
            "last_event": -1,
            "state_since_t": 1.0,
            "event_t": 0.0,
            "wall_ns": 100,
            "period_ns": 1_000_000,
            "compute_ns": 25_000,
            "c0.active": 1,
            "c0.satisfied": 1,
            "m0.active": 1,
            "m0.satisfied": 1,
            "q0": 42.0,
        }
    )
    header = HEADER.pack(
        MAGIC,
        1,
        st.size,
        schema["schema_hash"].encode(),
        layout["frame_layout_hash"].encode(),
        schema["runtime_provenance"]["producer_agent_id"].encode(),
        schema["runtime_provenance"]["activity_id"].encode(),
    )
    path.write_bytes(header + st.pack(*(flat[name] for name in names)))


def _source_tree(path: Path) -> Path:
    schema = _schema()
    layout = _layout(schema)
    path.mkdir()
    (path / "schema.json").write_text(json.dumps(schema, indent=4))
    (path / "frame_layout.json").write_text(json.dumps(layout, indent=4))
    (path / "provenance.jsonld").write_text(json.dumps(_provenance(), indent=4))
    (path / "provenance").mkdir()
    (path / "provenance" / "dsl.jsonld").write_text(json.dumps(_provenance(), indent=4))
    (path / "model.jsonld").write_text(json.dumps(_provenance(), indent=4))
    (path / "ir.json").write_text(json.dumps({"id": "test-ir"}))
    (path / "headers").mkdir()
    (path / "headers" / "runtime.hpp").write_text("// generated\n")
    (path / "ref_main.cpp").write_text("// generated\n")
    _write_frame_log(path / "frame_log.bin", schema, layout)
    (path / "frame_log.bin.health.json").write_text(
        json.dumps(
            {
                "attempted_frames": 1,
                "accepted_frames": 1,
                "written_frames": 1,
                "dropped_frames": 0,
                "complete": True,
            },
            indent=4,
        )
    )
    return path


def test_archive_replay_and_runtime_ttl_are_self_contained(tmp_path: Path) -> None:
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "copied-run"

    manifest = create_archive_manifest(
        run_dir,
        source_dir=source,
        run_id="run-test",
        streams=[
            {
                "id": "sim_front",
                "kind": "camera",
                "label": "Sim front camera",
                "mode": "mp4",
                "url": "media/sim_front.mp4",
                "frame_map": "media/sim_front.frames.jsonl",
            }
        ],
    )

    assert manifest["files"]["log_producer_executable"] is None
    assert manifest["files"]["rec"] == "rec.jsonld"
    assert manifest["files"]["frame_log_health"] == "logs/frame_log.bin.health.json"
    assert manifest["files"]["dsl_provenance"] == "provenance/dsl.jsonld"
    assert manifest["rec"] == {"path": "rec.jsonld", "run_id": "run-test"}
    assert manifest["files"]["controller"] == "controller/source"
    assert "controller/source" in manifest["artifacts"]
    assert "logs/frame_log.bin.health.json" in manifest["artifacts"]
    assert "provenance/dsl.jsonld" in manifest["artifacts"]
    assert "rec.jsonld" in manifest["artifacts"]
    assert verify_manifest(run_dir)["run_id"] == "run-test"
    header = validate_header(run_dir / "logs" / "frame_log.bin", _schema(), _layout(_schema()))
    assert header["producer_agent_id"] == "agent:controller_process"

    frames = decode_frames(run_dir / "logs" / "frame_log.bin")
    assert frames[0]["step"] == 7
    assert frames[0]["quantities"] == [42.0]
    assert "frames      1" in summarize(run_dir / "logs" / "frame_log.bin")
    assert "dropped 0" in summarize(run_dir / "logs" / "frame_log.bin")
    assert decode_frames(run_dir)[0]["step"] == 7
    assert f"archive     {run_dir}" in summarize(run_dir)
    assert replay.main([str(run_dir), "--verify"]) == 0
    with pytest.raises(ArchiveError, match="does not exist"):
        summarize(run_dir / "missing")

    runtime_ttl = write_runtime_ttl(run_dir, frames)
    assert runtime_ttl.exists()
    assert verify_manifest(run_dir)["artifacts"]["runtime/runtime.ttl"]["sha256"] == sha256_file(runtime_ttl)

    rec_doc = json.loads((run_dir / "rec.jsonld").read_text())
    assert rec_doc["status"] == "COMPLETED"
    assert rec_doc["role"] == "run_execution"
    assert "run" not in rec_doc
    assert "@graph" not in rec_doc
    assert any(row["role"] == "frame_log" for row in rec_doc["artefacts"])
    assert any(
        row["role"] == "frame_log_health"
        and row["wasGeneratedBy"] == prov_uri("activity:controller_execution")
        for row in rec_doc["artefacts"]
    )
    assert any(row["role"] == "runtime_ttl" for row in rec_doc["artefacts"])
    assert any(row["role"] == "runtime_ttl_recovery" for row in rec_doc["activities"])
    # rec references bundle contents by archive-relative path (portable, no machine path).
    dsl_resource = next(row for row in rec_doc["resources"] if row["role"] == "dsl_provenance")
    assert dsl_resource["atLocation"] == "provenance/dsl.jsonld"
    runtime_artifact = next(row for row in rec_doc["artefacts"] if row["role"] == "runtime_ttl")
    assert runtime_artifact["atLocation"] == "runtime/runtime.ttl"
    metrics = {row["name"]: row["value"] for row in rec_doc["metrics"]}
    assert metrics["frame_log_attempted_frames"] == 1
    assert metrics["frame_log_written_frames"] == 1
    assert metrics["frame_log_dropped_frames"] == 0
    assert metrics["frame_log_complete"] == 1


def _importing_manifest() -> dict:
    return {
        "@context": {
            "@version": 1.1,
            "xsd": "http://www.w3.org/2001/XMLSchema#",
            "app": "https://comp-rob2b.github.io/metamodels/application/",
            "import": {
                "@id": "app:import",
                "@type": "@id",
                "@context": {"@base": "https://secorolab.github.io/"},
            },
            "iri-map": {"@id": "app:iri-map", "@container": "@id"},
            "path": {"@id": "app:path", "@type": "xsd:string"},
        },
        "@id": "https://secorolab.github.io/models/generated/",
        "@graph": [
            {
                "import": ["sub.jsonld"],
                "iri-map": {"https://secorolab.github.io/": {"path": "models/"}},
            }
        ],
    }


def test_archive_vendors_imported_model_graph_and_verify_catches_dangling(tmp_path: Path) -> None:
    # The app manifest imports a model graph; the archive must vendor it next to
    # model/model.jsonld so the import resolves offline, and verify must reject an
    # archive where that imported graph is missing.
    source = _source_tree(tmp_path / "source")
    (source / "model.jsonld").write_text(json.dumps(_importing_manifest(), indent=4))
    (source / "sub.jsonld").write_text(
        json.dumps(
            {
                "@context": {"prov": "http://www.w3.org/ns/prov#"},
                "@graph": [{"@id": "https://example.test/x", "@type": "prov:Entity"}],
            }
        )
    )
    run_dir = tmp_path / "run"

    manifest = create_archive_manifest(run_dir, source_dir=source, run_id="run-test")
    assert manifest["files"]["model_imports"] == ["model/sub.jsonld"]
    assert (run_dir / "model" / "sub.jsonld").is_file()
    assert manifest["artifacts"]["model/sub.jsonld"]["role"] == "imported_model_graph"
    assert verify_manifest(run_dir)["run_id"] == "run-test"

    (run_dir / "model" / "sub.jsonld").unlink()
    with pytest.raises(ArchiveError, match="model/sub.jsonld: missing"):
        verify_manifest(run_dir)


def test_archive_is_provenance_complete_and_relative(tmp_path: Path) -> None:
    # The DSL's authored source (referenced by dsl.jsonld) is vendored into source/ and
    # its atLocation rewritten relative; a vendor asset the model only points at (via
    # codegen provenance) is NOT archived. The dsl provenance is imported but not
    # duplicated; the manifest import/iri-map are rewritten to resolve inside the archive.
    source = _source_tree(tmp_path / "source")
    authored = tmp_path / "inputs" / "model.robmot"
    authored.parent.mkdir()
    authored.write_text("robot { }\n")
    vendor = tmp_path / "vendor" / "gen3.xml"
    vendor.parent.mkdir()
    vendor.write_text("<mujoco/>\n")

    manifest_doc = _importing_manifest()
    manifest_doc["@graph"][0]["import"] = ["sub.jsonld", "provenance/dsl.jsonld"]
    (source / "model.jsonld").write_text(json.dumps(manifest_doc, indent=4))
    (source / "sub.jsonld").write_text(
        json.dumps({"@context": {"prov": "http://www.w3.org/ns/prov#"}, "@graph": []})
    )
    # dsl.jsonld references the authored source -> must be vendored + rewritten.
    dsl = _provenance()
    dsl["@graph"].append(
        {"@id": "https://example.test/entity/src", "@type": "Entity", "atLocation": authored.resolve().as_uri()}
    )
    (source / "provenance" / "dsl.jsonld").write_text(json.dumps(dsl, indent=4))
    # codegen.jsonld points at a vendor asset -> must be left alone, not archived.
    prov = _provenance()
    prov["@graph"].append(
        {"@id": "https://example.test/agent/robot", "@type": "Agent", "atLocation": vendor.resolve().as_uri()}
    )
    (source / "provenance.jsonld").write_text(json.dumps(prov, indent=4))
    run_dir = tmp_path / "run"

    manifest = create_archive_manifest(run_dir, source_dir=source, run_id="run-test")

    # Authored source vendored under source/, tracked, reachable.
    assert manifest["files"]["sources"] == ["source/model.robmot"]
    assert (run_dir / "source" / "model.robmot").is_file()
    assert manifest["artifacts"]["source/model.robmot"]["role"] == "source_model"

    # Vendor asset NOT archived; its reference left untouched.
    assert not (run_dir / "source" / "gen3.xml").exists()
    codegen = json.loads((run_dir / "provenance" / "codegen.jsonld").read_text())
    robot = next(n for n in codegen["@graph"] if n.get("@id") == "https://example.test/agent/robot")
    assert robot["atLocation"] == vendor.resolve().as_uri()

    # dsl provenance imported but not duplicated under model/.
    assert manifest["files"]["model_imports"] == ["model/sub.jsonld", "provenance/dsl.jsonld"]
    assert not (run_dir / "model" / "provenance").exists()

    # Manifest import + iri-map rewritten to resolve archive-relative.
    model = json.loads((run_dir / "model" / "model.jsonld").read_text())
    assert model["@graph"][0]["import"] == ["model/sub.jsonld", "provenance/dsl.jsonld"]
    assert model["@graph"][0]["iri-map"] == {"https://secorolab.github.io/": {"path": ".."}}

    # Authored-source atLocation rewritten relative to the dsl provenance doc.
    dsl_out = json.loads((run_dir / "provenance" / "dsl.jsonld").read_text())
    src_node = next(n for n in dsl_out["@graph"] if n.get("@id") == "https://example.test/entity/src")
    assert src_node["atLocation"] == "../source/model.robmot"

    assert verify_manifest(run_dir)["run_id"] == "run-test"


def test_manifest_hash_verification_rejects_mutation(tmp_path: Path) -> None:
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "run"
    create_archive_manifest(run_dir, source_dir=source, run_id="run-test")
    (run_dir / "contract" / "schema.json").write_text("{}")

    with pytest.raises(ArchiveError, match="schema.json: sha256 mismatch"):
        verify_manifest(run_dir)


def test_manifest_rejects_incomplete_stream_descriptor(tmp_path: Path) -> None:
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "run"
    create_archive_manifest(run_dir, source_dir=source, run_id="run-test", streams=[{"id": "cam"}])

    with pytest.raises(ArchiveError, match="missing kind"):
        verify_manifest(run_dir)
