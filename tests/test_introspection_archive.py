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
from motion_spec.introspection.frame_layout_spec import fields_with_offsets, frame_struct
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
    (path / "model.jsonld").write_text(json.dumps(_provenance(), indent=4))
    (path / "ir.json").write_text(json.dumps({"id": "test-ir"}))
    (path / "headers").mkdir()
    (path / "headers" / "runtime.hpp").write_text("// generated\n")
    (path / "ref_main.cpp").write_text("// generated\n")
    _write_frame_log(path / "frame_log.bin", schema, layout)
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
    assert manifest["files"]["rec"] == "rec.json"
    assert manifest["rec"] == {"path": "rec.json", "run_id": "run-test"}
    assert manifest["files"]["controller"] == "controller/source"
    assert "controller/source" in manifest["artifacts"]
    assert "rec.json" in manifest["artifacts"]
    assert verify_manifest(run_dir)["run_id"] == "run-test"
    header = validate_header(run_dir / "logs" / "frame_log.bin", _schema(), _layout(_schema()))
    assert header["producer_agent_id"] == "agent:controller_process"

    frames = decode_frames(run_dir / "logs" / "frame_log.bin")
    assert frames[0]["step"] == 7
    assert frames[0]["quantities"] == [42.0]
    assert "frames      1" in summarize(run_dir / "logs" / "frame_log.bin")

    runtime_ttl = write_runtime_ttl(run_dir, frames)
    assert runtime_ttl.exists()
    assert verify_manifest(run_dir)["artifacts"]["runtime/runtime.ttl"]["sha256"] == sha256_file(runtime_ttl)

    rec_doc = json.loads((run_dir / "rec.json").read_text())
    assert rec_doc["run"]["status"] == "COMPLETED"
    assert any(row["role"] == "frame_log" for row in rec_doc["artefacts"])
    assert any(row["role"] == "runtime_ttl" for row in rec_doc["artefacts"])
    assert any(row["role"] == "runtime_ttl_recovery" for row in rec_doc["activities"])


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
