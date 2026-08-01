# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
from pathlib import Path

import pytest
import rdflib

from motion_spec.introspection.archive import (
    ArchiveError,
    create_archive_manifest,
    sha256_file,
    _validate_runtime_shacl,
    verify_manifest,
)
from motion_spec.introspection.provenance import prov_uri, rec_run_lifecycle
from motion_spec.introspection import replay
from motion_spec.introspection.replay import decode_frames, summarize, validate_header
from motion_spec.introspection.runtime_graph import write_runtime_ttl
from motion_spec_dsl.rdf_parser.vocab import APP
from support import _provenance, _schema, _source_tree

REC = rdflib.Namespace("https://secorolab.github.io/metamodels/rec#")
PROV = rdflib.Namespace("http://www.w3.org/ns/prov#")
QUDT = rdflib.Namespace("http://qudt.org/schema/qudt/")


def _rec_entity_path(graph, label: str) -> str:
    """The archive-relative path REC recorded for the entity carrying `label`."""
    entity = next(e for e, value in graph.subject_objects(REC.label) if str(value) == label)
    return str(graph.value(graph.value(entity, PROV.atLocation), REC.path))

def test_archive_replay_and_runtime_ttl_are_self_contained(tmp_path: Path) -> None:
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "copied-run"

    manifest = create_archive_manifest(
        run_dir,
        source_dir=source,
        run_id="run-test",
    )

    assert "log_producer_executable" not in manifest["files"]
    assert manifest["files"]["rec"] == "rec.ld.json"
    assert manifest["files"]["frame_log_health"] == "logs/frame_log.pb.health.json"
    assert manifest["files"]["frame_log_proto"] == "contract/frame_log.proto"
    assert "frame_layout" not in manifest["files"]
    assert manifest["files"]["dsl_provenance"] == "provenance/dsl.ld.json"
    assert "rec" not in manifest
    assert manifest["files"]["controller"] == "controller/source"
    assert "artifacts" not in manifest
    # The archived proto is the generated semantic one (copied from source), not a static file.
    assert "double q0 = 3000;" in (run_dir / "contract" / "frame_log.proto").read_text()
    assert verify_manifest(run_dir)["run_id"] == "run-test"
    header = validate_header(run_dir / "logs" / "frame_log.pb", _schema())
    assert header["producer_agent_id"] == "agent:controller_process"

    frames = decode_frames(run_dir / "logs" / "frame_log.pb")
    assert frames[0]["step"] == 7
    assert frames[0]["quantities"] == {"q0": 42.0}
    assert "frames      1" in summarize(run_dir / "logs" / "frame_log.pb")
    assert "dropped 0" in summarize(run_dir / "logs" / "frame_log.pb")
    assert decode_frames(run_dir)[0]["step"] == 7
    assert f"archive     {run_dir}" in summarize(run_dir)
    assert replay.main([str(run_dir), "--verify"]) == 0
    with pytest.raises(ArchiveError, match="does not exist"):
        summarize(run_dir / "missing")

    runtime_ttl = write_runtime_ttl(run_dir, frames)
    assert runtime_ttl.exists()
    verify_manifest(run_dir)

    # REC records the archive as a PROV graph: lifecycle is an rdf:type on the run, and an
    # entity's role is its rec:label. See metamodels rec.shacl.ttl (RunExecutionShape).
    rec_graph = rdflib.Graph().parse(run_dir / "rec.ld.json", format="json-ld")
    assert rec_run_lifecycle(rec_graph)["status"] == "COMPLETED"
    labels = {str(value) for value in rec_graph.objects(None, REC.label)}
    assert {"frame_log", "frame_log_health", "runtime_ttl"} <= labels
    runtime_entity = next(
        entity
        for entity, label in rec_graph.subject_objects(REC.label)
        if str(label) == "runtime_ttl"
    )
    assert str(rec_graph.value(runtime_entity, REC.sha256)) == sha256_file(runtime_ttl)
    health = next(
        entity
        for entity, value in rec_graph.subject_objects(REC.label)
        if str(value) == "frame_log_health"
    )
    assert (
        health,
        rdflib.URIRef("http://www.w3.org/ns/prov#wasGeneratedBy"),
        rdflib.URIRef(prov_uri("activity:controller_execution")),
    ) in rec_graph
    assert (
        rdflib.URIRef(prov_uri("activity:runtime_ttl_recovery")),
        rdflib.RDF.type,
        rdflib.URIRef("http://www.w3.org/ns/prov#Activity"),
    ) in rec_graph
    # rec references bundle contents by archive-relative path (portable, no machine path).
    assert _rec_entity_path(rec_graph, "dsl_provenance") == "provenance/dsl.ld.json"
    assert _rec_entity_path(rec_graph, "runtime_ttl") == "runtime/runtime.ttl"
    metrics = {
        str(rec_graph.value(metric, REC.label) or metric).rsplit("/", 1)[0].rsplit("metric/", 1)[-1]: (
            rec_graph.value(metric, QUDT.value)
        )
        for metric in rec_graph.objects(None, REC.metrics)
    }
    assert metrics["frame_log_attempted_frames"].toPython() == 1
    assert metrics["frame_log_written_frames"].toPython() == 1
    assert metrics["frame_log_dropped_frames"].toPython() == 0
    assert metrics["frame_log_complete"].toPython() == 1


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
                "import": ["sub.ld.json"],
                "iri-map": {"https://secorolab.github.io/": {"path": "models/"}},
            }
        ],
    }


def test_archive_vendors_imported_model_graph_and_verify_catches_dangling(tmp_path: Path) -> None:
    # The app manifest imports a model graph; the archive must vendor it next to
    # model/model.ld.json so the import resolves offline, and verify must reject an
    # archive where that imported graph is missing.
    source = _source_tree(tmp_path / "source")
    (source / "model.ld.json").write_text(json.dumps(_importing_manifest(), indent=4))
    (source / "sub.ld.json").write_text(
        json.dumps(
            {
                "@context": {"prov": "http://www.w3.org/ns/prov#"},
                "@graph": [{"@id": "https://example.test/x", "@type": "prov:Entity"}],
            }
        )
    )
    run_dir = tmp_path / "run"

    manifest = create_archive_manifest(run_dir, source_dir=source, run_id="run-test")
    assert manifest["files"]["model_imports"] == ["model/sub.ld.json"]
    assert (run_dir / "model" / "sub.ld.json").is_file()
    assert "artifacts" not in manifest
    assert verify_manifest(run_dir)["run_id"] == "run-test"

    (run_dir / "model" / "sub.ld.json").unlink()
    with pytest.raises(ArchiveError, match="model/sub.ld.json: missing"):
        verify_manifest(run_dir)


def test_archive_is_provenance_complete_and_relative(tmp_path: Path) -> None:
    # The DSL's authored source (referenced by dsl.ld.json) is vendored into source/ and
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
    manifest_doc["@graph"][0]["import"] = ["sub.ld.json", "provenance/dsl.ld.json"]
    (source / "model.ld.json").write_text(json.dumps(manifest_doc, indent=4))
    (source / "sub.ld.json").write_text(
        json.dumps({"@context": {"prov": "http://www.w3.org/ns/prov#"}, "@graph": []})
    )
    # dsl.ld.json references the authored source -> must be vendored + rewritten.
    dsl = _provenance()
    dsl["@graph"].append(
        {"@id": "https://example.test/entity/src", "@type": "Entity", "atLocation": authored.resolve().as_uri()}
    )
    (source / "provenance" / "dsl.ld.json").write_text(json.dumps(dsl, indent=4))
    # motion-spec.ld.json points at a vendor asset -> must be left alone, not archived.
    prov = _provenance()
    prov["@graph"].append(
        {"@id": "https://example.test/agent/robot", "@type": "Agent", "atLocation": vendor.resolve().as_uri()}
    )
    (source / "provenance.ld.json").write_text(json.dumps(prov, indent=4))
    run_dir = tmp_path / "run"

    manifest = create_archive_manifest(run_dir, source_dir=source, run_id="run-test")

    # Authored source vendored under source/, tracked, reachable.
    assert manifest["files"]["sources"] == ["source/model.robmot"]
    assert (run_dir / "source" / "model.robmot").is_file()
    assert "artifacts" not in manifest

    # Vendor asset NOT archived; its reference left untouched.
    assert not (run_dir / "source" / "gen3.xml").exists()
    codegen = rdflib.Graph().parse(
        run_dir / "provenance" / "motion-spec.ld.json", format="json-ld"
    )
    robot = rdflib.URIRef("https://example.test/agent/robot")
    assert codegen.value(robot, PROV.atLocation) == rdflib.URIRef(vendor.resolve().as_uri())

    # dsl provenance imported but not duplicated under model/.
    assert set(manifest["files"]["model_imports"]) == {
        "model/sub.ld.json",
        "provenance/dsl.ld.json",
    }
    assert not (run_dir / "model" / "provenance").exists()

    # Manifest import + iri-map rewritten to resolve archive-relative.
    model = rdflib.Dataset().parse(run_dir / "model" / "model.ld.json", format="json-ld")
    assert {str(value) for _, _, value, _ in model.quads((None, APP["import"], None, None))} == {
        "https://secorolab.github.io/model/sub.ld.json",
        "https://secorolab.github.io/provenance/dsl.ld.json",
    }
    iri_root = rdflib.URIRef("https://secorolab.github.io/")
    assert next(model.quads((iri_root, APP.path, None, None)))[2] == rdflib.Literal("..")

    # Authored-source atLocation rewritten relative to the dsl provenance doc.
    dsl_out = rdflib.Graph().parse(run_dir / "provenance" / "dsl.ld.json", format="json-ld")
    src_node = rdflib.URIRef("https://example.test/entity/src")
    assert dsl_out.value(src_node, PROV.atLocation) == rdflib.URIRef(
        (run_dir / "source" / "model.robmot").resolve().as_uri()
    )

    assert verify_manifest(run_dir)["run_id"] == "run-test"


def test_manifest_hash_verification_rejects_mutation(tmp_path: Path) -> None:
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "run"
    create_archive_manifest(run_dir, source_dir=source, run_id="run-test")
    (run_dir / "contract" / "schema.json").write_text("{}")

    with pytest.raises(ArchiveError, match="schema.json: sha256 mismatch"):
        verify_manifest(run_dir)


def test_generation_owned_run_does_not_copy_static_artifacts(tmp_path: Path) -> None:
    flat = _source_tree(tmp_path / "flat")
    generation = tmp_path / "generation"
    generated = generation / "generated"
    for directory in ("contract", "model", "controller", "provenance"):
        (generated / directory).mkdir(parents=True, exist_ok=True)
    for source, target in (
        (flat / "schema.json", generated / "contract/schema.json"),
        (flat / "frame_log.proto", generated / "contract/frame_log.proto"),
        (flat / "provenance.ld.json", generated / "provenance/motion-spec.ld.json"),
        (flat / "provenance/dsl.ld.json", generated / "provenance/dsl.ld.json"),
        (flat / "model.ld.json", generated / "model/demo-app.ld.json"),
        (flat / "ir.json", generated / "model/ir.json"),
    ):
        target.write_bytes(source.read_bytes())

    run_dir = generation / "runs" / "run-1"
    manifest = create_archive_manifest(
        run_dir,
        source_dir=generated,
        run_id="run-1",
        frame_log=flat / "frame_log.pb",
    )

    assert {path.name for path in run_dir.iterdir()} == {"logs", "rec.ld.json", "manifest.json"}
    assert "provenance" not in manifest
    assert "rec" not in manifest
    assert json.dumps(manifest).count("../../generated/provenance/motion-spec.ld.json") == 1
    assert "artifacts" not in manifest


def test_runtime_shacl_rejects_unanchored_occurrence(tmp_path: Path) -> None:
    path = tmp_path / "runtime.ttl"
    path.write_text(
        """
@prefix ms-exec-trace: <https://secorolab.github.io/metamodels/motion-spec/execution-trace/> .
@prefix msrun: <https://secorolab.github.io/motion-spec/runtime/> .
@prefix prov: <http://www.w3.org/ns/prov#> .
@prefix xsd: <http://www.w3.org/2001/XMLSchema#> .

<run> a <https://secorolab.github.io/metamodels/execution-context#ExecutionContext> ;
    msrun:contractVersion 1 ;
    msrun:frameCount 1 ;
    msrun:runId "run-test" ;
    prov:wasGeneratedBy <activity> .

<event> a ms-exec-trace:EventOccurrence ;
    ms-exec-trace:event <https://example.test/E_DONE> ;
    ms-exec-trace:seq 0 .
""".lstrip()
    )

    with pytest.raises(ArchiveError, match="runtime SHACL validation failed"):
        _validate_runtime_shacl(path)
