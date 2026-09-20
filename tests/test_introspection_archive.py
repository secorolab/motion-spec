# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
import urllib.parse
from pathlib import Path

import pytest
import rdflib

from motion_spec.introspection.archive import (
    ArchiveError,
    create_archive_manifest,
    sha256_file,
    verify_manifest,
)
from motion_spec.introspection.provenance import (
    EXECUTION_DOCUMENT,
    GENERATION_DOCUMENT,
    GRAPH_DSL,
    prov_uri,
    read_generation_dataset,
    rec_document,
    rec_run_lifecycle_from_file,
)
from rec import State, Verdict
from motion_spec.introspection import replay
from motion_spec.introspection.replay import decode_frames, summarize, validate_header
from motion_spec_dsl.rdf_parser.vocab import APP
from support import _provenance, _schema, _source_tree, _start_run, _write_frame_log

REC = rdflib.Namespace("https://secorolab.github.io/metamodels/rec#")
PROV = rdflib.Namespace("http://www.w3.org/ns/prov#")
PROV_EXT = rdflib.Namespace("https://secorolab.github.io/metamodels/prov#")
QUDT = rdflib.Namespace("http://qudt.org/schema/qudt/")
SPDX = rdflib.Namespace("http://spdx.org/rdf/terms#")


def _rec_entity_location(graph, label: str, run_dir: Path) -> str:
    """Where in the archive REC put the entity carrying `label`.

    Stored relative, so the bundle moves; a parse resolves it against the document it sits in.
    """
    entity = next(e for e, value in graph.subject_objects(rdflib.RDFS.label) if str(value) == label)
    location = str(graph.value(entity, PROV.atLocation))
    return str(Path(urllib.parse.urlparse(location).path).relative_to(run_dir.resolve()))


def _executable(tmp_path: Path) -> Path:
    """What a run ran: the one input a manifest still names after the run is over."""
    path = tmp_path / "main"
    path.write_text("binary\n")
    return path


def _archived(tmp_path: Path, run_dir: Path, source: Path, **manifest) -> dict:
    """A run catalogued as the runner catalogues it, then archived."""
    executable = _executable(tmp_path)
    _start_run(run_dir, source, executable)
    return create_archive_manifest(
        run_dir, source_dir=source, run_id="run-test", log_producer_executable=executable, **manifest
    )


def test_archive_and_replay_are_self_contained(tmp_path: Path) -> None:
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "copied-run"

    manifest = _archived(tmp_path, run_dir, source)

    assert manifest["files"]["rec"] == "run-test.ld.json"
    assert manifest["files"]["execution"] == EXECUTION_DOCUMENT
    assert manifest["files"]["frame_log_health"] == "logs/frame_log.pb.health.json"
    assert manifest["files"]["frame_log_proto"] == "contract/frame_log.proto"
    assert "frame_layout" not in manifest["files"]
    # One provenance document, not one per tool.
    assert manifest["files"]["provenance"] == GENERATION_DOCUMENT
    assert "dsl_provenance" not in manifest["files"]
    assert "runtime_ttl" not in manifest["files"]
    assert "rec" not in manifest
    assert manifest["files"]["controller"] == "controller/source"
    assert "artifacts" not in manifest
    # The archived proto is the generated semantic one (copied from source), not a static file.
    assert "double q0 = 3000;" in (run_dir / "contract" / "frame_log.proto").read_text()
    assert verify_manifest(run_dir)["run_id"] == "run-test"
    # The header states the contract the log was written against and nothing about the run.
    assert validate_header(run_dir / "logs" / "frame_log.pb") == {
        "schema_hash": _schema()["schema_hash"]
    }

    assert decode_frames(run_dir / "logs" / "frame_log.pb")[0]["step"] == 7
    assert decode_frames(run_dir / "logs" / "frame_log.pb")[0]["quantities"] == {"q0": 42.0}
    assert "frames      1" in summarize(run_dir / "logs" / "frame_log.pb")
    assert "dropped 0" in summarize(run_dir / "logs" / "frame_log.pb")
    assert decode_frames(run_dir)[0]["step"] == 7
    assert f"archive     {run_dir}" in summarize(run_dir)
    assert replay.main([str(run_dir), "--verify"]) == 0
    with pytest.raises(ArchiveError, match="does not exist"):
        summarize(run_dir / "missing")


def test_rec_records_the_run_as_an_execution_of_what_it_used(tmp_path: Path) -> None:
    """One run, one node: rec types it, names what it used and hashes what it generated."""
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "run"
    _archived(tmp_path, run_dir, source)
    rec_path = rec_document(run_dir, "run-test")
    rec_graph = read_generation_dataset(rec_path)
    run = rdflib.URIRef(prov_uri("run:run-test"))

    lifecycle = rec_run_lifecycle_from_file(rec_path)
    assert (lifecycle["state"], lifecycle["verdict"]) == (State.COMPLETE, Verdict.PASSED)
    assert (run, rdflib.RDF.type, PROV_EXT.Execution) in rec_graph
    assert (run, PROV.used, None) in rec_graph
    assert (run, PROV.wasAssociatedWith, None) in rec_graph
    labels = {str(value) for value in rec_graph.objects(None, rdflib.RDFS.label)}
    assert {"frame_log", "frame_log_health", "log_producer_executable"} <= labels
    # The generation's own artifacts are the manifest's business, never the run's inputs.
    assert not {"provenance", "ir", "model"} & labels
    # Every artefact the run generated hangs off the run itself.
    log_entity = next(
        entity
        for entity, value in rec_graph.subject_objects(rdflib.RDFS.label)
        if str(value) == "frame_log"
    )
    assert (log_entity, PROV.wasGeneratedBy, run) in rec_graph
    # An integrity claim is an spdx:Checksum, and the location it is claimed for is relative.
    checksum = rec_graph.value(log_entity, SPDX.checksum)
    assert str(rec_graph.value(checksum, SPDX.checksumValue)) == sha256_file(
        run_dir / "logs" / "frame_log.pb.zst"
    )
    assert _rec_entity_location(rec_graph, "frame_log", run_dir) == "logs/frame_log.pb.zst"

    metrics = {
        str(rec_graph.value(metric, rdflib.RDFS.label)): rec_graph.value(metric, QUDT.value)
        for metric in rec_graph.subjects(rdflib.RDF.type, REC.Metric)
    }
    assert metrics["frame_log_attempted_frames"].toPython() == 1
    assert metrics["frame_log_written_frames"].toPython() == 1
    assert metrics["frame_log_dropped_frames"].toPython() == 0
    # The sixth counter the health file keeps, and one of the three `complete` is computed from.
    assert metrics["frame_log_write_errors"].toPython() == 0
    assert metrics["frame_log_complete"].toPython() == 1


def test_verify_rejects_a_file_that_no_longer_hashes_to_what_rec_recorded(tmp_path: Path) -> None:
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "run"
    _archived(tmp_path, run_dir, source)
    (run_dir / "logs" / "console.log").write_text("started\n")
    create_archive_manifest(
        run_dir, source_dir=source, run_id="run-test", log_producer_executable=_executable(tmp_path)
    )
    (run_dir / "logs" / "console.log").write_text("tampered\n")

    with pytest.raises(ArchiveError, match="sha256 mismatch"):
        verify_manifest(run_dir)


def test_replay_works_on_aborted_run_without_manifest(tmp_path: Path) -> None:
    # A run killed before the archiving step ran leaves logs/frame_log.pb but no
    # manifest.json. Replay must still read it -- the log carries its own decode contract.
    schema = _schema()
    run_dir = tmp_path / "aborted-run"
    (run_dir / "logs").mkdir(parents=True)
    _write_frame_log(run_dir / "logs" / "frame_log.pb", schema)
    assert not (run_dir / "manifest.json").exists()

    frame_log = run_dir / "logs" / "frame_log.pb"
    assert decode_frames(frame_log)[0]["step"] == 7
    assert decode_frames(run_dir)[0]["step"] == 7
    assert f"archive     {run_dir}" in summarize(frame_log)
    assert replay.main([str(frame_log), "--verify"]) == 0


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

    manifest = _archived(tmp_path, run_dir, source)
    assert manifest["files"]["model_imports"] == ["model/sub.ld.json"]
    assert (run_dir / "model" / "sub.ld.json").is_file()
    assert "artifacts" not in manifest
    assert verify_manifest(run_dir)["run_id"] == "run-test"

    (run_dir / "model" / "sub.ld.json").unlink()
    with pytest.raises(ArchiveError, match="model/sub.ld.json: missing"):
        verify_manifest(run_dir)


def test_archive_is_provenance_complete_and_relative(tmp_path: Path) -> None:
    # The DSL's authored source (referenced by the generation provenance) is vendored into
    # source/ and its atLocation rewritten relative; a vendor asset the model only points at is
    # NOT archived. The manifest import/iri-map are rewritten to resolve inside the archive.
    source = _source_tree(tmp_path / "source")
    authored = tmp_path / "inputs" / "model.robmot"
    authored.parent.mkdir()
    authored.write_text("robot { }\n")
    vendor = tmp_path / "vendor" / "gen3.xml"
    vendor.parent.mkdir()
    vendor.write_text("<mujoco/>\n")

    manifest_doc = _importing_manifest()
    (source / "model.ld.json").write_text(json.dumps(manifest_doc, indent=4))
    (source / "sub.ld.json").write_text(
        json.dumps({"@context": {"prov": "http://www.w3.org/ns/prov#"}, "@graph": []})
    )
    # The DSL's graph names the authored source (vendored + rewritten); motion-spec's names a
    # vendor asset the archive only points at.
    document = _provenance()
    document["@graph"][0]["@graph"].append(
        {
            "@id": "https://example.test/entity/asset",
            "@type": "Entity",
            "atLocation": vendor.resolve().as_uri(),
        }
    )
    document["@graph"].append(
        {
            "@id": str(GRAPH_DSL),
            "@graph": [
                {
                    "@id": "https://example.test/entity/src",
                    "@type": "Entity",
                    "atLocation": authored.resolve().as_uri(),
                }
            ],
        }
    )
    (source / GENERATION_DOCUMENT).write_text(json.dumps(document, indent=4))
    run_dir = tmp_path / "run"

    manifest = _archived(tmp_path, run_dir, source)

    # Authored source vendored under source/, tracked, reachable.
    assert manifest["files"]["sources"] == ["source/model.robmot"]
    assert (run_dir / "source" / "model.robmot").is_file()
    assert "artifacts" not in manifest

    # Vendor asset NOT archived; its reference left untouched.
    assert not (run_dir / "source" / "gen3.xml").exists()
    archived = rdflib.Dataset(default_union=True).parse(
        run_dir / GENERATION_DOCUMENT, format="json-ld"
    )
    asset = rdflib.URIRef("https://example.test/entity/asset")
    assert archived.value(asset, PROV.atLocation) == rdflib.URIRef(vendor.resolve().as_uri())
    # The authored source's location now names the archived copy.
    src_node = rdflib.URIRef("https://example.test/entity/src")
    assert archived.value(src_node, PROV.atLocation) == rdflib.URIRef(
        (run_dir / "source" / "model.robmot").resolve().as_uri()
    )
    # The document keeps its shape: the wrapper and the named graph survive the rewrite.
    written = json.loads((run_dir / GENERATION_DOCUMENT).read_text())
    assert written["schema_version"] == 1 and written["@graph"][0]["@graph"]

    # Manifest import + iri-map rewritten to resolve archive-relative.
    model = rdflib.Dataset().parse(run_dir / "model" / "model.ld.json", format="json-ld")
    assert {str(value) for _, _, value, _ in model.quads((None, APP["import"], None, None))} == {
        "https://secorolab.github.io/model/sub.ld.json"
    }
    iri_root = rdflib.URIRef("https://secorolab.github.io/")
    assert next(model.quads((iri_root, APP.path, None, None)))[2] == rdflib.Literal("..")

    assert verify_manifest(run_dir)["run_id"] == "run-test"


def test_manifest_lists_console_and_videos_only_when_present(tmp_path: Path) -> None:
    # A manifest that promises a file the run never wrote is a false record.
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "run"

    manifest = _archived(tmp_path, run_dir, source)
    executable = _executable(tmp_path)
    assert "console" not in manifest["files"]
    assert "videos" not in manifest["files"]
    assert verify_manifest(run_dir)["run_id"] == "run-test"

    (run_dir / "logs" / "console.log").write_text("started\n")
    (run_dir / "logs" / "wrist.mp4").write_bytes(b"\x00video")
    manifest = create_archive_manifest(
        run_dir, source_dir=source, run_id="run-test", log_producer_executable=executable
    )
    assert manifest["files"]["console"] == "logs/console.log"
    assert manifest["files"]["videos"] == ["logs/wrist.mp4"]
    assert verify_manifest(run_dir)["run_id"] == "run-test"
    # A video is something the run produced, so rec records it as an artefact of the run.
    rec_graph = read_generation_dataset(rec_document(run_dir, "run-test"))
    assert _rec_entity_location(rec_graph, "videos", run_dir) == "logs/wrist.mp4"


def test_logless_manifest_says_so_and_only_then_verifies_without_a_log(tmp_path: Path) -> None:
    # "recorded": false is the run stating it kept no log on purpose. A manifest that merely
    # lost its log claims nothing, and still fails.
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "run"
    # What such a run leaves instead: its console is the record, and the only thing it generated.
    (run_dir / "logs").mkdir(parents=True)
    (run_dir / "logs" / "console.log").write_text("started\n")

    manifest = _archived(tmp_path, run_dir, source, recorded=False)
    assert manifest["recorded"] is False
    assert "frame_log" not in manifest["files"]
    assert "frame_log_health" not in manifest["files"]
    assert verify_manifest(run_dir)["run_id"] == "run-test"

    del manifest["recorded"]
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=4) + "\n")
    with pytest.raises(ArchiveError, match="files.frame_log: missing"):
        verify_manifest(run_dir)


def _generation_tree(tmp_path: Path) -> tuple[Path, Path]:
    """A generation bundle and the flat source tree its artifacts were copied from."""
    flat = _source_tree(tmp_path / "flat")
    generated = tmp_path / "generation" / "generated"
    for directory in ("contract", "model", "controller"):
        (generated / directory).mkdir(parents=True, exist_ok=True)
    for source, target in (
        (flat / "frame_log.proto", generated / "contract/frame_log.proto"),
        (flat / GENERATION_DOCUMENT, generated / GENERATION_DOCUMENT),
        (flat / "model.ld.json", generated / "model/demo-app.ld.json"),
        (flat / "ir.json", generated / "model/ir.json"),
    ):
        target.write_bytes(source.read_bytes())
    return flat, generated


def test_generation_owned_run_does_not_copy_static_artifacts(tmp_path: Path) -> None:
    flat, generated = _generation_tree(tmp_path)

    run_dir = generated.parent / "runs" / "run-1"
    manifest = create_archive_manifest(
        run_dir, source_dir=generated, run_id="run-1", frame_log=flat / "frame_log.pb"
    )

    assert {path.name for path in run_dir.iterdir()} == {
        "logs",
        "run-1.ld.json",
        EXECUTION_DOCUMENT,
        "manifest.json",
    }
    assert "provenance" not in manifest
    assert "rec" not in manifest
    assert json.dumps(manifest).count(f"../../generated/{GENERATION_DOCUMENT}") == 1
    assert "artifacts" not in manifest


def test_generation_run_vendors_its_authored_source(tmp_path: Path) -> None:
    # The generated artifacts stay generation-relative, but the authored source is copied in:
    # it is kilobytes, and without it a run moved out of its generation shows no source lines.
    flat, generated = _generation_tree(tmp_path)
    (generated / "source").mkdir()
    (generated / "source" / "demo.robmot").write_text("guarded-motion (ns=demo) move {\n}\n")
    (generated / "source" / "demo.fsm").write_text("fsm demo {\n}\n")

    run_dir = generated.parent / "runs" / "run-1"
    manifest = create_archive_manifest(
        run_dir, source_dir=generated, run_id="run-1", frame_log=flat / "frame_log.pb"
    )

    assert manifest["files"]["sources"] == ["source/demo.fsm", "source/demo.robmot"]
    assert (run_dir / "source" / "demo.robmot").read_text().startswith("guarded-motion")
    # Everything the generation owns is still referenced where it lives, not duplicated.
    assert manifest["files"]["ir"] == "../../generated/model/ir.json"
    assert not (run_dir / "model").exists()


def test_verify_requires_the_run_to_be_a_recorded_execution(tmp_path: Path) -> None:
    """The gate on the rec document: the run node is an Execution that names what it used."""
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "run"
    create_archive_manifest(
        run_dir, source_dir=source, run_id="run-test", log_producer_executable=_executable(tmp_path)
    )
    rec_path = rec_document(run_dir, "run-test")
    rec_path.write_text(rec_path.read_text().replace('"Execution"', '"Activity"'))

    with pytest.raises(ArchiveError, match="missing REC provenance relationship"):
        verify_manifest(run_dir)
