# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
import urllib.parse
from pathlib import Path

import pytest
import rdflib

from motion_spec.runs.archive import (
    ArchiveError,
    create_archive_manifest,
    sha256_file,
    verify_manifest,
)
from motion_spec.runs.provenance import (
    EXECUTION_DOCUMENT,
    GENERATION_DOCUMENT,
    prov_uri,
    read_generation_dataset,
    rec_document,
    rec_run_lifecycle_from_file,
)
from rec import State, Verdict
from motion_spec.runs import replay
from motion_spec.runs.replay import decode_frames, summarize, validate_header
from support import _schema, _source_tree, _start_run, _write_frame_log

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
    if manifest.get("recorded", True):
        # The runtime writes its health report beside the log it wrote.
        (run_dir / "logs").mkdir(parents=True, exist_ok=True)
        health = "frame_log.pb.health.json"
        (run_dir / "logs" / health).write_bytes((source / health).read_bytes())
        manifest.setdefault("frame_log", source / "frame_log.pb")
    return create_archive_manifest(
        run_dir, source_dir=source, run_id="run-test", log_producer_executable=executable, **manifest
    )


def test_archive_references_its_generation_and_replays(tmp_path: Path) -> None:
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / "copied-run"

    manifest = _archived(tmp_path, run_dir, source)

    assert manifest["files"]["rec"] == "run-test.ld.json"
    assert manifest["files"]["execution"] == EXECUTION_DOCUMENT
    assert manifest["files"]["frame_log_health"] == "logs/frame_log.pb.health.json"
    # What the generation owns is referenced where it lives, relative to the run.
    assert manifest["files"]["frame_log_proto"] == "../source/contract/frame_log.proto"
    assert "frame_layout" not in manifest["files"]
    # One provenance document, not one per tool.
    assert manifest["files"]["provenance"] == f"../source/{GENERATION_DOCUMENT}"
    assert "dsl_provenance" not in manifest["files"]
    assert "runtime_ttl" not in manifest["files"]
    assert "rec" not in manifest
    assert manifest["files"]["controller"] == "../source/controller"
    assert "artifacts" not in manifest
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
    _, log_path, _, schema = replay.resolve_archive(run_dir)
    validate_header(log_path, schema)
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
    _, log_path, _, schema = replay.resolve_archive(frame_log)
    validate_header(log_path, schema)


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


def test_generation_owned_run_does_not_copy_static_artifacts(tmp_path: Path) -> None:
    generated = _source_tree(tmp_path / "generation" / "generated")

    run_dir = generated.parent / "runs" / "run-1"
    manifest = create_archive_manifest(
        run_dir, source_dir=generated, run_id="run-1", frame_log=generated / "frame_log.pb"
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
    generated = _source_tree(tmp_path / "generation" / "generated")
    (generated / "source").mkdir()
    (generated / "source" / "demo.robmot").write_text("guarded-motion (ns=demo) move {\n}\n")
    (generated / "source" / "demo.fsm").write_text("fsm demo {\n}\n")

    run_dir = generated.parent / "runs" / "run-1"
    manifest = create_archive_manifest(
        run_dir, source_dir=generated, run_id="run-1", frame_log=generated / "frame_log.pb"
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
        run_dir,
        source_dir=source,
        run_id="run-test",
        frame_log=source / "frame_log.pb",
        log_producer_executable=_executable(tmp_path),
    )
    rec_path = rec_document(run_dir, "run-test")
    rec_path.write_text(rec_path.read_text().replace('"Execution"', '"Activity"'))

    with pytest.raises(ArchiveError, match="missing REC provenance relationship"):
        verify_manifest(run_dir)
