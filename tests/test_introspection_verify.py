# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import shutil
from pathlib import Path

import pytest
from rdf_utils.models.prov import get_file_path, get_py_pkg_info
from rdf_utils.models.vocab import URI_PROV_EXT_TYPE_TRANSFORMATION
from rdf_utils.resolver import install_resolver
from rdflib import Dataset
from rdflib.namespace import DCTERMS, PROV, RDF
from rec.provenance import RUN_BASE
from support import EXAMPLES, SCHEMA

from motion_spec.generation.artifacts import build_frame_layout
from motion_spec.generation.pipeline import generate_model
from motion_spec.runs import replay
from motion_spec.runs.provenance import (
    GENERATION_DOCUMENT,
    read_generation_dataset,
    rec_run,
)
from motion_spec.runs.replay import decode_frames, summarize, validate_header
from motion_spec.runs.runner import record_outputs, start_rec_run
from motion_spec.runs.verify import RecordError, verify_run


@pytest.fixture
def run_dir(tmp_path: Path, source_tree: Path) -> Path:
    """A run the runner recorded: started, its program having left its log and console in
    `logs/`, and its outputs recorded once it exited."""
    run_dir = tmp_path / "runs" / "run-test"
    (run_dir / "logs").mkdir(parents=True)
    schema = build_frame_layout(SCHEMA)
    run, bundle_id = start_rec_run(run_dir, "run-test", source_tree, tmp_path / "main", schema)
    shutil.copyfile(source_tree / "frame_log.pb", run_dir / "logs" / "frame_log.pb")
    (run_dir / "logs" / "console.log").write_text("started\n")
    record_outputs(run_dir, run, bundle_id)
    return run_dir


def test_verify_rejects_a_file_that_no_longer_hashes_to_what_rec_recorded(
    run_dir: Path,
) -> None:
    (run_dir / "logs" / "console.log").write_text("tampered\n")

    with pytest.raises(RecordError, match="content differs"):
        verify_run(run_dir)


def test_verification_writes_nothing(run_dir: Path) -> None:
    before = {path.name for path in run_dir.rglob("*")}

    verify_run(run_dir)

    assert {path.name for path in run_dir.rglob("*")} == before


@pytest.mark.parametrize(
    ("old", "new"),
    [
        ('"prov-ext:Execution"', '"prov:Activity"'),
        (f"{RUN_BASE}run-test", "https://example.test/run/run-test"),
    ],
    ids=["not-an-execution", "another-run"],
)
def test_verify_requires_the_run_to_be_its_recorded_execution(run_dir: Path, old, new) -> None:
    rec_path = rec_run(run_dir).store.path("run-test")
    assert old in rec_path.read_text()
    rec_path.write_text(rec_path.read_text().replace(old, new))

    with pytest.raises(RecordError):
        verify_run(run_dir)


def test_replay_works_on_a_run_killed_before_its_outputs_were_recorded(
    tmp_path: Path, source_tree: Path
) -> None:
    # A run killed mid-run leaves its log and no record of it; the log carries its contract.
    run_dir = tmp_path / "runs" / "aborted-run"
    (run_dir / "logs").mkdir(parents=True)
    frame_log = run_dir / "logs" / "frame_log.pb"
    shutil.copyfile(source_tree / "frame_log.pb", frame_log)

    assert decode_frames(frame_log)[0]["step"] == 7
    assert decode_frames(run_dir)[0]["step"] == 7
    assert f"run         {run_dir}" in summarize(frame_log)
    _, log_path, schema = replay.resolve_run(frame_log)
    validate_header(log_path, schema)


def test_a_logless_run_verifies_and_its_provenance_says_it_kept_no_log(
    tmp_path: Path, source_tree: Path
) -> None:
    run_dir = tmp_path / "runs" / "run-quiet"
    (run_dir / "logs").mkdir(parents=True)
    schema = build_frame_layout(SCHEMA)
    run, bundle_id = start_rec_run(run_dir, "run-quiet", source_tree, tmp_path / "main", schema)
    (run_dir / "logs" / "console.log").write_text("started\n")

    record_outputs(run_dir, run, bundle_id, recorded=False)

    verify_run(run_dir)
    with pytest.raises(RecordError, match="recorded no frame log"):
        replay.resolve_run(run_dir)


def test_a_recorded_log_that_is_lost_fails_verification(run_dir: Path) -> None:
    (run_dir / "logs" / "frame_log.pb.zst").unlink()

    with pytest.raises(RecordError, match="frame_log.pb.zst: missing"):
        verify_run(run_dir)


def test_generated_sources_are_kept_as_alternates_of_the_originals_the_dsl_read(
    monkeypatch, tmp_path: Path
) -> None:
    monkeypatch.setenv("MOTION_SPEC_GEN", str(tmp_path))
    pick_and_place = EXAMPLES["pick_and_place"] / "pick_and_place.robmot"
    generated = generate_model(pick_and_place, tmp_path, stage="ir")
    install_resolver()
    dataset = Dataset().parse(generated / "model" / "pick_and_place.ld.json", format="json-ld")
    dsl = get_py_pkg_info("motion_spec_dsl").purl
    (provenance,) = [
        graph
        for graph in dataset.graphs()
        if (graph.identifier, PROV.wasAttributedTo, dsl) in graph
    ]
    (transformation,) = provenance.subjects(RDF.type, URI_PROV_EXT_TYPE_TRANSFORMATION)
    originals = set(provenance.objects(transformation, PROV.used))
    assert pick_and_place.resolve() in {get_file_path(provenance, source) for source in originals}
    generation = read_generation_dataset(generated / GENERATION_DOCUMENT)
    for original in originals:
        (copy,) = generation.subjects(PROV.alternateOf, original)
        assert get_file_path(generation, copy).parent == (generated / "source").resolve()
        assert generation.value(copy, DCTERMS.identifier) == provenance.value(
            original, DCTERMS.identifier
        )
        (activity,) = generation.objects(copy, PROV.wasGeneratedBy)
        assert (activity, PROV.used, original) in generation
