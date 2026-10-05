# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest
from support import EXAMPLES

from motion_spec.generation.pipeline import generate_model
from motion_spec.runs import replay
from motion_spec.runs.archive import ArchiveError, create_archive_manifest, verify_manifest
from motion_spec.runs.provenance import prov_uri, rec_document
from motion_spec.runs.replay import decode_frames, summarize, validate_header
from motion_spec.runs.runner import start_rec_run


@pytest.fixture
def run_dir(tmp_path: Path, source_tree: Path) -> Path:
    """A run catalogued the way the runner catalogues it before it launches the executable,
    with the health report the runtime writes beside its log."""
    run_dir = tmp_path / "run"
    (run_dir / "logs").mkdir(parents=True)
    (tmp_path / "main").write_text("binary\n")
    schema = json.loads((source_tree / "contract" / "frame_layout.json").read_text())
    start_rec_run(run_dir, "run-test", source_tree, tmp_path / "main", schema, [])
    shutil.copyfile(
        source_tree / "frame_log.pb.health.json", run_dir / "logs" / "frame_log.pb.health.json"
    )
    return run_dir


def test_verify_rejects_a_file_that_no_longer_hashes_to_what_rec_recorded(
    tmp_path: Path, source_tree: Path, run_dir: Path
) -> None:
    (run_dir / "logs" / "console.log").write_text("started\n")
    create_archive_manifest(
        run_dir,
        source_dir=source_tree,
        run_id="run-test",
        log_producer_executable=tmp_path / "main",
        frame_log=source_tree / "frame_log.pb",
    )
    (run_dir / "logs" / "console.log").write_text("tampered\n")

    with pytest.raises(ArchiveError, match="sha256 mismatch"):
        verify_manifest(run_dir)


def test_verification_writes_nothing(tmp_path: Path, source_tree: Path, run_dir: Path) -> None:
    create_archive_manifest(
        run_dir,
        source_dir=source_tree,
        run_id="run-test",
        log_producer_executable=tmp_path / "main",
        frame_log=source_tree / "frame_log.pb",
    )
    before = {path.name for path in run_dir.rglob("*")}

    assert verify_manifest(run_dir)["run_id"] == "run-test"

    assert {path.name for path in run_dir.rglob("*")} == before


@pytest.mark.parametrize(
    ("old", "new"),
    [
        ('"Execution"', '"Activity"'),
        (prov_uri("run:run-test"), "https://secorolab.github.io/rec/run/run-test"),
    ],
    ids=["not-an-execution", "another-run"],
)
def test_verify_requires_the_run_to_be_its_recorded_execution(
    tmp_path: Path, source_tree: Path, run_dir: Path, old, new
) -> None:
    create_archive_manifest(
        run_dir,
        source_dir=source_tree,
        run_id="run-test",
        log_producer_executable=tmp_path / "main",
        frame_log=source_tree / "frame_log.pb",
    )
    rec_path = rec_document(run_dir, "run-test")
    assert old in rec_path.read_text()
    rec_path.write_text(rec_path.read_text().replace(old, new))

    with pytest.raises(ArchiveError):
        verify_manifest(run_dir)


def test_replay_works_on_aborted_run_without_manifest(tmp_path: Path, source_tree: Path) -> None:
    # A run killed before archiving leaves its log but no manifest; the log carries its contract.
    run_dir = tmp_path / "aborted-run"
    (run_dir / "logs").mkdir(parents=True)
    frame_log = run_dir / "logs" / "frame_log.pb"
    shutil.copyfile(source_tree / "frame_log.pb", frame_log)

    assert decode_frames(frame_log)[0]["step"] == 7
    assert decode_frames(run_dir)[0]["step"] == 7
    assert f"archive     {run_dir}" in summarize(frame_log)
    _, log_path, _, schema = replay.resolve_archive(frame_log)
    validate_header(log_path, schema)


def test_logless_manifest_says_so_and_only_then_verifies_without_a_log(
    tmp_path: Path, source_tree: Path, run_dir: Path
) -> None:
    # "recorded": false is the run stating it kept no log on purpose; a lost log still fails.
    (run_dir / "logs" / "console.log").write_text("started\n")
    (run_dir / "logs" / "frame_log.pb.health.json").unlink()

    manifest = create_archive_manifest(
        run_dir,
        source_dir=source_tree,
        run_id="run-test",
        log_producer_executable=tmp_path / "main",
        recorded=False,
    )
    assert manifest["recorded"] is False
    assert "frame_log" not in manifest["files"]
    assert verify_manifest(run_dir)["run_id"] == "run-test"

    del manifest["recorded"]
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=4) + "\n")
    with pytest.raises(ArchiveError, match="files.frame_log: missing"):
        verify_manifest(run_dir)


def test_generated_manifest_is_portable(tmp_path: Path) -> None:
    pick_and_place = EXAMPLES["pick_and_place"] / "pick_and_place.robmot"
    generated = generate_model(pick_and_place, tmp_path, stage="ir")
    manifest = generated / "model" / "pick_and_place-app.ld.json"
    text = json.dumps(json.loads(manifest.read_text()))
    assert str(manifest.parent) not in text
    assert "https://secorolab.github.io/" in text
