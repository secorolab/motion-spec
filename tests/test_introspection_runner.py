# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

from pathlib import Path
import shutil

import rdflib

from motion_spec.introspection import runner
from motion_spec.introspection.archive import verify_manifest
from motion_spec.introspection.runner import run_cataloged
from motion_spec.introspection.provenance import prov_uri, rec_run_lifecycle

from support import _source_tree


REC = rdflib.Namespace("https://secorolab.github.io/metamodels/rec#")


def _log_copy_executable(path: Path) -> Path:
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import os\n"
        "import shutil\n"
        "import sys\n"
        "shutil.copyfile(sys.argv[1], os.environ['MOTION_SPEC_FRAME_LOG'])\n"
    )
    path.chmod(path.stat().st_mode | 0o111)
    return path


def _noisy_executable(path: Path, exit_code: int) -> Path:
    """Writes to both streams, copies a frame log when given one, then exits `exit_code`."""
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import os\n"
        "import shutil\n"
        "import sys\n"
        "print('hello from the run', flush=True)\n"
        "print('frame log:', repr(os.environ['MOTION_SPEC_FRAME_LOG']), flush=True)\n"
        "print('boom', file=sys.stderr, flush=True)\n"
        "if len(sys.argv) > 1:\n"
        "    shutil.copyfile(sys.argv[1], os.environ['MOTION_SPEC_FRAME_LOG'])\n"
        f"sys.exit({exit_code})\n"
    )
    path.chmod(path.stat().st_mode | 0o111)
    return path


def test_runner_catalogs_run_from_start_and_archives_outputs(tmp_path: Path) -> None:
    source = _source_tree(tmp_path / "source")
    executable = _log_copy_executable(tmp_path / "log-copy")
    run_dir = tmp_path / "run-001"

    result = run_cataloged(
        run_dir,
        source_dir=source,
        executable=executable,
        executable_args=[str(source / "frame_log.pb")],
        run_id="run-001",
        recover_runtime_ttl=True,
    )

    assert result == 0
    manifest = verify_manifest(run_dir)
    assert manifest["run_id"] == "run-001"
    assert manifest["files"]["frame_log"] == "logs/frame_log.pb"
    assert manifest["files"]["log_producer_executable"] == "controller/executable/log-copy"
    assert (run_dir / "runtime" / "runtime.ttl").exists()
    # Recovery ran, so the manifest names the file it wrote.
    assert manifest["files"]["runtime_ttl"] == "runtime/runtime.ttl"

    # REC writes a PROV graph: lifecycle is an rdf:type on the run, roles are rec:label.
    rec_graph = rdflib.Graph().parse(run_dir / "rec.ld.json", format="json-ld")
    lifecycle = rec_run_lifecycle(rec_graph)
    assert lifecycle["status"] == "COMPLETED"
    assert lifecycle["started_time"]
    assert lifecycle["completed_time"]
    labels = {str(value) for value in rec_graph.objects(None, REC.label)}
    assert {"log_producer_executable", "frame_log", "runtime_ttl"} <= labels
    assert (
        rdflib.URIRef(prov_uri("activity:run_cataloging")),
        rdflib.RDF.type,
        rdflib.URIRef("http://www.w3.org/ns/prov#Activity"),
    ) in rec_graph


def test_interrupted_runner_recovers_runtime_ttl(tmp_path: Path, monkeypatch) -> None:
    source = _source_tree(tmp_path / "source")
    executable = _log_copy_executable(tmp_path / "log-copy")
    run_dir = tmp_path / "run-002"

    def interrupt(_executable, _args, *, cwd, frame_log, run_id, rec_path):
        frame_log.parent.mkdir(parents=True)
        shutil.copyfile(source / "frame_log.pb", frame_log)
        runner._finish_rec_run(rec_path, run_id, "INTERRUPTED")
        return 130

    monkeypatch.setattr(runner, "_run_executable", interrupt)

    result = run_cataloged(
        run_dir,
        source_dir=source,
        executable=executable,
        run_id="run-002",
        recover_runtime_ttl=True,
    )

    assert result == 130
    assert (run_dir / "runtime" / "runtime.ttl").exists()
    rec_graph = rdflib.Graph().parse(run_dir / "rec.ld.json", format="json-ld")
    assert rec_run_lifecycle(rec_graph)["status"] == "INTERRUPTED"


def test_console_is_captured_and_mirrored(tmp_path: Path, capfd) -> None:
    source = _source_tree(tmp_path / "source")
    executable = _noisy_executable(tmp_path / "noisy", 0)
    run_dir = tmp_path / "run-003"

    result = run_cataloged(
        run_dir,
        source_dir=source,
        executable=executable,
        executable_args=[str(source / "frame_log.pb")],
        run_id="run-003",
    )

    assert result == 0
    console = (run_dir / "logs" / "console.log").read_text()
    assert "hello from the run" in console
    assert "boom" in console
    assert "hello from the run" in capfd.readouterr().out
    manifest = verify_manifest(run_dir)
    assert manifest["files"]["console"] == "logs/console.log"
    # Nothing recovered runtime.ttl here, so nothing promises it.
    assert "runtime_ttl" not in manifest["files"]


def test_run_without_a_log_still_catalogs_itself(tmp_path: Path) -> None:
    # --no-log: the runtime is told to record nothing (an empty path), and what the run leaves
    # -- console, rec, a manifest that says so -- must still stand on its own.
    source = _source_tree(tmp_path / "source")
    executable = _noisy_executable(tmp_path / "noisy-quiet", 0)
    run_dir = tmp_path / "run-005"

    # recover_runtime_ttl on: a logless run has nothing to recover and must not try.
    result = run_cataloged(
        run_dir,
        source_dir=source,
        executable=executable,
        run_id="run-005",
        record_log=False,
        recover_runtime_ttl=True,
    )

    assert result == 0
    console = (run_dir / "logs" / "console.log").read_text()
    assert "frame log: ''" in console
    assert not (run_dir / "logs" / "frame_log.pb").exists()
    assert (run_dir / "rec.ld.json").exists()
    manifest = verify_manifest(run_dir)
    assert manifest["recorded"] is False
    assert "frame_log" not in manifest["files"]
    assert manifest["files"]["console"] == "logs/console.log"


def test_crashed_run_leaves_its_error_output(tmp_path: Path) -> None:
    # No frame log, no manifest -- console.log is the only evidence of why the run died.
    source = _source_tree(tmp_path / "source")
    executable = _noisy_executable(tmp_path / "noisy-fail", 3)
    run_dir = tmp_path / "run-004"

    result = run_cataloged(run_dir, source_dir=source, executable=executable, run_id="run-004")

    assert result == 3
    assert not (run_dir / "manifest.json").exists()
    assert "boom" in (run_dir / "logs" / "console.log").read_text()
