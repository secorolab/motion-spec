# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil

import pytest
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


def test_a_run_records_how_it_was_launched(tmp_path: Path) -> None:
    """`rerun` repeats a run, so the run states what repeating it means -- nothing else does."""
    source = _source_tree(tmp_path / "source")
    executable = _log_copy_executable(tmp_path / "log-copy")
    generation = tmp_path / "generation"
    run_dir = generation / "runs" / "run-001"
    work = tmp_path / "work"
    work.mkdir()

    run_cataloged(
        run_dir,
        source_dir=source,
        executable=executable,
        executable_args=[str(source / "frame_log.pb"), "--headless"],
        run_id="run-001",
        cwd=work,
    )

    previous, invocation = runner.last_invocation(generation)
    assert previous == run_dir
    assert invocation["executable_args"] == [str(source / "frame_log.pb"), "--headless"]
    assert invocation["cwd"] == str(work)
    assert Path(invocation["executable"]) == executable.resolve()
    assert Path(invocation["source_dir"]) == source.resolve()


def test_the_last_run_is_the_one_repeated(tmp_path: Path) -> None:
    generation = tmp_path / "generation"
    (generation / "runs").mkdir(parents=True)

    with pytest.raises(runner.RunnerError, match="has not been run yet"):
        runner.last_invocation(generation)

    (generation / "runs" / "run-old").mkdir()
    with pytest.raises(runner.RunnerError, match="predates the invocation record"):
        runner.last_invocation(generation)

    for name, args in (("run-001", ["first"]), ("run-002", ["second"])):
        path = generation / "runs" / name
        path.mkdir()
        (path / runner.INVOCATION_FILE).write_text(json.dumps({"executable_args": args}))
        os.utime(path / runner.INVOCATION_FILE, (1, 1 if name == "run-001" else 2))

    previous, invocation = runner.last_invocation(generation)
    assert previous.name == "run-002"
    assert invocation["executable_args"] == ["second"]


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
