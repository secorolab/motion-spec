# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

from pathlib import Path

import rdflib

from motion_spec.introspection import runner
from motion_spec.introspection.archive import verify_manifest
from motion_spec.introspection.runner import run_cataloged
from motion_spec.provenance import prov_uri, rec_run_lifecycle

from test_introspection_archive import _source_tree


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


def test_runner_cli_accepts_options_after_run_dir(monkeypatch) -> None:
    captured = {}

    def fake_run_cataloged(run_dir, **kwargs):
        captured["run_dir"] = run_dir
        captured.update(kwargs)
        return 0

    monkeypatch.setattr(runner, "run_cataloged", fake_run_cataloged)

    assert (
        runner.main(
            [
                "runs/test",
                "--source-dir",
                "gen/model",
                "--run-id",
                "test",
                "--executable",
                "gen/model/build/main",
                "--recover-runtime-ttl",
                "--",
                "--headless",
                "--steps",
                "10",
            ]
        )
        == 0
    )
    assert captured["run_dir"] == "runs/test"
    assert captured["source_dir"] == "gen/model"
    assert captured["executable"] == "gen/model/build/main"
    assert captured["executable_args"] == ["--headless", "--steps", "10"]
    assert captured["run_id"] == "test"
    assert captured["recover_runtime_ttl"] is True
