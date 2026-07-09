# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
from pathlib import Path

from motion_spec.introspection import runner
from motion_spec.introspection.archive import verify_manifest
from motion_spec.introspection.runner import run_cataloged

from test_introspection_archive import _source_tree


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

    rec_doc = json.loads((run_dir / "rec.jsonld").read_text())
    assert rec_doc["status"] == "COMPLETED"
    assert rec_doc["startedAtTime"]
    assert rec_doc["endedAtTime"]
    assert "run" not in rec_doc
    assert "@graph" not in rec_doc
    assert any(row["role"] == "run_cataloging" for row in rec_doc["activities"])
    assert any(row["role"] == "log_producer_executable" for row in rec_doc["resources"])
    assert any(row["role"] == "frame_log" for row in rec_doc["artefacts"])
    assert any(row["role"] == "runtime_ttl" for row in rec_doc["artefacts"])


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
