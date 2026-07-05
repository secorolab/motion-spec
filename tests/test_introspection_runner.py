# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
from pathlib import Path

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
        executable_args=[str(source / "frame_log.bin")],
        run_id="run-001",
        recover_runtime_ttl=True,
    )

    assert result == 0
    manifest = verify_manifest(run_dir)
    assert manifest["run_id"] == "run-001"
    assert manifest["files"]["frame_log"] == "frame_log.bin"
    assert manifest["files"]["log_producer_executable"] == "generated/log_producer_executable/log-copy"
    assert (run_dir / "runtime.ttl").exists()

    rec_doc = json.loads((run_dir / "rec.json").read_text())
    assert rec_doc["run"]["status"] == "COMPLETED"
    assert rec_doc["run"]["started_time"]
    assert rec_doc["run"]["completed_time"]
    assert any(row["role"] == "run_cataloging" for row in rec_doc["activities"])
    assert any(row["role"] == "log_producer_executable" for row in rec_doc["resources"])
    assert any(row["role"] == "frame_log" for row in rec_doc["artefacts"])
    assert any(row["role"] == "runtime_ttl" for row in rec_doc["artefacts"])
