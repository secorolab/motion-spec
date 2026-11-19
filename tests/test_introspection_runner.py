# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
from datetime import UTC, datetime
from pathlib import Path

from motion_spec.runs.archive import verify_run
from motion_spec.runs.provenance import record_build, run_outputs
from motion_spec.runs.runner import run_cataloged

# Writes to both streams, copies `source_log` as its frame log when named, then exits with
# `exit_code`.
NOISY_EXECUTABLE = (
    "#!/usr/bin/env python3\n"
    "import os\n"
    "import shutil\n"
    "import sys\n"
    "print('frame log:', repr(os.environ['MOTION_SPEC_FRAME_LOG']), flush=True)\n"
    "print('blocks:', os.environ['MOTION_SPEC_SHM_NAME'], "
    "os.environ['MOTION_SPEC_CTRL_SHM_NAME'], flush=True)\n"
    "print('boom', file=sys.stderr, flush=True)\n"
    "if {source_log!r}:\n"
    "    shutil.copyfile({source_log!r}, os.environ['MOTION_SPEC_FRAME_LOG'])\n"
    "sys.exit({exit_code})\n"
)


def test_each_run_names_its_own_shared_memory_blocks(tmp_path: Path, source_tree: Path) -> None:
    executable = tmp_path / "noisy"
    executable.write_text(
        NOISY_EXECUTABLE.format(exit_code=0, source_log=str(source_tree / "frame_log.pb"))
    )
    executable.chmod(0o755)
    # A run runs what its generation's build generated.
    now = datetime.now(UTC)
    record_build(source_tree, source_tree / "controller", executable, started=now, ended=now)
    schema_hash = json.loads((source_tree / "contract" / "frame_layout.json").read_text())[
        "schema_hash"
    ][:16]

    for run_id in ("run-a", "run-b"):
        run_dir = tmp_path / "runs" / run_id
        assert (
            run_cataloged(run_dir, source_dir=source_tree, executable=executable, run_id=run_id)
            == 0
        )
        console = (run_dir / "logs" / "console.log").read_text()
        assert (
            f"blocks: /motion_spec_{schema_hash}_{run_id} /motion_spec_ctrl_{schema_hash}_{run_id}"
            in console
        )


def test_run_without_a_log_still_catalogs_itself(tmp_path: Path, source_tree: Path) -> None:
    # --no-log: the runtime is told to record nothing (an empty path), and what the run leaves
    # -- console, rec, provenance that says so -- must still stand on its own.
    executable = tmp_path / "noisy-quiet"
    executable.write_text(NOISY_EXECUTABLE.format(exit_code=0, source_log=""))
    executable.chmod(0o755)
    now = datetime.now(UTC)
    record_build(source_tree, source_tree / "controller", executable, started=now, ended=now)
    run_dir = tmp_path / "runs" / "run-005"

    result = run_cataloged(
        run_dir, source_dir=source_tree, executable=executable, run_id="run-005", record_log=False
    )

    assert result == 0
    assert "frame log: ''" in (run_dir / "logs" / "console.log").read_text()
    assert not (run_dir / "logs" / "frame_log.pb").exists()
    verify_run(run_dir)
    outputs = [path.resolve() for path in run_outputs(run_dir)]
    assert (run_dir / "logs" / "console.log").resolve() in outputs
    assert not any(path.name.startswith("frame_log.pb") for path in outputs)


def test_crashed_run_leaves_its_error_output(tmp_path: Path, source_tree: Path) -> None:
    # No frame log, no archive -- console.log is the only evidence of why the run died.
    executable = tmp_path / "noisy-fail"
    executable.write_text(NOISY_EXECUTABLE.format(exit_code=3, source_log=""))
    executable.chmod(0o755)
    run_dir = tmp_path / "runs" / "run-004"

    result = run_cataloged(run_dir, source_dir=source_tree, executable=executable, run_id="run-004")

    assert result == 3
    assert run_outputs(run_dir) == []
    assert "boom" in (run_dir / "logs" / "console.log").read_text()
