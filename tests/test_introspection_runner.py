# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
from pathlib import Path

from motion_spec.runs.archive import verify_manifest
from motion_spec.runs.provenance import rec_document
from motion_spec.runs.runner import run_cataloged

# Writes to both streams, copies a frame log when given one, then exits with `exit_code`.
NOISY_EXECUTABLE = (
    "#!/usr/bin/env python3\n"
    "import os\n"
    "import shutil\n"
    "import sys\n"
    "print('frame log:', repr(os.environ['MOTION_SPEC_FRAME_LOG']), flush=True)\n"
    "print('blocks:', os.environ['MOTION_SPEC_SHM_NAME'], "
    "os.environ['MOTION_SPEC_CTRL_SHM_NAME'], flush=True)\n"
    "print('boom', file=sys.stderr, flush=True)\n"
    "if len(sys.argv) > 1:\n"
    "    shutil.copyfile(sys.argv[1], os.environ['MOTION_SPEC_FRAME_LOG'])\n"
    "sys.exit({exit_code})\n"
)


def test_each_run_names_its_own_shared_memory_blocks(tmp_path: Path, source_tree: Path) -> None:
    executable = tmp_path / "noisy"
    executable.write_text(NOISY_EXECUTABLE.format(exit_code=0))
    executable.chmod(0o755)
    schema_hash = json.loads((source_tree / "contract" / "frame_layout.json").read_text())[
        "schema_hash"
    ][:16]

    for run_id in ("run-a", "run-b"):
        run_dir = tmp_path / run_id
        assert (
            run_cataloged(
                run_dir,
                source_dir=source_tree,
                executable=executable,
                executable_args=[str(source_tree / "frame_log.pb")],
                run_id=run_id,
            )
            == 0
        )
        console = (run_dir / "logs" / "console.log").read_text()
        assert (
            f"blocks: /motion_spec_{schema_hash}_{run_id} /motion_spec_ctrl_{schema_hash}_{run_id}"
            in console
        )


def test_run_without_a_log_still_catalogs_itself(tmp_path: Path, source_tree: Path) -> None:
    # --no-log: the runtime is told to record nothing (an empty path), and what the run leaves
    # -- console, rec, a manifest that says so -- must still stand on its own.
    executable = tmp_path / "noisy-quiet"
    executable.write_text(NOISY_EXECUTABLE.format(exit_code=0))
    executable.chmod(0o755)
    run_dir = tmp_path / "run-005"

    result = run_cataloged(
        run_dir, source_dir=source_tree, executable=executable, run_id="run-005", record_log=False
    )

    assert result == 0
    assert "frame log: ''" in (run_dir / "logs" / "console.log").read_text()
    assert not (run_dir / "logs" / "frame_log.pb").exists()
    assert rec_document(run_dir, "run-005").exists()
    manifest = verify_manifest(run_dir)
    assert manifest["recorded"] is False
    assert manifest["files"]["console"] == "logs/console.log"


def test_crashed_run_leaves_its_error_output(tmp_path: Path, source_tree: Path) -> None:
    # No frame log, no manifest -- console.log is the only evidence of why the run died.
    executable = tmp_path / "noisy-fail"
    executable.write_text(NOISY_EXECUTABLE.format(exit_code=3))
    executable.chmod(0o755)
    run_dir = tmp_path / "run-004"

    result = run_cataloged(run_dir, source_dir=source_tree, executable=executable, run_id="run-004")

    assert result == 3
    assert not (run_dir / "manifest.json").exists()
    assert "boom" in (run_dir / "logs" / "console.log").read_text()
