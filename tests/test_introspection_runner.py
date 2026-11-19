# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
import threading
from datetime import UTC, datetime
from pathlib import Path

from rdflib.namespace import PROV, RDF
from rec import State

from motion_spec.runs.provenance import (
    read_generation_dataset,
    rec_document,
    rec_run,
    record_build,
    run_outputs,
)
from motion_spec.runs.runner import run_cataloged
from motion_spec.runs.verify import verify_run

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
    # One document: rec's bundle and motion-spec's, every step of the run writing to its own.
    document = read_generation_dataset(rec_document(run_dir))
    assert len([g for g in document.graphs() if (g.identifier, RDF.type, PROV.Bundle) in g]) == 2
    verify_run(run_dir)
    outputs = [path.resolve() for path in run_outputs(run_dir)]
    assert (run_dir / "logs" / "console.log").resolve() in outputs
    assert not any(path.name.startswith("frame_log.pb") for path in outputs)


def test_a_run_cancelled_in_progress_stops_and_ends_canceled(
    tmp_path: Path, source_tree: Path
) -> None:
    # Copies its log, then runs until it is signalled, leaving its loop as the generated one does.
    executable = tmp_path / "until-stopped"
    executable.write_text(
        "#!/usr/bin/env python3\n"
        "import os, shutil, signal, sys, time\n"
        f"shutil.copyfile({str(source_tree / 'frame_log.pb')!r}, os.environ['MOTION_SPEC_FRAME_LOG'])\n"
        "signal.signal(signal.SIGTERM, lambda *_: sys.exit(130))\n"
        "time.sleep(60)\n"
    )
    executable.chmod(0o755)
    now = datetime.now(UTC)
    record_build(source_tree, source_tree / "controller", executable, started=now, ended=now)
    run_dir = tmp_path / "runs" / "run-006"
    # What `motion-spec cancel` does, from another handle on the run, while the program runs.
    asker = threading.Timer(3.0, lambda: rec_run(run_dir).cancel())
    asker.start()

    run_cataloged(run_dir, source_dir=source_tree, executable=executable, run_id="run-006")
    asker.join()

    assert rec_run(run_dir).lifecycle()[0] is State.CANCELED


def test_crashed_run_leaves_its_error_output(tmp_path: Path, source_tree: Path) -> None:
    # No frame log, no recorded outputs -- console.log is the only evidence of why the run died.
    executable = tmp_path / "noisy-fail"
    executable.write_text(NOISY_EXECUTABLE.format(exit_code=3, source_log=""))
    executable.chmod(0o755)
    run_dir = tmp_path / "runs" / "run-004"

    result = run_cataloged(run_dir, source_dir=source_tree, executable=executable, run_id="run-004")

    assert result == 3
    assert run_outputs(run_dir) == []
    assert "boom" in (run_dir / "logs" / "console.log").read_text()
