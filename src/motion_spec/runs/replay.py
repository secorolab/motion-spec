# SPDX-License-Identifier: MPL-2.0
"""Replay a run's frame log, and check the run from its record."""

from __future__ import annotations

import json
from pathlib import Path

from motion_spec.runs.provenance import run_generation, run_outputs
from motion_spec.runs.verify import FRAME_LOG, RecordError, verify_run
from motion_spec.telemetry import frame_log_pb


def resolve_run(path: Path | str) -> tuple[Path, Path, frame_log_pb.LogContract]:
    """A run directory or frame log PATH as (run directory, frame log, the log's contract).

    Raises:
        RecordError: the run has no frame log -- by choice when its provenance records the
            files it generated and the log is not among them.
    """
    input_path = Path(path)
    # Callers name the log the way it is written, and a recorded one has since been packed;
    # resolve before asking whether it is there, or the conventional name reads as missing.
    named = frame_log_pb.log_path(input_path)
    if not named.exists():
        raise RecordError(named, "does not exist")
    if named.is_dir():
        run_dir = named
    else:
        # A log sits in the run directory's `logs/`.
        run_dir = named.parent.parent if named.parent.name == "logs" else named.parent
    # Either name: a run still being written has the plain log, a recorded one the packed.
    log_path = frame_log_pb.log_path(run_dir / FRAME_LOG if input_path.is_dir() else input_path)
    if not log_path.exists():
        outputs = run_outputs(run_dir)
        if outputs and log_path.resolve() not in [output.resolve() for output in outputs]:
            raise RecordError(run_dir, "the run recorded no frame log")
        raise RecordError(log_path, "missing frame log")
    return run_dir, log_path, frame_log_pb.read_contract(log_path)


def validate_header(log_path: Path | str, contract=None) -> dict:
    """The log states its own identity, so there is nothing left to cross-check it against.

    A mismatch used to mean the log and schema.json had drifted apart; with the contract
    inside the log that class of error cannot arise. What can still fail is a log written by
    a runtime older than the embedded descriptor, which read_contract rejects.
    """
    if contract is None:
        contract = frame_log_pb.read_contract(log_path)
    header = contract.header
    if not header.schema_hash:
        raise RecordError(log_path, "frame log header carries no schema hash")
    return {"schema_hash": header.schema_hash}


def verify_recording(path: Path | str) -> int:
    """Check a recorded run end to end and return how many frames its log holds.

    Raises:
        RecordError: the run fails `verify_run`, the log was written against another contract
            than its generation's, or the log holds other than the frames the writer reported.
    """
    run_dir, log_path, contract = resolve_run(path)
    validate_header(log_path, contract)
    verify_run(run_dir)
    layout_path = run_generation(run_dir) / "generated" / "contract" / "frame_layout.json"
    expected = json.loads(layout_path.read_text())["schema_hash"]
    if contract.header.schema_hash != expected:
        raise RecordError(
            log_path,
            f"written against schema {contract.header.schema_hash}, the generation's is {expected}",
        )
    frames = sum(1 for _ in frame_log_pb.frame_records(log_path, contract))
    health = frame_log_pb.read_health(log_path)
    if health is None:
        raise RecordError(log_path, "ends without the writer's health record: it never closed")
    if health["written_frames"] != frames:
        raise RecordError(
            log_path,
            f"the writer reported {health['written_frames']} frames, the log holds {frames}",
        )
    return frames


def decode_frames(log_path: Path | str) -> list[dict]:
    _run_dir, log_path, contract = resolve_run(log_path)
    validate_header(log_path, contract)
    return list(frame_log_pb.frame_records(log_path, contract))


def summarize(log_path: Path | str) -> str:
    run_dir, log_path, contract = resolve_run(log_path)
    meta = validate_header(log_path, contract)
    count = 0
    last = None
    periods = []
    computes = []
    for record in frame_log_pb.frame_records(log_path, contract):
        last = record
        count += 1
        if record["step"] > 0:
            periods.append(record["timing"]["period_ns"])
        computes.append(record["timing"]["compute_ns"])
    if last is None:
        return f"log         {log_path}\nframes      0"
    states = contract.header.fsm_states
    final_state = last["fsm_state"]
    final_name = states[final_state].id if 0 <= final_state < len(states) else str(final_state)
    lines = [
        f"log         {log_path}",
        f"run         {run_dir}",
        f"frames      {count}",
        f"schema      {meta.get('schema_hash', '')}",
        f"final state {final_name}",
    ]
    if frame_log_pb.tail_is_partial(log_path):
        lines.append("log         truncated: the last record is short, the writer never closed")
    health = frame_log_pb.read_health(log_path)
    if health:
        lines.append(
            "log health  "
            f"attempted {health.get('attempted_frames')} "
            f"written {health.get('written_frames')} "
            f"dropped {health.get('dropped_frames')}"
        )
    if periods:
        lines.append(
            f"period[ms]  mean {sum(periods) / len(periods) / 1e6:.3f} max {max(periods) / 1e6:.3f}"
        )
    lines.append(
        f"compute[us] mean {sum(computes) / len(computes) / 1e3:.1f} max {max(computes) / 1e3:.1f}"
    )
    return "\n".join(lines)
