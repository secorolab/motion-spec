# SPDX-License-Identifier: MPL-2.0
"""Replay generated introspection frame logs from a run archive."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from motion_spec.introspection.archive import ArchiveError, load_manifest, verify_manifest
from motion_spec.introspection import frame_log_pb


def run_dir_for(log_path: Path) -> Path:
    if not log_path.exists():
        raise ArchiveError(f"{log_path}: does not exist")
    search_start = log_path if log_path.is_dir() else log_path.parent
    for path in (search_start, *search_start.parents):
        if (path / "manifest.json").exists():
            return path
    # No manifest yet -- e.g. a run killed before the archiving step ran. Fall back to the
    # conventional layout every archive writer uses instead of rejecting outright: a directory
    # passed directly is the run dir itself, and a bare frame log lives at <run_dir>/logs/.
    if log_path.is_dir():
        return log_path
    return search_start.parent if search_start.name == "logs" else search_start


def resolve_archive(path: Path | str) -> tuple[Path, Path, dict | None, dict]:
    input_path = Path(path)
    run_dir = run_dir_for(input_path)
    manifest_path = run_dir / "manifest.json"
    manifest = None
    frame_log_rel = "logs/frame_log.pb"
    if manifest_path.exists():
        _, manifest = load_manifest(run_dir)
        frame_log_rel = manifest["files"]["frame_log"]
    log_path = run_dir / frame_log_rel if input_path.is_dir() else input_path
    if not log_path.exists():
        raise ArchiveError(f"{log_path}: missing frame log")
    # The log carries its own decode contract; the manifest, when present, only locates it.
    return run_dir, log_path, manifest, frame_log_pb.read_contract(log_path)


def read_meta(log_path: Path | str) -> dict:
    path = Path(log_path)
    if path.suffix != ".pb":
        raise ArchiveError(f"{path}: expected a .pb frame log")
    return frame_log_pb.read_header(path)


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
        raise ArchiveError(f"{log_path}: frame log header carries no schema hash")
    return {
        "schema_hash": header.schema_hash,
        "producer_agent_id": header.producer_agent_id,
        "activity_id": header.activity_id,
    }


def read_health(log_path: Path | str) -> dict | None:
    health_path = Path(str(log_path) + ".health.json")
    if not health_path.exists():
        return None
    return json.loads(health_path.read_text())


def decode_frames(log_path: Path | str) -> list[dict]:
    _run_dir, log_path, _manifest, contract = resolve_archive(log_path)
    validate_header(log_path, contract)
    return list(frame_log_pb.frame_records(log_path, contract))


def runtime_frames(log_path: Path | str) -> tuple[list[dict], int]:
    records = decode_frames(log_path)
    return records, len(records)


def summarize(log_path: Path | str) -> str:
    run_dir, log_path, _manifest, contract = resolve_archive(log_path)
    meta = validate_header(log_path, contract)
    count = 0
    first = last = None
    periods = []
    computes = []
    for record in frame_log_pb.frame_records(log_path, contract):
        first = first or record
        last = record
        count += 1
        if record["step"] > 0:
            periods.append(record["timing"]["period_ns"])
        computes.append(record["timing"]["compute_ns"])
    if last is None:
        return f"log         {log_path}\nframes      0"
    periods.sort()
    computes.sort()
    states = contract.header.fsm_states
    final_state = last["fsm_state"]
    final_name = states[final_state].id if 0 <= final_state < len(states) else str(final_state)
    lines = [
        f"log         {log_path}",
        f"archive     {run_dir}",
        f"frames      {count}",
        f"writer      {meta.get('producer_agent_id', '')} {meta.get('activity_id', '')}",
        f"final state {final_name}",
    ]
    if frame_log_pb.tail_is_partial(log_path):
        lines.append("log         truncated: the last record is short, the writer never closed")
    health = read_health(log_path)
    if health:
        lines.append(
            "log health  "
            f"attempted {health.get('attempted_frames')} "
            f"written {health.get('written_frames')} "
            f"dropped {health.get('dropped_frames')}"
        )
    if periods:
        lines.append(
            f"period[ms]  mean {sum(periods) / len(periods) / 1e6:.3f} max {periods[-1] / 1e6:.3f}"
        )
    if computes:
        lines.append(
            f"compute[us] mean {sum(computes) / len(computes) / 1e3:.1f} max {computes[-1] / 1e3:.1f}"
        )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="motion-spec replay")
    parser.add_argument("log", help="frame_log.pb inside a motion-spec run archive")
    parser.add_argument("--jsonl", action="store_true", help="emit decoded frames as JSON Lines")
    parser.add_argument("--verify", action="store_true", help="verify manifest/header only")
    parser.add_argument(
        "--recover-runtime-ttl", action="store_true", help="write runtime.ttl from the frame log"
    )
    args = parser.parse_args(argv)
    try:
        if args.recover_runtime_ttl:
            from motion_spec.introspection.runtime_graph import write_runtime_ttl

            run_dir, log_path, _manifest, _contract = resolve_archive(args.log)
            records, frame_count = runtime_frames(log_path)
            out = write_runtime_ttl(run_dir, records, frame_count=frame_count)
            print(out)
        elif args.verify:
            run_dir, log_path, manifest, contract = resolve_archive(args.log)
            validate_header(log_path, contract)
            if manifest is not None:
                verify_manifest(run_dir)
                print("archive OK")
            else:
                print("archive OK (no manifest.json yet -- header verified only)")
        elif args.jsonl:
            for record in decode_frames(args.log):
                print(json.dumps(record, separators=(",", ":")))
        else:
            print(summarize(args.log))
    except ArchiveError as exc:
        parser.exit(2, f"{exc}\n")
    except BrokenPipeError:
        sys.stdout = None
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
