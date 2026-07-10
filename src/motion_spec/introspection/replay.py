# SPDX-License-Identifier: MPL-2.0
"""Replay generated introspection frame logs from a run archive."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Iterator
from pathlib import Path

from motion_spec.introspection.archive import ArchiveError, verify_manifest
from motion_spec.introspection import frame_log_pb


def run_dir_for(log_path: Path) -> Path:
    if not log_path.exists():
        raise ArchiveError(f"{log_path}: does not exist")
    if log_path.is_dir() and (log_path / "manifest.json").exists():
        return log_path
    for path in (log_path.parent, *log_path.parent.parents):
        if (path / "manifest.json").exists():
            return path
    return log_path.parent


def resolve_archive(path: Path | str) -> tuple[Path, Path, dict, dict, dict | None]:
    input_path = Path(path)
    run_dir = run_dir_for(input_path)
    manifest = verify_manifest(run_dir)
    files = manifest["files"]
    schema = json.loads((run_dir / files["schema"]).read_text())
    layout = None
    if files.get("frame_layout") and (run_dir / files["frame_layout"]).exists():
        layout = json.loads((run_dir / files["frame_layout"]).read_text())
    log_path = run_dir / files["frame_log"] if input_path.is_dir() else input_path
    if not log_path.exists():
        raise ArchiveError(f"{log_path}: missing frame log")
    return run_dir, log_path, manifest, schema, layout


def load_archive(log_path: Path | str) -> tuple[Path, dict, dict, dict | None]:
    run_dir, _log_path, manifest, schema, layout = resolve_archive(log_path)
    return run_dir, manifest, schema, layout


def read_meta(log_path: Path | str) -> dict:
    path = Path(log_path)
    if path.suffix != ".pb":
        raise ArchiveError(f"{path}: expected a .pb frame log")
    return frame_log_pb.read_header(path)


def validate_header(log_path: Path | str, schema: dict, layout: dict | None = None) -> dict:
    meta = read_meta(log_path)
    if meta.get("schema_hash") != schema.get("schema_hash"):
        raise ArchiveError(
            f"{log_path}: frame log header schema_hash {meta.get('schema_hash')} != schema.json "
            f"{schema.get('schema_hash')}"
        )
    expected = schema.get("runtime_provenance", {})
    if expected.get("producer_agent_id") and meta.get("producer_agent_id") != expected["producer_agent_id"]:
        raise ArchiveError(f"{log_path}: producer_agent_id does not match schema runtime provenance")
    if expected.get("activity_id") and meta.get("activity_id") != expected["activity_id"]:
        raise ArchiveError(f"{log_path}: activity_id does not match schema runtime provenance")
    return meta


def read_health(log_path: Path | str) -> dict | None:
    health_path = Path(str(log_path) + ".health.json")
    if not health_path.exists():
        return None
    return json.loads(health_path.read_text())


def _records(log_path: Path | str, schema: dict, layout: dict | None = None) -> Iterator[dict]:
    yield from frame_log_pb.frame_records(log_path, schema)


def frames(log_path: Path | str) -> Iterator[dict]:
    _run_dir, log_path, _manifest, schema, layout = resolve_archive(log_path)
    validate_header(log_path, schema, layout)
    yield from _records(log_path, schema, layout)


def decode_frames(log_path: Path | str) -> list[dict]:
    _run_dir, log_path, _manifest, schema, layout = resolve_archive(log_path)
    validate_header(log_path, schema, layout)
    return list(_records(log_path, schema, layout))


def runtime_frames(log_path: Path | str) -> tuple[list[dict], int]:
    records = decode_frames(log_path)
    return records, len(records)


def sampled_frames(log_path: Path | str) -> tuple[list[dict], int]:
    first = last = None
    count = 0
    for record in frames(log_path):
        if first is None:
            first = record
        last = record
        count += 1
    if first is None:
        return [], 0
    if first == last:
        return [first], count
    return [first, last], count


def summarize(log_path: Path | str) -> str:
    run_dir, log_path, _manifest, schema, layout = resolve_archive(log_path)
    meta = validate_header(log_path, schema, layout)
    count = 0
    first = last = None
    periods = []
    computes = []
    for record in _records(log_path, schema, layout):
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
    states = schema.get("fsm", {}).get("states", [])
    final_state = last["fsm_state"]
    final_name = states[final_state]["id"] if 0 <= final_state < len(states) else str(final_state)
    lines = [
        f"log         {log_path}",
        f"archive     {run_dir}",
        f"frames      {count}",
        f"writer      {meta.get('producer_agent_id', '')} {meta.get('activity_id', '')}",
        f"final state {final_name}",
    ]
    health = read_health(log_path)
    if health:
        lines.append(
            "log health  "
            f"attempted {health.get('attempted_frames')} "
            f"written {health.get('written_frames')} "
            f"dropped {health.get('dropped_frames')}"
        )
    if periods:
        lines.append(f"period[ms]  mean {sum(periods) / len(periods) / 1e6:.3f} max {periods[-1] / 1e6:.3f}")
    if computes:
        lines.append(f"compute[us] mean {sum(computes) / len(computes) / 1e3:.1f} max {computes[-1] / 1e3:.1f}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", help="frame_log.pb inside a motion-spec run archive")
    parser.add_argument("--jsonl", action="store_true", help="emit decoded frames as JSON Lines")
    parser.add_argument("--verify", action="store_true", help="verify manifest/header only")
    parser.add_argument("--recover-runtime-ttl", action="store_true", help="write runtime.ttl from the frame log")
    args = parser.parse_args(argv)
    try:
        if args.recover_runtime_ttl:
            from motion_spec.introspection.runtime_graph import write_runtime_ttl

            run_dir, log_path, _manifest, _schema, _layout = resolve_archive(args.log)
            records, frame_count = runtime_frames(log_path)
            out = write_runtime_ttl(run_dir, records, frame_count=frame_count)
            print(out)
        elif args.verify:
            _run_dir, log_path, _manifest, schema, layout = resolve_archive(args.log)
            validate_header(log_path, schema, layout)
            print("archive OK")
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
