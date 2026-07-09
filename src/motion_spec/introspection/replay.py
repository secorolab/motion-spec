# SPDX-License-Identifier: MPL-2.0
"""Replay generated introspection frame logs (``.mcap``) from a run archive."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Iterator
from pathlib import Path

from mcap.reader import make_reader

from motion_spec.introspection.archive import ArchiveError, verify_manifest
from motion_spec.introspection.frame_layout_spec import CSLOT, MSLOT, TSLOT

FRAME_TOPIC = "/motion_spec/frame"
LOG_TOPIC = "/motion_spec/log"


def run_dir_for(log_path: Path) -> Path:
    if not log_path.exists():
        raise ArchiveError(f"{log_path}: does not exist")
    if log_path.is_dir() and (log_path / "manifest.json").exists():
        return log_path
    for path in (log_path.parent, *log_path.parent.parents):
        if (path / "manifest.json").exists():
            return path
    return log_path.parent


def resolve_archive(path: Path | str) -> tuple[Path, Path, dict, dict, dict]:
    input_path = Path(path)
    run_dir = run_dir_for(input_path)
    manifest = verify_manifest(run_dir)
    files = manifest["files"]
    schema = json.loads((run_dir / files["schema"]).read_text())
    layout = json.loads((run_dir / files["frame_layout"]).read_text())
    log_path = run_dir / files["frame_log"] if input_path.is_dir() else input_path
    if not log_path.exists():
        raise ArchiveError(f"{log_path}: missing frame log")
    return run_dir, log_path, manifest, schema, layout


def load_archive(log_path: Path | str) -> tuple[Path, dict, dict, dict]:
    run_dir, _log_path, manifest, schema, layout = resolve_archive(log_path)
    return run_dir, manifest, schema, layout


def _frame_channel_metadata(reader) -> dict:
    """Metadata carried on the ``/motion_spec/frame`` channel (schema/layout hashes and
    runtime provenance the writer stamped there)."""
    summary = reader.get_summary()
    if summary is not None:
        for channel in summary.channels.values():
            if channel.topic == FRAME_TOPIC:
                return dict(channel.metadata)
    # No summary section: fall back to the first frame message's channel.
    for _schema, channel, _message in reader.iter_messages(topics=[FRAME_TOPIC]):
        return dict(channel.metadata)
    raise ArchiveError(f"{FRAME_TOPIC}: channel not found in mcap")


def read_meta(log_path: Path | str) -> dict:
    with Path(log_path).open("rb") as fh:
        return _frame_channel_metadata(make_reader(fh))


def validate_header(log_path: Path | str, schema: dict, layout: dict) -> dict:
    meta = read_meta(log_path)
    if meta.get("schema_hash") != schema.get("schema_hash"):
        raise ArchiveError(
            f"{log_path}: channel schema_hash {meta.get('schema_hash')} != schema.json "
            f"{schema.get('schema_hash')}"
        )
    if meta.get("frame_layout_hash") != layout.get("frame_layout_hash"):
        raise ArchiveError(
            f"{log_path}: channel frame_layout_hash {meta.get('frame_layout_hash')} != "
            f"frame_layout.json {layout.get('frame_layout_hash')}"
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


def _iter_records(log_path: Path | str) -> Iterator[dict]:
    """Decoded ``/motion_spec/frame`` messages — already the nested record shape the C++
    writer emits (`to_record` form), so no reshaping on read."""
    with Path(log_path).open("rb") as fh:
        reader = make_reader(fh)
        for _schema, _channel, message in reader.iter_messages(topics=[FRAME_TOPIC]):
            yield json.loads(message.data)


def frames(log_path: Path | str) -> Iterator[dict]:
    _run_dir, log_path, _manifest, schema, layout = resolve_archive(log_path)
    validate_header(log_path, schema, layout)
    yield from _iter_records(log_path)


def to_record(flat: dict, n_c: int, n_m: int, n_q: int, n_t: int, quantity_ids: list[str]) -> dict:
    """Canonical nested frame record from a flat name->value map. Mirrors the C++ writer;
    used by tests/fixtures (the read path gets this shape straight from the mcap)."""
    record = {
        key: flat[key]
        for key in (
            "t",
            "step",
            "fsm_state",
            "active_motion",
            "last_event",
            "state_since_t",
            "state_since_wall_ns",
            "event_t",
            "event_wall_ns",
        )
    }
    record["timing"] = {key: flat[key] for key in ("wall_ns", "period_ns", "compute_ns")}
    record["constraints"] = [{name: flat[f"c{idx}.{name}"] for name, _ in CSLOT} for idx in range(n_c)]
    record["monitors"] = [{name: flat[f"m{idx}.{name}"] for name, _ in MSLOT} for idx in range(n_m)]
    record["quantities"] = {quantity_ids[idx]: flat[f"q{idx}"] for idx in range(n_q)}
    start = max(0, flat["trigger_count"] - n_t)
    record["triggers"] = [
        {name: flat[f"tr{idx % n_t}.{name}"] for name, _ in TSLOT}
        for idx in range(start, flat["trigger_count"])
    ]
    return record


def decode_frames(log_path: Path | str) -> list[dict]:
    _run_dir, log_path, _manifest, schema, layout = resolve_archive(log_path)
    validate_header(log_path, schema, layout)
    return list(_iter_records(log_path))


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
    for record in _iter_records(log_path):
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
            f"written {health.get('mcap_written_frames')} "
            f"dropped {health.get('dropped_frames')}"
        )
    if periods:
        lines.append(f"period[ms]  mean {sum(periods) / len(periods) / 1e6:.3f} max {periods[-1] / 1e6:.3f}")
    if computes:
        lines.append(f"compute[us] mean {sum(computes) / len(computes) / 1e3:.1f} max {computes[-1] / 1e3:.1f}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", help="frame_log.mcap inside a motion-spec run archive")
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
