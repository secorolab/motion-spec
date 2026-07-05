# SPDX-License-Identifier: MPL-2.0
"""Replay generated introspection frame logs from a self-contained run folder."""

from __future__ import annotations

import argparse
import json
import struct
from collections.abc import Iterator
from pathlib import Path

from motion_spec.introspection.archive import ArchiveError, verify_manifest
from motion_spec.introspection.frame_layout_spec import CSLOT, MSLOT, TSLOT, frame_struct

MAGIC = b"MSFRMBIN"
LEGACY_MAGIC = b"MSRUNBIN"
HEADER = struct.Struct("<8sII32s32s128s128s")


def _cstr(raw: bytes) -> str:
    return raw.split(b"\0", 1)[0].decode("utf-8", errors="replace")


def run_dir_for(log_path: Path) -> Path:
    return log_path.parent


def load_archive(log_path: Path | str) -> tuple[Path, dict, dict, dict]:
    log_path = Path(log_path)
    run_dir = run_dir_for(log_path)
    manifest = verify_manifest(run_dir)
    files = manifest["files"]
    schema = json.loads((run_dir / files["schema"]).read_text())
    layout = json.loads((run_dir / files["frame_layout"]).read_text())
    return run_dir, manifest, schema, layout


def read_header(log_path: Path | str) -> dict:
    with Path(log_path).open("rb") as fh:
        data = fh.read(HEADER.size)
    if len(data) != HEADER.size:
        raise ArchiveError(f"{log_path}: truncated run header")
    magic, writer_version, frame_size, schema_hash, layout_hash, producer, activity = HEADER.unpack(data)
    if magic not in (MAGIC, LEGACY_MAGIC):
        raise ArchiveError(f"{log_path}: bad magic {magic!r}")
    return {
        "writer_version": writer_version,
        "frame_size": frame_size,
        "schema_hash": _cstr(schema_hash),
        "frame_layout_hash": _cstr(layout_hash),
        "producer_agent_id": _cstr(producer),
        "activity_id": _cstr(activity),
    }


def validate_header(log_path: Path | str, schema: dict, layout: dict) -> dict:
    header = read_header(log_path)
    if header["frame_size"] != layout["frame_size_bytes"]:
        raise ArchiveError(
            f"{log_path}: header frame_size {header['frame_size']} != frame_layout.json "
            f"{layout['frame_size_bytes']}"
        )
    if header["schema_hash"] != schema.get("schema_hash"):
        raise ArchiveError(
            f"{log_path}: header schema_hash {header['schema_hash']} != schema.json "
            f"{schema.get('schema_hash')}"
        )
    if header["frame_layout_hash"] != layout.get("frame_layout_hash"):
        raise ArchiveError(
            f"{log_path}: header frame_layout_hash {header['frame_layout_hash']} != "
            f"frame_layout.json {layout.get('frame_layout_hash')}"
        )
    expected = schema.get("runtime_provenance", {})
    if expected.get("producer_agent_id") and header["producer_agent_id"] != expected["producer_agent_id"]:
        raise ArchiveError(f"{log_path}: producer_agent_id does not match schema runtime provenance")
    if expected.get("activity_id") and header["activity_id"] != expected["activity_id"]:
        raise ArchiveError(f"{log_path}: activity_id does not match schema runtime provenance")
    return header


def frames(log_path: Path | str) -> Iterator[tuple[dict, int, int, int, int]]:
    log_path = Path(log_path)
    _run_dir, _manifest, schema, layout = load_archive(log_path)
    validate_header(log_path, schema, layout)
    st, names = frame_struct(schema["pools"])
    n_c = schema["pools"]["constraints"]
    n_m = schema["pools"]["monitors"]
    n_q = schema["pools"]["quantities"]
    n_t = schema["pools"]["triggers"]
    with log_path.open("rb") as fh:
        fh.seek(HEADER.size)
        while True:
            buf = fh.read(st.size)
            if not buf:
                break
            if len(buf) != st.size:
                raise ArchiveError(f"{log_path}: truncated frame payload")
            yield dict(zip(names, st.unpack(buf))), n_c, n_m, n_q, n_t


def to_record(flat: dict, n_c: int, n_m: int, n_q: int, n_t: int) -> dict:
    record = {
        key: flat[key]
        for key in (
            "t",
            "step",
            "fsm_state",
            "active_motion",
            "last_event",
            "state_since_t",
            "event_t",
        )
    }
    record["timing"] = {key: flat[key] for key in ("wall_ns", "period_ns", "compute_ns")}
    record["constraints"] = [{name: flat[f"c{idx}.{name}"] for name, _ in CSLOT} for idx in range(n_c)]
    record["monitors"] = [{name: flat[f"m{idx}.{name}"] for name, _ in MSLOT} for idx in range(n_m)]
    record["quantities"] = [flat[f"q{idx}"] for idx in range(n_q)]
    start = max(0, flat["trigger_count"] - n_t)
    record["triggers"] = [
        {name: flat[f"tr{idx % n_t}.{name}"] for name, _ in TSLOT}
        for idx in range(start, flat["trigger_count"])
    ]
    return record


def decode_frames(log_path: Path | str) -> list[dict]:
    return [to_record(flat, n_c, n_m, n_q, n_t) for flat, n_c, n_m, n_q, n_t in frames(log_path)]


def sampled_frames(log_path: Path | str) -> tuple[list[dict], int]:
    first = last = None
    count = 0
    for flat, n_c, n_m, n_q, n_t in frames(log_path):
        record = to_record(flat, n_c, n_m, n_q, n_t)
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
    run_dir, _manifest, schema, layout = load_archive(log_path)
    header = validate_header(log_path, schema, layout)
    count = 0
    first = last = None
    periods = []
    computes = []
    for flat, n_c, n_m, n_q, n_t in frames(log_path):
        record = to_record(flat, n_c, n_m, n_q, n_t)
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
        f"writer      v{header['writer_version']} {header['producer_agent_id']} {header['activity_id']}",
        f"final state {final_name}",
    ]
    if periods:
        lines.append(f"period[ms]  mean {sum(periods) / len(periods) / 1e6:.3f} max {periods[-1] / 1e6:.3f}")
    if computes:
        lines.append(f"compute[us] mean {sum(computes) / len(computes) / 1e3:.1f} max {computes[-1] / 1e3:.1f}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", help="frame_log.bin inside a motion-spec run archive")
    parser.add_argument("--jsonl", action="store_true", help="emit decoded frames as JSON Lines")
    parser.add_argument("--verify", action="store_true", help="verify manifest/header only")
    parser.add_argument("--recover-runtime-ttl", action="store_true", help="write runtime.ttl from the frame log")
    args = parser.parse_args(argv)
    try:
        if args.recover_runtime_ttl:
            from motion_spec.introspection.runtime_graph import write_runtime_ttl

            samples, frame_count = sampled_frames(args.log)
            out = write_runtime_ttl(Path(args.log).parent, samples, frame_count=frame_count)
            print(out)
        elif args.verify:
            _run_dir, _manifest, schema, layout = load_archive(args.log)
            validate_header(args.log, schema, layout)
            print("archive OK")
        elif args.jsonl:
            for record in decode_frames(args.log):
                print(json.dumps(record, separators=(",", ":")))
        else:
            print(summarize(args.log))
    except ArchiveError as exc:
        parser.exit(2, f"{exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
