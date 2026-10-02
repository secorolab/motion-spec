# SPDX-License-Identifier: MPL-2.0
"""Write generated frame-log fixtures for tests."""

from __future__ import annotations

import shutil
from pathlib import Path

from motion_spec.generation.artifacts import build_frame_log_header_record, field_names_and_format
from motion_spec.telemetry import frame_log_pb


def write_frame_log_proto(path: Path, schema: dict) -> None:
    """The shipped ``frame_log.proto``, as codegen copies it into a generation's contract."""
    shutil.copyfile(frame_log_pb.PROTO, path)


def flat_frame(schema: dict, **values) -> dict:
    """A zeroed flat frame (all struct field names) with ``values`` applied."""
    _fmt, names = field_names_and_format(schema["pools"])
    flat = {name: 0 for name in names}
    flat.update(values)
    return flat


def write_frame_log_pb(path: Path, schema: dict, flats: list[dict]) -> None:
    with open(path, "wb") as fh:
        # The same header the runtime writes: a fixture log must be as self-describing as a
        # real one, or it exercises a decode path production never takes.
        frame_log_pb.write_delimited(fh, build_frame_log_header_record(schema))
        for flat in flats:
            frame_log_pb.write_delimited(fh, frame_log_pb.frame_record(flat, schema))
