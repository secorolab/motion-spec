# SPDX-License-Identifier: MPL-2.0
"""Write generated frame-log fixtures for tests."""

from __future__ import annotations

import struct
from pathlib import Path

from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.frame_layout_spec import field_names_and_format


def flat_frame(schema: dict, **values) -> dict:
    """A zeroed flat frame (all struct field names) with ``values`` applied."""
    _fmt, names = field_names_and_format(schema["pools"])
    flat = {name: 0 for name in names}
    flat.update(values)
    return flat


def write_frame_log_pb(path: Path, schema: dict, layout: dict, flats: list[dict]) -> None:
    fmt, names = field_names_and_format(schema["pools"])
    with open(path, "wb") as fh:
        frame_log_pb.write_delimited(fh, frame_log_pb.header_record(schema, layout))
        for flat in flats:
            frame = struct.pack(fmt, *(flat[name] for name in names))
            frame_log_pb.write_delimited(
                fh,
                frame_log_pb.frame_record(int(flat["step"]), int(flat["wall_ns"]), frame),
            )
