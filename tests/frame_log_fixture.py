# SPDX-License-Identifier: MPL-2.0
"""Write generated frame-log fixtures for tests."""

from __future__ import annotations

from pathlib import Path

import json
import shutil

import pytest

from motion_spec.generation.codegen import render_template
from motion_spec.introspection import frame_log_pb
from motion_spec.generation.artifacts import (
    build_frame_log_header_record,
    build_frame_log_proto_fields,
    field_names_and_format,
)


def write_frame_log_proto(path: Path, schema: dict) -> None:
    """Render the semantic ``frame_log.proto`` for a fixture through the real codegen template,
    so archive fixtures exercise the same StringTemplate render production uses (not a copy)."""
    if shutil.which("stst") is None:
        pytest.skip("requires stst to render frame_log.proto")
    protobuf = schema.get("protobuf") or build_frame_log_proto_fields(schema)
    payload = path.parent / ".frame_log_proto_payload.json"
    payload.write_text(json.dumps({"introspection_artifacts": {"frame_layout": {"protobuf": protobuf}}}))
    render_template("stst", "frame_log_proto", payload, path)
    payload.unlink()


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
