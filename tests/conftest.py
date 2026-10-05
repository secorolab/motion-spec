# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

"""The on-disk generated tree a run is catalogued and archived against."""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest
from support import FRAME, PROVENANCE, SCHEMA

from motion_spec.generation.artifacts import (
    build_frame_layout,
    build_frame_log_header_record,
    field_names_and_format,
)
from motion_spec.runs.provenance import GENERATION_DOCUMENT
from motion_spec.telemetry import frame_log_pb


@pytest.fixture
def source_tree(tmp_path: Path) -> Path:
    """A generated/ tree as generation lays it out, plus the log a run of it would write."""
    path = tmp_path / "source"
    (path / "contract").mkdir(parents=True)
    (path / "contract" / "frame_layout.json").write_text(
        json.dumps(build_frame_layout(SCHEMA), indent=4)
    )
    shutil.copyfile(frame_log_pb.PROTO, path / "contract" / "frame_log.proto")
    (path / GENERATION_DOCUMENT).parent.mkdir()
    (path / GENERATION_DOCUMENT).write_text(json.dumps(PROVENANCE, indent=4))
    (path / "model").mkdir()
    (path / "model" / "model-app.ld.json").write_text(json.dumps(PROVENANCE, indent=4))
    (path / "model" / "ir.json").write_text(
        json.dumps({"configuration": {"platform": {"simulated": True}}})
    )
    (path / "controller" / "headers").mkdir(parents=True)
    (path / "controller" / "headers" / "runtime.hpp").write_text("// generated\n")
    (path / "controller" / "main.cpp").write_text("// generated\n")
    flat = {name: 0 for name in field_names_and_format(SCHEMA["pools"])[1]} | FRAME
    with (path / "frame_log.pb").open("wb") as log:
        frame_log_pb.write_delimited(log, build_frame_log_header_record(SCHEMA))
        frame_log_pb.write_delimited(log, frame_log_pb.frame_record(flat, SCHEMA))
    (path / "frame_log.pb.health.json").write_text(
        json.dumps(
            {
                "attempted_frames": 1,
                "accepted_frames": 1,
                "written_frames": 1,
                "write_errors": 0,
                "dropped_frames": 0,
                "complete": True,
            },
            indent=4,
        )
    )
    return path
