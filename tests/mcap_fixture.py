# SPDX-License-Identifier: MPL-2.0
"""Write a frame_log.mcap fixture mirroring the generated C++ writer (tests only)."""

from __future__ import annotations

import json
from pathlib import Path

from mcap.writer import Writer

from motion_spec.introspection.frame_layout_spec import (
    field_names_and_format,
    frame_json_schema,
    quantity_ids,
)
from motion_spec.introspection.replay import to_record


def flat_frame(schema: dict, **values) -> dict:
    """A zeroed flat frame (all struct field names) with ``values`` applied."""
    _fmt, names = field_names_and_format(schema["pools"])
    flat = {name: 0 for name in names}
    flat.update(values)
    return flat


def records_from_flats(schema: dict, flats: list[dict]) -> list[dict]:
    pools = schema["pools"]
    qids = quantity_ids(schema["quantities"])
    return [
        to_record(flat, pools["constraints"], pools["monitors"], pools["quantities"], pools["triggers"], qids)
        for flat in flats
    ]


def write_frame_log_mcap(path: Path, schema: dict, layout: dict, records: list[dict]) -> None:
    with open(path, "wb") as fh:
        writer = Writer(fh)
        writer.start()
        schema_id = writer.register_schema(
            name="motion_spec.introspection.Frame",
            encoding="jsonschema",
            data=json.dumps(frame_json_schema(schema["quantities"])).encode(),
        )
        channel_id = writer.register_channel(
            topic="/motion_spec/frame",
            message_encoding="json",
            schema_id=schema_id,
            metadata={
                "schema_hash": schema["schema_hash"],
                "frame_layout_hash": layout["frame_layout_hash"],
                "producer_agent_id": schema["runtime_provenance"]["producer_agent_id"],
                "activity_id": schema["runtime_provenance"]["activity_id"],
            },
        )
        for rec in records:
            wall = int(rec["timing"]["wall_ns"])
            writer.add_message(
                channel_id=channel_id,
                log_time=wall,
                publish_time=wall,
                data=json.dumps(rec).encode(),
                sequence=int(rec["step"]) & 0xFFFFFFFF,
            )
        writer.finish()
