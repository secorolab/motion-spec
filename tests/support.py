# SPDX-License-Identifier: MPL-2.0
"""Shared archive test fixtures: a minimal maintained schema, its derived frame layout, a stub
PROV document, and the on-disk source tree they assemble into."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from motion_spec.generation.artifacts import fields_with_offsets
from motion_spec.introspection.provenance import (
    GENERATION_DOCUMENT,
    GRAPH_MOTION_SPEC,
    PROV_CONTEXT,
    SCHEMA_VERSION,
    prov_uri,
)

from frame_log_fixture import flat_frame, write_frame_log_pb, write_frame_log_proto


def _hash_doc(doc: dict) -> str:
    return hashlib.sha256(json.dumps(doc, sort_keys=True).encode()).hexdigest()[:16]


def _schema() -> dict:
    schema = {
        "schema_version": 1,
        "frame_layout_version": 1,
        "runtime_rdf_contract_version": 1,
        "generated_by": "test",
        "ir_path": "ir.json",
        "pools": {"constraints": 1, "monitors": 1, "quantities": 1, "triggers": 2},
        "timing": {"nominal_period_ns": 1_000_000},
        "fsm": {
            "states": [{"index": 0, "id": "S_START", "uri": "https://example.test/S_START"}],
            "events": [],
            "end": 0,
        },
        "by_motion": {},
        "platform": {"name": "MuJoCo", "simulated": True, "backend": "mj_kdl"},
        "quantities": [{"index": 0, "id": "q0"}],
    }
    schema["schema_hash"] = _hash_doc(schema)
    return schema


def _layout(schema: dict) -> dict:
    fields, size = fields_with_offsets(schema["pools"])
    layout = {
        "frame_layout_version": 1,
        "schema_version": 1,
        "runtime_rdf_contract_version": 1,
        "pools": schema["pools"],
        "field_bytes": 8,
        "frame_size_bytes": size,
        "schema_hash": schema["schema_hash"],
        "platform": schema["platform"],
        "fields": fields,
    }
    layout["frame_layout_hash"] = _hash_doc(layout)
    return layout


def _provenance() -> dict:
    """The generation document in miniature: one named graph holding one transformation."""
    return {
        "schema_version": SCHEMA_VERSION,
        "@context": PROV_CONTEXT,
        "@graph": [
            {
                "@id": str(GRAPH_MOTION_SPEC),
                "@graph": [
                    {"@id": prov_uri("entity:app_manifest"), "@type": "Entity"},
                    {
                        "@id": prov_uri("entity:motion_spec_ir"),
                        "@type": "Entity",
                        "wasGeneratedBy": prov_uri("activity:motion_spec_ir_generation"),
                    },
                    {
                        "@id": prov_uri("activity:motion_spec_ir_generation"),
                        "@type": ["Activity", "Transformation"],
                        "used": prov_uri("entity:app_manifest"),
                        "wasAssociatedWith": prov_uri("agent:motion_spec"),
                    },
                    {
                        "@id": prov_uri("agent:motion_spec"),
                        "@type": ["Agent", "SoftwareAgent"],
                        "name": "motion-spec",
                    },
                    {
                        "@id": prov_uri("agent:modelled:arm1"),
                        "@type": ["Agent", "ModelledAgent"],
                        "name": "arm1",
                    },
                ],
            }
        ],
    }


def _write_frame_log(path: Path, schema: dict) -> None:
    flat = flat_frame(
        schema,
        t=1.25,
        step=7,
        fsm_state=0,
        active_motion=-1,
        last_event=-1,
        state_since_t=1.0,
        event_t=0.0,
        wall_ns=100,
        period_ns=1_000_000,
        compute_ns=25_000,
        **{"c0.active": 1, "c0.satisfied": 1, "m0.active": 1, "m0.satisfied": 1, "q0": 42.0},
    )
    write_frame_log_pb(path, schema, [flat])


def _start_run(run_dir: Path, source: Path, executable: Path, run_id: str = "run-test") -> None:
    """Catalogue a run the way the runner does before it launches the executable."""
    from motion_spec.introspection.runner import _start_rec_run

    run_dir.mkdir(parents=True, exist_ok=True)
    schema = json.loads((source / "frame_layout.json").read_text())
    _start_rec_run(run_dir, run_id, source, executable, schema, [])


def _source_tree(path: Path) -> Path:
    schema = _schema()
    layout = _layout(schema)
    path.mkdir()
    (path / "schema.json").write_text(json.dumps(schema, indent=4))
    (path / "frame_layout.json").write_text(json.dumps(layout, indent=4))
    write_frame_log_proto(path / "frame_log.proto", schema)
    (path / GENERATION_DOCUMENT).write_text(json.dumps(_provenance(), indent=4))
    (path / "model.ld.json").write_text(json.dumps(_provenance(), indent=4))
    (path / "ir.json").write_text(json.dumps({"id": "test-ir"}))
    (path / "headers").mkdir()
    (path / "headers" / "runtime.hpp").write_text("// generated\n")
    (path / "main.cpp").write_text("// generated\n")
    _write_frame_log(path / "frame_log.pb", schema)
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
