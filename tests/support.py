# SPDX-License-Identifier: MPL-2.0
"""Shared archive/runtime-graph test fixtures: a minimal maintained schema, its derived
frame layout, a stub PROV document, and the on-disk source tree they assemble into."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from motion_spec.generation.artifacts import fields_with_offsets

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
        "graph": "model.ld.json",
        "context": {},
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
        "provenance_contexts": [{"id": "prov", "source": "src/metamodels/prov.json"}],
        "runtime_provenance": {
            "activity_id": "activity:controller_execution",
            "producer_agent_id": "agent:controller_process",
            "runtime_agent_id": "agent:runtime:mujoco",
        },
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
        "runtime_provenance": schema["runtime_provenance"],
        "fields": fields,
    }
    layout["frame_layout_hash"] = _hash_doc(layout)
    return layout


def _provenance() -> dict:
    return {
        "@context": {
            "prov": "http://www.w3.org/ns/prov#",
            "Entity": "prov:Entity",
            "Activity": "prov:Activity",
            "Agent": "prov:Agent",
            "SoftwareAgent": "prov:SoftwareAgent",
            "used": {"@id": "prov:used", "@type": "@id"},
            "wasGeneratedBy": {"@id": "prov:wasGeneratedBy", "@type": "@id"},
            "wasAssociatedWith": {"@id": "prov:wasAssociatedWith", "@type": "@id"},
            "atLocation": {"@id": "prov:atLocation", "@type": "@id"},
        },
        "@graph": [
            {"@id": "https://example.test/entity/schema", "@type": "Entity"},
            {
                "@id": "https://example.test/entity/run_bin",
                "@type": "Entity",
                "wasGeneratedBy": "https://example.test/activity/run",
            },
            {
                "@id": "https://example.test/activity/run",
                "@type": "Activity",
                "used": "https://example.test/entity/schema",
                "wasAssociatedWith": "https://example.test/agent/producer",
            },
            {"@id": "https://example.test/agent/producer", "@type": ["SoftwareAgent", "Agent"]},
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


def _source_tree(path: Path) -> Path:
    schema = _schema()
    layout = _layout(schema)
    path.mkdir()
    (path / "schema.json").write_text(json.dumps(schema, indent=4))
    (path / "frame_layout.json").write_text(json.dumps(layout, indent=4))
    write_frame_log_proto(path / "frame_log.proto", schema)
    (path / "provenance.ld.json").write_text(json.dumps(_provenance(), indent=4))
    (path / "provenance").mkdir()
    (path / "provenance" / "dsl.ld.json").write_text(json.dumps(_provenance(), indent=4))
    (path / "model.ld.json").write_text(json.dumps(_provenance(), indent=4))
    (path / "ir.json").write_text(json.dumps({"id": "test-ir"}))
    (path / "headers").mkdir()
    (path / "headers" / "runtime.hpp").write_text("// generated\n")
    (path / "ref_main.cpp").write_text("// generated\n")
    _write_frame_log(path / "frame_log.pb", schema)
    (path / "frame_log.pb.health.json").write_text(
        json.dumps(
            {
                "attempted_frames": 1,
                "accepted_frames": 1,
                "written_frames": 1,
                "dropped_frames": 0,
                "complete": True,
            },
            indent=4,
        )
    )
    return path
