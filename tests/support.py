# SPDX-License-Identifier: MPL-2.0
"""Shared test data: the shipped examples, a minimal schema, a dashboard schema, a stub PROV
document, and the generation loader."""

from __future__ import annotations

import importlib.util
from importlib.resources import files
from pathlib import Path

import pytest

from motion_spec.runs.provenance import GRAPH_MOTION_SPEC, PROV_CONTEXT, prov_uri

# A model that publishes or subscribes to a ROS topic resolves its messages through rosidl.
REQUIRES_ROS = pytest.mark.skipif(
    importlib.util.find_spec("rosidl_runtime_py") is None,
    reason="no rosidl_runtime_py; source the ROS distribution",
)

# Where the installed DSL ships its example models, as `motion-spec examples` reads them.
DSL_MODELS = Path(str(files("motion_spec_dsl") / "models"))

# Each shipped example's directory by its name, whatever its order number.
EXAMPLES = {path.name[3:]: path for path in DSL_MODELS.glob("[0-9][0-9]_*")}

METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"

# Nothing recomputes a schema hash after generation; it only has to agree across the files.
SCHEMA = {
    "frame_layout_version": 1,
    "runtime_rdf_contract_version": 1,
    "generated_by": "test",
    "ir_path": "ir.json",
    "pools": {
        "constraints": 1,
        "monitors": 1,
        "quantities": 1,
        "devices": 0,
        "triggers": 2,
        "poses": 0,
        "twists": 0,
        "wrenches": 0,
    },
    "timing": {"nominal_period_ns": 1_000_000},
    "control_period_ns": 1_000_000,
    "fsm": {
        "namespace": "https://example.test/",
        "start": 0,
        "end": 0,
        "states": [{"index": 0, "id": "S_START", "uri": "https://example.test/S_START"}],
        "events": [],
        "transitions": [],
    },
    "by_motion": {},
    "platform": {"name": "MuJoCo", "simulated": True, "backend": "mj_kdl"},
    "quantities": [{"index": 0, "id": "q0"}],
    "devices": [],
    "cameras": [],
    "constants": [],
    "spatial": {"poses": [], "twists": [], "wrenches": []},
    "schema_hash": "0123456789abcdef",
}

# The one logged frame of a run of SCHEMA.
FRAME = {
    "t": 1.25,
    "step": 7,
    "fsm_state": 0,
    "active_motion": -1,
    "last_event": -1,
    "state_since_t": 1.0,
    "event_t": 0.0,
    "wall_ns": 100,
    "period_ns": 1_000_000,
    "compute_ns": 25_000,
    "c0.active": 1,
    "c0.satisfied": 1,
    "m0.active": 1,
    "m0.satisfied": 1,
    "q0": 42.0,
}

# A schema with constraint, monitor, quantity, trigger and spatial slots, for the dashboard.
DASHBOARD_SCHEMA = {
    "frame_layout_version": 1,
    "runtime_rdf_contract_version": 1,
    "pools": {
        "constraints": 1,
        "monitors": 1,
        "quantities": 1,
        "devices": 0,
        "triggers": 2,
        "poses": 1,
        "twists": 0,
        "wrenches": 0,
    },
    "control_period_ns": 1_000_000,
    "devices": [],
    "cameras": [],
    "constants": [],
    "spatial": {
        "poses": [{"index": 0, "id": "tcp_pose", "uri": "https://example.test/tcp_pose"}],
        "twists": [],
        "wrenches": [],
    },
    "quantities": [{"index": 0, "id": "dist", "uri": "https://example.test/dist"}],
    "fsm": {
        "namespace": "https://example.test/",
        "start": 0,
        "end": 0,
        "states": [{"index": 0, "id": "S_MOVE", "uri": "https://example.test/S_MOVE"}],
        "events": [],
        "transitions": [],
    },
    "by_motion": {
        "move": {
            "index": 0,
            "uri": "https://example.test/move",
            "fsm_state": "S_MOVE",
            "controllers": [
                {
                    "index": 0,
                    "id": "ctrl_x",
                    "uri": "https://example.test/ctrl_x",
                    "constraint_uri": "https://example.test/constraint_x",
                    "gains": {},
                }
            ],
            "monitors": [
                {
                    "index": 0,
                    "id": "done_mon",
                    "uri": "https://example.test/done_mon",
                    "event_uri": None,
                    "phase": "until",
                    "watched": [],
                }
            ],
        }
    },
    "platform": {"name": "MuJoCo", "simulated": True, "backend": "mj_kdl"},
    "schema_hash": "fedcba9876543210",
}

# The generation document in miniature: one named graph holding one transformation.
PROVENANCE = {
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
