# SPDX-License-Identifier: MPL-2.0
"""A schema exercising every slot category, for the dashboard tests.

Deliberately not the minimal one in support.py: the shm reader has to resolve constraint,
monitor, quantity, trigger and spatial slots, and a gated motion, to prove it yields the same
record the frame log does.
"""

from __future__ import annotations

from support import _hash_doc

MOVE = "https://example.test/S_MOVE"
CTRL = "https://example.test/ctrl_x"
CONSTRAINT = "https://example.test/constraint_x"
ERROR_SIGNAL = "https://example.test/err_x"
OUTPUT_SIGNAL = "https://example.test/out_x"
MONITOR = "https://example.test/done_mon"
QUANTITY = "https://example.test/dist"
POSE = "https://example.test/tcp_pose"


def schema() -> dict:
    doc = {
        "schema_version": 1,
        "frame_layout_version": 1,
        "runtime_rdf_contract_version": 1,
        "pools": {"constraints": 1, "monitors": 1, "quantities": 1, "triggers": 2, "poses": 1},
        "spatial": {
            "poses": [{"index": 0, "id": "tcp_pose", "uri": POSE}],
            "twists": [],
            "wrenches": [],
        },
        "quantities": [{"index": 0, "id": "dist", "uri": QUANTITY}],
        "fsm": {
            "states": [{"index": 0, "id": "S_MOVE", "uri": MOVE}],
            "events": [],
            "transitions": [],
            "end": 0,
        },
        "by_motion": {
            # No motion uri: the writer then names each occupancy by its state, which keeps
            # these views' story per-state -- the design fallback for a contract this old.
            "move": {
                "index": 0,
                "id": "move",
                "controllers": [
                    {"index": 0, "id": "ctrl_x", "uri": CTRL, "constraint_uri": CONSTRAINT}
                ],
                "monitors": [{"index": 0, "id": "done_mon", "uri": MONITOR}],
            }
        },
        "platform": {"name": "MuJoCo", "simulated": True, "backend": "mj_kdl"},
        "runtime_provenance": {
            "activity_id": "activity:controller_execution",
            "producer_agent_id": "agent:controller_process",
            "runtime_agent_id": "agent:runtime:mujoco",
        },
    }
    doc["schema_hash"] = _hash_doc(doc)
    return doc


def model_jsonld() -> dict:
    """The controller's signal declarations -- what a value observation observes."""
    return {
        "@context": {
            "cstr-hdl": "https://comp-rob2b.github.io/metamodels/task/constraint-handler#",
            "error-signal": {"@id": "cstr-hdl:error-signal", "@type": "@id"},
            "control-signal": {"@id": "cstr-hdl:control-signal", "@type": "@id"},
            "constraint": {"@id": "cstr-hdl:constraint", "@type": "@id"},
        },
        "@graph": [
            {"@id": CTRL, "error-signal": ERROR_SIGNAL, "control-signal": OUTPUT_SIGNAL},
            {"@id": MONITOR, "constraint": CONSTRAINT},
        ],
    }
