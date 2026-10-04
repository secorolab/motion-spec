# SPDX-License-Identifier: MPL-2.0
"""Shared test data: the shipped examples, a minimal schema, a dashboard schema, a stub PROV
document, and the generation loader."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path

from motion_spec.runs.provenance import GRAPH_MOTION_SPEC, PROV_CONTEXT, SCHEMA_VERSION, prov_uri

# Where the installed DSL ships its example models, as `motion-spec examples` reads them.
DSL_MODELS = Path(str(files("motion_spec_dsl") / "models"))

# Each shipped example's directory by its name, whatever its order number.
EXAMPLES = {path.name[3:]: path for path in DSL_MODELS.glob("[0-9][0-9]_*")}

METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"

# Nothing recomputes a schema hash after generation; it only has to agree across the files.
SCHEMA = {
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
    "schema_version": 1,
    "frame_layout_version": 1,
    "runtime_rdf_contract_version": 1,
    "pools": {"constraints": 1, "monitors": 1, "quantities": 1, "triggers": 2, "poses": 1},
    "spatial": {
        "poses": [{"index": 0, "id": "tcp_pose", "uri": "https://example.test/tcp_pose"}],
        "twists": [],
        "wrenches": [],
    },
    "quantities": [{"index": 0, "id": "dist", "uri": "https://example.test/dist"}],
    "fsm": {
        "states": [{"index": 0, "id": "S_MOVE", "uri": "https://example.test/S_MOVE"}],
        "events": [],
        "transitions": [],
        "end": 0,
    },
    "by_motion": {
        "move": {
            "index": 0,
            "id": "move",
            "controllers": [
                {
                    "index": 0,
                    "id": "ctrl_x",
                    "uri": "https://example.test/ctrl_x",
                    "constraint_uri": "https://example.test/constraint_x",
                }
            ],
            "monitors": [{"index": 0, "id": "done_mon", "uri": "https://example.test/done_mon"}],
        }
    },
    "platform": {"name": "MuJoCo", "simulated": True, "backend": "mj_kdl"},
    "schema_hash": "fedcba9876543210",
}

# The generation document in miniature: one named graph holding one transformation.
PROVENANCE = {
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


def load_model(authored, out: Path):
    """A parsed robmot loaded as generation loads it, in memory: `(Model, framed FSM or None)`."""
    import rdflib
    from coord_dsl.generators.fsm import gen_json
    from coord_dsl.rdf.fsm import get_fsm_graph
    from motion_spec_dsl.gens import generate
    from scene_dsl.rdf.scenex import create_scenex_model_graph

    from motion_spec.rdf_parser.model import Model

    dataset, _provenance = generate(authored, out)
    fsm = None
    for imported in authored.imports:
        for document in imported._tx_loaded_models:
            source = Path(document._tx_filename).resolve()
            if source.suffix == ".scenex":
                graph = create_scenex_model_graph(document)
            elif source.suffix == ".fsm":
                graph, fsm_ref = get_fsm_graph(document)
                fsm = {**gen_json(graph, fsm_ref), "namespace_uri": document.fsm.ns.uri}
            else:
                continue
            named = dataset.graph(rdflib.URIRef(source.as_uri()))
            named += graph
    model = Model(
        graph=dataset,
        app_path=(out / f"{Path(authored._tx_filename).stem}-app.ld.json").resolve(),
        namespaces=tuple(namespace.uri for namespace in authored.namespaces),
    )
    return model, fsm
