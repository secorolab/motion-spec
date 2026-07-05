# SPDX-License-Identifier: MPL-2.0
"""Static introspection artifacts generated from the enriched motion-spec IR."""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

SCHEMA_VERSION = 1
FRAME_LAYOUT_VERSION = 1
RUNTIME_RDF_CONTRACT_VERSION = 1
FIELD_BYTES = 8
TRIGGER_POOL_SIZE = 32
MSPROV = "https://secorolab.github.io/motion-spec/provenance/"
MSPROV_PREFIX = "msprov:"
PROV_AGENT = "http://www.w3.org/ns/prov#Agent"
PROV_SOFTWARE_AGENT = "http://www.w3.org/ns/prov#SoftwareAgent"
TYPE_PREFIXES = {
    "http://www.w3.org/ns/prov#": "prov:",
    "https://secorolab.github.io/metamodels/acceptance-criteria/bdd#": "bdd:",
    "https://secorolab.github.io/metamodels/agent#": "agn:",
    "https://secorolab.github.io/metamodels/observation#": "obs:",
    "https://secorolab.github.io/metamodels/runtime#": "rt:",
}
METAMODEL_CONTEXTS = [
    ("https://secorolab.github.io/metamodels/prov.json", Path("prov.json"), True),
    (
        "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/agent.json",
        Path("acceptance-criteria/bdd/agent.json"),
        True,
    ),
    (
        "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/bdd.json",
        Path("acceptance-criteria/bdd/bdd.json"),
        True,
    ),
    (
        "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/observation.json",
        Path("acceptance-criteria/bdd/observation.json"),
        True,
    ),
    (
        "https://secorolab.github.io/metamodels/runtime/runtime.json",
        Path("runtime/runtime.json"),
        False,
    ),
]

HEADER = [
    ("seq", "Q"),
    ("t", "d"),
    ("step", "Q"),
    ("fsm_state", "q"),
    ("active_motion", "q"),
    ("last_event", "q"),
    ("state_since_t", "d"),
    ("state_since_wall_ns", "q"),
    ("event_t", "d"),
    ("event_wall_ns", "q"),
    ("wall_ns", "q"),
    ("period_ns", "q"),
    ("compute_ns", "q"),
]
CSLOT = [
    ("active", "q"),
    ("error", "d"),
    ("output", "d"),
    ("satisfied", "q"),
    ("sat_t", "d"),
    ("measured", "d"),
    ("setpoint", "d"),
]
MSLOT = [("active", "q"), ("value", "d"), ("satisfied", "q"), ("sat_t", "d")]
TSLOT = [("kind", "q"), ("idx", "q"), ("fsm_state", "q"), ("t", "d"), ("wall_ns", "q")]


def fields_with_offsets(pools: dict) -> tuple[list[dict], int]:
    fields = []
    offset = 0

    def add(name: str, fmt: str) -> None:
        nonlocal offset
        fields.append({"name": name, "fmt": fmt, "offset": offset, "size": FIELD_BYTES})
        offset += FIELD_BYTES

    for name, fmt in HEADER:
        add(name, fmt)
    for idx in range(pools["constraints"]):
        for name, fmt in CSLOT:
            add(f"c{idx}.{name}", fmt)
    for idx in range(pools["monitors"]):
        for name, fmt in MSLOT:
            add(f"m{idx}.{name}", fmt)
    for idx in range(pools["quantities"]):
        add(f"q{idx}", "d")
    for idx in range(pools["triggers"]):
        for name, fmt in TSLOT:
            add(f"tr{idx}.{name}", fmt)
    add("trigger_count", "q")
    return fields, offset


def _uri_by_id(ir: dict) -> dict:
    return {
        row["id"]: row["uri"]
        for row in ir.get("introspection", {}).get("uris", ir.get("uris", []))
        if isinstance(row, dict) and row.get("id") and row.get("uri")
    }


def _slug(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "item"


def _prov_iri(identifier: str) -> str:
    kind, _, name = identifier.partition(":")
    if not name:
        kind, name = "id", identifier
    return f"{MSPROV_PREFIX}{_slug(kind)}/{_slug(name)}"


def _location_iri(value: str | None) -> str | None:
    if not value:
        return None
    if value.startswith(("http://", "https://", "file://")):
        return value
    return Path(value).resolve().as_uri()


def _agent_types(types: list[str]) -> list[str]:
    result = list(types)
    has_agent = PROV_AGENT in result or "prov:Agent" in result
    has_software_agent = PROV_SOFTWARE_AGENT in result or "prov:SoftwareAgent" in result
    if has_software_agent and not has_agent:
        result.append(PROV_AGENT)
    return result


def _compact_type(type_id: str) -> str:
    for base, prefix in TYPE_PREFIXES.items():
        if type_id.startswith(base):
            return prefix + type_id[len(base) :]
    return type_id


def _compact_types(types: list[str]) -> list[str]:
    return [_compact_type(type_id) for type_id in types]


def _metamodels_root() -> Path:
    roots = []
    for start in (Path.cwd(), Path(__file__).resolve()):
        roots.extend([start, *start.parents])
    for root in roots:
        for candidate in (root / "src" / "metamodels", root / "metamodels"):
            if (candidate / "prov.json").exists():
                return candidate
    raise RuntimeError("Could not locate src/metamodels for local JSON-LD context fallback.")


def _metamodel_contexts() -> list[str]:
    root = _metamodels_root()
    contexts = []
    for url, local_path, is_published in METAMODEL_CONTEXTS:
        contexts.append(url if is_published else (root / local_path).resolve().as_uri())
    return contexts


def _signal_id(value) -> str | None:
    if isinstance(value, dict):
        return value.get("id")
    if isinstance(value, str):
        return value
    return None


def _controller_slot(controller: dict, index: int, motion: dict, uri_by_id: dict) -> dict:
    error_id = _signal_id(controller.get("error_signal"))
    output_id = _signal_id(controller.get("control_signal")) or controller.get("output_signal")
    reference_id = _signal_id(controller.get("reference_signal"))
    return {
        "index": index,
        "id": controller.get("id"),
        "uri": uri_by_id.get(controller.get("id")),
        "motion": motion.get("id"),
        "type": controller.get("type"),
        "gains": {
            key: controller[key]
            for key in (
                "proportional_gain",
                "integral_gain",
                "derivative_gain",
                "decay_rate",
                "stiffness",
                "damping",
            )
            if controller.get(key) is not None
        },
        "error_signal": error_id,
        "error_signal_uri": uri_by_id.get(error_id),
        "reference_signal": reference_id,
        "reference_signal_uri": uri_by_id.get(reference_id),
        "output_signal": output_id,
        "output_signal_uri": uri_by_id.get(output_id),
    }


def _monitor_slot(monitor: dict, index: int, motion: dict, uri_by_id: dict, phase: str) -> dict:
    error_id = _signal_id(monitor.get("error")) or monitor.get("error_signal")
    event_id = monitor.get("event")
    return {
        "index": index,
        "id": monitor.get("id"),
        "uri": uri_by_id.get(monitor.get("id")),
        "motion": motion.get("id"),
        "phase": phase,
        "type": monitor.get("monitor_type") or monitor.get("type"),
        "trigger": "edge" if monitor.get("is_edge_triggered") else "level",
        "event": event_id,
        "event_uri": monitor.get("event_uri") or uri_by_id.get(event_id),
        "event_name": monitor.get("event_name"),
        "flag": monitor.get("flag"),
        "error_signal": error_id,
        "error_signal_uri": uri_by_id.get(error_id),
        "fallback_motion": monitor.get("fallback_motion"),
    }


def _fsm_meta(fsm_ir: dict | None) -> dict:
    if not fsm_ir:
        return {
            "namespace": None,
            "start": None,
            "end": None,
            "states": [],
            "events": [],
            "transitions": [],
        }
    state_index = {state: idx for idx, state in enumerate(fsm_ir.get("states", []))}
    event_index = {event: idx for idx, event in enumerate(fsm_ir.get("events", []))}
    transition_event = {
        reaction.get("do_transition"): reaction.get("when_event")
        for reaction in fsm_ir.get("reactions_table", [])
    }
    return {
        "namespace": fsm_ir.get("namespace_uri"),
        "start": state_index.get(fsm_ir.get("start_state")),
        "end": state_index.get(fsm_ir.get("end_state")),
        "states": [
            {
                "index": idx,
                "id": state,
                "uri": (fsm_ir.get("state_uris") or {}).get(state),
                "motion": None,
            }
            for state, idx in state_index.items()
        ],
        "events": [
            {
                "index": idx,
                "id": event,
                "uri": (fsm_ir.get("event_uris") or {}).get(event),
            }
            for event, idx in event_index.items()
        ],
        "transitions": [
            {
                "id": transition.get("id"),
                "uri": transition.get("uri"),
                "from": state_index.get(transition.get("from_state")),
                "to": state_index.get(transition.get("to_state")),
                "event": transition_event.get(transition.get("id")),
                "event_index": event_index.get(transition_event.get(transition.get("id"))),
            }
            for transition in fsm_ir.get("transitions_table", [])
        ],
    }


def build_schema(ir: dict, *, ir_path: Path, output_dir: Path, fsm_ir: dict | None) -> dict:
    introspection = ir.get("introspection") or {}
    uri_by_id = _uri_by_id(ir)
    fsm = _fsm_meta(fsm_ir)
    motions = ir.get("unique_motions") or ir.get("motions", [])
    motion_by_id = {motion.get("id"): motion for motion in motions}
    states = fsm["states"]
    state_by_id = {state["id"]: state for state in states}
    by_state = {}

    for motion in motions:
        state_id = motion.get("fsm_state") or motion.get("id")
        if state_id in state_by_id:
            state_by_id[state_id]["motion"] = motion.get("id")
        elif not fsm_ir:
            states.append(
                {
                    "index": len(states),
                    "id": state_id,
                    "uri": uri_by_id.get(motion.get("id")),
                    "motion": motion.get("id"),
                }
            )
        else:
            continue

        controller_slots = [
            _controller_slot(controller, idx, motion, uri_by_id)
            for idx, controller in enumerate(motion.get("controllers", []))
        ]
        monitor_sources = []
        for phase in ("while", "until"):
            monitor_sources.extend((phase, monitor, motion) for monitor in motion.get(f"{phase}_monitors", []))
        for gate_motion_id in motion.get("fsm_when_gate_motions", []):
            gate_motion = motion_by_id.get(gate_motion_id)
            if gate_motion is None:
                continue
            monitor_sources.extend(
                ("when", monitor, gate_motion)
                for monitor in gate_motion.get("when_monitors", [])
            )
        monitor_slots = [
            _monitor_slot(monitor, idx, owner, uri_by_id, phase)
            for idx, (phase, monitor, owner) in enumerate(monitor_sources)
        ]
        by_state[state_id] = {
            "controllers": controller_slots,
            "monitors": monitor_slots,
        }

    quantities = [
        {
            "index": idx,
            **quantity,
        }
        for idx, quantity in enumerate(introspection.get("quantities", []))
    ]
    pools = {
        "constraints": max((len(entry["controllers"]) for entry in by_state.values()), default=0),
        "monitors": max((len(entry["monitors"]) for entry in by_state.values()), default=0),
        "quantities": len(quantities),
        "triggers": max(TRIGGER_POOL_SIZE, len(fsm.get("events", []))),
    }
    provenance = introspection.get("provenance", {})
    contexts = {
        item["id"]: item["uri"]
        for item in provenance.get("contexts", [])
        if item.get("id") and item.get("uri")
    }
    runtime_provenance = _runtime_provenance(provenance)
    schema = {
        "schema_version": SCHEMA_VERSION,
        "frame_layout_version": FRAME_LAYOUT_VERSION,
        "runtime_rdf_contract_version": RUNTIME_RDF_CONTRACT_VERSION,
        "generated_by": "motion_spec.codegen",
        "ir_path": str(ir_path),
        "output_dir": str(output_dir),
        "graph": next(
            (
                entity.get("path")
                for entity in provenance.get("entities", [])
                if entity.get("role") == "app_manifest"
            ),
            str(ir_path),
        ),
        "context": contexts,
        "pools": pools,
        "timing": {"nominal_period_ns": introspection.get("control_period_ns")},
        "control_period_ns": introspection.get("control_period_ns"),
        "fsm": fsm,
        "by_state": by_state,
        "motions": introspection.get("motions", []),
        "controllers": introspection.get("controllers", []),
        "monitors": introspection.get("monitors", []),
        "quantities": quantities,
        "signals": introspection.get("signals", []),
        "provenance_contexts": provenance.get("contexts", []),
        "runtime_provenance": runtime_provenance,
    }
    schema["schema_hash"] = hashlib.sha256(
        json.dumps(schema, sort_keys=True).encode()
    ).hexdigest()[:16]
    return schema


def _runtime_provenance(provenance: dict) -> dict:
    activity = next(
        (
            item
            for item in provenance.get("activities", [])
            if item.get("role") == "controller_execution"
            or item.get("id") == "activity:controller_execution"
        ),
        None,
    )
    if activity is None or not activity.get("wasAssociatedWith"):
        raise RuntimeError(
            "Introspection provenance must identify the controller execution activity "
            "and associated producer agent."
        )
    producer_id = activity["wasAssociatedWith"]
    producer = next(
        (item for item in provenance.get("agents", []) if item.get("id") == producer_id),
        None,
    )
    if producer is None:
        raise RuntimeError(
            f"Introspection provenance activity {activity['id']} references missing agent {producer_id}."
        )
    return {
        "activity_id": activity["id"],
        "producer_agent_id": producer_id,
        "runtime_agent_id": producer.get("actedOnBehalfOf"),
    }


def build_frame_layout(schema: dict) -> dict:
    fields, frame_size = fields_with_offsets(schema["pools"])
    layout = {
        "frame_layout_version": FRAME_LAYOUT_VERSION,
        "schema_version": schema["schema_version"],
        "runtime_rdf_contract_version": schema["runtime_rdf_contract_version"],
        "pools": schema["pools"],
        "field_bytes": FIELD_BYTES,
        "frame_size_bytes": frame_size,
        "schema_hash": schema["schema_hash"],
        "runtime_provenance": schema["runtime_provenance"],
        "fields": fields,
    }
    layout["frame_layout_hash"] = hashlib.sha256(
        json.dumps(layout, sort_keys=True).encode()
    ).hexdigest()[:16]
    return layout


def _cpp_string(value: str | None) -> str:
    return json.dumps(value or "")


def _shared_expr(signal_id: str | None, shared_ids: set[str]) -> str:
    if signal_id and signal_id in shared_ids:
        return f"shared.{signal_id}"
    return "0.0"


def build_introspection_model(schema: dict, ir: dict) -> dict:
    shared_ids = {
        item.get("id")
        for item in ir.get("shared_data", [])
        if isinstance(item, dict) and item.get("id")
    }
    states = []
    for state in schema.get("fsm", {}).get("states", []):
        state_id = state.get("id")
        if state_id not in schema.get("by_state", {}):
            continue
        entry = schema["by_state"][state_id]
        controllers = []
        for slot in entry.get("controllers", []):
            controllers.append(
                {
                    "uri": _cpp_string(slot.get("uri")),
                    "error_expr": _shared_expr(slot.get("error_signal"), shared_ids),
                    "output_expr": _shared_expr(slot.get("output_signal"), shared_ids),
                }
            )
        monitors = []
        for slot in entry.get("monitors", []):
            monitors.append(
                {
                    "uri": _cpp_string(slot.get("uri")),
                    "value_expr": _shared_expr(slot.get("error_signal"), shared_ids),
                }
            )
        states.append(
            {
                "index": state.get("index", -1),
                "controllers": controllers,
                "monitors": monitors,
            }
        )
    return {"states": states}


def build_provenance_document(ir: dict, output_dir: Path) -> dict:
    prov = (ir.get("introspection") or {}).get("provenance", {})
    graph = []

    def add_node(identifier: str, types: list[str], **properties) -> str:
        node_id = _prov_iri(identifier)
        node = {"@id": node_id, "@type": _compact_types(types)}
        node.update(
            {k: v for k, v in properties.items() if k != "role" and v is not None and v != []}
        )
        graph.append(node)
        return node_id

    input_entity_ids = []
    for entity in prov.get("entities", []):
        entity_id = add_node(
            entity.get("id", "entity"),
            entity.get("types") or ["prov:Entity"],
            role=entity.get("role"),
            atLocation=_location_iri(entity.get("path") or entity.get("source")),
            wasGeneratedBy=_prov_iri(entity["wasGeneratedBy"]) if entity.get("wasGeneratedBy") else None,
            wasDerivedFrom=_prov_iri(entity["wasDerivedFrom"]) if entity.get("wasDerivedFrom") else None,
        )
        if entity.get("role") != "motion_spec_ir":
            input_entity_ids.append(entity_id)

    artifact_entities = {
        name: add_node(
            f"entity:generated_{name}",
            ["prov:Entity"],
            role=f"generated_{name}",
            atLocation=_location_iri(str(output_dir / name)),
            wasGeneratedBy=_prov_iri("activity:code_generation"),
        )
        for name in ("schema.json", "frame_layout.json", "frame_layout.h", "provenance.jsonld")
    }

    required_agents = {
        activity["wasAssociatedWith"]
        for activity in prov.get("activities", [])
        if activity.get("wasAssociatedWith")
    }
    emitted_agents = set()

    for activity in prov.get("activities", []):
        add_node(
            activity.get("id", "activity"),
            activity.get("types") or ["prov:Activity"],
            used=[_prov_iri(item) for item in activity.get("used", [])],
            wasAssociatedWith=_prov_iri(activity["wasAssociatedWith"])
            if activity.get("wasAssociatedWith")
            else None,
        )
    codegen_activity = add_node(
        "activity:code_generation",
        ["prov:Activity"],
        used=input_entity_ids,
        wasAssociatedWith=_prov_iri("agent:motion_spec_codegen"),
    )
    add_node(
        "activity:build",
        ["prov:Activity"],
        used=list(artifact_entities.values()),
        wasInformedBy=codegen_activity,
        wasAssociatedWith=_prov_iri("agent:build_toolchain"),
    )

    for agent in prov.get("agents", []):
        emitted_agents.add(agent.get("id", "agent"))
        add_node(
            agent.get("id", "agent"),
            _agent_types(agent.get("types") or [PROV_AGENT]),
            role=agent.get("role"),
            actedOnBehalfOf=_prov_iri(agent["actedOnBehalfOf"])
            if agent.get("actedOnBehalfOf")
            else None,
        )
    for agent_id in sorted(required_agents - emitted_agents):
        add_node(agent_id, [PROV_SOFTWARE_AGENT, PROV_AGENT])
    add_node(
        "agent:motion_spec_codegen",
        [PROV_SOFTWARE_AGENT, PROV_AGENT, "obs:ObservationProvider"],
        role="code_generator",
    )
    add_node("agent:build_toolchain", [PROV_SOFTWARE_AGENT, PROV_AGENT], role="build_toolchain")
    add_node(
        "agent:replay_process",
        [PROV_SOFTWARE_AGENT, PROV_AGENT],
        role="expected_replay_process",
    )
    add_node(
        "agent:dashboard_process",
        [PROV_SOFTWARE_AGENT, PROV_AGENT],
        role="expected_dashboard_process",
    )

    return {
        "schema_version": SCHEMA_VERSION,
        "runtime_rdf_contract_version": RUNTIME_RDF_CONTRACT_VERSION,
        "@context": [*_metamodel_contexts(), {"msprov": MSPROV}],
        "@graph": [
            {"@id": "msprov:bundle/static-provenance", "@type": "prov:Bundle"},
            *graph,
        ],
    }


def write_introspection_artifacts(ir: dict, *, ir_path: Path, output_dir: Path, fsm_ir: dict | None) -> dict:
    schema = build_schema(ir, ir_path=ir_path, output_dir=output_dir, fsm_ir=fsm_ir)
    layout = build_frame_layout(schema)
    end_state = schema.get("fsm", {}).get("end")
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "schema.json").write_text(json.dumps(schema, indent=4) + "\n")
    (output_dir / "frame_layout.json").write_text(json.dumps(layout, indent=4) + "\n")
    (output_dir / "provenance.jsonld").write_text(
        json.dumps(build_provenance_document(ir, output_dir), indent=4) + "\n"
    )
    return {
        "schema_hash": schema["schema_hash"],
        "frame_layout_hash": layout["frame_layout_hash"],
        "frame_layout": {
            "schema_version": schema["schema_version"],
            "frame_layout_version": layout["frame_layout_version"],
            "runtime_rdf_contract_version": schema["runtime_rdf_contract_version"],
            "pools": schema["pools"],
            "frame_size_bytes": layout["frame_size_bytes"],
            "schema_hash": schema["schema_hash"],
            "frame_layout_hash": layout["frame_layout_hash"],
            "runtime_activity_id": schema["runtime_provenance"]["activity_id"],
            "runtime_producer_agent_id": schema["runtime_provenance"]["producer_agent_id"],
            "runtime_agent_id": schema["runtime_provenance"].get("runtime_agent_id") or "",
            "end_state": end_state if end_state is not None else -1,
            "nominal_period_ns": schema.get("control_period_ns") or 0,
        },
        "model": build_introspection_model(schema, ir),
    }
