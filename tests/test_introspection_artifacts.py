# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
from pathlib import Path

from rdflib import Graph
from rdf_utils.resolver import IriToFileResolver, install_resolver

from motion_spec import codegen
from motion_spec.introspection.artifacts import (
    build_frame_layout,
    build_provenance_document,
    build_schema,
    fields_with_offsets,
)


def _sample_ir() -> dict:
    return {
        "control_period_ns": 2_000_000,
        "unique_motions": [
            {
                "id": "move",
                "handler": "move_handler",
                "fsm_state": "S_MOVE",
                "controllers": [
                    {
                        "id": "ctrl_x",
                        "type": "ProportionalIntegralDerivative",
                        "proportional_gain": 1.0,
                        "integral_gain": 0.0,
                        "derivative_gain": 0.1,
                        "error_signal": {"id": "err_x"},
                        "control_signal": {"id": "out_x"},
                    }
                ],
                "while_monitors": [],
                "until_monitors": [
                    {
                        "id": "done_mon",
                        "monitor_type": "EdgeTriggeredMonitor",
                        "is_edge_triggered": True,
                        "event": "E_DONE",
                        "event_uri": "https://example.test/fsm/E_DONE",
                        "event_name": "E_DONE",
                    }
                ],
                "fsm_when_gate_motions": ["guarded"],
            },
            {
                "id": "guarded",
                "handler": "guarded_handler",
                "controllers": [],
                "when_monitors": [
                    {
                        "id": "ready_mon",
                        "monitor_type": "LevelTriggeredMonitor",
                        "is_edge_triggered": False,
                        "flag": "ready",
                        "error": {"id": "err_x"},
                        "fallback_motion": "move",
                    }
                ],
            },
        ],
        "introspection": {
            "control_period_ns": 2_000_000,
            "uris": [
                {"id": "move", "uri": "https://example.test/move"},
                {"id": "ctrl_x", "uri": "https://example.test/ctrl_x"},
                {"id": "err_x", "uri": "https://example.test/err_x"},
                {"id": "out_x", "uri": "https://example.test/out_x"},
                {"id": "done_mon", "uri": "https://example.test/done_mon"},
            ],
            "motions": [{"id": "move", "handler": "move_handler"}],
            "controllers": [{"id": "ctrl_x"}],
            "monitors": [{"id": "done_mon"}],
            "quantities": [
                {
                    "id": "err_x",
                    "uri": "https://example.test/err_x",
                    "type": "Quantity",
                    "unit": ["M"],
                    "quantity_kind": ["Length"],
                }
            ],
            "signals": [{"id": "ctrl_x.error_signal", "quantity": "err_x"}],
            "provenance": {
                "contexts": [
                    {"id": "prov", "uri": "http://www.w3.org/ns/prov#"},
                    {
                        "id": "runtime",
                        "uri": "https://secorolab.github.io/metamodels/runtime#Runtime",
                    },
                ],
                "entities": [
                    {
                        "id": "entity:app_manifest",
                        "types": ["prov:Entity"],
                        "role": "app_manifest",
                        "path": "/tmp/app.json",
                    },
                    {
                        "id": "entity:motion_spec_ir",
                        "types": ["prov:Entity"],
                        "role": "motion_spec_ir",
                        "wasGeneratedBy": "activity:motion_spec_ir_generation",
                        "wasDerivedFrom": "entity:app_manifest",
                    }
                ],
                "activities": [
                    {
                        "id": "activity:motion_spec_ir_generation",
                        "types": ["prov:Activity"],
                        "used": ["entity:app_manifest"],
                        "wasAssociatedWith": "agent:motion_spec_ir_gen",
                    },
                    {
                        "id": "activity:controller_execution",
                        "types": [
                            "prov:Activity",
                            "bdd:SimulatedExecution",
                        ],
                        "used": ["entity:motion_spec_ir"],
                        "wasAssociatedWith": "agent:controller_process",
                        "role": "controller_execution",
                    }
                ],
                "agents": [
                    {
                        "id": "agent:runtime:mujoco",
                        "types": [
                            "prov:SoftwareAgent",
                            "rt:MuJoCoRuntime",
                        ],
                        "role": "runtime_runner",
                    },
                    {
                        "id": "agent:controller_process",
                        "types": ["prov:SoftwareAgent"],
                        "role": "controller_process",
                        "actedOnBehalfOf": "agent:runtime:mujoco",
                    },
                    {
                        "id": "agent:modelled:robot",
                        "types": ["prov:Agent", "agn:ModelledAgent"],
                        "role": "robot",
                        "model": "src/mj_kdl_wrapper/third_party/menagerie/kinova_gen3/gen3.xml",
                    },
                ],
            },
        },
    }


def _sample_fsm() -> dict:
    return {
        "namespace_uri": "https://example.test/fsm/",
        "start_state": "S_START",
        "end_state": "S_DONE",
        "states": ["S_START", "S_MOVE", "S_DONE"],
        "state_uris": {
            "S_START": "https://example.test/fsm/S_START",
            "S_MOVE": "https://example.test/fsm/S_MOVE",
            "S_DONE": "https://example.test/fsm/S_DONE",
        },
        "events": ["E_STEP", "E_DONE"],
        "event_uris": {
            "E_STEP": "https://example.test/fsm/E_STEP",
            "E_DONE": "https://example.test/fsm/E_DONE",
        },
        "transitions_table": [
            {"id": "T_START_MOVE", "from_state": "S_START", "to_state": "S_MOVE"},
            {"id": "T_MOVE_DONE", "from_state": "S_MOVE", "to_state": "S_DONE"},
        ],
        "reactions_table": [
            {"when_event": "E_STEP", "do_transition": "T_START_MOVE"},
            {"when_event": "E_DONE", "do_transition": "T_MOVE_DONE"},
        ],
    }


def test_schema_and_frame_layout_are_consistent(tmp_path: Path) -> None:
    schema = build_schema(
        _sample_ir(),
        ir_path=tmp_path / "ir.json",
        output_dir=tmp_path,
        fsm_ir=_sample_fsm(),
    )
    layout = build_frame_layout(schema)
    fields, size = fields_with_offsets(schema["pools"])

    assert schema["schema_version"] == 1
    assert schema["runtime_rdf_contract_version"] == 1
    assert schema["runtime_provenance"] == {
        "activity_id": "activity:controller_execution",
        "producer_agent_id": "agent:controller_process",
        "runtime_agent_id": "agent:runtime:mujoco",
    }
    assert schema["fsm"]["states"][1]["motion"] == "move"
    assert schema["by_state"]["S_MOVE"]["controllers"][0]["id"] == "ctrl_x"
    assert schema["by_state"]["S_MOVE"]["monitors"][0]["id"] == "done_mon"
    assert schema["by_state"]["S_MOVE"]["monitors"][1]["id"] == "ready_mon"
    assert layout["fields"] == fields
    assert layout["frame_size_bytes"] == size
    assert layout["pools"] == schema["pools"]


def test_codegen_samples_logged_quantity_components(tmp_path: Path, monkeypatch) -> None:
    ir = _sample_ir()
    ir.update(
        {
            "backend": "mj_kdl",
            "has_arm": False,
            "has_mobile_base": False,
            "arm_solvers": [],
            "base_velocity_solvers": [],
            "base_force_solvers": [],
            "cstr_hdl": [],
            "motions": ir["unique_motions"],
            "data": [],
            "closures": {
                "ctrl_x": {
                    "id": "ctrl_x",
                    "type": "Controller",
                    "error_signal": "err_x",
                    "control_signal": "out_x",
                },
                "eval_err_x": {
                    "id": "eval_err_x",
                    "type": "ErrorEvaluator",
                    "constraint": "EqualityConstraint",
                    "quantity": "measured_x",
                    "reference_value": "setpoint_x",
                    "error": "err_x",
                }
            },
            "views": {
                "wrench_force": {
                    "superobject": {"id": "wrench_ee", "type": "Wrench"},
                    "subobject": {"id": "wrench_force", "type": "Quantity"},
                    "subspace": "Force",
                    "axis": None,
                },
                "wrench_force_x": {
                    "superobject": {"id": "wrench_ee", "type": "Wrench"},
                    "subobject": {"id": "wrench_force_x", "type": "Quantity"},
                    "subspace": "Force",
                    "axis": "X",
                },
            },
            "shared_schedule": [],
            "schedule": [],
            "shared_data": [
                {"id": "err_x", "type": "Quantity"},
                {"id": "pose_ee", "type": "Pose"},
                {"id": "twist_ee", "type": "VelocityTwist"},
                {"id": "wrench_ee", "type": "Wrench"},
                {"id": "wrench_force", "type": "Quantity"},
                {"id": "wrench_force_x", "type": "Quantity"},
                {"id": "measured_x", "type": "Quantity"},
                {"id": "setpoint_x", "type": "Quantity"},
                {"id": "ready_flag", "type": "Bool"},
                {"id": "settle_count", "type": "IntCounter"},
            ],
            "wrench_outputs": [],
            "scene": {},
            "trace": {},
        }
    )
    ir["introspection"]["quantities"].extend(
        [
            {"id": "pose_ee", "type": "Pose", "reference_frame": "world"},
            {"id": "twist_ee", "type": "VelocityTwist", "reference_frame": "base"},
            {"id": "wrench_ee", "type": "Wrench"},
            {"id": "wrench_force", "type": "Quantity"},
            {"id": "wrench_force_x", "type": "Quantity"},
        ]
    )
    ir_path = tmp_path / "ir.json"
    ir_path.write_text(json.dumps(ir))
    monkeypatch.setattr(codegen, "render_template", lambda *args, **kwargs: None)

    codegen.generate_code(ir_path, tmp_path, "stst")

    schema = json.loads((tmp_path / "schema.json").read_text())
    quantities = {quantity["id"]: quantity for quantity in schema["quantities"]}
    assert quantities["err_x"]["sample_expr"] == "shared.err_x"
    assert quantities["pose_ee.position.x"]["source_id"] == "pose_ee"
    assert quantities["pose_ee.orientation.z"]["component"] == "orientation.z"
    assert quantities["twist_ee.angular.x"]["sample_expr"] == "shared.twist_ee.rot[0]"
    assert quantities["wrench_ee.force.z"]["sample_expr"] == "shared.wrench_ee.force[2]"
    assert "wrench_force" not in quantities
    assert quantities["wrench_force_x"]["sample_expr"] == "shared.wrench_ee.force[0]"
    assert quantities["ready_flag"]["sample_expr"] == "shared.ready_flag ? 1.0 : 0.0"
    assert quantities["ready_flag"]["source_type"] == "Bool"
    assert quantities["settle_count"]["sample_expr"] == "static_cast<double>(shared.settle_count)"
    assert quantities["settle_count"]["source_type"] == "IntCounter"
    assert quantities["ctrl_x_error_integral"]["sample_expr"] == "shared.ctrl_x_error_integral"
    assert quantities["ctrl_x_error_integral"]["role"] == "controller_internal_state"
    assert quantities["ctrl_x_previous_error"]["sample_expr"] == "shared.ctrl_x_previous_error"
    assert quantities["ctrl_x_first_sample"]["sample_expr"] == "shared.ctrl_x_first_sample ? 1.0 : 0.0"
    assert quantities["ctrl_x_first_sample"]["role"] == "controller_internal_state"
    assert quantities["ctrl_x_first_sample"]["source_type"] == "Bool"
    assert schema["pools"]["quantities"] == len(schema["quantities"])

    payload = json.loads((tmp_path / ".stst" / "ir.json").read_text())
    assert {
        "index": quantities["pose_ee.position.x"]["index"],
        "expr": "shared.pose_ee.p[0]",
    } in payload["introspection_artifacts"]["model"]["quantities"]
    controller = next(
        controller
        for state in payload["introspection_artifacts"]["model"]["states"]
        for controller in state["controllers"]
        if controller["error_expr"] == "shared.err_x"
    )
    assert controller["measured_expr"] == "shared.measured_x"
    assert controller["setpoint_expr"] == "shared.setpoint_x"

    # Spatial channels: pose (well-known PoseInFrame) + twist/wrench (custom), one per data
    # object, with the object's reference frame threaded through as frame_id (never faked).
    fl = payload["introspection_artifacts"]["frame_layout"]
    model = payload["introspection_artifacts"]["model"]
    assert (schema["pools"]["poses"], schema["pools"]["twists"], schema["pools"]["wrenches"]) == (1, 1, 1)
    assert schema["spatial"]["poses"][0]["topic"] == "/motion_spec/pose/pose_ee"
    assert schema["spatial"]["poses"][0]["frame_id"] == "world"
    assert schema["spatial"]["twists"][0]["frame_id"] == "base"
    assert schema["spatial"]["wrenches"][0]["frame_id"] == ""  # no reference frame -> empty, not faked
    assert fl["poses"][0]["topic"] == "/motion_spec/pose/pose_ee"
    assert {"index": 0, "expr": "shared.pose_ee"} in model["poses"]
    assert {"index": 0, "expr": "shared.twist_ee"} in model["twists"]
    assert {"index": 0, "expr": "shared.wrench_ee"} in model["wrenches"]
def test_provenance_document_is_jsonld_and_prov_shacl_conformant(tmp_path: Path) -> None:
    pyshacl = __import__("pyshacl")
    seed = build_provenance_document(_sample_ir(), tmp_path)
    path = tmp_path / "provenance.jsonld"
    path.write_text(json.dumps(seed))
    metamodels = Path(__file__).resolve().parents[2] / "metamodels"
    install_resolver(
        IriToFileResolver(
            {"https://secorolab.github.io/metamodels/": str(metamodels)},
            download=False,
        )
    )
    graph = Graph().parse(path, format="json-ld")
    assert (None, None, None) in graph
    # The robot model is recorded as a portable vendor-qualified reference (matching the
    # mj_kdl_wrapper cache layout), never a machine-specific absolute path.
    model_ref = next(n for n in seed["@graph"] if n.get("role") == "robot")["has-agn-model"]
    assert model_ref == "menagerie:kinova_gen3/gen3.xml"
    shape_path = metamodels / "prov.shacl.ttl"
    conforms, _, report = pyshacl.validate(graph, shacl_graph=str(shape_path))
    assert conforms, report

