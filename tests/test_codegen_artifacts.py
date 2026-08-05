# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
from pathlib import Path

import pytest

from rdflib import Graph
from rdf_utils.resolver import IriToFileResolver, install_resolver

from motion_spec.classes.closures import closure_output_ids
from motion_spec.generation import codegen
from motion_spec.rdf_parser.ir import (
    DerivedIriRegistry,
    _annotate_controller_signals,
    _assert_every_id_resolves,
    add_controller_internal_state_logging,
    add_quantity_samples,
    add_spatial_samples,
)
from motion_spec.generation.artifacts import (
    PROTO_FIELD_BASES,
    build_frame_layout,
    build_frame_log_proto_fields,
    build_introspection_model,
    build_schema,
    fields_with_offsets,
)
from motion_spec.introspection.provenance import (
    build_derivation_document,
    build_provenance_document,
)


def _stub_registry(introspection: dict) -> DerivedIriRegistry:
    """Registry seeded with an IRI for every controller the sample IR declares."""
    from rdflib import URIRef

    return DerivedIriRegistry(
        [
            (controller["id"], URIRef(f"https://example.org/model/{controller['id']}"))
            for controller in introspection.get("controllers", [])
        ]
    )


def _sample_ir() -> dict:
    return {
        "control_period_ns": 2_000_000,
        "unique_motions": [
            {
                "id": "move",
                "index": 0,
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
                "index": 1,
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
                        "id": "execution-context",
                        "uri": "https://secorolab.github.io/metamodels/execution-context#",
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
                            "exec:Simulation",
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


def test_closure_output_stays_direct_when_it_is_also_a_view() -> None:
    view = {
        "subobject": {"id": "end_x"},
        "superobject": {"id": "end_pose"},
        "subspace": "position",
        "axis": "X",
    }
    direct_ids = closure_output_ids({"type": "Addition", "out": "end_x"})

    assert codegen._views_for_access({"end_pose_x": view}, direct_ids) == {}


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
    assert schema["by_motion"]["move"]["controllers"][0]["id"] == "ctrl_x"
    assert schema["by_motion"]["move"]["monitors"][0]["id"] == "done_mon"
    assert schema["by_motion"]["move"]["monitors"][1]["id"] == "ready_mon"
    assert layout["fields"] == fields
    assert layout["frame_size_bytes"] == size
    assert layout["pools"] == schema["pools"]
    # The wire field mapping is folded into the schema before hashing.
    assert schema["protobuf"]["runtime_frame"] == "RuntimeFrame"
    assert schema["protobuf"]["fields"]["constraints"][0] == {
        "index": 0,
        "id": "constraint_0",
        "name": "constraint_0",
        "number": 1000,
    }


def test_frame_log_proto_field_naming_and_numbering() -> None:
    schema = {
        "pools": {"constraints": 2, "monitors": 1, "quantities": 4, "triggers": 3},
        "quantities": [
            {"index": 0, "id": "direction_ctrl_cg_support_z.x"},  # dots -> underscores
            {"index": 1, "id": "home_pose"},
            {"index": 2, "id": "home.pose"},  # sanitizes to a duplicate -> suffixed
            {"index": 3, "id": "3dof"},  # leading digit -> field_ prefix
        ],
        "spatial": {
            "poses": [{"index": 0, "id": "pose_ee_base"}],
            "twists": [],
            "wrenches": [{"index": 0, "id": "wrench force"}],
        },
    }
    proto = build_frame_log_proto_fields(schema)
    fields = proto["fields"]

    quantity_names = {q["id"]: q["name"] for q in fields["quantities"]}
    assert quantity_names["direction_ctrl_cg_support_z.x"] == "direction_ctrl_cg_support_z_x"
    assert quantity_names["home_pose"] == "home_pose"
    assert quantity_names["home.pose"] == "home_pose_2"  # deterministic de-dup suffix
    assert quantity_names["3dof"] == "field_3dof"  # numeric-leading gets field_ prefix

    # Constraint/monitor/trigger slots are reused per state -> slot-stable names, never model ids.
    assert [c["name"] for c in fields["constraints"]] == ["constraint_0", "constraint_1"]
    assert [m["name"] for m in fields["monitors"]] == ["monitor_0"]
    assert [t["name"] for t in fields["triggers"]] == ["trigger_0", "trigger_1", "trigger_2"]

    # Every field number sits in its category's range, and all numbers/names are unique.
    for category, base in PROTO_FIELD_BASES.items():
        for entry in fields.get(category, []):
            assert entry["number"] == base + entry["index"]
    all_numbers = [e["number"] for cat in fields.values() for e in cat]
    all_names = [e["name"] for cat in fields.values() for e in cat]
    assert len(all_numbers) == len(set(all_numbers))
    assert len(all_names) == len(set(all_names))


def test_frame_log_proto_fields_advance_past_large_categories() -> None:
    schema = {
        "pools": {"constraints": 0, "monitors": 0, "quantities": 1001, "triggers": 1},
        "quantities": [{"index": index, "id": f"q_{index}"} for index in range(1001)],
        "spatial": {"poses": [], "twists": [], "wrenches": []},
    }

    fields = build_frame_log_proto_fields(schema)["fields"]

    assert fields["quantities"][-1]["number"] == 4000
    assert fields["triggers"][0]["number"] == 4001


def test_codegen_samples_logged_quantity_components(tmp_path: Path, monkeypatch) -> None:
    ir = _sample_ir()
    ir["unique_motions"][0]["until_monitors"][0]["error"] = {
        "id": "pose_ee",
        "type": "Pose",
    }
    ir.update(
        {
            "backend": "mj_kdl",
            "has_serial_chain": False,
            "has_mobile_base": False,
            "serial_chain_solvers": [],
            "platform_velocity_solvers": [],
            "platform_force_solvers": [],
            "cstr_hdl": [],
            "motions": ir["unique_motions"],
            "data": [],
            "pose_components": {},
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
    # Run the introspection-piece derivations the way _build_introspection now does
    # (this synthetic ir is assembled by hand, so drive the pieces directly).
    for motion in ir["motions"]:
        _annotate_controller_signals(motion.get("controllers", []), ir["closures"])
    _annotate_controller_signals(ir["introspection"]["controllers"], ir["closures"])
    add_controller_internal_state_logging(
        ir["closures"],
        ir["shared_data"],
        ir["introspection"],
        ir["motions"],
        _stub_registry(ir["introspection"]),
    )
    add_quantity_samples(ir["introspection"], ir["shared_data"], ir["views"])
    add_spatial_samples(ir["introspection"], ir["shared_data"])
    ir_path = tmp_path / "ir.json"
    ir_path.write_text(json.dumps(ir))
    monkeypatch.setattr(codegen, "render_template", lambda *args, **kwargs: None)
    monkeypatch.setattr(codegen, "compile_frame_log_proto", lambda *args, **kwargs: None)

    codegen.generate_code(ir_path, tmp_path, "stst")

    schema = json.loads((tmp_path / "schema.json").read_text())
    quantities = {quantity["id"]: quantity for quantity in schema["quantities"]}
    # sample_desc is the backend-agnostic descriptor; the C++ expression is rendered
    # by the sample-expr template (shared_data.stg).
    assert quantities["err_x"]["sample_desc"] == {"kind": "shared", "id": "err_x"}
    # An object carried by a whole-object spatial slot gets no per-axis scalar rows too --
    # that pair was the same value on the wire twice, in two representations.
    assert not [qid for qid in quantities if qid.startswith(("pose_ee.", "twist_ee.", "wrench_ee."))]
    assert "wrench_force" not in quantities
    assert quantities["wrench_force_x"]["sample_desc"] == {"kind": "access", "ref": "wrench_force_x"}
    assert quantities["ready_flag"]["sample_desc"] == {"kind": "bool", "id": "ready_flag"}
    assert quantities["ready_flag"]["source_type"] == "Bool"
    assert quantities["settle_count"]["sample_desc"] == {"kind": "int", "id": "settle_count"}
    assert quantities["settle_count"]["source_type"] == "IntCounter"
    assert quantities["ctrl_x_error_integral"]["sample_desc"] == {
        "kind": "shared", "id": "ctrl_x_error_integral"}
    assert quantities["ctrl_x_error_integral"]["role"] == "controller_internal_state"
    assert quantities["ctrl_x_first_sample"]["sample_desc"] == {"kind": "bool", "id": "ctrl_x_first_sample"}
    assert quantities["ctrl_x_first_sample"]["role"] == "controller_internal_state"
    assert quantities["ctrl_x_first_sample"]["source_type"] == "Bool"
    assert schema["pools"]["quantities"] == len(schema["quantities"])

    payload = json.loads((tmp_path / ".stst" / "ir.json").read_text())
    assert not [
        sample
        for sample in payload["introspection_artifacts"]["model"]["quantities"]
        if sample["desc"].get("kind") in {"pose_pos", "pose_orient"}
    ]
    controller = next(
        controller
        for case in payload["introspection_artifacts"]["model"]["motions"]
        for controller in case["controllers"]
        if controller["error_expr"] == "shared.err_x"
    )
    # measured/setpoint carry abstract signal ids now; the template renders them via
    # access-expr (here plain shared signals -> shared.measured_x / shared.setpoint_x).
    assert controller["measured_signal"] == "measured_x"
    assert controller["setpoint_signal"] == "setpoint_x"
    monitors = [
        monitor
        for case in build_introspection_model(schema, ir)["motions"]
        for monitor in case["monitors"]
        if not monitor["has_active"]
    ]
    assert any(monitor["value_expr"] == "shared.err_x" for monitor in monitors)
    assert any(monitor["composite_error"] for monitor in monitors)
    assert all("satisfied_expr" not in monitor for monitor in monitors)

    # Spatial samples: one pose/twist/wrench per data object, serialized into the frame
    # record's pose/twist/wrench fields.
    model = payload["introspection_artifacts"]["model"]
    assert (schema["pools"]["poses"], schema["pools"]["twists"], schema["pools"]["wrenches"]) == (1, 1, 1)
    assert schema["spatial"]["poses"][0]["id"] == "pose_ee"
    assert {"index": 0, "id": "pose_ee"} in model["poses"]
    assert {"index": 0, "id": "twist_ee"} in model["twists"]
    assert {"index": 0, "id": "wrench_ee"} in model["wrenches"]


def test_provenance_document_is_jsonld_and_prov_shacl_conformant(tmp_path: Path) -> None:
    pyshacl = __import__("pyshacl")
    seed = build_provenance_document(_sample_ir(), tmp_path)
    path = tmp_path / "provenance.ld.json"
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


# --- derived-entity IRIs (plan 015) ---------------------------------------------------------
def _registry(**authored):
    from rdflib import URIRef

    return DerivedIriRegistry([(id_, URIRef(uri)) for id_, uri in authored.items()])


def test_derived_iri_extends_its_parent_path():
    iris = _registry(ctrl_x="https://example.org/m/handler/ctrl-x")
    minted = iris.register(
        "ctrl_x_error_integral",
        "https://example.org/m/handler/ctrl-x",
        "error_integral",
        DerivedIriRegistry.DERIVATION,
    )
    assert minted == "https://example.org/m/handler/ctrl-x/error-integral"
    assert iris.iri_of("ctrl_x_error_integral") == minted
    # A minted IRI is a proper path extension, so the parent is recoverable from it.
    assert minted.rsplit("/", 1)[0] == iris.iri_of("ctrl_x")


def test_authored_iri_is_never_shadowed_by_a_derived_one():
    iris = _registry(ctrl_x="https://example.org/m/authored/ctrl-x")
    returned = iris.register(
        "ctrl_x", "https://example.org/m/other", "ctrl-x", DerivedIriRegistry.DERIVATION
    )
    assert returned == "https://example.org/m/authored/ctrl-x"
    assert iris.rows() == []


def test_registering_one_id_with_two_iris_raises():
    iris = _registry()
    iris.register("err_x", "https://example.org/m/a", "err", DerivedIriRegistry.DERIVATION)
    with pytest.raises(RuntimeError, match="derived IRI collision"):
        iris.register("err_x", "https://example.org/m/b", "err", DerivedIriRegistry.DERIVATION)


def test_repeated_identical_registration_is_a_no_op():
    iris = _registry()
    for _ in range(2):
        iris.register("err_x", "https://example.org/m/a", "err", DerivedIriRegistry.DERIVATION)
    assert len(iris.rows()) == 1


def test_totality_assertion_lists_every_unresolved_id():
    introspection = {
        "uris": [{"id": "ctrl_x", "uri": "https://example.org/m/ctrl-x"}],
        "controllers": [{"id": "ctrl_x"}, {"id": "ctrl_y"}],
        "quantities": [{"id": "q_missing"}],
        "dataflow": {"member_a": {"producer": {"kind": "closure", "id": "producer_missing"}}},
    }
    with pytest.raises(RuntimeError) as excinfo:
        _assert_every_id_resolves(introspection)
    message = str(excinfo.value)
    # Every gap in one build, not just the first.
    for missing in ("ctrl_y", "q_missing", "member_a", "producer_missing"):
        assert missing in message
    assert "ctrl_x" not in message.split("\n", 1)[1]


def test_a_row_carrying_its_own_uri_needs_no_table_entry():
    introspection = {
        "uris": [],
        "quantities": [{"id": "q_a", "uri": "https://example.org/m/q-a"}],
        "signals": [{"id": "ctrl_x.error_signal", "quantity": "q_a", "uri": "https://example.org/m/q-a"}],
    }
    _assert_every_id_resolves(introspection)


def test_derivation_document_links_each_node_to_its_parent():
    iris = _registry(ctrl_x="https://example.org/m/ctrl-x")
    iris.register(
        "ctrl_x_lin_x", "https://example.org/m/ctrl-x", "lin_x", DerivedIriRegistry.SPECIALIZATION
    )
    iris.register(
        "ctrl_x_error_integral",
        "https://example.org/m/ctrl-x",
        "error_integral",
        DerivedIriRegistry.DERIVATION,
    )
    document = build_derivation_document({"introspection": {"derivations": iris.nodes()}})
    graph = Graph().parse(data=json.dumps(document), format="json-ld")
    prov = "http://www.w3.org/ns/prov#"
    links = {
        (str(s), str(p)): str(o)
        for s, p, o in graph
        if str(p) in (f"{prov}specializationOf", f"{prov}wasDerivedFrom")
    }
    assert links[("https://example.org/m/ctrl-x/lin-x", f"{prov}specializationOf")] == (
        "https://example.org/m/ctrl-x"
    )
    assert links[("https://example.org/m/ctrl-x/error-integral", f"{prov}wasDerivedFrom")] == (
        "https://example.org/m/ctrl-x"
    )
