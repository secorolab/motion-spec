# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
import re
from pathlib import Path
from types import SimpleNamespace

import pytest
from motion_spec.setup import find_stst
from rdf_utils.resolver import IriToFileResolver, install_resolver
from rdflib import Dataset, Graph, URIRef
from rdflib.namespace import PROV, RDF

from motion_spec.classes.base import DataclassJSONEncoder
from motion_spec.classes.geometry import Axis, Frame, Point, Subspace, View, Wrench
from motion_spec.classes.handlers import PIDController
from motion_spec.classes.motion import BlackboardValue, ComponentRef, PoseComponents
from motion_spec.classes.qudt import Quantity, QuantityKind, Unit
from motion_spec.generation import codegen
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
from motion_spec.rdf_parser import communication, constraint_handler, quantities
from motion_spec.rdf_parser.model import Model
from rdf_utils.constraints import ConstraintViolation


def _quantity(id_: str, value: float | None = None) -> Quantity:
    return Quantity(id_, QuantityKind("Length"), Unit("M"), value, False)


def _wrench(id_: str) -> Wrench:
    return Wrench(
        id=id_,
        quantity_kind=[],
        reference_point=Point(f"{id_}_origin"),
        as_seen_by=Frame(id_),
        unit=[],
    )


def _model(**authored: str) -> Model:
    """A `Model` whose graph authors one subject per `id=uri` kwarg."""
    graph = Dataset(default_union=True)
    graph.bind("prov", PROV)
    for uri in authored.values():
        graph.add((URIRef(uri), RDF.type, PROV.Entity))
    return Model(
        graph=graph, app_path=Path("model-app.ld.json"), imported_models=[], imported_provenance=[]
    )


def _sample_ir() -> dict:
    return {
        "configuration": {"control_period_ns": 2_000_000, "backend": "mj_kdl", "platform": {}},
        "resources": {"robots": [], "by_kind": {}, "by_id": {}},
        "composition": {"scene": {}},
        "computation": {"shared_data": [], "closures": {}, "views": {}},
        "coordination": {
            "motions": [
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
            ]
        },
        "communication": {
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
                        },
                    ],
                    "activities": [
                        {
                            "id": "activity:motion_spec_ir_generation",
                            "types": ["prov:Activity", "ms-prov:SpecCompilation"],
                            "used": ["entity:app_manifest"],
                            "wasAssociatedWith": "agent:motion_spec_ir_gen",
                        },
                        {
                            "id": "activity:controller_execution",
                            "types": ["prov:Activity", "bdd:SimulatedExecution"],
                            "used": ["entity:motion_spec_ir"],
                            "wasAssociatedWith": "agent:controller_process",
                            "role": "controller_execution",
                        },
                    ],
                    "agents": [
                        {
                            "id": "agent:runtime:mujoco",
                            "types": ["prov:SoftwareAgent", "exec:Simulation"],
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
            }
        },
    }


def test_closure_output_stays_direct_when_it_is_also_a_view() -> None:
    view = View("end_pose_x", _quantity("end_pose"), _quantity("end_x"), Subspace.Linear, Axis.X)
    closure = {"type": "Addition", "out": "end_x"}

    access = quantities.views_for_access({"end_pose_x": view}, [], [], {"add": closure}, {})
    # The closure writes end_x, so nothing reads it through the view; the view stays addressable
    # by its own id, which is how a constraint naming a pooled relation reaches it.
    assert "end_x" not in access
    assert access == {"end_pose_x": view}


def _component_views() -> dict:
    """One pose's z read off itself, and the same quantity bound into another pose's z."""
    source = _quantity("home_pose")
    component = _quantity("home_pose_position_z")
    return {
        "read": View("read", source, component, Subspace.Linear, Axis.Z),
        "bind": View("bind", _quantity("target_pose"), component, Subspace.Linear, Axis.Z),
    }


def test_component_bound_into_a_pose_is_not_a_reading_of_it() -> None:
    """The binding must not compete with the source pose's own view (both are axis z of a
    pose, so the ambiguity rule would otherwise drop the reading and leave a zero)."""
    components = {
        "target_pose": PoseComponents(
            "quaternion", position_z=ComponentRef(ref="home_pose_position_z")
        )
    }

    index = quantities.views_for_access(_component_views(), [], [], {}, components)

    assert index["home_pose_position_z"].superobject.id == "home_pose"


def test_a_quantity_no_view_agrees_on_is_rejected() -> None:
    """Two readings that disagree and no direct write: nothing can compute it, and rendering
    it as its own shared field would compile to a zero."""
    with pytest.raises(ConstraintViolation, match="home_pose_position_z"):
        quantities.views_for_access(_component_views(), [], [], {}, {})


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
        _sample_ir(), ir_path=tmp_path / "ir.json", output_dir=tmp_path, fsm_ir=_sample_fsm()
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
    # Generation refuses an FSM-less IR, so the fixture carries the FSM it is coordinated by.
    ir["coordination"]["fsm"] = _sample_fsm()
    ir["coordination"]["motions"][0]["until_monitors"][0]["error"] = {
        "id": "pose_ee",
        "type": "Pose",
    }

    closures = {
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
        },
    }
    wrench_ee = _wrench("wrench_ee")
    views = {
        "wrench_force": View(
            "wrench_force", wrench_ee, _quantity("wrench_force"), Subspace.Linear, None
        ),
        "wrench_force_x": View(
            "wrench_force_x", wrench_ee, _quantity("wrench_force_x"), Subspace.Linear, Axis.X
        ),
    }
    shared_data = [
        BlackboardValue(id="err_x", type="Quantity"),
        BlackboardValue(id="pose_ee", type="Pose"),
        BlackboardValue(id="twist_ee", type="VelocityTwist"),
        BlackboardValue(id="wrench_ee", type="Wrench"),
        BlackboardValue(id="wrench_force", type="Quantity"),
        BlackboardValue(id="wrench_force_x", type="Quantity"),
        BlackboardValue(id="measured_x", type="Quantity"),
        BlackboardValue(id="setpoint_x", type="Quantity"),
        BlackboardValue(id="ready_flag", type="Bool"),
        BlackboardValue(id="settle_count", type="IntCounter"),
    ]
    ir["computation"].update({"closures": closures, "views": views})

    introspection = ir["communication"]["introspection"]
    introspection["quantities"].extend(
        [
            {"id": "pose_ee", "type": "Pose", "reference_frame": "world"},
            {"id": "twist_ee", "type": "VelocityTwist", "reference_frame": "base"},
            {"id": "wrench_ee", "type": "Wrench"},
            {"id": "wrench_force", "type": "Quantity"},
            {"id": "wrench_force_x", "type": "Quantity"},
        ]
    )

    # Run the introspection-piece derivations the way `build_introspection` now does (this
    # synthetic ir is assembled by hand, so drive the pieces directly).
    ctrl_x = PIDController(
        id="ctrl_x",
        control_signal=_quantity("out_x"),
        error_signal=_quantity("err_x"),
        proportional_gain=1.0,
        integral_gain=0.0,
        derivative_gain=0.1,
    )
    constraint_handler.annotate_controller_signals([ctrl_x], closures)
    model = _model(ctrl_x="https://example.org/model/ctrl_x")
    rows = introspection["quantities"]
    seen = {
        "shared": {item.id for item in shared_data if item.id},
        "rows": {row["id"] for row in rows if row.get("id")},
    }
    communication._add_controller_state(
        model, closures, shared_data, rows, seen, [SimpleNamespace(controllers=[ctrl_x])]
    )
    communication.add_quantity_samples(introspection, shared_data, views)
    communication.add_spatial_samples(introspection, shared_data)

    # Cross the derivation-stage records into the plain-dict shape the published IR carries.
    ir["computation"]["shared_data"] = json.loads(json.dumps(shared_data, cls=DataclassJSONEncoder))
    ir["coordination"]["motions"][0]["controllers"] = [
        json.loads(json.dumps(ctrl_x, cls=DataclassJSONEncoder))
    ]

    ir_path = tmp_path / "ir.json"
    ir_path.write_text(json.dumps(ir, cls=DataclassJSONEncoder))
    monkeypatch.setattr(codegen, "render_template", lambda *args, **kwargs: None)
    monkeypatch.setattr(codegen, "compile_frame_log_proto", lambda *args, **kwargs: None)

    codegen.generate_code(ir_path, tmp_path, "stst")

    # schema.json is not an artifact any more -- the contract lives in the log header.
    schema = build_schema(
        ir, ir_path=ir_path, output_dir=tmp_path, fsm_ir=ir["coordination"].get("fsm")
    )
    quantities_by_id = {quantity["id"]: quantity for quantity in schema["quantities"]}
    # sample_desc is the backend-agnostic descriptor; the C++ expression is rendered
    # by the sample-expr template (shared_data.stg).
    assert quantities_by_id["err_x"]["sample_desc"] == {"kind": "shared", "id": "err_x"}
    # An object carried by a whole-object spatial slot gets no per-axis scalar rows too --
    # that pair was the same value on the wire twice, in two representations.
    assert not [
        qid for qid in quantities_by_id if qid.startswith(("pose_ee.", "twist_ee.", "wrench_ee."))
    ]
    assert "wrench_force" not in quantities_by_id
    assert quantities_by_id["wrench_force_x"]["sample_desc"] == {
        "kind": "access",
        "ref": "wrench_force_x",
    }
    assert quantities_by_id["ready_flag"]["sample_desc"] == {"kind": "bool", "id": "ready_flag"}
    assert quantities_by_id["ready_flag"]["source_type"] == "Bool"
    assert quantities_by_id["settle_count"]["sample_desc"] == {"kind": "int", "id": "settle_count"}
    assert quantities_by_id["settle_count"]["source_type"] == "IntCounter"
    assert quantities_by_id["ctrl_x_error_integral"]["sample_desc"] == {
        "kind": "shared",
        "id": "ctrl_x_error_integral",
    }
    assert quantities_by_id["ctrl_x_error_integral"]["role"] == "controller_internal_state"
    assert quantities_by_id["ctrl_x_first_sample"]["sample_desc"] == {
        "kind": "bool",
        "id": "ctrl_x_first_sample",
    }
    assert quantities_by_id["ctrl_x_first_sample"]["role"] == "controller_internal_state"
    assert quantities_by_id["ctrl_x_first_sample"]["source_type"] == "Bool"
    assert schema["pools"]["quantities"] == len(schema["quantities"])

    payload = json.loads((tmp_path / ".stst" / "ir.json").read_text())
    assert not [
        sample
        for sample in payload["communication"]["introspection_artifacts"]["model"]["quantities"]
        if sample["desc"].get("kind") in {"pose_pos", "pose_orient"}
    ]
    controller = next(
        controller
        for case in payload["communication"]["introspection_artifacts"]["model"]["motions"]
        for controller in case["controllers"]
        if controller["error_signal"] == "err_x"
    )
    # Every signal reaches the template as an abstract id; the C++ access is rendered there
    # (shared-sig for error/output, access-expr for measured/setpoint).
    assert controller["measured_signal"] == "measured_x"
    assert controller["setpoint_signal"] == "setpoint_x"
    monitors = [
        monitor
        for case in build_introspection_model(schema, ir)["motions"]
        for monitor in case["monitors"]
        if not monitor["has_active"]
    ]
    assert any(monitor["value_signal"] == "err_x" for monitor in monitors)
    assert any(monitor["composite_error"] for monitor in monitors)
    assert all("satisfied_expr" not in monitor for monitor in monitors)

    # Spatial samples: one pose/twist/wrench per data object, serialized into the frame
    # record's pose/twist/wrench fields.
    model_payload = payload["communication"]["introspection_artifacts"]["model"]
    assert (schema["pools"]["poses"], schema["pools"]["twists"], schema["pools"]["wrenches"]) == (
        1,
        1,
        1,
    )
    assert schema["spatial"]["poses"][0]["id"] == "pose_ee"
    assert {"index": 0, "id": "pose_ee"} in model_payload["poses"]
    assert {"index": 0, "id": "twist_ee"} in model_payload["twists"]
    assert {"index": 0, "id": "wrench_ee"} in model_payload["wrenches"]


def test_codegen_refuses_a_model_that_declares_no_fsm(tmp_path: Path) -> None:
    ir_path = tmp_path / "ir.json"
    ir_path.write_text(json.dumps(_sample_ir(), cls=DataclassJSONEncoder))

    with pytest.raises(RuntimeError, match="declares no FSM"):
        codegen.generate_code(ir_path, tmp_path, "stst")


def test_provenance_document_is_jsonld_and_prov_shacl_conformant(tmp_path: Path) -> None:
    metamodels = Path(__file__).resolve().parents[2] / "metamodels"
    if not metamodels.exists():
        pytest.skip("metamodels is not in this checkout")
    pyshacl = __import__("pyshacl")
    seed = build_provenance_document(_sample_ir(), tmp_path)
    path = tmp_path / "provenance.ld.json"
    path.write_text(json.dumps(seed))
    install_resolver(
        IriToFileResolver(
            {"https://secorolab.github.io/metamodels/": str(metamodels)}, download=False
        )
    )
    graph = Graph().parse(path, format="json-ld")
    assert (None, None, None) in graph
    # The robot model is recorded as a portable vendor-qualified reference (matching the
    # mj_kdl_wrapper cache layout), never a machine-specific absolute path.
    robot = next(n for n in seed["@graph"] if n["@id"].endswith("agent/modelled_robot"))
    assert robot["has-agn-model"] == "menagerie:kinova_gen3/gen3.xml"
    shape_path = metamodels / "prov.shacl.ttl"
    conforms, _, report = pyshacl.validate(graph, shacl_graph=str(shape_path))
    assert conforms, report


def test_usage_roles_replace_the_minted_role_predicate(tmp_path: Path) -> None:
    """A role is the part an entity played for one activity, not a label hung on the entity."""
    document = build_provenance_document(_sample_ir(), tmp_path)
    assert not any("role" in node for node in document["@graph"])
    assert "role" not in dict(document["@context"][-1])
    usages = [
        usage
        for node in document["@graph"]
        for usage in node.get("qualifiedUsage", [])
        if node["@id"].endswith("activity/motion_spec_ir_generation")
    ]
    assert usages == [
        {
            "@type": "Usage",
            "entity": "msprov:entity/app_manifest",
            "hadRole": "msprov:role/app_manifest",
        }
    ]
    assert {"@id": "msprov:role/app_manifest", "@type": ["prov:Role"]} in document["@graph"]


def test_compilation_activities_are_typed_but_execution_is_not(tmp_path: Path) -> None:
    types = {
        node["@id"]: node["@type"]
        for node in build_provenance_document(_sample_ir(), tmp_path)["@graph"]
    }
    assert "ms-prov:SpecCompilation" in types["msprov:activity/code_generation"]
    assert "ms-prov:SpecCompilation" in types["msprov:activity/motion_spec_ir_generation"]
    assert types["msprov:activity/controller_execution"] == [
        "prov:Activity",
        "bdd:SimulatedExecution",
    ]
    assert types["msprov:activity/build"] == ["prov:Activity"]


# --- derived-entity IRIs (plan 015) ---------------------------------------------------------


def test_derived_iri_extends_its_parent_path():
    model = _model(ctrl_x="https://example.org/m/handler/ctrl-x")
    minted = model.register_derived(
        "ctrl_x_error_integral",
        "https://example.org/m/handler/ctrl-x",
        "error_integral",
        PROV.wasDerivedFrom,
    )
    assert minted == "https://example.org/m/handler/ctrl-x/error-integral"
    assert model.iri_of("ctrl_x_error_integral") == minted
    # A minted IRI is a proper path extension, so the parent is recoverable from it.
    assert minted.rsplit("/", 1)[0] == model.iri_of("ctrl_x")


def test_authored_iri_is_never_shadowed_by_a_derived_one():
    model = _model(ctrl_x="https://example.org/m/authored/ctrl-x")
    returned = model.register_derived(
        "ctrl_x", "https://example.org/m/other", "ctrl-x", PROV.wasDerivedFrom
    )
    assert returned == "https://example.org/m/authored/ctrl-x"
    assert model.derivation_nodes() == []


def test_registering_one_id_with_two_iris_raises():
    model = _model()
    model.register_derived("err_x", "https://example.org/m/a", "err", PROV.wasDerivedFrom)
    with pytest.raises(ConstraintViolation, match="minted for two derived entities"):
        model.register_derived("err_x", "https://example.org/m/b", "err", PROV.wasDerivedFrom)


def test_repeated_identical_registration_is_a_no_op():
    model = _model()
    for _ in range(2):
        model.register_derived("err_x", "https://example.org/m/a", "err", PROV.wasDerivedFrom)
    assert len(model.uri_rows()) == 1


def test_totality_assertion_lists_every_unresolved_id():
    introspection = {
        "uris": [{"id": "ctrl_x", "uri": "https://example.org/m/ctrl-x"}],
        "controllers": [{"id": "ctrl_x"}, {"id": "ctrl_y"}],
        "quantities": [{"id": "q_missing"}],
        "dataflow": {"member_a": {"producer": {"kind": "closure", "id": "producer_missing"}}},
    }
    with pytest.raises(RuntimeError) as excinfo:
        communication._check_every_id_resolves(introspection)
    message = str(excinfo.value)
    # Every gap in one build, not just the first.
    for missing in ("ctrl_y", "q_missing", "member_a", "producer_missing"):
        assert missing in message
    assert "ctrl_x" not in message.split("\n", 1)[1]


def test_a_row_carrying_its_own_uri_needs_no_table_entry():
    introspection = {
        "uris": [],
        "quantities": [{"id": "q_a", "uri": "https://example.org/m/q-a"}],
        "signals": [
            {"id": "ctrl_x.error_signal", "quantity": "q_a", "uri": "https://example.org/m/q-a"}
        ],
    }
    communication._check_every_id_resolves(introspection)


def test_derivation_document_links_each_node_to_its_parent():
    model = _model(ctrl_x="https://example.org/m/ctrl-x")
    model.register_derived(
        "ctrl_x_lin_x", "https://example.org/m/ctrl-x", "lin_x", PROV.specializationOf
    )
    model.register_derived(
        "ctrl_x_error_integral",
        "https://example.org/m/ctrl-x",
        "error_integral",
        PROV.wasDerivedFrom,
    )
    document = build_derivation_document(
        {"communication": {"introspection": {"derivations": model.derivation_nodes()}}}
    )
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


# A plain data member: a type, then the identifier, then an initialiser, array bound or `;`.
_MEMBER = re.compile(r"^\s*[\w:]+(?:\s*[*&])?\s+(\w+)\s*(?:=|;|\[)")


def test_generated_blackboard_holds_only_contracted_members(tmp_path: Path) -> None:
    """Every `struct shared_data` member is an `ir["computation"]["shared_data"]` id, so it
    carries a producer/consumer contract; anything else belongs in a struct that is not the
    blackboard."""
    if find_stst() is None:
        pytest.skip("no stst; run `motion-spec setup`")
    shared_data = [
        {"id": "clock_time_s", "type": "Quantity"},
        {"id": "err_x", "type": "Quantity"},
        {"id": "pose_ee", "type": "Pose"},
        {"id": "ready_flag", "type": "Bool"},
        {"id": "settle_count", "type": "IntCounter"},
    ]
    payload = tmp_path / "ir.json"
    payload.write_text(
        json.dumps(
            {
                "configuration": {"backend": "mj_kdl"},
                "resources": {"by_kind": {}},
                "computation": {"shared_data": shared_data},
                "coordination": {},
                "communication": {},
            }
        )
    )
    header = tmp_path / "shared_state.hpp"
    codegen.render_template("stst", "shared_state_header", payload, header)

    body = header.read_text().split("struct shared_data {", 1)[1].split("\n};", 1)[0]
    members = []
    for line in body.splitlines():
        if not line.strip():
            continue
        match = _MEMBER.match(line)
        assert match, f"not a plain data member of the blackboard: {line!r}"
        members.append(match.group(1))
    assert members
    assert set(members) <= {item["id"] for item in shared_data}
