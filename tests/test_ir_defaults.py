# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

from __future__ import annotations

import json
from pathlib import Path

import pytest
from motion_spec_dsl.rdf_parser.vocab import (
    AGN,
    ALGO_EXT,
    CSTR,
    CSTR_HDL,
    EXEC,
    GEOM_COORD,
    KC,
    QUDT_QKIND,
    QUDT_SCHEMA,
    SLV,
)
from rdf_utils.models.vocab import URI_KC_TYPE_SERIAL
from rdf_utils.namespace import NS_MM_GEOM, NS_MM_KC_EXT
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import RDF, XSD

from motion_spec.classes.entities import PIDController
from motion_spec.rdf_parser.ir import (
    LINEAR_AXES,
    SOLVER_SEMANTICS_BY_ALGORITHM,
    ControllerDerivation,
    GuardedMotionBlock,
    Parser,
    SceneRobot,
    SceneSpec,
    SolverDerivationContext,
    _annotate_rne_gravity,
    DerivedIriRegistry,
    _build_introspection,
    _derived_controllers,
    _derived_motion_drivers,
    _fixed_attachments,
    _robot_setups_from_graph,
    ops_generic,
)


def test_fixed_attachments_root_a_branched_multi_robot_scene_at_world() -> None:
    graph = Graph()
    world = URIRef("https://example.test/world")
    table = URIRef("https://example.test/table")
    table_top = URIRef(f"{table}/top")
    trees = {
        URIRef("https://example.test/arm1"),
        URIRef("https://example.test/arm2"),
    }

    def fixed(name: str, frame_a: URIRef, frame_b: URIRef) -> None:
        joint = URIRef(f"https://example.test/{name}")
        graph.add((joint, RDF.type, KC.Joint))
        graph.add((joint, KC["between-attachments"], frame_a))
        graph.add((joint, KC["between-attachments"], frame_b))

    fixed("world-table", URIRef(f"{world}/origin"), table_top)
    for tree in trees:
        root = URIRef(f"{tree}/base")
        root_frame = URIRef(f"{root}/origin")
        tip = URIRef(f"{tree}/tip")
        tip_frame = URIRef(f"{tip}/origin")
        graph.add((tree, NS_MM_KC_EXT["tip"], tip_frame))
        fixed(f"{tree.rsplit('/', 1)[-1]}-table", table_top, root_frame)
        joint = URIRef(f"{tree}/moving")
        graph.add((joint, RDF.type, KC.Joint))
        graph.add((joint, RDF.type, KC.RevoluteJoint))
        graph.add((joint, KC["between-attachments"], root_frame))
        graph.add((joint, KC["between-attachments"], tip_frame))

    attachments, root = _fixed_attachments(graph, trees)

    assert root == world
    assert attachments[table][:2] == ("World", "")
    assert {attachments[URIRef(f"{tree}/base")][:2] for tree in trees} == {
        ("Site", "top")
    }
def _quantity(graph: Graph, name: str) -> URIRef:
    node = URIRef(f"https://example.test/{name}")
    graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    graph.add((node, RDF.type, QUDT_QKIND.Force))
    graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Force))
    graph.add((node, QUDT_SCHEMA.unit, URIRef("https://qudt.org/vocab/unit/N")))
    return node


def test_parser_scopes_repeated_nested_reference_ids() -> None:
    graph = Graph()
    graph.bind("example", "https://example.test/")
    first = URIRef("https://example.test/motion/spec/path1/reference")
    second = URIRef("https://example.test/motion/spec/path2/reference")
    graph.add((first, RDF.type, RDF.Property))
    graph.add((second, RDF.type, RDF.Property))

    parser = Parser(graph)

    assert parser.id(first) == "motion_path1_reference"
    assert parser.id(second) == "motion_path2_reference"


def _pid_graph(*, kp: float | None = 1.0) -> tuple[Graph, URIRef]:
    graph = Graph()
    controller = URIRef("https://example.test/controller")
    graph.add((controller, RDF.type, CSTR_HDL.Controller))
    graph.add((controller, RDF.type, CSTR_HDL.ProportionalIntegralDerivative))
    graph.add((controller, CSTR_HDL["error-signal"], _quantity(graph, "error")))
    graph.add((controller, CSTR_HDL["control-signal"], _quantity(graph, "control")))
    if kp is not None:
        graph.add((controller, CSTR_HDL["proportional-gain"], Literal(kp, datatype=XSD.double)))
    return graph, controller


def test_agent_model_may_bind_the_assembled_kinematic_tree() -> None:
    graph = Graph()
    base = URIRef("https://example.test/arm/base")
    root = URIRef(f"{base}/root")
    tool = URIRef("https://example.test/gripper/tool")
    tcp = URIRef(f"{tool}/tcp")
    tree = URIRef("https://example.test/assembled")
    joint = URIRef(f"{tree}/fixed")
    agent = URIRef("https://example.test/robot")
    modelled = URIRef("https://example.test/modelled-robot")
    model = URIRef("https://example.test/robot-model")

    graph.add((modelled, RDF.type, AGN.ModelledAgent))
    graph.add((modelled, AGN["of-agent"], agent))
    graph.add((modelled, AGN["has-agent-model"], model))
    graph.add((model, EXEC["has-kinematic-tree"], tree))
    graph.add((model, EXEC.path, Literal("kinova_gen3.xml")))
    graph.add((tree, RDF.type, NS_MM_GEOM["KinematicTree"]))
    graph.add((tree, RDF.type, URI_KC_TYPE_SERIAL))
    graph.add((tree, NS_MM_KC_EXT["root"], root))
    graph.add((tree, NS_MM_KC_EXT["tip"], tcp))
    graph.add((joint, RDF.type, KC.Joint))
    graph.add((joint, KC["between-attachments"], root))
    graph.add((joint, KC["between-attachments"], tcp))

    setups, _ordered = _robot_setups_from_graph(graph)

    assert setups[agent][1:7] == ("base", "tool", "tool", "KinovaGen3", "", "")


def test_introspection_contract_carries_control_and_provenance() -> None:
    graph, controller_node = _pid_graph(kp=2.0)
    graph.add((controller_node, CSTR_HDL["integral-gain"], Literal(0.1, datatype=XSD.double)))
    graph.add((controller_node, CSTR_HDL["derivative-gain"], Literal(0.3, datatype=XSD.double)))
    monitor_node = URIRef("https://example.test/monitor")
    event_node = URIRef("https://example.test/events/complete")
    graph.add((monitor_node, RDF.type, CSTR_HDL.Monitor))
    graph.add((monitor_node, RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
    graph.add((monitor_node, CSTR_HDL.event, event_node))

    parser = Parser(graph)
    controller = PIDController(
        id=parser.id(controller_node),
        control_signal=parser.quantity(graph.value(controller_node, CSTR_HDL["control-signal"])),
        error_signal=parser.quantity(graph.value(controller_node, CSTR_HDL["error-signal"])),
        measured_derivative=None,
        proportional_gain=2.0,
        integral_gain=0.1,
        derivative_gain=0.3,
        decay_rate=None,
        output_saturation=None,
        integral_saturation=None,
        type=parser.id(CSTR_HDL.ProportionalIntegralDerivative),
    )
    monitor = parser.monitor_entry(monitor_node)
    motion = GuardedMotionBlock(
        id="move",
        name="move",
        description=None,
        handler="move_handler",
        when_evaluators=[],
        while_evaluators=[],
        until_evaluators=[],
        controllers=[controller],
        when_monitors=[],
        while_monitors=[],
        until_monitors=[monitor],
        when_schedule=[],
        while_schedule=[],
        until_schedule=[],
    )

    introspection = _build_introspection(
        app_model_path=Path("/tmp/app.json"),
        imported_models=["https://example.test/imported.json"],
        imported_provenance=["/tmp/generated/provenance/dsl.ld.json"],
        iris=DerivedIriRegistry([]),
        # A motion is a subject in every authored model; this hand-built graph has only the
        # controller and monitor, so its IRI is supplied here.
        id_nodes=[(parser.id(node), node) for node in graph.subjects()]
        + [("move", URIRef("https://example.test/move"))],
        node_by_id={parser.id(node): node for node in graph.subjects()},
        motions=[motion],
        data_structures=[controller.error_signal, controller.control_signal],
        control_period_ns=2_000_000,
        backend="mj_kdl",
        scene=SceneSpec(robots=[SceneRobot(id="robot", path="robot.xml")]),
        closures={},
        views={},
        shared_data=[],
        serial_chain_solvers=[],
        platform={"uri": None, "name": None, "simulated": True, "backend": "mj_kdl"},
    )

    assert introspection["contract_version"] == 1
    assert introspection["controllers"][0]["proportional_gain"] == 2.0
    assert introspection["controllers"][0]["output_signal"] == "control"
    assert introspection["monitors"][0]["trigger"] == "edge"
    assert introspection["monitors"][0]["event_uri"] == "https://example.test/events/complete"
    assert {"id": "control", "uri": "https://example.test/control"} in introspection["uris"]
    runtime_activity = next(
        activity
        for activity in introspection["provenance"]["activities"]
        if activity["id"] == "activity:controller_execution"
    )
    assert runtime_activity["wasAssociatedWith"] == "agent:controller_process"


def test_velocity_profile_operator_closure_exposes_codegen_fields() -> None:
    graph = Graph()
    op = URIRef("https://example.test/profile-op")
    graph.add((op, RDF.type, ALGO_EXT.VelocityProfile))
    for pred, name in (
        (ALGO_EXT["target"], "goal"),
        (ALGO_EXT["in"], "measured-velocity"),
        (ALGO_EXT["maximum-velocity"], "max-velocity"),
        (ALGO_EXT["maximum-acceleration"], "max-acceleration"),
        (ALGO_EXT["maximum-jerk"], "max-jerk"),
        (ALGO_EXT["out"], "reference"),
    ):
        graph.add((op, pred, URIRef(f"https://example.test/{name}")))
    constraint = URIRef("https://example.test/constraint")
    controller = URIRef("https://example.test/controller")
    graph.add((constraint, CSTR["reference-value"], URIRef("https://example.test/reference")))
    # The value the profile starts from is the constraint's own quantity.
    graph.add((constraint, CSTR.quantity, URIRef("https://example.test/measured")))
    graph.add((controller, CSTR_HDL.constraint, constraint))
    graph.add((op, ALGO_EXT["shape"], ALGO_EXT["s-curve"]))

    closure = Parser(graph).closures(ops_generic)["profile_op"]

    assert closure["type"] == "VelocityProfile"
    assert closure["goal"] == "goal"
    assert closure["measured"] == "measured"
    assert closure["in"] == "measured_velocity"
    assert closure["maximum_velocity"] == "max_velocity"
    assert closure["maximum_acceleration"] == "max_acceleration"
    assert closure["maximum_jerk"] == "max_jerk"
    assert closure["out"] == "reference"
    assert closure["controller"] == "controller"
    assert str(closure["shape"]) == "s_curve"


def test_velocity_profile_without_controller_fails_clearly() -> None:
    graph = Graph()
    op = URIRef("https://example.test/profile-op")
    graph.add((op, RDF.type, ALGO_EXT.VelocityProfile))
    graph.add((op, ALGO_EXT["out"], URIRef("https://example.test/reference")))

    with pytest.raises(ValueError, match="not bound to a constraint"):
        Parser(graph).closures(ops_generic)


def test_solver_ir_carries_rne_algorithm_and_gravity() -> None:
    graph = Graph()
    solver = URIRef("https://example.test/solver")
    gravity = URIRef("https://example.test/gravity")
    graph.add((solver, RDF.type, SLV.SolverWithInputAndOutput))
    graph.add((solver, SLV.solver, SLV.RecursiveNewtonEulerAlgorithm))
    graph.add((solver, SLV.gravity, gravity))
    graph.add((gravity, GEOM_COORD["x"], Literal(0.0, datatype=XSD.double)))
    graph.add((gravity, GEOM_COORD["y"], Literal(0.0, datatype=XSD.double)))
    graph.add((gravity, GEOM_COORD["z"], Literal(9.81, datatype=XSD.double)))

    entry = Parser(graph).solver_with_input_and_output(solver)

    assert entry.algorithm == "RNE"
    # The authored value is the ACHD root acceleration, carried through unchanged.
    assert entry.root_acc == [0.0, 0.0, 9.81]
    assert entry.gravity is None

    # Only the MuJoCo backend needs the opposite-sign gravity for its RNE bridge.
    _annotate_rne_gravity([entry], [], "robif2b")
    assert entry.gravity is None
    _annotate_rne_gravity([entry], [], "mj_kdl")
    assert entry.gravity == [0.0, 0.0, -9.81]


def test_rne_uses_acceleration_while_achd_uses_acceleration_energy() -> None:
    def derive(algorithm: URIRef):
        graph = Graph()
        solver = URIRef(f"https://example.test/{algorithm.rsplit('/', 1)[-1]}")
        driver = URIRef(f"{solver}/driver")
        handler = URIRef(f"{solver}/handler")
        motion = URIRef(f"{solver}/motion")
        controller = URIRef(f"{solver}/controller")
        constraint = URIRef(f"{solver}/constraint")
        quantity = URIRef(f"{solver}/position/x")
        graph.add((solver, SLV["solver"], algorithm))
        graph.add((solver, SLV["motion-drivers"], driver))
        graph.add((controller, RDF.type, CSTR_HDL.ProportionalIntegralDerivative))
        for predicate, value in (
            (CSTR_HDL["proportional-gain"], 1.0),
            (CSTR_HDL["integral-gain"], 0.0),
            (CSTR_HDL["derivative-gain"], 0.0),
        ):
            graph.add((controller, predicate, Literal(value, datatype=XSD.double)))
        plan = ControllerDerivation(
            handler, motion, controller, solver, constraint, quantity, None, LINEAR_AXES[:1]
        )
        context = SolverDerivationContext(
            {handler: (plan,)},
            {solver: (plan,)},
            {solver: SOLVER_SEMANTICS_BY_ALGORITHM[algorithm]},
            frozenset(),
            DerivedIriRegistry([]),
        )
        parser = Parser(graph)
        return (
            _derived_controllers(graph, parser, context, plan)[0].control_signal,
            _derived_motion_drivers(graph, parser, context, solver)[0],
        )

    rne_signal, rne_drivers = derive(SLV.RecursiveNewtonEulerAlgorithm)
    achd_signal, achd_drivers = derive(
        SLV.AccelerationConstrainedHybridDynamicsAlgorithm
    )

    assert (rne_signal.quantity_kind.id, rne_signal.unit.id) == (
        "LinearAcceleration",
        "M_PER_SEC2",
    )
    assert rne_drivers.acceleration_constraint == []
    assert rne_drivers.cartesian_acceleration[0].acceleration == rne_signal
    assert (achd_signal.quantity_kind.id, achd_signal.unit.id) == (
        "AccelerationEnergy",
        "N_M2_PER_SEC2",
    )
    assert achd_drivers.cartesian_acceleration == []
    assert achd_drivers.acceleration_constraint[0].acceleration_energy == achd_signal


def test_edge_monitor_carries_full_event_uri_and_enum_token() -> None:
    graph = Graph()
    monitor = URIRef("https://example.test/mon")
    event_uri = "http://example.org/coord/E_OBJ_REACHED"
    event = URIRef(event_uri)
    graph.add((monitor, RDF.type, CSTR_HDL.Monitor))
    graph.add((monitor, RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
    graph.add((monitor, CSTR_HDL.event, event))

    entry = Parser(graph).monitor_entry(monitor)

    assert entry.event_uri == event_uri
    # event_name is the coord-dsl FSM enum token (local name, upper-cased, '-' -> '_').
    assert entry.event_name == "E_OBJ_REACHED"


# --- real-world execution (plan 019) ---------------------------------------------------------
def test_real_world_execution_rejects_scene_objects() -> None:
    """A scene object's pose comes from the simulator; on hardware nothing measures it."""
    from rdf_utils.constraints import ConstraintViolation

    from motion_spec.rdf_parser.ir import _reject_scene_objects_on_hardware
    from motion_spec_dsl.rdf_parser.vocab import ENV

    graph = Graph()
    context = URIRef("https://example.test/real-exec")
    cube = URIRef("https://example.test/modelled-cube")
    graph.add((cube, RDF.type, ENV.ModelledObject))
    with pytest.raises(ConstraintViolation, match="cannot use scene objects"):
        _reject_scene_objects_on_hardware(graph, context)


def test_simulation_keeps_its_scene_objects() -> None:
    from motion_spec.rdf_parser.ir import _reject_scene_objects_on_hardware

    # No real-world context -> nothing to reject.
    _reject_scene_objects_on_hardware(Graph(), None)


def test_an_authored_sensor_rate_reaches_the_ir() -> None:
    """scene-dsl emits sens:update-rate as a QUDT quantity; motion-spec used to drop it."""
    from motion_spec.rdf_parser.ir import _hertz
    from motion_spec_dsl.rdf_parser.vocab import QUDT_SCHEMA
    from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT

    graph = Graph()
    rate = URIRef("https://example.test/wrist_ft/update-rate")
    graph.add((rate, QUDT_SCHEMA.value, Literal(1000.0, datatype=XSD.double)))
    graph.add((rate, QUDT_SCHEMA.unit, QUDT_UNIT["HZ"]))
    assert _hertz(graph, rate) == 1000.0
    assert _hertz(graph, None) is None


_ARM_SECTION = (
    "[agents.arm1]\nip='10.0.0.1'\nuser='u'\npassword='p'\nport=1\n"
    "port_real_time=2\nsession_timeout_ms=3\nconnection_timeout_ms=4\n"
)
_FT_SECTION = "[arm1.wrist_ft]\nport='ttyUSB0'\nbaudrate=19200\nslave_address=9\n"
_GRIPPER_SECTION = "[agents.gripper1]\nport='ttyUSB1'\nbaudrate=115200\nslave_address=9\n"


def _real_source(tmp_path, *devices: dict) -> Path:
    """A generation whose IR binds `devices` on one real-world chain."""
    source = tmp_path / "generated"
    (source / "model").mkdir(parents=True, exist_ok=True)
    (source / "model" / "ir.json").write_text(
        json.dumps(
            {
                "platform": {"simulated": False, "config": "robot.toml"},
                "serial_chain_solvers": [
                    {"id": "arm_solver", "config_key": "agents.arm1", "devices": list(devices)}
                ],
            }
        )
    )
    return source


def test_robot_config_must_cover_every_bound_device(tmp_path) -> None:
    from motion_spec.introspection.runner import RunnerError, _validate_robot_config

    source = _real_source(
        tmp_path,
        {"kind": "KinovaGen3", "config_key": "agents.arm1", "drives": ""},
        {"kind": "RobotiqFT300s", "config_key": "arm1.wrist_ft", "drives": "wrist_ft"},
    )
    config = tmp_path / "robot.toml"

    with pytest.raises(RunnerError, match="robot config not found"):
        _validate_robot_config(source, tmp_path)

    config.write_text("[agents.arm1]\nip = '10.0.0.1'\n")
    with pytest.raises(RunnerError, match="is missing user"):
        _validate_robot_config(source, tmp_path)

    config.write_text(_ARM_SECTION)
    with pytest.raises(RunnerError, match=r"no \[arm1.wrist_ft\] section for the bound Robotiq"):
        _validate_robot_config(source, tmp_path)

    # The FT sensor is a serial device: the arm's network keys say nothing about it.
    config.write_text(_ARM_SECTION + "[arm1.wrist_ft]\nport='ttyUSB0'\n")
    with pytest.raises(RunnerError, match=r"\[arm1.wrist_ft\] is missing baudrate"):
        _validate_robot_config(source, tmp_path)

    config.write_text(_ARM_SECTION + _FT_SECTION)
    _validate_robot_config(source, tmp_path)

    config.write_text(_ARM_SECTION + _FT_SECTION + _GRIPPER_SECTION)
    with pytest.raises(RunnerError, match=r"\[agents.gripper1\] configures nothing"):
        _validate_robot_config(source, tmp_path)


def test_the_authored_device_decides_which_sections_the_config_needs(tmp_path) -> None:
    """The gripper's route is authored, not inferred: one section under the arm's device, two
    under separate ones. A section the run cannot reach is as wrong as a missing one."""
    from motion_spec.introspection.runner import RunnerError, _validate_robot_config

    ft = {"kind": "RobotiqFT300s", "config_key": "arm1.wrist_ft", "drives": "wrist_ft"}
    config = tmp_path / "robot.toml"

    interconnect = _real_source(
        tmp_path, {"kind": "KinovaGen3-2F85", "config_key": "agents.arm1", "drives": ""}, ft
    )
    config.write_text(_ARM_SECTION + _FT_SECTION)
    _validate_robot_config(interconnect, tmp_path)
    config.write_text(_ARM_SECTION + _FT_SECTION + _GRIPPER_SECTION)
    with pytest.raises(RunnerError, match=r"\[agents.gripper1\] configures nothing"):
        _validate_robot_config(interconnect, tmp_path)

    separate = _real_source(
        tmp_path,
        {"kind": "KinovaGen3", "config_key": "agents.arm1", "drives": ""},
        {"kind": "Robotiq2F85", "config_key": "agents.gripper1", "drives": ""},
        ft,
    )
    _validate_robot_config(separate, tmp_path)
    config.write_text(_ARM_SECTION + _FT_SECTION)
    with pytest.raises(RunnerError, match=r"no \[agents.gripper1\] section"):
        _validate_robot_config(separate, tmp_path)


def test_a_simulated_run_needs_no_robot_config(tmp_path) -> None:
    from motion_spec.introspection.runner import _validate_robot_config

    source = tmp_path / "generated"
    (source / "model").mkdir(parents=True)
    (source / "model" / "ir.json").write_text(json.dumps({"platform": {"simulated": True}}))
    _validate_robot_config(source, tmp_path)
