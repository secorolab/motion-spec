# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

from __future__ import annotations

import json
import subprocess
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
    MOT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    SLV,
)
from rdf_utils.models.vocab import URI_KC_TYPE_SERIAL
from rdf_utils.namespace import NS_MM_GEOM, NS_MM_KC_EXT
from rdflib import Dataset, Literal, URIRef
from rdflib.namespace import RDF, XSD, Namespace

from motion_spec.classes.handlers import PIDController
from motion_spec.classes.motion import MotionUnit
from motion_spec.classes.scene import MjcfSceneRobot, MjcfSceneSpec
from motion_spec.classes.solvers import CartesianAccelerationDriven
from motion_spec.rdf_parser import (
    communication,
    constraint_handler,
    coordination,
    operations,
    quantities,
    resources,
)
from motion_spec.rdf_parser.constraint_handler import ControllerDerivation, SolverDerivationContext
from motion_spec.rdf_parser.ir import generate_ir
from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.quantities import LINEAR_AXES


def _model(graph: Dataset) -> Model:
    return Model(
        graph=graph, app_path=Path("/tmp/app.json"), imported_models=[], imported_provenance=[]
    )


def test_fixed_attachments_root_a_branched_multi_robot_scene_at_world() -> None:
    graph = Dataset(default_union=True)
    world = URIRef("https://example.test/world")
    table = URIRef("https://example.test/table")
    table_top = URIRef(f"{table}/top")
    trees = {URIRef("https://example.test/arm1"), URIRef("https://example.test/arm2")}

    def frame_on(body: URIRef, frame: URIRef) -> URIRef:
        """Declare the frame as a simplex of its body, the way a scene graph states it."""
        graph.add((body, RDF.type, NS_MM_GEOM["RigidBody"]))
        graph.add((body, NS_MM_GEOM["simplices"], frame))
        return frame

    def fixed(name: str, frame_a: URIRef, frame_b: URIRef) -> None:
        joint = URIRef(f"https://example.test/{name}")
        graph.add((joint, RDF.type, KC.Joint))
        graph.add((joint, KC["between-attachments"], frame_a))
        graph.add((joint, KC["between-attachments"], frame_b))

    fixed("world-table", frame_on(world, URIRef(f"{world}/origin")), frame_on(table, table_top))
    for tree in trees:
        root = URIRef(f"{tree}/base")
        root_frame = frame_on(root, URIRef(f"{root}/origin"))
        tip = URIRef(f"{tree}/tip")
        tip_frame = frame_on(tip, URIRef(f"{tip}/origin"))
        graph.add((tree, NS_MM_KC_EXT["tip"], tip_frame))
        fixed(f"{tree.rsplit('/', 1)[-1]}-table", table_top, root_frame)
        joint = URIRef(f"{tree}/moving")
        graph.add((joint, RDF.type, KC.Joint))
        graph.add((joint, RDF.type, KC.RevoluteJoint))
        graph.add((joint, KC["between-attachments"], root_frame))
        graph.add((joint, KC["between-attachments"], tip_frame))

    attachments, root = resources.fixed_attachments(_model(graph), trees)

    assert root == world
    assert attachments[table][:2] == ("World", "")
    assert {attachments[URIRef(f"{tree}/base")][:2] for tree in trees} == {("Site", "top")}


def _quantity(graph: Dataset, name: str) -> URIRef:
    node = URIRef(f"https://example.test/{name}")
    graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    graph.add((node, RDF.type, QUDT_QKIND.Force))
    graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Force))
    graph.add((node, QUDT_SCHEMA.unit, URIRef("https://qudt.org/vocab/unit/N")))
    return node


def test_parser_scopes_repeated_nested_reference_ids() -> None:
    graph = Dataset(default_union=True)
    graph.bind("example", "https://example.test/")
    first = URIRef("https://example.test/motion/spec/path1/reference")
    second = URIRef("https://example.test/motion/spec/path2/reference")
    graph.add((first, RDF.type, RDF.Property))
    graph.add((second, RDF.type, RDF.Property))

    model = _model(graph)

    assert model.id(first) == "motion_path1_reference"
    assert model.id(second) == "motion_path2_reference"


def _pid_graph(*, kp: float | None = 1.0) -> tuple[Dataset, URIRef]:
    graph = Dataset(default_union=True)
    controller = URIRef("https://example.test/controller")
    graph.add((controller, RDF.type, CSTR_HDL.Controller))
    graph.add((controller, RDF.type, CSTR_HDL.ProportionalIntegralDerivative))
    graph.add((controller, CSTR_HDL["error-signal"], _quantity(graph, "error")))
    graph.add((controller, CSTR_HDL["control-signal"], _quantity(graph, "control")))
    if kp is not None:
        graph.add((controller, CSTR_HDL["proportional-gain"], Literal(kp, datatype=XSD.double)))
    return graph, controller


def test_agent_model_may_bind_the_assembled_kinematic_tree() -> None:
    graph = Dataset(default_union=True)
    base = URIRef("https://example.test/arm/base")
    root = URIRef(f"{base}/root")
    tool = URIRef("https://example.test/gripper/tool")
    tcp = URIRef(f"{tool}/tcp")
    tree = URIRef("https://example.test/assembled")
    joint = URIRef(f"{tree}/fixed")
    agent = URIRef("https://example.test/robot")
    modelled = URIRef("https://example.test/modelled-robot")
    model_node = URIRef("https://example.test/robot-model")

    agent_set = URIRef("https://example.test/robots")
    bdd = Namespace("https://secorolab.github.io/metamodels/acceptance-criteria/bdd#")
    graph.add((agent_set, bdd["elements"], agent))
    graph.add((modelled, RDF.type, AGN.ModelledAgent))
    graph.add((modelled, AGN["of-agent"], agent))
    graph.add((modelled, AGN["has-agent-model"], model_node))
    mapping = URIRef("https://example.test/robot-model/maps-assembled")
    graph.add((model_node, EXEC["has-mapping"], mapping))
    graph.add((mapping, EXEC.maps, tree))
    graph.add((model_node, EXEC.path, Literal("kinova_gen3.xml")))
    graph.add((tree, RDF.type, NS_MM_GEOM["KinematicTree"]))
    graph.add((tree, RDF.type, URI_KC_TYPE_SERIAL))
    graph.add((tree, NS_MM_KC_EXT["root"], root))
    graph.add((tree, NS_MM_KC_EXT["tip"], tcp))
    # Each frame is a simplex of its body, the way a scene graph states it.
    for body, frame in ((base, root), (tool, tcp)):
        graph.add((body, RDF.type, NS_MM_GEOM["RigidBody"]))
        graph.add((body, NS_MM_GEOM["simplices"], frame))
    graph.add((joint, RDF.type, KC.Joint))
    graph.add((joint, KC["between-attachments"], root))
    graph.add((joint, KC["between-attachments"], tcp))

    setups, _ordered = resources.robot_setups(_model(graph))
    setup = setups[agent]

    # `_ChainSetup` composes `chain`/`hardware` bindings now, rather than one flat tuple; the
    # invariant is unchanged -- root/tool naming and the sniffed robot model.
    assert (
        setup.chain.root,
        setup.chain.end,
        setup.chain.tip,
        setup.hardware.model,
        setup.hardware.tool_body,
        setup.hardware.tcp_frame,
    ) == ("base", "tool", "tool", "KinovaGen3", "", "")


def test_introspection_contract_carries_control_and_provenance() -> None:
    graph, controller_node = _pid_graph(kp=2.0)
    graph.add((controller_node, CSTR_HDL["integral-gain"], Literal(0.1, datatype=XSD.double)))
    graph.add((controller_node, CSTR_HDL["derivative-gain"], Literal(0.3, datatype=XSD.double)))
    monitor_node = URIRef("https://example.test/monitor")
    event_node = URIRef("https://example.test/events/complete")
    graph.add((monitor_node, RDF.type, CSTR_HDL.Monitor))
    graph.add((monitor_node, RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
    graph.add((monitor_node, CSTR_HDL.event, event_node))
    # A motion is a subject in every authored model; this hand-built graph has only the
    # controller and monitor, so its IRI is supplied here for the introspection id/IRI check.
    graph.add((URIRef("https://example.test/move"), RDF.type, MOT.GuardedMotion))

    model = Model(
        graph=graph,
        app_path=Path("/tmp/app.json"),
        imported_models=["https://example.test/imported.json"],
        imported_provenance=["/tmp/generated/provenance/dsl.ld.json"],
    )
    controller = PIDController(
        id=model.id(controller_node),
        control_signal=quantities.quantity(
            model, graph.value(controller_node, CSTR_HDL["control-signal"])
        ),
        error_signal=quantities.quantity(
            model, graph.value(controller_node, CSTR_HDL["error-signal"])
        ),
        measured_derivative=None,
        proportional_gain=2.0,
        integral_gain=0.1,
        derivative_gain=0.3,
        decay_rate=None,
        output_saturation=None,
        integral_saturation=None,
        type=model.id(CSTR_HDL.ProportionalIntegralDerivative),
    )
    monitor = coordination.monitor_entry(model, monitor_node)
    motion = MotionUnit(
        id="move",
        name="move",
        description=[],
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
    computation = quantities.build_indexes(
        model, {}, [controller.error_signal, controller.control_signal], {}
    )
    robots = resources.Robots(
        serial_chains=[], platform_velocity=[], platform_force=[], schedule_steps=[]
    )
    scene = MjcfSceneSpec(robots=[MjcfSceneRobot(id="robot", path="robot.xml")])
    platform = {"uri": None, "name": None, "simulated": True, "backend": "mj_kdl"}

    introspection, _values = communication.build_introspection(
        model, [motion], computation, [], robots, scene, platform, 2_000_000, "mj_kdl"
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
    graph = Dataset(default_union=True)
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

    closure = operations.build_closures(_model(graph), operations.OPS_GENERIC)["profile_op"]

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
    from rdf_utils.constraints import ConstraintViolation

    graph = Dataset(default_union=True)
    op = URIRef("https://example.test/profile-op")
    graph.add((op, RDF.type, ALGO_EXT.VelocityProfile))
    graph.add((op, ALGO_EXT["out"], URIRef("https://example.test/reference")))

    with pytest.raises(ConstraintViolation, match="not bound to a constraint"):
        operations.build_closures(_model(graph), operations.OPS_GENERIC)


def test_solver_ir_carries_rne_algorithm_and_gravity() -> None:
    graph = Dataset(default_union=True)
    solver = URIRef("https://example.test/solver")
    gravity = URIRef("https://example.test/gravity")
    graph.add((solver, RDF.type, SLV.SolverWithInputAndOutput))
    graph.add((solver, SLV.solver, SLV.RecursiveNewtonEulerAlgorithm))
    graph.add((solver, SLV.gravity, gravity))
    graph.add((gravity, GEOM_COORD["x"], Literal(0.0, datatype=XSD.double)))
    graph.add((gravity, GEOM_COORD["y"], Literal(0.0, datatype=XSD.double)))
    graph.add((gravity, GEOM_COORD["z"], Literal(9.81, datatype=XSD.double)))

    entry = resources._solver_with_input_and_output(_model(graph), solver, resources._EMPTY_SETUP)

    assert entry.algorithm is CartesianAccelerationDriven
    assert entry.algorithm_name == "RNE"
    # The authored value is the ACHD root acceleration, carried through unchanged.
    assert entry.derived_root_acceleration == [0.0, 0.0, 9.81]
    assert entry.gravity is None

    # The sign flip is KDL's inverse-dynamics convention, not a simulator's, so it is derived
    # wherever an RNE solver is built -- on hardware a missing gravity is silent and dangerous.
    resources.annotate_runtime([entry], [], "mj_kdl")
    assert entry.gravity == [0.0, 0.0, -9.81]


def test_rne_uses_acceleration_while_achd_uses_acceleration_energy() -> None:
    def derive(algorithm: URIRef):
        graph = Dataset(default_union=True)
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
        model = _model(graph)
        context = SolverDerivationContext(
            model=model,
            controllers_by_handler={handler: (plan,)},
            controllers_by_solver={solver: (plan,)},
            algorithm_by_solver={solver: constraint_handler.solver_algorithm(model, solver)},
            shared_constraints=frozenset(),
        )
        return (
            constraint_handler._derived_controllers(model, context, plan)[0].control_signal,
            constraint_handler.motion_drivers(model, context, solver)[0],
        )

    rne_signal, rne_drivers = derive(SLV.RecursiveNewtonEulerAlgorithm)
    achd_signal, achd_drivers = derive(SLV.AccelerationConstrainedHybridDynamicsAlgorithm)

    assert (rne_signal.quantity_kind.id, rne_signal.unit.id) == ("LinearAcceleration", "M_PER_SEC2")
    assert rne_drivers.acceleration_constraint == []
    assert rne_drivers.cartesian_acceleration[0].acceleration == rne_signal
    assert (achd_signal.quantity_kind.id, achd_signal.unit.id) == (
        "AccelerationEnergy",
        "N_M2_PER_SEC2",
    )
    assert achd_drivers.cartesian_acceleration == []
    assert achd_drivers.acceleration_constraint[0].acceleration_energy == achd_signal


def test_edge_monitor_carries_full_event_uri_and_enum_token() -> None:
    graph = Dataset(default_union=True)
    monitor = URIRef("https://example.test/mon")
    event_uri = "http://example.org/coord/E_OBJ_REACHED"
    event = URIRef(event_uri)
    graph.add((monitor, RDF.type, CSTR_HDL.Monitor))
    graph.add((monitor, RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
    graph.add((monitor, CSTR_HDL.event, event))

    entry = coordination.monitor_entry(_model(graph), monitor)

    assert entry.event_uri == event_uri
    # event_name is the coord-dsl FSM enum token (local name, upper-cased, '-' -> '_').
    assert entry.event_name == "E_OBJ_REACHED"


# --- real-world execution (plan 019) ---------------------------------------------------------
def test_real_world_execution_rejects_scene_objects() -> None:
    """A scene object's pose comes from the simulator; on hardware nothing measures it."""
    from motion_spec_dsl.rdf_parser.vocab import ENV
    from rdf_utils.constraints import ConstraintViolation

    graph = Dataset(default_union=True)
    context = URIRef("https://example.test/real-exec")
    cube = URIRef("https://example.test/modelled-cube")
    graph.add((cube, RDF.type, ENV.ModelledObject))
    with pytest.raises(ConstraintViolation, match="cannot use scene objects"):
        resources._reject_scene_objects_on_hardware(_model(graph), context)


def test_simulation_keeps_its_scene_objects() -> None:
    # No real-world context -> nothing to reject.
    resources._reject_scene_objects_on_hardware(_model(Dataset(default_union=True)), None)


def test_an_authored_sensor_rate_reaches_the_ir() -> None:
    """scene-dsl emits sens:update-rate as a QUDT quantity; motion-spec used to drop it.

    Read through scene-dsl's own parser, on the exact call the IR makes."""
    from motion_spec_dsl.rdf_parser.vocab import QUDT_SCHEMA, SENSORS
    from rdf_utils.models.common import ModelBase
    from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT
    from scene_dsl.rdf_parser.sensors import get_update_rate

    graph = Dataset(default_union=True)
    sensor = URIRef("https://example.test/wrist_ft")
    rate = URIRef(f"{sensor}/update-rate")
    graph.add((sensor, RDF.type, SENSORS.ForceTorqueSensor))
    graph.add((sensor, SENSORS["update-rate"], rate))
    graph.add((rate, QUDT_SCHEMA.value, Literal(1000.0, datatype=XSD.double)))
    graph.add((rate, QUDT_SCHEMA.unit, QUDT_UNIT["HZ"]))
    assert get_update_rate(graph, ModelBase(node_id=sensor, graph=graph)) == 1000.0

    # A sensor without a rate is a broken model, not a missing value: the grammar makes
    # update-rate mandatory on every sensor spec.
    rateless = URIRef("https://example.test/rateless")
    graph.add((rateless, RDF.type, SENSORS.ForceTorqueSensor))
    with pytest.raises(ValueError, match="invalid update-rate"):
        get_update_rate(graph, ModelBase(node_id=rateless, graph=graph))


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
                "configuration": {"platform": {"simulated": False, "config": "robot.toml"}},
                "resources": {
                    "robots": [
                        {
                            "id": "arm_solver",
                            "kind": "serial_chain",
                            "config_key": "agents.arm1",
                            "devices": list(devices),
                        }
                    ]
                },
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
    (source / "model" / "ir.json").write_text(
        json.dumps({"configuration": {"platform": {"simulated": True}}})
    )
    _validate_robot_config(source, tmp_path)


def test_an_authored_band_rides_the_constraint_term() -> None:
    """The satisfaction test takes the band the model authored, not the global default.

    The term is what every reader renders from -- the motion's condition, the level monitor and
    the introspection sample -- so the band has to travel with it or only some of them see it.
    """
    from motion_spec.classes.handlers import ConstraintEvaluator, EvaluatorType
    from motion_spec.classes.qudt import Quantity, QuantityKind, Unit
    from motion_spec.rdf_parser.coordination import evaluator_term

    error = Quantity(
        id="near_err",
        quantity_kind=QuantityKind(id="Length"),
        unit=Unit(id="M"),
        value=None,
        has_view=False,
    )

    def evaluator(tolerance):
        return ConstraintEvaluator(
            id="near",
            type_=EvaluatorType.ErrorEvaluator,
            constraint=None,
            error=error,
            tolerance=tolerance,
        )

    band = Quantity(
        id="pos_band",
        quantity_kind=QuantityKind(id="Length"),
        unit=Unit(id="M"),
        value=0.002,
        has_view=False,
    )
    assert evaluator_term(evaluator(band)) == {
        "kind": "constraint",
        "error_id": "near_err",
        "tolerance_id": "pos_band",
    }
    # Omitted, not empty: the template reads a present-but-empty id as a shared value.
    assert evaluator_term(evaluator(None)) == {"kind": "constraint", "error_id": "near_err"}


@pytest.fixture(scope="module")
def dual_ir(tmp_path_factory) -> dict:
    """The dual-arm IR, the only maintained model whose runtimes carry a scoping prefix."""
    model = Path(__file__).parents[2] / "motion-spec-dsl" / "models" / "pick_place_dual"
    if not model.exists():
        pytest.skip("motion-spec-dsl is not in this checkout")
    outdir = tmp_path_factory.mktemp("pick_place_dual") / "generated" / "model"
    subprocess.run(
        ["textx", "generate", "pick_place_dual.robmot", "--target", "jsonld", "-o", str(outdir)],
        cwd=model,
        check=True,
    )
    return generate_ir(outdir / "pick_place_dual-app.ld.json")


def test_chain_joints_are_unprefixed_and_the_runtime_prefix_is_published(dual_ir: dict) -> None:
    """The chain states the scene's joint names; the runtime states the scope they are read in.

    Publishing both is what lets a backend compose the runtime-scoped name itself instead of the
    IR shipping one backend's concatenation of them.
    """
    solvers = dual_ir["resources"]["by_kind"]["serial_chain"]
    prefixes = {solver.runtime.prefix for solver in solvers}
    assert prefixes == {"kinova1_", "kinova2_"}
    for solver in solvers:
        assert solver.chain.joints == [f"joint_{number}" for number in range(1, 8)]
        assert solver.chain.tree == "world_tree"
        assert solver.chain.name.endswith("_tree_chain")


def test_configuration_publishes_the_model_name(dual_ir: dict) -> None:
    """One scalar the backends name their generated artifacts from, instead of one per solver."""
    assert dual_ir["configuration"]["model_name"] == "pick_place_dual"
