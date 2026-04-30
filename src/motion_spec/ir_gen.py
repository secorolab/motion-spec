# SPDX-License-Identifier: MPL-2.0
"""Intermediate representation (IR) generator for motion specification models.

This module parses RDF graphs containing motion specification models and generates
a JSON intermediate representation suitable for code generation.
"""

import sys
import argparse
from dataclasses import dataclass, field, is_dataclass, asdict
from enum import Enum
import collections
import re
import json
from pathlib import Path
import rdflib
from rdf_utils.resolver import IriToFileResolver, install_resolver
from functools import wraps
from rdflib.namespace import RDF
from rdflib import URIRef
from motion_spec.namespace import (
    APP,
    QUDT_SCHEMA,
    QUDT_QKIND,
    QUDT_UNIT,
    GEOM_ENT,
    GEOM_REL,
    GEOM_COORD,
    GEOM_OP,
    RBDYN_ENT,
    RBDYN_COORD,
    RBDYN_OP,
    MAP,
    CSTR,
    MOT,
    CSTR_HDL,
    SLV,
)


class JSONEncoder(json.JSONEncoder):
    def default(self, o):
        if is_dataclass(o) and not isinstance(o, type):
            return asdict(o)
        return super().default(o)


def parse_argument(g, closure_id, argument, to_id):
    # For each of the key differentiate if there is one or more associated value
    entry = list(g[closure_id:argument])
    if len(entry) == 0:
        return None
    elif len(entry) == 1:
        return to_id(entry[0])
    else:
        return [to_id(e) for e in entry]


@dataclass
class Operator:
    type_: URIRef
    input: list[URIRef]
    output: list[URIRef]
    parameters: list = field(default_factory=list)

    def closure_step(self, g, to_id, closure_id):
        closure = {"id": to_id(closure_id), "type": to_id(self.type_)}

        for input in self.input:
            closure[to_id(input)] = parse_argument(g, closure_id, input, to_id)
        for output in self.output:
            closure[to_id(output)] = parse_argument(g, closure_id, output, to_id)
        for param in self.parameters:
            closure[to_id(param)] = parse_argument(g, closure_id, param, to_id)

        return closure

    def from_operator_to_input(self, g, operator_id):
        data_structures = set()

        for in_ in self.input:
            for data_in in g.objects(operator_id, in_):
                data_structures.add(data_in)

        return data_structures

    def from_output_to_operator(self, g, data_out):
        for out in self.output:
            return [op for op in g.subjects(out, data_out) if g[op : RDF["type"] : self.type_]]

    def is_schedulable(self):
        return True

    def scheduler_step(self, g, data_out):
        data_structures = set()
        schedule = []

        for out in self.output:
            for call in g[:out:data_out]:
                if not g[call : RDF["type"] : self.type_]:
                    continue

                # We will only record this call if it has any input
                has_any_input = False

                for in_ in self.input:
                    for data_in in g.objects(call, in_):
                        data_structures.add(data_in)
                        has_any_input = True

                if has_any_input:
                    schedule.append(call)

        return {"data_structures": data_structures, "schedule": schedule}


@dataclass
class Specification:
    """
    A specification is not a computation and, hence, has neither a closure nor
    an entry in a schedule but contributes in finding further computations by
    propagating from outputs to inputs.
    """

    type_: URIRef
    input: list[URIRef]
    output: list[URIRef]
    parameters: list = field(default_factory=list)

    def closure_step(self, g, to_id, closure_id):
        return None

    def from_operator_to_input(self, g, operator_id):
        data_structures = set()

        for in_ in self.input:
            for data_in in g.objects(operator_id, in_):
                data_structures.add(data_in)

        return data_structures

    def from_output_to_operator(self, g, data_out):
        for out in self.output:
            return [op for op in g.subjects(out, data_out) if g[op : RDF["type"] : self.type_]]

    def is_schedulable(self):
        """
        A specification is not callable.
        """
        return False

    def scheduler_step(self, g, data_out):
        data_structures = set()
        for out in self.output:
            for call in g.subjects(out, data_out):
                for in_ in self.input:
                    for data_in in g.objects(call, in_):
                        data_structures.add(data_in)

        return {"data_structures": data_structures, "schedule": []}


class ErrorEvaluator:
    def __init__(self):
        self.type_ = CSTR_HDL["ErrorEvaluator"]
        self.cstr_op = [
            Operator(
                type_=CSTR["EqualityConstraint"],
                input=[CSTR["quantity"], CSTR["reference-value"]],
                output=[CSTR_HDL["error"]],
            ),
            Operator(
                type_=CSTR["GreaterThanConstraint"],
                input=[CSTR["quantity"], CSTR["threshold"]],
                output=[CSTR_HDL["error"]],
            ),
            Operator(
                type_=CSTR["LessThanConstraint"],
                input=[CSTR["quantity"], CSTR["threshold"]],
                output=[CSTR_HDL["error"]],
            ),
            Operator(
                type_=CSTR["BilateralConstraint"],
                input=[CSTR["quantity"], CSTR["lower-threshold"], CSTR["upper-threshold"]],
                output=[CSTR_HDL["error"]],
            ),
        ]

    def closure_step(self, g, to_id, closure_id):
        constraint_id = g.value(closure_id, CSTR_HDL["constraint"])

        for operator in self.cstr_op:
            if operator.type_ not in g[constraint_id : RDF["type"]]:
                continue

            closure = {
                "id": to_id(closure_id),
                "type": "ErrorEvaluator",
                "constraint": to_id(operator.type_),
            }

            for input in operator.input:
                closure[to_id(input)] = parse_argument(g, constraint_id, input, to_id)
            for output in operator.output:
                closure[to_id(output)] = parse_argument(g, closure_id, output, to_id)
            for param in operator.parameters:
                closure[to_id(param)] = parse_argument(g, closure_id, param, to_id)

            # Only return the first matching type
            return closure

        # Should never happen
        return None

    def from_operator_to_input(self, g, operator_id):
        data_structures = set()

        for op in self.cstr_op:
            if op.type_ not in g[operator_id : CSTR_HDL["constraint"] / RDF["type"]]:
                continue

            for in_ in op.input:
                for data_in in g.objects(operator_id, CSTR_HDL["constraint"] / in_):
                    data_structures.add(data_in)

        return data_structures

    def from_output_to_operator(self, g, data_out):
        for operator in self.cstr_op:
            for out in operator.output:
                return [op for op in g.subjects(out, data_out) if g[op : RDF["type"] : self.type_]]

    def is_schedulable(self):
        """
        An error evaluator is not callable (but it will be added separately to
        the schedule!).
        """
        return False

    def scheduler_step(self, g, data_out):
        data_structures = set()
        schedule = []

        for op in self.cstr_op:
            for out in op.output:
                for call in g.subjects(out, data_out):
                    if op.type_ not in g[call : CSTR_HDL["constraint"] / RDF["type"]]:
                        continue

                    # We will only record this call if it has any input
                    has_any_input = False

                    for in_ in op.input:
                        for data_in in g.objects(call, CSTR_HDL["constraint"] / in_):
                            data_structures.add(data_in)
                            has_any_input = True

                    if has_any_input:
                        schedule.append(call)

        return {"data_structures": data_structures, "schedule": schedule}


class AssignmentEvaluator:
    def __init__(self):
        self.type_ = CSTR_HDL["AssignmentEvaluator"]
        self.cstr_op = Operator(
            type_=CSTR["EqualityConstraint"],
            input=[CSTR["quantity"], CSTR["reference-value"]],
            output=[],
        )

    def closure_step(self, g, to_id, closure_id):
        constraint_id = g.value(closure_id, CSTR_HDL["constraint"])

        if self.cstr_op.type_ not in g[constraint_id : RDF["type"]]:
            return None

        closure = {
            "id": to_id(closure_id),
            "type": "AssignmentEvaluator",
            "constraint": to_id(self.cstr_op.type_),
        }

        for input in self.cstr_op.input:
            closure[to_id(input)] = parse_argument(g, constraint_id, input, to_id)
        for param in self.cstr_op.parameters:
            closure[to_id(param)] = parse_argument(g, closure_id, param, to_id)

        # Only return the first matching type
        return closure

    def from_operator_to_input(self, g, operator_id):
        if self.cstr_op.type_ not in g[operator_id : CSTR_HDL["constraint"] / RDF["type"]]:
            return set()

        data_structures = set()
        for in_ in self.cstr_op.input:
            for data_in in g.objects(operator_id, CSTR_HDL["constraint"] / in_):
                data_structures.add(data_in)

        return data_structures

    def is_schedulable(self):
        return True

    def scheduler_step(self, g, data_out):
        return {"data_structures": [], "schedule": []}


ops_generic = [
    Operator(
        type_=GEOM_OP["RotateDirectionDistalToProximalWithPose"],
        input=[GEOM_OP["pose"], GEOM_OP["from"]],
        output=[GEOM_OP["to"]],
    ),
    Operator(
        type_=GEOM_OP["ComposePose"],
        input=[GEOM_OP["in1"], GEOM_OP["in2"]],
        output=[GEOM_OP["composite"]],
    ),
    Operator(
        type_=GEOM_OP["InvertPose"],
        input=[GEOM_OP["pose"]],
        output=[GEOM_OP["out"]],
    ),
    Operator(
        type_=GEOM_OP["RotateVelocityTwistToProximalWithPose"],
        input=[GEOM_OP["pose"], GEOM_OP["from"]],
        output=[GEOM_OP["to"]],
    ),
    Operator(
        type_=GEOM_OP["PoseToAngleAroundAxis"],
        input=[GEOM_OP["pose"]],
        output=[GEOM_OP["angle"]],
        parameters=[GEOM_OP["axis"]],
    ),
    Operator(
        type_=GEOM_OP["PoseToLinearDistance"], input=[GEOM_OP["pose"]], output=[GEOM_OP["distance"]]
    ),
    Operator(
        type_=GEOM_OP["PoseToDirection"], input=[GEOM_OP["pose"]], output=[GEOM_OP["direction"]]
    ),
    Operator(
        type_=GEOM_OP["PlanarAngleFromDirections"],
        input=[GEOM_OP["from-directions"]],
        output=[GEOM_OP["angle"]],
    ),
    Operator(type_=GEOM_OP["InvertAngle"], input=[GEOM_OP["in"]], output=[GEOM_OP["out"]]),
    Operator(
        type_=RBDYN_OP["AddWrench"],
        input=[RBDYN_OP["in1"], RBDYN_OP["in2"]],
        output=[RBDYN_OP["out"]],
    ),
    Operator(
        type_=RBDYN_OP["RotateWrenchToDistalWithPose"],
        input=[RBDYN_OP["pose"], RBDYN_OP["from"]],
        output=[RBDYN_OP["to"]],
    ),
    Operator(
        type_=RBDYN_OP["RotateWrenchToProximalWithPose"],
        input=[RBDYN_OP["pose"], RBDYN_OP["from"]],
        output=[RBDYN_OP["to"]],
    ),
    Operator(
        type_=RBDYN_OP["TransformWrenchToProximal"],
        input=[RBDYN_OP["pose"], RBDYN_OP["from"]],
        output=[RBDYN_OP["to"]],
    ),
    Operator(
        type_=RBDYN_OP["WrenchFromPositionDirectionAndMagnitude"],
        input=[RBDYN_OP["magnitude"], RBDYN_OP["direction"], RBDYN_OP["position"]],
        output=[RBDYN_OP["wrench"]],
    ),
    Specification(
        type_=MAP["View"],
        input=[MAP["superobject"], MAP["subobject"]],
        output=[MAP["superobject"], MAP["subobject"]],
    ),
    Operator(
        type_=GEOM_OP["PoseDiffEvaluator"],
        input=[GEOM_OP["in1"], GEOM_OP["in2"]],
        output=[GEOM_OP["out"]],
    ),
]

ops_cstr_hdl = [
    Operator(
        type_=CSTR_HDL["Controller"],
        input=[CSTR_HDL["error-signal"]],
        output=[CSTR_HDL["control-signal"]],
        parameters=[
            CSTR_HDL["proportional-gain"],
            CSTR_HDL["integral-gain"],
            CSTR_HDL["derivative-gain"],
            CSTR_HDL["decay-rate"],
        ],
    ),
    AssignmentEvaluator(),
    ErrorEvaluator(),
]

ops_slv = [
    Specification(type_=SLV["CartesianForceSpecification"], input=[SLV["force"]], output=[]),
    Specification(
        type_=SLV["AccelerationConstraint"], input=[SLV["acceleration-energy"]], output=[]
    ),
    Specification(type_=SLV["ForceDistributionSolver"], input=[SLV["force"]], output=[]),
]


class Subspace(str, Enum):
    Position = "Position"
    AngularVelocity = "AngularVelocity"
    LinearVelocity = "LinearVelocity"
    AngularAcceleration = "AngularAcceleration"
    LinearAcceleration = "LinearAcceleration"
    Torque = "Torque"
    Force = "Force"


class Axis(str, Enum):
    X = "X"
    Y = "Y"
    Z = "Z"


class UnilateralConstraintType(str, Enum):
    GreaterThan = "GreaterThan"
    LessThan = "LessThan"


class EvaluatorType(str, Enum):
    AssignmentEvaluator = "AssignmentEvaluator"
    ErrorEvaluator = "ErrorEvaluator"


@dataclass
class Point:
    id: str
    type: str = field(default="Point")


@dataclass
class Frame:
    id: str
    type: str = field(default="Frame")


@dataclass
class QuantityKind:
    id: str
    type: str = field(default="QuantityKind")


@dataclass
class Unit:
    id: str
    type: str = field(default="Unit")


@dataclass
class Quantity:
    id: str
    quantity_kind: QuantityKind
    unit: Unit
    value: float
    has_view: bool
    type: str = field(default="Quantity")


@dataclass
class PoseQuantity:
    id: str
    quantity_kind: QuantityKind
    unit: Unit
    has_view: bool
    value: None = None
    type: str = field(default="PoseQuantity")


@dataclass
class SimplicialComplex:
    id: str
    type: str = field(default="SimplicialComplex")


@dataclass
class Direction:
    id: str
    quantity_kind: list[QuantityKind]
    as_seen_by: Frame
    unit: list[Unit]
    direction: list[float] | None
    type: str = field(default="Direction")


@dataclass
class Position:
    id: str
    of: SimplicialComplex | Point
    with_respect_to: SimplicialComplex | Point
    quantity_kind: QuantityKind
    as_seen_by: Frame
    unit: Unit
    position: list[float] | None
    type: str = field(default="Position")


@dataclass
class Pose:
    id: str
    of: SimplicialComplex | Frame
    with_respect_to: SimplicialComplex | Frame
    quantity_kind: list[QuantityKind]
    as_seen_by: Frame
    unit: list[Unit]
    direction_cosine_x: list[float] | None
    direction_cosine_y: list[float] | None
    direction_cosine_z: list[float] | None
    position: list[float] | None
    type: str = field(default="Pose")


@dataclass
class VelocityTwist:
    id: str
    of: SimplicialComplex
    with_respect_to: SimplicialComplex
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    type: str = field(default="VelocityTwist")


@dataclass
class AccelerationTwist:
    id: str
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    type: str = field(default="AccelerationTwist")


@dataclass
class Wrench:
    id: str
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    type: str = field(default="Wrench")


@dataclass
class View:
    id: str
    superobject: VelocityTwist | AccelerationTwist | Wrench
    subobject: Quantity
    subspace: Subspace
    axis: Axis
    type: str = field(default="View")


@dataclass
class EqualityConstraint:
    reference_value: Quantity
    type: str = field(default="EqualityConstraint")


@dataclass
class UnilateralConstraint:
    type_: UnilateralConstraintType
    threshold: Quantity
    type: str = field(default="UnilateralConstraint")


@dataclass
class BilateralConstraint:
    lower_threshold: Quantity
    upper_threshold: Quantity
    type: str = field(default="BilateralConstraint")


@dataclass
class Constraint:
    id: str
    quantity: Quantity
    parameter: EqualityConstraint | UnilateralConstraint | BilateralConstraint
    type: str = field(default="Constraint")


@dataclass
class GuardedMotion:
    id: str
    when: list[Constraint]
    while_: list[Constraint]
    until: list[Constraint]
    type: str = field(default="GuardedMotion")


@dataclass
class ConstraintEvaluator:
    id: str
    type_: EvaluatorType
    constraint: Constraint
    error: Quantity | None
    type: str = field(default="ConstraintEvaluator")


@dataclass
class Controller:
    id: str
    error_signal: Quantity
    control_signal: Quantity
    proportional_gain: float
    integral_gain: float
    derivative_gain: float
    decay_rate: float | None
    type: str = field(default="Controller")


@dataclass
class MonitorEntry:
    id: str
    monitor_type: str
    error: Quantity
    flag: str | None
    event: str | None
    event_idx: int | None
    is_edge_triggered: bool
    type: str = field(default="MonitorEntry")


@dataclass
class ConstraintHandler:
    id: str
    motion: GuardedMotion
    evaluators: list[ConstraintEvaluator]
    controllers: list[Controller]
    monitors: list[MonitorEntry]
    order: int = 0
    type: str = field(default="ConstraintHandler")


@dataclass
class SnapshotCapture:
    target_id: str
    source_id: str
    source_closure_id: str | None = None
    type: str = field(default="SnapshotCapture")


@dataclass
class RelativePoseCapture:
    id: str
    fk_pose_id: str
    type: str = field(default="RelativePoseCapture")


@dataclass
class GuardedMotionBlock:
    id: str
    handler: str
    motion: GuardedMotion

    # Evaluators
    when_evaluators: list[ConstraintEvaluator]
    while_evaluators: list[ConstraintEvaluator]
    until_evaluators: list[ConstraintEvaluator]

    # Controllers and Monitors
    controllers: list[Controller]
    when_monitors: list[MonitorEntry]
    while_monitors: list[MonitorEntry]
    until_monitors: list[MonitorEntry]

    # Schedules
    # when_schedule is independent: it runs in can_start, a separate C++ function.
    # while_schedule and until_schedule are NOT independent schedules; they are
    # complementary slices of a single shared active computation graph (same C++
    # control function). They must be built from one shared Parser so that steps
    # common to both phases are emitted only once and deduplication is correct.
    when_schedule: list[str]
    while_schedule: list[str]
    until_schedule: list[str]

    # FSM Interaction
    when_events: list[str]
    while_events: list[str]
    until_events: list[str]
    has_until_condition: bool = False

    # Solver Integration
    arm_solvers: list = field(default_factory=list)

    # Snapshot captures (once-at-start scalar assignments)
    snapshots: list = field(default_factory=list)

    # Relative-from-start pose computations (e.g. pose_start_ee)
    relative_poses: list = field(default_factory=list)

    type: str = field(default="GuardedMotionBlock")


@dataclass
class AccelerationConstraint:
    id: str
    subspace: Subspace
    axis: Axis
    acceleration_energy: Quantity
    as_seen_by: Frame | None = None
    base_aligned: bool = True
    type: str = field(default="AccelerationConstraint")


@dataclass
class AccelerationConstraintSpecification:
    id: str
    constraints: list[AccelerationConstraint]
    attached_to: SimplicialComplex | None
    type: str = field(default="AccelerationConstraintSpecification")


@dataclass
class CartesianForceSpecification:
    id: str
    force: Wrench
    attached_to: SimplicialComplex
    type: str = field(default="CartesianForceSpecification")


@dataclass
class MotionDrivers:
    id: str
    acceleration_constraint: list[AccelerationConstraintSpecification]
    cartesian_force: list[CartesianForceSpecification]
    type: str = field(default="MotionDrivers")


@dataclass
class MotionArmSolver:
    id: str
    output: list
    motion_driver: MotionDrivers
    has_cartesian_force: bool = False
    root_acc: list[float] | None = None
    type: str = field(default="MotionArmSolver")


@dataclass
class SolverWithInputAndOutput:
    id: str
    motion_drivers: list[MotionDrivers]
    output: list
    urdf: str = ""
    chain_root: str = ""
    chain_end: str = ""
    robot_type: str = ""
    robot_model: str = ""
    root_acc: list[float] | None = None
    type: str = field(default="SolverWithInputAndOutput")


@dataclass
class VelocityCompositionSolver:
    id: str
    configuration: str
    velocity: VelocityTwist
    type: str = field(default="VelocityCompositionSolver")


@dataclass
class ForceDistributionSolver:
    id: str
    configuration: str
    force: Wrench
    type: str = field(default="ForceDistributionSolver")


def memoize(func):
    @wraps(func)
    def decorator(self, *args, **kwargs):
        key = args + tuple(kwargs.items())
        if key not in self.cache:
            self.cache[key] = func(self, *args, **kwargs)
        return self.cache[key]

    return decorator


def escape(s):
    s = re.sub(r"[^0-9A-Za-z_]", "_", str(s))
    if s and s[0].isdigit():
        s = f"_{s}"
    return s


class Parser:
    def __init__(self, g):
        self.cache = dict()
        self.g = g
        self.sched = set()

    def id(self, x):
        try:
            q = self.g.compute_qname(x)
            return escape(q[2])
        except:
            return x

    @memoize
    def velocity_composition_solver(self, id_):
        assert SLV["VelocityCompositionSolver"] in self.g[id_ : RDF["type"]]

        conf = self.id(self.g.value(id_, SLV["configuration"]))
        velocity = self.velocity_twist(self.g.value(id_, SLV["velocity"]))

        return VelocityCompositionSolver(self.id(id_), conf, velocity)

    @memoize
    def force_distribution_solver(self, id_):
        assert SLV["ForceDistributionSolver"] in self.g[id_ : RDF["type"]]

        conf = self.id(self.g.value(id_, SLV["configuration"]))
        force = self.wrench(self.g.value(id_, SLV["force"]))

        return ForceDistributionSolver(self.id(id_), conf, force)

    @memoize
    def solver_with_input_and_output(self, id_):
        assert SLV["SolverWithInputAndOutput"] in self.g[id_ : RDF["type"]]

        io_dispatcher = [
            (GEOM_COORD["PoseCoordinate"], self.pose),
            (GEOM_COORD["VelocityTwistCoordinate"], self.velocity_twist),
        ]

        drv = []
        for motion_driver in self.g[id_ : SLV["motion-drivers"]]:
            drv.append(self.motion_drivers(motion_driver))
        out = []
        for o in self.g[id_ : SLV["output"]]:
            for type_, func in io_dispatcher:
                if type_ in self.g[o : RDF["type"]]:
                    out.append(func(o))

        urdf = str(self.g.value(id_, APP["urdf"]) or "")
        chain_root = str(self.g.value(id_, APP["chain-root"]) or "")
        chain_end = str(self.g.value(id_, APP["chain-end"]) or "")
        robot_type = str(self.g.value(id_, APP["robot-type"]) or "")
        robot_model = str(self.g.value(id_, APP["robot-model"]) or "")
        gravity_node = self.g.value(id_, SLV["gravity-value"])
        gravity = self.parse_xyz(gravity_node) if gravity_node else None
        root_acc = [-v for v in gravity] if gravity else None

        return SolverWithInputAndOutput(
            self.id(id_), drv, out, urdf, chain_root, chain_end, robot_type, robot_model, root_acc
        )

    @memoize
    def motion_drivers(self, id_):
        assert SLV["MotionDrivers"] in self.g[id_ : RDF["type"]]

        spec_acc = []
        spec_frc = []

        for a in self.g[id_ : SLV["acceleration-constraint"]]:
            spec_acc.append(self.acceleration_constraint_specification(a))

        for f in self.g[id_ : SLV["cartesian-force"]]:
            spec_frc.append(self.cartesian_force_specification(f))

        return MotionDrivers(self.id(id_), spec_acc, spec_frc)

    @memoize
    def cartesian_force_specification(self, id_):
        assert SLV["CartesianForceSpecification"] in self.g[id_ : RDF["type"]]

        force = self.wrench(self.g.value(id_, SLV["force"]))
        attached_to = self.simplicial_complex(self.g.value(id_, SLV["attached-to"]))

        return CartesianForceSpecification(self.id(id_), force, attached_to)

    @memoize
    def acceleration_constraint_specification(self, id_):
        assert SLV["AccelerationConstraintSpecification"] in self.g[id_ : RDF["type"]]

        constraints = []
        for c in self.g[id_ : SLV["constraints"]]:
            constraints.append(self.acceleration_constraint(c))
        attached_to_node = self.g.value(id_, SLV["attached-to"])
        attached_to = self.simplicial_complex(attached_to_node) if attached_to_node else None

        return AccelerationConstraintSpecification(self.id(id_), constraints, attached_to)

    @memoize
    def acceleration_constraint(self, id_):
        assert SLV["AccelerationConstraint"] in self.g[id_ : RDF["type"]]
        assert SLV["AxisAligned"] in self.g[id_ : RDF["type"]]

        subspace = self.subspace(self.g.value(id_, SLV["subspace"]))
        axis = self.axis(self.g.value(id_, SLV["axis"]))
        e_acc = self.quantity(self.g.value(id_, SLV["acceleration-energy"]))
        as_seen_by_node = self.g.value(id_, GEOM_COORD["as-seen-by"])
        as_seen_by = self.frame(as_seen_by_node) if as_seen_by_node else None

        return AccelerationConstraint(self.id(id_), subspace, axis, e_acc, as_seen_by)

    @memoize
    def subspace(self, id_):
        d = {
            MAP["position"]: Subspace.Position,
            MAP["angular-velocity"]: Subspace.AngularVelocity,
            MAP["linear-velocity"]: Subspace.LinearVelocity,
            MAP["angular-acceleration"]: Subspace.AngularAcceleration,
            MAP["linear-acceleration"]: Subspace.LinearAcceleration,
            MAP["torque"]: Subspace.Torque,
            MAP["force"]: Subspace.Force,
            SLV["angular-acceleration"]: Subspace.AngularAcceleration,
            SLV["linear-acceleration"]: Subspace.LinearAcceleration,
        }
        assert id_ in d.keys()

        return d[id_]

    @memoize
    def axis(self, id_):
        d = {
            MAP["x"]: Axis.X,
            MAP["y"]: Axis.Y,
            MAP["z"]: Axis.Z,
            SLV["x"]: Axis.X,
            SLV["y"]: Axis.Y,
            SLV["z"]: Axis.Z,
        }
        assert id_ in d.keys()

        return d[id_]

    @memoize
    def constraint_handler(self, id_):
        assert CSTR_HDL["ConstraintHandler"] in self.g[id_ : RDF["type"]]

        motion = self.guarded_motion(self.g.value(id_, CSTR_HDL["motion"]))

        evaluators = []
        for e in self.g[id_ : CSTR_HDL["evaluators"]]:
            if GEOM_OP["PoseDiffEvaluator"] in self.g[e : RDF["type"]]:
                continue  # handled via schedule traversal
            evaluators.append(self.constraint_evaluator(e))

        controllers = []
        for c in self.g[id_ : CSTR_HDL["controllers"]]:
            controllers.append(self.controller(c))

        monitors = []
        for m in self.g[id_ : CSTR_HDL["monitors"]]:
            monitors.append(self.monitor_entry(m))

        order_value = self.g.value(id_, APP["order"])
        order = int(order_value.value) if order_value is not None else 0

        return ConstraintHandler(self.id(id_), motion, evaluators, controllers, monitors, order)

    @memoize
    def monitor_entry(self, id_):
        assert CSTR_HDL["Monitor"] in self.g[id_ : RDF["type"]]

        error = self.quantity(self.g.value(id_, CSTR_HDL["error"]))

        if CSTR_HDL["LevelTriggeredMonitor"] in self.g[id_ : RDF["type"]]:
            flag = self.id(self.g.value(id_, CSTR_HDL["flag"]))
            return MonitorEntry(self.id(id_), "LevelTriggeredMonitor", error, flag, None, None, False)

        event = self.id(self.g.value(id_, CSTR_HDL["event"]))
        return MonitorEntry(self.id(id_), "EdgeTriggeredMonitor", error, None, event, None, True)

    @memoize
    def constraint_evaluator(self, id_):
        assert CSTR_HDL["ConstraintEvaluator"] in self.g[id_ : RDF["type"]]

        constraint = self.constraint(self.g.value(id_, CSTR_HDL["constraint"]))

        if CSTR_HDL["AssignmentEvaluator"] in self.g[id_ : RDF["type"]]:
            t = EvaluatorType.AssignmentEvaluator
            error = None
        else:
            t = EvaluatorType.ErrorEvaluator
            error = self.quantity(self.g.value(id_, CSTR_HDL["error"]))

        return ConstraintEvaluator(self.id(id_), t, constraint, error)

    @memoize
    def controller(self, id_):
        assert CSTR_HDL["Controller"] in self.g[id_ : RDF["type"]]
        assert CSTR_HDL["ProportionalIntegralDerivative"] in self.g[id_ : RDF["type"]]

        error_signal = self.quantity(self.g.value(id_, CSTR_HDL["error-signal"]))
        control_signal = self.quantity(self.g.value(id_, CSTR_HDL["control-signal"]))

        decay_rate = None
        if CSTR_HDL["DecayingIntegralTerm"] in self.g[id_ : RDF["type"]]:
            decay_rate = self.g.value(id_, CSTR_HDL["decay-rate"]).value

        p = self.g.value(id_, CSTR_HDL["proportional-gain"]).value
        i = self.g.value(id_, CSTR_HDL["integral-gain"]).value
        d = self.g.value(id_, CSTR_HDL["derivative-gain"]).value

        return Controller(self.id(id_), error_signal, control_signal, p, i, d, decay_rate)

    @memoize
    def guarded_motion(self, id_):
        assert MOT["GuardedMotion"] in self.g[id_ : RDF["type"]]

        when = []
        for c in self.g[id_ : MOT["when"]]:
            when.append(self.constraint(c))

        while_ = []
        for c in self.g[id_ : MOT["while"]]:
            while_.append(self.constraint(c))

        until = []
        for c in self.g[id_ : MOT["until"]]:
            until.append(self.constraint(c))

        return GuardedMotion(self.id(id_), when, while_, until)

    @memoize
    def constraint(self, id_):
        assert CSTR["Constraint"] in self.g[id_ : RDF["type"]]

        quantity = self.quantity(self.g.value(id_, CSTR["quantity"]))

        parameter = None
        if CSTR["EqualityConstraint"] in self.g[id_ : RDF["type"]]:
            parameter = self.equality_constraint(id_)
        elif CSTR["UnilateralConstraint"] in self.g[id_ : RDF["type"]]:
            parameter = self.unilateral_constraint(id_)
        else:
            parameter = self.bilateral_constraint(id_)

        return Constraint(self.id(id_), quantity, parameter)

    @memoize
    def equality_constraint(self, id_):
        assert CSTR["EqualityConstraint"] in self.g[id_ : RDF["type"]]

        reference_value = self.quantity(self.g.value(id_, CSTR["reference-value"]))

        return EqualityConstraint(reference_value)

    @memoize
    def unilateral_constraint(self, id_):
        assert CSTR["UnilateralConstraint"] in self.g[id_ : RDF["type"]]

        threshold = self.quantity(self.g.value(id_, CSTR["threshold"]))
        type_ = UnilateralConstraintType.LessThan
        if CSTR["GreaterThanConstraint"] in self.g[id_ : RDF["type"]]:
            type_ = UnilateralConstraintType.GreaterThan

        return UnilateralConstraint(type_, threshold)

    @memoize
    def bilateral_constraint(self, id_):
        assert CSTR["BilateralConstraint"] in self.g[id_ : RDF["type"]]

        lower_threshold = self.quantity(self.g.value(id_, CSTR["lower-threshold"]))
        upper_threshold = self.quantity(self.g.value(id_, CSTR["upper-threshold"]))

        return BilateralConstraint(lower_threshold, upper_threshold)

    @memoize
    def direction(self, id_):
        assert GEOM_COORD["DirectionCoordinate"] in self.g[id_ : RDF["type"]]
        assert GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]]

        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.quantity_kind(k))
        for k in self.g[id_ : QUDT_SCHEMA["quantity-kind"]]:
            quantity_kind.append(self.quantity_kind(k))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = self.unit(self.g.value(id_, QUDT_SCHEMA["unit"]))
        direction = self.parse_xyz(id_)

        return Direction(self.id(id_), quantity_kind, as_seen_by, [Unit(unit)], direction)

    def parse_vector3(self, node):
        from rdflib import collection

        items = list(collection.Collection(self.g, node))
        if len(items) != 3:
            return None

        # Get the Python representation of the associated RDF literal
        return [float(v.toPython()) for v in items]

    def parse_direction_cosine_xyz(self, node):
        cos_x = self.parse_vector3(self.g.value(node, GEOM_COORD["direction-cosine-x"]))
        cos_y = self.parse_vector3(self.g.value(node, GEOM_COORD["direction-cosine-y"]))
        cos_z = self.parse_vector3(self.g.value(node, GEOM_COORD["direction-cosine-z"]))

        if cos_x is None or cos_y is None or cos_z is None:
            return None

        mat_col_major = []
        for col in [cos_x, cos_y, cos_z]:
            mat_col_major.extend(col)

        return mat_col_major

    def parse_xyz(self, node):
        x = self.g.value(node, GEOM_COORD["x"])
        y = self.g.value(node, GEOM_COORD["y"])
        z = self.g.value(node, GEOM_COORD["z"])

        if x is None or y is None or z is None:
            return None

        return [float(v.value) for v in (x, y, z)]

    @memoize
    def position(self, id_):
        assert GEOM_COORD["PositionCoordinate"] in self.g[id_ : RDF["type"]]
        assert GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]]

        of = self.point(self.g.value(id_, GEOM_REL["of"]))
        wrt = self.point(self.g.value(id_, GEOM_REL["with-respect-to"]))
        quantity_kind = self.quantity_kind(self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = self.unit(self.g.value(id_, QUDT_SCHEMA["unit"]))
        pos = self.parse_xyz(id_)

        return Position(
            self.id(id_), of, wrt, QuantityKind(quantity_kind), as_seen_by, Unit(unit), pos
        )

    @memoize
    def pose(self, id_):
        assert GEOM_COORD["PoseCoordinate"] in self.g[id_ : RDF["type"]]
        assert GEOM_COORD["DirectionCosineXYZ"] in self.g[id_ : RDF["type"]]
        assert GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]]

        of = self.frame(self.g.value(id_, GEOM_REL["of"]))
        wrt = self.frame(self.g.value(id_, GEOM_REL["with-respect-to"]))
        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.quantity_kind(k))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = []
        for u in self.g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.unit(u))
        dc_x = self.parse_vector3(self.g.value(id_, GEOM_COORD["direction-cosine-x"]))
        dc_y = self.parse_vector3(self.g.value(id_, GEOM_COORD["direction-cosine-y"]))
        dc_z = self.parse_vector3(self.g.value(id_, GEOM_COORD["direction-cosine-z"]))
        pos = self.parse_xyz(id_)

        return Pose(self.id(id_), of, wrt, quantity_kind, as_seen_by, unit, dc_x, dc_y, dc_z, pos)

    @memoize
    def velocity_twist(self, id_):
        assert GEOM_COORD["VelocityTwistCoordinate"] in self.g[id_ : RDF["type"]]
        assert GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]]

        of = self.simplicial_complex(self.g.value(id_, GEOM_REL["of"]))
        wrt = self.simplicial_complex(self.g.value(id_, GEOM_REL["with-respect-to"]))
        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.quantity_kind(k))
        reference_point = self.point(self.g.value(id_, GEOM_REL["reference-point"]))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = []
        for u in self.g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.unit(u))

        return VelocityTwist(
            self.id(id_), of, wrt, quantity_kind, reference_point, as_seen_by, unit
        )

    @memoize
    def acceleration_twist(self, id_):
        assert GEOM_COORD["AccelerationTwistCoordinate"] in self.g[id_ : RDF["type"]]
        assert GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]]

        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.quantity_kind(k))
        reference_point = self.point(self.g.value(id_, GEOM_REL["reference-point"]))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = []
        for u in self.g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.unit(u))

        return AccelerationTwist(self.id(id_), quantity_kind, reference_point, as_seen_by, unit)

    @memoize
    def wrench(self, id_):
        assert RBDYN_COORD["WrenchCoordinate"] in self.g[id_ : RDF["type"]]
        #assert(RBDYN_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]])

        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.quantity_kind(k))
        reference_point = self.point(self.g.value(id_, RBDYN_ENT["reference-point"]))
        as_seen_by = self.frame(self.g.value(id_, RBDYN_COORD["as-seen-by"]))
        unit = []
        for u in self.g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.unit(u))

        return Wrench(self.id(id_), quantity_kind, reference_point, as_seen_by, unit)

    @memoize
    def quantity(self, id_):
        assert QUDT_SCHEMA["Quantity"] in self.g[id_ : RDF["type"]]

        quantity_kind_node = (
            self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"])
            or self.g.value(id_, QUDT_SCHEMA["quantity-kind"])
        )
        quantity_kind = self.quantity_kind(quantity_kind_node)

        unit = self.unit(self.g.value(id_, QUDT_SCHEMA["unit"]))
        has_view = (id_, ~MAP["subobject"], None) in self.g

        if GEOM_REL["Pose"] in self.g[id_ : RDF["type"]]:
            return PoseQuantity(self.id(id_), QuantityKind(quantity_kind), Unit(unit), has_view)

        value = None
        if (id_, QUDT_SCHEMA["value"], None) in self.g:
            value = float(self.g.value(id_, QUDT_SCHEMA["value"]))
        return Quantity(self.id(id_), QuantityKind(quantity_kind), Unit(unit), value, has_view)

    @memoize
    def quantity_kind(self, id_):
        return self.id(id_)

    @memoize
    def unit(self, id_):
        return self.id(id_)

    @memoize
    def simplicial_complex(self, id_):
        assert GEOM_ENT["SimplicialComplex"] in self.g[id_ : RDF["type"]]

        return SimplicialComplex(self.id(id_))

    @memoize
    def frame(self, id_):
        assert GEOM_ENT["Frame"] in self.g[id_ : RDF["type"]]

        return Frame(self.id(id_))

    @memoize
    def point(self, id_):
        assert GEOM_ENT["Point"] in self.g[id_ : RDF["type"]]

        return Point(self.id(id_))

    def view(self):
        dispatcher = [
            (MAP["DirectionCoordinateView"], self.direction),
            (MAP["PoseCoordinateView"], self.pose),
            (MAP["VelocityTwistCoordinateView"], self.velocity_twist),
            (MAP["AccelerationTwistCoordinateView"], self.acceleration_twist),
            (MAP["WrenchCoordinateView"], self.wrench),
        ]

        view_map = {}
        for view in self.g[: RDF["type"] : MAP["View"]]:
            superobject = None
            for type_, func in dispatcher:
                if type_ not in self.g[view : RDF["type"]]:
                    continue

                superobject_id = self.g.value(view, MAP["superobject"])
                superobject = func(superobject_id)
                break

            subobject = self.quantity(self.g.value(view, MAP["subobject"]))
            subspace = self.subspace(self.g.value(view, MAP["subspace"]))
            axis = self.axis(self.g.value(view, MAP["axis"]))

            assert superobject is not None
            view_map[self.id(subobject.id)] = View(
                self.id(view), superobject, subobject, subspace, axis
            )

        return view_map

    def data_structures(self):
        dispatcher = [
            (GEOM_COORD["DirectionCoordinate"], self.direction),
            (GEOM_COORD["PositionCoordinate"], self.position),
            (GEOM_COORD["PoseCoordinate"], self.pose),
            (GEOM_COORD["VelocityTwistCoordinate"], self.velocity_twist),
            (GEOM_COORD["AccelerationTwistCoordinate"], self.acceleration_twist),
            (RBDYN_COORD["WrenchCoordinate"], self.wrench),
            (QUDT_SCHEMA["Quantity"], self.quantity),
        ]

        data_structures = []
        for type_, func in dispatcher:
            for dstruct in self.g[: RDF["type"] : type_]:
                data_structures.append(func(dstruct))

        return data_structures

    def closures(self, operators):
        closures = {}
        for operator in operators:
            for closure in self.g.subjects(RDF["type"], operator.type_):
                cl = operator.closure_step(self.g, self.id, closure)
                if cl:
                    closures[self.id(closure)] = cl

        return closures

    def schedule(self, start, ops):
        # Start at a SolverWithInputAndOutput
        # Then traverse along data structure and collect function blocks
        q = collections.deque()
        data_structures = set()
        sched = []
        for v in start:
            for op in ops:
                if op.type_ not in self.g[v : RDF["type"]]:
                    continue

                call = self.id(v)
                if op.is_schedulable() and call not in set(self.sched):
                    sched.append(call)
                    self.sched.add(call)

                for data_in in op.from_operator_to_input(self.g, v):
                    q.append(data_in)
                    data_structures.add(data_in)

        # This is how an operator/call looks in the "forward" direction:
        #   data_in --(in)--> operator/call --(out)--> data_out
        # But it will be traversed in the "backward" direction.
        # Note that there may exist multiple inputs and outputs.
        while len(q) > 0:
            data_out = q.pop()
            # Iterate through all allowed operators and their outputs
            for op in ops:
                res = op.scheduler_step(self.g, data_out)

                for call in res["schedule"]:
                    call = self.id(call)
                    if call and call not in set(self.sched):
                        sched.append(call)
                        self.sched.add(call)

                for data_in in res["data_structures"]:
                    # We have already visited this data structure,
                    # so skip it
                    if data_in in data_structures:
                        continue

                    q.append(data_in)
                    data_structures.add(data_in)

        sched.reverse()
        return sched

    def get_schedule(self) -> list[str]:
        s = list(self.sched)
        s.reverse()
        return s


def _upstream_dependencies(data_id: str, closure_input_map: dict[str, set[str]]) -> set[str]:
    result: set[str] = set()
    pending = list(closure_input_map.get(data_id, set()))
    while pending:
        item = pending.pop()
        if item in result:
            continue
        result.add(item)
        pending.extend(closure_input_map.get(item, set()))
    return result


def _body_name(name: str | None) -> str | None:
    if name is None:
        return None
    for prefix in ("frame_", "frame-", "link_", "link-"):
        if name.startswith(prefix):
            return name[len(prefix):]
    return name


def _mark_acceleration_constraint_frames(solver):
    root_body = _body_name(getattr(solver, "chain_root", None))
    for driver in solver.motion_drivers:
        for acc_spec in driver.acceleration_constraint:
            for constraint in acc_spec.constraints:
                axis_frame = getattr(constraint.as_seen_by, "id", None)
                constraint.base_aligned = axis_frame is None or _body_name(axis_frame) == root_body


def _arm_solvers_for_handler(handler, slv_arm, closure_input_map=None):
    """Find arm solvers whose motion drivers consume this handler's controller outputs.

    A handler drives an arm solver through controller ``control_signal`` quantities.
    Those quantities are referenced by the solver graph either as
    ``acceleration-energy`` entries in acceleration constraints or as ``force``
    entries in cartesian-force specifications.
    """
    closure_input_map = closure_input_map or {}
    handler_output_ids = {c.control_signal.id for c in handler.controllers}
    if not handler_output_ids:
        return []

    result = []
    motion_driver_id = f"driver_{handler.motion.id.removeprefix('motion_')}"
    for solver in slv_arm:
        matched = []
        for driver in solver.motion_drivers:
            driver_output_ids = {
                ac.acceleration_energy.id
                for ac_spec in driver.acceleration_constraint
                for ac in ac_spec.constraints
            }
            for force_spec in driver.cartesian_force:
                driver_output_ids.add(force_spec.force.id)
                driver_output_ids.update(
                    _upstream_dependencies(force_spec.force.id, closure_input_map)
                )
            if handler_output_ids & driver_output_ids:
                matched.append(driver)
        if not matched:
            continue
        selected = next((driver for driver in matched if driver.id == motion_driver_id), matched[0])
        result.append(MotionArmSolver(
            id=solver.id,
            output=solver.output,
            motion_driver=selected,
            has_cartesian_force=bool(selected.cartesian_force),
            root_acc=solver.root_acc,
        ))

    return result


def _relative_poses_for_motion(evaluators, view_map, arm_solvers):
    """Detect Pose quantities whose wrt frame ends in _start and pair them with FK outputs."""
    fk_poses: dict[str, str] = {}  # of_id → FK pose id
    for solver in arm_solvers:
        for out in solver.output:
            if getattr(out, "type", "") == "Pose":
                of_id = getattr(getattr(out, "of", None), "id", None)
                if of_id:
                    fk_poses[of_id] = out.id

    start_rel_poses: dict[str, object] = {}
    for ev in evaluators:
        qty = getattr(getattr(ev, "constraint", None), "quantity", None)
        if qty is None or not getattr(qty, "has_view", False):
            continue
        view = view_map.get(qty.id)
        if view is None:
            continue
        wrt = getattr(getattr(view, "superobject", None), "with_respect_to", None)
        if wrt and getattr(wrt, "id", "").endswith("_start"):
            pose_id = view.superobject.id
            start_rel_poses[pose_id] = view.superobject

    result = []
    for pose_id, pose in start_rel_poses.items():
        of_id = getattr(getattr(pose, "of", None), "id", None)
        fk_pose_id = fk_poses.get(of_id) if of_id else None
        if fk_pose_id:
            result.append(RelativePoseCapture(id=pose_id, fk_pose_id=fk_pose_id))
    return result


def _constraint_reference_value_id(constraint):
    param = getattr(constraint, "parameter", None)
    ref = getattr(param, "reference_value", None) if param else None
    ref_id = getattr(ref, "id", None)
    return ref_id if ref_id else None


def _snapshot_reference_value_ids(evaluators, constraints):
    ref_val_ids = set()
    for ev in evaluators:
        if ev.constraint is None:
            continue
        ref_id = _constraint_reference_value_id(ev.constraint)
        if ref_id:
            ref_val_ids.add(ref_id)
    for constraint in constraints:
        ref_id = _constraint_reference_value_id(constraint)
        if ref_id:
            ref_val_ids.add(ref_id)
    return ref_val_ids


def _snapshots_for_motion(evaluators, constraints, snapshot_source_map, view_map, closure_output_map):
    ref_val_ids = _snapshot_reference_value_ids(evaluators, constraints)
    result = []
    seen = set()
    for target_id in sorted(ref_val_ids):
        if target_id not in snapshot_source_map or target_id in seen:
            continue
        seen.add(target_id)
        source_id = snapshot_source_map[target_id]
        source_closure_id = None if source_id in view_map else closure_output_map.get(source_id)
        result.append(SnapshotCapture(
            target_id=target_id,
            source_id=source_id,
            source_closure_id=source_closure_id,
        ))
    return result


_CLOSURE_OUTPUT_FIELDS = {
    "PoseToAngleAroundAxis": "angle",
    "PoseToLinearDistance": "distance",
    "PoseToDirection": "direction",
    "RotateDirectionDistalToProximalWithPose": "to",
    "ComposePose": "composite",
    "InvertPose": "out",
    "PoseDiffEvaluator": "out",
    "RotateVelocityTwistToProximalWithPose": "to",
    "InvertAngle": "out",
    "AddWrench": "out",
    "RotateWrenchToDistalWithPose": "to",
    "RotateWrenchToProximalWithPose": "to",
    "TransformWrenchToProximal": "to",
    "WrenchFromPositionDirectionAndMagnitude": "wrench",
}


def build_motion_units(g, p, handlers, node_by_id, slv_arm,
                       snapshot_source_map=None, view_map=None,
                       closure_output_map=None, closure_input_map=None):
    snapshot_source_map = snapshot_source_map or {}
    view_map = view_map or {}
    closure_output_map = closure_output_map or {}
    closure_input_map = closure_input_map or {}
    motions = []

    for handler in handlers:
        handler_node = node_by_id[handler.id]
        motion_node = g.value(handler_node, CSTR_HDL["motion"])
        motion = handler.motion

        # Classify constraints by motion phase via RDF traversal
        when_constraint_nodes = set(g[motion_node : MOT["when"]])
        while_constraint_nodes = set(g[motion_node : MOT["while"]])
        until_constraint_nodes = set(g[motion_node : MOT["until"]])

        # Classify evaluators by which phase their constraint belongs to
        when_eval_nodes, while_eval_nodes, until_eval_nodes = [], [], []
        for eval_node in g[handler_node : CSTR_HDL["evaluators"]]:
            cstr_node = g.value(eval_node, CSTR_HDL["constraint"])
            if cstr_node in when_constraint_nodes:
                when_eval_nodes.append(eval_node)
            elif cstr_node in while_constraint_nodes:
                while_eval_nodes.append(eval_node)
            elif cstr_node in until_constraint_nodes:
                until_eval_nodes.append(eval_node)

        # Classify controllers: driven by while-evaluator error outputs.
        # PoseDiffEvaluator exposes its components as normal MAP views over the
        # operator output twist, so collect those view subobjects here.
        while_error_nodes = set()
        while_pose_eval_nodes = []
        for n in while_eval_nodes:
            if GEOM_OP["PoseDiffEvaluator"] in g[n : RDF["type"]]:
                while_pose_eval_nodes.append(n)
                pose_diff_out = g.value(n, GEOM_OP["out"])
                for view_node in g.subjects(MAP["superobject"], pose_diff_out):
                    error_node = g.value(view_node, MAP["subobject"])
                    if error_node is not None:
                        while_error_nodes.add(error_node)
                continue
            error_node = g.value(n, CSTR_HDL["error"])
            if error_node is not None:
                while_error_nodes.add(error_node)
        while_error_nodes.discard(None)
        ctrl_nodes = [
            n for n in g[handler_node : CSTR_HDL["controllers"]]
            if g.value(n, CSTR_HDL["error-signal"]) in while_error_nodes
        ]

        # Classify monitors by the constraint they watch (via cstr-hdl:constraint).
        # Fall back to error-signal bucketing for monitors without a constraint link.
        when_cstr_nodes = {g.value(n, CSTR_HDL["constraint"]) for n in when_eval_nodes}
        while_cstr_nodes = {g.value(n, CSTR_HDL["constraint"]) for n in while_eval_nodes}
        until_cstr_nodes = {g.value(n, CSTR_HDL["constraint"]) for n in until_eval_nodes}
        when_cstr_nodes.discard(None)
        while_cstr_nodes.discard(None)
        until_cstr_nodes.discard(None)

        when_error_nodes = {g.value(n, CSTR_HDL["error"]) for n in when_eval_nodes}
        until_error_nodes = {g.value(n, CSTR_HDL["error"]) for n in until_eval_nodes}
        when_error_nodes.discard(None)
        until_error_nodes.discard(None)

        when_mon_nodes, while_mon_nodes, until_mon_nodes = [], [], []
        for mon_node in g[handler_node : CSTR_HDL["monitors"]]:
            mon_cstr = g.value(mon_node, CSTR_HDL["constraint"])
            if mon_cstr is not None:
                if mon_cstr in when_cstr_nodes:
                    when_mon_nodes.append(mon_node)
                elif mon_cstr in while_cstr_nodes:
                    while_mon_nodes.append(mon_node)
                elif mon_cstr in until_cstr_nodes:
                    until_mon_nodes.append(mon_node)
            else:
                mon_error = g.value(mon_node, CSTR_HDL["error"])
                if mon_error in when_error_nodes:
                    when_mon_nodes.append(mon_node)
                elif mon_error in while_error_nodes:
                    while_mon_nodes.append(mon_node)
                elif mon_error in until_error_nodes:
                    until_mon_nodes.append(mon_node)

        # Validate that classified sets are subsets of what the handler declares.
        # Evaluators and controllers whose constraints/errors are not linked to any
        # motion phase (when/while/until) are silently excluded from all schedules --
        # this matches the original behaviour and occurs in models such as sc1 where
        # some handler-level evaluators exist outside the motion phase graph.
        all_eval_nodes = set(g[handler_node : CSTR_HDL["evaluators"]])
        classified_eval_nodes = set(when_eval_nodes) | set(while_eval_nodes) | set(until_eval_nodes)
        assert classified_eval_nodes <= all_eval_nodes, (
            f"Handler {handler.id}: classified evaluators not a subset of handler evaluators"
        )

        all_ctrl_nodes = set(g[handler_node : CSTR_HDL["controllers"]])
        assert set(ctrl_nodes) <= all_ctrl_nodes, (
            f"Handler {handler.id}: classified controllers not a subset of handler controllers"
        )

        all_mon_nodes = set(g[handler_node : CSTR_HDL["monitors"]])
        classified_mon_nodes = set(when_mon_nodes) | set(while_mon_nodes) | set(until_mon_nodes)
        assert classified_mon_nodes <= all_mon_nodes, (
            f"Handler {handler.id}: classified monitors not a subset of handler monitors"
        )

        # Build schedules from RDF node traversal.
        # when_schedule runs in can_start (a separate C++ function), so it gets its own
        # Parser with a fresh dedup set.
        # while_schedule and until_schedule are complementary slices of a single shared
        # active computation graph (both emitted inside the same control C++ function).
        # They must share one Parser (p_active) so that steps evaluated in both phases
        # are emitted only once and deduplication across the two slices is correct.
        p_when = Parser(g)
        when_schedule = p_when.schedule(when_eval_nodes, ops_generic + ops_cstr_hdl)

        handler_arm_solvers = _arm_solvers_for_handler(handler, slv_arm, closure_input_map)
        cartesian_force_nodes = []
        for solver in handler_arm_solvers:
            driver_node = node_by_id.get(solver.motion_driver.id)
            if driver_node is None:
                continue
            cartesian_force_nodes.extend(g[driver_node : SLV["cartesian-force"]])

        p_active = Parser(g)
        while_schedule = p_active.schedule(
            [n for n in while_eval_nodes if n not in while_pose_eval_nodes]
            + ctrl_nodes,
            ops_generic + ops_cstr_hdl,
        )
        while_schedule.extend(
            p_active.schedule(cartesian_force_nodes, ops_generic + ops_slv + ops_cstr_hdl)
        )
        until_schedule = p_active.schedule(until_eval_nodes, ops_generic + ops_cstr_hdl)

        # Until evaluators have no controllers whose error-signal would drive their
        # backward discovery. Append them explicitly after their dependencies so the
        # template emits the computation calls in the correct order.
        for n in until_eval_nodes:
            eval_id = p.id(n)
            if eval_id not in p_active.sched:
                until_schedule.append(eval_id)
                p_active.sched.add(eval_id)

        # Same for when evaluators: the can_start template inlines them via
        # when_evaluators, but any prerequisite generic ops still need scheduling.
        # Append when evaluators that were not discovered through backward traversal.
        for n in when_eval_nodes:
            eval_id = p.id(n)
            if eval_id not in p_when.sched:
                when_schedule.append(eval_id)
                p_when.sched.add(eval_id)

        # Build Python objects from classified RDF nodes
        when_evaluators = [p.constraint_evaluator(n) for n in when_eval_nodes]
        while_evaluators = [
            p.constraint_evaluator(n)
            for n in while_eval_nodes
            if GEOM_OP["PoseDiffEvaluator"] not in g[n : RDF["type"]]
        ]
        until_evaluators = [p.constraint_evaluator(n) for n in until_eval_nodes]
        controllers = [p.controller(n) for n in ctrl_nodes]
        when_monitors = [p.monitor_entry(n) for n in when_mon_nodes]
        while_monitors = [p.monitor_entry(n) for n in while_mon_nodes]
        until_monitors = [p.monitor_entry(n) for n in until_mon_nodes]

        motions.append(
            GuardedMotionBlock(
                id=handler.motion.id,
                handler=handler.id,
                motion=handler.motion,
                when_evaluators=when_evaluators,
                while_evaluators=while_evaluators,
                until_evaluators=until_evaluators,
                controllers=controllers,
                when_monitors=when_monitors,
                while_monitors=while_monitors,
                until_monitors=until_monitors,
                when_schedule=when_schedule,
                while_schedule=while_schedule,
                until_schedule=until_schedule,
                when_events=[m.event for m in when_monitors if m.event is not None],
                while_events=[m.event for m in while_monitors if m.event is not None],
                until_events=[m.event for m in until_monitors if m.event is not None],
                has_until_condition=bool(until_evaluators),
                arm_solvers=handler_arm_solvers,
                relative_poses=_relative_poses_for_motion(
                    while_evaluators + when_evaluators + until_evaluators,
                    view_map,
                    _arm_solvers_for_handler(handler, slv_arm),
                ),
                snapshots=_snapshots_for_motion(
                    while_evaluators + when_evaluators + until_evaluators,
                    motion.while_ + motion.when + motion.until,
                    snapshot_source_map, view_map, closure_output_map,
                ),
            )
        )

    return sorted(motions, key=lambda motion: next(h.order for h in handlers if h.id == motion.handler))


def _filter_shared_data(data_structures, schedule, closures, view_map=None, fk_output_ids=None):
    referenced: set[str] = set(schedule)
    for c in closures.values():
        if isinstance(c, dict):
            for v in c.values():
                if isinstance(v, str):
                    referenced.add(v)
    if view_map:
        for view in view_map.values():
            so = getattr(view, "superobject", None)
            if so:
                referenced.add(so.id)
    if fk_output_ids:
        referenced.update(fk_output_ids)

    result = []
    for item in data_structures:
        if item.type == "Quantity" and item.has_view:
            continue
        if (item.type == "Quantity"
                and getattr(item, "value", None) is None
                and getattr(getattr(item, "quantity_kind", None), "id", "x") is None
                and item.id not in referenced):
            continue
        if item.type in ("Pose", "VelocityTwist") and item.id not in referenced:
            continue
        result.append(item)
    return result


def generate_ir(manifest_path):
    app_model_path = Path(manifest_path).resolve()

    # Load top-level, application model
    g = rdflib.Dataset(default_union=True)
    g.parse(str(app_model_path), format="json-ld")

    # Load IRI map
    url_map = {}
    for key in g.objects(predicate=APP["iri-map"]):
        node = g.value(key, APP["path"])
        if not isinstance(node, rdflib.Literal):
            continue

        relative_path = node.value
        if Path(relative_path).is_absolute():
            url_map[str(key)] = relative_path
            continue

        if relative_path == "models/":
            manifest_dir_path = app_model_path.parent
            models_subdir_path = app_model_path.parent / "models"
            imports = list(g.objects(predicate=APP["import"]))

            if imports:
                first_import_url = str(imports[0])
                import_path = None
                for base_url in [str(k) for k in g.objects(predicate=APP["iri-map"])]:
                    if first_import_url.startswith(base_url):
                        import_path = first_import_url[len(base_url) :]
                        break

                if import_path and (manifest_dir_path / import_path).exists():
                    url_map[str(key)] = str(manifest_dir_path)
                elif import_path and (models_subdir_path / import_path).exists():
                    url_map[str(key)] = str(models_subdir_path)
                else:
                    url_map[str(key)] = str(models_subdir_path)
            else:
                url_map[str(key)] = str(models_subdir_path)
            continue

        absolute_path = app_model_path.parent / relative_path
        if not absolute_path.exists():
            absolute_path = Path.cwd() / relative_path
        url_map[str(key)] = str(absolute_path)

    install_resolver(IriToFileResolver(url_map))

    # Load/import the referenced models
    for model in list(g.objects(predicate=APP["import"])):
        g.parse(location=model, format="json-ld")

    p = Parser(g)
    node_by_id = {}
    for node in g.subjects():
        try:
            node_by_id[p.id(node)] = node
        except Exception:
            continue

    sched1 = []
    slv_base_vel = []
    sched2 = []
    hdl = []
    sched3 = []
    slv_arm = []
    sched4 = []
    slv_base_frc = []

    # Construct the computational graph that feeds into the mobile base's
    # velocity composition solver.
    for s in g.subjects(RDF.type, SLV["VelocityCompositionSolver"]):
        slv_base_vel.append(p.velocity_composition_solver(s))
        sched1.extend(p.schedule([s], ops_generic + ops_slv))

    # Traverse backward from the "proximal" constraint handler.
    for h in g.subjects(RDF.type, CSTR_HDL["ConstraintHandler"]):
        hdl.append(p.constraint_handler(h))
        start = g[h : CSTR_HDL["evaluators"] | CSTR_HDL["controllers"]]
        sched2.extend(p.schedule(start, ops_generic + ops_cstr_hdl))

    # Traverse backward from the "distal" solver configuration.
    # The "parser" keeps track of the previously visited closures/computations
    # so that they are visited only once.
    for s in g.subjects(RDF.type, SLV["SolverWithInputAndOutput"]):
        solver = p.solver_with_input_and_output(s)
        _mark_acceleration_constraint_frames(solver)
        slv_arm.append(solver)
        start = g[
            s : SLV["motion-drivers"]
            / ((SLV["acceleration-constraint"] / SLV["constraints"]) | (SLV["cartesian-force"]))
        ]
        sched3.extend(p.schedule(start, ops_generic + ops_slv))

    # Construct the computational graph that feeds into the mobile base's force
    # distribution solver.
    for s in g.subjects(RDF.type, SLV["ForceDistributionSolver"]):
        slv_base_frc.append(p.force_distribution_solver(s))
        sched4.extend(p.schedule([s], ops_generic + ops_slv))

    event_idx = 0
    for handler in hdl:
        for monitor in handler.monitors:
            if monitor.monitor_type == "EdgeTriggeredMonitor":
                monitor.event_idx = event_idx
                event_idx += 1

    # Extract all views, data structures and closures
    closures = p.closures(ops_generic + ops_slv + ops_cstr_hdl)
    view_map = p.view()
    data_structures = p.data_structures()

    # Build snapshot lookup maps
    snapshot_source_map: dict[str, str] = {}
    for snap_node in g.subjects(RDF.type, APP["Snapshot"]):
        source_node = g.value(snap_node, APP["snapshot-of"])
        if source_node is not None:
            snapshot_source_map[p.id(snap_node)] = p.id(source_node)

    closure_output_map: dict[str, str] = {}
    closure_input_map: dict[str, set[str]] = {}
    for cid, c in closures.items():
        if not isinstance(c, dict):
            continue
        inputs = {
            v for k, v in c.items()
            if k not in {"id", "type"} and isinstance(v, str)
        }
        out_field = _CLOSURE_OUTPUT_FIELDS.get(c.get("type", ""))
        if out_field:
            out_val = c.get(out_field)
            if isinstance(out_val, str):
                closure_output_map[out_val] = cid
                closure_input_map[out_val] = {v for v in inputs if v != out_val}

    wrench_outputs = [
        item for item in data_structures
        if item.type == "Wrench" and item.id not in closure_output_map
    ]

    # Compose the overall schedule via concatenation
    return {
        "slv_arm": slv_arm,
        "slv_base_vel": slv_base_vel,
        "slv_base_frc": slv_base_frc,
        "cstr_hdl": hdl,
        "motions": build_motion_units(
            g, p, hdl, node_by_id, slv_arm,
            snapshot_source_map=snapshot_source_map,
            view_map=view_map,
            closure_output_map=closure_output_map,
            closure_input_map=closure_input_map,
        ),
        "data": data_structures,
        "closures": closures,
        "shared_schedule": sched1 + sched3 + sched4,
        "schedule": sched1 + sched2 + sched3 + sched4,
        "views": view_map,
        "shared_data": _filter_shared_data(
            data_structures, sched1 + sched2 + sched3 + sched4, closures,
            view_map=view_map,
            fk_output_ids={out.id for s in slv_arm for out in s.output},
        ),
        "wrench_outputs": wrench_outputs,
        "has_arm": bool(slv_arm),
        "has_mobile_base": bool(slv_base_vel or slv_base_frc),
        "arm_solvers": slv_arm,
        "base_velocity_solvers": slv_base_vel,
        "base_force_solvers": slv_base_frc,
    }


def main():
    """Generate intermediate representation (IR) from motion specification models."""
    parser = argparse.ArgumentParser(
        description="Generate intermediate representation (IR) JSON from motion specification models",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s manifest.json --console           # Print IR to console
  %(prog)s manifest.json -o output.json     # Save IR to file
  %(prog)s manifest.json --output -         # Print IR to console (alternative)
        """,
    )

    parser.add_argument("manifest", help="Path to the application manifest JSON file")

    output_group = parser.add_mutually_exclusive_group(required=True)
    output_group.add_argument(
        "-o", "--output", metavar="FILE", help="Output file path (use '-' for stdout)"
    )
    output_group.add_argument(
        "-c", "--console", action="store_true", help="Print output to console"
    )

    args = parser.parse_args()

    # Determine output destination
    if args.console or (args.output and args.output == "-"):
        output_file = None  # stdout
    else:
        output_file = Path(args.output)

    ir = generate_ir(args.manifest)

    # Output IR to file or stdout
    ir_json = json.dumps(ir, cls=JSONEncoder, indent=4)

    if output_file is None:
        # Output to stdout
        print(ir_json)
    else:
        # Output to file
        try:
            output_file.parent.mkdir(parents=True, exist_ok=True)
            with open(output_file, "w") as f:
                f.write(ir_json)
            print(f"IR written to {output_file}", file=sys.stderr)
        except IOError as e:
            print(f"Error writing to {output_file}: {e}", file=sys.stderr)
            sys.exit(1)


if __name__ == "__main__":
    main()
