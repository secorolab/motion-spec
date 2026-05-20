# SPDX-License-Identifier: MPL-2.0
"""Intermediate representation (IR) generator for motion specification models.

This module parses RDF graphs containing motion specification models and generates
a JSON intermediate representation suitable for code generation.
"""

from __future__ import annotations

import sys
import argparse
from dataclasses import dataclass, field, is_dataclass, asdict
from enum import Enum
import collections
import math
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
    ENV,
    QUDT_SCHEMA,
    GEOM_ENT,
    GEOM_REL,
    GEOM_COORD,
    GEOM_OP,
    RBDYN_ENT,
    RBDYN_COORD,
    RBDYN_OP,
    KC_STAT,
    MAP,
    CSTR,
    MJ,
    MOT,
    MOT_EXT,
    TRAJ,
    RT,
    CSTR_HDL,
    SIM,
    SNAP,
    SLV,
    VALUE_ROLE,
)

PACKAGE_ROOT = Path(__file__).resolve().parents[2]


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
    ids = [to_id(e) for e in entry]
    unique = list(dict.fromkeys(ids))  # deduplicate, preserving order
    if len(unique) == 1:
        return unique[0]
    return unique


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
        type_=RBDYN_OP["AddQuantity"],
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
        input=[MAP["superobject"]],
        output=[MAP["subobject"]],
    ),
    Operator(
        type_=GEOM_OP["PoseDiffEvaluator"],
        input=[GEOM_OP["in1"], GEOM_OP["in2"]],
        output=[GEOM_OP["out"]],
    ),
    Operator(
        type_=TRAJ["Lerp"],
        input=[TRAJ["start"], TRAJ["goal"], TRAJ["alpha"]],
        output=[TRAJ["trajectory"]],
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
    Specification(type_=SLV["JointForceSpecification"], input=[SLV["force"]], output=[]),
    Specification(
        type_=SLV["AccelerationConstraint"], input=[SLV["acceleration-energy"]], output=[]
    ),
    Specification(type_=SLV["ForceDistributionSolver"], input=[SLV["force"]], output=[]),
]


class Subspace(str, Enum):
    Position = "Position"
    Rotation = "Rotation"
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


class ControlMode(str, Enum):
    JointTorque = "JointTorque"


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
    is_scene_object: bool = False
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
    reference_value: str | None = None
    roles: list[str] = field(default_factory=list)
    type: str = field(default="Quantity")


@dataclass
class Trajectory:
    id: str
    quantity_kind: QuantityKind
    unit: Unit
    has_view: bool
    value: None = None
    reference_value: str | None = None
    roles: list[str] = field(default_factory=list)
    type: str = field(default="Trajectory")


@dataclass
class JointPosition:
    id: str
    joint_name: str
    type: str = field(default="JointPosition")


@dataclass
class PoseQuantity:
    id: str
    quantity_kind: QuantityKind
    unit: Unit
    has_view: bool
    value: None = None
    roles: list[str] = field(default_factory=list)
    type: str = field(default="Pose")


@dataclass
class SimplicialComplex:
    id: str
    is_scene_object: bool = False
    type: str = field(default="SimplicialComplex")


@dataclass
class SceneObject:
    id: str
    body: str = ""
    is_scene_object: bool = True
    type: str = field(default="SceneObject")


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
    of: SimplicialComplex | Point | Frame | SceneObject
    with_respect_to: SimplicialComplex | Point | Frame | SceneObject
    quantity_kind: QuantityKind
    as_seen_by: Frame
    unit: Unit
    position: list[float] | None
    type: str = field(default="Position")


@dataclass
class Pose:
    id: str
    of: SimplicialComplex | Frame | SceneObject | None
    with_respect_to: SimplicialComplex | Frame | None
    quantity_kind: list[QuantityKind]
    as_seen_by: Frame | None
    unit: list[Unit]
    direction_cosine_x: list[float] | None
    direction_cosine_y: list[float] | None
    direction_cosine_z: list[float] | None
    position: list[float] | None
    euler_axes_sequence: str | None = None
    roles: list[str] = field(default_factory=list)
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
    roles: list[str] = field(default_factory=list)
    type: str = field(default="VelocityTwist")


@dataclass
class AccelerationTwist:
    id: str
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    roles: list[str] = field(default_factory=list)
    type: str = field(default="AccelerationTwist")


@dataclass
class Wrench:
    id: str
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    roles: list[str] = field(default_factory=list)
    type: str = field(default="Wrench")


@dataclass
class View:
    id: str
    superobject: Pose | VelocityTwist | AccelerationTwist | Wrench
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
    until_any: bool = False
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
    stiffness: float = 0.0
    damping: float = 0.0
    type: str = "Controller"


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
class PoseAxisErrorComponent:
    quantity: str
    error: str
    reference: str
    subspace: str
    axis: str
    eval_id: str
    type: str = field(default="PoseAxisErrorComponent")


@dataclass
class PoseAxisErrorGroup:
    id: str
    pose: str
    components: list[PoseAxisErrorComponent]
    superobject_type: str = "Pose"
    has_angular: bool = False
    linear_x: str | None = None
    linear_y: str | None = None
    linear_z: str | None = None
    angular_x: str | None = None
    angular_y: str | None = None
    angular_z: str | None = None
    type: str = field(default="PoseAxisErrorGroup")


@dataclass
class ConstraintHandler:
    id: str
    motion: GuardedMotion
    control_mode: str
    evaluators: list[ConstraintEvaluator]
    controllers: list[Controller]
    monitors: list[MonitorEntry]
    actions: list[GripperAction] = field(default_factory=list)
    order: int = 0
    type: str = field(default="ConstraintHandler")


@dataclass
class SnapshotCapture:
    target_id: str
    source_id: str
    source_closure_id: str | None = None
    type: str = field(default="SnapshotCapture")


@dataclass
class SceneRelativePose:
    id: str
    fk_pose_id: str
    scene_pose_id: str
    base_seen: bool = False
    type: str = field(default="SceneRelativePose")


@dataclass
class RelativePoseCapture:
    id: str
    fk_pose_id: str
    type: str = field(default="RelativePoseCapture")


@dataclass
class GuardedMotionBlock:
    id: str
    handler: str
    control_mode: str
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
    gripper_actions: list[GripperAction]

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
    until_any: bool = False

    # Solver Integration
    arm_solvers: list = field(default_factory=list)

    # Snapshot captures (once-at-start scalar assignments)
    snapshots: list = field(default_factory=list)

    # Relative-from-start pose computations (e.g. pose_start_ee)
    relative_poses: list = field(default_factory=list)

    # Continuous relative pose of FK frame wrt scene object body (e.g. pose_ee_wrt_cube)
    scene_relative_poses: list[SceneRelativePose] = field(default_factory=list)

    # Pose coordinate-view scalar constraints grouped back into one KDL::diff pose error.
    pose_axis_error_groups: list[PoseAxisErrorGroup] = field(default_factory=list)

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
class JointForceSpecification:
    id: str
    force_id: str
    joint_name: str
    type: str = field(default="JointForceSpecification")


@dataclass
class MotionDrivers:
    id: str
    acceleration_constraint: list[AccelerationConstraintSpecification]
    cartesian_force: list[CartesianForceSpecification]
    joint_force: list[JointForceSpecification] = field(default_factory=list)
    has_cartesian_force: bool = False
    type: str = field(default="MotionDrivers")


@dataclass
class MotionArmSolver:
    id: str
    output: list
    motion_driver: MotionDrivers
    control_mode: str
    root_acc: list[float] | None = None
    chain_root: str = ""
    chain_end: str = ""
    type: str = field(default="MotionArmSolver")


@dataclass
class SolverWithInputAndOutput:
    id: str
    motion_drivers: list[MotionDrivers]
    output: list
    control_mode: str = ""
    urdf: str = ""
    chain_root: str = ""
    chain_end: str = ""
    chain_tip: str = ""
    robot_model: str = ""
    tool_body: str = ""
    tcp_site: str = ""
    root_acc: list[float] | None = None
    type: str = field(default="SolverWithInputAndOutput")


@dataclass
class SceneAttachment:
    id: str
    path: str
    attach_to: str
    prefix: str = ""
    pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    euler: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    actuator: str = ""
    open_command: float = 0.0
    closed_command: float = 0.0
    type: str = field(default="SceneAttachment")


@dataclass
class SceneRobot:
    id: str
    path: str
    pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    euler: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    attachments: list[SceneAttachment] = field(default_factory=list)
    type: str = field(default="SceneRobot")


@dataclass
class SceneObjectSpec:
    id: str
    body: str
    path: str = ""
    pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    euler: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    fixed: bool = False
    shape: str = "BOX"
    size: list[float] = field(default_factory=lambda: [0.03, 0.03, 0.03])
    mass: float = 0.1
    friction: list[float] = field(default_factory=lambda: [0.5, 0.005, 0.0001])
    type: str = field(default="SceneObjectSpec")


@dataclass
class GripperAction:
    id: str
    attachment_id: str
    command: str
    actuator: str = ""
    value: float = 0.0
    type: str = field(default="GripperAction")


@dataclass
class SceneSpec:
    robots: list[SceneRobot] = field(default_factory=list)
    objects: list[SceneObjectSpec] = field(default_factory=list)
    type: str = field(default="SceneSpec")


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
        except Exception:
            return x

    def label(self, x):
        try:
            q = self.g.compute_qname(x)
            return q[2]
        except Exception:
            return str(x)

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
            (KC_STAT["JointPositionCoordinate"], self.joint_position),
        ]

        drv = []
        for motion_driver in self.g[id_ : SLV["motion-drivers"]]:
            drv.append(self.motion_drivers(motion_driver))
        out = []
        for o in self.g[id_ : SLV["output"]]:
            for type_, func in io_dispatcher:
                if type_ in self.g[o : RDF["type"]]:
                    out.append(func(o))

        gravity_node = self.g.value(id_, SLV["gravity-value"])
        gravity = self.parse_xyz(gravity_node) if gravity_node else None
        root_acc = list(gravity) if gravity else None

        return SolverWithInputAndOutput(
            id=self.id(id_),
            motion_drivers=drv,
            output=out,
            root_acc=root_acc,
        )

    @memoize
    def motion_drivers(self, id_):
        assert SLV["MotionDrivers"] in self.g[id_ : RDF["type"]]

        spec_acc = []
        spec_frc = []
        spec_jf = []

        for a in self.g[id_ : SLV["acceleration-constraint"]]:
            spec_acc.append(self.acceleration_constraint_specification(a))

        for f in self.g[id_ : SLV["cartesian-force"]]:
            spec_frc.append(self.cartesian_force_specification(f))

        for jf in self.g[id_ : SLV["joint-force"]]:
            spec_jf.append(self.joint_force_specification(jf))

        return MotionDrivers(
            self.id(id_),
            spec_acc,
            spec_frc,
            spec_jf,
            has_cartesian_force=bool(spec_frc),
        )

    def joint_force_specification(self, id_):
        assert SLV["JointForceSpecification"] in self.g[id_ : RDF["type"]]
        force_node = self.g.value(id_, SLV["force"])
        force_id = self.id(force_node) if force_node is not None else ""
        joint_node = self.g.value(id_, SLV["attached-to"])
        joint_name = self.label(joint_node) if joint_node is not None else ""
        return JointForceSpecification(self.id(id_), force_id, joint_name)

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
            MAP["rotation"]: Subspace.Rotation,
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
        control_mode_node = self.g.value(id_, CSTR_HDL["control-mode"])
        if control_mode_node is None:
            raise ValueError(f"Constraint handler '{self.id(id_)}' is missing control-mode.")
        control_mode = self.id(control_mode_node)
        try:
            ControlMode(control_mode)
        except ValueError as exc:
            raise ValueError(
                f"Constraint handler '{self.id(id_)}' uses unsupported control mode "
                f"'{control_mode}'."
            ) from exc

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

        actions = []
        for action in self.g[id_ : CSTR_HDL["actions"]]:
            actions.append(self.gripper_action(action))

        order_value = self.g.value(id_, APP["order"])
        order = int(order_value.value) if order_value is not None else 0

        return ConstraintHandler(
            self.id(id_), motion, control_mode, evaluators, controllers, monitors, actions, order
        )

    @memoize
    def gripper_action(self, id_):
        attachment = self.g.value(id_, CSTR_HDL["target-attachment"])
        command = str(self.g.value(id_, CSTR_HDL["command"]) or "")
        return GripperAction(self.id(id_), self.id(attachment), command)

    @memoize
    def monitor_entry(self, id_):
        assert CSTR_HDL["Monitor"] in self.g[id_ : RDF["type"]]

        error = self.quantity(self.g.value(id_, CSTR_HDL["error"]))

        if CSTR_HDL["LevelTriggeredMonitor"] in self.g[id_ : RDF["type"]]:
            flag = self.id(self.g.value(id_, CSTR_HDL["flag"]))
            return MonitorEntry(
                self.id(id_), "LevelTriggeredMonitor", error, flag, None, None, False
            )

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

        is_pid = CSTR_HDL["ProportionalIntegralDerivative"] in self.g[id_ : RDF["type"]]
        is_impedance = CSTR_HDL["ImpedanceController"] in self.g[id_ : RDF["type"]]
        assert is_pid or is_impedance, (
            f"Controller {id_} must be ProportionalIntegralDerivative or ImpedanceController"
        )

        error_signal = self.quantity(self.g.value(id_, CSTR_HDL["error-signal"]))
        control_signal = self.quantity(self.g.value(id_, CSTR_HDL["control-signal"]))

        if is_pid:
            decay_rate = None
            if CSTR_HDL["DecayingIntegralTerm"] in self.g[id_ : RDF["type"]]:
                decay_rate = self.g.value(id_, CSTR_HDL["decay-rate"]).value
            p = self._optional_float(id_, CSTR_HDL["proportional-gain"], 0.0)
            i = self._optional_float(id_, CSTR_HDL["integral-gain"], 0.0)
            d = self._optional_float(id_, CSTR_HDL["derivative-gain"], 0.0)
            return Controller(
                self.id(id_),
                error_signal,
                control_signal,
                p,
                i,
                d,
                decay_rate,
                type=self.id(CSTR_HDL.ProportionalIntegralDerivative),
            )
        else:
            ks = self._optional_float(id_, CSTR_HDL["stiffness"], 0.0)
            kd = self._optional_float(id_, CSTR_HDL["damping"], 0.0)
            return Controller(
                self.id(id_),
                error_signal,
                control_signal,
                0.0,
                0.0,
                0.0,
                None,
                stiffness=ks,
                damping=kd,
                type=self.id(CSTR_HDL.ImpedanceController),
            )

    def _optional_float(self, subject, predicate, default: float) -> float:
        value = self.g.value(subject, predicate)
        return default if value is None else float(value.value)

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
        until_any = False
        for c in self.g[id_ : MOT["until"]]:
            if MOT_EXT.ConstraintDisjunction in self.g[c : RDF["type"]]:
                until_any = True
                for member in self.g[c : MOT_EXT["has-constraint"]]:
                    until.append(self.constraint(member))
            else:
                until.append(self.constraint(c))

        return GuardedMotion(self.id(id_), when, while_, until, until_any)

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

        of = self.position_reference(self.g.value(id_, GEOM_REL["of"]))
        wrt = self.position_reference(self.g.value(id_, GEOM_REL["with-respect-to"]))
        quantity_kind = self.quantity_kind(self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = self.unit(self.g.value(id_, QUDT_SCHEMA["unit"]))
        pos = self.parse_xyz(id_)

        return Position(
            self.id(id_), of, wrt, QuantityKind(quantity_kind), as_seen_by, Unit(unit), pos
        )

    def position_reference(self, id_):
        if ENV.RigidObject in self.g[id_ : RDF["type"]]:
            return self.scene_object(id_)
        if GEOM_ENT.Frame in self.g[id_ : RDF["type"]]:
            return self.frame(id_)
        if GEOM_ENT.SimplicialComplex in self.g[id_ : RDF["type"]]:
            return self.simplicial_complex(id_)
        if GEOM_ENT.Point in self.g[id_ : RDF["type"]]:
            return self.point(id_)
        raise ValueError(f"Unsupported position reference node: {id_}")

    @memoize
    def pose(self, id_):
        assert GEOM_COORD["PoseCoordinate"] in self.g[id_ : RDF["type"]]
        assert GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]]

        of_node = self.g.value(id_, GEOM_REL["of"])
        if of_node is None:
            of = None
        elif ENV.RigidObject in self.g[of_node : RDF["type"]] and GEOM_ENT.Frame not in self.g[
            of_node : RDF["type"]
        ]:
            of = self.scene_object(of_node)
        else:
            of = self.frame(of_node)
        wrt_node = self.g.value(id_, GEOM_REL["with-respect-to"])
        wrt = self.frame(wrt_node) if wrt_node is not None else None
        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.quantity_kind(k))
        as_seen_by_node = self.g.value(id_, GEOM_COORD["as-seen-by"])
        as_seen_by = self.frame(as_seen_by_node) if as_seen_by_node is not None else None
        unit = []
        for u in self.g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.unit(u))
        dc_x = self.parse_vector3(self.g.value(id_, GEOM_COORD["direction-cosine-x"]))
        dc_y = self.parse_vector3(self.g.value(id_, GEOM_COORD["direction-cosine-y"]))
        dc_z = self.parse_vector3(self.g.value(id_, GEOM_COORD["direction-cosine-z"]))
        pos = self.parse_xyz(id_)
        euler_axes_sequence = None
        for coord in self.g.objects(id_, GEOM_COORD["has-coordinate"]):
            if GEOM_COORD["EulerAngles"] in self.g[coord : RDF["type"]]:
                axes = self.g.value(coord, GEOM_COORD["axes-sequence"])
                euler_axes_sequence = str(axes) if axes is not None else None
                break

        return Pose(
            self.id(id_),
            of,
            wrt,
            quantity_kind,
            as_seen_by,
            unit,
            dc_x,
            dc_y,
            dc_z,
            pos,
            euler_axes_sequence,
            self.roles(id_),
        )

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
            self.id(id_), of, wrt, quantity_kind, reference_point, as_seen_by, unit, self.roles(id_)
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

        return AccelerationTwist(self.id(id_), quantity_kind, reference_point, as_seen_by, unit, self.roles(id_))

    @memoize
    def wrench(self, id_):
        assert RBDYN_COORD["WrenchCoordinate"] in self.g[id_ : RDF["type"]]
        # assert(RBDYN_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]])

        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.quantity_kind(k))
        reference_point = self.point(self.g.value(id_, RBDYN_ENT["reference-point"]))
        as_seen_by = self.frame(self.g.value(id_, RBDYN_COORD["as-seen-by"]))
        unit = []
        for u in self.g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.unit(u))

        return Wrench(self.id(id_), quantity_kind, reference_point, as_seen_by, unit, self.roles(id_))

    @memoize
    def quantity(self, id_):
        assert QUDT_SCHEMA["Quantity"] in self.g[id_ : RDF["type"]]

        quantity_kind_node = self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]) or self.g.value(
            id_, QUDT_SCHEMA["quantity-kind"]
        )
        quantity_kind = self.quantity_kind(quantity_kind_node)

        unit = self.unit(self.g.value(id_, QUDT_SCHEMA["unit"]))
        has_view = (id_, ~MAP["subobject"], None) in self.g
        reference_node = self.g.value(id_, CSTR["reference-value"])
        reference_value = self.id(reference_node) if reference_node is not None else None

        if GEOM_REL["Pose"] in self.g[id_ : RDF["type"]]:
            return PoseQuantity(self.id(id_), QuantityKind(quantity_kind), Unit(unit), has_view, roles=self.roles(id_))
        if TRAJ["Trajectory"] in self.g[id_ : RDF["type"]]:
            return Trajectory(
                self.id(id_),
                QuantityKind(quantity_kind),
                Unit(unit),
                has_view,
                reference_value=reference_value,
                roles=self.roles(id_),
            )

        value = None
        if (id_, QUDT_SCHEMA["value"], None) in self.g:
            value = float(self.g.value(id_, QUDT_SCHEMA["value"]))
        return Quantity(
            self.id(id_),
            QuantityKind(quantity_kind),
            Unit(unit),
            value,
            has_view,
            reference_value,
            self.roles(id_),
        )

    @memoize
    def joint_position(self, id_):
        assert KC_STAT["JointPositionCoordinate"] in self.g[id_ : RDF["type"]]
        joint_node = self.g.value(id_, GEOM_REL["of"])
        joint_name = self.label(joint_node) if joint_node is not None else ""
        return JointPosition(self.id(id_), joint_name)

    def roles(self, id_):
        role_ns = str(VALUE_ROLE._NS)
        return sorted(
            self.id(t)
            for t in self.g[id_ : RDF["type"]]
            if str(t).startswith(role_ns)
        )

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
    def scene_object(self, id_):
        assert ENV.RigidObject in self.g[id_ : RDF["type"]]
        body = str(self.g.value(id_, MJ["body-name"]) or self.id(id_))
        return SceneObject(self.id(id_), body)

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

        return _dedupe_by_id(data_structures)

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
        scheduled_nodes = {}
        for v in start:
            for op in ops:
                if op.type_ not in self.g[v : RDF["type"]]:
                    continue

                call = self.id(v)
                if op.is_schedulable() and call not in set(self.sched):
                    sched.append(call)
                    scheduled_nodes[call] = v
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

                for call_node in res["schedule"]:
                    call = self.id(call_node)
                    if call and call not in set(self.sched):
                        sched.append(call)
                        scheduled_nodes[call] = call_node
                        self.sched.add(call)

                for data_in in res["data_structures"]:
                    # We have already visited this data structure,
                    # so skip it
                    if data_in in data_structures:
                        continue

                    q.append(data_in)
                    data_structures.add(data_in)

        sched.reverse()
        return self._topological_schedule(sched, scheduled_nodes, ops)

    def _operator_outputs(self, node, op):
        outputs = set()
        for out in getattr(op, "output", []):
            outputs.update(self.g.objects(node, out))
        return outputs

    def _topological_schedule(self, sched, scheduled_nodes, ops):
        """Order scheduled calls so producers run before their consumers."""
        if len(sched) < 2:
            return sched

        order = {call: index for index, call in enumerate(sched)}
        call_inputs: dict[str, set] = {}
        output_producer = {}

        for call in sched:
            node = scheduled_nodes.get(call)
            if node is None:
                continue
            inputs = set()
            outputs = set()
            for op in ops:
                if op.type_ not in self.g[node : RDF["type"]]:
                    continue
                inputs.update(op.from_operator_to_input(self.g, node))
                outputs.update(self._operator_outputs(node, op))
            call_inputs[call] = inputs
            for output in outputs:
                output_producer.setdefault(output, call)

        deps = {
            call: {
                output_producer[data]
                for data in inputs
                if output_producer.get(data) is not None and output_producer[data] != call
            }
            for call, inputs in call_inputs.items()
        }

        result = []
        temporary = set()
        permanent = set()

        def visit(call):
            if call in permanent:
                return
            if call in temporary:
                return
            temporary.add(call)
            for dep in sorted(deps.get(call, ()), key=lambda item: order.get(item, 0)):
                visit(dep)
            temporary.remove(call)
            permanent.add(call)
            result.append(call)

        for call in sched:
            visit(call)
        return result

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
            return name[len(prefix) :]
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
            for jf_spec in driver.joint_force:
                driver_output_ids.add(jf_spec.force_id)
            if handler_output_ids & driver_output_ids:
                matched.append(driver)
        if not matched:
            continue
        selected = next((driver for driver in matched if driver.id == motion_driver_id), matched[0])
        result.append(
            MotionArmSolver(
                id=solver.id,
                output=solver.output,
                motion_driver=selected,
                control_mode=handler.control_mode,
                root_acc=solver.root_acc,
                chain_root=solver.chain_root,
                chain_end=solver.chain_end,
            )
        )

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
    ref_id = ref if isinstance(ref, str) else getattr(ref, "id", None)
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


def _snapshots_for_motion(
    evaluators,
    constraints,
    snapshot_source_map,
    view_map,
    closure_output_map,
    data_reference_map=None,
    schedule=None,
    closures=None,
):
    ref_val_ids = _snapshot_reference_value_ids(evaluators, constraints)
    closures = closures or {}
    data_reference_map = data_reference_map or {}

    def object_id(value):
        if isinstance(value, dict):
            return value.get("id")
        return getattr(value, "id", None)

    def object_field(value, field):
        if isinstance(value, dict):
            return value.get(field)
        return getattr(value, field, None)

    def add_reference(ref_id):
        if not isinstance(ref_id, str):
            return
        pending = [ref_id]
        expanded = set()
        while pending:
            current = pending.pop()
            if current in expanded:
                continue
            expanded.add(current)
            ref_val_ids.add(current)
            referenced = data_reference_map.get(current)
            if referenced:
                pending.append(referenced)
            for view in view_map.values():
                superobject = object_field(view, "superobject")
                if object_id(superobject) != current:
                    continue
                subobject = object_field(view, "subobject")
                subobject_id = object_id(subobject)
                if subobject_id:
                    pending.append(subobject_id)

    for ref_id in list(ref_val_ids):
        add_reference(ref_id)
    for step in schedule or []:
        closure = closures.get(step) or {}
        for value in closure.values():
            if isinstance(value, str):
                add_reference(value)
            elif isinstance(value, list):
                for item in value:
                    add_reference(item)
    result = []
    seen = set()
    for target_id in sorted(ref_val_ids):
        if target_id not in snapshot_source_map or target_id in seen:
            continue
        seen.add(target_id)
        source_id = snapshot_source_map[target_id]
        source_closure_id = None if source_id in view_map else closure_output_map.get(source_id)
        result.append(
            SnapshotCapture(
                target_id=target_id,
                source_id=source_id,
                source_closure_id=source_closure_id,
            )
        )
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
    "AddQuantity": "out",
    "RotateWrenchToDistalWithPose": "to",
    "RotateWrenchToProximalWithPose": "to",
    "TransformWrenchToProximal": "to",
    "WrenchFromPositionDirectionAndMagnitude": "wrench",
}


def _scene_relative_poses_for_motion(view_map, arm_solvers):
    """For each view whose wrt-frame is a scene object, emit the requested relative pose."""
    fk_pose_by_frame: dict[str, str] = {}
    scene_pose_by_id: dict[str, tuple[str, str | None]] = {}
    for solver in arm_solvers:
        for out in solver.output:
            if getattr(out, "type", "") != "Pose":
                continue
            of = getattr(out, "of", None)
            if of is None:
                continue
            if getattr(of, "is_scene_object", False):
                scene_pose_by_id[of.id] = (
                    out.id,
                    getattr(getattr(out, "with_respect_to", None), "id", None),
                )
                body = getattr(of, "body", None)
                if body:
                    scene_pose_by_id[body] = scene_pose_by_id[of.id]
            else:
                fk_pose_by_frame[of.id] = out.id

    seen: set[str] = set()
    result: list[SceneRelativePose] = []
    for view in view_map.values():
        so = getattr(view, "superobject", None)
        if so is None:
            continue
        pose_id = getattr(so, "id", None)
        if not pose_id or pose_id in seen:
            continue
        of = getattr(so, "of", None)
        wrt = getattr(so, "with_respect_to", None)
        if of is None or wrt is None:
            continue
        if getattr(of, "is_scene_object", False):
            continue
        if not getattr(wrt, "is_scene_object", False):
            continue
        fk_pose_id = fk_pose_by_frame.get(getattr(of, "id", ""))
        scene_pose = scene_pose_by_id.get(getattr(wrt, "id", ""))
        if fk_pose_id and scene_pose:
            scene_pose_id, scene_wrt_id = scene_pose
            as_seen_by_id = getattr(getattr(so, "as_seen_by", None), "id", None)
            seen.add(pose_id)
            result.append(
                SceneRelativePose(
                    id=pose_id,
                    fk_pose_id=fk_pose_id,
                    scene_pose_id=scene_pose_id,
                    base_seen=as_seen_by_id == scene_wrt_id,
                )
            )
    return result


_GROUPABLE_SO_TYPES = {"Pose", "VelocityTwist", "AccelerationTwist", "Wrench"}

_SUBSPACE_TO_GROUP_AXIS: dict[Subspace, tuple[str, bool]] = {
    Subspace.Position:           ("linear",  False),
    Subspace.Rotation:           ("angular", True),
    Subspace.LinearVelocity:     ("linear",  False),
    Subspace.AngularVelocity:    ("angular", True),
    Subspace.LinearAcceleration: ("linear",  False),
    Subspace.AngularAcceleration:("angular", True),
    Subspace.Force:              ("linear",  False),
    Subspace.Torque:             ("angular", True),
}


def _pose_axis_error_groups_for_motion(eval_nodes, p, view_map):
    groups: dict[str, PoseAxisErrorGroup] = {}
    for eval_node in eval_nodes:
        if GEOM_OP["PoseDiffEvaluator"] in p.g[eval_node : RDF["type"]]:
            continue
        if CSTR_HDL["ErrorEvaluator"] not in p.g[eval_node : RDF["type"]]:
            continue

        evaluator = p.constraint_evaluator(eval_node)
        if not isinstance(evaluator.constraint.parameter, EqualityConstraint):
            continue
        if evaluator.error is None:
            continue

        quantity = evaluator.constraint.quantity
        view = view_map.get(quantity.id)
        if view is None:
            continue
        so_type = getattr(view.superobject, "type", None)
        if so_type not in _GROUPABLE_SO_TYPES:
            continue

        mapping = _SUBSPACE_TO_GROUP_AXIS.get(view.subspace)
        if mapping is None:
            qkind = getattr(getattr(quantity, "quantity_kind", None), "id", "")
            quantity_id = getattr(quantity, "id", "")
            if (
                so_type == "Pose"
                and (
                    "Angle" in qkind
                    or "angle" in qkind.lower()
                    or "rotation" in quantity_id
                )
            ):
                mapping = ("angular", True)
            else:
                continue
        subspace, is_angular = mapping

        so_id = view.superobject.id
        group = groups.setdefault(
            so_id,
            PoseAxisErrorGroup(
                id=f"pose_axis_error_{so_id}",
                pose=so_id,
                components=[],
                superobject_type=so_type,
            ),
        )
        if is_angular:
            group.has_angular = True
        group.components.append(
            PoseAxisErrorComponent(
                quantity=evaluator.constraint.quantity.id,
                error=evaluator.error.id,
                reference=evaluator.constraint.parameter.reference_value.id,
                subspace=subspace,
                axis=view.axis.value,
                eval_id=evaluator.id,
            )
        )
        setattr(group, f"{subspace}_{view.axis.value.lower()}", evaluator.constraint.parameter.reference_value.id)

    return [group for group in groups.values() if len(group.components) > 1]


def build_motion_units(
    g,
    p,
    handlers,
    node_by_id,
    slv_arm,
    snapshot_source_map=None,
    view_map=None,
    closure_output_map=None,
    data_reference_map=None,
    closure_input_map=None,
    closures=None,
):
    snapshot_source_map = snapshot_source_map or {}
    view_map = view_map or {}
    closure_output_map = closure_output_map or {}
    data_reference_map = data_reference_map or {}
    closure_input_map = closure_input_map or {}
    motions = []

    for handler in handlers:
        handler_node = node_by_id[handler.id]
        motion_node = g.value(handler_node, CSTR_HDL["motion"])
        motion = handler.motion

        # Classify constraints by motion phase via RDF traversal
        when_constraint_nodes = set(g[motion_node : MOT["when"]])
        while_constraint_nodes = set(g[motion_node : MOT["while"]])
        _raw_until = set(g[motion_node : MOT["until"]])
        until_constraint_nodes = set()
        for node in _raw_until:
            if MOT_EXT.ConstraintDisjunction in g[node : RDF["type"]]:
                until_constraint_nodes.update(g[node : MOT_EXT["has-constraint"]])
            else:
                until_constraint_nodes.add(node)

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
            n
            for n in g[handler_node : CSTR_HDL["controllers"]]
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
        handler_output_ids = {c.control_signal.id for c in handler.controllers}
        cartesian_force_nodes = []
        for solver in handler_arm_solvers:
            driver_node = node_by_id.get(solver.motion_driver.id)
            if driver_node is None:
                continue
            for cf_node in g[driver_node : SLV["cartesian-force"]]:
                cf_force = g.value(cf_node, SLV["force"])
                if cf_force is None:
                    continue
                cf_force_id = p.id(cf_force)
                upstream = {cf_force_id} | _upstream_dependencies(cf_force_id, closure_input_map)
                if upstream & handler_output_ids:
                    cartesian_force_nodes.append(cf_node)

        p_active = Parser(g)
        pose_axis_error_groups = _pose_axis_error_groups_for_motion(
            while_eval_nodes, p_active, view_map
        )
        pose_axis_error_eval_ids = {
            component.eval_id
            for group in pose_axis_error_groups
            for component in group.components
        }
        pose_axis_error_quantity_ids = {
            component.quantity
            for group in pose_axis_error_groups
            for component in group.components
        }
        pose_axis_error_compute_ids = {
            closure_output_map[quantity_id]
            for quantity_id in pose_axis_error_quantity_ids
            if quantity_id in closure_output_map
        }
        grouped_while_eval_nodes = {
            node
            for node in while_eval_nodes
            if p_active.id(node) in pose_axis_error_eval_ids
        }
        while_schedule = p_active.schedule(
            [
                n
                for n in while_eval_nodes
                if n not in while_pose_eval_nodes and n not in grouped_while_eval_nodes
            ]
            + ctrl_nodes,
            ops_generic + ops_cstr_hdl,
        )
        while_schedule = [
            step
            for step in while_schedule
            if step not in pose_axis_error_eval_ids and step not in pose_axis_error_compute_ids
        ]
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
                control_mode=handler.control_mode,
                motion=handler.motion,
                when_evaluators=when_evaluators,
                while_evaluators=while_evaluators,
                until_evaluators=until_evaluators,
                controllers=controllers,
                when_monitors=when_monitors,
                while_monitors=while_monitors,
                until_monitors=until_monitors,
                gripper_actions=handler.actions,
                when_schedule=when_schedule,
                while_schedule=while_schedule,
                until_schedule=until_schedule,
                when_events=[m.event for m in when_monitors if m.event is not None],
                while_events=[m.event for m in while_monitors if m.event is not None],
                until_events=[m.event for m in until_monitors if m.event is not None],
                has_until_condition=bool(until_evaluators),
                until_any=handler.motion.until_any,
                arm_solvers=handler_arm_solvers,
                relative_poses=_relative_poses_for_motion(
                    while_evaluators + when_evaluators + until_evaluators,
                    view_map,
                    _arm_solvers_for_handler(handler, slv_arm),
                ),
                scene_relative_poses=_scene_relative_poses_for_motion(
                    view_map,
                    handler_arm_solvers,
                ),
                pose_axis_error_groups=pose_axis_error_groups,
                snapshots=_snapshots_for_motion(
                    while_evaluators + when_evaluators + until_evaluators,
                    motion.while_ + motion.when + motion.until,
                    snapshot_source_map,
                    view_map,
                    closure_output_map,
                    data_reference_map,
                    while_schedule + when_schedule + until_schedule,
                    closures,
                ),
            )
        )

    return sorted(
        motions, key=lambda motion: next(h.order for h in handlers if h.id == motion.handler)
    )


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
        if item.type == "Quantity" and item.has_view and item.id not in referenced:
            continue
        if (
            item.type == "Quantity"
            and getattr(item, "value", None) is None
            and getattr(getattr(item, "quantity_kind", None), "id", "x") is None
            and item.id not in referenced
        ):
            continue
        if item.type in ("Pose", "VelocityTwist") and item.id not in referenced:
            continue
        result.append(item)
    return _dedupe_by_id(result)


def _dedupe_by_id(items):
    result = []
    seen = set()
    for item in items:
        key = getattr(item, "id", None)
        if key is None:
            result.append(item)
            continue
        if key in seen:
            continue
        seen.add(key)
        result.append(item)
    return result


def _xyz_or_zero(g, node):
    if node is None:
        return [0.0, 0.0, 0.0]
    values = [g.value(node, GEOM_COORD[axis]) for axis in ("x", "y", "z")]
    if any(v is None for v in values):
        return [0.0, 0.0, 0.0]
    return [float(v.value) for v in values]


def _orientation_degrees(g, node):
    result = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    if node is None:
        return [0.0, 0.0, 0.0]
    for coord in g.objects(node, GEOM_COORD["has-coordinate"]):
        axis = str(g.value(coord, GEOM_COORD["angle-axis"]) or "")
        if axis not in result:
            continue
        value_node = g.value(coord, QUDT_SCHEMA.value)
        value = 0.0 if value_node is None else float(value_node.value)
        unit = str(g.value(coord, QUDT_SCHEMA.unit) or "")
        if unit.endswith("RAD"):
            value = math.degrees(value)
        result[axis] = value
    return [result["roll"], result["pitch"], result["yaw"]]


def _position_of(g, obj_node):
    for pos_node in g.subjects(GEOM_REL["of"], obj_node):
        if GEOM_COORD["PositionCoordinate"] in g[pos_node:RDF.type]:
            return _xyz_or_zero(g, pos_node)
    return [0.0, 0.0, 0.0]


def _path_of_model(g, model_node):
    return str(g.value(model_node, SIM.path) or "") if model_node else ""


def _mj_body_name(g, node):
    return str(g.value(node, MJ["body-name"]) or "") if node else ""


def _site_name(g, node):
    return str(g.value(node, MJ["site-name"]) or "") if node else ""


def _prefix_from_tool_body(tool_body):
    if "_" in tool_body:
        return tool_body.rsplit("_", 1)[0] + "_"
    return ""


def _attachment_defaults(path, tool_body):
    pos = [0.0, 0.0, 0.0]
    euler = [0.0, 0.0, 0.0]
    prefix = _prefix_from_tool_body(tool_body)
    if "robotiq_2f85" in path:
        pos[2] = -0.061525
        euler[0] = 180.0
        prefix = prefix or "g_"
    return prefix, pos, euler


def _attachments_for_robot(g, env_node, robot_node, tool_body):
    attachments = []
    for candidate in g.objects(env_node, ENV["has-object"]):
        if g.value(candidate, SLV["attached-to"]) != robot_node:
            continue
        path = _path_of_model(g, candidate)
        if not path:
            continue
        attach_to = _mj_body_name(g, g.value(candidate, MJ["attach-to-body"]))
        prefix, pos, euler = _attachment_defaults(path, tool_body)
        prefix = str(g.value(candidate, MJ["attach-prefix"]) or prefix)
        pos = _xyz_or_zero(g, g.value(candidate, MJ["attach-position"])) if g.value(candidate, MJ["attach-position"]) else pos
        euler = _orientation_degrees(g, g.value(candidate, MJ["attach-orientation"])) if g.value(candidate, MJ["attach-orientation"]) else euler
        actuator = str(g.value(candidate, MJ["actuator-name"]) or "")
        open_command = g.value(candidate, MJ["open-command"])
        closed_command = g.value(candidate, MJ["closed-command"])
        attachments.append(
            SceneAttachment(
                id=_id_from_uri(candidate),
                path=path,
                attach_to=attach_to,
                prefix=prefix,
                pos=pos,
                euler=euler,
                actuator=actuator,
                open_command=0.0 if open_command is None else float(open_command.value),
                closed_command=0.0 if closed_command is None else float(closed_command.value),
            )
        )
    return attachments


def _scene_from_graph(g):
    scene = SceneSpec()
    for env_node in g.subjects(RDF.type, ENV.Workspace):
        robot_nodes = []
        for obj_node in g.objects(env_node, ENV["has-object"]):
            if g.value(obj_node, GEOM_ENT["kinematic-chain"]) is not None:
                robot_nodes.append(obj_node)

        for robot_node in robot_nodes:
            model_node = g.value(robot_node, ENV["has-object-model"])
            robot_path = _path_of_model(g, model_node)
            if not robot_path:
                continue

            tool_body = _mj_body_name(g, g.value(robot_node, MJ["tool-body"]))
            attachments = _attachments_for_robot(g, env_node, robot_node, tool_body)

            scene.robots.append(
                SceneRobot(
                    id=_id_from_uri(robot_node),
                    path=robot_path,
                    pos=_position_of(g, robot_node),
                    attachments=attachments,
                )
            )

        for obj_node in g.objects(env_node, ENV["has-object"]):
            if obj_node in robot_nodes:
                continue
            if ENV.RigidObject not in g[obj_node:RDF.type]:
                continue
            model_node = g.value(obj_node, ENV["has-object-model"])
            path = _path_of_model(g, model_node)
            body = _mj_body_name(g, obj_node) or _id_from_uri(obj_node)
            shape = str(g.value(obj_node, MJ["shape"]) or "box").upper()
            size = _xyz_or_zero(g, g.value(obj_node, MJ["size"])) if g.value(obj_node, MJ["size"]) else [0.03, 0.03, 0.03]
            mass_node = g.value(obj_node, MJ["mass"])
            friction = [
                float((g.value(obj_node, MJ["friction-slide"]) or rdflib.Literal(0.5)).value),
                float((g.value(obj_node, MJ["friction-torsion"]) or rdflib.Literal(0.005)).value),
                float((g.value(obj_node, MJ["friction-roll"]) or rdflib.Literal(0.0001)).value),
            ]
            scene.objects.append(
                SceneObjectSpec(
                    id=_id_from_uri(obj_node),
                    body=body,
                    path=path,
                    pos=_position_of(g, obj_node),
                    fixed=bool(path),
                    shape=shape,
                    size=size,
                    mass=0.1 if mass_node is None else float(mass_node.value),
                    friction=friction,
                )
            )
        break
    return scene


def _id_from_uri(node):
    return str(node).rstrip("/").split("/")[-1].split("#")[-1].replace("-", "_")


def _resolve_gripper_actions(handlers, scene):
    attachments = {
        attachment.id: attachment
        for robot in scene.robots
        for attachment in robot.attachments
    }

    for handler in handlers:
        for action in handler.actions:
            attachment = attachments.get(action.attachment_id)
            if attachment is None:
                continue
            action.actuator = attachment.actuator
            if action.command == "close":
                action.value = attachment.closed_command
            elif action.command == "open":
                action.value = attachment.open_command


def _robot_setup_from_graph(g):
    for env_node in g.subjects(RDF.type, ENV.Workspace):
        for obj_node in g.objects(env_node, ENV["has-object"]):
            chain = g.value(obj_node, GEOM_ENT["kinematic-chain"])
            if chain is None:
                continue
            start = g.value(chain, GEOM_ENT.start)
            end = g.value(chain, GEOM_ENT.end)
            chain_root = str(g.value(start, MJ["body-name"]) or "")
            chain_end = str(g.value(end, MJ["body-name"]) or "")
            model_node = g.value(obj_node, ENV["has-object-model"])
            urdf = str(g.value(model_node, SIM.path) or "") if model_node else ""
            robot_model = str(model_node).rstrip("/").split("/")[-1].split("#")[-1] if model_node else ""
            tool_body = _mj_body_name(g, g.value(obj_node, MJ["tool-body"]))
            tcp_site = _site_name(g, g.value(obj_node, MJ["tcp-site"]))
            attachments = _attachments_for_robot(g, env_node, obj_node, tool_body)
            chain_tip = chain_end
            if tcp_site and chain_end == tcp_site and attachments:
                chain_tip = attachments[0].attach_to
            if chain_root or chain_end or urdf:
                return urdf, chain_root, chain_end, chain_tip, robot_model, tool_body, tcp_site
    return "", "", "", "", "", "", ""


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
            source_path = PACKAGE_ROOT / relative_path
            absolute_path = source_path if source_path.exists() else Path.cwd() / relative_path
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
    (
        _urdf,
        _chain_root,
        _chain_end,
        _chain_tip,
        _robot_model,
        _tool_body,
        _tcp_site,
    ) = _robot_setup_from_graph(g)
    scene = _scene_from_graph(g)

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

    _resolve_gripper_actions(hdl, scene)

    # Traverse backward from the "distal" solver configuration.
    # The "parser" keeps track of the previously visited closures/computations
    # so that they are visited only once.
    for s in g.subjects(RDF.type, SLV["SolverWithInputAndOutput"]):
        solver = p.solver_with_input_and_output(s)
        solver.urdf = _urdf
        solver.chain_root = _chain_root
        solver.chain_end = _chain_end
        solver.chain_tip = _chain_tip
        solver.robot_model = _robot_model
        solver.tool_body = _tool_body
        solver.tcp_site = _tcp_site
        _mark_acceleration_constraint_frames(solver)
        slv_arm.append(solver)
        start = g[
            s : SLV["motion-drivers"]
            / (
                (SLV["acceleration-constraint"] / SLV["constraints"])
                | (SLV["cartesian-force"])
                | (SLV["joint-force"])
            )
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
    for snap_node in g.subjects(RDF.type, SNAP.Snapshot):
        source_node = g.value(snap_node, SNAP["snapshot-of"])
        if source_node is not None:
            snapshot_source_map[p.id(snap_node)] = p.id(source_node)

    data_reference_map: dict[str, str] = {}
    for item in data_structures:
        ref = getattr(item, "reference_value", None)
        ref_id = ref if isinstance(ref, str) else getattr(ref, "id", None)
        if ref_id:
            data_reference_map[item.id] = ref_id

    for c in closures.values():
        if not isinstance(c, dict) or c.get("type") != "AssignmentEvaluator":
            continue
        quantity_id = c.get("quantity")
        ref_id = c.get("reference_value")
        if isinstance(quantity_id, str) and isinstance(ref_id, str):
            data_reference_map[quantity_id] = ref_id

    closure_output_map: dict[str, str] = {}
    closure_input_map: dict[str, set[str]] = {}
    for cid, c in closures.items():
        if not isinstance(c, dict):
            continue
        inputs = {v for k, v in c.items() if k not in {"id", "type"} and isinstance(v, str)}
        out_field = _CLOSURE_OUTPUT_FIELDS.get(c.get("type", ""))
        if out_field:
            out_val = c.get(out_field)
            if isinstance(out_val, str):
                closure_output_map[out_val] = cid
                closure_input_map[out_val] = {v for v in inputs if v != out_val}

    wrench_outputs = _dedupe_by_id(
        [
            item
            for item in data_structures
            if item.type == "Wrench" and item.id not in closure_output_map
        ]
    )

    motions = build_motion_units(
        g,
        p,
        hdl,
        node_by_id,
        slv_arm,
        snapshot_source_map=snapshot_source_map,
        view_map=view_map,
        closure_output_map=closure_output_map,
        data_reference_map=data_reference_map,
        closure_input_map=closure_input_map,
        closures=closures,
    )
    for solver in slv_arm:
        control_modes = {
            arm_solver.control_mode
            for motion in motions
            for arm_solver in motion.arm_solvers
            if arm_solver.id == solver.id and arm_solver.control_mode
        }
        if len(control_modes) == 1:
            solver.control_mode = next(iter(control_modes))
        elif len(control_modes) > 1:
            raise ValueError(
                f"Solver '{solver.id}' is used with multiple control modes: "
                f"{', '.join(sorted(control_modes))}."
            )
        else:
            raise ValueError(f"Solver '{solver.id}' is not associated with a control mode.")

    # Determine backend from runtime declaration in the environment spec
    _RUNTIME_TO_BACKEND = {
        str(RT.MuJoCoRuntime): "mj_kdl",
        str(RT.RealRobotRuntime): "robif2b",
    }
    backend = "robif2b"
    for runtime_iri in g.objects(predicate=RT["uses-runtime"]):
        mapped = _RUNTIME_TO_BACKEND.get(str(runtime_iri))
        if mapped:
            backend = mapped
            break

    # Compose the overall schedule via concatenation
    return {
        "slv_arm": slv_arm,
        "slv_base_vel": slv_base_vel,
        "slv_base_frc": slv_base_frc,
        "cstr_hdl": hdl,
        "motions": motions,
        "data": data_structures,
        "closures": closures,
        "shared_schedule": sched1 + sched3 + sched4,
        "schedule": sched1 + sched2 + sched3 + sched4,
        "views": view_map,
        "shared_data": _filter_shared_data(
            data_structures,
            sched1 + sched2 + sched3 + sched4,
            closures,
            view_map=view_map,
            fk_output_ids={out.id for s in slv_arm for out in s.output},
        ),
        "wrench_outputs": wrench_outputs,
        "has_arm": bool(slv_arm),
        "has_mobile_base": bool(slv_base_vel or slv_base_frc),
        "arm_solvers": slv_arm,
        "base_velocity_solvers": slv_base_vel,
        "base_force_solvers": slv_base_frc,
        "backend": backend,
        "scene": scene,
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
