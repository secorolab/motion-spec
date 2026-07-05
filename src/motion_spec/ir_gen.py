# SPDX-License-Identifier: MPL-2.0
"""Intermediate representation (IR) generator for motion specification models.

This module parses RDF graphs containing motion specification models and generates
a JSON intermediate representation suitable for code generation.
"""

from __future__ import annotations

import sys
import argparse
from dataclasses import dataclass, field, is_dataclass, asdict, replace
from enum import Enum
import collections
import itertools
import math
import os
import re
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import rdflib
from rdf_utils.resolver import IriToFileResolver, install_resolver
from functools import wraps
from rdflib.namespace import RDF
from rdflib import URIRef
from motion_spec.namespace import (
    APP,
    ENV,
    QUDT_SCHEMA,
    QUDT_QKIND,
    QUDT_UNIT,
    GEOM_ENT,
    GEOM_REL,
    GEOM_COORD,
    GEOM_COORD_EXT,
    GEOM_OP,
    GEOM_OP_EXT,
    RBDYN_ENT,
    RBDYN_COORD,
    RBDYN_OP,
    RBDYN_OP_EXT,
    KC_STAT,
    MAP,
    MAP_EXT,
    CSTR,
    CSTR_EXT,
    MJ,
    MOT,
    POLY,
    TRAJ,
    RT,
    CSTR_HDL,
    CSTR_HDL_EXT,
    EXEC,
    SNAP,
    SLV,
    SLV_EXT,
)
from motion_spec.manifest import build_url_map, metamodel_url_map


class JSONEncoder(json.JSONEncoder):
    def default(self, o):
        if is_dataclass(o) and not isinstance(o, type):
            return asdict(o)
        return super().default(o)


def parse_argument(g, closure_id, argument, to_id, resolve_value=False):
    # For each of the key differentiate if there is one or more associated value
    entry = list(g[closure_id:argument])
    if len(entry) == 0:
        return None

    def resolve(e):
        # Parameters are baked into generated code as literal text, so a qudt:Quantity-wrapped
        # constant must resolve to its scalar qudt:value here (bare literals / IRI refs pass through).
        if resolve_value and not isinstance(e, rdflib.Literal):
            qval = g.value(e, QUDT_SCHEMA["value"])
            if qval is not None:
                return qval
        return to_id(e)

    ids = [resolve(e) for e in entry]
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
            closure[to_id(param)] = parse_argument(g, closure_id, param, to_id, resolve_value=True)

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
            Operator(
                type_=CSTR_EXT["OutsideConstraint"],
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
                closure[to_id(param)] = parse_argument(g, closure_id, param, to_id, resolve_value=True)

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
            closure[to_id(param)] = parse_argument(g, closure_id, param, to_id, resolve_value=True)

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
        type_=RBDYN_OP_EXT["AddQuantity"],
        input=[RBDYN_OP["in1"], RBDYN_OP["in2"]],
        output=[RBDYN_OP["out"]],
    ),
    Operator(
        type_=RBDYN_OP_EXT["Norm"],
        input=[RBDYN_OP["in1"]],
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
        type_=GEOM_OP_EXT["PoseDiffEvaluator"],
        input=[GEOM_OP["in1"], GEOM_OP["in2"]],
        output=[GEOM_OP_EXT["out"]],
    ),
    Operator(
        type_=TRAJ["Lerp"],
        input=[TRAJ["start"], TRAJ["goal"], TRAJ["alpha"]],
        output=[TRAJ["trajectory"]],
        parameters=[TRAJ["profile"]],
    ),
    Operator(
        type_=CSTR_HDL_EXT["VelocityProfile"],
        input=[
            CSTR_HDL_EXT["goal"],
            CSTR_HDL_EXT["measured"],
            TRAJ["measured-velocity"],
            TRAJ["max-velocity"],
            TRAJ["max-acceleration"],
            TRAJ["max-jerk"],
        ],
        output=[CSTR_HDL_EXT["reference"]],
        parameters=[TRAJ["shape"], CSTR_HDL_EXT["controller"]],
    ),
    Operator(
        type_=CSTR_HDL_EXT["Admittance"],
        input=[CSTR_HDL_EXT["force"]],
        output=[CSTR_HDL_EXT["reference"]],
        parameters=[
            CSTR_HDL_EXT["mass"],
            CSTR_HDL_EXT["damping"],
            CSTR_HDL_EXT["stiffness"],
            CSTR_HDL_EXT["max-velocity"],
            CSTR_HDL_EXT["controller"],
        ],
    ),
    Operator(
        type_=TRAJ["Circle"],
        input=[TRAJ["start"], TRAJ["center"], TRAJ["plane-normal"],
               TRAJ["alpha"]],
        output=[TRAJ["trajectory"]],
    ),
    Operator(
        type_=TRAJ["Arc"],
        input=[TRAJ["start"], TRAJ["end"], TRAJ["amplitude"], TRAJ["plane-normal"],
               TRAJ["alpha"]],
        output=[TRAJ["trajectory"]],
    ),
    Operator(
        type_=TRAJ["Helix"],
        input=[TRAJ["start"], TRAJ["center"], TRAJ["axis"], TRAJ["pitch"],
               TRAJ["revolutions"], TRAJ["alpha"]],
        output=[TRAJ["trajectory"]],
    ),
    Operator(
        type_=TRAJ["Figure8"],
        input=[TRAJ["anchor"], TRAJ["radius"], TRAJ["plane-normal"],
               TRAJ["alpha"]],
        output=[TRAJ["trajectory"]],
        parameters=[TRAJ["form"]],
    ),
]

ops_cstr_hdl = [
    Operator(
        type_=CSTR_HDL["Controller"],
        input=[
            CSTR_HDL["error-signal"],
            CSTR_HDL_EXT["reference-signal"],
            CSTR_HDL_EXT["measured-derivative"],
        ],
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
        type_=SLV["AccelerationConstraint"],
        # `slv-ext:direction` is only present on DirectionAligned constraints; absent
        # on AxisAligned ones, where g.objects() simply yields nothing for it.
        input=[SLV["acceleration-energy"], SLV_EXT["direction"]],
        output=[],
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
    LinearDifference = "LinearDifference"
    AngularDifference = "AngularDifference"


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
    value: float | None
    has_view: bool
    authored: bool = False
    snapshot: bool = False
    reference_value: str | None = None
    type: str = field(default="Quantity")


@dataclass
class Saturation:
    id: str
    input_signal: Quantity
    output_signal: Quantity
    maximum: Quantity | None = None
    lower: Quantity | None = None
    upper: Quantity | None = None
    type: str = field(default="Saturation")


@dataclass
class FreeVector:
    id: str
    quantity_kind: QuantityKind
    unit: Unit
    vector: list[float] | None
    has_view: bool = False
    authored: bool = False
    snapshot: bool = False
    type: str = field(default="FreeVector")


@dataclass
class Trajectory:
    id: str
    quantity_kind: QuantityKind
    unit: Unit
    has_view: bool
    value: None = None
    authored: bool = False
    snapshot: bool = False
    value_kind: str | None = None
    type: str = field(default="Trajectory")


@dataclass
class JointPosition:
    id: str
    joint_name: str
    type: str = field(default="JointPosition")


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
class Orientation:
    id: str
    of: SimplicialComplex | Frame | SceneObject | None
    with_respect_to: SimplicialComplex | Frame | SceneObject | None
    quantity_kind: QuantityKind
    as_seen_by: Frame | None
    unit: Unit
    euler_axes_sequence: str | None = None
    has_view: bool = False
    authored: bool = False
    snapshot: bool = False
    type: str = field(default="Orientation")


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
    authored: bool = False
    snapshot: bool = False
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
    authored: bool = False
    snapshot: bool = False
    type: str = field(default="VelocityTwist")


@dataclass
class AccelerationTwist:
    id: str
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    authored: bool = False
    snapshot: bool = False
    type: str = field(default="AccelerationTwist")


@dataclass
class PoseDifference:
    id: str
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    authored: bool = False
    snapshot: bool = False
    type: str = field(default="PoseDifference")


@dataclass
class Wrench:
    id: str
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    authored: bool = False
    snapshot: bool = False
    # Non-empty when this wrench is measured from a force/torque sensor (the FT-read
    # solver-output reads and tares this sensor into shared.<id>.force). Empty for
    # computed/commanded wrenches.
    sensor_name: str = ""
    type: str = field(default="Wrench")


@dataclass
class View:
    id: str
    superobject: Pose | VelocityTwist | AccelerationTwist | PoseDifference | Wrench
    subobject: Quantity | Position | Orientation
    subspace: Subspace
    axis: Axis | None
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
class OutsideConstraint:
    lower_threshold: Quantity
    upper_threshold: Quantity
    type: str = field(default="OutsideConstraint")


@dataclass
class Constraint:
    id: str
    quantity: Quantity
    parameter: EqualityConstraint | UnilateralConstraint | BilateralConstraint | OutsideConstraint
    type: str = field(default="Constraint")


@dataclass
class GuardedMotion:
    id: str
    when: list[Constraint]
    while_: list[Constraint]
    until: list[Constraint]
    until_any: bool = False
    when_any: bool = False
    type: str = field(default="GuardedMotion")


@dataclass
class ConstraintEvaluator:
    id: str
    type_: EvaluatorType
    constraint: Constraint
    error: Quantity | None
    is_elapsed: bool = False
    elapsed_op: str | None = None
    elapsed_threshold_s: float | None = None
    type: str = field(default="ConstraintEvaluator")


@dataclass
class Controller:
    id: str
    control_signal: Quantity
    error_signal: Quantity | None = None
    reference_signal: Quantity | None = None
    measured_derivative: Quantity | None = None
    proportional_gain: float | None = None
    integral_gain: float | None = None
    derivative_gain: float | None = None
    decay_rate: float | None = None
    stiffness: float | None = None
    damping: float | None = None
    output_saturation: Saturation | None = None
    integral_saturation: Saturation | None = None
    type: str = "Controller"


@dataclass
class ForwardedCommand:
    id: str
    control_signal: Quantity
    target: str
    type: str = field(default="ForwardedCommand")


@dataclass
class MonitorEntry:
    id: str
    monitor_type: str
    error: Quantity | None
    flag: str | None
    event: str | None
    event_idx: int | None
    is_edge_triggered: bool
    is_until_aggregate: bool = False
    is_when_aggregate: bool = False
    event_uri: str | None = None
    event_name: str | None = None
    fallback_motion: str | None = None
    # Authored debounce duration (s); converted to debounce_steps once the loop period is known.
    # Stays None (not 0) when absent -- ST4's <if(x)> is true even for integer 0.
    debounce_duration_s: float | None = None
    debounce_steps: int | None = None
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
    order: int = 0
    type: str = field(default="ConstraintHandler")


@dataclass
class SnapshotCapture:
    target_id: str
    source_id: str
    source_closure_id: str | None = None
    # snap:sampled-on clock: "task" = sampled once, "entry" = re-sampled per entry.
    clock: str = "task"
    # persistent = shared-guarded so it survives the on-entry reset (see _snapshots_for_motion).
    persistent: bool = False
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
    # Schedules: when_schedule runs in can_start (own Parser). while_/until_schedule are slices of
    # one shared active graph, so they share a Parser -- common steps emit once and dedup correctly.
    when_schedule: list[str]
    while_schedule: list[str]
    until_schedule: list[str]

    # FSM Interaction
    when_events: list[str]
    while_events: list[str]
    until_events: list[str]
    has_elapsed: bool = False
    has_until_condition: bool = False
    until_any: bool = False
    when_any: bool = False

    # Solver Integration
    arm_solvers: list = field(default_factory=list)

    # Snapshot captures (sample-and-hold of a fluent on a clock)
    snapshots: list = field(default_factory=list)

    # True iff any snapshot samples `on entry` -> motion is reset on FSM re-entry.
    has_entry_snapshot: bool = False

    # Relative-from-start pose computations (e.g. pose_start_ee)
    relative_poses: list = field(default_factory=list)

    # Continuous relative pose of FK frame wrt scene object body (e.g. pose_ee_wrt_cube)
    scene_relative_poses: list[SceneRelativePose] = field(default_factory=list)

    # Pose coordinate-view scalar constraints grouped back into one KDL::diff pose error.
    pose_axis_error_groups: list[PoseAxisErrorGroup] = field(default_factory=list)

    # Direct robot command forwarding driven by FeedForward controllers.
    forwarded_commands: list[ForwardedCommand] = field(default_factory=list)

    type: str = field(default="GuardedMotionBlock")


@dataclass
class AccelerationConstraint:
    id: str
    subspace: Subspace
    # Exactly one of axis (AxisAligned) / direction (DirectionAligned) is set.
    axis: Axis | None
    acceleration_energy: Quantity
    as_seen_by: Frame | None = None
    base_aligned: bool = True
    direction: "Direction | None" = None
    saturation: Saturation | None = None
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
    algorithm: str = ""
    algorithm_is_rne: bool = False
    gravity: list[float] | None = None
    root_acc: list[float] | None = None
    chain_root: str = ""
    chain_end: str = ""
    torque_saturation: Saturation | None = None
    type: str = field(default="MotionArmSolver")


@dataclass
class SolverWithInputAndOutput:
    id: str
    motion_drivers: list[MotionDrivers]
    output: list
    algorithm: str = ""
    algorithm_is_rne: bool = False
    control_mode: str = ""
    urdf: str = ""
    chain_root: str = ""
    chain_end: str = ""
    chain_tip: str = ""
    robot_model: str = ""
    tool_body: str = ""
    tcp_site: str = ""
    ft_sensors: list[dict] = field(default_factory=list)
    gravity: list[float] | None = None
    root_acc: list[float] | None = None
    # DLS/Tikhonov regularization lambda, deduped across arm solvers into the
    # top-level IR key consumed by runtime_header.
    regularization: float | None = None
    torque_saturation: Saturation | None = None
    type: str = field(default="SolverWithInputAndOutput")


@dataclass
class SceneAttachment:
    id: str
    path: str
    attach_to: str
    attach_kind: str = "Body"
    prefix: str = ""
    pos: list[float] | None = None
    euler: list[float] | None = None
    actuator: str = ""
    type: str = field(default="SceneAttachment")


@dataclass
class SceneRobot:
    id: str
    path: str
    prefix: str = ""
    attach_kind: str = "World"
    attach_name: str = ""
    pos: list[float] | None = None
    euler: list[float] | None = None
    attachments: list[SceneAttachment] = field(default_factory=list)
    type: str = field(default="SceneRobot")


@dataclass
class SceneObjectSpec:
    id: str
    body: str
    path: str = ""
    attach_kind: str = "World"
    attach_name: str = ""
    pos: list[float] | None = None
    euler: list[float] | None = None
    fixed: bool = False
    shape: str | None = None
    size: list[float] | None = None
    color: list[float] | None = None
    mass: float | None = None
    friction: list[float] | None = None
    type: str = field(default="SceneObjectSpec")




@dataclass
class SceneSpec:
    robots: list[SceneRobot] = field(default_factory=list)
    objects: list[SceneObjectSpec] = field(default_factory=list)
    # Physics/control timestep from ENVIRONMENT.timestep; defaults to the backend
    # interval when the model omits it.
    timestep_s: float = 0.002
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
        # Scope by func identity so e.g. position(uri) and quantity(uri)
        # don't collide on the same (uri,) cache key.
        key = (func.__qualname__,) + args + tuple(kwargs.items())
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
    # A context quantity's id is its URI's last segment, so a name reused across motions' specs
    # (e.g. `support-z`) collapses to one id. Qualify only ambiguous names (same segment, >1 owner).
    _SPEC_OWNER_RE = re.compile(r"/([^/]+)/(?:Spec/spec|World/world)/")

    def __init__(self, g):
        self.cache = dict()
        self.g = g
        self.sched = set()
        self._ambiguous_context_ids = self._compute_ambiguous_context_ids()
        self._id_sources: dict[str, set[str]] = {}

    def _compute_ambiguous_context_ids(self):
        owners_by_id: dict[str, set[str]] = {}
        for s in set(self.g.subjects()):
            m = self._SPEC_OWNER_RE.search(str(s))
            if not m:
                continue
            try:
                local = escape(self.g.compute_qname(s)[2])
            except Exception:
                continue
            owners_by_id.setdefault(local, set()).add(m.group(1))
        return {lid for lid, owners in owners_by_id.items() if len(owners) > 1}

    def id(self, x):
        try:
            q = self.g.compute_qname(x)
            local = escape(q[2])
        except Exception:
            return x
        # Only context quantities (a motion's or the shared context's `Spec/spec` / `World/world`
        # members) become `shared.*` data fields and are vulnerable to the silent merge; constraint
        # names, metamodel predicates and aliases legitimately share an id and are scoped elsewhere.
        m = self._SPEC_OWNER_RE.search(str(x))
        if m:
            if local in self._ambiguous_context_ids:
                local = escape(f"{m.group(1)}-{q[2]}")
            self._id_sources.setdefault(local, set()).add(str(x))
        return local

    def assert_no_id_collisions(self):
        """Fail loudly if two distinct context-quantity URIs collapse to one generated id. That
        would silently merge unrelated `shared.*` fields (the class of bug that hid the support-z
        collision) -- a genuinely-shared quantity has a single URI, so >1 URI per id is a real
        collision the motion-qualified id scoping failed to resolve."""
        collisions = {i: sorted(u) for i, u in self._id_sources.items() if len(u) > 1}
        if collisions:
            details = "\n".join(
                f"  '{i}' <- {', '.join(u)}" for i, u in sorted(collisions.items())
            )
            raise ValueError(
                "id collision(s): distinct URIs map to one generated id and would be silently "
                "merged (e.g. a context-quantity name reused across motions with different "
                "definitions). Give them distinct names, or extend the motion-qualified id "
                f"scoping in Parser.id to cover their URI shape:\n{details}"
            )

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
            (RBDYN_COORD["WrenchCoordinate"], self.wrench),
        ]

        drv = []
        for motion_driver in self.g[id_ : SLV["motion-drivers"]]:
            drv.append(self.motion_drivers(motion_driver))
        out = []
        for o in self.g[id_ : SLV["output"]]:
            for type_, func in io_dispatcher:
                if type_ in self.g[o : RDF["type"]]:
                    out.append(func(o))

        gravity_node = self.g.value(id_, SLV_EXT["gravity-value"])
        gravity = self.parse_xyz(gravity_node) if gravity_node else None
        root_acc = list(gravity) if gravity else None
        algorithm_node = self.g.value(id_, SLV["solver"])
        algorithm = {
            SLV["AccelerationConstrainedHybridDynamicsAlgorithm"]: "ACHD",
            SLV["RecursiveNewtonEulerAlgorithm"]: "RNE",
        }.get(algorithm_node, self.id(algorithm_node) if algorithm_node else "")
        torque_saturation_node = next(
            (
                node
                for node in self.g.objects(id_, CSTR_HDL_EXT["limits"])
                if SLV_EXT["TorqueSaturation"] in self.g[node : RDF["type"]]
            ),
            None,
        )

        return SolverWithInputAndOutput(
            id=self.id(id_),
            motion_drivers=drv,
            output=out,
            algorithm=algorithm,
            algorithm_is_rne=algorithm == "RNE",
            gravity=list(gravity) if gravity else None,
            root_acc=root_acc,
            regularization=self._optional_float(id_, SLV_EXT["regularization"]),
            torque_saturation=(
                self.saturation(torque_saturation_node)
                if torque_saturation_node is not None
                else None
            ),
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
    def saturation(self, id_):
        assert CSTR_HDL_EXT["Saturation"] in self.g[id_ : RDF["type"]]
        input_signal = self.quantity(self.g.value(id_, CSTR_HDL_EXT["input-signal"]))
        output_signal = self.quantity(self.g.value(id_, CSTR_HDL_EXT["output-signal"]))
        maximum_node = self.g.value(id_, CSTR_HDL_EXT["maximum-absolute-value"])
        lower_node = self.g.value(id_, CSTR_HDL_EXT["lower-limit"])
        upper_node = self.g.value(id_, CSTR_HDL_EXT["upper-limit"])
        return Saturation(
            self.id(id_),
            input_signal,
            output_signal,
            self.quantity(maximum_node) if maximum_node is not None else None,
            self.quantity(lower_node) if lower_node is not None else None,
            self.quantity(upper_node) if upper_node is not None else None,
        )

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

        subspace = self.subspace(self.g.value(id_, SLV["subspace"]))
        e_acc = self.quantity(self.g.value(id_, SLV["acceleration-energy"]))
        saturation_node = next(
            (
                node
                for node in self.g.objects(id_, CSTR_HDL_EXT["limits"])
                if SLV_EXT["AccelerationSaturation"] in self.g[node : RDF["type"]]
            ),
            None,
        )
        saturation = self.saturation(saturation_node) if saturation_node is not None else None
        as_seen_by_node = self.g.value(id_, GEOM_COORD["as-seen-by"])
        as_seen_by = self.frame(as_seen_by_node) if as_seen_by_node else None

        if SLV_EXT["DirectionAligned"] in self.g[id_ : RDF["type"]]:
            direction = self.direction(self.g.value(id_, SLV_EXT["direction"]))
            return AccelerationConstraint(
                self.id(id_), subspace, None, e_acc, as_seen_by, direction=direction, saturation=saturation
            )

        assert SLV["AxisAligned"] in self.g[id_ : RDF["type"]]
        axis = self.axis(self.g.value(id_, SLV["axis"]))
        return AccelerationConstraint(self.id(id_), subspace, axis, e_acc, as_seen_by, saturation=saturation)

    @memoize
    def subspace(self, id_):
        d = {
            MAP["position"]: Subspace.Position,
            MAP_EXT["rotation"]: Subspace.Rotation,
            MAP_EXT["orientation"]: Subspace.Rotation,
            MAP["angular-velocity"]: Subspace.AngularVelocity,
            MAP["linear-velocity"]: Subspace.LinearVelocity,
            MAP["angular-acceleration"]: Subspace.AngularAcceleration,
            MAP["linear-acceleration"]: Subspace.LinearAcceleration,
            MAP["torque"]: Subspace.Torque,
            MAP["force"]: Subspace.Force,
            SLV["angular-acceleration"]: Subspace.AngularAcceleration,
            SLV["linear-acceleration"]: Subspace.LinearAcceleration,
            GEOM_COORD_EXT["linear"]: Subspace.LinearDifference,
            GEOM_COORD_EXT["angular"]: Subspace.AngularDifference,
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
            if GEOM_OP_EXT["PoseDiffEvaluator"] in self.g[e : RDF["type"]]:
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

        return ConstraintHandler(
            self.id(id_), motion, control_mode, evaluators, controllers, monitors, order
        )

    @memoize
    def monitor_entry(self, id_):
        assert CSTR_HDL["Monitor"] in self.g[id_ : RDF["type"]]

        is_until_aggregate = self.g.value(id_, CSTR_HDL_EXT["monitors-until"]) is not None
        is_when_aggregate = self.g.value(id_, CSTR_HDL_EXT["monitors-when"]) is not None
        error_node = self.g.value(id_, CSTR_HDL["error"])
        error = None if is_until_aggregate or is_when_aggregate or error_node is None else self.quantity(error_node)

        if CSTR_HDL["LevelTriggeredMonitor"] in self.g[id_ : RDF["type"]]:
            flag = self.id(self.g.value(id_, CSTR_HDL["flag"]))
            return MonitorEntry(
                self.id(id_), "LevelTriggeredMonitor", error, flag, None, None, False, is_until_aggregate, is_when_aggregate
            )

        event_node = self.g.value(id_, CSTR_HDL["event"])
        event = self.id(event_node)
        fallback_node = self.g.value(id_, CSTR_HDL_EXT["fallback-motion"])
        fallback_motion = self.id(fallback_node) if fallback_node is not None else None
        debounce_duration_s = self._optional_float(id_, CSTR_HDL_EXT["debounce-duration"])
        return MonitorEntry(
            self.id(id_), "EdgeTriggeredMonitor", error, None, event, None, True, is_until_aggregate, is_when_aggregate,
            event_uri=str(event_node), event_name=event.upper(), fallback_motion=fallback_motion,
            debounce_duration_s=debounce_duration_s,
        )

    @memoize
    def constraint_evaluator(self, id_):
        assert CSTR_HDL["ConstraintEvaluator"] in self.g[id_ : RDF["type"]]

        constraint_node = self.g.value(id_, CSTR_HDL["constraint"])
        constraint = self.constraint(constraint_node)

        if CSTR_HDL["AssignmentEvaluator"] in self.g[id_ : RDF["type"]]:
            t = EvaluatorType.AssignmentEvaluator
            error = None
        else:
            t = EvaluatorType.ErrorEvaluator
            error = self.quantity(self.g.value(id_, CSTR_HDL["error"]))

        # Timing constraint: measured quantity is the motion-state elapsed time (kind Time).
        # No solver error — codegen compares the world clock against the threshold directly.
        is_elapsed = False
        elapsed_op = None
        elapsed_threshold_s = None
        qnode = self.g.value(constraint_node, CSTR["quantity"])
        if qnode is not None and QUDT_QKIND["Time"] in self.g[qnode : QUDT_SCHEMA["hasQuantityKind"]]:
            is_elapsed = True
            elapsed_op = ">=" if CSTR["GreaterThanConstraint"] in self.g[constraint_node : RDF["type"]] else "<"
            thr = self.g.value(constraint_node, CSTR["threshold"])
            thr_val = float(self.g.value(thr, QUDT_SCHEMA["value"]))
            thr_unit = self.g.value(thr, QUDT_SCHEMA["unit"])
            elapsed_threshold_s = thr_val * (0.001 if thr_unit == QUDT_UNIT["MilliSEC"] else 1.0)

        return ConstraintEvaluator(
            self.id(id_), t, constraint, error,
            is_elapsed=is_elapsed, elapsed_op=elapsed_op, elapsed_threshold_s=elapsed_threshold_s,
        )

    @memoize
    def controller(self, id_):
        is_pid = CSTR_HDL["ProportionalIntegralDerivative"] in self.g[id_ : RDF["type"]]
        is_impedance = CSTR_HDL["ImpedanceController"] in self.g[id_ : RDF["type"]]
        is_feedforward = CSTR_HDL_EXT["FeedForwardController"] in self.g[id_ : RDF["type"]]
        assert is_pid or is_impedance or is_feedforward, (
            f"Controller {id_} must be ProportionalIntegralDerivative, ImpedanceController, "
            "or FeedForwardController"
        )

        error_node = self.g.value(id_, CSTR_HDL["error-signal"])
        ref_node = self.g.value(id_, CSTR_HDL_EXT["reference-signal"])
        measured_derivative_node = self.g.value(id_, CSTR_HDL_EXT["measured-derivative"])
        error_signal = self.quantity(error_node) if error_node is not None else None
        reference_signal = self.quantity(ref_node) if ref_node is not None else None
        measured_derivative = (
            self.quantity(measured_derivative_node)
            if measured_derivative_node is not None
            else None
        )
        control_signal = self.quantity(self.g.value(id_, CSTR_HDL["control-signal"]))
        output_saturation_node = next(
            (
                node
                for node in self.g.objects(id_, CSTR_HDL_EXT["limits"])
                if CSTR_HDL_EXT["IntegralSaturation"] not in self.g[node : RDF["type"]]
            ),
            None,
        )
        integral_saturation_node = next(
            (
                node
                for node in self.g.objects(id_, CSTR_HDL_EXT["limits"])
                if CSTR_HDL_EXT["IntegralSaturation"] in self.g[node : RDF["type"]]
            ),
            None,
        )
        output_saturation = (
            self.saturation(output_saturation_node) if output_saturation_node is not None else None
        )
        integral_saturation = (
            self.saturation(integral_saturation_node) if integral_saturation_node is not None else None
        )

        if is_pid:
            if error_signal is None:
                raise ValueError(f"PID controller '{self.id(id_)}' must have cstr-hdl:error-signal.")
            if reference_signal is not None:
                raise ValueError(f"PID controller '{self.id(id_)}' must not have cstr-hdl-ext:reference-signal.")
            decay_rate = None
            if CSTR_HDL["DecayingIntegralTerm"] in self.g[id_ : RDF["type"]]:
                decay_rate = self.g.value(id_, CSTR_HDL["decay-rate"]).value
            return Controller(
                id=self.id(id_),
                control_signal=control_signal,
                error_signal=error_signal,
                measured_derivative=measured_derivative,
                proportional_gain=self._required_float(id_, CSTR_HDL["proportional-gain"]),
                integral_gain=self._required_float(id_, CSTR_HDL["integral-gain"]),
                derivative_gain=self._required_float(id_, CSTR_HDL["derivative-gain"]),
                decay_rate=decay_rate,
                output_saturation=output_saturation,
                integral_saturation=integral_saturation,
                type=self.id(CSTR_HDL.ProportionalIntegralDerivative),
            )
        if is_impedance:
            if measured_derivative is not None:
                raise ValueError(
                    f"Impedance controller '{self.id(id_)}' must not have cstr-hdl-ext:measured-derivative."
                )
            if error_signal is None:
                raise ValueError(f"Impedance controller '{self.id(id_)}' must have cstr-hdl:error-signal.")
            if reference_signal is not None:
                raise ValueError(f"Impedance controller '{self.id(id_)}' must not have cstr-hdl-ext:reference-signal.")
            return Controller(
                id=self.id(id_),
                control_signal=control_signal,
                error_signal=error_signal,
                stiffness=self._required_float(id_, CSTR_HDL["stiffness"]),
                damping=self._required_float(id_, CSTR_HDL["damping"]),
                integral_gain=self._optional_float(id_, CSTR_HDL["integral-gain"]),
                output_saturation=output_saturation,
                type=self.id(CSTR_HDL.ImpedanceController),
            )
        if reference_signal is None:
            raise ValueError(f"FeedForward controller '{self.id(id_)}' must have cstr-hdl-ext:reference-signal.")
        if error_signal is not None:
            raise ValueError(f"FeedForward controller '{self.id(id_)}' must not have cstr-hdl:error-signal.")
        if measured_derivative is not None:
            raise ValueError(
                f"FeedForward controller '{self.id(id_)}' must not have cstr-hdl-ext:measured-derivative."
            )
        return Controller(
            id=self.id(id_),
            control_signal=control_signal,
            reference_signal=reference_signal,
            output_saturation=output_saturation,
            type=self.id(CSTR_HDL_EXT.FeedForwardController),
        )

    @memoize
    def forwarded_command(self, id_):
        assert SLV_EXT["ForwardedCommand"] in self.g[id_ : RDF["type"]]
        command_signal = self.quantity(self.g.value(id_, SLV_EXT["command-signal"]))
        target_node = self.g.value(id_, SLV["attached-to"])
        return ForwardedCommand(
            self.id(id_),
            command_signal,
            self.label(target_node) if target_node is not None else "",
        )

    def _optional_float(self, subject, predicate) -> float | None:
        value = self.g.value(subject, predicate)
        if value is None:
            return None
        literal = value if isinstance(value, rdflib.Literal) else self.g.value(value, QUDT_SCHEMA.value)
        if literal is None:
            raise ValueError(
                f"Controller '{self.id(subject)}' property '{self.id(predicate)}' must be a literal or a node with qudt:value."
            )
        return float(literal.value)

    def _required_float(self, subject, predicate) -> float:
        value = self._optional_float(subject, predicate)
        if value is None:
            raise ValueError(
                f"Controller '{self.id(subject)}' is missing required property '{self.id(predicate)}'."
            )
        return value

    @memoize
    def guarded_motion(self, id_):
        assert MOT["GuardedMotion"] in self.g[id_ : RDF["type"]]

        when = []
        when_any = False
        for c in self.g[id_ : MOT["when"]]:
            if CSTR_EXT.ConstraintDisjunction in self.g[c : RDF["type"]]:
                when_any = True
                for member in self.g[c : CSTR_EXT["has-constraint"]]:
                    when.append(self.constraint(member))
            else:
                when.append(self.constraint(c))

        while_ = []
        for c in self.g[id_ : MOT["while"]]:
            while_.append(self.constraint(c))

        until = []
        until_any = False
        for c in self.g[id_ : MOT["until"]]:
            if CSTR_EXT.ConstraintDisjunction in self.g[c : RDF["type"]]:
                until_any = True
                for member in self.g[c : CSTR_EXT["has-constraint"]]:
                    until.append(self.constraint(member))
            else:
                until.append(self.constraint(c))

        return GuardedMotion(self.id(id_), when, while_, until, until_any, when_any)

    @memoize
    def constraint(self, id_):
        assert CSTR["Constraint"] in self.g[id_ : RDF["type"]]

        quantity = self.quantity(self.g.value(id_, CSTR["quantity"]))

        parameter = None
        if CSTR["EqualityConstraint"] in self.g[id_ : RDF["type"]]:
            parameter = self.equality_constraint(id_)
        elif CSTR["UnilateralConstraint"] in self.g[id_ : RDF["type"]]:
            parameter = self.unilateral_constraint(id_)
        elif CSTR_EXT["OutsideConstraint"] in self.g[id_ : RDF["type"]]:
            parameter = self.outside_constraint(id_)
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
    def outside_constraint(self, id_):
        assert CSTR_EXT["OutsideConstraint"] in self.g[id_ : RDF["type"]]

        lower_threshold = self.quantity(self.g.value(id_, CSTR["lower-threshold"]))
        upper_threshold = self.quantity(self.g.value(id_, CSTR["upper-threshold"]))

        return OutsideConstraint(lower_threshold, upper_threshold)

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

    @memoize
    def orientation(self, id_):
        assert GEOM_COORD["OrientationCoordinate"] in self.g[id_ : RDF["type"]]

        def optional_pose_ref(node):
            if node is None:
                return None
            if ENV.RigidObject in self.g[node : RDF["type"]]:
                return self.scene_object(node)
            if GEOM_ENT.Frame in self.g[node : RDF["type"]]:
                return self.frame(node)
            if GEOM_ENT.SimplicialComplex in self.g[node : RDF["type"]]:
                return self.simplicial_complex(node)
            return None

        of = optional_pose_ref(self.g.value(id_, GEOM_REL["of"]))
        wrt = optional_pose_ref(self.g.value(id_, GEOM_REL["with-respect-to"]))
        quantity_kind = self.quantity_kind(self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]))
        as_seen_by_node = self.g.value(id_, GEOM_COORD["as-seen-by"])
        as_seen_by = self.frame(as_seen_by_node) if as_seen_by_node is not None else None
        unit = self.unit(self.g.value(id_, QUDT_SCHEMA["unit"]))
        axes = self.g.value(id_, GEOM_COORD["axes-sequence"])
        authored, snapshot = self.quantity_role_flags(id_)
        return Orientation(
            self.id(id_),
            of,
            wrt,
            QuantityKind(quantity_kind),
            as_seen_by,
            Unit(unit),
            str(axes) if axes is not None else None,
            (id_, ~MAP["subobject"], None) in self.g,
            authored=authored,
            snapshot=snapshot,
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
    def _pose_endpoint(self, node):
        if node is None:
            return None
        if ENV.RigidObject in self.g[node : RDF["type"]]:
            return self.scene_object(node)
        return self.frame(node)

    def _bare_pose(self, id_):
        # A geom-rel:Pose with no PoseCoordinate: a snapshot/reference pose whose
        # KDL::Frame is filled at runtime. Same IR shape as a coordinate pose, with
        # its frame endpoints but no authored coordinate values.
        as_seen_by_node = self.g.value(id_, GEOM_COORD["as-seen-by"])
        authored, snapshot = self.quantity_role_flags(id_)
        return Pose(
            self.id(id_),
            self._pose_endpoint(self.g.value(id_, GEOM_REL["of"])),
            self._pose_endpoint(self.g.value(id_, GEOM_REL["with-respect-to"])),
            [self.quantity_kind(k) for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]],
            self.frame(as_seen_by_node) if as_seen_by_node is not None else None,
            [self.unit(u) for u in self.g[id_ : QUDT_SCHEMA["unit"]]],
            None,
            None,
            None,
            None,
            authored=authored,
            snapshot=snapshot,
        )

    def pose(self, id_):
        assert GEOM_COORD["PoseCoordinate"] in self.g[id_ : RDF["type"]]
        assert GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]]

        of = self._pose_endpoint(self.g.value(id_, GEOM_REL["of"]))
        wrt = self._pose_endpoint(self.g.value(id_, GEOM_REL["with-respect-to"]))
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

        authored, snapshot = self.quantity_role_flags(id_)
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
            authored=authored,
            snapshot=snapshot,
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

        authored, snapshot = self.quantity_role_flags(id_)
        return VelocityTwist(
            self.id(id_),
            of,
            wrt,
            quantity_kind,
            reference_point,
            as_seen_by,
            unit,
            authored=authored,
            snapshot=snapshot,
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

        authored, snapshot = self.quantity_role_flags(id_)
        return AccelerationTwist(
            self.id(id_),
            quantity_kind,
            reference_point,
            as_seen_by,
            unit,
            authored=authored,
            snapshot=snapshot,
        )

    @memoize
    def pose_difference(self, id_):
        assert GEOM_COORD_EXT["PoseDifferenceCoordinate"] in self.g[id_ : RDF["type"]]
        assert GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]]

        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.quantity_kind(k))
        reference_point = self.point(self.g.value(id_, GEOM_REL["reference-point"]))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = []
        for u in self.g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.unit(u))

        authored, snapshot = self.quantity_role_flags(id_)
        return PoseDifference(
            self.id(id_),
            quantity_kind,
            reference_point,
            as_seen_by,
            unit,
            authored=authored,
            snapshot=snapshot,
        )

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

        sensor_name = str(self.g.value(id_, MJ["ft-sensor-ref"]) or "")
        authored, snapshot = self.quantity_role_flags(id_)
        return Wrench(
            self.id(id_),
            quantity_kind,
            reference_point,
            as_seen_by,
            unit,
            authored=authored,
            snapshot=snapshot,
            sensor_name=sensor_name,
        )

    @memoize
    def quantity(self, id_):
        assert QUDT_SCHEMA["Quantity"] in self.g[id_ : RDF["type"]]

        quantity_kind_node = self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]) or self.g.value(
            id_, QUDT_SCHEMA["quantity-kind"]
        )
        quantity_kind = self.quantity_kind(quantity_kind_node)

        unit = self.unit(self.g.value(id_, QUDT_SCHEMA["unit"]))
        has_view = (id_, ~MAP["subobject"], None) in self.g

        if GEOM_REL["Pose"] in self.g[id_ : RDF["type"]]:
            if GEOM_COORD["PoseCoordinate"] in self.g[id_ : RDF["type"]]:
                return self.pose(id_)
            return self._bare_pose(id_)
        if (
            GEOM_REL["Position"] in self.g[id_ : RDF["type"]]
            and GEOM_COORD["PositionCoordinate"] in self.g[id_ : RDF["type"]]
        ):
            return self.position(id_)
        if (
            GEOM_REL["Orientation"] in self.g[id_ : RDF["type"]]
            and GEOM_COORD["OrientationCoordinate"] in self.g[id_ : RDF["type"]]
        ):
            return self.orientation(id_)
        if TRAJ["Trajectory"] in self.g[id_ : RDF["type"]]:
            value_kind_node = next(
                (k for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]] if k != TRAJ.Trajectory),
                None,
            )
            authored, snapshot = self.quantity_role_flags(id_)
            return Trajectory(
                self.id(id_),
                QuantityKind(quantity_kind),
                Unit(unit),
                has_view,
                authored=authored,
                snapshot=snapshot,
                value_kind=self.quantity_kind(value_kind_node) if value_kind_node is not None else None,
            )

        if quantity_kind == "FreeVector" and GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]]:
            authored, snapshot = self.quantity_role_flags(id_)
            return FreeVector(
                self.id(id_),
                QuantityKind(quantity_kind),
                Unit(unit),
                self.parse_xyz(id_),
                has_view,
                authored=authored,
                snapshot=snapshot,
            )

        value = None
        if (id_, QUDT_SCHEMA["value"], None) in self.g:
            value = float(self.g.value(id_, QUDT_SCHEMA["value"]))
        reference_value = self.g.value(id_, CSTR["reference-value"])
        authored, snapshot = self.quantity_role_flags(id_)
        return Quantity(
            self.id(id_),
            QuantityKind(quantity_kind),
            Unit(unit),
            value,
            has_view,
            authored=authored,
            snapshot=snapshot,
            reference_value=self.id(reference_value) if reference_value is not None else None,
        )

    @memoize
    def joint_position(self, id_):
        assert KC_STAT["JointPositionCoordinate"] in self.g[id_ : RDF["type"]]
        joint_node = self.g.value(id_, GEOM_REL["of"])
        joint_name = self.label(joint_node) if joint_node is not None else ""
        return JointPosition(self.id(id_), joint_name)

    def _is_snapshot(self, id_):
        # snapshot == the node is a runtime snapshot (snap:Snapshot / snap:snapshot-of).
        return SNAP.Snapshot in self.g[id_ : RDF["type"]]

    def quantity_role_flags(self, id_):
        # (authored, snapshot), mutually exclusive: snapshot wins (mirrors old roles() elif).
        # authored == carries an authored value/coordinate and is not a runtime snapshot.
        snapshot = self._is_snapshot(id_)
        authored = (not snapshot) and self._is_authored(id_)
        return authored, snapshot

    def _is_authored(self, id_):
        if (id_, QUDT_SCHEMA["value"], None) in self.g:
            return True
        if (id_, CSTR["reference-value"], None) in self.g:
            return True
        if any(
            (id_, GEOM_COORD[c], None) in self.g
            for c in ("x", "y", "z", "direction-cosine-x", "direction-cosine-y", "direction-cosine-z")
        ):
            return True
        if any(True for _ in self.g.objects(id_, GEOM_COORD["has-coordinate"])):
            return True
        return False

    @memoize
    def quantity_kind(self, id_):
        return self.id(id_)

    @memoize
    def unit(self, id_):
        return self.id(id_)

    @memoize
    def simplicial_complex(self, id_):
        assert GEOM_ENT["SimplicialComplex"] in self.g[id_ : RDF["type"]]
        # rdf.py's _frame_body() mints this off a Frame (mj:attached-body) so Twist.of/wrt don't
        # fuse Frame and body onto one URI. Follow the link back so the id matches its name.
        owner = self.g.value(predicate=MJ["attached-body"], object=id_)
        return SimplicialComplex(self.id(owner if owner is not None else id_))

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
        # rdf.py's _frame_origin_point() mints this off a Frame (geom-ent:origin) so Position.of/wrt
        # don't fuse Frame and Point onto one URI. Follow the link back so the id matches its name.
        owner = self.g.value(predicate=GEOM_ENT["origin"], object=id_)
        return Point(self.id(owner if owner is not None else id_))

    def view(self):
        dispatcher = [
            (MAP["DirectionCoordinateView"], self.direction),
            (MAP["PoseCoordinateView"], self.pose),
            (MAP_EXT["PoseOrientationView"], self.pose),
            (MAP_EXT["PosePositionView"], self.pose),
            (MAP["VelocityTwistCoordinateView"], self.velocity_twist),
            (MAP["AccelerationTwistCoordinateView"], self.acceleration_twist),
            (MAP_EXT["PoseDifferenceView"], self.pose_difference),
            (MAP["WrenchCoordinateView"], self.wrench),
            (MAP_EXT["WrenchVectorView"], self.wrench),
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
            axis_node = self.g.value(view, MAP["axis"])
            axis = self.axis(axis_node) if axis_node is not None else None

            assert superobject is not None
            view_map[self.id(subobject.id)] = View(
                self.id(view), superobject, subobject, subspace, axis
            )

        return view_map

    def data_structures(self):
        dispatcher = [
            (GEOM_COORD["DirectionCoordinate"], self.direction),
            (GEOM_COORD["PositionCoordinate"], self.position),
            (GEOM_COORD["OrientationCoordinate"], self.orientation),
            (GEOM_COORD["PoseCoordinate"], self.pose),
            (GEOM_COORD["VelocityTwistCoordinate"], self.velocity_twist),
            (GEOM_COORD["AccelerationTwistCoordinate"], self.acceleration_twist),
            (GEOM_COORD_EXT["PoseDifferenceCoordinate"], self.pose_difference),
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
                    if operator.type_ == CSTR_HDL["Controller"]:
                        # output-saturation lives on cstr-hdl-ext:limits (the non-integral
                        # SignalLimiter); the closure the step template renders is built from
                        # input/output/param edges, so attach the clamp explicitly.
                        sat_node = next(
                            (
                                n
                                for n in self.g.objects(closure, CSTR_HDL_EXT["limits"])
                                if CSTR_HDL_EXT["IntegralSaturation"] not in self.g[n : RDF["type"]]
                            ),
                            None,
                        )
                        if sat_node is not None:
                            cl["output_saturation"] = self.saturation(sat_node)
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

        # An operator/call: data_in --(in)--> call --(out)--> data_out (traversed backward here;
        # there may be multiple inputs/outputs).
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

            # An inline/declared Pose is no operator's output, so follow its coordinate /
            # reference-value edges to schedule the closures producing its scalar components --
            # else an Arc/Lerp ending in a declared pose assembles from zeros -> wrong endpoint.
            for successor in itertools.chain(
                self.g.objects(data_out, GEOM_COORD["has-coordinate"]),
                self.g.objects(data_out, CSTR["reference-value"]),
            ):
                if successor not in data_structures:
                    q.append(successor)
                    data_structures.add(successor)

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


def _filtered_motion_driver(driver, handler_output_ids: set[str], closure_input_map):
    """Return the slice of a solver driver fed by the current handler outputs."""
    acceleration_specs = []
    for acc_spec in driver.acceleration_constraint:
        constraints = [
            ac
            for ac in acc_spec.constraints
            if ac.acceleration_energy.id in handler_output_ids
        ]
        if constraints:
            acceleration_specs.append(replace(acc_spec, constraints=constraints))

    cartesian_forces = []
    for force_spec in driver.cartesian_force:
        force_id = force_spec.force.id
        upstream = {force_id} | _upstream_dependencies(force_id, closure_input_map)
        if upstream & handler_output_ids:
            cartesian_forces.append(force_spec)

    joint_forces = [
        jf_spec
        for jf_spec in driver.joint_force
        if jf_spec.force_id in handler_output_ids
    ]

    if not acceleration_specs and not cartesian_forces and not joint_forces:
        return None

    return replace(
        driver,
        acceleration_constraint=acceleration_specs,
        cartesian_force=cartesian_forces,
        joint_force=joint_forces,
        has_cartesian_force=bool(cartesian_forces),
    )


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
            filtered = _filtered_motion_driver(driver, handler_output_ids, closure_input_map)
            if filtered is not None:
                matched.append(filtered)
        if not matched:
            continue
        selected = next((driver for driver in matched if driver.id == motion_driver_id), matched[0])
        result.append(
            MotionArmSolver(
                id=solver.id,
                output=solver.output,
                motion_driver=selected,
                control_mode=handler.control_mode,
                algorithm=solver.algorithm,
                algorithm_is_rne=solver.algorithm_is_rne,
                gravity=solver.gravity,
                root_acc=solver.root_acc,
                chain_root=solver.chain_root,
                chain_end=solver.chain_end,
                torque_saturation=solver.torque_saturation,
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
    return getattr(ref, "id", None)


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
    snapshot_clock_map=None,
):
    snapshot_clock_map = snapshot_clock_map or {}
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
                super_id = object_id(object_field(view, "superobject"))
                subobject_id = object_id(object_field(view, "subobject"))
                # superobject -> subobject (existing forward decomposition)
                if super_id == current and subobject_id:
                    pending.append(subobject_id)
                # subobject -> superobject: a snapshot of a composite pose is only
                # ever referenced through its scalar components (e.g. task_pose via
                # task_pose_position_y). Climb back so the composite snapshot is captured.
                if subobject_id == current and super_id:
                    pending.append(super_id)

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
                clock=snapshot_clock_map.get(target_id, "task"),
            )
        )

    # If the motion resets on re-entry (has any `on entry` sample), its `on task`
    # samples must survive that reset -> persistent. Otherwise nothing persists.
    has_entry = any(s.clock == "entry" for s in result)
    for s in result:
        s.persistent = has_entry and s.clock == "task"
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
    "Norm": "out",
    "RotateWrenchToDistalWithPose": "to",
    "RotateWrenchToProximalWithPose": "to",
    "TransformWrenchToProximal": "to",
    "WrenchFromPositionDirectionAndMagnitude": "wrench",
    "VelocityProfile": "reference",
    "Admittance": "reference",
}


def _scene_relative_poses_for_motion(view_map, arm_solvers, data_structures=None, evaluators=None):
    """For each view whose wrt-frame is a scene object, emit the requested relative pose."""
    fk_pose_by_frame: dict[str, str] = {}
    # Keys are the scene-object's id (the "of" of scene-object solver pose outputs).
    # A wrt_id lookup asks: "is this frame the subject of a tracked scene-object pose?"
    scene_pose_by_id: dict[str, tuple[str, str | None]] = {}
    solver_output_ids: set[str] = set()
    for solver in arm_solvers:
        for out in solver.output:
            if getattr(out, "type", "") != "Pose":
                continue
            solver_output_ids.add(out.id)
            of = getattr(out, "of", None)
            if of is None:
                chain_end = getattr(solver, "chain_end", None)
                if chain_end:
                    fk_pose_by_frame.setdefault(chain_end, out.id)
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

    for view in view_map.values():
        so = getattr(view, "superobject", None)
        if so is None or not hasattr(so, "of") or not hasattr(so, "with_respect_to"):
            continue
        if getattr(so, "id", None) not in solver_output_ids:
            continue
        of = getattr(so, "of", None)
        if of is None or getattr(of, "is_scene_object", False):
            continue
        fk_pose_by_frame.setdefault(getattr(of, "id", ""), so.id)

    candidate_poses = [
        getattr(view, "superobject", None)
        for view in view_map.values()
        if not getattr(getattr(view, "superobject", None), "authored", False)
    ]
    for evaluator in evaluators or []:
        constraint = getattr(evaluator, "constraint", None)
        quantity = getattr(constraint, "quantity", None)
        if quantity is not None and hasattr(quantity, "of") and hasattr(quantity, "with_respect_to"):
            candidate_poses.append(quantity)

    seen: set[str] = set()
    result: list[SceneRelativePose] = []
    for so in candidate_poses:
        if so is None:
            continue
        pose_id = getattr(so, "id", None)
        if not pose_id or pose_id in seen:
            continue
        of = getattr(so, "of", None)
        wrt = getattr(so, "with_respect_to", None)
        if of is None or wrt is None:
            continue
        of_id = getattr(of, "id", "")
        wrt_id = getattr(wrt, "id", "")
        if getattr(of, "is_scene_object", False) or of_id in scene_pose_by_id:
            continue
        scene_pose = scene_pose_by_id.get(wrt_id)
        if scene_pose is None and not getattr(wrt, "is_scene_object", False):
            continue
        fk_pose_id = fk_pose_by_frame.get(of_id)
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
        if GEOM_OP_EXT["PoseDiffEvaluator"] in p.g[eval_node : RDF["type"]]:
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

        # A whole-subspace view (e.g. keeping <pose>.position as a unit) has no per-axis
        # component, so it cannot join a per-axis pose-error group; it is controlled by its
        # own equality-constraint controller instead. Skip it rather than deref a None axis.
        if view.axis is None:
            continue

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
    data_structures=None,
    snapshot_clock_map=None,
):
    snapshot_clock_map = snapshot_clock_map or {}
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
        _raw_when = set(g[motion_node : MOT["when"]])
        when_constraint_nodes = set()
        for node in _raw_when:
            if CSTR_EXT.ConstraintDisjunction in g[node : RDF["type"]]:
                when_constraint_nodes.update(g[node : CSTR_EXT["has-constraint"]])
            else:
                when_constraint_nodes.add(node)
        while_constraint_nodes = set(g[motion_node : MOT["while"]])
        _raw_until = set(g[motion_node : MOT["until"]])
        until_constraint_nodes = set()
        for node in _raw_until:
            if CSTR_EXT.ConstraintDisjunction in g[node : RDF["type"]]:
                until_constraint_nodes.update(g[node : CSTR_EXT["has-constraint"]])
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

        def _is_elapsed_eval(eval_node):
            # Timing evaluator: measured quantity is kind Time. No kinematic computation,
            # so it must stay out of the schedule (the monitor reads the clock directly).
            cnode = g.value(eval_node, CSTR_HDL["constraint"])
            if cnode is None:
                return False
            qnode = g.value(cnode, CSTR["quantity"])
            return qnode is not None and QUDT_QKIND["Time"] in g[qnode : QUDT_SCHEMA["hasQuantityKind"]]

        # Classify controllers: driven by while-evaluator error outputs.
        # PoseDiffEvaluator exposes its components as normal MAP views over the
        # operator output twist, so collect those view subobjects here.
        while_error_nodes = set()
        while_pose_eval_nodes = []
        for n in while_eval_nodes:
            if GEOM_OP_EXT["PoseDiffEvaluator"] in g[n : RDF["type"]]:
                while_pose_eval_nodes.append(n)
                pose_diff_out = g.value(n, GEOM_OP_EXT["out"])
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
            if (
                g.value(n, CSTR_HDL["error-signal"]) in while_error_nodes
                or g.value(n, CSTR_HDL["constraint"]) in while_constraint_nodes
            )
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
            if g.value(mon_node, CSTR_HDL_EXT["monitors-when"]) is not None:
                when_mon_nodes.append(mon_node)
                continue
            if g.value(mon_node, CSTR_HDL_EXT["monitors-until"]) is not None:
                until_mon_nodes.append(mon_node)
                continue
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

        # Validate classified sets are subsets of what the handler declares. Evaluators/controllers
        # not linked to any motion phase are silently excluded from all schedules (e.g. sc1's extras).
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

        # when_schedule runs in can_start (own Parser + dedup set); while_/until share p_active
        # so steps evaluated in both phases emit once (see the _build_ir schedules note).
        p_when = Parser(g)
        when_schedule = p_when.schedule(
            [n for n in when_eval_nodes if not _is_elapsed_eval(n)], ops_generic + ops_cstr_hdl
        )

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

        # Direction-aligned ACHD constraints: their runtime direction (PoseToDirection) is no
        # evaluator/controller input, so seed the walk from the constraint nodes themselves.
        direction_constraint_nodes = []
        for solver in handler_arm_solvers:
            driver_node = node_by_id.get(solver.motion_driver.id)
            if driver_node is None:
                continue
            for spec_node in g[driver_node : SLV["acceleration-constraint"]]:
                for acc_node in g[spec_node : SLV["constraints"]]:
                    if SLV_EXT["DirectionAligned"] in g[acc_node : RDF["type"]]:
                        direction_constraint_nodes.append(acc_node)

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
                and not _is_elapsed_eval(n)
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
        while_schedule.extend(
            p_active.schedule(direction_constraint_nodes, ops_generic + ops_slv + ops_cstr_hdl)
        )

        # While evaluators watched only by a monitor (no controller consumes their error) aren't
        # reached by backward discovery; append them after their deps so their evaluate call emits.
        for n in while_eval_nodes:
            if GEOM_OP_EXT["PoseDiffEvaluator"] in g[n : RDF["type"]]:
                continue
            if _is_elapsed_eval(n):
                continue
            eval_id = p.id(n)
            if eval_id not in p_active.sched:
                while_schedule.append(eval_id)
                p_active.sched.add(eval_id)

        until_schedule = p_active.schedule(
            [n for n in until_eval_nodes if not _is_elapsed_eval(n)], ops_generic + ops_cstr_hdl
        )

        # Until evaluators have no controllers whose error-signal would drive their
        # backward discovery. Append them explicitly after their dependencies so the
        # template emits the computation calls in the correct order.
        for n in until_eval_nodes:
            if _is_elapsed_eval(n):
                continue
            eval_id = p.id(n)
            if eval_id not in p_active.sched:
                until_schedule.append(eval_id)
                p_active.sched.add(eval_id)

        # Same for when evaluators: the can_start template inlines them via
        # when_evaluators, but any prerequisite generic ops still need scheduling.
        # Append when evaluators that were not discovered through backward traversal.
        for n in when_eval_nodes:
            if _is_elapsed_eval(n):
                continue
            eval_id = p.id(n)
            if eval_id not in p_when.sched:
                when_schedule.append(eval_id)
                p_when.sched.add(eval_id)

        # Build Python objects from classified RDF nodes
        when_evaluators = [p.constraint_evaluator(n) for n in when_eval_nodes]
        while_evaluators = [
            p.constraint_evaluator(n)
            for n in while_eval_nodes
            if GEOM_OP_EXT["PoseDiffEvaluator"] not in g[n : RDF["type"]]
        ]
        until_evaluators = [p.constraint_evaluator(n) for n in until_eval_nodes]
        controllers = [p.controller(n) for n in ctrl_nodes]
        controller_output_ids = {c.control_signal.id for c in controllers if c.control_signal is not None}
        forwarded_commands = [
            p.forwarded_command(n)
            for n in g.subjects(RDF.type, SLV_EXT["ForwardedCommand"])
            if p.id(g.value(n, SLV_EXT["command-signal"])) in controller_output_ids
        ]
        when_monitors = [p.monitor_entry(n) for n in when_mon_nodes]
        while_monitors = [p.monitor_entry(n) for n in while_mon_nodes]
        until_monitors = [p.monitor_entry(n) for n in until_mon_nodes]

        _all_evaluators = when_evaluators + while_evaluators + until_evaluators
        has_elapsed = any(getattr(e, "is_elapsed", False) for e in _all_evaluators)

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
                when_schedule=when_schedule,
                while_schedule=while_schedule,
                until_schedule=until_schedule,
                when_events=[m.event for m in when_monitors if m.event is not None],
                while_events=[m.event for m in while_monitors if m.event is not None],
                until_events=[m.event for m in until_monitors if m.event is not None],
                has_elapsed=has_elapsed,
                has_until_condition=bool(until_evaluators),
                until_any=handler.motion.until_any,
                when_any=handler.motion.when_any,
                arm_solvers=handler_arm_solvers,
                relative_poses=_relative_poses_for_motion(
                    while_evaluators + when_evaluators + until_evaluators,
                    view_map,
                    _arm_solvers_for_handler(handler, slv_arm),
                ),
                scene_relative_poses=_scene_relative_poses_for_motion(
                    view_map,
                    handler_arm_solvers,
                    data_structures,
                    while_evaluators + when_evaluators + until_evaluators,
                ),
                pose_axis_error_groups=pose_axis_error_groups,
                forwarded_commands=forwarded_commands,
                snapshots=(_motion_snapshots := _snapshots_for_motion(
                    while_evaluators + when_evaluators + until_evaluators,
                    motion.while_ + motion.when + motion.until,
                    snapshot_source_map,
                    view_map,
                    closure_output_map,
                    data_reference_map,
                    while_schedule + when_schedule + until_schedule,
                    closures,
                    snapshot_clock_map,
                )),
                has_entry_snapshot=any(s.clock == "entry" for s in _motion_snapshots),
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


def _xyz_or_none(g, node):
    if node is None:
        return None
    values = [g.value(node, GEOM_COORD[axis]) for axis in ("x", "y", "z")]
    if any(v is None for v in values):
        return None
    return [float(v.value) for v in values]


def _orientation_degrees(g, node):
    result: dict[str, float | None] = {"roll": None, "pitch": None, "yaw": None}
    if node is None:
        return None
    for coord in g.objects(node, GEOM_COORD["has-coordinate"]):
        axis = str(g.value(coord, GEOM_COORD["angle-axis"]) or "")
        if axis not in result:
            continue
        value_node = g.value(coord, QUDT_SCHEMA.value)
        if value_node is None:
            continue
        value = float(value_node.value)
        unit = str(g.value(coord, QUDT_SCHEMA.unit) or "")
        if unit.endswith("RAD"):
            value = math.degrees(value)
        result[axis] = value
    if any(value is None for value in result.values()):
        return None
    return [result["roll"], result["pitch"], result["yaw"]]


def _position_of(g, obj_node):
    # geom-rel:Position.of targets the frame's origin Point (geom-ent:origin),
    # not the frame node itself -- see rdf.py's _frame_origin_point().
    origin = g.value(obj_node, GEOM_ENT["origin"]) or obj_node
    for pos_node in g.subjects(GEOM_REL["of"], origin):
        if GEOM_COORD["PositionCoordinate"] in g[pos_node:RDF.type]:
            return _xyz_or_none(g, pos_node)
    return None


def _orientation_of(g, obj_node):
    for orient_node in g.subjects(GEOM_REL["of"], obj_node):
        if GEOM_COORD["OrientationCoordinate"] in g[orient_node:RDF.type]:
            return _orientation_degrees(g, orient_node)
    return None


def _path_of_model(g, model_node):
    if not model_node:
        return ""
    return str(g.value(model_node, EXEC.path) or "")


def _mj_body_name(g, node):
    return str(g.value(node, MJ["body-name"]) or "") if node else ""


def _site_name(g, node):
    return str(g.value(node, MJ["site-name"]) or "") if node else ""


def _resolve_existing_path(path: str) -> Path | None:
    if not path:
        return None
    candidate = Path(path)
    if candidate.exists():
        return candidate
    for root in [Path.cwd(), *Path.cwd().parents]:
        candidate = root / path
        if candidate.exists():
            return candidate
    text = Path(path).as_posix()
    cache_root = None
    if os.environ.get("XDG_CACHE_HOME"):
        cache_root = Path(os.environ["XDG_CACHE_HOME"]) / "mj_kdl_wrapper"
    elif os.environ.get("HOME"):
        cache_root = Path(os.environ["HOME"]) / ".cache" / "mj_kdl_wrapper"

    def cache_path(marker: str, cache_subdir: str) -> Path | None:
        pos = text.find(marker)
        if pos == -1 or cache_root is None:
            return None
        return cache_root / cache_subdir / text[pos + len(marker):]

    menagerie_marker = "third_party/menagerie/"
    pos = text.find(menagerie_marker)
    if pos != -1 and os.environ.get("MJ_KDL_MENAGERIE"):
        candidate = Path(os.environ["MJ_KDL_MENAGERIE"]) / text[pos + len(menagerie_marker):]
        if candidate.exists():
            return candidate
    for candidate in (
        cache_path(menagerie_marker, "menagerie"),
        cache_path("src/mj_kdl_wrapper/assets/", "assets"),
        cache_path("src/examples/assets/", "assets"),
    ):
        if candidate is not None and candidate.exists():
            return candidate
    return None


def _mjcf_body_containing_site(path: str, site_name: str) -> str:
    resolved = _resolve_existing_path(path)
    if resolved is None or not site_name:
        return ""
    try:
        root = ET.parse(resolved).getroot()
    except ET.ParseError:
        return ""

    def visit_body(body) -> str:
        for site in body.findall("site"):
            if site.get("name") == site_name:
                return body.get("name") or ""
        for child in body.findall("body"):
            found = visit_body(child)
            if found:
                return found
        return ""

    for worldbody in root.findall("worldbody"):
        for body in worldbody.findall("body"):
            found = visit_body(body)
            if found:
                return found
    return ""


def _derive_tool_body_from_attachment(attachment: SceneAttachment, tcp_site: str) -> str:
    if not tcp_site:
        return ""
    local_site = tcp_site
    if attachment.prefix and tcp_site.startswith(attachment.prefix):
        local_site = tcp_site[len(attachment.prefix):]
    local_body = _mjcf_body_containing_site(attachment.path, local_site)
    if not local_body:
        return ""
    return f"{attachment.prefix}{local_body}"


def _attach_target_of(g, obj_node):
    kind = str(g.value(obj_node, MJ["attach-kind"]) or "world").title()
    if kind not in {"World", "Body", "Site", "Frame"}:
        kind = "World"
    name = str(g.value(obj_node, MJ["attach-name"]) or "")
    target = g.value(obj_node, SLV["attached-to"])
    if kind == "Site" and target is not None and ENV.Object in g[target:RDF.type]:
        target_name = _id_from_uri(target)
        if target_name and name and not name.startswith(f"{target_name}_{target_name}_"):
            name = f"{target_name}_{name}"
    return kind, name


def _transitive_attachment_nodes(g, env_node, robot_node):
    """Attachment nodes reachable from the robot through SLV:attached-to, parent-first.

    Supports chained attachments (e.g. gripper -> ft-sensor -> robot), not just those
    bolted directly to the robot. Ordered so a parent always precedes its children,
    which is the order MuJoCo needs (a child's target site only exists once its parent
    is attached).
    """
    candidates = []
    for c in g.objects(env_node, ENV["has-object"]):
        path = _path_of_model(g, c) or _path_of_model(g, g.value(c, ENV["has-object-model"]))
        if path:
            candidates.append((c, g.value(c, SLV["attached-to"])))
    ordered = []
    resolved = {robot_node}
    progress = True
    while progress:
        progress = False
        for c, target in candidates:
            if c in resolved or target not in resolved:
                continue
            ordered.append(c)
            resolved.add(c)
            progress = True
    return ordered


def _attachments_for_robot(g, env_node, robot_node, tool_body):
    attachments = []
    prefix_by_node = {}
    for candidate in _transitive_attachment_nodes(g, env_node, robot_node):
        path = _path_of_model(g, candidate)
        if not path:
            path = _path_of_model(g, g.value(candidate, ENV["has-object-model"]))
        if not path:
            continue
        attach_node = g.value(candidate, MJ["attach-to-body"])
        attach_kind_lit = g.value(candidate, MJ["attach-kind"])
        if attach_kind_lit is None:
            raise ValueError(
                f"Attachment '{candidate}' is missing mj:attach-kind; "
                f"add an 'attach-to:' entry to the .robmot model."
            )
        attach_kind = str(attach_kind_lit).title()
        if attach_kind not in {"Body", "Site", "Frame"}:
            raise ValueError(
                f"Attachment '{candidate}' has unsupported attach-kind '{attach_kind}'; "
                f"expected Body, Site, or Frame."
            )
        attach_to = _site_name(g, attach_node) if attach_kind == "Site" else _mj_body_name(g, attach_node)
        prefix_lit = g.value(candidate, MJ["attach-prefix"])
        prefix = str(prefix_lit) if prefix_lit is not None else ""
        # When attached to another (prefixed) attachment, that parent's sites/bodies
        # carry its prefix in the compiled model, so prefix the target name to match.
        parent_prefix = prefix_by_node.get(g.value(candidate, SLV["attached-to"]), "")
        if parent_prefix and attach_to and not attach_to.startswith(parent_prefix):
            attach_to = parent_prefix + attach_to
        pos = _xyz_or_none(g, g.value(candidate, MJ["attach-position"]))
        euler = _orientation_degrees(g, g.value(candidate, MJ["attach-orientation"]))
        actuator = str(g.value(candidate, MJ["actuator-name"]) or "")
        attachments.append(
            SceneAttachment(
                id=_id_from_uri(candidate),
                path=path,
                attach_to=attach_to,
                attach_kind=attach_kind,
                prefix=prefix,
                pos=pos,
                euler=euler,
                actuator=actuator,
            )
        )
        prefix_by_node[candidate] = prefix
    return attachments


def _color_rgba(g, owner_node):
    """Read a grouped mj:ColorRGBA value node (via mj:color) as [r, g, b, a].

    Returns None when the owner has no colour, so callers can fall back to their
    own defaults. Shared by scene objects and the trajectory-trace overlay.
    """
    color_node = g.value(owner_node, MJ["color"])
    if color_node is None:
        return None
    channels = [g.value(color_node, MJ[f"color-{ch}"]) for ch in ("r", "g", "b", "a")]
    if any(c is None for c in channels):
        return None
    return [float(c.toPython()) for c in channels]


def _trace_from_graph(g):
    """Read the optional MuJoCo trajectory-trace overlay config from the graph.

    The trace is a viewer-only overlay (a polyline of recent EE positions). When
    no TRACE block is declared it stays disabled, so headless runs and non-MuJoCo
    runtimes cost nothing. Defaults match the wrapper's built-in warm orange.
    """
    trace = {
        "enabled": False,
        "length": 4096,
        "color_r": 1.0,
        "color_g": 0.5,
        "color_b": 0.1,
        "color_a": 1.0,
        "targets": [],
        "has_targets": False,
    }
    trace_node = next(g.objects(predicate=MJ["has-trace"]), None)
    if trace_node is None:
        return trace

    enabled = g.value(trace_node, MJ["trace-enabled"])
    if enabled is not None:
        trace["enabled"] = bool(enabled.toPython())
    length = g.value(trace_node, MJ["trace-length"])
    if length is not None:
        trace["length"] = int(length.toPython())
    color = _color_rgba(g, trace_node)
    if color is not None:
        trace["color_r"], trace["color_g"], trace["color_b"], trace["color_a"] = color
    for target in g.objects(trace_node, MJ["trace-target"]):
        trace["targets"].append({"link": str(target)})
    trace["has_targets"] = bool(trace["targets"])
    return trace


def _scene_from_graph(g):
    scene = SceneSpec()
    for env_node in g.subjects(RDF.type, ENV.Workspace):
        _timestep = g.value(env_node, MJ["timestep"])
        if _timestep is not None:
            scene.timestep_s = float(_timestep.toPython())
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
            attach_kind, attach_name = _attach_target_of(g, robot_node)

            scene.robots.append(
                SceneRobot(
                    id=_id_from_uri(robot_node),
                    path=robot_path,
                    prefix=str(g.value(robot_node, MJ["prefix"]) or ""),
                    attach_kind=attach_kind,
                    attach_name=attach_name,
                    pos=_position_of(g, robot_node),
                    euler=_orientation_of(g, robot_node),
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
            obj_types = set(g[obj_node:RDF.type])
            if POLY.CuboidWithSize in obj_types:
                shape = "BOX"
            else:
                shape_lit = g.value(obj_node, MJ["shape"])
                shape = str(shape_lit).upper() if shape_lit is not None else None
            size_x = g.value(obj_node, POLY["x-size"])
            size_y = g.value(obj_node, POLY["y-size"])
            size_z = g.value(obj_node, POLY["z-size"])
            size = (
                [float(size_x.value), float(size_y.value), float(size_z.value)]
                if size_x is not None and size_y is not None and size_z is not None
                else None
            )
            mass_value = None
            for mass_node in g.subjects(RBDYN_ENT["of-body"], obj_node):
                if RBDYN_ENT.Mass in g[mass_node:RDF.type]:
                    mv = g.value(mass_node, RBDYN_ENT.mass)
                    if mv is not None:
                        mass_value = float(mv.value)
                        break
            attach_kind, attach_name = _attach_target_of(g, obj_node)
            f_slide = g.value(obj_node, MJ["friction-slide"])
            f_torsion = g.value(obj_node, MJ["friction-torsion"])
            f_roll = g.value(obj_node, MJ["friction-roll"])
            friction = (
                [float(f_slide.value), float(f_torsion.value), float(f_roll.value)]
                if f_slide is not None and f_torsion is not None and f_roll is not None
                else None
            )
            color = _color_rgba(g, obj_node)
            scene.objects.append(
                SceneObjectSpec(
                    id=_id_from_uri(obj_node),
                    body=body,
                    path=path,
                    attach_kind=attach_kind,
                    attach_name=attach_name,
                    pos=_position_of(g, obj_node),
                    fixed=bool(path),
                    shape=shape,
                    size=size,
                    color=color,
                    mass=mass_value,
                    friction=friction,
                )
            )
        break
    return scene


def _id_from_uri(node):
    return str(node).rstrip("/").split("/")[-1].split("#")[-1].replace("-", "_")



def _robot_setups_from_graph(g):
    """Per-robot solver chain setups for the whole workspace.

    Returns ``(setups_by_node, ordered)`` where ``setups_by_node`` maps each
    robot's graph node to its setup tuple
    ``(urdf, chain_root, chain_end, chain_tip, robot_model, tool_body, tcp_site, ft_sensors)``
    and ``ordered`` is the same tuples in declaration order. Names derived from
    the robot's own MJCF (``chain_tip``, ``tool_body``) get the robot's prefix
    prepended so they resolve in the prefixed, multi-robot MuJoCo scene;
    already-prefixed (authored) names are left untouched.
    """
    setups_by_node = {}
    ordered = []
    for env_node in g.subjects(RDF.type, ENV.Workspace):
        for obj_node in g.objects(env_node, ENV["has-object"]):
            chain = g.value(obj_node, GEOM_ENT["kinematic-chain"])
            if chain is None:
                continue
            prefix = str(g.value(obj_node, MJ["prefix"]) or "")
            start = g.value(chain, GEOM_ENT.start)
            end = g.value(chain, GEOM_ENT.end)
            chain_root = str(g.value(start, MJ["body-name"]) or "")
            chain_end = str(g.value(end, MJ["body-name"]) or "")
            model_node = g.value(obj_node, ENV["has-object-model"])
            urdf = _path_of_model(g, model_node)
            robot_model = str(model_node).rstrip("/").split("/")[-1].split("#")[-1] if model_node else ""
            tool_body = _mj_body_name(g, g.value(obj_node, MJ["tool-body"]))
            tcp_site = _site_name(g, g.value(obj_node, MJ["tcp-site"]))
            ft_sensors = sorted(
                (
                    {
                        "name": str(g.value(ft_node, MJ["sensor-name"]) or ""),
                        "frame_site": str(g.value(ft_node, MJ["frame-site"]) or ""),
                    }
                    for ft_node in g.objects(obj_node, MJ["ft-sensor"])
                ),
                key=lambda s: s["name"],
            )
            attachments = _attachments_for_robot(g, env_node, obj_node, tool_body)
            for attachment in _transitive_attachment_nodes(g, env_node, obj_node):
                tool_body = tool_body or _mj_body_name(g, g.value(attachment, MJ["tool-body"]))
                tcp_site = tcp_site or _site_name(g, g.value(attachment, MJ["tcp-site"]))
            if not tool_body and tcp_site:
                for attachment in attachments:
                    tool_body = _derive_tool_body_from_attachment(attachment, tcp_site)
                    if tool_body:
                        break
            chain_tip = chain_end
            if tcp_site and chain_end == tcp_site and attachments and attachments[0].attach_kind == "Body":
                chain_tip = attachments[0].attach_to
            elif tcp_site and chain_end == tcp_site and attachments and attachments[0].attach_kind == "Site":
                model_path = _path_of_model(g, model_node)
                site_body = _mjcf_body_containing_site(model_path, attachments[0].attach_to)
                if not site_body:
                    raise ValueError(
                        f"Cannot derive robot chain tip body: site '{attachments[0].attach_to}' "
                        f"was not found in model '{model_path}'."
                    )
                chain_tip = site_body
            if prefix:
                if chain_tip and not chain_tip.startswith(prefix):
                    chain_tip = prefix + chain_tip
                if tool_body and not tool_body.startswith(prefix):
                    tool_body = prefix + tool_body
            if not (chain_root or chain_end or urdf):
                continue
            setup = (urdf, chain_root, chain_end, chain_tip, robot_model, tool_body, tcp_site, ft_sensors)
            setups_by_node[obj_node] = setup
            ordered.append(setup)
    return setups_by_node, ordered


def _uri_table(id_nodes):
    return [
        {"id": id_, "uri": str(node)}
        for id_, node in sorted(id_nodes, key=lambda item: (item[0], str(item[1])))
        if isinstance(node, URIRef)
    ]


def _id_ref(value):
    if value is None:
        return None
    if isinstance(value, str):
        return value
    if isinstance(value, Enum):
        return value.value
    return getattr(value, "id", None)


def _id_refs(value):
    if value is None:
        return []
    if isinstance(value, list):
        return [ref for item in value if (ref := _id_ref(item))]
    ref = _id_ref(value)
    return [ref] if ref else []


def _compact(entry):
    return {k: v for k, v in entry.items() if v is not None and v != []}


def _dedupe_dicts(entries, key="id"):
    result = []
    seen = set()
    for entry in entries:
        value = entry.get(key)
        if value in seen:
            continue
        seen.add(value)
        result.append(entry)
    return result


def _build_introspection(
    *,
    app_model_path,
    imported_models,
    id_nodes,
    node_by_id,
    motions,
    data_structures,
    control_period_ns,
    backend,
    scene,
):
    uri_rows = _uri_table(id_nodes)
    uri_by_id = {row["id"]: row["uri"] for row in uri_rows}

    controllers = []
    monitors = []
    signals = []
    for motion in motions:
        for controller in motion.controllers:
            controller_entry = {
                "id": controller.id,
                "uri": uri_by_id.get(controller.id),
                "motion": motion.id,
                "type": controller.type,
                "proportional_gain": controller.proportional_gain,
                "integral_gain": controller.integral_gain,
                "derivative_gain": controller.derivative_gain,
                "decay_rate": controller.decay_rate,
                "stiffness": controller.stiffness,
                "damping": controller.damping,
                "error_signal": _id_ref(controller.error_signal),
                "reference_signal": _id_ref(controller.reference_signal),
                "measured_derivative": _id_ref(controller.measured_derivative),
                "output_signal": _id_ref(controller.control_signal),
            }
            controllers.append(_compact(controller_entry))
            for role in ("error_signal", "reference_signal", "measured_derivative", "control_signal"):
                quantity_id = _id_ref(getattr(controller, role))
                if quantity_id:
                    signals.append(
                        _compact(
                            {
                                "id": f"{controller.id}.{role}",
                                "uri": uri_by_id.get(quantity_id),
                                "quantity": quantity_id,
                                "role": role,
                                "owner": controller.id,
                            }
                        )
                    )
        for phase in ("when", "while", "until"):
            for monitor in getattr(motion, f"{phase}_monitors"):
                monitors.append(
                    _compact(
                        {
                            "id": monitor.id,
                            "uri": uri_by_id.get(monitor.id),
                            "motion": motion.id,
                            "phase": phase,
                            "type": monitor.monitor_type,
                            "trigger": "edge" if monitor.is_edge_triggered else "level",
                            "event": monitor.event,
                            "event_uri": monitor.event_uri,
                            "event_name": monitor.event_name,
                            "flag": monitor.flag,
                            "error_signal": _id_ref(monitor.error),
                            "fallback_motion": monitor.fallback_motion,
                            "debounce_duration_s": monitor.debounce_duration_s,
                            "debounce_steps": monitor.debounce_steps,
                        }
                    )
                )
                if monitor.error is not None:
                    signals.append(
                        _compact(
                            {
                                "id": f"{monitor.id}.error",
                                "uri": uri_by_id.get(monitor.error.id),
                                "quantity": monitor.error.id,
                                "role": "monitor_error",
                                "owner": monitor.id,
                            }
                        )
                    )

    quantities = []
    for item in data_structures:
        quantities.append(
            _compact(
                {
                    "id": item.id,
                    "uri": uri_by_id.get(item.id),
                    "type": item.type,
                    "unit": _id_refs(getattr(item, "unit", None)),
                    "quantity_kind": _id_refs(getattr(item, "quantity_kind", None)),
                    "reference_value": getattr(item, "reference_value", None),
                    "value": getattr(item, "value", None),
                    "authored": getattr(item, "authored", False),
                    "snapshot": getattr(item, "snapshot", False),
                }
            )
        )

    runtime_type = {
        "mj_kdl": "rt:MuJoCoRuntime",
        "robif2b": "rt:RealRobotRuntime",
    }.get(backend, "rt:Runtime")
    runtime_id = "agent:runtime:mujoco" if backend == "mj_kdl" else "agent:runtime:real_robot"
    runtime_activity_type = (
        "bdd:SimulatedExecution"
        if backend == "mj_kdl"
        else "bdd:ScenarioExecution"
    )

    entities = [
        {
            "id": "entity:app_manifest",
            "types": ["prov:Entity"],
            "role": "app_manifest",
            "path": str(app_model_path),
        },
        {
            "id": "entity:motion_spec_ir",
            "types": ["prov:Entity"],
            "role": "motion_spec_ir",
            "wasGeneratedBy": "activity:motion_spec_ir_generation",
            "wasDerivedFrom": "entity:app_manifest",
        },
    ]
    entities.extend(
        {
            "id": f"entity:imported_graph:{idx}",
            "types": ["prov:Entity"],
            "role": "imported_model_graph",
            "source": source,
            "wasDerivedFrom": "entity:app_manifest",
        }
        for idx, source in enumerate(imported_models)
    )

    agents = [
        {
            "id": "agent:motion_spec_ir_gen",
            "types": [
                "prov:SoftwareAgent",
                "obs:ObservationProvider",
            ],
            "role": "ir_generator",
        },
        {
            "id": runtime_id,
            "types": ["prov:SoftwareAgent", runtime_type],
            "role": "runtime_runner",
        },
        {
            "id": "agent:controller_process",
            "types": ["prov:SoftwareAgent"],
            "role": "controller_process",
            "actedOnBehalfOf": runtime_id,
        },
    ]
    agents.extend(
        {
            "id": f"agent:modelled:{robot.id}",
            "types": [
                "prov:Agent",
                "agn:ModelledAgent",
            ],
            "role": "robot",
            "model": robot.path,
        }
        for robot in scene.robots
    )

    return {
        "contract_version": 1,
        "control_period_ns": control_period_ns,
        "uris": uri_rows,
        "motions": [
            _compact(
                {
                    "id": motion.id,
                    "uri": uri_by_id.get(motion.id),
                    "handler": motion.handler,
                    "handler_uri": uri_by_id.get(motion.handler),
                    "control_mode": motion.control_mode,
                    "controllers": [controller.id for controller in motion.controllers],
                    "monitors": [
                        monitor.id
                        for group in (motion.when_monitors, motion.while_monitors, motion.until_monitors)
                        for monitor in group
                    ],
                }
            )
            for motion in motions
        ],
        "states": [],
        "controllers": _dedupe_dicts(controllers),
        "monitors": _dedupe_dicts(monitors),
        "quantities": _dedupe_dicts(quantities),
        "signals": _dedupe_dicts(signals),
        "provenance": {
            "contexts": [
                {
                    "id": "prov",
                    "uri": "http://www.w3.org/ns/prov#",
                    "source": "src/metamodels/prov.json",
                    "shape": "src/metamodels/prov.shacl.ttl",
                },
                {
                    "id": "bdd",
                    "uri": "https://secorolab.github.io/metamodels/acceptance-criteria/bdd#",
                    "source": "src/metamodels/acceptance-criteria/bdd/bdd.json",
                },
                {
                    "id": "agent",
                    "uri": "https://secorolab.github.io/metamodels/agent#",
                    "source": "src/metamodels/acceptance-criteria/bdd/agent.json",
                },
                {
                    "id": "observation",
                    "uri": "https://secorolab.github.io/metamodels/observation#",
                    "source": "src/metamodels/acceptance-criteria/bdd/observation.json",
                },
                {
                    "id": "runtime",
                    "uri": str(RT.Runtime),
                    "source": "src/metamodels/runtime/runtime.json",
                },
            ],
            "entities": entities,
            "activities": [
                {
                    "id": "activity:motion_spec_ir_generation",
                    "types": ["prov:Activity"],
                    "used": [entity["id"] for entity in entities if entity["role"] != "motion_spec_ir"],
                    "wasAssociatedWith": "agent:motion_spec_ir_gen",
                    "role": "motion_spec_ir_generation",
                },
                {
                    "id": "activity:controller_execution",
                    "types": ["prov:Activity", runtime_activity_type],
                    "used": ["entity:motion_spec_ir"],
                    "wasAssociatedWith": "agent:controller_process",
                    "role": "controller_execution",
                },
            ],
            "agents": agents,
        },
    }


def generate_ir(manifest_path):
    app_model_path = Path(manifest_path).resolve()

    # Load top-level, application model
    g = rdflib.Dataset(default_union=True)
    install_resolver(IriToFileResolver(metamodel_url_map(), download=False))
    g.parse(str(app_model_path), format="json-ld")

    # Load IRI map
    url_map = build_url_map(g, app_model_path)
    install_resolver(IriToFileResolver({**metamodel_url_map(), **url_map}))

    # Load/import the referenced models
    imported_models = list(dict.fromkeys(str(model) for model in g.objects(predicate=APP["import"])))
    for model in imported_models:
        g.parse(location=model, format="json-ld")

    p = Parser(g)
    node_by_id = {}
    id_nodes = []
    for node in sorted(g.subjects(), key=lambda item: str(item)):
        try:
            id_ = p.id(node)
        except Exception:
            continue
        id_nodes.append((id_, node))
        node_by_id.setdefault(id_, node)

    sched1 = []
    slv_base_vel = []
    sched2 = []
    hdl = []
    sched3 = []
    slv_arm = []
    sched4 = []
    slv_base_frc = []
    setups_by_node, ordered_setups = _robot_setups_from_graph(g)
    _default_setup = ordered_setups[0] if ordered_setups else ("", "", "", "", "", "", "", [])
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

    # Traverse backward from the "distal" solver configuration.
    # The "parser" keeps track of the previously visited closures/computations
    # so that they are visited only once.
    for s in g.subjects(RDF.type, SLV["SolverWithInputAndOutput"]):
        solver = p.solver_with_input_and_output(s)
        robot_node = g.value(s, SLV_EXT["robot"])
        (
            _urdf,
            _chain_root,
            _chain_end,
            _chain_tip,
            _robot_model,
            _tool_body,
            _tcp_site,
            _ft_sensors,
        ) = setups_by_node.get(robot_node, _default_setup)
        solver.urdf = _urdf
        solver.chain_root = _chain_root
        solver.chain_end = _chain_end
        solver.chain_tip = _chain_tip
        solver.robot_model = _robot_model
        solver.tool_body = _tool_body
        solver.tcp_site = _tcp_site
        solver.ft_sensors = _ft_sensors
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
    snapshot_clock_map: dict[str, str] = {}
    for snap_node in g.subjects(RDF.type, SNAP.Snapshot):
        source_node = g.value(snap_node, SNAP["snapshot-of"])
        if source_node is not None:
            snapshot_source_map[p.id(snap_node)] = p.id(source_node)
        # Sampling clock: "entry" re-samples each state activation; else "task" (once).
        clock_node = g.value(snap_node, SNAP["sampled-on"])
        snapshot_clock_map[p.id(snap_node)] = (
            "entry" if clock_node == SNAP["entry-clock"] else "task"
        )

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
        snapshot_clock_map=snapshot_clock_map,
        view_map=view_map,
        closure_output_map=closure_output_map,
        data_reference_map=data_reference_map,
        closure_input_map=closure_input_map,
        closures=closures,
        data_structures=data_structures,
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

    if scene.timestep_s <= 0:
        raise ValueError("ENVIRONMENT timestep must be positive.")
    control_period_ns = int(round(scene.timestep_s * 1e9))

    # Authorable control-loop tuning: sourced from the arm solver(s), same
    # single-value-across-the-scene contract as the timestep (one generated
    # control loop). Saturations are emitted only from explicit limits.
    def _single_solver_value(attr, label, default):
        values = {v for s in slv_arm if (v := getattr(s, attr)) is not None}
        if len(values) > 1:
            raise ValueError(
                f"Multiple {label} values found across arm solvers, but generated code has one "
                "control loop."
            )
        return next(iter(values), default)

    rne_damping_lambda = _single_solver_value("regularization", "Solver regularization", 0.05)

    # Per-monitor debounce (`for <FLOAT> <Unit>`): convert the authored seconds
    # into a step count now that the control period is known. Absent -> stays
    # None, which keeps the codegen on the existing rising-edge path (byte-identical).
    for handler in hdl:
        for monitor in handler.monitors:
            if monitor.debounce_duration_s is not None:
                monitor.debounce_steps = round(monitor.debounce_duration_s / (control_period_ns * 1e-9))

    # Safeguard: no two distinct URIs may collapse to one generated id (would silently merge).
    p.assert_no_id_collisions()

    shared_data = _filter_shared_data(
        data_structures,
        sched1 + sched2 + sched3 + sched4,
        closures,
        view_map=view_map,
        fk_output_ids={out.id for s in slv_arm for out in s.output},
    )
    # Promote FT tare state (bias + settle counter) to shared_data so it is captured once before
    # any push and reused across handlers, instead of a later mid-push state re-taring it to ~zero.
    _ft_tare_members = []
    _seen_ft_ids = set()
    for s in slv_arm:
        for out in s.output:
            if getattr(out, "type", None) == "Wrench" and getattr(out, "sensor_name", ""):
                if out.id in _seen_ft_ids:
                    continue
                _seen_ft_ids.add(out.id)
                _ft_tare_members.append({"id": f"{out.id}_ft_bias", "type": "FreeVector"})
                _ft_tare_members.append({"id": f"{out.id}_ft_settle", "type": "IntCounter"})
    shared_data = shared_data + _ft_tare_members

    # Persistent (once-per-run) snapshot guards live in shared_data so they
    # survive the motion-state reset applied on FSM re-entry (fixed traj targets).
    _persist_captured_members = []
    _seen_captured = set()
    for m in motions:
        for snap in getattr(m, "snapshots", []):
            if getattr(snap, "persistent", False) and snap.target_id not in _seen_captured:
                _seen_captured.add(snap.target_id)
                _persist_captured_members.append({"id": f"{snap.target_id}_captured", "type": "Bool"})
    shared_data = shared_data + _persist_captured_members

    introspection = _build_introspection(
        app_model_path=app_model_path,
        imported_models=imported_models,
        id_nodes=id_nodes,
        node_by_id=node_by_id,
        motions=motions,
        data_structures=data_structures,
        control_period_ns=control_period_ns,
        backend=backend,
        scene=scene,
    )

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
        "shared_data": shared_data,
        "wrench_outputs": wrench_outputs,
        "has_arm": bool(slv_arm),
        "has_mobile_base": bool(slv_base_vel or slv_base_frc),
        "control_period_ns": control_period_ns,
        "rne_damping_lambda": rne_damping_lambda,
        "arm_solvers": slv_arm,
        "base_velocity_solvers": slv_base_vel,
        "base_force_solvers": slv_base_frc,
        "backend": backend,
        "scene": scene,
        "trace": _trace_from_graph(g),
        # Flat id -> full model URI table; sorted for deterministic emission.
        "uris": introspection["uris"],
        "introspection": introspection,
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
