# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Intermediate representation (IR) generator for motion specification models.

This module parses RDF graphs containing motion specification models and generates
a JSON intermediate representation suitable for code generation.
"""

from __future__ import annotations

import collections
import itertools
import math
import re
import weakref
from dataclasses import asdict, dataclass, field, replace
from enum import Enum
from functools import wraps
from pathlib import Path
from urllib.parse import urlsplit

import rdflib
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.naming import get_valid_var_name
from rdf_utils.models.execution import get_path_of_node
from scene_dsl.rdf_parser.kinematics import body_of_frame, get_kinematic_mapping
from scene_dsl.rdf_parser.sensors import get_update_rate
from scene_dsl.rdf_parser.vocab import NS_MM_ROS
from rdf_utils.models.geom_coord import (
    OrientCoordModel,
    PoseCoordModel,
    PositionCoordModel,
    get_coord_vectorxyz,
    get_orientation_coord_vals,
)
from rdf_utils.models.geom_rel import OrientationModel, PoseModel, PositionModel
from rdf_utils.models.common import ModelBase, get_node_types
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_AXES_SEQ,
    URI_GEOM_PRED_ALPHA,
    URI_GEOM_PRED_BETA,
    URI_GEOM_PRED_DIRECTION_COSINE_X,
    URI_GEOM_PRED_DIRECTION_COSINE_Y,
    URI_GEOM_PRED_DIRECTION_COSINE_Z,
    URI_GEOM_PRED_GAMMA,
    URI_GEOM_PRED_OF,
    URI_GEOM_PRED_OF_ORIENT,
    URI_GEOM_PRED_OF_POSE,
    URI_GEOM_PRED_OF_POSITION,
    URI_GEOM_PRED_ORIGIN,
    URI_GEOM_PRED_SEEN_BY,
    URI_GEOM_PRED_WRT,
    URI_GEOM_PRED_W,
    URI_GEOM_PRED_X,
    URI_GEOM_PRED_Y,
    URI_GEOM_PRED_Z,
    URI_GEOM_TYPE_ANGLES_ABG,
    URI_GEOM_TYPE_DIRECTION_COSINE_XYZ,
    URI_GEOM_TYPE_EULER_ANGLES,
    URI_GEOM_TYPE_INTRINSIC,
    URI_GEOM_TYPE_POSE_COORD,
    URI_GEOM_TYPE_POSE,
    URI_GEOM_TYPE_POSITION,
    URI_GEOM_TYPE_POSITION_COORD,
    URI_GEOM_TYPE_ORIENT,
    URI_GEOM_TYPE_ORIENT_COORD,
    URI_GEOM_TYPE_ORIENT_REF,
    URI_GEOM_TYPE_FRAME,
    URI_GEOM_TYPE_POINT,
    URI_GEOM_TYPE_POSE_REF,
    URI_GEOM_TYPE_POSITION_REF,
    URI_GEOM_TYPE_VECTOR_XYZ,
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_QUDT_QK_LENGTH,
    URI_GEOM_TYPE_QUATERNION,
    URI_KC_TYPE_SERIAL,
    URI_QUDT_UNIT_CM,
    URI_QUDT_UNIT_DEG,
    URI_QUDT_UNIT_M,
    URI_QUDT_UNIT_MM,
    URI_QUDT_UNIT_RAD,
)
from rdf_utils.namespace import (
    NS_MM_KC_EXT,
    NS_MM_QUDT_QTY as QUDT_QTY,
    NS_MM_QUDT_UNIT as QUDT_UNIT,
)
from rdf_utils.resolver import IriToFileResolver, install_resolver
from rdf_utils.uri import (
    iri_is_descendant,
    iri_parent,
)
from rdflib import URIRef
from rdflib.namespace import RDF, SDO, split_uri

# fmt: off
from motion_spec.classes.entities import (
    AccelerationConstraint, AccelerationTwist, Axis, BilateralConstraint,
    CartesianAccelerationSpecification, CartesianForceSpecification, Constraint, ConstraintEvaluator, ConstraintHandler,
    DataclassJSONEncoder, Direction, EdgeMonitor, EqualityConstraint,
    EvaluatorType, FeedForwardController, ForceDistributionSolver, ForwardedCommand, Frame,
    FreeVector, GuardedMotion, GuardedMotionBlock, HandlerSerialChainSolver, ImpedanceController,
    JointForceSpecification, JointPosition, LevelMonitor, MotionDrivers, Orientation,
    OutsideConstraint, PIDController, Point, Pose, PoseAxisErrorComponent, PoseAxisErrorGroup,
    PoseDifference, Position, Provenance, Quantity, QuantityKind,
    RelativePoseCapture,
    Saturation, SceneAttachment, SceneObject, SceneObjectSpec, SceneRelativePose, SceneRobot,
    SceneSpec, Setpoint, SimplicialComplex, SnapshotCapture, SolverWithInputAndOutput, Subspace,
    UnilateralConstraint, UnilateralConstraintType, Unit, VelocityCompositionSolver,
    VelocityTwist, View, Wrench,
)
from motion_spec.classes.closures import closure_output_ids
from motion_spec_dsl.rdf_parser.vocab import (
    AGN, ALGO_EXT, APP, CSTR, CSTR_EXT, CSTR_HDL, CSTR_HDL_EXT, ENV, EXEC, GEOM_COORD,
    GEOM_ENT, GEOM_OP, GEOM_OP_EXT, GEOM_PATH, GEOM_REL, KC, KC_STAT, MAP, MAP_EXT, MOT, QUDT_QKIND,
    QUDT_SCHEMA, RBDYN_COORD, RBDYN_ENT, RBDYN_OP, SLV, SLV_EXT,
    SENSORS, SOSA, TIME,
)
# fmt: on
from motion_spec_dsl.rdf_parser.manifest import build_url_map, metamodel_url_map

# ROS interop: a monitor's `also publish to topic` clause is emitted as ros:channel-name /
# ros:type-name on the monitor node (ns from bdd-dsl's ROS metamodel).


def _ros_camel_to_snake(name: str) -> str:
    """rosidl message-name -> header stem (Trinary->trinary, TrinaryStamped->trinary_stamped)."""
    s = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", name)
    s = re.sub(r"([a-z\d])([A-Z])", r"\1_\2", s)
    return s.lower()


def _ros_type_parts(ros_type: str) -> tuple[str, str, str]:
    """`pkg/msg/CamelType` -> (pkg, "pkg/msg/camel_type.hpp", "pkg::msg::CamelType").

    Derives the include and C++ type from the rosidl naming rule rather than hardcoding.
    """
    parts = ros_type.split("/")
    pkg, msg_name = parts[0], parts[-1]
    sub = parts[1] if len(parts) == 3 else "msg"
    include = f"{pkg}/{sub}/{_ros_camel_to_snake(msg_name)}.hpp"
    cpp_type = f"{pkg}::{sub}::{msg_name}"
    return pkg, include, cpp_type

@dataclass(frozen=True)
class SpatialAxis:
    """One ordered linear or angular Cartesian direction.

    A path-following direction is only known at runtime, so it names the shared vector that
    carries it instead of a fixed frame axis; `axis` is then the vector's role on the path.
    """

    subspace: str
    axis: str
    direction: str | None = None

    @property
    def suffix(self) -> str:
        """Return the compatibility suffix used by generated IR identifiers."""
        prefix = "lin" if self.subspace == "linear-acceleration" else "ang"
        return f"{prefix}_{self.axis}"

    @property
    def frame_axis(self) -> str | None:
        """The fixed frame axis this direction is, or None when it is a runtime vector."""
        return None if self.direction is not None else self.axis


class AccelerationInputKind(str, Enum):
    """Physical acceleration input accepted by a solver algorithm."""

    None_ = "None"
    ConstraintEnergy = "ConstraintEnergy"
    CartesianAcceleration = "CartesianAcceleration"


@dataclass(frozen=True)
class SolverSemantics:
    """Algorithm-specific meanings needed while deriving executable solver inputs."""

    acceleration_input: AccelerationInputKind
    signal_prefix: str | None = None
    codegen_name: str = ""


SOLVER_SEMANTICS_BY_ALGORITHM = {
    SLV["AccelerationConstrainedHybridDynamicsAlgorithm"]: SolverSemantics(
        AccelerationInputKind.ConstraintEnergy, "eacc", "ACHD"
    ),
    SLV["RecursiveNewtonEulerAlgorithm"]: SolverSemantics(
        AccelerationInputKind.CartesianAcceleration, "acc", "RNE"
    ),
}
COMMAND_FORWARDING_SEMANTICS = SolverSemantics(AccelerationInputKind.None_)


@dataclass(frozen=True)
class SolverIdFactory:
    """Build compatibility IDs solely from authored RDF resource IDs."""

    controller: str
    motion: str

    def component_controller(self, axis: SpatialAxis) -> str:
        return f"{self.controller}_{axis.suffix}"

    def component_error(self, axis: SpatialAxis) -> str:
        return f"{self.controller}_err_{axis.suffix}"

    def component_energy(self, axis: SpatialAxis) -> str:
        return f"eacc_{self.controller}_{axis.suffix}"

    def component_acceleration(self, axis: SpatialAxis) -> str:
        return f"acc_{self.controller}_{axis.suffix}"

    def component_constraint(self, axis: SpatialAxis) -> str:
        return f"acc_cstr_{self.controller}_{axis.suffix}"

    def component_acceleration_specification(self, axis: SpatialAxis) -> str:
        return f"cart_acc_{self.controller}_{axis.suffix}"

    def component_measured_derivative(self, axis: SpatialAxis) -> str:
        return f"{self.controller}_measured_derivative_{axis.suffix}"

    def pose_evaluator(self) -> str:
        return f"eval_pose_diff_{self.controller}"

    def pose_difference(self) -> str:
        return f"pose_diff_{self.controller}"

    # IRI suffixes. The id puts its tag first (`eacc_<ctrl>_<axis>`); the IRI leads with the
    # parent, so the same derivation reads as `<ctrl-iri>/eacc-<axis>`.
    SUFFIXES = {
        "component_controller": "{axis}",
        "component_error": "err-{axis}",
        "component_energy": "eacc-{axis}",
        "component_acceleration": "acc-{axis}",
        "component_constraint": "acc-cstr-{axis}",
        "component_acceleration_specification": "cart-acc-{axis}",
        "component_measured_derivative": "measured-derivative-{axis}",
        "pose_evaluator": "eval-pose-diff",
        "pose_difference": "pose-diff",
    }

    # Whether a derivation narrows the parent (a per-axis component of it) or computes something
    # new from it. Decides prov:specializationOf vs prov:wasDerivedFrom.
    SPECIALIZATIONS = frozenset({"component_controller", "component_error"})

    def suffix(self, kind: str, axis: SpatialAxis | None = None) -> str:
        """IRI path segment for one derivation kind."""
        return self.SUFFIXES[kind].format(axis=_kebab(axis.suffix) if axis else "")


_AXIS_BY_FRAME_AXIS = {"x": Axis.X, "y": Axis.Y, "z": Axis.Z}

_AXIS_DERIVATIONS = (
    "component_controller",
    "component_error",
    "component_energy",
    "component_acceleration",
    "component_constraint",
    "component_acceleration_specification",
    "component_measured_derivative",
)


def _register_derived_family(iris, parent_node, ids: SolverIdFactory, axes) -> None:
    """Register every IRI this factory can mint for one controller and its axes.

    Registered per family rather than at each inline mint: the sites call different subsets, and
    a missed one silently leaves a slot unaddressable. An id registered but never used is inert.
    """
    parent = str(parent_node)
    for kind in ("pose_evaluator", "pose_difference"):
        iris.register(
            getattr(ids, kind)(), parent, ids.suffix(kind), DerivedIriRegistry.DERIVATION
        )
    for axis in axes or ():
        for kind in _AXIS_DERIVATIONS:
            iris.register(
                getattr(ids, kind)(axis),
                parent,
                ids.suffix(kind, axis),
                DerivedIriRegistry.SPECIALIZATION
                if kind in SolverIdFactory.SPECIALIZATIONS
                else DerivedIriRegistry.DERIVATION,
            )


LINEAR_AXES = tuple(SpatialAxis("linear-acceleration", axis) for axis in "xyz")
ANGULAR_AXES = tuple(SpatialAxis("angular-acceleration", axis) for axis in "xyz")
POSE_AXES = (*LINEAR_AXES, *ANGULAR_AXES)


def spatial_axes(
    *,
    controller_type: str,
    subspace: str | None,
    axis: str | None,
    command_type: str | None,
    relation: str,
    quantity_kind: str | None,
) -> tuple[SpatialAxis, ...]:
    """Return the ordered Cartesian directions implied by an authored controller."""
    if controller_type == "ImpedanceController":
        command_type = "Force"
    if command_type == "Force" or subspace == "force":
        return ()
    if command_type == "Torque" and quantity_kind == "JointPosition":
        return ()
    if quantity_kind == "Pose" and subspace in {None, "pose"} and relation == "EqualityConstraint":
        return POSE_AXES
    if subspace in {"position", "linear-velocity"}:
        return (SpatialAxis("linear-acceleration", axis),) if axis else LINEAR_AXES
    if subspace in {"orientation", "angular-velocity"}:
        return (SpatialAxis("angular-acceleration", axis),) if axis else ANGULAR_AXES
    if subspace == "distance" and axis is not None:
        return (SpatialAxis("linear-acceleration", axis),)
    if subspace == "rotation" and axis is not None:
        return (SpatialAxis("angular-acceleration", axis),)
    if subspace == "distance" and axis is None:
        return (SpatialAxis("linear-acceleration", "distance"),)
    return ()


def _is_elapsed_constraint(g, cstr_node) -> bool:
    """Whether a constraint node is a timing (elapsed) constraint."""
    return cstr_node is not None and CSTR_EXT["TimeConstraint"] in get_node_types(g, cstr_node)


def _duration_seconds(g, node) -> float:
    """The value in seconds of a Duration node, converting from the unit it was written in."""
    return _seconds(float(g.value(node, QUDT_SCHEMA["value"])), g.value(node, QUDT_SCHEMA["unit"]))


# ---------------------------------------------------------------------------
# DSL operators and specifications
# ---------------------------------------------------------------------------
def _term_name(node) -> str | None:
    if node is None:
        return None
    return split_uri(str(node))[1]


def _path_projection_outputs(g) -> dict[URIRef, dict[str, URIRef]]:
    """The local frame and measured speed each path projection produces, keyed by its path."""
    speed_by_direction = {
        g.value(op, GEOM_OP["direction"]): g.value(op, GEOM_OP_EXT["along-speed"])
        for op in g.subjects(RDF.type, GEOM_OP_EXT.TwistToLinearVelocityAlong)
    }
    outputs = {}
    for frame in g.subjects(RDF.type, GEOM_OP_EXT.PathTangentFrame):
        roles = {
            role: g.value(frame, GEOM_OP_EXT[role])
            for role in ("tangent", "normal-a", "normal-b")
        }
        roles["along-speed"] = speed_by_direction.get(roles["tangent"])
        outputs[g.value(frame, GEOM_OP_EXT.path)] = roles
    return outputs


def _path_following_axes(
    outputs: dict[str, URIRef], quantity: URIRef, subspace: str | None
) -> tuple[SpatialAxis, ...]:
    """The directions one path-following constraint controls.

    A path fixes geometry but not timing, so the three roles never mix: the driver commands
    the tangent alone, holding the frame on the path costs the two normals, and orientation
    tracking is the ordinary angular triple.
    """
    if quantity == outputs["along-speed"]:
        return (SpatialAxis("linear-acceleration", "tangent", outputs["tangent"]),)
    if subspace == "position":
        return (
            SpatialAxis("linear-acceleration", "normal_a", outputs["normal-a"]),
            SpatialAxis("linear-acceleration", "normal_b", outputs["normal-b"]),
        )
    if subspace == "orientation":
        return ANGULAR_AXES
    raise ValueError(
        f"Path-following constraint on '{quantity}' must control the speed along the path, "
        "its position, or its orientation."
    )


def _authored_controller_axes(g) -> dict[URIRef, tuple[SpatialAxis, ...]]:
    """Cartesian directions derived only from authored controller facts."""
    handler_controllers = set(g.objects(None, CSTR_HDL.controllers))
    projections = _path_projection_outputs(g)
    result = {}
    for controller in handler_controllers:
        constraint = g.value(controller, CSTR_HDL.constraint)
        quantity = g.value(constraint, CSTR.quantity) if constraint is not None else None
        if constraint is None or quantity is None:
            continue
        view = next(g.subjects(MAP.subobject, quantity), None)
        path = g.value(constraint, GEOM_OP_EXT.path)
        if path is not None:
            subspace = _term_name(g.value(view, MAP.subspace)) if view is not None else None
            result[controller] = _path_following_axes(projections[path], quantity, subspace)
            continue
        subspace = _term_name(g.value(view, MAP.subspace)) if view is not None else None
        axis = _term_name(g.value(view, MAP.axis)) if view is not None else None
        quantity_kind = None
        target = g.value(view, MAP.superobject) if view is not None else quantity
        target_types = get_node_types(g, target)
        if GEOM_COORD.PoseCoordinate in target_types:
            quantity_kind = "Pose"
        elif KC_STAT.JointPositionCoordinate in target_types:
            quantity_kind = "JointPosition"
        controller_types = get_node_types(g, controller)
        controller_type = next(
            (
                _term_name(type_)
                for type_ in (
                    CSTR_HDL.ImpedanceController,
                    CSTR_HDL.ProportionalIntegralDerivative,
                    CSTR_HDL_EXT.FeedForwardController,
                )
                if type_ in controller_types
            ),
            "",
        )
        relation = next(
            (
                _term_name(type_)
                for type_ in get_node_types(g, constraint)
                if type_ != CSTR.Constraint and _term_name(type_).endswith("Constraint")
            ),
            "",
        )
        command_type = g.value(controller, APP["command-type"])
        result[controller] = spatial_axes(
            controller_type=controller_type,
            subspace=subspace,
            axis=axis,
            command_type=str(command_type) if command_type is not None else None,
            relation=relation,
            quantity_kind=quantity_kind,
        )
    return result


@dataclass(frozen=True)
class ControllerDerivation:
    """Resolved authored facts needed to derive one controller's solver IR."""

    handler: URIRef
    motion: URIRef
    controller: URIRef
    solver: URIRef
    constraint: URIRef
    quantity: URIRef
    view: URIRef | None
    axes: tuple[SpatialAxis, ...]


@dataclass(frozen=True)
class SolverDerivationContext:
    """Immutable indexes for solver expansion, built once before IR emission."""

    controllers_by_handler: dict[URIRef, tuple[ControllerDerivation, ...]]
    controllers_by_solver: dict[URIRef, tuple[ControllerDerivation, ...]]
    semantics_by_solver: dict[URIRef, SolverSemantics]
    shared_constraints: frozenset[URIRef]
    # Carried here rather than threaded through six derivation signatures: the context already
    # reaches every site that mints an id, which is the only place the parent node is still known.
    iris: DerivedIriRegistry


def _solver_derivation_context(g, iris: DerivedIriRegistry) -> SolverDerivationContext:
    """Resolve controller ownership and command shape without generated solver nodes."""
    axes_by_controller = _authored_controller_axes(g)
    by_handler = {}
    by_solver: dict[URIRef, list[ControllerDerivation]] = collections.defaultdict(list)
    for handler in g.subjects(RDF.type, CSTR_HDL.ConstraintHandler):
        motion = g.value(handler, CSTR_HDL.motion)
        if not isinstance(motion, URIRef):
            raise ValueError(f"Constraint handler '{handler}' is missing its motion.")
        authored = sorted(
            dict.fromkeys(g.objects(handler, CSTR_HDL.controllers)),
            key=lambda node: int(getattr(g.value(node, APP.order), "value", 0)),
        )
        plans = []
        for controller in authored:
            solver = g.value(controller, CSTR_HDL_EXT.solver)
            constraint = g.value(controller, CSTR_HDL.constraint)
            quantity = g.value(constraint, CSTR.quantity) if constraint is not None else None
            if not all(isinstance(node, URIRef) for node in (solver, constraint, quantity)):
                raise ValueError(
                    f"Authored controller '{controller}' needs explicit solver, constraint, "
                    "and constraint quantity relations."
                )
            view = next(g.subjects(MAP.subobject, quantity), None)
            plan = ControllerDerivation(
                handler,
                motion,
                controller,
                solver,
                constraint,
                quantity,
                view if isinstance(view, URIRef) else None,
                axes_by_controller.get(controller, ()),
            )
            plans.append(plan)
            by_solver[solver].append(plan)
        by_handler[handler] = tuple(plans)
    constraint_counts = collections.Counter(
        plan.constraint for plans in by_handler.values() for plan in plans
    )
    solver_nodes = set(by_solver) | set(
        g.subjects(RDF.type, SLV.SolverWithInputAndOutput)
    )
    semantics = {solver: _resolve_solver_semantics(g, solver) for solver in solver_nodes}
    _validate_solver_derivations(g, by_handler, by_solver, semantics)
    return SolverDerivationContext(
        controllers_by_handler=by_handler,
        controllers_by_solver={node: tuple(plans) for node, plans in by_solver.items()},
        semantics_by_solver=semantics,
        shared_constraints=frozenset(
            constraint for constraint, count in constraint_counts.items() if count > 1
        ),
        iris=iris,
    )


def _resolve_solver_semantics(g, solver: URIRef) -> SolverSemantics:
    """Resolve a solver resource to explicit input semantics or reject it."""
    if SLV_EXT.CommandForwardingSolver in get_node_types(g, solver):
        return COMMAND_FORWARDING_SEMANTICS
    algorithm = g.value(solver, SLV["solver"])
    try:
        return SOLVER_SEMANTICS_BY_ALGORITHM[algorithm]
    except KeyError as exc:
        raise ValueError(f"Solver '{solver}' has unsupported algorithm '{algorithm}'.") from exc


def _validate_solver_derivations(g, by_handler, by_solver, semantics) -> None:
    """Enforce executable solver limits after authored RDF has resolved to solver plans."""
    for solver, plans in by_solver.items():
        if semantics[solver].acceleration_input != AccelerationInputKind.ConstraintEnergy:
            continue
        axes = [axis for plan in plans for axis in plan.axes]
        duplicates = [axis for axis, count in collections.Counter(axes).items() if count > 1]
        if duplicates:
            rendered = ", ".join(f"{axis.subspace}.{axis.axis}" for axis in duplicates)
            raise ValueError(f"ACHD solver '{solver}' repeats acceleration axis: {rendered}.")
        if len(axes) > 6:
            raise ValueError(f"ACHD solver '{solver}' has {len(axes)} axes; at most 6 are supported.")

    for handler, plans in by_handler.items():
        domains: dict[URIRef, set[str]] = collections.defaultdict(set)
        for plan in plans:
            command = str(g.value(plan.controller, APP["command-type"]) or "")
            subspace = _term_name(g.value(plan.view, MAP.subspace)) if plan.view else None
            domains[semantics[plan.solver].acceleration_input].add(
                "force" if command in {"Force", "Torque"} or subspace in {"force", "torque"} else "pose"
            )
        achd = domains[AccelerationInputKind.ConstraintEnergy]
        rne = domains[AccelerationInputKind.CartesianAcceleration]
        overlap = achd & rne
        if overlap:
            raise ValueError(
                f"Handler '{handler}' assigns ACHD and RNE to the same domain(s): "
                f"{', '.join(sorted(overlap))}."
            )


def _derived_quantity(
    id_: str, kind: str, unit: str, *, has_view: bool = False
) -> Quantity:
    """Construct a runtime scalar that is implied rather than authored."""
    return Quantity(
        id_,
        QuantityKind(kind),
        Unit(unit),
        None,
        has_view,
        provenance=Provenance(),
    )


def _motion_suffix(p, motion: URIRef) -> str:
    """Return the compatibility motion suffix from its authored motion resource."""
    motion_id = p.id(motion)
    return motion_id.removeprefix("motion_")


def _controller_signal_id(
    g, p, context: SolverDerivationContext, plan: ControllerDerivation
) -> str:
    """Derive a scalar controller output ID from its authored command semantics."""
    controller_id = p.id(plan.controller)
    types = get_node_types(g, plan.controller)
    command_type = str(g.value(plan.controller, APP["command-type"]) or "")
    if CSTR_HDL_EXT.FeedForwardController in types:
        return f"cmd_{controller_id}"
    if CSTR_HDL.ImpedanceController in types or command_type == "Force":
        return f"force_{controller_id}"
    target = g.value(plan.view, MAP.superobject) if plan.view is not None else plan.quantity
    if command_type == "Torque" and KC_STAT.JointPositionCoordinate in get_node_types(g, target):
        return f"tau_{controller_id}"
    quantity_id = p.id(plan.quantity)
    suffix = "" if plan.constraint in context.shared_constraints else f"_{_motion_suffix(p, plan.motion)}"
    semantics = context.semantics_by_solver[plan.solver]
    if semantics.signal_prefix is None:
        raise ValueError(f"Solver '{plan.solver}' does not accept acceleration signals.")
    return f"{semantics.signal_prefix}_{quantity_id}{suffix}"


def _acceleration_signal(
    id_: str, axis: SpatialAxis, input_kind: AccelerationInputKind
) -> Quantity:
    """Build an ACHD acceleration-energy or Cartesian acceleration signal."""
    if input_kind == AccelerationInputKind.ConstraintEnergy:
        return _derived_quantity(id_, "AccelerationEnergy", "N_M2_PER_SEC2")
    if input_kind == AccelerationInputKind.CartesianAcceleration:
        if axis.subspace == "linear-acceleration":
            return _derived_quantity(id_, "LinearAcceleration", "M_PER_SEC2")
        return _derived_quantity(id_, "AngularAcceleration", "RAD_PER_SEC2")
    raise ValueError(f"Input kind '{input_kind}' does not carry acceleration.")


def _saturation_for_signal(g, p, node, signal):
    """Parse authored saturation bounds and bind them to a derived signal."""
    if node is None:
        return None
    maximum = g.value(node, ALGO_EXT["maximum-absolute-value"])
    lower = g.value(node, ALGO_EXT["lower-bound"])
    upper = g.value(node, ALGO_EXT["upper-bound"])
    return Saturation(
        p.id(node),
        signal,
        signal,
        p.quantity(maximum) if maximum is not None else None,
        p.quantity(lower) if lower is not None else None,
        p.quantity(upper) if upper is not None else None,
    )


def _controller_saturations(g, p, controller, signal):
    """Bind authored output and integral bounds to a derived controller signal."""
    nodes = list(g.objects(controller, ALGO_EXT.limits))
    output_node = next(
        (
            node
            for node in nodes
            if (input_node := g.value(node, ALGO_EXT["in"])) is not None
            and g.value(input_node, QUDT_SCHEMA.hasQuantityKind) is not None
        ),
        None,
    )
    integral_node = next((node for node in nodes if node != output_node), None)
    return (
        _saturation_for_signal(g, p, output_node, signal),
        _saturation_for_signal(g, p, integral_node, signal),
    )


def _derived_controller(
    g,
    p,
    context: SolverDerivationContext,
    plan: ControllerDerivation,
    axis: SpatialAxis | None = None,
):
    """Build a controller dataclass from one authored controller and optional pose axis."""
    source_id = p.id(plan.controller)
    ids = SolverIdFactory(source_id, _motion_suffix(p, plan.motion))
    _register_derived_family(context.iris, plan.controller, ids, plan.axes)
    input_kind = context.semantics_by_solver[plan.solver].acceleration_input
    types = get_node_types(g, plan.controller)
    if axis is not None:
        controller_id = ids.component_controller(axis)
        signal = _acceleration_signal(
            (
                ids.component_energy(axis)
                if input_kind == AccelerationInputKind.ConstraintEnergy
                else ids.component_acceleration(axis)
            ),
            axis,
            input_kind,
        )
        is_linear = axis.subspace == "linear-acceleration"
        error = _derived_quantity(
            ids.component_error(axis),
            "Length" if is_linear else "Angle",
            "M" if is_linear else "RAD",
            has_view=True,
        )
        measured_source = g.value(plan.controller, CSTR_HDL["measured-velocity"])
        measured_derivative = (
            _derived_quantity(
                ids.component_measured_derivative(axis),
                "LinearVelocity" if is_linear else "AngularVelocity",
                "M_PER_SEC" if is_linear else "RAD_PER_SEC",
                has_view=True,
            )
            if measured_source is not None
            else None
        )
    else:
        controller_id = source_id
        signal_id = _controller_signal_id(g, p, context, plan)
        command_type = str(g.value(plan.controller, APP["command-type"]) or "")
        if CSTR_HDL_EXT.FeedForwardController in types:
            source = p.quantity(plan.quantity)
            signal = replace(
                source,
                id=signal_id,
                value=None,
                has_view=False,
                provenance=Provenance(),
                reference_value=None,
            )
        elif CSTR_HDL.ImpedanceController in types or command_type == "Force":
            signal = _derived_quantity(signal_id, "Force", "N")
        elif signal_id.startswith("tau_"):
            signal = _derived_quantity(signal_id, "Torque", "N_M")
        elif input_kind == AccelerationInputKind.CartesianAcceleration and plan.axes:
            signal = _acceleration_signal(signal_id, plan.axes[0], input_kind)
        else:
            signal = _derived_quantity(signal_id, "AccelerationEnergy", "N_M2_PER_SEC2")
        error_node = g.value(plan.controller, CSTR_HDL["error-signal"])
        error = p.quantity(error_node) if error_node is not None else None
        measured_node = g.value(plan.controller, CSTR_HDL["measured-velocity"])
        measured_derivative = p.quantity(measured_node) if measured_node is not None else None

    output_saturation, integral_saturation = _controller_saturations(g, p, plan.controller, signal)
    # The band belongs to the constraint this controller serves, so its logged verdict is
    # taken against the same one the monitor on that constraint uses.
    band_node = g.value(plan.constraint, CSTR_EXT["tolerance"])
    tolerance_id = p.id(band_node) if band_node is not None else ""

    if CSTR_HDL.ProportionalIntegralDerivative in types:
        decay_rate = None
        if CSTR_HDL.DecayingIntegralTerm in types:
            decay_rate = g.value(plan.controller, CSTR_HDL["decay-rate"]).value
        return PIDController(
            id=controller_id,
            control_signal=signal,
            error_signal=error,
            measured_derivative=measured_derivative,
            proportional_gain=p._required_float(plan.controller, CSTR_HDL["proportional-gain"]),
            integral_gain=p._required_float(plan.controller, CSTR_HDL["integral-gain"]),
            derivative_gain=p._required_float(plan.controller, CSTR_HDL["derivative-gain"]),
            decay_rate=decay_rate,
            output_saturation=output_saturation,
            integral_saturation=integral_saturation,
            tolerance_id=tolerance_id,
            type=p.id(CSTR_HDL.ProportionalIntegralDerivative),
        )
    if CSTR_HDL.ImpedanceController in types:
        return ImpedanceController(
            id=controller_id,
            control_signal=signal,
            error_signal=error,
            stiffness=p._required_float(plan.controller, CSTR_HDL.stiffness),
            damping=p._required_float(plan.controller, CSTR_HDL.damping),
            integral_gain=p._optional_float(plan.controller, CSTR_HDL["integral-gain"]),
            output_saturation=output_saturation,
            tolerance_id=tolerance_id,
            type=p.id(CSTR_HDL.ImpedanceController),
        )
    reference_node = g.value(plan.controller, CSTR_HDL_EXT["reference-signal"])
    return FeedForwardController(
        id=controller_id,
        control_signal=signal,
        reference_signal=p.quantity(reference_node) if reference_node is not None else None,
        output_saturation=output_saturation,
        tolerance_id=tolerance_id,
        type=p.id(CSTR_HDL_EXT.FeedForwardController),
    )


def _derived_controllers(g, p, context, plan: ControllerDerivation):
    """Expand a pose command per axis and leave scalar commands singular."""
    if len(plan.axes) > 1:
        return [_derived_controller(g, p, context, plan, axis) for axis in plan.axes]
    return [_derived_controller(g, p, context, plan)]


def _derived_acceleration_constraints(g, p, context, plan: ControllerDerivation):
    """Build ordered ACHD acceleration-energy constraints for one controller."""
    ids = SolverIdFactory(p.id(plan.controller), _motion_suffix(p, plan.motion))
    _register_derived_family(context.iris, plan.controller, ids, plan.axes)
    target = g.value(plan.view, MAP.superobject) if plan.view is not None else plan.quantity
    frame_node = g.value(target, GEOM_COORD["as-seen-by"])
    frame = p.frame(frame_node) if frame_node is not None else None
    result = []
    for axis in plan.axes:
        if len(plan.axes) > 1:
            constraint_id = ids.component_constraint(axis)
            energy_id = ids.component_energy(axis)
        else:
            suffix = (
                ""
                if plan.constraint in context.shared_constraints
                else f"_{_motion_suffix(p, plan.motion)}"
            )
            constraint_id = f"acc_cstr_{p.id(plan.quantity)}{suffix}"
            energy_id = f"eacc_{p.id(plan.quantity)}{suffix}"
            # Single-axis: the id is built off the quantity, not the controller, so it is the
            # quantity these derive from.
            for derived_id, tag in ((constraint_id, "acc-cstr"), (energy_id, "eacc")):
                context.iris.register(
                    derived_id,
                    str(plan.quantity),
                    f"{tag}{_kebab(suffix)}",
                    DerivedIriRegistry.DERIVATION,
                )
        result.append(
            AccelerationConstraint(
                id=constraint_id,
                subspace=(
                    Subspace.Linear
                    if axis.subspace == "linear-acceleration"
                    else Subspace.Angular
                ),
                axis=_AXIS_BY_FRAME_AXIS.get(axis.frame_axis),
                acceleration_energy=_derived_quantity(
                    energy_id, "AccelerationEnergy", "N_M2_PER_SEC2"
                ),
                as_seen_by=frame,
                direction=p.direction(axis.direction) if axis.direction is not None else None,
            )
        )
    return result


def _derived_cartesian_accelerations(g, p, context, plan: ControllerDerivation):
    """Build Cartesian acceleration commands resolved to joint acceleration before RNEA."""
    ids = SolverIdFactory(p.id(plan.controller), _motion_suffix(p, plan.motion))
    _register_derived_family(context.iris, plan.controller, ids, plan.axes)
    target = g.value(plan.view, MAP.superobject) if plan.view is not None else plan.quantity
    frame_node = g.value(target, GEOM_COORD["as-seen-by"])
    frame = p.frame(frame_node) if frame_node is not None else None
    result = []
    for axis in plan.axes:
        if len(plan.axes) > 1:
            specification_id = ids.component_acceleration_specification(axis)
            acceleration_id = ids.component_acceleration(axis)
        else:
            suffix = (
                ""
                if plan.constraint in context.shared_constraints
                else f"_{_motion_suffix(p, plan.motion)}"
            )
            specification_id = f"cart_acc_{p.id(plan.quantity)}{suffix}"
            acceleration_id = f"acc_{p.id(plan.quantity)}{suffix}"
            for derived_id, tag in ((specification_id, "cart-acc"), (acceleration_id, "acc")):
                context.iris.register(
                    derived_id,
                    str(plan.quantity),
                    f"{tag}{_kebab(suffix)}",
                    DerivedIriRegistry.DERIVATION,
                )
        result.append(
            CartesianAccelerationSpecification(
                id=specification_id,
                subspace=(
                    Subspace.Linear
                    if axis.subspace == "linear-acceleration"
                    else Subspace.Angular
                ),
                axis=_AXIS_BY_FRAME_AXIS.get(axis.frame_axis),
                acceleration=_acceleration_signal(
                    acceleration_id, axis, AccelerationInputKind.CartesianAcceleration
                ),
                as_seen_by=frame,
                direction=p.direction(axis.direction) if axis.direction is not None else None,
            )
        )
    return result


def _derived_motion_drivers(g, p, context, solver: URIRef) -> list[MotionDrivers]:
    """Build a solver's drivers from authored controllers plus authored force specs."""
    plans = context.controllers_by_solver.get(solver, ())
    input_kind = context.semantics_by_solver[solver].acceleration_input
    if input_kind == AccelerationInputKind.CartesianAcceleration:
        constraints = []
        accelerations = [
            acceleration
            for plan in plans
            for acceleration in _derived_cartesian_accelerations(g, p, context, plan)
        ]
    elif input_kind == AccelerationInputKind.ConstraintEnergy:
        constraints = [
            constraint
            for plan in plans
            for constraint in _derived_acceleration_constraints(g, p, context, plan)
        ]
        accelerations = []
    else:
        constraints = []
        accelerations = []
    result = []
    for driver in g.objects(solver, SLV["motion-drivers"]):
        cartesian = [
            p.cartesian_force_specification(node)
            for node in g.objects(driver, SLV["cartesian-force"])
        ]
        joint = [
            p.joint_force_specification(node)
            for node in g.objects(driver, SLV["joint-force"])
        ]
        result.append(
            MotionDrivers(
                id=p.id(driver),
                acceleration_constraint=constraints,
                cartesian_force=cartesian,
                cartesian_acceleration=accelerations,
                joint_force=joint,
                has_cartesian_force=bool(cartesian),
            )
        )
    return result


def _derive_solver_closures(g, p, context, closures: dict) -> None:
    """Replace graph-expanded controller closures with authored semantic derivations."""
    for plans in context.controllers_by_handler.values():
        for plan in plans:
            source_id = p.id(plan.controller)
            ids = SolverIdFactory(source_id, _motion_suffix(p, plan.motion))
            _register_derived_family(context.iris, plan.controller, ids, plan.axes)
            controllers = _derived_controllers(g, p, context, plan)
            closures.pop(source_id, None)
            for controller in controllers:
                closures.pop(controller.id, None)
                closures[controller.id] = {
                    "id": controller.id,
                    "type": "Controller",
                    "error_signal": getattr(getattr(controller, "error_signal", None), "id", None),
                    "reference_signal": getattr(
                        getattr(controller, "reference_signal", None), "id", None
                    ),
                    "measured_derivative": getattr(
                        getattr(controller, "measured_derivative", None), "id", None
                    ),
                    "control_signal": controller.control_signal.id,
                }
            if len(plan.axes) <= 1:
                continue
            target = g.value(plan.view, MAP.superobject)
            reference = g.value(plan.constraint, CSTR["reference-value"])
            reference_view = next(g.subjects(MAP.subobject, reference), None)
            if reference_view is not None:
                reference = g.value(reference_view, MAP.superobject)
            closures[ids.pose_evaluator()] = {
                "id": ids.pose_evaluator(),
                "type": "PoseDiffEvaluator",
                "in1": p.id(target),
                "in2": p.id(reference),
                "out": ids.pose_difference(),
                # Only the difference is computed at runtime; without materializing its per-axis
                # views the progress gate and the logged quantities read a never-written 0.0.
                "errors": [ids.component_error(axis) for axis in plan.axes],
            }


def _derive_solver_data(g, p, context, data: list, views: dict) -> None:
    """Add pose-command runtime quantities and views without RDF materialization."""
    differences = []
    errors = []
    signals = []
    derivatives = []
    derived_ids = set()
    for plans in context.controllers_by_handler.values():
        for plan in plans:
            controllers = _derived_controllers(g, p, context, plan)
            signals.extend(controller.control_signal for controller in controllers)
            derived_ids.update(controller.control_signal.id for controller in controllers)
            if len(plan.axes) <= 1:
                continue
            ids = SolverIdFactory(p.id(plan.controller), _motion_suffix(p, plan.motion))
            _register_derived_family(context.iris, plan.controller, ids, plan.axes)
            target = g.value(plan.view, MAP.superobject)
            frame_node = g.value(target, GEOM_COORD["as-seen-by"])
            difference = PoseDifference(
                ids.pose_difference(),
                ["Angle", "Length"],
                Point(f"point_{ids.pose_difference()}_origin"),
                p.frame(frame_node),
                ["M", "RAD"],
                provenance=Provenance(),
            )
            differences.append(difference)
            derived_ids.add(difference.id)
            for axis in plan.axes:
                is_linear = axis.subspace == "linear-acceleration"
                error = _derived_quantity(
                    ids.component_error(axis),
                    "Length" if is_linear else "Angle",
                    "M" if is_linear else "RAD",
                    has_view=True,
                )
                errors.append(error)
                derived_ids.add(error.id)
                views.pop(error.id, None)
                views[error.id] = View(
                    f"view_{error.id}",
                    difference,
                    error,
                    Subspace.Linear if is_linear else Subspace.Angular,
                    _AXIS_BY_FRAME_AXIS.get(axis.frame_axis),
                    direction=(
                        p.direction(axis.direction) if axis.direction is not None else None
                    ),
                )
                measured_source = g.value(plan.controller, CSTR_HDL["measured-velocity"])
                if measured_source is not None:
                    derivative = _derived_quantity(
                        ids.component_measured_derivative(axis),
                        "LinearVelocity" if is_linear else "AngularVelocity",
                        "M_PER_SEC" if is_linear else "RAD_PER_SEC",
                        has_view=True,
                    )
                    derivatives.append(derivative)
                    derived_ids.add(derivative.id)
                    views.pop(derivative.id, None)
                    views[derivative.id] = View(
                        f"view_{derivative.id}",
                        p.velocity_twist(measured_source),
                        derivative,
                        Subspace.Linear if is_linear else Subspace.Angular,
                        _AXIS_BY_FRAME_AXIS.get(axis.frame_axis),
                        direction=(
                            p.direction(axis.direction) if axis.direction is not None else None
                        ),
                    )

    data[:] = [item for item in data if item.id not in derived_ids]
    wrench_index = next(
        (index for index, item in enumerate(data) if item.type == "Wrench"), len(data)
    )
    data[wrench_index:wrench_index] = differences
    data.extend(errors)
    data.extend(derivatives)
    data.extend(signals)


def parse_argument(g, closure_id, argument, to_id, resolve_value=False):
    # For each of the key differentiate if there is one or more associated value
    """Resolve a closure argument (input/output/parameter) from the graph to id(s); a lone value
    collapses to a scalar and qudt:Quantity constants resolve to their scalar value.
    """
    entry = list(g[closure_id:argument])
    if len(entry) == 0:
        return None

    def resolve(e):
        # Parameters are baked into generated code as literal text, so a qudt:Quantity-wrapped
        # constant must resolve to its scalar qudt:value here (bare literals / IRI refs pass through).
        """Resolve one graph value to its id, unwrapping a qudt:Quantity constant to its scalar."""
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


def _reference_inputs(g, node):
    """Inputs a referenced data structure contributes to whoever reads it.

    A path is geometry: it has parameters but produces nothing, so it can never be found as
    a producer by the output-to-input walk. Its parameters are inputs of the operator that
    traverses it, and are yielded here so the walk reaches them.
    """
    if GEOM_PATH.Path not in get_node_types(g, node):
        return ()
    return tuple(obj for pred, obj in g.predicate_objects(node) if pred != RDF["type"])


@dataclass
class Operator:
    """A schedulable RDF computation mapping graph inputs/outputs/parameters to a closure and
    participating in output-to-input scheduling.
    """

    type_: URIRef
    input: list[URIRef]
    output: list[URIRef]
    parameters: list = field(default_factory=list)
    schedulable: bool = True

    def closure_step(self, g, to_id, closure_id):
        """Build the closure dict for this operator's call at closure_id."""
        closure = {"id": to_id(closure_id), "type": to_id(self.type_)}

        for input in self.input:
            closure[to_id(input)] = parse_argument(g, closure_id, input, to_id)
        for output in self.output:
            closure[to_id(output)] = parse_argument(g, closure_id, output, to_id)
        for param in self.parameters:
            closure[to_id(param)] = parse_argument(g, closure_id, param, to_id, resolve_value=True)

        return closure

    def from_operator_to_input(self, g, operator_id):
        """Data-structure nodes feeding this operator call's inputs."""
        data_structures = set()

        for in_ in self.input:
            for data_in in g.objects(operator_id, in_):
                data_structures.add(data_in)
                data_structures.update(_reference_inputs(g, data_in))

        return data_structures

    def scheduler_step(self, g, data_out):
        """Input data structures and schedulable calls producing data_out for this operator."""
        data_structures = set()
        schedule = []

        for out in self.output:
            # sorted(): see Parser.schedule -- these come back unordered and the append order
            # below becomes the emitted schedule order.
            for call in sorted(g[:out:data_out]):
                if self.type_ not in get_node_types(g, call):
                    continue

                # We will only record this call if it has any input
                has_any_input = False

                for in_ in self.input:
                    for data_in in g.objects(call, in_):
                        data_structures.add(data_in)
                        data_structures.update(_reference_inputs(g, data_in))
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
    schedulable: bool = False

    def from_operator_to_input(self, g, operator_id):
        """Data-structure nodes feeding this specification's inputs."""
        data_structures = set()

        for in_ in self.input:
            for data_in in g.objects(operator_id, in_):
                data_structures.add(data_in)
                data_structures.update(_reference_inputs(g, data_in))

        return data_structures

    def scheduler_step(self, g, data_out):
        """Input data structures for this specification (never schedulable)."""
        data_structures = set()
        for out in self.output:
            for call in g.subjects(out, data_out):
                for in_ in self.input:
                    for data_in in g.objects(call, in_):
                        data_structures.add(data_in)

        return {"data_structures": data_structures, "schedule": []}


class ErrorEvaluator:
    """Constraint-handler operator that emits a constraint error signal, dispatching on the
    constraint type (equality/greater/less/bilateral/outside).
    """

    def __init__(self):
        """Register the constraint operators this error evaluator dispatches over."""
        self.type_ = CSTR_HDL["ErrorEvaluator"]
        self.schedulable = False
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
        """Build the error-evaluator closure, dispatching on the constraint type."""
        constraint_id = g.value(closure_id, CSTR_HDL["constraint"])

        for operator in self.cstr_op:
            if operator.type_ not in get_node_types(g, constraint_id):
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
                closure[to_id(param)] = parse_argument(
                    g, closure_id, param, to_id, resolve_value=True
                )

            # Only return the first matching type
            return closure

        # Should never happen
        return None

    def from_operator_to_input(self, g, operator_id):
        """Data-structure nodes feeding the matching constraint's inputs."""
        data_structures = set()

        constraint_id = g.value(operator_id, CSTR_HDL["constraint"])
        constraint_types = get_node_types(g, constraint_id) if constraint_id is not None else set()
        for op in self.cstr_op:
            if op.type_ not in constraint_types:
                continue

            for in_ in op.input:
                for data_in in g.objects(operator_id, CSTR_HDL["constraint"] / in_):
                    data_structures.add(data_in)

        return data_structures

    def scheduler_step(self, g, data_out):
        """Input data structures and schedulable calls producing the error data_out."""
        data_structures = set()
        schedule = []

        for op in self.cstr_op:
            for out in op.output:
                # sorted(): see Parser.schedule -- append order becomes schedule order.
                for call in sorted(g.subjects(out, data_out)):
                    constraint_id = g.value(call, CSTR_HDL["constraint"])
                    constraint_types = (
                        get_node_types(g, constraint_id) if constraint_id is not None else set()
                    )
                    if op.type_ not in constraint_types:
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
    """Constraint operator that assigns a reference value to a quantity (no error output)."""

    def __init__(self):
        """Register the equality-constraint operator this assignment evaluator uses."""
        self.type_ = CSTR_HDL["AssignmentEvaluator"]
        self.schedulable = True
        self.cstr_op = Operator(
            type_=CSTR["EqualityConstraint"],
            input=[CSTR["quantity"], CSTR["reference-value"]],
            output=[],
        )

    def closure_step(self, g, to_id, closure_id):
        """Build the assignment-evaluator closure (equality constraint only)."""
        constraint_id = g.value(closure_id, CSTR_HDL["constraint"])

        if self.cstr_op.type_ not in get_node_types(g, constraint_id):
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
        """Data-structure nodes feeding the assignment's inputs."""
        constraint_id = g.value(operator_id, CSTR_HDL["constraint"])
        constraint_types = (
            get_node_types(g, constraint_id) if constraint_id is not None else set()
        )
        if self.cstr_op.type_ not in constraint_types:
            return set()

        data_structures = set()
        for in_ in self.cstr_op.input:
            for data_in in g.objects(operator_id, CSTR_HDL["constraint"] / in_):
                data_structures.add(data_in)

        return data_structures


def _op_output_preds(op):
    # Predicates op.scheduler_step queries against data_out; if none point into a
    # node, scheduler_step can only return empty, so it is safe to skip.
    """Output predicates an operator's scheduler queries; empty when the operator can never match,
    so its scheduler step can be skipped.
    """
    if isinstance(op, ErrorEvaluator):
        return {p for sub in op.cstr_op for p in sub.output}
    if isinstance(op, AssignmentEvaluator):
        return set()
    return set(op.output)


# A path is geometry: data with no output, so it is never found by the output-to-input walk
# and yields no closure of its own. The evaluator that traverses it is the computation, and
# folds the path's geometry into its call.
ops_path = [
    Specification(
        type_=GEOM_PATH["LinearPath"],
        input=[GEOM_PATH["start"], GEOM_PATH["goal"]],
        output=[],
    ),
    Specification(
        type_=GEOM_PATH["Circle"],
        input=[GEOM_PATH["start"], GEOM_PATH["center"], GEOM_PATH["plane-normal"]],
        output=[],
    ),
    Specification(
        type_=GEOM_PATH["Arc"],
        input=[
            GEOM_PATH["start"],
            GEOM_PATH["end"],
            GEOM_PATH["amplitude"],
            GEOM_PATH["plane-normal"],
        ],
        output=[],
    ),
    Specification(
        type_=GEOM_PATH["Helix"],
        input=[
            GEOM_PATH["start"],
            GEOM_PATH["center"],
            GEOM_PATH["axis"],
            GEOM_PATH["pitch"],
            GEOM_PATH["revolutions"],
        ],
        output=[],
    ),
    Specification(
        type_=GEOM_PATH["Figure8"],
        input=[
            GEOM_PATH["anchor"],
            GEOM_PATH["radius"],
            GEOM_PATH["plane-normal"],
            GEOM_PATH["direction"],
        ],
        output=[],
        parameters=[GEOM_PATH["form"]],
    ),
]


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
    Operator(type_=GEOM_OP["InvertPose"], input=[GEOM_OP["pose"]], output=[GEOM_OP["out"]]),
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
        type_=ALGO_EXT.Addition,
        input=[ALGO_EXT["in"]],
        output=[ALGO_EXT.out],
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
    Specification(type_=MAP["View"], input=[MAP["superobject"]], output=[MAP["subobject"]]),
    *ops_path,
    Operator(
        type_=GEOM_OP_EXT.PathProjection,
        input=[GEOM_OP_EXT.path, GEOM_OP["pose"]],
        output=[GEOM_OP_EXT["path-parameter"]],
    ),
    Operator(
        type_=GEOM_OP_EXT.PathTangentFrame,
        input=[GEOM_OP_EXT.path, GEOM_OP_EXT["path-parameter"]],
        output=[GEOM_OP_EXT.tangent, GEOM_OP_EXT["normal-a"], GEOM_OP_EXT["normal-b"]],
    ),
    Operator(
        type_=GEOM_OP_EXT.TwistToLinearVelocityAlong,
        input=[GEOM_OP["in"], GEOM_OP["direction"]],
        output=[GEOM_OP_EXT["along-speed"]],
    ),
    Operator(
        type_=GEOM_OP_EXT.PathEvaluator,
        input=[GEOM_OP_EXT.path, GEOM_OP_EXT["path-parameter"]],
        output=[GEOM_OP["out"]],
    ),
    Operator(
        type_=ALGO_EXT["VelocityProfile"],
        input=[
            ALGO_EXT["target"],
            ALGO_EXT["in"],
            ALGO_EXT["maximum-velocity"],
            ALGO_EXT["maximum-acceleration"],
            ALGO_EXT["maximum-jerk"],
        ],
        output=[ALGO_EXT["out"]],
        parameters=[ALGO_EXT["shape"]],
    ),
    Operator(
        type_=ALGO_EXT["Admittance"],
        input=[ALGO_EXT["in"]],
        output=[ALGO_EXT["out"]],
        parameters=[
            ALGO_EXT["mass"],
            ALGO_EXT["damping"],
            ALGO_EXT["stiffness"],
            CSTR_HDL["maximum-velocity"],
        ],
    ),
]

ops_cstr_hdl = [
    AssignmentEvaluator(),
    ErrorEvaluator(),
]

ops_slv = [
    Specification(type_=SLV["CartesianForceSpecification"], input=[SLV["force"]], output=[]),
    Specification(type_=SLV["JointForceSpecification"], input=[SLV["force"]], output=[]),
    Specification(type_=SLV["ForceDistributionSolver"], input=[SLV["force"]], output=[]),
]


# ---------------------------------------------------------------------------
# RDF parsing
# ---------------------------------------------------------------------------
def memoize(func):
    """Decorator caching a Parser method's result per instance, keyed by its arguments."""

    @wraps(func)
    def decorator(self, *args, **kwargs):
        # Scope by func identity so e.g. position(uri) and quantity(uri)
        # don't collide on the same (uri,) cache key.
        """Return the cached result, computing and storing it on first call."""
        key = (func.__qualname__,) + args + tuple(kwargs.items())
        if key not in self.cache:
            self.cache[key] = func(self, *args, **kwargs)
        return self.cache[key]

    return decorator


def escape(s):
    """Return a graph-safe identifier for a local name."""
    s = re.sub(r"[^0-9A-Za-z_]", "_", str(s))
    if s and s[0].isdigit():
        s = f"_{s}"
    return s


class Parser:
    # A context quantity's id is its URI's last segment, so a name reused across motions' specs
    # (e.g. `support-z`) collapses to one id. Qualify only ambiguous names (same segment, >1 owner).
    """Parses a motion-spec RDF dataset into IR pieces: ids, closures, views, data structures,
    handlers and solvers.
    """

    # scan depends only on the graph; memoize per-graph
    _ambiguous_cache: "weakref.WeakKeyDictionary" = weakref.WeakKeyDictionary()

    def __init__(self, g):
        """Bind the parser to an RDF graph and initialize its memoization cache."""
        self.cache = dict()
        self.g = g
        self.sched = set()
        cached = Parser._ambiguous_cache.get(g)
        if cached is None:
            cached = self._compute_ambiguous_context_ids()
            Parser._ambiguous_cache[g] = cached
        self._ambiguous_context_ids = cached
        self._id_sources: dict[str, set[str]] = {}
        self._id_cache: dict = {}

    def _expect_type(self, id_, type_):
        """Raise if `id_` lacks the expected rdf:type. Replaces bare asserts so
        the check survives `python -O` and names the offending node."""
        if type_ not in get_node_types(self.g, id_):
            raise ConstraintViolation(
                "motion-spec", f"Node '{id_}' is missing expected rdf:type '{type_}'"
            )

    def _compute_ambiguous_context_ids(self):
        """Local names shared by more than one node, which need model-scope prefixing."""
        sources_by_id: dict[str, set[str]] = {}
        for s in set(self.g.subjects()):
            if self._context_scope(s) is None:
                continue
            try:
                local = escape(self.g.compute_qname(s)[2])
            except Exception:
                continue
            sources_by_id.setdefault(local, set()).add(str(s))
        return {lid for lid, sources in sources_by_id.items() if len(sources) > 1}

    @staticmethod
    def _context_scope(node) -> tuple[str, str, tuple[str, ...]] | None:
        """Return the owner, section and member path of a context quantity IRI."""
        parts = tuple(part for part in urlsplit(str(node)).path.split("/") if part)
        for index in range(1, len(parts) - 1):
            if parts[index] in ("spec", "world"):
                return parts[index - 1], parts[index], parts[index + 1 :]
        return None

    def id(self, x):
        """Stable local id for a URI/node (scoped when the bare name is ambiguous)."""
        cached = self._id_cache.get(x)
        if cached is not None:
            return cached
        try:
            q = self.g.compute_qname(x)
            local = escape(q[2])
        except Exception:
            self._id_cache[x] = x
            return x
        # Only context quantities (a motion's or the shared context's `spec` / `world`
        # members) become `shared.*` data fields and are vulnerable to the silent merge; constraint
        # names, metamodel predicates and aliases legitimately share an id and are scoped elsewhere.
        scope = self._context_scope(x)
        if scope:
            owner, _section, member_path = scope
            if local in self._ambiguous_context_ids:
                local = escape("-".join((owner, *member_path)))
            self._id_sources.setdefault(local, set()).add(str(x))
        self._id_cache[x] = local
        return local

    def assert_no_id_collisions(self):
        """Fail loudly if two distinct context-quantity URIs collapse to one generated id. That
        would silently merge unrelated `shared.*` fields (the class of bug that hid the support-z
        collision) -- a genuinely-shared quantity has a single URI, so >1 URI per id is a real
        collision the motion-qualified id scoping failed to resolve."""
        collisions = {i: sorted(u) for i, u in self._id_sources.items() if len(u) > 1}
        if collisions:
            details = "\n".join(f"  '{i}' <- {', '.join(u)}" for i, u in sorted(collisions.items()))
            raise ValueError(
                "id collision(s): distinct URIs map to one generated id and would be silently "
                "merged (e.g. a context-quantity name reused across motions with different "
                "definitions). Give them distinct names, or extend the motion-qualified id "
                f"scoping in Parser.id to cover their URI shape:\n{details}"
            )

    def label(self, x):
        """Human-readable label of a node, if the graph carries one."""
        try:
            q = self.g.compute_qname(x)
            return q[2]
        except Exception:
            return str(x)

    @memoize
    def velocity_composition_solver(self, id_):
        """Parse a VelocityCompositionSolver at node."""
        self._expect_type(id_, SLV["VelocityCompositionSolver"])
        conf = self.id(self.g.value(id_, SLV["configuration"]))
        velocity = self.velocity_twist(self.g.value(id_, SLV["velocity"]))

        return VelocityCompositionSolver(self.id(id_), conf, velocity)

    @memoize
    def force_distribution_solver(self, id_):
        """Parse a ForceDistributionSolver at node."""
        self._expect_type(id_, SLV["ForceDistributionSolver"])
        conf = self.id(self.g.value(id_, SLV["configuration"]))
        force = self.wrench(self.g.value(id_, SLV["force"]))

        return ForceDistributionSolver(self.id(id_), conf, force)

    @memoize
    def solver_with_input_and_output(self, id_):
        """Parse a SolverWithInputAndOutput (chain, algorithm, drivers, outputs) at node."""
        self._expect_type(id_, SLV["SolverWithInputAndOutput"])
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
                if type_ in get_node_types(self.g, o):
                    out.append(func(o))

        # The authored value IS the Vereshchagin root acceleration, passed to ACHD as-is.
        # The opposite-sign gravity KDL's inverse-dynamics solver wants is derived in
        # _annotate_rne_gravity rather than here.
        gravity_node = self.g.value(id_, SLV.gravity)
        root_acc = self.parse_xyz(gravity_node) if gravity_node else None
        algorithm_node = self.g.value(id_, SLV["solver"])
        try:
            algorithm = SOLVER_SEMANTICS_BY_ALGORITHM[algorithm_node].codegen_name
        except KeyError as exc:
            raise ValueError(
                f"Solver '{id_}' has unsupported algorithm '{algorithm_node}'."
            ) from exc
        torque_saturation_node = next(
            (
                limit
                for limit in self.g.objects(id_, ALGO_EXT.limits)
                if QUDT_QKIND.Torque
                in self.g[self.g.value(limit, ALGO_EXT["in"]) : QUDT_SCHEMA.hasQuantityKind]
            ),
            None,
        )

        return SolverWithInputAndOutput(
            id=self.id(id_),
            motion_drivers=drv,
            output=out,
            algorithm=algorithm,
            root_acc=root_acc,
            torque_saturation=(
                self.saturation(torque_saturation_node)
                if torque_saturation_node is not None
                else None
            ),
        )

    @memoize
    def motion_drivers(self, id_):
        """Parse authored Cartesian and joint-force drivers at node."""
        self._expect_type(id_, SLV["MotionDrivers"])
        spec_frc = []
        spec_jf = []

        for f in self.g[id_ : SLV["cartesian-force"]]:
            spec_frc.append(self.cartesian_force_specification(f))

        for jf in self.g[id_ : SLV["joint-force"]]:
            spec_jf.append(self.joint_force_specification(jf))

        return MotionDrivers(
            id=self.id(id_),
            acceleration_constraint=[],
            cartesian_force=spec_frc,
            joint_force=spec_jf,
            has_cartesian_force=bool(spec_frc),
        )

    def joint_force_specification(self, id_):
        """Parse a JointForceSpecification at node."""
        self._expect_type(id_, SLV["JointForceSpecification"])
        force_node = self.g.value(id_, SLV["force"])
        force_id = self.id(force_node) if force_node is not None else ""
        joint_node = self.g.value(id_, SLV["attached-to"])
        joint_name = self.label(joint_node) if joint_node is not None else ""
        return JointForceSpecification(self.id(id_), force_id, joint_name)

    @memoize
    def cartesian_force_specification(self, id_):
        """Parse a CartesianForceSpecification at node."""
        self._expect_type(id_, SLV["CartesianForceSpecification"])
        force = self.wrench(self.g.value(id_, SLV["force"]))
        attached_to = self.simplicial_complex(self.g.value(id_, SLV["attached-to"]))

        return CartesianForceSpecification(self.id(id_), force, attached_to)

    @memoize
    def saturation(self, id_):
        """Parse a Saturation (input/output limits) at node."""
        self._expect_type(id_, ALGO_EXT.Saturation)
        input_signal = self.quantity(self.g.value(id_, ALGO_EXT["in"]))
        output_signal = self.quantity(self.g.value(id_, ALGO_EXT.out))
        maximum_node = self.g.value(id_, ALGO_EXT["maximum-absolute-value"])
        lower_node = self.g.value(id_, ALGO_EXT["lower-bound"])
        upper_node = self.g.value(id_, ALGO_EXT["upper-bound"])
        return Saturation(
            self.id(id_),
            input_signal,
            output_signal,
            self.quantity(maximum_node) if maximum_node is not None else None,
            self.quantity(lower_node) if lower_node is not None else None,
            self.quantity(upper_node) if upper_node is not None else None,
        )

    @memoize
    def subspace(self, id_):
        """Parse the Subspace (linear/angular half) of node."""
        d = {
            MAP["position"]: Subspace.Linear,
            MAP_EXT["position"]: Subspace.Linear,
            MAP_EXT["orientation"]: Subspace.Angular,
            MAP["angular-velocity"]: Subspace.Angular,
            MAP["linear-velocity"]: Subspace.Linear,
            MAP["angular-acceleration"]: Subspace.Angular,
            MAP["linear-acceleration"]: Subspace.Linear,
            MAP["torque"]: Subspace.Angular,
            MAP["force"]: Subspace.Linear,
            SLV["angular-acceleration"]: Subspace.Angular,
            SLV["linear-acceleration"]: Subspace.Linear,
            MAP_EXT["linear"]: Subspace.Linear,
            MAP_EXT["angular"]: Subspace.Angular,
        }
        if id_ not in d:
            raise ValueError(f"unknown subspace {id_}")

        return d[id_]

    @memoize
    def axis(self, id_):
        """Parse the Axis (X/Y/Z) of node."""
        d = {
            MAP["x"]: Axis.X,
            MAP["y"]: Axis.Y,
            MAP["z"]: Axis.Z,
            MAP["w"]: Axis.W,
            SLV["x"]: Axis.X,
            SLV["y"]: Axis.Y,
            SLV["z"]: Axis.Z,
        }
        if id_ not in d:
            raise ValueError(f"unknown axis {id_}")

        return d[id_]

    def constraint_handler(self, id_):
        """Parse a ConstraintHandler (evaluators, controllers, monitors) at node."""
        self._expect_type(id_, CSTR_HDL["ConstraintHandler"])
        motion = self.guarded_motion(self.g.value(id_, CSTR_HDL["motion"]))
        evaluators = []
        for e in self.g[id_ : CSTR_HDL["evaluators"]]:
            evaluators.append(self.constraint_evaluator(e))

        monitors = []
        for m in self.g[id_ : CSTR_HDL["monitors"]]:
            monitors.append(self.monitor_entry(m))

        order_value = self.g.value(id_, APP["order"])
        order = int(order_value.value) if order_value is not None else 0

        return ConstraintHandler(self.id(id_), motion, evaluators, [], monitors, order)

    @memoize
    def monitor_entry(self, id_):
        """Parse a monitor (level flag or edge event) at node."""
        self._expect_type(id_, CSTR_HDL["Monitor"])
        handler = next(self.g.subjects(CSTR_HDL.monitors, id_), None)
        motion = self.g.value(handler, CSTR_HDL.motion) if handler is not None else None
        monitored = set(self.g.objects(id_, CSTR_HDL.constraint))
        when = set(self.g.objects(motion, MOT.when)) if motion is not None else set()
        until = set(self.g.objects(motion, MOT.until)) if motion is not None else set()
        is_aggregate = len(monitored) > 1 or any(
            _is_constraint_aggregate(self.g, node) for node in monitored
        )
        is_until_aggregate = is_aggregate and monitored == until
        is_when_aggregate = is_aggregate and monitored == when
        # A named group is one of several until conditions, so it is not the whole section:
        # carry its members and logic instead, and let the terms be built from those.
        group_constraint_ids: list[str] = []
        group_any = False
        if not is_until_aggregate and not is_when_aggregate:
            group_node = next(
                (n for n in monitored if _is_constraint_aggregate(self.g, n)), None
            )
            if group_node is not None:
                group_constraint_ids = sorted(
                    self.id(c) for c in self.g[group_node : CSTR_EXT["has-constraint"]]
                )
                group_any = CSTR_EXT.ConstraintDisjunction in get_node_types(self.g, group_node)
        error_node = self.g.value(id_, CSTR_HDL["error"])
        error = (
            None
            if is_until_aggregate or is_when_aggregate or group_constraint_ids or error_node is None
            else self.quantity(error_node)
        )
        # The band belongs to the constraint, so a monitor carries it only when it watches one.
        tolerance_node = (
            self.g.value(next(iter(monitored)), CSTR_EXT["tolerance"])
            if error is not None and len(monitored) == 1
            else None
        )
        tolerance = self.quantity(tolerance_node) if tolerance_node is not None else None

        if CSTR_HDL["LevelTriggeredMonitor"] in get_node_types(self.g, id_):
            flag = self.id(self.g.value(id_, CSTR_HDL["flag"]))
            return LevelMonitor(
                self.id(id_),
                "LevelTriggeredMonitor",
                error,
                flag,
                tolerance=tolerance,
                is_until_aggregate=is_until_aggregate,
                is_when_aggregate=is_when_aggregate,
                group_constraint_ids=group_constraint_ids,
                group_any=group_any,
            )

        event_node = self.g.value(id_, CSTR_HDL["event"])
        event = self.id(event_node)
        fallback_node = self.g.value(id_, CSTR_HDL_EXT["fallback-motion"])
        fallback_motion = self.id(fallback_node) if fallback_node is not None else None
        debounce_duration_s = self._optional_seconds(id_, CSTR_HDL_EXT["debounce-duration"])
        ros_kwargs = {}
        ros_channel = self.g.value(id_, NS_MM_ROS["channel-name"])
        if ros_channel is not None:
            ros_type = str(self.g.value(id_, NS_MM_ROS["type-name"]) or "")
            pkg, include, cpp_type = _ros_type_parts(ros_type)
            ros_kwargs = dict(
                ros_channel=str(ros_channel),
                ros_type=ros_type,
                ros_pkg=pkg,
                ros_include=include,
                ros_cpp_type=cpp_type,
                ros_pub_id=f"{self.id(id_)}_pub".replace("-", "_"),
            )
        return EdgeMonitor(
            self.id(id_),
            "EdgeTriggeredMonitor",
            error,
            event,
            None,
            tolerance=tolerance,
            is_until_aggregate=is_until_aggregate,
            is_when_aggregate=is_when_aggregate,
            group_constraint_ids=group_constraint_ids,
            group_any=group_any,
            event_uri=str(event_node),
            event_name=event.upper(),
            fallback_motion=fallback_motion,
            debounce_duration_s=debounce_duration_s,
            **ros_kwargs,
        )

    @memoize
    def constraint_evaluator(self, id_):
        """Parse a ConstraintEvaluator (constraint, error, elapsed timing) at node."""
        self._expect_type(id_, CSTR_HDL["ConstraintEvaluator"])
        constraint_node = self.g.value(id_, CSTR_HDL["constraint"])
        constraint = self.constraint(constraint_node)

        if CSTR_HDL["AssignmentEvaluator"] in get_node_types(self.g, id_):
            t = EvaluatorType.AssignmentEvaluator
            error = None
        else:
            t = EvaluatorType.ErrorEvaluator
            error = self.quantity(self.g.value(id_, CSTR_HDL["error"]))

        # Timing constraint: measured quantity is the motion-state elapsed time. No solver
        # error — codegen compares the world clock against the threshold directly.
        is_elapsed = False
        elapsed_op = None
        elapsed_threshold_s = None
        elapsed_tolerance_s = None
        if _is_elapsed_constraint(self.g, constraint_node):
            is_elapsed = True
            types = get_node_types(self.g, constraint_node)
            if CSTR["GreaterThanConstraint"] in types:
                elapsed_op = ">="
                thr = self.g.value(constraint_node, CSTR["threshold"])
                elapsed_threshold_s = _duration_seconds(self.g, thr)
            elif CSTR["EqualityConstraint"] in types:
                elapsed_op = "=="
                thr = self.g.value(constraint_node, CSTR["reference-value"])
                elapsed_threshold_s = _duration_seconds(self.g, thr)
                tol = self.g.value(constraint_node, CSTR_EXT["tolerance"])
                elapsed_tolerance_s = _duration_seconds(self.g, tol)
            else:
                elapsed_op = "<"
                thr = self.g.value(constraint_node, CSTR["threshold"])
                elapsed_threshold_s = _duration_seconds(self.g, thr)

        # An authored band on a spatial equality; the elapsed branch reads its own above, in
        # seconds, because a duration's magnitude rides on qudt rather than on a shared value.
        tolerance_node = None if is_elapsed else self.g.value(constraint_node, CSTR_EXT["tolerance"])
        tolerance = self.quantity(tolerance_node) if tolerance_node is not None else None

        return ConstraintEvaluator(
            self.id(id_),
            t,
            constraint,
            error,
            tolerance=tolerance,
            is_elapsed=is_elapsed,
            elapsed_op=elapsed_op,
            elapsed_threshold_s=elapsed_threshold_s,
            elapsed_tolerance_s=elapsed_tolerance_s,
        )

    def _optional_float(self, subject, predicate) -> float | None:
        """Read an optional float-valued property, or None when absent."""
        value = self.g.value(subject, predicate)
        if value is None:
            return None
        literal = (
            value if isinstance(value, rdflib.Literal) else self.g.value(value, QUDT_SCHEMA.value)
        )
        if literal is None:
            raise ValueError(
                f"Controller '{self.id(subject)}' property '{self.id(predicate)}' must be a literal or a node with qudt:value."
            )
        return float(literal.value)

    def _optional_seconds(self, subject, predicate) -> float | None:
        """Read an optional duration in seconds, converting from the unit it was written in."""
        node = self.g.value(subject, predicate)
        if node is None:
            return None
        value = self._optional_float(subject, predicate)
        return None if value is None else _seconds(value, self.g.value(node, QUDT_SCHEMA.unit))

    def _required_float(self, subject, predicate) -> float:
        """Read a required float-valued property, raising when absent."""
        value = self._optional_float(subject, predicate)
        if value is None:
            raise ValueError(
                f"Controller '{self.id(subject)}' is missing required property '{self.id(predicate)}'."
            )
        return value

    @memoize
    def guarded_motion(self, id_):
        """Parse a GuardedMotion (when/while/until constraint sets) at node."""
        self._expect_type(id_, MOT["GuardedMotion"])
        when = []
        when_any = False
        for c in self.g[id_ : MOT["when"]]:
            if CSTR_EXT.ConstraintDisjunction in get_node_types(self.g, c):
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
            if _is_constraint_aggregate(self.g, c):
                # A section-wide disjunction makes the whole until 'any'; a named group keeps
                # its own logic on the monitor that targets it.
                if CSTR_EXT.ConstraintDisjunction in get_node_types(self.g, c):
                    until_any = True
                for member in self.g[c : CSTR_EXT["has-constraint"]]:
                    until.append(self.constraint(member))
            else:
                until.append(self.constraint(c))

        name_literal = self.g.value(id_, SDO.name)
        if name_literal is None:
            raise ValueError(f"GuardedMotion {id_} has no schema:name triple")
        name = str(name_literal)
        description = self.g.value(id_, SDO.description)
        return GuardedMotion(
            self.id(id_), when, while_, until, until_any, when_any,
            name=name, description=str(description) if description is not None else None,
        )

    @memoize
    def constraint(self, id_):
        """Parse a Constraint (quantity plus its parameter) at node."""
        self._expect_type(id_, CSTR["Constraint"])
        quantity = self.quantity(self.g.value(id_, CSTR["quantity"]))

        parameter = None
        if CSTR["EqualityConstraint"] in get_node_types(self.g, id_):
            parameter = self.equality_constraint(id_)
        elif CSTR["UnilateralConstraint"] in get_node_types(self.g, id_):
            parameter = self.unilateral_constraint(id_)
        elif CSTR_EXT["OutsideConstraint"] in get_node_types(self.g, id_):
            parameter = self.outside_constraint(id_)
        else:
            parameter = self.bilateral_constraint(id_)

        return Constraint(self.id(id_), quantity, parameter)

    @memoize
    def equality_constraint(self, id_):
        """Parse an EqualityConstraint at node."""
        self._expect_type(id_, CSTR["EqualityConstraint"])
        reference_value = self.quantity(self.g.value(id_, CSTR["reference-value"]))

        return EqualityConstraint(reference_value)

    @memoize
    def unilateral_constraint(self, id_):
        """Parse a UnilateralConstraint (greater/less threshold) at node."""
        self._expect_type(id_, CSTR["UnilateralConstraint"])
        threshold = self.quantity(self.g.value(id_, CSTR["threshold"]))
        type_ = UnilateralConstraintType.LessThan
        if CSTR["GreaterThanConstraint"] in get_node_types(self.g, id_):
            type_ = UnilateralConstraintType.GreaterThan

        return UnilateralConstraint(type_, threshold)

    @memoize
    def bilateral_constraint(self, id_):
        """Parse a BilateralConstraint (lower/upper threshold) at node."""
        self._expect_type(id_, CSTR["BilateralConstraint"])
        lower_threshold = self.quantity(self.g.value(id_, CSTR["lower-threshold"]))
        upper_threshold = self.quantity(self.g.value(id_, CSTR["upper-threshold"]))

        return BilateralConstraint(lower_threshold, upper_threshold)

    @memoize
    def outside_constraint(self, id_):
        """Parse an OutsideConstraint (lower/upper threshold) at node."""
        self._expect_type(id_, CSTR_EXT["OutsideConstraint"])
        lower_threshold = self.quantity(self.g.value(id_, CSTR["lower-threshold"]))
        upper_threshold = self.quantity(self.g.value(id_, CSTR["upper-threshold"]))

        return OutsideConstraint(lower_threshold, upper_threshold)

    @memoize
    def direction(self, id_):
        """Parse a Direction quantity at node."""
        self._expect_type(id_, GEOM_COORD["DirectionCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.id(k))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = self.id(self.g.value(id_, QUDT_SCHEMA["unit"]))
        direction = self.parse_xyz(id_)

        return Direction(self.id(id_), quantity_kind, as_seen_by, [Unit(unit)], direction)

    def parse_xyz(self, node):
        """Parse x/y/z scalar coordinates from node on SI, or None."""
        x = self.g.value(node, GEOM_COORD["x"])
        y = self.g.value(node, GEOM_COORD["y"])
        z = self.g.value(node, GEOM_COORD["z"])

        if x is None or y is None or z is None:
            return None

        return _si_all((v.value for v in (x, y, z)), self.g.value(node, QUDT_SCHEMA["unit"]))

    def _derived_reference_frames(self, id_):
        """Derive a reference value's frames from its RDF use or snapshot source."""
        source = next(
            (
                self.g.value(snapshot, ALGO_EXT["in"])
                for snapshot in self.g.subjects(ALGO_EXT["out"], id_)
            ),
            None,
        )
        if source is None:
            constraint = next(self.g.subjects(CSTR["reference-value"], id_), None)
            quantity = self.g.value(constraint, CSTR.quantity) if constraint is not None else None
            view = next(self.g.subjects(MAP.subobject, quantity), None)
            source = self.g.value(view, MAP.superobject) if view is not None else quantity
        if source is None:
            return None, None, None
        return (
            self.g.value(source, GEOM_REL.of),
            self.g.value(source, GEOM_REL["with-respect-to"]),
            self.g.value(source, GEOM_COORD["as-seen-by"]),
        )

    @memoize
    def position(self, id_):
        """Parse a Position quantity at node."""
        if URI_GEOM_TYPE_POSITION_COORD in get_node_types(self.g, id_):
            coordinate = PositionCoordModel(id_, self.g)
            relation = coordinate.position
        else:
            relation = PositionModel(id_, self.g)
            if len(relation.coordinate_ids) != 1:
                raise ValueError(f"Position '{id_}' needs exactly one coordinate")
            coordinate = PositionCoordModel(next(iter(relation.coordinate_ids)), self.g, relation)
        of = self.position_reference(relation.of_id)
        wrt = self.position_reference(relation.wrt_id)
        quantity_kind = self.id(self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]))
        as_seen_by = self.frame(coordinate.as_seen_by)
        length_unit = _length_unit(coordinate)
        unit = self.id(_si_unit(length_unit))
        values = get_coord_vectorxyz(coordinate, self.g)
        pos = _si_all(values, length_unit) if values is not None else None

        return Position(
            self.id(id_), of, wrt, QuantityKind(quantity_kind), as_seen_by, Unit(unit), pos
        )

    @memoize
    def orientation(self, id_):
        """Parse an Orientation quantity at node."""
        if URI_GEOM_TYPE_ORIENT_COORD in get_node_types(self.g, id_):
            coordinate = OrientCoordModel(id_, self.g)
            relation = coordinate.relation
        else:
            relation = OrientationModel(id_, self.g)
            if len(relation.coordinate_ids) != 1:
                raise ValueError(f"Orientation '{id_}' needs exactly one coordinate")
            coordinate = OrientCoordModel(next(iter(relation.coordinate_ids)), self.g, relation)

        def optional_pose_ref(node):
            """Resolve an optional pose reference (endpoint or bare pose) at node."""
            if node is None:
                return None
            if ENV.RigidObject in get_node_types(self.g, node):
                return self.scene_object(node)
            if GEOM_ENT.Frame in get_node_types(self.g, node):
                return self.frame(node)
            return None

        of = optional_pose_ref(relation.of_id)
        wrt = optional_pose_ref(relation.wrt_id)
        quantity_kind = self.id(self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]))
        as_seen_by = self.frame(coordinate.as_seen_by.id)
        units = set(self.g.objects(coordinate.id, QUDT_SCHEMA["unit"]))
        if self._orientation_composition(coordinate.id) is not None or coordinate.types & {
            URI_GEOM_TYPE_QUATERNION,
            URI_GEOM_TYPE_DIRECTION_COSINE_XYZ,
        }:
            unit = self.id(QUDT_UNIT.UNITLESS)
        else:
            angular_units = units & {URI_QUDT_UNIT_RAD, URI_QUDT_UNIT_DEG}
            if len(angular_units) != 1:
                raise ConstraintViolation(
                    "geometry",
                    f"OrientationCoordinate '{coordinate.id}' needs exactly one angular unit, "
                    f"found {angular_units}",
                )
            unit = self.id(_si_unit(next(iter(angular_units))))
        axes = self.g.value(coordinate.id, URI_GEOM_PRED_AXES_SEQ)
        provenance = self.quantity_provenance(id_)
        return Orientation(
            self.id(id_),
            of,
            wrt,
            QuantityKind(quantity_kind),
            as_seen_by,
            Unit(unit),
            str(axes) if axes is not None else "xyz",
            (id_, ~MAP["subobject"], None) in self.g,
            provenance=provenance,
        )

    def position_reference(self, id_):
        """A Position is of a Point with respect to a Point (geometry metamodel)."""
        if id_ is None:
            return None
        if GEOM_ENT.Point in get_node_types(self.g, id_):
            return self.point(id_)
        if GEOM_ENT.Frame in get_node_types(self.g, id_):
            return Point(self.id(id_))
        raise ValueError(f"Position reference must be a Point, got: {id_}")

    @memoize
    def _pose_endpoint(self, node):
        """Resolve a pose endpoint (frame/scene-object) to its id."""
        if node is None:
            return None
        if ENV.RigidObject in get_node_types(self.g, node):
            return self.scene_object(node)
        return self.frame(node)

    def _bare_pose(self, id_):
        # A geom-rel:Pose with no PoseCoordinate: a snapshot/reference pose whose
        # KDL::Frame is filled at runtime. Same IR shape as a coordinate pose, with
        # its frame endpoints but no authored coordinate values.
        """Parse a bare Pose (no view) at node."""
        of_node = self.g.value(id_, GEOM_REL["of"])
        wrt_node = self.g.value(id_, GEOM_REL["with-respect-to"])
        as_seen_by_node = self.g.value(id_, GEOM_COORD["as-seen-by"])
        if of_node is None or wrt_node is None or as_seen_by_node is None:
            inherited_of, inherited_wrt, inherited_as_seen_by = self._derived_reference_frames(id_)
            of_node = of_node or inherited_of
            wrt_node = wrt_node or inherited_wrt
            as_seen_by_node = as_seen_by_node or inherited_as_seen_by
        provenance = self.quantity_provenance(id_)
        return Pose(
            self.id(id_),
            self._pose_endpoint(of_node),
            self._pose_endpoint(wrt_node),
            [self.id(k) for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]],
            self.frame(as_seen_by_node) if as_seen_by_node is not None else None,
            [self.id(u) for u in self.g[id_ : QUDT_SCHEMA["unit"]]],
            None,
            None,
            None,
            None,
            provenance=provenance,
        )

    def pose(self, id_):
        """Parse a Pose quantity (endpoints, orientation, position) at node."""
        coordinate = PoseCoordModel(id_, self.g)
        relation = coordinate.relation
        of = self._pose_endpoint(relation.of_id)
        wrt = self._pose_endpoint(relation.wrt_id)
        quantity_kind = [
            self.id(k) for k in self.g[relation.id : QUDT_SCHEMA["hasQuantityKind"]]
        ]
        as_seen_by = self.frame(coordinate.as_seen_by.id)
        unit = list(
            dict.fromkeys(
                self.id(_si_unit(u))
                for component in (coordinate.position_coord.id, coordinate.orientation_coord.id)
                for u in self.g[component : QUDT_SCHEMA["unit"]]
            )
        )
        length_unit = _length_unit(coordinate.position_coord)
        position_values = get_coord_vectorxyz(coordinate.position_coord, self.g)
        pos = _si_all(position_values, length_unit) if position_values is not None else None
        orientation_node = coordinate.orientation_coord.id
        representation = self.orientation_representation(orientation_node)
        # A symbolic triple keeps its convention: the backend composes per-axis quaternions,
        # and the sequence decides both the axes and the order they multiply in.
        euler_axes_sequence = None
        euler_intrinsic = False
        if representation == "euler":
            axes = self.g.value(orientation_node, URI_GEOM_PRED_AXES_SEQ)
            euler_axes_sequence = str(axes) if axes is not None else None
            euler_intrinsic = URI_GEOM_TYPE_INTRINSIC in coordinate.orientation_coord.types

        provenance = self.quantity_provenance(id_)
        if not provenance.snapshot:
            provenance = Provenance(
                authored=any(
                    self._is_authored(component.id)
                    for component in (coordinate.position_coord, coordinate.orientation_coord)
                )
                or (
                    bool(
                        coordinate.orientation_coord.types
                        & {
                            URI_GEOM_TYPE_EULER_ANGLES,
                            URI_GEOM_TYPE_QUATERNION,
                            URI_GEOM_TYPE_DIRECTION_COSINE_XYZ,
                        }
                    )
                    or self._orientation_composition(coordinate.orientation_coord.id) is not None
                    and any(
                        self.g.value(view, MAP["axis"]) is not None
                        for view in self.g.subjects(MAP["superobject"], id_)
                    )
                ),
                snapshot=False,
            )
        rel_operands = None
        if representation == "relative":
            rel_operands = self._relative_orientation(orientation_node)
        return Pose(
            self.id(id_),
            of,
            wrt,
            quantity_kind,
            as_seen_by,
            unit,
            pos,
            euler_axes_sequence,
            euler_intrinsic,
            representation,
            orientation_operands=rel_operands,
            provenance=provenance,
        )

    def _orientation_composition(self, orientation_node):
        """The `geom-op-ext:ComposeOrientation` operator writing into this orientation, if any."""
        if orientation_node is None:
            return None
        return next(
            (
                operation
                for operation in self.g.subjects(GEOM_OP["composite"], orientation_node)
                if GEOM_OP_EXT.ComposeOrientation in get_node_types(self.g, operation)
            ),
            None,
        )

    def _relative_orientation(self, orientation_node):
        """The composition's two operands, in `geom-op:in1`/`in2` order: each is either
        `{"pose": <id>}` (the base, by id) or `{"delta": [...], "representation": ...}` (the
        delta's ordered component values and rotation representation)."""
        composition = self._orientation_composition(orientation_node)
        if composition is None:
            raise ValueError(
                f"Relative orientation '{orientation_node}' has no composition operator."
            )
        in1 = self.g.value(composition, GEOM_OP["in1"])
        in2 = self.g.value(composition, GEOM_OP["in2"])
        if in1 is None or in2 is None:
            raise ValueError(
                f"Orientation composition '{composition}' must declare both operands."
            )

        def _operand(node):
            types = get_node_types(self.g, node)
            if URI_GEOM_TYPE_POSE_COORD in types:
                return {"pose": self.id(node)}
            representation_types = {
                URI_GEOM_TYPE_ANGLES_ABG,
                URI_GEOM_TYPE_QUATERNION,
                URI_GEOM_TYPE_DIRECTION_COSINE_XYZ,
            }
            if types & representation_types:
                # A delta is literal by construction, so it folds to a quaternion here however
                # the model wrote it.
                rotation = get_orientation_coord_vals(ModelBase(node_id=node, graph=self.g), self.g)
                if rotation is None:
                    raise ValueError(
                        f"Relative orientation delta '{node}' has no literal components"
                    )
                return {
                    "delta": [{"value": float(value)} for value in rotation.as_quat()],
                    "representation": "quaternion",
                }
            raise ValueError(
                f"Relative orientation operand '{node}' is neither a pose nor an orientation"
            )

        operands = [_operand(in1), _operand(in2)]
        if sum("pose" in op for op in operands) != 1 or sum("delta" in op for op in operands) != 1:
            raise ValueError(
                f"Relative orientation '{orientation_node}' must compose exactly one base pose "
                "and one delta rotation."
            )
        return operands

    def orientation_representation(self, id_):
        """How an orientation's components arrive, not how the model wrote them.

        A rotation whose components are all literal denotes one rotation whichever way it was
        authored, and scipy resolves Euler angles, quaternions and direction cosines to the
        same quaternion, so all three report `quaternion`. What cannot be resolved ahead of
        time keeps its own shape: `euler` is a triple whose angles arrive at runtime (the RDF
        builder types a literal triple `AnglesAlphaBetaGamma`, so its absence marks one
        symbolic), and `relative` composes around a runtime pose.
        """
        if id_ is None:
            return "quaternion"
        types = get_node_types(self.g, id_)
        if self._orientation_composition(id_) is not None:
            return "relative"
        if URI_GEOM_TYPE_EULER_ANGLES in types and URI_GEOM_TYPE_ANGLES_ABG not in types:
            return "euler"
        return "quaternion"

    @memoize
    def velocity_twist(self, id_):
        """Parse a VelocityTwist quantity at node."""
        self._expect_type(id_, GEOM_COORD["VelocityTwistCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        of = self.simplicial_complex(self.g.value(id_, GEOM_REL["of"]))
        wrt = self.simplicial_complex(self.g.value(id_, GEOM_REL["with-respect-to"]))
        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.id(k))
        reference_point = self.point(self.g.value(id_, GEOM_REL["reference-point"]))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = []
        for u in self.g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.id(u))

        provenance = self.quantity_provenance(id_)
        return VelocityTwist(
            self.id(id_),
            of,
            wrt,
            quantity_kind,
            reference_point,
            as_seen_by,
            unit,
            provenance=provenance,
        )

    def _spatial_coordinate_fields(self, id_):
        """Shared field extraction for the 6D coordinate quantities.

        AccelerationTwist, PoseDifference and Wrench are distinct concepts with an
        identical coordinate structure; only the RDF predicates differ (geometry vs
        rigid-body-dynamics namespaces).
        """
        quantity_kind = [self.id(k) for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]]
        reference_point = self.point(self.g.value(id_, GEOM_REL["reference-point"]))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = [self.id(u) for u in self.g[id_ : QUDT_SCHEMA["unit"]]]
        provenance = self.quantity_provenance(id_)
        return quantity_kind, reference_point, as_seen_by, unit, provenance

    @memoize
    def acceleration_twist(self, id_):
        """Parse an AccelerationTwist quantity at node."""
        self._expect_type(id_, GEOM_COORD["AccelerationTwistCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        qk, ref, seen, unit, provenance = self._spatial_coordinate_fields(id_)
        return AccelerationTwist(self.id(id_), qk, ref, seen, unit, provenance=provenance)

    @memoize
    def pose_difference(self, id_):
        """Parse a PoseDifference quantity at node."""
        self._expect_type(id_, GEOM_COORD["PoseDifferenceCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        qk, ref, seen, unit, provenance = self._spatial_coordinate_fields(id_)
        return PoseDifference(self.id(id_), qk, ref, seen, unit, provenance=provenance)

    @memoize
    def wrench(self, id_):
        """Parse a Wrench quantity (with any FT sensor) at node."""
        self._expect_type(id_, RBDYN_COORD["WrenchCoordinate"])
        relation = self.g.value(id_, RBDYN_COORD["of-wrench"])
        if relation is None or RBDYN_ENT.Wrench not in get_node_types(self.g, relation):
            raise ConstraintViolation(
                "dynamics", f"WrenchCoordinate '{id_}' has no valid of-wrench relation"
            )
        qk = [self.id(k) for k in self.g[relation : QUDT_SCHEMA["hasQuantityKind"]]]
        reference = self.g.value(relation, RBDYN_ENT["reference-point"])
        seen_by = self.g.value(id_, RBDYN_COORD["as-seen-by"])
        if reference is None or seen_by is None:
            raise ConstraintViolation(
                "dynamics", f"WrenchCoordinate '{id_}' is missing reference-point/as-seen-by"
            )
        ref = self.point(reference)
        seen = self.frame(seen_by)
        unit = [self.id(u) for u in self.g[id_ : QUDT_SCHEMA["unit"]]]
        provenance = self.quantity_provenance(id_)
        sensor = self.g.value(id_, SOSA.madeBySensor)
        sensor_name = self.id(sensor) if sensor is not None else ""
        sensor_frame_node = self.g.value(sensor, SENSORS.frame) if sensor is not None else None
        if sensor is not None and sensor_frame_node is None:
            raise ConstraintViolation(
                "dynamics", f"WrenchCoordinate '{id_}' sensor '{sensor}' has no physical frame"
            )
        sensor_frame = self.frame(sensor_frame_node) if sensor_frame_node is not None else None
        return Wrench(
            self.id(id_), qk, ref, seen, unit, provenance, sensor_frame, sensor_name
        )

    def _is_duration(self, id_):
        """Authored durations carry the OWL-Time type; runtime elapsed time is a Time-kind
        quantity the clock fills, so it has a kind but no value."""
        if TIME["Duration"] in get_node_types(self.g, id_):
            return True
        return self.g.value(id_, QUDT_SCHEMA.hasQuantityKind) == QUDT_QTY["Time"]

    @memoize
    def quantity(self, id_):
        """Parse the quantity at node, dispatching on its RDF type."""
        if self._is_duration(id_):
            return self.duration_quantity(id_)
        self._expect_type(id_, QUDT_SCHEMA["Quantity"])
        quantity_kind_node = self.g.value(id_, QUDT_SCHEMA.hasQuantityKind)
        quantity_kind = self.id(quantity_kind_node)

        # Values below are converted, so the unit reported alongside them is the SI one.
        unit = self.id(_si_unit(self.g.value(id_, QUDT_SCHEMA["unit"])))
        has_view = (id_, ~MAP["subobject"], None) in self.g

        types = get_node_types(self.g, id_)
        if URI_GEOM_TYPE_POSE_COORD in types:
            return self.pose(id_)
        if URI_GEOM_TYPE_POSE in types:
            return self._bare_pose(id_)
        if URI_GEOM_TYPE_POSITION in types:
            return self.position(id_)
        if URI_GEOM_TYPE_ORIENT in types:
            return self.orientation(id_)
        if CSTR_HDL_EXT["SetpointGenerator"] in get_node_types(self.g, id_):
            value_kind_node = next(
                (k for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]] if k != CSTR_HDL_EXT.SetpointGenerator),
                None,
            )
            provenance = self.quantity_provenance(id_)
            return Setpoint(
                self.id(id_),
                QuantityKind(quantity_kind),
                Unit(unit),
                has_view,
                provenance=provenance,
                value_kind=self.id(value_kind_node) if value_kind_node is not None else None,
            )

        if quantity_kind == "FreeVector" and GEOM_COORD["VectorXYZ"] in get_node_types(
            self.g, id_
        ):
            provenance = self.quantity_provenance(id_)
            return FreeVector(
                self.id(id_),
                QuantityKind(quantity_kind),
                Unit(unit),
                self.parse_xyz(id_),
                has_view,
                provenance=provenance,
            )

        value = None
        if (id_, QUDT_SCHEMA["value"], None) in self.g:
            authored = self.g.value(id_, QUDT_SCHEMA["value"])
            value = _si(float(authored), self.g.value(id_, QUDT_SCHEMA["unit"]))
        reference_value = self.g.value(id_, CSTR["reference-value"])
        provenance = self.quantity_provenance(id_)
        return Quantity(
            self.id(id_),
            QuantityKind(quantity_kind),
            Unit(unit),
            value,
            has_view,
            provenance=provenance,
            reference_value=self.id(reference_value) if reference_value is not None else None,
        )

    @memoize
    def duration_quantity(self, id_):
        """Parse an authored duration or runtime elapsed-duration coordinate."""
        if not self._is_duration(id_):
            raise ValueError(f"Expected a duration at '{id_}'")
        # An elapsed coordinate has no authored value; the clock fills it at runtime.
        value_node = self.g.value(id_, QUDT_SCHEMA["value"])
        value = (
            None
            if value_node is None
            else _seconds(float(value_node), self.g.value(id_, QUDT_SCHEMA["unit"]))
        )
        provenance = self.quantity_provenance(id_)
        return Quantity(
            self.id(id_), QuantityKind("Duration"), Unit("Second"), value, False,
            provenance=provenance,
        )

    @memoize
    def joint_position(self, id_):
        """Parse a JointPosition quantity at node."""
        self._expect_type(id_, KC_STAT["JointPositionCoordinate"])
        self._expect_type(id_, KC_STAT["JointReference"])
        joint_node = self.g.value(id_, KC_STAT["of-joint"])
        if not isinstance(joint_node, URIRef):
            raise ConstraintViolation(
                "kinematic-chain", f"JointPositionCoordinate '{id_}' has no of-joint URI"
            )
        joint_name = self.label(joint_node) if joint_node is not None else ""
        return JointPosition(self.id(id_), joint_name)

    def quantity_provenance(self, id_):
        # Provenance(authored, snapshot), mutually exclusive: snapshot wins (mirrors old roles() elif).
        # authored == carries an authored value/coordinate and is not a runtime snapshot.
        """Parse a quantity's Provenance (authored / snapshot) at node."""
        snapshot = ALGO_EXT.Snapshot in get_node_types(self.g, id_)
        authored = (not snapshot) and self._is_authored(id_)
        return Provenance(authored=authored, snapshot=snapshot)

    def _is_authored(self, id_):
        """True when a quantity's value was authored by the user (not computed)."""
        # A path parameter carries a value only as its starting point on the curve; the
        # traversal drives it every tick, so the value is not the user's.
        if (None, GEOM_OP_EXT["path-parameter"], id_) in self.g:
            return False
        if (id_, QUDT_SCHEMA["value"], None) in self.g:
            return True
        if (id_, CSTR["reference-value"], None) in self.g:
            return True
        if any(
            (id_, predicate, None) in self.g
            for predicate in (
                URI_GEOM_PRED_X,
                URI_GEOM_PRED_Y,
                URI_GEOM_PRED_Z,
                URI_GEOM_PRED_W,
                URI_GEOM_PRED_ALPHA,
                URI_GEOM_PRED_BETA,
                URI_GEOM_PRED_GAMMA,
                URI_GEOM_PRED_DIRECTION_COSINE_X,
                URI_GEOM_PRED_DIRECTION_COSINE_Y,
                URI_GEOM_PRED_DIRECTION_COSINE_Z,
            )
        ):
            return True
        return False

    @memoize
    def simplicial_complex(self, id_):
        """Parse a SimplicialComplex, mapping a body-origin frame to its runtime body."""
        if not any(
            type_ in get_node_types(self.g, id_)
            for type_ in (GEOM_ENT.SimplicialComplex, GEOM_ENT.Frame)
        ):
            raise ValueError(f"Expected a rigid body or frame, got: {id_}")
        body = next(
            (
                owner
                for owner in self.g.subjects(GEOM_ENT.simplices, id_)
                if GEOM_ENT.RigidBody in get_node_types(self.g, owner)
            ),
            None,
        )
        if body is not None and self.id(id_) == f"{self.id(body)}_origin":
            return SimplicialComplex(self.id(body))
        return SimplicialComplex(self.id(id_))

    @memoize
    def scene_object(self, id_):
        """Parse a SceneObject at node."""
        self._expect_type(id_, ENV.RigidObject)
        return SceneObject(self.id(id_), self.id(id_))

    @memoize
    def frame(self, id_):
        """Parse a frame, mapping a scene-dsl body-origin frame to its runtime body."""
        self._expect_type(id_, GEOM_ENT["Frame"])
        body = next(
            (
                owner
                for owner in self.g.subjects(GEOM_ENT.simplices, id_)
                if GEOM_ENT.RigidBody in get_node_types(self.g, owner)
            ),
            None,
        )
        if body is not None and self.id(id_) == f"{self.id(body)}_origin":
            return Frame(self.id(body))
        return Frame(self.id(id_))

    @memoize
    def point(self, id_):
        """Parse a Point at node."""
        if not {GEOM_ENT.Point, GEOM_ENT.Frame} & get_node_types(self.g, id_):
            raise ValueError(f"Expected a point or frame, got: {id_}")
        return Point(self.id(id_))

    def view(self):
        """Parse a View (superobject, subobject, subspace, axis) at node."""
        dispatcher = [
            (MAP_EXT["PoseCoordinateView"], self.pose),
            (MAP_EXT["VelocityTwistCoordinateView"], self.velocity_twist),
            (MAP_EXT["AccelerationTwistCoordinateView"], self.acceleration_twist),
            (MAP_EXT["PoseDifferenceView"], self.pose_difference),
            (MAP_EXT["WrenchCoordinateView"], self.wrench),
        ]

        view_map = {}
        # Sorted: _views_for_access keeps the first view seen for a subobject, so an unordered
        # walk would publish a different (equivalent) view id on every generation.
        for view in sorted(self.g[: RDF["type"] : MAP["View"]]):
            superobject_id = self.g.value(view, MAP["superobject"])
            superobject = None
            for type_, func in dispatcher:
                if type_ not in get_node_types(self.g, view):
                    continue

                superobject = func(superobject_id)
                break

            if superobject is None:
                superobject = self.quantity(superobject_id)

            subobject = self.quantity(self.g.value(view, MAP["subobject"]))
            subspace = self.subspace(self.g.value(view, MAP["subspace"]))
            axis_node = self.g.value(view, MAP["axis"])
            axis = self.axis(axis_node) if axis_node is not None else None

            if superobject is None:
                raise ValueError(
                    f"MAP view {view} has an unrecognized type; no view dispatcher matched"
                )
            view_map[self.id(view)] = View(
                self.id(view), superobject, subobject, subspace, axis
            )

        return view_map

    def data_structures(self):
        """Parse every data-structure entity in the graph."""
        dispatcher = [
            (GEOM_COORD["DirectionCoordinate"], self.direction),
            # A combined PoseCoordinate is also a PositionCoordinate and an
            # OrientationCoordinate; dispatch the most specific type first.
            (GEOM_COORD["PoseCoordinate"], self.pose),
            (GEOM_COORD["PositionCoordinate"], self.position),
            (GEOM_COORD["OrientationCoordinate"], self.orientation),
            (GEOM_COORD["VelocityTwistCoordinate"], self.velocity_twist),
            (GEOM_COORD["AccelerationTwistCoordinate"], self.acceleration_twist),
            (GEOM_COORD["PoseDifferenceCoordinate"], self.pose_difference),
            (RBDYN_COORD["WrenchCoordinate"], self.wrench),
            (QUDT_SCHEMA["Quantity"], self.quantity),
        ]

        data_structures = []
        for type_, func in dispatcher:
            # Sort within the type group: keeps most-specific-type-first dispatch (which the
            # dedupe below relies on) while making the published order reproducible.
            for dstruct in sorted(self.g[: RDF["type"] : type_]):
                data_structures.append(func(dstruct))

        return _dedupe_by_id(data_structures)

    def _path_fields(self, path_node):
        """The path's geometry, as fields of the evaluator call that traverses it.

        Traversal is one computation: the shape decides the maths, so the closure takes the
        path's type and carries its parameters directly.
        """
        path_types = get_node_types(self.g, path_node)
        spec = next((s for s in ops_path if s.type_ in path_types), None)
        if spec is None:
            return {}
        fields = {"type": self.id(spec.type_)}
        for input_ in spec.input:
            fields[self.id(input_)] = parse_argument(self.g, path_node, input_, self.id)
        for param in spec.parameters:
            fields[self.id(param)] = parse_argument(
                self.g, path_node, param, self.id, resolve_value=True
            )
        return fields

    def closures(self, operators):
        """Build the closure for each call of the given operators."""
        closures = {}
        for operator in operators:
            for closure in self.g.subjects(RDF["type"], operator.type_):
                cl = (
                    operator.closure_step(self.g, self.id, closure)
                    if hasattr(operator, "closure_step")
                    else None
                )
                if cl:
                    if operator.type_ in {
                        GEOM_OP_EXT.PathProjection,
                        GEOM_OP_EXT.PathTangentFrame,
                        GEOM_OP_EXT.PathEvaluator,
                    }:
                        if operator.type_ == GEOM_OP_EXT.PathEvaluator:
                            reference = self.g.value(closure, GEOM_OP.out)
                            if reference is not None:
                                cl["setpoint"] = self.id(reference)
                        fields = self._path_fields(self.g.value(closure, GEOM_OP_EXT.path))
                        # The geometry decides the maths; each caller samples the same curve.
                        cl["shape"] = fields.pop("type")
                        cl.update(fields)
                    if operator.type_ in {ALGO_EXT.VelocityProfile, ALGO_EXT.Admittance}:
                        reference = self.g.value(closure, ALGO_EXT.out)
                        constraint = next(
                            self.g.subjects(CSTR["reference-value"], reference), None
                        )
                        if constraint is None:
                            raise ValueError(
                                f"{self.id(operator.type_)} '{self.id(closure)}' output is not bound to a constraint."
                            )
                        # Evaluators carry cstr-hdl:constraint too, and can win this lookup;
                        # filter state belongs to the controller.
                        controller = next(
                            (
                                node
                                for node in self.g.subjects(CSTR_HDL.constraint, constraint)
                                if CSTR_HDL.ConstraintEvaluator
                                not in get_node_types(self.g, node)
                            ),
                            None,
                        )
                        if controller is None:
                            raise ValueError(
                                f"{self.id(operator.type_)} '{self.id(closure)}' constraint has no controller."
                            )
                        cl["controller"] = self.id(controller)
                        if operator.type_ == ALGO_EXT.VelocityProfile:
                            # A physical profile starts from the constraint's measured quantity.
                            cl["measured"] = self.id(self.g.value(constraint, CSTR.quantity))
                            cl["goal"] = cl.pop("target")
                    closures[self.id(closure)] = cl

        return closures

    def schedule(self, start, ops):
        # Start at a SolverWithInputAndOutput
        # Then traverse along data structure and collect function blocks
        """Build the dependency-ordered schedule for the given operators."""
        q = collections.deque()
        data_structures = set()
        sched = []
        scheduled_nodes = {}
        for v in start:
            v_types = get_node_types(self.g, v)
            for op in ops:
                if op.type_ not in v_types:
                    continue

                call = self.id(v)
                if op.schedulable and call not in self.sched:
                    sched.append(call)
                    scheduled_nodes[call] = v
                    self.sched.add(call)

                # sorted(): these come back as sets, and set order over rdflib nodes varies
                # between processes. The traversal order decides the emitted schedule order, so
                # an unordered iteration here makes generation non-reproducible.
                for data_in in sorted(op.from_operator_to_input(self.g, v)):
                    q.append(data_in)
                    data_structures.add(data_in)

        # An operator/call: data_in --(in)--> call --(out)--> data_out (traversed backward here;
        # there may be multiple inputs/outputs).
        op_preds = [(_op_output_preds(op), op) for op in ops]
        while len(q) > 0:
            data_out = q.pop()
            preds_into = set(self.g.predicates(None, data_out))
            # Only ops whose output predicate points into data_out can match here.
            for out_set, op in op_preds:
                if out_set.isdisjoint(preds_into):
                    continue
                res = op.scheduler_step(self.g, data_out)

                for call_node in res["schedule"]:
                    call = self.id(call_node)
                    if call and call not in self.sched:
                        sched.append(call)
                        scheduled_nodes[call] = call_node
                        self.sched.add(call)

                for data_in in sorted(res["data_structures"]):
                    # We have already visited this data structure,
                    # so skip it
                    if data_in in data_structures:
                        continue

                    q.append(data_in)
                    data_structures.add(data_in)

            # An inline/declared Pose is no operator's output, so follow its per-axis views
            # to schedule the closures producing its scalar components.
            # sorted() for the same reason as above: both walks return unordered sets and the
            # queue order they set decides the emitted schedule order.
            view_subobjects = (
                self.g.value(view, MAP["subobject"])
                for view in sorted(self.g.subjects(MAP["superobject"], data_out))
                if self.g.value(view, MAP["axis"]) is not None
            )
            for successor in itertools.chain(
                (node for node in view_subobjects if node is not None),
                sorted(self.g.objects(data_out, CSTR["reference-value"])),
            ):
                if successor not in data_structures:
                    q.append(successor)
                    data_structures.add(successor)

        sched.reverse()
        return self._topological_schedule(sched, scheduled_nodes, ops)

    def _operator_outputs(self, node, op):
        """Output data ids produced by an operator call."""
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
            node_types = get_node_types(self.g, node)
            for op in ops:
                if op.type_ not in node_types:
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
            """Recurse operators feeding a data node, appending schedulable calls in dependency
            order.
            """
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


# ---------------------------------------------------------------------------
# Motion units
# ---------------------------------------------------------------------------
def _upstream_dependencies(data_id: str, closure_input_map: dict[str, set[str]]) -> set[str]:
    """Transitive set of data ids that feed the given id through the closure input map."""
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
    """Strip a frame_/link_ prefix to the bare MuJoCo body name."""
    if name is None:
        return None
    for prefix in ("frame_", "frame-", "link_", "link-"):
        if name.startswith(prefix):
            return name[len(prefix) :]
    return name


def _mark_acceleration_constraint_frames(solver):
    """Flag each of a solver's acceleration constraints as base-aligned when its axis frame is the
    chain root (or unset).
    """
    root_body = _body_name(getattr(solver, "chain_root", None))
    for driver in solver.motion_drivers:
        for constraint in driver.acceleration_constraint:
            axis_frame = getattr(constraint.as_seen_by, "id", None)
            constraint.base_aligned = axis_frame is None or _body_name(axis_frame) == root_body


def _serial_chain_solvers_for_handler(handler, slv_chain, solver_ids):
    """Select the arm solvers explicitly referenced by a handler's controllers."""
    result = []
    motion_driver_id = f"driver_{handler.motion.id.removeprefix('motion_')}"
    for solver in slv_chain:
        if solver.id not in solver_ids:
            continue
        matched = list(solver.motion_drivers)
        if not matched:
            continue
        selected = next((driver for driver in matched if driver.id == motion_driver_id), matched[0])
        result.append(
            HandlerSerialChainSolver(
                id=solver.id,
                output=solver.output,
                motion_driver=selected,
                algorithm=solver.algorithm,
                gravity=solver.gravity,
                root_acc=solver.root_acc,
                chain_root=solver.chain_root,
                chain_end=solver.chain_end,
                torque_saturation=solver.torque_saturation,
            )
        )

    return result


def _views_by_subobject(view_map):
    indexed: dict[str, list] = {}
    for view in view_map.values():
        subobject_id = _field(_field(view, "subobject"), "id")
        if subobject_id:
            indexed.setdefault(subobject_id, []).append(view)
    return indexed


def _views_for_access(view_map, shared_data, motions, closures) -> dict:
    """Index unambiguous MAP views by subobject, for the template's access expressions.

    Views are keyed by their own identity everywhere else; codegen instead resolves a quantity id
    to the superobject expression that reads it. A subobject that is written directly -- an
    authored or literal shared value, a snapshot target, a closure output -- is an ordinary shared
    quantity and keeps its own field, and one reused by views that disagree on how they access it
    must not silently pick one of them.
    """
    direct_ids = {
        _field(item, "id")
        for item in shared_data
        if _field(item, "id")
        and (
            _field(item, "value") is not None
            or _field(_field(item, "provenance"), "authored", False)
        )
    }
    direct_ids.update(
        _field(snapshot, "target_id")
        for motion in motions
        for snapshot in _field(motion, "snapshots", []) or []
    )
    direct_ids.update(
        output_id
        for closure in closures.values()
        for output_id in closure_output_ids(closure)
    )

    indexed: dict[str, object] = {}
    for view in view_map.values():
        subobject_id = _field(_field(view, "subobject"), "id")
        if not subobject_id or subobject_id in direct_ids:
            continue
        if subobject_id not in indexed:
            indexed[subobject_id] = view
            continue
        previous = indexed[subobject_id]
        if previous is None:
            continue
        if any(
            _field(previous, field) != _field(view, field)
            for field in ("superobject", "subspace", "axis", "direction")
        ):
            indexed[subobject_id] = None
    return {id_: view for id_, view in indexed.items() if view is not None}


def _unique_view_for_subobject(indexed_views, subobject_id, context):
    matches = indexed_views.get(subobject_id, ())
    if len(matches) > 1:
        raise ValueError(
            f"{context}: quantity '{subobject_id}' is the subobject of multiple MAP views"
        )
    return matches[0] if matches else None


def _relative_poses_for_motion(evaluators, view_map, serial_chain_solvers):
    """Detect Pose quantities whose wrt frame ends in _start and pair them with FK outputs."""
    fk_poses: dict[str, str] = {}  # of_id → FK pose id
    for solver in serial_chain_solvers:
        for out in solver.output:
            if getattr(out, "type", "") == "Pose":
                of_id = getattr(getattr(out, "of", None), "id", None)
                if of_id:
                    fk_poses[of_id] = out.id

    start_rel_poses: dict[str, object] = {}
    indexed_views = _views_by_subobject(view_map)
    for ev in evaluators:
        qty = getattr(getattr(ev, "constraint", None), "quantity", None)
        if qty is None or not getattr(qty, "has_view", False):
            continue
        view = _unique_view_for_subobject(indexed_views, qty.id, "relative pose lookup")
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
    """Id of a constraint's reference-value parameter, or None."""
    param = getattr(constraint, "parameter", None)
    ref = getattr(param, "reference_value", None) if param else None
    return getattr(ref, "id", None)


def _snapshot_reference_value_ids(evaluators, constraints):
    """Set of reference-value ids across the given evaluators and constraints."""
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
    snapshot_trigger_map=None,
    motion_token=None,
    snapshot_owner_map=None,
    motion_tokens=(),
):
    """Build a motion's initial snapshot captures from its references."""
    snapshot_trigger_map = snapshot_trigger_map or {}
    snapshot_owner_map = snapshot_owner_map or {}
    ref_val_ids = _snapshot_reference_value_ids(evaluators, constraints)
    closures = closures or {}
    data_reference_map = data_reference_map or {}

    subobjects_by_super: dict[str, list[str]] = {}
    supers_by_subobject: dict[str, list[str]] = {}
    for view in view_map.values():
        super_id = _field(_field(view, "superobject"), "id")
        subobject_id = _field(_field(view, "subobject"), "id")
        if super_id and subobject_id:
            subobjects_by_super.setdefault(super_id, []).append(subobject_id)
            supers_by_subobject.setdefault(subobject_id, []).append(super_id)

    def add_reference(ref_id):
        """Record a snapshot capture for one referenced id."""
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
            # superobject -> subobject (forward decomposition), and subobject ->
            # superobject (a composite pose's snapshot is only referenced through its
            # scalar components, so climb back to capture the composite).
            pending.extend(subobjects_by_super.get(current, ()))
            pending.extend(supers_by_subobject.get(current, ()))

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
        # Capture only what this motion declares. A snapshot owned by another motion is that
        # motion's to sample; re-capturing it here would overwrite its value with this
        # motion's pose. Unowned (shared-context) snapshots stay everyone's to capture.
        owner = snapshot_owner_map.get(target_id)
        if owner in motion_tokens and owner != motion_token:
            continue
        seen.add(target_id)
        source_id = snapshot_source_map[target_id]
        source_closure_id = (
            None if source_id in supers_by_subobject else closure_output_map.get(source_id)
        )
        result.append(
            SnapshotCapture(
                target_id=target_id,
                source_id=source_id,
                source_closure_id=source_closure_id,
                trigger_event=snapshot_trigger_map.get((motion_token, target_id)),
            )
        )
    return result


def _scene_relative_poses_for_motion(view_map, serial_chain_solvers, evaluators=None):
    """For each view whose wrt-frame is a scene object, emit the requested relative pose."""
    fk_pose_by_frame: dict[str, str] = {}
    # Keys are the scene-object's id (the "of" of scene-object solver pose outputs).
    # A wrt_id lookup asks: "is this frame the subject of a tracked scene-object pose?"
    scene_pose_by_id: dict[str, tuple[str, str | None]] = {}
    solver_output_ids: set[str] = set()
    for solver in serial_chain_solvers:
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
        if (
            quantity is not None
            and hasattr(quantity, "of")
            and hasattr(quantity, "with_respect_to")
        ):
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
    Subspace.Linear: ("linear", False),
    Subspace.Angular: ("angular", True),
}


def _pose_axis_error_groups_for_motion(eval_nodes, p, view_map):
    """Group a motion's per-axis pose error evaluators into PoseAxisErrorGroups, one per superobject
    pose.
    """
    groups: dict[str, PoseAxisErrorGroup] = {}
    indexed_views = _views_by_subobject(view_map)
    for eval_node in eval_nodes:
        if CSTR_HDL["ErrorEvaluator"] not in get_node_types(p.g, eval_node):
            continue

        evaluator = p.constraint_evaluator(eval_node)
        if not isinstance(evaluator.constraint.parameter, EqualityConstraint):
            continue
        if evaluator.error is None:
            continue

        quantity = evaluator.constraint.quantity
        view = _unique_view_for_subobject(
            indexed_views, quantity.id, "pose-axis error grouping"
        )
        if view is None:
            continue
        so_type = getattr(view.superobject, "type", None)
        if so_type not in _GROUPABLE_SO_TYPES:
            continue

        mapping = _SUBSPACE_TO_GROUP_AXIS.get(view.subspace)
        if mapping is None:
            qkind = getattr(getattr(quantity, "quantity_kind", None), "id", "")
            quantity_id = getattr(quantity, "id", "")
            if so_type == "Pose" and (
                "Angle" in qkind or "angle" in qkind.lower() or "rotation" in quantity_id
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
                id=f"pose_axis_error_{so_id}", pose=so_id, components=[], superobject_type=so_type
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
        setattr(
            group,
            f"{subspace}_{view.axis.value.lower()}",
            evaluator.constraint.parameter.reference_value.id,
        )

    return [group for group in groups.values() if len(group.components) > 1]


def build_motion_units(
    g,
    p,
    handlers,
    node_by_id,
    slv_chain,
    snapshot_source_map=None,
    view_map=None,
    closure_output_map=None,
    data_reference_map=None,
    closure_input_map=None,
    closures=None,
    data_structures=None,
    pose_components=None,
    fsm=None,
    derivation=None,
    snapshot_trigger_map=None,
    snapshot_owner_map=None,
    closure_owner_map=None,
):
    """Build the per-motion IR units (one motion per handler), each complete with schedules,
    monitors, controllers, conditions, declared poses, FSM wiring and function-interface flags;
    returns (motions, fsm_meta).
    """
    snapshot_source_map = snapshot_source_map or {}
    view_map = view_map or {}
    closure_output_map = closure_output_map or {}
    data_reference_map = data_reference_map or {}
    closure_input_map = closure_input_map or {}
    motions = []
    snapshot_owner_map = snapshot_owner_map or {}
    closure_owner_map = closure_owner_map or {}
    motion_tokens = {
        _motion_suffix(p, g.value(node_by_id[h.id], CSTR_HDL["motion"])) for h in handlers
    }
    for handler in handlers:
        handler_node = node_by_id[handler.id]
        motion_node = g.value(handler_node, CSTR_HDL["motion"])
        motion = handler.motion
        handler_plans = derivation.controllers_by_handler.get(handler_node, ())
        handler_solver_ids = {p.id(plan.solver) for plan in handler_plans}

        # Classify constraints by motion phase via RDF traversal
        _raw_when = set(g[motion_node : MOT["when"]])
        _raw_until = set(g[motion_node : MOT["until"]])
        when_constraint_nodes = _expanded_constraints(g, _raw_when)
        while_constraint_nodes = set(g[motion_node : MOT["while"]])
        until_constraint_nodes = _expanded_constraints(g, _raw_until)

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
            return _is_elapsed_constraint(g, g.value(eval_node, CSTR_HDL["constraint"]))

        # Classify controllers by the constraints active during the motion body.
        while_error_nodes = {g.value(n, CSTR_HDL["error"]) for n in while_eval_nodes}
        while_error_nodes.discard(None)
        active_plans = [plan for plan in handler_plans if plan.constraint in while_constraint_nodes]
        active_controllers = [
            controller
            for plan in active_plans
            for controller in _derived_controllers(g, p, derivation, plan)
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
            monitored = set(g.objects(mon_node, CSTR_HDL["constraint"]))
            if monitored and monitored == _raw_when:
                when_mon_nodes.append(mon_node)
                continue
            if monitored and monitored == _raw_until:
                until_mon_nodes.append(mon_node)
                continue
            # A group monitor names one of the section's nodes, not the whole section, and the
            # group node itself never appears in the expanded member sets below.
            if monitored and monitored <= _raw_when:
                when_mon_nodes.append(mon_node)
                continue
            if monitored and monitored <= _raw_until:
                until_mon_nodes.append(mon_node)
                continue
            if monitored:
                if monitored & when_cstr_nodes:
                    when_mon_nodes.append(mon_node)
                elif monitored & while_cstr_nodes:
                    while_mon_nodes.append(mon_node)
                elif monitored & until_cstr_nodes:
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
        if not classified_eval_nodes <= all_eval_nodes:
            raise ValueError(
                f"Handler {handler.id}: classified evaluators not a subset of handler evaluators"
            )

        all_mon_nodes = set(g[handler_node : CSTR_HDL["monitors"]])
        classified_mon_nodes = set(when_mon_nodes) | set(while_mon_nodes) | set(until_mon_nodes)
        if not classified_mon_nodes <= all_mon_nodes:
            raise ValueError(
                f"Handler {handler.id}: classified monitors not a subset of handler monitors"
            )

        # when_schedule runs in can_start (own Parser + dedup set); while_/until share p_active
        # so steps evaluated in both phases emit once (see the _build_ir schedules note).
        p_when = Parser(g)
        when_schedule = p_when.schedule(
            [n for n in when_eval_nodes if not _is_elapsed_eval(n)], ops_generic + ops_cstr_hdl
        )

        handler_chain_solvers = _serial_chain_solvers_for_handler(
            handler,
            slv_chain,
            handler_solver_ids,
        )
        handler_output_ids = {c.control_signal.id for c in handler.controllers}
        cartesian_force_nodes = []
        for solver in handler_chain_solvers:
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
        # Until monitors run before control each tick, so build until_schedule before the
        # while passes: derived quantities consumed by an until monitor (e.g. a target
        # computed from a snapshot) must be scheduled in the earlier phase, or the monitor
        # sees the previous tick's / default value on the first tick.
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

        pose_axis_error_groups = _pose_axis_error_groups_for_motion(
            while_eval_nodes, p_active, view_map
        )
        pose_axis_error_eval_ids = {
            component.eval_id for group in pose_axis_error_groups for component in group.components
        }
        grouped_while_eval_nodes = {
            node for node in while_eval_nodes if p_active.id(node) in pose_axis_error_eval_ids
        }
        # A grouped evaluator's error is emitted inline by its group, ahead of the schedule
        # block, but whatever produces its reference -- an admittance filter, a velocity
        # profile -- still has to run, and has to run first. Walk the grouped nodes before
        # the main pass so those producers land here rather than being skipped entirely.
        pre_group_schedule = [
            step
            for step in p_active.schedule(
                sorted(grouped_while_eval_nodes, key=str), ops_generic + ops_cstr_hdl
            )
            if step not in pose_axis_error_eval_ids
        ]
        plan_by_constraint = {plan.constraint: plan for plan in active_plans}
        pre_controller_evaluators = []
        trailing_evaluators = []
        for node in while_eval_nodes:
            if (
                node in grouped_while_eval_nodes or _is_elapsed_eval(node)
            ):
                continue
            plan = plan_by_constraint.get(g.value(node, CSTR_HDL.constraint))
            if plan is not None and CSTR_HDL_EXT.FeedForwardController not in get_node_types(
                g, plan.controller
            ):
                pre_controller_evaluators.append(node)
            else:
                trailing_evaluators.append(node)

        while_schedule = p_active.schedule(
            pre_controller_evaluators, ops_generic + ops_cstr_hdl
        )
        derived_controller_ids = {controller.id for controller in active_controllers}
        authored_controller_ids = {p.id(plan.controller) for plan in active_plans}
        while_schedule = [
            step
            for step in while_schedule
            if step not in derived_controller_ids and step not in authored_controller_ids
        ]
        for plan in active_plans:
            if len(plan.axes) <= 1:
                continue
            reference = g.value(plan.constraint, CSTR["reference-value"])
            reference_view = next(g.subjects(MAP.subobject, reference), None)
            reference_pose = g.value(reference_view, MAP.superobject)
            interpolation = next(g.subjects(GEOM_OP.out, reference_pose), None)
            if interpolation is not None:
                for step in p_active.schedule([interpolation], ops_generic + ops_cstr_hdl):
                    if step not in while_schedule:
                        while_schedule.append(step)
            evaluator_id = SolverIdFactory(
                p.id(plan.controller), _motion_suffix(p, plan.motion)
            ).pose_evaluator()
            if evaluator_id not in while_schedule:
                while_schedule.append(evaluator_id)
        for node in pre_controller_evaluators:
            eval_id = p.id(node)
            if eval_id not in while_schedule:
                while_schedule.append(eval_id)
        force_schedule = p_active.schedule(
            cartesian_force_nodes, ops_generic + ops_slv + ops_cstr_hdl
        )
        while_schedule.extend(step for step in force_schedule if step not in while_schedule)
        while_schedule.extend(
            p_active.schedule(trailing_evaluators, ops_generic + ops_cstr_hdl)
        )
        for node in trailing_evaluators:
            eval_id = p.id(node)
            if eval_id not in while_schedule:
                while_schedule.append(eval_id)

        # Drop closures another motion declares: the backward walk can reach them, and
        # running them here recomputes that motion's outputs while it is not active.
        motion_token_for_closures = _motion_suffix(p, motion_node)

        def _owned_steps(steps):
            return [
                step
                for step in steps
                if closure_owner_map.get(step, motion_token_for_closures)
                == motion_token_for_closures
            ]

        while_schedule = _owned_steps(while_schedule)
        pre_group_schedule = _owned_steps(pre_group_schedule)
        while_schedule.extend(
            controller.id
            for controller in reversed(active_controllers)
            if controller.id not in while_schedule
        )
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
        while_evaluators = [p.constraint_evaluator(n) for n in while_eval_nodes]
        until_evaluators = [p.constraint_evaluator(n) for n in until_eval_nodes]
        controllers = active_controllers
        forwarding_solvers = set(g.subjects(RDF.type, SLV_EXT.CommandForwardingSolver))
        forwarded_commands = []
        for plan in active_plans:
            if plan.solver not in forwarding_solvers:
                continue
            controller = _derived_controllers(g, p, derivation, plan)[0]
            constraint = plan.constraint
            quantity = g.value(constraint, CSTR.quantity)
            view = next(g.subjects(MAP.subobject, quantity), None)
            target_quantity = g.value(view, MAP.superobject) if view is not None else quantity
            target = g.value(target_quantity, KC_STAT["of-joint"])
            # Resolved from the joint, not the agent: a gripper's joint rides the arm's runtime.
            trees_by_solver_id = {
                solver.id: (solver.owned_trees or ()) for solver in slv_chain
            }
            chain_solver = next(
                (
                    solver
                    for solver in handler_chain_solvers
                    if target is not None
                    and any(
                        _tree_owns(tree, target)
                        for tree in trees_by_solver_id.get(solver.id, ())
                    )
                ),
                None,
            )
            if chain_solver is None:
                raise RuntimeError(
                    "command forwarding: joint "
                    f"'{p.label(target) if target is not None else target}' belongs to no "
                    "kinematic tree this handler's runtimes own"
                )
            runtime_solver = next(solver for solver in slv_chain if solver.id == chain_solver.id)
            forwarded_commands.append(
                ForwardedCommand(
                    f"cmd-fwd-{p.id(plan.controller)}",
                    controller.control_signal,
                    f"{runtime_solver.runtime_prefix}{p.label(target)}"
                    if target is not None
                    else "",
                    chain_solver.id,
                )
            )
        when_monitors = [p.monitor_entry(n) for n in when_mon_nodes]
        while_monitors = [p.monitor_entry(n) for n in while_mon_nodes]
        until_monitors = [p.monitor_entry(n) for n in until_mon_nodes]

        when_elapsed_ids = _elapsed_coordinate_ids(when_evaluators)
        active_elapsed_ids = _elapsed_coordinate_ids(while_evaluators + until_evaluators)
        has_when_elapsed = bool(when_elapsed_ids)
        has_active_elapsed = bool(active_elapsed_ids)
        has_elapsed = has_when_elapsed or has_active_elapsed

        motions.append(
            GuardedMotionBlock(
                id=handler.motion.id,
                handler=handler.id,
                name=handler.motion.name,
                description=(handler.motion.description or "").splitlines(),
                has_when_elapsed=has_when_elapsed,
                has_active_elapsed=has_active_elapsed,
                when_elapsed_ids=when_elapsed_ids,
                active_elapsed_ids=active_elapsed_ids,
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
                has_elapsed=has_elapsed,
                has_until_condition=bool(until_evaluators),
                until_any=handler.motion.until_any,
                when_any=handler.motion.when_any,
                serial_chain_solvers=handler_chain_solvers,
                relative_poses=_relative_poses_for_motion(
                    while_evaluators + when_evaluators + until_evaluators,
                    view_map,
                    handler_chain_solvers,
                ),
                scene_relative_poses=_scene_relative_poses_for_motion(
                    view_map,
                    handler_chain_solvers,
                    while_evaluators + when_evaluators + until_evaluators,
                ),
                pose_axis_error_groups=pose_axis_error_groups,
                while_pre_schedule=pre_group_schedule,
                forwarded_commands=forwarded_commands,
                snapshots=(
                    _snapshots_for_motion(
                        while_evaluators + when_evaluators + until_evaluators,
                        motion.while_ + motion.when + motion.until,
                        snapshot_source_map,
                        view_map,
                        closure_output_map,
                        data_reference_map,
                        while_schedule + when_schedule + until_schedule,
                        closures,
                        snapshot_trigger_map,
                        _motion_suffix(p, motion_node),
                        snapshot_owner_map,
                        motion_tokens,
                    )
                ),
                path_projections=_path_projections_for_motion(
                    pre_group_schedule + while_schedule, closures
                ),
            )
        )

    # Invariant: one motion maps to exactly one constraint handler. A repeated
    # motion id (the same motion driven by two handlers) is rejected rather than
    # silently merged — that ambiguity is a modelling error, not a compose feature.
    handler_by_motion: dict[str, str] = {}
    for motion in motions:
        if motion.id in handler_by_motion:
            raise ValueError(
                f"Motion '{motion.id}' is governed by more than one constraint handler "
                f"('{handler_by_motion[motion.id]}' and '{motion.handler}'). Each motion "
                f"must map to exactly one handler; split the motion or merge the handlers."
            )
        handler_by_motion[motion.id] = motion.handler

    ordered = sorted(
        motions, key=lambda motion: next(h.order for h in handlers if h.id == motion.handler)
    )
    for motion in ordered:
        _set_motion_conditions(motion)
        _add_group_type_flags(motion.pose_axis_error_groups)
        # Controller signal ids + this motion's declared-pose components.
        _annotate_controller_signals(motion.controllers, closures or {})
        motion_refs = collect_motion_references(motion, closures or {})
        motion.declared_pose_components = declared_pose_component_entries(
            data_structures or [], pose_components or {}, motion_refs
        )
    # FSM wiring tags monitors/motions and yields the header/step meta; then the
    # function-interface capability booleans, then the gate calls (which read them).
    fsm_meta = _apply_fsm_wiring(ordered, fsm)
    add_motion_function_interfaces(ordered)
    _apply_fsm_gate_calls(ordered, fsm_meta["cpp_namespace"])
    return ordered, fsm_meta


def _path_projections_for_motion(schedule: list, closures: dict) -> list[dict]:
    """The path projections a motion runs, with the measurements that re-arm on entry."""
    speeds = [
        closures[call]["along_speed"]
        for call in dict.fromkeys(schedule)
        if isinstance(closures.get(call), dict)
        and closures[call].get("type") == "TwistToLinearVelocityAlong"
    ]
    return [
        {
            "id": call,
            "parameter": closures[call]["path_parameter"],
            "along_speed": speed,
        }
        for (call, speed) in zip(
            (
                call
                for call in dict.fromkeys(schedule)
                if isinstance(closures.get(call), dict)
                and closures[call].get("type") == "PathProjection"
            ),
            speeds,
        )
    ]


# ---------------------------------------------------------------------------
# Scene, geometry and robot setups
# ---------------------------------------------------------------------------
def _filter_shared_data(data_structures, schedule, closures, view_map=None, fk_output_ids=None):
    """Select data structures needed by scheduled calls, views, closures, or FK outputs."""
    referenced: set[str] = set(schedule)
    for c in closures.values():
        if isinstance(c, dict):
            for v in c.values():
                # An n-ary port (algo-ext:in) arrives as a list of operand ids.
                for item in v if isinstance(v, list) else [v]:
                    if isinstance(item, str):
                        referenced.add(item)
    if view_map:
        for view in view_map.values():
            for endpoint in (view.superobject, view.subobject):
                if endpoint:
                    referenced.add(endpoint.id)
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
    """Deduplicate items by id, keeping the first occurrence."""
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


_LENGTH_UNITS = {URI_QUDT_UNIT_M, URI_QUDT_UNIT_CM, URI_QUDT_UNIT_MM}

_TEMPORAL_UNITS = {QUDT_UNIT["SEC"], QUDT_UNIT["MilliSEC"]}

# The DSL records the unit a model was written in and never rescales a value, so putting one
# on SI is the reader's job -- codegen emits metres, radians and seconds. A unit absent here
# is already SI (N, N-M, M-PER-SEC2, KiloGM, UNITLESS, ...).
_SI_EQUIVALENT = {
    QUDT_UNIT["CentiM"]: (QUDT_UNIT["M"], 1e-2),
    QUDT_UNIT["MilliM"]: (QUDT_UNIT["M"], 1e-3),
    QUDT_UNIT["DEG"]: (QUDT_UNIT["RAD"], math.pi / 180.0),
    QUDT_UNIT["CentiM-PER-SEC"]: (QUDT_UNIT["M-PER-SEC"], 1e-2),
    QUDT_UNIT["DEG-PER-SEC"]: (QUDT_UNIT["RAD-PER-SEC"], math.pi / 180.0),
    QUDT_UNIT["DEG-PER-SEC2"]: (QUDT_UNIT["RAD-PER-SEC2"], math.pi / 180.0),
    QUDT_UNIT["MilliSEC"]: (QUDT_UNIT["SEC"], 1e-3),
}


def _si_unit(unit):
    """The SI unit `unit` converts to; `unit` itself when it already is SI."""
    return _SI_EQUIVALENT.get(unit, (unit, 1.0))[0]


def _length_unit(coordinate):
    """A position coordinate's length unit, rejecting anything that is not one."""
    if coordinate.unit not in _LENGTH_UNITS:
        raise ConstraintViolation(
            "geometry",
            f"Position coordinate '{coordinate.id}' has an unrecognized length "
            f"unit '{coordinate.unit}'.",
        )
    return coordinate.unit


def _si(value: float, unit) -> float:
    """`value`, authored in `unit`, on SI."""
    return value * _SI_EQUIVALENT.get(unit, (unit, 1.0))[1]


def _si_all(values, unit) -> list[float]:
    """Each of `values`, authored in `unit`, on SI."""
    return [_si(float(value), unit) for value in values]


def _seconds(value: float, unit) -> float:
    """A duration the model authored, in seconds."""
    if unit not in _TEMPORAL_UNITS:
        raise ConstraintViolation("units", f"'{unit}' is not a duration this can read")
    return _si(value, unit)


def _frames_of(g, node):
    if GEOM_ENT.Frame in get_node_types(g, node):
        return [node]
    return [
        frame
        for frame in g.objects(node, GEOM_ENT.simplices)
        if GEOM_ENT.Frame in get_node_types(g, frame)
    ]


def _position_of(g, node):
    """Position of a body or frame from its authored scene-dsl pose, in metres, or None."""
    for frame in _frames_of(g, node):
        origin = g.value(frame, GEOM_ENT.origin) or frame
        for position_id in g.subjects(URI_GEOM_PRED_OF, origin):
            if URI_GEOM_TYPE_POSITION not in get_node_types(g, position_id):
                continue
            position = PositionModel(position_id=position_id, graph=g)
            for coordinate_id in position.coordinate_ids:
                coordinate = PositionCoordModel(
                    coord_id=coordinate_id, graph=g, position=position
                )
                if URI_DISTRIB_TYPE_SAMPLED_QUANTITY in coordinate.types:
                    raise ConstraintViolation(
                        "geometry",
                        f"Sampled placement coordinate '{coordinate.id}' is unsupported",
                    )
                if (value := get_coord_vectorxyz(coordinate, g)) is None:
                    continue
                return _si_all(value, _length_unit(coordinate))
    return None


def _orientation_of(g, node):
    """Rotation of a body or frame from its authored scene pose, as a quaternion
    [x, y, z, w], or None when the pose declares no orientation."""
    for frame in _frames_of(g, node):
        for orientation_node in g.subjects(GEOM_REL.of, frame):
            if GEOM_REL.Orientation not in get_node_types(g, orientation_node):
                continue
            orientation = OrientationModel(orn_id=orientation_node, graph=g)
            for coord_id in orientation.coordinate_ids:
                coord = OrientCoordModel(coord_id=coord_id, graph=g, orientation=orientation)
                if URI_DISTRIB_TYPE_SAMPLED_QUANTITY in coord.types:
                    raise ConstraintViolation(
                        "geometry", f"Sampled placement coordinate '{coord.id}' is unsupported"
                    )
                rotation = get_orientation_coord_vals(coord, g)
                if rotation is not None:
                    return list(rotation.as_quat())
    return None


def _optional_path_of_model(g, model_node):
    """Return an optional agent model path; pathless agent models are not runtime assets."""
    path = g.value(model_node, EXEC.path) if model_node is not None else None
    return str(path) if path is not None else ""


def _model_mappings(g, model, target_type):
    """Return (scene target, model entity) mappings of the requested RDF type.

    scene-dsl reads each mapping; what a mapping means is its metamodel's to say, not ours.
    """
    mappings = (
        get_kinematic_mapping(mapping, g)
        for mapping in sorted(g.objects(model, EXEC["has-mapping"]), key=str)
    )
    return [
        (mapping.target_id, mapping.entity or "")
        for mapping in mappings
        if mapping.target_type == target_type
    ]


def _mapped_targets(g, model_type, target_type):
    """All scene targets of a given type mapped by models of model_type."""
    return {
        target
        for model in g.subjects(RDF.type, model_type)
        for target, _entity in _model_mappings(g, model, target_type)
    }


# The MuJoCo trajectory trace is a viewer-only overlay authored in the scene, not the motion
# spec; it is no longer part of this graph, so it stays disabled and costs nothing for headless
# / non-MuJoCo runtimes. Codegen guards on trace.enabled.
_TRACE_DISABLED = {
    "enabled": False,
    "length": 4096,
    "color_r": 1.0,
    "color_g": 0.5,
    "color_b": 0.1,
    "color_a": 1.0,
    "targets": [],
    "has_targets": False,
}


def _leaf(node):
    return split_uri(str(node))[1]


def _tree_owns(tree, node):
    return iri_is_descendant(tree, node)


def _kinematic_adjacency(g):
    adjacency = collections.defaultdict(list)
    fixed = []
    for joint in g.subjects(RDF.type, KC.Joint):
        frames = list(g.objects(joint, KC["between-attachments"]))
        if len(frames) != 2:
            continue
        body_a, body_b = (body_of_frame(f, g) for f in frames)
        if body_a == body_b:
            continue
        adjacency[body_a].append((body_b, frames[0], frames[1], joint))
        adjacency[body_b].append((body_a, frames[1], frames[0], joint))
        if get_node_types(g, joint) == {KC.Joint}:
            fixed.append((frames[0], frames[1]))
    return adjacency, fixed


def _distances(adjacency, source):
    distances = {source: 0}
    queue = collections.deque([source])
    while queue:
        node = queue.popleft()
        for neighbor, *_ in adjacency[node]:
            if neighbor not in distances:
                distances[neighbor] = distances[node] + 1
                queue.append(neighbor)
    return distances


def _body_path(adjacency, start, end):
    """Oriented body/frame edges on the shortest kinematic path from start to end."""
    parents = {start: None}
    queue = collections.deque([start])
    while queue and end not in parents:
        node = queue.popleft()
        for neighbor, frame, neighbor_frame, joint in adjacency[node]:
            if neighbor not in parents:
                parents[neighbor] = (node, frame, neighbor_frame, joint)
                queue.append(neighbor)
    if end not in parents:
        return []
    path = []
    node = end
    while parents[node] is not None:
        parent, parent_frame, child_frame, joint = parents[node]
        path.append((parent, node, parent_frame, child_frame, joint))
        node = parent
    return list(reversed(path))


def _fixed_attachments(g, bound_trees):
    """Fixed scene/model boundaries, oriented from the world's root toward robot tips."""
    adjacency, fixed = _kinematic_adjacency(g)
    if not fixed:
        return {}, None
    leaves = [body for body in adjacency if len(adjacency[body]) == 1]
    tip_distances = [
        _distances(adjacency, body_of_frame(tip, g))
        for tip in g.objects(None, NS_MM_KC_EXT["tip"])
    ]

    def distance_from_nearest_tip(body):
        distances = [distance[body] for distance in tip_distances if body in distance]
        return min(distances, default=-1)

    root = max(leaves, key=distance_from_nearest_tip) if leaves else None
    from_root = _distances(adjacency, root) if root is not None else {}

    def owner(body):
        return next(
            (
                tree
                for tree in sorted(bound_trees, key=lambda item: (-len(str(item)), str(item)))
                if _tree_owns(tree, body)
            ),
            None,
        )

    modelled_bodies = _mapped_targets(g, ENV["ObjectModel"], GEOM_ENT.RigidBody)
    attachments = {}
    for frame_a, frame_b in fixed:
        parent_frame, child_frame = (
            (frame_a, frame_b)
            if from_root.get(body_of_frame(frame_a, g), 1 << 30)
            <= from_root.get(body_of_frame(frame_b, g), 1 << 30)
            else (frame_b, frame_a)
        )
        parent_body, child_body = (body_of_frame(f, g) for f in (parent_frame, child_frame))
        if parent_body == root:
            attachments[child_body] = ("World", "", child_frame, parent_body)
        elif child_body in modelled_bodies or owner(parent_body) != owner(child_body):
            attachments[child_body] = (
                "Site",
                _leaf(parent_frame),
                child_frame,
                parent_body,
            )
    return attachments, root


def _is_constraint_aggregate(g, node) -> bool:
    """True for an until/when group node: a conjunction or disjunction of constraints."""
    types = get_node_types(g, node)
    return bool({CSTR_EXT.ConstraintDisjunction, CSTR_EXT.ConstraintConjunction} & types)


def _expanded_constraints(g, raw_nodes) -> set:
    """The phase's constraints, with any when/until aggregate replaced by its members."""
    return {
        member
        for node in raw_nodes
        for member in (
            g[node : CSTR_EXT["has-constraint"]] if _is_constraint_aggregate(g, node) else (node,)
        )
    }


def _sensor_kind(g, sensor) -> str:
    """The sensor's kind as the IR names it; empty for a kind codegen does not model."""
    types = get_node_types(g, sensor)
    return next((name for uri, name in SENSOR_KINDS.items() if uri in types), "")


def _device_of(g, element):
    """The deployed system that realizes `element`, or None when nothing does.

    The device is a node the execution context owns; the element it stands for belongs to the
    scene, so the binding is read backwards from the device rather than off the element.
    """
    return next(iter(g.subjects(EXEC["realizes"], element)), None)


def _device_kind(g, element) -> str:
    """The hardware kind realizing `element`, or empty when it is unbound."""
    device = _device_of(g, element)
    return str(g.value(device, SDO.model) or "") if device is not None else ""


def _config_key(g, element, agent, drives: str) -> str:
    """What `robot.toml` calls the device on `element`.

    A hosted sensor is named by its owning agent's leaf plus its own -- `runtime_prefix` inside
    `drives` is empty on a single-robot model, so it cannot be reused here. An agent is named by
    the scenex namespace the model referred to it through, which survives in its IRI as the
    segment before the model's own.
    """
    if drives:
        return f"{_leaf(agent)}.{_leaf(element)}"
    # The set the scene declares the agent in -- `agn set (ns=..) pickplace_agents { agent arm1 }`
    # is addressed as `pickplace_agents.arm1`. Taken from the graph rather than from a segment of
    # the agent's IRI: the IRI path is namespace layout, not the name the model author wrote, and
    # the two disagree whenever the namespace is not called after the set.
    bdd = rdflib.Namespace("https://secorolab.github.io/metamodels/acceptance-criteria/bdd#")
    owner = next(g.subjects(bdd["elements"], element), None)
    if owner is None:
        raise ValueError(f"Agent '{element}' belongs to no declared agent set.")
    return f"{_leaf(owner)}.{_leaf(element)}"


def _bound_devices(g, agent, runtime_prefix, hosted, chain_bindings, agent_by_tree) -> list[dict]:
    """The hardware bound on this chain: what each device is, where it is configured, what it drives.

    The kind is the authored name, passed through untouched: which device a model named is the
    deployment fact, and only the backend's templates interpret it. `drives` names the sensor a
    sensor device reads, and is empty for one that moves a joint.
    """

    def entry(node, drives=""):
        device = _device_of(g, node)
        if device is None:
            return None
        return {
            "kind": str(g.value(device, SDO.model) or ""),
            "config_key": _config_key(g, node, agent, drives),
            "drives": drives,
        }

    owners = [agent]
    for binding in chain_bindings:
        owner = agent_by_tree.get(binding["tree"])
        # A tree may be bound by several models; the agent behind it is named once.
        if owner is not None and owner not in owners:
            owners.append(owner)
    found = [entry(owner) for owner in owners]
    found += [entry(sensor, f"{runtime_prefix}{_leaf(sensor)}") for sensor in hosted]
    return [device for device in found if device is not None]


def _agent_assemblies(g, attach_by_body):
    """Resolve model bindings into runtime robot assets, attachments, and chain bounds."""
    adjacency, _fixed = _kinematic_adjacency(g)
    bound_model_trees = _mapped_targets(g, AGN["AgentModel"], GEOM_ENT.KinematicTree)
    body_names_by_tree = {
        tree: {
            _leaf(body)
            for body in g.subjects(RDF.type, GEOM_ENT.RigidBody)
            if _tree_owns(tree, body)
        }
        for tree in bound_model_trees
    }
    serials = sorted(
        (
            (
                tree,
                g.value(tree, NS_MM_KC_EXT["root"]),
                g.value(tree, NS_MM_KC_EXT["tip"]),
            )
            for tree in g.subjects(RDF.type, URI_KC_TYPE_SERIAL)
        ),
        key=lambda item: str(item[0]),
    )
    # A chain may span several agents; the agent owning its root drives it.
    bindings_by_modelled = {}
    for modelled in sorted(g.subjects(RDF.type, AGN.ModelledAgent), key=str):
        rows = []
        for model in sorted(g.objects(modelled, AGN["has-agent-model"]), key=str):
            path = _optional_path_of_model(g, model)
            if not path:
                continue
            for tree, entity in _model_mappings(g, model, GEOM_ENT.KinematicTree):
                rows.append({"model": model, "tree": tree, "path": path, "entity": entity})
        bindings_by_modelled[modelled] = rows
    bindings = [row for rows in bindings_by_modelled.values() for row in rows]
    agent_by_tree = {
        row["tree"]: g.value(modelled, AGN["of-agent"])
        for modelled, rows in bindings_by_modelled.items()
        for row in rows
    }

    def binding_for(node):
        return next((binding for binding in bindings if _tree_owns(binding["tree"], node)), None)

    result = []
    for modelled in sorted(g.subjects(RDF.type, AGN.ModelledAgent), key=str):
        agent = g.value(modelled, AGN["of-agent"])
        own = bindings_by_modelled[modelled]
        if agent is None or not own:
            continue

        def own_binding_for(node, own=own):
            return next((binding for binding in own if _tree_owns(binding["tree"], node)), None)

        serial = next(
            (
                (tree, root, tip)
                for tree, root, tip in serials
                if root is not None
                and tip is not None
                and (
                    any(binding["tree"] == tree for binding in own)
                    or own_binding_for(root) is not None
                )
            ),
            None,
        )
        if serial is None:
            continue
        serial_tree, root_frame, tip_frame = serial
        root_binding = own_binding_for(root_frame) or next(
            binding for binding in own if binding["tree"] == serial_tree
        )
        tip_binding = binding_for(tip_frame) or root_binding
        root_body, tip_body = (body_of_frame(f, g) for f in (root_frame, tip_frame))
        duplicate_root = sum(
            _leaf(root_body) in names for names in body_names_by_tree.values()
        ) > 1
        runtime_prefix = f"{_leaf(root_binding['tree'])}_" if duplicate_root else ""
        runtime_root = f"{runtime_prefix}{_leaf(root_body)}"
        path = _body_path(adjacency, root_body, tip_body)
        # Scoped to this chain's path: two arms must not claim each other's models.
        chain_bodies = [root_body, tip_body, *(body for edge in path for body in edge[:2])]
        chain_bindings = [
            binding
            for binding in bindings
            if any(_tree_owns(binding["tree"], body) for body in chain_bodies)
        ]
        chain_tip_body = tip_body if root_binding["tree"] == serial_tree else root_body
        if root_binding["tree"] != serial_tree:
            for parent_body, child_body, *_ in path:
                if _tree_owns(root_binding["tree"], child_body):
                    chain_tip_body = child_body
                elif _tree_owns(root_binding["tree"], parent_body):
                    chain_tip_body = parent_body
                    break

        attachments = []
        for binding in chain_bindings:
            if binding is root_binding or binding["path"] == root_binding["path"]:
                continue
            boundary = next(
                (
                    edge
                    for edge in path
                    if _tree_owns(binding["tree"], edge[1])
                    and not _tree_owns(binding["tree"], edge[0])
                ),
                None,
            )
            if boundary is None:
                continue
            _parent_body, child_body, parent_frame, _child_frame, _joint = boundary
            entity = binding["entity"]
            child_name = _leaf(child_body)
            prefix = child_name[: -len(entity)] if entity and child_name.endswith(entity) else ""
            attachments.append(
                SceneAttachment(
                    id=_leaf(binding["tree"]),
                    path=binding["path"],
                    attach_to=_leaf(parent_frame),
                    attach_kind="Site",
                    prefix=prefix,
                )
            )

        attach_kind, attach_name, placement_frame, _parent_body = attach_by_body.get(
            root_body, ("World", "", root_frame, None)
        )
        hosted = sorted(g.objects(modelled, SOSA.hosts), key=str)
        sensors = [
            {
                "id": f"{runtime_prefix}{_leaf(sensor)}",
                "type": kind,
                "frame_site": f"{runtime_prefix}{_leaf(frame)}",
                "update_rate_hz": get_update_rate(g, ModelBase(node_id=sensor, graph=g)),
                "observes": sorted(_leaf(observed) for observed in g.objects(sensor, SOSA.observes)),
            }
            for sensor in hosted
            if (kind := _sensor_kind(g, sensor))
            and (frame := g.value(sensor, SENSORS.frame)) is not None
        ]
        agent_device = _device_of(g, agent)
        device = str(g.value(agent_device, SDO.model) or "") if agent_device else ""
        # An agent is named by the scenex alias it was referred to through, whether or not
        # hardware is bound to it: a simulated deployment addresses it in exactly the same
        # way to state where it starts.
        config_key = _config_key(g, agent, agent, "")
        result.append(
            {
                "agent": agent,
                "device": device,
                "config_key": config_key,
                "sensors": sensors,
                "devices": _bound_devices(
                    g, agent, runtime_prefix, hosted, chain_bindings, agent_by_tree
                ),
                "path": root_binding["path"],
                "prefix": runtime_prefix,
                "trees": [binding["tree"] for binding in chain_bindings],
                "serial_chain": serial_tree,
                "root_body": root_body,
                "chain_root": runtime_root,
                "chain_tip": f"{runtime_prefix}{_leaf(chain_tip_body)}",
                "tool_body": (
                    f"{runtime_prefix}{_leaf(tip_body)}"
                    if tip_binding is not root_binding
                    else ""
                ),
                "tcp_site": (
                    f"{runtime_prefix}{_leaf(tip_frame)}"
                    if tip_binding is not root_binding
                    else ""
                ),
                "attach_kind": attach_kind,
                "attach_name": attach_name,
                "placement_frame": placement_frame,
                "attachments": attachments,
            }
        )
    return result


def _scene_from_graph(g):
    """Build the scene (robots + objects with model paths, placement and attachment) from
    the scene-dsl (`.scenex`) graph. Geometry comes from the referenced mjcf assets, so
    procedural geometry fields stay unset; placement between attached frames is coincident
    (identity), and the weld target (`attach_kind`/`attach_name`) is derived from the
    fixed-joint tree by `_fixed_attachments`.
    """
    scene = SceneSpec()
    context = next(g.subjects(RDF.type, EXEC.ExecutionContext), None)
    if context is not None:
        timestep = g.value(context, EXEC.timestep)
        value = g.value(timestep, QUDT_SCHEMA.value)
        if value is not None:
            scene.timestep_s = _seconds(
                float(value.toPython()), g.value(timestep, QUDT_SCHEMA.unit)
            )

    bound_trees = _mapped_targets(g, AGN["AgentModel"], GEOM_ENT.KinematicTree)
    attach_by_body, _root = _fixed_attachments(g, bound_trees)
    object_ids_by_body = {
        body: _leaf(obj)
        for modelled in g.subjects(RDF.type, ENV["ModelledObject"])
        if (obj := g.value(modelled, ENV["of-object"])) is not None
        for model in g.objects(modelled, ENV["has-object-model"])
        for body, _entity in _model_mappings(g, model, GEOM_ENT.RigidBody)
    }
    for body, (kind, name, frame, parent_body) in list(attach_by_body.items()):
        if kind == "Site" and parent_body in object_ids_by_body:
            parent_frame = rdflib.Namespace(f"{parent_body}/")[name]
            reference_frame = next(
                (
                    reference
                    for pose in g.subjects(GEOM_REL.of, parent_frame)
                    if (reference := g.value(pose, GEOM_REL["with-respect-to"])) is not None
                    and GEOM_ENT.Frame in get_node_types(g, reference)
                    and body_of_frame(reference, g) == parent_body
                ),
                None,
            )
            if reference_frame is not None:
                name = _leaf(reference_frame)
                frame = parent_frame
            name = f"{object_ids_by_body[parent_body]}_{name}"
        attach_by_body[body] = (kind, name, frame, parent_body)
    for modelled in sorted(g.subjects(RDF.type, ENV["ModelledObject"]), key=str):
        obj = g.value(modelled, ENV["of-object"])
        mapped = next(
            (
                (model, body)
                for model in sorted(g.objects(modelled, ENV["has-object-model"]), key=str)
                for body, _entity in _model_mappings(g, model, GEOM_ENT.RigidBody)
            ),
            None,
        )
        if obj is None or mapped is None:
            continue
        model, body = mapped
        path = get_path_of_node(g, model)
        attach_kind, attach_name, placement_frame, _parent_body = attach_by_body.get(
            body, ("World", "", body, None)
        )
        scene.objects.append(
            SceneObjectSpec(
                id=_leaf(obj),
                body=_leaf(body),
                path=path,
                fixed=body in attach_by_body,
                attach_kind=attach_kind,
                attach_name=attach_name,
                pos=_position_of(g, placement_frame),
                quat=_orientation_of(g, placement_frame),
            )
        )

    for assembly in _agent_assemblies(g, attach_by_body):
        scene.robots.append(
            SceneRobot(
                id=_leaf(assembly["agent"]),
                path=assembly["path"],
                prefix=assembly["prefix"],
                attach_kind=assembly["attach_kind"],
                attach_name=assembly["attach_name"],
                pos=_position_of(g, assembly["placement_frame"]),
                quat=_orientation_of(g, assembly["placement_frame"]),
                attachments=assembly["attachments"],
            )
        )

    _expand_scene_geometry(scene)
    return scene

def _expand_scene_geometry(scene) -> None:
    """Expand placement vectors and (present) procedural geometry onto the native scene
    items so the scene is codegen-complete at construction. Env placement shorthand: an
    omitted position/orientation means zero/identity. Path-backed objects take geometry
    from their MJCF/URDF asset, so their flat size/color/friction fields stay unset.
    Missing required geometry is not raised here (that is ``_validate_scene``) so building a
    scene never depends on a downstream pass."""
    for robot in scene.robots:
        expand_vector_fields(robot, "pos")
        expand_vector_fields(robot, "quat", ("x", "y", "z", "w"), default=[0.0, 0.0, 0.0, 1.0])
        for attachment in robot.attachments:
            expand_vector_fields(attachment, "pos")
            expand_vector_fields(
                attachment, "quat", ("x", "y", "z", "w"), default=[0.0, 0.0, 0.0, 1.0]
            )
    for obj in scene.objects:
        expand_vector_fields(obj, "pos")
        expand_vector_fields(obj, "quat", ("x", "y", "z", "w"), default=[0.0, 0.0, 0.0, 1.0])
        obj.has_path = bool(obj.path)
        if obj.has_path:
            continue
        if obj.size is not None:
            obj.size_x, obj.size_y, obj.size_z = (
                float(obj.size[0]),
                float(obj.size[1]),
                float(obj.size[2]),
            )
        if obj.color is not None:
            obj.color_r, obj.color_g, obj.color_b, obj.color_a = (
                float(obj.color[0]),
                float(obj.color[1]),
                float(obj.color[2]),
                float(obj.color[3]),
            )
        if obj.friction is not None:
            obj.friction_slide, obj.friction_torsion, obj.friction_roll = (
                float(obj.friction[0]),
                float(obj.friction[1]),
                float(obj.friction[2]),
            )


def _validate_scene(scene) -> None:
    """Every procedural (non-path) scene object must carry full geometry — the model must
    declare it; silent defaults are not applied. Runs at construction time in generate_ir."""
    for obj in scene.objects:
        if bool(obj.path):
            continue
        require_field(obj.id, "size", obj.size)
        require_field(obj.id, "color", obj.color)
        require_field(obj.id, "friction", obj.friction)
        require_field(obj.id, "shape", obj.shape)
        require_field(obj.id, "mass", obj.mass)


def _scene_chain(trees, assembly):
    """The agent assembly's declared serial chain, with MuJoCo runtime joint names."""
    from motion_spec.generation.scene_kdl import chain_for_iri

    name, tree, joints = chain_for_iri(trees, str(assembly["serial_chain"]))
    return name, tree, [f"{assembly['prefix']}{joint}" for joint in joints]


def _robot_setups_from_graph(g):
    """Per-robot solver chain setups, sourced from the scene-dsl (`.scenex`) graph.

    Returns ``(setups_by_node, ordered)`` where ``setups_by_node`` maps each robot's
    abstract agent node (the target of a solver's ``agn:of-agent``) to its setup tuple
    ``(urdf, chain_root, chain_end, chain_tip, robot_model, tool_body, tcp_site,
    sensors, devices, runtime_prefix, owned_trees, kdl_chain, kdl_tree, kdl_joints,
    config_key)``.

    ``kdl_chain`` names the scene-derived chain builder emitted beside the controller and
    ``kdl_joints`` lists its joints as MuJoCo knows them, in KDL order -- see plan 013.

    Chain bodies come from a serial-composition ``geom:KinematicTree``'s
    ``kc-ext:root`` / ``kc-ext:tip`` frames. A scene-dsl frame URI is
    ``.../<robot>/<body>/<frame|site>``, so the body is the second-to-last path
    segment and the tip site is the last. The model path comes from the robot's
    ``agn:ModelledAgent`` -> ``agn:has-agent-model`` -> ``exec-ctx:path``.
    """
    def _robot_model_from_path(path):
        low = str(path).lower()
        for hint, canonical in (("kinova_gen3", "KinovaGen3"), ("gen3", "KinovaGen3")):
            if hint in low:
                return canonical
        return ""

    def _robot_model_for(assembly):
        """The agent's device: authored when bound, else sniffed from the asset path."""
        # The arm-with-gripper pairing is wiring; the manipulator is still the arm.
        device = assembly.get("device") or ""
        if device:
            return "KinovaGen3" if device == "KinovaGen3-2F85" else device
        return _robot_model_from_path(assembly["path"])

    from scene_dsl.kdl_tree import build_kdl_trees

    try:
        trees = build_kdl_trees(g)
    except ConstraintViolation:
        # Some graph-only consumers use an incomplete scene fixture. They retain their
        # assembly metadata but cannot provide a KDL chain until Scene DSL can parse it.
        trees = []
    setups_by_node, ordered = {}, []
    bound_trees = _mapped_targets(g, AGN["AgentModel"], GEOM_ENT.KinematicTree)
    attach_by_body, _root = _fixed_attachments(g, bound_trees)
    for assembly in _agent_assemblies(g, attach_by_body):
        setup = (
            assembly["path"],
            assembly["chain_root"],
            assembly["chain_tip"],
            assembly["chain_tip"],
            _robot_model_for(assembly),
            assembly["tool_body"],
            assembly["tcp_site"],
            assembly["sensors"],
            assembly["devices"],
            assembly["prefix"],
            assembly["trees"],
            *_scene_chain(trees, assembly),
            assembly.get("config_key") or "",
        )
        setups_by_node[assembly["agent"]] = setup
        ordered.append(setup)
    return setups_by_node, ordered


# ---------------------------------------------------------------------------
# Introspection artifact
# ---------------------------------------------------------------------------
def _uri_table(id_nodes):
    """Sorted [{id, uri}] rows for every id that maps to a URIRef."""
    return [
        {"id": id_, "uri": str(node)}
        for id_, node in sorted(id_nodes, key=lambda item: (item[0], str(item[1])))
        if isinstance(node, URIRef)
    ]


def _kebab(text: str) -> str:
    """Kebab-case an id fragment for use as an IRI path segment."""
    return text.replace("_", "-").lower()


class DerivedIriRegistry:
    """IRIs for codegen-derived entities, minted as a path segment under the parent they came from.

    An id is a lossy projection of its IRI (Parser.id keeps only the local name), so a derived
    entity's IRI cannot be recovered from its id downstream -- it has to be recorded where the
    derivation happens, against the parent node that is still in hand there.
    """

    SPECIALIZATION = "specializationOf"
    DERIVATION = "wasDerivedFrom"

    def __init__(self, id_nodes):
        self._authored = {
            id_: str(node) for id_, node in id_nodes if isinstance(node, URIRef)
        }
        self._derived: dict[str, dict] = {}

    def iri_of(self, id_):
        """IRI for an id, authored or already derived; None when neither."""
        entry = self._derived.get(id_)
        return entry["uri"] if entry else self._authored.get(id_)

    def register(self, id_, parent_iri, suffix, relation, types=()):
        """Mint <parent_iri>/<suffix> for id_ and record how it relates to its parent."""
        if not id_ or not parent_iri:
            return None
        # An authored node always wins: a derived IRI must never shadow a model's own.
        authored = self._authored.get(id_)
        if authored is not None:
            return authored
        uri = f"{parent_iri.rstrip('/')}/{_kebab(suffix)}"
        existing = self._derived.get(id_)
        if existing is not None:
            if existing["uri"] != uri:
                raise RuntimeError(
                    f"derived IRI collision: '{id_}' minted as both "
                    f"{existing['uri']} and {uri}"
                )
            return uri
        self._derived[id_] = {
            "uri": uri,
            "parent": parent_iri,
            "relation": relation,
            "types": list(types),
        }
        return uri

    def rows(self):
        """[{id, uri}] rows for the introspection uris table, sorted by id."""
        return [
            {"id": id_, "uri": entry["uri"]}
            for id_, entry in sorted(self._derived.items())
        ]

    def nodes(self):
        """Derivation-graph nodes: what each derived entity is, and what it came from."""
        return [
            {
                "id": entry["uri"],
                "types": ["prov:Entity", *entry["types"]],
                "relation": entry["relation"],
                "parent": entry["parent"],
            }
            for _, entry in sorted(self._derived.items())
        ]


def _id_ref(value):
    """Normalize a value to its id string (str/Enum/.id), or None."""
    if value is None:
        return None
    if isinstance(value, str):
        return value
    if isinstance(value, Enum):
        return value.value
    return getattr(value, "id", None)


def _prune(row: dict) -> dict:
    """Drop the keys an introspection row leaves unset."""
    return {k: v for k, v in row.items() if v is not None and v != []}


def _dedupe_dicts(entries):
    """Deduplicate dict rows by id, keeping the first occurrence."""
    result = []
    seen = set()
    for entry in entries:
        value = entry.get("id")
        if value in seen:
            continue
        seen.add(value)
        result.append(entry)
    return result


def _build_introspection(
    *,
    app_model_path,
    imported_models,
    imported_provenance,
    id_nodes,
    node_by_id,
    motions,
    data_structures,
    control_period_ns,
    backend,
    scene,
    closures,
    views,
    shared_data,
    serial_chain_solvers,
    platform,
    iris,
):
    """Build the introspection artifact (uris, motions, controllers, monitors, quantities,
    provenance) and fold in the controller-state and frame-log samples.
    """
    # Appended, never substituted: authored nodes keep their own IRIs, derived ones extend them.
    # Last-wins on a repeated id is deliberate and predates this -- constraint names, metamodel
    # predicates and aliases legitimately share a bare id (see Parser.assert_no_id_collisions,
    # which polices only the context quantities where a merge would be silent).
    uri_rows = _uri_table(id_nodes) + iris.rows()
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
                "proportional_gain": getattr(controller, "proportional_gain", None),
                "integral_gain": getattr(controller, "integral_gain", None),
                "derivative_gain": getattr(controller, "derivative_gain", None),
                "decay_rate": getattr(controller, "decay_rate", None),
                "stiffness": getattr(controller, "stiffness", None),
                "damping": getattr(controller, "damping", None),
                "error_signal": _id_ref(getattr(controller, "error_signal", None)),
                "tolerance_signal": getattr(controller, "tolerance_id", "") or None,
                "reference_signal": _id_ref(getattr(controller, "reference_signal", None)),
                "measured_derivative": _id_ref(getattr(controller, "measured_derivative", None)),
                "output_signal": _id_ref(controller.control_signal),
            }
            controllers.append(_prune(controller_entry))
            for role in (
                "error_signal",
                "reference_signal",
                "measured_derivative",
                "control_signal",
            ):
                quantity_id = _id_ref(getattr(controller, role, None))
                if quantity_id:
                    signal_entry = {
                        "id": f"{controller.id}.{role}",
                        "uri": uri_by_id.get(quantity_id),
                        "quantity": quantity_id,
                        "role": role,
                        "owner": controller.id,
                    }
                    signals.append(_prune(signal_entry))
        for phase in ("when", "while", "until"):
            for monitor in getattr(motion, f"{phase}_monitors"):
                monitor_entry = {
                    "id": monitor.id,
                    "uri": uri_by_id.get(monitor.id),
                    "motion": motion.id,
                    "phase": phase,
                    "type": monitor.monitor_type,
                    "trigger": "edge" if monitor.is_edge_triggered else "level",
                    "event": getattr(monitor, "event", None),
                    "event_uri": getattr(monitor, "event_uri", None),
                    "event_name": getattr(monitor, "event_name", None),
                    "flag": getattr(monitor, "flag", None),
                    "error_signal": _id_ref(monitor.error),
                    "tolerance_signal": _id_ref(getattr(monitor, "tolerance", None)),
                    "fallback_motion": getattr(monitor, "fallback_motion", None),
                    "debounce_duration_s": getattr(monitor, "debounce_duration_s", None),
                    "debounce_steps": getattr(monitor, "debounce_steps", None),
                }
                monitors.append(_prune(monitor_entry))
                if monitor.error is not None:
                    signal_entry = {
                        "id": f"{monitor.id}.error",
                        "uri": uri_by_id.get(monitor.error.id),
                        "quantity": monitor.error.id,
                        "role": "monitor_error",
                        "owner": monitor.id,
                    }
                    signals.append(_prune(signal_entry))

    quantities = []
    for item in data_structures:
        quantity_entry = {
            "id": item.id,
            "uri": uri_by_id.get(item.id),
            "type": item.type,
            "reference_value": getattr(item, "reference_value", None),
            "value": getattr(item, "value", None),
            "authored": getattr(getattr(item, "provenance", None), "authored", False),
            "snapshot": getattr(getattr(item, "provenance", None), "snapshot", False),
        }
        quantities.append(_prune(quantity_entry))

    # One authored fact -- the exec-context's platform -- decides all three. Never re-derived from
    # the backend token, and never by matching substrings of the agent id downstream.
    runtime_type = "exec:Simulation" if platform["simulated"] else "exec:RealWorld"
    runtime_id = f"agent:runtime:{get_valid_var_name(platform['name']).casefold()}" if platform[
        "simulated"
    ] else "agent:runtime:real_robot"
    runtime_activity_type = (
        "bdd:SimulatedExecution" if platform["simulated"] else "bdd:ScenarioExecution"
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
        }
        for idx, source in enumerate(imported_models)
    )
    entities.extend(
        {
            "id": f"entity:imported_provenance:{idx}",
            "types": ["prov:Entity"],
            "role": "imported_provenance",
            "source": source,
        }
        for idx, source in enumerate(imported_provenance)
    )

    agents = [
        {
            "id": "agent:motion_spec_ir_gen",
            "types": ["prov:SoftwareAgent", "obs:ObservationProvider"],
            "role": "ir_generator",
        },
        {"id": runtime_id, "types": ["prov:SoftwareAgent", runtime_type], "role": "runtime_runner"},
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
            "types": ["prov:Agent", "agn:ModelledAgent"],
            "role": "robot",
            "model": robot.path,
        }
        for robot in scene.robots
    )

    introspection = {
        "contract_version": 1,
        "control_period_ns": control_period_ns,
        "uris": uri_rows,
        "motions": [
            _prune(
                {
                    "id": motion.id,
                    "uri": uri_by_id.get(motion.id),
                    "handler": motion.handler,
                    "handler_uri": uri_by_id.get(motion.handler),
                    "controllers": [controller.id for controller in motion.controllers],
                    "monitors": [
                        monitor.id
                        for group in (
                            motion.when_monitors,
                            motion.while_monitors,
                            motion.until_monitors,
                        )
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
            # Only id/uri are consumed; the canonical uri is the published IRI, which
            # the rdf-utils resolver maps to a local checkout. No local source/shape
            # paths are baked in — they were dead metadata and non-portable.
            "contexts": [
                {"id": "prov", "uri": "http://www.w3.org/ns/prov#"},
                {
                    "id": "bdd",
                    "uri": "https://secorolab.github.io/metamodels/acceptance-criteria/bdd#",
                },
                {"id": "agent", "uri": "https://secorolab.github.io/metamodels/agent#"},
                {"id": "observation", "uri": "https://secorolab.github.io/metamodels/observation#"},
                {"id": "execution-context", "uri": str(EXEC.ExecutionContext)},
            ],
            "entities": entities,
            "activities": [
                {
                    "id": "activity:motion_spec_ir_generation",
                    "types": ["prov:Activity"],
                    "used": [
                        entity["id"] for entity in entities if entity["role"] != "motion_spec_ir"
                    ],
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

    # Fold the introspection-facing derivations into the introspection piece itself:
    # controller signal ids, controller-internal-state logging (which grows shared_data),
    # then the frame-log quantity/spatial samples that read them.
    _annotate_controller_signals(introspection["controllers"], closures)
    add_controller_internal_state_logging(closures, shared_data, introspection, motions, iris)
    add_control_parameters(closures, shared_data, introspection, motions, iris)
    add_joint_space_logging(
        serial_chain_solvers, motions, shared_data, introspection, backend, iris
    )
    # Both lists are complete here and the sample passes below turn their order into indices the
    # frame layout and the generated struct are built from: order them once, before that happens.
    shared_data.sort(key=lambda item: _field(item, "id") or "")
    introspection["quantities"].sort(key=lambda row: row.get("id") or "")
    add_quantity_samples(introspection, shared_data, views)
    add_spatial_samples(introspection, shared_data)
    values = annotate_dataflow(
        introspection, shared_data, closures, motions, serial_chain_solvers, views
    )
    # The derivation registry grew while folding the samples in, so the table is rebuilt here and
    # the rows built before that are backfilled from it.
    introspection["uris"] = _uri_table(id_nodes) + iris.rows()
    introspection["derivations"] = iris.nodes()
    _backfill_uris(introspection)
    _assert_every_id_resolves(introspection)
    return introspection, values


def _backfill_uris(introspection: dict) -> None:
    """Attach the IRI to every row minted before the derivation registry was complete."""
    uri_by_id = {
        row["id"]: row["uri"] for row in introspection.get("uris", []) if row.get("uri")
    }
    for key in ("controllers", "monitors", "motions", "quantities", "quantity_samples"):
        for row in introspection.get(key) or ():
            if isinstance(row, dict) and not row.get("uri"):
                uri = uri_by_id.get(row.get("source_id") or row.get("id"))
                if uri:
                    row["uri"] = uri
    for rows in (introspection.get("spatial_samples") or {}).values():
        for row in rows:
            if isinstance(row, dict) and not row.get("uri"):
                uri = uri_by_id.get(row.get("id"))
                if uri:
                    row["uri"] = uri


# Ids that name a slot in the frame log or a row in the introspection artifact. Every one of them
# has to resolve to an IRI, or a run graph cannot make a statement about what the log recorded.
def _assert_every_id_resolves(introspection: dict) -> None:
    """Fail loudly when an introspection id has no IRI, listing every one rather than the first."""
    uri_by_id = {
        row["id"]: row["uri"] for row in introspection.get("uris", []) if row.get("uri")
    }
    unresolved: dict[str, set] = {}

    def check(id_, origin: str) -> None:
        if isinstance(id_, str) and id_ and id_ not in uri_by_id:
            unresolved.setdefault(id_, set()).add(origin)

    def check_row(row, key: str) -> None:
        """A row is resolved if it carries a uri; otherwise its id must be in the table."""
        if _field(row, "uri"):
            return
        # A signal row is a binding, not an entity: its id is a synthetic `<owner>.<role>` and
        # its uri is the quantity's, so the quantity is what has to resolve.
        if key == "signals":
            check(_field(row, "quantity"), key)
            return
        check(_field(row, "source_id") or _field(row, "id"), key)

    for key in ("controllers", "monitors", "motions", "quantities", "signals", "quantity_samples"):
        for row in introspection.get(key) or ():
            check_row(row, key)
    for pool, rows in (introspection.get("spatial_samples") or {}).items():
        for row in rows:
            check(_field(row, "id"), f"spatial_samples.{pool}")
    for member_id, entry in (introspection.get("dataflow") or {}).items():
        check(member_id, "dataflow")
        check((entry.get("producer") or {}).get("id"), "dataflow.producer")

    if unresolved:
        details = "\n".join(
            f"  '{id_}' (from {', '.join(sorted(origins))})"
            for id_, origins in sorted(unresolved.items())
        )
        raise RuntimeError(
            "introspection: ids with no IRI -- a derived entity was minted without registering "
            f"its IRI against the node it came from:\n{details}"
        )


# ---------------------------------------------------------------------------
# Graph loading
# ---------------------------------------------------------------------------
def _resolve_import_location(location: str, url_map: dict[str, str]) -> str:
    """Map an import-location URL to its local file path via the url map."""
    for base, root in sorted(url_map.items(), key=lambda item: len(item[0]), reverse=True):
        if location.startswith(base):
            return str((Path(root) / location[len(base) :]).resolve())
    return location


def _load_graph(manifest_path):
    """Load the app manifest and its imports into one dataset; return (app path, graph, imported
    models, imported provenance).
    """
    app_model_path = Path(manifest_path).resolve()
    g = rdflib.Dataset(default_union=True)
    install_resolver(IriToFileResolver(metamodel_url_map(), download=False))
    g.parse(str(app_model_path), format="json-ld")

    url_map = build_url_map(g, app_model_path)
    install_resolver(IriToFileResolver({**metamodel_url_map(), **url_map}, download=False))

    imported_files = list(dict.fromkeys(str(model) for model in g.objects(predicate=APP["import"])))
    _PROV_SUFFIX = "/provenance/dsl.ld.json"
    imported_provenance = [
        _resolve_import_location(item, url_map)
        for item in imported_files
        if item.endswith(_PROV_SUFFIX)
    ]
    imported_model_locations = [i for i in imported_files if not i.endswith(_PROV_SUFFIX)]
    imported_models = [_resolve_import_location(item, url_map) for item in imported_model_locations]
    for model in imported_model_locations:
        g.parse(location=model, format="json-ld")
    return app_model_path, g, imported_models, imported_provenance


def _node_indexes(g, p: Parser):
    """Build the (node_by_id, id_nodes) lookup indexes from the parser."""
    node_by_id = {}
    id_nodes = []
    for node in sorted(g.subjects(), key=lambda item: str(item)):
        try:
            id_ = p.id(node)
        except Exception:
            continue
        id_nodes.append((id_, node))
        node_by_id.setdefault(id_, node)
    return node_by_id, id_nodes


# ---------------------------------------------------------------------------
# Solver sections
# ---------------------------------------------------------------------------
def _world_solver_outputs(
    g, p: Parser, chain_root: str, runtime_prefix: str, owned_trees, scene_objects
):
    """Parse runtime observations in the solver's reference frame."""
    object_ids_by_body = {obj.body: obj.id for obj in scene_objects}
    outputs = []

    def _runtime_frame(frame_node):
        """Runtime body/site name for a scene frame owned by this solver."""
        frame = p.frame(frame_node)
        frame_tree = iri_parent(body_of_frame(frame_node, g))
        return replace(
            frame,
            id=f"{runtime_prefix}{frame.id}" if frame_tree in owned_trees else frame.id,
        )

    for type_, parse in (
        (GEOM_COORD.PoseCoordinate, p.pose),
        (GEOM_COORD.VelocityTwistCoordinate, p.velocity_twist),
        (KC_STAT.JointPositionCoordinate, p.joint_position),
        (RBDYN_COORD.WrenchCoordinate, p.wrench),
    ):
        for node in sorted(g.subjects(RDF.type, type_), key=str):
            scope = p._context_scope(node)
            if scope is None or scope[1] != "world":
                continue
            if type_ == KC_STAT.JointPositionCoordinate:
                joint = g.value(node, KC_STAT["of-joint"])
                if joint is None or not any(_tree_owns(tree, joint) for tree in owned_trees):
                    continue
            frame_node = None
            if type_ != KC_STAT.JointPositionCoordinate:
                seen_by_predicate = (
                    RBDYN_COORD["as-seen-by"]
                    if type_ == RBDYN_COORD.WrenchCoordinate
                    else GEOM_COORD["as-seen-by"]
                )
                frame_node = g.value(node, seen_by_predicate)
                if frame_node is None:
                    _of, _wrt, frame_node = p._derived_reference_frames(node)
            if type_ == RBDYN_COORD.WrenchCoordinate:
                sensor = g.value(node, SOSA.madeBySensor)
                sensor_frame_node = g.value(sensor, SENSORS.frame) if sensor is not None else None
                if sensor_frame_node is None or not any(
                    _tree_owns(tree, sensor_frame_node) for tree in owned_trees
                ):
                    continue
            elif frame_node is not None:
                frame_body = body_of_frame(frame_node, g)
                frame_tree = iri_parent(frame_body)
                runtime_frame = _leaf(frame_body)
                if frame_tree in owned_trees:
                    runtime_frame = f"{runtime_prefix}{_leaf(frame_body)}"
                if runtime_frame != chain_root:
                    continue
            output = parse(node)
            if type_ == KC_STAT.JointPositionCoordinate:
                output = replace(output, joint_name=f"{runtime_prefix}{output.joint_name}")
            elif type_ == RBDYN_COORD.WrenchCoordinate:
                relation = g.value(node, RBDYN_COORD["of-wrench"])
                reference_node = g.value(relation, RBDYN_ENT["reference-point"])
                output = replace(
                    output,
                    sensor_name=f"{runtime_prefix}{output.sensor_name}",
                    sensor_frame=_runtime_frame(sensor_frame_node),
                    reference_point=replace(
                        output.reference_point, id=_runtime_frame(reference_node).id
                    ),
                    as_seen_by=_runtime_frame(frame_node),
                )
            frame = getattr(output, "as_seen_by", None)
            if frame_node is None and frame is not None and frame.id != chain_root:
                continue
            of = getattr(output, "of", None)
            if getattr(of, "id", None) in object_ids_by_body:
                output.of = SceneObject(object_ids_by_body[of.id], of.id)
            outputs.append(output)
    return _dedupe_by_id(outputs)


def _solver_sections(
    g,
    p: Parser,
    setups_by_node: dict,
    default_setup,
    derivation: SolverDerivationContext,
    scene_objects,
):
    """Parse the solver/handler sections: base-velocity, arm and base-force solvers with their
    schedules.
    """
    sched1 = []
    slv_platform_vel = []
    sched2 = []
    hdl = []
    sched3 = []
    slv_chain = []
    sched4 = []
    slv_platform_frc = []

    for s in sorted(g.subjects(RDF.type, SLV["VelocityCompositionSolver"]), key=str):
        slv_platform_vel.append(p.velocity_composition_solver(s))
        sched1.extend(p.schedule([s], ops_generic + ops_slv))

    handler_nodes = sorted(
        g.subjects(RDF.type, CSTR_HDL["ConstraintHandler"]),
        key=lambda node: int(getattr(g.value(node, APP.order), "value", 0)),
    )
    for h in handler_nodes:
        handler = p.constraint_handler(h)
        handler.controllers = [
            controller
            for plan in derivation.controllers_by_handler.get(h, ())
            for controller in _derived_controllers(g, p, derivation, plan)
        ]
        hdl.append(handler)
        evaluator_nodes = list(g.objects(h, CSTR_HDL.evaluators))
        sched2.extend(p.schedule(evaluator_nodes, ops_generic + ops_cstr_hdl))
        plans = derivation.controllers_by_handler.get(h, ())
        for plan in plans:
            if len(plan.axes) > 1 or CSTR_HDL_EXT.FeedForwardController in get_node_types(
                g, plan.controller
            ):
                continue
            error = g.value(plan.controller, CSTR_HDL["error-signal"])
            evaluator = next(g.subjects(CSTR_HDL.error, error), None)
            if evaluator is not None:
                evaluator_id = p.id(evaluator)
                if evaluator_id not in sched2:
                    sched2.append(evaluator_id)
        sched2.extend(
            controller.id
            for plan in reversed(plans)
            for controller in reversed(_derived_controllers(g, p, derivation, plan))
        )
        sched2.extend(
            SolverIdFactory(p.id(plan.controller), _motion_suffix(p, plan.motion)).pose_evaluator()
            for plan in reversed(plans)
            if len(plan.axes) > 1
        )

    solver_nodes = []
    for handler in handler_nodes:
        for plan in derivation.controllers_by_handler.get(handler, ()):
            if plan.solver not in solver_nodes and SLV.SolverWithInputAndOutput in get_node_types(
                g, plan.solver
            ):
                solver_nodes.append(plan.solver)
    solver_nodes.extend(
        sorted(
            set(g.subjects(RDF.type, SLV.SolverWithInputAndOutput)) - set(solver_nodes),
            key=str,
        )
    )
    for s in solver_nodes:
        solver = p.solver_with_input_and_output(s)
        solver.motion_drivers = _derived_motion_drivers(g, p, derivation, s)
        robot_node = g.value(s, AGN["of-agent"])
        (
            solver.urdf,
            solver.chain_root,
            solver.chain_end,
            solver.chain_tip,
            solver.robot_model,
            solver.tool_body,
            solver.tcp_site,
            solver.sensors,
            solver.devices,
            solver.runtime_prefix,
            solver.owned_trees,
            solver.kdl_chain,
            solver.kdl_tree,
            solver.kdl_joints,
            solver.config_key,
        ) = setups_by_node.get(robot_node, default_setup)
        solver.output = _dedupe_by_id(
            [
                *solver.output,
                *_world_solver_outputs(
                    g,
                    p,
                    solver.chain_root,
                    solver.runtime_prefix,
                    solver.owned_trees,
                    scene_objects,
                ),
            ]
        )
        _mark_acceleration_constraint_frames(solver)
        slv_chain.append(solver)
        start = g[
            s
            : SLV["motion-drivers"]
            / ((SLV["cartesian-force"]) | (SLV["joint-force"]))
        ]
        sched3.extend(p.schedule(start, ops_generic + ops_slv))

    for s in sorted(g.subjects(RDF.type, SLV["ForceDistributionSolver"]), key=str):
        slv_platform_frc.append(p.force_distribution_solver(s))
        sched4.extend(p.schedule([s], ops_generic + ops_slv))

    # velocity-distribution and force-composition are authorable in the DSL (they complete
    # the quantity x operation 2x2 the hddc2b runtime implements), but no motion.stg/hddc2b.stg
    # wiring exists for them yet: velocity-distribution needs a per-solver actuation-mode
    # switch (kelo_cmd.ctrl_mode is hardcoded to ROBIF2B_CTRL_MODE_FORCE) and force-composition
    # needs a measured wheel/drive torque signal that the kelo measurement struct does not
    # expose. Fail loudly instead of silently dropping them from codegen.
    for unsupported_type, label in (
        (SLV_EXT.VelocityDistributionSolver, "velocity-distribution"),
        (SLV_EXT.ForceCompositionSolver, "force-composition"),
    ):
        unsupported = sorted(g.subjects(RDF.type, unsupported_type), key=str)
        if unsupported:
            raise ValueError(
                f"Mobile-platform solver '{unsupported[0]}' uses algorithm '{label}', which "
                "the motion-spec code generator does not implement yet (no motion.stg/"
                "hddc2b.stg wiring exists for it). Use velocity-composition or "
                "force-distribution instead."
            )

    return slv_platform_vel, sched1, hdl, sched2, slv_chain, sched3, slv_platform_frc, sched4


def _assign_monitor_event_indexes(handlers) -> None:
    """Assign each monitor a stable per-handler event index."""
    event_idx = 0
    for handler in handlers:
        for monitor in handler.monitors:
            if monitor.monitor_type == "EdgeTriggeredMonitor":
                monitor.event_idx = event_idx
                event_idx += 1


def _snapshot_maps(g, p: Parser) -> tuple[dict, dict, dict]:
    """One walk over the snapshots for the three maps codegen asks about.

    ``source``: each snapshot output to its source quantity.

    ``owner``: each snapshot's output to the motion that declares it. Every motion captures each
    snapshot it *references*, and they all write the same shared slot, so a motion re-capturing
    another's snapshot silently retargets it. The owner is the motion segment of the quantity's
    URI: <app>/<motion>/spec/<name>.

    ``trigger``: each event-triggered snapshot to its trigger event's local name, keyed by
    (declaring motion, output id). Every motion referencing a snapshot captures it, but only the
    motion that declares it re-samples on the trigger, so the key carries the owner.
    """
    source_map: dict[str, str] = {}
    owner_map: dict[str, str] = {}
    trigger_map: dict[tuple[str, str], str] = {}
    for snap_node in g.subjects(RDF.type, ALGO_EXT.Snapshot):
        output_node = g.value(snap_node, ALGO_EXT.out)
        if output_node is None:
            continue
        output_id = p.id(output_node)
        source_node = g.value(snap_node, ALGO_EXT["in"])
        if source_node is not None:
            source_map[output_id] = p.id(source_node)
        scope = p._context_scope(output_node)
        if scope is None:
            continue
        owner = get_valid_var_name(scope[0])
        owner_map[output_id] = owner
        trigger_node = g.value(snap_node, ALGO_EXT["trigger"])
        if trigger_node is not None:
            trigger_map[(owner, output_id)] = get_valid_var_name(_leaf(trigger_node)).upper()
    return source_map, owner_map, trigger_map


def _closure_owner_map(g, p: Parser, closures) -> dict[str, str]:
    """Map each closure to the motion whose context declares the quantities it reads.

    The backward schedule walk can reach a closure belonging to another motion, which then
    runs (and mutates its outputs) whenever that unrelated motion is active. Ownership is the
    motion segment of the quantities it references: <app>/<motion>/spec/<name>. A closure
    reading only shared context has no owner and stays available to every motion.
    """
    owner_map: dict[str, str] = {}
    for node in set(g.subjects()):
        if not isinstance(node, URIRef):
            continue
        closure_id = p.id(node)
        if closure_id not in closures:
            continue
        owners = set()
        for obj in g.objects(node, None):
            if not isinstance(obj, URIRef):
                continue
            scope = p._context_scope(obj)
            if scope is not None and scope[1] == "spec":
                owners.add(get_valid_var_name(scope[0]))
        if len(owners) == 1:
            owner_map[closure_id] = owners.pop()
    return owner_map


def _pose_frames(g, pose) -> tuple[URIRef, URIRef]:
    """Return a pose quantity's authored `(of, with-respect-to)` frames."""
    relation = (
        PoseModel(pose, g)
        if URI_GEOM_TYPE_POSE in get_node_types(g, pose)
        else PoseCoordModel(pose, g).relation
    )
    return relation.of_id, relation.wrt_id


def _emit_derived_pose(g, node: URIRef, of_frame: URIRef, wrt_frame: URIRef) -> None:
    """Materialize one runtime-derived pose in comp-rob2b relation/coordinate form."""
    origins = []
    for frame in (of_frame, wrt_frame):
        origin = g.value(frame, URI_GEOM_PRED_ORIGIN)
        if not isinstance(origin, URIRef):
            origin = URIRef(f"{frame}-origin")
            g.add((frame, URI_GEOM_PRED_ORIGIN, origin))
        g.add((frame, RDF.type, URI_GEOM_TYPE_FRAME))
        g.add((origin, RDF.type, URI_GEOM_TYPE_POINT))
        origins.append(origin)

    pose_relation = URIRef(f"{node}-pose-rel")
    position_relation = URIRef(f"{node}-position-rel")
    orientation_relation = URIRef(f"{node}-orientation-rel")
    for relation, relation_type, of_entity, wrt_entity in (
        (pose_relation, URI_GEOM_TYPE_POSE, of_frame, wrt_frame),
        (position_relation, URI_GEOM_TYPE_POSITION, origins[0], origins[1]),
        (orientation_relation, URI_GEOM_TYPE_ORIENT, of_frame, wrt_frame),
    ):
        g.add((relation, RDF.type, relation_type))
        g.add((relation, RDF.type, QUDT_SCHEMA.Quantity))
        g.add((relation, URI_GEOM_PRED_OF, of_entity))
        g.add((relation, URI_GEOM_PRED_WRT, wrt_entity))
    g.add((position_relation, QUDT_SCHEMA.hasQuantityKind, URI_QUDT_QK_LENGTH))
    for reference_type in (URI_GEOM_TYPE_POSITION_REF, URI_GEOM_TYPE_ORIENT_REF):
        g.add((pose_relation, RDF.type, reference_type))
    g.add((pose_relation, URI_GEOM_PRED_OF_POSITION, position_relation))
    g.add((pose_relation, URI_GEOM_PRED_OF_ORIENT, orientation_relation))

    g.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    for coordinate_type in (
        URI_GEOM_TYPE_POSE_COORD,
        URI_GEOM_TYPE_POSE_REF,
        URI_GEOM_TYPE_POSITION_COORD,
        URI_GEOM_TYPE_POSITION_REF,
        URI_GEOM_TYPE_ORIENT_COORD,
        URI_GEOM_TYPE_ORIENT_REF,
        URI_GEOM_TYPE_VECTOR_XYZ,
    ):
        g.add((node, RDF.type, coordinate_type))
    g.add((node, URI_GEOM_PRED_OF_POSE, pose_relation))
    g.add((node, URI_GEOM_PRED_OF_POSITION, position_relation))
    g.add((node, URI_GEOM_PRED_OF_ORIENT, orientation_relation))
    g.add((node, URI_GEOM_PRED_SEEN_BY, wrt_frame))
    g.add((node, QUDT_SCHEMA.unit, URI_QUDT_UNIT_M))
    g.add((node, QUDT_SCHEMA.unit, URI_QUDT_UNIT_RAD))


def _materialize_linear_distance_operations(g) -> None:
    """Expand authored linear-distance relations into codegen operations.

    The DSL graph states only the two pose endpoints. This operational expansion belongs
    here because its inverse/composition path is derivable from the complete RDF graph.
    """

    def derived(node, suffix):
        return URIRef(f"{node}.derived-{suffix}")

    def emit_pose(node, of_frame, wrt_frame):
        _emit_derived_pose(g, node, of_frame, wrt_frame)

    edges = collections.defaultdict(list)
    for pose in g.subjects(RDF.type, GEOM_REL.Pose):
        try:
            of_frame, wrt_frame = _pose_frames(g, pose)
        except ValueError:
            continue
        edges[wrt_frame].append((of_frame, pose, False))
        edges[of_frame].append((wrt_frame, pose, True))

    for distance in list(g.subjects(RDF.type, GEOM_REL.LinearDistance)):
        if next(g.subjects(CSTR.quantity, distance), None) is None:
            continue
        endpoints = list(dict.fromkeys(g.objects(distance, GEOM_REL["between-entities"])))
        if len(endpoints) != 2:
            raise ValueError(f"Linear distance {distance} needs exactly two pose endpoints.")
        start, end = endpoints
        start_of, start_wrt = _pose_frames(g, start)
        end_of, end_wrt = _pose_frames(g, end)

        path = ()
        if start_wrt != end_wrt:
            queue = collections.deque([(start_wrt, ())])
            seen = {start_wrt}
            while queue:
                frame, current_path = queue.popleft()
                for next_frame, pose, inverted in edges[frame]:
                    if next_frame in seen:
                        continue
                    next_path = (*current_path, (pose, inverted))
                    if next_frame == end_wrt:
                        path = next_path
                        queue.clear()
                        break
                    seen.add(next_frame)
                    queue.append((next_frame, next_path))
            if not path:
                raise ValueError(
                    f"Linear distance {distance} has no pose path from {start_wrt} to {end_wrt}."
                )

        current = None
        current_wrt = start_wrt
        for index, (pose, inverted) in enumerate(path):
            pose_of, pose_wrt = _pose_frames(g, pose)
            step = pose
            step_of, step_wrt = pose_of, pose_wrt
            if inverted:
                step = derived(distance, f"path-{index}-inverse")
                emit_pose(step, pose_wrt, pose_of)
                operation = derived(distance, f"path-{index}-invert")
                g.add((operation, RDF.type, GEOM_OP.InvertPose))
                g.add((operation, GEOM_OP.pose, pose))
                g.add((operation, GEOM_OP.out, step))
                step_of, step_wrt = pose_wrt, pose_of
            if current is None:
                current = step
                current_wrt = step_wrt
                continue
            composite = derived(distance, f"path-{index}-pose")
            emit_pose(composite, step_of, current_wrt)
            operation = derived(distance, f"path-{index}-compose")
            g.add((operation, RDF.type, GEOM_OP.ComposePose))
            g.add((operation, GEOM_OP.in1, current))
            g.add((operation, GEOM_OP.in2, step))
            g.add((operation, GEOM_OP.composite, composite))
            current = composite

        end_in_start_reference = end
        if current is not None:
            end_in_start_reference = derived(distance, "end-in-start-reference")
            emit_pose(end_in_start_reference, end_of, start_wrt)
            operation = derived(distance, "compose-reference-path")
            g.add((operation, RDF.type, GEOM_OP.ComposePose))
            g.add((operation, GEOM_OP.in1, current))
            g.add((operation, GEOM_OP.in2, end))
            g.add((operation, GEOM_OP.composite, end_in_start_reference))

        inverse_start = derived(distance, "inverse-start")
        emit_pose(inverse_start, start_wrt, start_of)
        invert_start = derived(distance, "invert-start")
        g.add((invert_start, RDF.type, GEOM_OP.InvertPose))
        g.add((invert_start, GEOM_OP.pose, start))
        g.add((invert_start, GEOM_OP.out, inverse_start))

        relative_pose = derived(distance, "relative-pose")
        emit_pose(relative_pose, end_of, start_of)
        compose_relative = derived(distance, "compose-relative-pose")
        g.add((compose_relative, RDF.type, GEOM_OP.ComposePose))
        g.add((compose_relative, GEOM_OP.in1, inverse_start))
        g.add((compose_relative, GEOM_OP.in2, end_in_start_reference))
        g.add((compose_relative, GEOM_OP.composite, relative_pose))

        operation = derived(distance, "magnitude")
        g.add((operation, RDF.type, GEOM_OP.PoseToLinearDistance))
        g.add((operation, GEOM_OP.pose, relative_pose))
        g.add((operation, GEOM_OP.distance, distance))


def _materialize_pose_reference_transforms(g) -> None:
    """Re-express a full-pose equality reference into the constrained pose's frame.

    A full-pose EqualityConstraint states its reference in whatever frame it was
    authored; when that differs from the constrained quantity's `with-respect-to`
    frame the comparison first needs the reference re-expressed. The transform path
    is derivable from the complete RDF graph, so the composition belongs here.
    """
    def derived(node, suffix):
        return URIRef(f"{node}.derived-{suffix}")

    def emit_pose(node, of_frame, wrt_frame):
        _emit_derived_pose(g, node, of_frame, wrt_frame)

    edges = collections.defaultdict(list)
    for pose in g.subjects(RDF.type, GEOM_REL.Pose):
        try:
            of_frame, wrt_frame = _pose_frames(g, pose)
        except ValueError:
            continue
        edges[wrt_frame].append((of_frame, pose, False))
        edges[of_frame].append((wrt_frame, pose, True))

    for constraint in list(g.subjects(RDF.type, CSTR.EqualityConstraint)):
        quantity = g.value(constraint, CSTR.quantity)
        reference = g.value(constraint, CSTR["reference-value"])
        if quantity is None or reference is None:
            continue
        if GEOM_REL.Pose not in get_node_types(g, quantity):
            continue
        if GEOM_REL.Pose not in get_node_types(g, reference):
            continue
        try:
            target_of, target_wrt = _pose_frames(g, quantity)
            source_of, source_wrt = _pose_frames(g, reference)
        except ValueError:
            # A coordinate-authored goal pose (position/orientation values, no explicit
            # of/with-respect-to frames) is already stated in the constrained pose's
            # frame; there is no cross-frame reference to re-express.
            continue
        if source_wrt == target_wrt:
            continue
        if source_of != target_of:
            raise ValueError(
                f"Equality constraint {constraint} compares a pose of {target_of} "
                f"to a reference of {source_of}."
            )

        queue = collections.deque([(target_wrt, ())])
        seen = {target_wrt}
        path = ()
        while queue:
            frame, current_path = queue.popleft()
            for next_frame, pose, inverted in edges[frame]:
                if next_frame in seen:
                    continue
                next_path = (*current_path, (pose, inverted))
                if next_frame == source_wrt:
                    path = next_path
                    queue.clear()
                    break
                seen.add(next_frame)
                queue.append((next_frame, next_path))
        if not path:
            raise ValueError(
                f"Equality constraint {constraint} has no pose path from "
                f"{target_wrt} to {source_wrt}."
            )

        current = None
        current_wrt = target_wrt
        for index, (pose, inverted) in enumerate(path):
            pose_of, pose_wrt = _pose_frames(g, pose)
            step = pose
            step_of, step_wrt = pose_of, pose_wrt
            if inverted:
                step = derived(constraint, f"path-{index}-inverse")
                emit_pose(step, pose_wrt, pose_of)
                operation = derived(constraint, f"path-{index}-invert")
                g.add((operation, RDF.type, GEOM_OP.InvertPose))
                g.add((operation, GEOM_OP.pose, pose))
                g.add((operation, GEOM_OP.out, step))
                step_of, step_wrt = pose_wrt, pose_of
            if current is None:
                current = step
                current_wrt = step_wrt
                continue
            composite = derived(constraint, f"path-{index}-pose")
            emit_pose(composite, step_of, current_wrt)
            operation = derived(constraint, f"path-{index}-compose")
            g.add((operation, RDF.type, GEOM_OP.ComposePose))
            g.add((operation, GEOM_OP.in1, current))
            g.add((operation, GEOM_OP.in2, step))
            g.add((operation, GEOM_OP.composite, composite))
            current = composite

        reference_in_target = derived(constraint, "reference-in-target")
        emit_pose(reference_in_target, source_of, target_wrt)
        operation = derived(constraint, "compose-reference")
        g.add((operation, RDF.type, GEOM_OP.ComposePose))
        g.add((operation, GEOM_OP.in1, current))
        g.add((operation, GEOM_OP.in2, reference))
        g.add((operation, GEOM_OP.composite, reference_in_target))

        g.remove((constraint, CSTR["reference-value"], reference))
        g.add((constraint, CSTR["reference-value"], reference_in_target))


def _data_reference_map(data_structures, closures: dict) -> dict[str, str]:
    """Map each data id to the ids it references through closures."""
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
    return data_reference_map


def _closure_maps(closures: dict) -> tuple[dict[str, str], dict[str, set[str]]]:
    """Build (closure_output_map, closure_input_map): data id to the closures producing/consuming
    it.
    """
    closure_output_map: dict[str, str] = {}
    closure_input_map: dict[str, set[str]] = {}
    for cid, c in closures.items():
        if not isinstance(c, dict):
            continue
        outputs = closure_output_ids(c)
        inputs = {
            value
            for key, value in c.items()
            if key not in {"id", "type"} and isinstance(value, str) and value not in outputs
        }
        for out_val in outputs:
            closure_output_map[out_val] = cid
            closure_input_map[out_val] = inputs
    return closure_output_map, closure_input_map


# Which codegen backend serves an authored simulation platform. The platform is the model's; the
# backend is an implementation detail of running it, so the mapping lives here and nowhere else.
_SIMULATION_BACKENDS = {"mujoco": "mj_kdl"}


def _platform_from_graph(g) -> dict:
    """The execution platform the model declares, as one record every consumer reads.

    Returns the authored node's IRI, its name, whether it is simulated, and the backend that
    serves it. Downstream code must take platform identity from here rather than re-deriving it
    from the backend token or by matching substrings of a file path -- a model that declares
    `platform: simulation { name: "Gazebo" }` is not MuJoCo, and nothing should have to guess.
    """
    simulation = next(g.subjects(RDF.type, EXEC.Simulation), None)
    if simulation is None:
        real = next(g.subjects(RDF.type, EXEC.RealWorld), None)
        _reject_scene_objects_on_hardware(g, real)
        _reject_undriven_devices(g, real)
        _reject_unbound_sensors_on_hardware(g, real)
        return {
            "uri": str(real) if real is not None else None,
            "name": None,
            "simulated": False,
            "backend": "robif2b",
            "config": str(_config_path(g, real)) if real is not None else None,
        }
    name = str(g.value(simulation, SDO.name) or "")
    backend = _SIMULATION_BACKENDS.get(name.casefold())
    if backend is None:
        raise ValueError(f"Unsupported simulation platform '{name}'.")
    return {
        "uri": str(simulation),
        "name": name,
        "simulated": True,
        "backend": backend,
        "config": _config_path(g, simulation),
    }


def _config_path(g, context) -> str | None:
    """The deployment config's path, from exec:has-resource -> exec:path.

    The DSL resolves it against the model that declares it, so it is a path both codegen and
    the generated program can open -- neither knows the .robmot's directory.
    """
    config = g.value(context, EXEC["has-resource"])
    return str(g.value(config, EXEC.path)) if config is not None else None


def _reject_undriven_devices(g, context) -> None:
    """Reject a bound device the backend would silently ignore.

    The grammar decides what a model may name; this decides what the backend can actually
    drive. A device the templates do not cover must fail here rather than generate a
    controller that quietly leaves it dead.
    """
    if context is None:
        return
    bound = sorted(
        {
            str(g.value(device, SDO.model) or "")
            for device in g.subjects(EXEC["realizes"], None)
        }
    )
    undriven = [name for name in bound if name not in DRIVEN_DEVICES]
    if undriven:
        raise ConstraintViolation(
            "platform",
            f"no backend support for device(s): {', '.join(undriven)}. "
            f"Driven: {', '.join(sorted(DRIVEN_DEVICES))}. Remove the binding, or add "
            "the driver templates before binding it.",
        )


def _reject_scene_objects_on_hardware(g, context) -> None:
    """Reject scene objects on hardware: nothing measures their pose without perception."""
    if context is None:
        return
    objects = sorted(_leaf(node) for node in g.subjects(RDF.type, ENV.ModelledObject))
    if objects:
        raise ConstraintViolation(
            "platform",
            f"real-world execution cannot use scene objects ({', '.join(objects)}): their poses "
            "come from a simulator, and nothing measures them on hardware. Remove them, or model "
            "the location as an authored frame.",
        )


def _reject_unbound_sensors_on_hardware(g, context) -> None:
    """Reject a sensor a model reads from but binds no device to.

    In simulation the simulator answers for every sensor in the scene. On hardware a reading
    comes from a device or from nowhere, so a wrench sourced from an unbound sensor would
    generate a controller reading uninitialised memory every tick.
    """
    if context is None:
        return
    unbound = sorted(
        _leaf(sensor)
        for sensor in set(g.objects(None, SOSA.madeBySensor))
        if _device_of(g, sensor) is None
    )
    if unbound:
        raise ConstraintViolation(
            "platform",
            f"real-world execution reads sensor(s) with no device bound: {', '.join(unbound)}. "
            "Bind one in the platform block, or stop reading the sensor.",
        )


def _apply_monitor_debounce(handlers, control_period_ns: int) -> None:
    """Convert each monitor's debounce duration to a step count from the control period."""
    for handler in handlers:
        for monitor in handler.monitors:
            if getattr(monitor, "debounce_duration_s", None) is not None:
                monitor.debounce_steps = round(
                    monitor.debounce_duration_s / (control_period_ns * 1e-9)
                )


def _shared_runtime_members(slv_chain, iris, control_period_ns: int, platform_uri) -> list[dict]:
    """Extra shared-data members for force/torque sensor state and the measured control period."""
    # The dt every integrator steps with, measured by the loop from the backend clock, nominal
    # until the first measurement exists. A contracted shared value rather than a hardcoded struct
    # member, so it carries a producer and lands in the frame log like any other.
    if not platform_uri:
        raise RuntimeError("measured dt: the execution platform has no IRI to derive a clock from")
    # Which clock it is -- sim seconds or monotonic seconds -- is the platform's to say, so the
    # port derives from the exec context and the measurement derives from the port.
    clock_iri = iris.register("clock", platform_uri, "clock", DerivedIriRegistry.DERIVATION)
    iris.register("clock_time_s", clock_iri, "clock_time_s", DerivedIriRegistry.DERIVATION)
    iris.register("dt_measured_s", clock_iri, "dt_measured_s", DerivedIriRegistry.DERIVATION)
    # The clock reading itself is contracted too: the loop writes it from the same port every
    # tick, and elapsed conditions read it like any other shared value.
    members = [
        {"id": "clock_time_s", "type": "Quantity"},
        {"id": "dt_measured_s", "type": "Quantity", "value": control_period_ns * 1e-9},
    ]
    seen_ft_ids = set()
    for s in slv_chain:
        for out in s.output:
            if getattr(out, "type", None) == "Wrench" and getattr(out, "sensor_name", ""):
                if out.id in seen_ft_ids:
                    continue
                seen_ft_ids.add(out.id)
                # The tare state is computed from the reading, so it derives from that sensor
                # output's own node.
                sensor_iri = iris.iri_of(out.id)
                if sensor_iri is None:
                    raise RuntimeError(
                        f"ft tare state: sensor output '{out.id}' has no IRI to derive from"
                    )
                for suffix, member_type in (("ft_bias", "Wrench"), ("ft_settle", "IntCounter")):
                    member_id = f"{out.id}_{suffix}"
                    members.append({"id": member_id, "type": member_type})
                    iris.register(
                        member_id, sensor_iri, suffix, DerivedIriRegistry.DERIVATION
                    )

    return members


# ---------------------------------------------------------------------------
# Codegen-facing helpers. Each is invoked while constructing the piece it belongs to
# (scene, solvers, closures, motions, introspection) so generate_ir builds a complete IR
# in one forward pass — the assembled ir dict is final and is never re-processed.
# ---------------------------------------------------------------------------


SUPPORTED_ROBOT_MODELS = {"KinovaGen3"}

# The devices this backend has driver templates for. A name the grammar accepts but that is
# missing here is rejected at generation; add it once its templates land.
DRIVEN_DEVICES = {"KinovaGen3", "KinovaGen3-2F85", "Robotiq2F85", "RobotiqFT300s"}

# The sensor kinds the IR models, as the graph types them and as templates dispatch on them.
SENSOR_KINDS = {SENSORS.ForceTorqueSensor: "ForceTorque"}


def _validate_solvers(serial_chain_solvers, backend: str) -> None:
    """Reject unsupported robot models, and scene-object pose sync on the robif2b backend."""
    unsupported = {
        _field(s, "robot_model")
        for s in serial_chain_solvers
        if _field(s, "robot_model") and _field(s, "robot_model") not in SUPPORTED_ROBOT_MODELS
    }
    if unsupported:
        raise RuntimeError(
            f"Unsupported robot model(s): {', '.join(sorted(unsupported))}. "
            f"Supported: {', '.join(sorted(SUPPORTED_ROBOT_MODELS))}"
        )

    if backend != "robif2b":
        return

    for solver in serial_chain_solvers:
        for out in _field(solver, "output", []):
            if _field(out, "type") != "Pose":
                continue
            entity = _field(out, "of") or {}
            if _field(entity, "is_scene_object"):
                obj_id = _field(entity, "id") or _field(entity, "body") or _field(out, "id")
                raise RuntimeError(
                    "robif2b backend cannot sync scene-object pose output "
                    f"'{_field(out, 'id')}' for '{obj_id}'; world/scene object pose sync "
                    "is only implemented for mj_kdl."
                )


def _runtime_signature(solver, backend: str) -> tuple:
    """Identity tuple of a solver's runtime (backend, chain, tool) for deduplicating runtimes."""
    return (
        backend,
        _field(solver, "robot_model", ""),
        _field(solver, "urdf", ""),
        _field(solver, "chain_root", ""),
        _field(solver, "chain_tip") or _field(solver, "chain_end", ""),
        _field(solver, "tool_body", ""),
        _field(solver, "tcp_site", ""),
    )


def _annotate_rne_gravity(serial_chain_solvers, motions) -> None:
    """Derive the gravity vector an RNE solver is built with.

    The authored solver value is the Vereshchagin root acceleration, which ACHD takes as-is.
    KDL's inverse-dynamics solver wants gravity with the opposite sign, so the negation belongs
    wherever one is constructed -- which is every backend that runs RNE, not just the simulated
    one. A solver that never builds an RNE reads the field and finds nothing.
    """
    for solver in list(serial_chain_solvers) + [
        s for motion in motions for s in _field(motion, "serial_chain_solvers", [])
    ]:
        root_acc = _field(solver, "root_acc")
        if root_acc:
            _set_field(solver, "gravity", [-component or 0.0 for component in root_acc])


def _annotate_runtime_robots(serial_chain_solvers, motions, backend: str) -> None:
    """Assign runtime_id/runtime_owner across solvers sharing a runtime and normalize empty tool
    fields.
    """
    runtime_by_signature: dict[tuple, str] = {}
    owner_by_runtime: dict[str, str] = {}
    solvers_by_id = {_field(solver, "id"): solver for solver in serial_chain_solvers}

    for solver in serial_chain_solvers:
        solver_id = _field(solver, "id", "")
        signature = _runtime_signature(solver, backend)
        runtime_id = runtime_by_signature.setdefault(signature, solver_id)
        owner_by_runtime.setdefault(runtime_id, solver_id)
        _set_field(solver, "runtime_id", runtime_id)
        _set_field(solver, "runtime_owner", solver_id == owner_by_runtime[runtime_id])
        # ST4's <if(x)> treats "" as truthy. Convert empty strings to None so
        # the template's <if(solver.tool_body)> branch is correctly skipped
        # for bare robots (no gripper / tool attached).
        if not _field(solver, "tool_body"):
            _set_field(solver, "tool_body", None)
        if not _field(solver, "tcp_site"):
            _set_field(solver, "tcp_site", None)

    for motion in motions:
        for solver in _field(motion, "serial_chain_solvers", []):
            canonical = solvers_by_id.get(_field(solver, "id"))
            if canonical is None:
                continue
            _set_field(
                solver, "runtime_id", _field(canonical, "runtime_id") or _field(solver, "id", "")
            )
            _set_field(solver, "runtime_owner", _field(canonical, "runtime_owner", True))
        for command in _field(motion, "forwarded_commands", []):
            canonical = solvers_by_id.get(_field(command, "robot_id"))
            if canonical is not None:
                _set_field(command, "robot_id", _field(canonical, "runtime_id"))


def _add_group_type_flags(groups: list) -> None:
    """Set is_pose/is_twist/is_wrench on pose-axis error groups from their superobject type."""
    for g in groups:
        so_type = _field(g, "superobject_type", "Pose")
        _set_field(g, "is_pose", so_type == "Pose")
        _set_field(g, "is_twist", so_type in ("VelocityTwist", "AccelerationTwist"))
        _set_field(g, "is_wrench", so_type == "Wrench")


# ---------------------------------------------------------------------------
# Codegen-facing helpers
# ---------------------------------------------------------------------------
def _field(obj, key, default=None):
    """Read a field from either a dict or a dataclass instance, so derivations can run
    on the native IR (dataclasses) without a dict round-trip. str-Enum values are
    normalized to their string value so native access matches the serialized dict."""
    if obj is None:
        return default
    val = obj.get(key, default) if isinstance(obj, dict) else getattr(obj, key, default)
    return val.value if isinstance(val, Enum) else val


def _set_field(obj, key, value) -> None:
    """Set a field on either a dict or a dataclass instance (the field must exist on the
    dataclass for it to serialize)."""
    if isinstance(obj, dict):
        obj[key] = value
    else:
        setattr(obj, key, value)


def _as_dict(obj) -> dict:
    """Plain-dict view of a dict or dataclass (for building rows from all fields)."""
    return obj if isinstance(obj, dict) else asdict(obj)


def _motion_done_terms(motion) -> list:
    """Structured UNTIL-member terms that end a motion (edge monitors → event flag, level
    monitors → their boolean flag). Joined by until_any and rendered by bool-condition."""
    mid = _field(motion, "id")
    terms = []
    for monitor in _field(motion, "until_monitors", []):
        if _field(monitor, "is_edge_triggered"):
            terms.append({"kind": "event", "motion_id": mid, "monitor_id": _field(monitor, "id")})
        else:
            terms.append({"kind": "flag", "motion_id": mid, "flag": _field(monitor, "flag")})
    return terms


def expand_vector_fields(
    item,
    field: str,
    component_names: tuple[str, ...] = ("x", "y", "z"),
    default: list[float] | None = None,
) -> None:
    """Expand a vector field into <field>_<component> parts. An omitted value falls back to
    ``default`` (zero vector for position, identity quaternion for orientation); a value whose
    arity does not match component_names raises rather than being padded or truncated."""
    values = _field(item, field)
    if values is None:
        # Env placement shorthand: omitted position/orientation means zero/identity.
        values = list(default) if default is not None else [0.0] * len(component_names)
    if not isinstance(values, list) or len(values) != len(component_names):
        item_id = _field(item, "id", "<unknown>")
        raise ValueError(
            f"Scene item '{item_id}' has invalid '{field}'; expected {len(component_names)} values."
        )
    for name, value in zip(component_names, values):
        _set_field(item, f"{field}_{name}", value)


def require_field(obj_id: str, field: str, value) -> None:
    """Raise if a required procedural scene-object field is missing."""
    if value is None:
        raise ValueError(
            f"Procedural scene object '{obj_id}' is missing required field "
            f"'{field}'. Add it to the .robmot model — silent defaults are no "
            f"longer applied."
        )


def _pose_component(component_id: str, data_by_id: dict) -> dict:
    """Structured pose component: either a literal ``value`` or a ``ref`` id that the
    backend template renders via access-expr. Backend-agnostic — no C++/KDL here."""
    component = data_by_id.get(component_id)
    reference_value = _field(component, "reference_value")
    if reference_value:
        return {"value": None, "ref": reference_value}
    value = _field(component, "value")
    if value is not None:
        return {"value": str(value), "ref": None}
    return {"value": None, "ref": component_id}


def _signal_id(value):
    """Id string of a signal value (a str or an object with an id)."""
    if isinstance(value, str):
        return value
    return _field(value, "id")


def _annotate_controller_signals(controllers, closures: dict) -> None:
    """Fold the measured/setpoint signal ids onto each controller from its error-evaluator
    closure. Emits abstract ids only; the C++ access expression is rendered backend-side by
    access-expr (shared_data.stg). Runs during construction of the controllers' piece."""
    error_sources = {
        closure.get("error"): closure
        for closure in closures.values()
        if isinstance(closure, dict)
        and closure.get("type") == "ErrorEvaluator"
        and closure.get("error")
    }
    for controller in controllers:
        error_id = _signal_id(_field(controller, "error_signal"))
        source = error_sources.get(error_id) or {}
        measured_id = source.get("quantity")
        setpoint_id = _signal_id(_field(controller, "reference_signal")) or source.get(
            "reference_value"
        )
        if measured_id:
            _set_field(controller, "measured_signal", measured_id)
        if setpoint_id:
            _set_field(controller, "setpoint_signal", setpoint_id)


def add_controller_internal_state_logging(
    closures: dict, shared_data: list, introspection: dict, motions, iris
) -> None:
    """Log stateful controllers' internal state (error integral, previous error, first-sample flag)
    as shared_data items and introspection quantities.
    """
    quantities = introspection.setdefault("quantities", [])
    shared_ids = {_field(item, "id") for item in shared_data if _field(item, "id")}
    quantity_ids = {_field(item, "id") for item in quantities if _field(item, "id")}

    def add_shared(item_id: str, item_type: str, controller_id: str, state_name: str) -> None:
        """Append a controller-internal-state shared_data item (once per id)."""
        if item_id not in shared_ids:
            shared_data.append(
                {
                    "id": item_id,
                    "type": item_type,
                    "controller": controller_id,
                    "role": "controller_internal_state",
                    "state": state_name,
                }
            )
            shared_ids.add(item_id)

    def add_quantity(item_id: str, controller_id: str, state_name: str) -> None:
        """Append a controller-internal-state introspection quantity (once per id)."""
        if item_id not in quantity_ids:
            quantities.append(
                {
                    "id": item_id,
                    "type": "Quantity",
                    "controller": controller_id,
                    "role": "controller_internal_state",
                    "state": state_name,
                }
            )
            quantity_ids.add(item_id)

    stateful_types = {"ProportionalIntegralDerivative", "ImpedanceController"}
    stateful_ids = {
        _field(controller, "id")
        for source in (motions, [{"controllers": introspection.get("controllers", [])}])
        for entry in source
        for controller in (_field(entry, "controllers", []) or [])
        if _field(controller, "type") in stateful_types and _field(controller, "id")
    }

    for closure in closures.values():
        if not isinstance(closure, dict) or closure.get("type") != "Controller":
            continue
        controller_id = closure.get("id")
        if controller_id not in stateful_ids:
            continue
        samples = [
            ("error_integral", "Quantity", "error_integral"),
            ("previous_error", "Quantity", "previous_error"),
            ("first_sample", "Bool", "is_first_sample"),
        ]
        closure_samples = []
        # The parent is resolved through the registry, not the graph: a per-axis controller is
        # itself derived and has no graph node of its own.
        parent_iri = iris.iri_of(controller_id)
        if parent_iri is None:
            raise RuntimeError(
                f"controller internal state: '{controller_id}' has no IRI to derive from"
            )
        for state_name, item_type, getter in samples:
            item_id = f"{controller_id}_{state_name}"
            add_shared(item_id, item_type, controller_id, state_name)
            if item_type == "Quantity":
                add_quantity(item_id, controller_id, state_name)
            iris.register(
                item_id, parent_iri, state_name, DerivedIriRegistry.DERIVATION
            )
            closure_samples.append({"id": item_id, "getter": getter})
        closure["internal_state_samples"] = closure_samples


# The authored numbers each controller kind tunes with: gains-struct field name -> the field the
# parsed controller carries it on, and whether the model must author it. Order is the struct's
# field order in runtime.stg. The optional ones stood at 0.0 when unauthored before they moved
# here; the rest were rendered unconditionally, so an absent one must still fail.
_CONTROL_GAINS = {
    "ProportionalIntegralDerivative": (
        ("kp", "proportional_gain", True),
        ("ki", "integral_gain", True),
        ("kd", "derivative_gain", True),
        ("decay_rate", "decay_rate", False),
    ),
    "ImpedanceController": (
        ("stiffness", "stiffness", True),
        ("damping", "damping", True),
        ("integral_gain", "integral_gain", False),
    ),
}

# Admittance parameters, carried on the closure as authored literals.
_ADMITTANCE_PARAMETERS = ("mass", "damping", "stiffness", "maximum_velocity")


def add_control_parameters(closures: dict, shared_data: list, introspection: dict, motions, iris):
    """Publish every authored control parameter as a shared value the step call reads.

    A gain baked into a constructor can neither be reported nor ever vary; as a shared value it
    carries a producer (`authored`, so `init`) and lands in the run's header record like any
    other authored literal.
    """
    quantities = introspection.setdefault("quantities", [])
    shared_ids = {_field(item, "id") for item in shared_data if _field(item, "id")}
    quantity_ids = {_field(item, "id") for item in quantities if _field(item, "id")}

    def publish(owner_id: str, name: str, value, required: bool = True) -> str:
        """Append the shared value and introspection quantity for one parameter; return its id."""
        parent_iri = iris.iri_of(owner_id)
        if parent_iri is None:
            raise RuntimeError(f"control parameter: '{owner_id}' has no IRI to derive from")
        if value is None and required:
            raise RuntimeError(f"control parameter: '{owner_id}' authors no '{name}'")
        item_id = f"{owner_id}_{name}"
        row = {
            "id": item_id,
            "type": "Quantity",
            "value": float(value) if value is not None else 0.0,
            "owner": owner_id,
            "role": "control_parameter",
            "parameter": name,
        }
        if item_id not in shared_ids:
            shared_data.append(dict(row))
            shared_ids.add(item_id)
        if item_id not in quantity_ids:
            quantities.append(dict(row))
            quantity_ids.add(item_id)
        iris.register(item_id, parent_iri, name, DerivedIriRegistry.DERIVATION)
        return item_id

    controller_by_id = {
        _field(controller, "id"): controller
        for motion in motions
        for controller in (_field(motion, "controllers", []) or [])
    }
    for closure in closures.values():
        if not isinstance(closure, dict):
            continue
        if closure.get("type") == "Admittance":
            for name in _ADMITTANCE_PARAMETERS:
                closure[name] = publish(closure["id"], name, closure[name])
            continue
        if closure.get("type") != "Controller":
            continue
        controller = controller_by_id.get(closure.get("id"))
        gains = _CONTROL_GAINS.get(_field(controller, "type"))
        if not gains:
            continue
        closure["controller_type"] = _field(controller, "type")
        closure["gains"] = {
            name: publish(closure["id"], name, _field(controller, source), required)
            for name, source, required in gains
        }
        # The bounds are authored shared quantities already; the call site reads them by id.
        closure["integral_saturation"] = _field(controller, "integral_saturation")


# One declaration per joint-space channel: what writes it, and what it measures. Kept in one place
# so the producer cannot drift from the mirror expression -- the template's joint-space-expr-<name>
# reads exactly the thing named here.
#
# Quantity kinds are not a guess: scene-dsl keeps only RevoluteJointModel joints in a chain
# (`scene_dsl/kdl_tree.py`), so every entry in kdl_joints is revolute and its position is an angle.
#
# `backends` is None where every backend carries the signal, or the backends that actually
# measure it. A channel is logged only where it exists: a slot that is structurally zero is not a
# measurement, and logging it every tick would say the arm is weightless.
JointSpaceChannel = collections.namedtuple(
    "JointSpaceChannel", ("name", "producer", "quantity_kind", "unit", "backends")
)

_JOINT_SPACE_CHANNELS = (
    JointSpaceChannel("q", "port", "Angle", "RAD", None),
    JointSpaceChannel("qd", "port", "AngularVelocity", "RAD_PER_SEC", None),
    JointSpaceChannel("qdd", "solver", "AngularAcceleration", "RAD_PER_SEC2", None),
    JointSpaceChannel("tau_ctrl", "solver", "Torque", "N_M", None),
    # robif2b reads eff_msr off the hardware. mj_kdl has no equivalent: under torque control the
    # wrapper nulls the position actuator and applies the command through qfrc_applied, so
    # jnt_trq_msr (which mirrors qfrc_actuator) is zero by construction, not by measurement.
    # Add "mj_kdl" here the day the wrapper reports the joint's actual generalized force.
    JointSpaceChannel("tau_msr", "sensor", "Torque", "N_M", ("robif2b",)),
)

# Emitted only where a torque limit is authored. Its producer is that saturation, not the solver:
# without a limit the command port just holds tau_ctrl, and the channel does not exist at all.
_JOINT_SPACE_CMD_CHANNEL = JointSpaceChannel("tau_cmd", "saturation", "Torque", "N_M", None)


def _cpp_identifier(name: str) -> str:
    """Sanitize a MuJoCo joint name into a C++ identifier fragment."""
    return re.sub(r"[^0-9A-Za-z_]", "_", name)


def add_joint_space_logging(
    serial_chain_solvers, motions, shared_data: list, introspection: dict, backend: str, iris
) -> None:
    """Mirror each runtime's joint-space signals into shared_data so the frame log can carry them.

    Keyed by runtime rather than by solver: q/qd/tau_msr/tau_cmd are the arm's ports, and
    tau_ctrl/qdd follow the same key because the command port is last-writer-wins, so a
    runtime-keyed slot records what the port received even when several solvers on one runtime
    are active in the same tick. Keying by solver would multiply the field count by the number of
    motions (5x on the arc, 10x on the dual arm) to record the same ports.
    """
    quantities = introspection.setdefault("quantities", [])
    shared_ids = {_field(item, "id") for item in shared_data if _field(item, "id")}

    # The per-motion copies (HandlerSerialChainSolver) are what the templates render, and they
    # carry neither runtime_id nor kdl_joints -- derive from the top-level solvers, assign to both.
    copies_by_id: dict[str, list] = {}
    for motion in motions:
        for solver in _field(motion, "serial_chain_solvers", []) or []:
            copies_by_id.setdefault(_field(solver, "id"), []).append(solver)

    by_runtime: dict[str, list] = {}
    for solver in serial_chain_solvers:
        by_runtime.setdefault(_field(solver, "runtime_id") or _field(solver, "id"), []).append(
            solver
        )

    for runtime_id, solvers in by_runtime.items():
        joints = _field(solvers[0], "kdl_joints") or []
        if not joints:
            raise RuntimeError(
                f"joint-space logging: solver '{_field(solvers[0], 'id')}' has no kdl_joints; "
                "the shared ids are compile-time names, so a wrong joint count mislabels "
                "every channel"
            )
        # tau_cmd can differ from tau_ctrl only where a limit clamps it, so its producer is that
        # saturation. Several solvers on one runtime could each carry one; name it only when the
        # runtime has exactly one, the same rule the rest of the contract uses.
        saturations = {
            _field(_field(solver, "torque_saturation"), "id")
            for solver in solvers
            if _field(solver, "torque_saturation")
        }
        available = tuple(
            channel
            for channel in _JOINT_SPACE_CHANNELS
            if channel.backends is None or backend in channel.backends
        )
        channels = available + ((_JOINT_SPACE_CMD_CHANNEL,) if saturations else ())
        producer_id = {
            "port": runtime_id,
            "sensor": runtime_id,
            "solver": _sole({_field(solver, "id") for solver in solvers}),
            "saturation": _sole(saturations),
        }

        ids_by_channel: dict[str, list] = {channel.name: [] for channel in channels}
        # Mirrors are keyed by runtime, so they derive from the runtime's solver node.
        runtime_iri = iris.iri_of(runtime_id) or iris.iri_of(_field(solvers[0], "id"))
        if runtime_iri is None:
            raise RuntimeError(
                f"joint-space logging: runtime '{runtime_id}' has no IRI to derive from"
            )
        for index, joint in enumerate(joints):
            for channel in channels:
                shared_id = f"{runtime_id}_{channel.name}_{_cpp_identifier(joint)}"
                if shared_id in shared_ids:
                    raise RuntimeError(
                        f"joint-space logging: id '{shared_id}' collides with an existing "
                        "shared value"
                    )
                shared_ids.add(shared_id)
                entry = {
                    "id": shared_id,
                    "type": "Quantity",
                    "quantity_kind": {"id": channel.quantity_kind, "type": "QuantityKind"},
                    "unit": {"id": channel.unit, "type": "Unit"},
                    "runtime": runtime_id,
                    "role": "joint_space",
                    "channel": channel.name,
                    "joint": joint,
                    # Declared where it is known -- the mirror site -- so the dataflow contract
                    # reads it rather than re-deriving it from the channel name.
                    "producer": {
                        "kind": channel.producer,
                        "id": producer_id[channel.producer],
                    },
                }
                shared_data.append(entry)
                quantities.append(dict(entry))
                iris.register(
                    shared_id,
                    runtime_iri,
                    f"{channel.name}-{_cpp_identifier(joint)}",
                    DerivedIriRegistry.DERIVATION,
                )
                ids_by_channel[channel.name].append({"id": shared_id, "index": index})

        # Every solver on the runtime mirrors the same ids: whichever motion is active writes them.
        samples = [
            {"id": sample["id"], "channel": channel.name, "index": sample["index"]}
            for channel in available
            for sample in ids_by_channel[channel.name]
        ]
        cmd_ids = ids_by_channel.get(_JOINT_SPACE_CMD_CHANNEL.name, [])
        for solver in solvers:
            cmd_samples = cmd_ids if _field(solver, "torque_saturation") else []
            for target in (solver, *copies_by_id.get(_field(solver, "id"), ())):
                _set_field(target, "joint_space_samples", samples)
                _set_field(target, "joint_space_cmd_samples", cmd_samples)


# Types that get a whole-object frame-log slot (PoseSlot/TwistSlot/WrenchSlot).
_SPATIAL_SLOT_KINDS = {"Pose": "poses", "VelocityTwist": "twists", "Wrench": "wrenches"}


def _spatial_slot_ids(shared_data: list) -> set:
    """Ids carried by a whole-object spatial slot, so no per-axis scalar rows are emitted too.

    AccelerationTwist and PoseDifference have no slot, so their scalar rows are the only record
    of them and must survive.
    """
    return {
        _field(item, "id")
        for item in shared_data
        if _field(item, "type") in _SPATIAL_SLOT_KINDS and _field(item, "id")
    }


def add_quantity_samples(introspection: dict, shared_data: list, views: dict) -> None:
    """Build the per-quantity frame-log sample descriptors from the introspection quantities and
    shared data.
    """
    shared_ids = {_field(item, "id") for item in shared_data if _field(item, "id")}
    spatial_ids = _spatial_slot_ids(shared_data)
    indexed_views = _views_by_subobject(views)
    samples = []

    # Each sample carries a backend-agnostic descriptor (kind + ids/axis); the C++
    # sample expression is rendered by the sample-expr template (shared_data.stg).
    def add(source, component: str, desc: dict) -> None:
        """Append one scalar frame-log sample row for a source and component."""
        src = _as_dict(source)
        row = {key: value for key, value in src.items() if key != "index"}
        source_id = src.get("id")
        row.update(
            {
                "id": source_id if not component else f"{source_id}.{component}",
                "source_id": source_id,
                "component": component or None,
                "type": "Scalar",
                "source_type": src.get("type"),
                "sample_desc": desc,
            }
        )
        samples.append(row)

    def add_axes(source, prefix: str, make_desc) -> None:
        """Append per-axis (x/y/z) sample rows for a vector quantity."""
        for idx, axis in enumerate(("x", "y", "z")):
            add(source, f"{prefix}.{axis}" if prefix else axis, make_desc(idx))

    def scalar_view(data_id: str) -> bool:
        """True when a data id has no view or its view selects a single axis."""
        matches = indexed_views.get(data_id, ())
        return not matches or all(_field(view, "axis") is not None for view in matches)

    for quantity in introspection.get("quantities", []):
        qid = quantity.get("id")
        if not qid or qid in spatial_ids:
            continue
        qtype = quantity.get("type")
        if qtype == "Quantity":
            if (
                quantity.get("value") is not None
                and qid not in shared_ids
                and qid not in indexed_views
            ):
                add(quantity, "", {"kind": "literal", "value": str(quantity["value"])})
            elif qid in indexed_views and scalar_view(qid):
                # A scalar view resolves to a composite-member access only for these
                # superobject types; other superobjects (e.g. PoseDifference) sample the
                # quantity's own shared field instead.
                qviews = indexed_views[qid]
                so_types = {_field(_field(view, "superobject"), "type") for view in qviews}
                if len(so_types) != 1:
                    raise ValueError(
                        f"quantity sampling: '{qid}' belongs to incompatible MAP views"
                    )
                so_type = next(iter(so_types))
                if so_type in {"Pose", "Wrench", "VelocityTwist", "AccelerationTwist"}:
                    add(quantity, "", {"kind": "access", "ref": qid})
                else:
                    add(quantity, "", {"kind": "shared", "id": qid})
            elif qid in shared_ids and qid not in indexed_views:
                add(quantity, "", {"kind": "shared", "id": qid})
        elif qtype in {"Position", "Direction", "FreeVector"} and qid in shared_ids:
            add_axes(quantity, "", lambda i, q=qid: {"kind": "vec", "id": q, "axis": i})
        elif qtype == "Orientation" and qid in shared_ids:
            add_axes(quantity, "", lambda i, q=qid: {"kind": "orientation", "id": q, "axis": i})
        elif qtype in {"Pose", "Setpoint"} and qid in shared_ids:
            add_axes(
                quantity, "position", lambda i, q=qid: {"kind": "pose_pos", "id": q, "axis": i}
            )
            add_axes(
                quantity,
                "orientation",
                lambda i, q=qid: {"kind": "pose_orient", "id": q, "axis": i},
            )
        elif (
            qtype in {"VelocityTwist", "AccelerationTwist", "PoseDifference"} and qid in shared_ids
        ):
            add_axes(
                quantity,
                "angular",
                lambda i, q=qid: {"kind": "member", "id": q, "member": "rot", "axis": i},
            )
            add_axes(
                quantity,
                "linear",
                lambda i, q=qid: {"kind": "member", "id": q, "member": "vel", "axis": i},
            )
        elif qtype == "Wrench" and qid in shared_ids:
            add_axes(
                quantity,
                "torque",
                lambda i, q=qid: {"kind": "member", "id": q, "member": "torque", "axis": i},
            )
            add_axes(
                quantity,
                "force",
                lambda i, q=qid: {"kind": "member", "id": q, "member": "force", "axis": i},
            )

    sampled_ids = {sample.get("source_id") for sample in samples}
    for item in shared_data:
        item_id = _field(item, "id")
        if not item_id or item_id in sampled_ids:
            continue
        if _field(item, "type") == "Bool":
            add(item, "", {"kind": "bool", "id": item_id})
        elif _field(item, "type") == "IntCounter":
            add(item, "", {"kind": "int", "id": item_id})
        elif item_id in _PORT_PRODUCERS:
            # No model entity declares it, so it has no `quantities` row to be sampled from.
            add(item, "", {"kind": "shared", "id": item_id})

    introspection["quantity_samples"] = samples


def add_spatial_samples(introspection: dict, shared_data: list) -> None:
    """Add per-object pose, velocity-twist and wrench frame-log samples."""
    spatial = {"poses": [], "twists": [], "wrenches": []}
    for item in shared_data:
        iid = _field(item, "id")
        pool = _SPATIAL_SLOT_KINDS.get(_field(item, "type"))
        if not iid or pool is None:
            continue
        spatial[pool].append({"id": iid, "index": len(spatial[pool])})
    introspection["spatial_samples"] = spatial


# Storage follows from write cadence, in one place. A value written once says nothing new when
# repeated per tick, and one never written is not a runtime value at all.
_STORAGE_BY_CADENCE = {"never": "absent", "init": "record", "tick": "log"}

# Shared values the control loop writes from a backend port, not from any model entity. Their
# initial literal is a fallback, not an authored constant, so the contract is stated here rather
# than inferred from the value being present.
_PORT_PRODUCERS = {
    "clock_time_s": {"kind": "port", "id": "clock"},
    "dt_measured_s": {"kind": "port", "id": "clock"},
}

_MOTION_SCHEDULES = ("when_schedule", "while_pre_schedule", "while_schedule", "until_schedule")

_LITERAL_FIELDS = ("position", "direction", "orientation", "value", "vector")

# Fields on a shared-data entry that carry the numbers behind a `vec` sample descriptor.
_LITERAL_VECTORS = ("position", "direction", "vector")


def _storage_for(cadence) -> str:
    """Derive where a value belongs from when it is written (never override this per value)."""
    return "log" if isinstance(cadence, dict) else _STORAGE_BY_CADENCE[cadence]


def _sole(ids: set) -> str | None:
    """The one id in a set, or None when several instances write the same value."""
    return next(iter(ids)) if len(ids) == 1 else None


def _constant_value(item, desc: dict):
    """The authored number a `cadence: init` sample row carries, for the schema header."""
    kind = desc.get("kind")
    if kind == "literal":
        return float(desc["value"])
    if kind in {"shared", "access", "bool", "int"}:
        return _field(item, "value")
    if kind == "vec":
        for field in _LITERAL_VECTORS:
            values = _field(item, field)
            if values is not None:
                return values[desc["axis"]]
    return None


def _agent_home_positions(platform, solvers) -> dict:
    """Each agent's reset joint configuration, keyed the way `_config_key` names it.

    Where a robot starts decides every run, so it is authored beside the model rather than
    baked into a template no run artifact could report. There is no default: a simulated
    deployment that states none is rejected, because a wrong home is silent and a missing one
    should not be.
    """
    import tomllib

    if not platform.get("simulated"):
        return {}
    owners = [s for s in solvers if _field(s, "runtime_owner")]
    if not owners:
        return {}
    config_path = platform.get("config")
    if not config_path:
        raise ValueError(
            "A simulated platform must declare `config: \"<file>.toml\"` in its exec-context, "
            "stating a [<agent>] home for every agent it drives."
        )
    resolved = Path(config_path)
    if not resolved.is_file():
        raise ValueError(f"{resolved} does not exist, but the exec-context declares it.")
    config = tomllib.loads(resolved.read_text())
    homes = {
        f"{alias}.{leaf}": [float(v) for v in entry["home"]]
        for alias, entries in config.items()
        if isinstance(entries, dict)
        for leaf, entry in entries.items()
        if isinstance(entry, dict) and entry.get("home")
    }
    missing = sorted(
        _field(s, "config_key") for s in owners if _field(s, "config_key") not in homes
    )
    if missing:
        raise ValueError(
            f"{resolved} states no home for {missing}. Every agent the model drives needs one; "
            "add a [<agent>] section with `home = [...]`."
        )
    return homes


def annotate_dataflow(
    introspection: dict, shared_data: list, closures: dict, motions, serial_chain_solvers, views
) -> dict:
    """Give every shared value its producer, its write cadence, and the storage those imply, then
    apply that contract: drop what nothing writes, move what is written once into the header, and
    project the roles the templates ask about (``values``) off the same classification.

    Cadence -- not motion membership -- decides gating: a value written by several motions carries
    all of them, and one no motion's step function writes falls back to ``tick``, so nothing live
    is gated away by being attributed to a single motion.
    """
    closure_by_output: dict[str, set] = {}
    for closure_id, closure in closures.items():
        if isinstance(closure, dict):
            for out_id in closure_output_ids(closure):
                closure_by_output.setdefault(out_id, set()).add(closure_id)

    solver_by_output: dict[str, set] = {}
    sensor_outputs: set = set()
    # The readings themselves, without the tare companions below: only these are supplied by the
    # platform, and only they get an external-measurement pointer.
    measured_outputs: set = set()
    for solver in serial_chain_solvers:
        for out in _field(solver, "output", []) or []:
            out_id = _field(out, "id")
            solver_by_output.setdefault(out_id, set()).add(_field(solver, "id"))
            if _field(out, "sensor_name"):
                sensor_outputs.add(out_id)
                measured_outputs.add(out_id)
                # tare state, written alongside the reading (_shared_runtime_members)
                for companion in (f"{out_id}_ft_bias", f"{out_id}_ft_settle"):
                    solver_by_output.setdefault(companion, set()).add(_field(solver, "id"))
                    sensor_outputs.add(companion)

    # Which motions' step functions write a value. A union, never last-writer-wins: every motion
    # instantiates its own solver over the same shared outputs, and one closure can be scheduled
    # by several motions -- attributing such a value to one motion would gate away live data.
    owners: dict[str, set] = {}
    # Poses, snapshots and per-axis errors are written by the pose-composition, snapshot and
    # error-decomposition blocks, which are emitted per motion rather than scheduled as closures.
    pose_ids: set = set()
    snapshot_ids: set = set()
    axis_error_ids: set = set()

    def own(data_id, motion_id) -> None:
        if isinstance(data_id, str):
            owners.setdefault(data_id, set()).add(motion_id)

    for motion in motions:
        motion_id = _field(motion, "id")
        for schedule in _MOTION_SCHEDULES:
            for closure_id in _field(motion, schedule, []) or []:
                for out_id in closure_output_ids(closures.get(closure_id) or {}):
                    own(out_id, motion_id)
        for solver in _field(motion, "serial_chain_solvers", []) or []:
            for out in _field(solver, "output", []) or []:
                out_id = _field(out, "id")
                own(out_id, motion_id)
                if _field(out, "sensor_name"):
                    own(f"{out_id}_ft_bias", motion_id)
                    own(f"{out_id}_ft_settle", motion_id)
            # Joint-space mirrors are written by whichever motion's solver ran (add_joint_space_
            # logging), so the runtime's channels are live in every motion that drives it.
            for sample in (_field(solver, "joint_space_samples", []) or []) + (
                _field(solver, "joint_space_cmd_samples", []) or []
            ):
                own(_field(sample, "id"), motion_id)
        for group in (
            _field(motion, "declared_pose_components", []) or [],
            _field(motion, "relative_poses", []) or [],
        ):
            for entry in group:
                own(_field(entry, "id"), motion_id)
                pose_ids.add(_field(entry, "id"))
        for snapshot in _field(motion, "snapshots", []) or []:
            own(_field(snapshot, "target_id"), motion_id)
            snapshot_ids.add(_field(snapshot, "target_id"))
            # The source closure runs inside the snapshot block, not from a schedule.
            for out_id in closure_output_ids(
                closures.get(_field(snapshot, "source_closure_id")) or {}
            ):
                own(out_id, motion_id)
        for group in _field(motion, "pose_axis_error_groups", []) or []:
            for component in _field(group, "components", []) or []:
                own(_field(component, "error"), motion_id)
                axis_error_ids.add(_field(component, "error"))

    def contract(item) -> tuple[dict, object]:
        """(producer, cadence) for one shared-data member."""
        item_id = _field(item, "id")
        if item_id in _PORT_PRODUCERS:
            return _PORT_PRODUCERS[item_id], "tick"
        motion_ids = owners.get(item_id)
        cadence = {"motions": sorted(motion_ids)} if motion_ids else "tick"
        if _field(item, "role") == "joint_space":
            # Declared at the mirror site (add_joint_space_logging), which is the only place that
            # knows what the expression reads.
            return _field(item, "producer"), cadence
        if item_id in closure_by_output:
            producer_ids = closure_by_output[item_id]
            types = {(closures.get(cid) or {}).get("type") for cid in producer_ids}
            kind = "controller" if types == {"Controller"} else "closure"
            return {"kind": kind, "id": _sole(producer_ids)}, cadence
        if item_id in solver_by_output:
            kind = "sensor" if item_id in sensor_outputs else "solver"
            return {"kind": kind, "id": _sole(solver_by_output[item_id])}, cadence
        for kind, ids in (
            ("pose", pose_ids),
            ("snapshot", snapshot_ids),
            ("decomposition", axis_error_ids),
        ):
            if item_id in ids:
                return {"kind": kind, "id": item_id}, cadence
        # `is not None`, not truthiness: an authored 0.0 is a value, not a missing one.
        if any(_field(item, field) is not None for field in _LITERAL_FIELDS):
            return {"kind": "authored", "id": None}, "init"
        return {"kind": "none", "id": None}, "never"

    # A view is a projection of its superobject, so it is written exactly when that is. One
    # subobject can MAP into several superobjects, so collect them all rather than let the last
    # view win -- the value is live whenever any of them is recomputed.
    superobjects_of: dict[str, set] = {}
    for view in (views or {}).values():
        subobject_id = _field(_field(view, "subobject"), "id")
        superobject_id = _field(_field(view, "superobject"), "id")
        if subobject_id and superobject_id:
            superobjects_of.setdefault(subobject_id, set()).add(superobject_id)

    # One artifact holds the contract, rather than three fields smeared over every entity
    # dataclass: storage is derived from cadence in exactly one place and cannot drift from it.
    dataflow = {}
    items_by_id = {}
    for item in shared_data:
        item_id = _field(item, "id")
        if not item_id:
            continue
        items_by_id[item_id] = item
        producer, cadence = contract(item)
        dataflow[item_id] = {
            "producer": producer,
            "cadence": cadence,
            "storage": _storage_for(cadence),
        }

    for subobject_id, superobject_ids in superobjects_of.items():
        entry = dataflow.get(subobject_id)
        sources = [dataflow[sid] for sid in sorted(superobject_ids) if sid in dataflow]
        if entry is None or not sources:
            continue
        live = [source["cadence"] for source in sources if source["storage"] == "log"]
        if live:
            # A read of a recomputed superobject is live even when the view itself was authored
            # with a literal: the literal is only what the superobject started from.
            cadence = (
                "tick"
                if any(not isinstance(source, dict) for source in live)
                else {"motions": sorted({m for source in live for m in source["motions"]})}
            )
        elif entry["producer"]["kind"] == "none":
            cadence = sources[0]["cadence"]
        else:
            continue
        entry["producer"] = {"kind": "view", "id": _sole(superobject_ids)}
        entry["cadence"] = cadence
        entry["storage"] = _storage_for(cadence)

    for member_id, consumers in _consumers_by_id(introspection, closures).items():
        entry = dataflow.get(member_id)
        if entry is not None:
            entry["consumers"] = consumers

    # A value nothing writes but something reads is not model metadata -- it is a broken binding,
    # and dropping it would silently feed the reader a zero forever.
    orphans = {
        member_id: entry["consumers"]
        for member_id, entry in dataflow.items()
        if entry["cadence"] == "never" and entry.get("consumers")
    }
    if orphans:
        raise RuntimeError(
            "dataflow: read but never written: "
            + "; ".join(
                f"{member_id} (read by {', '.join(c['id'] for c in consumers)})"
                for member_id, consumers in sorted(orphans.items())
            )
        )

    introspection["dataflow"] = dataflow
    _apply_dataflow(introspection, shared_data, items_by_id, dataflow)

    # Layer-B projections of the same contract: "which values play role X?" answered from the
    # producer classification above, never by a second scan with its own rule. Externally
    # measured = a solver output the platform supplies (it has a sensor), which the view turns
    # into a pointer member plus a measurement local. Returned, not stored on introspection:
    # the projection is published once, at the top level.
    return {
        "externally_measured": sorted(
            (item for item in shared_data if _field(item, "id") in measured_outputs),
            key=lambda item: _field(item, "id"),
        )
    }


def _consumers_by_id(introspection: dict, closures: dict) -> dict[str, list]:
    """Who reads each shared value: the monitors, controllers and closures bound to it."""
    consumers: dict[str, list] = {}

    def add(member_id, kind: str, reader_id, role: str) -> None:
        if isinstance(member_id, str) and reader_id:
            consumers.setdefault(member_id, []).append(
                {"kind": kind, "id": reader_id, "role": role}
            )

    for monitor in introspection.get("monitors", []):
        add(monitor.get("error_signal"), "monitor", monitor.get("id"), "error")
        # Without this the band is a shared value nothing reads, and the contract drops it.
        add(monitor.get("tolerance_signal"), "monitor", monitor.get("id"), "tolerance")
    for controller in introspection.get("controllers", []):
        for role in ("error_signal", "measured_signal", "setpoint_signal"):
            add(controller.get(role), "controller", controller.get("id"), role)
    for closure_id, closure in closures.items():
        if not isinstance(closure, dict):
            continue
        outputs = closure_output_ids(closure)
        for key, value in closure.items():
            if key not in {"id", "type"} and isinstance(value, str) and value not in outputs:
                add(value, "closure", closure_id, key)
    # Readers are collected from dicts whose order is the graph's; the list is an artifact.
    for readers in consumers.values():
        readers.sort(key=lambda reader: (reader["kind"], reader["id"], reader["role"]))
    return consumers


def _apply_dataflow(introspection: dict, shared_data: list, items_by_id: dict, dataflow: dict):
    """Act on the contract: absent values leave the program, init values leave the per-tick frame."""
    shared_data[:] = [
        item
        for item in shared_data
        if dataflow.get(_field(item, "id"), {}).get("storage") != "absent"
    ]

    logged, constants, unattributed = [], [], []
    for sample in introspection.get("quantity_samples", []):
        entry = dataflow.get(sample.get("source_id"))
        if entry is None:
            unattributed.append(sample.get("id"))
            continue
        sample.update(entry)
        if entry["storage"] == "log":
            logged.append(sample)
        elif entry["storage"] == "record":
            value = _constant_value(items_by_id[sample["source_id"]], sample["sample_desc"])
            if value is None:
                unattributed.append(sample.get("id"))
                continue
            constants.append({"id": sample["id"], "source_id": sample["source_id"], "value": value})
    if unattributed:
        raise RuntimeError(f"dataflow: samples with no resolvable contract: {sorted(unattributed)}")
    introspection["quantity_samples"] = logged
    introspection["constants"] = constants

    spatial = introspection.get("spatial_samples") or {}
    for pool, rows in spatial.items():
        kept = [row for row in rows if dataflow.get(row["id"], {}).get("storage") == "log"]
        spatial[pool] = [dict(row, index=idx) for idx, row in enumerate(kept)]


def _index_by_id(items: list) -> dict:
    """Index IR items (dicts or dataclasses) by their id (skips id-less entries)."""
    return {iid: item for item in items if (iid := _field(item, "id"))}


_ORIENTATION_COMPONENTS = {
    "quaternion": ("x", "y", "z", "w"),
    "euler": ("x", "y", "z"),  # symbolic only: the angles arrive at runtime
    "relative": (),
}


def _empty_pose_entry(representation: str, euler_axes: str | None = None) -> dict:
    """Blank component slots for a pose, sized to what will fill them.

    A symbolic Euler triple is filled one angle per authored axis, and the axes are the
    sequence the model wrote -- `zyx` fills z, y and x -- so it is sized by that rather than
    by a fixed component list.
    """
    entry = {"representation": representation}
    entry.update({f"position_{axis}": None for axis in ("x", "y", "z")})
    names = tuple(euler_axes) if euler_axes else _ORIENTATION_COMPONENTS[representation]
    entry.update({f"orientation_{name}": None for name in names})
    return entry


def build_pose_components(views: dict, data: list, graph=None, pose_nodes=None) -> dict:
    """Resolve declared/inline poses into per-axis structured components (a literal value or a
    reference id).
    """
    data_by_id = _index_by_id(data)
    components: dict[str, dict] = {}
    if graph is not None:
        for pose in data:
            if _field(pose, "type") != "Pose":
                continue
            pose_id = _field(pose, "id")
            coord_id = (pose_nodes or {}).get(pose_id)
            if coord_id is None:
                continue
            coordinate = PoseCoordModel(coord_id, graph)
            representation = _field(pose, "orientation_representation") or "quaternion"
            entry = _empty_pose_entry(representation, _field(pose, "euler_axes_sequence"))
            position = get_coord_vectorxyz(coordinate.position_coord, graph)
            if position is not None:
                length_unit = _length_unit(coordinate.position_coord)
                for axis, value in zip("xyz", _si_all(position, length_unit)):
                    entry[f"position_{axis}"] = {"value": str(value), "ref": None}
            if representation == "quaternion":
                # Whatever the model authored -- Euler angles, a quaternion, direction cosines --
                # scipy resolves it to one rotation, and it leaves here as a quaternion. Anything
                # sourced at runtime keeps its own shape and is rendered, not resolved.
                rotation = get_orientation_coord_vals(coordinate.orientation_coord, graph)
                values = rotation.as_quat() if rotation is not None else None
                labels = "xyzw"
            else:
                values = None
                labels = ()
            if values is not None:
                for label, value in zip(labels, values):
                    entry[f"orientation_{label}"] = {"value": str(float(value)), "ref": None}
            if representation == "relative":
                entry["orientation_operands"] = _field(pose, "orientation_operands")
            if any(value is not None for key, value in entry.items() if key != "representation"):
                components[pose_id] = entry
    for view in views.values():
        superobject = _field(view, "superobject")
        so_type = _field(superobject, "type")
        so_prov = _field(superobject, "provenance") or {}
        is_declared_pose = bool(_field(so_prov, "authored") or _field(so_prov, "snapshot"))
        if so_type != "Pose":
            continue
        if not (is_declared_pose or _field(superobject, "euler_axes_sequence")):
            continue
        pose_id = _field(superobject, "id")
        representation = _field(superobject, "orientation_representation") or "quaternion"
        axis = str(_field(view, "axis") or "").lower()
        subobject = _field(_field(view, "subobject"), "id")
        if not subobject or axis not in {"x", "y", "z", "w"}:
            continue
        entry = components.setdefault(
            pose_id,
            _empty_pose_entry(representation, _field(superobject, "euler_axes_sequence")),
        )
        if representation == "relative":
            entry["orientation_operands"] = _field(superobject, "orientation_operands")
        prefix = "position" if _field(view, "subspace") == "Linear" else "orientation"
        entry[f"{prefix}_{axis}"] = _pose_component(subobject, data_by_id)
    for pose_id, parts in components.items():
        missing = [name for name, value in parts.items() if value is None]
        if missing:
            raise ValueError(
                f"Declared pose '{pose_id}' is missing required components: {', '.join(missing)}."
            )
        if parts["representation"] == "euler":
            parts["euler_factors"] = _euler_factors(pose_id, parts, data_by_id)
    return components


def _euler_factors(pose_id: str, parts: dict, data_by_id: dict) -> list[dict]:
    """A symbolic Euler triple as per-axis rotations, in the order they multiply.

    An extrinsic sequence turns about axes that stay put, so the rotation authored last is
    applied to the result of the others and multiplies on the left; an intrinsic one turns
    about axes carried along by the previous rotations, so the order reverses. Each component
    renders wherever its value comes from, so an angle measured or computed at runtime
    composes exactly like a constant.
    """
    pose = data_by_id.get(pose_id)
    sequence = _field(pose, "euler_axes_sequence") or "xyz"
    factors = [
        {"axis": axis, "component": parts[f"orientation_{axis}"]}
        for axis in sequence
        if parts.get(f"orientation_{axis}") is not None
    ]
    if len(factors) != len(sequence):
        raise ValueError(
            f"Euler pose '{pose_id}' has no component for every axis of '{sequence}'."
        )
    return factors if _field(pose, "euler_intrinsic") else list(reversed(factors))


def resolve_lerp_closures(closures: dict, pose_components: dict) -> None:
    """Fold each linear-path goal into components or a shared-signal ref."""
    for closure in closures.values():
        if closure.get("shape") != "LinearPath":
            continue
        goal = closure.get("goal")
        if not isinstance(goal, str):
            continue
        if goal in pose_components and closure.get("type") == "PathProjection":
            # Emit the structured pose components; the template builds the pose frame. Only
            # the projection assigns: it is scheduled before the frame and the evaluator,
            # which read the same shared goal.
            closure["goal_components"] = pose_components[goal]
            closure["assign_goal"] = True
        else:
            # goal is a shared signal id (already on the closure as closure["goal"]).
            closure["assign_goal"] = False


def resolve_arc_closures(closures: dict, data: list) -> None:
    """Validate that each Arc closure's end is a Pose (the template renders its
    position/orientation).
    """
    data_by_id = _index_by_id(data)

    def is_pose(data) -> bool:
        """True when a data structure is a Pose quantity."""
        qkind = _field(data, "quantity_kind")
        qkind_ids = qkind if isinstance(qkind, list) else [qkind]
        return _field(data, "type") == "Pose" or any(
            _field(item, "id") == "Pose" for item in qkind_ids
        )

    for closure in closures.values():
        if closure.get("shape") != "Arc":
            continue
        end = closure.get("end")
        end_data = data_by_id.get(end)
        if not isinstance(end, str) or not is_pose(end_data):
            raise ValueError("Arc path end must be a Pose quantity.")
        # end is a validated Pose shared signal; the template renders shared.<end>.p/.M.


def declared_pose_component_entries(
    data: list, pose_components: dict, referenced_ids: set[str] | None = None
) -> list[dict]:
    """Authored declared-pose component entries, optionally restricted to the referenced ids."""
    data_by_id = _index_by_id(data)
    entries = []
    for pose_id, parts in pose_components.items():
        if referenced_ids is not None and pose_id not in referenced_ids:
            continue
        item = data_by_id.get(pose_id)
        item_prov = _field(item, "provenance") or {}
        if not _field(item_prov, "authored") or _field(item_prov, "snapshot"):
            continue
        entries.append({"id": pose_id, **parts})
    return entries


def collect_motion_references(motion, closures: dict) -> set[str]:
    """Every id a motion references, including through its scheduled closures."""
    refs: set[str] = set()

    def visit(value):
        """Recurse a value collecting every string id it references."""
        if isinstance(value, str):
            refs.add(value)
        elif isinstance(value, dict):
            for item in value.values():
                visit(item)
        elif isinstance(value, list):
            for item in value:
                visit(item)

    visit(_as_dict(motion))
    for schedule_name in ("when_schedule", "while_schedule", "until_schedule"):
        for step in _field(motion, schedule_name, []):
            closure = closures.get(step)
            if closure:
                visit(closure)
    return refs


def _elapsed_coordinate_id(e) -> str:
    """The shared value an elapsed constraint measures: its own authored duration coordinate.

    A timing constraint's error signal is the elapsed duration itself, so this is where the
    motion writes the seconds and where every reader -- the condition inside the motion, the
    introspection sample outside it, the frame log -- finds them."""
    coordinate = _field(_field(e, "error"), "id")
    if not coordinate:
        raise ValueError(
            f"elapsed constraint '{_field(e, 'id')}' has no duration coordinate to measure into"
        )
    return coordinate


def _elapsed_coordinate_ids(evaluators) -> list[str]:
    """Each phase's elapsed coordinates, deduplicated, in authored order."""
    return list(
        dict.fromkeys(
            _elapsed_coordinate_id(e) for e in evaluators if getattr(e, "is_elapsed", False)
        )
    )


def _evaluator_term(e) -> dict:
    """Structured boolean term for an evaluator: an elapsed timing predicate (the seconds a
    phase has been running, against a threshold) or a solver constraint-satisfied check.
    Rendered to C++ by the bool-condition template.

    Every term kind reads shared and nothing else, so the same condition renders identically
    wherever it is needed -- inside the motion, whose state holds the phase's start, and in
    the introspection sample, which runs outside it."""
    if _field(e, "is_elapsed"):
        op = _field(e, "elapsed_op") or ">="
        thr = _field(e, "elapsed_threshold_s") or 0.0
        # Pre-format the threshold (fixed 6-decimal) so the emitted literal is stable.
        if op == "==":
            tol = _field(e, "elapsed_tolerance_s") or 0.0
            return {
                "kind": "elapsed-eq",
                "elapsed_id": _elapsed_coordinate_id(e),
                "threshold": f"{thr:.6f}",
                "tolerance": f"{tol:.6f}",
            }
        return {
            "kind": "elapsed",
            "elapsed_id": _elapsed_coordinate_id(e),
            "op": op,
            "threshold": f"{thr:.6f}",
        }
    term = {"kind": "constraint", "error_id": _field(_field(e, "error"), "id")}
    # Omitted, not empty: ST4 reads an empty string as present, and would emit a bare `shared.`.
    tolerance_id = _field(_field(e, "tolerance"), "id")
    if tolerance_id:
        term["tolerance_id"] = tolerance_id
    return term


def _set_monitor_conditions(motion, evaluators_key: str, monitors_key: str, any_key: str) -> None:
    """Stamp the structured active-phase terms onto the aggregate monitor + any
    elapsed-error monitors. Rendered to C++ by the bool-condition template."""
    evaluators = _field(motion, evaluators_key, [])
    terms = [
        _evaluator_term(e)
        for e in evaluators
        if _field(e, "error") or _field(e, "is_elapsed")
    ]
    any_flag = bool(_field(motion, any_key))
    elapsed_terms_by_error = {
        _field(_field(e, "error"), "id"): _evaluator_term(e)
        for e in evaluators
        if _field(e, "is_elapsed") and _field(e, "error")
    }
    aggregate_key = "is_until_aggregate" if any_key == "until_any" else "is_when_aggregate"

    def _stamp(monitor, active_terms, active_any):
        _set_field(monitor, "active_terms", active_terms)
        _set_field(monitor, "active_terms_present", bool(active_terms))
        _set_field(monitor, "active_any", active_any)
        _set_field(monitor, "has_active", True)

    for monitor in _field(motion, monitors_key, []):
        group_ids = set(_field(monitor, "group_constraint_ids") or ())
        if group_ids:
            _stamp(
                monitor,
                [
                    _evaluator_term(e)
                    for e in evaluators
                    if _field(_field(e, "constraint"), "id") in group_ids
                    and (_field(e, "error") or _field(e, "is_elapsed"))
                ],
                bool(_field(monitor, "group_any")),
            )
            continue
        if _field(monitor, aggregate_key):
            _stamp(monitor, terms, any_flag)
            continue
        error_id = _field(_field(monitor, "error"), "id")
        if error_id in elapsed_terms_by_error:
            _stamp(monitor, [elapsed_terms_by_error[error_id]], False)


def _set_motion_conditions(motion) -> None:
    """Fold the UNTIL/WHEN/done structured boolean terms onto a motion (rendered to C++ by
    the bool-condition template). WHEN joins with when_any, done with until_any."""
    _set_monitor_conditions(motion, "until_evaluators", "until_monitors", "until_any")
    when_terms = [
        _evaluator_term(e)
        for e in _field(motion, "when_evaluators", [])
        if _field(e, "error") or _field(e, "is_elapsed")
    ]
    _set_field(motion, "when_terms", when_terms)
    _set_field(motion, "when_terms_present", bool(when_terms))
    _set_monitor_conditions(motion, "when_evaluators", "when_monitors", "when_any")
    done_terms = _motion_done_terms(motion)
    _set_field(motion, "done_terms", done_terms)
    _set_field(motion, "done_terms_present", bool(done_terms))


def _records_events(monitors: list) -> bool:
    """Whether any of these monitors records an event occurrence into the coordination buffer."""
    return any(_field(m, "is_edge_triggered") for m in monitors)


def add_motion_function_interfaces(motions: list) -> None:
    """Fold per-motion capability booleans (which context objects — state, shared, robot —
    each generated function needs). The C++ signatures and call args are built from these
    by the sig-params / sig-args templates; ir_gen carries no C++ type names.

    Also assigns each motion its introspection index. It is ir_gen's own ordering, and both the
    frame-log schema and the generated sample switch read this one field -- so no consumer has to
    agree with a second generator about which index means which motion."""
    for index, motion in enumerate(motions):
        _set_field(motion, "index", index)
        has_when_elapsed = any(
            _field(e, "is_elapsed") for e in _field(motion, "when_evaluators", [])
        )
        has_when_logic = bool(_field(motion, "when_schedule") or _field(motion, "when_evaluators"))
        when_mons = _field(motion, "when_monitors") or []
        until_mons = _field(motion, "until_monitors") or []
        while_mons = _field(motion, "while_monitors") or []
        has_pose = bool(_field(motion, "declared_pose_components"))
        when_sched = bool(_field(motion, "when_schedule"))
        until_sched = bool(_field(motion, "until_schedule"))
        when_fsm = any(_field(m, "fsm_namespace") for m in when_mons)
        until_fsm = any(_field(m, "fsm_namespace") for m in until_mons)
        has_forwarded_commands = bool(_field(motion, "forwarded_commands"))
        has_serial_chain = bool(_field(motion, "serial_chain_solvers"))

        _set_field(motion, "can_start_needs_state", has_when_elapsed)
        _set_field(motion, "can_start_needs_shared", has_when_logic)
        _set_field(motion, "can_start_needs_robot", False)

        _set_field(motion, "when_needs_state", has_when_elapsed or bool(when_mons))
        _set_field(
            motion,
            "when_needs_shared",
            has_when_elapsed or has_pose or when_sched or bool(when_mons),
        )
        _set_field(motion, "when_needs_robot", when_fsm)

        _set_field(motion, "until_needs_state", bool(until_mons))
        _set_field(motion, "until_needs_shared", until_sched or bool(until_mons))
        _set_field(motion, "until_needs_robot", until_fsm)

        _set_field(motion, "monitor_needs_state", bool(when_mons) or bool(until_mons))
        _set_field(
            motion,
            "monitor_needs_shared",
            when_sched or bool(when_mons) or until_sched or bool(until_mons),
        )
        _set_field(motion, "monitor_needs_robot", when_fsm or until_fsm)

        _set_field(motion, "apply_needs_state", has_serial_chain)
        # tau_cmd is read back from the command port in the stage block, inside apply_. Gate on
        # torque_saturation, not on joint_space_cmd_samples: this runs before _build_introspection,
        # so the sample list does not exist yet. Keep in sync with add_joint_space_logging.
        logs_joint_cmd = any(
            _field(solver, "torque_saturation")
            for solver in (_field(motion, "serial_chain_solvers") or [])
        )
        _set_field(motion, "apply_needs_shared", has_forwarded_commands or logs_joint_cmd)
        _set_field(motion, "apply_needs_robot", has_serial_chain or has_forwarded_commands)

        # An edge is the occurrence, so only an edge-triggered monitor records one -- a flag
        # monitor holds a level and never reaches the coordination buffer.
        when_events = _records_events(when_mons)
        until_events = _records_events(until_mons)
        control_events = _records_events(while_mons)
        _set_field(motion, "when_needs_events", when_events)
        _set_field(motion, "until_needs_events", until_events)
        _set_field(motion, "monitor_needs_events", when_events or until_events)
        _set_field(motion, "control_needs_events", control_events)
        _set_field(motion, "step_needs_events", until_events or control_events)


_FSM_NS = "https://secorolab.github.io/metamodels/behaviour/fsm#"
_EL_NS = "https://secorolab.github.io/metamodels/behaviour/event-loop#"


# ---------------------------------------------------------------------------
# FSM wiring
# ---------------------------------------------------------------------------
def _fsm_from_graph(g) -> dict | None:
    """Frame the FSM named graph (states/events/transitions/reactions, folded into the
    model dataset by motion-spec-dsl) into the same dict shape the standalone .hpp uses,
    so codegen needs no fsm_ir.json read. None when the model imports no .fsm."""
    FSM = rdflib.Namespace(_FSM_NS)
    EL = rdflib.Namespace(_EL_NS)
    fsm_ref = next(iter(g.subjects(RDF["type"], FSM["FiniteStateMachine"])), None)
    if fsm_ref is None:
        return None

    def ident(uri):
        """FSM identifier token (upper-cased var name) for a graph URI."""
        return get_valid_var_name(g.compute_qname(uri)[2]).upper()

    # Graph iteration order is rdflib's, not the model's: sort so two generations of the same
    # model agree on state/event order -- event order especially, since indices are assigned
    # from it and get baked into the generated C++.
    state_uris = dict(sorted((ident(s), str(s)) for s in g.objects(fsm_ref, FSM["states"])))
    states = list(state_uris)
    event_loop_node = g.value(fsm_ref, EL["event-loop"])
    event_uris = dict(
        sorted((ident(e), str(e)) for e in g.objects(event_loop_node, EL["has-event"]))
    )
    events = list(event_uris)

    transitions_table = []
    for tr in g.objects(fsm_ref, FSM["transitions"]):
        transitions_table.append(
            {
                "id": ident(tr),
                "uri": str(tr),
                "from_state": ident(g.value(tr, FSM["transition-from"])),
                "to_state": ident(g.value(tr, FSM["transition-to"])),
            }
        )
    reactions_table = []
    for rx in g.objects(fsm_ref, FSM["reactions"]):
        fires = sorted(ident(ev) for ev in g.objects(rx, FSM["fires-events"]))
        reactions_table.append(
            {
                "id": ident(rx),
                "uri": str(rx),
                "when_event": ident(g.value(rx, EL["ref-event"])),
                "do_transition": ident(g.value(rx, FSM["do-transition"])),
                "fires_events": fires,
                "num_fires": len(fires),
            }
        )
    transitions_table.sort(key=lambda row: row["id"])
    reactions_table.sort(key=lambda row: row["id"])

    description_node = g.value(fsm_ref, FSM["description"])
    # Event/state IRIs share the FSM node's parent path (…/<model>/fsm/); is_fsm_event
    # matches monitor event IRIs against it.
    namespace_uri = str(rdflib.Namespace(f"{iri_parent(fsm_ref)}/"))
    return {
        "name": str(g.value(fsm_ref, FSM["name"])),
        "description": str(description_node) if description_node is not None else None,
        "start_state": ident(g.value(fsm_ref, FSM["start-state"])),
        "end_state": ident(g.value(fsm_ref, FSM["end-state"])),
        "states": states,
        "state_uris": state_uris,
        "events": events,
        "event_uris": event_uris,
        "transitions_table": transitions_table,
        "reactions_table": reactions_table,
        "namespace_uri": namespace_uri,
    }


def _event_to_state(fsm: dict) -> dict[str, str]:
    """Map each FSM event token to the state it transitions out of (the state the motion
    runs in): the from-state of the transition the event's reaction fires."""
    transition_from = {t["id"]: t["from_state"] for t in fsm["transitions_table"]}
    return {
        r["when_event"]: transition_from[r["do_transition"]]
        for r in fsm["reactions_table"]
        if r["do_transition"] in transition_from
    }


def is_fsm_event(monitor, fsm_ns_uri: str | None) -> bool:
    # A monitor fires the FSM only when its event lives in the FSM's namespace;
    # standalone (monitor-owned) events keep the existing warn stub.
    """True when a monitor fires an FSM event: edge-triggered with an event in the FSM namespace."""
    return bool(
        fsm_ns_uri
        and _field(monitor, "is_edge_triggered")
        and iri_is_descendant(fsm_ns_uri, _field(monitor, "event_uri") or "")
    )


def _apply_fsm_wiring(motions, fsm) -> dict:
    """Tag FSM-event monitors + their motions from the framed FSM, and return the codegen
    wiring (C++ namespace, header, heartbeat) that is folded into ``ir["fsm"]``. Runs before the function-interface pass so the FSM-added robot
    param is picked up. No tagging when the model has no FSM. Runs during motion construction."""
    fsm_namespace = fsm["name"].lower() if fsm else None
    events = fsm.get("events", []) if fsm else []
    fsm_event_index = {event: idx for idx, event in enumerate(events)}
    fsm_step_event = "E_STEP" if "E_STEP" in events else None
    # The heartbeat is the clock, so it is not logged every tick. Where a transition's guard *is*
    # the clock, though, that single occurrence is what caused the state change and has to stay
    # observable -- so name those transitions and let the runtime record the event just for them.
    transitions_by_id = {t.get("id"): t for t in (fsm.get("transitions_table", []) if fsm else [])}
    fsm_step_transitions = [
        {"from": transition.get("from_state"), "to": transition.get("to_state")}
        for reaction in (fsm.get("reactions_table", []) if fsm else [])
        if reaction.get("when_event") == fsm_step_event
        for transition in [transitions_by_id.get(reaction.get("do_transition"))]
        if transition
    ]
    meta = {
        "cpp_namespace": fsm_namespace,
        "header": f"{fsm['name']}.hpp" if fsm else None,
        "step_event": fsm_step_event,
        "step_event_idx": fsm_event_index.get(fsm_step_event, -1),
        "step_transitions": fsm_step_transitions,
    }
    if fsm_namespace is None:
        # Without an FSM the sequencer advances on a motion's own `until`, so a motion that
        # declares none can never be left and every motion after it is unreachable.
        stuck = [_field(m, "id") for m in motions if not _field(m, "has_until_condition")]
        if stuck:
            raise ValueError(
                f"motions {sorted(stuck)} declare no 'until' condition and the model imports no "
                "FSM, so nothing can end them; add an 'until' condition or coordinate the model "
                "with an FSM"
            )
        return meta

    fsm_ns_uri = fsm.get("namespace_uri")
    event_state = _event_to_state(fsm)
    by_id = {_field(m, "id"): m for m in motions}

    def stamp_event(monitor):
        """Bind an FSM-event monitor to the FSM's namespace and event slot."""
        _set_field(monitor, "fsm_namespace", fsm_namespace)
        _set_field(
            monitor,
            "fsm_event_idx",
            fsm_event_index.get(_field(monitor, "event_name") or "", -1),
        )

    def tag_run_state(motion, monitors):
        """Tag FSM-event monitors and set their motion's run state."""
        for monitor in monitors:
            if is_fsm_event(monitor, fsm_ns_uri):
                stamp_event(monitor)
                state = event_state.get(_field(monitor, "event_name") or "")
                if state and not _field(motion, "fsm_state"):
                    _set_field(motion, "fsm_state", state)

    for motion in motions:
        # An event-triggered snapshot re-samples when its trigger is in the current event
        # buffer; that only compiles if the event is one this FSM declares.
        for snapshot in _field(motion, "snapshots", []) or []:
            trigger = _field(snapshot, "trigger_event")
            if not trigger:
                continue
            if trigger not in fsm_event_index:
                raise ValueError(
                    f"Snapshot '{_field(snapshot, 'target_id')}' in motion "
                    f"'{_field(motion, 'id')}' triggers on '{trigger}', which the FSM "
                    f"'{fsm_namespace}' does not declare."
                )
            _set_field(snapshot, "fsm_namespace", fsm_namespace)
        tag_run_state(
            motion, _field(motion, "until_monitors", []) + _field(motion, "while_monitors", [])
        )
        for monitor in _field(motion, "when_monitors", []):
            if not is_fsm_event(monitor, fsm_ns_uri):
                continue
            stamp_event(monitor)
            fallback_id = _field(monitor, "fallback_motion")
            if not fallback_id:
                raise ValueError(
                    f"WHEN monitor '{_field(monitor, 'id')}' on FSM-wired motion "
                    f"'{_field(motion, 'id')}' must declare a waiting hold motion "
                    f"(e.g. '... otherwise hold <hold-motion>'). A WHEN precondition "
                    f"without a fallback would leave the arm uncommanded while waiting."
                )
            fallback = by_id.get(fallback_id)
            if fallback is None:
                raise ValueError(
                    f"WHEN monitor '{_field(monitor, 'id')}' names unknown fallback motion "
                    f"'{fallback_id}'."
                )
            state = event_state.get(_field(monitor, "event_name") or "")
            if state and not _field(fallback, "fsm_state"):
                _set_field(fallback, "fsm_state", state)
            gates = _field(fallback, "fsm_when_gate_motions")
            if gates is None:
                gates = []
                _set_field(fallback, "fsm_when_gate_motions", gates)
            if _field(motion, "id") not in gates:
                gates.append(_field(motion, "id"))
    return meta


def _apply_fsm_gate_calls(motions, fsm_namespace) -> None:
    """Fold each fallback state's WHEN-evaluation gate calls: the gated motion id plus its
    when-signature capability booleans. The C++ ``monitor_when_<id>(<args>)`` call is
    rendered by the template via sig-args. Runs after function interfaces so when_needs_*
    are available."""
    if fsm_namespace is None:
        return
    by_id = {_field(m, "id"): m for m in motions}
    for fallback in motions:
        gate_ids = _field(fallback, "fsm_when_gate_motions")
        if not gate_ids:
            continue
        _set_field(
            fallback,
            "fsm_when_gate_calls",
            [
                {
                    "gid": gate_id,
                    "needs_state": _field(by_id[gate_id], "when_needs_state", False),
                    "needs_shared": _field(by_id[gate_id], "when_needs_shared", False),
                    "needs_robot": _field(by_id[gate_id], "when_needs_robot", False),
                    "needs_events": _field(by_id[gate_id], "when_needs_events", False),
                }
                for gate_id in gate_ids
                if gate_id in by_id
            ],
        )


# ---------------------------------------------------------------------------
# Top-level orchestration
# ---------------------------------------------------------------------------
def generate_ir(manifest_path):
    """Build the complete IR for a model manifest in one forward pass and return it as a dict."""
    app_model_path, g, imported_models, imported_provenance = _load_graph(manifest_path)
    from motion_spec.generation.scene_kdl import kdl_header_name

    kdl_header = kdl_header_name(app_model_path)
    _materialize_pose_reference_transforms(g)
    _materialize_linear_distance_operations(g)

    p = Parser(g)
    node_by_id, id_nodes = _node_indexes(g, p)
    iris = DerivedIriRegistry(id_nodes)
    setups_by_node, ordered_setups = _robot_setups_from_graph(g)
    default_setup = (
        ordered_setups[0]
        if ordered_setups
        else ("", "", "", "", "", "", "", [], [], "", [], "", "", [], "")
    )
    # Derive backend + FSM up front: both are pure functions of the graph and are inputs to
    # downstream construction (solver validation, runtime-robot annotation, motion FSM wiring).
    platform = _platform_from_graph(g)
    backend = platform["backend"]
    fsm = _fsm_from_graph(g)
    scene = _scene_from_graph(g)
    _validate_scene(scene)
    derivation = _solver_derivation_context(g, iris)

    (slv_platform_vel, sched1, hdl, sched2, slv_chain, sched3, slv_platform_frc, sched4) = _solver_sections(
        g, p, setups_by_node, default_setup, derivation, scene.objects
    )
    for solver in slv_chain:
        solver.kdl_header = kdl_header
    _assign_monitor_event_indexes(hdl)

    closures = p.closures(ops_generic + ops_slv + ops_cstr_hdl)
    _derive_solver_closures(g, p, derivation, closures)
    view_map = p.view()
    data_structures = p.data_structures()
    _derive_solver_data(g, p, derivation, data_structures, view_map)
    snapshot_source_map, snapshot_owner_map, snapshot_trigger_map = _snapshot_maps(g, p)
    closure_owner_map = _closure_owner_map(g, p, closures)
    data_reference_map = _data_reference_map(data_structures, closures)
    closure_output_map, closure_input_map = _closure_maps(closures)

    # Resolve declared-pose components and path goals from views/data/closures before
    # motions are built (per-motion declared poses reference them).
    pose_nodes = {
        p.id(node): node for node in g.subjects(RDF.type, URI_GEOM_TYPE_POSE_COORD)
    }
    pose_components = build_pose_components(view_map, data_structures, g, pose_nodes)
    resolve_lerp_closures(closures, pose_components)
    resolve_arc_closures(closures, data_structures)

    motions, fsm_meta = build_motion_units(
        g,
        p,
        hdl,
        node_by_id,
        slv_chain,
        snapshot_source_map=snapshot_source_map,
        view_map=view_map,
        closure_output_map=closure_output_map,
        data_reference_map=data_reference_map,
        closure_input_map=closure_input_map,
        closures=closures,
        data_structures=data_structures,
        pose_components=pose_components,
        fsm=fsm,
        derivation=derivation,
        snapshot_trigger_map=snapshot_trigger_map,
        snapshot_owner_map=snapshot_owner_map,
        closure_owner_map=closure_owner_map,
    )
    _validate_solvers(slv_chain, backend)
    _annotate_runtime_robots(slv_chain, motions, backend)
    _annotate_rne_gravity(slv_chain, motions)

    if scene.timestep_s <= 0:
        raise ValueError("ENVIRONMENT timestep must be positive.")
    control_period_ns = int(round(scene.timestep_s * 1e9))
    _apply_monitor_debounce(hdl, control_period_ns)

    # Safeguard: no two distinct URIs may collapse to one generated id (would silently merge).
    p.assert_no_id_collisions()

    shared_data = _filter_shared_data(
        data_structures,
        sched1 + sched2 + sched3 + sched4,
        closures,
        view_map=view_map,
        fk_output_ids={out.id for s in slv_chain for out in s.output},
    )
    shared_data = shared_data + _shared_runtime_members(
        slv_chain, iris, control_period_ns, platform.get("uri")
    )

    introspection, values = _build_introspection(
        app_model_path=app_model_path,
        imported_models=imported_models,
        imported_provenance=imported_provenance,
        id_nodes=id_nodes,
        node_by_id=node_by_id,
        motions=motions,
        data_structures=data_structures,
        control_period_ns=control_period_ns,
        backend=backend,
        scene=scene,
        closures=closures,
        views=view_map,
        shared_data=shared_data,
        serial_chain_solvers=slv_chain,
        platform=platform,
        iris=iris,
    )

    schedule = sched1 + sched2 + sched3 + sched4
    shared_schedule = sched1 + sched3 + sched4

    # Collect the distinct ROS publishers a monitor's `also publish to topic` needs, so codegen
    # links rclcpp/realtime_tools and sets up nodes/publishers only when publishing is present.
    _by_pub_id = {}
    for motion in motions:
        for phase in ("when_monitors", "until_monitors", "while_monitors"):
            for mon in getattr(motion, phase, []) or []:
                channel = getattr(mon, "ros_channel", None)
                if channel is None:
                    continue
                _by_pub_id.setdefault(
                    mon.ros_pub_id,
                    {
                        "pub_id": mon.ros_pub_id,
                        "channel": channel,
                        "cpp_type": mon.ros_cpp_type,
                        "include": mon.ros_include,
                        "pkg": mon.ros_pkg,
                    },
                )
    ros_publishers = list(_by_pub_id.values())
    ros_packages = sorted({p["pkg"] for p in ros_publishers})

    # An arm and a wheeled base are both actuated resources the program commands, so they ride
    # in one kind-tagged collection instead of one top-level list and one optional subsystem.
    # The by-kind views are filtered off `robots` here, in one place, because the templates
    # dispatch on kind and ST4 cannot filter.
    robots = [*slv_chain, *slv_platform_vel, *slv_platform_frc]
    resources = {
        "robots": robots,
        "by_kind": {
            "serial_chain": [r for r in robots if r.kind == "serial_chain"],
            # ST4 treats only null/absent as falsy -- an empty list is truthy -- so the base
            # rides as an object-or-absent, which is the presence guard the templates need.
            "mobile_base": (
                {"velocity_solvers": slv_platform_vel, "force_solvers": slv_platform_frc}
                if any(r.kind == "mobile_base" for r in robots)
                else None
            ),
        },
    }

    ir = {
        # Read cross-package by motion-spec-dsl's solver-derivation tests.
        "cstr_hdl": hdl,
        "motions": motions,
        "closures": closures,
        "views": _views_for_access(view_map, shared_data, motions, closures),
        "shared_data": shared_data,
        # Layer-B projections of the dataflow contract (annotate_dataflow), keyed by the model
        # role a value plays -- not by the C++ construct the view builds from it.
        "values": values,
        "has_serial_chain": bool(slv_chain),
        # Every actuated resource the program commands, plus the by-kind views built above.
        "resources": resources,
        # One key per optional subsystem, absent when the model has none. ST4 treats only
        # null/absent as falsy -- an empty list is truthy -- so an absent object is the guard
        # the templates need, and no separate has_* flag has to be kept in step with it.
        "ros": (
            {
                "publishers": ros_publishers,
                "packages": ros_packages,
                "node_name": "motion_spec_monitor",
            }
            if ros_publishers
            else None
        ),
        # Elapsed constraints compare seconds from the runtime clock (MuJoCo sim seconds /
        # real monotonic wall clock).
        "needs_clock_time": any(m.has_elapsed for m in motions),
        "control_period_ns": control_period_ns,
        "backend": backend,
        # The authored execution platform, so provenance and the runtime graph read the model's
        # own answer instead of matching substrings of a derived id.
        "platform": platform,
        "agent_homes": _agent_home_positions(platform, slv_chain),
        "scene": scene,
        "trace": _TRACE_DISABLED,
        "introspection": introspection,
        # FSM (states/events/transitions/reactions) framed from the FSM named graph that
        # motion-spec-dsl folds into the model dataset, plus the codegen wiring derived from
        # it; None when no .fsm is imported.
        "fsm": {**fsm, **fsm_meta} if fsm else None,
    }
    # ir is complete by construction — every codegen-facing field was computed while its
    # piece was built (scene / solvers / closures / motions / introspection). Codegen only
    # loads ir.json and renders; there is no post-assembly derivation pass.
    return ir


