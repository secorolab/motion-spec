# SPDX-License-Identifier: MPL-2.0
"""Intermediate representation (IR) generator for motion specification models.

This module parses RDF graphs containing motion specification models and generates
a JSON intermediate representation suitable for code generation.
"""

from __future__ import annotations

import argparse
import collections
import hashlib
import itertools
import json
import math
import re
import sys
import weakref
from dataclasses import asdict, dataclass, field, replace
from enum import Enum
from functools import wraps
from pathlib import Path
from urllib.parse import urlsplit

import rdflib
from rdf_utils.naming import get_valid_var_name
from rdf_utils.models.vocab import URI_KC_TYPE_SERIAL
from rdf_utils.namespace import NS_MM_KC_EXT, NS_MM_QUDT_QTY, NS_MM_QUDT_UNIT
from rdf_utils.resolver import IriToFileResolver, install_resolver
from rdf_utils.uri import (
    iri_is_descendant,
    iri_parent,
)
from rdflib import URIRef
from rdflib.namespace import RDF, split_uri

# fmt: off
from motion_spec.entities import (
    AccelerationConstraint, AccelerationTwist, Axis, BilateralConstraint,
    CartesianForceSpecification, Constraint, ConstraintEvaluator, ConstraintHandler,
    DataclassJSONEncoder, Direction, EdgeMonitor, EqualityConstraint,
    EvaluatorType, FeedForwardController, ForceDistributionSolver, ForwardedCommand, Frame,
    FreeVector, GuardedMotion, GuardedMotionBlock, HandlerArmSolver, ImpedanceController,
    JointForceSpecification, JointPosition, LevelMonitor, MotionDrivers, Orientation,
    OutsideConstraint, PIDController, Point, Pose, PoseAxisErrorComponent, PoseAxisErrorGroup,
    PoseDifference, Position, Provenance, Quantity, QuantityKind, RelativePoseCapture,
    Saturation, SceneAttachment, SceneObject, SceneObjectSpec, SceneRelativePose, SceneRobot,
    SceneSpec, SimplicialComplex, SnapshotCapture, SolverWithInputAndOutput, Subspace,
    Trajectory, UnilateralConstraint, UnilateralConstraintType, Unit, VelocityCompositionSolver,
    VelocityTwist, View, Wrench,
)
# fmt: on
from motion_spec.manifest import build_url_map, metamodel_url_map
from motion_spec.derive_solver import AccelerationAxis, SolverIdFactory, acceleration_axes

# fmt: off
from motion_spec.namespace import (
    AGN, ALGO_EXT, APP, CSTR, CSTR_EXT, CSTR_HDL, CSTR_HDL_EXT, ENV, EXEC, GEOM_COORD,
    GEOM_ENT, GEOM_OP, GEOM_OP_EXT, GEOM_PATH, GEOM_REL, KC, KC_STAT, MAP, MAP_EXT, MOT, QUDT_QKIND,
    QUDT_SCHEMA, RBDYN_COORD, RBDYN_ENT, RBDYN_OP, SLV, SLV_EXT,
    SENSORS, SOSA,
)
# fmt: on


# ---------------------------------------------------------------------------
# DSL operators and specifications
# ---------------------------------------------------------------------------
def _term_name(node) -> str | None:
    if node is None:
        return None
    return split_uri(str(node))[1]


def _authored_controller_axes(g) -> dict[URIRef, tuple[AccelerationAxis, ...]]:
    """Acceleration axes derived only from authored controller facts."""
    handler_controllers = set(g.objects(None, CSTR_HDL.controllers))
    result = {}
    for controller in handler_controllers:
        constraint = g.value(controller, CSTR_HDL.constraint)
        quantity = g.value(constraint, CSTR.quantity) if constraint is not None else None
        if constraint is None or quantity is None:
            continue
        view = next(g.subjects(MAP.subobject, quantity), None)
        subspace = _term_name(g.value(view, MAP.subspace)) if view is not None else None
        axis = _term_name(g.value(view, MAP.axis)) if view is not None else None
        quantity_kind = None
        target = g.value(view, MAP.superobject) if view is not None else quantity
        target_types = set(g.objects(target, RDF.type))
        if GEOM_COORD.PoseCoordinate in target_types:
            quantity_kind = "Pose"
        elif KC_STAT.JointPositionCoordinate in target_types:
            quantity_kind = "JointPosition"
        controller_types = set(g.objects(controller, RDF.type))
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
                for type_ in g.objects(constraint, RDF.type)
                if type_ != CSTR.Constraint and _term_name(type_).endswith("Constraint")
            ),
            "",
        )
        command_type = g.value(controller, APP["command-type"])
        result[controller] = acceleration_axes(
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
    axes: tuple[AccelerationAxis, ...]


@dataclass(frozen=True)
class SolverDerivationContext:
    """Immutable indexes for solver expansion, built once before IR emission."""

    controllers_by_handler: dict[URIRef, tuple[ControllerDerivation, ...]]
    controllers_by_solver: dict[URIRef, tuple[ControllerDerivation, ...]]
    shared_constraints: frozenset[URIRef]


def _solver_derivation_context(g) -> SolverDerivationContext:
    """Resolve controller ownership and command shape without generated solver nodes."""
    axes_by_controller = _authored_controller_axes(g)
    by_handler = {}
    by_solver: dict[URIRef, list[ControllerDerivation]] = collections.defaultdict(list)
    for handler in g.subjects(RDF.type, CSTR_HDL.ConstraintHandler):
        motion = g.value(handler, CSTR_HDL.motion)
        if not isinstance(motion, URIRef):
            raise ValueError(f"Constraint handler '{handler}' is missing its motion.")
        authored = []
        for controller in g.objects(handler, CSTR_HDL.controllers):
            if controller not in authored:
                authored.append(controller)
        authored.sort(key=lambda node: int(getattr(g.value(node, APP.order), "value", 0)))
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
    return SolverDerivationContext(
        controllers_by_handler=by_handler,
        controllers_by_solver={node: tuple(plans) for node, plans in by_solver.items()},
        shared_constraints=frozenset(
            constraint for constraint, count in constraint_counts.items() if count > 1
        ),
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
    types = set(g.objects(plan.controller, RDF.type))
    command_type = str(g.value(plan.controller, APP["command-type"]) or "")
    if CSTR_HDL_EXT.FeedForwardController in types:
        return f"cmd_{controller_id}"
    if CSTR_HDL.ImpedanceController in types or command_type == "Force":
        return f"force_{controller_id}"
    target = g.value(plan.view, MAP.superobject) if plan.view is not None else plan.quantity
    if command_type == "Torque" and KC_STAT.JointPositionCoordinate in g[target : RDF.type]:
        return f"tau_{controller_id}"
    quantity_id = p.id(plan.quantity)
    suffix = "" if plan.constraint in context.shared_constraints else f"_{_motion_suffix(p, plan.motion)}"
    return f"eacc_{quantity_id}{suffix}"


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
    axis: AccelerationAxis | None = None,
):
    """Build a controller dataclass from one authored controller and optional pose axis."""
    source_id = p.id(plan.controller)
    ids = SolverIdFactory(source_id, _motion_suffix(p, plan.motion))
    if axis is not None:
        controller_id = ids.component_controller(axis)
        signal = _derived_quantity(
            ids.component_energy(axis), "AccelerationEnergy", "N_M2_PER_SEC2"
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
                f"{source_id}_measured_derivative_{axis.suffix}",
                "LinearVelocity" if is_linear else "AngularVelocity",
                "M_PER_SEC" if is_linear else "RAD_PER_SEC",
                has_view=True,
            )
            if measured_source is not None
            else None
        )
        output_saturation, integral_saturation = _controller_saturations(
            g, p, plan.controller, signal
        )
    else:
        controller_id = source_id
        signal_id = _controller_signal_id(g, p, context, plan)
        types = set(g.objects(plan.controller, RDF.type))
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
        else:
            signal = _derived_quantity(signal_id, "AccelerationEnergy", "N_M2_PER_SEC2")
        error_node = g.value(plan.controller, CSTR_HDL["error-signal"])
        error = p.quantity(error_node) if error_node is not None else None
        measured_node = g.value(plan.controller, CSTR_HDL["measured-velocity"])
        measured_derivative = p.quantity(measured_node) if measured_node is not None else None
        output_saturation, integral_saturation = _controller_saturations(
            g, p, plan.controller, signal
        )

    types = set(g.objects(plan.controller, RDF.type))
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
            type=p.id(CSTR_HDL.ImpedanceController),
        )
    reference_node = g.value(plan.controller, CSTR_HDL_EXT["reference-signal"])
    return FeedForwardController(
        id=controller_id,
        control_signal=signal,
        reference_signal=p.quantity(reference_node) if reference_node is not None else None,
        output_saturation=output_saturation,
        type=p.id(CSTR_HDL_EXT.FeedForwardController),
    )


def _derived_controllers(g, p, context, plan: ControllerDerivation):
    """Expand a pose command per axis and leave scalar commands singular."""
    if len(plan.axes) > 1:
        return [_derived_controller(g, p, context, plan, axis) for axis in plan.axes]
    return [_derived_controller(g, p, context, plan)]


def _solver_limit(g, p, solver: URIRef, axis: AccelerationAxis):
    """Return the solver saturation matching a derived acceleration subspace."""
    kind = (
        QUDT_QKIND.LinearAcceleration
        if axis.subspace == "linear-acceleration"
        else QUDT_QKIND.AngularAcceleration
    )
    node = next(
        (
            limit
            for limit in g.objects(solver, ALGO_EXT.limits)
            if kind
            in g[g.value(limit, ALGO_EXT["in"]) : QUDT_SCHEMA.hasQuantityKind]
        ),
        None,
    )
    return p.saturation(node) if node is not None else None


def _derived_acceleration_constraints(g, p, context, plan: ControllerDerivation):
    """Build ordered acceleration constraints for one authored controller."""
    ids = SolverIdFactory(p.id(plan.controller), _motion_suffix(p, plan.motion))
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
        result.append(
            AccelerationConstraint(
                id=constraint_id,
                subspace=(
                    Subspace.Linear
                    if axis.subspace == "linear-acceleration"
                    else Subspace.Angular
                ),
                axis={"x": Axis.X, "y": Axis.Y, "z": Axis.Z}.get(axis.axis),
                acceleration_energy=_derived_quantity(
                    energy_id, "AccelerationEnergy", "N_M2_PER_SEC2"
                ),
                as_seen_by=frame,
                saturation=_solver_limit(g, p, plan.solver, axis),
            )
        )
    return result


def _derived_motion_drivers(g, p, context, solver: URIRef) -> list[MotionDrivers]:
    """Build a solver's drivers from authored controllers plus authored force specs."""
    acceleration = [
        constraint
        for plan in context.controllers_by_solver.get(solver, ())
        for constraint in _derived_acceleration_constraints(g, p, context, plan)
    ]
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
                p.id(driver),
                acceleration,
                cartesian,
                joint,
                has_cartesian_force=bool(cartesian),
            )
        )
    return result


def _literal_text(g, subject, predicate):
    """Return a controller parameter in the text form used by closure IR."""
    value = g.value(subject, predicate)
    if value is None:
        return None
    literal = value if isinstance(value, rdflib.Literal) else g.value(value, QUDT_SCHEMA.value)
    return str(literal) if literal is not None else None


def _derive_solver_closures(g, p, context, closures: dict) -> None:
    """Replace graph-expanded controller closures with authored semantic derivations."""
    for plans in context.controllers_by_handler.values():
        for plan in plans:
            source_id = p.id(plan.controller)
            ids = SolverIdFactory(source_id, _motion_suffix(p, plan.motion))
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
                    "measured_velocity": getattr(
                        getattr(controller, "measured_derivative", None), "id", None
                    ),
                    "control_signal": controller.control_signal.id,
                    "proportional_gain": _literal_text(
                        g, plan.controller, CSTR_HDL["proportional-gain"]
                    ),
                    "integral_gain": _literal_text(
                        g, plan.controller, CSTR_HDL["integral-gain"]
                    ),
                    "derivative_gain": _literal_text(
                        g, plan.controller, CSTR_HDL["derivative-gain"]
                    ),
                    "decay_rate": _literal_text(g, plan.controller, CSTR_HDL["decay-rate"]),
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
                    {"x": Axis.X, "y": Axis.Y, "z": Axis.Z}[axis.axis],
                )
                measured_source = g.value(plan.controller, CSTR_HDL["measured-velocity"])
                if measured_source is not None:
                    derivative = _derived_quantity(
                        f"{p.id(plan.controller)}_measured_derivative_{axis.suffix}",
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
                        {"x": Axis.X, "y": Axis.Y, "z": Axis.Z}[axis.axis],
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
    if not g[node : RDF["type"] : GEOM_PATH.Path]:
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

    def from_output_to_operator(self, g, data_out):
        """Operator calls of this type that produce data_out."""
        return [
            op
            for out in self.output
            for op in g.subjects(out, data_out)
            if g[op : RDF["type"] : self.type_]
        ]

    def scheduler_step(self, g, data_out):
        """Input data structures and schedulable calls producing data_out for this operator."""
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

    def from_output_to_operator(self, g, data_out):
        """Specification calls of this type that produce data_out."""
        return [
            op
            for out in self.output
            for op in g.subjects(out, data_out)
            if g[op : RDF["type"] : self.type_]
        ]

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

        for op in self.cstr_op:
            if op.type_ not in g[operator_id : CSTR_HDL["constraint"] / RDF["type"]]:
                continue

            for in_ in op.input:
                for data_in in g.objects(operator_id, CSTR_HDL["constraint"] / in_):
                    data_structures.add(data_in)

        return data_structures

    def from_output_to_operator(self, g, data_out):
        """Evaluator calls that produce the given error data_out."""
        outputs = {out for operator in self.cstr_op for out in operator.output}
        return [
            op
            for out in outputs
            for op in g.subjects(out, data_out)
            if g[op : RDF["type"] : self.type_]
        ]

    def scheduler_step(self, g, data_out):
        """Input data structures and schedulable calls producing the error data_out."""
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
        """Data-structure nodes feeding the assignment's inputs."""
        if self.cstr_op.type_ not in g[operator_id : CSTR_HDL["constraint"] / RDF["type"]]:
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
        input=[GEOM_PATH["anchor"], GEOM_PATH["radius"], GEOM_PATH["plane-normal"]],
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
        type_=GEOM_OP_EXT.PathEvaluator,
        input=[GEOM_OP_EXT.path, GEOM_OP_EXT["path-parameter"]],
        output=[GEOM_OP["out"]],
        parameters=[GEOM_OP_EXT["easing"]],
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


class LocalIdMap:
    """Stable generated ids for RDF nodes: local name when unique, scoped name on collision."""

    def __init__(self, graph, nodes):
        """Build the id map over the given nodes, scoping names that collide across models."""
        self.graph = graph
        grouped: dict[str, list] = {}
        for node in sorted({n for n in nodes if n is not None}, key=str):
            grouped.setdefault(get_valid_var_name(graph.compute_qname(node)[2]), []).append(node)
        self.ids = {}
        for base, members in grouped.items():
            if len(members) == 1:
                self.ids[members[0]] = base
                continue
            used = set()
            for node in members:
                scoped = self._scoped_id(node, base)
                if scoped in used:
                    scoped = f"{scoped}_{hashlib.sha1(str(node).encode()).hexdigest()[:8]}"
                used.add(scoped)
                self.ids[node] = scoped

    def _scoped_id(self, node, base: str) -> str:
        """Local id for a node, prefixed with its model scope when the bare name collides."""
        try:
            prefix, _namespace, local = self.graph.compute_qname(node)
        except Exception:
            prefix, local = "", base
        if prefix:
            return get_valid_var_name(f"{prefix}_{local}")
        return f"{base}_{hashlib.sha1(str(node).encode()).hexdigest()[:8]}"

    def __getitem__(self, node) -> str:
        """Local id for a node (empty string for None)."""
        if node not in self.ids:
            self.ids[node] = get_valid_var_name(self.graph.compute_qname(node)[2])
        return self.ids[node]


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
        if type_ not in self.g[id_ : RDF["type"]]:
            raise ValueError(f"node {id_} is missing expected rdf:type {type_}")

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
        for index in range(1, len(parts) - 2):
            if parts[index : index + 2] in (("Spec", "spec"), ("World", "world")):
                return parts[index - 1], parts[index], parts[index + 2 :]
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
        # Only context quantities (a motion's or the shared context's `Spec/spec` / `World/world`
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
                if type_ in self.g[o : RDF["type"]]:
                    out.append(func(o))

        # Authored gravity-value IS the solver root acceleration (KDL Vereshchagin
        # root_acc.vel) — taken as truth, no sign flip. If a MuJoCo env ever needs a
        # different world gravity than the solver, hardcode it there with a TODO;
        # do not re-derive it from this by negation.
        gravity_node = self.g.value(id_, SLV.gravity)
        root_acc = self.parse_xyz(gravity_node) if gravity_node else None
        algorithm_node = self.g.value(id_, SLV["solver"])
        algorithm = {
            SLV["AccelerationConstrainedHybridDynamicsAlgorithm"]: "ACHD",
            SLV["RecursiveNewtonEulerAlgorithm"]: "RNE",
        }.get(algorithm_node, self.id(algorithm_node) if algorithm_node else "")
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
            algorithm_is_rne=algorithm == "RNE",
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
            self.id(id_), [], spec_frc, spec_jf, has_cartesian_force=bool(spec_frc)
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
            SLV["x"]: Axis.X,
            SLV["y"]: Axis.Y,
            SLV["z"]: Axis.Z,
        }
        if id_ not in d:
            raise ValueError(f"unknown axis {id_}")

        return d[id_]

    @memoize
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
                group_any = CSTR_EXT.ConstraintDisjunction in self.g[group_node : RDF.type]
        error_node = self.g.value(id_, CSTR_HDL["error"])
        error = (
            None
            if is_until_aggregate or is_when_aggregate or group_constraint_ids or error_node is None
            else self.quantity(error_node)
        )

        if CSTR_HDL["LevelTriggeredMonitor"] in self.g[id_ : RDF["type"]]:
            flag = self.id(self.g.value(id_, CSTR_HDL["flag"]))
            return LevelMonitor(
                self.id(id_),
                "LevelTriggeredMonitor",
                error,
                flag,
                is_until_aggregate=is_until_aggregate,
                is_when_aggregate=is_when_aggregate,
                group_constraint_ids=group_constraint_ids,
                group_any=group_any,
            )

        event_node = self.g.value(id_, CSTR_HDL["event"])
        event = self.id(event_node)
        fallback_node = self.g.value(id_, CSTR_HDL_EXT["fallback-motion"])
        fallback_motion = self.id(fallback_node) if fallback_node is not None else None
        debounce_duration_s = self._optional_float(id_, CSTR_HDL_EXT["debounce-duration"])
        return EdgeMonitor(
            self.id(id_),
            "EdgeTriggeredMonitor",
            error,
            event,
            None,
            is_until_aggregate=is_until_aggregate,
            is_when_aggregate=is_when_aggregate,
            group_constraint_ids=group_constraint_ids,
            group_any=group_any,
            event_uri=str(event_node),
            event_name=event.upper(),
            fallback_motion=fallback_motion,
            debounce_duration_s=debounce_duration_s,
        )

    @memoize
    def constraint_evaluator(self, id_):
        """Parse a ConstraintEvaluator (constraint, error, elapsed timing) at node."""
        self._expect_type(id_, CSTR_HDL["ConstraintEvaluator"])
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
        if (
            qnode is not None
            and NS_MM_QUDT_QTY["Time"] in self.g[qnode : QUDT_SCHEMA.hasQuantityKind]
        ):
            is_elapsed = True
            elapsed_op = (
                ">="
                if CSTR["GreaterThanConstraint"] in self.g[constraint_node : RDF["type"]]
                else "<"
            )
            thr = self.g.value(constraint_node, CSTR["threshold"])
            thr_val = float(self.g.value(thr, QUDT_SCHEMA["value"]))
            thr_unit = self.g.value(thr, QUDT_SCHEMA["unit"])
            elapsed_threshold_s = thr_val * (
                0.001 if thr_unit == NS_MM_QUDT_UNIT["MilliSEC"] else 1.0
            )

        return ConstraintEvaluator(
            self.id(id_),
            t,
            constraint,
            error,
            is_elapsed=is_elapsed,
            elapsed_op=elapsed_op,
            elapsed_threshold_s=elapsed_threshold_s,
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
            if _is_constraint_aggregate(self.g, c):
                # A section-wide disjunction makes the whole until 'any'; a named group keeps
                # its own logic on the monitor that targets it.
                if CSTR_EXT.ConstraintDisjunction in self.g[c : RDF["type"]]:
                    until_any = True
                for member in self.g[c : CSTR_EXT["has-constraint"]]:
                    until.append(self.constraint(member))
            else:
                until.append(self.constraint(c))

        return GuardedMotion(self.id(id_), when, while_, until, until_any, when_any)

    @memoize
    def constraint(self, id_):
        """Parse a Constraint (quantity plus its parameter) at node."""
        self._expect_type(id_, CSTR["Constraint"])
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
        if CSTR["GreaterThanConstraint"] in self.g[id_ : RDF["type"]]:
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

    def parse_vector3(self, node):
        """Parse a 3-vector coordinate list from node, or None."""
        from rdflib import collection

        items = list(collection.Collection(self.g, node))
        if len(items) != 3:
            return None

        # Get the Python representation of the associated RDF literal
        return [float(v.toPython()) for v in items]

    def parse_xyz(self, node):
        """Parse x/y/z scalar coordinates from node, or None."""
        x = self.g.value(node, GEOM_COORD["x"])
        y = self.g.value(node, GEOM_COORD["y"])
        z = self.g.value(node, GEOM_COORD["z"])

        if x is None or y is None or z is None:
            return None

        return [float(v.value) for v in (x, y, z)]

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
        self._expect_type(id_, GEOM_COORD["PositionCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        of_node = self.g.value(id_, GEOM_REL["of"])
        wrt_node = self.g.value(id_, GEOM_REL["with-respect-to"])
        as_seen_by_node = self.g.value(id_, GEOM_COORD["as-seen-by"])
        if of_node is None or wrt_node is None or as_seen_by_node is None:
            inherited_of, inherited_wrt, inherited_as_seen_by = self._derived_reference_frames(id_)
            of_node = of_node or inherited_of
            wrt_node = wrt_node or inherited_wrt
            as_seen_by_node = as_seen_by_node or inherited_as_seen_by
        of = self.position_reference(of_node)
        wrt = self.position_reference(wrt_node)
        quantity_kind = self.id(self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]))
        as_seen_by = self.frame(as_seen_by_node)
        unit = self.id(self.g.value(id_, QUDT_SCHEMA["unit"]))
        pos = self.parse_xyz(id_)

        return Position(
            self.id(id_), of, wrt, QuantityKind(quantity_kind), as_seen_by, Unit(unit), pos
        )

    @memoize
    def orientation(self, id_):
        """Parse an Orientation quantity at node."""
        self._expect_type(id_, GEOM_COORD["OrientationCoordinate"])

        def optional_pose_ref(node):
            """Resolve an optional pose reference (endpoint or bare pose) at node."""
            if node is None:
                return None
            if ENV.RigidObject in self.g[node : RDF["type"]]:
                return self.scene_object(node)
            if GEOM_ENT.Frame in self.g[node : RDF["type"]]:
                return self.frame(node)
            return None

        of = optional_pose_ref(self.g.value(id_, GEOM_REL["of"]))
        wrt = optional_pose_ref(self.g.value(id_, GEOM_REL["with-respect-to"]))
        quantity_kind = self.id(self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]))
        as_seen_by_node = self.g.value(id_, GEOM_COORD["as-seen-by"])
        as_seen_by = self.frame(as_seen_by_node) if as_seen_by_node is not None else None
        unit = self.id(self.g.value(id_, QUDT_SCHEMA["unit"]))
        axes = self.g.value(id_, GEOM_COORD["axes-sequence"])
        provenance = self.quantity_provenance(id_)
        return Orientation(
            self.id(id_),
            of,
            wrt,
            QuantityKind(quantity_kind),
            as_seen_by,
            Unit(unit),
            str(axes) if axes is not None else None,
            (id_, ~MAP["subobject"], None) in self.g,
            provenance=provenance,
        )

    def position_reference(self, id_):
        """A Position is of a Point with respect to a Point (geometry metamodel)."""
        if id_ is None:
            return None
        if GEOM_ENT.Point in self.g[id_ : RDF["type"]]:
            return self.point(id_)
        if GEOM_ENT.Frame in self.g[id_ : RDF["type"]]:
            return Point(self.id(id_))
        raise ValueError(f"Position reference must be a Point, got: {id_}")

    @memoize
    def _pose_endpoint(self, node):
        """Resolve a pose endpoint (frame/scene-object) to its id."""
        if node is None:
            return None
        if ENV.RigidObject in self.g[node : RDF["type"]]:
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
        self._expect_type(id_, GEOM_COORD["PoseCoordinate"])
        of_node = self.g.value(id_, GEOM_REL["of"])
        wrt_node = self.g.value(id_, GEOM_REL["with-respect-to"])
        as_seen_by_node = self.g.value(id_, GEOM_COORD["as-seen-by"])
        if of_node is None or wrt_node is None or as_seen_by_node is None:
            inherited_of, inherited_wrt, inherited_as_seen_by = self._derived_reference_frames(id_)
            of_node = of_node or inherited_of
            wrt_node = wrt_node or inherited_wrt
            as_seen_by_node = as_seen_by_node or inherited_as_seen_by
        of = self._pose_endpoint(of_node)
        wrt = self._pose_endpoint(wrt_node)
        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.id(k))
        as_seen_by = self.frame(as_seen_by_node) if as_seen_by_node is not None else None
        unit = []
        for u in self.g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.id(u))
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

        provenance = self.quantity_provenance(id_)
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
            provenance=provenance,
        )

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

    def _spatial_coordinate_fields(self, id_, ref_point_pred, as_seen_by_pred):
        """Shared field extraction for the 6D coordinate quantities.

        AccelerationTwist, PoseDifference and Wrench are distinct concepts with an
        identical coordinate structure; only the RDF predicates differ (geometry vs
        rigid-body-dynamics namespaces).
        """
        quantity_kind = [self.id(k) for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]]
        reference_point = self.point(self.g.value(id_, ref_point_pred))
        as_seen_by = self.frame(self.g.value(id_, as_seen_by_pred))
        unit = [self.id(u) for u in self.g[id_ : QUDT_SCHEMA["unit"]]]
        provenance = self.quantity_provenance(id_)
        return quantity_kind, reference_point, as_seen_by, unit, provenance

    @memoize
    def acceleration_twist(self, id_):
        """Parse an AccelerationTwist quantity at node."""
        self._expect_type(id_, GEOM_COORD["AccelerationTwistCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        qk, ref, seen, unit, provenance = self._spatial_coordinate_fields(
            id_, GEOM_REL["reference-point"], GEOM_COORD["as-seen-by"]
        )
        return AccelerationTwist(self.id(id_), qk, ref, seen, unit, provenance=provenance)

    @memoize
    def pose_difference(self, id_):
        """Parse a PoseDifference quantity at node."""
        self._expect_type(id_, GEOM_COORD["PoseDifferenceCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        qk, ref, seen, unit, provenance = self._spatial_coordinate_fields(
            id_, GEOM_REL["reference-point"], GEOM_COORD["as-seen-by"]
        )
        return PoseDifference(self.id(id_), qk, ref, seen, unit, provenance=provenance)

    @memoize
    def wrench(self, id_):
        """Parse a Wrench quantity (with any FT sensor) at node."""
        self._expect_type(id_, RBDYN_COORD["WrenchCoordinate"])
        qk, ref, seen, unit, provenance = self._spatial_coordinate_fields(
            id_, RBDYN_ENT["reference-point"], RBDYN_COORD["as-seen-by"]
        )
        sensor = self.g.value(id_, SOSA.madeBySensor)
        sensor_name = self.id(sensor) if sensor is not None else ""
        return Wrench(
            self.id(id_), qk, ref, seen, unit, provenance=provenance, sensor_name=sensor_name
        )

    @memoize
    def quantity(self, id_):
        """Parse the quantity at node, dispatching on its RDF type."""
        self._expect_type(id_, QUDT_SCHEMA["Quantity"])
        quantity_kind_node = self.g.value(id_, QUDT_SCHEMA.hasQuantityKind)
        quantity_kind = self.id(quantity_kind_node)

        unit = self.id(self.g.value(id_, QUDT_SCHEMA["unit"]))
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
        if CSTR_HDL_EXT["SetpointGenerator"] in self.g[id_ : RDF["type"]]:
            value_kind_node = next(
                (k for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]] if k != CSTR_HDL_EXT.SetpointGenerator),
                None,
            )
            provenance = self.quantity_provenance(id_)
            return Trajectory(
                self.id(id_),
                QuantityKind(quantity_kind),
                Unit(unit),
                has_view,
                provenance=provenance,
                value_kind=self.id(value_kind_node) if value_kind_node is not None else None,
            )

        if quantity_kind == "FreeVector" and GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]]:
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
            value = float(self.g.value(id_, QUDT_SCHEMA["value"]))
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
    def joint_position(self, id_):
        """Parse a JointPosition quantity at node."""
        self._expect_type(id_, KC_STAT["JointPositionCoordinate"])
        joint_node = self.g.value(id_, KC_STAT["of-joint"])
        joint_name = self.label(joint_node) if joint_node is not None else ""
        return JointPosition(self.id(id_), joint_name)

    def quantity_provenance(self, id_):
        # Provenance(authored, snapshot), mutually exclusive: snapshot wins (mirrors old roles() elif).
        # authored == carries an authored value/coordinate and is not a runtime snapshot.
        """Parse a quantity's Provenance (authored / snapshot) at node."""
        snapshot = ALGO_EXT.Snapshot in self.g[id_ : RDF["type"]]
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
            (id_, GEOM_COORD[c], None) in self.g
            for c in (
                "x",
                "y",
                "z",
                "direction-cosine-x",
                "direction-cosine-y",
                "direction-cosine-z",
            )
        ):
            return True
        if any(True for _ in self.g.objects(id_, GEOM_COORD["has-coordinate"])):
            return True
        return False

    @memoize
    def simplicial_complex(self, id_):
        """Parse a SimplicialComplex, mapping a body-origin frame to its runtime body."""
        if not any(
            type_ in self.g[id_ : RDF.type]
            for type_ in (GEOM_ENT.SimplicialComplex, GEOM_ENT.Frame)
        ):
            raise ValueError(f"Expected a rigid body or frame, got: {id_}")
        body = next(
            (
                owner
                for owner in self.g.subjects(GEOM_ENT.simplices, id_)
                if GEOM_ENT.RigidBody in self.g[owner : RDF.type]
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
                if GEOM_ENT.RigidBody in self.g[owner : RDF.type]
            ),
            None,
        )
        if body is not None and self.id(id_) == f"{self.id(body)}_origin":
            return Frame(self.id(body))
        return Frame(self.id(id_))

    @memoize
    def point(self, id_):
        """Parse a Point at node."""
        if not any(type_ in self.g[id_ : RDF.type] for type_ in (GEOM_ENT.Point, GEOM_ENT.Frame)):
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
        for view in self.g[: RDF["type"] : MAP["View"]]:
            superobject = None
            for type_, func in dispatcher:
                if type_ not in self.g[view : RDF["type"]]:
                    continue

                superobject_id = self.g.value(view, MAP["superobject"])
                superobject = func(superobject_id)
                break

            if superobject is None:
                superobject_id = self.g.value(view, MAP["superobject"])
                superobject = self.quantity(superobject_id)

            subobject = self.quantity(self.g.value(view, MAP["subobject"]))
            subspace = self.subspace(self.g.value(view, MAP["subspace"]))
            axis_node = self.g.value(view, MAP["axis"])
            axis = self.axis(axis_node) if axis_node is not None else None

            if superobject is None:
                raise ValueError(
                    f"MAP view {view} has an unrecognized type; no view dispatcher matched"
                )
            view_map[self.id(subobject.id)] = View(
                self.id(view), superobject, subobject, subspace, axis
            )

        return view_map

    def data_structures(self):
        """Parse every data-structure entity in the graph."""
        dispatcher = [
            (GEOM_COORD["DirectionCoordinate"], self.direction),
            (GEOM_COORD["PositionCoordinate"], self.position),
            (GEOM_COORD["OrientationCoordinate"], self.orientation),
            (GEOM_COORD["PoseCoordinate"], self.pose),
            (GEOM_COORD["VelocityTwistCoordinate"], self.velocity_twist),
            (GEOM_COORD["AccelerationTwistCoordinate"], self.acceleration_twist),
            (GEOM_COORD["PoseDifferenceCoordinate"], self.pose_difference),
            (RBDYN_COORD["WrenchCoordinate"], self.wrench),
            (QUDT_SCHEMA["Quantity"], self.quantity),
        ]

        data_structures = []
        for type_, func in dispatcher:
            for dstruct in self.g[: RDF["type"] : type_]:
                data_structures.append(func(dstruct))

        return _dedupe_by_id(data_structures)

    def _path_fields(self, path_node):
        """The path's geometry, as fields of the evaluator call that traverses it.

        Traversal is one computation: the shape decides the maths, so the closure takes the
        path's type and carries its parameters directly.
        """
        spec = next((s for s in ops_path if s.type_ in self.g[path_node : RDF["type"]]), None)
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
                    if operator.type_ == GEOM_OP_EXT.PathEvaluator:
                        # The evaluator's out port is the pose setpoint the motion tracks.
                        reference = self.g.value(closure, GEOM_OP.out)
                        if reference is not None:
                            cl["trajectory"] = self.id(reference)
                        cl.update(self._path_fields(self.g.value(closure, GEOM_OP_EXT.path)))
                    if operator.type_ in {ALGO_EXT.VelocityProfile, ALGO_EXT.Admittance}:
                        reference = self.g.value(closure, ALGO_EXT.out)
                        constraint = next(
                            self.g.subjects(CSTR["reference-value"], reference), None
                        )
                        # Evaluators carry cstr-hdl:constraint too, and can win this lookup;
                        # the filter state this closure steps lives on the controller, so
                        # skip them.
                        controller = next(
                            (
                                node
                                for node in self.g.subjects(CSTR_HDL.constraint, constraint)
                                if CSTR_HDL.ConstraintEvaluator not in self.g[node : RDF["type"]]
                            ),
                            None,
                        )
                        cl["controller"] = self.id(controller)
                        if operator.type_ == ALGO_EXT.VelocityProfile:
                            # The profile starts from the constraint's own quantity and
                            # drives it to the target.
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
            v_types = set(self.g[v : RDF["type"]])
            for op in ops:
                if op.type_ not in v_types:
                    continue

                call = self.id(v)
                if op.schedulable and call not in self.sched:
                    sched.append(call)
                    scheduled_nodes[call] = v
                    self.sched.add(call)

                for data_in in op.from_operator_to_input(self.g, v):
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
            node_types = set(self.g[node : RDF["type"]])
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


def _arm_solvers_for_handler(handler, slv_arm, solver_ids):
    """Select the arm solvers explicitly referenced by a handler's controllers."""
    result = []
    motion_driver_id = f"driver_{handler.motion.id.removeprefix('motion_')}"
    for solver in slv_arm:
        if solver.id not in solver_ids:
            continue
        matched = list(solver.motion_drivers)
        if not matched:
            continue
        selected = next((driver for driver in matched if driver.id == motion_driver_id), matched[0])
        result.append(
            HandlerArmSolver(
                id=solver.id,
                output=solver.output,
                motion_driver=selected,
                algorithm=solver.algorithm,
                algorithm_is_rne=solver.algorithm_is_rne,
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

    def object_id(value):
        """Id of a value, whether a dict row or an object."""
        if isinstance(value, dict):
            return value.get("id")
        return getattr(value, "id", None)

    def object_field(value, field):
        """Read a field from a value, whether a dict row or an object."""
        if isinstance(value, dict):
            return value.get(field)
        return getattr(value, field, None)

    subobjects_by_super: dict[str, list[str]] = {}
    supers_by_subobject: dict[str, list[str]] = {}
    for view in view_map.values():
        super_id = object_id(object_field(view, "superobject"))
        subobject_id = object_id(object_field(view, "subobject"))
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
        source_closure_id = None if source_id in view_map else closure_output_map.get(source_id)
        result.append(
            SnapshotCapture(
                target_id=target_id,
                source_id=source_id,
                source_closure_id=source_closure_id,
                trigger_event=snapshot_trigger_map.get((motion_token, target_id)),
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
    "Addition": "out",
    "RotateWrenchToDistalWithPose": "to",
    "RotateWrenchToProximalWithPose": "to",
    "TransformWrenchToProximal": "to",
    "WrenchFromPositionDirectionAndMagnitude": "wrench",
    "VelocityProfile": "out",
    "Admittance": "out",
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
    for eval_node in eval_nodes:
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
    slv_arm,
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
    primary_robot_id = next((s.id for s in slv_arm if getattr(s, "id", "")), "")

    for handler in handlers:
        handler_node = node_by_id[handler.id]
        motion_node = g.value(handler_node, CSTR_HDL["motion"])
        motion = handler.motion
        handler_plans = derivation.controllers_by_handler.get(handler_node, ())
        handler_solver_ids = {p.id(plan.solver) for plan in handler_plans}

        # Classify constraints by motion phase via RDF traversal
        _raw_when = set(g[motion_node : MOT["when"]])
        when_constraint_nodes = set()
        for node in _raw_when:
            if _is_constraint_aggregate(g, node):
                when_constraint_nodes.update(g[node : CSTR_EXT["has-constraint"]])
            else:
                when_constraint_nodes.add(node)
        while_constraint_nodes = set(g[motion_node : MOT["while"]])
        _raw_until = set(g[motion_node : MOT["until"]])
        until_constraint_nodes = set()
        for node in _raw_until:
            if _is_constraint_aggregate(g, node):
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
            return (
                qnode is not None
                and NS_MM_QUDT_QTY["Time"] in g[qnode : QUDT_SCHEMA.hasQuantityKind]
            )

        # Classify controllers by the constraints active during the motion body.
        while_error_nodes = set()
        for n in while_eval_nodes:
            error_node = g.value(n, CSTR_HDL["error"])
            if error_node is not None:
                while_error_nodes.add(error_node)
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

        handler_arm_solvers = _arm_solvers_for_handler(
            handler,
            slv_arm,
            handler_solver_ids,
        )
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
            if plan is not None and CSTR_HDL_EXT.FeedForwardController not in g[
                plan.controller : RDF.type
            ]:
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
            agent = g.value(plan.solver, AGN["of-agent"])
            arm_solver_ids = {
                p.id(node)
                for node in g.subjects(AGN["of-agent"], agent)
                if SLV.SolverWithInputAndOutput in g[node : RDF.type]
            }
            arm_solver = next(
                solver
                for solver in handler_arm_solvers
                if solver.id in arm_solver_ids
            )
            runtime_solver = next(solver for solver in slv_arm if solver.id == arm_solver.id)
            forwarded_commands.append(
                ForwardedCommand(
                    f"cmd-fwd-{p.id(plan.controller)}",
                    controller.control_signal,
                    f"{runtime_solver.runtime_prefix}{p.label(target)}"
                    if target is not None
                    else "",
                    arm_solver.id,
                )
            )
        when_monitors = [p.monitor_entry(n) for n in when_mon_nodes]
        while_monitors = [p.monitor_entry(n) for n in while_mon_nodes]
        until_monitors = [p.monitor_entry(n) for n in until_mon_nodes]

        has_when_elapsed = any(getattr(e, "is_elapsed", False) for e in when_evaluators)
        has_active_elapsed = any(
            getattr(e, "is_elapsed", False) for e in while_evaluators + until_evaluators
        )
        has_elapsed = has_when_elapsed or has_active_elapsed

        motions.append(
            GuardedMotionBlock(
                id=handler.motion.id,
                handler=handler.id,
                command_robot_id=primary_robot_id,
                has_when_elapsed=has_when_elapsed,
                has_active_elapsed=has_active_elapsed,
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
                arm_solvers=handler_arm_solvers,
                relative_poses=_relative_poses_for_motion(
                    while_evaluators + when_evaluators + until_evaluators,
                    view_map,
                    _arm_solvers_for_handler(
                        handler, slv_arm, handler_solver_ids
                    ),
                ),
                scene_relative_poses=_scene_relative_poses_for_motion(
                    view_map,
                    handler_arm_solvers,
                    data_structures,
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
    data_by_id = _index_by_id(data_structures or [])
    for motion in ordered:
        _set_motion_conditions(motion)
        _add_group_type_flags(motion.pose_axis_error_groups)
        _set_motion_trajectory_progress(motion, closures or {}, data_by_id)
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
    _apply_fsm_gate_calls(ordered, fsm_meta["fsm_namespace"])
    return ordered, fsm_meta


# ---------------------------------------------------------------------------
# Scene, geometry and robot setups
# ---------------------------------------------------------------------------
def _filter_shared_data(data_structures, schedule, closures, view_map=None, fk_output_ids=None):
    """Select the data structures that become shared_data fields (scheduled/closure/FK outputs),
    excluding view sub-objects.
    """
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


def _xyz_or_none(g, node):
    """Read a coordinate node's x/y/z as floats, or None if any axis is missing."""
    if node is None:
        return None
    values = [g.value(node, GEOM_COORD[axis]) for axis in ("x", "y", "z")]
    if any(v is None for v in values):
        return None
    return [float(v.value) for v in values]


def _orientation_degrees(g, node):
    """Read an orientation node's roll/pitch/yaw in degrees, or None if incomplete."""
    if node is None:
        return None
    values = [g.value(node, GEOM_COORD[axis]) for axis in ("alpha", "beta", "gamma")]
    if any(value is None for value in values):
        return None
    result = [float(value) for value in values]
    if str(g.value(node, QUDT_SCHEMA.unit) or "").endswith("RAD"):
        result = [math.degrees(value) for value in result]
    return result


def _frames_of(g, node):
    if GEOM_ENT.Frame in g[node : RDF.type]:
        return [node]
    return [frame for frame in g.objects(node, GEOM_ENT.simplices) if GEOM_ENT.Frame in g[frame : RDF.type]]


def _position_of(g, node):
    """Position of a body or frame from its authored scene-dsl pose, or None."""
    for frame in _frames_of(g, node):
        origin = g.value(frame, GEOM_ENT.origin) or frame
        for position in g.subjects(GEOM_REL.of, origin):
            coordinate = next(g.subjects(GEOM_COORD["of-position"], position), None)
            if coordinate is not None:
                return _xyz_or_none(g, coordinate)
    return None


def _orientation_of(g, node):
    """Orientation of a body or frame from its authored scene-dsl pose, or None."""
    for frame in _frames_of(g, node):
        for orientation in g.subjects(GEOM_REL.of, frame):
            coordinate = next(g.subjects(GEOM_COORD["of-orientation"], orientation), None)
            if coordinate is not None:
                return _orientation_degrees(g, coordinate)
    return None


def _path_of_model(g, model_node):
    """Asset path (exec:path) of a model node, or the empty string."""
    if not model_node:
        return ""
    return str(g.value(model_node, EXEC.path) or "")


def _model_mappings(g, model, target_type):
    """Return (scene target, model entity) mappings of the requested RDF type."""
    mappings = [
        (target, str(g.value(mapping, EXEC["model-entity"]) or ""))
        for mapping in sorted(g.objects(model, EXEC["has-mapping"]), key=str)
        if (target := g.value(mapping, EXEC.maps)) is not None
        and target_type in g[target : RDF.type]
    ]
    if mappings:
        return mappings
    legacy_predicate = {
        GEOM_ENT.KinematicTree: EXEC["has-kinematic-tree"],
        GEOM_ENT.RigidBody: EXEC["has-body"],
    }.get(target_type)
    if legacy_predicate is None:
        return []
    return [
        (target, str(g.value(model, EXEC["model-entity"]) or ""))
        for target in sorted(g.objects(model, legacy_predicate), key=str)
        if target_type in g[target : RDF.type]
    ]


def _mapped_targets(g, model_type, target_type):
    """All scene targets of a given type mapped by models of model_type."""
    return {
        target
        for model in g.subjects(RDF.type, model_type)
        for target, _entity in _model_mappings(g, model, target_type)
    }


def _trace_from_graph(g):
    """Read the optional MuJoCo trajectory-trace overlay config from the graph.

    The trace is a viewer-only overlay (a polyline of recent EE positions). When
    no TRACE block is declared it stays disabled, so headless runs and non-MuJoCo
    runtimes cost nothing. Defaults match the wrapper's built-in warm orange.
    """
    # The MuJoCo trajectory trace is a viewer-only overlay authored in the scene, not
    # the motion spec; it is no longer part of this graph, so it stays disabled and
    # costs nothing for headless / non-MuJoCo runtimes. Codegen guards on trace.enabled.
    return {
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


def _body_of(frame):
    return iri_parent(frame)


def _tree_owns(tree, node):
    return iri_is_descendant(tree, node)


def _kinematic_adjacency(g):
    adjacency = collections.defaultdict(list)
    fixed = []
    for joint in g.subjects(RDF.type, KC.Joint):
        frames = list(g.objects(joint, KC["between-attachments"]))
        if len(frames) != 2:
            continue
        body_a, body_b = map(_body_of, frames)
        if body_a == body_b:
            continue
        adjacency[body_a].append((body_b, frames[0], frames[1], joint))
        adjacency[body_b].append((body_a, frames[1], frames[0], joint))
        if set(g.objects(joint, RDF.type)) == {KC.Joint}:
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
        _distances(adjacency, _body_of(tip))
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
            if from_root.get(_body_of(frame_a), 1 << 30)
            <= from_root.get(_body_of(frame_b), 1 << 30)
            else (frame_b, frame_a)
        )
        parent_body, child_body = map(_body_of, (parent_frame, child_frame))
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
    types = set(g[node : RDF["type"]])
    return bool({CSTR_EXT.ConstraintDisjunction, CSTR_EXT.ConstraintConjunction} & types)


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
    result = []
    for modelled in sorted(g.subjects(RDF.type, AGN.ModelledAgent), key=str):
        agent = g.value(modelled, AGN["of-agent"])
        bindings = []
        for model in sorted(g.objects(modelled, AGN["has-agent-model"]), key=str):
            path = _path_of_model(g, model)
            for tree, entity in _model_mappings(g, model, GEOM_ENT.KinematicTree):
                if not path:
                    continue
                bindings.append(
                    {
                        "model": model,
                        "tree": tree,
                        "path": path,
                        "entity": entity,
                    }
                )
        if agent is None or not bindings:
            continue

        def binding_for(node):
            return next((binding for binding in bindings if _tree_owns(binding["tree"], node)), None)

        serial = next(
            (
                (tree, root, tip)
                for tree, root, tip in serials
                if root is not None
                and tip is not None
                and (
                    any(binding["tree"] == tree for binding in bindings)
                    or (binding_for(root) is not None and binding_for(tip) is not None)
                )
            ),
            None,
        )
        if serial is None:
            continue
        serial_tree, root_frame, tip_frame = serial
        root_binding = binding_for(root_frame) or next(
            binding for binding in bindings if binding["tree"] == serial_tree
        )
        tip_binding = binding_for(tip_frame) or root_binding
        root_body, tip_body = map(_body_of, (root_frame, tip_frame))
        duplicate_root = sum(
            _leaf(root_body) in names for names in body_names_by_tree.values()
        ) > 1
        runtime_prefix = f"{_leaf(root_binding['tree'])}_" if duplicate_root else ""
        runtime_root = f"{runtime_prefix}{_leaf(root_body)}"
        path = _body_path(adjacency, root_body, tip_body)
        chain_tip_body = tip_body if root_binding["tree"] == serial_tree else root_body
        if root_binding["tree"] != serial_tree:
            for parent_body, child_body, *_ in path:
                if _tree_owns(root_binding["tree"], child_body):
                    chain_tip_body = child_body
                elif _tree_owns(root_binding["tree"], parent_body):
                    chain_tip_body = parent_body
                    break

        attachments = []
        for binding in bindings:
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
        ft_sensors = [
            {
                "name": f"{runtime_prefix}{_leaf(sensor)}",
                "frame_site": f"{runtime_prefix}{_leaf(frame)}",
            }
            for sensor in sorted(g.objects(modelled, SOSA.hosts), key=str)
            if SENSORS.ForceTorqueSensor in g[sensor : RDF["type"]]
            and (frame := g.value(sensor, SENSORS.frame)) is not None
        ]
        result.append(
            {
                "agent": agent,
                "ft_sensors": ft_sensors,
                "path": root_binding["path"],
                "prefix": runtime_prefix,
                "trees": [binding["tree"] for binding in bindings],
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
        unit = g.value(timestep, QUDT_SCHEMA.unit)
        if value is not None:
            scale = 0.001 if unit == NS_MM_QUDT_UNIT["MilliSEC"] else 1.0
            scene.timestep_s = float(value.toPython()) * scale

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
                    and GEOM_ENT.Frame in g[reference : RDF.type]
                    and _body_of(reference) == parent_body
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
                if _path_of_model(g, model)
            ),
            None,
        )
        if obj is None or mapped is None:
            continue
        model, body = mapped
        path = _path_of_model(g, model)
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
                euler=_orientation_of(g, placement_frame),
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
                euler=_orientation_of(g, assembly["placement_frame"]),
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
        expand_vector_fields(robot, "euler")
        for attachment in robot.attachments:
            expand_vector_fields(attachment, "pos")
            expand_vector_fields(attachment, "euler")
    for obj in scene.objects:
        expand_vector_fields(obj, "pos")
        expand_vector_fields(obj, "euler")
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


def _robot_setups_from_graph(g):
    """Per-robot solver chain setups, sourced from the scene-dsl (`.scenex`) graph.

    Returns ``(setups_by_node, ordered)`` where ``setups_by_node`` maps each robot's
    abstract agent node (the target of a solver's ``agn:of-agent``) to its setup tuple
    ``(urdf, chain_root, chain_end, chain_tip, robot_model, tool_body, tcp_site,
    ft_sensors, runtime_prefix, owned_trees)``.

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

    setups_by_node, ordered = {}, []
    bound_trees = _mapped_targets(g, AGN["AgentModel"], GEOM_ENT.KinematicTree)
    attach_by_body, _root = _fixed_attachments(g, bound_trees)
    for assembly in _agent_assemblies(g, attach_by_body):
        setup = (
            assembly["path"],
            assembly["chain_root"],
            assembly["chain_tip"],
            assembly["chain_tip"],
            _robot_model_from_path(assembly["path"]),
            assembly["tool_body"],
            assembly["tcp_site"],
            assembly["ft_sensors"],
            assembly["prefix"],
            assembly["trees"],
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


def _id_ref(value):
    """Normalize a value to its id string (str/Enum/.id), or None."""
    if value is None:
        return None
    if isinstance(value, str):
        return value
    if isinstance(value, Enum):
        return value.value
    return getattr(value, "id", None)


def _id_refs(value):
    """Map _id_ref over a list of values."""
    if value is None:
        return []
    if isinstance(value, list):
        return [ref for item in value if (ref := _id_ref(item))]
    ref = _id_ref(value)
    return [ref] if ref else []


def _dedupe_dicts(entries, key="id"):
    """Deduplicate dict rows by id, keeping the first occurrence."""
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
):
    """Build the introspection artifact (uris, motions, controllers, monitors, quantities,
    provenance) and fold in the controller-state and frame-log samples.
    """
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
                "proportional_gain": getattr(controller, "proportional_gain", None),
                "integral_gain": getattr(controller, "integral_gain", None),
                "derivative_gain": getattr(controller, "derivative_gain", None),
                "decay_rate": getattr(controller, "decay_rate", None),
                "stiffness": getattr(controller, "stiffness", None),
                "damping": getattr(controller, "damping", None),
                "error_signal": _id_ref(getattr(controller, "error_signal", None)),
                "reference_signal": _id_ref(getattr(controller, "reference_signal", None)),
                "measured_derivative": _id_ref(getattr(controller, "measured_derivative", None)),
                "output_signal": _id_ref(controller.control_signal),
            }
            controllers.append(
                {k: v for k, v in controller_entry.items() if v is not None and v != []}
            )
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
                    signals.append(
                        {k: v for k, v in signal_entry.items() if v is not None and v != []}
                    )
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
                    "fallback_motion": getattr(monitor, "fallback_motion", None),
                    "debounce_duration_s": getattr(monitor, "debounce_duration_s", None),
                    "debounce_steps": getattr(monitor, "debounce_steps", None),
                }
                monitors.append(
                    {k: v for k, v in monitor_entry.items() if v is not None and v != []}
                )
                if monitor.error is not None:
                    signal_entry = {
                        "id": f"{monitor.id}.error",
                        "uri": uri_by_id.get(monitor.error.id),
                        "quantity": monitor.error.id,
                        "role": "monitor_error",
                        "owner": monitor.id,
                    }
                    signals.append(
                        {k: v for k, v in signal_entry.items() if v is not None and v != []}
                    )

    quantities = []
    for item in data_structures:
        quantity_entry = {
            "id": item.id,
            "uri": uri_by_id.get(item.id),
            "type": item.type,
            "unit": _id_refs(getattr(item, "unit", None)),
            "quantity_kind": _id_refs(getattr(item, "quantity_kind", None)),
            # Reference frame the spatial value is expressed in — the frame_id for the
            # pose/twist/wrench spatial samples.
            "reference_frame": _id_ref(getattr(item, "as_seen_by", None))
            or _id_ref(getattr(item, "with_respect_to", None)),
            "reference_value": getattr(item, "reference_value", None),
            "value": getattr(item, "value", None),
            "authored": getattr(getattr(item, "provenance", None), "authored", False),
            "snapshot": getattr(getattr(item, "provenance", None), "snapshot", False),
        }
        quantities.append({k: v for k, v in quantity_entry.items() if v is not None and v != []})

    runtime_type = {"mj_kdl": "exec:Simulation", "robif2b": "exec:RealWorld"}.get(
        backend, "exec:ExecutionContext"
    )
    runtime_id = "agent:runtime:mujoco" if backend == "mj_kdl" else "agent:runtime:real_robot"
    runtime_activity_type = (
        "bdd:SimulatedExecution" if backend == "mj_kdl" else "bdd:ScenarioExecution"
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
            {
                k: v
                for k, v in {
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
                }.items()
                if v is not None and v != []
            }
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
    add_controller_internal_state_logging(closures, shared_data, introspection, motions)
    add_quantity_samples(introspection, shared_data, views)
    add_spatial_samples(introspection, shared_data)
    return introspection


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
    imported_provenance = []
    imported_model_locations = []
    for item in imported_files:
        if item.endswith("/provenance/dsl.ld.json"):
            imported_provenance.append(_resolve_import_location(item, url_map))
        else:
            imported_model_locations.append(item)
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
    for type_, parse in (
        (GEOM_COORD.PoseCoordinate, p.pose),
        (GEOM_COORD.VelocityTwistCoordinate, p.velocity_twist),
        (KC_STAT.JointPositionCoordinate, p.joint_position),
        (RBDYN_COORD.WrenchCoordinate, p.wrench),
    ):
        for node in sorted(g.subjects(RDF.type, type_), key=str):
            scope = p._context_scope(node)
            if scope is None or scope[1] != "World":
                continue
            if type_ == KC_STAT.JointPositionCoordinate:
                joint = g.value(node, KC_STAT["of-joint"])
                if joint is None or not any(_tree_owns(tree, joint) for tree in owned_trees):
                    continue
            frame_node = g.value(node, GEOM_COORD["as-seen-by"])
            if frame_node is None:
                _of, _wrt, frame_node = p._derived_reference_frames(node)
            if frame_node is not None:
                frame_body = _body_of(frame_node)
                frame_tree = iri_parent(frame_body)
                runtime_frame = _leaf(frame_body)
                if frame_tree in owned_trees:
                    runtime_frame = f"{runtime_prefix}{_leaf(frame_body)}"
                if runtime_frame != chain_root:
                    continue
            output = parse(node)
            if type_ == KC_STAT.JointPositionCoordinate:
                output = replace(output, joint_name=f"{runtime_prefix}{output.joint_name}")
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
    slv_base_vel = []
    sched2 = []
    hdl = []
    sched3 = []
    slv_arm = []
    sched4 = []
    slv_base_frc = []

    for s in g.subjects(RDF.type, SLV["VelocityCompositionSolver"]):
        slv_base_vel.append(p.velocity_composition_solver(s))
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
            if len(plan.axes) > 1 or CSTR_HDL_EXT.FeedForwardController in g[
                plan.controller : RDF.type
            ]:
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
            if plan.solver not in solver_nodes and SLV.SolverWithInputAndOutput in g[
                plan.solver : RDF.type
            ]:
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
            solver.ft_sensors,
            solver.runtime_prefix,
            solver.owned_trees,
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
        slv_arm.append(solver)
        start = g[
            s
            : SLV["motion-drivers"]
            / ((SLV["cartesian-force"]) | (SLV["joint-force"]))
        ]
        sched3.extend(p.schedule(start, ops_generic + ops_slv))

    for s in g.subjects(RDF.type, SLV["ForceDistributionSolver"]):
        slv_base_frc.append(p.force_distribution_solver(s))
        sched4.extend(p.schedule([s], ops_generic + ops_slv))

    return slv_base_vel, sched1, hdl, sched2, slv_arm, sched3, slv_base_frc, sched4


def _assign_monitor_event_indexes(handlers) -> None:
    """Assign each monitor a stable per-handler event index."""
    event_idx = 0
    for handler in handlers:
        for monitor in handler.monitors:
            if monitor.monitor_type == "EdgeTriggeredMonitor":
                monitor.event_idx = event_idx
                event_idx += 1


def _snapshot_source_map(g, p: Parser) -> dict[str, str]:
    """Map each snapshot output to its source quantity."""
    snapshot_source_map: dict[str, str] = {}
    for snap_node in g.subjects(RDF.type, ALGO_EXT.Snapshot):
        source_node = g.value(snap_node, ALGO_EXT["in"])
        output_node = g.value(snap_node, ALGO_EXT.out)
        if source_node is not None and output_node is not None:
            snapshot_source_map[p.id(output_node)] = p.id(source_node)
    return snapshot_source_map


def _closure_owner_map(g, p: Parser, closures) -> dict[str, str]:
    """Map each closure to the motion whose context declares the quantities it reads.

    The backward schedule walk can reach a closure belonging to another motion, which then
    runs (and mutates its outputs) whenever that unrelated motion is active. Ownership is the
    motion segment of the quantities it references: <app>/<motion>/Spec/spec/<name>. A closure
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
            if scope is not None and scope[1] == "Spec":
                owners.add(get_valid_var_name(scope[0]))
        if len(owners) == 1:
            owner_map[closure_id] = owners.pop()
    return owner_map


def _snapshot_owner_map(g, p: Parser) -> dict[str, str]:
    """Map each snapshot's output to the motion that declares it.

    Every motion captures each snapshot it *references*, and they all write the same shared
    slot, so a motion re-capturing another's snapshot silently retargets it. The owner is the
    motion segment of the quantity's URI: <app>/<motion>/Spec/spec/<name>.
    """
    owner_map: dict[str, str] = {}
    for snap_node in g.subjects(RDF.type, ALGO_EXT.Snapshot):
        output_node = g.value(snap_node, ALGO_EXT.out)
        if output_node is None:
            continue
        scope = p._context_scope(output_node)
        if scope is None:
            continue
        owner_map[p.id(output_node)] = get_valid_var_name(scope[0])
    return owner_map


def _snapshot_trigger_map(g, p: Parser) -> dict[tuple[str, str], str]:
    """Map each event-triggered snapshot to its trigger event's local name, keyed by
    (declaring motion, output id). Every motion referencing a snapshot captures it, but only
    the motion that declares it re-samples on the trigger, so the key carries the owner. That
    owner is the motion segment of the quantity's URI: <app>/<motion>/Spec/spec/<name>.
    """
    trigger_map: dict[tuple[str, str], str] = {}
    for snap_node in g.subjects(RDF.type, ALGO_EXT.Snapshot):
        trigger_node = g.value(snap_node, ALGO_EXT["trigger"])
        output_node = g.value(snap_node, ALGO_EXT.out)
        if trigger_node is None or output_node is None:
            continue
        scope = p._context_scope(output_node)
        if scope is None:
            continue
        owner = get_valid_var_name(scope[0])
        trigger_map[(owner, p.id(output_node))] = get_valid_var_name(_leaf(trigger_node)).upper()
    return trigger_map


def _pose_frames(g, pose) -> tuple[URIRef, URIRef]:
    """Return a pose quantity's authored `(of, with-respect-to)` frames."""
    of_frame = g.value(pose, GEOM_REL.of)
    wrt_frame = g.value(pose, GEOM_REL["with-respect-to"])
    if not isinstance(of_frame, URIRef) or not isinstance(wrt_frame, URIRef):
        raise ValueError(f"Pose {pose} needs explicit of/with-respect-to frames.")
    return of_frame, wrt_frame


def _materialize_linear_distance_operations(g) -> None:
    """Expand authored linear-distance relations into codegen operations.

    The DSL graph states only the two pose endpoints. This operational expansion belongs
    here because its inverse/composition path is derivable from the complete RDF graph.
    """

    def derived(node, suffix):
        return URIRef(f"{node}.derived-{suffix}")

    def emit_pose(node, of_frame, wrt_frame):
        g.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        g.add((node, RDF.type, GEOM_REL.Pose))
        g.add((node, RDF.type, GEOM_COORD.PoseCoordinate))
        g.add((node, GEOM_REL.of, of_frame))
        g.add((node, GEOM_REL["with-respect-to"], wrt_frame))
        g.add((node, GEOM_COORD["as-seen-by"], wrt_frame))

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
        g.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        g.add((node, RDF.type, GEOM_REL.Pose))
        g.add((node, RDF.type, GEOM_COORD.PoseCoordinate))
        g.add((node, GEOM_REL.of, of_frame))
        g.add((node, GEOM_REL["with-respect-to"], wrt_frame))
        g.add((node, GEOM_COORD["as-seen-by"], wrt_frame))

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
        if GEOM_REL.Pose not in g[quantity : RDF.type]:
            continue
        if GEOM_REL.Pose not in g[reference : RDF.type]:
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
        inputs = {v for k, v in c.items() if k not in {"id", "type"} and isinstance(v, str)}
        out_field = _CLOSURE_OUTPUT_FIELDS.get(c.get("type", ""))
        if out_field:
            out_val = c.get(out_field)
            if isinstance(out_val, str):
                closure_output_map[out_val] = cid
                closure_input_map[out_val] = {v for v in inputs if v != out_val}
    return closure_output_map, closure_input_map


def _backend_from_graph(g) -> str:
    """Select the runtime backend from the authored execution context."""
    simulation = next(g.subjects(RDF.type, EXEC.Simulation), None)
    if simulation is not None:
        name = str(g.value(simulation, EXEC["platform-name"]) or "").casefold()
        if name == "mujoco":
            return "mj_kdl"
        raise ValueError(f"Unsupported simulation platform '{name}'.")
    return "robif2b"


def _is_robif2b_communication_test(g) -> bool:
    """Return whether the real-world target requests the bounded hardware smoke test."""
    real_world = next(g.subjects(RDF.type, EXEC.RealWorld), None)
    if real_world is None:
        return False
    name = str(g.value(real_world, EXEC["platform-name"]) or "").casefold()
    return name == "robif2b-communication-test"


def _apply_monitor_debounce(handlers, control_period_ns: int) -> None:
    """Convert each monitor's debounce duration to a step count from the control period."""
    for handler in handlers:
        for monitor in handler.monitors:
            if getattr(monitor, "debounce_duration_s", None) is not None:
                monitor.debounce_steps = round(
                    monitor.debounce_duration_s / (control_period_ns * 1e-9)
                )


def _shared_runtime_members(slv_arm) -> list[dict]:
    """Extra shared-data members for force/torque sensor state."""
    members = []
    seen_ft_ids = set()
    for s in slv_arm:
        for out in s.output:
            if getattr(out, "type", None) == "Wrench" and getattr(out, "sensor_name", ""):
                if out.id in seen_ft_ids:
                    continue
                seen_ft_ids.add(out.id)
                members.append({"id": f"{out.id}_ft_bias", "type": "FreeVector"})
                members.append({"id": f"{out.id}_ft_settle", "type": "IntCounter"})

    return members


# ---------------------------------------------------------------------------
# Codegen-facing helpers. Each is invoked while constructing the piece it belongs to
# (scene, solvers, closures, motions, introspection) so generate_ir builds a complete IR
# in one forward pass — the assembled ir dict is final and is never re-processed.
# ---------------------------------------------------------------------------


SUPPORTED_ROBOT_MODELS = {"KinovaGen3"}


def _validate_solvers(arm_solvers, backend: str) -> None:
    """Reject unsupported robot models, and scene-object pose sync on the robif2b backend."""
    unsupported = {
        _field(s, "robot_model")
        for s in arm_solvers
        if _field(s, "robot_model") and _field(s, "robot_model") not in SUPPORTED_ROBOT_MODELS
    }
    if unsupported:
        raise RuntimeError(
            f"Unsupported robot model(s): {', '.join(sorted(unsupported))}. "
            f"Supported: {', '.join(sorted(SUPPORTED_ROBOT_MODELS))}"
        )

    if backend != "robif2b":
        return

    for solver in arm_solvers:
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


def _annotate_runtime_robots(arm_solvers, motions, backend: str) -> None:
    """Assign runtime_id/runtime_owner across solvers sharing a runtime and normalize empty tool
    fields.
    """
    runtime_by_signature: dict[tuple, str] = {}
    owner_by_runtime: dict[str, str] = {}
    solvers_by_id = {_field(solver, "id"): solver for solver in arm_solvers}

    for solver in arm_solvers:
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
        for solver in _field(motion, "arm_solvers", []):
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


def _add_group_type_flags(groups: list) -> list:
    """Set is_pose/is_twist/is_wrench on pose-axis error groups from their superobject type."""
    for g in groups:
        so_type = _field(g, "superobject_type", "Pose")
        _set_field(g, "is_pose", so_type == "Pose")
        _set_field(g, "is_twist", so_type in ("VelocityTwist", "AccelerationTwist"))
        _set_field(g, "is_wrench", so_type == "Wrench")
    return groups


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


def expand_vector_fields(item, field: str) -> None:
    """Expand a 3-vector field into <field>_x/_y/_z components (an omitted value means zero)."""
    values = _field(item, field)
    if values is None:
        # Env placement shorthand: omitted position/orientation means zero/identity.
        values = [0.0, 0.0, 0.0]
    if not isinstance(values, list) or len(values) != 3:
        item_id = _field(item, "id", "<unknown>")
        raise ValueError(f"Scene item '{item_id}' has invalid '{field}'; expected three values.")
    _set_field(item, f"{field}_x", values[0])
    _set_field(item, f"{field}_y", values[1])
    _set_field(item, f"{field}_z", values[2])


def require_field(obj_id: str, field: str, value):
    """Return value, raising if a required procedural scene-object field is missing."""
    if value is None:
        raise ValueError(
            f"Procedural scene object '{obj_id}' is missing required field "
            f"'{field}'. Add it to the .robmot model — silent defaults are no "
            f"longer applied."
        )
    return value


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
    closures: dict, shared_data: list, introspection: dict, motions
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
        for state_name, item_type, getter in samples:
            item_id = f"{controller_id}_{state_name}"
            add_shared(item_id, item_type, controller_id, state_name)
            if item_type == "Quantity":
                add_quantity(item_id, controller_id, state_name)
            closure_samples.append({"id": item_id, "getter": getter})
        closure["internal_state_samples"] = closure_samples


def add_quantity_samples(introspection: dict, shared_data: list, views: dict) -> None:
    """Build the per-quantity frame-log sample descriptors from the introspection quantities and
    shared data.
    """
    shared_ids = {_field(item, "id") for item in shared_data if _field(item, "id")}
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
        view = views.get(data_id)
        return not view or _field(view, "axis") is not None

    for quantity in introspection.get("quantities", []):
        qid = quantity.get("id")
        if not qid:
            continue
        qtype = quantity.get("type")
        if qtype == "Quantity":
            if quantity.get("value") is not None and qid not in shared_ids and qid not in views:
                add(quantity, "", {"kind": "literal", "value": str(quantity["value"])})
            elif qid in views and scalar_view(qid):
                # A scalar view resolves to a composite-member access only for these
                # superobject types; other superobjects (e.g. PoseDifference) sample the
                # quantity's own shared field instead.
                so_type = _field(_field(views.get(qid), "superobject"), "type")
                if so_type in {"Pose", "Wrench", "VelocityTwist", "AccelerationTwist"}:
                    add(quantity, "", {"kind": "access", "ref": qid})
                else:
                    add(quantity, "", {"kind": "shared", "id": qid})
            elif qid in shared_ids and qid not in views:
                add(quantity, "", {"kind": "shared", "id": qid})
        elif qtype in {"Position", "Direction", "FreeVector"} and qid in shared_ids:
            add_axes(quantity, "", lambda i, q=qid: {"kind": "vec", "id": q, "axis": i})
        elif qtype == "Orientation" and qid in shared_ids:
            add_axes(quantity, "", lambda i, q=qid: {"kind": "orientation", "id": q, "axis": i})
        elif qtype in {"Pose", "Trajectory"} and qid in shared_ids:
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

    introspection["quantity_samples"] = samples


def add_spatial_samples(introspection: dict, shared_data: list) -> None:
    """Add per-object pose, velocity-twist and wrench frame-log samples."""
    shared_ids = {_field(item, "id") for item in shared_data if _field(item, "id")}
    kinds = {"Pose": "poses", "VelocityTwist": "twists", "Wrench": "wrenches"}
    spatial = {"poses": [], "twists": [], "wrenches": []}
    for item in shared_data:
        iid = _field(item, "id")
        pool = kinds.get(_field(item, "type"))
        if not iid or iid not in shared_ids or pool is None:
            continue
        spatial[pool].append({"id": iid, "index": len(spatial[pool])})
    introspection["spatial_samples"] = spatial


def _index_by_id(items: list) -> dict:
    """Index IR items (dicts or dataclasses) by their id (skips id-less entries)."""
    out = {}
    for item in items:
        iid = _field(item, "id")
        if iid:
            out[iid] = item
    return out


def build_pose_components(views: dict, data: list) -> dict:
    """Resolve declared/inline poses into per-axis structured components (a literal value or a
    reference id).
    """
    data_by_id = _index_by_id(data)
    components: dict[str, dict] = {}
    for view in views.values():
        superobject = _field(view, "superobject")
        so_type = _field(superobject, "type")
        so_prov = _field(superobject, "provenance") or {}
        is_declared_pose = bool(_field(so_prov, "authored") or _field(so_prov, "snapshot"))
        if so_type != "Pose":
            continue
        if not (is_declared_pose or _field(superobject, "euler_axes_sequence")):
            continue
        # Only include inline-defined poses (those where components have values/references).
        subobject_id = _field(_field(view, "subobject"), "id")
        subobject_data = data_by_id.get(subobject_id)
        if (
            not _field(subobject_data, "reference_value")
            and _field(subobject_data, "value") is None
        ):
            continue
        pose_id = _field(superobject, "id")
        entry = components.setdefault(
            pose_id,
            {
                "position_x": None,
                "position_y": None,
                "position_z": None,
                "orientation_x": None,
                "orientation_y": None,
                "orientation_z": None,
            },
        )
        axis = str(_field(view, "axis") or "").lower()
        if axis not in {"x", "y", "z"}:
            continue
        subobject = _field(_field(view, "subobject"), "id")
        if not subobject:
            continue
        prefix = "position" if _field(view, "subspace") == "Linear" else "orientation"
        entry[f"{prefix}_{axis}"] = _pose_component(subobject, data_by_id)
    for pose_id, parts in components.items():
        missing = [name for name, value in parts.items() if value is None]
        if missing:
            raise ValueError(
                f"Declared pose '{pose_id}' is missing required components: {', '.join(missing)}."
            )
    return components


def resolve_lerp_closures(closures: dict, pose_components: dict) -> None:
    """Fold each linear-path goal into components or a shared-signal ref."""
    for closure in closures.values():
        if closure.get("type") != "LinearPath":
            continue
        goal = closure.get("goal")
        if not isinstance(goal, str):
            continue
        if goal in pose_components:
            # Emit the structured pose components; the template builds the pose frame.
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
        if closure.get("type") != "Arc":
            continue
        end = closure.get("end")
        end_data = data_by_id.get(end)
        if not isinstance(end, str) or not is_pose(end_data):
            raise ValueError("Arc trajectory end must be a Pose quantity.")
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


def _set_motion_trajectory_progress(motion, closures: dict, data_by_id: dict) -> None:
    """Fold the time-driven trajectory parameter ids (Progress-kind, non-Arc while-schedule
    closures) onto a motion (dict or dataclass)."""
    ids: list[str] = []
    for step in _field(motion, "while_schedule", []):
        closure = closures.get(step)
        if not closure or _field(closure, "type") not in {
            "LinearPath",
            "Circle",
            "Arc",
            "Helix",
            "Figure8",
        }:
            continue
        alpha_id = _field(closure, "path_parameter") or _field(closure, "alpha")
        alpha_data = data_by_id.get(alpha_id)
        qkind = _field(_field(alpha_data, "quantity_kind"), "id")
        if (
            qkind in {"Progress", "Dimensionless"}
            and _field(closure, "type") != "Arc"
            and alpha_id not in ids
        ):
            ids.append(alpha_id)
    _set_field(motion, "time_trajectory_progress_ids", ids)


def _evaluator_term(e, start_field: str) -> dict:
    """Structured boolean term for an evaluator: an elapsed timing predicate (world clock
    vs threshold from the selected state timestamp) or a solver constraint-satisfied check.
    Rendered to C++ by the bool-condition template."""
    if _field(e, "is_elapsed"):
        op = _field(e, "elapsed_op") or ">="
        thr = _field(e, "elapsed_threshold_s") or 0.0
        # Pre-format the threshold (fixed 6-decimal) so the emitted literal is stable.
        return {"kind": "elapsed", "start_field": start_field, "op": op, "threshold": f"{thr:.6f}"}
    return {"kind": "constraint", "error_id": _field(_field(e, "error"), "id")}


def _set_monitor_conditions(
    motion, evaluators_key: str, monitors_key: str, start_field: str, any_key: str
) -> None:
    """Stamp the structured active-phase terms onto the aggregate monitor + any
    elapsed-error monitors. Rendered to C++ by the bool-condition template."""
    evaluators = _field(motion, evaluators_key, [])
    terms = [
        _evaluator_term(e, start_field)
        for e in evaluators
        if _field(e, "error") or _field(e, "is_elapsed")
    ]
    any_flag = bool(_field(motion, any_key))
    elapsed_terms_by_error = {
        _field(_field(e, "error"), "id"): _evaluator_term(e, start_field)
        for e in evaluators
        if _field(e, "is_elapsed") and _field(e, "error")
    }
    aggregate_key = "is_until_aggregate" if any_key == "until_any" else "is_when_aggregate"
    for monitor in _field(motion, monitors_key, []):
        group_ids = set(_field(monitor, "group_constraint_ids") or ())
        if group_ids:
            group_terms = [
                _evaluator_term(e, start_field)
                for e in evaluators
                if _field(_field(e, "constraint"), "id") in group_ids
                and (_field(e, "error") or _field(e, "is_elapsed"))
            ]
            _set_field(monitor, "active_terms", group_terms)
            _set_field(monitor, "active_terms_present", bool(group_terms))
            _set_field(monitor, "active_any", bool(_field(monitor, "group_any")))
            _set_field(monitor, "has_active", True)
            continue
        if _field(monitor, aggregate_key):
            _set_field(monitor, "active_terms", terms)
            _set_field(monitor, "active_terms_present", bool(terms))
            _set_field(monitor, "active_any", any_flag)
            _set_field(monitor, "has_active", True)
            continue
        error_id = _field(_field(monitor, "error"), "id")
        if error_id in elapsed_terms_by_error:
            _set_field(monitor, "active_terms", [elapsed_terms_by_error[error_id]])
            _set_field(monitor, "active_terms_present", True)
            _set_field(monitor, "active_any", False)
            _set_field(monitor, "has_active", True)


def _set_motion_conditions(motion) -> None:
    """Fold the UNTIL/WHEN/done structured boolean terms onto a motion (rendered to C++ by
    the bool-condition template). WHEN joins with when_any, done with until_any."""
    _set_monitor_conditions(
        motion, "until_evaluators", "until_monitors", "motion_start_time", "until_any"
    )
    when_terms = [
        _evaluator_term(e, "when_start_time")
        for e in _field(motion, "when_evaluators", [])
        if _field(e, "error") or _field(e, "is_elapsed")
    ]
    _set_field(motion, "when_terms", when_terms)
    _set_field(motion, "when_terms_present", bool(when_terms))
    _set_monitor_conditions(
        motion, "when_evaluators", "when_monitors", "when_start_time", "when_any"
    )
    done_terms = _motion_done_terms(motion)
    _set_field(motion, "done_terms", done_terms)
    _set_field(motion, "done_terms_present", bool(done_terms))


def add_motion_function_interfaces(motions: list) -> None:
    """Fold per-motion capability booleans (which context objects — state, shared, robot —
    each generated function needs). The C++ signatures and call args are built from these
    by the sig-params / sig-args templates; ir_gen carries no C++ type names."""
    for motion in motions:
        has_when_elapsed = any(
            _field(e, "is_elapsed") for e in _field(motion, "when_evaluators", [])
        )
        has_when_logic = bool(_field(motion, "when_schedule") or _field(motion, "when_evaluators"))
        when_mons = _field(motion, "when_monitors") or []
        until_mons = _field(motion, "until_monitors") or []
        has_pose = bool(_field(motion, "declared_pose_components"))
        when_sched = bool(_field(motion, "when_schedule"))
        until_sched = bool(_field(motion, "until_schedule"))
        when_fsm = any(_field(m, "fsm_namespace") for m in when_mons)
        until_fsm = any(_field(m, "fsm_namespace") for m in until_mons)
        has_forwarded_commands = bool(_field(motion, "forwarded_commands"))
        has_arm = bool(_field(motion, "arm_solvers"))

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

        _set_field(motion, "apply_needs_state", has_arm)
        _set_field(motion, "apply_needs_shared", has_forwarded_commands)
        _set_field(motion, "apply_needs_robot", has_arm or has_forwarded_commands)


_FSM_NS = "https://secorolab.github.io/metamodels/behaviour/fsm#"


# ---------------------------------------------------------------------------
# FSM wiring
# ---------------------------------------------------------------------------
def _fsm_from_graph(g) -> dict | None:
    """Frame the FSM named graph (states/events/transitions/reactions, folded into the
    model dataset by motion-spec-dsl) into the same dict shape the standalone .hpp uses,
    so codegen needs no fsm_ir.json read. None when the model imports no .fsm."""
    FSM = rdflib.Namespace(_FSM_NS)
    fsm_ref = next(iter(g.subjects(RDF["type"], FSM["FSM"])), None)
    if fsm_ref is None:
        return None

    def ident(uri):
        """FSM identifier token (upper-cased var name) for a graph URI."""
        return get_valid_var_name(g.compute_qname(uri)[2]).upper()

    states, state_uris = [], {}
    for s in g.objects(fsm_ref, FSM["states"]):
        key = ident(s)
        states.append(key)
        state_uris[key] = str(s)
    events, event_uris = [], {}
    for e in g.objects(fsm_ref, FSM["events"]):
        key = ident(e)
        events.append(key)
        event_uris[key] = str(e)

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
        fires = [ident(ev) for ev in g.objects(rx, FSM["fires-events"])]
        reactions_table.append(
            {
                "id": ident(rx),
                "uri": str(rx),
                "when_event": ident(g.value(rx, FSM["when-event"])),
                "do_transition": ident(g.value(rx, FSM["do-transition"])),
                "fires_events": fires,
                "num_fires": len(fires),
            }
        )

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
    """Tag FSM-event monitors + their motions from the framed FSM, and return the FSM
    header/step meta fields. Runs before the function-interface pass so the FSM-added robot
    param is picked up. No tagging when the model has no FSM. Runs during motion construction."""
    fsm_namespace = fsm["name"].lower() if fsm else None
    events = fsm.get("events", []) if fsm else []
    fsm_event_index = {event: idx for idx, event in enumerate(events)}
    fsm_step_event = "E_STEP" if "E_STEP" in events else None
    meta = {
        "fsm_namespace": fsm_namespace,
        "fsm_header": f"{fsm['name']}.hpp" if fsm else None,
        "fsm_step_event": fsm_step_event,
        "fsm_step_event_idx": fsm_event_index.get(fsm_step_event, -1),
    }
    if fsm_namespace is None:
        return meta

    fsm_ns_uri = fsm.get("namespace_uri")
    event_state = _event_to_state(fsm)
    by_id = {_field(m, "id"): m for m in motions}

    def tag_run_state(motion, monitors):
        """Tag FSM-event monitors and set their motion's run state."""
        for monitor in monitors:
            if is_fsm_event(monitor, fsm_ns_uri):
                _set_field(monitor, "fsm_namespace", fsm_namespace)
                _set_field(
                    monitor,
                    "fsm_event_idx",
                    fsm_event_index.get(_field(monitor, "event_name") or "", -1),
                )
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
            _set_field(monitor, "fsm_namespace", fsm_namespace)
            _set_field(
                monitor,
                "fsm_event_idx",
                fsm_event_index.get(_field(monitor, "event_name") or "", -1),
            )
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
    _materialize_pose_reference_transforms(g)
    _materialize_linear_distance_operations(g)

    p = Parser(g)
    node_by_id, id_nodes = _node_indexes(g, p)
    setups_by_node, ordered_setups = _robot_setups_from_graph(g)
    default_setup = (
        ordered_setups[0]
        if ordered_setups
        else ("", "", "", "", "", "", "", [], "", [])
    )
    # Derive backend + FSM up front: both are pure functions of the graph and are inputs to
    # downstream construction (solver validation, runtime-robot annotation, motion FSM wiring).
    backend = _backend_from_graph(g)
    robif2b_communication_test = _is_robif2b_communication_test(g)
    fsm = _fsm_from_graph(g)
    scene = _scene_from_graph(g)
    _validate_scene(scene)
    derivation = _solver_derivation_context(g)

    (slv_base_vel, sched1, hdl, sched2, slv_arm, sched3, slv_base_frc, sched4) = _solver_sections(
        g, p, setups_by_node, default_setup, derivation, scene.objects
    )
    _assign_monitor_event_indexes(hdl)

    closures = p.closures(ops_generic + ops_slv + ops_cstr_hdl)
    _derive_solver_closures(g, p, derivation, closures)
    view_map = p.view()
    data_structures = p.data_structures()
    _derive_solver_data(g, p, derivation, data_structures, view_map)
    snapshot_source_map = _snapshot_source_map(g, p)
    snapshot_trigger_map = _snapshot_trigger_map(g, p)
    snapshot_owner_map = _snapshot_owner_map(g, p)
    closure_owner_map = _closure_owner_map(g, p, closures)
    data_reference_map = _data_reference_map(data_structures, closures)
    closure_output_map, closure_input_map = _closure_maps(closures)

    # Resolve declared-pose components and trajectory goals from views/data/closures before
    # motions are built (per-motion declared poses reference them).
    pose_components = build_pose_components(view_map, data_structures)
    resolve_lerp_closures(closures, pose_components)
    resolve_arc_closures(closures, data_structures)

    wrench_outputs = _dedupe_by_id(
        [
            item
            for item in data_structures
            if item.type == "Wrench" and item.id not in closure_output_map
        ]
    )

    motions, fsm_meta = build_motion_units(
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
        data_structures=data_structures,
        pose_components=pose_components,
        fsm=fsm,
        derivation=derivation,
        snapshot_trigger_map=snapshot_trigger_map,
        snapshot_owner_map=snapshot_owner_map,
        closure_owner_map=closure_owner_map,
    )
    _validate_solvers(slv_arm, backend)
    if robif2b_communication_test and slv_arm:
        raise ValueError("robif2b-communication-test must not declare an arm solver.")
    _annotate_runtime_robots(slv_arm, motions, backend)

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
        fk_output_ids={out.id for s in slv_arm for out in s.output},
    )
    shared_data = shared_data + _shared_runtime_members(slv_arm)

    introspection = _build_introspection(
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
    )

    schedule = sched1 + sched2 + sched3 + sched4
    shared_schedule = sched1 + sched3 + sched4
    ir = {
        "slv_arm": slv_arm,
        "slv_base_vel": slv_base_vel,
        "slv_base_frc": slv_base_frc,
        "cstr_hdl": hdl,
        "motions": motions,
        "data": data_structures,
        "closures": closures,
        "shared_schedule": shared_schedule,
        "schedule": schedule,
        "views": view_map,
        "shared_data": shared_data,
        "pose_components": pose_components,
        "declared_pose_components": declared_pose_component_entries(
            data_structures, pose_components
        ),
        "wrench_outputs": wrench_outputs,
        "has_arm": bool(slv_arm),
        "has_mobile_base": bool(slv_base_vel or slv_base_frc),
        # Elapsed constraints compare seconds from the runtime clock (MuJoCo sim seconds /
        # real monotonic wall clock).
        "needs_clock_time": any(m.has_elapsed for m in motions),
        "control_period_ns": control_period_ns,
        "arm_solvers": slv_arm,
        "base_velocity_solvers": slv_base_vel,
        "base_force_solvers": slv_base_frc,
        "backend": backend,
        "robif2b_communication_test": robif2b_communication_test,
        "scene": scene,
        "trace": _trace_from_graph(g),
        "uris": introspection["uris"],
        "introspection": introspection,
        # FSM (states/events/transitions/reactions) framed from the FSM named graph that
        # motion-spec-dsl folds into the model dataset; None when no .fsm is imported.
        "fsm": fsm,
        **fsm_meta,
    }
    # ir is complete by construction — every codegen-facing field was computed while its
    # piece was built (scene / solvers / closures / motions / introspection). Codegen only
    # loads ir.json and renders; there is no post-assembly derivation pass.
    return ir


def main(argv: list[str] | None = None):
    """Generate intermediate representation (IR) from motion specification models."""
    parser = argparse.ArgumentParser(
        prog="motion-spec ir",
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

    args = parser.parse_args(argv)

    # Determine output destination
    if args.console or (args.output and args.output == "-"):
        output_file = None  # stdout
    else:
        output_file = Path(args.output)

    ir = generate_ir(args.manifest)

    # Output IR to file or stdout
    ir_json = json.dumps(ir, cls=DataclassJSONEncoder, indent=4)

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
