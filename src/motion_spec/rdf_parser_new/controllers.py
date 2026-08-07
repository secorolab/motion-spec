# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The control law: authored controllers become per-axis control records.

In order: the solver families and what each may be driven by, the derived ids, the derived ids and the
IRIs they register, the controller records themselves, the closures and data they imply, and last
the tables that say what state and which gains a controller type carries.

This is the seam a controller DSL replaces. Its inputs are the model, the closures and the data
structures; its outputs are the `SolverDerivationContext` and the controller and driver records.
No other module derives a controller, and nothing here appends to the frame log: what a controller
contributes to introspection is exported as a table for `communication.py` to read.
"""

from __future__ import annotations

import collections
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import NamedTuple

from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT,
    APP,
    CSTR,
    CSTR_EXT,
    CSTR_HDL,
    CSTR_HDL_EXT,
    GEOM_COORD,
    GEOM_OP,
    GEOM_OP_EXT,
    KC_STAT,
    MAP,
    QUDT_SCHEMA,
    SLV,
    SLV_EXT,
)
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdflib import URIRef
from rdflib.namespace import PROV, RDF
from scene_dsl.rdf_parser.common import ensure_one_obj_uri

from motion_spec.classes.entities import (
    AccelerationConstraint,
    CartesianAccelerationSpecification,
    CartesianForceSpecification,
    FeedForwardController,
    ImpedanceController,
    JointForceSpecification,
    MotionDrivers,
    PIDController,
    Point,
    PoseDifference,
    Provenance,
    Quantity,
    QuantityKind,
    Saturation,
    Unit,
    View,
)
from motion_spec.rdf_parser_new import quantities
from motion_spec.rdf_parser_new.model import kebab, local_name

__all__ = [
    "ADMITTANCE_PARAMETERS",
    "CONTROLLER_GAIN_FIELDS",
    "CONTROLLER_SIGNAL_ROLES",
    "CONTROLLER_STATE_FIELDS",
    "SOLVER_ALGORITHMS",
    "ControllerDerivation",
    "MotionDriveInput",
    "SolverAlgorithm",
    "SolverDerivationContext",
    "SolverIdFactory",
    "annotate_controller_signals",
    "augment_closures",
    "augment_data",
    "authored_motion_drivers",
    "cartesian_force_specification",
    "joint_force_specification",
    "motion_drivers",
    "saturation",
    "solver_algorithm",
    "solver_derivation_context",
]


class MotionDriveInput(str, Enum):
    """The physical quantity a solver algorithm accepts to drive the chain.

    Vereshchagin's acceleration-constrained hybrid dynamics is posed as a constrained optimisation
    over Gauss's principle, so each constrained direction is driven by an *acceleration energy*
    (N-m2/s2) rather than by an acceleration. Recursive Newton-Euler is driven by the Cartesian
    acceleration itself. A command-forwarding solver runs no dynamics and is driven by neither:
    its controller's output goes straight to the joint.
    """

    NONE = "None"
    ACCELERATION_ENERGY = "AccelerationEnergy"
    CARTESIAN_ACCELERATION = "CartesianAcceleration"


@dataclass(frozen=True)
class SolverAlgorithm:
    """What one solver family means for the records derived against it.

    Attributes:
        drive_input: the quantity its motion drivers carry
        signal_prefix: the tag a control signal derived for it leads with
        codegen_name: the algorithm name the runtime and the templates dispatch on
        forwards_commands: whether its controller's output goes straight to a joint
    """

    drive_input: MotionDriveInput
    signal_prefix: str | None = None
    codegen_name: str = ""
    forwards_commands: bool = False


# Every solver family the code generator can run, keyed by the term that identifies it -- the
# algorithm a solver names, or the type a command-forwarding solver carries.
#
# Vereshchagin's acceleration-constrained hybrid dynamics is posed as a constrained optimisation
# over Gauss's principle, so each constrained direction is driven by an acceleration energy
# (N-m2/s2) and the derived signals lead with `eacc`. Recursive Newton-Euler is driven by the
# Cartesian acceleration itself, and leads with `acc`. A command-forwarding solver runs no
# dynamics at all: its controller's output goes straight to the joint.
SOLVER_ALGORITHMS = {
    SLV["AccelerationConstrainedHybridDynamicsAlgorithm"]: SolverAlgorithm(
        MotionDriveInput.ACCELERATION_ENERGY, "eacc", "ACHD"
    ),
    SLV["RecursiveNewtonEulerAlgorithm"]: SolverAlgorithm(
        MotionDriveInput.CARTESIAN_ACCELERATION, "acc", "RNE"
    ),
    SLV_EXT.CommandForwardingSolver: SolverAlgorithm(MotionDriveInput.NONE, forwards_commands=True),
}
# A solver naming no algorithm drives nothing: it is read for state and watched by monitors.
_MONITOR_ONLY = SolverAlgorithm(MotionDriveInput.NONE)


def solver_algorithm(model, solver: URIRef) -> SolverAlgorithm:
    """The family a solver belongs to, and so what may be derived against it.

    Raises:
        ConstraintViolation: the solver names an algorithm no backend implements.
    """
    for type_ in get_node_types(model.graph, solver):
        if type_ in SOLVER_ALGORITHMS:
            return SOLVER_ALGORITHMS[type_]
    algorithm = model.graph.value(solver, SLV["solver"])
    if algorithm is None:
        return _MONITOR_ONLY
    if algorithm not in SOLVER_ALGORITHMS:
        raise ConstraintViolation(
            "solver", f"Solver '{solver}' has unsupported algorithm '{algorithm}'."
        )

    return SOLVER_ALGORITHMS[algorithm]


@dataclass(frozen=True)
class SolverIdFactory:
    """The ids one authored controller implies, built solely from authored RDF resource ids."""

    controller: str
    motion: str

    def component_controller(self, axis: quantities.SpatialAxis) -> str:
        return f"{self.controller}_{axis.suffix}"

    def component_error(self, axis: quantities.SpatialAxis) -> str:
        return f"{self.controller}_err_{axis.suffix}"

    def component_energy(self, axis: quantities.SpatialAxis) -> str:
        return f"eacc_{self.controller}_{axis.suffix}"

    def component_acceleration(self, axis: quantities.SpatialAxis) -> str:
        return f"acc_{self.controller}_{axis.suffix}"

    def component_constraint(self, axis: quantities.SpatialAxis) -> str:
        return f"acc_cstr_{self.controller}_{axis.suffix}"

    def component_acceleration_specification(self, axis: quantities.SpatialAxis) -> str:
        return f"cart_acc_{self.controller}_{axis.suffix}"

    def component_measured_derivative(self, axis: quantities.SpatialAxis) -> str:
        return f"{self.controller}_measured_derivative_{axis.suffix}"

    def pose_evaluator(self) -> str:
        return f"eval_pose_diff_{self.controller}"

    def pose_difference(self) -> str:
        return f"pose_diff_{self.controller}"


# Every id a controller can imply, with the IRI segment it is minted under and how it relates to
# the controller: a per-axis component *narrows* the controller, anything computed from it is
# derived. The id leads with its tag (`eacc_<ctrl>_<axis>`), the IRI with the parent
# (`<ctrl-iri>/eacc-<axis>`). The bound methods sit in the table so every one has a visible
# reference; the sites each mint a different subset, and a missed registration leaves a slot
# unaddressable. An unused registration is inert.
_AXIS_DERIVATIONS = (
    (SolverIdFactory.component_controller, "{axis}", PROV.specializationOf),
    (SolverIdFactory.component_error, "err-{axis}", PROV.specializationOf),
    (SolverIdFactory.component_energy, "eacc-{axis}", PROV.wasDerivedFrom),
    (SolverIdFactory.component_acceleration, "acc-{axis}", PROV.wasDerivedFrom),
    (SolverIdFactory.component_constraint, "acc-cstr-{axis}", PROV.wasDerivedFrom),
    (SolverIdFactory.component_acceleration_specification, "cart-acc-{axis}", PROV.wasDerivedFrom),
    (
        SolverIdFactory.component_measured_derivative,
        "measured-derivative-{axis}",
        PROV.wasDerivedFrom,
    ),
)
_WHOLE_DERIVATIONS = (
    (SolverIdFactory.pose_evaluator, "eval-pose-diff"),
    (SolverIdFactory.pose_difference, "pose-diff"),
)


def _solver_ids(model, context, plan) -> SolverIdFactory:
    """The id factory for one controller, with its whole derived-IRI family registered."""
    ids = SolverIdFactory(model.id(plan.controller), model.motion_suffix(plan.motion))
    parent = str(plan.controller)
    for derive, segment in _WHOLE_DERIVATIONS:
        model.register_derived(derive(ids), parent, segment, PROV.wasDerivedFrom)
    for spatial_axis in plan.axes:
        for derive, segment, relation in _AXIS_DERIVATIONS:
            model.register_derived(
                derive(ids, spatial_axis),
                parent,
                segment.format(axis=kebab(spatial_axis.suffix)),
                relation,
            )

    return ids


def _path_projection_outputs(model) -> dict:
    """The local frame and measured speed each path projection produces, keyed by its path."""
    graph = model.graph
    speed_by_direction = {
        graph.value(op, GEOM_OP["direction"]): graph.value(op, GEOM_OP_EXT["along-speed"])
        for op in graph.subjects(RDF.type, GEOM_OP_EXT.TwistToLinearVelocityAlong)
    }
    outputs = {}
    for frame in graph.subjects(RDF.type, GEOM_OP_EXT.PathTangentFrame):
        roles = {
            role: graph.value(frame, GEOM_OP_EXT[role])
            for role in ("tangent", "normal-a", "normal-b")
        }
        roles["along-speed"] = speed_by_direction.get(roles["tangent"])
        outputs[graph.value(frame, GEOM_OP_EXT.path)] = roles

    return outputs


def _path_following_axes(outputs, quantity, subspace) -> tuple[quantities.SpatialAxis, ...]:
    """The directions one path-following constraint controls.

    A path fixes geometry but not timing, so the roles never mix: the tangent is timing, the two
    normals hold the frame on the path, and orientation tracking is the ordinary angular triple.
    """
    if quantity == outputs["along-speed"]:
        return (quantities.SpatialAxis("linear-acceleration", "tangent", outputs["tangent"]),)
    if subspace == "position":
        return (
            quantities.SpatialAxis("linear-acceleration", "normal_a", outputs["normal-a"]),
            quantities.SpatialAxis("linear-acceleration", "normal_b", outputs["normal-b"]),
        )
    if subspace == "orientation":
        return quantities.ANGULAR_AXES
    raise ValueError(
        f"Path-following constraint on '{quantity}' must control the speed along the path, "
        "its position, or its orientation."
    )


# The controller types the axis decision distinguishes, and the target coordinate kinds it asks
# about, both as local names because that is what `spatial_axes` decides on.
_CONTROLLER_TYPES = (
    CSTR_HDL.ImpedanceController,
    CSTR_HDL.ProportionalIntegralDerivative,
    CSTR_HDL_EXT.FeedForwardController,
)
_TARGET_QUANTITY_KINDS = (
    (GEOM_COORD.PoseCoordinate, "Pose"),
    (KC_STAT.JointPositionCoordinate, "JointPosition"),
)


def _authored_controller_axes(model) -> dict:
    """Cartesian directions per authored controller, derived only from authored facts."""
    graph = model.graph
    projections = _path_projection_outputs(model)
    result = {}
    for controller in set(graph.objects(None, CSTR_HDL.controllers)):
        constraint = graph.value(controller, CSTR_HDL.constraint)
        quantity = graph.value(constraint, CSTR.quantity) if constraint is not None else None
        if constraint is None or quantity is None:
            continue
        view = next(graph.subjects(MAP.subobject, quantity), None)
        subspace = local_name(graph.value(view, MAP.subspace)) if view is not None else None
        path = graph.value(constraint, GEOM_OP_EXT.path)
        if path is not None:
            result[controller] = _path_following_axes(projections[path], quantity, subspace)
            continue
        target = graph.value(view, MAP.superobject) if view is not None else quantity
        target_types = get_node_types(graph, target)
        controller_types = get_node_types(graph, controller)
        command_type = graph.value(controller, APP["command-type"])
        result[controller] = quantities.spatial_axes(
            controller_type=next(
                (local_name(t) for t in _CONTROLLER_TYPES if t in controller_types), ""
            ),
            subspace=subspace,
            axis=local_name(graph.value(view, MAP.axis)) if view is not None else None,
            command_type=str(command_type) if command_type is not None else None,
            relation=next(
                (
                    local_name(type_)
                    for type_ in get_node_types(graph, constraint)
                    if type_ != CSTR.Constraint and local_name(type_).endswith("Constraint")
                ),
                "",
            ),
            quantity_kind=next(
                (name for type_, name in _TARGET_QUANTITY_KINDS if type_ in target_types), None
            ),
        )

    return result


@dataclass(frozen=True)
class ControllerDerivation:
    """The authored facts one controller's solver IR is derived from."""

    handler: URIRef
    motion: URIRef
    controller: URIRef
    solver: URIRef
    constraint: URIRef
    quantity: URIRef
    view: URIRef | None
    axes: tuple[quantities.SpatialAxis, ...]


@dataclass(frozen=True)
class SolverDerivationContext:
    """Immutable indexes for solver expansion, built once before any IR record is emitted.

    Carries the model rather than threading it through six signatures: every site that mints a
    derived id reaches the context, and those are the only places the parent node is still known.
    """

    model: object
    controllers_by_handler: dict
    controllers_by_solver: dict
    algorithm_by_solver: dict
    shared_constraints: frozenset
    _derived: dict = field(default_factory=dict, compare=False, repr=False)

    def controllers_for(self, plan: ControllerDerivation) -> tuple:
        """The derived per-axis controllers for one authored controller, computed once.

        Memoized so every consumer sees the same records: the controller dataclasses compare by
        identity, and a motion, its handler and the introspection rows must agree on which object
        each id names.
        """
        if plan not in self._derived:
            self._derived[plan] = tuple(_derived_controllers(self.model, self, plan))

        return self._derived[plan]


def solver_derivation_context(model) -> SolverDerivationContext:
    """Resolve controller ownership and command shape, before any solver node is generated.

    Raises:
        ValueError: a controller lacks its solver, constraint or quantity, or a handler's
            controllers cannot be assigned to solvers that can run them.
    """
    graph = model.graph
    axes_by_controller = _authored_controller_axes(model)
    by_handler = {}
    by_solver = collections.defaultdict(list)
    for handler in graph.subjects(RDF.type, CSTR_HDL.ConstraintHandler):
        motion = ensure_one_obj_uri(graph, handler, CSTR_HDL.motion)
        if motion is None:
            raise ValueError(f"Constraint handler '{handler}' is missing its motion.")
        authored = sorted(
            dict.fromkeys(graph.objects(handler, CSTR_HDL.controllers)),
            key=lambda node: int(getattr(graph.value(node, APP.order), "value", 0)),
        )
        plans = []
        for controller in authored:
            solver = ensure_one_obj_uri(graph, controller, CSTR_HDL_EXT.solver)
            constraint = ensure_one_obj_uri(graph, controller, CSTR_HDL.constraint)
            quantity = (
                ensure_one_obj_uri(graph, constraint, CSTR.quantity)
                if constraint is not None
                else None
            )
            if solver is None or constraint is None or quantity is None:
                raise ValueError(
                    f"Authored controller '{controller}' needs explicit solver, constraint, "
                    "and constraint quantity relations."
                )
            view = next(graph.subjects(MAP.subobject, quantity), None)
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
    solver_nodes = set(by_solver) | set(graph.subjects(RDF.type, SLV.SolverWithInputAndOutput))
    algorithms = {solver: solver_algorithm(model, solver) for solver in solver_nodes}
    _validate_solver_derivations(model, by_handler, by_solver, algorithms)

    return SolverDerivationContext(
        model=model,
        controllers_by_handler=by_handler,
        controllers_by_solver={node: tuple(plans) for node, plans in by_solver.items()},
        algorithm_by_solver=algorithms,
        shared_constraints=frozenset(
            node for node, count in constraint_counts.items() if count > 1
        ),
    )


def _validate_solver_derivations(model, by_handler, by_solver, algorithms) -> None:
    """Enforce executable solver limits, which only hold once authored RDF has resolved to plans."""
    for solver, plans in by_solver.items():
        if algorithms[solver].drive_input != MotionDriveInput.ACCELERATION_ENERGY:
            continue
        axes = [axis for plan in plans for axis in plan.axes]
        duplicates = [axis for axis, count in collections.Counter(axes).items() if count > 1]
        if duplicates:
            rendered = ", ".join(f"{axis.subspace}.{axis.axis}" for axis in duplicates)
            raise ValueError(f"ACHD solver '{solver}' repeats acceleration axis: {rendered}.")
        if len(axes) > 6:
            raise ValueError(
                f"ACHD solver '{solver}' has {len(axes)} axes; at most 6 are supported."
            )

    for handler, plans in by_handler.items():
        domains = collections.defaultdict(set)
        for plan in plans:
            command = str(model.graph.value(plan.controller, APP["command-type"]) or "")
            subspace = local_name(model.graph.value(plan.view, MAP.subspace)) if plan.view else None
            domains[algorithms[plan.solver].drive_input].add(
                "force"
                if command in {"Force", "Torque"} or subspace in {"force", "torque"}
                else "pose"
            )
        overlap = (
            domains[MotionDriveInput.ACCELERATION_ENERGY]
            & domains[MotionDriveInput.CARTESIAN_ACCELERATION]
        )
        if overlap:
            raise ValueError(
                f"Handler '{handler}' assigns ACHD and RNE to the same domain(s): "
                f"{', '.join(sorted(overlap))}."
            )


def _derived_quantity(id_: str, kind: str, unit: str, *, has_view: bool = False) -> Quantity:
    """A runtime scalar the model implies rather than authors."""
    return Quantity(id_, QuantityKind(kind), Unit(unit), None, has_view, provenance=Provenance())


def _acceleration_signal(id_: str, axis: quantities.SpatialAxis, input_kind) -> Quantity:
    """The acceleration-energy or Cartesian-acceleration signal a solver input kind accepts."""
    if input_kind == MotionDriveInput.ACCELERATION_ENERGY:
        return _derived_quantity(id_, "AccelerationEnergy", "N_M2_PER_SEC2")
    if input_kind == MotionDriveInput.CARTESIAN_ACCELERATION:
        if axis.subspace == "linear-acceleration":
            return _derived_quantity(id_, "LinearAcceleration", "M_PER_SEC2")

        return _derived_quantity(id_, "AngularAcceleration", "RAD_PER_SEC2")
    raise ValueError(f"Input kind '{input_kind}' does not carry acceleration.")


def _motion_scoped(model, context, plan) -> str:
    """The motion suffix a derived id carries, empty when its constraint is shared."""
    if plan.constraint in context.shared_constraints:
        return ""

    return f"_{model.motion_suffix(plan.motion)}"


def _controller_signal_id(model, context, plan) -> str:
    """The scalar output id a controller writes, from its authored command type."""
    graph = model.graph
    controller_id = model.id(plan.controller)
    types = get_node_types(graph, plan.controller)
    command_type = str(graph.value(plan.controller, APP["command-type"]) or "")
    if CSTR_HDL_EXT.FeedForwardController in types:
        return f"cmd_{controller_id}"
    if CSTR_HDL.ImpedanceController in types or command_type == "Force":
        return f"force_{controller_id}"
    target = graph.value(plan.view, MAP.superobject) if plan.view is not None else plan.quantity
    if command_type == "Torque" and KC_STAT.JointPositionCoordinate in get_node_types(
        graph, target
    ):
        return f"tau_{controller_id}"
    algorithm = context.algorithm_by_solver[plan.solver]
    if algorithm.signal_prefix is None:
        raise ValueError(f"Solver '{plan.solver}' does not accept acceleration signals.")

    return (
        f"{algorithm.signal_prefix}_{model.id(plan.quantity)}{_motion_scoped(model, context, plan)}"
    )


def _axis_error(ids: SolverIdFactory, axis: quantities.SpatialAxis) -> Quantity:
    """The per-axis component of a pose difference the controller drives to zero."""
    linear = axis.subspace == "linear-acceleration"

    return _derived_quantity(
        ids.component_error(axis),
        "Length" if linear else "Angle",
        "M" if linear else "RAD",
        has_view=True,
    )


def _axis_derivative(ids: SolverIdFactory, axis: quantities.SpatialAxis) -> Quantity:
    """The per-axis component of the measured velocity feeding a derivative term."""
    linear = axis.subspace == "linear-acceleration"

    return _derived_quantity(
        ids.component_measured_derivative(axis),
        "LinearVelocity" if linear else "AngularVelocity",
        "M_PER_SEC" if linear else "RAD_PER_SEC",
        has_view=True,
    )


def _axis_view(model, superobject, subobject, axis: quantities.SpatialAxis) -> View:
    """The view selecting one axis of a spatial superobject."""
    return View(
        f"view_{subobject.id}",
        superobject,
        subobject,
        axis.half,
        quantities.AXIS_BY_NAME.get(axis.frame_axis),
        direction=(
            quantities.direction(model, axis.direction) if axis.direction is not None else None
        ),
    )


def _saturation_for_signal(model, node, signal) -> Saturation | None:
    """Authored saturation bounds, bound to a derived signal."""
    if node is None:
        return None
    bounds = [
        model.graph.value(node, predicate)
        for predicate in (
            ALGO_EXT["maximum-absolute-value"],
            ALGO_EXT["lower-bound"],
            ALGO_EXT["upper-bound"],
        )
    ]
    maximum, lower, upper = (
        quantities.quantity(model, bound) if bound is not None else None for bound in bounds
    )

    return Saturation(model.id(node), signal, signal, maximum, lower, upper)


class _Saturations(NamedTuple):
    """The two bounds a controller may author, both on its derived control signal."""

    output: Saturation | None
    integral: Saturation | None


def _controller_saturations(model, controller, signal) -> _Saturations:
    """The authored output and integral bounds, both bound to the derived control signal."""
    nodes = list(model.graph.objects(controller, ALGO_EXT.limits))
    output_node = next(
        (
            node
            for node in nodes
            if (input_node := model.graph.value(node, ALGO_EXT["in"])) is not None
            and model.graph.value(input_node, QUDT_SCHEMA.hasQuantityKind) is not None
        ),
        None,
    )
    integral_node = next((node for node in nodes if node != output_node), None)

    return _Saturations(
        _saturation_for_signal(model, output_node, signal),
        _saturation_for_signal(model, integral_node, signal),
    )


def _whole_controller_signal(model, context, plan, types):
    """The control signal a scalar (non-per-axis) controller writes."""
    graph = model.graph
    signal_id = _controller_signal_id(model, context, plan)
    command_type = str(graph.value(plan.controller, APP["command-type"]) or "")
    if CSTR_HDL_EXT.FeedForwardController in types:
        # A forwarded command carries the commanded quantity's own shape, not a derived one.

        return replace(
            quantities.quantity(model, plan.quantity),
            id=signal_id,
            value=None,
            has_view=False,
            provenance=Provenance(),
            reference_value=None,
        )
    if CSTR_HDL.ImpedanceController in types or command_type == "Force":
        return _derived_quantity(signal_id, "Force", "N")
    if signal_id.startswith("tau_"):
        return _derived_quantity(signal_id, "Torque", "N_M")
    input_kind = context.algorithm_by_solver[plan.solver].drive_input
    if input_kind == MotionDriveInput.CARTESIAN_ACCELERATION and plan.axes:
        return _acceleration_signal(signal_id, plan.axes[0], input_kind)

    return _derived_quantity(signal_id, "AccelerationEnergy", "N_M2_PER_SEC2")


def _derived_controller(model, context, plan, axis: quantities.SpatialAxis | None = None):
    """One controller record: per axis when the command is a pose, singular otherwise."""
    graph = model.graph
    ids = _solver_ids(model, context, plan)
    types = get_node_types(graph, plan.controller)
    measured_source = graph.value(plan.controller, CSTR_HDL["measured-velocity"])
    if axis is not None:
        input_kind = context.algorithm_by_solver[plan.solver].drive_input
        payload_id = (
            ids.component_energy(axis)
            if input_kind == MotionDriveInput.ACCELERATION_ENERGY
            else ids.component_acceleration(axis)
        )
        controller_id = ids.component_controller(axis)
        signal = _acceleration_signal(payload_id, axis, input_kind)
        error = _axis_error(ids, axis)
        measured_derivative = _axis_derivative(ids, axis) if measured_source is not None else None
    else:
        controller_id = model.id(plan.controller)
        signal = _whole_controller_signal(model, context, plan, types)
        error_node = graph.value(plan.controller, CSTR_HDL["error-signal"])
        error = quantities.quantity(model, error_node) if error_node is not None else None
        measured_derivative = (
            quantities.quantity(model, measured_source) if measured_source is not None else None
        )

    output_saturation, integral_saturation = _controller_saturations(model, plan.controller, signal)
    # The band belongs to the constraint, so the logged verdict matches the monitor's.
    band = graph.value(plan.constraint, CSTR_EXT["tolerance"])
    tolerance_id = model.id(band) if band is not None else ""

    if CSTR_HDL.ProportionalIntegralDerivative in types:
        decay = graph.value(plan.controller, CSTR_HDL["decay-rate"])

        return PIDController(
            id=controller_id,
            control_signal=signal,
            error_signal=error,
            measured_derivative=measured_derivative,
            proportional_gain=quantities.required_float(
                model, plan.controller, CSTR_HDL["proportional-gain"]
            ),
            integral_gain=quantities.required_float(
                model, plan.controller, CSTR_HDL["integral-gain"]
            ),
            derivative_gain=quantities.required_float(
                model, plan.controller, CSTR_HDL["derivative-gain"]
            ),
            decay_rate=decay.value if CSTR_HDL.DecayingIntegralTerm in types else None,
            output_saturation=output_saturation,
            integral_saturation=integral_saturation,
            tolerance_id=tolerance_id,
            type=model.id(CSTR_HDL.ProportionalIntegralDerivative),
        )
    if CSTR_HDL.ImpedanceController in types:
        return ImpedanceController(
            id=controller_id,
            control_signal=signal,
            error_signal=error,
            stiffness=quantities.required_float(model, plan.controller, CSTR_HDL.stiffness),
            damping=quantities.required_float(model, plan.controller, CSTR_HDL.damping),
            integral_gain=quantities.optional_float(
                model, plan.controller, CSTR_HDL["integral-gain"]
            ),
            output_saturation=output_saturation,
            tolerance_id=tolerance_id,
            type=model.id(CSTR_HDL.ImpedanceController),
        )
    reference_node = graph.value(plan.controller, CSTR_HDL_EXT["reference-signal"])

    return FeedForwardController(
        id=controller_id,
        control_signal=signal,
        reference_signal=(
            quantities.quantity(model, reference_node) if reference_node is not None else None
        ),
        output_saturation=output_saturation,
        tolerance_id=tolerance_id,
        type=model.id(CSTR_HDL_EXT.FeedForwardController),
    )


def _derived_controllers(model, context, plan) -> list:
    """Expand a pose command per axis; a scalar command stays singular."""
    if len(plan.axes) > 1:
        return [_derived_controller(model, context, plan, axis) for axis in plan.axes]

    return [_derived_controller(model, context, plan)]


@dataclass(frozen=True)
class AccelerationDriver:
    """How one solver input kind names and shapes the per-axis records it accepts."""

    spec_id: object
    payload_id: object
    spec_tag: str
    payload_tag: str
    record: type
    payload_field: str


_ACCELERATION_DRIVERS = {
    MotionDriveInput.ACCELERATION_ENERGY: AccelerationDriver(
        SolverIdFactory.component_constraint,
        SolverIdFactory.component_energy,
        "acc-cstr",
        "eacc",
        AccelerationConstraint,
        "acceleration_energy",
    ),
    MotionDriveInput.CARTESIAN_ACCELERATION: AccelerationDriver(
        SolverIdFactory.component_acceleration_specification,
        SolverIdFactory.component_acceleration,
        "cart-acc",
        "acc",
        CartesianAccelerationSpecification,
        "acceleration",
    ),
}


class _DriverIds(NamedTuple):
    """The record id and the payload id one acceleration driver mints for one axis."""

    spec: str
    payload: str


def _acceleration_driver_ids(model, context, plan, driver, axis, multi_axis) -> _DriverIds:
    """The two ids for one axis, registering them when they derive from a quantity."""
    if multi_axis:
        ids = _solver_ids(model, context, plan)

        return _DriverIds(driver.spec_id(ids, axis), driver.payload_id(ids, axis))
    # Single-axis ids are built off the quantity, so that is what they derive from.
    suffix = _motion_scoped(model, context, plan)
    quantity_id = model.id(plan.quantity)
    ids = []
    for tag in (driver.spec_tag, driver.payload_tag):
        derived_id = f"{tag.replace('-', '_')}_{quantity_id}{suffix}"
        model.register_derived(
            derived_id, str(plan.quantity), f"{tag}{kebab(suffix)}", PROV.wasDerivedFrom
        )
        ids.append(derived_id)

    return _DriverIds(*ids)


def _derived_acceleration_drivers(model, context, plan, input_kind) -> list:
    """One controller's ordered per-axis acceleration records, for its solver's input."""
    driver = _ACCELERATION_DRIVERS[input_kind]
    target = (
        model.graph.value(plan.view, MAP.superobject) if plan.view is not None else plan.quantity
    )
    frame_node = model.graph.value(target, GEOM_COORD["as-seen-by"])
    frame = quantities.frame(model, frame_node) if frame_node is not None else None
    multi_axis = len(plan.axes) > 1
    records = []
    for axis in plan.axes:
        spec_id, payload_id = _acceleration_driver_ids(
            model, context, plan, driver, axis, multi_axis
        )
        records.append(
            driver.record(
                id=spec_id,
                subspace=axis.half,
                axis=quantities.AXIS_BY_NAME.get(axis.frame_axis),
                as_seen_by=frame,
                direction=(
                    quantities.direction(model, axis.direction)
                    if axis.direction is not None
                    else None
                ),
                **{driver.payload_field: _acceleration_signal(payload_id, axis, input_kind)},
            )
        )

    return records


def motion_drivers(model, context, solver: URIRef) -> list:
    """A solver's drivers: the accelerations its controllers imply, plus its authored forces.

    Parameters:
        solver: the `slv:SolverWithInputAndOutput` node whose drivers these are

    Returns:
        one `MotionDrivers` record per authored `slv:motion-drivers` node
    """
    plans = context.controllers_by_solver.get(solver, ())
    input_kind = context.algorithm_by_solver[solver].drive_input
    accelerations = (
        [
            record
            for plan in plans
            for record in _derived_acceleration_drivers(model, context, plan, input_kind)
        ]
        if input_kind in _ACCELERATION_DRIVERS
        else []
    )

    return [
        MotionDrivers(
            id=model.id(node),
            acceleration_constraint=(
                accelerations if input_kind == MotionDriveInput.ACCELERATION_ENERGY else []
            ),
            cartesian_force=[
                cartesian_force_specification(model, force)
                for force in model.graph.objects(node, SLV["cartesian-force"])
            ],
            cartesian_acceleration=(
                accelerations if input_kind == MotionDriveInput.CARTESIAN_ACCELERATION else []
            ),
            joint_force=[
                joint_force_specification(model, force)
                for force in model.graph.objects(node, SLV["joint-force"])
            ],
            has_cartesian_force=bool(list(model.graph.objects(node, SLV["cartesian-force"]))),
        )
        for node in model.graph.objects(solver, SLV["motion-drivers"])
    ]


def augment_closures(model, context, closures: dict) -> None:
    """Replace the graph-expanded controller closures with the authored semantic derivations.

    In place: the controller nodes the operator walk found are removed, and one closure per
    derived controller takes their place, plus the pose-difference evaluator a per-axis command
    needs.
    """
    graph = model.graph
    for plans in context.controllers_by_handler.values():
        for plan in plans:
            ids = _solver_ids(model, context, plan)
            closures.pop(model.id(plan.controller), None)
            for controller in context.controllers_for(plan):
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
            target = graph.value(plan.view, MAP.superobject)
            reference = graph.value(plan.constraint, CSTR["reference-value"])
            reference_view = next(graph.subjects(MAP.subobject, reference), None)
            if reference_view is not None:
                reference = graph.value(reference_view, MAP.superobject)
            closures[ids.pose_evaluator()] = {
                "id": ids.pose_evaluator(),
                "type": "PoseDiffEvaluator",
                "in1": model.id(target),
                "in2": model.id(reference),
                "out": ids.pose_difference(),
                # Only the difference is computed at runtime; without materializing its per-axis
                # views the progress gate and the logged quantities read a never-written 0.0.
                "errors": [ids.component_error(axis) for axis in plan.axes],
            }


def augment_data(model, context, data: list, views: dict) -> None:
    """Add the runtime quantities and views a pose command implies, without RDF materialization.

    In place, on both lists: the pose differences and their per-axis error and measured-derivative
    components, the views selecting each axis, and every controller's control signal.
    """
    graph = model.graph
    differences, errors, derivatives, signals = [], [], [], []
    derived_ids = set()
    for plans in context.controllers_by_handler.values():
        for plan in plans:
            controllers = context.controllers_for(plan)
            signals.extend(controller.control_signal for controller in controllers)
            derived_ids.update(controller.control_signal.id for controller in controllers)
            if len(plan.axes) <= 1:
                continue
            ids = _solver_ids(model, context, plan)
            target = graph.value(plan.view, MAP.superobject)
            difference = PoseDifference(
                ids.pose_difference(),
                ["Angle", "Length"],
                Point(f"point_{ids.pose_difference()}_origin"),
                quantities.frame(model, graph.value(target, GEOM_COORD["as-seen-by"])),
                ["M", "RAD"],
                provenance=Provenance(),
            )
            differences.append(difference)
            derived_ids.add(difference.id)
            measured_source = graph.value(plan.controller, CSTR_HDL["measured-velocity"])
            for axis in plan.axes:
                error = _axis_error(ids, axis)
                errors.append(error)
                derived_ids.add(error.id)
                # Re-inserted rather than assigned: position in `views` is the emission order.
                views.pop(error.id, None)
                views[error.id] = _axis_view(model, difference, error, axis)
                if measured_source is None:
                    continue
                derivative = _axis_derivative(ids, axis)
                derivatives.append(derivative)
                derived_ids.add(derivative.id)
                views.pop(derivative.id, None)
                views[derivative.id] = _axis_view(
                    model, quantities.velocity_twist(model, measured_source), derivative, axis
                )

    data[:] = [item for item in data if item.id not in derived_ids]
    wrench_index = next(
        (index for index, item in enumerate(data) if item.type == "Wrench"), len(data)
    )
    data[wrench_index:wrench_index] = differences
    data.extend(errors)
    data.extend(derivatives)
    data.extend(signals)


def annotate_controller_signals(controllers, closures: dict) -> None:
    """Fold the measured and setpoint signal ids onto each controller record.

    Abstract ids only, taken from the error-evaluator closure that feeds the controller; the C++
    access expression is the view's to render.
    """
    error_sources = {
        closure["error"]: closure
        for closure in closures.values()
        if closure.get("type") == "ErrorEvaluator" and closure.get("error")
    }
    for controller in controllers:
        error = getattr(controller, "error_signal", None)
        source = error_sources.get(error if isinstance(error, str) else getattr(error, "id", None))
        source = source or {}
        reference = getattr(controller, "reference_signal", None)
        setpoint_id = (
            reference
            if isinstance(reference, str)
            else getattr(reference, "id", None) or source.get("reference_value")
        )
        if source.get("quantity"):
            controller.measured_signal = source["quantity"]
        if setpoint_id:
            controller.setpoint_signal = setpoint_id


@dataclass(frozen=True)
class StateField:
    """One value a stateful controller keeps between ticks, and how its step call reads it."""

    name: str
    type: str
    getter: str


# Per controller type, the internal state it keeps. A controller type absent here is stateless.
_PID_STATE = (
    StateField("error_integral", "Quantity", "error_integral"),
    StateField("previous_error", "Quantity", "previous_error"),
    StateField("first_sample", "Bool", "is_first_sample"),
)
CONTROLLER_STATE_FIELDS = {
    "ProportionalIntegralDerivative": _PID_STATE,
    "ImpedanceController": _PID_STATE,
}

# Per controller type, the gains struct: the field name the step call reads, the record field it
# comes from, and whether the model must author it. Order is the struct's field order.
CONTROLLER_GAIN_FIELDS = {
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
ADMITTANCE_PARAMETERS = ("mass", "damping", "stiffness", "maximum_velocity")

# The signals a controller binds, in the order their introspection rows are emitted.
CONTROLLER_SIGNAL_ROLES = (
    "error_signal",
    "reference_signal",
    "measured_derivative",
    "control_signal",
)


def saturation(model, node) -> Saturation:
    """An authored `algo-ext:Saturation`: the signal it limits and the bounds it applies.

    Raises:
        ConstraintViolation: the node is not a Saturation.
    """
    model.expect_type(node, ALGO_EXT.Saturation)
    graph = model.graph
    bounds = [
        graph.value(node, predicate)
        for predicate in (
            ALGO_EXT["maximum-absolute-value"],
            ALGO_EXT["lower-bound"],
            ALGO_EXT["upper-bound"],
        )
    ]
    maximum, lower, upper = (
        quantities.quantity(model, bound) if bound is not None else None for bound in bounds
    )

    return Saturation(
        model.id(node),
        quantities.quantity(model, graph.value(node, ALGO_EXT["in"])),
        quantities.quantity(model, graph.value(node, ALGO_EXT.out)),
        maximum,
        lower,
        upper,
    )


def joint_force_specification(model, node) -> JointForceSpecification:
    """An authored joint-space force: the wrench id it applies and the joint it acts on."""
    model.expect_type(node, SLV["JointForceSpecification"])
    force = model.graph.value(node, SLV["force"])
    joint = model.graph.value(node, SLV["attached-to"])

    return JointForceSpecification(
        model.id(node),
        model.id(force) if force is not None else "",
        model.label(joint) if joint is not None else "",
    )


def cartesian_force_specification(model, node) -> CartesianForceSpecification:
    """An authored Cartesian force: the wrench it applies and the body it acts on."""
    model.expect_type(node, SLV["CartesianForceSpecification"])

    return CartesianForceSpecification(
        model.id(node),
        quantities.wrench(model, model.graph.value(node, SLV["force"])),
        quantities.simplicial_complex(model, model.graph.value(node, SLV["attached-to"])),
    )


def authored_motion_drivers(model, node) -> MotionDrivers:
    """The drivers a solver authors directly, before its controllers are derived."""
    model.expect_type(node, SLV["MotionDrivers"])
    cartesian = [
        cartesian_force_specification(model, force)
        for force in model.graph[node : SLV["cartesian-force"]]
    ]

    return MotionDrivers(
        id=model.id(node),
        acceleration_constraint=[],
        cartesian_force=cartesian,
        joint_force=[
            joint_force_specification(model, force)
            for force in model.graph[node : SLV["joint-force"]]
        ],
        has_cartesian_force=bool(cartesian),
    )
