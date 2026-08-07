# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Everything controller: authored controller parsing, the per-axis derivations they imply,
their closure records and their gain/parameter emission.

This module is the controller-DSL replacement boundary. A controller consumes named shared
values resolved by reference, and nothing here reaches into graph parsing beyond the controller
subgraph it owns. The surface the rest of the lowering uses:

``_solver_derivation_context(g, iris)`` -> ``SolverDerivationContext``;
``_derived_controllers(g, p, context, plan)`` -> PID/Impedance/FeedForward records;
``_derived_motion_drivers(g, p, context, solver)`` -> ``MotionDrivers``;
``_derive_solver_closures(g, p, context, closures)`` and
``_derive_solver_data(g, p, context, data, views)`` (in-place);
``_annotate_controller_signals``, ``add_controller_internal_state_logging`` and
``add_control_parameters`` (in-place, introspection-facing).

Record shapes -- controller ids, gains, saturation and type; controller closure entries; gain
shared-members with their dataflow contracts -- are the contract; the templates and the runtime
consume only these."""

from __future__ import annotations

import collections
from dataclasses import (
    dataclass, replace,
)
from rdf_utils.models.common import get_node_types
from rdflib import URIRef
from rdflib.namespace import RDF
# fmt: off
from motion_spec.classes.entities import (
    AccelerationConstraint, Axis, CartesianAccelerationSpecification, FeedForwardController,
    ImpedanceController, MotionDrivers, PIDController, Point, PoseDifference, Provenance,
    Quantity, QuantityKind, Saturation, Subspace, Unit, View,
)
# fmt: on
# fmt: off
from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT, APP, CSTR, CSTR_EXT, CSTR_HDL, CSTR_HDL_EXT, GEOM_COORD, GEOM_OP, GEOM_OP_EXT,
    KC_STAT, MAP, QUDT_SCHEMA, SLV,
)
# fmt: on

from motion_spec.rdf_parser.records import (
    _prune,
    _field, _kebab, _set_field, _signal_id,
)
from motion_spec.rdf_parser.graph import (
    _id_ref,
    AccelerationInputKind, DerivedIriRegistry, SolverSemantics, _resolve_solver_semantics,
    _term_name,
)

@dataclass(frozen=True)
class SpatialAxis:
    """One ordered linear or angular Cartesian direction. A path-following direction is known only
    at runtime, so it names the shared vector carrying it instead of a fixed frame axis.
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

    # The id leads with its tag (`eacc_<ctrl>_<axis>`), the IRI with the parent
    # (`<ctrl-iri>/eacc-<axis>`).
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
    """Register every IRI this factory can mint for one controller and its axes: the sites each mint
    a different subset, and a missed one leaves a slot unaddressable. An unused id is inert.
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
    """The directions one path-following constraint controls. A path fixes geometry but not timing,
    so the roles never mix: the tangent is timing, the two normals hold the frame on the path, and
    orientation tracking is the ordinary angular triple.
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
    # Carried here, not threaded through six signatures: the context reaches every site that
    # mints an id, and those are the only places the parent node is still known.
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
def _solver_ids(p, context: SolverDerivationContext, plan: ControllerDerivation) -> SolverIdFactory:
    """Ids for one controller, with its whole derived-IRI family registered."""
    ids = SolverIdFactory(p.id(plan.controller), _motion_suffix(p, plan.motion))
    _register_derived_family(context.iris, plan.controller, ids, plan.axes)
    return ids
def _axis_subspace(axis: SpatialAxis) -> Subspace:
    return Subspace.Linear if axis.subspace == "linear-acceleration" else Subspace.Angular
def _axis_error(ids: SolverIdFactory, axis: SpatialAxis) -> Quantity:
    """The per-axis component of a pose difference the controller drives to zero."""
    linear = axis.subspace == "linear-acceleration"
    return _derived_quantity(
        ids.component_error(axis),
        "Length" if linear else "Angle",
        "M" if linear else "RAD",
        has_view=True,
    )
def _axis_derivative(ids: SolverIdFactory, axis: SpatialAxis) -> Quantity:
    """The per-axis component of the measured velocity feeding a derivative term."""
    linear = axis.subspace == "linear-acceleration"
    return _derived_quantity(
        ids.component_measured_derivative(axis),
        "LinearVelocity" if linear else "AngularVelocity",
        "M_PER_SEC" if linear else "RAD_PER_SEC",
        has_view=True,
    )
def _axis_view(p, superobject, subobject, axis: SpatialAxis) -> View:
    """The view selecting one axis of a spatial superobject."""
    return View(
        f"view_{subobject.id}",
        superobject,
        subobject,
        _axis_subspace(axis),
        _AXIS_BY_FRAME_AXIS.get(axis.frame_axis),
        direction=p.direction(axis.direction) if axis.direction is not None else None,
    )
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
    ids = _solver_ids(p, context, plan)
    input_kind = context.semantics_by_solver[plan.solver].acceleration_input
    types = get_node_types(g, plan.controller)
    if axis is not None:
        controller_id = ids.component_controller(axis)
        payload_id = (
            ids.component_energy(axis)
            if input_kind == AccelerationInputKind.ConstraintEnergy
            else ids.component_acceleration(axis)
        )
        signal = _acceleration_signal(payload_id, axis, input_kind)
        error = _axis_error(ids, axis)
        measured_source = g.value(plan.controller, CSTR_HDL["measured-velocity"])
        measured_derivative = _axis_derivative(ids, axis) if measured_source is not None else None
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
    # The band belongs to the constraint, so the logged verdict matches the monitor's.
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
# Per solver input kind: id-factory kinds for the record and its payload, the IRI tags, the
# record class, and the field the payload fills. Single-axis ids reuse the tags with `-` as `_`.
_ACCELERATION_DRIVERS = {
    AccelerationInputKind.ConstraintEnergy: (
        "component_constraint",
        "component_energy",
        "acc-cstr",
        "eacc",
        AccelerationConstraint,
        "acceleration_energy",
    ),
    AccelerationInputKind.CartesianAcceleration: (
        "component_acceleration_specification",
        "component_acceleration",
        "cart-acc",
        "acc",
        CartesianAccelerationSpecification,
        "acceleration",
    ),
}
def _derived_acceleration_drivers(
    g, p, context, plan: ControllerDerivation, input_kind: AccelerationInputKind
):
    """Build one controller's ordered per-axis acceleration records for its solver's input."""
    spec_kind, payload_kind, spec_tag, payload_tag, record, payload_field = _ACCELERATION_DRIVERS[
        input_kind
    ]
    ids = _solver_ids(p, context, plan)
    target = g.value(plan.view, MAP.superobject) if plan.view is not None else plan.quantity
    frame_node = g.value(target, GEOM_COORD["as-seen-by"])
    frame = p.frame(frame_node) if frame_node is not None else None
    result = []
    for axis in plan.axes:
        if len(plan.axes) > 1:
            spec_id = getattr(ids, spec_kind)(axis)
            payload_id = getattr(ids, payload_kind)(axis)
        else:
            # Single-axis ids are built off the quantity, so that is what they derive from.
            suffix = (
                ""
                if plan.constraint in context.shared_constraints
                else f"_{_motion_suffix(p, plan.motion)}"
            )
            quantity_id = p.id(plan.quantity)
            spec_id = f"{spec_tag.replace('-', '_')}_{quantity_id}{suffix}"
            payload_id = f"{payload_tag.replace('-', '_')}_{quantity_id}{suffix}"
            for derived_id, tag in ((spec_id, spec_tag), (payload_id, payload_tag)):
                context.iris.register(
                    derived_id,
                    str(plan.quantity),
                    f"{tag}{_kebab(suffix)}",
                    DerivedIriRegistry.DERIVATION,
                )
        result.append(
            record(
                id=spec_id,
                subspace=_axis_subspace(axis),
                axis=_AXIS_BY_FRAME_AXIS.get(axis.frame_axis),
                as_seen_by=frame,
                direction=p.direction(axis.direction) if axis.direction is not None else None,
                **{payload_field: _acceleration_signal(payload_id, axis, input_kind)},
            )
        )
    return result
def _derived_motion_drivers(g, p, context, solver: URIRef) -> list[MotionDrivers]:
    """Build a solver's drivers from authored controllers plus authored force specs."""
    plans = context.controllers_by_solver.get(solver, ())
    input_kind = context.semantics_by_solver[solver].acceleration_input
    records = [
        record
        for plan in plans
        for record in _derived_acceleration_drivers(g, p, context, plan, input_kind)
    ] if input_kind in _ACCELERATION_DRIVERS else []
    constraints = records if input_kind == AccelerationInputKind.ConstraintEnergy else []
    accelerations = records if input_kind == AccelerationInputKind.CartesianAcceleration else []
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
            ids = _solver_ids(p, context, plan)
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
            ids = _solver_ids(p, context, plan)
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
            measured_source = g.value(plan.controller, CSTR_HDL["measured-velocity"])
            for axis in plan.axes:
                error = _axis_error(ids, axis)
                errors.append(error)
                derived_ids.add(error.id)
                # Re-inserted rather than assigned: position in `views` is the emission order.
                views.pop(error.id, None)
                views[error.id] = _axis_view(p, difference, error, axis)
                if measured_source is not None:
                    derivative = _axis_derivative(ids, axis)
                    derivatives.append(derivative)
                    derived_ids.add(derivative.id)
                    views.pop(derivative.id, None)
                    views[derivative.id] = _axis_view(
                        p, p.velocity_twist(measured_source), derivative, axis
                    )

    data[:] = [item for item in data if item.id not in derived_ids]
    wrench_index = next(
        (index for index, item in enumerate(data) if item.type == "Wrench"), len(data)
    )
    data[wrench_index:wrench_index] = differences
    data.extend(errors)
    data.extend(derivatives)
    data.extend(signals)
def _annotate_controller_signals(controllers, closures: dict) -> None:
    """Fold the measured/setpoint signal ids onto each controller from its error-evaluator closure.
    Abstract ids only; the C++ access expression is rendered by access-expr (shared_data.stg).
    """
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

    def add_once(rows, seen, item_id: str, item_type: str, controller_id: str, name: str) -> None:
        """Append one controller-internal-state row, at most once per id."""
        if item_id in seen:
            return
        rows.append(
            {
                "id": item_id,
                "type": item_type,
                "controller": controller_id,
                "role": "controller_internal_state",
                "state": name,
            }
        )
        seen.add(item_id)

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
        # Through the registry, not the graph: a per-axis controller is itself derived.
        parent_iri = iris.iri_of(controller_id)
        if parent_iri is None:
            raise RuntimeError(
                f"controller internal state: '{controller_id}' has no IRI to derive from"
            )
        for state_name, item_type, getter in samples:
            item_id = f"{controller_id}_{state_name}"
            add_once(shared_data, shared_ids, item_id, item_type, controller_id, state_name)
            if item_type == "Quantity":
                add_once(quantities, quantity_ids, item_id, "Quantity", controller_id, state_name)
            iris.register(
                item_id, parent_iri, state_name, DerivedIriRegistry.DERIVATION
            )
            closure_samples.append({"id": item_id, "getter": getter})
        closure["internal_state_samples"] = closure_samples
# Gains-struct field name -> the field the parsed controller carries it on, and whether the model
# must author it. Order is the struct's field order; an absent required one must fail.
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
    """Publish every authored control parameter as a shared value the step call reads. A gain baked
    into a constructor can neither be reported nor vary; as a shared value it carries a producer
    (`authored`, so `init`) and lands in the run's header record.
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
        # Both bounds cross here: the parser binds them to the same signal, and a bound that
        # stops at the controller record is a limit the model authored and the robot never sees.
        closure["integral_saturation"] = _field(controller, "integral_saturation")
        closure["output_saturation"] = _field(controller, "output_saturation")


# Row order is the emitted JSON key order.
_CONTROLLER_GAIN_FIELDS = (
    "proportional_gain",
    "integral_gain",
    "derivative_gain",
    "decay_rate",
    "stiffness",
    "damping",
)
_CONTROLLER_SIGNAL_ROLES = (
    "error_signal",
    "reference_signal",
    "measured_derivative",
    "control_signal",
)


def controller_rows(controller, motion_id: str, uri_by_id: dict):
    """The introspection row for one controller, and one row per signal it binds."""
    entry = {
        "id": controller.id,
        "uri": uri_by_id.get(controller.id),
        "motion": motion_id,
        "type": controller.type,
        **{name: getattr(controller, name, None) for name in _CONTROLLER_GAIN_FIELDS},
        "error_signal": _id_ref(getattr(controller, "error_signal", None)),
        "tolerance_signal": getattr(controller, "tolerance_id", "") or None,
        "reference_signal": _id_ref(getattr(controller, "reference_signal", None)),
        "measured_derivative": _id_ref(getattr(controller, "measured_derivative", None)),
        "output_signal": _id_ref(controller.control_signal),
    }
    signals = []
    for role in _CONTROLLER_SIGNAL_ROLES:
        quantity_id = _id_ref(getattr(controller, role, None))
        if quantity_id:
            signals.append(
                _prune(
                    {
                        "id": f"{controller.id}.{role}",
                        "uri": uri_by_id.get(quantity_id),
                        "quantity": quantity_id,
                        "role": role,
                        "owner": controller.id,
                    }
                )
            )
    return _prune(entry), signals
