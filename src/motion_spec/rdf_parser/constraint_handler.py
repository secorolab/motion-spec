# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The control law: authored controllers become per-axis control records.

In order: the solver families and what each may be driven by, the derived ids, the derived ids and the
IRIs they register, the controller records themselves, the functions and data they imply, and last
the tables that say what state and which gains a controller type carries.

This is the seam a controller DSL replaces. Its inputs are the model, the functions and the data
structures; its outputs are the `SolverDerivationContext` and the controller and driver records.
No other module derives a controller, and nothing here appends to the frame log: what a controller
contributes to telemetry is exported as a table for `communication.py` to read.
"""

from __future__ import annotations

import collections
from dataclasses import dataclass, field, replace
from typing import NamedTuple

from motion_spec_dsl.vocab import (
    ALGO_EXT,
    APP,
    CSTR,
    CSTR_EXT,
    CSTR_HDL,
    CSTR_HDL_EXT,
    GEOM_COORD,
    GEOM_OP,
    GEOM_OP_EXT,
    KC_OP,
    KC_OP_EXT,
    KC_STAT,
    MAP,
    MAP_EXT,
    MOT,
    QUDT_SCHEMA,
    RBDYN_OP,
    RBDYN_OP_EXT,
    SLV,
    SLV_EXT,
)
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import ModelBase, get_node_types
from rdf_utils.models.geom_coord import get_coord_vectorxyz
from rdf_utils.namespace import NS_MM_QUDT_QTY, NS_MM_QUDT_UNIT
from rdflib import Literal, URIRef
from rdflib.namespace import PROV, RDF, split_uri
from scene_dsl.rdf_parser.common import ensure_one_obj_uri, ensure_one_typed_subject_uri

from motion_spec.classes.dynamics import Saturation
from motion_spec.classes.geometry import Point, PoseDifference, Subspace, View
from motion_spec.classes.handlers import (
    FeedForwardController,
    ImpedanceController,
    PIDController,
    StateField,
)
from motion_spec.classes.motion import DataValue
from motion_spec.classes.qudt import Provenance, Quantity, QuantityKind, Unit
from motion_spec.classes.solvers import (
    AccelerationConstraint,
    CartesianForceSpecification,
    JointForceSpecification,
    MotionDrivers,
    PoseTarget,
    SolverFamily,
)
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.model import identifier, kebab, local_name

_MOBILE_PLATFORM = SolverFamily()

# an unmapped algorithm is unsupported
SOLVER_FAMILIES = {
    SLV["AccelerationConstrainedHybridDynamicsAlgorithm"]: SolverFamily(
        name="ACHD",
        driver_field="acceleration_constraint",
        payload_field="acceleration_energy",
        payload_kinds={
            Subspace.Linear: ("AccelerationEnergy", "N-M2-PER-SEC2"),
            Subspace.Angular: ("AccelerationEnergy", "N-M2-PER-SEC2"),
        },
        max_axes=6,
        axes_must_be_distinct=True,
    ),
    SLV["RecursiveNewtonEulerAlgorithm"]: SolverFamily(
        name="RNE",
        driver_field="cartesian_acceleration",
        payload_field="acceleration",
        payload_kinds={
            Subspace.Linear: ("LinearAcceleration", "M-PER-SEC2"),
            Subspace.Angular: ("AngularAcceleration", "RAD-PER-SEC2"),
        },
    ),
    KC_OP.ForwardPositionKinematics: SolverFamily(name="FPK", read_only=True),
    KC_OP_EXT.ForwardVelocityKinematics: SolverFamily(name="FVK", read_only=True),
    KC_OP_EXT.InverseVelocityKinematics: SolverFamily(
        name="IVK",
        driver_field="cartesian_velocity",
        payload_field="velocity",
        payload_kinds={
            Subspace.Linear: ("LinearVelocity", "M-PER-SEC"),
            Subspace.Angular: ("AngularVelocity", "RAD-PER-SEC"),
        },
        command_mode="position",
    ),
    KC_OP_EXT.InversePositionKinematics: SolverFamily(name="IPK", command_mode="position"),
    SLV_EXT.CommandForwardingSolver: SolverFamily(forwards_commands=True),
    SLV.VelocityCompositionSolver: _MOBILE_PLATFORM,
    SLV.ForceDistributionSolver: _MOBILE_PLATFORM,
    SLV_EXT.VelocityDistributionSolver: _MOBILE_PLATFORM,
    SLV_EXT.ForceCompositionSolver: _MOBILE_PLATFORM,
}


def solver_algorithm(model, solver: URIRef) -> SolverFamily:
    """The family a solver belongs to, and so what may be derived against it.

    Raises:
        ConstraintViolation: the solver names an algorithm no backend implements.
    """
    for type_ in get_node_types(model.graph, solver):
        if type_ in SOLVER_FAMILIES:
            return SOLVER_FAMILIES[type_]
    algorithm = model.graph.value(solver, SLV["solver"])
    if algorithm is None:
        raise ConstraintViolation("solver", f"solver '{solver}' names no algorithm")
    if algorithm not in SOLVER_FAMILIES:
        raise ConstraintViolation(
            "solver", f"Solver '{solver}' has unsupported algorithm '{algorithm}'."
        )

    return SOLVER_FAMILIES[algorithm]


@dataclass(frozen=True)
class SolverIdFactory:
    """The ids one authored controller implies. Each id registers its IRI as it is minted -- the
    id leads with its tag (`eacc_<ctrl>_<axis>`), the IRI with the controller
    (`<ctrl-iri>/eacc-<axis>`) -- so an id nothing asks for names nothing.
    """

    model: object
    controller: URIRef

    def component_controller(self, axis: quantities.SpatialAxis) -> str:
        id_ = f"{self.model.id(self.controller)}_{axis.suffix}"
        types = tuple(get_node_types(self.model.graph, self.controller))
        self.model.register_derived(
            id_, str(self.controller), kebab(axis.suffix), PROV.specializationOf, types=types
        )
        return id_

    def component_error(self, axis: quantities.SpatialAxis) -> str:
        id_ = f"{self.model.id(self.controller)}_err_{axis.suffix}"
        self.model.register_derived(
            id_, str(self.controller), f"err-{kebab(axis.suffix)}", PROV.wasDerivedFrom
        )
        return id_

    def component_energy(self, axis: quantities.SpatialAxis) -> str:
        id_ = f"eacc_{self.model.id(self.controller)}_{axis.suffix}"
        self.model.register_derived(
            id_, str(self.controller), f"eacc-{kebab(axis.suffix)}", PROV.wasDerivedFrom
        )
        return id_

    def component_acceleration(self, axis: quantities.SpatialAxis) -> str:
        id_ = f"acc_{self.model.id(self.controller)}_{axis.suffix}"
        self.model.register_derived(
            id_, str(self.controller), f"acc-{kebab(axis.suffix)}", PROV.wasDerivedFrom
        )
        return id_

    def component_constraint(self, axis: quantities.SpatialAxis) -> str:
        id_ = f"acc_cstr_{self.model.id(self.controller)}_{axis.suffix}"
        self.model.register_derived(
            id_,
            str(self.controller),
            f"acc-cstr-{kebab(axis.suffix)}",
            PROV.wasDerivedFrom,
            types=(SLV.AccelerationConstraint,),
        )
        return id_

    def component_acceleration_specification(self, axis: quantities.SpatialAxis) -> str:
        id_ = f"cart_acc_{self.model.id(self.controller)}_{axis.suffix}"
        self.model.register_derived(
            id_,
            str(self.controller),
            f"cart-acc-{kebab(axis.suffix)}",
            PROV.wasDerivedFrom,
            types=(SLV.AccelerationConstraint,),
        )
        return id_

    def component_velocity(self, axis: quantities.SpatialAxis) -> str:
        id_ = f"vel_{self.model.id(self.controller)}_{axis.suffix}"
        self.model.register_derived(
            id_, str(self.controller), f"vel-{kebab(axis.suffix)}", PROV.wasDerivedFrom
        )
        return id_

    def component_velocity_specification(self, axis: quantities.SpatialAxis) -> str:
        id_ = f"cart_vel_{self.model.id(self.controller)}_{axis.suffix}"
        self.model.register_derived(
            id_, str(self.controller), f"cart-vel-{kebab(axis.suffix)}", PROV.wasDerivedFrom
        )
        return id_

    def component_measured_derivative(self, axis: quantities.SpatialAxis) -> str:
        id_ = f"{self.model.id(self.controller)}_measured_derivative_{axis.suffix}"
        self.model.register_derived(
            id_,
            str(self.controller),
            f"measured-derivative-{kebab(axis.suffix)}",
            PROV.wasDerivedFrom,
        )
        return id_

    def pose_evaluator(self) -> str:
        id_ = f"eval_pose_diff_{self.model.id(self.controller)}"
        self.model.register_derived(
            id_, str(self.controller), "eval-pose-diff", PROV.wasDerivedFrom
        )
        return id_

    def pose_difference(self) -> str:
        id_ = f"pose_diff_{self.model.id(self.controller)}"
        self.model.register_derived(
            id_,
            str(self.controller),
            "pose-diff",
            PROV.wasDerivedFrom,
            types=(GEOM_COORD.PoseDifferenceCoordinate,),
        )
        return id_


_CONTROLLER_TYPES = (
    CSTR_HDL.ImpedanceController,
    CSTR_HDL.ProportionalIntegralDerivative,
    CSTR_HDL_EXT.FeedForwardController,
)
_TARGET_TYPES = (GEOM_COORD.PoseCoordinate, KC_STAT.JointPositionCoordinate)


def is_moment_plan(model, plan) -> bool:
    """Whether a controller commands a Cartesian moment: an `f_ext` couple, not a solver row."""
    graph = model.graph
    if str(graph.value(plan.controller, APP["command-type"]) or "") != "Torque":
        return False
    target = graph.value(plan.view, MAP.superobject) if plan.view is not None else plan.quantity
    return KC_STAT.JointPositionCoordinate not in get_node_types(graph, target)


class ControlDirections(NamedTuple):
    """What one controller's constraint contributes to its solver: the Cartesian directions, the
    view they are taken in, the shared norm its error is divided by, and the frame its gradient is
    stated in. Only an expression constraint sets the last two.
    """

    axes: tuple
    view: URIRef | None = None
    gradient_norm: URIRef | None = None
    gradient_frame: URIRef | None = None


# a commutative operation repeats one operand predicate
_EXPRESSION_OPS = {
    ALGO_EXT.Addition: ("add", (ALGO_EXT["in"],)),
    ALGO_EXT.Subtraction: ("subtract", (ALGO_EXT["minuend"], ALGO_EXT["subtrahend"])),
    ALGO_EXT.Multiplication: ("multiply", (ALGO_EXT["in"],)),
    ALGO_EXT.Division: ("divide", (ALGO_EXT["dividend"], ALGO_EXT["divisor"])),
}


def expression_operation(graph, node):
    """The arithmetic operation writing `node`, as `(name, operands)`, or None when `node` is an
    ordinary quantity nothing computes.
    """
    written = [
        (op, kinds)
        for op in graph.subjects(ALGO_EXT.out, node)
        if (kinds := get_node_types(graph, op) & _EXPRESSION_OPS.keys())
    ]
    if not written:
        return None
    if len(written) > 1 or len(written[0][1]) > 1:
        raise ConstraintViolation(
            "control",
            f"'{node}' is written by {len(written)} arithmetic operations -- an expression node "
            "has one operation of one kind",
        )
    op, (type_,) = written[0]
    name, predicates = _EXPRESSION_OPS[type_]
    if len(predicates) == 1:
        return name, list(graph.objects(op, predicates[0]))
    return name, [graph.value(op, predicate) for predicate in predicates]


def expression_subspace(model, node) -> str | None:
    """The half an expression's moved terms lie in, read off the first one it reaches: `distance`
    along a line, `rotation` about an axis; None where nothing under NODE moves.

    The model states one half per controlled expression, so the first term speaks for all.
    """
    graph = model.graph
    operation = expression_operation(graph, node)
    if operation is not None:
        return next(
            (half for operand in operation[1] if (half := expression_subspace(model, operand))),
            None,
        )
    subspace = operator_gradient(model, node)[1]
    if subspace is not None:
        return subspace
    view = quantities.view_of(graph, node)
    if view is None:
        return None
    subspace_node = graph.value(view, MAP.subspace)
    angular = subspace_node is not None and quantities.subspace(subspace_node) == Subspace.Angular
    return _GRADIENT_OUTPUTS[GEOM_OP["angle"] if angular else GEOM_OP["distance"]]


def view_axes(model, controller, constraint, view_node, target_node, *, subspace=None, axis=None):
    """The Cartesian directions one view of a driven quantity contributes to `controller`.

    Every branch of the axis decision asks the same question of one view; an expression asks it
    once per measured leaf instead of once for the whole constraint. `subspace`/`axis` name the
    half a quantity is driven in when it has no view to read them off, and are ignored otherwise.
    """
    graph = model.graph
    target = target_node
    if view_node is not None:
        target = graph.value(view_node, MAP.superobject)
        subspace = local_name(graph.value(view_node, MAP.subspace))
        axis = local_name(graph.value(view_node, MAP.axis))
    command_type = graph.value(controller, APP["command-type"])
    controller_types = get_node_types(graph, controller)
    target_types = get_node_types(graph, target)

    return quantities.spatial_axes(
        controller_type=next(
            (type_ for type_ in _CONTROLLER_TYPES if type_ in controller_types), None
        ),
        subspace=subspace,
        axis=axis,
        command_type=str(command_type) if command_type is not None else None,
        relation=(
            CSTR.EqualityConstraint
            if CSTR.EqualityConstraint in get_node_types(graph, constraint)
            else None
        ),
        target_type=next((type_ for type_ in _TARGET_TYPES if type_ in target_types), None),
    )


# a distance moves along its gradient, an angle turns about it
_GRADIENT_OUTPUTS = {GEOM_OP["distance"]: "distance", GEOM_OP["angle"]: "rotation"}


def operator_gradient(model, quantity):
    """`(gradient, subspace, moment, norm)`: the direction `quantity` is driven along, the
    subspace that row sits in, its angular companion, and the shared norm its error is divided by.

    Written by the operator that writes the scalar itself; for a direction pair held off zero,
    whose angle comes from a `PlanarAngleFromDirections` with no gradient output of its own, by the
    `AngleGradientFromDirections` op paired with it. A controlled expression's top operator names
    the unit gradient its terms combine to, and the norm that unit was taken from; a geometric
    operator's gradient is unit already, so it names none.
    """
    graph = model.graph
    written = [
        (op, subspace)
        for predicate, subspace in _GRADIENT_OUTPUTS.items()
        for op in graph.subjects(predicate, quantity)
        if (op, GEOM_OP_EXT["gradient"], None) in graph
    ]
    expression = [
        op
        for op in graph.subjects(ALGO_EXT.out, quantity)
        if (op, GEOM_OP_EXT["gradient"], None) in graph
    ]
    if len(written) + len(expression) > 1:
        raise ConstraintViolation(
            "control",
            f"'{quantity}' is written by {len(written) + len(expression)} operators with a "
            "gradient -- one writes it",
        )
    if expression:
        op = expression[0]
        return (
            graph.value(op, GEOM_OP_EXT["gradient"]),
            expression_subspace(model, quantity),
            graph.value(op, GEOM_OP_EXT["gradient-moment"]),
            graph.value(op, GEOM_OP_EXT["norm"]),
        )
    if written:
        op, subspace = written[0]
        return (
            graph.value(op, GEOM_OP_EXT["gradient"]),
            subspace,
            graph.value(op, GEOM_OP_EXT["gradient-moment"]),
            None,
        )
    op = alignment_op(model, GEOM_OP_EXT.AngleGradientFromDirections, quantity)
    if op is not None:
        return (
            graph.value(op, GEOM_OP_EXT["gradient"]),
            _GRADIENT_OUTPUTS[GEOM_OP["angle"]],
            None,
            None,
        )
    return None, None, None, None


def authored_controller_axes(model) -> dict:
    """Cartesian directions per authored controller, derived only from authored facts."""
    graph = model.graph
    speed_by_direction = {
        graph.value(op, GEOM_OP["direction"]): graph.value(op, GEOM_OP_EXT["along-speed"])
        for op in graph.subjects(RDF.type, GEOM_OP_EXT.TwistToLinearVelocityAlong)
    }
    projections = {}
    for frame in graph.subjects(RDF.type, GEOM_OP_EXT.PathTangentFrame):
        roles = {
            role: graph.value(frame, GEOM_OP_EXT[role])
            for role in ("tangent", "normal-a", "normal-b")
        }
        roles["along-speed"] = speed_by_direction.get(roles["tangent"])
        projections[graph.value(frame, GEOM_OP_EXT.path)] = roles
    result = {}
    for controller in set(graph.objects(None, CSTR_HDL.controllers)):
        constraint = graph.value(controller, CSTR_HDL.constraint)
        quantity = graph.value(constraint, CSTR.quantity) if constraint is not None else None
        if constraint is None or quantity is None:
            continue
        view = quantities.view_of(graph, quantity)
        subspace = graph.value(view, MAP.subspace) if view is not None else None
        rotation_op = alignment_op(model, GEOM_OP_EXT.RotationVectorFromDirections, quantity)
        if rotation_op is not None:
            reference = graph.value(rotation_op, GEOM_OP.in2)
            components = [graph.value(reference, GEOM_COORD[axis]) for axis in "xyz"]
            if all(value is not None for value in components):
                # the reference axis's rotation-vector component is always zero
                free = max(range(3), key=lambda i: abs(float(components[i])))
                result[controller] = ControlDirections(
                    tuple(
                        quantities.SpatialAxis(
                            subspace=quantities.Subspace.Angular, axis=axis, direction=None
                        )
                        for index, axis in enumerate("xyz")
                        if index != free
                    ),
                    view,
                )
                continue
        path = graph.value(constraint, GEOM_OP_EXT.path)
        if path is not None:
            # tangent is timing; the normals hold the frame on the path
            outputs = projections[path]
            if quantity == outputs["along-speed"]:
                axes = (quantities.SpatialAxis(Subspace.Linear, "tangent", outputs["tangent"]),)
            elif subspace == MAP_EXT.position:
                axes = (
                    quantities.SpatialAxis(Subspace.Linear, "normal_a", outputs["normal-a"]),
                    quantities.SpatialAxis(Subspace.Linear, "normal_b", outputs["normal-b"]),
                )
            elif subspace == MAP_EXT.orientation:
                axes = quantities.ANGULAR_AXES
            else:
                raise ConstraintViolation(
                    "control",
                    f"Path-following constraint on '{quantity}' must control the speed along the "
                    "path, its position, or its orientation.",
                )
            result[controller] = ControlDirections(axes, view)
            continue
        gradient, gradient_subspace, moment, norm = operator_gradient(model, quantity)
        if gradient is not None:
            # recomputed every cycle, so the row names the shared vector
            axes = view_axes(
                model,
                controller,
                constraint,
                None,
                quantity,
                subspace=gradient_subspace,
                axis="gradient",
            )
            result[controller] = ControlDirections(
                tuple(replace(axis, direction=gradient, moment=moment) for axis in axes),
                None,
                norm,
                graph.value(gradient, GEOM_COORD["as-seen-by"]),
            )
            continue
        if expression_operation(graph, quantity) is not None:
            raise ConstraintViolation(
                "control",
                f"controller '{model.id(controller)}' drives an expression that names no "
                "gradient -- there is no direction to drive it along",
            )
        result[controller] = ControlDirections(
            view_axes(model, controller, constraint, view, quantity), view
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
    # error = gradient_norm * error along the driven direction
    gradient_norm: URIRef | None = None
    gradient_frame: URIRef | None = None


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
    joint_impedance_by_handler: dict = field(default_factory=dict)
    _derived: dict = field(default_factory=dict, compare=False, repr=False)

    def controllers_for(self, plan: ControllerDerivation) -> tuple:
        """The derived per-axis controllers for one authored controller, computed once.

        Memoized so every consumer sees the same records: the controller dataclasses compare by
        identity, and a motion, its handler and the telemetry rows must agree on which object
        each id names.
        """
        if plan not in self._derived:
            self._derived[plan] = (
                tuple(derived_controller(self.model, self, plan, axis) for axis in plan.axes)
                if len(plan.axes) > 1
                else (derived_controller(self.model, self, plan),)
            )
        return self._derived[plan]


def solver_derivation_context(model) -> SolverDerivationContext:
    """Resolve controller ownership and command shape, before any solver node is generated.

    Raises:
        ConstraintViolation: a controller lacks its solver, constraint or quantity, or a handler's
            controllers cannot be assigned to solvers that can run them.
    """
    graph = model.graph
    axes_by_controller = authored_controller_axes(model)
    by_handler = {}
    by_solver = collections.defaultdict(list)
    joint_impedances = {}
    for handler in graph.subjects(RDF.type, CSTR_HDL.ConstraintHandler):
        motion = ensure_one_obj_uri(graph, handler, CSTR_HDL.motion)
        if motion is None:
            raise ConstraintViolation(
                "control", f"Constraint handler '{handler}' is missing its motion."
            )
        authored = sorted(
            set(graph.objects(handler, CSTR_HDL.controllers)),
            key=lambda node: int(graph.value(node, APP.order, default=Literal(0)).value),
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
                raise ConstraintViolation(
                    "control",
                    f"Authored controller '{controller}' needs explicit solver, constraint, "
                    "and constraint quantity relations.",
                )
            if solver_algorithm(model, solver).name == "IPK":
                joint_impedances[(handler, solver)] = controller
                continue
            directions = axes_by_controller.get(controller) or ControlDirections(
                (), quantities.view_of(graph, quantity)
            )
            plan = ControllerDerivation(
                handler,
                motion,
                controller,
                solver,
                constraint,
                quantity,
                directions.view if isinstance(directions.view, URIRef) else None,
                directions.axes,
                directions.gradient_norm,
                directions.gradient_frame,
            )
            plans.append(plan)
            by_solver[solver].append(plan)
        by_handler[handler] = tuple(plans)

    solver_nodes = set(by_solver) | set(graph.subjects(RDF.type, SLV.SolverWithInputAndOutput))
    algorithms = {solver: solver_algorithm(model, solver) for solver in solver_nodes}

    for solver, plans in by_solver.items():
        family = algorithms[solver]
        if family.read_only:
            raise ConstraintViolation(
                "solver",
                f"Solver '{solver}' only reads its chain, but controller "
                f"'{plans[0].controller}' drives it.",
            )
        # moment axes are f_ext directions, not solver rows
        axes = [axis for plan in plans if not is_moment_plan(model, plan) for axis in plan.axes]
        if family.axes_must_be_distinct:
            duplicates = [axis for axis, count in collections.Counter(axes).items() if count > 1]
            if duplicates:
                rendered = ", ".join(f"{axis.subspace}.{axis.axis}" for axis in duplicates)
                raise ConstraintViolation(
                    "solver",
                    f"{family.name} solver '{solver}' repeats acceleration axis: "
                    f"{rendered}.",
                )
        if family.max_axes is not None and len(axes) > family.max_axes:
            raise ConstraintViolation(
                "solver",
                f"{family.name} solver '{solver}' has {len(axes)} axes; at most "
                f"{family.max_axes} are supported.",
            )

    for handler, plans in by_handler.items():
        domains = collections.defaultdict(set)
        for plan in plans:
            command = str(model.graph.value(plan.controller, APP["command-type"]) or "")
            subspace = model.graph.value(plan.view, MAP.subspace) if plan.view else None
            domains[algorithms[plan.solver].name].add(
                "force"
                if command in {"Force", "Torque"} or subspace in {MAP.force, MAP.torque}
                else "pose"
            )
        overlap = domains["ACHD"] & domains["RNE"]
        if overlap:
            raise ConstraintViolation(
                "solver",
                f"Handler '{handler}' assigns ACHD and RNE to the same domain(s): "
                f"{', '.join(overlap)}.",
            )

    return SolverDerivationContext(
        model=model,
        controllers_by_handler=by_handler,
        controllers_by_solver={node: tuple(plans) for node, plans in by_solver.items()},
        algorithm_by_solver=algorithms,
        joint_impedance_by_handler=joint_impedances,
    )


def derived_quantity(id_: str, kind: URIRef, unit: URIRef, *, has_view: bool = False) -> Quantity:
    """A runtime scalar the model implies rather than authors, of a QUDT kind and unit."""
    return Quantity(
        id_,
        QuantityKind(identifier(local_name(kind)), str(kind)),
        Unit(identifier(local_name(unit)), str(unit)),
        None,
        has_view,
        provenance=Provenance(),
    )


def controller_signal_id(model, context, plan) -> tuple[str, bool]:
    """The scalar output id a controller writes, from its authored command type.

    The id is registered here because this is where it is minted, and the only place that knows
    which of the forms below it took -- and so what the signal derives from. An acceleration
    payload is the id the controller's one acceleration driver mints for its axis.

    Returns:
        the id, and whether it was minted as a joint torque command
    """
    graph = model.graph
    controller_id = model.id(plan.controller)
    types = get_node_types(graph, plan.controller)
    command_type = str(graph.value(plan.controller, APP["command-type"]) or "")

    authored = graph.value(plan.controller, CSTR_HDL["control-signal"])
    if authored is not None:
        return model.id(authored), False
    target = graph.value(plan.view, MAP.superobject) if plan.view is not None else plan.quantity
    family = context.algorithm_by_solver[plan.solver]
    signal_id = None
    joint_torque = False
    if is_moment_plan(model, plan):
        signal_id = f"moment_{controller_id}"
    # only a command straight to a device is named `cmd_`
    elif CSTR_HDL.ImpedanceController in types or command_type == "Force":
        signal_id = f"force_{controller_id}"
    elif CSTR_HDL_EXT.FeedForwardController in types and family.payload_field != "velocity":
        signal_id = f"cmd_{controller_id}"
    elif command_type == "Torque" and KC_STAT.JointPositionCoordinate in get_node_types(
        graph, target
    ):
        signal_id = f"tau_{controller_id}"
        joint_torque = True
    if signal_id is not None:
        model.register_derived(signal_id, str(plan.controller), "output", PROV.wasDerivedFrom)
        return signal_id, joint_torque
    if family.driver_field is None:
        raise ConstraintViolation(
            "solver", f"Solver '{plan.solver}' does not accept acceleration signals."
        )
    if not plan.axes:
        raise ConstraintViolation(
            "controller", f"'{plan.controller}' drives no axis for its acceleration to act along"
        )

    return (
        _DRIVER_ID_METHODS[family.name][1](SolverIdFactory(model, plan.controller), plan.axes[0]),
        False,
    )


def axis_error(ids: SolverIdFactory, axis: quantities.SpatialAxis) -> Quantity:
    """The per-axis component of a pose difference the controller drives to zero."""
    linear = axis.subspace == Subspace.Linear
    return derived_quantity(
        ids.component_error(axis),
        NS_MM_QUDT_QTY["Length" if linear else "Angle"],
        NS_MM_QUDT_UNIT["M" if linear else "RAD"],
        has_view=True,
    )


def axis_derivative(ids: SolverIdFactory, axis: quantities.SpatialAxis) -> Quantity:
    """The per-axis component of the measured velocity feeding a derivative term."""
    linear = axis.subspace == Subspace.Linear
    return derived_quantity(
        ids.component_measured_derivative(axis),
        NS_MM_QUDT_QTY["LinearVelocity" if linear else "AngularVelocity"],
        NS_MM_QUDT_UNIT["M-PER-SEC" if linear else "RAD-PER-SEC"],
        has_view=True,
    )


def axis_view(model, superobject, subobject, axis: quantities.SpatialAxis) -> View:
    return View(
        f"view_{subobject.id}",
        superobject,
        subobject,
        axis.subspace,
        quantities.AXIS_BY_NAME.get(axis.frame_axis),
        direction=(
            quantities.direction(model, axis.direction) if axis.direction is not None else None
        ),
    )


SATURATION_BOUNDS = (
    ALGO_EXT["maximum-absolute-value"],
    ALGO_EXT["lower-bound"],
    ALGO_EXT["upper-bound"],
)


def saturation_bounds(model, node) -> list:
    """The maximum-absolute, lower and upper bound a saturation states, None where it states none."""
    bounds = []
    for predicate in SATURATION_BOUNDS:
        bound = model.graph.value(node, predicate)
        bounds.append(quantities.quantity(model, bound) if bound is not None else None)
    return bounds


def saturation_for_signal(model, node, signal) -> Saturation | None:
    if node is None:
        return None
    return Saturation(model.id(node), signal, signal, *saturation_bounds(model, node))


def whole_controller_signal(model, context, plan, types):
    """The control signal a scalar (non-per-axis) controller writes."""
    graph = model.graph
    signal_id, joint_torque = controller_signal_id(model, context, plan)
    command_type = str(graph.value(plan.controller, APP["command-type"]) or "")
    if CSTR_HDL_EXT.FeedForwardController in types:
        return replace(
            quantities.quantity(model, plan.quantity),
            id=signal_id,
            value=None,
            has_view=False,
            provenance=Provenance(),
            reference_value=None,
        )
    if is_moment_plan(model, plan):
        return derived_quantity(signal_id, NS_MM_QUDT_QTY["Torque"], NS_MM_QUDT_UNIT["N-M"])
    if CSTR_HDL.ImpedanceController in types or command_type == "Force":
        return derived_quantity(signal_id, NS_MM_QUDT_QTY["Force"], NS_MM_QUDT_UNIT["N"])
    # only its name marks an authored signal a torque
    if joint_torque or (
        graph.value(plan.controller, CSTR_HDL["control-signal"]) is not None
        and signal_id.startswith("tau_")
    ):
        return derived_quantity(signal_id, NS_MM_QUDT_QTY["Torque"], NS_MM_QUDT_UNIT["N-M"])
    family = context.algorithm_by_solver[plan.solver]
    if family.driver_field is not None and plan.axes:
        return family.payload(signal_id, plan.axes[0].subspace)

    return derived_quantity(
        signal_id, NS_MM_QUDT_QTY["AccelerationEnergy"], NS_MM_QUDT_UNIT["N-M2-PER-SEC2"]
    )


def derived_controller(model, context, plan, axis: quantities.SpatialAxis | None = None):
    """One controller record: per axis when the command is a pose, singular otherwise."""
    graph = model.graph
    ids = SolverIdFactory(model, plan.controller)
    types = get_node_types(graph, plan.controller)
    measured_source = graph.value(plan.controller, CSTR_HDL["measured-velocity"])
    if axis is not None:
        controller_id = ids.component_controller(axis)
        if is_moment_plan(model, plan):
            unit_vector = tuple(float(axis.axis == name) for name in "xyz")
            moments = [
                signal
                for signal in graph.objects(plan.controller, CSTR_HDL["control-signal"])
                for op in graph.subjects(RBDYN_OP_EXT.moment, signal)
                if get_coord_vectorxyz(ModelBase(graph.value(op, RBDYN_OP.direction), graph), graph)
                == unit_vector
            ]
            if len(moments) != 1:
                raise ConstraintViolation(
                    "controller",
                    f"'{plan.controller}' states {len(moments)} moment signals along {axis.suffix}",
                )
            signal = derived_quantity(
                model.id(moments[0]), NS_MM_QUDT_QTY["Torque"], NS_MM_QUDT_UNIT["N-M"]
            )
        else:
            family = context.algorithm_by_solver[plan.solver]
            signal = family.payload(_DRIVER_ID_METHODS[family.name][1](ids, axis), axis.subspace)
        error = axis_error(ids, axis)
        measured_derivative = axis_derivative(ids, axis) if measured_source is not None else None
    else:
        controller_id = model.id(plan.controller)
        signal = whole_controller_signal(model, context, plan, types)
        error_node = graph.value(plan.controller, CSTR_HDL["error-signal"])
        error = quantities.quantity(model, error_node) if error_node is not None else None
        measured_derivative = (
            quantities.quantity(model, measured_source) if measured_source is not None else None
        )

    limits = set(graph.objects(plan.controller, ALGO_EXT.limits))
    output_limits = {
        node
        for node in limits
        if (input_node := graph.value(node, ALGO_EXT["in"])) is not None
        and (input_node, QUDT_SCHEMA.hasQuantityKind, None) in graph
    }
    integral_limits = limits - output_limits
    if len(output_limits) > 1 or len(integral_limits) > 1:
        raise ConstraintViolation(
            "control",
            f"controller '{model.id(plan.controller)}' states {len(output_limits)} output and "
            f"{len(integral_limits)} integral bounds -- it states at most one of each",
        )
    output_saturation = saturation_for_signal(model, next(iter(output_limits), None), signal)
    integral_saturation = saturation_for_signal(model, next(iter(integral_limits), None), signal)
    # the logged verdict must match the monitor's
    band = graph.value(plan.constraint, CSTR_EXT["tolerance"])
    tolerance_id = model.id(band) if band is not None else ""
    gradient_norm = model.id(plan.gradient_norm) if plan.gradient_norm is not None else None

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
            gradient_norm=gradient_norm,
            constraint=model.id(plan.constraint),
            constraint_uri=str(plan.constraint),
            type="ProportionalIntegralDerivative",
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
            gradient_norm=gradient_norm,
            constraint=model.id(plan.constraint),
            constraint_uri=str(plan.constraint),
            type="ImpedanceController",
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
        constraint=model.id(plan.constraint),
        constraint_uri=str(plan.constraint),
        type="FeedForwardController",
    )


_DRIVER_ID_METHODS = {
    "ACHD": (SolverIdFactory.component_constraint, SolverIdFactory.component_energy),
    "RNE": (
        SolverIdFactory.component_acceleration_specification,
        SolverIdFactory.component_acceleration,
    ),
    "IVK": (SolverIdFactory.component_velocity_specification, SolverIdFactory.component_velocity),
}


def motion_drivers(model, context, solver: URIRef, chain) -> list:
    """A solver's drivers: the accelerations or velocities its controllers imply, the pose an
    IPK solver solves for, plus its authored forces.

    Parameters:
        solver: the `slv:SolverWithInputAndOutput` node whose drivers these are
        chain: the `ChainBinding` it runs on, which an IPK target must be the tip of

    Returns:
        one `MotionDrivers` record per authored `slv:motion-drivers` node
    """
    plans = context.controllers_by_solver.get(solver, ())
    family = context.algorithm_by_solver[solver]
    # `MotionDrivers.acceleration_constraint` has no default
    driver_slots = {"acceleration_constraint": [], "cartesian_acceleration": [], "cartesian_velocity": []}
    if family.driver_field is not None:
        spec_method, payload_method = _DRIVER_ID_METHODS[family.name]
        for plan in plans:
            if is_moment_plan(model, plan):
                continue
            target = (
                model.graph.value(plan.view, MAP.superobject)
                if plan.view is not None
                else plan.quantity
            )
            frame_node = model.graph.value(target, GEOM_COORD["as-seen-by"]) or plan.gradient_frame
            frame = quantities.frame(model, frame_node) if frame_node is not None else None
            ids = SolverIdFactory(model, plan.controller)
            for axis in plan.axes:
                spec_id, payload_id = spec_method(ids, axis), payload_method(ids, axis)
                driver_slots[family.driver_field].append(
                    AccelerationConstraint(
                        id=spec_id,
                        subspace=axis.subspace,
                        axis=quantities.AXIS_BY_NAME.get(axis.frame_axis),
                        as_seen_by=frame,
                        direction=(
                            quantities.direction(model, axis.direction)
                            if axis.direction is not None
                            else None
                        ),
                        moment_direction=(
                            quantities.direction(model, axis.moment)
                            if axis.moment is not None
                            else None
                        ),
                        **{family.payload_field: family.payload(payload_id, axis.subspace)},
                    )
                )

    return [
        MotionDrivers(
            id=model.id(node),
            cartesian_force=[
                cartesian_force_specification(model, force)
                for force in model.graph.objects(node, SLV["cartesian-force"])
            ],
            joint_force=[
                joint_force_specification(model, force)
                for force in model.graph.objects(node, SLV["joint-force"])
            ],
            has_cartesian_force=(node, SLV["cartesian-force"], None) in model.graph,
            handler=model.id(model.graph.value(node, PROV.wasDerivedFrom)),
            pose_target=(
                pose_target(model, context, solver, node, chain) if family.name == "IPK" else None
            ),
            **driver_slots,
        )
        for node in model.graph.objects(solver, SLV["motion-drivers"])
    ]


def pose_target(model, context, solver: URIRef, driver: URIRef, chain) -> PoseTarget:
    """What an IPK solver solves for in its handler's motion, and the joint impedance toward it.

    The target is a world pose's `.position` and `.orientation` held equal to one reference
    pose's: the pair its impedance controller drives one of, or, without one, the uncontrolled
    pair of the motion whose pose is on the chain. Both poses are stated in one frame.

    Raises:
        ConstraintViolation: no single such pair, a pose not of the chain tip, or a joint gain for
            a joint the chain does not articulate.
    """
    graph = model.graph
    handler = graph.value(driver, PROV.wasDerivedFrom)
    impedance = context.joint_impedance_by_handler.get((handler, solver))
    parts = collections.defaultdict(dict)
    for constraint in graph.objects(graph.value(handler, CSTR_HDL.motion), MOT["while"]):
        view = graph.value(constraint, CSTR.quantity)
        reference = graph.value(constraint, CSTR["reference-value"])
        subspace = graph.value(view, MAP.subspace)
        measured = graph.value(view, MAP.superobject)
        if (
            (constraint, RDF.type, CSTR.EqualityConstraint) in graph
            and subspace in (MAP_EXT.position, MAP_EXT.orientation)
            and graph.value(view, MAP.axis) is None
            and measured is not None
            and GEOM_COORD.PoseCoordinate in get_node_types(graph, measured)
            and graph.value(reference, MAP.subspace) == subspace
            and (target := graph.value(reference, MAP.superobject)) is not None
        ):
            parts[(measured, target)][subspace] = constraint
    pairs = {key: set(pair.values()) for key, pair in parts.items() if len(pair) == 2}
    if impedance is not None:
        driven = graph.value(impedance, CSTR_HDL.constraint)
        candidates = [key for key, pair in pairs.items() if driven in pair]
    else:
        controlled = {
            graph.value(controller, CSTR_HDL.constraint)
            for controller in graph.objects(handler, CSTR_HDL.controllers)
        }
        candidates = [
            key
            for key, pair in pairs.items()
            if not pair & controlled
            and (of := quantities.pose(model, key[0]).of) is not None
            and of.uri in chain.frames
        ]
    if len(candidates) != 1:
        raise ConstraintViolation(
            "solver",
            f"IPK solver '{model.id(solver)}' finds {len(candidates)} pose targets of its chain tip "
            f"in handler '{model.id(handler)}' -- it solves for exactly one",
        )
    measured, target = candidates[0]
    of = quantities.pose(model, measured).of
    placement = chain.frames.get(of.uri) if of is not None else None
    if placement is None or placement["index"] != chain.tip_segment or placement["offset"] is not None:
        raise ConstraintViolation(
            "solver",
            f"IPK solver '{model.id(solver)}' solves for '{model.id(measured)}', which is not the "
            f"pose of its chain tip '{chain.tip}' -- inverse kinematics solves for the tip",
        )
    gains = {"stiffness": [], "damping": []}
    for name, predicate in (("stiffness", CSTR_HDL.stiffness), ("damping", CSTR_HDL.damping)):
        nodes = list(graph.objects(impedance, predicate)) if impedance is not None else []
        by_joint = {
            split_uri(joint)[1]: float(graph.value(node, QUDT_SCHEMA.value))
            for node in nodes
            if (joint := graph.value(node, KC_STAT["of-joint"])) is not None
        }
        unknown = set(by_joint) - set(chain.joints)
        if unknown or (by_joint and len(by_joint) != len(chain.joints)):
            raise ConstraintViolation(
                "controller",
                f"joint impedance '{model.id(impedance)}' states {name} for joints "
                f"{sorted(by_joint)} -- chain '{chain.name}' articulates {chain.joints}, each once",
            )
        if by_joint:
            gains[name] = [by_joint[joint] for joint in chain.joints]
        elif nodes:
            gains[name] = [float(graph.value(nodes[0], QUDT_SCHEMA.value))] * len(chain.joints)

    return PoseTarget(
        id=f"pose_target_{model.id(driver)}",
        measured=model.id(measured),
        reference=model.id(target),
        **gains,
    )


def augment_functions(model, context, functions: dict) -> None:
    """Replace the graph-expanded controller functions with the authored semantic derivations.

    In place: the controller nodes the operator walk found are removed, and one function per
    derived controller takes their place, plus the pose-difference evaluator a per-axis command
    needs.
    """
    graph = model.graph
    # a joint impedance runs inside the IPK solver
    for controller in context.joint_impedance_by_handler.values():
        functions.pop(model.id(controller), None)
    for plans in context.controllers_by_handler.values():
        for plan in plans:
            functions.pop(model.id(plan.controller), None)
            for controller in context.controllers_for(plan):
                functions.pop(controller.id, None)
                error = controller.error_signal
                reference = (
                    controller.reference_signal
                    if isinstance(controller, FeedForwardController)
                    else None
                )
                derivative = (
                    controller.measured_derivative
                    if isinstance(controller, PIDController)
                    else None
                )
                functions[controller.id] = {
                    "id": controller.id,
                    "type": "Controller",
                    "error_signal": error.id if isinstance(error, Quantity) else None,
                    "reference_signal": reference.id if isinstance(reference, Quantity) else None,
                    "measured_derivative": derivative.id if derivative is not None else None,
                    "control_signal": controller.control_signal.id,
                    "gradient_norm": controller.gradient_norm,
                }
            if len(plan.axes) <= 1:
                continue
            # a banded alignment's op returns only the rotation beyond the cone
            rotation_op = alignment_op(
                model, GEOM_OP_EXT.RotationVectorFromDirections, plan.quantity
            )
            if rotation_op is not None:
                rotation_function = functions.get(model.id(rotation_op))
                if rotation_function is not None:
                    upper = graph.value(plan.constraint, CSTR["upper-threshold"])
                    band = model.id(upper) if upper is not None else None
                    # the ops are shared across motions
                    if "band" in rotation_function and rotation_function["band"] != band:
                        raise ConstraintViolation(
                            "constraint-handler",
                            f"alignment '{rotation_function['id']}' is banded differently by two "
                            f"motions ('{rotation_function['band']}' and '{band}'); state one band "
                            "for it.",
                        )
                    rotation_function["band"] = band
                continue
            target = graph.value(plan.view, MAP.superobject)
            reference = graph.value(plan.constraint, CSTR["reference-value"])
            reference_view = quantities.view_of(graph, reference)
            if reference_view is not None:
                reference = graph.value(reference_view, MAP.superobject)
            ids = SolverIdFactory(model, plan.controller)
            evaluator_id = ids.pose_evaluator()
            functions[evaluator_id] = {
                "id": evaluator_id,
                "type": "PoseDiffEvaluator",
                "in1": model.id(target),
                "in2": model.id(reference),
                "out": ids.pose_difference(),
                # unmaterialized axis views would read a never-written 0.0
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
            ids = SolverIdFactory(model, plan.controller)
            rotation_op = alignment_op(
                model, GEOM_OP_EXT.RotationVectorFromDirections, plan.quantity
            )
            if rotation_op is not None:
                vector = quantities.quantity(model, graph.value(rotation_op, GEOM_OP.out))
                measured_source = graph.value(plan.controller, CSTR_HDL["measured-velocity"])
                for axis in plan.axes:
                    error = axis_error(ids, axis)
                    errors.append(error)
                    derived_ids.add(error.id)
                    views[error.id] = axis_view(model, vector, error, axis)
                    if measured_source is None:
                        continue
                    derivative = axis_derivative(ids, axis)
                    derivatives.append(derivative)
                    derived_ids.add(derivative.id)
                    views.pop(derivative.id, None)
                    views[derivative.id] = axis_view(
                        model, quantities.velocity_twist(model, measured_source), derivative, axis
                    )
                continue
            target = graph.value(plan.view, MAP.superobject)
            difference = PoseDifference(
                id=ids.pose_difference(),
                quantity_kind=[
                    QuantityKind("Angle", str(NS_MM_QUDT_QTY.Angle)),
                    QuantityKind("Length", str(NS_MM_QUDT_QTY.Length)),
                ],
                reference_point=Point(f"point_{ids.pose_difference()}_origin"),
                as_seen_by=quantities.frame(model, graph.value(target, GEOM_COORD["as-seen-by"])),
                unit=["M", "RAD"],
                provenance=Provenance(),
            )
            differences.append(difference)
            derived_ids.add(difference.id)
            measured_source = graph.value(plan.controller, CSTR_HDL["measured-velocity"])
            for axis in plan.axes:
                error = axis_error(ids, axis)
                errors.append(error)
                derived_ids.add(error.id)
                # re-inserted: position in `views` is the emission order
                views.pop(error.id, None)
                views[error.id] = axis_view(model, difference, error, axis)
                if measured_source is None:
                    continue
                derivative = axis_derivative(ids, axis)
                derivatives.append(derivative)
                derived_ids.add(derivative.id)
                views.pop(derivative.id, None)
                views[derivative.id] = axis_view(
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


def annotate_controller_signals(controllers, functions: dict, evaluators=()) -> None:
    """Fold the measured and setpoint signal ids onto each controller record.

    Abstract ids only, taken from the error-evaluator function that feeds the controller; the C++
    access expression is the view's to render.
    """
    error_sources = {
        function["error"]: function
        for function in functions.values()
        if function.get("type") == "ErrorEvaluator" and function.get("error")
    }
    # a deduplicated constraint shares its id across motions
    sources_by_constraint = {
        constraint_id: function
        for evaluator in evaluators
        if evaluator.constraint is not None
        and (constraint_id := evaluator.constraint.id)
        and (function := functions.get(evaluator.id))
    }
    for controller in controllers:
        error = controller.error_signal
        error_id = error.id if isinstance(error, Quantity) else error
        source = error_sources.get(error_id)
        if source is None:
            # a feed-forward controller still logs its evaluator's values
            source = sources_by_constraint.get(controller.constraint)
            if source is not None and error is None and source.get("error"):
                controller.error_signal = source["error"]
        source = source or {}
        reference = (
            controller.reference_signal if isinstance(controller, FeedForwardController) else None
        )
        if isinstance(reference, str):
            setpoint_id = reference
        else:
            reference_id = reference.id if isinstance(reference, Quantity) else None
            setpoint_id = reference_id or source.get("reference_value")
        if source.get("quantity"):
            controller.measured_signal = source["quantity"]
        if setpoint_id:
            controller.setpoint_signal = setpoint_id


# a controller type absent here is stateless
_PID_STATE = (
    StateField("error_integral", "Quantity", "error_integral"),
    StateField("previous_error", "Quantity", "previous_error"),
    StateField("first_sample", "Bool", "is_first_sample"),
)
CONTROLLER_STATE_FIELDS = {
    "ProportionalIntegralDerivative": _PID_STATE,
    "ImpedanceController": _PID_STATE,
}

# (step-call field, record field, required); order is the struct's field order
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


def add_controller_state(model, functions: dict, algorithm_data: list, motions) -> None:
    """Give each stateful controller the D-blocks it keeps between ticks.

    A value the controller integrates is a runtime value like any other: without a slot it cannot
    be reported, and its step call has nowhere to keep it.
    """
    state_by_controller = {
        controller.id: CONTROLLER_STATE_FIELDS[controller.type]
        for motion in motions
        for controller in motion.controllers
        if controller.type in CONTROLLER_STATE_FIELDS
    }
    present = {item.id for item in algorithm_data}
    for function in functions.values():
        fields = (
            state_by_controller.get(function["id"])
            if function.get("type") == "Controller"
            else None
        )
        if not fields:
            continue
        # a per-axis controller is itself derived
        parent = model.iri_of(function["id"])
        if parent is None:
            raise RuntimeError(
                f"controller internal state: '{function['id']}' has no IRI to derive from"
            )
        samples = []
        for state in fields:
            member_id = f"{function['id']}_{state.name}"
            if member_id not in present:
                present.add(member_id)
                algorithm_data.append(
                    DataValue(
                        id=member_id,
                        type=state.type,
                        role="controller_internal_state",
                        controller=function["id"],
                        state=state.name,
                    )
                )
            model.register_derived(member_id, parent, state.name, PROV.wasDerivedFrom)
            samples.append({"id": member_id, "getter": state.getter})
        function["internal_state_samples"] = samples


def add_control_parameters(model, functions: dict, algorithm_data: list, motions) -> None:
    """Give every authored control parameter a D-block the step call reads.

    A gain baked into a constructor can neither be reported nor vary; as a D-block it has a
    producer -- authored, so written once -- and is recorded once for the run.
    """
    controller_by_id = {
        controller.id: controller for motion in motions for controller in motion.controllers
    }
    present = {item.id for item in algorithm_data}
    for function in functions.values():
        # admittance parameters are already D-blocks
        if function.get("type") != "Controller":
            continue
        controller = controller_by_id.get(function["id"])
        gains = CONTROLLER_GAIN_FIELDS.get(controller.type if controller is not None else None)
        if not gains:
            continue
        owner_id = function["id"]
        parent = model.iri_of(owner_id)
        if parent is None:
            raise RuntimeError(f"control parameter: '{owner_id}' has no IRI to derive from")
        function["controller_type"] = controller.type
        function["gains"] = {}
        for name, source, required in gains:
            value = getattr(controller, source)
            if value is None and required:
                raise ConstraintViolation(
                    "control", f"control parameter: '{owner_id}' authors no '{name}'"
                )
            member_id = f"{owner_id}_{name}"
            if member_id not in present:
                present.add(member_id)
                algorithm_data.append(
                    DataValue(
                        id=member_id,
                        type="Quantity",
                        value=float(value) if value is not None else 0.0,
                        role="control_parameter",
                        owner=owner_id,
                        parameter=name,
                    )
                )
            model.register_derived(member_id, parent, name, PROV.wasDerivedFrom)
            function["gains"][name] = member_id
        # a bound left on the controller record never reaches the robot
        function["integral_saturation"] = (
            controller.integral_saturation if isinstance(controller, PIDController) else None
        )
        function["output_saturation"] = controller.output_saturation


def saturation(model, node) -> Saturation:
    """An authored `algo-ext:Saturation`: the signal it limits and the bounds it applies.

    Raises:
        ConstraintViolation: the node is not a Saturation.
    """
    graph = model.graph
    maximum, lower, upper = saturation_bounds(model, node)

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
    force = model.graph.value(node, SLV["force"])
    joint = model.graph.value(node, SLV["attached-to"])
    return JointForceSpecification(
        model.id(node),
        model.id(force) if force is not None else "",
        model.label(joint) if joint is not None else "",
    )


def cartesian_force_specification(model, node) -> CartesianForceSpecification:
    """An authored Cartesian force: the wrench it applies, the body it acts on, and the
    controller it derives from."""
    return CartesianForceSpecification(
        model.id(node),
        quantities.wrench(model, model.graph.value(node, SLV["force"])),
        quantities.simplicial_complex(model, model.graph.value(node, SLV["attached-to"])),
        model.id(model.graph.value(node, PROV.wasDerivedFrom)),
    )


def alignment_chain_ops(model, quantity):
    """The rotate-direction and angle ops producing the alignment angle `quantity`, in
    dependency order, or `[]` when `quantity` is not an alignment angle."""
    graph = model.graph
    angle_op = ensure_one_typed_subject_uri(
        graph, quantity, GEOM_OP.angle, GEOM_OP.PlanarAngleFromDirections
    )
    if angle_op is None:
        return []
    rotate_ops = [
        op
        for direction in graph.objects(angle_op, GEOM_OP["from-directions"])
        for op in graph.subjects(GEOM_OP.to, direction)
        if GEOM_OP.RotateDirectionDistalToProximalWithPose in get_node_types(graph, op)
    ]
    return [*rotate_ops, angle_op]


def alignment_op(model, type_, quantity):
    """The `type_` op paired with `quantity`'s `PlanarAngleFromDirections` angle op -- the one
    reading the same two rotated/reference directions -- or None when `quantity` is not one.
    `RotationVectorFromDirections` drives `quantity` pointwise onto its reference, the 2-DOF row
    shape a cone or a zero target takes; `AngleGradientFromDirections` drives it to a target off
    zero, one axis to turn about, and the angle op it pairs with writes no gradient itself.
    """
    graph = model.graph
    angle_op = ensure_one_typed_subject_uri(
        graph, quantity, GEOM_OP.angle, GEOM_OP.PlanarAngleFromDirections
    )
    if angle_op is None:
        return None
    directions = set(graph.objects(angle_op, GEOM_OP["from-directions"]))
    paired = [
        op
        for op in graph.subjects(RDF.type, type_)
        if {graph.value(op, GEOM_OP.in1), graph.value(op, GEOM_OP.in2)} == directions
    ]
    if len(paired) > 1:
        raise ConstraintViolation(
            "control",
            f"{len(paired)} '{local_name(type_)}' ops read the directions of '{quantity}' -- one "
            "pairs with its angle",
        )
    return paired[0] if paired else None
