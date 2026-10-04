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
    KC_OP,
    KC_OP_EXT,
    KC_STAT,
    MAP,
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
from rdflib.namespace import PROV, RDF
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
    SolverFamily,
)
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.model import identifier, kebab, local_name

_MOBILE_PLATFORM = SolverFamily()

# Every solver family the code generator can run, keyed by the term that identifies it -- the
# algorithm a solver names, or the type a command-forwarding or mobile-platform solver carries.
# ACHD (Vereshchagin) drives each constrained direction by an acceleration energy, RNE by the
# Cartesian acceleration itself; forward kinematics only reads its chain. An unmapped algorithm
# (`slv:ArticulatedBodyAlgorithm`) is unsupported.
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
        # One axis of the controller narrows it, and is a controller of the same kind.
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
        return (quantities.SpatialAxis(Subspace.Linear, "tangent", outputs["tangent"]),)
    if subspace == "position":
        return (
            quantities.SpatialAxis(Subspace.Linear, "normal_a", outputs["normal-a"]),
            quantities.SpatialAxis(Subspace.Linear, "normal_b", outputs["normal-b"]),
        )
    if subspace == "orientation":
        return quantities.ANGULAR_AXES
    raise ConstraintViolation(
        "control",
        f"Path-following constraint on '{quantity}' must control the speed along the path, "
        "its position, or its orientation.",
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


def _is_moment_plan(model, plan) -> bool:
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


# The four arithmetic operations an expression tree is built from, each as the operand
# predicates to read it back through. A commutative operation states its operands on one
# repeated predicate; the two ordered ones name each role.
_EXPRESSION_OPS = {
    ALGO_EXT.Addition: ("add", (ALGO_EXT["in"],)),
    ALGO_EXT.Subtraction: ("subtract", (ALGO_EXT["minuend"], ALGO_EXT["subtrahend"])),
    ALGO_EXT.Multiplication: ("multiply", (ALGO_EXT["in"],)),
    ALGO_EXT.Division: ("divide", (ALGO_EXT["dividend"], ALGO_EXT["divisor"])),
}
# The view subspaces a component turns about an axis in; every other one moves along a line.
_ANGULAR_VIEW_SUBSPACES = {"orientation", "angular-velocity", "angular-acceleration", "torque"}


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


def _expression_subspace(model, node) -> str | None:
    """The half an expression's moved terms lie in, read off the first one it reaches: `distance`
    along a line, `rotation` about an axis; None where nothing under NODE moves.

    The model states one half per controlled expression, so the first term speaks for all.
    """
    graph = model.graph
    operation = expression_operation(graph, node)
    if operation is not None:
        return next(
            (half for operand in operation[1] if (half := _expression_subspace(model, operand))),
            None,
        )
    subspace = _operator_gradient(model, node)[1]
    if subspace is not None:
        return subspace
    view = quantities.view_of(graph, node)
    if view is None:
        return None
    angular = local_name(graph.value(view, MAP.subspace)) in _ANGULAR_VIEW_SUBSPACES
    return _GRADIENT_OUTPUTS[GEOM_OP["angle"] if angular else GEOM_OP["distance"]]


def _view_axes(model, controller, constraint, view_node, target_node, *, subspace=None, axis=None):
    """The Cartesian directions one view of a driven quantity contributes to `controller`.

    Every branch of the axis decision asks the same question of one view; an expression asks it
    once per measured leaf instead of once for the whole constraint. `subspace`/`axis` name the
    half a quantity is driven in when it has no view to read them off, and are ignored otherwise.
    """
    graph = model.graph
    target = graph.value(view_node, MAP.superobject) if view_node is not None else target_node
    command_type = graph.value(controller, APP["command-type"])

    return quantities.spatial_axes(
        controller_type=next(
            (
                local_name(type_)
                for type_ in _CONTROLLER_TYPES
                if type_ in get_node_types(graph, controller)
            ),
            "",
        ),
        subspace=local_name(graph.value(view_node, MAP.subspace))
        if view_node is not None
        else subspace,
        axis=local_name(graph.value(view_node, MAP.axis)) if view_node is not None else axis,
        command_type=str(command_type) if command_type is not None else None,
        relation=(
            "EqualityConstraint"
            if CSTR.EqualityConstraint in get_node_types(graph, constraint)
            else ""
        ),
        quantity_kind=next(
            (
                name
                for type_, name in _TARGET_QUANTITY_KINDS
                if type_ in get_node_types(graph, target)
            ),
            None,
        ),
    )


# The output a geometric operator writes its scalar to, and the subspace the gradient paired with
# it drives: a distance moves along its gradient, an angle turns about it.
_GRADIENT_OUTPUTS = {GEOM_OP["distance"]: "distance", GEOM_OP["angle"]: "rotation"}


def _operator_gradient(model, quantity):
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
            _expression_subspace(model, quantity),
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
    op = alignment_gradient_op(model, quantity)
    if op is not None:
        return (
            graph.value(op, GEOM_OP_EXT["gradient"]),
            _GRADIENT_OUTPUTS[GEOM_OP["angle"]],
            None,
            None,
        )
    return None, None, None, None


def operator_gradient_directions(model, controller, constraint, quantity):
    """The one solver row an operator's or an expression's scalar is driven along, or None when
    nothing writes `quantity` with a gradient.

    The direction is recomputed every cycle, so the row names the shared vector carrying it
    instead of a frame axis -- the shape a path-following row already takes.
    """
    graph = model.graph
    gradient, subspace, moment, norm = _operator_gradient(model, quantity)
    if gradient is None:
        return None
    axes = _view_axes(
        model, controller, constraint, None, quantity, subspace=subspace, axis="gradient"
    )
    return ControlDirections(
        tuple(replace(axis, direction=gradient, moment=moment) for axis in axes),
        None,
        norm,
        graph.value(gradient, GEOM_COORD["as-seen-by"]),
    )


def alignment_axes(model, quantity):
    """The angular axes an alignment drives: every axis but the reference direction's own, whose
    rotation-vector component is identically zero. `()` when `quantity` is not an alignment angle.
    """
    rotation_op = alignment_rotation_op(model, quantity)
    if rotation_op is None:
        return ()
    graph = model.graph
    reference = graph.value(rotation_op, GEOM_OP.in2)
    components = [graph.value(reference, GEOM_COORD[axis]) for axis in "xyz"]
    if any(value is None for value in components):
        return ()
    free = max(range(3), key=lambda i: abs(float(components[i])))
    return tuple(
        quantities.SpatialAxis(subspace=quantities.Subspace.Angular, axis=axis, direction=None)
        for index, axis in enumerate("xyz")
        if index != free
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
        view = quantities.view_of(graph, quantity)
        subspace = local_name(graph.value(view, MAP.subspace)) if view is not None else None
        aligned = alignment_axes(model, quantity)
        if aligned:
            result[controller] = ControlDirections(aligned, view)
            continue
        path = graph.value(constraint, GEOM_OP_EXT.path)
        if path is not None:
            result[controller] = ControlDirections(
                _path_following_axes(projections[path], quantity, subspace), view
            )
            continue
        operator_gradient = operator_gradient_directions(model, controller, constraint, quantity)
        if operator_gradient is not None:
            result[controller] = operator_gradient
            continue
        # An expression names no view of its own: only the gradient its operator names says where
        # it is driven.
        if expression_operation(graph, quantity) is not None:
            raise ConstraintViolation(
                "control",
                f"controller '{model.id(controller)}' drives an expression that names no "
                "gradient -- there is no direction to drive it along",
            )
        result[controller] = ControlDirections(
            _view_axes(model, controller, constraint, view, quantity), view
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
    # An expression is driven along its unit gradient, so the error the controller reads is this
    # shared norm times the one along the direction it drives. None for every plain view.
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
    _derived: dict = field(default_factory=dict, compare=False, repr=False)

    def controllers_for(self, plan: ControllerDerivation) -> tuple:
        """The derived per-axis controllers for one authored controller, computed once.

        Memoized so every consumer sees the same records: the controller dataclasses compare by
        identity, and a motion, its handler and the telemetry rows must agree on which object
        each id names.
        """
        if plan not in self._derived:
            self._derived[plan] = tuple(_derived_controllers(self.model, self, plan))
        return self._derived[plan]


def solver_derivation_context(model) -> SolverDerivationContext:
    """Resolve controller ownership and command shape, before any solver node is generated.

    Raises:
        ConstraintViolation: a controller lacks its solver, constraint or quantity, or a handler's
            controllers cannot be assigned to solvers that can run them.
    """
    graph = model.graph
    axes_by_controller = _authored_controller_axes(model)
    by_handler = {}
    by_solver = collections.defaultdict(list)
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
    _validate_solver_derivations(model, by_handler, by_solver, algorithms)

    return SolverDerivationContext(
        model=model,
        controllers_by_handler=by_handler,
        controllers_by_solver={node: tuple(plans) for node, plans in by_solver.items()},
        algorithm_by_solver=algorithms,
    )


def _validate_solver_derivations(model, by_handler, by_solver, algorithms) -> None:
    """Enforce executable solver limits, which only hold once authored RDF has resolved to plans."""
    for solver, plans in by_solver.items():
        family = algorithms[solver]
        if family.read_only:
            raise ConstraintViolation(
                "solver",
                f"Solver '{solver}' only reads its chain, but controller "
                f"'{plans[0].controller}' drives it.",
            )
        # A moment plan's axes are f_ext directions, not solver rows, so they claim no budget.
        axes = [axis for plan in plans if not _is_moment_plan(model, plan) for axis in plan.axes]
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
            subspace = local_name(model.graph.value(plan.view, MAP.subspace)) if plan.view else None
            domains[algorithms[plan.solver].name].add(
                "force"
                if command in {"Force", "Torque"} or subspace in {"force", "torque"}
                else "pose"
            )
        overlap = domains["ACHD"] & domains["RNE"]
        if overlap:
            raise ConstraintViolation(
                "solver",
                f"Handler '{handler}' assigns ACHD and RNE to the same domain(s): "
                f"{', '.join(overlap)}.",
            )


def _derived_quantity(id_: str, kind: URIRef, unit: URIRef, *, has_view: bool = False) -> Quantity:
    """A runtime scalar the model implies rather than authors, of a QUDT kind and unit."""
    return Quantity(
        id_,
        QuantityKind(identifier(local_name(kind)), str(kind)),
        Unit(identifier(local_name(unit)), str(unit)),
        None,
        has_view,
        provenance=Provenance(),
    )


def _controller_signal_id(model, context, plan) -> str:
    """The scalar output id a controller writes, from its authored command type.

    The id is registered here because this is where it is minted, and the only place that knows
    which of the forms below it took -- and so what the signal derives from. An acceleration
    payload is the id the controller's one acceleration driver mints for its axis.
    """
    graph = model.graph
    controller_id = model.id(plan.controller)
    types = get_node_types(graph, plan.controller)
    command_type = str(graph.value(plan.controller, APP["command-type"]) or "")

    # A force or moment command feeds the magnitude of a wrench the model builds, and the model
    # names that magnitude itself.
    authored = graph.value(plan.controller, CSTR_HDL["control-signal"])
    if authored is not None:
        return model.id(authored)
    target = graph.value(plan.view, MAP.superobject) if plan.view is not None else plan.quantity
    signal_id = None
    # A Cartesian moment feeds the moment slot of a wrench, so it is named after that.
    if _is_moment_plan(model, plan):
        signal_id = f"moment_{controller_id}"
    # A force command is named after the wrench magnitude it feeds, whichever control law
    # produced it; only a command that goes straight to a device is named `cmd_`.
    elif CSTR_HDL.ImpedanceController in types or command_type == "Force":
        signal_id = f"force_{controller_id}"
    elif CSTR_HDL_EXT.FeedForwardController in types:
        signal_id = f"cmd_{controller_id}"
    elif command_type == "Torque" and KC_STAT.JointPositionCoordinate in get_node_types(
        graph, target
    ):
        signal_id = f"tau_{controller_id}"
    if signal_id is not None:
        model.register_derived(signal_id, str(plan.controller), "output", PROV.wasDerivedFrom)
        return signal_id
    family = context.algorithm_by_solver[plan.solver]
    if family.driver_field is None:
        raise ConstraintViolation(
            "solver", f"Solver '{plan.solver}' does not accept acceleration signals."
        )
    if not plan.axes:
        raise ConstraintViolation(
            "controller", f"'{plan.controller}' drives no axis for its acceleration to act along"
        )

    return _DRIVER_ID_METHODS[family.name][1](SolverIdFactory(model, plan.controller), plan.axes[0])


def _axis_error(ids: SolverIdFactory, axis: quantities.SpatialAxis) -> Quantity:
    """The per-axis component of a pose difference the controller drives to zero."""
    linear = axis.subspace == Subspace.Linear
    return _derived_quantity(
        ids.component_error(axis),
        NS_MM_QUDT_QTY["Length" if linear else "Angle"],
        NS_MM_QUDT_UNIT["M" if linear else "RAD"],
        has_view=True,
    )


def _axis_derivative(ids: SolverIdFactory, axis: quantities.SpatialAxis) -> Quantity:
    """The per-axis component of the measured velocity feeding a derivative term."""
    linear = axis.subspace == Subspace.Linear
    return _derived_quantity(
        ids.component_measured_derivative(axis),
        NS_MM_QUDT_QTY["LinearVelocity" if linear else "AngularVelocity"],
        NS_MM_QUDT_UNIT["M-PER-SEC" if linear else "RAD-PER-SEC"],
        has_view=True,
    )


def _axis_view(model, superobject, subobject, axis: quantities.SpatialAxis) -> View:
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


def _saturation_for_signal(model, node, signal) -> Saturation | None:
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
    graph = model.graph
    nodes = set(graph.objects(controller, ALGO_EXT.limits))
    # The output bound reads a quantity; the integral bound reads the controller's own state.
    outputs = {
        node
        for node in nodes
        if (input_node := graph.value(node, ALGO_EXT["in"])) is not None
        and (input_node, QUDT_SCHEMA.hasQuantityKind, None) in graph
    }
    integrals = nodes - outputs
    if len(outputs) > 1 or len(integrals) > 1:
        raise ConstraintViolation(
            "control",
            f"controller '{model.id(controller)}' states {len(outputs)} output and "
            f"{len(integrals)} integral bounds -- it states at most one of each",
        )
    output_node = next(iter(outputs), None)
    integral_node = next(iter(integrals), None)

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
    if _is_moment_plan(model, plan):
        return _derived_quantity(signal_id, NS_MM_QUDT_QTY["Torque"], NS_MM_QUDT_UNIT["N-M"])
    if CSTR_HDL.ImpedanceController in types or command_type == "Force":
        return _derived_quantity(signal_id, NS_MM_QUDT_QTY["Force"], NS_MM_QUDT_UNIT["N"])
    if signal_id.startswith("tau_"):
        return _derived_quantity(signal_id, NS_MM_QUDT_QTY["Torque"], NS_MM_QUDT_UNIT["N-M"])
    family = context.algorithm_by_solver[plan.solver]
    if family.driver_field is not None and plan.axes:
        return family.payload(signal_id, plan.axes[0].subspace)

    return _derived_quantity(
        signal_id, NS_MM_QUDT_QTY["AccelerationEnergy"], NS_MM_QUDT_UNIT["N-M2-PER-SEC2"]
    )


def _derived_controller(model, context, plan, axis: quantities.SpatialAxis | None = None):
    """One controller record: per axis when the command is a pose, singular otherwise."""
    graph = model.graph
    ids = SolverIdFactory(model, plan.controller)
    types = get_node_types(graph, plan.controller)
    measured_source = graph.value(plan.controller, CSTR_HDL["measured-velocity"])
    if axis is not None:
        controller_id = ids.component_controller(axis)
        if _is_moment_plan(model, plan):
            # A moment feeds f_ext: its payload is the signal the wrench op along this axis takes.
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
            signal = _derived_quantity(
                model.id(moments[0]), NS_MM_QUDT_QTY["Torque"], NS_MM_QUDT_UNIT["N-M"]
            )
        else:
            family = context.algorithm_by_solver[plan.solver]
            signal = family.payload(_DRIVER_ID_METHODS[family.name][1](ids, axis), axis.subspace)
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


def _derived_controllers(model, context, plan) -> list:
    """Expand a pose command per axis; a scalar command stays singular."""
    if len(plan.axes) > 1:
        return [_derived_controller(model, context, plan, axis) for axis in plan.axes]
    return [_derived_controller(model, context, plan)]


# Per family, the `SolverIdFactory` methods that mint its spec/payload ids.
_DRIVER_ID_METHODS = {
    "ACHD": (
        SolverIdFactory.component_constraint,
        SolverIdFactory.component_energy,
    ),
    "RNE": (
        SolverIdFactory.component_acceleration_specification,
        SolverIdFactory.component_acceleration,
    ),
}


class _DriverIds(NamedTuple):
    """The record id and the payload id one acceleration driver mints for one axis."""

    spec: str
    payload: str


def _acceleration_driver_ids(model, context, plan, family, axis) -> _DriverIds:
    """The two ids for one axis of a controller's acceleration driver."""
    ids = SolverIdFactory(model, plan.controller)
    spec_id, payload_id = _DRIVER_ID_METHODS[family.name]

    return _DriverIds(spec_id(ids, axis), payload_id(ids, axis))


def _derived_acceleration_drivers(model, context, plan, family) -> list:
    """One controller's ordered per-axis acceleration records, for its solver's family."""
    target = (
        model.graph.value(plan.view, MAP.superobject) if plan.view is not None else plan.quantity
    )
    frame_node = model.graph.value(target, GEOM_COORD["as-seen-by"]) or plan.gradient_frame
    frame = quantities.frame(model, frame_node) if frame_node is not None else None
    records = []
    for axis in plan.axes:
        spec_id, payload_id = _acceleration_driver_ids(model, context, plan, family, axis)
        records.append(
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
                    quantities.direction(model, axis.moment) if axis.moment is not None else None
                ),
                **{family.payload_field: family.payload(payload_id, axis.subspace)},
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
    family = context.algorithm_by_solver[solver]
    driven = family.driver_field is not None
    accelerations = (
        [
            record
            for plan in plans
            if not _is_moment_plan(model, plan)
            for record in _derived_acceleration_drivers(model, context, plan, family)
        ]
        if driven
        else []
    )
    # Both slots default empty; only the family's own slot (driver_field) takes the drivers, so
    # `MotionDrivers.acceleration_constraint` -- which has no default -- is always supplied.
    driver_slots = {"acceleration_constraint": [], "cartesian_acceleration": []}
    if driven:
        driver_slots[family.driver_field] = accelerations

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
            has_cartesian_force=bool(list(model.graph.objects(node, SLV["cartesian-force"]))),
            handler=model.id(model.graph.value(node, PROV.wasDerivedFrom)),
            **driver_slots,
        )
        for node in model.graph.objects(solver, SLV["motion-drivers"])
    ]


def _bind_alignment_band(model, plan, function) -> None:
    """Give an alignment's rotation-vector op the angle it is allowed to keep.

    The ops are one set for the whole model, so two motions banding the same alignment
    differently would need two of them; say so rather than let the last one win.

    Raises:
        ConstraintViolation: two motions state different bands for one alignment.
    """
    if function is None:
        return
    upper = model.graph.value(plan.constraint, CSTR["upper-threshold"])
    band = model.id(upper) if upper is not None else None
    if "band" in function and function["band"] != band:
        raise ConstraintViolation(
            "constraint-handler",
            f"alignment '{function['id']}' is banded differently by two motions "
            f"('{function['band']}' and '{band}'); state one band for it.",
        )
    function["band"] = band


def augment_functions(model, context, functions: dict) -> None:
    """Replace the graph-expanded controller functions with the authored semantic derivations.

    In place: the controller nodes the operator walk found are removed, and one function per
    derived controller takes their place, plus the pose-difference evaluator a per-axis command
    needs.
    """
    graph = model.graph
    for plans in context.controllers_by_handler.values():
        for plan in plans:
            ids = SolverIdFactory(model, plan.controller)
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
            # An alignment drives the rotation vector's components; there is no pose pair to
            # difference, and its ops already produce the error. A banded alignment tolerates a
            # cone, so the op carries the band and returns only the rotation beyond it.
            rotation_op = alignment_rotation_op(model, graph.value(plan.constraint, CSTR.quantity))
            if rotation_op is not None:
                _bind_alignment_band(model, plan, functions.get(model.id(rotation_op)))
                continue
            target = graph.value(plan.view, MAP.superobject)
            reference = graph.value(plan.constraint, CSTR["reference-value"])
            reference_view = quantities.view_of(graph, reference)
            if reference_view is not None:
                reference = graph.value(reference_view, MAP.superobject)
            functions[ids.pose_evaluator()] = {
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
            ids = SolverIdFactory(model, plan.controller)
            rotation_op = alignment_rotation_op(model, graph.value(plan.constraint, CSTR.quantity))
            if rotation_op is not None:
                vector = quantities.quantity(model, graph.value(rotation_op, GEOM_OP.out))
                measured_source = graph.value(plan.controller, CSTR_HDL["measured-velocity"])
                for axis in plan.axes:
                    error = _axis_error(ids, axis)
                    errors.append(error)
                    derived_ids.add(error.id)
                    views[error.id] = _axis_view(model, vector, error, axis)
                    # An alignment error is a rotation vector, so its derivative is the measured
                    # angular velocity on the same axis -- the pose-difference branch's term.
                    if measured_source is None:
                        continue
                    derivative = _axis_derivative(ids, axis)
                    derivatives.append(derivative)
                    derived_ids.add(derivative.id)
                    views.pop(derivative.id, None)
                    views[derivative.id] = _axis_view(
                        model, quantities.velocity_twist(model, measured_source), derivative, axis
                    )
                continue
            target = graph.value(plan.view, MAP.superobject)
            difference = PoseDifference(
                id=ids.pose_difference(),
                quantity_kind=["Angle", "Length"],
                reference_point=Point(f"point_{ids.pose_difference()}_origin"),
                as_seen_by=quantities.frame(model, graph.value(target, GEOM_COORD["as-seen-by"])),
                unit=["M", "RAD"],
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
    # This motion's own evaluators, by the constraint they evaluate: a deduplicated constraint
    # shares its id across motions, so the global functions cannot disambiguate.
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
            # A feed-forward controller consumes no error, but its constraint's evaluator still
            # says what is measured against what -- fold those on so the logged slot carries the
            # real values instead of zeros.
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
        # Through the registry, not the graph: a per-axis controller is itself derived.
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
        # An admittance's parameters are authored quantities the function already names, so they
        # are D-blocks on their own; publishing them again would be a second source of truth.
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
        # The bounds are authored shared quantities already; the call site reads them by id. Both
        # cross here: the reader binds them to the same signal, and a bound that stops at the
        # controller record is a limit the model authored and the robot never sees.
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


def authored_motion_drivers(model, node) -> MotionDrivers:
    """The drivers a solver authors directly, before its controllers are derived."""
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
        handler=model.id(model.graph.value(node, PROV.wasDerivedFrom)),
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


def _alignment_op(model, quantity, type_):
    """The `type_` op paired with `quantity`'s `PlanarAngleFromDirections` angle op -- the one
    reading the same two rotated/reference directions -- or None when `quantity` is not one.
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


def alignment_rotation_op(model, quantity):
    """The `RotationVectorFromDirections` op driving `quantity` pointwise onto its reference: the
    2-DOF row shape a cone or a zero target takes."""
    return _alignment_op(model, quantity, GEOM_OP_EXT.RotationVectorFromDirections)


def alignment_gradient_op(model, quantity):
    """The `AngleGradientFromDirections` op driving `quantity` to a target off zero: one axis to
    turn about, so one row, and the angle op it pairs with writes no gradient itself."""
    return _alignment_op(model, quantity, GEOM_OP_EXT.AngleGradientFromDirections)
