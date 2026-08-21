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
import math
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
    KC_STAT,
    MAP,
    QUDT_SCHEMA,
    SLV,
    SLV_EXT,
)
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdf_utils.namespace import NS_MM_QUDT_QTY, NS_MM_QUDT_UNIT
from rdflib import Literal, URIRef
from rdflib.namespace import PROV, RDF, XSD
from scene_dsl.rdf_parser.common import ensure_one_obj_uri

from motion_spec.classes.dynamics import Saturation
from motion_spec.classes.geometry import Point, PoseDifference, Subspace, View
from motion_spec.classes.handlers import (
    FeedForwardController,
    ImpedanceController,
    PIDController,
    StateField,
)
from motion_spec.classes.qudt import Provenance, Quantity, QuantityKind, Unit
from motion_spec.classes.solvers import (
    AccelerationEnergyDriven,
    CartesianAccelerationDriven,
    CartesianForceSpecification,
    CommandForwarding,
    DynamicsSolverFamily,
    JointForceSpecification,
    MotionDrivers,
    Unsolved,
)
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.model import kebab, local_name, si

# Every solver family the code generator can run, keyed by the term that identifies it -- the
# algorithm a solver names, or the type a command-forwarding solver carries.
#
# Vereshchagin's acceleration-constrained hybrid dynamics is posed as a constrained optimisation
# over Gauss's principle, so each constrained direction is driven by an acceleration energy
# (N-m2/s2) and the derived signals lead with `eacc`. Recursive Newton-Euler is driven by the
# Cartesian acceleration itself, and leads with `acc`. A command-forwarding solver runs no
# dynamics at all: its controller's output goes straight to the joint. `slv:ArticulatedBodyAlgorithm`
# gets no entry: an unmapped algorithm is the "unsupported" answer below, not a fourth class.
_SOLVER_FAMILIES = {
    SLV["AccelerationConstrainedHybridDynamicsAlgorithm"]: AccelerationEnergyDriven,
    SLV["RecursiveNewtonEulerAlgorithm"]: CartesianAccelerationDriven,
    SLV_EXT.CommandForwardingSolver: CommandForwarding,
}


def solver_algorithm(model, solver: URIRef) -> type[DynamicsSolverFamily]:
    """The family a solver belongs to, and so what may be derived against it.

    Raises:
        ConstraintViolation: the solver names an algorithm no backend implements.
    """
    for type_ in get_node_types(model.graph, solver):
        if type_ in _SOLVER_FAMILIES:
            return _SOLVER_FAMILIES[type_]
    algorithm = model.graph.value(solver, SLV["solver"])
    if algorithm is None:
        return Unsolved
    if algorithm not in _SOLVER_FAMILIES:
        raise ConstraintViolation(
            "solver", f"Solver '{solver}' has unsupported algorithm '{algorithm}'."
        )

    return _SOLVER_FAMILIES[algorithm]


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

    def component_moment(self, axis: quantities.SpatialAxis) -> str:
        return f"moment_{self.controller}_{axis.suffix}"

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
    (SolverIdFactory.component_controller, "{axis}", PROV.specializationOf, ()),
    (SolverIdFactory.component_error, "err-{axis}", PROV.specializationOf, ()),
    (SolverIdFactory.component_energy, "eacc-{axis}", PROV.wasDerivedFrom, ()),
    (SolverIdFactory.component_acceleration, "acc-{axis}", PROV.wasDerivedFrom, ()),
    (
        SolverIdFactory.component_constraint,
        "acc-cstr-{axis}",
        PROV.wasDerivedFrom,
        (SLV.AccelerationConstraint,),
    ),
    (
        SolverIdFactory.component_acceleration_specification,
        "cart-acc-{axis}",
        PROV.wasDerivedFrom,
        (SLV.AccelerationConstraint,),
    ),
    (
        SolverIdFactory.component_measured_derivative,
        "measured-derivative-{axis}",
        PROV.wasDerivedFrom,
        (),
    ),
    (SolverIdFactory.component_moment, "moment-{axis}", PROV.wasDerivedFrom, ()),
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
        for derive, segment, relation, types in _AXIS_DERIVATIONS:
            model.register_derived(
                derive(ids, spatial_axis),
                parent,
                segment.format(axis=kebab(spatial_axis.suffix)),
                relation,
                types=types,
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
    view they are taken in, the norm of the gradient they came from, and the frame that gradient
    is stated in. Only an expression constraint sets the last two to anything but the default.
    """

    axes: tuple
    view: URIRef | None = None
    gradient_norm: float = 1.0
    gradient_frame: URIRef | None = None


class _LinearForm(NamedTuple):
    """An expression read as `sum(coefficient * measured leaf) + constant`.

    `constant` is None when the constant part is a runtime value: nothing is wrong with that in
    itself, it only stops a subtree from being used as a coefficient.
    """

    coefficients: dict
    constant: float | None


# The four arithmetic operations an expression tree is built from, each as the operand
# predicates to read it back through. A commutative operation states its operands on one
# repeated predicate; the two ordered ones name each role.
_EXPRESSION_OPS = {
    ALGO_EXT.Addition: ("add", (ALGO_EXT["in"],)),
    ALGO_EXT.Subtraction: ("subtract", (ALGO_EXT["minuend"], ALGO_EXT["subtrahend"])),
    ALGO_EXT.Multiplication: ("multiply", (ALGO_EXT["in"],)),
    ALGO_EXT.Division: ("divide", (ALGO_EXT["dividend"], ALGO_EXT["divisor"])),
}
# Below this the gradient states no direction: a normalized one would be numerical noise.
_GRADIENT_EPSILON = 1e-9


def expression_operation(graph, node):
    """The arithmetic operation writing `node`, as `(name, operands)`, or None when `node` is an
    ordinary quantity nothing computes.
    """
    for op in graph.subjects(ALGO_EXT.out, node):
        types = get_node_types(graph, op)
        for type_, (name, predicates) in _EXPRESSION_OPS.items():
            if type_ not in types:
                continue
            if len(predicates) == 1:
                # sorted(): a commutative operation states its operands unordered.
                return name, sorted(graph.objects(op, predicates[0]), key=str)
            return name, [graph.value(op, predicate) for predicate in predicates]
    return None


def _constant_product(forms) -> float | None:
    """The product of forms that carry no measured leaf, or None when any is runtime-valued."""
    product = 1.0
    for form in forms:
        if form.constant is None:
            return None
        product *= form.constant
    return product


def _scaled(form: _LinearForm, factor: float) -> _LinearForm:
    return _LinearForm(
        {leaf: value * factor for leaf, value in form.coefficients.items()},
        None if form.constant is None else form.constant * factor,
    )


def _linear_form(model, node, constraint) -> _LinearForm:
    """`node` as a linear form over the measured leaves it is built from.

    Raises:
        ConstraintViolation: the expression is not affine in its measured leaves, or a
            coefficient is only known at runtime -- either way the alpha row it would need
            cannot be stated before the cycle runs.
    """
    graph = model.graph
    operation = expression_operation(graph, node)
    if operation is None:
        if quantities.view_of(graph, node) is not None:
            return _LinearForm({node: 1.0}, 0.0)
        value = graph.value(node, QUDT_SCHEMA.value)
        unit = graph.value(node, QUDT_SCHEMA.unit)
        return _LinearForm({}, None if value is None else si(float(value), unit))

    name, operands = operation
    forms = [_linear_form(model, operand, constraint) for operand in operands]
    if name == "add":
        return _sum_forms(forms)
    if name == "subtract":
        return _sum_forms([forms[0], _scaled(forms[1], -1.0)])
    if name == "multiply":
        measured = [form for form in forms if form.coefficients]
        if len(measured) > 1:
            raise ConstraintViolation(
                "control",
                f"Constraint '{model.id(constraint)}' multiplies two measured views, so its "
                "gradient depends on what it measures; a controller cannot drive that yet. A "
                "monitor accepts it.",
            )
        if not measured:
            return _LinearForm({}, _constant_product(forms))
        scale = _constant_product([form for form in forms if not form.coefficients])
        if scale is None:
            raise ConstraintViolation(
                "control",
                f"Constraint '{model.id(constraint)}' scales a measured view by a runtime "
                "value; a controlled expression needs coefficients it can state at generation "
                "time. A monitor accepts it.",
            )
        return _scaled(measured[0], scale)

    dividend, divisor = forms
    if divisor.coefficients or divisor.constant is None:
        raise ConstraintViolation(
            "control",
            f"Constraint '{model.id(constraint)}' divides by a value it measures; a controlled "
            "expression divides only by a constant. A monitor accepts it.",
        )
    if divisor.constant == 0.0:
        raise ConstraintViolation(
            "control", f"Constraint '{model.id(constraint)}' divides by zero."
        )
    return _scaled(dividend, 1.0 / divisor.constant)


def _sum_forms(forms) -> _LinearForm:
    coefficients: dict = {}
    for form in forms:
        for leaf, value in form.coefficients.items():
            coefficients[leaf] = coefficients.get(leaf, 0.0) + value
    constants = [form.constant for form in forms]
    return _LinearForm(coefficients, None if None in constants else sum(constants))


def _gradient_direction(model, controller, frame_node, components) -> URIRef:
    """Materialize the unit gradient as a direction coordinate the solver row reads, composed of
    the same DirectionCoordinate/VectorXYZ/as-seen-by terms an authored direction carries.
    """
    node = URIRef(f"{controller}-gradient")
    graph = model.graph
    graph.add((node, RDF.type, GEOM_COORD.DirectionCoordinate))
    graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
    graph.add((node, QUDT_SCHEMA["hasQuantityKind"], NS_MM_QUDT_QTY["Dimensionless"]))
    graph.add((node, QUDT_SCHEMA.unit, NS_MM_QUDT_UNIT["UNITLESS"]))
    graph.add((node, GEOM_COORD["as-seen-by"], frame_node))
    for name in "xyz":
        graph.add((node, GEOM_COORD[name], Literal(components.get(name, 0.0), datatype=XSD.double)))
    return node


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
        relation=next(
            (
                local_name(type_)
                for type_ in get_node_types(graph, constraint)
                if type_ != CSTR.Constraint and local_name(type_).endswith("Constraint")
            ),
            "",
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
    """The direction `quantity` is driven along and the subspace that row sits in.

    Written either by the operator that writes the scalar itself, or -- for a direction pair held
    off zero, whose angle comes from a `PlanarAngleFromDirections` with no gradient output of its
    own -- by the `AngleGradientFromDirections` op paired with it.
    """
    graph = model.graph
    for predicate, subspace in _GRADIENT_OUTPUTS.items():
        for op in sorted(graph.subjects(predicate, quantity), key=str):
            gradient = graph.value(op, GEOM_OP_EXT["gradient"])
            if gradient is not None:
                return gradient, subspace
    op = alignment_gradient_op(model, quantity)
    if op is not None:
        return graph.value(op, GEOM_OP_EXT["gradient"]), _GRADIENT_OUTPUTS[GEOM_OP["angle"]]
    return None, None


def operator_gradient_directions(model, controller, constraint, quantity):
    """The one solver row a plane/line operator's scalar is driven along, or None when nothing
    writes `quantity` that way.

    The operator recomputes the direction every cycle, so the row names the shared vector carrying
    it instead of a frame axis -- the shape a path-following row already takes.
    """
    graph = model.graph
    gradient, subspace = _operator_gradient(model, quantity)
    if gradient is None:
        return None
    axes = _view_axes(
        model, controller, constraint, None, quantity, subspace=subspace, axis="gradient"
    )
    return ControlDirections(
        tuple(replace(axis, direction=gradient) for axis in axes),
        None,
        1.0,
        graph.value(gradient, GEOM_COORD["as-seen-by"]),
    )


def gradient_directions(model, controller, constraint, quantity):
    """The one alpha column a controlled expression drives.

    An expression is driven along its own gradient: each measured leaf contributes its
    coefficient to that leaf's own Cartesian direction, and the column is their normalized sum.
    A single positive coefficient on one axis leaves the frame axis itself -- exactly the row the
    same constraint gets when it is written as a plain view.

    Raises:
        ConstraintViolation: the expression measures nothing, its gradient vanishes, its leaves
            are seen by different frames, or it spans both the linear and the angular half.
    """
    graph = model.graph
    form = _linear_form(model, quantity, constraint)
    components: dict = {}
    frames = set()
    leaf_view = None
    for leaf, coefficient in sorted(form.coefficients.items(), key=lambda item: str(item[0])):
        view = quantities.view_of(graph, leaf)
        axes = _view_axes(model, controller, constraint, view, leaf)
        if not axes:
            # This command drives no solver row at all (a force command); neither does the
            # expression built from it.
            return ControlDirections((), view)
        if len(axes) != 1:
            raise ConstraintViolation(
                "control",
                f"Constraint '{model.id(constraint)}' combines '{model.id(leaf)}', which names a "
                "whole subspace; select one axis of it.",
            )
        leaf_view = leaf_view if leaf_view is not None else view
        components[axes[0]] = components.get(axes[0], 0.0) + coefficient
        frames.add(graph.value(graph.value(view, MAP.superobject), GEOM_COORD["as-seen-by"]))

    components = {
        axis: value for axis, value in components.items() if abs(value) > _GRADIENT_EPSILON
    }
    if not components:
        raise ConstraintViolation(
            "control",
            f"Constraint '{model.id(constraint)}' has no measured view a solver moves, so there "
            "is no direction to drive it along.",
        )
    if len(frames) > 1:
        raise ConstraintViolation(
            "control",
            f"Constraint '{model.id(constraint)}' combines views seen by different frames "
            f"({', '.join(sorted(model.id(frame) for frame in frames))}); state them all in one "
            "frame, since the gradient is one vector in one frame.",
        )
    if len({axis.subspace for axis in components}) > 1:
        raise ConstraintViolation(
            "control",
            f"Constraint '{model.id(constraint)}' spans the linear and the angular subspace; one "
            "solver row carries one of them, so split it into one constraint per subspace.",
        )
    frame_node = next(iter(frames))
    norm = math.sqrt(sum(value * value for value in components.values()))
    axis, coefficient = next(iter(components.items()))
    if len(components) == 1 and coefficient > 0.0:
        return ControlDirections((axis,), leaf_view, norm, frame_node)
    direction = _gradient_direction(
        model,
        controller,
        frame_node,
        {component.axis: value / norm for component, value in components.items()},
    )
    column = quantities.SpatialAxis(axis.subspace, "gradient", direction)

    return ControlDirections((column,), leaf_view, norm, frame_node)


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
        # An expression names no view of its own: its row is the gradient over the views it is
        # built from, so the axes and the frame come from those leaves instead.
        if expression_operation(graph, quantity) is not None:
            result[controller] = gradient_directions(model, controller, constraint, quantity)
            continue
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
    # An expression is driven along a normalized gradient, so the error the controller reads is
    # this many times the one along the direction it drives. 1.0 for every plain view.
    gradient_norm: float = 1.0
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
        family = algorithms[solver]
        # A moment plan's axes are f_ext directions, not solver rows, so they claim no budget.
        axes = [axis for plan in plans if not _is_moment_plan(model, plan) for axis in plan.axes]
        if family.axes_must_be_distinct:
            duplicates = [axis for axis, count in collections.Counter(axes).items() if count > 1]
            if duplicates:
                rendered = ", ".join(f"{axis.subspace}.{axis.axis}" for axis in duplicates)
                raise ConstraintViolation(
                    "solver",
                    f"{family.codegen_name} solver '{solver}' repeats acceleration axis: "
                    f"{rendered}.",
                )
        if family.max_axes is not None and len(axes) > family.max_axes:
            raise ConstraintViolation(
                "solver",
                f"{family.codegen_name} solver '{solver}' has {len(axes)} axes; at most "
                f"{family.max_axes} are supported.",
            )

    for handler, plans in by_handler.items():
        domains = collections.defaultdict(set)
        for plan in plans:
            command = str(model.graph.value(plan.controller, APP["command-type"]) or "")
            subspace = local_name(model.graph.value(plan.view, MAP.subspace)) if plan.view else None
            domains[algorithms[plan.solver]].add(
                "force"
                if command in {"Force", "Torque"} or subspace in {"force", "torque"}
                else "pose"
            )
        overlap = domains[AccelerationEnergyDriven] & domains[CartesianAccelerationDriven]
        if overlap:
            raise ConstraintViolation(
                "solver",
                f"Handler '{handler}' assigns ACHD and RNE to the same domain(s): "
                f"{', '.join(sorted(overlap))}.",
            )


def _gain(model, plan, predicate, *, required=True) -> float | None:
    """An authored gain, divided by the norm of the gradient its constraint is driven along.

    A controller on an expression reads an error in the expression's units while it drives a
    normalized direction, and the two differ by exactly that norm; carrying it here leaves the
    authored gain acting on the error along the direction, which is what the model states. Every
    plain view has norm 1, so nothing else moves.
    """
    read = quantities.required_float if required else quantities.optional_float
    value = read(model, plan.controller, predicate)
    return None if value is None else value / plan.gradient_norm


def _derived_quantity(id_: str, kind: str, unit: str, *, has_view: bool = False) -> Quantity:
    """A runtime scalar the model implies rather than authors."""
    return Quantity(id_, QuantityKind(kind), Unit(unit), None, has_view, provenance=Provenance())


def _motion_scoped(model, context, plan) -> str:
    """The motion suffix a derived id carries, empty when its constraint is shared."""
    if plan.constraint in context.shared_constraints:
        return ""
    return f"_{model.motion_suffix(plan.motion)}"


def _controller_signal_id(model, context, plan) -> str:
    """The scalar output id a controller writes, from its authored command type.

    The id is registered here because this is where it is minted, and the only place that knows
    which of the forms below it took -- and so what the signal derives from. A command the
    controller alone decides derives from the controller; an acceleration payload is named after
    the quantity it drives, and is the same id the acceleration driver mints per axis.
    """
    graph = model.graph
    controller_id = model.id(plan.controller)
    types = get_node_types(graph, plan.controller)
    command_type = str(graph.value(plan.controller, APP["command-type"]) or "")

    def controller_output(signal_id: str) -> str:
        model.register_derived(signal_id, str(plan.controller), "output", PROV.wasDerivedFrom)
        return signal_id

    # A Cartesian moment feeds the moment slot of a wrench, so it is named after that.
    if _is_moment_plan(model, plan):
        return controller_output(f"moment_{controller_id}")
    # A force command is named after the wrench magnitude it feeds, whichever control law
    # produced it; only a command that goes straight to a device is named `cmd_`.
    if CSTR_HDL.ImpedanceController in types or command_type == "Force":
        return controller_output(f"force_{controller_id}")
    if CSTR_HDL_EXT.FeedForwardController in types:
        return controller_output(f"cmd_{controller_id}")
    target = graph.value(plan.view, MAP.superobject) if plan.view is not None else plan.quantity
    if command_type == "Torque" and KC_STAT.JointPositionCoordinate in get_node_types(
        graph, target
    ):
        return controller_output(f"tau_{controller_id}")
    family = context.algorithm_by_solver[plan.solver]
    if not hasattr(family, "payload"):
        raise ConstraintViolation(
            "solver", f"Solver '{plan.solver}' does not accept acceleration signals."
        )

    suffix = _motion_scoped(model, context, plan)
    signal_id = f"{family.signal_prefix}_{model.id(plan.quantity)}{suffix}"
    # Same id and same IRI the single-axis driver mints, so registering it twice is one fact
    # stated twice rather than a collision -- and a controller with no axis has no driver to
    # state it at all.
    model.register_derived(
        signal_id, str(plan.quantity), f"{family.signal_prefix}{kebab(suffix)}", PROV.wasDerivedFrom
    )

    return signal_id


def _axis_error(ids: SolverIdFactory, axis: quantities.SpatialAxis) -> Quantity:
    """The per-axis component of a pose difference the controller drives to zero."""
    linear = axis.subspace == Subspace.Linear
    return _derived_quantity(
        ids.component_error(axis),
        "Length" if linear else "Angle",
        "M" if linear else "RAD",
        has_view=True,
    )


def _axis_derivative(ids: SolverIdFactory, axis: quantities.SpatialAxis) -> Quantity:
    """The per-axis component of the measured velocity feeding a derivative term."""
    linear = axis.subspace == Subspace.Linear
    return _derived_quantity(
        ids.component_measured_derivative(axis),
        "LinearVelocity" if linear else "AngularVelocity",
        "M_PER_SEC" if linear else "RAD_PER_SEC",
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
    if _is_moment_plan(model, plan):
        return _derived_quantity(signal_id, "Torque", "N_M")
    if CSTR_HDL.ImpedanceController in types or command_type == "Force":
        return _derived_quantity(signal_id, "Force", "N")
    if signal_id.startswith("tau_"):
        return _derived_quantity(signal_id, "Torque", "N_M")
    family = context.algorithm_by_solver[plan.solver]
    if issubclass(family, CartesianAccelerationDriven) and plan.axes:
        return family.payload(signal_id, plan.axes[0].subspace)

    return _derived_quantity(signal_id, "AccelerationEnergy", "N_M2_PER_SEC2")


def _derived_controller(model, context, plan, axis: quantities.SpatialAxis | None = None):
    """One controller record: per axis when the command is a pose, singular otherwise."""
    graph = model.graph
    ids = _solver_ids(model, context, plan)
    types = get_node_types(graph, plan.controller)
    measured_source = graph.value(plan.controller, CSTR_HDL["measured-velocity"])
    if axis is not None:
        controller_id = ids.component_controller(axis)
        if _is_moment_plan(model, plan):
            # A moment feeds f_ext, so its payload is the wrench's moment magnitude, not a row.
            signal = _derived_quantity(ids.component_moment(axis), "Torque", "N_M")
        else:
            family = context.algorithm_by_solver[plan.solver]
            payload_id = (
                ids.component_energy(axis)
                if issubclass(family, AccelerationEnergyDriven)
                else ids.component_acceleration(axis)
            )
            signal = family.payload(payload_id, axis.subspace)
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
            proportional_gain=_gain(model, plan, CSTR_HDL["proportional-gain"]),
            integral_gain=_gain(model, plan, CSTR_HDL["integral-gain"]),
            derivative_gain=_gain(model, plan, CSTR_HDL["derivative-gain"]),
            decay_rate=decay.value if CSTR_HDL.DecayingIntegralTerm in types else None,
            output_saturation=output_saturation,
            integral_saturation=integral_saturation,
            tolerance_id=tolerance_id,
            constraint=model.id(plan.constraint),
            constraint_uri=str(plan.constraint),
            type=model.id(CSTR_HDL.ProportionalIntegralDerivative),
        )
    if CSTR_HDL.ImpedanceController in types:
        return ImpedanceController(
            id=controller_id,
            control_signal=signal,
            error_signal=error,
            stiffness=_gain(model, plan, CSTR_HDL.stiffness),
            damping=_gain(model, plan, CSTR_HDL.damping),
            integral_gain=_gain(model, plan, CSTR_HDL["integral-gain"], required=False),
            output_saturation=output_saturation,
            tolerance_id=tolerance_id,
            constraint=model.id(plan.constraint),
            constraint_uri=str(plan.constraint),
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
        constraint=model.id(plan.constraint),
        constraint_uri=str(plan.constraint),
        type=model.id(CSTR_HDL_EXT.FeedForwardController),
    )


def _derived_controllers(model, context, plan) -> list:
    """Expand a pose command per axis; a scalar command stays singular."""
    if len(plan.axes) > 1:
        return [_derived_controller(model, context, plan, axis) for axis in plan.axes]
    return [_derived_controller(model, context, plan)]


# Per family, the `SolverIdFactory` methods that mint its multi-axis spec/payload ids. Keyed by
# the family class itself; `family.id_tags` names the matching single-axis tag pair.
_DRIVER_ID_METHODS = {
    AccelerationEnergyDriven: (
        SolverIdFactory.component_constraint,
        SolverIdFactory.component_energy,
    ),
    CartesianAccelerationDriven: (
        SolverIdFactory.component_acceleration_specification,
        SolverIdFactory.component_acceleration,
    ),
}


class _DriverIds(NamedTuple):
    """The record id and the payload id one acceleration driver mints for one axis."""

    spec: str
    payload: str


def _acceleration_driver_ids(model, context, plan, family, axis, multi_axis) -> _DriverIds:
    """The two ids for one axis, registering them when they derive from a quantity."""
    if multi_axis:
        ids = _solver_ids(model, context, plan)
        spec_id, payload_id = _DRIVER_ID_METHODS[family]

        return _DriverIds(spec_id(ids, axis), payload_id(ids, axis))
    # Single-axis ids are built off the quantity, so that is what they derive from.
    suffix = _motion_scoped(model, context, plan)
    quantity_id = model.id(plan.quantity)
    spec_tag, payload_tag = family.id_tags
    ids = []
    for tag in (spec_tag, payload_tag):
        derived_id = f"{tag.replace('-', '_')}_{quantity_id}{suffix}"
        types = (SLV.AccelerationConstraint,) if tag == spec_tag else ()
        model.register_derived(
            derived_id,
            str(plan.quantity),
            f"{tag}{kebab(suffix)}",
            PROV.wasDerivedFrom,
            types=types,
        )
        ids.append(derived_id)

    return _DriverIds(*ids)


def _derived_acceleration_drivers(model, context, plan, family) -> list:
    """One controller's ordered per-axis acceleration records, for its solver's family."""
    target = (
        model.graph.value(plan.view, MAP.superobject) if plan.view is not None else plan.quantity
    )
    frame_node = model.graph.value(target, GEOM_COORD["as-seen-by"]) or plan.gradient_frame
    frame = quantities.frame(model, frame_node) if frame_node is not None else None
    multi_axis = len(plan.axes) > 1
    records = []
    for axis in plan.axes:
        spec_id, payload_id = _acceleration_driver_ids(
            model, context, plan, family, axis, multi_axis
        )
        records.append(
            family.driver(
                id=spec_id,
                subspace=axis.subspace,
                axis=quantities.AXIS_BY_NAME.get(axis.frame_axis),
                as_seen_by=frame,
                direction=(
                    quantities.direction(model, axis.direction)
                    if axis.direction is not None
                    else None
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
    driven = hasattr(family, "payload")
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
            **driver_slots,
        )
        for node in model.graph.objects(solver, SLV["motion-drivers"])
    ]


def _bind_alignment_band(model, plan, closure) -> None:
    """Give an alignment's rotation-vector op the angle it is allowed to keep.

    The ops are one set for the whole model, so two motions banding the same alignment
    differently would need two of them; say so rather than let the last one win.

    Raises:
        ConstraintViolation: two motions state different bands for one alignment.
    """
    if closure is None:
        return
    upper = model.graph.value(plan.constraint, CSTR["upper-threshold"])
    band = model.id(upper) if upper is not None else None
    if "band" in closure and closure["band"] != band:
        raise ConstraintViolation(
            "constraint-handler",
            f"alignment '{closure['id']}' is banded differently by two motions "
            f"('{closure['band']}' and '{band}'); state one band for it.",
        )
    closure["band"] = band


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
            # An alignment drives the rotation vector's components; there is no pose pair to
            # difference, and its ops already produce the error. A banded alignment tolerates a
            # cone, so the op carries the band and returns only the rotation beyond it.
            rotation_op = alignment_rotation_op(model, graph.value(plan.constraint, CSTR.quantity))
            if rotation_op is not None:
                _bind_alignment_band(model, plan, closures.get(model.id(rotation_op)))
                continue
            target = graph.value(plan.view, MAP.superobject)
            reference = graph.value(plan.constraint, CSTR["reference-value"])
            reference_view = quantities.view_of(graph, reference)
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
ADMITTANCE_PARAMETERS = (
    "mass",
    "damping",
    "stiffness",
    "maximum_velocity",
    "maximum_absolute_value",
    "lower_threshold",
    "upper_threshold",
)

# The signals a controller binds, in the order their introspection rows are emitted.
CONTROLLER_SIGNAL_ROLES = (
    "error_signal",
    "reference_signal",
    "measured_derivative",
    "control_signal",
    "measured_signal",
    "setpoint_signal",
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


def alignment_chain_ops(model, quantity):
    """The rotate-direction and angle ops producing the alignment angle `quantity`, in
    dependency order, or `[]` when `quantity` is not an alignment angle."""
    graph = model.graph
    angle_op = next(
        (
            op
            for op in graph.subjects(GEOM_OP.angle, quantity)
            if GEOM_OP.PlanarAngleFromDirections in get_node_types(graph, op)
        ),
        None,
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
    angle_op = next(
        (
            op
            for op in graph.subjects(GEOM_OP.angle, quantity)
            if GEOM_OP.PlanarAngleFromDirections in get_node_types(graph, op)
        ),
        None,
    )
    if angle_op is None:
        return None
    directions = set(graph.objects(angle_op, GEOM_OP["from-directions"]))
    return next(
        (
            op
            for op in graph.subjects(RDF.type, type_)
            if {graph.value(op, GEOM_OP.in1), graph.value(op, GEOM_OP.in2)} == directions
        ),
        None,
    )


def alignment_rotation_op(model, quantity):
    """The `RotationVectorFromDirections` op driving `quantity` pointwise onto its reference: the
    2-DOF row shape a cone or a zero target takes."""
    return _alignment_op(model, quantity, GEOM_OP_EXT.RotationVectorFromDirections)


def alignment_gradient_op(model, quantity):
    """The `AngleGradientFromDirections` op driving `quantity` to a target off zero: one axis to
    turn about, so one row, and the angle op it pairs with writes no gradient itself."""
    return _alignment_op(model, quantity, GEOM_OP_EXT.AngleGradientFromDirections)
