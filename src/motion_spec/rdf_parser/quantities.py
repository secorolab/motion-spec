# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What values exist, and who writes them.

In order: the readers, the values derived from them, the blackboard, and -- last, because it
rests on all three -- the dataflow contract.

This module never asks in what order a value is written -- that is ``operations.py``'s question.
It is the only module that knows what is on the blackboard, and the dataflow contract at the foot
of the file is the single answer to *who writes this value*.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import NamedTuple

import rdflib
from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT,
    CSTR,
    CSTR_EXT,
    CSTR_HDL,
    CSTR_HDL_EXT,
    ENV,
    EXEC,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_OP,
    GEOM_OP_EXT,
    GEOM_REL,
    KC_STAT,
    MAP,
    MAP_EXT,
    QUDT_SCHEMA,
    RBDYN_COORD,
    RBDYN_ENT,
    SENSORS,
    SLV,
    SOSA,
    TIME,
)
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import ModelBase, get_node_types
from rdf_utils.models.vocab import (
    URI_TIME_PRED_AFTER_EVT,
    URI_TIME_PRED_OF_CONSTRAINT,
    URI_TIME_TYPE_AFTER_EVT,
)
from rdf_utils.models.geom_coord import (
    OrientCoordModel,
    PoseCoordModel,
    PositionCoordModel,
    get_coord_vectorxyz,
    get_orientation_coord_vals,
    to_metres,
)
from rdf_utils.models.geom_rel import OrientationModel, PositionModel
from rdf_utils.models.vocab import (
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_GEOM_PRED_ALPHA,
    URI_GEOM_PRED_AXES_SEQ,
    URI_GEOM_PRED_BETA,
    URI_GEOM_PRED_DIRECTION_COSINE_X,
    URI_GEOM_PRED_DIRECTION_COSINE_Y,
    URI_GEOM_PRED_DIRECTION_COSINE_Z,
    URI_GEOM_PRED_GAMMA,
    URI_GEOM_PRED_W,
    URI_GEOM_PRED_X,
    URI_GEOM_PRED_Y,
    URI_GEOM_PRED_Z,
    URI_GEOM_TYPE_ANGLES_ABG,
    URI_GEOM_TYPE_DIRECTION_COSINE_XYZ,
    URI_GEOM_TYPE_EULER_ANGLES,
    URI_GEOM_TYPE_INTRINSIC,
    URI_GEOM_TYPE_ORIENT,
    URI_GEOM_TYPE_ORIENT_COORD,
    URI_GEOM_TYPE_POSE,
    URI_GEOM_TYPE_POSE_COORD,
    URI_GEOM_TYPE_POSITION,
    URI_GEOM_TYPE_POSITION_COORD,
    URI_GEOM_TYPE_QUATERNION,
    URI_QUDT_UNIT_DEG,
    URI_QUDT_UNIT_RAD,
)
from rdf_utils.namespace import NS_MM_KC_EXT, NS_MM_QUDT_QTY, NS_MM_QUDT_UNIT
from rdf_utils.naming import get_valid_var_name
from rdflib import URIRef
from rdflib.namespace import PROV, RDF, SOSA
from scene_dsl.rdf_parser.common import ensure_one_obj_uri
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.base import dedupe_by_id
from motion_spec.classes.constraints import (
    BilateralConstraint,
    Constraint,
    EqualityConstraint,
    GoalStatus,
    OutsideConstraint,
    UnilateralConstraint,
    UnilateralConstraintType,
)
from motion_spec.classes.dynamics import JointPosition
from motion_spec.classes.geometry import (
    AccelerationTwist,
    Axis,
    Direction,
    Frame,
    Orientation,
    Point,
    Pose,
    PoseDifference,
    Position,
    SceneObject,
    SimplicialComplex,
    Subspace,
    VelocityTwist,
    View,
    Wrench,
)
from motion_spec.classes.motion import (
    ComponentRef,
    PoseComponents,
    PoseErrorComponent,
    PoseErrorRegroup,
    RelativePoseCapture,
    SceneRelativePose,
    SnapshotCapture,
)
from motion_spec.classes.qudt import (
    FreeVector,
    Provenance,
    Quantity,
    QuantityKind,
    SetpointQuantity,
    Unit,
)
from motion_spec.rdf_parser.model import (
    length_unit,
    local_name,
    reader,
    seconds,
    si,
    si_all,
    si_unit,
)
from motion_spec.rdf_parser.operations import (
    closure_maps,
    recorded_coord_policy,
    closure_output_ids,
    closure_owner_map,
    data_reference_map,
)


def duration_seconds(model, node) -> float:
    """The value in seconds of a Duration node, converting from the unit it was written in."""
    graph = model.graph
    return seconds(
        float(graph.value(node, QUDT_SCHEMA["value"])), graph.value(node, QUDT_SCHEMA["unit"])
    )


def optional_float(model, subject, predicate) -> float | None:
    """A float-valued property, or None when the subject does not carry it.

    A model may state the number directly or wrap it in a qudt node; both read the same here.

    Raises:
        ConstraintViolation: the property is present but carries neither a literal nor a
            qudt:value.
    """
    value = model.graph.value(subject, predicate)
    if value is None:
        return None
    literal = (
        value if isinstance(value, rdflib.Literal) else model.graph.value(value, QUDT_SCHEMA.value)
    )
    if literal is None:
        raise ConstraintViolation(
            "quantity",
            f"Controller '{model.id(subject)}' property '{model.id(predicate)}' must be a "
            "literal or a node with qudt:value.",
        )

    return float(literal.value)


def optional_seconds(model, subject, predicate) -> float | None:
    """A duration-valued property in seconds, converting from the unit it was written in."""
    node = model.graph.value(subject, predicate)
    if node is None:
        return None
    value = optional_float(model, subject, predicate)
    return None if value is None else seconds(value, model.graph.value(node, QUDT_SCHEMA.unit))


def required_float(model, subject, predicate) -> float:
    """A float-valued property the model must author.

    Raises:
        ConstraintViolation: the property is absent, or present without a readable number.
    """
    value = optional_float(model, subject, predicate)
    if value is None:
        raise ConstraintViolation(
            "quantity",
            f"Controller '{model.id(subject)}' is missing required property "
            f"'{model.id(predicate)}'.",
        )
    return value


def is_constraint_aggregate(model, node) -> bool:
    """True for an until/when group node: a conjunction or disjunction of constraints."""
    types = get_node_types(model.graph, node)
    return bool({CSTR_EXT.ConstraintDisjunction, CSTR_EXT.ConstraintConjunction} & types)


_SUBSPACES = {
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
# Every RDF term that names a Cartesian axis, whatever metamodel states it.
_AXES = {
    MAP["x"]: Axis.X,
    MAP["y"]: Axis.Y,
    MAP["z"]: Axis.Z,
    MAP["w"]: Axis.W,
    SLV["x"]: Axis.X,
    SLV["y"]: Axis.Y,
    SLV["z"]: Axis.Z,
}
# The same axes by the name a derived direction carries, which is a string rather than a term.
AXIS_BY_NAME = {"x": Axis.X, "y": Axis.Y, "z": Axis.Z}


@dataclass(frozen=True)
class SpatialAxis:
    """One ordered linear or angular Cartesian direction.

    A path-following direction is known only at runtime, so it names the shared vector carrying it
    instead of a fixed frame axis.
    """

    subspace: Subspace
    axis: str
    direction: str | None = None

    @property
    def suffix(self) -> str:
        """The fragment this direction contributes to a derived id."""
        prefix = "lin" if self.subspace == Subspace.Linear else "ang"
        return f"{prefix}_{self.axis}"

    @property
    def frame_axis(self) -> str | None:
        """The fixed frame axis this direction is, or None when it is a runtime vector."""
        return None if self.direction is not None else self.axis


LINEAR_AXES = tuple(SpatialAxis(Subspace.Linear, name) for name in "xyz")
ANGULAR_AXES = tuple(SpatialAxis(Subspace.Angular, name) for name in "xyz")
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
    """The ordered Cartesian directions an authored command controls.

    Parameters:
        controller_type: local name of the controller's RDF type
        subspace: local name of the view's subspace predicate, when it has a view
        axis: local name of the view's axis predicate, when it selects one
        command_type: the authored `app:command-type`, when there is one
        relation: local name of the constraint's relation type
        quantity_kind: `Pose` or `JointPosition` when the target is one of those coordinates

    Returns:
        the directions, empty when nothing Cartesian is commanded
    """
    # An impedance on an angular subspace states Torque; only an unstated one defaults to Force.
    if controller_type == "ImpedanceController" and command_type != "Torque":
        command_type = "Force"
    if command_type == "Force" or subspace == "force":
        return ()
    if command_type == "Torque" and quantity_kind == "JointPosition":
        return ()
    if quantity_kind == "Pose" and subspace in {None, "pose"} and relation == "EqualityConstraint":
        return POSE_AXES
    if subspace in {"position", "linear-velocity"}:
        return (SpatialAxis(Subspace.Linear, axis),) if axis else LINEAR_AXES
    if subspace in {"orientation", "angular-velocity"}:
        return (SpatialAxis(Subspace.Angular, axis),) if axis else ANGULAR_AXES
    if subspace == "distance" and axis is not None:
        return (SpatialAxis(Subspace.Linear, axis),)
    if subspace == "rotation" and axis is not None:
        return (SpatialAxis(Subspace.Angular, axis),)
    if subspace == "distance" and axis is None:
        return (SpatialAxis(Subspace.Linear, "distance"),)

    return ()


_XYZ_PREDICATES = (URI_GEOM_PRED_X, URI_GEOM_PRED_Y, URI_GEOM_PRED_Z)


def subspace(node) -> Subspace:
    """The linear or angular half a view predicate selects."""
    if node not in _SUBSPACES:
        raise ConstraintViolation("geometry", f"unknown subspace {node}")
    return _SUBSPACES[node]


def axis(node) -> Axis:
    """The Cartesian axis a view predicate selects."""
    if node not in _AXES:
        raise ConstraintViolation("geometry", f"unknown axis {node}")
    return _AXES[node]


def _reject_sampled(coordinate) -> None:
    """A placement drawn from a distribution has no value until a sampler runs, and nothing
    downstream can wait for one. rdf-utils offers `get_or_sample_*` for the sampling path; these
    readers take the strict one, so the rejection is stated once here.
    """
    if URI_DISTRIB_TYPE_SAMPLED_QUANTITY in coordinate.types:
        raise ConstraintViolation(
            "geometry", f"Sampled placement coordinate '{coordinate.id}' is unsupported"
        )


def position_values(model, coordinate) -> list[float] | None:
    """xyz of a position coordinate in metres, or None when it carries no vector.

    Parameters:
        coordinate: a `PositionCoordModel`, or the position half of a `PoseCoordModel`

    Raises:
        ConstraintViolation: the coordinate is sampled, or its unit is not a length.
    """
    _reject_sampled(coordinate)
    values = get_coord_vectorxyz(coordinate, model.graph)
    return None if values is None else to_metres(values, coordinate.unit, coordinate.id)


def orientation_quaternion(model, coordinate) -> list[float] | None:
    """Rotation of an orientation coordinate as [x, y, z, w], or None.

    `get_orientation_coord_vals` is what normalizes the representations -- Euler with its axes
    sequence and intrinsic flag, quaternion, direction cosines -- so there is nothing to dispatch
    on here. (`get_quaternion_xyzw` is the quaternion-only reader and rejects the others.)
    """
    _reject_sampled(coordinate)
    rotation = get_orientation_coord_vals(coordinate, model.graph)
    return None if rotation is None else [float(value) for value in rotation.as_quat()]


def position_coordinate_values(model, position_node) -> list[float] | None:
    """xyz of a position relation's coordinate, in metres, or None when none carries a vector."""
    relation = PositionModel(position_id=position_node, graph=model.graph)
    for coordinate_id in relation.coordinate_ids:
        coordinate = PositionCoordModel(
            coord_id=coordinate_id, graph=model.graph, position=relation
        )
        values = position_values(model, coordinate)
        if values is not None:
            return values
    return None


def orientation_relation_quaternion(model, orientation_node) -> list[float] | None:
    """Rotation of an orientation relation's coordinate as [x, y, z, w], or None."""
    relation = OrientationModel(orn_id=orientation_node, graph=model.graph)
    for coordinate_id in relation.coordinate_ids:
        coordinate = OrientCoordModel(
            coord_id=coordinate_id, graph=model.graph, orientation=relation
        )
        rotation = orientation_quaternion(model, coordinate)
        if rotation is not None:
            return rotation
    return None


def parse_xyz(model, node) -> list[float] | None:
    """The x/y/z scalars authored on a node, on SI, or None when it carries no vector.

    Not `get_coord_vectorxyz`, which requires a VectorXYZ-typed coordinate model: this reads the
    same predicates off a plain node -- a direction, a free vector, a gravity triple.
    """
    graph = model.graph
    values = [graph.value(node, predicate) for predicate in _XYZ_PREDICATES]
    if any(value is None for value in values):
        return None
    return si_all((value.value for value in values), graph.value(node, QUDT_SCHEMA["unit"]))


@reader
def position(model, node) -> Position:
    """A Position quantity: of a point with respect to another, in metres."""
    graph = model.graph
    if URI_GEOM_TYPE_POSITION_COORD in get_node_types(graph, node):
        coordinate = PositionCoordModel(node, graph)
        relation = coordinate.position
    else:
        relation = PositionModel(node, graph)
        if len(relation.coordinate_ids) != 1:
            raise ConstraintViolation("geometry", f"Position '{node}' needs exactly one coordinate")
        coordinate = PositionCoordModel(next(iter(relation.coordinate_ids)), graph, relation)

    return Position(
        model.id(node),
        position_reference(model, relation.of_id),
        position_reference(model, relation.wrt_id),
        QuantityKind(model.id(graph.value(node, QUDT_SCHEMA["hasQuantityKind"]))),
        frame(model, coordinate.as_seen_by),
        Unit(model.id(si_unit(length_unit(coordinate)))),
        position_values(model, coordinate),
    )


def position_reference(model, node) -> Point | None:
    """A Position is of a Point with respect to a Point (geometry metamodel)."""
    if node is None:
        return None
    if GEOM_ENT.Point in get_node_types(model.graph, node):
        return point(model, node)
    if GEOM_ENT.Frame in get_node_types(model.graph, node):
        return Point(model.id(node))
    raise ConstraintViolation("geometry", f"Position reference must be a Point, got: {node}")


@reader
def orientation(model, node) -> Orientation:
    """An Orientation quantity of a frame or object with respect to another."""
    graph = model.graph
    if URI_GEOM_TYPE_ORIENT_COORD in get_node_types(graph, node):
        coordinate = OrientCoordModel(node, graph)
        relation = coordinate.relation
    else:
        relation = OrientationModel(node, graph)
        if len(relation.coordinate_ids) != 1:
            raise ConstraintViolation(
                "geometry", f"Orientation '{node}' needs exactly one coordinate"
            )
        coordinate = OrientCoordModel(next(iter(relation.coordinate_ids)), graph, relation)

    axes = graph.value(coordinate.id, URI_GEOM_PRED_AXES_SEQ)

    return Orientation(
        model.id(node),
        _optional_pose_reference(model, relation.of_id),
        _optional_pose_reference(model, relation.wrt_id),
        QuantityKind(model.id(graph.value(node, QUDT_SCHEMA["hasQuantityKind"]))),
        frame(model, coordinate.as_seen_by.id),
        Unit(_orientation_unit(model, coordinate)),
        str(axes) if axes is not None else "xyz",
        (node, ~MAP["subobject"], None) in graph,
        provenance=quantity_provenance(model, node),
    )


def _optional_pose_reference(model, node):
    """A pose endpoint that may be absent, an object or a frame."""
    if node is None:
        return None
    types = get_node_types(model.graph, node)
    if ENV.RigidObject in types:
        return scene_object(model, node)
    if GEOM_ENT.Frame in types:
        return frame(model, node)
    return None


# Rotations whose components already resolve to one quaternion, and the wider set a literal
# delta may be written in.
_RESOLVED_ROTATION = frozenset({URI_GEOM_TYPE_QUATERNION, URI_GEOM_TYPE_DIRECTION_COSINE_XYZ})
_LITERAL_ROTATION = _RESOLVED_ROTATION | {URI_GEOM_TYPE_ANGLES_ABG}


def _orientation_unit(model, coordinate) -> str:
    """A rotation is unitless unless it is authored as angles, which must be in exactly one."""
    if _orientation_composition(model, coordinate.id) is not None or (
        coordinate.types & _RESOLVED_ROTATION
    ):
        return model.id(NS_MM_QUDT_UNIT.UNITLESS)
    units = set(model.graph.objects(coordinate.id, QUDT_SCHEMA["unit"]))
    angular = units & {URI_QUDT_UNIT_RAD, URI_QUDT_UNIT_DEG}
    if len(angular) != 1:
        raise ConstraintViolation(
            "geometry",
            f"OrientationCoordinate '{coordinate.id}' needs exactly one angular unit, "
            f"found {angular}",
        )

    return model.id(si_unit(next(iter(angular))))


def _orientation_composition(model, node):
    """The `geom-op-ext:ComposeOrientation` operator writing into this orientation, if any."""
    if node is None:
        return None

    return next(
        (
            operation
            for operation in model.graph.subjects(GEOM_OP["composite"], node)
            if GEOM_OP_EXT.ComposeOrientation in get_node_types(model.graph, operation)
        ),
        None,
    )


def orientation_representation(model, node) -> str:
    """How an orientation's components arrive, not how the model wrote them.

    All-literal rotations resolve to the same quaternion however they were authored. What cannot
    be resolved ahead of time keeps its own shape: `euler` is a triple whose angles arrive at
    runtime, `relative` composes around a runtime pose.
    """
    if node is None:
        return "quaternion"
    types = get_node_types(model.graph, node)
    if _orientation_composition(model, node) is not None:
        return "relative"
    if URI_GEOM_TYPE_EULER_ANGLES in types and URI_GEOM_TYPE_ANGLES_ABG not in types:
        return "euler"
    return "quaternion"


def _relative_orientation(model, node) -> list[dict]:
    """The composition's two operands, in `geom-op:in1`/`in2` order: each is either
    `{"pose": <id>}` (the base, by id) or `{"delta": [...], "representation": ...}` (the delta's
    ordered component values and rotation representation).
    """
    graph = model.graph
    composition = _orientation_composition(model, node)
    if composition is None:
        raise ConstraintViolation(
            "geometry", f"Relative orientation '{node}' has no composition operator."
        )
    in1 = ensure_one_obj_uri(graph, composition, GEOM_OP["in1"])
    in2 = ensure_one_obj_uri(graph, composition, GEOM_OP["in2"])
    if in1 is None or in2 is None:
        raise ConstraintViolation(
            "geometry", f"Orientation composition '{composition}' must declare both operands."
        )

    def operand(operand_node) -> dict:
        types = get_node_types(graph, operand_node)
        if URI_GEOM_TYPE_POSE_COORD in types:
            return {"pose": model.id(operand_node)}
        if types & _LITERAL_ROTATION:
            # A delta is literal by construction, so it folds to a quaternion here.
            rotation = orientation_quaternion(model, ModelBase(node_id=operand_node, graph=graph))
            if rotation is None:
                raise ConstraintViolation(
                    "geometry",
                    f"Relative orientation delta '{operand_node}' has no literal components",
                )

            return {
                "delta": [{"value": value} for value in rotation],
                "representation": "quaternion",
            }
        raise ConstraintViolation(
            "geometry",
            f"Relative orientation operand '{operand_node}' is neither a pose nor an orientation",
        )

    operands = [operand(in1), operand(in2)]
    if sum("pose" in op for op in operands) != 1 or sum("delta" in op for op in operands) != 1:
        raise ConstraintViolation(
            "geometry",
            f"Relative orientation '{node}' must compose exactly one base pose "
            "and one delta rotation.",
        )

    return operands


@reader
def _pose_endpoint(model, node):
    """A pose endpoint as the entity it names: a scene object or a frame."""
    if node is None:
        return None
    if ENV.RigidObject in get_node_types(model.graph, node):
        return scene_object(model, node)
    return frame(model, node)


def view_of(graph, operand):
    """The MAP view an operand resolves through. A constraint names the view itself (relations
    are pooled per frame pair); an operand that is a subobject still finds the view sampling it.
    """
    if graph.value(operand, MAP.subobject) is not None:
        return operand
    return next(graph.subjects(MAP.subobject, operand), None)


class ReferenceFrames(NamedTuple):
    """The three frames a spatial quantity is stated against."""

    of: object
    with_respect_to: object
    as_seen_by: object


def derived_reference_frames(model, node) -> ReferenceFrames:
    """A reference value's frames, taken from its snapshot source or from its RDF use.

    A snapshot or reference pose carries no frames of its own: it inherits them from what it
    captures, or from the quantity whose constraint names it.
    """
    graph = model.graph
    source = graph.value(node, PROV.wasDerivedFrom)
    if source is None:
        owner = next(graph.subjects(CSTR["reference-value"], node), None)
        quantity_node = graph.value(owner, CSTR.quantity) if owner is not None else None
        view = view_of(graph, quantity_node) if quantity_node is not None else None
        source = graph.value(view, MAP.superobject) if view is not None else quantity_node
    if source is None:
        return ReferenceFrames(None, None, None)

    return ReferenceFrames(
        graph.value(source, GEOM_REL.of),
        graph.value(source, GEOM_REL["with-respect-to"]),
        graph.value(source, GEOM_COORD["as-seen-by"]),
    )


def _bare_pose(model, node) -> Pose:
    """A geom-rel:Pose with no PoseCoordinate: a snapshot or reference pose filled at runtime.

    Same IR shape as a coordinate pose, with frame endpoints but no authored values.
    """
    graph = model.graph
    of_node = graph.value(node, GEOM_REL["of"])
    wrt_node = graph.value(node, GEOM_REL["with-respect-to"])
    seen_node = graph.value(node, GEOM_COORD["as-seen-by"])
    if of_node is None or wrt_node is None or seen_node is None:
        inherited_of, inherited_wrt, inherited_seen = derived_reference_frames(model, node)
        of_node = of_node or inherited_of
        wrt_node = wrt_node or inherited_wrt
        seen_node = seen_node or inherited_seen

    return Pose(
        model.id(node),
        _pose_endpoint(model, of_node),
        _pose_endpoint(model, wrt_node),
        [model.id(kind) for kind in graph[node : QUDT_SCHEMA["hasQuantityKind"]]],
        frame(model, seen_node) if seen_node is not None else None,
        [model.id(unit) for unit in graph[node : QUDT_SCHEMA["unit"]]],
        None,
        None,
        None,
        None,
        provenance=quantity_provenance(model, node),
    )


def pose(model, node) -> Pose:
    """A Pose quantity: its endpoints, its position and how its orientation arrives."""
    graph = model.graph
    coordinate = PoseCoordModel(node, graph, coord_policy=recorded_coord_policy)
    relation = coordinate.relation
    orientation_node = coordinate.orientation_coord.id
    representation = orientation_representation(model, orientation_node)

    # A symbolic triple keeps its convention: the backend composes per-axis quaternions, and the
    # sequence decides both the axes and the order they multiply in. A config pose is stamped
    # Euler for lack of an authored coordinate to inspect, but it carries no per-axis factors --
    # its whole frame is read from the deployment config, not composed from a sequence.
    euler_axes_sequence = None
    euler_intrinsic = False
    if representation == "euler" and not _is_config_pose(model, node):
        axes = graph.value(orientation_node, URI_GEOM_PRED_AXES_SEQ)
        euler_axes_sequence = str(axes) if axes is not None else None
        euler_intrinsic = URI_GEOM_TYPE_INTRINSIC in coordinate.orientation_coord.types

    units = list(
        dict.fromkeys(
            model.id(si_unit(unit))
            for component in (coordinate.position_coord.id, coordinate.orientation_coord.id)
            for unit in graph[component : QUDT_SCHEMA["unit"]]
        )
    )

    return Pose(
        model.id(node),
        _pose_endpoint(model, relation.of_id),
        _pose_endpoint(model, relation.wrt_id),
        [model.id(kind) for kind in graph[relation.id : QUDT_SCHEMA["hasQuantityKind"]]],
        frame(model, coordinate.as_seen_by.id),
        units,
        position_values(model, coordinate.position_coord),
        euler_axes_sequence,
        euler_intrinsic,
        representation,
        orientation_operands=(
            _relative_orientation(model, orientation_node) if representation == "relative" else None
        ),
        provenance=_pose_provenance(model, node, coordinate),
    )


def _pose_provenance(model, node, coordinate) -> Provenance:
    """A pose is authored when either component carries values, or when its rotation is a literal
    form, or when a composition writes it and a per-axis view reads it back.
    """
    if _is_snapshot(model, node) or _is_config_pose(model, node):
        return Provenance(authored=False)
    components = (coordinate.position_coord, coordinate.orientation_coord)
    literal_rotation = coordinate.orientation_coord.types & (
        _RESOLVED_ROTATION | {URI_GEOM_TYPE_EULER_ANGLES}
    )
    composed_and_viewed = _orientation_composition(
        model, coordinate.orientation_coord.id
    ) is not None and any(
        model.graph.value(view, MAP["axis"]) is not None
        for view in model.graph.subjects(MAP["superobject"], node)
    )
    authored = (
        any(_is_authored(model, component.id) for component in components)
        or bool(literal_rotation)
        or composed_and_viewed
    )

    return Provenance(authored=authored)


@reader
def direction(model, node) -> Direction:
    """A Direction quantity: a unit vector as seen by a frame."""
    model.expect_type(node, GEOM_COORD["DirectionCoordinate"])
    model.expect_type(node, GEOM_COORD["VectorXYZ"])
    graph = model.graph

    return Direction(
        model.id(node),
        [model.id(kind) for kind in graph[node : QUDT_SCHEMA["hasQuantityKind"]]],
        frame(model, graph.value(node, GEOM_COORD["as-seen-by"])),
        [Unit(model.id(graph.value(node, QUDT_SCHEMA["unit"])))],
        parse_xyz(model, node),
    )


def _spatial_fields(model, node) -> dict:
    """The `SpatialCoordinate` base fields, shared by the 6D coordinate quantities and read off
    one node the same way regardless of which one subclasses it.
    """
    graph = model.graph
    return {
        "id": model.id(node),
        "quantity_kind": [model.id(kind) for kind in graph[node : QUDT_SCHEMA["hasQuantityKind"]]],
        "reference_point": point(model, graph.value(node, GEOM_REL["reference-point"])),
        "as_seen_by": frame(model, graph.value(node, GEOM_COORD["as-seen-by"])),
        "unit": [model.id(unit) for unit in graph[node : QUDT_SCHEMA["unit"]]],
        "provenance": quantity_provenance(model, node),
    }


@reader
def velocity_twist(model, node) -> VelocityTwist:
    """A VelocityTwist quantity: of a body with respect to another, about a reference point."""
    model.expect_type(node, GEOM_COORD["VelocityTwistCoordinate"])
    model.expect_type(node, GEOM_COORD["VectorXYZ"])
    return VelocityTwist(
        of=simplicial_complex(model, model.graph.value(node, GEOM_REL["of"])),
        with_respect_to=simplicial_complex(
            model, model.graph.value(node, GEOM_REL["with-respect-to"])
        ),
        **_spatial_fields(model, node),
    )


@reader
def acceleration_twist(model, node) -> AccelerationTwist:
    """An AccelerationTwist quantity."""
    model.expect_type(node, GEOM_COORD["AccelerationTwistCoordinate"])
    model.expect_type(node, GEOM_COORD["VectorXYZ"])
    return AccelerationTwist(**_spatial_fields(model, node))


@reader
def pose_difference(model, node) -> PoseDifference:
    """A PoseDifference quantity."""
    model.expect_type(node, GEOM_COORD["PoseDifferenceCoordinate"])
    model.expect_type(node, GEOM_COORD["VectorXYZ"])
    return PoseDifference(**_spatial_fields(model, node))


@reader
def wrench(model, node) -> Wrench:
    """A Wrench quantity, with the force/torque sensor measuring it when there is one."""
    graph = model.graph
    model.expect_type(node, RBDYN_COORD["WrenchCoordinate"])
    relation = graph.value(node, RBDYN_COORD["of-wrench"])
    if relation is None or RBDYN_ENT.Wrench not in get_node_types(graph, relation):
        raise ConstraintViolation(
            "dynamics", f"WrenchCoordinate '{node}' has no valid of-wrench relation"
        )
    reference = graph.value(relation, RBDYN_ENT["reference-point"])
    seen_by = graph.value(node, RBDYN_COORD["as-seen-by"])
    if reference is None or seen_by is None:
        raise ConstraintViolation(
            "dynamics", f"WrenchCoordinate '{node}' is missing reference-point/as-seen-by"
        )
    sensor = graph.value(node, SOSA.madeBySensor)
    sensor_frame_node = graph.value(sensor, SENSORS.frame) if sensor is not None else None
    if sensor is not None and sensor_frame_node is None:
        raise ConstraintViolation(
            "dynamics", f"WrenchCoordinate '{node}' sensor '{sensor}' has no physical frame"
        )

    return Wrench(
        id=model.id(node),
        quantity_kind=[model.id(kind) for kind in graph[relation : QUDT_SCHEMA["hasQuantityKind"]]],
        reference_point=point(model, reference),
        as_seen_by=frame(model, seen_by),
        unit=[model.id(unit) for unit in graph[node : QUDT_SCHEMA["unit"]]],
        provenance=quantity_provenance(model, node),
        sensor_frame=frame(model, sensor_frame_node) if sensor_frame_node is not None else None,
        sensor_name=model.id(sensor) if sensor is not None else "",
        retare_event_uris=tuple(
            str(event)
            for schedule in graph.subjects(URI_TIME_PRED_OF_CONSTRAINT, node)
            for event in graph.objects(schedule, URI_TIME_PRED_AFTER_EVT)
        ),
    )


def _is_duration(model, node) -> bool:
    """Authored durations carry the OWL-Time type; runtime elapsed time is a Time-kind quantity
    the clock fills, so it has a kind but no value.
    """
    if TIME["Duration"] in get_node_types(model.graph, node):
        return True
    return model.graph.value(node, QUDT_SCHEMA.hasQuantityKind) == NS_MM_QUDT_QTY["Time"]


def perceived_written_poses(model) -> dict[str, list[dict]]:
    """Per perception source, the world poses it writes and the frame each must arrive in.

    A source is any node stating the objects it observes -- a detect act that asks once, or a
    topic the model stands subscribed to. A subscription names the world pose it writes, so its
    reference frame is the fixed frame the pose arrives in. A detect target remains an object and
    resolves to its one world pose.

    A node that observes nothing -- a published topic -- contributes no rows, so every consumer
    passes over it without filtering.

    Raises:
        ConstraintViolation: a source observes an object no world pose is stated of, so a
            detection has nowhere to land.
    """
    graph = model.graph
    world_poses = [
        pose(model, node)
        for node in sorted(graph.subjects(RDF["type"], GEOM_COORD["PoseCoordinate"]), key=str)
        if getattr(model.context_scope(node), "section", None) == "world"
    ]
    sources = sorted(
        set(graph.subjects(RDF["type"], NS_MM_ROS["Action"]))
        | set(graph.subjects(RDF["type"], NS_MM_ROS["Topic"])),
        key=str,
    )
    written: dict[str, list[dict]] = {}
    for act in sources:
        rows = []
        for target in sorted(graph.objects(act, SOSA.hasFeatureOfInterest), key=str):
            if NS_MM_ROS["Topic"] in get_node_types(graph, act):
                item = pose(model, target)
                target_iri = item.of.uri
                rows.append(
                    {
                        "target_iri": target_iri,
                        "pose_id": item.id,
                        "frame_id": item.with_respect_to.id,
                        "frame_iri": getattr(item.with_respect_to, "uri", ""),
                    }
                )
                continue
            located = _body_or_self(model, target)
            matched = [item for item in world_poses if getattr(item.of, "id", None) == located]
            if not matched:
                raise ConstraintViolation(
                    "communication",
                    f"source '{model.id(act)}' observes '{located}', but no world pose is stated "
                    "of it, so a detection has nowhere to land",
                )
            rows.extend(
                {
                    "target_iri": str(target),
                    "pose_id": item.id,
                    "frame_id": item.with_respect_to.id,
                    # A scene object as an endpoint carries no frame IRI; the consumer that needs
                    # one to place the pose says so itself.
                    "frame_iri": getattr(item.with_respect_to, "uri", ""),
                }
                for item in matched
            )
        written[str(act)] = rows

    return written


def goal_status_act(model, node):
    """The action node whose goal status this node holds, or None if it holds no status."""
    if node is None:
        return None
    return next(
        (
            act
            for act in model.graph.objects(node, PROV.wasDerivedFrom)
            if NS_MM_ROS["Action"] in get_node_types(model.graph, act)
        ),
        None,
    )


@reader
def quantity(model, node):
    """The quantity at a node, dispatching on its RDF type."""
    viewed = model.graph.value(node, MAP.subobject)
    if viewed is not None:
        # A constraint names the per-quantity view. The subobject is the pooled relation (or
        # scalar); when the relation carries several coordinates the view's superobject pins
        # which sampling is meant, so a component-typed superobject dispatches as coordinate.
        graph = model.graph
        superobject = graph.value(node, MAP.superobject)
        subspace = graph.value(node, MAP.subspace)
        if graph.value(node, MAP.axis) is not None:
            # A single-axis view names its own scalar; only whole-component views need the
            # superobject to pin one sampling of the pooled relation.
            superobject = None
        sup_types = get_node_types(graph, superobject) if superobject is not None else set()
        if subspace == MAP_EXT.position and URI_GEOM_TYPE_POSITION_COORD in sup_types:
            node = superobject
        elif subspace == MAP_EXT.orientation and URI_GEOM_TYPE_ORIENT_COORD in sup_types:
            node = superobject
        elif subspace in (MAP_EXT.position, MAP_EXT.orientation) and superobject is not None:
            # A composite pose: its own component coordinate is the recorded selection.
            component = local_name(subspace)
            usage = graph.value(
                rdflib.URIRef(f"{superobject}-{component}-selection"), PROV.qualifiedUsage
            )
            chosen = graph.value(usage, PROV.entity) if usage is not None else None
            node = chosen if chosen is not None else viewed
        else:
            node = viewed
    if goal_status_act(model, node) is not None:
        return GoalStatus(model.id(node))
    if _is_duration(model, node):
        return duration_quantity(model, node)
    model.expect_type(node, QUDT_SCHEMA["Quantity"])
    graph = model.graph
    types = get_node_types(graph, node)
    if URI_GEOM_TYPE_POSE_COORD in types:
        return pose(model, node)
    if URI_GEOM_TYPE_POSE in types:
        return _bare_pose(model, node)
    if URI_GEOM_TYPE_POSITION in types:
        return position(model, node)
    if URI_GEOM_TYPE_ORIENT in types:
        return orientation(model, node)

    kind = QuantityKind(model.id(graph.value(node, QUDT_SCHEMA.hasQuantityKind)))
    # Values below are converted, so the unit reported alongside them is the SI one.
    unit = Unit(model.id(si_unit(graph.value(node, QUDT_SCHEMA["unit"]))))
    has_view = (node, ~MAP["subobject"], None) in graph
    provenance = quantity_provenance(model, node)

    if CSTR_HDL_EXT["SetpointGenerator"] in types:
        value_kind = next(
            (
                candidate
                for candidate in graph[node : QUDT_SCHEMA["hasQuantityKind"]]
                if candidate != CSTR_HDL_EXT.SetpointGenerator
            ),
            None,
        )

        return SetpointQuantity(
            model.id(node),
            kind,
            unit,
            has_view,
            provenance=provenance,
            value_kind=model.id(value_kind) if value_kind is not None else None,
        )
    if kind.id == "FreeVector" and GEOM_COORD["VectorXYZ"] in types:
        return FreeVector(
            model.id(node), kind, unit, parse_xyz(model, node), has_view, provenance=provenance
        )

    value = None
    if (node, QUDT_SCHEMA["value"], None) in graph:
        authored = graph.value(node, QUDT_SCHEMA["value"])
        value = si(float(authored), graph.value(node, QUDT_SCHEMA["unit"]))
    reference = graph.value(node, CSTR["reference-value"])

    return Quantity(
        model.id(node),
        kind,
        unit,
        value,
        has_view,
        provenance=provenance,
        reference_value=model.id(reference) if reference is not None else None,
    )


@reader
def duration_quantity(model, node) -> Quantity:
    """An authored duration or a runtime elapsed-duration coordinate, in seconds."""
    if not _is_duration(model, node):
        raise ValueError(f"Expected a duration at '{node}'")
    # An elapsed coordinate has no authored value; the clock fills it at runtime.
    authored = model.graph.value(node, QUDT_SCHEMA["value"])
    value = (
        None
        if authored is None
        else seconds(float(authored), model.graph.value(node, QUDT_SCHEMA["unit"]))
    )

    return Quantity(
        model.id(node),
        QuantityKind("Duration"),
        Unit("Second"),
        value,
        False,
        provenance=quantity_provenance(model, node),
    )


@reader
def joint_position(model, node) -> JointPosition:
    """A JointPosition quantity, named by the joint it reads."""
    model.expect_type(node, KC_STAT["JointPositionCoordinate"])
    model.expect_type(node, KC_STAT["JointReference"])
    joint = model.graph.value(node, KC_STAT["of-joint"])
    if not isinstance(joint, URIRef):
        raise ConstraintViolation(
            "kinematic-chain", f"JointPositionCoordinate '{node}' has no of-joint URI"
        )
    return JointPosition(
        model.id(node), model.label(joint), normalization=_normalization(model, node)
    )


def _normalization(model, node) -> dict | None:
    """The interval a measurement is read into, as its world block stated it."""
    graph = model.graph
    interval = graph.value(node, ALGO_EXT["normalization"])
    if interval is None:
        return None
    bounds = {}
    for edge in ("lower", "upper"):
        bound = graph.value(interval, ALGO_EXT[f"{edge}-bound"])
        value = graph.value(bound, QUDT_SCHEMA.value) if bound is not None else None
        if value is None:
            return None
        bounds[edge] = float(value.toPython())
    return bounds


# The predicates whose presence means a user wrote the value down.
_AUTHORED_VALUE_PREDICATES = (
    QUDT_SCHEMA["value"],
    CSTR["reference-value"],
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


def _is_authored(model, node) -> bool:
    """True when a quantity's value was authored by the user rather than computed."""
    # A path parameter's value is only its starting point; the traversal drives it per tick.
    if (None, GEOM_OP_EXT["path-parameter"], node) in model.graph:
        return False
    return any((node, predicate, None) in model.graph for predicate in _AUTHORED_VALUE_PREDICATES)


def _is_snapshot(model, node) -> bool:
    """Whether a node is itself a runtime sample-and-hold capture target. Its own graph fact
    -- not carried by `Provenance`, which only states whether a value was authored.

    A snapshot derives from its source (`prov:wasDerivedFrom`) and is sampled on a scheduled
    event (a time constraint stated `of-constraint` the quantity); a sensor tare carries the
    schedule but no derivation, and a derived slot the derivation but no schedule.
    """
    graph = model.graph
    return (
        graph.value(node, PROV.wasDerivedFrom) is not None
        and next(graph.subjects(URI_TIME_PRED_OF_CONSTRAINT, node), None) is not None
    )


def _is_config_pose(model, node) -> bool:
    """Whether a pose's value is read from the deployment config file rather than authored in
    the model. Its orientation is stamped Euler-typed by the DSL for lack of an authored
    coordinate to inspect -- that stamp must not be read as a literal, decomposed pose.
    """
    return (node, EXEC["has-resource"], None) in model.graph


@reader
def snapshot_target_ids(model) -> frozenset:
    """Every id whose node is a snapshot target, for readers that hold a record id rather than a
    graph node and so cannot call `_is_snapshot` directly.
    """
    graph = model.graph
    return frozenset(
        model.id(node)
        for node in set(graph.objects(None, URI_TIME_PRED_OF_CONSTRAINT))
        if graph.value(node, PROV.wasDerivedFrom) is not None
    )


def quantity_provenance(model, node) -> Provenance:
    """Where a quantity's value comes from. Authored and snapshot are mutually exclusive: a
    snapshot wins, because its authored literal is only the value it holds before the first
    capture.
    """
    return Provenance(authored=not _is_snapshot(model, node) and _is_authored(model, node))


@reader
def simplicial_complex(model, node) -> SimplicialComplex:
    """A rigid body, mapping a body-origin frame to the body itself."""
    types = get_node_types(model.graph, node)
    if not {GEOM_ENT.SimplicialComplex, GEOM_ENT.Frame} & types:
        raise ConstraintViolation("geometry", f"Expected a rigid body or frame, got: {node}")
    return SimplicialComplex(_body_or_self(model, node), uri=str(node))


@reader
def frame(model, node) -> Frame:
    """A reference frame, mapping a scene-dsl body-origin frame to its runtime body."""
    model.expect_type(node, GEOM_ENT["Frame"])
    return Frame(_body_or_self(model, node), uri=str(node))


def body_of(model, node):
    """The rigid body a frame is a simplex of, or None when no body carries it."""
    return next(
        (
            owner
            for owner in model.graph.subjects(GEOM_ENT.simplices, node)
            if GEOM_ENT.RigidBody in get_node_types(model.graph, owner)
        ),
        None,
    )


def placement_frame(model, node):
    """The frame a body is at, or the node itself when it already is one."""
    if GEOM_ENT.Frame in get_node_types(model.graph, node):
        return node
    return model.graph.value(node, NS_MM_KC_EXT["root"])


def _body_or_self(model, node) -> str:
    """The id of the body a frame is the root of, or the node's own id.

    The body states its root in the graph, so that relation answers this rather than the
    frame's name: a root named anything but `<body>_origin` stands for its body just as much.
    """
    body = body_of(model, node)
    if body is not None and placement_frame(model, body) == node:
        return model.id(body)

    return model.id(node)


@reader
def point(model, node) -> Point:
    """A Point, such as a frame origin."""
    if not {GEOM_ENT.Point, GEOM_ENT.Frame} & get_node_types(model.graph, node):
        raise ConstraintViolation("geometry", f"Expected a point or frame, got: {node}")
    return Point(model.id(node), uri=str(node))


@reader
def scene_object(model, node) -> SceneObject:
    """A scene object referenced as a spatial endpoint."""
    model.expect_type(node, ENV.RigidObject)
    return SceneObject(model.id(node), model.id(node))


@reader
def constraint(model, node) -> Constraint:
    """A Constraint: the quantity it bounds, together with its parameter.

    A goal status is not measured against an operand of its own kind: the status it must reach
    rides on the evaluator, so the constraint states the slot and nothing else.
    """
    model.expect_type(node, CSTR["Constraint"])
    quantity_node = model.graph.value(node, CSTR["quantity"])
    if goal_status_act(model, quantity_node) is not None:
        return Constraint(model.id(node), quantity(model, quantity_node), None)
    types = get_node_types(model.graph, node)
    if CSTR["EqualityConstraint"] in types:
        parameter = _equality_constraint(model, node)
    elif CSTR["UnilateralConstraint"] in types:
        parameter = _unilateral_constraint(model, node)
    elif CSTR_EXT["OutsideConstraint"] in types:
        parameter = _outside_constraint(model, node)
    else:
        parameter = _bilateral_constraint(model, node)

    return Constraint(
        model.id(node), quantity(model, model.graph.value(node, CSTR["quantity"])), parameter
    )


def _threshold(model, node, predicate) -> Quantity:
    return quantity(model, model.graph.value(node, predicate))


@reader
def _equality_constraint(model, node) -> EqualityConstraint:
    model.expect_type(node, CSTR["EqualityConstraint"])
    return EqualityConstraint(_threshold(model, node, CSTR["reference-value"]))


@reader
def _unilateral_constraint(model, node) -> UnilateralConstraint:
    model.expect_type(node, CSTR["UnilateralConstraint"])
    type_ = UnilateralConstraintType.LessThan
    if CSTR["GreaterThanConstraint"] in get_node_types(model.graph, node):
        type_ = UnilateralConstraintType.GreaterThan
    return UnilateralConstraint(type_, _threshold(model, node, CSTR["threshold"]))


@reader
def _bilateral_constraint(model, node) -> BilateralConstraint:
    model.expect_type(node, CSTR["BilateralConstraint"])
    return BilateralConstraint(
        _threshold(model, node, CSTR["lower-threshold"]),
        _threshold(model, node, CSTR["upper-threshold"]),
    )


@reader
def _outside_constraint(model, node) -> OutsideConstraint:
    model.expect_type(node, CSTR_EXT["OutsideConstraint"])
    return OutsideConstraint(
        _threshold(model, node, CSTR["lower-threshold"]),
        _threshold(model, node, CSTR["upper-threshold"]),
    )


# The superobject reader each view type dispatches to.
_VIEW_SUPEROBJECTS = (
    (MAP_EXT["PoseCoordinateView"], pose),
    (MAP_EXT["VelocityTwistCoordinateView"], velocity_twist),
    (MAP_EXT["AccelerationTwistCoordinateView"], acceleration_twist),
    (MAP_EXT["PoseDifferenceView"], pose_difference),
    (MAP_EXT["WrenchCoordinateView"], wrench),
)
# A combined PoseCoordinate is also a PositionCoordinate and an OrientationCoordinate; dispatch
# the most specific type first, and dedupe by id afterwards.
_DATA_STRUCTURE_READERS = (
    (GEOM_COORD["DirectionCoordinate"], direction),
    (GEOM_COORD["PoseCoordinate"], pose),
    (GEOM_COORD["PositionCoordinate"], position),
    (GEOM_COORD["OrientationCoordinate"], orientation),
    (GEOM_COORD["VelocityTwistCoordinate"], velocity_twist),
    (GEOM_COORD["AccelerationTwistCoordinate"], acceleration_twist),
    (GEOM_COORD["PoseDifferenceCoordinate"], pose_difference),
    (RBDYN_COORD["WrenchCoordinate"], wrench),
    (QUDT_SCHEMA["Quantity"], quantity),
)


def read_views(model) -> dict:
    """Every MAP view in the graph, by view id.

    Returns:
        one `View` per `map:View` node, carrying the superobject it cuts, the subobject it names,
        the subspace and, when it selects one, the axis

    Raises:
        ConstraintViolation: a view's type matches no superobject reader.
    """
    graph = model.graph
    views = {}
    # sorted(): views_for_access keeps the first view seen for a subobject, so an unordered walk
    # would publish a different (equivalent) view id on every generation.
    for node in sorted(graph[: RDF["type"] : MAP["View"]]):
        types = get_node_types(graph, node)
        superobject_node = graph.value(node, MAP["superobject"])
        read = next((func for type_, func in _VIEW_SUPEROBJECTS if type_ in types), quantity)
        superobject = read(model, superobject_node)
        if superobject is None:
            raise ConstraintViolation(
                "geometry", f"MAP view {node} has an unrecognized type; no view reader matched"
            )
        axis_node = graph.value(node, MAP["axis"])
        views[model.id(node)] = View(
            model.id(node),
            superobject,
            # The view itself, not its bare subobject: a pooled relation has several
            # coordinates and the view's superobject pins which sampling is meant.
            quantity(model, node),
            subspace(graph.value(node, MAP["subspace"])),
            axis(axis_node) if axis_node is not None else None,
        )

    return views


def _is_pooled_relation(model, node) -> bool:
    """A relation sampled by several coordinates: the samplings are the data, not the relation."""
    types = get_node_types(model.graph, node)
    if URI_GEOM_TYPE_POSITION in types and URI_GEOM_TYPE_POSITION_COORD not in types:
        return len(PositionModel(node, model.graph).coordinate_ids) > 1
    if URI_GEOM_TYPE_ORIENT in types and URI_GEOM_TYPE_ORIENT_COORD not in types:
        return len(OrientationModel(node, model.graph).coordinate_ids) > 1
    return False


def read_data_structures(model) -> list:
    """Every data-structure entity in the graph.

    Returns:
        the records, grouped by type in most-specific-first order and deduplicated by id, so a
        combined pose coordinate is read as a pose rather than as its position half
    """
    return dedupe_by_id(
        [
            read(model, node)
            # Sort within the type group: keeps the most-specific-first dispatch the dedupe
            # relies on, while making the published order reproducible.
            for type_, read in _DATA_STRUCTURE_READERS
            for node in sorted(model.graph[: RDF["type"] : type_])
            # A pooled relation is no slot of its own: each of its samplings is one.
            if not _is_pooled_relation(model, node)
        ]
    )


def views_by_subobject(views: dict) -> dict[str, list]:
    """Every view onto each subobject id."""
    indexed: dict[str, list] = {}
    for view in views.values():
        subobject_id = getattr(view.subobject, "id", None)
        if subobject_id:
            indexed.setdefault(subobject_id, []).append(view)
    return indexed


def _unique_view(indexed, subobject_id, context):
    """The one view onto a subobject, or None; several disagreeing views is an error."""
    matches = indexed.get(subobject_id, ())
    if len(matches) > 1:
        raise ConstraintViolation(
            "geometry",
            f"{context}: quantity '{subobject_id}' is the subobject of multiple MAP views",
        )
    return matches[0] if matches else None


def views_for_access(
    views: dict, shared_data: list, motions, closures: dict, pose_components: dict
) -> dict:
    """Unambiguous MAP views by subobject, for the view's access expressions.

    A subobject written directly -- an authored or literal shared value, a snapshot target, a
    closure output -- is an ordinary shared quantity and keeps its own field; one reused by views
    that disagree on how they access it must not silently pick one of them.

    Raises:
        ConstraintViolation: a subobject is left with neither a direct write nor one agreed
            reading, so nothing could compute it.
    """
    # A declared pose's components are bound *into* it. Reading one back off the pose it helps
    # define is circular -- and it is the quantity's own superobject that says how to read it --
    # so the binding is not a candidate reading, however much it looks like one.
    bound_into_pose = {
        (pose_id, component["ref"])
        for pose_id, parts in pose_components.items()
        for component in asdict(parts).values()
        if isinstance(component, dict) and component.get("ref")
    }
    direct_ids = {
        item.id
        for item in shared_data
        if getattr(item, "id", None)
        and (
            getattr(item, "value", None) is not None
            or getattr(getattr(item, "provenance", None), "authored", False)
        )
    }
    direct_ids.update(snapshot.target_id for motion in motions for snapshot in motion.snapshots)
    direct_ids.update(
        output_id for closure in closures.values() for output_id in closure_output_ids(closure)
    )

    indexed: dict[str, object] = {}
    for view in views.values():
        # A constraint may name the view itself (relations are pooled), so every view also
        # maps under its own id; view ids are unique, so this never conflicts.
        indexed.setdefault(view.id, view)
        subobject_id = getattr(view.subobject, "id", None)
        if not subobject_id or subobject_id in direct_ids:
            continue
        if subobject_id == getattr(view.superobject, "id", None):
            # A whole-component view of a pooled relation reads the superobject's own
            # sampling; the superobject is computed elsewhere, not through this view.
            continue
        if (getattr(view.superobject, "id", None), subobject_id) in bound_into_pose:
            continue
        previous = indexed.setdefault(subobject_id, view)
        if previous is view or previous is None:
            continue
        if any(
            getattr(previous, name) != getattr(view, name)
            for name in ("superobject", "subspace", "axis", "direction")
        ):
            indexed[subobject_id] = None

    # Dropping the reading here used to leave the quantity to render as its own shared field --
    # a field nothing writes, which compiles to a zero and flies the robot at it. Nothing can
    # compute this quantity, so say so instead of emitting the zero.
    unreadable = sorted(id_ for id_, view in indexed.items() if view is None)
    if unreadable:
        raise ConstraintViolation(
            "geometry",
            "no way to compute "
            + ", ".join(f"'{id_}'" for id_ in unreadable)
            + ": neither written directly nor read through a single agreed MAP view",
        )

    return dict(indexed)


def expanded_constraints(model, nodes) -> set:
    """A phase's constraints, with any when/until aggregate replaced by its members."""
    return {
        member
        for node in nodes
        for member in (
            model.graph[node : CSTR_EXT["has-constraint"]]
            if is_constraint_aggregate(model, node)
            else (node,)
        )
    }


def relative_poses_for_motion(evaluators, views: dict, serial_chain_solvers) -> list:
    """Poses stated with respect to a `_start` frame, paired with the FK output they capture."""
    fk_poses = {
        out.of.id: out.id
        for solver in serial_chain_solvers
        for out in solver.output
        if getattr(out, "type", "") == "Pose" and getattr(getattr(out, "of", None), "id", None)
    }
    indexed = views_by_subobject(views)
    start_relative: dict[str, object] = {}
    for evaluator in evaluators:
        quantity_record = getattr(getattr(evaluator, "constraint", None), "quantity", None)
        if quantity_record is None or not getattr(quantity_record, "has_view", False):
            continue
        view = _unique_view(indexed, quantity_record.id, "relative pose lookup")
        if view is None:
            continue
        wrt = getattr(view.superobject, "with_respect_to", None)
        if wrt and getattr(wrt, "id", "").endswith("_start"):
            start_relative[view.superobject.id] = view.superobject

    result = []
    for pose_id, pose_record in start_relative.items():
        of_id = getattr(getattr(pose_record, "of", None), "id", None)
        fk_pose_id = fk_poses.get(of_id) if of_id else None
        if fk_pose_id:
            result.append(RelativePoseCapture(id=pose_id, fk_pose_id=fk_pose_id))

    return result


class _ScenePose(NamedTuple):
    """The solver output tracking a scene object, and the frame it is stated against."""

    pose_id: str
    with_respect_to: str | None


class _TrackedPoses(NamedTuple):
    """Which FK output tracks each frame, and which tracks each scene object's body."""

    fk_by_frame: dict[str, str]
    scene_by_id: dict[str, _ScenePose]


def _fk_and_scene_poses(serial_chain_solvers, views, solvers_by_id: dict) -> _TrackedPoses:
    """Which FK output tracks each frame, and which tracks each scene object's body.

    `serial_chain_solvers` are per-motion slices: their solver is resolved through
    `solvers_by_id`, as templates do through `resources.by_id`.
    """
    fk_by_frame: dict[str, str] = {}
    # Keyed by the scene-object's id; a wrt_id lookup asks whether that frame is the subject of a
    # tracked scene-object pose.
    scene_by_id: dict[str, _ScenePose] = {}
    output_ids: set[str] = set()
    for solver in serial_chain_solvers:
        for out in solver.output:
            if getattr(out, "type", "") != "Pose":
                continue
            output_ids.add(out.id)
            of = out.of
            if of is None:
                chain_end = solvers_by_id[solver.solver_id].chain.end
                if chain_end:
                    fk_by_frame.setdefault(chain_end, out.id)
                continue
            if getattr(of, "is_scene_object", False):
                entry = _ScenePose(
                    out.id, getattr(getattr(out, "with_respect_to", None), "id", None)
                )
                scene_by_id[of.id] = entry
                if getattr(of, "body", None):
                    scene_by_id[of.body] = entry
            else:
                fk_by_frame[of.id] = out.id

    for view in views.values():
        superobject = view.superobject
        if not hasattr(superobject, "of") or not hasattr(superobject, "with_respect_to"):
            continue
        if superobject.id not in output_ids:
            continue
        of = superobject.of
        if of is None or getattr(of, "is_scene_object", False):
            continue
        fk_by_frame.setdefault(getattr(of, "id", ""), superobject.id)

    return _TrackedPoses(fk_by_frame, scene_by_id)


def scene_relative_poses_for_motion(
    views: dict, serial_chain_solvers, solvers_by_id: dict, evaluators=()
) -> list:
    """For each pose stated with respect to a scene object, the relative pose it asks for."""
    fk_by_frame, scene_by_id = _fk_and_scene_poses(serial_chain_solvers, views, solvers_by_id)
    candidates = [
        view.superobject
        for view in views.values()
        if not getattr(view.superobject, "authored", False)
    ]
    candidates.extend(
        evaluator.constraint.quantity
        for evaluator in evaluators
        if getattr(getattr(evaluator, "constraint", None), "quantity", None) is not None
        and hasattr(evaluator.constraint.quantity, "of")
        and hasattr(evaluator.constraint.quantity, "with_respect_to")
    )

    seen: set[str] = set()
    result: list[SceneRelativePose] = []
    for candidate in candidates:
        pose_id = getattr(candidate, "id", None)
        of = getattr(candidate, "of", None)
        wrt = getattr(candidate, "with_respect_to", None)
        if not pose_id or pose_id in seen or of is None or wrt is None:
            continue
        if getattr(of, "is_scene_object", False) or of.id in scene_by_id:
            continue
        scene_pose = scene_by_id.get(getattr(wrt, "id", ""))
        if scene_pose is None and not getattr(wrt, "is_scene_object", False):
            continue
        fk_pose_id = fk_by_frame.get(getattr(of, "id", ""))
        if not fk_pose_id or not scene_pose:
            continue
        seen.add(pose_id)
        result.append(
            SceneRelativePose(
                id=pose_id,
                fk_pose_id=fk_pose_id,
                scene_pose_id=scene_pose.pose_id,
                base_seen=getattr(getattr(candidate, "as_seen_by", None), "id", None)
                == scene_pose.with_respect_to,
            )
        )

    return result


_GROUPABLE_SUPEROBJECTS = {"Pose", "VelocityTwist", "AccelerationTwist", "Wrench"}
_GROUP_AXIS_BY_SUBSPACE = {Subspace.Linear: ("linear", False), Subspace.Angular: ("angular", True)}


def pose_axis_error_groups_for_motion(model, evaluators_by_node: dict, views: dict) -> list:
    """A motion's per-axis error evaluators, regrouped into one group per superobject.

    Parameters:
        evaluators_by_node: the phase's evaluator records, keyed by the node each was read from

    Returns:
        the groups with more than one component; a lone axis needs no regrouping, its own
        equality-constraint controller drives it
    """
    groups: dict[str, PoseErrorRegroup] = {}
    indexed = views_by_subobject(views)
    for node, evaluator in evaluators_by_node.items():
        if CSTR_HDL["ErrorEvaluator"] not in get_node_types(model.graph, node):
            continue
        if not isinstance(evaluator.constraint.parameter, EqualityConstraint):
            continue
        if evaluator.error is None:
            continue
        quantity_record = evaluator.constraint.quantity
        view = _unique_view(indexed, quantity_record.id, "pose-axis error grouping")
        if view is None:
            continue
        superobject_type = getattr(view.superobject, "type", None)
        if superobject_type not in _GROUPABLE_SUPEROBJECTS:
            continue
        # Which half of the superobject the view selects, or None when it joins no group.
        mapping = _GROUP_AXIS_BY_SUBSPACE.get(view.subspace)
        if mapping is None and superobject_type == "Pose":
            kind = getattr(getattr(quantity_record, "quantity_kind", None), "id", "")
            if "Angle" in kind or "angle" in kind.lower() or "rotation" in quantity_record.id:
                mapping = ("angular", True)
        # A whole-subspace view has no per-axis component, so it cannot join a per-axis group;
        # its own equality-constraint controller drives it.
        if mapping is None or view.axis is None:
            continue
        half, is_angular = mapping

        superobject_id = view.superobject.id
        group = groups.setdefault(
            superobject_id,
            PoseErrorRegroup(
                id=f"pose_axis_error_{superobject_id}",
                pose=superobject_id,
                components=[],
                superobject_type=superobject_type,
            ),
        )
        group.has_angular = group.has_angular or is_angular
        reference_id = evaluator.constraint.parameter.reference_value.id
        group.components.append(
            PoseErrorComponent(
                quantity=quantity_record.id,
                error=evaluator.error.id,
                reference=reference_id,
                subspace=half,
                axis=view.axis.value,
                eval_id=evaluator.id,
            )
        )
        setattr(group, f"{half}_{view.axis.value.lower()}", reference_id)

    return [group for group in groups.values() if len(group.components) > 1]


def _reference_value_id(constraint_record) -> str | None:
    """The id of a constraint's reference-value parameter, or None."""
    parameter = getattr(constraint_record, "parameter", None)
    reference = getattr(parameter, "reference_value", None) if parameter else None
    return getattr(reference, "id", None)


def snapshots_for_motion(
    evaluators, constraints, indexes, views: dict, schedule, closures: dict, token, tokens
) -> list:
    """A motion's sample-and-hold captures, from every reference value it reaches.

    Parameters:
        token: the motion's own suffix, as `snapshot_owner` names declaring motions
        tokens: every motion's suffix, so an unowned snapshot can be told from another's

    Returns:
        one capture per snapshot target this motion may sample, sorted by target id
    """
    referenced = {
        reference
        for record in [
            *(e.constraint for e in evaluators if e.constraint is not None),
            *constraints,
        ]
        if (reference := _reference_value_id(record))
    }
    subobjects_by_super: dict[str, list[str]] = {}
    supers_by_subobject: dict[str, list[str]] = {}
    for view in views.values():
        super_id = getattr(view.superobject, "id", None)
        subobject_id = getattr(view.subobject, "id", None)
        if super_id and subobject_id:
            subobjects_by_super.setdefault(super_id, []).append(subobject_id)
            supers_by_subobject.setdefault(subobject_id, []).append(super_id)

    def expand(reference_id) -> None:
        """Record every id reachable from one reference, in both view directions."""
        if not isinstance(reference_id, str):
            return
        pending = [reference_id]
        seen = set()
        while pending:
            current = pending.pop()
            if current in seen:
                continue
            seen.add(current)
            referenced.add(current)
            referred = indexes.data_reference.get(current)
            if referred:
                pending.append(referred)
            # superobject -> subobject (forward decomposition), and subobject -> superobject (a
            # composite pose's snapshot is only referenced through its scalar components, so
            # climb back to capture the composite).
            pending.extend(subobjects_by_super.get(current, ()))
            pending.extend(supers_by_subobject.get(current, ()))
            # A composed orientation names its base pose only through the compose operator.
            parts = indexes.pose_components.get(current)
            if parts is not None:
                pending.extend(
                    operand["pose"]
                    for operand in parts.orientation_operands or ()
                    if isinstance(operand.get("pose"), str)
                )

    for reference_id in list(referenced):
        expand(reference_id)
    for step in schedule:
        for value in (closures.get(step) or {}).values():
            for item in value if isinstance(value, list) else [value]:
                expand(item)

    result = []
    for target_id in sorted(referenced):
        if target_id not in indexes.snapshot_source:
            continue
        # Capture only what this motion declares: re-capturing another motion's snapshot would
        # overwrite its value. A shared-context snapshot is owned by no motion, so every motion
        # that reads it emits the capture -- guarded once for the run, not once per activation.
        owner = indexes.snapshot_owner.get(target_id)
        if owner in tokens and owner != token:
            continue
        trigger = indexes.snapshot_trigger.get((token, target_id))
        source_id = indexes.snapshot_source[target_id]
        result.append(
            SnapshotCapture(
                target_id=target_id,
                source_id=source_id,
                scope="event" if trigger else ("entry" if owner in tokens else "task"),
                source_closure_id=(
                    None
                    if source_id in supers_by_subobject
                    else indexes.closure_output.get(source_id)
                ),
                trigger_event=trigger,
            )
        )

    return result


def collect_motion_references(motion, closures: dict) -> set[str]:
    """Every id a motion references, including through the closures its schedules run.

    Returns:
        every string the motion record or one of its closures holds -- a superset of the ids,
        which is what the callers restrict a global table down to
    """
    references: set[str] = set()

    def visit(value) -> None:
        """Collect every string a value holds, however deeply nested."""
        if isinstance(value, str):
            references.add(value)
        elif isinstance(value, dict):
            for item in value.values():
                visit(item)
        elif isinstance(value, list):
            for item in value:
                visit(item)

    visit(asdict(motion))
    for name in ("when_schedule", "while_schedule", "until_schedule"):
        for step in getattr(motion, name):
            visit(closures.get(step))

    return references


def collect_motion_input_references(motion, closures: dict) -> set[str]:
    """Every value the motion's computations and captures read, excluding solver outputs."""
    references: set[str] = set()

    def visit(value) -> None:
        if isinstance(value, str):
            references.add(value)
        elif isinstance(value, dict):
            for item in value.values():
                visit(item)
        elif isinstance(value, list):
            for item in value:
                visit(item)

    for name in ("when_schedule", "while_pre_schedule", "while_schedule", "until_schedule"):
        for step in getattr(motion, name):
            visit(closures.get(step))
    visit([asdict(snapshot) for snapshot in motion.snapshots])
    return references


def elapsed_coordinate_id(evaluator) -> str:
    """The shared value an elapsed constraint measures: its own authored duration coordinate.

    The error signal is the elapsed duration itself, so this is where the motion writes the
    seconds and where the condition, the introspection sample and the frame log all find them.
    """
    coordinate = getattr(evaluator.error, "id", None)
    if not coordinate:
        raise ConstraintViolation(
            "quantity",
            f"elapsed constraint '{evaluator.id}' has no duration coordinate to measure into",
        )
    return coordinate


def elapsed_coordinate_ids(evaluators) -> list[str]:
    """A phase's elapsed coordinates, deduplicated, in authored order."""
    return list(
        dict.fromkeys(
            elapsed_coordinate_id(evaluator) for evaluator in evaluators if evaluator.is_elapsed
        )
    )


_POSITION_FIELDS = ("position_x", "position_y", "position_z")


def _required_pose_component_fields(representation: str, euler_axes_sequence: str | None) -> tuple:
    """The component fields a pose's `representation` requires: position always;
    `orientation_x/y/z/w` for `quaternion`; one `orientation_*` per character of the Euler
    sequence for `euler`; none for `relative`. Stated once; both the completeness check below
    and `_euler_factors` read it.
    """
    if representation == "quaternion":
        return _POSITION_FIELDS + (
            "orientation_x",
            "orientation_y",
            "orientation_z",
            "orientation_w",
        )
    if representation == "euler":
        return _POSITION_FIELDS + tuple(
            f"orientation_{name}" for name in (euler_axes_sequence or "")
        )

    return _POSITION_FIELDS


def _pose_component(component_id: str, data_by_id: dict) -> ComponentRef:
    """A pose component as either a literal `value` or a `ref` id the backend template renders
    via access-expr. Backend-agnostic -- no target syntax here.
    """
    component = data_by_id.get(component_id)
    reference = getattr(component, "reference_value", None)
    if reference:
        return ComponentRef(ref=reference)
    value = getattr(component, "value", None)
    if value is not None:
        return ComponentRef(value=str(value))
    return ComponentRef(ref=component_id)


def _authored_pose_entry(model, pose_record, coordinate_node) -> PoseComponents | None:
    """The literal components a coordinate-authored pose carries, or None when it carries none."""
    coordinate = PoseCoordModel(coordinate_node, model.graph, coord_policy=recorded_coord_policy)
    representation = pose_record.orientation_representation or "quaternion"
    entry = PoseComponents(representation)
    values = position_values(model, coordinate.position_coord)
    if values is not None:
        for name, value in zip("xyz", values):
            setattr(entry, f"position_{name}", ComponentRef(value=str(value)))
    if representation == "quaternion":
        # Euler angles, quaternion or direction cosines all resolve to one quaternion here;
        # anything sourced at runtime keeps its shape and is rendered, not resolved.
        rotation = orientation_quaternion(model, coordinate.orientation_coord)
        for name, value in zip("xyzw", rotation or ()):
            setattr(entry, f"orientation_{name}", ComponentRef(value=str(value)))
    if representation == "relative":
        entry.orientation_operands = pose_record.orientation_operands
    filled = any(
        value is not None for key, value in asdict(entry).items() if key != "representation"
    )

    return entry if filled else None


def _build_pose_components(model, views: dict, data: list) -> dict:
    """Declared and inline poses as typed, fully-published `PoseComponents` (DECISION 10)."""
    data_by_id = {item.id: item for item in data if getattr(item, "id", None)}
    pose_nodes = {
        model.id(node): node for node in model.graph.subjects(RDF.type, URI_GEOM_TYPE_POSE_COORD)
    }
    components: dict[str, PoseComponents] = {}
    for item in data:
        if item.type != "Pose" or item.id not in pose_nodes:
            continue
        entry = _authored_pose_entry(model, item, pose_nodes[item.id])
        if entry is not None:
            components[item.id] = entry

    snapshot_ids = snapshot_target_ids(model)
    for view in views.values():
        superobject = view.superobject
        # A snapshot pose is captured whole at runtime, so a per-axis view of it is a reading
        # off the captured frame, never a component bound into the pose.
        if superobject.type != "Pose" or superobject.id in snapshot_ids:
            continue
        if not (superobject.provenance.authored or superobject.euler_axes_sequence):
            continue
        component_axis = str(view.axis.value if view.axis else "").lower()
        subobject_id = getattr(view.subobject, "id", None)
        if not subobject_id or component_axis not in {"x", "y", "z", "w"}:
            continue
        representation = superobject.orientation_representation or "quaternion"
        entry = components.setdefault(superobject.id, PoseComponents(representation))
        if representation == "relative":
            entry.orientation_operands = superobject.orientation_operands
        prefix = "position" if view.subspace == Subspace.Linear else "orientation"
        setattr(entry, f"{prefix}_{component_axis}", _pose_component(subobject_id, data_by_id))

    for pose_id, parts in components.items():
        euler_axes_sequence = getattr(data_by_id.get(pose_id), "euler_axes_sequence", None)
        required = _required_pose_component_fields(parts.representation, euler_axes_sequence)
        missing = [name for name in required if getattr(parts, name) is None]
        if missing:
            raise ConstraintViolation(
                "geometry",
                f"Declared pose '{pose_id}' is missing required components: {', '.join(missing)}.",
            )
        if parts.representation == "euler":
            parts.euler_factors = _euler_factors(pose_id, parts, data_by_id)

    return components


def _euler_factors(pose_id: str, parts: PoseComponents, data_by_id: dict) -> list[dict]:
    """A symbolic Euler triple as per-axis rotations, in the order they multiply.

    An extrinsic sequence turns about axes that stay put, so the rotation authored last multiplies
    on the left; an intrinsic one turns about axes carried along by the previous rotations, so the
    order reverses. Each component renders wherever its value comes from.
    """
    pose_record = data_by_id.get(pose_id)
    sequence = getattr(pose_record, "euler_axes_sequence", None) or "xyz"
    factors = [
        {"axis": name, "component": getattr(parts, f"orientation_{name}")}
        for name in sequence
        if getattr(parts, f"orientation_{name}") is not None
    ]
    if len(factors) != len(sequence):
        raise ConstraintViolation(
            "geometry", f"Euler pose '{pose_id}' has no component for every axis of '{sequence}'."
        )

    return factors if getattr(pose_record, "euler_intrinsic", False) else list(reversed(factors))


def declared_pose_component_entries(
    model, data: list, pose_components: dict, referenced=None
) -> list:
    """Authored declared-pose component entries, restricted to the ids a motion references."""
    data_by_id = {item.id: item for item in data if getattr(item, "id", None)}
    snapshot_ids = snapshot_target_ids(model)
    entries = []
    for pose_id, parts in pose_components.items():
        if referenced is not None and pose_id not in referenced:
            continue
        provenance = getattr(data_by_id.get(pose_id), "provenance", None)
        if provenance is None or not provenance.authored or pose_id in snapshot_ids:
            continue
        entries.append({"id": pose_id, **asdict(parts)})

    return entries


@dataclass(frozen=True)
class ComputationIndexes:
    """Everything a motion asks about the computation, resolved once before any motion is built."""

    snapshot_source: dict
    snapshot_owner: dict
    snapshot_trigger: dict
    closure_owner: dict
    closure_output: dict
    closure_input: dict
    data_reference: dict
    pose_components: dict


class _SnapshotMaps(NamedTuple):
    """The three things a snapshot output is looked up by."""

    source: dict
    owner: dict
    trigger: dict


def _snapshot_maps(model) -> _SnapshotMaps:
    """One walk over the snapshots for the three maps codegen asks about.

    ``source``: each snapshot output to its source quantity. ``owner``: each output to the motion
    declaring it, taken from the motion segment of the quantity URI -- every motion captures each
    snapshot it references and they share one slot, so without an owner a motion silently
    retargets another's. ``trigger``: each event-triggered snapshot to its trigger event's local
    name, keyed by (declaring motion, output id) since only the owner re-samples.
    """
    graph = model.graph
    source: dict[str, str] = {}
    owner: dict[str, str] = {}
    trigger: dict[tuple[str, str], str] = {}
    for schedule in graph.subjects(RDF.type, URI_TIME_TYPE_AFTER_EVT):
        output_node = graph.value(schedule, URI_TIME_PRED_OF_CONSTRAINT)
        if output_node is None:
            continue
        source_node = graph.value(output_node, PROV.wasDerivedFrom)
        if source_node is None:
            # A sensor tare: scheduled the same way, but nothing is derived.
            continue
        output_id = model.id(output_node)
        source[output_id] = model.id(source_node)
        scope = model.context_scope(output_node)
        if scope is None:
            continue
        owner[output_id] = get_valid_var_name(scope[0])
        trigger_node = graph.value(schedule, URI_TIME_PRED_AFTER_EVT)
        if trigger_node is not None:
            trigger[(owner[output_id], output_id)] = get_valid_var_name(
                local_name(trigger_node)
            ).upper()

    return _SnapshotMaps(source, owner, trigger)


@dataclass(frozen=True)
class Computation:
    """What is computed this run: the closures, the values they read and write, the views onto
    those values, and every lookup a motion makes over the three.
    """

    closures: dict
    data_structures: list
    views: dict
    indexes: ComputationIndexes


def build_indexes(model, closures: dict, data_structures: list, views: dict) -> Computation:
    """Resolve every per-motion lookup once, before any motion is built.

    Raises:
        ConstraintViolation: a declared pose is missing a component, or an Euler pose has no
            component for every axis of its sequence.
    """
    source, owner, trigger = _snapshot_maps(model)
    closure_output, closure_input = closure_maps(closures)
    indexes = ComputationIndexes(
        snapshot_source=source,
        snapshot_owner=owner,
        snapshot_trigger=trigger,
        closure_owner=closure_owner_map(model, closures),
        closure_output=closure_output,
        closure_input=closure_input,
        data_reference=data_reference_map(data_structures, closures),
        pose_components=_build_pose_components(model, views, data_structures),
    )

    return Computation(closures, data_structures, views, indexes)


def filter_shared_data(data_structures, schedule, closures: dict, views: dict, fk_output_ids):
    """The data structures that reach the blackboard.

    Anything a scheduled call, a view, a closure or an FK output names is shared. What nothing
    names is kept only if it is a value in its own right: a viewed quantity, a kindless one and a
    whole pose or twist are all projections of something else and drop out.

    Parameters:
        schedule: every scheduled call id, across all four solver-section schedules
        fk_output_ids: the ids the chain solvers write, which no schedule mentions
    """
    referenced: set[str] = set(schedule) | set(fk_output_ids)
    for closure in closures.values():
        for value in closure.values():
            # An n-ary port (algo-ext:in) arrives as a list of operand ids.
            for item in value if isinstance(value, list) else [value]:
                if isinstance(item, str):
                    referenced.add(item)
    for view in views.values():
        for endpoint in (view.superobject, view.subobject):
            if endpoint:
                referenced.add(endpoint.id)
    # A composed orientation's base pose is read by the pose materializer, not by any
    # schedule, closure or view.
    for item in data_structures:
        for operand in getattr(item, "orientation_operands", None) or ():
            base = operand.get("pose")
            if isinstance(base, str):
                referenced.add(base)

    result = []
    for item in data_structures:
        if item.id in referenced:
            result.append(item)
            continue
        if item.type == "Quantity" and item.has_view:
            continue
        if item.type == "Quantity" and item.value is None and item.quantity_kind.id is None:
            continue
        if item.type in ("Pose", "VelocityTwist"):
            continue
        result.append(item)

    return dedupe_by_id(result)


# Storage follows from write cadence, in one place. A value written once says nothing new when
# repeated per tick, and one never written is not a runtime value at all.
_STORAGE_BY_CADENCE = {"never": "absent", "init": "record", "tick": "log"}
# Shared values the control loop writes from a backend port, not from any model entity. Their
# initial literal is a fallback, not an authored constant, so the contract is stated here rather
# than inferred from the value being present.
PORT_PRODUCERS = {
    "clock_time_s": {"kind": "port", "id": "clock"},
    "dt_measured_s": {"kind": "port", "id": "clock"},
}
_MOTION_SCHEDULES = ("when_schedule", "while_pre_schedule", "while_schedule", "until_schedule")
_LITERAL_FIELDS = ("position", "direction", "orientation", "value", "vector")
# Fields on a shared-data entry that carry the numbers behind a `vec` sample descriptor.
_LITERAL_VECTORS = ("position", "direction", "vector")


class _Contract(NamedTuple):
    """What a member's dataflow entry is built from, before storage is derived."""

    producer: dict
    cadence: object


def _sole(ids) -> str | None:
    """The one id in a set, or None when several instances write the same value."""
    return next(iter(ids)) if len(ids) == 1 else None


def _constant_value(item, desc: dict):
    """The authored number a `cadence: init` sample row carries, for the schema header."""
    kind = desc.get("kind")
    if kind == "literal":
        return float(desc["value"])
    if kind in {"shared", "access", "bool", "int"}:
        return getattr(item, "value", None)
    if kind == "vec":
        for name in _LITERAL_VECTORS:
            values = getattr(item, name, None)
            if values is not None:
                return values[desc["axis"]]

    return None


class _SolverWrites(NamedTuple):
    """Which solver writes each output, and which of those are sensor readings."""

    by_output: dict
    sensor_outputs: set


def _writers_by_output(serial_chain_solvers) -> _SolverWrites:
    """Which solver writes each output, and which of those are sensor readings (the tare
    companions are written alongside the reading, not measured).
    """
    by_output: dict[str, set] = {}
    sensor_outputs: set = set()
    for solver in serial_chain_solvers:
        # Full solvers carry gripper outputs on their gripper device(s), not as a flat list.
        gripper_outputs = [out for device in solver.devices for out in device.joint_outputs]
        for out in [*solver.output, *gripper_outputs]:
            by_output.setdefault(out.id, set()).add(solver.id)
            if not getattr(out, "sensor_name", ""):
                continue
            sensor_outputs.add(out.id)
            # tare state, written alongside the reading (resources.shared_runtime_members)
            for companion in (
                f"{out.id}_ft_bias",
                f"{out.id}_ft_bias_new",
                f"{out.id}_ft_settle",
                f"{out.id}_ft_tares",
            ):
                by_output.setdefault(companion, set()).add(solver.id)
                sensor_outputs.add(companion)

    return _SolverWrites(by_output, sensor_outputs)


class _ValueOwners(NamedTuple):
    """Which motions write each value, and which values a per-motion block is responsible for."""

    owners: dict
    block_ids: dict


def _owners_by_value(motions, closures: dict) -> _ValueOwners:
    """Which motions write each value, and which values the per-motion blocks -- rather than any
    schedule -- are responsible for.

    A union, never last-writer-wins: several motions can write one value, and attributing it to a
    single motion would gate away live data.
    """
    owners: dict[str, set] = {}
    # Poses, snapshots and per-axis errors are written by the pose-composition, snapshot and
    # error-decomposition blocks, which are emitted per motion rather than scheduled as closures.
    block_ids = {"pose": set(), "snapshot": set(), "decomposition": set()}

    def own(data_id, motion_id, kind=None) -> None:
        if not isinstance(data_id, str):
            return
        owners.setdefault(data_id, set()).add(motion_id)
        if kind is not None:
            block_ids[kind].add(data_id)

    for motion in motions:
        for name in _MOTION_SCHEDULES:
            for closure_id in getattr(motion, name):
                for out_id in closure_output_ids(closures.get(closure_id) or {}):
                    own(out_id, motion.id)
        for solver in motion.serial_chain_solvers:
            for out in [*solver.output, *solver.gripper_joint_outputs]:
                own(out.id, motion.id)
                if getattr(out, "sensor_name", ""):
                    own(f"{out.id}_ft_bias", motion.id)
                    own(f"{out.id}_ft_settle", motion.id)
            # Joint-space mirrors are written by whichever motion's solver ran, so the runtime's
            # channels are live in every motion that drives it.
            for sample in [*solver.joint_space_samples, *solver.joint_space_cmd_samples]:
                own(sample["id"], motion.id)
        for entry in [*motion.declared_pose_components, *motion.relative_poses]:
            own(entry["id"] if isinstance(entry, dict) else entry.id, motion.id, "pose")
        for snapshot in motion.snapshots:
            own(snapshot.target_id, motion.id, "snapshot")
            # The source closure runs inside the snapshot block, not from a schedule.
            for out_id in closure_output_ids(closures.get(snapshot.source_closure_id) or {}):
                own(out_id, motion.id)
        for group in motion.pose_axis_error_groups:
            for component in group.components:
                own(component.error, motion.id, "decomposition")

    return _ValueOwners(owners, block_ids)


def annotate_dataflow(
    introspection: dict,
    shared_data: list,
    closures: dict,
    motions,
    serial_chain_solvers,
    views,
    subscriptions=(),
    config_poses=(),
) -> None:
    """Give every shared value its producer, its write cadence and the storage those imply, then
    apply that contract: drop what nothing writes and move what is written once into the header.

    Cadence -- not motion membership -- decides gating: a value written by several motions carries
    all of them, and one no motion's step function writes falls back to ``tick``.
    """
    closure_by_output: dict[str, set] = {}
    for closure_id, closure in closures.items():
        for out_id in closure_output_ids(closure):
            closure_by_output.setdefault(out_id, set()).add(closure_id)
    solver_by_output, sensor_outputs = _writers_by_output(serial_chain_solvers)
    owners, block_ids = _owners_by_value(motions, closures)
    # Named by the mechanism that produced the value, so the artifact says whether a pose was
    # asked for once or arrived on a standing channel.
    perceived_by_output = {
        **{
            written_id: {"kind": "action", "id": client["act_id"]}
            for motion in motions
            for client in motion.action_clients
            for written_id in (
                client["status_id"],
                *(row["pose_id"] for row in client["written_poses"]),
            )
        },
        **{
            row["pose_id"]: {"kind": "subscription", "id": sub["sub_id"]}
            for sub in subscriptions
            for row in sub["written_poses"]
        },
    }

    config_pose_ids = {entry["id"] for entry in config_poses}

    def contract(item) -> _Contract:
        """The producer and write cadence of one shared-data member."""
        if item.id in config_pose_ids:
            # Read from the deployment config before the loop, so it is live in every state. Not
            # `init`: that storage is a constant the artifact carries, and this number is only
            # known once the run has read the file it is named for.
            return _Contract({"kind": "config", "id": item.id}, "tick")
        if item.id in PORT_PRODUCERS:
            return _Contract(PORT_PRODUCERS[item.id], "tick")
        if item.id in perceived_by_output:
            # A detection lands on whatever tick it arrives on, from the executor thread rather
            # than inside a motion's step, so it is live in every state.
            return _Contract(perceived_by_output[item.id], "tick")
        motion_ids = owners.get(item.id)
        cadence = {"motions": sorted(motion_ids)} if motion_ids else "tick"
        if getattr(item, "role", None) == "joint_space":
            # Declared at the mirror site, which is the only place that knows what it reads.

            return _Contract(item.producer, cadence)
        if item.id in closure_by_output:
            producers = closure_by_output[item.id]
            types = {(closures.get(cid) or {}).get("type") for cid in producers}
            kind = "controller" if types == {"Controller"} else "closure"

            return _Contract({"kind": kind, "id": _sole(producers)}, cadence)
        if item.id in solver_by_output:
            kind = "sensor" if item.id in sensor_outputs else "solver"

            return _Contract({"kind": kind, "id": _sole(solver_by_output[item.id])}, cadence)
        for kind, ids in block_ids.items():
            if item.id in ids:
                return _Contract({"kind": kind, "id": item.id}, cadence)
        # `is not None`, not truthiness: an authored 0.0 is a value, not a missing one.
        if any(getattr(item, name, None) is not None for name in _LITERAL_FIELDS):
            return _Contract({"kind": "authored", "id": None}, "init")

        return _Contract({"kind": "none", "id": None}, "never")

    # One artifact holds the contract, so storage is derived from cadence in exactly one place.
    dataflow = {}
    items_by_id = {}
    for item in shared_data:
        if not item.id:
            continue
        items_by_id[item.id] = item
        producer, cadence = contract(item)
        dataflow[item.id] = {
            "producer": producer,
            "cadence": cadence,
            # Storage follows from cadence, here and nowhere else.
            "storage": "log" if isinstance(cadence, dict) else _STORAGE_BY_CADENCE[cadence],
        }

    _apply_view_liveness(dataflow, views)
    for member_id, consumers in _consumers_by_id(introspection, closures).items():
        if member_id in dataflow:
            dataflow[member_id]["consumers"] = consumers

    # A value nothing writes but something reads is a broken binding, not metadata: dropping it
    # would feed the reader a zero forever.
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


def _apply_view_liveness(dataflow: dict, views) -> None:
    """A view is written exactly when its superobject is.

    One subobject can MAP into several superobjects, so all of them count: the value is live
    whenever any of them is recomputed.
    """
    superobjects_of: dict[str, set] = {}
    for view in (views or {}).values():
        subobject_id = getattr(view.subobject, "id", None)
        superobject_id = getattr(view.superobject, "id", None)
        if subobject_id and superobject_id:
            superobjects_of.setdefault(subobject_id, set()).add(superobject_id)

    for subobject_id, superobject_ids in superobjects_of.items():
        entry = dataflow.get(subobject_id)
        sources = [dataflow[sid] for sid in sorted(superobject_ids) if sid in dataflow]
        if entry is None or not sources:
            continue
        live = [source["cadence"] for source in sources if source["storage"] == "log"]
        if live:
            # Live even when the view was authored with a literal: that is only the start value.
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
        entry["storage"] = "log" if isinstance(cadence, dict) else _STORAGE_BY_CADENCE[cadence]


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
        outputs = closure_output_ids(closure)
        for key, value in closure.items():
            if key not in {"id", "type"} and isinstance(value, str) and value not in outputs:
                add(value, "closure", closure_id, key)
    # Readers are collected from dicts whose order is the graph's; the list is an artifact.
    for readers in consumers.values():
        readers.sort(key=lambda entry: (entry["kind"], entry["id"], entry["role"]))

    return consumers


def _apply_dataflow(introspection: dict, shared_data: list, items_by_id: dict, dataflow: dict):
    """Act on the contract: absent values leave the program, init values leave the per-tick frame."""
    shared_data[:] = [
        item for item in shared_data if dataflow.get(item.id, {}).get("storage") != "absent"
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
            row = {
                "id": sample["id"],
                "source_id": sample["source_id"],
                "value": value,
                "uri": sample.get("uri"),
                # Who reads it, as the dataflow contract already resolved it.
                "consumers": entry.get("consumers"),
            }
            constants.append({key: val for key, val in row.items() if val is not None})
    if unattributed:
        raise RuntimeError(f"dataflow: samples with no resolvable contract: {sorted(unattributed)}")
    introspection["quantity_samples"] = logged
    introspection["constants"] = constants

    for pool, rows in (introspection.get("spatial_samples") or {}).items():
        kept = [row for row in rows if dataflow.get(row["id"], {}).get("storage") == "log"]
        introspection["spatial_samples"][pool] = [
            dict(row, index=index) for index, row in enumerate(kept)
        ]
