# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What values exist: the readers of every quantity, pose, frame, constraint and placement.

The views onto these values are ``views.py``'s, and who writes and reads each one is ``data_access.py``'s.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import NamedTuple

import rdflib
from motion_spec_dsl.rdf_parser.vocab import (
    ACT,
    AGN,
    ALGO_EXT,
    CSTR,
    CSTR_EXT,
    CSTR_HDL_EXT,
    ENV,
    EST,
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
from rdf_utils.models.geom_coord import (
    OrientCoordModel,
    PoseCoordModel,
    PositionCoordModel,
    find_pose_path,
    get_coord_vectorxyz,
    get_orientation_coord_vals,
    get_pose_coords,
    get_transform_between_frames,
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
    URI_GEOM_PRED_OF_POSE,
    URI_GEOM_PRED_W,
    URI_GEOM_PRED_X,
    URI_GEOM_PRED_Y,
    URI_GEOM_PRED_Z,
    URI_GEOM_TYPE_ANGLES_ABG,
    URI_GEOM_TYPE_DIRECTION_COSINE_XYZ,
    URI_GEOM_TYPE_EULER_ANGLES,
    URI_GEOM_TYPE_INTRINSIC,
    URI_GEOM_TYPE_KGRAPH,
    URI_GEOM_TYPE_ORIENT,
    URI_GEOM_TYPE_ORIENT_COORD,
    URI_GEOM_TYPE_POSE,
    URI_GEOM_TYPE_POSE_COORD,
    URI_GEOM_TYPE_POSITION,
    URI_GEOM_TYPE_POSITION_COORD,
    URI_GEOM_TYPE_QUATERNION,
    URI_QUDT_UNIT_DEG,
    URI_QUDT_UNIT_RAD,
    URI_TIME_PRED_AFTER_EVT,
    URI_TIME_PRED_OF_CONSTRAINT,
)
from rdf_utils.namespace import NS_MM_KC_EXT, NS_MM_QUDT_QTY, NS_MM_QUDT_UNIT
from rdflib import Graph, URIRef
from rdflib.namespace import PROV, RDF
from scene_dsl.rdf.sensors import URI_SENS_TYPE_CAMERA
from scene_dsl.rdf_parser.common import ensure_one_obj_uri, ensure_one_typed_subject_uri
from scene_dsl.rdf_parser.kinematics import body_of_frame, get_kinematic_mapping, pose_between
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.constraints import (
    BandConstraint,
    Constraint,
    EqualityConstraint,
    GoalStatus,
    UnilateralConstraint,
    UnilateralConstraintType,
)
from motion_spec.classes.dynamics import JointQuantity
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
    SimplicialComplex,
    Subspace,
    VelocityTwist,
    Wrench,
    WrenchEstimator,
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
from motion_spec.rdf_parser.operations import recorded_coord_policy


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
    # A second runtime vector filling this row's other subspace: a scalar of a body-fixed
    # primitive changes with both halves of the twist, and both land in one solver column.
    moment: str | None = None

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


def position_values(model, coordinate) -> list[float] | None:
    """xyz of a position coordinate in metres, or None when it carries no vector.

    Parameters:
        coordinate: a `PositionCoordModel`, or the position half of a `PoseCoordModel`

    Raises:
        ConstraintViolation: the coordinate's unit is not a length.
    """
    values = get_coord_vectorxyz(coordinate, model.graph)
    return None if values is None else to_metres(values, coordinate.unit, coordinate.id)


def orientation_quaternion(model, coordinate) -> list[float] | None:
    """Rotation of an orientation coordinate as [x, y, z, w], or None.

    `get_orientation_coord_vals` is what normalizes the representations -- Euler with its axes
    sequence and intrinsic flag, quaternion, direction cosines -- so there is nothing to dispatch
    on here. (`get_quaternion_xyzw` is the quaternion-only reader and rejects the others.)
    """
    rotation = get_orientation_coord_vals(coordinate, model.graph)
    return None if rotation is None else [float(value) for value in rotation.as_quat()]


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
        return frame(model, object_root_frame(model, node))
    if GEOM_ENT.Frame in types:
        return frame(model, node)
    return None


# Rotations whose components already resolve to one quaternion, and the wider set a literal
# delta may be written in.
_RESOLVED_ROTATION = {URI_GEOM_TYPE_QUATERNION, URI_GEOM_TYPE_DIRECTION_COSINE_XYZ}
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

    return ensure_one_typed_subject_uri(
        model.graph, node, GEOM_OP["composite"], GEOM_OP_EXT.ComposeOrientation
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


def relative_orientation(model, node) -> list[dict]:
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

    operands = []
    for operand_node in (in1, in2):
        types = get_node_types(graph, operand_node)
        if URI_GEOM_TYPE_POSE_COORD in types:
            operands.append({"pose": model.id(operand_node)})
            continue
        if not types & _LITERAL_ROTATION:
            raise ConstraintViolation(
                "geometry",
                f"Relative orientation operand '{operand_node}' is neither a pose nor an "
                "orientation",
            )
        # A delta is literal by construction, so it folds to a quaternion here.
        rotation = orientation_quaternion(model, ModelBase(node_id=operand_node, graph=graph))
        if rotation is None:
            raise ConstraintViolation(
                "geometry", f"Relative orientation delta '{operand_node}' has no literal components"
            )
        operands.append(
            {"delta": [{"value": value} for value in rotation], "representation": "quaternion"}
        )
    if sum("pose" in op for op in operands) != 1 or sum("delta" in op for op in operands) != 1:
        raise ConstraintViolation(
            "geometry",
            f"Relative orientation '{node}' must compose exactly one base pose "
            "and one delta rotation.",
        )

    return operands


@reader
def _pose_endpoint(model, node):
    """A pose endpoint as the frame it names, an object standing for its root body's frame."""
    if node is None:
        return None
    if ENV.RigidObject in get_node_types(model.graph, node):
        return frame(model, object_root_frame(model, node))
    return frame(model, node)


def view_of(graph, operand):
    """The MAP view an operand resolves through. A constraint names the view itself (relations
    are pooled per frame pair); an operand that is a subobject still finds the view sampling it.
    """
    if (operand, MAP.subobject, None) in graph:
        return operand
    return graph.value(predicate=MAP.subobject, object=operand)


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
        owner = graph.value(predicate=CSTR["reference-value"], object=node)
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
        {
            model.id(si_unit(unit))
            for component in (coordinate.position_coord.id, coordinate.orientation_coord.id)
            for unit in graph[component : QUDT_SCHEMA["unit"]]
        }
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
            relative_orientation(model, orientation_node) if representation == "relative" else None
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
        (view, MAP["axis"], None) in model.graph
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
    return AccelerationTwist(**_spatial_fields(model, node))


@reader
def pose_difference(model, node) -> PoseDifference:
    """A PoseDifference quantity."""
    return PoseDifference(**_spatial_fields(model, node))


@reader
def wrench(model, node) -> Wrench:
    """A Wrench quantity, with the force/torque sensor measuring it when there is one."""
    graph = model.graph
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
    observation = ensure_one_typed_subject_uri(graph, node, SOSA.observedProperty, SOSA.Observation)
    sensor = graph.value(observation, SOSA.madeBySensor) if observation is not None else None
    sensor_frame_node = graph.value(sensor, SENSORS.frame) if sensor is not None else None
    if sensor is not None and sensor_frame_node is None:
        raise ConstraintViolation(
            "dynamics", f"WrenchCoordinate '{node}' sensor '{sensor}' has no physical frame"
        )
    observer = graph.value(node, EST["estimated-by"])
    estimator = None
    if observer is not None:
        agent_node = graph.value(observer, AGN["of-agent"])
        if agent_node is None:
            raise ConstraintViolation(
                "dynamics", f"momentum observer '{observer}' names no agent to run on"
            )
        gain = graph.value(observer, EST["estimation-gain"])
        filter_constant = graph.value(observer, EST["filter-constant"])
        estimator = WrenchEstimator(
            agent=model.id(agent_node),
            estimation_gain_hz=float(graph.value(gain, QUDT_SCHEMA["value"])),
            filter_constant=float(graph.value(filter_constant, QUDT_SCHEMA["value"])),
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
        estimator=estimator,
        retare_event_uris=tuple(
            str(event)
            for schedule in graph.subjects(URI_TIME_PRED_OF_CONSTRAINT, node)
            for event in graph.objects(schedule, URI_TIME_PRED_AFTER_EVT)
        ),
    )


def is_duration(model, node) -> bool:
    """Authored durations carry the OWL-Time type; runtime elapsed time is a Time-kind quantity
    the clock fills, so it has a kind but no value.
    """
    if TIME["Duration"] in get_node_types(model.graph, node):
        return True
    return (node, QUDT_SCHEMA.hasQuantityKind, NS_MM_QUDT_QTY["Time"]) in model.graph


def _observed_at_id(model, pose_node) -> str | None:
    """The slot holding when this pose was last observed, when an age constraint asked for it."""
    instant = model.graph.value(pose_node, SOSA.phenomenonTime)
    return model.id(instant) if instant is not None else None


def perceived_written_poses(model) -> dict[str, list[dict]]:
    """Per perception source, the world poses it writes and the frame each must arrive in.

    A source is any node stating the objects it observes -- a detect act that asks once, or a
    topic the model stands subscribed to. A subscription names the world pose it writes, so its
    reference frame is the fixed frame the pose arrives in. A detect target remains an object and
    resolves to its one world pose.

    A node that observes nothing -- a published topic -- contributes no rows, so every consumer
    passes over it without filtering. So does one observing only a camera: that channel carries
    images for a viewer to read, and states no pose for the run to act on.

    Raises:
        ConstraintViolation: a source observes an object no world pose is stated of, so a
            detection has nowhere to land.
    """
    graph = model.graph
    world_poses = [
        pose(model, node)
        for node in graph.subjects(RDF["type"], GEOM_COORD["PoseCoordinate"])
        if (node, RDF["type"], SOSA.ObservableProperty) in graph
    ]
    sources = set(graph.subjects(RDF["type"], NS_MM_ROS["Action"])) | set(
        graph.subjects(RDF["type"], NS_MM_ROS["Topic"])
    )
    written: dict[str, list[dict]] = {}
    for act in sources:
        rows = []
        for target in graph.objects(act, SOSA.hasFeatureOfInterest):
            # A camera is what a channel carries, not a pose it writes: a viewer reads those
            # images, and nothing in the loop does.
            if URI_SENS_TYPE_CAMERA in get_node_types(graph, target):
                continue
            if NS_MM_ROS["Topic"] in get_node_types(graph, act):
                item = pose(model, target)
                target_iri = item.of.uri
                # The reading's own of/wrt when the channel states them: the composition into
                # this quantity is the model's, so both ends travel with the row.
                observed = graph.value(act, SOSA.observedProperty)
                of_frame = graph.value(observed, GEOM_REL.of)
                rows.append(
                    {
                        "target_iri": target_iri,
                        "pose_id": item.id,
                        "frame_id": item.with_respect_to.id,
                        "frame_iri": item.with_respect_to.uri,
                        "observed_of_iri": str(of_frame or ""),
                        # The body the reading places: its root segment is what the world model
                        # binds, so every other frame on it follows from the one measurement.
                        "observed_body_iri": str(
                            body_of_frame(of_frame, graph) if of_frame is not None else ""
                        ),
                        "observed_wrt_iri": str(
                            graph.value(observed, GEOM_REL["with-respect-to"]) or ""
                        ),
                        "target_of_iri": str(target_iri),
                        "observed_at_id": _observed_at_id(model, target),
                    }
                )
                continue
            located = _body_or_self(model, target)
            matched = [
                item for item in world_poses if item.of is not None and item.of.id == located
            ]
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
                    "frame_iri": item.with_respect_to.uri,
                    "observed_at_id": _observed_at_id(model, model.node_by_id[item.id]),
                }
                for item in matched
            )
        written[str(act)] = rows

    return written


def goal_status_act(model, node):
    """The action node whose goal status this node holds, or None if it holds no status."""
    if node is None:
        return None
    acts = [
        act
        for act in model.graph.objects(node, PROV.wasDerivedFrom)
        if NS_MM_ROS["Action"] in get_node_types(model.graph, act)
    ]
    if len(acts) > 1:
        raise ConstraintViolation(
            "communication",
            f"'{node}' holds the goal status of {len(acts)} actions -- it holds one",
        )
    return acts[0] if acts else None


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
        if (node, MAP.axis, None) in graph:
            # A single-axis view names its own scalar; only whole-component views need the
            # superobject to pin one sampling of the pooled relation.
            superobject = None
        sup_types = get_node_types(graph, superobject) if superobject is not None else set()
        if (
            subspace == MAP_EXT.position
            and URI_GEOM_TYPE_POSITION_COORD in sup_types
            or subspace == MAP_EXT.orientation
            and URI_GEOM_TYPE_ORIENT_COORD in sup_types
        ):
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
    if is_duration(model, node):
        return duration_quantity(model, node)
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
    if GEOM_COORD["VelocityTwistCoordinate"] in types:
        return velocity_twist(model, node)

    kind_node = graph.value(node, QUDT_SCHEMA.hasQuantityKind)
    kind = QuantityKind(model.id(kind_node), str(kind_node))
    # Values below are converted, so the unit reported alongside them is the SI one.
    unit_node = graph.value(node, QUDT_SCHEMA["unit"])
    si_node = si_unit(unit_node)
    unit = Unit(model.id(si_node), str(si_node))
    has_view = (node, ~MAP["subobject"], None) in graph
    provenance = quantity_provenance(model, node)

    if CSTR_HDL_EXT["SetpointGenerator"] in types:
        return SetpointQuantity(
            model.id(node), kind, unit, has_view, provenance=provenance, value_kind=kind.id
        )
    if kind.id == "FreeVector" and GEOM_COORD["VectorXYZ"] in types:
        return FreeVector(
            model.id(node), kind, unit, parse_xyz(model, node), has_view, provenance=provenance
        )

    authored = graph.value(node, QUDT_SCHEMA["value"])
    value = si(float(authored), unit_node) if authored is not None else None
    reference = graph.value(node, CSTR["reference-value"])

    return Quantity(
        model.id(node),
        kind,
        unit,
        value,
        has_view,
        provenance=provenance,
        reference_value=model.id(reference) if reference is not None else None,
        sampled=URI_DISTRIB_TYPE_SAMPLED_QUANTITY in types,
    )


@reader
def duration_quantity(model, node) -> Quantity:
    """An authored duration or a runtime elapsed-duration coordinate, in seconds."""
    if not is_duration(model, node):
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
        QuantityKind("Duration", str(NS_MM_QUDT_QTY["Time"])),
        Unit("Second", str(NS_MM_QUDT_UNIT["SEC"])),
        value,
        False,
        provenance=quantity_provenance(model, node),
        sampled=URI_DISTRIB_TYPE_SAMPLED_QUANTITY in get_node_types(model.graph, node),
    )


# The joint-space coordinates, by the IR type each reads as.
JOINT_QUANTITY_TYPES = {
    KC_STAT.JointPositionCoordinate: "JointPosition",
    KC_STAT.JointVelocityCoordinate: "JointVelocity",
    ACT.JointCurrent: "JointCurrent",
}


@reader
def joint_quantity(model, node) -> JointQuantity:
    """A joint position, velocity or motor current, named by the joint it reads."""
    joint = model.graph.value(node, KC_STAT["of-joint"])
    if not isinstance(joint, URIRef):
        raise ConstraintViolation("kinematic-chain", f"joint quantity '{node}' has no of-joint URI")
    types = get_node_types(model.graph, node)
    return JointQuantity(
        model.id(node),
        model.label(joint),
        next(kind for rdf_type, kind in JOINT_QUANTITY_TYPES.items() if rdf_type in types),
        joint_uri=str(joint),
        normalization=_normalization(model, node),
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
    return (node, PROV.wasDerivedFrom, None) in graph and (
        None,
        URI_TIME_PRED_OF_CONSTRAINT,
        node,
    ) in graph


def _is_config_pose(model, node) -> bool:
    """Whether a pose's value is read from the deployment config file rather than authored in
    the model. Its orientation is stamped Euler-typed by the DSL for lack of an authored
    coordinate to inspect -- that stamp must not be read as a literal, decomposed pose.
    """
    return (node, EXEC["has-resource"], None) in model.graph


@reader
def snapshot_target_ids(model) -> set:
    """Every id whose node is a snapshot target, for readers that hold a record id rather than a
    graph node and so cannot call `_is_snapshot` directly.
    """
    graph = model.graph
    return {
        model.id(node)
        for node in graph.objects(None, URI_TIME_PRED_OF_CONSTRAINT)
        if (node, PROV.wasDerivedFrom, None) in graph
    }


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
    return Frame(_body_or_self(model, node), uri=str(node))


def body_of(model, node):
    """The rigid body a frame is a simplex of, or None when no body carries it."""
    return ensure_one_typed_subject_uri(model.graph, node, GEOM_ENT.simplices, GEOM_ENT.RigidBody)


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


def object_root_frame(model, node):
    """The frame the root body of a modelled scene object stands at.

    An object is placed in the scene through the asset that models it, and the first body that
    asset maps is the one it is spawned as -- so that body's own frame is where the object is.

    Raises:
        ConstraintViolation: no asset maps the object onto a scene body.
    """
    graph = model.graph
    for modelled in graph.subjects(ENV["of-object"], node):
        for asset in graph.objects(modelled, ENV["has-object-model"]):
            for mapping in graph.objects(asset, EXEC["has-mapping"]):
                mapped = get_kinematic_mapping(mapping, graph)
                if mapped.target_type == GEOM_ENT.RigidBody:
                    return placement_frame(model, mapped.target_id)
    raise ConstraintViolation(
        "geometry",
        f"'{model.id(node)}' is referenced as a spatial endpoint, but no asset maps it onto a "
        "body of the scene, so nothing says where it is.",
    )


def anchor_frame(model):
    """The frame the scene stands on: its ground, and the origin every placement resolves into.

    The graph declares it, so nothing here guesses which of its roots the scene hangs from.
    """
    graph = model.graph
    anchors = {
        anchor
        for kgraph in graph.subjects(RDF.type, URI_GEOM_TYPE_KGRAPH)
        for anchor in graph.objects(kgraph, NS_MM_KC_EXT["anchor"])
    }
    if len(anchors) != 1:
        raise ConstraintViolation(
            "kinematics",
            f"the scene needs exactly one anchor to stand on, found {len(anchors)}"
            f"{': ' + ', '.join(map(str, anchors)) if anchors else ''}",
        )
    return anchors.pop()


def placement_of(model, attachment, anchor):
    """Where an attached body sits, in the frame of whatever it is bolted to.

    A body the scene places itself resolves against the anchor the runtime is built on. One
    bolted to another model resolves against that model's own root frame, which is where its
    site is: composing to the anchor instead would count its host's placement twice.
    """
    kind, _name, frame, parent = attachment
    if kind != "World":
        return frame_placement(model, frame, placement_frame(model, parent))
    # What the runtime places is the body, so its root frame -- not whichever of its frames a
    # joint happens to hang it by, which may sit anywhere on it.
    is_frame = GEOM_ENT.Frame in get_node_types(model.graph, frame)
    return frame_placement(model, body_of_frame(frame, model.graph) if is_frame else frame, anchor)


@reader
def _placement_graph(model):
    """The poses that place something, which is not every pose the graph relates.

    A context quantity relates two frames the scene has already placed: a world pose the run
    computes each cycle, a spec pose it aims at. Both are the shortest way between their frames,
    so a search left to walk them answers where a body sits with a target the arm is moving to,
    or with a coordinate that holds no value until the first cycle.
    """
    graph = Graph()
    for triple in model.graph.triples((None, None, None)):
        graph.add(triple)
    # A relation is pooled per frame pair, so a context quantity is a coordinate on a
    # relation the scene also samples: strip the quantity's own nodes, keep the relation --
    # unless nothing places it any more, in which case the relation goes too.
    for type_ in (URI_GEOM_TYPE_POSE, URI_GEOM_TYPE_POSE_COORD):
        for node in model.graph.subjects(RDF["type"], type_):
            if next(model.graph.subjects(PROV.hadMember, node), None) is not None:
                graph.remove((node, None, None))
    # A pose an operation computes each cycle holds no coordinates until the run; it places nothing.
    for predicate in (GEOM_OP.composite, GEOM_OP.out):
        for node in model.graph.objects(None, predicate):
            graph.remove((node, None, None))
    for relation in list(graph.subjects(RDF["type"], URI_GEOM_TYPE_POSE)):
        if next(graph.subjects(URI_GEOM_PRED_OF_POSE, relation), None) is None:
            graph.remove((relation, None, None))

    return graph


def _reject_sampled_placement(model, frame, wrt) -> None:
    """A placement is built into the world before the run draws anything.

    Raises:
        ConstraintViolation: a pose placing this frame is drawn at run time.
    """
    graph = _placement_graph(model)
    for pose, coords in get_pose_coords(graph=graph, poses=find_pose_path(frame, wrt, graph) or []):
        for coord in coords:
            for node in (coord.id, coord.position_coord.id, coord.orientation_coord.id):
                if URI_DISTRIB_TYPE_SAMPLED_QUANTITY in get_node_types(model.graph, node):
                    raise ConstraintViolation(
                        "geometry",
                        f"'{pose.id}' places '{frame}' by the drawn coordinate '{node}': a "
                        f"placement cannot be drawn, sample a frame on the body instead",
                    )


def frame_placement(model, node, wrt):
    """Where a body or frame sits in `wrt`, in metres and [x, y, z, w].

    Composed along the poses that place it, so a scene may author a placement against any
    frame it likes and still be read against the one it is assembled on. A body no pose
    leads to is placed by the joint that holds it, and comes back coincident.
    """
    frame = placement_frame(model, node)
    if frame is None:
        return None, None
    _reject_sampled_placement(model, frame, wrt)
    graph = _placement_graph(model)
    transform = get_transform_between_frames(frame, wrt, graph)
    if transform is None:
        # A pose reads one way, but it relates both frames: a scene that places a body's root
        # against one of its own frames still says where that frame is on the body.
        reverse = get_transform_between_frames(wrt, frame, graph)
        transform = reverse.inv() if reverse is not None else None
    if transform is None:
        transform = pose_between(frame, wrt, graph)
    if transform is None:
        return None, None
    return list(transform.translation), list(transform.rotation.as_quat())


@reader
def constraint(model, node) -> Constraint:
    """A Constraint: the quantity it bounds, together with its parameter.

    A goal status is not measured against an operand of its own kind: the status it must reach
    rides on the evaluator, so the constraint states the slot and nothing else.
    """
    quantity_node = model.graph.value(node, CSTR["quantity"])
    if goal_status_act(model, quantity_node) is not None:
        return Constraint(model.id(node), quantity(model, quantity_node), None)
    types = get_node_types(model.graph, node)
    if CSTR["EqualityConstraint"] in types:
        parameter = _equality_constraint(model, node)
    elif CSTR["UnilateralConstraint"] in types:
        parameter = _unilateral_constraint(model, node)
    else:
        parameter = _band_constraint(model, node)

    return Constraint(model.id(node), quantity(model, quantity_node), parameter)


def _threshold(model, node, predicate) -> Quantity:
    return quantity(model, model.graph.value(node, predicate))


@reader
def _equality_constraint(model, node) -> EqualityConstraint:
    return EqualityConstraint(_threshold(model, node, CSTR["reference-value"]))


@reader
def _unilateral_constraint(model, node) -> UnilateralConstraint:
    type_ = UnilateralConstraintType.LessThan
    if CSTR["GreaterThanConstraint"] in get_node_types(model.graph, node):
        type_ = UnilateralConstraintType.GreaterThan
    return UnilateralConstraint(type_, _threshold(model, node, CSTR["threshold"]))


@reader
def _band_constraint(model, node) -> BandConstraint:
    outside = CSTR_EXT["OutsideConstraint"] in get_node_types(model.graph, node)
    return BandConstraint(
        _threshold(model, node, CSTR["lower-threshold"]),
        _threshold(model, node, CSTR["upper-threshold"]),
        "OutsideConstraint" if outside else "BilateralConstraint",
    )
