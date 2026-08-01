# SPDX-License-Identifier: MPL-2.0
from __future__ import annotations

import pytest
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import RDF
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import (
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_GEOM_PRED_ALPHA,
    URI_GEOM_PRED_AXES_SEQ,
    URI_GEOM_PRED_BETA,
    URI_GEOM_PRED_GAMMA,
    URI_GEOM_PRED_W,
    URI_GEOM_TYPE_ANGLES_ABG,
    URI_GEOM_TYPE_EULER_ANGLES,
    URI_GEOM_TYPE_EXTRINSIC,
    URI_GEOM_TYPE_ORIENT_REF,
    URI_GEOM_TYPE_POSITION_REF,
    URI_GEOM_TYPE_VECTOR_XYZ,
    URI_QUDT_UNIT_CM,
    URI_QUDT_UNIT_RAD,
)
from scipy.spatial.transform import Rotation

from motion_spec.rdf_parser.ir import (
    Parser,
    _materialize_linear_distance_operations,
    _materialize_pose_reference_transforms,
    _orientation_of,
    _position_of,
)
from motion_spec.rdf_parser.vocab import (
    CSTR,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_OP,
    GEOM_OP_EXT,
    GEOM_REL,
    MAP,
    QUDT_SCHEMA,
)

BASE = "https://example.test/"


def _u(name: str) -> URIRef:
    return URIRef(BASE + name)


def _frame(g: Graph, name: str) -> URIRef:
    node = _u(name)
    origin = _u(f"{name}-origin")
    g.add((node, RDF.type, GEOM_ENT.Frame))
    g.add((node, GEOM_ENT.origin, origin))
    g.add((origin, RDF.type, GEOM_ENT.Point))
    return node


def _pose(g: Graph, name: str, of_frame: URIRef, wrt_frame: URIRef) -> URIRef:
    node = _u(name)
    g.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    g.add((node, RDF.type, GEOM_REL.Pose))
    g.add((node, RDF.type, GEOM_COORD.PoseCoordinate))
    g.add((node, GEOM_REL.of, of_frame))
    g.add((node, GEOM_REL["with-respect-to"], wrt_frame))
    g.add((node, GEOM_COORD["as-seen-by"], wrt_frame))
    return node


def _derived(node: URIRef, suffix: str) -> URIRef:
    return URIRef(f"{node}.derived-{suffix}")


# --------------------------------------------------------------------------- #
# Linear-distance materialization
# --------------------------------------------------------------------------- #
def _distance_graph(*, end_wrt_name: str) -> tuple[Graph, URIRef]:
    """Distance between `shoulder wrt base` and `ee wrt <end_wrt_name>`, with a
    base<-table connecting pose available for the cross-frame path.
    """
    g = Graph()
    base = _frame(g, "frame-base")
    table = _frame(g, "frame-table")
    shoulder = _frame(g, "frame-shoulder")
    ee = _frame(g, "frame-ee")
    _pose(g, "pose-table-base", table, base)
    start = _pose(g, "pose-shoulder-base", shoulder, base)
    end = _pose(g, "pose-ee-x", ee, base if end_wrt_name == "frame-base" else table)

    distance = _u("dist")
    g.add((distance, RDF.type, GEOM_REL.LinearDistance))
    g.add((distance, GEOM_REL["between-entities"], start))
    g.add((distance, GEOM_REL["between-entities"], end))
    constraint = _u("c-dist")
    g.add((constraint, RDF.type, CSTR.Constraint))
    g.add((constraint, CSTR.quantity, distance))
    return g, distance


def test_distance_materializes_magnitude_op() -> None:
    g, distance = _distance_graph(end_wrt_name="frame-base")
    _materialize_linear_distance_operations(g)

    magnitudes = [
        op
        for op in g.subjects(RDF.type, GEOM_OP.PoseToLinearDistance)
        if g.value(op, GEOM_OP.distance) == distance
    ]
    assert len(magnitudes) == 1
    assert g.value(magnitudes[0], GEOM_OP.pose) == _derived(distance, "relative-pose")
    assert (_derived(distance, "invert-start"), RDF.type, GEOM_OP.InvertPose) in g


def test_distance_cross_frame_composes_reference_path() -> None:
    g, distance = _distance_graph(end_wrt_name="frame-table")
    _materialize_linear_distance_operations(g)

    end_in_start = _derived(distance, "end-in-start-reference")
    assert (None, GEOM_OP.composite, end_in_start) in g
    assert URIRef(f"{end_in_start}-pose-rel") in set(g.subjects(RDF.type, GEOM_REL.Pose))


def test_distance_same_frame_skips_reference_path() -> None:
    g, distance = _distance_graph(end_wrt_name="frame-base")
    _materialize_linear_distance_operations(g)

    end_in_start = _derived(distance, "end-in-start-reference")
    assert (None, GEOM_OP.composite, end_in_start) not in g
    assert (None, RDF.type, GEOM_OP.PoseToLinearDistance) in g


# --------------------------------------------------------------------------- #
# Pose-reference transform (full-pose equality across frames)
# --------------------------------------------------------------------------- #
def _equality_graph(*, reference_wrt_name: str) -> tuple[Graph, URIRef, URIRef]:
    """`pose ee-wrt-base` equal to a reference `ee wrt <reference_wrt_name>`."""
    g = Graph()
    base = _frame(g, "frame-base")
    table = _frame(g, "frame-table")
    ee = _frame(g, "frame-ee")
    _pose(g, "pose-table-base", table, base)
    target = _pose(g, "pose-ee-base", ee, base)
    reference = _pose(
        g, "ref-ee", ee, base if reference_wrt_name == "frame-base" else table
    )

    constraint = _u("c-eq")
    g.add((constraint, RDF.type, CSTR.Constraint))
    g.add((constraint, RDF.type, CSTR.EqualityConstraint))
    g.add((constraint, CSTR.quantity, target))
    g.add((constraint, CSTR["reference-value"], reference))
    return g, constraint, reference


def test_pose_reference_cross_frame_reexpresses_reference() -> None:
    g, constraint, reference = _equality_graph(reference_wrt_name="frame-table")
    _materialize_pose_reference_transforms(g)

    reexpressed = _derived(constraint, "reference-in-target")
    assert g.value(constraint, CSTR["reference-value"]) == reexpressed
    compose = _derived(constraint, "compose-reference")
    assert (compose, RDF.type, GEOM_OP.ComposePose) in g
    assert g.value(compose, GEOM_OP.in2) == reference
    assert g.value(compose, GEOM_OP.composite) == reexpressed
    relation = URIRef(f"{reexpressed}-pose-rel")
    assert g.value(relation, GEOM_REL["with-respect-to"]) == _u("frame-base")


def test_pose_reference_same_frame_is_noop() -> None:
    g, constraint, reference = _equality_graph(reference_wrt_name="frame-base")
    _materialize_pose_reference_transforms(g)

    assert g.value(constraint, CSTR["reference-value"]) == reference
    assert (None, RDF.type, GEOM_OP.ComposePose) not in g


def test_pose_reference_rejects_body_mismatch() -> None:
    g, constraint, _ = _equality_graph(reference_wrt_name="frame-table")
    reference = g.value(constraint, CSTR["reference-value"])
    g.remove((reference, GEOM_REL.of, None))
    g.add((reference, GEOM_REL.of, _frame(g, "frame-other")))

    with pytest.raises(ValueError, match="compares a pose"):
        _materialize_pose_reference_transforms(g)


# --------------------------------------------------------------------------- #
# Relative orientation: geom-op:in1/in2 operands, read back in slot order
# --------------------------------------------------------------------------- #
def _delta_node(g: Graph, name: str, values: tuple[float, float, float]) -> URIRef:
    """A standalone Euler delta quantity in canonical rdf-utils form."""
    node = _u(name)
    for type_ in (QUDT_SCHEMA.Quantity, URI_GEOM_TYPE_EULER_ANGLES, URI_GEOM_TYPE_ANGLES_ABG):
        g.add((node, RDF.type, type_))
    g.add((node, RDF.type, URI_GEOM_TYPE_EXTRINSIC))
    g.add((node, URI_GEOM_PRED_AXES_SEQ, Literal("xyz")))
    g.add((node, QUDT_SCHEMA.unit, URI_QUDT_UNIT_RAD))
    for predicate, value in zip(
        (URI_GEOM_PRED_ALPHA, URI_GEOM_PRED_BETA, URI_GEOM_PRED_GAMMA), values
    ):
        g.add((node, predicate, Literal(float(value))))
    return node


def _relative_orientation_graph(*, in1_is_pose: bool) -> tuple[Graph, URIRef]:
    """A RelativeOrientation composing `pose-ee-base` with a delta, slotted into
    `geom-op:in1`/`in2` base-first (`in1_is_pose`) or delta-first.
    """
    g = Graph()
    base = _frame(g, "frame-base")
    ee = _frame(g, "frame-ee")
    base_pose = _pose(g, "pose-ee-base", ee, base)
    delta = _delta_node(g, "delta", (-0.75, 0.0, 0.0))

    orientation = _u("relative-orientation")
    g.add((orientation, RDF.type, GEOM_OP_EXT.RelativeOrientation))
    g.add((orientation, GEOM_REL.of, ee))
    g.add((orientation, GEOM_COORD["as-seen-by"], base))
    g.add((delta, GEOM_COORD["as-seen-by"], ee if in1_is_pose else base))

    in1, in2 = (base_pose, delta) if in1_is_pose else (delta, base_pose)
    g.add((orientation, GEOM_OP["in1"], in1))
    g.add((orientation, GEOM_OP["in2"], in2))
    return g, orientation


def test_relative_orientation_reads_operands_in_slot_order() -> None:
    g, orientation = _relative_orientation_graph(in1_is_pose=True)
    operands = Parser(g)._relative_orientation(orientation)
    assert "pose" in operands[0] and "delta" in operands[1]
    assert operands[1]["delta"] == [{"value": -0.75}, {"value": 0.0}, {"value": 0.0}]
    assert operands[1]["representation"] == "euler"

    g, orientation = _relative_orientation_graph(in1_is_pose=False)
    operands = Parser(g)._relative_orientation(orientation)
    assert "delta" in operands[0] and "pose" in operands[1]


def test_relative_orientation_rejects_a_mismatched_operand_pair() -> None:
    g, orientation = _relative_orientation_graph(in1_is_pose=True)
    # Two poses in the slots: not one pose + one delta.
    other_pose = _pose(
        g, "pose-other", _frame(g, "frame-other-body"), _frame(g, "frame-other-wrt")
    )
    g.remove((orientation, GEOM_OP["in2"], None))
    g.add((orientation, GEOM_OP["in2"], other_pose))

    with pytest.raises(ValueError, match="exactly one base pose and one delta"):
        Parser(g)._relative_orientation(orientation)


# --------------------------------------------------------------------------- #
# Scene placement: representation-aware orientation, unit-aware position
# --------------------------------------------------------------------------- #
def _scene_frame(g: Graph, name: str) -> URIRef:
    """A frame with an origin point (rdf-utils's FrameModel requires one)."""
    frame = _u(name)
    g.add((frame, RDF.type, GEOM_ENT.Frame))
    g.add((frame, GEOM_ENT.origin, _u(f"{name}-origin")))
    return frame


def _scene_position_coord(
    g: Graph, of_frame: URIRef, wrt_frame: URIRef, xyz: tuple[float, float, float]
) -> URIRef:
    """A Position coordinate at `of_frame`'s origin, wrt `wrt_frame`'s origin -- `_position_of`
    looks a frame up by its origin point, matching how scene-dsl authors a placement."""
    position = _u("position")
    coord = _u("position-coord")
    g.add((position, RDF.type, GEOM_REL.Position))
    g.add((position, GEOM_REL.of, g.value(of_frame, GEOM_ENT.origin)))
    g.add((position, GEOM_REL["with-respect-to"], g.value(wrt_frame, GEOM_ENT.origin)))
    g.add((coord, RDF.type, GEOM_COORD.PositionCoordinate))
    g.add((coord, RDF.type, URI_GEOM_TYPE_POSITION_REF))
    g.add((coord, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
    g.add((coord, GEOM_COORD["of-position"], position))
    g.add((coord, GEOM_COORD["as-seen-by"], wrt_frame))
    for predicate, value in zip((GEOM_COORD.x, GEOM_COORD.y, GEOM_COORD.z), xyz):
        g.add((coord, predicate, Literal(float(value))))
    return coord


def test_orientation_of_reads_quaternion_placement_as_quat() -> None:
    """A quaternion-authored placement is read back as [x, y, z, w], unmodified."""
    g = Graph()
    frame = _scene_frame(g, "frame-object")
    wrt = _scene_frame(g, "frame-world")

    orientation = _u("orientation")
    coord = _u("orientation-coord")
    g.add((orientation, RDF.type, GEOM_REL.Orientation))
    g.add((orientation, GEOM_REL.of, frame))
    g.add((orientation, GEOM_REL["with-respect-to"], wrt))
    g.add((coord, RDF.type, GEOM_COORD.OrientationCoordinate))
    g.add((coord, RDF.type, URI_GEOM_TYPE_ORIENT_REF))
    g.add((coord, RDF.type, GEOM_COORD.Quaternion))
    g.add((coord, GEOM_COORD["of-orientation"], orientation))
    g.add((coord, GEOM_COORD["as-seen-by"], wrt))

    x, y, z, w = Rotation.from_euler("xyz", (0.3, -0.2, 0.75)).as_quat()
    for predicate, value in zip((GEOM_COORD.x, GEOM_COORD.y, GEOM_COORD.z), (x, y, z)):
        g.add((coord, predicate, Literal(float(value))))
    g.add((coord, URI_GEOM_PRED_W, Literal(float(w))))

    result = _orientation_of(g, frame)
    assert result == pytest.approx((x, y, z, w))


def test_position_of_scales_to_metres_and_rejects_a_missing_unit() -> None:
    """Reading x/y/z alone ignores the coordinate's unit; a cm-authored placement must
    scale, not pass through as if it were already metres. A missing unit is not a
    default -- it's an error.
    """
    g = Graph()
    frame = _scene_frame(g, "frame-object")
    wrt = _scene_frame(g, "frame-world")
    coord = _scene_position_coord(g, frame, wrt, (150.0, -50.0, 720.0))
    g.add((coord, QUDT_SCHEMA.unit, URI_QUDT_UNIT_CM))

    assert _position_of(g, frame) == pytest.approx([1.5, -0.5, 7.2])

    g2 = Graph()
    frame2 = _scene_frame(g2, "frame-object")
    wrt2 = _scene_frame(g2, "frame-world")
    _scene_position_coord(g2, frame2, wrt2, (1.0, 2.0, 3.0))  # no unit triple added

    with pytest.raises(ConstraintViolation, match="length unit"):
        _position_of(g2, frame2)


def test_sampled_scene_placements_are_rejected() -> None:
    """Sampling has no motion-spec seed semantics yet, so it must never become identity."""
    g = Graph()
    frame = _scene_frame(g, "frame-object")
    wrt = _scene_frame(g, "frame-world")
    coord = _scene_position_coord(g, frame, wrt, (1.0, 2.0, 3.0))
    g.add((coord, RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY))
    g.add((coord, QUDT_SCHEMA.unit, URI_QUDT_UNIT_CM))

    with pytest.raises(ConstraintViolation, match="Sampled placement coordinate"):
        _position_of(g, frame)
