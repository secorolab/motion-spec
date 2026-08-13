# SPDX-License-Identifier: MPL-2.0
from __future__ import annotations

from pathlib import Path

import pytest
from motion_spec_dsl.rdf_parser.vocab import (
    CSTR,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_OP,
    GEOM_OP_EXT,
    GEOM_REL,
    QUDT_SCHEMA,
)
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
    URI_GEOM_PRED_OF_ORIENT,
    URI_GEOM_PRED_OF_POSE,
    URI_GEOM_PRED_OF_POSITION,
    URI_GEOM_PRED_SEEN_BY,
    URI_GEOM_TYPE_ORIENT_REF,
    URI_GEOM_TYPE_POSE,
    URI_GEOM_TYPE_POSE_COORD,
    URI_GEOM_TYPE_POSE_REF,
    URI_GEOM_TYPE_POSITION_REF,
    URI_GEOM_TYPE_VECTOR_XYZ,
    URI_QUDT_UNIT_CM,
    URI_QUDT_UNIT_M,
    URI_QUDT_UNIT_RAD,
)
from rdflib import Dataset, Graph, Literal, URIRef
from rdflib.namespace import RDF
from scipy.spatial.transform import Rotation

from motion_spec.rdf_parser.model import Model, local_name
from motion_spec.rdf_parser.operations import (
    _materialize_linear_distance_operations,
    _materialize_pose_reference_transforms,
)
from motion_spec.rdf_parser.quantities import _relative_orientation, orientation_representation
from motion_spec.rdf_parser.resources import _placement

BASE = "https://example.test/"


def _u(name: str) -> URIRef:
    return URIRef(BASE + name)


def _model(g: Graph) -> Model:
    return Model(
        graph=g, app_path=Path("model-app.ld.json"), imported_models=[], imported_provenance=[]
    )


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
def _distance_coordinate(g: Graph, name: str, relation: URIRef) -> URIRef:
    """One constrained sampling of `relation`: the node the operations hang off."""
    coordinate = _u(name)
    g.add((coordinate, RDF.type, GEOM_COORD.DistanceReference))
    g.add((coordinate, GEOM_COORD.of, relation))
    constraint = _u(f"c-{name}")
    g.add((constraint, RDF.type, CSTR.Constraint))
    g.add((constraint, CSTR.quantity, coordinate))
    return coordinate


def _distance_graph(*, end_wrt_name: str) -> tuple[Graph, URIRef]:
    """Distance between `shoulder wrt base` and `ee wrt <end_wrt_name>`, with a
    base<-table connecting pose available for the cross-frame path.
    """
    g = Dataset(default_union=True)
    base = _frame(g, "frame-base")
    table = _frame(g, "frame-table")
    shoulder = _frame(g, "frame-shoulder")
    ee = _frame(g, "frame-ee")
    _pose(g, "pose-table-base", table, base)
    start = _pose(g, "pose-shoulder-base", shoulder, base)
    end = _pose(g, "pose-ee-x", ee, base if end_wrt_name == "frame-base" else table)

    relation = _u("dist-rel")
    g.add((relation, RDF.type, GEOM_REL.LinearDistance))
    g.add((relation, GEOM_REL["between-entities"], start))
    g.add((relation, GEOM_REL["between-entities"], end))
    return g, _distance_coordinate(g, "dist", relation)


def test_distance_materializes_magnitude_op() -> None:
    g, distance = _distance_graph(end_wrt_name="frame-base")
    _materialize_linear_distance_operations(_model(g))

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
    _materialize_linear_distance_operations(_model(g))

    end_in_start = _derived(distance, "end-in-start-reference")
    assert (None, GEOM_OP.composite, end_in_start) in g
    assert URIRef(f"{end_in_start}-pose-rel") in set(g.subjects(RDF.type, GEOM_REL.Pose))


def test_distance_same_frame_skips_reference_path() -> None:
    g, distance = _distance_graph(end_wrt_name="frame-base")
    _materialize_linear_distance_operations(_model(g))

    end_in_start = _derived(distance, "end-in-start-reference")
    assert (None, GEOM_OP.composite, end_in_start) not in g
    assert (None, RDF.type, GEOM_OP.PoseToLinearDistance) in g


def test_each_coordinate_of_one_relation_computes_its_own_value() -> None:
    """Two motions measuring the same two poses share the relation; sharing the derivation
    would leave the second reading the first's value.
    """
    g, first = _distance_graph(end_wrt_name="frame-base")
    second = _distance_coordinate(g, "dist-2", g.value(first, GEOM_COORD.of))
    _materialize_linear_distance_operations(_model(g))

    outputs = {
        g.value(op, GEOM_OP.distance) for op in g.subjects(RDF.type, GEOM_OP.PoseToLinearDistance)
    }
    assert outputs == {first, second}
    assert _derived(first, "relative-pose") != _derived(second, "relative-pose")


# --------------------------------------------------------------------------- #
# Pose-reference transform (full-pose equality across frames)
# --------------------------------------------------------------------------- #
def _equality_graph(*, reference_wrt_name: str) -> tuple[Graph, URIRef, URIRef]:
    """`pose ee-wrt-base` equal to a reference `ee wrt <reference_wrt_name>`."""
    g = Dataset(default_union=True)
    base = _frame(g, "frame-base")
    table = _frame(g, "frame-table")
    ee = _frame(g, "frame-ee")
    _pose(g, "pose-table-base", table, base)
    target = _pose(g, "pose-ee-base", ee, base)
    reference = _pose(g, "ref-ee", ee, base if reference_wrt_name == "frame-base" else table)

    constraint = _u("c-eq")
    g.add((constraint, RDF.type, CSTR.Constraint))
    g.add((constraint, RDF.type, CSTR.EqualityConstraint))
    g.add((constraint, CSTR.quantity, target))
    g.add((constraint, CSTR["reference-value"], reference))
    return g, constraint, reference


def test_pose_reference_cross_frame_reexpresses_reference() -> None:
    g, constraint, reference = _equality_graph(reference_wrt_name="frame-table")
    _materialize_pose_reference_transforms(_model(g))

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
    _materialize_pose_reference_transforms(_model(g))

    assert g.value(constraint, CSTR["reference-value"]) == reference
    assert (None, RDF.type, GEOM_OP.ComposePose) not in g


def test_pose_reference_rejects_body_mismatch() -> None:
    g, constraint, _ = _equality_graph(reference_wrt_name="frame-table")
    reference = g.value(constraint, CSTR["reference-value"])
    g.remove((reference, GEOM_REL.of, None))
    g.add((reference, GEOM_REL.of, _frame(g, "frame-other")))

    with pytest.raises(ConstraintViolation, match="compares a pose"):
        _materialize_pose_reference_transforms(_model(g))


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
    """An orientation composing `pose-ee-base` with a delta, slotted into
    `geom-op:in1`/`in2` base-first (`in1_is_pose`) or delta-first.
    """
    g = Dataset(default_union=True)
    base = _frame(g, "frame-base")
    ee = _frame(g, "frame-ee")
    base_pose = _pose(g, "pose-ee-base", ee, base)
    delta = _delta_node(g, "delta", (-0.75, 0.0, 0.0))

    orientation = _u("relative-orientation")
    g.add((orientation, RDF.type, GEOM_COORD.OrientationCoordinate))
    g.add((orientation, GEOM_REL.of, ee))
    g.add((orientation, GEOM_COORD["as-seen-by"], base))
    g.add((delta, GEOM_COORD["as-seen-by"], ee if in1_is_pose else base))

    composition = _u("relative-orientation-composition")
    g.add((composition, RDF.type, GEOM_OP_EXT.ComposeOrientation))
    in1, in2 = (base_pose, delta) if in1_is_pose else (delta, base_pose)
    g.add((composition, GEOM_OP["in1"], in1))
    g.add((composition, GEOM_OP["in2"], in2))
    g.add((composition, GEOM_OP["composite"], orientation))
    return g, orientation


def test_orientation_without_a_composition_operator_is_not_relative() -> None:
    """The slots alone do not make a composition: only the typed operator does."""
    g, orientation = _relative_orientation_graph(in1_is_pose=True)
    g.remove((None, RDF.type, GEOM_OP_EXT.ComposeOrientation))

    assert orientation_representation(_model(g), orientation) != "relative"
    with pytest.raises(ConstraintViolation, match="no composition operator"):
        _relative_orientation(_model(g), orientation)


def test_relative_orientation_reads_operands_in_slot_order() -> None:
    g, orientation = _relative_orientation_graph(in1_is_pose=True)
    operands = _relative_orientation(_model(g), orientation)
    assert "pose" in operands[0] and "delta" in operands[1]
    # The delta is authored as an Euler triple and leaves as the quaternion it denotes.
    assert operands[1]["representation"] == "quaternion"
    assert [c["value"] for c in operands[1]["delta"]] == pytest.approx(
        [-0.36627253, 0.0, 0.0, 0.93050762]
    )

    g, orientation = _relative_orientation_graph(in1_is_pose=False)
    operands = _relative_orientation(_model(g), orientation)
    assert "delta" in operands[0] and "pose" in operands[1]


def test_relative_orientation_rejects_a_mismatched_operand_pair() -> None:
    g, orientation = _relative_orientation_graph(in1_is_pose=True)
    # Two poses in the slots: not one pose + one delta.
    other_pose = _pose(g, "pose-other", _frame(g, "frame-other-body"), _frame(g, "frame-other-wrt"))
    composition = _u("relative-orientation-composition")
    g.remove((composition, GEOM_OP["in2"], None))
    g.add((composition, GEOM_OP["in2"], other_pose))

    with pytest.raises(ConstraintViolation, match="exactly one base pose and one delta"):
        _relative_orientation(_model(g), orientation)


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
    position = _u(f"position-{local_name(of_frame)}")
    coord = _u(f"position-coord-{local_name(of_frame)}")
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
    g.add((coord, QUDT_SCHEMA.unit, URI_QUDT_UNIT_M))
    return coord


def _scene_orientation_coord(g: Graph, of_frame: URIRef, wrt_frame: URIRef, quat) -> URIRef:
    """An Orientation coordinate at `of_frame`, wrt `wrt_frame`, as a quaternion."""
    orientation = _u(f"orientation-{local_name(of_frame)}")
    coord = _u(f"orientation-coord-{local_name(of_frame)}")
    g.add((orientation, RDF.type, GEOM_REL.Orientation))
    g.add((orientation, GEOM_REL.of, of_frame))
    g.add((orientation, GEOM_REL["with-respect-to"], wrt_frame))
    g.add((coord, RDF.type, GEOM_COORD.OrientationCoordinate))
    g.add((coord, RDF.type, URI_GEOM_TYPE_ORIENT_REF))
    g.add((coord, RDF.type, GEOM_COORD.Quaternion))
    g.add((coord, GEOM_COORD["of-orientation"], orientation))
    g.add((coord, GEOM_COORD["as-seen-by"], wrt_frame))
    x, y, z, w = quat
    for predicate, value in zip((GEOM_COORD.x, GEOM_COORD.y, GEOM_COORD.z), (x, y, z)):
        g.add((coord, predicate, Literal(float(value))))
    g.add((coord, URI_GEOM_PRED_W, Literal(float(w))))
    return coord


def _scene_pose(g: Graph, of_frame: URIRef, wrt_frame: URIRef, xyz, quat=(0.0, 0.0, 0.0, 1.0)):
    """A placement as scene-dsl emits one: a Pose over a position and an orientation.

    `_placement` composes whole poses, so a position with no orientation beside it places
    nothing -- which is what a scene author writes anyway.
    """
    position_coord = _scene_position_coord(g, of_frame, wrt_frame, xyz)
    orientation_coord = _scene_orientation_coord(g, of_frame, wrt_frame, quat)
    pose = _u(f"pose-{local_name(of_frame)}")
    coord = _u(f"pose-coord-{local_name(of_frame)}")
    g.add((pose, RDF.type, URI_GEOM_TYPE_POSE))
    g.add((pose, GEOM_REL.of, of_frame))
    g.add((pose, GEOM_REL["with-respect-to"], wrt_frame))
    g.add((pose, RDF.type, URI_GEOM_TYPE_POSITION_REF))
    g.add((pose, URI_GEOM_PRED_OF_POSITION, g.value(position_coord, GEOM_COORD["of-position"])))
    g.add((pose, RDF.type, URI_GEOM_TYPE_ORIENT_REF))
    g.add((pose, URI_GEOM_PRED_OF_ORIENT, g.value(orientation_coord, GEOM_COORD["of-orientation"])))
    g.add((coord, RDF.type, URI_GEOM_TYPE_POSE_REF))
    g.add((coord, RDF.type, URI_GEOM_TYPE_POSE_COORD))
    g.add((coord, URI_GEOM_PRED_OF_POSE, pose))
    g.add((coord, URI_GEOM_PRED_SEEN_BY, wrt_frame))
    return position_coord


def test_orientation_of_reads_quaternion_placement_as_quat() -> None:
    """A quaternion-authored placement is read back as [x, y, z, w], unmodified."""
    g = Dataset(default_union=True)
    frame = _scene_frame(g, "frame-object")
    wrt = _scene_frame(g, "frame-world")
    quat = tuple(Rotation.from_euler("xyz", (0.3, -0.2, 0.75)).as_quat())
    _scene_pose(g, frame, wrt, (0.0, 0.0, 0.0), quat)

    assert _placement(_model(g), frame, wrt)[1] == pytest.approx(quat)


def test_a_placement_composes_through_the_frames_between_it_and_the_anchor() -> None:
    """A scene places against whatever frame it likes; the runtime is built on one."""
    g = Dataset(default_union=True)
    anchor = _scene_frame(g, "frame-ground")
    middle = _scene_frame(g, "frame-world")
    frame = _scene_frame(g, "frame-object")
    _scene_pose(g, middle, anchor, (0.0, 0.0, 0.72))
    _scene_pose(g, frame, middle, (-0.9, 1.8, 0.05))

    assert _placement(_model(g), frame, anchor)[0] == pytest.approx([-0.9, 1.8, 0.77])


def test_a_frame_no_pose_leads_to_is_coincident() -> None:
    """A body a joint holds is placed by the joint, not by a pose, so it composes to nothing."""
    g = Dataset(default_union=True)
    anchor = _scene_frame(g, "frame-ground")
    frame = _scene_frame(g, "frame-jointed")

    assert _placement(_model(g), frame, anchor) == (None, None)


def test_position_of_scales_to_metres_and_rejects_a_missing_unit() -> None:
    """Reading x/y/z alone ignores the coordinate's unit; a cm-authored placement must
    scale, not pass through as if it were already metres. A missing unit is not a
    default -- it's an error.
    """
    g = Dataset(default_union=True)
    frame = _scene_frame(g, "frame-object")
    wrt = _scene_frame(g, "frame-world")
    coord = _scene_pose(g, frame, wrt, (150.0, -50.0, 720.0))
    g.remove((coord, QUDT_SCHEMA.unit, URI_QUDT_UNIT_M))
    g.add((coord, QUDT_SCHEMA.unit, URI_QUDT_UNIT_CM))

    assert _placement(_model(g), frame, wrt)[0] == pytest.approx([1.5, -0.5, 7.2])

    g2 = Dataset(default_union=True)
    frame2 = _scene_frame(g2, "frame-object")
    wrt2 = _scene_frame(g2, "frame-world")
    coord2 = _scene_pose(g2, frame2, wrt2, (1.0, 2.0, 3.0))
    g2.remove((coord2, QUDT_SCHEMA.unit, URI_QUDT_UNIT_M))  # a placement with no unit at all

    with pytest.raises(ConstraintViolation):
        _placement(_model(g2), frame2, wrt2)


def test_sampled_scene_placements_are_rejected() -> None:
    """Sampling has no motion-spec seed semantics yet, so it must never become identity."""
    g = Dataset(default_union=True)
    frame = _scene_frame(g, "frame-object")
    wrt = _scene_frame(g, "frame-world")
    coord = _scene_pose(g, frame, wrt, (1.0, 2.0, 3.0))
    g.add((coord, RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY))

    with pytest.raises(ConstraintViolation):
        _placement(_model(g), frame, wrt)
