# SPDX-License-Identifier: MPL-2.0
from __future__ import annotations

import pytest
from rdflib import Graph, URIRef
from rdflib.namespace import RDF

from motion_spec.ir_gen import (
    Parser,
    _materialize_linear_distance_operations,
    _materialize_pose_reference_transforms,
)
from motion_spec.namespace import (
    CSTR,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_OP,
    GEOM_REL,
    QUDT_QKIND,
    QUDT_SCHEMA,
    QUDT_UNIT,
    RBDYN_COORD,
    RBDYN_ENT,
    SOSA,
)

BASE = "https://example.test/"


def _u(name: str) -> URIRef:
    return URIRef(BASE + name)


def _frame(g: Graph, name: str) -> URIRef:
    node = _u(name)
    g.add((node, RDF.type, GEOM_ENT.Frame))
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
    assert end_in_start in set(g.subjects(RDF.type, GEOM_REL.Pose))


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
    assert g.value(reexpressed, GEOM_REL["with-respect-to"]) == _u("frame-base")


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
# SOSA force/torque observation
# --------------------------------------------------------------------------- #
def _wrench_graph(*, sensor: URIRef | None) -> tuple[Graph, URIRef]:
    g = Graph()
    node = _u("wrench")
    g.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    g.add((node, RDF.type, RBDYN_COORD.WrenchCoordinate))
    g.add((node, RDF.type, GEOM_COORD.VectorXYZ))
    g.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Force))
    g.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Torque))
    g.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.N))
    point = _u("ref-point")
    g.add((point, RDF.type, GEOM_ENT.Point))
    g.add((node, RBDYN_ENT["reference-point"], point))
    g.add((node, RBDYN_COORD["as-seen-by"], _frame(g, "frame-ee")))
    if sensor is not None:
        g.add((node, SOSA.madeBySensor, sensor))
    return g, node


def test_wrench_reads_sosa_made_by_sensor() -> None:
    sensor = _u("ft_sensor")
    g, node = _wrench_graph(sensor=sensor)
    wrench = Parser(g).wrench(node)
    assert wrench.sensor_name == Parser(g).id(sensor)
    assert wrench.sensor_name != ""


def test_wrench_without_sensor_has_empty_name() -> None:
    g, node = _wrench_graph(sensor=None)
    wrench = Parser(g).wrench(node)
    assert wrench.sensor_name == ""
