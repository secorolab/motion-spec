# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""What the controller draws at startup, and the two ways the scene and the draw disagree."""

import pytest
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import (
    URI_DISTRIB_PRED_DIM,
    URI_DISTRIB_PRED_FROM_DISTRIB,
    URI_DISTRIB_PRED_LOWER,
    URI_DISTRIB_PRED_UPPER,
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_DISTRIB_TYPE_UNIFORM,
    URI_GEOM_TYPE_VECTOR_XYZ,
)
from rdflib import BNode, Graph, Literal, URIRef
from rdflib.collection import Collection
from rdflib.namespace import RDF, split_uri

from motion_spec.rdf_parser.sampling import sampled_quantities

NS = "https://example.test/"
COORD = URIRef(f"{NS}marker_position")
DISTRIBUTION = URIRef(f"{NS}marker_spread")


class _Model:
    """Only what sampled_quantities reads off a model: the graph, and a name per node."""

    def __init__(self, graph: Graph) -> None:
        self.graph = graph

    def id(self, node) -> str:
        return split_uri(node)[1]


def _uniform(graph: Graph, node: URIRef, dimension: int) -> None:
    graph.add((node, RDF.type, URI_DISTRIB_TYPE_UNIFORM))
    graph.add((node, URI_DISTRIB_PRED_DIM, Literal(dimension)))
    for predicate, bound in ((URI_DISTRIB_PRED_LOWER, -1.0), (URI_DISTRIB_PRED_UPPER, 1.0)):
        if dimension == 1:
            graph.add((node, predicate, Literal(bound)))
            continue
        head = BNode()
        Collection(graph, head, [Literal(bound)] * dimension)
        graph.add((node, predicate, head))


def _drawn_position(graph: Graph, *, placed: bool = False) -> None:
    graph.add((COORD, RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY))
    if placed:
        graph.add((COORD, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
    graph.add((COORD, URI_DISTRIB_PRED_FROM_DISTRIB, DISTRIBUTION))
    _uniform(graph, DISTRIBUTION, 3)


def _tree(unplaced: list[dict]) -> dict:
    return {"name": "arm", "cpp_name": "arm", "unplaced_frames": unplaced}


def _unplaced_marker() -> dict:
    return {
        "name": "arm/link1/marker",
        "iri": f"{NS}marker",
        "parent": "arm/link1",
        "body": "arm/link1",
        "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "position_coord_iri": str(COORD),
    }


def test_an_unplaced_frame_with_a_distribution_is_drawn_into_its_tree() -> None:
    """The ordinary case: scene-dsl could not place it, so the run draws it and adds the segment."""
    graph = Graph()
    _drawn_position(graph)

    [drawn] = sampled_quantities(_Model(graph), [_tree([_unplaced_marker()])])

    assert drawn.segment == "arm/link1/marker"
    assert drawn.parent == "arm/link1"
    assert drawn.tree == "arm"
    assert drawn.rotation == [0.0, 0.0, 0.0, 1.0]
    assert drawn.size == 3
    # A frame joins its tree as a segment; only a scalar lands in a member of its own.
    assert drawn.shared_member is None


def test_a_scalar_is_drawn_into_a_member_of_its_own() -> None:
    graph = Graph()
    node = URIRef(f"{NS}stiffness")
    graph.add((node, RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY))
    graph.add((node, URI_DISTRIB_PRED_FROM_DISTRIB, DISTRIBUTION))
    _uniform(graph, DISTRIBUTION, 1)

    [drawn] = sampled_quantities(_Model(graph), [_tree([])])

    assert drawn.shared_member == "stiffness"
    assert drawn.size == 1
    assert drawn.segment is None
    assert drawn.tree is None


def test_an_unplaced_frame_nothing_draws_is_an_error() -> None:
    """No coordinates and no distribution: the frame would never be placed at all."""
    graph = Graph()

    with pytest.raises(ConstraintViolation, match="nothing places it"):
        sampled_quantities(_Model(graph), [_tree([_unplaced_marker()])])


def test_a_drawn_position_the_scene_also_places_is_an_error() -> None:
    """Coordinates in the graph and a distribution to draw from: the two disagree."""
    graph = Graph()
    _drawn_position(graph, placed=True)

    with pytest.raises(ConstraintViolation, match="nothing should draw it"):
        sampled_quantities(_Model(graph), [_tree([])])


def test_a_tree_from_an_older_scene_dsl_says_so() -> None:
    graph = Graph()
    _drawn_position(graph)

    with pytest.raises(ConstraintViolation, match="does not report unplaced frames"):
        sampled_quantities(_Model(graph), [{"name": "arm", "cpp_name": "arm"}])
