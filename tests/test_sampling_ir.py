# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""What the controller draws at startup, and the two ways the scene and the draw disagree."""

from pathlib import Path

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
from rdflib import Dataset

from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.sampling import sampled_quantities

NS = "https://example.test/"

# A marker position drawn from a 3-d uniform distribution.
DRAWN_POSITION = f"""
@prefix ex: <{NS}> .
ex:marker_position a <{URI_DISTRIB_TYPE_SAMPLED_QUANTITY}> ;
    <{URI_DISTRIB_PRED_FROM_DISTRIB}> ex:marker_spread .
ex:marker_spread a <{URI_DISTRIB_TYPE_UNIFORM}> ; <{URI_DISTRIB_PRED_DIM}> 3 ;
    <{URI_DISTRIB_PRED_LOWER}> ( -1.0e0 -1.0e0 -1.0e0 ) ;
    <{URI_DISTRIB_PRED_UPPER}> ( 1.0e0 1.0e0 1.0e0 ) .
"""

DRAWN_SCALAR = f"""
@prefix ex: <{NS}> .
ex:stiffness a <{URI_DISTRIB_TYPE_SAMPLED_QUANTITY}> ;
    <{URI_DISTRIB_PRED_FROM_DISTRIB}> ex:marker_spread .
ex:marker_spread a <{URI_DISTRIB_TYPE_UNIFORM}> ; <{URI_DISTRIB_PRED_DIM}> 1 ;
    <{URI_DISTRIB_PRED_LOWER}> -1.0e0 ; <{URI_DISTRIB_PRED_UPPER}> 1.0e0 .
"""

UNPLACED_MARKER = {
    "name": "arm/link1/marker",
    "iri": f"{NS}marker",
    "parent": "arm/link1",
    "body": "arm/link1",
    "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
    "position_coord_iri": f"{NS}marker_position",
}


def test_an_unplaced_frame_with_a_distribution_is_drawn_into_its_tree() -> None:
    """The ordinary case: scene-dsl could not place it, so the run draws it and adds the segment."""
    graph = Dataset(default_union=True)
    graph.default_graph.parse(data=DRAWN_POSITION, format="turtle")

    [drawn] = sampled_quantities(
        Model(graph=graph, app_path=Path("model-app.ld.json"), namespaces=(NS,)),
        [{"name": "arm", "cpp_name": "arm", "unplaced_frames": [UNPLACED_MARKER]}],
    )

    assert drawn.segment == "arm/link1/marker"
    assert drawn.parent == "arm/link1"
    assert drawn.tree == "arm"
    assert drawn.rotation == [0.0, 0.0, 0.0, 1.0]
    assert drawn.size == 3
    # A frame joins its tree as a segment; only a scalar lands in a member of its own.
    assert drawn.data_member is None


def test_a_scalar_is_drawn_into_a_member_of_its_own() -> None:
    graph = Dataset(default_union=True)
    graph.default_graph.parse(data=DRAWN_SCALAR, format="turtle")

    [drawn] = sampled_quantities(
        Model(graph=graph, app_path=Path("model-app.ld.json"), namespaces=(NS,)),
        [{"name": "arm", "cpp_name": "arm", "unplaced_frames": []}],
    )

    assert drawn.data_member == "stiffness"
    assert drawn.size == 1
    assert drawn.segment is None
    assert drawn.tree is None


# No coordinates and no distribution: the frame would never be placed at all. Coordinates in the
# graph and a distribution to draw from: the two disagree.
@pytest.mark.parametrize(
    ("data", "unplaced", "rejection"),
    [
        ("", [UNPLACED_MARKER], "nothing places it"),
        (
            DRAWN_POSITION + f"ex:marker_position a <{URI_GEOM_TYPE_VECTOR_XYZ}> .\n",
            [],
            "nothing should draw it",
        ),
    ],
    ids=["nothing-draws", "scene-also-places"],
)
def test_a_frame_drawn_and_placed_by_anything_but_exactly_one_source_is_an_error(
    data: str, unplaced: list, rejection: str
) -> None:
    graph = Dataset(default_union=True)
    graph.default_graph.parse(data=data, format="turtle")

    with pytest.raises(ConstraintViolation, match=rejection):
        sampled_quantities(
            Model(graph=graph, app_path=Path("model-app.ld.json"), namespaces=(NS,)),
            [{"name": "arm", "cpp_name": "arm", "unplaced_frames": unplaced}],
        )
