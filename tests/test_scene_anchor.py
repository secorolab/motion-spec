# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The scene is anchored where the graph says it is: on the frame its kgraph declares.

Nothing infers it from the shape of the graph -- a scene with several roots, or one whose
world is a joint's parent, would otherwise be a guess, and the guess decides both where every
placement is composed from and where the floor is.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import rdflib
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import URI_EXEC_PRED_PATH, URI_GEOM_TYPE_KGRAPH
from rdf_utils.namespace import NS_MM_KC_EXT
from rdflib.namespace import RDF

from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.resources import _asset_path, anchor_frame

NS = rdflib.Namespace("https://example.test/")


def _model(*anchors: str) -> Model:
    """A graph whose kgraph declares the given frames as its anchor."""
    graph = rdflib.Dataset()
    graph.add((NS["kgraph"], RDF.type, URI_GEOM_TYPE_KGRAPH))
    for anchor in anchors:
        graph.add((NS["kgraph"], NS_MM_KC_EXT["anchor"], NS[anchor]))
    return Model(graph=graph, app_path=Path("."), imported_models=[], imported_provenance=[])


def test_the_anchor_is_the_frame_the_graph_declares() -> None:
    assert anchor_frame(_model("ground")) == NS["ground"]


def test_a_graph_declaring_no_anchor_is_an_error() -> None:
    with pytest.raises(ConstraintViolation, match="exactly one anchor"):
        anchor_frame(_model())


def test_a_graph_declaring_two_anchors_is_an_error() -> None:
    """Two anchors is not a choice to make silently: each puts the floor somewhere else."""
    with pytest.raises(ConstraintViolation, match="exactly one anchor"):
        anchor_frame(_model("ground", "table_top"))


def test_asset_path_keeps_a_path_no_package_qualifies() -> None:
    model = _model("ground")
    model.graph.add((NS["asset"], URI_EXEC_PRED_PATH, rdflib.Literal("a.xml")))
    assert _asset_path(model.graph, NS["asset"]) == "a.xml"
