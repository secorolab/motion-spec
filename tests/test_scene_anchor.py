# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The scene is anchored on the frame its kgraph declares, never inferred from the graph."""

from __future__ import annotations

from pathlib import Path

import pytest
import rdflib
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import URI_GEOM_TYPE_KGRAPH
from rdf_utils.namespace import NS_MM_KC_EXT
from rdflib.namespace import RDF

from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.resources import anchor_frame

NS = rdflib.Namespace("https://example.test/")


def test_a_graph_declaring_two_anchors_is_an_error() -> None:
    """Two anchors is not a choice to make silently: each puts the floor somewhere else."""
    graph = rdflib.Dataset()
    graph.add((NS["kgraph"], RDF.type, URI_GEOM_TYPE_KGRAPH))
    for anchor in ("ground", "table_top"):
        graph.add((NS["kgraph"], NS_MM_KC_EXT["anchor"], NS[anchor]))
    with pytest.raises(ConstraintViolation, match="exactly one anchor"):
        anchor_frame(Model(graph=graph, app_path=Path(".")))
