# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The floor is measured from the frame the scene is anchored at, not from whatever the arm is
bolted to: a scene that mounts its arm on a table would otherwise put the ground at the table top.
"""

from __future__ import annotations

from pathlib import Path

import rdflib
from rdf_utils.models.vocab import (
    URI_EXEC_PRED_PATH,
    URI_GEOM_PRED_SIMPLICES,
    URI_GEOM_TYPE_KGRAPH,
    URI_GEOM_TYPE_RIGID_BODY,
    URI_KC_EXT_PRED_ROOT,
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_TYPE_JOINT,
)
from rdflib.namespace import RDF

from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.resources import _asset_path, _world_body

NS = rdflib.Namespace("https://example.test/")


def _model() -> Model:
    """A graph rooted at two bodies: a world nothing holds, and a table the arm is jointed to."""
    graph = rdflib.Dataset()
    graph.add((NS["kgraph"], RDF.type, URI_GEOM_TYPE_KGRAPH))
    for body, frame in (("world", "root"), ("table", "top-center")):
        graph.add((NS[body], RDF.type, URI_GEOM_TYPE_RIGID_BODY))
        graph.add((NS[body], URI_GEOM_PRED_SIMPLICES, NS[frame]))
        graph.add((NS["kgraph"], URI_KC_EXT_PRED_ROOT, NS[frame]))
    graph.add((NS["arm_on_table"], RDF.type, URI_KC_TYPE_JOINT))
    graph.add((NS["arm_on_table"], URI_KC_PRED_BETWEEN_ATTACHMENTS, NS["top-center"]))
    graph.add((NS["arm_on_table"], URI_KC_PRED_BETWEEN_ATTACHMENTS, NS["base_link"]))
    return Model(graph=graph, app_path=Path("."), imported_models=[], imported_provenance=[])


def test_world_body_is_the_root_no_joint_holds() -> None:
    assert _world_body(_model()) == NS["world"]


def test_asset_path_keeps_a_path_no_package_qualifies() -> None:
    model = _model()
    model.graph.add((NS["asset"], URI_EXEC_PRED_PATH, rdflib.Literal("a.xml")))
    assert _asset_path(model.graph, NS["asset"]) == "a.xml"
