# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A free body the platform has no provider for fails at generation, not at the first tick."""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from rdf_utils.constraints import ConstraintViolation
from rdflib import Graph
from scene_dsl.kdl_tree import build_kdl_trees
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph
from support import EXAMPLES

from motion_spec.classes.scene import MjcfSceneObject, MjcfSceneSpec
from motion_spec.rdf_parser.runtime import world_ports

# These scenes declare no agents, so no agent model maps a tree onto an MJCF body.
_NO_AGENTS = SimpleNamespace(graph=Graph())


@pytest.fixture(scope="module")
def trees() -> list[dict]:
    scene = EXAMPLES["pick_and_place"] / "pick_and_place.scenex"
    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(scene))
    return build_kdl_trees(graph, scene.parent)


def test_a_free_body_nothing_places_fails_while_generating(trees) -> None:
    with pytest.raises(ConstraintViolation, match="free body the scene does not place"):
        world_ports(_NO_AGENTS, trees, MjcfSceneSpec(), [], [], [], [], "mj_kdl")


def test_a_free_body_only_the_scene_places_has_no_provider_on_hardware(trees) -> None:
    cube = next(
        tree["root_iri"]
        for tree in trees
        if not any(segment["joint"] for segment in tree["segments"])
    )
    scene = MjcfSceneSpec()
    scene.objects.append(MjcfSceneObject(id="cube", body="cube", body_iri=cube))
    with pytest.raises(ConstraintViolation, match="only a subscription can place a free body"):
        world_ports(_NO_AGENTS, trees, scene, [], [], [], [], "robif2b")
