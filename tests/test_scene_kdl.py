# SPDX-License-Identifier: MPL-2.0
"""Motion-spec's adapter over scene-dsl's renderer-ready KDL IR."""

from scene_dsl.kdl_tree import build_kdl_trees
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph
from support import DSL_MODELS, example

from motion_spec.generation.scene_kdl import chain_for_iri

MODELS = DSL_MODELS

from conftest import requires_workspace

pytestmark = requires_workspace(MODELS)


def test_scene_kdl_adapter_derives_solver_chains() -> None:
    scene = example("pick_and_place") / "pick_and_place.scenex"
    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(scene))

    # The namespace is the one scene-dsl's header for this scene declares.
    trees = [
        {**tree, "namespace": "pick_and_place"} for tree in build_kdl_trees(graph, scene.parent)
    ]
    chain = next(chain for tree in trees for chain in tree["chains"])
    record = chain_for_iri(trees, chain["iri"])
    assert record["namespace"] == "pick_and_place"
    assert record["joints"] == [f"joint_{number}" for number in range(1, 8)]
    # The world model binds a measurement to the segment a joint moves, so every chain joint has
    # to name one -- and the chain's own endpoints have to be nameable in the whole tree.
    assert len(record["joint_segments"]) == len(record["joints"])
    assert all(record["joint_segments"])
    names = set(record["world_segments"].values())
    assert record["world_root"] in names
    assert record["world_tip"] in names
    assert set(record["joint_segments"]) <= names
