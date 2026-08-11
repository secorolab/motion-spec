# SPDX-License-Identifier: MPL-2.0
"""Motion-spec's adapter over scene-dsl's renderer-ready KDL IR."""

from pathlib import Path

from scene_dsl.kdl_tree import build_kdl_trees
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph

from motion_spec.generation.scene_kdl import (
    chain_for_iri,
    kdl_header_name,
    model_stem,
    write_scene_kdl_header,
)

MODELS = Path(__file__).parents[2] / "motion-spec-dsl" / "models"

from conftest import requires_workspace

pytestmark = requires_workspace(MODELS)


def test_scene_kdl_adapter_derives_solver_chains_and_writes_header(tmp_path: Path) -> None:
    scene = MODELS / "pick_place_single" / "pick_place_single.scenex"
    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(scene))

    trees = build_kdl_trees(graph, scene.parent)
    chain = next(chain for tree in trees for chain in tree["chains"])
    record = chain_for_iri(trees, chain["iri"])
    assert record["joints"] == [f"joint_{number}" for number in range(1, 8)]
    # The world model binds a measurement to the segment a joint moves, so every chain joint has
    # to name one -- and the chain's own endpoints have to be nameable in the whole tree.
    assert len(record["joint_segments"]) == len(record["joints"])
    assert all(record["joint_segments"])
    names = set(record["world_segments"].values())
    assert record["world_root"] in names
    assert record["world_tip"] in names
    assert set(record["joint_segments"]) <= names

    header = write_scene_kdl_header(graph, tmp_path, scene.name, scene.parent)
    assert header.name == "pick_place_single.kdl.hpp"
    # The written header and the IR's published `configuration.model_name` share one derivation,
    # so the include a backend composes can never name a file the pipeline did not write.
    manifest = "pick_place_single-app.ld.json"
    assert kdl_header_name(manifest) == header.name
    assert kdl_header_name(manifest) == f"{model_stem(manifest)}.kdl.hpp"
    assert "make_chain_kinova_2f85_chain" in header.read_text()
