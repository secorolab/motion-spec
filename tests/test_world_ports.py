# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The world model's port table: one row per physical variable, resolved while generating.

Every measured and commanded variable the program touches is a row here, so a model asking for a
reading the platform cannot take, or a push the platform cannot apply, fails at generation rather
than at the first tick.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from rdf_utils.constraints import ConstraintViolation
from scene_dsl.kdl_tree import build_kdl_trees
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph

from motion_spec.classes.motion import ForwardedCommandStep
from motion_spec.classes.scene import MjcfSceneObject, MjcfSceneSpec
from motion_spec.rdf_parser.resources import world_ports

from conftest import requires_workspace

MODELS = Path(__file__).parents[2] / "motion-spec-dsl" / "models"

pytestmark = requires_workspace(MODELS)


class _Motion:
    def __init__(self, commands=(), perturbations=()):
        self.forwarded_commands = list(commands)
        self.perturbations = list(perturbations)


@pytest.fixture(scope="module")
def trees() -> list[dict]:
    scene = MODELS / "pick_place_single" / "pick_place_single.scenex"
    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(scene))
    return build_kdl_trees(graph, scene.parent)


def _scene(body_iri: str) -> MjcfSceneSpec:
    scene = MjcfSceneSpec()
    scene.objects.append(MjcfSceneObject(id="cube", body="cube", body_iri=body_iri))
    return scene


def _cube_iri(trees) -> str:
    return next(tree["root_iri"] for tree in trees if tree["free_root"])


def test_a_free_body_the_scene_places_becomes_a_measured_port(trees) -> None:
    ports = world_ports(trees, _scene(_cube_iri(trees)), [], [], [], [], "mj_kdl")
    [free_root] = ports["free_roots"]
    assert (free_root.segment, free_root.mapping, free_root.slot) == (
        "pick_place_graph/cube",
        "cube",
        0,
    )


def test_a_free_body_nothing_places_fails_while_generating(trees) -> None:
    with pytest.raises(ConstraintViolation, match="free body the scene does not place"):
        world_ports(trees, MjcfSceneSpec(), [], [], [], [], "mj_kdl")


def test_a_perception_channel_placing_the_root_is_its_provider(trees) -> None:
    # The subscription binds that base itself, so the table owes it no row and no scene object.
    subscription = {
        "written_poses": [{"reframed": True, "observed_body_segment": "pick_place_graph/cube"}]
    }
    ports = world_ports(trees, MjcfSceneSpec(), [], [], [], [subscription], "robif2b")
    assert ports["free_roots"] == []


def test_a_forwarded_command_takes_a_command_port(trees) -> None:
    command = ForwardedCommandStep("cmd", None, "g_left_driver_joint", "arm")
    ports = world_ports(trees, _scene(_cube_iri(trees)), [], [_Motion([command])], [], [], "mj_kdl")
    [aux] = ports["aux_cmds"]
    assert (aux.mapping, aux.owner_id, aux.slot) == ("g_left_driver_joint", "arm", 0)
    assert command.world_slot == 0


def test_a_perturbation_on_hardware_fails_while_generating(trees) -> None:
    bodies = [{"body": "cube", "members": []}]
    with pytest.raises(ConstraintViolation, match="no actuator on robif2b"):
        world_ports(trees, _scene(_cube_iri(trees)), [], [], bodies, [], "robif2b")
