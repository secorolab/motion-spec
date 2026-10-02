# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The world model's port table: one row per physical variable, resolved while generating.

Every measured and commanded variable the program touches is a row here, so a model asking for a
reading the platform cannot take, or a push the platform cannot apply, fails at generation rather
than at the first tick.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from rdf_utils.constraints import ConstraintViolation
from rdflib import Graph
from scene_dsl.kdl_tree import build_kdl_trees
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph

from motion_spec.classes.bindings import RuntimeBinding
from motion_spec.classes.dynamics import JointPosition
from motion_spec.classes.motion import ForwardedCommandStep
from motion_spec.classes.scene import MjcfSceneObject, MjcfSceneSpec
from motion_spec.rdf_parser.resources import world_ports

from conftest import requires_workspace
from support import DSL_MODELS, example

MODELS = DSL_MODELS

pytestmark = requires_workspace(MODELS)

# These scenes declare no agents, so no agent model maps a tree onto an MJCF body.
_NO_AGENTS = SimpleNamespace(graph=Graph())


class _Motion:
    def __init__(self, commands=(), perturbations=()):
        self.forwarded_commands = list(commands)
        self.perturbations = list(perturbations)


def _trees(model: str) -> list[dict]:
    scene = example(model) / f"{model}.scenex"
    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(scene))
    return build_kdl_trees(graph, scene.parent)


@pytest.fixture(scope="module")
def trees() -> list[dict]:
    return _trees("pick_and_place")


@pytest.fixture(scope="module")
def dual_trees() -> list[dict]:
    return _trees("dual_arm_pick_and_place")


class _Solver:
    """The little of a chain solver the port table reads."""

    def __init__(self, runtime_id: str, outputs):
        self.runtime = RuntimeBinding(
            id=runtime_id, owner=True, prefix="", owned_trees=[], config_key=""
        )
        self.output = list(outputs)
        self.devices = []


def _scene(body_iri: str) -> MjcfSceneSpec:
    scene = MjcfSceneSpec()
    scene.objects.append(MjcfSceneObject(id="cube", body="cube", body_iri=body_iri))
    return scene


def _is_free_body(tree) -> bool:
    return not any(segment["joint"] for segment in tree["segments"])


def _cube_iri(trees) -> str:
    return next(tree["root_iri"] for tree in trees if _is_free_body(tree))


def test_a_free_body_the_scene_places_becomes_a_measured_port(trees) -> None:
    ports = world_ports(_NO_AGENTS, trees, _scene(_cube_iri(trees)), [], [], [], [], "mj_kdl")
    [free_root] = ports["free_roots"]
    assert (free_root.segment, free_root.mapping, free_root.slot) == (
        "pick_and_place_graph/cube",
        "cube",
        0,
    )


def test_a_free_body_nothing_places_fails_while_generating(trees) -> None:
    with pytest.raises(ConstraintViolation, match="free body the scene does not place"):
        world_ports(_NO_AGENTS, trees, MjcfSceneSpec(), [], [], [], [], "mj_kdl")


def test_a_free_body_only_the_scene_places_has_no_provider_on_hardware(trees) -> None:
    with pytest.raises(ConstraintViolation, match="only a subscription can place a free body"):
        world_ports(_NO_AGENTS, trees, _scene(_cube_iri(trees)), [], [], [], [], "robif2b")


def test_a_perception_channel_placing_the_root_is_its_provider(trees) -> None:
    # The subscription binds that base itself, so the table owes it no row and no scene object.
    subscription = {
        "written_poses": [{"reframed": True, "observed_body_segment": "pick_and_place_graph/cube"}]
    }
    ports = world_ports(_NO_AGENTS, trees, MjcfSceneSpec(), [], [], [], [subscription], "robif2b")
    assert ports["free_roots"] == []


def test_a_forwarded_command_takes_a_command_port(trees) -> None:
    command = ForwardedCommandStep("cmd", None, "g_left_driver_joint", "arm")
    ports = world_ports(_NO_AGENTS, trees, _scene(_cube_iri(trees)), [], [_Motion([command])], [], [], "mj_kdl")
    [aux] = ports["aux_cmds"]
    assert (aux.mapping, aux.owner_id, aux.slot) == ("g_left_driver_joint", "arm", 0)
    assert command.world_slot == 0


def test_two_grippers_on_two_arms_take_two_distinct_segments(dual_trees) -> None:
    """Both grippers declare a joint of the same local name, so a name match would put both
    measurements on one segment -- which the world model refuses as two authorities for it."""
    mimics = sorted(
        segment["joint"]["iri"]
        for tree in dual_trees
        for segment in tree["segments"]
        if segment["joint"] is not None and segment["joint"]["iri"].endswith("g_left_driver_joint")
    )
    assert len(mimics) == 2
    solvers = [
        _Solver("arm1", [JointPosition("grip1", "kinova1_g_left_driver_joint", mimics[0])]),
        _Solver("arm2", [JointPosition("grip2", "kinova2_g_left_driver_joint", mimics[1])]),
    ]
    scene = MjcfSceneSpec()
    for tree in dual_trees:
        if _is_free_body(tree):
            scene.objects.append(
                MjcfSceneObject(id=tree["name"], body=tree["name"], body_iri=tree["root_iri"])
            )
    ports = world_ports(_NO_AGENTS, dual_trees, scene, solvers, [], [], [], "mj_kdl")
    segments = [port.segment for port in ports["joints"]]
    assert all(segments) and len(set(segments)) == 2


def test_a_perturbation_on_hardware_fails_while_generating(trees) -> None:
    bodies = [{"body": "cube", "members": []}]
    # Perception places the cube, so the push is the only thing hardware has no provider for.
    placed = {
        "written_poses": [{"reframed": True, "observed_body_segment": "pick_and_place_graph/cube"}]
    }
    with pytest.raises(ConstraintViolation, match="no actuator on robif2b"):
        world_ports(_NO_AGENTS, trees, MjcfSceneSpec(), [], [], bodies, [placed], "robif2b")
