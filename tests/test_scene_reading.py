# SPDX-License-Identifier: MPL-2.0
"""Characterization tests for the IRI-shape scene-reading helpers (plan 022 step 1).

Pins today's output of body resolution/_mapped_targets/_kinematic_adjacency across the three
maintained models, so a graph-derived replacement fails loudly if it changes an answer. Values
below are recorded as-is, not reasoned "correct" answers -- some look surprising (see the report)
but are recorded verbatim per plan 022 step 1's instruction.

_tree_owns is gone: step 3 replaced it with the graph-derived agent-assembly derivation in
resources._agent_bindings/_agent_assemblies, covered by the three-model behaviour gate instead.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest
from motion_spec_dsl.rdf_parser.vocab import AGN, GEOM_ENT
from rdflib.namespace import RDF
from scene_dsl.rdf_parser.kinematics import body_of_frame

from motion_spec.rdf_parser.ir import generate_ir
from motion_spec.rdf_parser.model import load_model
from motion_spec.rdf_parser.resources import kinematic_adjacency, mapped_targets

MODELS = Path(__file__).parents[2] / "motion-spec-dsl" / "models"

from conftest import requires_workspace

pytestmark = requires_workspace(MODELS)
MODEL_NAMES = ["pick_place_single", "pick_place_dual", "admittance_arc_single"]


def _characterize(name, tmp_path):
    """DSL-compile <name>.robmot to JSON-LD, run the current IR pipeline once, and record what
    body_of_frame/mapped_targets/kinematic_adjacency produce for it today."""
    model_dir = MODELS / name
    outdir = tmp_path / "generated" / "model"
    subprocess.run(
        ["textx", "generate", f"{name}.robmot", "--target", "jsonld", "-o", str(outdir)],
        cwd=model_dir,
        check=True,
    )
    manifest = outdir / f"{name}-app.ld.json"
    model = load_model(manifest)
    g = model.graph

    frames = sorted(g.subjects(RDF.type, GEOM_ENT.Frame), key=str)
    body_of = {str(frame): str(body_of_frame(frame, g)) for frame in frames}

    targets = {str(t) for t in mapped_targets(model, AGN["AgentModel"], GEOM_ENT.KinematicTree)}

    adjacency, _fixed = kinematic_adjacency(model)
    edges = sorted(
        {
            tuple(sorted((str(body), str(neighbor))))
            for body, neighbors in adjacency.items()
            for neighbor, *_rest in neighbors
        }
    )

    generate_ir(manifest)

    return body_of, targets, edges


@pytest.fixture(scope="module", params=MODEL_NAMES)
def characterized(request, tmp_path_factory):
    name = request.param
    body_of, targets, edges = _characterize(name, tmp_path_factory.mktemp(name))
    return name, body_of, targets, edges


def test_body_of_every_frame(characterized):
    """The body each geom:Frame sits on, as scene-dsl resolves it from the graph."""
    name, body_of, _targets, _edges = characterized
    expected = _BODY_OF[name]
    assert len(body_of) == len(expected)
    assert body_of == expected


def test_mapped_targets_kinematic_tree(characterized):
    """mapped_targets(model, AGN.AgentModel, GEOM_ENT.KinematicTree)."""
    name, _body_of, targets, _edges = characterized
    assert targets == _MAPPED_TARGETS[name]


def test_kinematic_adjacency_edges(characterized):
    """kinematic_adjacency(model) edges, as sorted (body_a, body_b) string pairs."""
    name, _body_of, _targets, edges = characterized
    assert edges == _ADJACENCY_EDGES[name]


_BODY_OF = {
    "pick_place_single": {
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/ft_tree/wrist_ft_body/wrist_ft_body_origin": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/ft_tree/wrist_ft_body",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/ft_tree/wrist_ft_body/wrist_ft_mount": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/ft_tree/wrist_ft_body",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/ft_tree/wrist_ft_body/wrist_ft_site": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/ft_tree/wrist_ft_body",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base/g_base_com": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base/g_base_lumped_com": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base/g_base_origin": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base/g_left_driver_joint_anchor": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base/g_pinch": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base_mount/g_base_mount_com": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base_mount",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base_mount/g_base_mount_origin": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base_mount",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base_mount/g_mount_interface": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base_mount",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_left_driver/g_left_driver_com": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_left_driver",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_left_driver/g_left_driver_origin": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_left_driver",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/base_link/base_link_com": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/base_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/base_link/base_link_origin": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/base_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/base_link/joint_1_anchor": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/base_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/bracelet_link/bracelet_link_com": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/bracelet_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/bracelet_link/bracelet_link_origin": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/bracelet_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/bracelet_link/pinch_site": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/bracelet_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/bracelet_link/wrist_anchor": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/bracelet_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/forearm_link/forearm_link_com": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/forearm_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/forearm_link/forearm_link_origin": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/forearm_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/forearm_link/joint_5_anchor": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/forearm_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_1_link/half_arm_1_link_com": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_1_link/half_arm_1_link_origin": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_1_link/joint_3_anchor": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_2_link/half_arm_2_link_com": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_2_link/half_arm_2_link_origin": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_2_link/joint_4_anchor": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/shoulder_link/joint_2_anchor": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/shoulder_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/shoulder_link/shoulder_link_com": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/shoulder_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/shoulder_link/shoulder_link_origin": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/shoulder_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_1_link/joint_6_anchor": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_1_link/spherical_wrist_1_link_com": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_1_link/spherical_wrist_1_link_origin": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_2_link/joint_7_anchor": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_2_link/spherical_wrist_2_link_com": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_2_link/spherical_wrist_2_link_origin": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/pick_place_graph/cube/cube_origin": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/pick_place_graph/cube",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/world_tree/table/table_com": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/world_tree/table",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/world_tree/table/table_top": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/world_tree/table",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/world_tree/world_body/world": "https://secorolab.github.io/models/scenes/pick-place-single-mjc/world_tree/world_body",
    },
    "pick_place_dual": {
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base/g_base_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base/g_base_lumped_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base/g_base_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base/g_left_driver_joint_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base/g_pinch": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base_mount/g_base_mount_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base_mount",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base_mount/g_base_mount_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base_mount",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base_mount/g_mount_interface": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base_mount",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_left_driver/g_left_driver_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_left_driver",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_left_driver/g_left_driver_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_left_driver",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base/g_base_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base/g_base_lumped_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base/g_base_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base/g_left_driver_joint_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base/g_pinch": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base_mount/g_base_mount_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base_mount",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base_mount/g_base_mount_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base_mount",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base_mount/g_mount_interface": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base_mount",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_left_driver/g_left_driver_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_left_driver",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_left_driver/g_left_driver_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_left_driver",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/base_link/base_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/base_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/base_link/base_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/base_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/base_link/joint_1_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/base_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/bracelet_link/bracelet_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/bracelet_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/bracelet_link/bracelet_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/bracelet_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/bracelet_link/pinch_site": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/bracelet_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/bracelet_link/wrist_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/bracelet_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/forearm_link/forearm_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/forearm_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/forearm_link/forearm_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/forearm_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/forearm_link/joint_5_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/forearm_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_1_link/half_arm_1_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_1_link/half_arm_1_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_1_link/joint_3_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_2_link/half_arm_2_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_2_link/half_arm_2_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_2_link/joint_4_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/shoulder_link/joint_2_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/shoulder_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/shoulder_link/shoulder_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/shoulder_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/shoulder_link/shoulder_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/shoulder_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_1_link/joint_6_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_1_link/spherical_wrist_1_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_1_link/spherical_wrist_1_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_2_link/joint_7_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_2_link/spherical_wrist_2_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_2_link/spherical_wrist_2_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/base_link/base_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/base_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/base_link/base_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/base_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/base_link/joint_1_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/base_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/bracelet_link/bracelet_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/bracelet_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/bracelet_link/bracelet_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/bracelet_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/bracelet_link/pinch_site": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/bracelet_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/bracelet_link/wrist_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/bracelet_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/forearm_link/forearm_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/forearm_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/forearm_link/forearm_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/forearm_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/forearm_link/joint_5_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/forearm_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_1_link/half_arm_1_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_1_link/half_arm_1_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_1_link/joint_3_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_2_link/half_arm_2_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_2_link/half_arm_2_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_2_link/joint_4_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/shoulder_link/joint_2_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/shoulder_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/shoulder_link/shoulder_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/shoulder_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/shoulder_link/shoulder_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/shoulder_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_1_link/joint_6_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_1_link/spherical_wrist_1_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_1_link/spherical_wrist_1_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_1_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_2_link/joint_7_anchor": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_2_link/spherical_wrist_2_link_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_2_link/spherical_wrist_2_link_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_2_link",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/pick_place_graph/cube/cube_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/pick_place_graph/cube",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/pick_place_graph/cube2/cube2_origin": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/pick_place_graph/cube2",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/table/kinova1_mount": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/table",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/table/kinova2_mount": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/table",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/table/table_com": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/table",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/table/table_top": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/table",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/world_body/world": "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/world_body",
    },
    "admittance_arc_single": {
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/ft_tree/wrist_ft_body/wrist_ft_body_origin": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/ft_tree/wrist_ft_body",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/ft_tree/wrist_ft_body/wrist_ft_mount": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/ft_tree/wrist_ft_body",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/ft_tree/wrist_ft_body/wrist_ft_site": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/ft_tree/wrist_ft_body",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base/g_base_com": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base/g_base_lumped_com": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base/g_base_origin": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base/g_left_driver_joint_anchor": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base/g_pinch": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base_mount/g_base_mount_com": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base_mount",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base_mount/g_base_mount_origin": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base_mount",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base_mount/g_mount_interface": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base_mount",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_left_driver/g_left_driver_com": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_left_driver",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_left_driver/g_left_driver_origin": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_left_driver",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/base_link/base_link_com": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/base_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/base_link/base_link_origin": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/base_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/base_link/joint_1_anchor": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/base_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/bracelet_link/bracelet_link_com": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/bracelet_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/bracelet_link/bracelet_link_origin": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/bracelet_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/bracelet_link/pinch_site": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/bracelet_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/bracelet_link/wrist_anchor": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/bracelet_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/forearm_link/forearm_link_com": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/forearm_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/forearm_link/forearm_link_origin": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/forearm_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/forearm_link/joint_5_anchor": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/forearm_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_1_link/half_arm_1_link_com": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_1_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_1_link/half_arm_1_link_origin": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_1_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_1_link/joint_3_anchor": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_1_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_2_link/half_arm_2_link_com": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_2_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_2_link/half_arm_2_link_origin": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_2_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_2_link/joint_4_anchor": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_2_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/shoulder_link/joint_2_anchor": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/shoulder_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/shoulder_link/shoulder_link_com": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/shoulder_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/shoulder_link/shoulder_link_origin": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/shoulder_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_1_link/joint_6_anchor": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_1_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_1_link/spherical_wrist_1_link_com": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_1_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_1_link/spherical_wrist_1_link_origin": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_1_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_2_link/joint_7_anchor": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_2_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_2_link/spherical_wrist_2_link_com": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_2_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_2_link/spherical_wrist_2_link_origin": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_2_link",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/world_tree/table/table_com": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/world_tree/table",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/world_tree/table/table_top": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/world_tree/table",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/world_tree/world_body/world": "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/world_tree/world_body",
    },
}

_MAPPED_TARGETS = {
    "pick_place_single": {
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/ft_tree",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper",
        "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova",
    },
    "pick_place_dual": {
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1",
        "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2",
    },
    "admittance_arc_single": {
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/ft_tree",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper",
        "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova",
    },
}

_ADJACENCY_EDGES = {
    "pick_place_single": [
        (
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/ft_tree/wrist_ft_body",
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base_mount",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/ft_tree/wrist_ft_body",
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/bracelet_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base",
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base_mount",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base",
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_left_driver",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/base_link",
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/shoulder_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/base_link",
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/world_tree/table",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/bracelet_link",
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/forearm_link",
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/forearm_link",
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_1_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_1_link",
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_1_link",
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/shoulder_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_1_link",
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/world_tree/table",
            "https://secorolab.github.io/models/scenes/pick-place-single-mjc/world_tree/world_body",
        ),
    ],
    "pick_place_dual": [
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base_mount",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_left_driver",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base_mount",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/bracelet_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base_mount",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_left_driver",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base_mount",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/bracelet_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/base_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/shoulder_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/base_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/table",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/bracelet_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/forearm_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/forearm_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_1_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_1_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_1_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/shoulder_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_1_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/base_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/shoulder_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/base_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/table",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/bracelet_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/forearm_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/forearm_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_1_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_1_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_1_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/shoulder_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_1_link",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/table",
            "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/world_body",
        ),
    ],
    "admittance_arc_single": [
        (
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/ft_tree/wrist_ft_body",
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base_mount",
        ),
        (
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/ft_tree/wrist_ft_body",
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/bracelet_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base",
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base_mount",
        ),
        (
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_base",
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/gripper/g_left_driver",
        ),
        (
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/base_link",
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/shoulder_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/base_link",
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/world_tree/table",
        ),
        (
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/bracelet_link",
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/forearm_link",
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/forearm_link",
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_1_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_1_link",
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/half_arm_1_link",
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/shoulder_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_1_link",
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/kinova/spherical_wrist_2_link",
        ),
        (
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/world_tree/table",
            "https://secorolab.github.io/models/scenes/admittance-arc-single-mjc/world_tree/world_body",
        ),
    ],
}
