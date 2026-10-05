# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Where a frame sits on a solver's chain is resolved while generating, so a frame the chain
never reaches is an error here rather than a dead controller on its first tick."""

from __future__ import annotations

import pytest
from motion_spec_dsl.rdf_parser.vocab import SLV
from rdf_utils.constraints import ConstraintViolation

from motion_spec.classes.bindings import ChainBinding, HardwareBinding, RuntimeBinding
from motion_spec.classes.dynamics import JointQuantity
from motion_spec.classes.geometry import Frame, SimplicialComplex
from motion_spec.classes.solvers import (
    JointForceSpecification,
    MotionDrivers,
    SolverWithInputAndOutput,
)
from motion_spec.rdf_parser.agents import place_on_chain
from motion_spec.rdf_parser.constraint_handler import SOLVER_FAMILIES
from motion_spec.rdf_parser.runtime import index_chain_joints

SITE = "https://example.test/ft_tree/wrist_ft_body/wrist_ft_site"
BODY = "https://example.test/ft_tree/wrist_ft_body"
OFFSET_SITE = "https://example.test/ft_tree/wrist_ft_body/wrist_ft_offset"

FT_CHAIN = ChainBinding(
    root="base_link",
    end="tip",
    tip="tip",
    tree="tree",
    namespace="scene",
    name="chain",
    joints=[],
    frames={SITE: {"index": 8, "offset": None}, OFFSET_SITE: {"index": 8, "offset": {"x": 0.1}}},
    bodies={BODY: 8},
    world_segments={
        SITE: "wrist_ft_body/wrist_ft_site",
        OFFSET_SITE: "wrist_ft_body/wrist_ft_offset",
        BODY: "wrist_ft_body",
    },
)


# An offset frame still fails where the read stays chain-relative: `f_ext[index - 1]` and the
# velocity solver are indexed by the chain, which has no segment for a frame hanging off one.
@pytest.mark.parametrize(
    ("element", "may_be_off_chain", "rejection"),
    [
        (Frame("elbow", uri="https://example.test/other/elbow"), False, "not on the chain"),
        (Frame("wrist_ft_offset", uri=OFFSET_SITE), False, "no segment of 'chain' stands for"),
        (
            SimplicialComplex("wrist_ft_offset", uri=OFFSET_SITE),
            False,
            "no segment of 'chain' stands for",
        ),
        (Frame("nowhere", uri="https://example.test/nowhere"), True, "absent from every tree"),
    ],
    ids=["off-chain", "offset-frame", "offset-body", "on-no-tree"],
)
def test_a_frame_the_chain_cannot_index_fails_while_generating(
    element, may_be_off_chain: bool, rejection: str
) -> None:
    with pytest.raises(ConstraintViolation, match=rejection):
        place_on_chain(FT_CHAIN, element, "solver", may_be_off_chain, {})


# An output names the joint runtime-scoped while the chain stores it bare; and nothing at run
# time can add a torque to a joint the chain has no slot for.
@pytest.mark.parametrize(
    ("prefix", "joint_forces", "rejection"),
    [
        ("kinova1_", [], None),
        (
            "",
            [JointForceSpecification("jf", "f_grip", "g_left_driver_joint")],
            "g_left_driver_joint",
        ),
    ],
    ids=["scoped-read", "force-off-chain"],
)
def test_a_joint_resolves_to_its_chain_index_or_fails_while_generating(
    prefix: str, joint_forces: list, rejection: str | None
) -> None:
    solver = SolverWithInputAndOutput(
        id="arm_solver",
        motion_drivers=[
            MotionDrivers(
                id="drivers",
                acceleration_constraint=[],
                cartesian_force=[],
                handler="handler",
                joint_force=joint_forces,
            )
        ],
        output=[JointQuantity("q_wrist", f"{prefix}joint_2", "JointPosition")],
        chain=ChainBinding(
            root="base_link",
            end="tip",
            tip="tip",
            tree="tree",
            namespace="scene",
            name="chain",
            joints=["joint_1", "joint_2", "joint_3"],
        ),
        hardware=HardwareBinding(urdf="arm.urdf", model="arm", tool_body="", tcp_frame=""),
        runtime=RuntimeBinding(id="rt", owner=True, prefix=prefix, owned_trees=[], config_key=""),
        algorithm=SOLVER_FAMILIES[SLV["AccelerationConstrainedHybridDynamicsAlgorithm"]],
    )
    if rejection:
        with pytest.raises(ConstraintViolation, match=rejection):
            index_chain_joints(solver)
    else:
        index_chain_joints(solver)
        assert solver.output[0].joint_index == 1
