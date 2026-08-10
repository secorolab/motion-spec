# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Where a frame sits on a solver's chain is resolved while generating, so a frame the chain
never reaches is an error here rather than a dead controller on its first tick."""

from __future__ import annotations

import pytest
from rdf_utils.constraints import ConstraintViolation

from motion_spec.classes.bindings import ChainBinding, HardwareBinding, RuntimeBinding
from motion_spec.classes.dynamics import JointPosition
from motion_spec.classes.geometry import Frame, SimplicialComplex
from motion_spec.classes.solvers import (
    JointForceSpecification,
    MotionDrivers,
    SolverWithInputAndOutput,
)
from motion_spec.rdf_parser.resources import _index_chain_joints, _place_on_chain

SITE = "https://example.test/ft_tree/wrist_ft_body/wrist_ft_site"
BODY = "https://example.test/ft_tree/wrist_ft_body"


def _chain() -> ChainBinding:
    return ChainBinding(
        root="base_link",
        end="tip",
        tip="tip",
        tree="tree",
        name="chain",
        joints=[],
        frames={SITE: {"index": 8, "offset": None}},
        bodies={BODY: 8},
    )


def test_a_frame_that_is_no_segment_resolves_through_the_body_carrying_it() -> None:
    frame = Frame("wrist_ft_site", uri=SITE)
    _place_on_chain(_chain(), frame, "solver")
    assert frame.segment == 8


def test_a_body_resolves_to_the_segment_standing_for_it() -> None:
    body = SimplicialComplex("wrist_ft_body", uri=BODY)
    _place_on_chain(_chain(), body, "solver")
    assert body.segment == 8


def test_a_frame_the_chain_never_reaches_fails_while_generating() -> None:
    with pytest.raises(ConstraintViolation, match="not on the chain"):
        _place_on_chain(_chain(), Frame("elbow", uri="https://example.test/other/elbow"), "solver")


def _solver(prefix: str, joint_forces=()) -> SolverWithInputAndOutput:
    return SolverWithInputAndOutput(
        id="arm_solver",
        motion_drivers=[
            MotionDrivers(
                id="drivers",
                acceleration_constraint=[],
                cartesian_force=[],
                joint_force=list(joint_forces),
            )
        ],
        output=[JointPosition("q_wrist", f"{prefix}joint_2")],
        chain=ChainBinding(
            root="base_link",
            end="tip",
            tip="tip",
            tree="tree",
            name="chain",
            joints=["joint_1", "joint_2", "joint_3"],
        ),
        hardware=HardwareBinding(urdf="arm.urdf", model="arm", tool_body="", tcp_frame=""),
        runtime=RuntimeBinding(id="rt", owner=True, prefix=prefix, owned_trees=[], config_key=""),
    )


def test_a_joint_read_resolves_to_its_index_however_the_name_is_scoped() -> None:
    # An output names the joint runtime-scoped; the chain stores it bare.
    solver = _solver("kinova1_")
    _index_chain_joints(solver)
    assert solver.output[0].joint_index == 1


def test_a_joint_force_names_the_joint_bare_and_still_resolves() -> None:
    solver = _solver("kinova1_", [JointForceSpecification("jf", "f_elbow", "joint_3")])
    _index_chain_joints(solver)
    assert solver.motion_drivers[0].joint_force[0].joint_index == 2


def test_a_joint_force_off_the_chain_fails_while_generating() -> None:
    # Nothing at run time can add a torque to a joint the chain has no slot for.
    solver = _solver("", [JointForceSpecification("jf", "f_grip", "g_left_driver_joint")])
    with pytest.raises(ConstraintViolation, match="g_left_driver_joint"):
        _index_chain_joints(solver)


def test_a_joint_the_chain_does_not_articulate_reads_as_no_index() -> None:
    # A gripper mimic: only a simulated backend can answer for it, so it carries no chain index
    # rather than failing the whole model.
    solver = _solver("")
    solver.output = [JointPosition("gripper_pos", "g_left_driver_joint")]
    _index_chain_joints(solver)
    assert solver.output[0].joint_index is None
