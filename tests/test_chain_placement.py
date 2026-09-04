# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Where a frame sits on a solver's chain is resolved while generating, so a frame the chain
never reaches is an error here rather than a dead controller on its first tick."""

from __future__ import annotations

import pytest
from rdf_utils.constraints import ConstraintViolation

from motion_spec.classes.bindings import ChainBinding, HardwareBinding, RuntimeBinding
from motion_spec.classes.dynamics import JointPosition
from motion_spec.classes.geometry import (
    Frame,
    Pose,
    SimplicialComplex,
    Subspace,
    VelocityTwist,
    Wrench,
)
from motion_spec.classes.solvers import (
    AccelerationConstraint,
    CartesianForceSpecification,
    JointForceSpecification,
    MotionDrivers,
    SolverWithInputAndOutput,
)
from motion_spec.rdf_parser.resources import _index_chain_joints, _place_on_chain, _placed_on_chain

SITE = "https://example.test/ft_tree/wrist_ft_body/wrist_ft_site"
BODY = "https://example.test/ft_tree/wrist_ft_body"
OFFSET_SITE = "https://example.test/ft_tree/wrist_ft_body/wrist_ft_offset"
SEGMENT = "wrist_ft_body/wrist_ft_site"
OFFSET_SEGMENT = "wrist_ft_body/wrist_ft_offset"


def _chain() -> ChainBinding:
    return ChainBinding(
        root="base_link",
        end="tip",
        tip="tip",
        tree="tree",
        name="chain",
        joints=[],
        frames={
            SITE: {"index": 8, "offset": None},
            OFFSET_SITE: {"index": 8, "offset": {"x": 0.1}},
        },
        bodies={BODY: 8},
        world_segments={SITE: SEGMENT, OFFSET_SITE: OFFSET_SEGMENT, BODY: "wrist_ft_body"},
    )


def test_a_frame_that_is_no_segment_resolves_through_the_body_carrying_it() -> None:
    frame = Frame("wrist_ft_site", uri=SITE)
    _place_on_chain(_chain(), frame, "solver", False)
    assert frame.segment == 8


def test_a_body_resolves_to_the_segment_standing_for_it() -> None:
    body = SimplicialComplex("wrist_ft_body", uri=BODY)
    _place_on_chain(_chain(), body, "solver", False)
    assert body.segment == 8


def test_a_frame_the_chain_never_reaches_fails_while_generating() -> None:
    with pytest.raises(ConstraintViolation, match="not on the chain"):
        _place_on_chain(
            _chain(), Frame("elbow", uri="https://example.test/other/elbow"), "solver", False
        )


def test_an_offset_frame_a_world_read_asks_for_resolves_to_its_own_leaf_segment() -> None:
    # Plan 04 gives it a segment of its own, so the offset is composed in the tree, not at run time.
    frame = Frame("wrist_ft_offset", uri=OFFSET_SITE)
    assert _place_on_chain(_chain(), frame, "solver", True) == OFFSET_SEGMENT


def test_the_same_offset_still_fails_where_the_read_stays_chain_relative() -> None:
    # `f_ext[index - 1]` and the velocity solver are indexed by the chain, which has no segment
    # standing for a frame that only hangs off one.
    with pytest.raises(ConstraintViolation, match="no segment of 'chain' stands for"):
        _place_on_chain(_chain(), Frame("wrist_ft_offset", uri=OFFSET_SITE), "solver", False)
    with pytest.raises(ConstraintViolation, match="no segment of 'chain' stands for"):
        _place_on_chain(
            _chain(), SimplicialComplex("wrist_ft_offset", uri=OFFSET_SITE), "solver", False
        )


def _spatial(cls, **extra):
    return cls(id="q", quantity_kind=[], reference_point=None, as_seen_by=None, unit=[], **extra)


def test_which_reads_move_to_the_world_model_is_decided_once() -> None:
    pose = Pose(
        "pose_ee",
        of=None,
        with_respect_to=None,
        quantity_kind=[],
        as_seen_by=None,
        unit=[],
        position=None,
    )
    assert _placed_on_chain(pose, "mj_kdl") == (("of", True),)
    # A twist states the point it is taken about, which stays on chain FK; the frame it asked
    # to be seen in is a posed frame like any other, so that one reads the world model.
    twist = _spatial(VelocityTwist, of=None, with_respect_to=None)
    assert _placed_on_chain(twist, "mj_kdl") == (("of", False), ("as_seen_by", True))
    force = CartesianForceSpecification("f", force=None, attached_to=None)
    assert _placed_on_chain(force, "robif2b") == (("attached_to", False),)
    constraint = AccelerationConstraint("c", subspace=Subspace.Linear, axis=None)
    assert _placed_on_chain(constraint, "mj_kdl") == (("as_seen_by", True),)
    # The simulator answers a wrench's transform frames from its own scene, by name; only the
    # sensor frame is placed, because the load hanging off it is walked from the world model.
    wrench = _spatial(Wrench, sensor_frame=None)
    assert _placed_on_chain(wrench, "mj_kdl") == (("sensor_frame", True),)
    assert [attribute for attribute, _ in _placed_on_chain(wrench, "robif2b")] == [
        "sensor_frame",
        "reference_point",
        "as_seen_by",
    ]


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
