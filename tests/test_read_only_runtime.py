# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

import pytest
from rdf_utils.constraints import ConstraintViolation

from motion_spec.classes.bindings import ChainBinding, HardwareBinding, RuntimeBinding
from motion_spec.classes.motion import MotionSolverSlice, MotionUnit
from motion_spec.classes.solvers import MotionDrivers, SolverWithInputAndOutput
from motion_spec.rdf_parser.resources import annotate_runtime


def solver(sid, chain_end, driven):
    return SolverWithInputAndOutput(
        id=sid,
        motion_drivers=[
            MotionDrivers(
                id=f"{sid}_drivers",
                acceleration_constraint=["c"] if driven else [],
                cartesian_force=[],
            )
        ],
        output=[],
        chain=ChainBinding(root="base", end=chain_end, tip="", tree="", name="", joints=[]),
        hardware=HardwareBinding(urdf="arm.urdf", model="arm", tool_body="", tcp_frame=""),
        runtime=RuntimeBinding(id="", owner=False, prefix="", owned_trees=[], config_key=""),
    )


def slice_(sid, read_only):
    return MotionSolverSlice(
        id=sid, solver_id=sid, output=[], motion_driver=None, read_only=read_only
    )


def motion(mid, solvers):
    return MotionUnit(
        id=mid,
        motion_id="",
        name=mid,
        description=[],
        when_evaluators=[],
        while_evaluators=[],
        until_evaluators=[],
        controllers=[],
        when_monitors=[],
        while_monitors=[],
        until_monitors=[],
        when_schedule=[],
        while_schedule=[],
        until_schedule=[],
        serial_chain_solvers=solvers,
    )


def test_read_only_on_commanded_runtime_is_rejected() -> None:
    chain = [solver("a", "wrist", driven=True), solver("b", "wrist", driven=False)]
    motions = [
        motion("m1", [slice_("a", read_only=False)]),
        motion("m2", [slice_("b", read_only=True)]),
    ]
    with pytest.raises(ConstraintViolation, match="torque-commanded"):
        annotate_runtime(chain, motions, "mj_kdl")


def test_read_only_on_own_runtime_is_allowed() -> None:
    chain = [solver("a", "wrist", driven=True), solver("b", "elbow", driven=False)]
    motions = [
        motion("m1", [slice_("a", read_only=False)]),
        motion("m2", [slice_("b", read_only=True)]),
    ]
    annotate_runtime(chain, motions, "mj_kdl")
