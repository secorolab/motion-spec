# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

import contextlib

import pytest
from motion_spec_dsl.rdf_parser.vocab import KC_OP, SLV
from rdf_utils.constraints import ConstraintViolation

from motion_spec.classes.bindings import ChainBinding, HardwareBinding, RuntimeBinding
from motion_spec.classes.motion import MotionSolverSlice, MotionUnit
from motion_spec.classes.solvers import MotionDrivers, SolverWithInputAndOutput
from motion_spec.rdf_parser.constraint_handler import SOLVER_FAMILIES
from motion_spec.rdf_parser.runtime import annotate_runtime


@pytest.mark.parametrize(
    ("read_end", "rejection"),
    [("wrist", "drives it with torque"), ("elbow", None)],
    ids=["commanded-runtime", "own-runtime"],
)
def test_a_read_only_slice_on_a_torque_commanded_runtime_is_rejected(read_end, rejection) -> None:
    """Solver `a` torque-drives the chain to the wrist; `b` only reads, to READ_END."""
    chain = [
        SolverWithInputAndOutput(
            id=sid,
            motion_drivers=[
                MotionDrivers(
                    id=f"{sid}_drivers",
                    acceleration_constraint=["c"] if driven else [],
                    cartesian_force=[],
                    handler="handler",
                )
            ],
            output=[],
            chain=ChainBinding(
                root="base", end=end, tip="", tree="", namespace="", name="", joints=[]
            ),
            hardware=HardwareBinding(urdf="arm.urdf", model="arm", tool_body="", tcp_frame=""),
            runtime=RuntimeBinding(id="", owner=False, prefix="", owned_trees=[], config_key=""),
            # A dynamics family is what torque-streams a runtime; forward kinematics only reads it.
            algorithm=SOLVER_FAMILIES[
                SLV["RecursiveNewtonEulerAlgorithm"] if driven else KC_OP.ForwardPositionKinematics
            ],
        )
        for sid, end, driven in (("a", "wrist", True), ("b", read_end, False))
    ]
    motions = [
        MotionUnit(
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
            serial_chain_solvers=[
                MotionSolverSlice(
                    id=sid, solver_id=sid, output=[], motion_driver=None, read_only=read_only
                )
            ],
        )
        for mid, sid, read_only in (("m1", "a", False), ("m2", "b", True))
    ]
    with (
        pytest.raises(ConstraintViolation, match=rejection)
        if rejection
        else contextlib.nullcontext()
    ):
        annotate_runtime(chain, motions, "mj_kdl")
