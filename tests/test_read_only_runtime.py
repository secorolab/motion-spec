# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

import pytest

from motion_spec.rdf_parser.solvers import _annotate_runtime_robots


def solver(sid, chain_end, driven):
    return {
        "id": sid,
        "chain_root": "base",
        "chain_end": chain_end,
        "robot_model": "arm",
        "urdf": "arm.urdf",
        "tool_body": None,
        "tcp_site": None,
        "runtime_id": "",
        "runtime_owner": False,
        "motion_drivers": [{"acceleration_constraint": ["c"] if driven else []}],
    }


def motion(mid, solvers):
    return {"id": mid, "serial_chain_solvers": solvers, "forwarded_commands": []}


def test_read_only_on_commanded_runtime_is_rejected() -> None:
    chain = [solver("a", "wrist", driven=True), solver("b", "wrist", driven=False)]
    motions = [
        motion("m1", [{"id": "a", "read_only": False}]),
        motion("m2", [{"id": "b", "read_only": True}]),
    ]
    with pytest.raises(ValueError, match="torque-commanded"):
        _annotate_runtime_robots(chain, motions, "mj_kdl")


def test_read_only_on_own_runtime_is_allowed() -> None:
    chain = [solver("a", "wrist", driven=True), solver("b", "elbow", driven=False)]
    motions = [
        motion("m1", [{"id": "a", "read_only": False}]),
        motion("m2", [{"id": "b", "read_only": True}]),
    ]
    _annotate_runtime_robots(chain, motions, "mj_kdl")
