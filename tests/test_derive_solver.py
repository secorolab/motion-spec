# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

from motion_spec.derive_solver import ANGULAR_AXES, LINEAR_AXES, POSE_AXES, acceleration_axes


def derive(subspace=None, axis=None, **kwargs):
    return acceleration_axes(
        controller_type=kwargs.get("controller_type", "ProportionalIntegralDerivative"),
        subspace=subspace,
        axis=axis,
        command_type=kwargs.get("command_type"),
        relation=kwargs.get("relation", "EqualityConstraint"),
        quantity_kind=kwargs.get("quantity_kind"),
    )


def test_acceleration_axis_decision_table() -> None:
    assert derive("pose", quantity_kind="Pose") == POSE_AXES
    assert derive("position", "x") == LINEAR_AXES[:1]
    assert derive("position") == LINEAR_AXES
    assert derive("orientation", "z") == ANGULAR_AXES[2:]
    assert derive("orientation") == ANGULAR_AXES
    assert derive("distance", "y") == LINEAR_AXES[1:2]
    assert derive("rotation", "x") == ANGULAR_AXES[:1]
    assert derive("distance") == (type(LINEAR_AXES[0])("linear-acceleration", "distance"),)
    assert derive("force", command_type="Force") == ()
    assert derive(None, command_type="Torque", quantity_kind="JointPosition") == ()
    assert derive("position", controller_type="ImpedanceController") == ()
