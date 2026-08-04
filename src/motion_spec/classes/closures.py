# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Semantic roles of values produced by compiler IR closure records."""

from __future__ import annotations


_OUTPUT_FIELDS = {
    "AssignmentEvaluator": ("quantity",),
    "ErrorEvaluator": ("error",),
    "PoseDiffEvaluator": ("out",),
    "PathProjection": ("path_parameter",),
    "PathTangentFrame": ("tangent", "normal_a", "normal_b"),
    "TwistToLinearVelocityAlong": ("along_speed",),
    "PathEvaluator": ("out", "setpoint"),
    "VelocityProfile": ("out",),
    "Admittance": ("out",),
    "Controller": ("control_signal",),
    "PlanarAngleFromDirections": ("angle",),
    "InvertAngle": ("out",),
    "RotateDirectionDistalToProximalWithPose": ("to",),
    "ComposePose": ("composite",),
    "InvertPose": ("out",),
    "PoseToLinearDistance": ("distance",),
    "PoseToDirection": ("direction",),
    "PoseToAngleAroundAxis": ("angle",),
    "RotateVelocityTwistToProximalWithPose": ("to",),
    "WrenchFromPositionDirectionAndMagnitude": ("wrench",),
    "RotateWrenchToDistalWithPose": ("to",),
    "RotateWrenchToProximalWithPose": ("to",),
    "TransformWrenchToProximal": ("to",),
    "Addition": ("out",),
    "AddWrench": ("out",),
}


def closure_output_ids(closure: dict) -> set[str]:
    """Return data IDs written by a generated closure call."""
    outputs = {
        value
        for field in _OUTPUT_FIELDS.get(closure.get("type"), ())
        if isinstance((value := closure.get(field)), str)
    }
    outputs.update(
        sample["id"]
        for sample in closure.get("internal_state_samples", ())
        if isinstance(sample, dict) and isinstance(sample.get("id"), str)
    )
    if closure.get("assign_goal") and isinstance(closure.get("goal"), str):
        outputs.add(closure["goal"])
    return outputs
