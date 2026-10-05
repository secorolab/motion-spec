# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Lowering a standing subscription: which poses it writes, against which frame, and that it is
their one writer. The model cases lower the `perception` fixture, where subscription -> written
pose -> the chain that no longer computes it can be read at once."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from rdf_utils.constraints import ConstraintViolation
from rdflib import URIRef
from rdflib.namespace import split_uri
from support import REQUIRES_ROS

from motion_spec.generation.pipeline import generate_model
from motion_spec.rdf_parser import communication

MODEL = Path(__file__).parent / "fixtures" / "perception"


@pytest.fixture(scope="module")
def subscription_ir(tmp_path_factory) -> dict:
    generation = tmp_path_factory.mktemp("subscription_generation")
    generated = generate_model(MODEL / "perception.robmot", generation, stage="ir")
    return json.loads((generated / "model" / "ir.json").read_text())


@REQUIRES_ROS
def test_the_subscription_writes_its_poses_against_a_world_model_segment(subscription_ir):
    """The frame a detection arrives in is read off its header at run time; the frame the pose is
    stated against is the model's, handed over as the segment the runtime composes into."""
    (subscription,) = subscription_ir["communication"]["ros"]["subscriptions"]
    assert subscription["channel"] == "/recognized_objects"
    assert subscription["cpp_type"] == "vision_msgs::msg::Detection3DArray"
    written = subscription["written_poses"]
    assert {row["pose_id"] for row in written} == {"shared_world_pose_table_cam"}
    assert {row["frame_id"] for row in written} == {"wrist_ft_site"}
    assert {row["frame_segment"] for row in written} == {"ft_tree/wrist_ft_body/wrist_ft_site"}
    # The pose is written onto the frame the model says the detection is of.
    assert {split_uri(URIRef(row["target_iri"]))[1] for row in written} == {"table_top"}


def test_a_frame_the_tree_has_no_segment_for_is_rejected():
    """A detection composed against a frame the world model does not hold would land wherever the
    identity puts it."""
    with pytest.raises(ConstraintViolation, match="no segment standing for it"):
        communication._segment_of({}, "https://example.test/camera", "object-poses")


@REQUIRES_ROS
def test_no_chain_computes_a_pose_the_subscription_writes(subscription_ir):
    """One writer: the topic writes it, so the per-tick scene-object sync must not."""
    outputs = {
        out["id"]
        for solver in subscription_ir["resources"]["by_kind"]["serial_chain"]
        for out in solver["output"]
    }
    assert "shared_world_pose_table_cam" not in outputs
    data_access = subscription_ir["computation"]["data_access"]
    assert data_access["shared_world_pose_table_cam"]["write"] == {
        "kind": "subscription",
        "id": "ros_subscribers_table_top",
    }


@REQUIRES_ROS
def test_the_subscription_stamps_its_pose_and_a_freshness_gate_holds_in_state(subscription_ir):
    """The channel that writes the pose dates it (minus infinity until the first message), and a
    motion gated on that age holds inside its own state while the idle hold steps."""
    (subscription,) = subscription_ir["communication"]["ros"]["subscriptions"]
    assert {row["observed_at_id"] for row in subscription["written_poses"]} == {
        "pose_table_cam_observed"
    }
    members = {member["id"]: member for member in subscription_ir["computation"]["data"]}
    assert members["pose_table_cam_observed"]["unset"] is True
    data_access = subscription_ir["computation"]["data_access"]
    assert data_access["pose_table_cam_observed"]["write"] == {
        "kind": "subscription",
        "id": "ros_subscribers_table_top",
    }
    motions = {motion["id"]: motion for motion in subscription_ir["coordination"]["motions"]}
    watch = motions["handler_watch"]
    assert watch["has_when_gate"] is True
    assert watch["when_gate_hold"]["id"] == "handler_hold_idle"
    assert watch["when_observation_ages"] == [
        {"coordinate": "table_seen_elapsed", "observed_at": "pose_table_cam_observed"}
    ]
