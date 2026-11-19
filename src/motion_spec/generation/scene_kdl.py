# SPDX-License-Identifier: MPL-2.0
"""The solver chain each robot assembly builds, taken from scene-dsl's KDL trees."""

from __future__ import annotations

from rdf_utils.constraints import ConstraintViolation


def segment_of_joint(trees: list[dict], joint_uri: str) -> str:
    """The tree segment the given scene joint moves, or "" when no tree carries it.

    Matched by identity, never by name: two grippers on two arms declare the same local joint
    name, and a suffix match would place both on whichever segment came first.
    """
    if not joint_uri:
        return ""
    for tree in trees:
        for segment in tree["segments"]:
            joint = segment["joint"]
            if joint is not None and joint["iri"] == joint_uri:
                return segment["name"]

    return ""


def chain_for_iri(trees: list[dict], chain_iri: str) -> dict:
    """The generated C++ builders, MuJoCo joints and segment lookups for one declared chain."""
    for tree in trees:
        for chain in tree["chains"]:
            if chain["iri"] == chain_iri:
                # the world model binds measurements to segments, not joints
                segment_by_joint = {
                    segment["joint"]["name"]: segment["name"]
                    for segment in tree["segments"]
                    if segment["joint"] is not None
                }
                joint_segments = []
                for joint in chain["joints"]:
                    if joint["name"] not in segment_by_joint:
                        raise ConstraintViolation(
                            "kinematics",
                            f"chain '{chain['name']}' articulates joint '{joint['name']}', which "
                            f"no segment of tree '{tree['name']}' carries",
                        )
                    joint_segments.append(segment_by_joint[joint["name"]])
                # a body's root frame has no segment: resolve it through its chain placement
                world_segments = {
                    tree["root_iri"]: tree["root"],
                    **{segment["iri"]: segment["name"] for segment in tree["segments"]},
                }
                name_by_index = {
                    0: chain["root"],
                    **{index: world_segments[body] for body, index in chain["bodies"].items()},
                }
                for iri, placement in chain["frames"].items():
                    if iri not in world_segments and placement["index"] in name_by_index:
                        world_segments[iri] = name_by_index[placement["index"]]
                return {
                    "name": chain["cpp_name"],
                    "tree": tree["cpp_name"],
                    "namespace": tree["namespace"],
                    "joints": [joint["local_name"] for joint in chain["joints"]],
                    "joint_segments": joint_segments,
                    "frames": chain["frames"],
                    "bodies": chain["bodies"],
                    "tip_segment": chain["tip_index"],
                    "world_root": chain["root"],
                    "world_tip": chain["tip"],
                    "tree_root": tree["root"],
                    "world_segments": world_segments,
                }
    return {
        "name": "",
        "tree": "",
        "namespace": "",
        "joints": [],
        "joint_segments": [],
        "frames": {},
        "bodies": {},
        "tip_segment": 0,
        "world_root": "",
        "world_tip": "",
        "tree_root": "",
        "world_segments": {},
    }
