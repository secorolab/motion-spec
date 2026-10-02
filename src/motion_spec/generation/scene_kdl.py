# SPDX-License-Identifier: MPL-2.0
"""The solver chain, taken from the scene model rather than re-derived from the MJCF.

`scene-dsl` lowers the scene graph into KDL segments and renders them as C++ (plan 012);
this module is the seam where `motion-spec` picks that up: it names the chain each robot
assembly should build and lists that chain's joints as MuJoCo knows them.
See `plans/013-kdl-chain-from-scenex.md`.
"""

from __future__ import annotations

from rdf_utils.constraints import ConstraintViolation


def _joint_segments(tree: dict, chain: dict) -> list[str]:
    """The tree segment each chain joint moves, in chain-joint order.

    The world model binds a measurement to a segment, not to a joint, so the chain's joint order
    has to be carried over to the names the built tree knows. A joint with no segment would bind
    the wrong state, and only after startup, so it is an error here.
    """
    segment_by_joint = {
        segment["joint"]["name"]: segment["name"]
        for segment in tree["segments"]
        if segment["joint"] is not None
    }
    segments = []
    for joint in chain["joints"]:
        if joint["name"] not in segment_by_joint:
            raise ConstraintViolation(
                "kinematics",
                f"chain '{chain['name']}' articulates joint '{joint['name']}', which no segment "
                f"of tree '{tree['name']}' carries",
            )
        segments.append(segment_by_joint[joint["name"]])

    return segments


def _world_segments(tree: dict, chain: dict) -> dict[str, str]:
    """Every scene element this chain reaches, by IRI, named as the built tree names it.

    Plan 04 gives every posed frame its own KDL leaf, so a frame the chain slice can only reach
    as "a parent plus a constant offset" is an exact segment of the tree. A body's own root frame
    is where the body's segment already is, and carries no segment of its own -- it resolves
    through the body the chain placement points at.
    """
    by_iri = {tree["root_iri"]: tree["root"]}
    by_iri.update({segment["iri"]: segment["name"] for segment in tree["segments"]})
    name_by_index = {0: chain["root"]}
    name_by_index.update({index: by_iri[body] for body, index in chain["bodies"].items()})
    for iri, placement in chain["frames"].items():
        if iri not in by_iri and placement["index"] in name_by_index:
            by_iri[iri] = name_by_index[placement["index"]]

    return by_iri


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
                return {
                    "name": chain["cpp_name"],
                    "tree": tree["cpp_name"],
                    "namespace": tree["namespace"],
                    "joints": [joint["local_name"] for joint in chain["joints"]],
                    "joint_segments": _joint_segments(tree, chain),
                    "frames": chain["frames"],
                    "bodies": chain["bodies"],
                    "tip_segment": chain["tip_index"],
                    "world_root": chain["root"],
                    "world_tip": chain["tip"],
                    "tree_root": tree["root"],
                    "world_segments": _world_segments(tree, chain),
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
