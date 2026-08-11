# SPDX-License-Identifier: MPL-2.0
"""The solver chain, taken from the scene model rather than re-derived from the MJCF.

`scene-dsl` lowers the scene graph into KDL segments and renders them as C++ (plan 012);
this module is the seam where `motion-spec` picks that up: it names the chain each robot
assembly should build, lists that chain's joints as MuJoCo knows them, and writes the
header the generated controller includes. See `plans/013-kdl-chain-from-scenex.md`.
"""

from __future__ import annotations

from pathlib import Path

from jinja2 import Environment, FileSystemLoader
from rdf_utils.constraints import ConstraintViolation
from rdflib import Graph
from scene_dsl.kdl_tree import build_kdl_trees

NAMESPACE = "scene_kdl"


def model_stem(source: str | Path) -> str:
    """The motion model's own name, stripped of the manifest suffix it arrives with."""
    name = Path(source).name
    return name[: -len("-app.ld.json")] if name.endswith("-app.ld.json") else Path(name).stem


def kdl_header_name(source: str | Path) -> str:
    """The controller-local KDL header named after its source motion model."""
    return f"{model_stem(source)}.kdl.hpp"


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


def chain_for_iri(trees: list[dict], chain_iri: str) -> dict:
    """The generated C++ builders, MuJoCo joints and segment lookups for one declared chain."""
    for tree in trees:
        for chain in tree["chains"]:
            if chain["iri"] == chain_iri:
                return {
                    "name": chain["cpp_name"],
                    "tree": tree["cpp_name"],
                    "joints": [joint["local_name"] for joint in chain["joints"]],
                    "joint_segments": _joint_segments(tree, chain),
                    "frames": chain["frames"],
                    "bodies": chain["bodies"],
                    "tip_segment": chain["tip_index"],
                    "world_root": chain["root"],
                    "world_tip": chain["tip"],
                    "world_segments": _world_segments(tree, chain),
                }
    return {
        "name": "",
        "tree": "",
        "joints": [],
        "joint_segments": [],
        "frames": {},
        "bodies": {},
        "tip_segment": 0,
        "world_root": "",
        "world_tip": "",
        "world_segments": {},
    }


def _template_dir() -> Path:
    import scene_dsl

    return Path(scene_dsl.__file__).parent / "templates"


def write_scene_kdl_header(
    graph: Graph, output_dir: Path, source: str, base_dir: Path | None = None
) -> Path:
    """Render the scene's KDL builders next to the controller that includes them."""
    env = Environment(loader=FileSystemLoader(_template_dir()), keep_trailing_newline=True)
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / kdl_header_name(source)
    path.write_text(
        env.get_template("kdl.hpp.jinja2").render(
            {
                "data": {
                    "name": NAMESPACE,
                    "source": source,
                    "trees": build_kdl_trees(graph, base_dir),
                }
            }
        )
    )
    return path
