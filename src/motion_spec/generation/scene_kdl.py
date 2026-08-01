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
from scene_dsl.rdf_parser.kinematics import JointKind, TreeModel, build_kinematic_model

HEADER_NAME = "scene_kdl.hpp"
NAMESPACE = "scene_kdl"


def _leaf(name: str) -> str:
    """A scene element as MuJoCo knows it: the `.ktree` already carries the attach prefix."""
    return name.rsplit("/", 1)[-1]


def _joints_between(tree: TreeModel, root: str, tip: str) -> list[str]:
    """The joints a chain crosses, root first, named as MuJoCo names them."""
    by_name = {segment.name: segment for segment in tree.segments}
    walk, name = [], tip
    while name != root:
        segment = by_name.get(name)
        if segment is None:
            return []
        if segment.joint is not None and segment.joint.kind is JointKind.REVOLUTE:
            walk.append(_leaf(segment.joint.name))
        name = segment.hook
    return list(reversed(walk))


def chains_by_root(graph: Graph, base_dir: Path | None = None) -> dict[str, tuple[str, list[str]]]:
    """Every declared chain, keyed by the runtime name of the body it starts at.

    A robot assembly is identified downstream by its chain root body. One arm names that
    body bare (`base_link`); two instances of one arm would collide there, so the IR
    prefixes them with the tree (`kinova1_base_link`). Key both spellings, and drop the
    bare one when it is ambiguous rather than let a lookup pick an arm at random.
    """
    found: dict[str, tuple[str, list[str]]] = {}
    ambiguous: set[str] = set()
    try:
        trees = build_kinematic_model(graph, base_dir)
    except ConstraintViolation:
        # A graph that cannot be lowered has no chains to name. Emitting the header is
        # where that is an error and is reported; here it just means there is nothing.
        return found
    for tree in trees:
        for chain in tree.chains:
            entry = (chain.name, _joints_between(tree, chain.root, chain.tip))
            found[chain.root.replace("/", "_")] = entry
            leaf = _leaf(chain.root)
            if leaf in found:
                ambiguous.add(leaf)
            found[leaf] = entry
    for leaf in ambiguous:
        found.pop(leaf, None)
    return found


def _template_dir() -> Path:
    import scene_dsl

    return Path(scene_dsl.__file__).parent / "templates"


def write_scene_kdl_header(
    graph: Graph, output_dir: Path, source: str, base_dir: Path | None = None
) -> Path:
    """Render the scene's KDL builders next to the controller that includes them."""
    env = Environment(loader=FileSystemLoader(_template_dir()), keep_trailing_newline=True)
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / HEADER_NAME
    path.write_text(
        env.get_template("kdl.hpp.jinja2").render(
            {
                "data": {
                    "name": NAMESPACE,
                    "source": source,
                    "trees": build_kinematic_model(graph, base_dir),
                }
            }
        )
    )
    return path
