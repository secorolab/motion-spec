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
from rdflib import Graph
from scene_dsl.kdl_tree import build_kdl_trees

NAMESPACE = "scene_kdl"


def kdl_header_name(source: str | Path) -> str:
    """The controller-local KDL header named after its source motion model."""
    name = Path(source).name
    stem = name[: -len("-app.ld.json")] if name.endswith("-app.ld.json") else Path(name).stem
    return f"{stem}.kdl.hpp"


def chain_for_iri(trees: list[dict], chain_iri: str) -> tuple[str, str, list[str]]:
    """The generated C++ builders and MuJoCo joints for one declared serial chain."""
    for tree in trees:
        for chain in tree["chains"]:
            if chain["iri"] == chain_iri:
                return (
                    chain["cpp_name"],
                    tree["cpp_name"],
                    [joint["local_name"] for joint in chain["joints"]],
                )
    return "", "", []


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
