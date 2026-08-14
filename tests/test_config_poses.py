# SPDX-License-Identifier: MPL-2.0
"""A pose read from robot.toml. The section is named after the pose, and what the config must
state for it is checked here rather than left to fail as a zero pose at runtime."""

from __future__ import annotations

import pytest
from motion_spec_dsl.rdf_parser.vocab import EXEC
from rdf_utils.constraints import ConstraintViolation
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import RDF, SDO

from motion_spec.rdf_parser.model import local_name
from motion_spec.rdf_parser.resources import config_poses

APP = "https://secorolab.github.io/models/demo/"
POSE = URIRef(f"{APP}shared/spec/look-at-table")
CONTEXT = URIRef(f"{APP}demo-exec")
RESOURCE = URIRef(f"{APP}demo-exec.config")


class _Model:
    """The two things `config_poses` asks a model for."""

    def __init__(self) -> None:
        self.graph = Graph()
        self.graph.add((CONTEXT, RDF.type, EXEC.ExecutionContext))
        self.graph.add((CONTEXT, EXEC["has-resource"], RESOURCE))
        self.graph.add((RESOURCE, EXEC.path, Literal("/somewhere/robot.toml")))
        self.graph.add((POSE, EXEC["has-resource"], RESOURCE))
        self.graph.add((POSE, SDO.identifier, Literal("poses.table")))

    @staticmethod
    def id(node) -> str:
        return local_name(node).replace("-", "_")


_STATED = {"poses": {"table": {"position": [0.4, 0.24, 0.22], "orientation": [3.14, 0, 1.5]}}}


def test_the_authored_key_is_the_section_it_reads() -> None:
    """Not derived from the pose name: `[config.poses.table]` on a pose called look-at-table
    means poses.table, and nothing quietly renames it."""
    assert config_poses(_Model(), _STATED) == [{"id": "look_at_table", "config_key": "poses.table"}]


def test_the_exec_context_is_not_itself_a_config_pose() -> None:
    """It carries the same edge -- it is what declares the file in the first place."""
    entries = config_poses(_Model(), _STATED)

    assert [entry["id"] for entry in entries] == ["look_at_table"]


@pytest.mark.parametrize(
    ("config", "message"),
    [
        ({}, "does not state"),
        (
            {"poses": {"look-at-table": {"position": [0, 0, 0], "orientation": [0, 0, 0]}}},
            "does not state",
        ),
        ({"poses": {"table": {"position": [0.4, 0.24, 0.22]}}}, "orientation"),
        ({"poses": {"table": {"position": [0.4, 0.24], "orientation": [0, 0, 0]}}}, "position"),
    ],
)
def test_a_config_that_does_not_state_the_pose_is_rejected(config: dict, message: str) -> None:
    with pytest.raises(ConstraintViolation, match=message):
        config_poses(_Model(), config)
