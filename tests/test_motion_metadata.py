# SPDX-License-Identifier: MPL-2.0
"""Guarded-motion metadata on the way to C++: the name is required."""

from __future__ import annotations

import pytest


def test_guarded_motion_without_a_name_raises() -> None:
    """A missing schema:name must fail loudly, not render the string 'None' into C++."""
    from rdflib import Graph, Literal, Namespace, URIRef
    from rdflib.namespace import RDF, SDO

    from motion_spec.rdf_parser.ir import Parser
    from motion_spec_dsl.rdf_parser.vocab import MOT

    EX = Namespace("https://example.test/")
    graph = Graph()
    motion = URIRef(EX["motion"])
    graph.add((motion, RDF.type, MOT["GuardedMotion"]))

    with pytest.raises(ValueError, match="schema:name"):
        Parser(graph).guarded_motion(motion)

    graph.add((motion, SDO.name, Literal("approach")))
    parsed = Parser(graph).guarded_motion(motion)
    assert parsed.name == "approach"
    assert parsed.description is None
