# SPDX-License-Identifier: MPL-2.0
"""Guarded-motion metadata on the way to C++: the name is required, the description is
escaped into a line comment."""

from __future__ import annotations

import pytest

from motion_spec.generation.codegen import _escape_for_line_comment


ESCAPES = [
    pytest.param('say "hello"', 'say "hello"', id="quotes_survive"),
    pytest.param("first\nsecond", "first\\nsecond", id="newline_escaped"),
    pytest.param("first\r\nsecond", "first\\nsecond", id="crlf_escaped"),
    pytest.param("Grüße — 90° ✔", "Grüße — 90° ✔", id="unicode_survives"),
    pytest.param("end */ then code", "end *\\/ then code", id="comment_terminator_escaped"),
]


@pytest.mark.parametrize(("text", "expected"), ESCAPES)
def test_description_cannot_break_out_of_a_line_comment(text: str, expected: str) -> None:
    escaped = _escape_for_line_comment(text)
    assert escaped == expected
    assert "\n" not in escaped
    assert "*/" not in escaped


def test_guarded_motion_without_a_name_raises() -> None:
    """A missing schema:name must fail loudly, not render the string 'None' into C++."""
    from rdflib import Graph, Literal, Namespace, URIRef
    from rdflib.namespace import RDF, SDO

    from motion_spec.rdf_parser.ir import Parser
    from motion_spec.rdf_parser.vocab import MOT

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
