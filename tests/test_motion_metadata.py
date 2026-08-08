# SPDX-License-Identifier: MPL-2.0
"""Guarded-motion metadata on the way to C++: the name is required."""

from __future__ import annotations

import pytest


def test_guarded_motion_without_a_name_raises() -> None:
    """A missing schema:name must fail loudly, not render the string 'None' into C++."""
    from pathlib import Path

    from motion_spec_dsl.rdf_parser.vocab import MOT
    from rdf_utils.constraints import ConstraintViolation
    from rdflib import Dataset, Literal, Namespace, URIRef
    from rdflib.namespace import RDF, SDO

    from motion_spec.rdf_parser.coordination import guarded_motion
    from motion_spec.rdf_parser.model import Model

    EX = Namespace("https://example.test/")
    graph = Dataset(default_union=True)
    motion = URIRef(EX["motion"])
    graph.add((motion, RDF.type, MOT["GuardedMotion"]))
    model = Model(
        graph=graph, app_path=Path("model-app.ld.json"), imported_models=[], imported_provenance=[]
    )

    with pytest.raises(ConstraintViolation, match="schema:name"):
        guarded_motion(model, motion)

    graph.add((motion, SDO.name, Literal("approach")))
    parsed = guarded_motion(model, motion)
    assert parsed.name == "approach"
    assert parsed.description is None
