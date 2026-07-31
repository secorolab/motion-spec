# SPDX-License-Identifier: MPL-2.0
"""IR parsing must reject mobile-platform algorithms that no motion.stg/hddc2b.stg template
implements yet, rather than silently dropping them (StringTemplate renders a missing attribute
as empty with no error)."""

from __future__ import annotations

import pytest
from rdflib import Graph, URIRef
from rdflib.namespace import RDF

from motion_spec.rdf_parser.ir import Parser, _solver_derivation_context, _solver_sections
from motion_spec.rdf_parser.vocab import SLV_EXT

NS = "https://example.test/"


@pytest.mark.parametrize(
    ("rdf_type", "label"),
    [
        (SLV_EXT.VelocityDistributionSolver, "velocity-distribution"),
        (SLV_EXT.ForceCompositionSolver, "force-composition"),
    ],
)
def test_unimplemented_mobile_platform_algorithm_is_rejected(rdf_type, label) -> None:
    g = Graph()
    node = URIRef(f"{NS}bad-solver")
    g.add((node, RDF.type, rdf_type))
    p = Parser(g)
    derivation = _solver_derivation_context(g)

    with pytest.raises(ValueError, match=label):
        _solver_sections(g, p, {}, None, derivation, None)
