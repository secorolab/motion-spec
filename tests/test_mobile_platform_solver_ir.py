# SPDX-License-Identifier: MPL-2.0
"""IR parsing must reject mobile-platform algorithms that no motion.stg/hddc2b.stg template
implements yet, rather than silently dropping them (StringTemplate renders a missing attribute
as empty with no error)."""

from __future__ import annotations

from pathlib import Path

import pytest
from motion_spec_dsl.rdf_parser.vocab import SLV_EXT
from rdf_utils.constraints import ConstraintViolation
from rdflib import Dataset, URIRef
from rdflib.namespace import RDF

from motion_spec.rdf_parser import constraint_handler, operations, resources
from motion_spec.rdf_parser.model import Model

NS = "https://example.test/"


@pytest.mark.parametrize(
    ("rdf_type", "label"),
    [
        (SLV_EXT.VelocityDistributionSolver, "velocity-distribution"),
        (SLV_EXT.ForceCompositionSolver, "force-composition"),
    ],
)
def test_unimplemented_mobile_platform_algorithm_is_rejected(rdf_type, label) -> None:
    graph = Dataset(default_union=True)
    graph.add((URIRef(f"{NS}bad-solver"), RDF.type, rdf_type))
    model = Model(
        graph=graph, app_path=Path("model-app.ld.json"), imported_models=[], imported_provenance=[]
    )
    derivation = constraint_handler.solver_derivation_context(model)

    with pytest.raises(ConstraintViolation, match=label):
        resources.build_robots(model, operations.Schedule(model), {}, derivation, [], "mj_kdl")
