# SPDX-License-Identifier: MPL-2.0
"""SPARQL over a run's model dataset.

The DSL's document, found through the generation's provenance, with the scene and FSM graphs
beside it, each document its own named graph.
The dashboard mints no vocabulary of its own.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import rdflib
from motion_spec_dsl.rdf_parser.vocab import EXEC
from rdf_utils.models.prov import get_file_path
from rdflib.namespace import PROV, SDO

from motion_spec.runs.provenance import (
    GENERATION_DOCUMENT,
    kept_dsl_document,
    read_generation_dataset,
)


def load_document(path: Path, dataset: rdflib.Dataset) -> None:
    """Parse one document into DATASET, in a graph named by its file unless it names its own."""
    path = path.resolve()
    dataset.graph(rdflib.URIRef(path.as_uri())).parse(path, format="json-ld")


@lru_cache(maxsize=64)
def deployed_devices(generation_dir: Path | str) -> tuple[str, ...]:
    """The device models the execution context realizes, as the model names them.

    This is what a real platform is made of; a simulation carries a name of its own instead, so
    nothing here has to tell one platform kind from another.
    """
    document = kept_dsl_document(Path(generation_dir) / "generated")
    if document is None:
        return ()
    dataset = rdflib.Dataset(default_union=True)
    load_document(document, dataset)
    return tuple(
        {
            str(dataset.value(device, SDO.model))
            for device in dataset.subjects(EXEC["realizes"], None)
            if dataset.value(device, SDO.model)
        }
    )


class GraphService:
    """One run's queryable model dataset."""

    def __init__(self, generation_dir: Path):
        generated = generation_dir / "generated"
        self.dataset = rdflib.Dataset(default_union=True)
        document = kept_dsl_document(generated)
        if document is not None:
            load_document(document, self.dataset)
        # The scene and FSM graphs the IR names nodes of, which the DSL's document holds none of:
        # the other model documents the provenance attributes.
        provenance = read_generation_dataset(generated / GENERATION_DOCUMENT)
        dsl = next(provenance.subjects(SDO.name, rdflib.Literal("motion_spec_dsl")), None)
        for entity, agent in set(provenance.subject_objects(PROV.wasAttributedTo)):
            if agent != dsl and (entity, PROV.atLocation, None) in provenance:
                load_document(get_file_path(provenance, entity), self.dataset)

    def query(self, sparql: str) -> tuple[str, object]:
        """(result type, payload) for a SPARQL query over the current dataset.

        SELECT and ASK answer with `(variable names, rows)`. CONSTRUCT and DESCRIBE answer with
        their triples: `result.vars` is None for both, so rows alone would flatten them away.
        """
        result = self.dataset.query(sparql)
        if result.type in ("CONSTRUCT", "DESCRIBE"):
            return result.type, [tuple(triple) for triple in result]
        if result.type == "ASK":
            return result.type, (["answer"], [(result.askAnswer,)])
        headers = [str(var) for var in (result.vars or [])]
        return result.type, (headers, [tuple(row) for row in result])
