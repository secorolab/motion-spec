# SPDX-License-Identifier: MPL-2.0
"""SPARQL over a run's model dataset.

The DSL's document, found through the generation's provenance, with the scene and FSM graphs
beside it, each document its own named graph.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import rdflib
from motion_spec_dsl.vocab import EXEC
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
    """The device models the execution context realizes, as the model names them."""
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
        # the scene and FSM graphs live in the other attributed documents
        provenance = read_generation_dataset(generated / GENERATION_DOCUMENT)
        dsl = next(provenance.subjects(SDO.name, rdflib.Literal("motion_spec_dsl")), None)
        for entity, agent in set(provenance.subject_objects(PROV.wasAttributedTo)):
            if agent == dsl:
                continue
            path = get_file_path(provenance, entity)
            if path is not None:
                load_document(path, self.dataset)

    def query(self, sparql: str) -> tuple[str, list[str], list[tuple]]:
        """(result type, column names, rows) for a SPARQL query over the current dataset.

        SELECT and ASK answer with their variables' bindings. CONSTRUCT and DESCRIBE answer with
        the graph they build, one (subject, predicate, object) row per triple.
        """
        result = self.dataset.query(sparql)
        if result.graph is not None:
            return result.type, ["subject", "predicate", "object"], list(result.graph)
        if result.type == "ASK":
            return result.type, ["answer"], [(result.askAnswer,)]
        variables = result.vars or []
        return (
            result.type,
            [str(var) for var in variables],
            [tuple(binding.get(var) for var in variables) for binding in result.bindings],
        )
