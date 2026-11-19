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
from rdflib.namespace import SDO

from motion_spec.runs.provenance import kept_dsl_document

MODEL_REL = Path("generated") / "model"


def model_document(generation_dir: Path | str) -> Path | None:
    """The document the DSL generated, as the generation's provenance records it."""
    return kept_dsl_document(Path(generation_dir) / "generated")


def graph_sizes(dataset: rdflib.Dataset) -> dict[str, int]:
    """Triples per named graph. A JSON-LD file that declares a graph of its own lands there,
    not in the graph it was parsed into, so this is what a parse has to be measured against."""
    return {str(graph.identifier): len(graph) for graph in dataset.graphs()}


def graph_growth(before: dict[str, int], after: dict[str, int]) -> dict:
    """What one parse added: the total, and which named graphs it went into."""
    grew = {name: size - before.get(name, 0) for name, size in after.items()}
    return {
        "triples": sum(grew.values()),
        "graphs": [name for name, count in grew.items() if count],
    }


def load_document(path: Path | str, dataset: rdflib.Dataset) -> dict:
    """Parse one document into DATASET, in a graph named by its file unless it names its own.

    Returns what it read: the file, and the triples it put in each named graph.
    """
    path = Path(path).resolve()
    before = graph_sizes(dataset)
    dataset.graph(rdflib.URIRef(path.as_uri())).parse(path, format="json-ld")
    return {"iri": path.as_uri(), "path": str(path), **graph_growth(before, graph_sizes(dataset))}


@lru_cache(maxsize=64)
def deployed_devices(generation_dir: Path | str) -> tuple[str, ...]:
    """The device models the execution context realizes, as the model names them.

    This is what a real platform is made of; a simulation carries a name of its own instead, so
    nothing here has to tell one platform kind from another.
    """
    document = model_document(generation_dir)
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

    def __init__(
        self,
        generation_dir: Path | str,
        *,
        document: Path | None = None,
        graphs: list[Path] | None = None,
    ):
        self.generation_dir = Path(generation_dir)
        self.dataset = rdflib.Dataset(default_union=True)
        # A run names its own model document; the generation is only where one is found without it.
        if not (document and document.is_file()):
            document = model_document(self.generation_dir)
        # The scene and FSM graphs the IR names nodes of, which the DSL's document holds none of.
        if graphs is None:
            model_dir = self.generation_dir / MODEL_REL
            graphs = [
                path
                for pattern in ("*.scenex.ld.json", "*.fsm.ld.json")
                for path in model_dir.glob(pattern)
            ]
        documents = [document, *graphs] if document is not None else graphs
        self.sources = [load_document(path, self.dataset) for path in documents]

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

    def namespaces(self) -> dict[str, str]:
        """Bound prefix -> IRI, for CURIE display and query autocompletion."""
        return {prefix: str(ns) for prefix, ns in self.dataset.namespaces()}
