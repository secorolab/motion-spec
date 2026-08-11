# SPDX-License-Identifier: MPL-2.0
"""SPARQL over a run: the model graph, its occurrences, and the latest frame's values.

Three named graphs, each with a different lifetime:

* ``urn:model``   -- parsed once from the generation's app manifest and its imports.
* ``urn:runtime`` -- occurrences plus interval-sampled values, appended as frames arrive.
* ``urn:live``    -- the newest frame only, cleared and refilled per query so per-tick values
  never accumulate.

Values enter as ``sosa:Observation`` instances against IRIs the model already declares; the
dashboard mints no vocabulary of its own.
"""

from __future__ import annotations

from pathlib import Path

import rdflib
from motion_spec_dsl.rdf_parser.manifest import build_url_map, metamodel_url_map
from motion_spec_dsl.rdf_parser.vocab import APP, CSTR_HDL
from rdf_utils.resolver import IriToFileResolver, install_resolver

from motion_spec.introspection.runtime_graph import (
    IncrementalProjector,
    bind_namespaces,
    condition_map_from_paths,
    frame_observations,
)

MODEL_GRAPH = rdflib.URIRef("urn:model")
RUNTIME_GRAPH = rdflib.URIRef("urn:runtime")
LIVE_GRAPH = rdflib.URIRef("urn:live")
MODEL_REL = Path("generated") / "model"


def model_manifest(generation_dir: Path | str) -> Path | None:
    """The generation's `-app` manifest, the root the rest of the model graph hangs off."""
    matches = sorted((Path(generation_dir) / MODEL_REL).glob("*-app.ld.json"))
    return matches[0] if matches else None


def load_model_graph(manifest_path: Path | str, dataset: rdflib.Dataset, graph: rdflib.Graph):
    """Parse an app manifest and everything it imports, resolving IRIs the way
    `motion-spec check` does: the shared metamodel checkout merged with the model's own
    iri-map, longest prefix first, so nothing reaches the network."""
    manifest_path = Path(manifest_path).resolve()
    graph.parse(manifest_path, format="json-ld")
    url_map = {**metamodel_url_map(), **build_url_map(dataset, manifest_path)}
    install_resolver(
        IriToFileResolver(
            dict(sorted(url_map.items(), key=lambda item: len(item[0]), reverse=True)),
            download=False,
        )
    )
    for target in {o for _s, _p, o, _g in dataset.quads((None, APP["import"], None, None))}:
        graph.parse(location=str(target), format="json-ld")
    return graph


def signal_map(model: rdflib.Graph) -> dict[str, dict]:
    """Controller IRI -> its error/control signal IRIs, as the model declares them.

    Those signals are what a value observation is *of*: the dashboard observes a property the
    model already names, rather than inventing one per slot.
    """
    mapping: dict[str, dict] = {}
    for role, predicate in (
        ("error", CSTR_HDL["error-signal"]),
        ("output", CSTR_HDL["control-signal"]),
    ):
        for subject, obj in model.subject_objects(predicate):
            if isinstance(obj, rdflib.URIRef):
                mapping.setdefault(str(subject), {})[role] = obj
    return mapping


class GraphService:
    """One run's queryable graph, kept current from its store."""

    def __init__(self, generation_dir: Path | str, store, *, sample_interval_s: float | None = 1.0):
        self.generation_dir = Path(generation_dir)
        self.store = store
        self.sample_interval_s = sample_interval_s
        # default_union so a plain { ?s ?p ?o } spans model, runtime and live, which is what
        # someone typing into the console means.
        self.dataset = rdflib.Dataset(default_union=True)
        self.model = self.dataset.graph(MODEL_GRAPH)
        self.runtime = self.dataset.graph(RUNTIME_GRAPH)
        self.live = self.dataset.graph(LIVE_GRAPH)
        self._projector: IncrementalProjector | None = None
        self._fed = 0
        manifest = model_manifest(self.generation_dir)
        if manifest is not None:
            load_model_graph(manifest, self.dataset, self.model)
        self.signals = signal_map(self.model)
        bind_namespaces(self.dataset, store.run_id)

    def _ensure_projector(self) -> IncrementalProjector | None:
        """Build the projector once the run's log contract is known (it carries the header)."""
        if self._projector is None and self.store.contract is not None:
            header = self.store.contract.header
            self._projector = IncrementalProjector(
                self.runtime,
                self.store.run_id,
                header,
                condition_map_from_paths(sorted((self.generation_dir / MODEL_REL).glob("*.json"))),
                sample_interval_s=self.sample_interval_s,
                signal_map=self.signals,
            )
            bind_namespaces(self.dataset, self.store.run_id, fsm_namespace=header.fsm_namespace)
        return self._projector

    def _quantity_iris(self) -> dict:
        contract = self.store.contract
        if contract is None:
            return {}
        return {
            qid: contract.iri_by_id[qid]
            for qid in contract.quantity_ids
            if qid in contract.iri_by_id
        }

    def sync(self) -> None:
        """Project whatever frames the store has gained, then refresh the live overlay."""
        projector = self._ensure_projector()
        if projector is not None:
            frames = self.store.snapshot()
            for frame in frames[self._fed :]:
                projector.feed(frame)
            self._fed = len(frames)
        self.refresh_live()

    def refresh_live(self) -> None:
        """Replace the live graph with the newest frame's values -- replace, never accumulate."""
        self.live.remove((None, None, None))
        frame, contract = self.store.latest, self.store.contract
        if frame is None or contract is None:
            return
        frame_observations(
            self.live,
            self.store.run_id,
            contract.header,
            frame,
            signal_map=self.signals,
            quantity_iris=self._quantity_iris(),
            satisfied=True,
        )

    def query(self, sparql: str) -> tuple[list[str], list[tuple]]:
        """(variable names, rows) for a SPARQL query over the current dataset."""
        self.sync()
        result = self.dataset.query(sparql)
        headers = [str(var) for var in (result.vars or [])]
        return headers, [tuple(row) for row in result]

    def namespaces(self) -> dict[str, str]:
        """Bound prefix -> IRI, for CURIE display and query autocompletion."""
        return {prefix: str(ns) for prefix, ns in self.dataset.namespaces()}
