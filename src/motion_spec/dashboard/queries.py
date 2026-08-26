# SPDX-License-Identifier: MPL-2.0

"""SPARQL over a run's graph, and the questions worth keeping."""

from __future__ import annotations

import json
import time
from pathlib import Path

import rdflib

from motion_spec.dashboard.catalog import classify_quads, graph_name, term_graphs
from motion_spec.dashboard.graph import GraphService
from motion_spec.dashboard.roots import LAYOUT_REL, json_file
from motion_spec.dashboard.store import RunStore
from motion_spec.dashboard.tail import FrameLogTail
from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.replay import resolve_archive

GRAPH_SAMPLE_S = 0.1  # the graph wants the shape of a run, not its every tick
# A picture of a hundred thousand triples is a locked browser, not an answer.
GRAPH_MAX_TRIPLES = 20_000


_GRAPHS: dict[tuple[str, int, bool], GraphService] = {}


def run_model_manifest(run_dir: Path) -> Path | None:
    """The model graph this run names, wherever the manifest says it lives."""
    manifest = json_file(run_dir / "manifest.json").get("files", {})
    named = manifest.get("model")
    if not named:
        return None
    path = (run_dir / named).resolve()
    return path if path.is_file() else None


def run_runtime_ttl(run_dir: Path) -> Path | None:
    """The archived runtime graph this run names -- the validated record, not a re-projection."""
    named = json_file(run_dir / "manifest.json").get("files", {}).get("runtime_ttl")
    if not named:
        return None
    path = (run_dir / named).resolve()
    return path if path.is_file() else None


def generation_graph(generation_dir: Path) -> GraphService:
    """A generation's model graph on its own -- no run, so nothing recorded to merge in."""
    return GraphService(generation_dir, RunStore(generation_dir.name))


def run_graph(run_dir: Path) -> GraphService:
    """One run's queryable dataset: its model, plus what the recording says happened.

    Reading the frames is what fills `urn:runtime` and `urn:live` for a run still being
    written. A run that archived a `runtime.ttl` has the record already, and sweeping its log
    to rediscover it would cost tens of seconds and emit a second occurrence for every one
    already there. Kept per log revision: a finished run is read once, a growing one again.
    """
    _, log, _manifest, contract = resolve_archive(run_dir)
    frames = run_runtime_ttl(run_dir) is None
    key = (str(log), log.stat().st_size, frames)
    if key not in _GRAPHS:
        store = RunStore(run_dir.name, contract)
        tail = FrameLogTail(log)
        if frames and tail.open():
            # shaping every frame to project a tenth of them is the waste, not the reading
            period = (contract.header.nominal_period_ns or 1_000_000) / 1e9
            stride = max(1, round(GRAPH_SAMPLE_S / period))
            while records := tail.poll(stride):
                store.add_frames(records)
            tail.close()
        _GRAPHS.clear()
        _GRAPHS[key] = GraphService(
            run_dir.parent.parent,
            store,
            manifest=run_model_manifest(run_dir),
            runtime_ttl=run_runtime_ttl(run_dir),
        )
    return _GRAPHS[key]


def query_graph(path: Path) -> GraphService:
    """The dataset a query is asked of: a run's, or a generation's model on its own.

    A generation with no run still has a model graph worth exploring, and the Explore page is
    the only way left to look at one -- so a query aimed at a generation answers from it.
    """
    if (path / LAYOUT_REL).exists():
        return generation_graph(path)
    return run_graph(path)


QUERIES_REL = "queries.json"


def saved_queries(run_dir: Path) -> list:
    """The queries kept beside this run."""
    stored = json_file(run_dir / QUERIES_REL)
    return stored.get("queries", []) if isinstance(stored, dict) else []


def save_queries(run_dir: Path, queries: list) -> dict:
    """Keep a run's queries with the run, so they outlive the browser that wrote them."""
    if not frame_log_pb.log_path(run_dir / "logs" / "frame_log.pb").exists():
        raise ValueError("queries belong to a run")
    texts = [str(query) for query in queries][:200]
    (run_dir / QUERIES_REL).write_text(json.dumps({"queries": texts}, indent=1))
    return {"saved": len(texts)}


def run_query(run_dir: Path, sparql: str) -> dict:
    """Answer one SPARQL query against a run, or say why it could not be answered.

    The answer carries both projections of the one result: the rows, and the classified graph
    the same terms make. A table and a picture disagreeing would be two results, not two views.
    """
    service = query_graph(run_dir)
    started = time.perf_counter()
    try:
        kind, headers, rows = _query_rows(service, sparql)
    except Exception as exc:
        detail = f"{type(exc).__name__}: {exc}"
        if not len(service.model):
            # no prefixes, no model triples: the generation this run names is not there
            detail += (
                ". This run's model graph is unavailable -- the generation it names is missing, "
                "so only its recorded observations can be queried."
            )
        raise ValueError(detail) from exc
    prefixes = service.namespaces()
    return {
        "type": kind,
        "model_triples": len(service.model),
        "headers": headers or ["result"],
        "rows": [[curie(term, prefixes) for term in row] for row in rows[:500]],
        "count": len(rows),
        "truncated": len(rows) > 500,
        "graph": result_graph(service, kind, rows),
        "runtime_source": service.runtime_source,
        "elapsed_ms": round((time.perf_counter() - started) * 1000, 1),
        "namespaces": prefixes,
    }


def _query_rows(service: GraphService, sparql: str) -> tuple[str, list[str], list]:
    """Answer any query shape as headers and rows; a graph comes back as its triples."""
    kind, payload = service.query(sparql)
    if kind in ("CONSTRUCT", "DESCRIBE"):
        return kind, ["subject", "predicate", "object"], payload
    headers, rows = payload
    return kind, headers, rows


def result_graph(service: GraphService, kind: str, rows: list) -> dict | None:
    """The result drawn as a graph, or None when there is nothing to draw.

    A constructed graph is its own triples. Bindings are drawn as the IRIs they bound, plus the
    unbound terms that join two of them -- a row set whose members are related only through a
    handler or a motion draws as that relation rather than as a field of loose dots.
    Either way the picture answers the same query the table does.
    """
    if kind in ("CONSTRUCT", "DESCRIBE"):
        quads = [(*triple, _home_graph(service, triple)) for triple in rows]
    elif kind == "SELECT":
        drawn = _connected(
            service, {t for row in rows for t in row if isinstance(t, rdflib.URIRef)}
        )
        quads = [
            (subject, predicate, obj, graph_name(context))
            for subject, predicate, obj, context in service.dataset.quads((None, None, None, None))
            if subject in drawn
            and (obj in drawn or isinstance(obj, rdflib.Literal) or predicate == rdflib.RDF.type)
        ]
    else:
        return None  # an ASK answers yes or no; there is no picture of that
    if not quads or len(quads) > GRAPH_MAX_TRIPLES:
        return None
    return classify_quads(quads, term_graphs(service.dataset))


def _connected(service: GraphService, bound: set) -> set:
    """The bound terms, plus every term that stands between two of them.

    One hop is as far as this goes: a term reached from a single answer is that answer's
    neighbourhood, which is what Expand is for, not part of the answer.
    """
    joins: dict = {}
    for subject, predicate, obj, _context in service.dataset.quads((None, None, None, None)):
        if predicate == rdflib.RDF.type or isinstance(obj, rdflib.Literal):
            continue
        for term, other in ((subject, obj), (obj, subject)):
            if other in bound and term not in bound:
                joins.setdefault(term, set()).add(other)
    return bound | {term for term, reached in joins.items() if len(reached) > 1}


def _home_graph(service: GraphService, triple) -> str:
    """Which named graph a result triple came from; a derived one came from none of them."""
    return next((graph_name(quad[3]) for quad in service.dataset.quads(triple)), "result")


def curie(term, prefixes: dict) -> str | None:
    """A term as its shortest bound prefix form, so a table stays readable."""
    if term is None:
        return None
    text = str(term)
    if not isinstance(term, rdflib.URIRef):
        return text
    prefix, namespace = max(
        ((prefix, ns) for prefix, ns in prefixes.items() if text.startswith(ns)),
        key=lambda item: len(item[1]),
        default=(None, None),
    )
    return f"{prefix}:{text[len(namespace) :]}" if prefix else text
