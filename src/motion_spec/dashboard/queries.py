# SPDX-License-Identifier: MPL-2.0

"""SPARQL over a run's graph, and the questions worth keeping."""

from __future__ import annotations

import json
import time
from pathlib import Path

import rdflib

from motion_spec.dashboard.catalog import rdf_name
from motion_spec.dashboard.graph import MODEL_GRAPH, GraphService, load_model_graph, model_manifest
from motion_spec.dashboard.roots import json_file
from motion_spec.dashboard.sources import declaration_lines
from motion_spec.dashboard.store import RunStore
from motion_spec.dashboard.tail import FrameLogTail
from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.replay import resolve_archive

GRAPH_SAMPLE_S = 0.1  # the graph wants the shape of a run, not its every tick


_GRAPHS: dict[tuple[str, int, bool], GraphService] = {}


def run_model_manifest(run_dir: Path) -> Path | None:
    """The model graph this run names, wherever the manifest says it lives."""
    manifest = json_file(run_dir / "manifest.json").get("files", {})
    named = manifest.get("model")
    if not named:
        return None
    path = (run_dir / named).resolve()
    return path if path.is_file() else None


def run_graph(run_dir: Path, *, frames: bool) -> GraphService:
    """One run's queryable dataset: its model, plus what the recording says happened.

    Reading the frames is what fills `urn:runtime` and `urn:live`, and on a long run that
    costs tens of seconds, so a query that asks only about the model does not pay for it.
    Kept per log revision: a finished run is read once, a growing one is read again.
    """
    _, log, _manifest, contract = resolve_archive(run_dir)
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
            run_dir.parent.parent, store, manifest=run_model_manifest(run_dir)
        )
    return _GRAPHS[key]


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
    """Answer one SPARQL query against a run, or say why it could not be answered."""
    # only a query that reaches for the recording pays for reading it
    recorded = any(word in sparql for word in ("urn:runtime", "urn:live", "sosa", "GRAPH ?"))
    service = run_graph(run_dir, frames=recorded)
    started = time.perf_counter()
    try:
        headers, rows = _query_rows(service, sparql)
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
        "model_triples": len(service.model),
        "headers": headers or ["result"],
        "rows": [[curie(term, prefixes) for term in row] for row in rows[:500]],
        "count": len(rows),
        "truncated": len(rows) > 500,
        "elapsed_ms": round((time.perf_counter() - started) * 1000, 1),
        "namespaces": prefixes,
    }


def _query_rows(service: GraphService, sparql: str) -> tuple[list[str], list]:
    """Answer any query shape as headers and rows: ASK says so, a graph comes back as text."""
    service.sync()
    result = service.dataset.query(sparql)
    if result.type == "ASK":
        return ["answer"], [(result.askAnswer,)]
    if result.type in ("CONSTRUCT", "DESCRIBE"):
        return ["triples"], [
            (line,) for line in result.serialize(format="turtle").decode().splitlines() if line
        ]
    return [str(var) for var in (result.vars or [])], [tuple(row) for row in result]


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


# A declared quantity is the one that carries its own value; a measured or derived quantity is
# given its value at run time and reads as unreferenced here for reasons that are not the
# author's. Referenced means named by anything at all -- a constraint's tolerance, threshold or
# reference value, a controller's bound, a solver limit, a path parameter -- so a term that is
# the object of no triple is read by nothing in the design. `zero-length` and `zero-angle` are
# the case this must not flag: they are tolerances, so a constraint names them.
UNUSED_DECLARATION = """
PREFIX qudt: <http://qudt.org/schema/qudt/>
SELECT ?term ?value WHERE {
  ?term a qudt:Quantity ; qudt:value ?value .
  FILTER NOT EXISTS { ?subject ?predicate ?term }
}
"""

# Warn, never error: the graph can say a term is read by nothing, but only the author knows
# whether that is a mistake or a value being kept for the next edit.
LINT_SEVERITY = "warn"


def model_lint(generation_dir: Path) -> dict:
    """What the design graph says about itself: terms declared and read by nothing.

    Design graph only -- no run, no frame log. A generation with no model manifest lints to
    nothing rather than failing, so the page can ask about any generation it lists.
    """
    manifest = model_manifest(generation_dir)
    if manifest is None:
        return {"items": []}
    dataset = rdflib.Dataset(default_union=True)
    model = load_model_graph(manifest, dataset, dataset.graph(MODEL_GRAPH))
    authored = next((generation_dir / "generated/source").glob("*.robmot"), None)
    lines = declaration_lines(authored.read_text()) if authored else {}
    items = [
        {
            "rule": "unused-declaration",
            "severity": LINT_SEVERITY,
            "iri": str(term),
            "name": name,
            "source_line": lines.get(name),
            "why": f"declared as {value} and read by nothing in the model",
        }
        for term, value in model.query(UNUSED_DECLARATION)
        if (name := rdf_name(term))
    ]
    # In the order they are read in; what an imported graph declares has no line here, so it
    # sorts to the end rather than to the top.
    return {
        "items": sorted(
            items,
            key=lambda item: (item["source_line"] is None, item["source_line"] or 0, item["name"]),
        )
    }
