# SPDX-License-Identifier: MPL-2.0

"""SPARQL over a run's graph, and the questions worth keeping."""

from __future__ import annotations

import json
import secrets
import time
from datetime import datetime, timezone
from pathlib import Path

import rdflib

from motion_spec.dashboard.catalog import classify_quads, graph_name, rdf_name, term_graphs
from motion_spec.dashboard.graph import (
    MODEL_GRAPH,
    GraphService,
    load_model_graph,
    model_manifest,
)
from motion_spec.dashboard.metadata import LOCK, generation_of, write_document
from motion_spec.dashboard.roots import LAYOUT_REL, json_file
from motion_spec.dashboard.sources import declaration_lines
from motion_spec.dashboard.store import RunStore
from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.replay import resolve_archive

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


def generation_graph(generation_dir: Path) -> GraphService:
    """A generation's model graph on its own."""
    return GraphService(generation_dir, RunStore(generation_dir.name))


def run_graph(run_dir: Path) -> GraphService:
    """One run's queryable model graph. Kept per log revision, as the run list is."""
    _, log, _manifest, contract = resolve_archive(run_dir)
    key = (str(log), log.stat().st_size, True)
    if key not in _GRAPHS:
        _GRAPHS.clear()
        _GRAPHS[key] = GraphService(
            run_dir.parent.parent,
            RunStore(run_dir.name, contract),
            manifest=run_model_manifest(run_dir),
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
NOTES_REL = "notes.json"


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


def run_notes(run_dir: Path) -> dict:
    """Every note kept beside this run, as written."""
    stored = json_file(run_dir / NOTES_REL)
    notes = stored.get("notes") if isinstance(stored, dict) else None
    return {
        "notes": [note for note in notes if isinstance(note, dict)]
        if isinstance(notes, list)
        else []
    }


def save_notes(run_dir: Path, notes: list) -> dict:
    """Replace the run's notes with the list given. A note without an id or a time gets both here,
    so the client never invents either."""
    generation = generation_of(run_dir)
    if not isinstance(notes, list) or len(notes) > 200:
        raise ValueError("notes must be a list of at most 200 entries")
    kept = []
    for note in notes[:200]:
        if not isinstance(note, dict):
            raise ValueError("each note must be an object")  # noqa: TRY004 -- HTTP input validation
        frame = note.get("frame")
        end_frame = note.get("end_frame")
        if frame is not None and (generation == run_dir or type(frame) is not int or frame < 0):
            raise ValueError("a frame must be a nonnegative integer on a run note")
        if end_frame is not None and (
            frame is None or type(end_frame) is not int or end_frame < frame
        ):
            raise ValueError("note range must end at or after its frame")
        tags = note.get("tags") or []
        if not isinstance(tags, list) or any(not isinstance(tag, str) for tag in tags):
            raise ValueError("note tags must be a list of strings")
        kept.append(
            {
                "id": str(note.get("id") or secrets.token_hex(6)),
                "created": str(
                    note.get("created") or datetime.now(timezone.utc).isoformat(timespec="seconds")
                ),
                "text": str(note.get("text") or "")[:4000],
                "frame": frame,
                "end_frame": end_frame,
                "tags": list(dict.fromkeys(str(tag).strip() for tag in tags if str(tag).strip()))[
                    :20
                ],
            }
        )
    payload = {"notes": kept}
    with LOCK:
        write_document(run_dir / NOTES_REL, payload)
    return payload


PAGE_SIZE = 500


def run_query(run_dir: Path, sparql: str, offset: int = 0) -> dict:
    """Answer one SPARQL query against a run, or say why it could not be answered.

    The answer carries both projections of the one result: the rows, and the classified graph
    the same terms make. A table and a picture disagreeing would be two results, not two views.
    The table pages through the rows PAGE_SIZE at a time; the graph is always the whole answer.
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
    offset = max(0, min(int(offset), max(0, len(rows) - 1)))
    return {
        "type": kind,
        "model_triples": len(service.model),
        "headers": headers or ["result"],
        "rows": [
            [curie(term, prefixes) for term in row] for row in rows[offset : offset + PAGE_SIZE]
        ],
        "count": len(rows),
        "offset": offset,
        "page_size": PAGE_SIZE,
        "graph": result_graph(service, kind, rows),
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


# Only declared quantities: one given its value at run time reads as unreferenced for reasons
# that are not the author's. Referenced means the object of any triple at all, which is why
# `zero-length` and `zero-angle` are not flagged -- they are tolerances a constraint names.
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

    Asked of the whole dataset, not of `urn:model`: a JSON-LD file that declares a graph of its
    own lands there instead, and a term those triples name is read, not unused. Nothing but the
    design is loaded here, so the union is the design.
    """
    manifest = model_manifest(generation_dir)
    if manifest is None:
        return {"items": []}
    dataset = rdflib.Dataset(default_union=True)
    load_model_graph(manifest, dataset, dataset.graph(MODEL_GRAPH))
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
        for term, value in dataset.query(UNUSED_DECLARATION)
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
