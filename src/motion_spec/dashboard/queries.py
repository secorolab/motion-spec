# SPDX-License-Identifier: MPL-2.0

"""SPARQL over a run's graph, and the questions worth keeping."""

from __future__ import annotations

import secrets
import time
from datetime import UTC, datetime
from pathlib import Path

import rdflib

from motion_spec.dashboard import roots
from motion_spec.dashboard.catalog import classify_quads, graph_name, term_graphs
from motion_spec.dashboard.graph import GraphService, load_document
from motion_spec.dashboard.metadata import LOCK, generation_of, write_document
from motion_spec.dashboard.roots import HEADER_REL, STATE_REL, json_file
from motion_spec.dashboard.runs import RunInfo
from motion_spec.dashboard.sources import declaration_lines
from motion_spec.runs.provenance import (
    GENERATION_DOCUMENT,
    kept_dsl_document,
    kept_sources,
    read_generation_dataset,
    run_generation,
)
from motion_spec.runs.replay import resolve_run
from motion_spec.runs.verify import RecordError

# a larger picture locks the browser
GRAPH_MAX_TRIPLES = 20_000


_GRAPHS: dict[tuple[str, int], GraphService] = {}


QUERIES_REL = "queries.json"
NOTES_REL = "notes.json"


def saved_queries(run_dir: Path) -> list:
    """The queries kept beside this run."""
    return json_file(
        roots.GENERATIONS / STATE_REL / run_dir.relative_to(roots.GENERATIONS) / QUERIES_REL
    ).get("queries", [])


def save_queries(run_dir: Path, queries: list) -> dict:
    """Keep a run's queries with the run, so they outlive the browser that wrote them."""
    if not RunInfo(run_dir).log_path.exists():
        raise ValueError("queries belong to a run")
    texts = [str(query) for query in queries][:200]
    with LOCK:
        write_document(
            roots.GENERATIONS / STATE_REL / run_dir.relative_to(roots.GENERATIONS) / QUERIES_REL,
            {"queries": texts},
        )
    return {"saved": len(texts)}


def run_notes(run_dir: Path) -> dict:
    """Every note kept beside this run, as written."""
    return {
        "notes": json_file(
            roots.GENERATIONS / STATE_REL / run_dir.relative_to(roots.GENERATIONS) / NOTES_REL
        ).get("notes", [])
    }


def save_notes(run_dir: Path, notes: list) -> dict:
    """Replace the run's notes with the list given. A note without an id or a time gets both here,
    so the client never invents either."""
    generation = generation_of(run_dir)
    if not isinstance(notes, list) or len(notes) > 200:
        raise ValueError("notes must be a list of at most 200 entries")
    kept = []
    for note in notes:
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
        named = list(dict.fromkeys(tag.strip() for tag in tags if tag.strip()))
        kept.append(
            {
                "id": str(note.get("id") or secrets.token_hex(6)),
                "created": str(
                    note.get("created") or datetime.now(UTC).isoformat(timespec="seconds")
                ),
                "text": str(note.get("text") or "")[:4000],
                "frame": frame,
                "end_frame": end_frame,
                "tags": named[:20],
            }
        )
    payload = {"notes": kept}
    with LOCK:
        write_document(
            roots.GENERATIONS / STATE_REL / run_dir.relative_to(roots.GENERATIONS) / NOTES_REL,
            payload,
        )
    return payload


PAGE_SIZE = 500


def run_query(run_dir: Path, sparql: str, offset: int = 0) -> dict:
    """Answer one SPARQL query against a run, or say why it could not be answered.

    The answer carries the rows, paged PAGE_SIZE at a time, and the whole result as a graph.
    A generation path is answered from its model graph.
    """
    if (run_dir / HEADER_REL).exists():
        service = GraphService(run_dir)
    else:
        _, log, _contract = resolve_run(run_dir)
        key = (str(log), log.stat().st_size)
        if key not in _GRAPHS:
            generation = run_generation(run_dir)
            if generation is None:
                raise RecordError(run_dir, "the run's document names no generation's provenance")
            _GRAPHS.clear()
            _GRAPHS[key] = GraphService(generation)
        service = _GRAPHS[key]
    started = time.perf_counter()
    try:
        kind, headers, rows = service.query(sparql)
    except Exception as exc:
        detail = f"{type(exc).__name__}: {exc}"
        if not len(service.dataset):
            detail += (
                ". This run's model graph is unavailable -- the generation it names is missing, "
                "so only its recorded observations can be queried."
            )
        raise ValueError(detail) from exc
    prefixes = {prefix: str(ns) for prefix, ns in service.dataset.namespaces()}
    offset = max(0, min(int(offset), max(0, len(rows) - 1)))
    return {
        "type": kind,
        "model_triples": len(service.dataset),
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


def result_graph(service: GraphService, kind: str, rows: list) -> dict | None:
    """The result drawn as a graph, or None when there is nothing to draw.

    A constructed graph is its own triples. Bindings are drawn as the IRIs they bound, plus the
    unbound terms that join two of them.
    """
    if kind in ("CONSTRUCT", "DESCRIBE"):
        # a derived triple is in no named graph
        quads = [
            (
                *triple,
                next((graph_name(quad[3]) for quad in service.dataset.quads(triple)), "result"),
            )
            for triple in rows
        ]
    elif kind == "SELECT":
        bound = {t for row in rows for t in row if isinstance(t, rdflib.URIRef)}
        # one hop only; a single answer's neighbourhood is Expand's
        joins: dict = {}
        for subject, predicate, obj, _context in service.dataset.quads((None, None, None, None)):
            if predicate == rdflib.RDF.type or isinstance(obj, rdflib.Literal):
                continue
            for term, other in ((subject, obj), (obj, subject)):
                if other in bound and term not in bound:
                    joins.setdefault(term, set()).add(other)
        drawn = bound | {term for term, reached in joins.items() if len(reached) > 1}
        quads = [
            (subject, predicate, obj, graph_name(context))
            for subject, predicate, obj, context in service.dataset.quads((None, None, None, None))
            if subject in drawn
            and (obj in drawn or isinstance(obj, rdflib.Literal) or predicate == rdflib.RDF.type)
        ]
    else:
        return None
    if not quads or len(quads) > GRAPH_MAX_TRIPLES:
        return None
    return classify_quads(quads, term_graphs(service.dataset))


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
        default=("", ""),
    )
    return f"{prefix}:{text[len(namespace) :]}" if prefix else text


# declared values only: a run-time value is unreferenced by design
UNUSED_DECLARATION = """
PREFIX qudt: <http://qudt.org/schema/qudt/>
SELECT ?term ?value WHERE {
  ?term a qudt:Quantity ; qudt:value ?value .
  FILTER NOT EXISTS { ?subject ?predicate ?term }
}
"""

LINT_SEVERITY = "warn"


def model_lint(generation_dir: Path) -> dict:
    """What the design graph says about itself: terms declared and read by nothing.

    A generation with no model document lints to nothing rather than failing.
    """
    generated = generation_dir / "generated"
    document = kept_dsl_document(generated)
    if document is None:
        return {"items": []}
    dataset = rdflib.Dataset(default_union=True)
    load_document(document, dataset)
    provenance = read_generation_dataset(generated / GENERATION_DOCUMENT)
    authored = next((path for path in kept_sources(provenance) if path.suffix == ".robmot"), None)
    lines = declaration_lines(authored) if authored else {}
    term, value = rdflib.Variable("term"), rdflib.Variable("value")
    items = [
        {
            "rule": "unused-declaration",
            "severity": LINT_SEVERITY,
            "iri": str(binding[term]),
            "name": name,
            "source_line": lines.get(name),
            "why": f"declared as {binding[value]} and read by nothing in the model",
        }
        for binding in dataset.query(UNUSED_DECLARATION).bindings
        if (name := str(binding[term]).rsplit("/", 1)[-1].rsplit("#", 1)[-1])
    ]
    return {
        "items": sorted(
            items,
            key=lambda item: (item["source_line"] is None, item["source_line"] or 0, item["name"]),
        )
    }
