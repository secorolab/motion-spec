# SPDX-License-Identifier: MPL-2.0

"""SPARQL over a run's graph, and the questions worth keeping."""

from __future__ import annotations

import json
import time
from pathlib import Path

import rdflib
from rdf_utils.uri import iri_parent

from motion_spec.dashboard.catalog import rdf_name
from motion_spec.dashboard.graph import GraphService
from motion_spec.dashboard.roots import json_file
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
            run_dir.parent.parent,
            store,
            manifest=run_model_manifest(run_dir),
            runtime_ttl=run_runtime_ttl(run_dir),
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


# -- temporal and causal views -------------------------------------------------------------
#
# All three read the archived runtime graph joined to the design graph, and open no frame log:
# a step converts to seconds through the tick rate the run recorded, not the log header.

# What a span was: the runtime graph names only the design IRI, so the *kind* of element comes
# from the design graph's own rdf:type. Coordination elements live in the behaviour metamodel
# family (fsm# today, behaviour-tree# next), which is why nothing here reads "state".
BEHAVIOUR_MM = "https://secorolab.github.io/metamodels/behaviour/"

VIEW_PREFIXES = """
PREFIX trace:    <https://secorolab.github.io/metamodels/motion-spec/execution-trace/>
PREFIX prov:     <http://www.w3.org/ns/prov#>
PREFIX time:     <http://www.w3.org/2006/time#>
PREFIX sosa:     <http://www.w3.org/ns/sosa/>
PREFIX qudt:     <http://qudt.org/schema/qudt/>
PREFIX sens:     <https://secorolab.github.io/metamodels/robot/sensors#>
PREFIX exec:     <https://secorolab.github.io/metamodels/execution-context#>
PREFIX cstr:     <https://comp-rob2b.github.io/metamodels/task/constraint#>
PREFIX cstr-hdl: <https://comp-rob2b.github.io/metamodels/task/constraint-handler#>
PREFIX ch:       <https://secorolab.github.io/metamodels/task/constraint-handler#>
"""

# The run's own tick rate, hung off the agent that produced its frames. Scoped through the run
# so the sensors declaring update rates of their own cannot answer instead.
RUN_PERIOD = (
    VIEW_PREFIXES
    + """
SELECT ?hz WHERE {
    ?run a exec:ExecutionContext ; prov:wasGeneratedBy/prov:wasAssociatedWith ?producer .
    ?producer sens:update-rate/qudt:value ?hz .
} LIMIT 1
"""
)

# Every occupancy, in order, with what moved control into it. `?element` may be bound by the
# caller to narrow the timeline to one design IRI.
ACTIVITY_TIMELINE = (
    VIEW_PREFIXES
    + f"""
SELECT ?occ ?element ?seq ?beginStep ?endStep ?transition ?event WHERE {{
    ?occ a trace:ActivityOccurrence ; trace:seq ?seq ; prov:used ?element ;
         time:hasBeginning/trace:step ?beginStep .
    FILTER EXISTS {{ ?element a ?kind . FILTER(STRSTARTS(STR(?kind), "{BEHAVIOUR_MM}")) }}
    OPTIONAL {{ ?occ time:hasEnd/trace:step ?endStep }}
    OPTIONAL {{
        ?occ prov:wasInformedBy ?flow .
        ?flow a trace:ControlFlowOccurrence ; prov:used ?transition .
        OPTIONAL {{ ?flow prov:wasInformedBy/prov:used ?event }}
    }}
}} ORDER BY ?beginStep ?seq
"""
)

# Temporal containment is derived here, never stored: a constraint span counts as satisfied
# during an occupancy when both its endpoints fall inside the occupancy's. `?occ` is bound by
# the caller -- left open it is a cross join of every span against every other, which costs
# ten seconds on a real run to answer a question nobody asked of all eleven rows at once.
CONSTRAINTS_DURING_ACTIVITY = (
    VIEW_PREFIXES
    + """
SELECT DISTINCT ?constraint WHERE {
    ?occ time:hasBeginning/trace:step ?from ; time:hasEnd/trace:step ?to .
    ?held a trace:ActivityOccurrence ; prov:used ?constraint ;
          time:hasBeginning/trace:step ?heldFrom ; time:hasEnd/trace:step ?heldTo .
    ?constraint a cstr:Constraint .
    FILTER(?heldFrom >= ?from && ?heldTo <= ?to)
}
"""
)

# One row per arming: when the watched condition first held (the span's beginning), when the
# gate fired (its end), how often it broke meanwhile, and the dwell the *design* graph declares
# -- joined, never copied. A monitor that never fired keeps its row with the span left unbound.
GATE_ANALYSIS = (
    VIEW_PREFIXES
    + """
SELECT ?monitor ?seq ?firstHeldStep ?firedStep ?rearmCount ?event ?declaredDwell
       (GROUP_CONCAT(DISTINCT STR(?member); SEPARATOR=" ") AS ?members)
WHERE {
    ?monitor a cstr-hdl:Monitor .
    OPTIONAL { ?monitor ch:debounce-duration/qudt:value ?declaredDwell }
    OPTIONAL { ?monitor cstr-hdl:event ?event }
    OPTIONAL {
        ?occ a trace:ActivityOccurrence ; prov:used ?monitor ; trace:seq ?seq ;
             time:hasBeginning/trace:step ?firstHeldStep .
        OPTIONAL { ?occ time:hasEnd/trace:step ?firedStep }
        OPTIONAL { ?occ trace:rearmCount ?rearmCount }
        OPTIONAL {
            ?occ prov:wasInformedBy ?watch .
            ?watch a trace:ActivityOccurrence ; prov:used ?member .
        }
    }
}
GROUP BY ?monitor ?seq ?firstHeldStep ?firedStep ?rearmCount ?event ?declaredDwell
ORDER BY ?seq ?monitor
"""
)

# What the model declares can be occupied at all -- unchanged by a run that stopped entering
# one of them, which is exactly what makes it usable as the two runs' shared identity.
DECLARED_ACTIVITIES = (
    VIEW_PREFIXES
    + f"""
SELECT DISTINCT ?element WHERE {{
    ?element a ?kind . FILTER(STRSTARTS(STR(?kind), "{BEHAVIOUR_MM}"))
}}
"""
)


def views_graph(run_dir: Path) -> GraphService:
    """The dataset these views read: the archived runtime graph joined to the design graph.

    A run that kept a runtime.ttl answers from it alone, with an empty store -- the log is
    never opened, not even for its header, and that boundary is what makes these views cheap.
    A run still executing has no archived graph, so it falls back to 015's projection; nothing
    here reads a frame, and that projection is strided, so its armings report as unavailable.
    """
    archived = run_runtime_ttl(run_dir)
    if archived is None:
        return run_graph(run_dir, frames=True)
    return GraphService(
        run_dir.parent.parent,
        RunStore(run_dir.name),
        manifest=run_model_manifest(run_dir),
        runtime_ttl=archived,
    )


def _rows(service: GraphService, sparql: str, **bindings) -> list:
    service.sync()
    return list(service.dataset.query(sparql, initBindings=bindings or None))


def _step(term) -> int | None:
    return None if term is None else int(term)


def _seconds(steps: int | None, period_s: float | None) -> float | None:
    """A step count as seconds, or nothing when the run recorded no tick rate."""
    if steps is None or period_s is None:
        return None
    return round(steps * period_s, 3)


def _period_s(service: GraphService) -> float | None:
    rows = _rows(service, RUN_PERIOD)
    hz = float(rows[0][0]) if rows and rows[0][0] is not None else 0.0
    return 1 / hz if hz else None


def _qualified_name(iri) -> str:
    """A monitor's name with the handler it belongs to, since `mon-opened` names five of them."""
    try:
        return f"{rdf_name(iri_parent(iri))}/{rdf_name(iri)}"
    except ValueError:
        return rdf_name(iri)


def _names(concatenated) -> list[str]:
    """The display names in a GROUP_CONCAT of IRIs, in the order the group produced them."""
    return [rdf_name(iri) for iri in str(concatenated or "").split() if iri]


def timeline(run_dir: Path, iri: str | None = None) -> dict:
    """Every occupancy this run recorded, in order, with what moved control into it.

    Repeated occupancies stay separate rows: a re-entry is usually the thing being looked for.
    Narrowed to one design IRI, each row also carries what held throughout it -- the interval
    filter is a cross join, so it is answered for the spans actually asked about.
    """
    service = views_graph(run_dir)
    period_s = _period_s(service)
    bindings = {"element": rdflib.URIRef(iri)} if iri else {}
    spans = []
    for occ, element, _seq, begin, end, transition, event in _rows(
        service, ACTIVITY_TIMELINE, **bindings
    ):
        begin_step, end_step = _step(begin), _step(end)
        satisfied = (
            [rdf_name(row[0]) for row in _rows(service, CONSTRAINTS_DURING_ACTIVITY, occ=occ)]
            if iri
            else []
        )
        spans.append(
            {
                "occurrence": str(occ),
                "element": str(element),
                "name": rdf_name(element),
                "begin_step": begin_step,
                "end_step": end_step,
                "entered_s": _seconds(begin_step, period_s),
                "duration_s": _seconds(
                    None if end_step is None else end_step - begin_step, period_s
                ),
                "transition": None if transition is None else rdf_name(transition),
                "event": None if event is None else rdf_name(event),
                "satisfied": sorted(satisfied),
            }
        )
    return {
        "period_s": period_s,
        "runtime_source": service.runtime_source or "unavailable",
        "spans": spans,
    }


def gates(run_dir: Path) -> dict:
    """Each gate's arming: how long it waited, how often it re-armed, against its declared dwell.

    An observation, never a verdict -- a long wait may be exactly what the author wanted. A
    projection has only strided frames behind it, so its waits and re-arms are reported
    unavailable rather than as numbers a reader would trust.
    """
    service = views_graph(run_dir)
    period_s = _period_s(service)
    archived = service.runtime_source == "archive"
    rows = []
    for monitor, _seq, held, fired, rearm, event, dwell, members in _rows(service, GATE_ANALYSIS):
        first_held_step, fired_step = _step(held), _step(fired)
        waited = (
            None if first_held_step is None or fired_step is None else fired_step - first_held_step
        )
        rows.append(
            {
                "monitor": str(monitor),
                "monitor_name": _qualified_name(monitor),
                "event": None if event is None else str(event),
                "event_name": None if event is None else rdf_name(event),
                "members": _names(members),
                "first_held_step": first_held_step,
                "fired_step": fired_step,
                "waited_s": _seconds(waited, period_s) if archived else None,
                "rearm_count": (int(rearm) if rearm is not None else None) if archived else None,
                "declared_dwell_s": None if dwell is None else float(dwell),
            }
        )
    return {
        "period_s": period_s,
        "runtime_source": service.runtime_source or "unavailable",
        "gates": rows,
    }


def _declared(run_dir: Path) -> set[str]:
    return {str(row[0]) for row in _rows(views_graph(run_dir), DECLARED_ACTIVITIES)}


def compare(left_dir: Path, right_dir: Path) -> dict:
    """Two runs' timelines aligned occurrence by occurrence, in the left run's order.

    An element entered in only one of them keeps its row with the other side blank: an element
    that stopped being entered is exactly the regression this view exists to show. Two runs of
    different models align what aligns, and the payload says they are different models.
    """
    left, right = timeline(left_dir), timeline(right_dir)
    pending: dict[str, list[dict]] = {}
    for span in right["spans"]:
        pending.setdefault(span["element"], []).append(span)
    rows = []
    for span in left["spans"]:
        queue = pending.get(span["element"]) or []
        rows.append(_aligned(span, queue.pop(0) if queue else None))
    for span in (span for queue in pending.values() for span in queue):
        rows.append(_aligned(None, span))
    return {
        "same_model": _declared(left_dir) == _declared(right_dir),
        "left": left,
        "right": right,
        "activities": rows,
    }


def _aligned(left: dict | None, right: dict | None) -> dict:
    both = [side["duration_s"] for side in (left, right) if side]
    delta = (
        right["duration_s"] - left["duration_s"] if left and right and None not in both else None
    )
    return {
        "element": (left or right)["element"],
        "name": (left or right)["name"],
        "left_s": left["duration_s"] if left else None,
        "right_s": right["duration_s"] if right else None,
        "delta_s": None if delta is None else round(delta, 3),
    }


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
