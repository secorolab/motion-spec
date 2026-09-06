# SPDX-License-Identifier: MPL-2.0
"""Project a run's occurrence stream into its runtime RDF, live and after the fact."""

from __future__ import annotations

import json
import math
import os
import threading
import time
from datetime import datetime, timezone
from decimal import Decimal
from pathlib import Path

import rdflib
from google.protobuf.message import DecodeError
from rdflib.plugins.serializers.nt import _nt_row

from motion_spec.introspection.archive import ArchiveError, load_manifest, sha256_file
from motion_spec_dsl.rdf_parser.vocab import CSTR_HDL
from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.provenance import (
    MSPROV,
    prov_uri,
    rec_run_lifecycle,
    rec_types,
    run_entity_uri,
)


def _dt_literal(wall_ns) -> rdflib.Literal | None:
    """Absolute wall time (epoch nanoseconds) as an xsd:dateTime (rdflib canonicalizes to +00:00,
    matching motion-spec.ld.json / rec.ld.json / provenance.ld.json)."""
    if wall_ns is None:
        return None
    sec, ns = divmod(int(wall_ns), 1_000_000_000)
    dt = datetime.fromtimestamp(sec, timezone.utc).replace(microsecond=ns // 1000)
    return rdflib.Literal(dt)


def _model_graphs(paths):
    """Each readable model graph among `paths`. A graph that will not parse is skipped: the
    maps below are lookups, and a missing entry degrades one occurrence rather than the run."""
    for path in paths:
        path = Path(path)
        if not path.exists() or path.suffix != ".json":
            continue
        try:
            yield rdflib.Graph().parse(path, format="json-ld")
        except Exception:  # noqa: S112 -- an unreadable import is not a reason to lose the run
            continue


def _model_paths(run_dir: Path, manifest: dict):
    return [run_dir / rel for rel in manifest.get("files", {}).get("model_imports") or []]


def declared_dwells_from_paths(paths) -> dict[str, float]:
    """Map monitor IRI -> its declared debounce, in seconds, from the model graph.

    Read to decide whether an arming waited long enough to be worth its member detail. The
    value itself never reaches the runtime graph: it is a design fact, and a consumer that
    wants to compare it against the observed dwell joins the two graphs.
    """
    dwells: dict[str, float] = {}
    for mg in _model_graphs(paths):
        for subject, duration in mg.subject_objects(CSTR_HDL["debounce-duration"]):
            value = mg.value(duration, QUDT.value)
            if value is not None:
                dwells[str(subject)] = float(value)
    return dwells


def units_from_graphs(graphs) -> dict[str, rdflib.URIRef]:
    """Map each design IRI to its declared qudt:unit, read from the model graphs.

    A quantity carries its unit directly. A constraint or monitor does not: it borrows the
    one unit its referenced quantities agree on, walked twice so a monitor reaches its
    constraint's quantity. An element whose references mix units stays unmapped -- no unit
    is honest, a guessed one is wrong.
    """
    units: dict[str, rdflib.URIRef] = {}
    edges: dict[str, set] = {}
    for mg in graphs:
        for subject, predicate, obj in mg:
            if not isinstance(subject, rdflib.URIRef) or not isinstance(obj, rdflib.URIRef):
                continue
            if predicate == QUDT.unit:
                units[str(subject)] = obj
            else:
                edges.setdefault(str(subject), set()).add(str(obj))
    for _ in range(2):
        for subject, targets in edges.items():
            if subject in units:
                continue
            borrowed = {units[t] for t in targets if t in units}
            if len(borrowed) == 1:
                units[subject] = borrowed.pop()
    return units


def units_from_paths(paths) -> dict[str, rdflib.URIRef]:
    """`units_from_graphs` over the model graphs a run archive vendored."""
    return units_from_graphs(_model_graphs(paths))


def signal_map_from_graphs(graphs) -> dict[str, dict]:
    """Controller IRI -> its error/control signal IRIs, as the model declares them.

    Those signals are what a sampled value is an observation *of*: the graph observes a
    property the model already names rather than minting one per slot.
    """
    mapping: dict[str, dict] = {}
    for mg in graphs:
        for role, predicate in (
            ("error", CSTR_HDL["error-signal"]),
            ("output", CSTR_HDL["control-signal"]),
        ):
            for subject, obj in mg.subject_objects(predicate):
                if isinstance(obj, rdflib.URIRef):
                    mapping.setdefault(str(subject), {})[role] = obj
    return mapping


def signal_map_from_paths(paths) -> dict[str, dict]:
    """`signal_map_from_graphs` over the model graphs a run archive vendored."""
    return signal_map_from_graphs(_model_graphs(paths))


PROV = rdflib.Namespace("http://www.w3.org/ns/prov#")
SOSA = rdflib.Namespace("http://www.w3.org/ns/sosa/")
BDD = rdflib.Namespace("https://secorolab.github.io/metamodels/acceptance-criteria/bdd#")
AGN = rdflib.Namespace("https://secorolab.github.io/metamodels/agent#")
OBS = rdflib.Namespace("https://secorolab.github.io/metamodels/observation#")
EXEC = rdflib.Namespace("https://secorolab.github.io/metamodels/execution-context#")
MSRUN = rdflib.Namespace("https://secorolab.github.io/motion-spec/runtime/")
# Vocabulary lives in the metamodel namespace the SHACL shape targets; MSRUN stays the base
# for this run's instance nodes. Emitting types under MSRUN left every sh:targetClass
# unmatched, so the shape validated nothing.
MS_PROV = rdflib.Namespace("https://secorolab.github.io/metamodels/motion-spec/prov#")
TIME = rdflib.Namespace("http://www.w3.org/2006/time#")
DCTERMS = rdflib.Namespace("http://purl.org/dc/terms/")
SENS = rdflib.Namespace("https://secorolab.github.io/metamodels/robot/sensors#")
QUDT = rdflib.Namespace("http://qudt.org/schema/qudt/")
QKIND = rdflib.Namespace("http://qudt.org/vocab/quantitykind/")
UNIT = rdflib.Namespace("http://qudt.org/vocab/unit/")
RUNTIME_RDF_CONTRACT_VERSION = 4
# Member edge occurrences kept per watched member of one interesting arming.
MEMBER_EDGE_LIMIT = 8
# How far past its own declared dwell a gate must hold its activity before the members it
# watches are worth recording. A gate that fired as soon as it armed was never waiting.
SLOW_GATE_FACTOR = 2.0
# How often the growing Turtle is rewritten while a run executes. The N-Triples journal beside
# it is complete per occurrence; this is only how fresh a reader of the Turtle finds it.
TTL_REWRITE_INTERVAL_S = 2.0
OCCURRENCE_REL = "logs/occurrences.pb"
RUNTIME_TTL_REL = "runtime/runtime.ttl"
RUNTIME_NT_REL = "runtime/runtime.nt"
# The graph is one projection of the stream the run appended to; nothing re-derives it.
PROJECTION_ACTIVITY = "activity:runtime_projection"
# rec lifecycle states in which the run never produced what it was meant to.
_FAILED_STATUSES = {"FAILED", "INTERRUPTED", "CANCELLED"}


def _node(identifier: str) -> rdflib.URIRef:
    safe = identifier.replace(":", "/")
    return MSRUN[safe]


def _scoped(family: str, run_id: str, *parts) -> rdflib.URIRef:
    """Per-run sample/occurrence node under `<family>/<run_id>/`, with a slash-free local
    (parts joined by '-') so it collapses to a CURIE against the bound family prefix."""
    local = "-".join(str(p) for p in parts)
    return MSRUN[f"{family}/{run_id}/{local}"]


def _literal(g: rdflib.Graph, subject: rdflib.URIRef, predicate: rdflib.URIRef, value) -> None:
    if value is None:
        return
    # Floats are float64 (double) throughout the pipeline (cpp/bin/replay). Preserve the exact
    # value: Decimal(repr(x)) round-trips to the same double and serializes as a plain decimal.
    # (rdflib's Turtle writer emits xsd:double in scientific notation AND truncates it to ~7
    # sig figs, which would silently lose precision — so xsd:double is not usable here.)
    if isinstance(value, float):
        if math.isfinite(value):
            value = Decimal(repr(value))
        # non-finite inf/nan fall through as a Python float -> xsd:double, which represents them
    g.add((subject, predicate, rdflib.Literal(value)))


def _result(g: rdflib.Graph, subject: rdflib.URIRef, value, unit=None) -> None:
    """The observed value as a qudt:QuantityValue: the number and its declared unit.

    The unit is joined from the design graph at write time; a truth value is dimensionless
    and counts as unit:UNITLESS. A value nothing resolved a unit for carries none -- no unit
    is honest, a guessed one is wrong. A None value records no result at all.
    """
    if value is None:
        return
    node = rdflib.URIRef(f"{subject}/result")
    g.add((subject, SOSA.hasResult, node))
    g.add((node, rdflib.RDF.type, QUDT.QuantityValue))
    _literal(g, node, QUDT.value, value)
    if isinstance(value, bool):
        unit = UNIT.UNITLESS
    if unit is not None:
        g.add((node, QUDT.unit, unit))


def _named(rows) -> dict[int, dict]:
    """Index -> {id, uri} for a header table (FSM states or events)."""
    return {row.number: {"id": row.id, "uri": row.iri or None} for row in rows}


def _state_maps(header) -> tuple[dict[int, dict], dict[int, dict]]:
    return _named(header.fsm_states), _named(header.fsm_events)


def _slot_rows(rows) -> list[dict]:
    """Header slot messages as the dicts the occurrence builders read."""
    return [
        {
            "index": row.number,
            "id": row.id,
            "uri": row.iri or None,
            "constraint_uri": row.constraint_iri or None,
            "event_uri": row.event_iri or None,
            # A gate watches several constraints; each carries the error signal it is judged
            # by, so the members of an `until` can be read one at a time.
            "watched": [
                {
                    "id": w.id,
                    "uri": w.iri or None,
                    "error_id": w.error_id or None,
                    "tolerance_id": w.tolerance_id or None,
                }
                for w in row.watched
            ],
        }
        for row in rows
    ]


def _motion_meta(header) -> dict[int, dict]:
    """Motion index -> its slot metadata. The frame's active_motion selects it, so the same
    resolution works whatever coordinator drove the run."""
    return {
        motion.index: {
            "index": motion.index,
            "id": motion.id,
            "uri": motion.iri or None,
            "controllers": _slot_rows(motion.controllers),
            "monitors": _slot_rows(motion.monitors),
        }
        for motion in header.motions
    }


def _transition_rows(header) -> list[dict]:
    """Header transition messages as dicts; -1 endpoints mean 'not declared'."""
    return [
        {
            "id": row.id,
            "uri": row.iri or None,
            "from": row.from_state if row.from_state >= 0 else None,
            "to": row.to_state if row.to_state >= 0 else None,
            "event_index": row.event_index if row.event_index >= 0 else None,
            "event_indices": list(row.event_indices),
        }
        for row in header.fsm_transitions
    ]


def _state_meta(states: dict[int, dict], state_idx: int) -> dict | None:
    """The FSM state a frame was in -- coordinator context for the occurrence, not slot identity."""
    return states.get(state_idx)


def _slot_uri(slot: dict, *keys: str) -> rdflib.URIRef | None:
    for key in keys:
        value = slot.get(key)
        if value:
            return rdflib.URIRef(value)
    return None


def _instant_node(run_id: str, step: int) -> rdflib.URIRef:
    return _node(f"instant:{run_id}:{step}")


def _trs_node(run_id: str) -> rdflib.URIRef:
    """The run's tick scale: the one time reference system every position is counted on."""
    return _node(f"trs:{run_id}")


def _tick_scale(g: rdflib.Graph, run_id: str, nominal_period_ns) -> rdflib.URIRef:
    """The run's TRS, carrying the tick rate as the sensors metamodel's update rate.

    This is the only place a period reaches the graph: a step converts to seconds through the
    scale its own positions are counted on, without opening the frame log for one header field.
    """
    trs = _trs_node(run_id)
    if (trs, rdflib.RDF.type, TIME.TRS) in g:
        return trs
    g.add((trs, rdflib.RDF.type, TIME.TRS))
    if nominal_period_ns:
        rate = _node(f"quantity:{run_id}:tick_rate")
        g.add((trs, SENS["update-rate"], rate))
        g.add((rate, rdflib.RDF.type, QUDT.Quantity))
        g.add((rate, QUDT.hasQuantityKind, QKIND.Frequency))
        g.add((rate, QUDT.unit, UNIT.HZ))
        _literal(g, rate, QUDT.value, 1e9 / nominal_period_ns)
    return trs


def _instant(
    g: rdflib.Graph, run_id: str, step: int, trs: rdflib.URIRef, wall_ns=None
) -> rdflib.URIRef:
    """The instant at `step`, positioned on the run's tick scale. Idempotent."""
    node = _instant_node(run_id, step)
    if (node, TIME.inTimePosition, None) not in g:
        g.add((node, rdflib.RDF.type, TIME.Instant))
        position = rdflib.BNode()
        g.add((node, TIME.inTimePosition, position))
        g.add((position, TIME.numericPosition, rdflib.Literal(step)))
        g.add((position, TIME.hasTRS, trs))
    dt = _dt_literal(wall_ns)
    if dt is not None and (node, PROV.generatedAtTime, None) not in g:
        g.add((node, PROV.generatedAtTime, dt))
    return node


def _fired_transition(candidates: list, observed: set):
    """Which of the transitions between one pair of states actually fired.

    Transitions are identified by their own id, so two transitions between the same pair of states
    stay distinct. When a pair has several, the events observed around the state change decide
    which one it was; if that does not single one out, the caller records the state change without
    naming a transition rather than guessing.
    """
    if len(candidates) == 1:
        return candidates[0]
    matched = [
        transition
        for transition in candidates
        if observed & set(transition.get("event_indices") or [])
    ]
    return matched[0] if len(matched) == 1 else None


def _observation(
    g: rdflib.Graph,
    run_id: str,
    step,
    wall_ns,
    trs: rdflib.URIRef,
    sensor: rdflib.URIRef,
    kind: str,
    slot,
    role: str,
    prop: rdflib.URIRef,
    value,
    feature: rdflib.URIRef | None = None,
    unit: rdflib.URIRef | None = None,
) -> rdflib.URIRef:
    """One sosa:Observation of a slot's value at a frame -- an instance node, no new vocabulary.

    A sample is something we saw, not something the spec commanded: it stays an observation
    however long it holds, and its result time is the instant on the run's tick scale.
    """
    node = _scoped("observation", run_id, step, f"{kind}{slot}", role)
    g.add((node, rdflib.RDF.type, SOSA.Observation))
    g.add((node, SOSA.observedProperty, prop))
    g.add((node, SOSA.madeBySensor, sensor))
    if feature is not None:
        g.add((node, SOSA.hasFeatureOfInterest, feature))
    _result(g, node, value, unit)
    g.add((node, SOSA.resultTime, _instant(g, run_id, step, trs, wall_ns)))
    return node


def frame_observations(
    g: rdflib.Graph,
    run_id: str,
    header,
    frame: dict,
    *,
    signal_map: dict | None = None,
    quantity_iris: dict | None = None,
    satisfied: bool = False,
    units: dict | None = None,
) -> None:
    """sosa:Observations for one frame's active slots.

    Controller error/output and monitor values always; constraint satisfaction and quantities
    only when asked for. Sampled history takes the former, the live tier takes all of them --
    dense per-tick quantities would swamp the graph and the frame log already holds them.
    """
    sensor = rdflib.URIRef(prov_uri(header.producer_agent_id or "agent:controller_process"))
    # The scale itself is defined once, with the run; an observation only counts on it.
    trs = _trs_node(run_id)
    wall_ns = frame.get("timing", {}).get("wall_ns")
    step = frame["step"]
    meta = _motion_meta(header).get(frame.get("active_motion", -1), {})
    controllers = meta.get("controllers") or []
    monitors = meta.get("monitors") or []
    for idx, slot in enumerate(frame.get("constraints", [])):
        if not slot.get("active") or idx >= len(controllers):
            continue
        controller_uri = _slot_uri(controllers[idx], "uri")
        constraint_uri = _slot_uri(controllers[idx], "constraint_uri")
        signals = (signal_map or {}).get(str(controller_uri)) or {}
        for role, value in (("error", slot.get("error")), ("output", slot.get("output"))):
            prop = signals.get(role)
            if prop is None:
                continue
            _observation(
                g,
                run_id,
                step,
                wall_ns,
                trs,
                sensor,
                "c",
                idx,
                role,
                prop,
                float(value),
                feature=constraint_uri,
                unit=(units or {}).get(str(prop)),
            )
        if satisfied and constraint_uri is not None:
            _observation(
                g,
                run_id,
                step,
                wall_ns,
                trs,
                sensor,
                "c",
                idx,
                "satisfied",
                constraint_uri,
                bool(slot.get("satisfied")),
            )
    for idx, slot in enumerate(frame.get("monitors", [])):
        if not slot.get("active") or idx >= len(monitors):
            continue
        monitor_uri = _slot_uri(monitors[idx], "uri")
        if monitor_uri is None:
            continue
        _observation(
            g,
            run_id,
            step,
            wall_ns,
            trs,
            sensor,
            "m",
            idx,
            "value",
            monitor_uri,
            float(slot["value"]),
            unit=(units or {}).get(str(monitor_uri)),
        )
    for idx, (qid, iri) in enumerate(sorted((quantity_iris or {}).items())):
        if qid not in frame.get("quantities", {}):
            continue
        _observation(
            g,
            run_id,
            step,
            wall_ns,
            trs,
            sensor,
            "q",
            idx,
            "value",
            rdflib.URIRef(iri),
            float(frame["quantities"][qid]),
            unit=(units or {}).get(str(iri)),
        )


class IncrementalProjector:
    """One run's occurrence projection, fed an occurrence at a time.

    The runtime detects the edges and appends them; this holds the policy alone -- what an edge
    means, how long a span lasts, what caused it. The same projector drives a run being written
    and one being re-read, so there is one answer either way. Continuous scalars stay in the
    frame log; only semantic edges land in the graph, as activities nested by
    prov:wasInformedBy (run -> motion -> maintenance):

      * ms-prov:MotionExecution spans the interval one motion was in force for.
      * ms-prov:ConstraintMaintenance spans one interval a constraint the spec commands was
        held for -- a goal constraint satisfied, a gate's condition arming, a watched member
        of a gate holding. Unsatisfied needs no occurrence: it is the gap between two held
        spans, and each re-arm is its own maintenance, so a re-arm count is COUNT(*).
      * A plain prov:Activity at one instant marks control moving -- a transition, or the
        event that drove it. Neither is one of the four classes, so neither invents one. The
        heartbeat that drives the FSM is deliberately not recorded, because the frame log is a
        time series and every frame already is the tick.

    A condition with no authoring referent is not an activity at all: it stays a
    sosa:Observation (see `frame_observations`). Nothing is named after a state or a
    transition: the single prov:used points at the design IRI, whose own rdf:type says what
    kind of element it was.

    What a maintenance holds is an entity: it is prov:wasGeneratedBy the maintenance that
    achieved it, and prov:wasInvalidatedBy the activity that lost it.

    Occurrences are linked to the occurrence that informed them (prov:wasInformedBy), so
    "which monitor firing caused this transition" is a graph walk. A cause is written only
    where it is unique -- a missing link is honest, a guessed one poisons every query.

    Occurrences arrive in tick order and, within a tick, in the order the runtime emitted them:
    events first (a state change has to be able to name the event that drove it), then the
    entry, then the edges under it, then the sampled value set. A monitor's firing is settled at
    the tick's end, once that monitor's own edge for the tick has been applied.
    """

    def __init__(
        self,
        g: rdflib.Graph,
        run_id: str,
        header,
        *,
        signal_map: dict | None = None,
        declared_dwells: dict | None = None,
        units: dict | None = None,
    ):
        self.g = g
        self.run_id = run_id
        self.header = header
        self.run = rdflib.URIRef(prov_uri(f"run:{run_id}"))
        self.producer = rdflib.URIRef(
            prov_uri(header.producer_agent_id or "agent:controller_process")
        )
        self.trs = _tick_scale(g, run_id, header.nominal_period_ns)
        self.signal_map = signal_map or {}
        # Read to decide whether an arming is worth its member detail, never written: the
        # declared dwell belongs to the design graph and a query joins the two.
        self.declared_dwells = declared_dwells or {}
        self.units = units or {}
        self.states, self.events = _state_maps(header)
        self.motions = _motion_meta(header)
        # A watched member's band is a declared constant, not a per-tick signal: read here to
        # decide whether the member held, and likewise never written into the graph.
        self.constants = {c.id: c.value for c in header.constants}
        self.period_s = (header.nominal_period_ns or 0) / 1e9 or None
        # Indexed by state pair but holding every transition between that pair, so two
        # transitions between the same states driven by different events stay distinct.
        self.transitions: dict = {}
        for transition in _transition_rows(header):
            self.transitions.setdefault((transition.get("from"), transition.get("to")), []).append(
                transition
            )
        self.anchors: set = set()
        self.prev_state = None
        self.seen_events: set = set()
        # An event fires on one tick and the state change lands on the next, so the events that
        # could have caused a change span this tick and the previous one.
        self.frame_events: set = set()
        self.prev_frame_events: set = set()
        self.frame_event_uris: set = set()
        # Firings owed to the tick being read: a monitor fires once its edge for the tick has
        # been applied, so they are settled when the tick ends.
        self.pending_fires: dict = {}
        # Whether the tick being read entered a state, and the motion its occurrences name --
        # settled at the tick's end, because the tick a state is entered on still runs the
        # motion that is leaving.
        self.tick_entered = False
        self.tick_motion: dict | None = None
        # The last value each monitor reported, so a firing that carries no edge of its own
        # still records what the monitor read.
        self.monitor_value: dict = {}
        self.first_step: int | None = None
        self.last_step: int | None = None
        self.tick_wall = None
        # Wall time per anchored step, so an instant materialized at the end still carries one.
        self.step_wall: dict = {}
        self.activity_began: int | None = None
        # Open spans, closed when the opposite edge arrives or the run ends.
        self.open_activity: rdflib.URIRef | None = None
        # The active motion is set one tick after the state that runs it, so the motion in
        # force is resolved from the first intra-state frame; a span that never resolves one
        # falls back to the coordination state it spans.
        self.open_referent: rdflib.URIRef | None = None
        self.open_state_uri: rdflib.URIRef | None = None
        # The motion in force when the run ended -- what a failed run's outcome was lost to.
        self.last_activity: rdflib.URIRef | None = None
        self.open_constraint: dict = {}
        self.open_monitor: dict = {}
        # What each maintenance held, minted with it and invalidated if the hold is lost.
        self.goal_of: dict = {}
        # Per monitor slot, the arming in progress: when its condition first held, how many
        # times it broke since the activity began, and the member edges seen meanwhile. The
        # break count decides whether the member lanes are worth recording; it is never emitted
        # -- each arming is its own maintenance, so a consumer counts them.
        self.first_held: dict = {}
        self.rearm: dict = {}
        self.member_pending: dict = {}
        # Instance-level causes, kept only as long as they can still inform something.
        self.event_occ: dict = {}
        self.frame_event_occ: list = []
        self.monitor_occ_for_event: dict = {}

    # -- node construction -------------------------------------------------------------

    def _occurrence(self, cls: rdflib.URIRef, disc, wall_ns, step: int, *, span: bool):
        """One occurrence of `cls`, positioned on the run's tick scale.

        A span is the interval itself -- its beginning is this instant, its end is filled in
        when it closes. An instant-anchored occurrence names the one instant it happened at.
        The wall time it carries lives on that instant, not restated here.
        """
        node = _scoped("occurrence", self.run_id, wall_ns if wall_ns is not None else 0, disc)
        self.g.add((node, rdflib.RDF.type, cls))
        self.g.add((node, PROV.wasAssociatedWith, self.producer))
        instant = self._instant(step, wall_ns)
        self.g.add((node, TIME.hasBeginning if span else TIME.hasTime, instant))
        return node

    def _instant(self, step: int, wall_ns=None) -> rdflib.URIRef:
        self.anchors.add(step)
        return _instant(self.g, self.run_id, step, self.trs, wall_ns)

    def _close(self, node: rdflib.URIRef | None, step: int) -> None:
        """Close a span at `step`. Idempotent, so closing the run twice is harmless."""
        if node is None or (node, TIME.hasEnd, None) in self.g:
            return
        self.g.add((node, TIME.hasEnd, self._instant(step)))

    def _used(self, occ: rdflib.URIRef, referent: rdflib.URIRef) -> None:
        """The design element this occurrence carried out, named by its own design IRI."""
        self.g.add((occ, PROV.used, referent))

    def _informed_by(self, node: rdflib.URIRef, cause: rdflib.URIRef | None) -> None:
        if cause is not None:
            self.g.add((node, PROV.wasInformedBy, cause))

    def _lost(self, occ: rdflib.URIRef | None) -> None:
        """Record that what a maintenance held was lost, by the activity that broke it.

        The breaker is the event occurrence at this tick when exactly one fired, and the motion
        that was in force otherwise -- a missing cause is honest, a guessed one poisons the
        chain, and the enclosing motion is always true.
        """
        goal = self.goal_of.get(occ)
        if goal is None:
            return
        breaker = self.frame_event_occ[0] if len(self.frame_event_occ) == 1 else self.open_activity
        self._invalidated_by(goal, breaker)

    def _invalidated_by(self, goal: rdflib.URIRef, breaker: rdflib.URIRef | None) -> None:
        if breaker is not None:
            self.g.add((goal, PROV.wasInvalidatedBy, breaker))

    # -- occurrence builders -----------------------------------------------------------

    def _state_change(self, cur, state, entry, step, observed_events: set) -> None:
        """Close the motion that ended, record the control flow, open the one that began."""
        prev_state = self.prev_state
        self._finish_activity(step)
        control_flow = None
        if prev_state is not None:
            control_flow = self._control_flow(prev_state, cur, entry, step, observed_events)
        if state and state.get("uri"):
            occ = self._occurrence(MS_PROV.MotionExecution, f"s{cur}", entry, step, span=True)
            self._informed_by(occ, self.run)
            self._informed_by(occ, control_flow)
            self.open_activity = occ
            self.open_state_uri = rdflib.URIRef(state["uri"])

    def _resolve_motion(self, meta: dict) -> None:
        """Name the motion the open span was running, once the frame resolves one."""
        if self.open_activity is None or self.open_referent is not None:
            return
        motion_uri = _slot_uri(meta, "uri")
        if motion_uri is not None:
            self._used(self.open_activity, motion_uri)
            self.open_referent = motion_uri

    def _finish_activity(self, step: int) -> None:
        """Close the open motion span, naming the state it spanned if no motion ever resolved."""
        if self.open_activity is None:
            return
        if self.open_referent is None and self.open_state_uri is not None:
            self.g.add((self.open_activity, PROV.used, self.open_state_uri))
        self._close(self.open_activity, step)
        self.open_activity, self.open_referent, self.open_state_uri = None, None, None

    def _control_flow(self, prev_state, cur, entry, step, observed_events: set):
        tr = _fired_transition(self.transitions.get((prev_state, cur)) or [], observed_events)
        if not tr or not tr.get("uri"):
            return None
        occ = self._occurrence(
            PROV.Activity, f"t{tr.get('id', f'{prev_state}-{cur}')}", entry, step, span=False
        )
        self.g.add((occ, PROV.used, rdflib.URIRef(tr["uri"])))
        # The event that drove it survives as the link to its own occurrence, written only when
        # exactly one observed event matched what the transition declares.
        declared = set(tr.get("event_indices") or [])
        if tr.get("event_index") is not None:
            declared.add(tr["event_index"])
        fired = observed_events & declared
        if len(fired) == 1:
            self._informed_by(occ, self.event_occ.get(next(iter(fired))))
        return occ

    def _constraint_edge(self, controllers: list, occurrence: dict, wall, step: int) -> None:
        """Span a goal constraint's satisfied stretches, closing one on its falling edge.

        Unsatisfied gets no occurrence of its own: it is the gap between two satisfied spans.
        The value carried is the observed error, never the declared band.

        The runtime reports every latch as unsatisfied at a state entry, so a constraint already
        satisfied when its motion begins opens a span there. Without that its later loss -- the
        falling edge that fires a goal-lost event -- would close nothing and leave no trace.
        """
        idx = occurrence["index"]
        if idx >= len(controllers):
            return
        constraint_uri = _slot_uri(controllers[idx], "constraint_uri")
        if constraint_uri is None:  # only goal constraints, not pure regulation
            return
        ended = self.open_constraint.pop(idx, None)
        self._close(ended, step)
        if not occurrence["satisfied"]:
            self._lost(ended)
            return
        value = occurrence.get("value")
        self.open_constraint[idx] = self._maintenance(
            f"c{idx}", constraint_uri, None if value is None else float(value), wall, step
        )

    def _maintenance(self, disc, referent, value, wall, step: int, *, plan_step: bool = True):
        """One interval a commanded constraint was held for, and the goal it thereby held.

        `plan_step` is false for a gate's watched members: they are already steps in their own
        right, and this occurrence is a detail of the arming that wanted them rather than the
        span that carries out the step.
        """
        occ = self._occurrence(MS_PROV.ConstraintMaintenance, disc, wall, step, span=True)
        if plan_step:
            self._used(occ, referent)
        else:
            self.g.add((occ, PROV.used, referent))
        self._informed_by(occ, self.open_activity or self.run)
        _result(self.g, occ, value, self.units.get(str(referent)))
        goal = _scoped("goal", self.run_id, wall if wall is not None else 0, disc)
        self.g.add((goal, rdflib.RDF.type, PROV.Entity))
        self.g.add((goal, PROV.wasGeneratedBy, occ))
        self.goal_of[occ] = goal
        return occ

    def _monitor_edge(self, monitors: list, occurrence: dict, wall, step: int) -> None:
        """Maintain one span per arming of a monitor, closed when it breaks or fires.

        Every arming is its own maintenance, so how often a gate re-armed is COUNT(*) over
        them. The observed dwell is the interval this emits; the *declared* dwell stays in the
        design graph, where a query joins it. The two are compared, never copied together.
        """
        idx = occurrence["index"]
        if idx >= len(monitors):
            return
        slot = monitors[idx]
        monitor_uri = _slot_uri(slot, "uri", "monitor_uri")
        self.monitor_value[idx] = occurrence.get("value")
        if occurrence["satisfied"]:
            if monitor_uri is not None:
                self.first_held[idx] = step
                self.open_monitor[idx] = self._maintenance(f"m{idx}", monitor_uri, None, wall, step)
            # A monitor that names no event (a flag, a `while` term) fires on its rising edge.
            if not slot.get("event_uri"):
                self.pending_fires[idx] = slot
        else:
            self.rearm[idx] = self.rearm.get(idx, 0) + 1
            self.first_held.pop(idx, None)
            broken = self.open_monitor.pop(idx, None)
            self._close(broken, step)
            self._lost(broken)

    def _fire_monitor(self, idx: int, slot: dict, wall, step: int) -> None:
        monitor_uri = _slot_uri(slot, "uri", "monitor_uri")
        if monitor_uri is None:
            return
        rearm = self.rearm.get(idx, 0)
        occ = self.open_monitor.pop(idx, None)
        if occ is None:  # fired without an observed arming edge: the firing tick is the arming
            occ = self._maintenance(f"m{idx}", monitor_uri, None, wall, step)
        self._close(occ, step)
        value = self.monitor_value.get(idx)
        _result(
            self.g, occ, None if value is None else float(value), self.units.get(str(monitor_uri))
        )
        if self._interesting(monitor_uri, rearm, step):
            self._emit_members(idx, occ, wall, step)
        self.member_pending.pop(idx, None)
        self.rearm[idx] = 0
        self.first_held.pop(idx, None)
        if slot.get("event_uri"):
            self.monitor_occ_for_event[slot["event_uri"]] = occ
        if self._interesting(monitor_uri, rearm, step):
            self._emit_members(idx, occ, wall, step)
        self.member_pending.pop(idx, None)
        self.rearm[idx] = 0
        self.first_held.pop(idx, None)
        if slot.get("event_uri"):
            self.monitor_occ_for_event[slot["event_uri"]] = occ

    def _interesting(self, monitor_uri: rdflib.URIRef, rearm: int, step: int) -> bool:
        """Whether this arming's member behaviour is worth recording.

        Either the watched condition broke and re-armed, or the gate held its activity far
        longer than its own declared dwell. The second clause is the one that catches a gate
        waiting on a single slow term, where nothing re-armed and the member lanes are exactly
        what a reader needs.
        """
        if rearm > 0:
            return True
        declared = self.declared_dwells.get(str(monitor_uri))
        if declared is None or not declared or self.period_s is None or self.activity_began is None:
            return False
        return (step - self.activity_began) * self.period_s > declared * SLOW_GATE_FACTOR

    def _member_edge(self, monitors: list, occurrence: dict, step: int) -> None:
        """Buffer one held/lost edge of a gate's watched member.

        They become occurrences only if the arming turns out interesting, and are capped per
        member -- the armings themselves are not, so a truncated series stays recognisable.
        """
        idx, ordinal = occurrence["index"], occurrence["member"]
        if idx >= len(monitors):
            return
        watched = monitors[idx].get("watched") or []
        if ordinal >= len(watched):
            return
        member = watched[ordinal]
        if not member.get("uri"):
            return
        pending = self.member_pending.setdefault(idx, {}).setdefault(member["id"], [])
        if len(pending) >= MEMBER_EDGE_LIMIT:
            return
        pending.append(
            (bool(occurrence["satisfied"]), member["uri"], float(occurrence["value"]), step)
        )

    def _emit_members(self, idx: int, monitor_occ: rdflib.URIRef, wall, fired: int) -> None:
        """Span each buffered held stretch of a gate's members, hung off the arming that wanted
        them. A stretch ends at the next not-held edge, or at the firing if it never broke."""
        for member_id, edges in (self.member_pending.get(idx) or {}).items():
            for order, (held, member_uri, error, step) in enumerate(edges):
                if not held:
                    continue
                ends_at = next((e[3] for e in edges[order + 1 :] if not e[0]), fired)
                occ = self._maintenance(
                    f"w{idx}-{member_id}-{order}",
                    rdflib.URIRef(member_uri),
                    error,
                    wall,
                    step,
                    plan_step=False,
                )
                self._close(occ, ends_at)
                self._informed_by(monitor_occ, occ)

    def _event(self, monitors: list, occurrence: dict, step: int) -> None:
        """Record one event firing, and owe the tick's end every monitor that names it."""
        eidx = occurrence["index"]
        event = self.events.get(eidx) or {}
        self.frame_events.add(eidx)
        uri = event.get("uri")
        if not uri:  # the shape wants a referent; an unnamed event has none
            return
        self.frame_event_uris.add(uri)
        for idx, slot in enumerate(monitors):
            if slot.get("event_uri") == uri:
                self.pending_fires[idx] = slot
        ekey = (eidx, occurrence["wall_ns"])
        if ekey in self.seen_events:
            return
        self.seen_events.add(ekey)
        occ = self._occurrence(PROV.Activity, f"e{eidx}", occurrence["wall_ns"], step, span=False)
        self.g.add((occ, PROV.used, rdflib.URIRef(uri)))
        self._informed_by(occ, self.monitor_occ_for_event.pop(uri, None))
        self.event_occ[eidx] = occ
        self.frame_event_occ.append(occ)

    def _entry(self, occurrence: dict, wall, step: int) -> None:
        """Close what the outgoing motion held and open the span the new state runs.

        Slot indices are motion-local, so no arming carries across the boundary. The spans open
        under the outgoing motion end here -- dropping them instead would leave a beginning with
        no end in every query that asks how long one held.
        """
        for occ in (*self.open_constraint.values(), *self.open_monitor.values()):
            self._close(occ, step)
        self.open_constraint, self.open_monitor = {}, {}
        self.first_held, self.rearm, self.member_pending = {}, {}, {}
        self.activity_began = step
        cur = occurrence["fsm_state"]
        from_state = occurrence["from_state"]
        self.prev_state = None if from_state < 0 else from_state
        self._state_change(
            cur,
            _state_meta(self.states, cur),
            occurrence["state_since_wall_ns"] or wall,
            step,
            self.frame_events | self.prev_frame_events,
        )
        self.prev_state = cur

    # -- driving -----------------------------------------------------------------------

    def _motion(self, occurrence: dict) -> dict:
        return self.motions.get(occurrence.get("active_motion", -1), {})

    def _end_tick(self) -> None:
        """Settle the tick just read: fire what its events armed, and age its event set.

        An event fires on one tick and the state change lands on the next, so the events a
        change could have been caused by span this tick and the one before it.
        """
        if not self.tick_entered and self.tick_motion is not None:
            self._resolve_motion(self.tick_motion)
        for idx, slot in self.pending_fires.items():
            self._fire_monitor(idx, slot, self.tick_wall, self.last_step)
        self.pending_fires = {}
        self.tick_entered, self.tick_motion = False, None
        self.prev_frame_events = self.frame_events
        self.frame_events, self.frame_event_uris, self.frame_event_occ = set(), set(), []

    def feed(self, occurrence: dict) -> set:
        """Project one occurrence; return the steps it anchored an occurrence at."""
        step = occurrence["step"]
        if self.last_step is not None and step != self.last_step:
            self._end_tick()
        if self.first_step is None:
            self.first_step = step
        self.last_step = step
        wall = occurrence["wall_ns"]
        self.step_wall.setdefault(step, wall)
        self.tick_wall = wall
        before = set(self.anchors)
        kind = occurrence["kind"]
        meta = self._motion(occurrence)
        monitors = meta.get("monitors") or []
        if kind == "STATE_CHANGE":
            self.tick_entered = True
            self._entry(occurrence, wall, step)
            return self.anchors - before
        self.tick_motion = meta
        if kind == "EVENT":
            self._event(monitors, occurrence, step)
        elif kind == "CONSTRAINT_EDGE":
            self._constraint_edge(meta.get("controllers") or [], occurrence, wall, step)
        elif kind == "MONITOR_EDGE":
            self._monitor_edge(monitors, occurrence, wall, step)
        elif kind == "MEMBER_EDGE":
            self._member_edge(monitors, occurrence, step)
        elif kind == "SAMPLE":
            self._sample(occurrence)
        return self.anchors - before

    def _sample(self, occurrence: dict) -> None:
        """The value set the runtime sampled: observations of every slot the motion drove."""
        frame = occurrence.get("frame")
        if frame is None:
            return
        for idx, slot in enumerate(frame.get("monitors") or []):
            if slot.get("active"):
                self.monitor_value[idx] = slot.get("value")
        frame_observations(
            self.g, self.run_id, self.header, frame, signal_map=self.signal_map, units=self.units
        )

    def close(self) -> set:
        """Close every span still open at the run's last occurrence.

        Without this the final occupancy silently drops out of every duration query, and the
        sum of activity durations no longer equals the run.
        """
        if self.last_step is None:
            return self.anchors
        self._end_tick()
        for occ in (*self.open_constraint.values(), *self.open_monitor.values()):
            self._close(occ, self.last_step)
        self.last_activity = self.open_activity
        self._finish_activity(self.last_step)
        return self.anchors


def _project_occurrences(
    g: rdflib.Graph, run_id: str, header, occurrences, model_paths
) -> IncrementalProjector:
    """Project a whole occurrence stream; return the projector, which holds the steps that
    carry an occurrence (the instants worth materializing)."""
    projector = IncrementalProjector(g, run_id, header, **model_facts(model_paths))
    for occurrence in occurrences:
        projector.feed(occurrence)
    projector.close()
    return projector


def model_facts(model_paths) -> dict:
    """What the projector reads out of the design graph: the declared dwells it judges an
    arming by, the units it labels a value with, and the signals a sample observes."""
    return {
        "declared_dwells": declared_dwells_from_paths(model_paths),
        "units": units_from_paths(model_paths),
        "signal_map": signal_map_from_paths(model_paths),
    }


def _add_rec_timing(g: rdflib.Graph, run_dir: Path, files: dict, run: rdflib.URIRef) -> dict:
    """Stamp the run with the wall-clock lifecycle rec observed, and return that lifecycle."""
    rec_path = run_dir / files.get("rec", "rec.ld.json")
    if not rec_path.exists():
        return {}
    lifecycle = rec_run_lifecycle(rdflib.Graph().parse(rec_path, format="json-ld"))
    for key, pred in (("started_time", PROV.startedAtTime), ("completed_time", PROV.endedAtTime)):
        if lifecycle.get(key):
            g.add((run, pred, rdflib.Literal(lifecycle[key], datatype=rdflib.XSD.dateTime)))
    return lifecycle


def bind_namespaces(g, run_id: str, *, fsm_namespace: str = "") -> None:
    """Bind the runtime graph's prefixes, so every emitted node displays as a CURIE."""
    for prefix, ns in {
        "prov": PROV,
        "sosa": SOSA,
        "bdd": BDD,
        "agn": AGN,
        "obs": OBS,
        "exec": EXEC,
        "msrun": MSRUN,
        "ms-prov": MS_PROV,
        "time": TIME,
        "dcterms": DCTERMS,
        "sens": SENS,
        "qudt": QUDT,
        "qkind": QKIND,
        "unit": UNIT,
        # Run and file entities are minted in the shared provenance space, run-scoped, so two
        # runs of one generation never collapse onto the same node.
        "ent": rdflib.Namespace(f"{MSPROV}entity/run/{run_id}/"),
        "run": rdflib.Namespace(f"{MSPROV}run/"),
        "trs": rdflib.Namespace(f"{MSRUN}trs/"),
        "mspact": rdflib.Namespace(f"{MSPROV}activity/"),
        "mspagent": rdflib.Namespace(f"{MSPROV}agent/"),
    }.items():
        g.bind(prefix, ns)
    # Compact the per-run node families and model FSM nodes into CURIEs by binding a prefix at
    # each family's `<family>/<run_id>/` boundary (locals are slash-free — see _scoped).
    for prefix, family in (
        ("inst", "instant"),
        ("cs", "controller-sample"),
        ("mons", "monitor-sample"),
        ("sig", "signal"),
        ("occ", "occurrence"),
        ("goal", "goal"),
        ("qty", "quantity"),
        ("obsv", "observation"),
    ):
        g.bind(prefix, rdflib.Namespace(f"{MSRUN}{family}/{run_id}/"))
    if fsm_namespace:
        g.bind("mfsm", rdflib.Namespace(fsm_namespace))


def _run_document(
    g: rdflib.Graph,
    run_dir: Path,
    files: dict,
    header,
    run_id: str,
    last_activity: rdflib.URIRef | None = None,
) -> dict:
    """The run-level statement of the document: the run, its agents, the files it read and
    wrote, and this graph's own provenance. Returns the rec lifecycle it read.

    `files` is the archive's own mapping, so what the document says a file is at is what the
    manifest says. A run still executing has no manifest yet: it states the paths the runtime
    itself wrote to, and the runner restates them once the archive has settled.
    """
    # The run, the agents and the execution activity are shared provenance concepts: emit the
    # same canonical msprov IRIs the codegen and rec graphs use so all three join on one node
    # per concept (rather than three parallel ones). File entities are run-scoped, so two runs
    # of one generation never collapse onto the same entity.
    run = rdflib.URIRef(prov_uri(f"run:{run_id}"))
    activity = rdflib.URIRef(prov_uri(header.activity_id or "activity:controller_execution"))
    producer = rdflib.URIRef(prov_uri(header.producer_agent_id or "agent:controller_process"))
    runtime = rdflib.URIRef(prov_uri(header.runtime_agent_id or "agent:runtime"))
    occurrences = rdflib.URIRef(run_entity_uri(run_id, "occurrences"))
    frame_log = rdflib.URIRef(run_entity_uri(run_id, "frame_log"))
    proto_entity = rdflib.URIRef(run_entity_uri(run_id, "frame_log_proto"))
    model_entity = rdflib.URIRef(run_entity_uri(run_id, "model_jsonld"))
    provenance_entity = rdflib.URIRef(run_entity_uri(run_id, "provenance_jsonld"))

    # The run is something that happened over an interval, never an entity.
    g.add((run, rdflib.RDF.type, MS_PROV.TaskExecution))
    g.add((run, PROV.used, model_entity))
    g.add((run, PROV.wasAssociatedWith, producer))
    g.add((activity, rdflib.RDF.type, PROV.Activity))
    # Simulated vs real is the model's declaration, not an assumption and not a substring match.
    g.add(
        (
            activity,
            rdflib.RDF.type,
            BDD.SimulatedExecution if header.simulated else BDD.ScenarioExecution,
        )
    )
    g.add((activity, PROV.wasAssociatedWith, producer))
    g.add((activity, PROV.used, proto_entity))
    g.add((activity, PROV.used, model_entity))
    g.add((activity, PROV.used, provenance_entity))
    # obs:ObservationProvider is a prov:SoftwareAgent by axiom; co-typing it here would only
    # restate what the metamodel derives.
    g.add((producer, rdflib.RDF.type, OBS.ObservationProvider))
    g.add((producer, PROV.actedOnBehalfOf, runtime))
    g.add((runtime, rdflib.RDF.type, PROV.SoftwareAgent))
    if header.simulated:
        g.add((runtime, rdflib.RDF.type, EXEC.Simulation))
    # The stream the graph is projected from is always there -- it is what is being read.
    g.add((occurrences, rdflib.RDF.type, PROV.Entity))
    g.add((occurrences, PROV.wasGeneratedBy, activity))
    g.add(
        (
            occurrences,
            PROV.atLocation,
            rdflib.URIRef(os.path.relpath(files.get("occurrences", OCCURRENCE_REL), "runtime")),
        )
    )
    if files.get("frame_log"):
        g.add((frame_log, rdflib.RDF.type, PROV.Entity))
        g.add((frame_log, PROV.wasGeneratedBy, activity))
        g.add(
            (
                frame_log,
                PROV.atLocation,
                rdflib.URIRef(os.path.relpath(files["frame_log"], "runtime")),
            )
        )
    for entity, key in (
        (proto_entity, "frame_log_proto"),
        (model_entity, "model"),
        (provenance_entity, "provenance"),
    ):
        rel = files.get(key)
        if rel and (run_dir / rel).exists():
            g.add((entity, rdflib.RDF.type, PROV.Entity))
            g.add((entity, PROV.atLocation, rdflib.URIRef(os.path.relpath(rel, "runtime"))))
    # The run id is already the run's own IRI, and the frame count is the last instant's position.
    g.add((run, DCTERMS.hasVersion, rdflib.Literal(RUNTIME_RDF_CONTRACT_VERSION)))
    lifecycle = _add_rec_timing(g, run_dir, files, run)

    # Provenance of this runtime.ttl document itself: one projection of the occurrence stream the
    # run appended to, by the same agent the header names as the run's observer.
    runtime_doc = rdflib.URIRef(run_entity_uri(run_id, "runtime_ttl"))
    projection = rdflib.URIRef(prov_uri(PROJECTION_ACTIVITY))
    generated_at = _dt_literal(time.time_ns())
    g.add((runtime_doc, rdflib.RDF.type, PROV.Entity))
    g.add((runtime_doc, PROV.atLocation, rdflib.URIRef("runtime.ttl")))
    g.add((runtime_doc, PROV.wasGeneratedBy, projection))
    g.add((runtime_doc, PROV.wasDerivedFrom, occurrences))
    g.add((runtime_doc, PROV.generatedAtTime, generated_at))
    g.add((projection, rdflib.RDF.type, PROV.Activity))
    g.add((projection, PROV.used, occurrences))
    g.add((projection, PROV.wasAssociatedWith, producer))
    g.add((projection, PROV.endedAtTime, generated_at))
    # A run that did not complete never produced what it was meant to: the outcome is an entity
    # nothing generated, lost to whatever was in force when the run stopped.
    if lifecycle.get("status") in _FAILED_STATUSES and last_activity is not None:
        outcome = rdflib.URIRef(run_entity_uri(run_id, "outcome"))
        g.add((outcome, rdflib.RDF.type, PROV.Entity))
        g.add((outcome, PROV.wasInvalidatedBy, last_activity))
    return lifecycle


def _materialize_extent(g: rdflib.Graph, run_id: str, projector: IncrementalProjector) -> None:
    """Give every anchored step its instant, and bound the run by the first and the last.

    Only the steps that carry an occurrence get one: the dense per-tick curve -- every frame,
    all continuous scalars -- stays in the frame log for numeric analysis.
    """
    if projector.first_step is None:
        return
    trs = _trs_node(run_id)
    for step in projector.anchors | {projector.first_step, projector.last_step}:
        _instant(g, run_id, step, trs, projector.step_wall.get(step))
    run = rdflib.URIRef(prov_uri(f"run:{run_id}"))
    g.add((run, TIME.hasBeginning, _instant_node(run_id, projector.first_step)))
    g.add((run, TIME.hasEnd, _instant_node(run_id, projector.last_step)))


def occurrence_log(run_dir: Path, files: dict) -> Path:
    """The occurrence stream this run recorded, wherever the archive put it."""
    return run_dir / (files.get("occurrences") or OCCURRENCE_REL)


def project_runtime(run_dir: Path | str) -> rdflib.Graph:
    """The whole runtime graph of an archived run, re-read from its occurrence stream."""
    run_dir, manifest = load_manifest(run_dir)
    files = manifest.get("files", {})
    stream = occurrence_log(run_dir, files)
    if not stream.exists():
        raise ArchiveError(f"{stream}: the run recorded no occurrence stream to project")
    # The run's contract comes from the stream itself, not a companion artifact.
    contract = frame_log_pb.read_contract(stream)
    run_id = manifest["run_id"]
    g = rdflib.Graph()
    bind_namespaces(g, run_id, fsm_namespace=contract.header.fsm_namespace)
    projector = _project_occurrences(
        g,
        run_id,
        contract.header,
        frame_log_pb.occurrence_records(stream, contract),
        _model_paths(run_dir, manifest),
    )
    _run_document(g, run_dir, files, contract.header, run_id, projector.last_activity)
    _materialize_extent(g, run_id, projector)
    return g


def write_runtime_ttl(run_dir: Path | str) -> Path:
    """Re-project an archived run's occurrence stream over its runtime.ttl."""
    run_dir = Path(run_dir)
    graph = project_runtime(run_dir)
    manifest_path = run_dir / "manifest.json"
    runtime_rel = RUNTIME_TTL_REL
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        runtime_rel = manifest.get("files", {}).get("runtime_ttl") or runtime_rel
    out = run_dir / runtime_rel
    out.parent.mkdir(parents=True, exist_ok=True)
    graph.serialize(out, format="turtle")
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        manifest.setdefault("files", {})["runtime_ttl"] = runtime_rel
        record_runtime_ttl_with_rec(run_dir, manifest, out)
        manifest_path.write_text(json.dumps(manifest, indent=4) + "\n")
    return out


def record_runtime_ttl_with_rec(run_dir: Path, manifest: dict, runtime_ttl: Path) -> None:
    """Record the projected graph as an artefact of the projection, with its digest.

    Called once the file is final: rec records a checksum, so a file still being appended to
    would be recorded as something it is no longer.
    """
    try:
        from motion_spec.introspection.provenance import ensure_local_rec_importable

        ensure_local_rec_importable()
        from rec import Run
        from rec.observers import FileObserver
    except ImportError as exc:
        raise RuntimeError("REC is required to update runtime.ttl provenance") from exc

    files = manifest.get("files", {})
    rec_path = run_dir / files.get("rec", "rec.ld.json")
    if not rec_path.exists():
        return
    run_id = manifest.get("run_id")
    # The same agent the runtime document names, read from the stream's own header, so the two
    # records describe one activity rather than two.
    header = frame_log_pb.read_contract(occurrence_log(run_dir, files)).header
    observer = FileObserver(rec_path, run_iri=prov_uri(f"run:{run_id}"))
    run = Run(observers=[observer], run_id=run_id)
    run.add_activity(
        prov_uri(PROJECTION_ACTIVITY),
        rec_types(["prov:Activity"]),
        associated_with=prov_uri(header.producer_agent_id or "agent:controller_process"),
    )
    run.add_artefact(
        str(runtime_ttl.resolve()),
        gen_activity=prov_uri(PROJECTION_ACTIVITY),
        archive_path=str(runtime_ttl.relative_to(run_dir)),
        title="runtime_ttl",
        sha256=sha256_file(runtime_ttl),
        size_bytes=runtime_ttl.stat().st_size,
    )
    observer.close()


class JournaledGraph(rdflib.Graph):
    """A graph that also appends every triple it gains to an open N-Triples journal.

    The journal is the crash record: it is complete up to the last occurrence projected, where
    the Turtle beside it is only as fresh as the last rewrite. Removals are not journaled -- the
    only triples ever retracted are the document's own, which the next rewrite restates.
    """

    def __init__(self, journal, **kwargs):
        super().__init__(**kwargs)
        self.journal = journal

    def add(self, triple):
        self.journal.write(_nt_row(triple))
        self.journal.flush()
        return super().add(triple)


class RuntimeGraphWriter:
    """Projects a run's occurrence stream into `runtime/runtime.ttl` while the run executes.

    One projector, fed from the stream the runtime is appending to: `runtime.nt` gains a line
    per triple as it is added and `runtime.ttl` is replaced atomically at most every
    `TTL_REWRITE_INTERVAL_S` and at close, so a run that dies leaves a graph complete up to its
    last occurrence rather than nothing at all.
    """

    def __init__(
        self,
        run_dir: Path | str,
        *,
        run_id: str,
        occurrence_log: Path | str,
        model_paths=(),
        poll_s: float = 0.2,
    ):
        self.run_dir = Path(run_dir)
        self.run_id = run_id
        self.occurrence_log = Path(occurrence_log)
        self.model_paths = list(model_paths)
        self.poll_s = poll_s
        self.ttl_path = self.run_dir / RUNTIME_TTL_REL
        self.nt_path = self.run_dir / RUNTIME_NT_REL
        self.graph: JournaledGraph | None = None
        self.projector: IncrementalProjector | None = None
        self._document: set = set()
        self._contract = None
        self._fh = None
        self._journal = None
        self._offset = 0
        self._written_at = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._follow, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> Path | None:
        """Drain what the run appended last, close every open span, and write the final Turtle."""
        self._stop.set()
        self._thread.join()
        if self.projector is None:
            return None
        self.projector.close()
        _materialize_extent(self.graph, self.run_id, self.projector)
        self._write_ttl()
        if self._fh is not None:
            self._fh.close()
            self._fh = None
        return self.ttl_path

    def rewrite(self, files: dict) -> Path | None:
        """Restate the document with the paths the archive settled on, and write it out.

        Only the run-level statement is replaced; every projected occurrence stands. Without
        this the graph would keep naming files at the paths the runtime wrote them to, which
        archiving has since moved.
        """
        if self.graph is None:
            return None
        for triple in self._document:
            self.graph.remove(triple)
        self._document = self._state_document(files)
        self._write_ttl()
        if self._journal is not None:
            self._journal.close()
            self._journal = None
        return self.ttl_path

    def _state_document(self, files: dict) -> set:
        document = rdflib.Graph()
        _run_document(
            document,
            self.run_dir,
            files,
            self._contract.header,
            self.run_id,
            None if self.projector is None else self.projector.last_activity,
        )
        for triple in document:
            self.graph.add(triple)
        return set(document)

    def _follow(self) -> None:
        while not self._stop.wait(self.poll_s):
            self._drain()
        self._drain()

    def _open(self) -> bool:
        """True once the stream's header record is complete and the projector is built."""
        if self.projector is not None:
            return True
        if not self.occurrence_log.exists():
            return False
        try:
            self._contract = frame_log_pb.read_contract(self.occurrence_log)
        except (ArchiveError, DecodeError):
            return False  # header still being written
        self.nt_path.parent.mkdir(parents=True, exist_ok=True)
        self._journal = self.nt_path.open("w", encoding="utf-8")
        self.graph = JournaledGraph(self._journal)
        bind_namespaces(self.graph, self.run_id, fsm_namespace=self._contract.header.fsm_namespace)
        self.projector = IncrementalProjector(
            self.graph, self.run_id, self._contract.header, **model_facts(self.model_paths)
        )
        self._document = self._state_document(
            {"occurrences": os.path.relpath(self.occurrence_log, self.run_dir)}
        )
        self._fh = self.occurrence_log.open("rb")
        return True

    def _drain(self) -> None:
        if not self._open():
            return
        records, self._offset = frame_log_pb.stream_occurrences(
            self._fh, self._contract, self._offset
        )
        for record in records:
            self.projector.feed(record)
        if records and time.monotonic() - self._written_at >= TTL_REWRITE_INTERVAL_S:
            self._write_ttl()

    def _write_ttl(self) -> None:
        """Replace the Turtle in one step, so a reader never finds it half written."""
        self.ttl_path.parent.mkdir(parents=True, exist_ok=True)
        pending = self.ttl_path.with_suffix(".ttl.tmp")
        self.graph.serialize(pending, format="turtle")
        os.replace(pending, self.ttl_path)
        self._written_at = time.monotonic()
