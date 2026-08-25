# SPDX-License-Identifier: MPL-2.0
"""Recover runtime RDF from archive-local run data."""

from __future__ import annotations

import json
import math
import os
import time
from datetime import datetime, timezone
from decimal import Decimal
from pathlib import Path

import rdflib

from motion_spec.introspection.archive import load_manifest, sha256_file
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
# Events are the only trigger kind the runtime actually emits; state/constraint/monitor edges
# are synthesized here from the per-tick frame scan (see _project_occurrences).
KIND_EVENT = 1
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
    _literal(g, node, SOSA.hasSimpleResult, value)
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
        )


class IncrementalProjector:
    """One run's occurrence projection, fed a frame at a time.

    Owns the state the per-tick scan carries across frames, so a batch replay and a live
    dashboard session drive the identical projection code. Continuous scalars stay in the
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

    Edge detection resets at state boundaries: slot indices are motion-local (slot i is a
    different controller under a different motion), so only intra-state comparison is valid.

    With `sample_interval_s` set, controller and monitor values are additionally sampled at
    that spacing in sim time. Left None (the archive path) no observations are emitted.
    """

    def __init__(
        self,
        g: rdflib.Graph,
        run_id: str,
        header,
        *,
        sample_interval_s: float | None = None,
        signal_map: dict | None = None,
        declared_dwells: dict | None = None,
    ):
        self.g = g
        self.run_id = run_id
        self.header = header
        self.run = rdflib.URIRef(prov_uri(f"run:{run_id}"))
        self.producer = rdflib.URIRef(
            prov_uri(header.producer_agent_id or "agent:controller_process")
        )
        self.trs = _tick_scale(g, run_id, header.nominal_period_ns)
        self.sample_interval_s = sample_interval_s
        self.signal_map = signal_map or {}
        # Read to decide whether an arming is worth its member detail, never written: the
        # declared dwell belongs to the design graph and a query joins the two.
        self.declared_dwells = declared_dwells or {}
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
        self.prev_csat: list | None = None
        self.prev_msat: list | None = None
        self.seen_events: set = set()
        # An event fires on one tick and the state change lands on the next, so the events that
        # could have caused a change span this frame and the previous one.
        self.prev_frame_events: set = set()
        self.frame_event_uris: set = set()
        self.last_sample_t: float | None = None
        self.last_step: int | None = None
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
        self.member_prev: dict = {}
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

    def _constraint_edges(
        self, controllers: list, csat: list, prev: list, frame: dict, wall, step: int
    ) -> None:
        """Span each goal constraint's satisfied stretches, closing one on its falling edge.

        Unsatisfied gets no occurrence of its own: it is the gap between two satisfied spans.
        The value carried is the observed error, never the declared band.

        Entry passes an all-unsatisfied `prev`, so a constraint that is already satisfied when
        its motion begins opens a span there. Without that its later loss -- the falling edge
        that fires a goal-lost event -- would close nothing and leave no trace of the loss.
        """
        for idx, now in enumerate(csat):
            if idx >= len(prev) or idx >= len(controllers) or now == prev[idx]:
                continue
            constraint_uri = _slot_uri(controllers[idx], "constraint_uri")
            if constraint_uri is None:  # only goal constraints, not pure regulation
                continue
            ended = self.open_constraint.pop(idx, None)
            self._close(ended, step)
            if not now:
                self._lost(ended)
                continue
            value = frame["constraints"][idx].get("error")
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
        _literal(self.g, occ, SOSA.hasSimpleResult, value)
        goal = _scoped("goal", self.run_id, wall if wall is not None else 0, disc)
        self.g.add((goal, rdflib.RDF.type, PROV.Entity))
        self.g.add((goal, PROV.wasGeneratedBy, occ))
        self.goal_of[occ] = goal
        return occ

    def _monitor_edges(self, monitors: list, msat: list, frame: dict, wall, step: int) -> None:
        """Maintain one span per arming of each monitor, closed when it breaks or fires.

        Every arming is its own maintenance, so how often a gate re-armed is COUNT(*) over
        them. The observed dwell is the interval this emits; the *declared* dwell stays in the
        design graph, where a query joins it. The two are compared, never copied together.
        """
        prev = self.prev_msat
        for idx, now in enumerate(msat):
            if idx >= len(monitors):
                continue
            monitor_uri = _slot_uri(monitors[idx], "uri", "monitor_uri")
            was = None if prev is None or idx >= len(prev) else prev[idx]
            if now and not was and monitor_uri is not None:
                self.first_held[idx] = step
                self.open_monitor[idx] = self._maintenance(f"m{idx}", monitor_uri, None, wall, step)
            elif was and not now:
                self.rearm[idx] = self.rearm.get(idx, 0) + 1
                self.first_held.pop(idx, None)
                broken = self.open_monitor.pop(idx, None)
                self._close(broken, step)
                self._lost(broken)
            self._watch_members(idx, monitors[idx], frame, step)
            # A monitor that names an event has fired when that event fired; one that names
            # none (a flag, a `while` term) is recorded on the raw rising edge instead.
            event_uri = monitors[idx].get("event_uri")
            fired = event_uri in self.frame_event_uris if event_uri else bool(now and was is False)
            if fired:
                self._fire_monitor(idx, monitors[idx], frame, wall, step)

    def _fire_monitor(self, idx: int, slot: dict, frame: dict, wall, step: int) -> None:
        monitor_uri = _slot_uri(slot, "uri", "monitor_uri")
        if monitor_uri is None:
            return
        rearm = self.rearm.get(idx, 0)
        occ = self.open_monitor.pop(idx, None)
        if occ is None:  # fired without an observed arming edge: the firing tick is the arming
            occ = self._maintenance(f"m{idx}", monitor_uri, None, wall, step)
        self._close(occ, step)
        slots = frame.get("monitors") or []
        value = slots[idx].get("value") if idx < len(slots) else None
        _literal(self.g, occ, SOSA.hasSimpleResult, None if value is None else float(value))
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

    def _watch_members(self, idx: int, slot: dict, frame: dict, step: int) -> None:
        """Buffer the satisfied/unsatisfied edges of a gate's watched members.

        They become occurrences only if the arming turns out interesting, and are capped per
        member -- the armings themselves are not, so a truncated series stays recognisable.
        """
        quantities = frame.get("quantities") or {}
        for member in slot.get("watched") or []:
            error_id, member_uri = member.get("error_id"), member.get("uri")
            if not error_id or not member_uri:
                continue
            error = quantities.get(error_id)
            band = quantities.get(
                member.get("tolerance_id"), self.constants.get(member.get("tolerance_id"))
            )
            if error is None or band is None:
                continue
            held = abs(float(error)) <= float(band)
            key = (idx, member["id"])
            was = self.member_prev.get(key)
            self.member_prev[key] = held
            if was is None or was == held:
                continue
            pending = self.member_pending.setdefault(idx, {}).setdefault(member["id"], [])
            if len(pending) >= MEMBER_EDGE_LIMIT:
                continue
            pending.append((held, member_uri, float(error), step))

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

    def _triggers(self, frame: dict, step: int) -> None:
        for trigger in frame.get("triggers", []):
            if trigger.get("kind") != KIND_EVENT:
                continue
            eidx = trigger.get("idx")
            event = self.events.get(eidx) or {}
            ekey = (eidx, trigger.get("wall_ns"))
            if ekey in self.seen_events:
                continue
            self.seen_events.add(ekey)
            if not event.get("uri"):  # the shape wants a referent; an unnamed event has none
                continue
            occ = self._occurrence(
                PROV.Activity, f"e{eidx}", trigger.get("wall_ns"), step, span=False
            )
            self.g.add((occ, PROV.used, rdflib.URIRef(event["uri"])))
            self._informed_by(occ, self.monitor_occ_for_event.pop(event["uri"], None))
            self.event_occ[eidx] = occ
            self.frame_event_occ.append(occ)

    # -- driving -----------------------------------------------------------------------

    def feed(self, frame: dict) -> set:
        """Project one frame; return the steps it anchored an occurrence at."""
        step = frame["step"]
        self.last_step = step
        wall = frame.get("timing", {}).get("wall_ns")
        cur = frame.get("fsm_state", -1)
        frame_events = {
            trigger.get("idx")
            for trigger in frame.get("triggers", [])
            if trigger.get("kind") == KIND_EVENT
        }
        self.frame_event_uris = {
            (self.events.get(idx) or {}).get("uri") for idx in frame_events
        } - {None}
        state = _state_meta(self.states, cur)
        meta = self.motions.get(frame.get("active_motion", -1), {})
        controllers = meta.get("controllers") or meta.get("constraints") or []
        monitors = meta.get("monitors") or []
        csat = [
            bool(c.get("active")) and bool(c.get("satisfied")) for c in frame.get("constraints", [])
        ]
        msat = [
            bool(m.get("active")) and bool(m.get("satisfied")) for m in frame.get("monitors", [])
        ]

        before = set(self.anchors)
        self.frame_event_occ = []
        # Events first: a state change on this same tick has to be able to name the event
        # instance that caused it, not just the event definition.
        self._triggers(frame, step)
        if cur != self.prev_state:
            # Slot indices are motion-local, so no arming carries across the boundary. The
            # spans open under the outgoing motion end here -- dropping them instead would
            # leave a beginning with no end in every query that asks how long one held.
            for occ in (*self.open_constraint.values(), *self.open_monitor.values()):
                self._close(occ, step)
            self.open_constraint, self.open_monitor = {}, {}
            self.first_held, self.rearm, self.member_prev, self.member_pending = {}, {}, {}, {}
            self.activity_began = step
            self._state_change(
                cur,
                state,
                frame.get("state_since_wall_ns") or wall,
                step,
                frame_events | self.prev_frame_events,
            )
            self._constraint_edges(controllers, csat, [False] * len(csat), frame, wall, step)
        else:
            self._resolve_motion(meta)
            self._constraint_edges(controllers, csat, self.prev_csat or [], frame, wall, step)
            self._monitor_edges(monitors, msat, frame, wall, step)

        if self.sample_interval_s is not None:
            t = frame.get("t", 0.0)
            if self.last_sample_t is None or t >= self.last_sample_t + self.sample_interval_s:
                self.last_sample_t = t
                frame_observations(
                    self.g, self.run_id, self.header, frame, signal_map=self.signal_map
                )

        self.prev_state, self.prev_csat, self.prev_msat = cur, csat, msat
        self.prev_frame_events = frame_events
        return self.anchors - before

    def close(self) -> set:
        """Close every span still open at the run's last frame.

        Without this the final occupancy silently drops out of every duration query, and the
        sum of activity durations no longer equals the run.
        """
        if self.last_step is None:
            return self.anchors
        for occ in (*self.open_constraint.values(), *self.open_monitor.values()):
            self._close(occ, self.last_step)
        self.last_activity = self.open_activity
        self._finish_activity(self.last_step)
        return self.anchors


def _project_occurrences(
    g: rdflib.Graph, run_id: str, header, frames: list[dict], dwells: dict
) -> IncrementalProjector:
    """Synthesize the discrete event graph from the per-tick frame scan; return the projector,
    which holds the steps that carry an occurrence (the instants worth materializing)."""
    projector = IncrementalProjector(g, run_id, header, declared_dwells=dwells)
    for frame in frames:
        projector.feed(frame)
    projector.close()
    return projector


def _add_rec_timing(g: rdflib.Graph, run_dir: Path, manifest: dict, run: rdflib.URIRef) -> dict:
    """Stamp the run with the wall-clock lifecycle rec observed, and return that lifecycle."""
    rec_rel = manifest.get("files", {}).get("rec", "rec.ld.json")
    rec_path = run_dir / rec_rel
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


def project_runtime(run_dir: Path | str, frames: list[dict]) -> rdflib.Graph:
    run_dir, manifest = load_manifest(run_dir)
    # The run's contract comes from the log itself, not a companion artifact.
    header = frame_log_pb.read_contract(run_dir / manifest["files"]["frame_log"]).header
    g = rdflib.Graph()
    bind_namespaces(g, manifest["run_id"], fsm_namespace=header.fsm_namespace)

    run_id = manifest["run_id"]
    # The run, the agents and the execution activity are shared provenance concepts: emit the
    # same canonical msprov IRIs the codegen and rec graphs use so all three join on one node
    # per concept (rather than three parallel ones). File entities are run-scoped, so two runs
    # of one generation never collapse onto the same entity.
    run = rdflib.URIRef(prov_uri(f"run:{run_id}"))
    activity = rdflib.URIRef(prov_uri(header.activity_id or "activity:controller_execution"))
    producer = rdflib.URIRef(prov_uri(header.producer_agent_id or "agent:controller_process"))
    runtime = rdflib.URIRef(prov_uri(header.runtime_agent_id or "agent:runtime"))
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
    g.add((frame_log, rdflib.RDF.type, PROV.Entity))
    g.add((frame_log, PROV.wasGeneratedBy, activity))
    g.add(
        (
            frame_log,
            PROV.atLocation,
            rdflib.URIRef(os.path.relpath(manifest["files"]["frame_log"], "runtime")),
        )
    )
    for entity, key in (
        (proto_entity, "frame_log_proto"),
        (model_entity, "model"),
        (provenance_entity, "provenance"),
    ):
        rel = manifest.get("files", {}).get(key)
        if rel and (run_dir / rel).exists():
            g.add((entity, rdflib.RDF.type, PROV.Entity))
            g.add((entity, PROV.atLocation, rdflib.URIRef(os.path.relpath(rel, "runtime"))))
    # The run id is already the run's own IRI, and the frame count is the last instant's position.
    g.add((run, DCTERMS.hasVersion, rdflib.Literal(RUNTIME_RDF_CONTRACT_VERSION)))
    lifecycle = _add_rec_timing(g, run_dir, manifest, run)

    # Provenance of this runtime.ttl document itself: recovered from the frame log by the
    # introspection recovery agent (same IRIs the rec graph uses), stamped at generation time.
    runtime_doc = rdflib.URIRef(run_entity_uri(run_id, "runtime_ttl"))
    recovery_activity = rdflib.URIRef(prov_uri("activity:runtime_ttl_recovery"))
    recovery_agent = rdflib.URIRef(prov_uri("agent:replay_process"))
    generated_at = _dt_literal(time.time_ns())
    g.add((runtime_doc, rdflib.RDF.type, PROV.Entity))
    g.add((runtime_doc, PROV.atLocation, rdflib.URIRef("runtime.ttl")))
    g.add((runtime_doc, PROV.wasGeneratedBy, recovery_activity))
    g.add((runtime_doc, PROV.wasDerivedFrom, frame_log))
    g.add((runtime_doc, PROV.generatedAtTime, generated_at))
    g.add((recovery_activity, rdflib.RDF.type, PROV.Activity))
    g.add((recovery_activity, PROV.used, frame_log))
    g.add((recovery_activity, PROV.wasAssociatedWith, recovery_agent))
    g.add((recovery_activity, PROV.endedAtTime, generated_at))
    g.add((recovery_agent, rdflib.RDF.type, OBS.ObservationProvider))
    g.add((recovery_agent, rdflib.RDFS.label, rdflib.Literal("motion_spec runtime.ttl recovery")))

    # Synthesize the discrete event graph, then materialize an instant only for the steps that
    # actually anchor an occurrence (plus the run's first/last for bounds). The dense per-tick
    # curve — every frame, all continuous scalars — stays in the frame log for numeric analysis.
    if frames:
        model_paths = _model_paths(run_dir, manifest)
        projector = _project_occurrences(
            g, run_id, header, frames, declared_dwells_from_paths(model_paths)
        )
        trs = _trs_node(run_id)
        emit_steps = projector.anchors | {frames[0]["step"], frames[-1]["step"]}
        for frame in frames:
            if frame["step"] in emit_steps:
                _instant(g, run_id, frame["step"], trs, frame.get("timing", {}).get("wall_ns"))
        g.add((run, TIME.hasBeginning, _instant_node(run_id, frames[0]["step"])))
        g.add((run, TIME.hasEnd, _instant_node(run_id, frames[-1]["step"])))
        # A run that did not complete never produced what it was meant to: the outcome is an
        # entity nothing generated, lost to whatever was in force when the run stopped.
        if lifecycle.get("status") in _FAILED_STATUSES and projector.last_activity is not None:
            outcome = rdflib.URIRef(run_entity_uri(run_id, "outcome"))
            g.add((outcome, rdflib.RDF.type, PROV.Entity))
            g.add((outcome, PROV.wasInvalidatedBy, projector.last_activity))
    return g


def write_runtime_ttl(run_dir: Path | str, frames: list[dict]) -> Path:
    run_dir = Path(run_dir)
    graph = project_runtime(run_dir, frames)
    manifest_path = run_dir / "manifest.json"
    runtime_rel = "runtime/runtime.ttl"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        runtime_rel = manifest.get("files", {}).get("runtime_ttl") or runtime_rel
    out = run_dir / runtime_rel
    out.parent.mkdir(parents=True, exist_ok=True)
    graph.serialize(out, format="turtle")
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        manifest.setdefault("files", {})["runtime_ttl"] = runtime_rel
        _record_runtime_ttl_with_rec(run_dir, manifest, out)
        manifest_path.write_text(json.dumps(manifest, indent=4) + "\n")
    return out


def _record_runtime_ttl_with_rec(run_dir: Path, manifest: dict, runtime_ttl: Path) -> None:
    try:
        from motion_spec.introspection.provenance import ensure_local_rec_importable

        ensure_local_rec_importable()
        from rec import Run
        from rec.observers import FileObserver
    except ImportError as exc:
        raise RuntimeError("REC is required to update runtime.ttl provenance") from exc

    rec_path = run_dir / manifest.get("files", {}).get("rec", "rec.ld.json")
    if not rec_path.exists():
        return
    run_id = manifest.get("run_id")
    observer = FileObserver(rec_path, run_iri=prov_uri(f"run:{run_id}"))
    run = Run(observers=[observer], run_id=run_id)
    run.add_agent(
        prov_uri("agent:replay_process"),
        rec_types(["prov:SoftwareAgent", "obs:ObservationProvider"]),
    )
    run.add_activity(
        prov_uri("activity:runtime_ttl_recovery"),
        rec_types(["prov:Activity"]),
        associated_with=prov_uri("agent:replay_process"),
    )
    run.add_artefact(
        str(runtime_ttl.resolve()),
        gen_activity=prov_uri("activity:runtime_ttl_recovery"),
        archive_path=str(runtime_ttl.relative_to(run_dir)),
        title="runtime_ttl",
        sha256=sha256_file(runtime_ttl),
        size_bytes=runtime_ttl.stat().st_size,
    )
    observer.close()
