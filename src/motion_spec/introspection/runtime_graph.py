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
from motion_spec.introspection.provenance import MSPROV, prov_uri, rec_run_lifecycle, rec_types


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
TRACE = rdflib.Namespace("https://secorolab.github.io/metamodels/motion-spec/execution-trace/")
TIME = rdflib.Namespace("http://www.w3.org/2006/time#")
DCTERMS = rdflib.Namespace("http://purl.org/dc/terms/")
SENS = rdflib.Namespace("https://secorolab.github.io/metamodels/robot/sensors#")
QUDT = rdflib.Namespace("http://qudt.org/schema/qudt/")
QKIND = rdflib.Namespace("http://qudt.org/vocab/quantitykind/")
UNIT = rdflib.Namespace("http://qudt.org/vocab/unit/")
RUNTIME_RDF_CONTRACT_VERSION = 3
# Member edge occurrences kept per watched member of one interesting arming. The arming's
# rearmCount is never capped, so a consumer comparing the two always sees a truncated series
# for what it is.
MEMBER_EDGE_LIMIT = 8
# How far past its own declared dwell a gate must hold its activity before the members it
# watches are worth recording. A gate that fired as soon as it armed was never waiting.
SLOW_GATE_FACTOR = 2.0
# Events are the only trigger kind the runtime actually emits; state/constraint/monitor edges
# are synthesized here from the per-tick frame scan (see _project_occurrences).
KIND_EVENT = 1


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


def _frame_node(run_id: str, step: int) -> rdflib.URIRef:
    return _node(f"frame:{run_id}:{step}")


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
    sensor: rdflib.URIRef,
    kind: str,
    slot,
    role: str,
    prop: rdflib.URIRef,
    value,
    feature: rdflib.URIRef | None = None,
) -> rdflib.URIRef:
    """One sosa:Observation of a slot's value at a frame -- an instance node, no new vocabulary."""
    node = _scoped("observation", run_id, step, f"{kind}{slot}", role)
    g.add((node, rdflib.RDF.type, SOSA.Observation))
    g.add((node, SOSA.observedProperty, prop))
    g.add((node, SOSA.madeBySensor, sensor))
    if feature is not None:
        g.add((node, SOSA.hasFeatureOfInterest, feature))
    _literal(g, node, SOSA.hasSimpleResult, value)
    dt = _dt_literal(wall_ns)
    if dt is not None:
        g.add((node, SOSA.resultTime, dt))
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
            g, run_id, step, wall_ns, sensor, "m", idx, "value", monitor_uri, float(slot["value"])
        )
    for idx, (qid, iri) in enumerate(sorted((quantity_iris or {}).items())):
        if qid not in frame.get("quantities", {}):
            continue
        _observation(
            g,
            run_id,
            step,
            wall_ns,
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
    frame log; only semantic edges land in the graph, under two classes:

      * ActivityOccurrence spans an interval something was in force for -- a coordination
        element active, a goal constraint satisfied, a monitor's condition holding until it
        fired, a watched member of a gate holding. Unsatisfied needs no occurrence: it is the
        gap between two satisfied spans.
      * ControlFlowOccurrence marks the instant control moved -- a transition, or the event
        that drove it. The heartbeat that drives the FSM is deliberately not recorded, because
        the frame log is a time series and every frame already is the tick.

    Neither class is named after a state or a transition: the single prov:used points at the
    design IRI, whose own rdf:type says what kind of element it was.

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
        self.seq = 0
        self.last_step: int | None = None
        self.activity_began: int | None = None
        # Open spans, closed when the opposite edge arrives or the run ends.
        self.open_activity: rdflib.URIRef | None = None
        self.open_constraint: dict = {}
        # Per monitor slot, the arming in progress: when its condition first held, how many
        # times it broke since the activity began, and the member edges seen meanwhile.
        self.first_held: dict = {}
        self.rearm: dict = {}
        self.member_prev: dict = {}
        self.member_pending: dict = {}
        # Instance-level causes, kept only as long as they can still inform something.
        self.event_occ: dict = {}
        self.monitor_occ_for_event: dict = {}

    # -- node construction -------------------------------------------------------------

    def _occurrence(self, typename: str, disc, wall_ns, step: int, *, span: bool) -> rdflib.URIRef:
        """A discrete occurrence: a prov:Activity, ordered by seq, anchored to its frame.

        A span is the interval itself -- its beginning is this frame, its end is filled in when
        it closes. An instant names the one frame it happened at.
        """
        node = _scoped(
            "occurrence", self.run_id, typename, wall_ns if wall_ns is not None else 0, disc
        )
        self.g.add((node, rdflib.RDF.type, TRACE[typename]))
        self.g.add((node, rdflib.RDF.type, PROV.Activity))
        self.g.add((node, TRACE.seq, rdflib.Literal(self.seq)))
        self.seq += 1
        if span:
            self.g.add((node, rdflib.RDF.type, TIME.ProperInterval))
            self.g.add((node, TIME.hasBeginning, _frame_node(self.run_id, step)))
        else:
            self.g.add((node, TRACE.atFrame, _frame_node(self.run_id, step)))
        self.anchors.add(step)
        dt = _dt_literal(wall_ns)
        if dt is not None:
            self.g.add((node, PROV.startedAtTime, dt))
        return node

    def _close(self, node: rdflib.URIRef | None, step: int) -> None:
        """Close a span at `step`. Idempotent, so closing the run twice is harmless."""
        if node is None or (node, TIME.hasEnd, None) in self.g:
            return
        self.g.add((node, TIME.hasEnd, _frame_node(self.run_id, step)))
        self.anchors.add(step)

    def _informed_by(self, node: rdflib.URIRef, cause: rdflib.URIRef | None) -> None:
        if cause is not None:
            self.g.add((node, PROV.wasInformedBy, cause))

    # -- occurrence builders -----------------------------------------------------------

    def _state_change(self, cur, state, entry, step, observed_events: set) -> None:
        """Close the activity that ended, record the control flow, open the one that began."""
        prev_state = self.prev_state
        self._close(self.open_activity, step)
        self.open_activity = None
        control_flow = None
        if prev_state is not None:
            control_flow = self._control_flow(prev_state, cur, entry, step, observed_events)
        if state and state.get("uri"):
            occ = self._occurrence("ActivityOccurrence", f"s{cur}", entry, step, span=True)
            self.g.add((occ, PROV.used, rdflib.URIRef(state["uri"])))
            self._informed_by(occ, control_flow)
            self.open_activity = occ

    def _control_flow(self, prev_state, cur, entry, step, observed_events: set):
        tr = _fired_transition(self.transitions.get((prev_state, cur)) or [], observed_events)
        if not tr or not tr.get("uri"):
            return None
        occ = self._occurrence(
            "ControlFlowOccurrence",
            f"t{tr.get('id', f'{prev_state}-{cur}')}",
            entry,
            step,
            span=False,
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
            self._close(self.open_constraint.pop(idx, None), step)
            if not now:
                continue
            value = frame["constraints"][idx].get("error")
            self.open_constraint[idx] = self._constraint_span(
                f"c{idx}", constraint_uri, idx, None if value is None else float(value), wall, step
            )

    def _constraint_span(self, disc, constraint_uri, slot_index, value, wall, step: int):
        occ = self._occurrence("ActivityOccurrence", disc, wall, step, span=True)
        self.g.add((occ, PROV.used, constraint_uri))
        _literal(self.g, occ, TRACE.slotIndex, slot_index)
        _literal(self.g, occ, SOSA.hasSimpleResult, value)
        return occ

    def _monitor_edges(self, monitors: list, msat: list, frame: dict, wall, step: int) -> None:
        """Track each monitor's arming, and close it into a span when it fires.

        The observed dwell is the interval this emits; the *declared* dwell stays in the design
        graph, where a query joins it. The two are compared, never copied together.
        """
        prev = self.prev_msat
        for idx, now in enumerate(msat):
            if idx >= len(monitors):
                continue
            was = None if prev is None or idx >= len(prev) else prev[idx]
            if now and not was:
                self.first_held[idx] = step
            elif was and not now:
                self.rearm[idx] = self.rearm.get(idx, 0) + 1
                self.first_held.pop(idx, None)
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
        began = self.first_held.get(idx, step)
        rearm = self.rearm.get(idx, 0)
        occ = self._occurrence("ActivityOccurrence", f"m{idx}", wall, began, span=True)
        self._close(occ, step)
        self.g.add((occ, PROV.used, monitor_uri))
        _literal(self.g, occ, TRACE.slotIndex, idx)
        _literal(self.g, occ, TRACE.rearmCount, rearm)
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
        member -- rearmCount is not, so a truncated series is always recognisable as one.
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
                occ = self._constraint_span(
                    f"w{idx}-{member_id}-{order}",
                    rdflib.URIRef(member_uri),
                    None,
                    error,
                    wall,
                    step,
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
                "ControlFlowOccurrence", f"e{eidx}", trigger.get("wall_ns"), step, span=False
            )
            self.g.add((occ, PROV.used, rdflib.URIRef(event["uri"])))
            self._informed_by(occ, self.monitor_occ_for_event.pop(event["uri"], None))
            _literal(self.g, occ, TRACE.slotIndex, eidx)
            self.event_occ[eidx] = occ

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
        # Events first: a state change on this same tick has to be able to name the event
        # instance that caused it, not just the event definition.
        self._triggers(frame, step)
        if cur != self.prev_state:
            # Slot indices are motion-local, so no arming carries across the boundary. The
            # spans open under the outgoing motion end here -- dropping them instead would
            # leave a beginning with no end in every query that asks how long one held.
            for occ in self.open_constraint.values():
                self._close(occ, step)
            self.open_constraint = {}
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
        self._close(self.open_activity, self.last_step)
        for occ in self.open_constraint.values():
            self._close(occ, self.last_step)
        return self.anchors


def _project_occurrences(
    g: rdflib.Graph, run_id: str, header, frames: list[dict], dwells: dict
) -> set:
    """Synthesize the discrete event graph from the per-tick frame scan; return the set of steps
    that carry an occurrence (the frames worth materializing)."""
    projector = IncrementalProjector(g, run_id, header, declared_dwells=dwells)
    for frame in frames:
        projector.feed(frame)
    return projector.close()


def _add_rec_timing(
    g: rdflib.Graph, run_dir: Path, manifest: dict, activity: rdflib.URIRef
) -> None:
    rec_rel = manifest.get("files", {}).get("rec", "rec.ld.json")
    rec_path = run_dir / rec_rel
    if not rec_path.exists():
        return
    lifecycle = rec_run_lifecycle(rdflib.Graph().parse(rec_path, format="json-ld"))
    for key, pred in (("started_time", PROV.startedAtTime), ("completed_time", PROV.endedAtTime)):
        if lifecycle.get(key):
            g.add((activity, pred, rdflib.Literal(lifecycle[key], datatype=rdflib.XSD.dateTime)))


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
        "trace": TRACE,
        "time": TIME,
        "dcterms": DCTERMS,
        "sens": SENS,
        "qudt": QUDT,
        "qkind": QKIND,
        "unit": UNIT,
        "ent": rdflib.Namespace(f"{MSRUN}entity/"),
        "run": rdflib.Namespace(f"{MSRUN}run/"),
        "qty": rdflib.Namespace(f"{MSRUN}quantity/"),
        "mspact": rdflib.Namespace(f"{MSPROV}activity/"),
        "mspagent": rdflib.Namespace(f"{MSPROV}agent/"),
    }.items():
        g.bind(prefix, ns)
    # Compact the per-run node families and model FSM nodes into CURIEs by binding a prefix at
    # each family's `<family>/<run_id>/` boundary (locals are slash-free — see _scoped).
    for prefix, family in (
        ("frm", "frame"),
        ("cs", "controller-sample"),
        ("mons", "monitor-sample"),
        ("sig", "signal"),
        ("occ", "occurrence"),
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

    run = _node(f"run:{manifest['run_id']}")
    # Agents and the execution activity are shared provenance concepts: emit the same
    # canonical msprov IRIs the codegen graph uses so the runtime, codegen and rec graphs
    # join on one node per concept (rather than three parallel ones).
    activity = rdflib.URIRef(prov_uri(header.activity_id or "activity:controller_execution"))
    producer = rdflib.URIRef(prov_uri(header.producer_agent_id or "agent:controller_process"))
    runtime = rdflib.URIRef(prov_uri(header.runtime_agent_id or "agent:runtime"))
    frame_log = _node("entity:frame_log")
    proto_entity = _node("entity:frame_log_proto")
    model_entity = _node("entity:model_jsonld")
    provenance_entity = _node("entity:provenance_jsonld")

    g.add((run, rdflib.RDF.type, PROV.Entity))
    g.add((run, rdflib.RDF.type, EXEC.ExecutionContext))
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
    g.add((producer, rdflib.RDF.type, PROV.SoftwareAgent))
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
    # The run id is already the run's own IRI, and the frame count is the last Frame's step.
    g.add((run, DCTERMS.hasVersion, rdflib.Literal(RUNTIME_RDF_CONTRACT_VERSION)))
    # The run's tick rate, so a step converts to seconds without opening the frame log for one
    # header field. It hangs off the provider that produced the frames, as a qudt frequency --
    # the shape the sensors metamodel already defines for an update rate.
    if header.nominal_period_ns:
        rate = _node(f"quantity:{manifest['run_id']}:tick_rate")
        g.add((producer, SENS["update-rate"], rate))
        g.add((rate, rdflib.RDF.type, QUDT.Quantity))
        g.add((rate, QUDT.hasQuantityKind, QKIND.Frequency))
        g.add((rate, QUDT.unit, UNIT.HZ))
        _literal(g, rate, QUDT.value, 1e9 / header.nominal_period_ns)
    g.add((run, PROV.wasGeneratedBy, activity))
    _add_rec_timing(g, run_dir, manifest, activity)

    # Provenance of this runtime.ttl document itself: recovered from the frame log by the
    # introspection recovery agent (same IRIs the rec graph uses), stamped at generation time.
    runtime_doc = _node("entity:runtime_ttl")
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
    g.add((recovery_agent, rdflib.RDF.type, PROV.SoftwareAgent))
    g.add((recovery_agent, rdflib.RDF.type, OBS.ObservationProvider))
    g.add((recovery_agent, rdflib.RDFS.label, rdflib.Literal("motion_spec runtime.ttl recovery")))

    # Synthesize the discrete event graph, then materialize a Frame node only for the steps that
    # actually anchor an occurrence (plus the run's first/last for bounds). The dense per-tick
    # curve — every frame, all continuous scalars — stays in the frame log for numeric analysis.
    if frames:
        model_paths = _model_paths(run_dir, manifest)
        anchors = _project_occurrences(
            g, manifest["run_id"], header, frames, declared_dwells_from_paths(model_paths)
        )
        emit_steps = anchors | {frames[0]["step"], frames[-1]["step"]}
        for frame in frames:
            if frame["step"] not in emit_steps:
                continue
            frame_node = _frame_node(manifest["run_id"], frame["step"])
            g.add((frame_node, rdflib.RDF.type, TRACE.Frame))
            # A frame is the instant an occurrence anchors to, so interval endpoints reuse it
            # instead of a parallel set of time:Instant nodes.
            g.add((frame_node, rdflib.RDF.type, TIME.Instant))
            g.add((frame_node, TRACE.step, rdflib.Literal(frame["step"])))
            frame_dt = _dt_literal(frame.get("timing", {}).get("wall_ns"))
            if frame_dt is not None:
                g.add((frame_node, PROV.generatedAtTime, frame_dt))
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
    observer = FileObserver(rec_path)
    run = Run(observers=[observer], run_id=manifest.get("run_id"))
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
