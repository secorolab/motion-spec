# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The FSM, framed from its named graph and wired to the monitors that fire it: the state each
motion runs in, the events snapshots and re-tares wait on, and the holds a gated motion runs.
"""

from __future__ import annotations

from rdf_utils.constraints import ConstraintViolation
from rdf_utils.uri import iri_is_descendant

from motion_spec.classes.geometry import Wrench
from motion_spec.classes.solvers import SolverWithInputAndOutput


def apply_fsm_wiring(motions, fsm, solvers) -> dict:
    """Tag the monitors that fire the FSM, and return the wiring codegen needs beside it.

    Raises:
        ConstraintViolation: a motion declares no `until` and the model imports no FSM, so nothing
            can end it; a snapshot triggers on an event the FSM does not declare; or a WHEN-gated
            motion names no hold motion, or an unknown one.
    """
    namespace = fsm["name"].lower() if fsm else None
    events = fsm["event_uris"] if fsm else {}
    step_event = "E_STEP" if "E_STEP" in events else None
    # The heartbeat is the clock and is not logged every tick, but where a transition's guard is
    # the clock, that occurrence caused the state change -- so name those transitions.
    transitions = {row["id"]: row for row in (fsm.get("transitions_table", []) if fsm else [])}
    meta = {
        "cpp_namespace": namespace,
        "header": f"{fsm['name']}.hpp" if fsm else None,
        "start_state": fsm["start_state"] if fsm else None,
        "step_event": step_event,
        "step_transitions": [
            {"from": transition["from_state"], "to": transition["to_state"]}
            for reaction in (fsm.get("reactions_table", []) if fsm else [])
            if reaction["when_event"] == step_event
            for transition in [transitions.get(reaction["do_transition"])]
            if transition
        ],
    }
    if namespace is None:
        # Without an FSM the sequencer advances on a motion's own `until`, so one declaring none
        # can never be left and every motion after it is unreachable.
        stuck = [motion.id for motion in motions if not motion.has_until_condition]
        if stuck:
            raise ConstraintViolation(
                "coordination",
                f"motions {stuck} declare no 'until' condition and the model imports no "
                "FSM, so nothing can end them; add an 'until' condition or coordinate the model "
                "with an FSM",
            )
        _resolve_occurrence_events(motions, fsm, namespace)

        return meta

    namespace_uri = fsm.get("namespace_uri")
    # coord-dsl's own token per event IRI: the name its enum gives the event.
    event_by_uri = {uri: token for token, uri in events.items()}
    state_by_event = {
        reaction["when_event"]: transitions[reaction["do_transition"]]["from_state"]
        for reaction in fsm["reactions_table"]
        if reaction["do_transition"] in transitions
    }
    # A fallback names the motion specification, which several handlers may realize.
    units_by_motion: dict[str, list] = {}
    for motion in motions:
        units_by_motion.setdefault(motion.motion_id, []).append(motion)
    state_by_uri = {uri: name for name, uri in fsm.get("state_uris", {}).items()}
    for motion in motions:
        if not motion.runs_in_state:
            continue
        state = state_by_uri.get(motion.runs_in_state)
        if state is None:
            raise ConstraintViolation(
                "coordination",
                f"motion '{motion.id}' says it runs in '{motion.runs_in_state}', which the FSM "
                f"'{namespace}' does not declare as a state.",
            )
        motion.fsm_state = state
        if state == fsm["end_state"]:
            if not motion.until_monitors:
                raise ConstraintViolation(
                    "coordination",
                    f"motion '{motion.id}' runs in the end state '{state}', so its until is what "
                    "ends the run; it needs a monitor on that until, or the loop never leaves.",
                )
            motion.runs_in_end_state = True
            meta["end_motion"] = motion.id

    # A sensor re-tares on occurrences the same way a snapshot re-samples on them.
    for solver in solvers:
        if not isinstance(solver, SolverWithInputAndOutput):
            continue
        for out in solver.output:
            if not isinstance(out, Wrench) or not out.retare_event_uris:
                continue
            names = []
            for uri in out.retare_event_uris:
                name = event_by_uri.get(uri)
                if name is None:
                    raise ConstraintViolation(
                        "coordination",
                        f"Wrench '{out.id}' re-tares on '{uri}', which '{namespace}' does not "
                        "declare.",
                    )
                names.append(f"{namespace}::{name}")
            out.retare_events = tuple(names)

    for motion in motions:
        # An event-triggered snapshot only compiles when the FSM declares the event it waits on.
        for snapshot in motion.snapshots:
            if not snapshot.trigger_event:
                continue
            trigger = event_by_uri.get(snapshot.trigger_event)
            if trigger is None:
                raise ConstraintViolation(
                    "coordination",
                    f"Snapshot '{snapshot.target_id}' in motion '{motion.id}' triggers on "
                    f"'{snapshot.trigger_event}', which the FSM '{namespace}' does not declare.",
                )
            snapshot.trigger_event = trigger
            snapshot.fsm_namespace = namespace
        gated = [(monitor, False) for monitor in [*motion.until_monitors, *motion.while_monitors]]
        gated += [(monitor, True) for monitor in motion.when_monitors]
        for monitor, when_gate in gated:
            # A monitor fires the FSM only when its event lives in the FSM's namespace; a
            # standalone, monitor-owned event keeps its own stub.
            if not monitor.is_edge_triggered or not iri_is_descendant(
                namespace_uri or "", monitor.event_uri or ""
            ):
                continue
            event_name = event_by_uri.get(monitor.event_uri)
            if event_name is None:
                raise ConstraintViolation(
                    "coordination",
                    f"monitor '{monitor.id}' fires '{monitor.event_uri}', which the FSM "
                    f"'{namespace}' does not declare.",
                )
            monitor.fsm_namespace = namespace
            monitor.event_name = event_name
            state = state_by_event.get(event_name)
            if not when_gate:
                if state and not motion.fsm_state:
                    motion.fsm_state = state
                continue
            fallback = _when_gate_fallback(motion, monitor, units_by_motion)
            if state is None:
                _bind_in_state_gate(motion, monitor, fallback)
                continue
            if not fallback.fsm_state:
                fallback.fsm_state = state
            if motion.id not in fallback.fsm_when_gate_motions:
                fallback.fsm_when_gate_motions.append(motion.id)

    _apply_reentry_events(motions, fsm)
    _resolve_occurrence_events(motions, fsm, namespace)
    _check_every_commanding_motion_runs(motions, fsm, meta)

    return meta


def _reachable_states(fsm) -> set:
    """The states the FSM can reach from its start state.

    A transition fires only when a reaction names it, so one no reaction refers to is not an
    edge: following it would call a state reachable that the generated FSM can never enter.
    """
    transitions = {row["id"]: row for row in fsm["transitions_table"]}
    outgoing: dict = {}
    for reaction in fsm["reactions_table"]:
        transition = transitions.get(reaction["do_transition"])
        if transition is not None:
            outgoing.setdefault(transition["from_state"], set()).add(transition["to_state"])

    reached = {fsm["start_state"]}
    pending = [fsm["start_state"]]
    while pending:
        for state in outgoing.get(pending.pop(), ()):
            if state not in reached:
                reached.add(state)
                pending.append(state)

    return reached


def _check_every_commanding_motion_runs(motions, fsm, meta) -> None:
    """Motions and the states that run them have to cover each other.

    A motion reaches its state by firing the event that leaves it, so a motion that fires
    nothing is bound to nothing: the dispatch gets no case, the motion never steps, and the
    generated program runs its loop commanding whatever the drivers start at -- zero torque on
    a torque-controlled arm, which is an arm that falls.

    The same hole opens from the other side. A state the FSM can sit in with no motion bound to
    it renders no `case` either, and the loop keeps calling the driver every tick with whatever
    was staged last, so the arm is uncommanded for exactly as long as the FSM stays there. Only
    two states are allowed to run nothing: the end state, which the loop breaks on before it
    dispatches unless a motion is bound there -- then it stays until that motion's until has
    fired -- and a state the heartbeat leaves, which the FSM does not dwell in.
    """
    namespace = meta["cpp_namespace"]
    orphaned = [
        motion.id
        for motion in motions
        if motion.controllers and not motion.fsm_state and not motion.is_when_gate_hold
    ]
    if orphaned:
        raise ConstraintViolation(
            "coordination",
            f"{', '.join(orphaned)} command a robot but no state of the FSM '{namespace}' runs "
            "them. A motion is bound to the state its monitor's event leaves, so a motion that "
            "declares no monitor firing an FSM event is never stepped.",
        )

    passed_through = {transition["from"] for transition in meta["step_transitions"]}
    bound = {motion.fsm_state for motion in motions} | passed_through | {fsm["end_state"]}
    idle = [state for state in _reachable_states(fsm) if state not in bound]
    if idle:
        raise ConstraintViolation(
            "coordination",
            f"the FSM '{namespace}' can be in {', '.join(idle)}, but no motion runs there. The "
            "dispatch gets no case for a state nothing is bound to, so the loop steps no motion "
            "while the FSM sits in it and the robot keeps whatever command was staged last -- "
            "zero torque, if nothing has run yet. Bind a motion to it with 'runs-in', or give "
            "the state a transition the heartbeat takes so the FSM passes straight through.",
        )


def _resolve_occurrence_events(motions, fsm, namespace) -> None:
    """Resolve each announced event to the enum token the generated FSM names it by.

    A monitor announces events it need not fire itself, so the tokens come from the FSM's own
    table rather than from the monitor's trigger.

    Raises:
        ConstraintViolation: a monitor announces an event no imported FSM declares, so there is
            no table to read the IRI from.
    """
    token_by_uri = {uri: token for token, uri in (fsm or {}).get("event_uris", {}).items()}
    for motion in motions:
        for monitor in [*motion.when_monitors, *motion.while_monitors, *motion.until_monitors]:
            ros = monitor.ros
            if ros is None or ros.occurrence_path is None:
                continue
            tokens = []
            for uri in ros.occurrence_events:
                token = token_by_uri.get(uri)
                if token is None:
                    raise ConstraintViolation(
                        "coordination",
                        f"monitor '{monitor.id}' announces '{uri}', which no imported FSM "
                        "declares; an event with no IRI table has nothing to publish from",
                    )
                tokens.append(token)
            ros.occurrence_events = tokens
            ros.occurrence_namespace = namespace


def _apply_reentry_events(motions, fsm) -> None:
    """A self-transition's fired events re-enter the state's motion.

    The model opts in by authoring `fires` on the retry reaction; the loop consumes the fired
    event and deactivates the motion, so entry runs again (snapshots re-capture, goals re-send).
    """
    tables = fsm or {}
    self_state = {
        row["id"]: row["from_state"]
        for row in tables.get("transitions_table", [])
        if row["from_state"] == row["to_state"]
    }
    fired_by_state: dict = {}
    for row in tables.get("reactions_table", []):
        state = self_state.get(row["do_transition"])
        if state is not None:
            fired_by_state.setdefault(state, set()).update(row["fires_events"])
    for motion in motions:
        motion.reentry_events = list(fired_by_state.get(motion.fsm_state, ()))


def _when_gate_fallback(motion, monitor, units_by_motion):
    """The hold motion that runs while a WHEN-gated motion waits for its event.

    Raises:
        ConstraintViolation: the fallback names a motion two handlers realize, so nothing says
            which of them holds while the gated motion waits.
    """
    if not monitor.fallback_motion:
        raise ConstraintViolation(
            "coordination",
            f"WHEN monitor '{monitor.id}' on FSM-wired motion '{motion.id}' must declare a "
            "waiting hold motion (e.g. '... otherwise hold <hold-motion>'). A WHEN precondition "
            "without a fallback would leave the arm uncommanded while waiting.",
        )
    candidates = units_by_motion.get(monitor.fallback_motion, [])
    if not candidates:
        raise ConstraintViolation(
            "coordination",
            f"WHEN monitor '{monitor.id}' names unknown fallback motion "
            f"'{monitor.fallback_motion}'.",
        )
    if len(candidates) > 1:
        raise ConstraintViolation(
            "coordination",
            f"WHEN monitor '{monitor.id}' on motion '{motion.id}' falls back to "
            f"'{monitor.fallback_motion}', which is realized by more than one constraint "
            f"handler ({', '.join(unit.id for unit in candidates)}), so nothing says "
            "which of them holds while the gated motion waits. Name the handler's own motion.",
        )

    return candidates[0]


def _bind_in_state_gate(motion, monitor, fallback) -> None:
    """A WHEN monitor whose event drives no transition gates the motion inside its own state.

    Raises:
        ConstraintViolation: the motion names no `runs-in` state, so there is no state to hold
            in; or two of its monitors name different holds.
    """
    if not motion.fsm_state:
        raise ConstraintViolation(
            "coordination",
            f"WHEN monitor '{monitor.id}' fires '{monitor.event_name}', which no reaction "
            f"consumes, so '{motion.id}' is gated inside its own state -- but it names no "
            "'runs-in' state to hold in.",
        )
    if motion.when_gate_hold and motion.when_gate_hold["id"] != fallback.id:
        raise ConstraintViolation(
            "coordination",
            f"'{motion.id}' is gated by two WHEN monitors naming different holds "
            f"('{motion.when_gate_hold['id']}', '{fallback.id}'); one motion holds while it waits.",
        )
    monitor.opens_gate = True
    motion.has_when_gate = True
    motion.when_gate_hold = {"id": fallback.id}
    fallback.is_when_gate_hold = True


def apply_fsm_gate_calls(motions, namespace) -> None:
    """Fold each hold motion's WHEN-evaluation gate calls.

    The gated motion's id plus its when-signature capability booleans, so the generated call is
    built from the same flags the function's own signature is.
    """
    if namespace is None:
        return
    by_id = {motion.id: motion for motion in motions}
    for motion in motions:
        if motion.has_when_gate:
            hold = by_id[motion.when_gate_hold["id"]]
            motion.when_gate_hold = {
                "id": hold.id,
                "index": hold.index,
                "needs_events": hold.step_needs_events,
            }
    for fallback in motions:
        if not fallback.fsm_when_gate_motions:
            continue
        fallback.fsm_when_gate_calls = [
            {
                "gid": gate_id,
                "needs_state": by_id[gate_id].when_needs_state,
                "needs_data": by_id[gate_id].when_needs_data,
                "needs_robot": by_id[gate_id].when_needs_robot,
                "needs_events": by_id[gate_id].when_needs_events,
            }
            for gate_id in fallback.fsm_when_gate_motions
            if gate_id in by_id
        ]
