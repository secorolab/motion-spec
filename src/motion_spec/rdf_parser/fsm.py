# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The FSM, framed from its named graph and wired to the monitors that fire it: the state each
motion runs in, the events snapshots and re-tares wait on, and the holds a gated motion runs.
"""

from __future__ import annotations

from rdf_utils.constraints import ConstraintViolation

from motion_spec.classes.geometry import Wrench
from motion_spec.classes.solvers import SolverWithInputAndOutput


def apply_fsm_wiring(motions, fsm, solvers) -> dict:
    """Tag the monitors that fire the FSM, and return the wiring codegen needs beside it.

    Raises:
        ConstraintViolation: a motion declares no `until` and the model imports no FSM, so nothing
            can end it; a snapshot triggers on an event the FSM does not declare; or a WHEN-gated
            motion names no hold motion, or an unknown one.
    """
    if not fsm:
        # without an FSM, a motion with no until is never left
        stuck = [motion.id for motion in motions if not motion.has_until_condition]
        if stuck:
            raise ConstraintViolation(
                "coordination",
                f"motions {stuck} declare no 'until' condition and the model imports no "
                "FSM, so nothing can end them; add an 'until' condition or coordinate the model "
                "with an FSM",
            )
        resolve_occurrence_events(motions, {}, None)

        return {
            "cpp_namespace": None,
            "header": None,
            "start_state": None,
            "step_event": None,
            "step_transitions": [],
        }

    namespace = fsm["name"].lower()
    event_by_uri = {uri: token for token, uri in fsm["event_uris"].items()}
    step_event = "E_STEP" if "E_STEP" in fsm["event_uris"] else None
    state_by_event, outgoing, fired_by_state, step_transitions = fsm_edges(fsm, step_event)
    meta = {
        "cpp_namespace": namespace,
        "header": f"{fsm['name']}.hpp",
        "start_state": fsm["start_state"],
        "step_event": step_event,
        "step_transitions": step_transitions,
    }
    bind_run_states(motions, fsm, namespace, meta)
    wire_retare_events(solvers, event_by_uri, namespace)

    units_by_motion: dict[str, list] = {}
    for motion in motions:
        units_by_motion.setdefault(motion.motion_id, []).append(motion)
    for motion in motions:
        wire_motion_events(motion, event_by_uri, namespace, state_by_event, units_by_motion)

    for motion in motions:
        motion.reentry_events = list(fired_by_state.get(motion.fsm_state, ()))

    resolve_occurrence_events(motions, event_by_uri, namespace)

    # an unbound motion never steps: a torque-controlled arm at zero torque falls
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

    passed_through = {transition["from"] for transition in step_transitions}
    bound = {motion.fsm_state for motion in motions} | passed_through | {fsm["end_state"]}
    reached = {fsm["start_state"]}
    pending = [fsm["start_state"]]
    while pending:
        for state in outgoing.get(pending.pop(), ()):
            if state not in reached:
                reached.add(state)
                pending.append(state)
    idle = [state for state in reached if state not in bound]
    if idle:
        raise ConstraintViolation(
            "coordination",
            f"the FSM '{namespace}' can be in {', '.join(idle)}, but no motion runs there. The "
            "dispatch gets no case for a state nothing is bound to, so the loop steps no motion "
            "while the FSM sits in it and the robot keeps whatever command was staged last -- "
            "zero torque, if nothing has run yet. Bind a motion to it with 'runs-in', or give "
            "the state a transition the heartbeat takes so the FSM passes straight through.",
        )

    return meta


def fsm_edges(fsm, step_event) -> tuple[dict, dict, dict, list]:
    """Read the FSM's reactions as edges: the state each event leaves, each state's successors,
    the events a self-transition fires per state, and the transitions the step event takes.
    """
    transitions = {row["id"]: row for row in fsm.get("transitions_table", [])}
    # a transition no reaction names is not an edge
    state_by_event, outgoing, fired_by_state, step_transitions = {}, {}, {}, []
    for reaction in fsm["reactions_table"]:
        transition = transitions.get(reaction["do_transition"])
        if transition is None:
            continue
        source, target = transition["from_state"], transition["to_state"]
        state_by_event[reaction["when_event"]] = source
        outgoing.setdefault(source, set()).add(target)
        if source == target:
            fired_by_state.setdefault(source, set()).update(reaction["fires_events"])
        if reaction["when_event"] == step_event:
            step_transitions.append({"from": source, "to": target})
    return state_by_event, outgoing, fired_by_state, step_transitions


def bind_run_states(motions, fsm, namespace, meta: dict) -> None:
    """Bind each motion that names a `runs-in` state to it, and record the end-state motion.

    Raises:
        ConstraintViolation: the state is not one the FSM declares, or a motion runs in the end
            state without a monitor on its until.
    """
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
        if state != fsm["end_state"]:
            continue
        if not motion.until_monitors:
            raise ConstraintViolation(
                "coordination",
                f"motion '{motion.id}' runs in the end state '{state}', so its until is what "
                "ends the run; it needs a monitor on that until, or the loop never leaves.",
            )
        motion.runs_in_end_state = True
        meta["end_motion"] = motion.id


def wire_retare_events(solvers, event_by_uri: dict, namespace) -> None:
    """Name the FSM events each solver's output wrench re-tares on.

    Raises:
        ConstraintViolation: a wrench re-tares on an event the FSM does not declare.
    """
    wrenches = [
        out
        for solver in solvers
        if isinstance(solver, SolverWithInputAndOutput)
        for out in solver.output
        if isinstance(out, Wrench) and out.retare_event_uris
    ]
    for out in wrenches:
        names = []
        for uri in out.retare_event_uris:
            name = event_by_uri.get(uri)
            if name is None:
                raise ConstraintViolation(
                    "coordination",
                    f"Wrench '{out.id}' re-tares on '{uri}', which '{namespace}' does not declare.",
                )
            names.append(f"{namespace}::{name}")
        out.retare_events = tuple(names)


def wire_motion_events(
    motion, event_by_uri: dict, namespace, state_by_event: dict, units_by_motion: dict
) -> None:
    """Wire MOTION's snapshot triggers and monitors to the FSM's events, binding the motion to the
    state its until/while events leave and its WHEN monitors to their hold motions.

    Raises:
        ConstraintViolation: a snapshot triggers on an event the FSM does not declare, or a WHEN
            monitor's hold is missing or ambiguous (see `gate_when_monitor`).
    """
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
    for monitor in motion.until_monitors + motion.while_monitors:
        event_name = wire_monitor_event(monitor, event_by_uri, namespace)
        if event_name is None:
            continue
        state = state_by_event.get(event_name)
        if state and not motion.fsm_state:
            motion.fsm_state = state
    for monitor in motion.when_monitors:
        event_name = wire_monitor_event(monitor, event_by_uri, namespace)
        if event_name is not None:
            gate_when_monitor(motion, monitor, state_by_event.get(event_name), units_by_motion)


def gate_when_monitor(motion, monitor, state, units_by_motion: dict) -> None:
    """Bind the hold motion MONITOR names to the state the gated MOTION waits in.

    STATE is the one MONITOR's event leaves; None when no reaction consumes the event, so MOTION
    is gated inside its own state and its hold runs there.

    Raises:
        ConstraintViolation: the monitor names no hold, an unknown one or one realized by several
            handlers; or an in-state gate has no `runs-in` state, or a second, different hold.
    """
    if not monitor.fallback_motion:
        raise ConstraintViolation(
            "coordination",
            f"WHEN monitor '{monitor.id}' on FSM-wired motion '{motion.id}' must declare a "
            "waiting hold motion (e.g. '... otherwise hold <hold-motion>'). A WHEN "
            "precondition without a fallback would leave the arm uncommanded while "
            "waiting.",
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
            "which of them holds while the gated motion waits. Name the handler's own "
            "motion.",
        )
    fallback = candidates[0]
    if state is not None:
        if not fallback.fsm_state:
            fallback.fsm_state = state
        if motion.id not in fallback.fsm_when_gate_motions:
            fallback.fsm_when_gate_motions.append(motion.id)
        return
    if not motion.fsm_state:
        raise ConstraintViolation(
            "coordination",
            f"WHEN monitor '{monitor.id}' fires '{monitor.event_name}', which no "
            f"reaction consumes, so '{motion.id}' is gated inside its own state -- but "
            "it names no 'runs-in' state to hold in.",
        )
    if motion.when_gate_hold and motion.when_gate_hold["id"] != fallback.id:
        raise ConstraintViolation(
            "coordination",
            f"'{motion.id}' is gated by two WHEN monitors naming different holds "
            f"('{motion.when_gate_hold['id']}', '{fallback.id}'); one motion holds "
            "while it waits.",
        )
    monitor.opens_gate = True
    motion.has_when_gate = True
    motion.when_gate_hold = {"id": fallback.id}
    fallback.is_when_gate_hold = True


def wire_monitor_event(monitor, event_by_uri: dict, namespace) -> str | None:
    """Name MONITOR's event as the FSM does; None when it raises no FSM event."""
    # a monitor-owned event keeps its own stub
    if not monitor.is_edge_triggered:
        return None
    event_name = event_by_uri.get(monitor.event_uri)
    if event_name is not None:
        monitor.fsm_namespace = namespace
        monitor.event_name = event_name
    return event_name


def resolve_occurrence_events(motions, token_by_uri: dict, namespace) -> None:
    """Resolve each announced event to the enum token the generated FSM names it by.

    A monitor announces events it need not fire itself, so the tokens come from the FSM's own
    table rather than from the monitor's trigger.

    Raises:
        ConstraintViolation: a monitor announces an event no imported FSM declares, so there is
            no table to read the IRI from.
    """
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


def apply_fsm_gate_calls(motions, namespace) -> None:
    """Fold each hold motion's WHEN-evaluation gate calls.

    The gated motion's id plus its when-signature capability booleans, so the generated call is
    built from the same flags the function's own signature is.
    """
    if namespace is None:
        return
    by_id = {motion.id: motion for motion in motions}
    for motion in motions:
        if not motion.has_when_gate:
            continue
        hold = by_id[motion.when_gate_hold["id"]]
        motion.when_gate_hold = {
            "id": hold.id,
            "index": hold.index,
            "needs_events": hold.step_needs_events,
        }
    for fallback in motions:
        fallback.fsm_when_gate_calls = [
            {
                "gid": gate.id,
                "needs_state": gate.when_needs_state,
                "needs_data": gate.when_needs_data,
                "needs_robot": gate.when_needs_robot,
                "needs_events": gate.when_needs_events,
            }
            for gate in map(by_id.__getitem__, fallback.fsm_when_gate_motions)
        ]
