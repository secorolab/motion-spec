# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What runs when.

In order: the readers for a handler and the evaluators, monitors and motion it binds; the
handlers themselves; the four steps a motion is built in -- which constraints belong to each
phase, the three schedules, the motion unit, and what is folded on once every motion exists; the
boolean terms a condition renders from; and the FSM, framed from its named graph and wired to the
monitors that fire it.

Nothing else in the package sequences anything.
"""

from __future__ import annotations

import re

from motion_spec_dsl.rdf_parser.vocab import (
    APP,
    CSTR,
    CSTR_EXT,
    CSTR_HDL,
    CSTR_HDL_EXT,
    GEOM_OP,
    KC_STAT,
    MAP,
    MOT,
    SLV,
)
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdf_utils.namespace import NS_MM_EL
from rdf_utils.naming import get_valid_var_name
from rdf_utils.uri import iri_is_descendant, iri_parent
from rdflib.namespace import RDF, SDO
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.constraints import GuardedMotion
from motion_spec.classes.handlers import (
    ConstraintEvaluator,
    ConstraintHandler,
    EdgeMonitor,
    EvaluatorType,
    LevelMonitor,
    RosPublication,
)
from motion_spec.classes.motion import ForwardedCommandStep, MotionSolverSlice, MotionUnit
from motion_spec.classes.solvers import CommandForwarding
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.constraint_handler import SolverIdFactory, annotate_controller_signals
from motion_spec.rdf_parser.model import reader
from motion_spec.rdf_parser.operations import (
    OPS_GENERIC,
    OPS_HANDLER,
    OPS_SOLVER,
    Schedule,
    path_projections_for_motion,
)
from motion_spec.rdf_parser.vocab import (
    URI_FSM_PRED_DESCRIPTION,
    URI_FSM_PRED_DO_TRANSITION,
    URI_FSM_PRED_END_STATE,
    URI_FSM_PRED_FIRES_EVENTS,
    URI_FSM_PRED_NAME,
    URI_FSM_PRED_REACTIONS,
    URI_FSM_PRED_START_STATE,
    URI_FSM_PRED_STATES,
    URI_FSM_PRED_TRANSITION_FROM,
    URI_FSM_PRED_TRANSITION_TO,
    URI_FSM_PRED_TRANSITIONS,
    URI_FSM_TYPE_FSM,
)

_PHASES = ("when", "while", "until")
_PHASE_PREDICATES = {"when": MOT["when"], "while": MOT["while"], "until": MOT["until"]}


def _is_elapsed_constraint(model, node) -> bool:
    """Whether a constraint is a timing constraint, measured against the clock rather than a solver."""
    return node is not None and CSTR_EXT["TimeConstraint"] in get_node_types(model.graph, node)


@reader
def guarded_motion(model, node) -> GuardedMotion:
    """A GuardedMotion: the constraints it starts on, holds during and ends on.

    A section-wide disjunction makes the whole phase an 'any'; a named group inside one keeps its
    own logic on the monitor that targets it, so it expands to its members here either way.

    Raises:
        ConstraintViolation: the motion carries no `schema:name`, so nothing can name its step
            function.
    """
    graph = model.graph
    phases: dict[str, list] = {}
    any_flags = {"when": False, "until": False}
    for phase in _PHASES:
        members = []
        for node_ in graph[node : _PHASE_PREDICATES[phase]]:
            if phase == "while" or not quantities.is_constraint_aggregate(model, node_):
                members.append(quantities.constraint(model, node_))
                continue
            if CSTR_EXT.ConstraintDisjunction in get_node_types(graph, node_):
                any_flags[phase] = True
            members.extend(
                quantities.constraint(model, member)
                for member in graph[node_ : CSTR_EXT["has-constraint"]]
            )
        phases[phase] = members

    name = graph.value(node, SDO.name)
    if name is None:
        raise ConstraintViolation("coordination", f"GuardedMotion {node} has no schema:name triple")
    description = graph.value(node, SDO.description)

    return GuardedMotion(
        model.id(node),
        phases["when"],
        phases["while"],
        phases["until"],
        any_flags["until"],
        any_flags["when"],
        name=str(name),
        description=str(description) if description is not None else None,
    )


def constraint_handler(model, node) -> ConstraintHandler:
    """A ConstraintHandler: the motion it governs, and the evaluators and monitors it binds."""
    model.expect_type(node, CSTR_HDL["ConstraintHandler"])
    graph = model.graph

    return ConstraintHandler(
        model.id(node),
        guarded_motion(model, graph.value(node, CSTR_HDL["motion"])),
        [constraint_evaluator(model, item) for item in graph[node : CSTR_HDL["evaluators"]]],
        [],
        [monitor_entry(model, item) for item in graph[node : CSTR_HDL["monitors"]]],
        int(getattr(model.graph.value(node, APP.order), "value", 0)),
    )


# Timing relations, and how each states its threshold. An elapsed constraint has no solver error:
# codegen compares the world clock to the threshold, so the reader resolves both to seconds.
_ELAPSED_RELATIONS = (
    (CSTR["GreaterThanConstraint"], ">=", CSTR["threshold"]),
    (CSTR["EqualityConstraint"], "==", CSTR["reference-value"]),
)


@reader
def constraint_evaluator(model, node) -> ConstraintEvaluator:
    """A ConstraintEvaluator: the constraint it watches, and the error signal it writes."""
    model.expect_type(node, CSTR_HDL["ConstraintEvaluator"])
    graph = model.graph
    constraint_node = graph.value(node, CSTR_HDL["constraint"])
    assignment = CSTR_HDL["AssignmentEvaluator"] in get_node_types(graph, node)
    error_node = None if assignment else graph.value(node, CSTR_HDL["error"])

    is_elapsed = _is_elapsed_constraint(model, constraint_node)
    operator, threshold, elapsed_tolerance = None, None, None
    if is_elapsed:
        types = get_node_types(graph, constraint_node)
        operator, predicate = next(
            ((op, pred) for type_, op, pred in _ELAPSED_RELATIONS if type_ in types),
            ("<", CSTR["threshold"]),
        )
        threshold = quantities.duration_seconds(model, graph.value(constraint_node, predicate))
        if operator == "==":
            band_node = graph.value(constraint_node, CSTR_EXT["tolerance"])
            elapsed_tolerance = quantities.duration_seconds(model, band_node)
    # An authored band on a spatial equality; the elapsed branch reads its own, in seconds,
    # because a duration's magnitude rides on qudt rather than on a shared value.
    band = None if is_elapsed else graph.value(constraint_node, CSTR_EXT["tolerance"])

    return ConstraintEvaluator(
        model.id(node),
        EvaluatorType.AssignmentEvaluator if assignment else EvaluatorType.ErrorEvaluator,
        quantities.constraint(model, constraint_node),
        quantities.quantity(model, error_node) if error_node is not None else None,
        tolerance=quantities.quantity(model, band) if band is not None else None,
        is_elapsed=is_elapsed,
        elapsed_op=operator,
        elapsed_threshold_s=threshold,
        elapsed_tolerance_s=elapsed_tolerance,
    )


def _monitored_group(model, monitored, is_section_aggregate):
    """The named until/when group a monitor targets, as its member ids and its logic.

    A named group is one of several conditions, so it is not the whole section: the monitor
    carries its members and their logic, and the terms are built from those.
    """
    if is_section_aggregate:
        return [], False
    group = next(
        (node for node in monitored if quantities.is_constraint_aggregate(model, node)), None
    )
    if group is None:
        return [], False
    members = sorted(model.id(member) for member in model.graph[group : CSTR_EXT["has-constraint"]])
    return members, CSTR_EXT.ConstraintDisjunction in get_node_types(model.graph, group)


@reader
def monitor_entry(model, node):
    """A monitor: a level flag its constraint sets continuously, or an edge event it fires once."""
    model.expect_type(node, CSTR_HDL["Monitor"])
    graph = model.graph
    handler = next(graph.subjects(CSTR_HDL.monitors, node), None)
    motion = graph.value(handler, CSTR_HDL.motion) if handler is not None else None
    monitored = set(graph.objects(node, CSTR_HDL.constraint))
    sections = {
        phase: set(graph.objects(motion, _PHASE_PREDICATES[phase])) if motion is not None else set()
        for phase in ("when", "until")
    }
    aggregate = len(monitored) > 1 or any(
        quantities.is_constraint_aggregate(model, item) for item in monitored
    )
    is_until_aggregate = aggregate and monitored == sections["until"]
    is_when_aggregate = aggregate and monitored == sections["when"]
    group_ids, group_any = _monitored_group(
        model, monitored, is_until_aggregate or is_when_aggregate
    )

    error_node = graph.value(node, CSTR_HDL["error"])
    error = (
        None
        if is_until_aggregate or is_when_aggregate or group_ids or error_node is None
        else quantities.quantity(model, error_node)
    )
    # The band belongs to the constraint, so a monitor carries it only when it watches one.
    band = (
        graph.value(next(iter(monitored)), CSTR_EXT["tolerance"])
        if error is not None and len(monitored) == 1
        else None
    )
    shared = {
        "tolerance": quantities.quantity(model, band) if band is not None else None,
        "is_until_aggregate": is_until_aggregate,
        "is_when_aggregate": is_when_aggregate,
        "group_constraint_ids": group_ids,
        "group_any": group_any,
    }
    if CSTR_HDL["LevelTriggeredMonitor"] in get_node_types(graph, node):
        flag = model.id(graph.value(node, CSTR_HDL["flag"]))

        return LevelMonitor(model.id(node), "LevelTriggeredMonitor", error, flag, **shared)

    event_node = graph.value(node, CSTR_HDL["event"])
    event = model.id(event_node)
    fallback = graph.value(node, CSTR_HDL_EXT["fallback-motion"])

    return EdgeMonitor(
        model.id(node),
        "EdgeTriggeredMonitor",
        error,
        event,
        None,
        **shared,
        event_uri=str(event_node),
        event_name=event.upper(),
        fallback_motion=model.id(fallback) if fallback is not None else None,
        debounce_duration_s=quantities.optional_seconds(
            model, node, CSTR_HDL_EXT["debounce-duration"]
        ),
        **_ros_publication(model, node),
    )


def _ros_publication(model, node) -> dict:
    """The ROS topic a monitor also publishes on, when the model asks for one."""
    channel = model.graph.value(node, NS_MM_ROS["channel-name"])
    if channel is None:
        return {}
    ros_type = str(model.graph.value(node, NS_MM_ROS["type-name"]) or "")
    # `pkg/msg/CamelType` split by the rosidl naming rule.
    parts = ros_type.split("/")
    package, message = parts[0], parts[-1]
    sub = parts[1] if len(parts) == 3 else "msg"
    stem = re.sub(r"([a-z\d])([A-Z])", r"\1_\2", re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", message))

    return {
        "ros": RosPublication(
            str(channel),
            ros_type,
            package,
            f"{package}/{sub}/{stem.lower()}.hpp",
            f"{package}::{sub}::{message}",
            f"{model.id(node)}_pub".replace("-", "_"),
        )
    }


def build_constraint_handlers(model, schedule, derivation):
    """The constraint handlers, and the calls their evaluators and controllers imply.

    The controllers are attached here rather than read: an authored controller becomes one record
    per axis, and `constraint_handler.py` is the only module that decides how many.

    Returns:
        `(handlers, steps)`: the handler records in declaration order, and every call the
        active scope emitted for them
    """
    graph = model.graph
    handlers, steps = [], []
    for node in sorted(
        graph.subjects(RDF.type, CSTR_HDL["ConstraintHandler"]),
        key=lambda item: int(getattr(graph.value(item, APP.order), "value", 0)),
    ):
        handler = constraint_handler(model, node)
        plans = derivation.controllers_by_handler.get(node, ())
        handler.controllers = [
            controller for plan in plans for controller in derivation.controllers_for(plan)
        ]
        handlers.append(handler)
        steps.extend(
            schedule.of(list(graph.objects(node, CSTR_HDL.evaluators)), OPS_GENERIC + OPS_HANDLER)
        )
        # A single-axis controller's evaluator is not reachable from its own error signal, so
        # append it once its dependencies are scheduled.
        for plan in plans:
            if len(plan.axes) > 1 or CSTR_HDL_EXT.FeedForwardController in get_node_types(
                graph, plan.controller
            ):
                continue
            error = graph.value(plan.controller, CSTR_HDL["error-signal"])
            evaluator = next(graph.subjects(CSTR_HDL.error, error), None)
            if evaluator is not None and model.id(evaluator) not in steps:
                steps.append(model.id(evaluator))
        # Controllers run after what feeds them, and a pose command's difference evaluator after
        # every controller reading it; both in reverse, so the emitted order is the authored one.
        steps.extend(
            controller.id
            for plan in reversed(plans)
            for controller in reversed(derivation.controllers_for(plan))
        )
        steps.extend(
            SolverIdFactory(
                model.id(plan.controller), model.motion_suffix(plan.motion)
            ).pose_evaluator()
            for plan in reversed(plans)
            if len(plan.axes) > 1
        )

    return handlers, steps


def assign_event_indexes(handlers) -> None:
    """Give every edge monitor a stable index into the run's event buffer."""
    index = 0
    for handler in handlers:
        for monitor in handler.monitors:
            if monitor.monitor_type == "EdgeTriggeredMonitor":
                monitor.event_idx = index
                index += 1


def apply_monitor_debounce(handlers, control_period_ns: int) -> None:
    """Convert each monitor's authored debounce duration into a tick count."""
    for handler in handlers:
        for monitor in handler.monitors:
            if getattr(monitor, "debounce_duration_s", None) is not None:
                monitor.debounce_steps = round(
                    monitor.debounce_duration_s / (control_period_ns * 1e-9)
                )


class PhaseNodes:
    """Which constraints, evaluators and monitors of one handler belong to which phase.

    A monitor names the constraints it watches; one that names none is bucketed by the error
    signal it reads instead, which is the only other thing tying it to a phase.
    """

    def __init__(self, model, handler, handler_node, motion_node):
        graph = model.graph
        self.handler_node = handler_node
        self.raw = {phase: set(graph[motion_node : _PHASE_PREDICATES[phase]]) for phase in _PHASES}
        self.constraints = {
            phase: quantities.expanded_constraints(model, self.raw[phase])
            if phase != "while"
            else set(self.raw[phase])
            for phase in _PHASES
        }
        self.evaluators = {phase: [] for phase in _PHASES}
        for node in graph[handler_node : CSTR_HDL["evaluators"]]:
            constraint = graph.value(node, CSTR_HDL["constraint"])
            phase = next((p for p in _PHASES if constraint in self.constraints[p]), None)
            if phase is not None:
                self.evaluators[phase].append(node)

        self.watched = {
            phase: {graph.value(node, CSTR_HDL["constraint"]) for node in self.evaluators[phase]}
            - {None}
            for phase in _PHASES
        }
        self.errors = {
            phase: {graph.value(node, CSTR_HDL["error"]) for node in self.evaluators[phase]}
            - {None}
            for phase in _PHASES
        }
        self.monitors = {phase: [] for phase in _PHASES}
        for node in graph[handler_node : CSTR_HDL["monitors"]]:
            phase = self._monitor_phase(graph, node)
            if phase is not None:
                self.monitors[phase].append(node)
        self._check(model, handler, handler_node)

    def _monitor_phase(self, graph, node) -> str | None:
        """The phase a monitor belongs to, by what it watches or, failing that, what it reads."""
        monitored = set(graph.objects(node, CSTR_HDL["constraint"]))
        if monitored:
            # A group monitor names one of the section's nodes, not the whole section, and the
            # group node itself never appears in the expanded member sets.
            for phase in ("when", "until"):
                if monitored == self.raw[phase] or monitored <= self.raw[phase]:
                    return phase

            return next((p for p in _PHASES if monitored & self.watched[p]), None)
        error = graph.value(node, CSTR_HDL["error"])

        return next((p for p in _PHASES if error in self.errors[p]), None)

    def _check(self, model, handler, handler_node) -> None:
        """Every classified node must be one the handler declares.

        An evaluator or controller bound to no phase is silently excluded from all schedules,
        which is intended; inventing one that the handler never declared is not.
        """
        for kind, declared_predicate in (
            ("evaluators", CSTR_HDL["evaluators"]),
            ("monitors", CSTR_HDL["monitors"]),
        ):
            declared = set(model.graph[handler_node:declared_predicate])
            classified = {node for phase in _PHASES for node in getattr(self, kind)[phase]}
            if not classified <= declared:
                raise ValueError(
                    f"Handler {handler.id}: classified {kind} not a subset of handler {kind}"
                )


def _upstream_dependencies(data_id: str, closure_inputs: dict) -> set:
    """Every data id that feeds one id, however many closures deep."""
    result: set[str] = set()
    pending = list(closure_inputs.get(data_id, set()))
    while pending:
        item = pending.pop()
        if item in result:
            continue
        result.add(item)
        pending.extend(closure_inputs.get(item, set()))
    return result


def _handler_chain_solvers(handler, serial_chains, solver_ids) -> list:
    """The arm solvers this handler commands, sliced to the motion driver it drives them with.

    No copy of the solver's own facts (`algorithm`, `gravity`, `chain.root`, ...) -- a
    template reaches them through `solver_id` into `resources.by_id`.
    """
    result = []
    driver_id = f"driver_{handler.motion.id.removeprefix('motion_')}"
    for solver in serial_chains:
        if solver.id not in solver_ids or not solver.motion_drivers:
            continue
        drivers = solver.motion_drivers
        selected = next((driver for driver in drivers if driver.id == driver_id), drivers[0])
        result.append(
            MotionSolverSlice(
                id=solver.id,
                solver_id=solver.id,
                output=solver.output,
                motion_driver=selected,
                read_only=not (
                    selected.acceleration_constraint
                    or selected.cartesian_force
                    or selected.cartesian_acceleration
                    or selected.joint_force
                ),
            )
        )

    return result


def _cartesian_force_nodes(model, chain_solvers, handler, computation) -> list:
    """The authored Cartesian forces a handler's own controllers ultimately drive."""
    graph = model.graph
    outputs = {controller.control_signal.id for controller in handler.controllers}
    found = []
    for solver in chain_solvers:
        driver_node = model.node_by_id.get(solver.motion_driver.id)
        if driver_node is None:
            continue
        for node in graph[driver_node : SLV["cartesian-force"]]:
            force = graph.value(node, SLV["force"])
            if force is None:
                continue
            force_id = model.id(force)
            upstream = {force_id} | _upstream_dependencies(
                force_id, computation.indexes.closure_input
            )
            if upstream & outputs:
                found.append(node)

    return found


def _append_new(steps: list, candidates) -> None:
    """Append the calls this list does not already carry, keeping their order."""
    for step in candidates:
        if step not in steps:
            steps.append(step)


class MotionSchedules:
    """The three call sequences one motion runs, in the order the loop runs them."""

    def __init__(self, when: list, while_pre: list, active: list, until: list):
        self.when = when
        self.while_pre = while_pre
        self.active = active
        self.until = until


def _when_schedule(model, phase: PhaseNodes) -> list:
    """The calls `can_start` runs, in their own scope: a step evaluated in both phases emits in
    both, so the when block never shares the active block's emitted-once set.
    """
    scope = Schedule(model)
    live = [
        node
        for node in phase.evaluators["when"]
        if not _is_elapsed_constraint(model, model.graph.value(node, CSTR_HDL["constraint"]))
    ]
    steps = scope.of(live, OPS_GENERIC + OPS_HANDLER)
    # can_start inlines the when evaluators, but their prerequisite generic ops still need
    # scheduling, and the evaluators themselves are not reachable from anything.
    for node in live:
        if scope.claim(model.id(node)):
            steps.append(model.id(node))

    return steps


def _motion_schedules(
    model, phase, handler, chain_solvers, derivation, groups, computation
) -> MotionSchedules:
    """The three schedules a motion runs, in loop order."""
    graph = model.graph
    scope = Schedule(model)

    def live(nodes):
        return [
            node
            for node in nodes
            if not _is_elapsed_constraint(model, graph.value(node, CSTR_HDL["constraint"]))
        ]

    # Until monitors run before control each tick, so build the until schedule first: a quantity
    # an until monitor consumes must be scheduled in the earlier phase, or the monitor reads the
    # previous tick's value on its first tick.
    until = scope.of(live(phase.evaluators["until"]), OPS_GENERIC + OPS_HANDLER)
    # Until evaluators have no controller error signal to drive backward discovery, so append
    # them after their dependencies to keep the emitted call order right.
    for node in live(phase.evaluators["until"]):
        if scope.claim(model.id(node)):
            until.append(model.id(node))

    grouped_ids = {component.eval_id for group in groups for component in group.components}
    grouped_nodes = {node for node in phase.evaluators["while"] if model.id(node) in grouped_ids}
    # A grouped evaluator's error is emitted inline ahead of the schedule block, but whatever
    # produces its reference still has to run first -- so walk the grouped nodes before the main
    # pass rather than skipping those producers entirely.
    while_pre = [
        step
        for step in scope.of(sorted(grouped_nodes, key=str), OPS_GENERIC + OPS_HANDLER)
        if step not in grouped_ids
    ]

    active_plans = _active_plans(phase, derivation)
    by_constraint = {plan.constraint: plan for plan in active_plans}
    leading, trailing = [], []
    for node in live(phase.evaluators["while"]):
        if node in grouped_nodes:
            continue
        plan = by_constraint.get(graph.value(node, CSTR_HDL.constraint))
        feed_forward = plan is not None and CSTR_HDL_EXT.FeedForwardController in get_node_types(
            graph, plan.controller
        )
        (trailing if plan is None or feed_forward else leading).append(node)

    active = scope.of(leading, OPS_GENERIC + OPS_HANDLER)
    # The controller calls themselves are appended in authored order below, so drop whatever the
    # backward walk found of them.
    controller_ids = {
        controller.id for plan in active_plans for controller in derivation.controllers_for(plan)
    } | {model.id(plan.controller) for plan in active_plans}
    active = [step for step in active if step not in controller_ids]
    _append_new(active, _pose_command_steps(model, scope, active_plans))
    _append_new(active, [model.id(node) for node in leading])
    force_nodes = _cartesian_force_nodes(model, chain_solvers, handler, computation)
    _append_new(active, scope.of(force_nodes, OPS_GENERIC + OPS_SOLVER + OPS_HANDLER))
    active.extend(scope.of(trailing, OPS_GENERIC + OPS_HANDLER))
    _append_new(active, [model.id(node) for node in trailing])

    return MotionSchedules(_when_schedule(model, phase), while_pre, active, until)


def _pose_command_steps(model, scope, active_plans) -> list:
    """The interpolation and difference calls a per-axis pose command adds to the active block."""
    graph = model.graph
    steps = []
    for plan in active_plans:
        if len(plan.axes) <= 1:
            continue
        reference = graph.value(plan.constraint, CSTR["reference-value"])
        reference_view = next(graph.subjects(MAP.subobject, reference), None)
        interpolation = next(
            graph.subjects(GEOM_OP.out, graph.value(reference_view, MAP.superobject)), None
        )
        if interpolation is not None:
            steps.extend(scope.of([interpolation], OPS_GENERIC + OPS_HANDLER))
        steps.append(
            SolverIdFactory(
                model.id(plan.controller), model.motion_suffix(plan.motion)
            ).pose_evaluator()
        )

    return steps


def _active_plans(phase: PhaseNodes, derivation) -> list:
    """The authored controllers whose constraints this motion holds during."""
    plans = derivation.controllers_by_handler.get(phase.handler_node, ())
    return [plan for plan in plans if plan.constraint in phase.constraints["while"]]


def build_motions(model, handlers, robots, computation, derivation, fsm):
    """Build one motion unit per handler, then fold on what only the whole set decides.

    Returns:
        `(motions, fsm_meta)`: the motions in the order the handlers declare them, and the FSM
        wiring codegen needs alongside the framed FSM
    """
    motions = []
    tokens = {
        model.motion_suffix(model.graph.value(model.node_by_id[handler.id], CSTR_HDL["motion"]))
        for handler in handlers
    }
    # A per-motion slice copies nothing off its solver: `solver_id` resolves through the
    # resources index, exactly as templates do through `resources.by_id`.
    solvers_by_id = robots.by_id
    for handler in handlers:
        handler_node = model.node_by_id[handler.id]
        motion_node = model.graph.value(handler_node, CSTR_HDL["motion"])
        phase = PhaseNodes(model, handler, handler_node, motion_node)
        # A declared solver counts even when no controller routes to it: a monitor-only handler
        # still owns its arm runtime for state reading, FK and command forwarding.
        solver_ids = {
            model.id(plan.solver)
            for plan in derivation.controllers_by_handler.get(handler_node, ())
        } | {
            model.id(solver)
            for solver in model.graph.objects(handler_node, CSTR_HDL_EXT["runs-solver"])
        }
        chain_solvers = _handler_chain_solvers(handler, robots.serial_chains, solver_ids)
        evaluators = {
            name: [constraint_evaluator(model, node) for node in phase.evaluators[name]]
            for name in _PHASES
        }
        groups = quantities.pose_axis_error_groups_for_motion(
            model, dict(zip(phase.evaluators["while"], evaluators["while"])), computation.views
        )
        schedules = _motion_schedules(
            model, phase, handler, chain_solvers, derivation, groups, computation
        )
        active_controllers = [
            controller
            for plan in _active_plans(phase, derivation)
            for controller in derivation.controllers_for(plan)
        ]
        # Drop the calls another motion owns: the backward walk can reach its closures, and
        # running them here would recompute its outputs while it is inactive.
        token = model.motion_suffix(motion_node)
        owner = computation.indexes.closure_owner
        schedules.active = [step for step in schedules.active if owner.get(step, token) == token]
        schedules.while_pre = [
            step for step in schedules.while_pre if owner.get(step, token) == token
        ]
        _append_new(schedules.active, [c.id for c in reversed(active_controllers)])
        motions.append(
            _motion_unit(
                model,
                handler,
                phase,
                chain_solvers,
                robots.serial_chains,
                evaluators,
                groups,
                schedules,
                active_controllers,
                derivation,
                computation,
                motion_node,
                tokens,
                solvers_by_id,
            )
        )

    return _finish_motions(model, motions, handlers, computation, fsm, solvers_by_id)


def _motion_unit(
    model,
    handler,
    phase,
    chain_solvers,
    runtime_solvers,
    evaluators,
    groups,
    schedules,
    active_controllers,
    derivation,
    computation,
    motion_node,
    tokens,
    solvers_by_id,
):
    """One motion's IR unit: its evaluators, monitors, schedules and everything it captures."""
    all_evaluators = evaluators["while"] + evaluators["when"] + evaluators["until"]
    when_elapsed = quantities.elapsed_coordinate_ids(evaluators["when"])
    active_elapsed = quantities.elapsed_coordinate_ids(evaluators["while"] + evaluators["until"])

    return MotionUnit(
        id=handler.motion.id,
        handler=handler.id,
        name=handler.motion.name,
        description=(handler.motion.description or "").splitlines(),
        has_when_elapsed=bool(when_elapsed),
        has_active_elapsed=bool(active_elapsed),
        when_elapsed_ids=when_elapsed,
        active_elapsed_ids=active_elapsed,
        when_evaluators=evaluators["when"],
        while_evaluators=evaluators["while"],
        until_evaluators=evaluators["until"],
        controllers=active_controllers,
        when_monitors=[monitor_entry(model, node) for node in phase.monitors["when"]],
        while_monitors=[monitor_entry(model, node) for node in phase.monitors["while"]],
        until_monitors=[monitor_entry(model, node) for node in phase.monitors["until"]],
        when_schedule=schedules.when,
        while_schedule=schedules.active,
        until_schedule=schedules.until,
        has_elapsed=bool(when_elapsed or active_elapsed),
        has_until_condition=bool(evaluators["until"]),
        until_any=handler.motion.until_any,
        when_any=handler.motion.when_any,
        serial_chain_solvers=chain_solvers,
        relative_poses=quantities.relative_poses_for_motion(
            all_evaluators, computation.views, chain_solvers
        ),
        scene_relative_poses=quantities.scene_relative_poses_for_motion(
            computation.views, chain_solvers, solvers_by_id, all_evaluators
        ),
        pose_axis_error_groups=groups,
        while_pre_schedule=schedules.while_pre,
        forwarded_commands=_forwarded_commands(
            model, phase, chain_solvers, runtime_solvers, derivation
        ),
        snapshots=quantities.snapshots_for_motion(
            all_evaluators,
            handler.motion.while_ + handler.motion.when + handler.motion.until,
            computation.indexes,
            computation.views,
            schedules.active + schedules.when + schedules.until,
            computation.closures,
            model.motion_suffix(motion_node),
            tokens,
        ),
        path_projections=path_projections_for_motion(
            schedules.while_pre + schedules.active, computation.closures
        ),
    )


def _forwarded_commands(model, phase, chain_solvers, runtime_solvers, derivation) -> list:
    """The controller outputs written straight to a joint rather than through a solver."""
    graph = model.graph
    # Resolved from the joint, not the agent: a gripper's joint rides the arm's runtime.
    owned_trees = {solver.id: solver.runtime.owned_trees or () for solver in runtime_solvers}
    commands = []
    for plan in _active_plans(phase, derivation):
        if not issubclass(derivation.algorithm_by_solver[plan.solver], CommandForwarding):
            continue
        controller = derivation.controllers_for(plan)[0]
        quantity = graph.value(plan.constraint, CSTR.quantity)
        view = next(graph.subjects(MAP.subobject, quantity), None)
        target_quantity = graph.value(view, MAP.superobject) if view is not None else quantity
        target = graph.value(target_quantity, KC_STAT["of-joint"])
        chain_solver = next(
            (
                solver
                for solver in chain_solvers
                if target is not None
                and any(iri_is_descendant(tree, target) for tree in owned_trees.get(solver.id, ()))
            ),
            None,
        )
        if chain_solver is None:
            raise RuntimeError(
                "command forwarding: joint "
                f"'{model.label(target) if target is not None else target}' belongs to no "
                "kinematic tree this handler's runtimes own"
            )
        runtime = next(s for s in runtime_solvers if s.id == chain_solver.id)
        commands.append(
            ForwardedCommandStep(
                f"cmd-fwd-{model.id(plan.controller)}",
                controller.control_signal,
                f"{runtime.runtime.prefix}{model.label(target)}" if target is not None else "",
                chain_solver.id,
            )
        )

    return commands


# Per superobject type, the branch flags a pose-axis error group renders through.
_GROUP_TYPE_FLAGS = {
    "is_pose": ("Pose",),
    "is_twist": ("VelocityTwist", "AccelerationTwist"),
    "is_wrench": ("Wrench",),
}


def _finish_motions(model, motions, handlers, computation, fsm, solvers_by_id):
    """Fold on everything that needs the whole set of motions to be known.

    Raises:
        ConstraintViolation: two handlers govern one motion, so nothing decides which drives it.
    """
    by_motion: dict[str, str] = {}
    for motion in motions:
        if motion.id in by_motion:
            raise ConstraintViolation(
                "coordination",
                f"Motion '{motion.id}' is governed by more than one constraint handler "
                f"('{by_motion[motion.id]}' and '{motion.handler}'). Each motion must map to "
                f"exactly one handler; split the motion or merge the handlers.",
            )
        by_motion[motion.id] = motion.handler

    order_by_handler = {handler.id: handler.order for handler in handlers}
    ordered = sorted(motions, key=lambda motion: order_by_handler[motion.handler])
    for motion in ordered:
        _set_motion_conditions(motion)
        for group in motion.pose_axis_error_groups:
            for flag, types in _GROUP_TYPE_FLAGS.items():
                setattr(group, flag, group.superobject_type in types)
        annotate_controller_signals(motion.controllers, computation.closures)
        motion.declared_pose_components = quantities.declared_pose_component_entries(
            model,
            computation.data_structures,
            computation.indexes.pose_components,
            quantities.collect_motion_references(motion, computation.closures),
        )
    # Ordered: the FSM wiring tags monitors and motions, then the capability booleans, then the
    # gate calls that read them.
    meta = _apply_fsm_wiring(ordered, fsm)
    _add_motion_function_interfaces(ordered, solvers_by_id)
    _apply_fsm_gate_calls(ordered, meta["cpp_namespace"])

    return ordered, meta


def evaluator_term(evaluator) -> dict:
    """The boolean term one evaluator contributes to a condition.

    An elapsed timing predicate or a solver constraint-satisfied check. Every term kind reads
    shared state and nothing else, so the same condition renders identically inside the motion and
    in the introspection sample that runs outside it.
    """
    if evaluator.is_elapsed:
        operator = evaluator.elapsed_op or ">="
        threshold = evaluator.elapsed_threshold_s or 0.0
        elapsed_id = quantities.elapsed_coordinate_id(evaluator)
        # Pre-format the threshold so the emitted literal is stable.
        if operator == "==":
            tolerance = evaluator.elapsed_tolerance_s or 0.0

            return {
                "kind": "elapsed-eq",
                "elapsed_id": elapsed_id,
                "threshold": f"{threshold:.6f}",
                "tolerance": f"{tolerance:.6f}",
            }

        return {
            "kind": "elapsed",
            "elapsed_id": elapsed_id,
            "op": operator,
            "threshold": f"{threshold:.6f}",
        }

    term = {"kind": "constraint", "error_id": getattr(evaluator.error, "id", None)}
    # Omitted, not empty: ST4 reads an empty string as present, and would emit a bare access.
    tolerance_id = getattr(evaluator.tolerance, "id", None)
    if tolerance_id:
        term["tolerance_id"] = tolerance_id

    return term


def _stamp_terms(monitor, terms, any_flag) -> None:
    monitor.active_terms = terms
    monitor.active_terms_present = bool(terms)
    monitor.active_any = any_flag
    monitor.has_active = True


def _set_monitor_conditions(motion, phase: str) -> None:
    """Stamp one phase's terms onto the aggregate monitor and any elapsed-error monitors."""
    evaluators = getattr(motion, f"{phase}_evaluators")
    any_flag = bool(getattr(motion, f"{phase}_any"))
    aggregate_field = f"is_{phase}_aggregate"
    terms = [
        evaluator_term(evaluator)
        for evaluator in evaluators
        if evaluator.error or evaluator.is_elapsed
    ]
    elapsed_by_error = {
        evaluator.error.id: evaluator_term(evaluator)
        for evaluator in evaluators
        if evaluator.is_elapsed and evaluator.error
    }
    for monitor in getattr(motion, f"{phase}_monitors"):
        group_ids = set(monitor.group_constraint_ids or ())
        if group_ids:
            _stamp_terms(
                monitor,
                [
                    evaluator_term(evaluator)
                    for evaluator in evaluators
                    if evaluator.constraint.id in group_ids
                    and (evaluator.error or evaluator.is_elapsed)
                ],
                bool(monitor.group_any),
            )
            continue
        if getattr(monitor, aggregate_field):
            _stamp_terms(monitor, terms, any_flag)
            continue
        error_id = getattr(monitor.error, "id", None)
        if error_id in elapsed_by_error:
            _stamp_terms(monitor, [elapsed_by_error[error_id]], False)


def _set_motion_conditions(motion) -> None:
    """Fold the until, when and done boolean terms onto a motion."""
    _set_monitor_conditions(motion, "until")
    motion.when_terms = [
        evaluator_term(evaluator)
        for evaluator in motion.when_evaluators
        if evaluator.error or evaluator.is_elapsed
    ]
    motion.when_terms_present = bool(motion.when_terms)
    _set_monitor_conditions(motion, "when")
    # An edge monitor's occurrence ends the motion; a level monitor holds a flag instead.
    motion.done_terms = [
        {"kind": "event", "motion_id": motion.id, "monitor_id": monitor.id}
        if monitor.is_edge_triggered
        else {"kind": "flag", "motion_id": motion.id, "flag": monitor.flag}
        for monitor in motion.until_monitors
    ]
    motion.done_terms_present = bool(motion.done_terms)


def _add_motion_function_interfaces(motions: list, solvers_by_id: dict) -> None:
    """Fold the capability booleans each generated function's signature is built from.

    Also assigns each motion its introspection index, so the frame-log schema and the generated
    sample switch read one field rather than agreeing with a second generator.
    """
    for index, motion in enumerate(motions):
        motion.index = index
        when_mons = motion.when_monitors
        until_mons = motion.until_monitors
        has_when_elapsed = any(evaluator.is_elapsed for evaluator in motion.when_evaluators)
        when_sched = bool(motion.when_schedule)
        until_sched = bool(motion.until_schedule)
        when_fsm = any(monitor.fsm_namespace for monitor in when_mons)
        until_fsm = any(monitor.fsm_namespace for monitor in until_mons)
        has_chain = bool(motion.serial_chain_solvers)

        motion.can_start_needs_state = has_when_elapsed
        motion.can_start_needs_shared = bool(when_sched or motion.when_evaluators)
        motion.can_start_needs_robot = False

        motion.when_needs_state = has_when_elapsed or bool(when_mons)
        motion.when_needs_shared = (
            has_when_elapsed
            or bool(motion.declared_pose_components)
            or when_sched
            or bool(when_mons)
        )
        motion.when_needs_robot = when_fsm

        motion.until_needs_state = bool(until_mons)
        motion.until_needs_shared = until_sched or bool(until_mons)
        motion.until_needs_robot = until_fsm

        motion.monitor_needs_state = bool(when_mons) or bool(until_mons)
        motion.monitor_needs_shared = (
            when_sched or bool(when_mons) or until_sched or bool(until_mons)
        )
        motion.monitor_needs_robot = when_fsm or until_fsm

        motion.apply_needs_state = has_chain
        # Gate on the torque limit, not on the joint-space samples: this runs before the
        # introspection artifact exists, so the sample list does not yet.
        motion.apply_needs_shared = bool(motion.forwarded_commands) or any(
            solvers_by_id[solver.solver_id].torque_saturation
            for solver in motion.serial_chain_solvers
        )
        motion.apply_needs_robot = has_chain or bool(motion.forwarded_commands)

        # An edge is the occurrence: a flag monitor holds a level and never reaches the buffer.
        when_events = any(monitor.is_edge_triggered for monitor in when_mons)
        until_events = any(monitor.is_edge_triggered for monitor in until_mons)
        control_events = any(monitor.is_edge_triggered for monitor in motion.while_monitors)
        motion.when_needs_events = when_events
        motion.until_needs_events = until_events
        motion.monitor_needs_events = when_events or until_events
        motion.control_needs_events = control_events
        motion.step_needs_events = until_events or control_events


def read_fsm(model) -> dict | None:
    """The FSM named graph, framed the way codegen reads it.

    States, events, transitions and reactions, in the same shape the standalone header uses, so
    codegen needs no second read of the FSM document. None when the model imports no `.fsm`.

    Returns:
        the framed FSM, with every table sorted -- event indices are assigned from this order and
        baked into the generated C++, so two generations must agree on it
    """
    graph = model.graph
    fsm_node = next(iter(graph.subjects(RDF["type"], URI_FSM_TYPE_FSM)), None)
    if fsm_node is None:
        return None

    def token(uri):
        return get_valid_var_name(graph.compute_qname(uri)[2]).upper()

    state_uris = dict(
        sorted((token(s), str(s)) for s in graph.objects(fsm_node, URI_FSM_PRED_STATES))
    )
    event_loop = graph.value(fsm_node, NS_MM_EL["event-loop"])
    event_uris = dict(
        sorted((token(e), str(e)) for e in graph.objects(event_loop, NS_MM_EL["has-event"]))
    )
    transitions = sorted(
        (
            {
                "id": token(node),
                "uri": str(node),
                "from_state": token(graph.value(node, URI_FSM_PRED_TRANSITION_FROM)),
                "to_state": token(graph.value(node, URI_FSM_PRED_TRANSITION_TO)),
            }
            for node in graph.objects(fsm_node, URI_FSM_PRED_TRANSITIONS)
        ),
        key=lambda row: row["id"],
    )
    reactions = sorted(
        (
            {
                "id": token(node),
                "uri": str(node),
                "when_event": token(graph.value(node, NS_MM_EL["ref-event"])),
                "do_transition": token(graph.value(node, URI_FSM_PRED_DO_TRANSITION)),
                "fires_events": sorted(
                    token(event) for event in graph.objects(node, URI_FSM_PRED_FIRES_EVENTS)
                ),
                "num_fires": len(list(graph.objects(node, URI_FSM_PRED_FIRES_EVENTS))),
            }
            for node in graph.objects(fsm_node, URI_FSM_PRED_REACTIONS)
        ),
        key=lambda row: row["id"],
    )
    description = graph.value(fsm_node, URI_FSM_PRED_DESCRIPTION)

    return {
        "name": str(graph.value(fsm_node, URI_FSM_PRED_NAME)),
        "description": str(description) if description is not None else None,
        "start_state": token(graph.value(fsm_node, URI_FSM_PRED_START_STATE)),
        "end_state": token(graph.value(fsm_node, URI_FSM_PRED_END_STATE)),
        "states": list(state_uris),
        "state_uris": state_uris,
        "events": list(event_uris),
        "event_uris": event_uris,
        "transitions_table": transitions,
        "reactions_table": reactions,
        # Event and state IRIs share the FSM node's parent path; a monitor's event is matched
        # against it to tell an FSM event from a monitor-owned one.
        "namespace_uri": str(model.child_node(iri_parent(fsm_node), "")),
    }


def _apply_fsm_wiring(motions, fsm) -> dict:
    """Tag the monitors that fire the FSM, and return the wiring codegen needs beside it.

    Raises:
        ConstraintViolation: a motion declares no `until` and the model imports no FSM, so nothing
            can end it; a snapshot triggers on an event the FSM does not declare; or a WHEN-gated
            motion names no hold motion, or an unknown one.
    """
    namespace = fsm["name"].lower() if fsm else None
    events = fsm.get("events", []) if fsm else []
    index_by_event = {event: index for index, event in enumerate(events)}
    step_event = "E_STEP" if "E_STEP" in events else None
    # The heartbeat is the clock and is not logged every tick, but where a transition's guard is
    # the clock, that occurrence caused the state change -- so name those transitions.
    transitions = {row["id"]: row for row in (fsm.get("transitions_table", []) if fsm else [])}
    meta = {
        "cpp_namespace": namespace,
        "header": f"{fsm['name']}.hpp" if fsm else None,
        "step_event": step_event,
        "step_event_idx": index_by_event.get(step_event, -1),
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
                f"motions {sorted(stuck)} declare no 'until' condition and the model imports no "
                "FSM, so nothing can end them; add an 'until' condition or coordinate the model "
                "with an FSM",
            )

        return meta

    namespace_uri = fsm.get("namespace_uri")
    state_by_event = {
        reaction["when_event"]: transitions[reaction["do_transition"]]["from_state"]
        for reaction in fsm["reactions_table"]
        if reaction["do_transition"] in transitions
    }
    by_id = {motion.id: motion for motion in motions}

    def fires_fsm_event(monitor) -> bool:
        """A monitor fires the FSM only when its event lives in the FSM's namespace; a
        standalone, monitor-owned event keeps its own stub."""
        return bool(
            monitor.is_edge_triggered
            and iri_is_descendant(namespace_uri or "", monitor.event_uri or "")
        )

    def stamp(monitor):
        monitor.fsm_namespace = namespace
        monitor.fsm_event_idx = index_by_event.get(monitor.event_name or "", -1)

    for motion in motions:
        # An event-triggered snapshot only compiles when the FSM declares the event it waits on.
        for snapshot in motion.snapshots:
            if not snapshot.trigger_event:
                continue
            if snapshot.trigger_event not in index_by_event:
                raise ConstraintViolation(
                    "coordination",
                    f"Snapshot '{snapshot.target_id}' in motion '{motion.id}' triggers on "
                    f"'{snapshot.trigger_event}', which the FSM '{namespace}' does not declare.",
                )
            snapshot.fsm_namespace = namespace
        for monitor in [*motion.until_monitors, *motion.while_monitors]:
            if not fires_fsm_event(monitor):
                continue
            stamp(monitor)
            state = state_by_event.get(monitor.event_name or "")
            if state and not motion.fsm_state:
                motion.fsm_state = state
        for monitor in motion.when_monitors:
            if not fires_fsm_event(monitor):
                continue
            stamp(monitor)
            fallback = _when_gate_fallback(motion, monitor, by_id)
            state = state_by_event.get(monitor.event_name or "")
            if state and not fallback.fsm_state:
                fallback.fsm_state = state
            if motion.id not in fallback.fsm_when_gate_motions:
                fallback.fsm_when_gate_motions.append(motion.id)

    return meta


def _when_gate_fallback(motion, monitor, by_id):
    """The hold motion that runs while a WHEN-gated motion waits for its event."""
    if not monitor.fallback_motion:
        raise ConstraintViolation(
            "coordination",
            f"WHEN monitor '{monitor.id}' on FSM-wired motion '{motion.id}' must declare a "
            "waiting hold motion (e.g. '... otherwise hold <hold-motion>'). A WHEN precondition "
            "without a fallback would leave the arm uncommanded while waiting.",
        )
    fallback = by_id.get(monitor.fallback_motion)
    if fallback is None:
        raise ConstraintViolation(
            "coordination",
            f"WHEN monitor '{monitor.id}' names unknown fallback motion "
            f"'{monitor.fallback_motion}'.",
        )

    return fallback


def _apply_fsm_gate_calls(motions, namespace) -> None:
    """Fold each hold motion's WHEN-evaluation gate calls.

    The gated motion's id plus its when-signature capability booleans, so the generated call is
    built from the same flags the function's own signature is.
    """
    if namespace is None:
        return
    by_id = {motion.id: motion for motion in motions}
    for fallback in motions:
        if not fallback.fsm_when_gate_motions:
            continue
        fallback.fsm_when_gate_calls = [
            {
                "gid": gate_id,
                "needs_state": by_id[gate_id].when_needs_state,
                "needs_shared": by_id[gate_id].when_needs_shared,
                "needs_robot": by_id[gate_id].when_needs_robot,
                "needs_events": by_id[gate_id].when_needs_events,
            }
            for gate_id in fallback.fsm_when_gate_motions
            if gate_id in by_id
        ]
