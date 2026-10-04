# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What runs when.

In order: the readers for a handler and the evaluators, monitors and motion it binds; the
handlers themselves; the four steps a motion is built in -- which constraints belong to each
phase, the three schedules, the motion unit, and what is folded on once every motion exists; and
the boolean terms a condition renders from. What a monitor publishes is ``ros_messages.py``'s,
and the FSM wiring ``fsm.py``'s.

Nothing else in the package sequences anything.
"""

from __future__ import annotations

from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT,
    APP,
    CSTR,
    CSTR_EXT,
    CSTR_HDL,
    CSTR_HDL_EXT,
    GEOM_OP,
    GEOM_OP_EXT,
    KC_STAT,
    MAP,
    MOT,
    SLV,
    SOSA,
    TIME,
)
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdf_utils.uri import iri_is_descendant
from rdflib import Literal
from rdflib.namespace import RDF, SDO
from scene_dsl.rdf_parser.common import ensure_one_typed_subject_uri

from motion_spec.classes.constraints import ConstraintTransition, GuardedMotion
from motion_spec.classes.geometry import Wrench
from motion_spec.classes.handlers import (
    ConstraintEvaluator,
    ConstraintHandler,
    EdgeMonitor,
    EvaluatorType,
    LevelMonitor,
)
from motion_spec.classes.motion import ForwardedCommandStep, MotionSolverSlice, MotionUnit
from motion_spec.rdf_parser import perturbations, quantities
from motion_spec.rdf_parser.constraint_handler import (
    SolverIdFactory,
    alignment_chain_ops,
    alignment_gradient_op,
    alignment_rotation_op,
    annotate_controller_signals,
)
from motion_spec.rdf_parser.fsm import apply_fsm_gate_calls, apply_fsm_wiring
from motion_spec.rdf_parser.model import reader
from motion_spec.rdf_parser.operations import (
    OPS_GENERIC,
    OPS_HANDLER,
    OPS_SOLVER,
    Schedule,
    path_projections_for_motion,
)
from motion_spec.rdf_parser.ros_messages import ros_publication
from motion_spec.rdf_parser.views import (
    collect_motion_input_references,
    collect_motion_references,
    declared_pose_component_entries,
    elapsed_coordinate_id,
    elapsed_coordinate_ids,
    expanded_constraints,
    observation_ages,
    pose_axis_error_groups_for_motion,
    snapshots_for_motion,
)

_PHASES = ("when", "while", "until")
_PHASE_PREDICATES = {"when": MOT["when"], "while": MOT["while"], "until": MOT["until"]}


def _is_elapsed_constraint(model, node) -> bool:
    """Whether a constraint is a timing constraint, measured against the clock rather than a solver."""
    return node is not None and CSTR_EXT["TimeConstraint"] in get_node_types(model.graph, node)


def _observation_instant(model, constraint_node) -> str | None:
    """The id of the instant an age constraint counts from, or None for one counting from entry.

    The constraint states the interval it measures; its beginning is an observation instant when a
    world quantity states it as its sosa:phenomenonTime.

    Raises:
        ConstraintViolation: the instant belongs to a quantity nothing perceives -- no subscription
            observes it and no detect act writes it -- so no reading could ever fill it.
    """
    graph = model.graph
    interval = graph.value(constraint_node, TIME.hasTime)
    begin = graph.value(interval, TIME.hasBeginning) if interval is not None else None
    observed = (
        graph.value(predicate=SOSA.phenomenonTime, object=begin) if begin is not None else None
    )
    if observed is None:
        return None
    perceived = {
        row["pose_id"]
        for rows in quantities.perceived_written_poses(model).values()
        for row in rows
    }
    if model.id(observed) not in perceived:
        raise ConstraintViolation(
            "coordination",
            f"'{model.id(constraint_node)}' measures the time since '{model.id(observed)}' was "
            "observed, but nothing observes it: no subscriber names it and no detect act writes "
            "it, so its observation instant can never be filled.",
        )
    return model.id(begin)


def _constraint_transition(model, node) -> ConstraintTransition:
    """One when/until object as a transition: an expression node over its members, or a lone
    constraint standing for itself.
    """
    if not quantities.is_constraint_aggregate(model, node):
        return ConstraintTransition(model.id(node), False, (quantities.constraint(model, node),))

    return ConstraintTransition(
        model.id(node),
        CSTR_EXT.ConstraintDisjunction in get_node_types(model.graph, node),
        tuple(
            quantities.constraint(model, member)
            for member in model.graph[node : CSTR_EXT["has-constraint"]]
        ),
    )


@reader
def guarded_motion(model, node) -> GuardedMotion:
    """A GuardedMotion: the constraints it starts on, holds during and ends on.

    Each when/until object is one transition, so a named group and a whole-section expression
    keep their own logic rather than collapsing into one flag for the phase.

    Raises:
        ConstraintViolation: the motion carries no `schema:name`, so nothing can name its step
            function.
    """
    graph = model.graph
    name = graph.value(node, SDO.name)
    if name is None:
        raise ConstraintViolation("coordination", f"GuardedMotion {node} has no schema:name triple")
    description = graph.value(node, SDO.description)

    return GuardedMotion(
        model.id(node),
        [_constraint_transition(model, item) for item in graph[node : MOT["when"]]],
        [quantities.constraint(model, item) for item in graph[node : MOT["while"]]],
        [_constraint_transition(model, item) for item in graph[node : MOT["until"]]],
        name=str(name),
        description=str(description) if description is not None else None,
    )


def constraint_handler(model, node) -> ConstraintHandler:
    """A ConstraintHandler: the motion it governs, and the evaluators and monitors it binds."""
    graph = model.graph

    return ConstraintHandler(
        model.id(node),
        guarded_motion(model, graph.value(node, CSTR_HDL["motion"])),
        [constraint_evaluator(model, item) for item in graph[node : CSTR_HDL["evaluators"]]],
        [],
        [monitor_entry(model, item) for item in graph[node : CSTR_HDL["monitors"]]],
        int(graph.value(node, APP.order, default=Literal(0)).value),
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
    graph = model.graph
    constraint_node = graph.value(node, CSTR_HDL["constraint"])
    assignment = CSTR_HDL["AssignmentEvaluator"] in get_node_types(graph, node)
    error_node = None if assignment else graph.value(node, CSTR_HDL["error"])

    status_slot = quantities.goal_status_act(model, graph.value(constraint_node, CSTR["quantity"]))
    goal_status = None
    if status_slot is not None:
        reference = graph.value(constraint_node, CSTR["reference-value"])
        goal_status = str(graph.value(reference, RDF.value))

    is_elapsed = _is_elapsed_constraint(model, constraint_node)
    operator, threshold, elapsed_tolerance, observed_at = None, None, None, None
    if is_elapsed:
        observed_at = _observation_instant(model, constraint_node)
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
    # because a duration's magnitude rides on qudt rather than on a D-block.
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
        observed_at_id=observed_at,
        goal_status=goal_status,
    )


def _monitored_expression(model, monitored):
    """The expression node a monitor targets: its member ids, IRIs, tolerances, and its logic.

    A named group and a whole-section conjunction/disjunction are the same thing here: one
    condition carrying its own members and join, which the monitor's terms are built from.
    """
    expressions = [node for node in monitored if quantities.is_constraint_aggregate(model, node)]
    if not expressions:
        return [], [], [], False
    if len(expressions) > 1:
        raise ConstraintViolation(
            "coordination",
            f"a monitor watches {len(expressions)} constraint groups -- it watches one condition",
        )
    expression = expressions[0]
    nodes = list(model.graph[expression : CSTR_EXT["has-constraint"]])
    members = [model.id(member) for member in nodes]
    bands = [model.graph.value(member, CSTR_EXT["tolerance"]) for member in nodes]

    return (
        members,
        [str(member) for member in nodes],
        [model.id(band) if band is not None else "" for band in bands],
        CSTR_EXT.ConstraintDisjunction in get_node_types(model.graph, expression),
    )


@reader
def monitor_entry(model, node):
    """A monitor: a level flag its constraint sets continuously, or an edge event it fires once."""
    graph = model.graph
    handler = ensure_one_typed_subject_uri(
        graph, node, CSTR_HDL.monitors, CSTR_HDL.ConstraintHandler
    )
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
    group_ids, group_uris, group_bands, group_any = _monitored_expression(model, monitored)

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
        "group_constraint_uris": group_uris,
        "group_constraint_tolerances": group_bands,
        "group_any": group_any,
        "constraint_ids": [model.id(item) for item in monitored],
        "constraint_uris": [str(item) for item in monitored],
    }
    types = get_node_types(graph, node)
    publication = ros_publication(model, node)
    # A monitor that only publishes is neither edge- nor level-triggered: it names no signal,
    # it just reports the state of the constraint it watches.
    if CSTR_HDL["EdgeTriggeredMonitor"] not in types:
        flag_node = graph.value(node, CSTR_HDL["flag"])
        return LevelMonitor(
            id=model.id(node),
            monitor_type="LevelTriggeredMonitor",
            error=error,
            flag=model.id(flag_node) if flag_node is not None else None,
            **shared,
            **publication,
        )

    event_node = graph.value(node, CSTR_HDL["event"])
    event = model.id(event_node)
    fallback = graph.value(node, CSTR_HDL_EXT["fallback-motion"])
    debounce = graph.value(node, CSTR_HDL_EXT["debounce-duration"])

    return EdgeMonitor(
        id=model.id(node),
        monitor_type="EdgeTriggeredMonitor",
        error=error,
        event=event,
        event_idx=None,
        **shared,
        event_uri=str(event_node),
        fallback_motion=model.id(fallback) if fallback is not None else None,
        debounce_id=model.id(debounce) if debounce is not None else None,
        **publication,
    )


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
        key=lambda item: int(graph.value(item, APP.order, default=Literal(0)).value),
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
            SolverIdFactory(model, plan.controller).pose_evaluator()
            for plan in reversed(plans)
            if len(plan.axes) > 1 and alignment_rotation_op(model, plan.quantity) is None
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
            phase: expanded_constraints(model, self.raw[phase])
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
                if monitored <= self.raw[phase]:
                    return phase

            return next((p for p in _PHASES if monitored & self.watched[p]), None)
        error = graph.value(node, CSTR_HDL["error"])

        return next((p for p in _PHASES if error in self.errors[p]), None)

    def _check(self, model, handler, handler_node) -> None:
        """Every classified node must be one the handler declares.

        An evaluator or controller bound to no phase is silently excluded from all schedules,
        which is intended; inventing one that the handler never declared is not.
        """
        for kind, by_phase, declared_predicate in (
            ("evaluators", self.evaluators, CSTR_HDL["evaluators"]),
            ("monitors", self.monitors, CSTR_HDL["monitors"]),
        ):
            declared = set(model.graph[handler_node:declared_predicate])
            classified = {node for phase in _PHASES for node in by_phase[phase]}
            if not classified <= declared:
                raise ValueError(
                    f"Handler {handler.id}: classified {kind} not a subset of handler {kind}"
                )


def _upstream_dependencies(data_id: str, function_inputs: dict) -> set:
    """Every data id that feeds one id, however many functions deep."""
    result: set[str] = set()
    pending = list(function_inputs.get(data_id, ()))
    while pending:
        item = pending.pop()
        if item in result:
            continue
        result.add(item)
        pending.extend(function_inputs.get(item, ()))
    return result


def _handler_chain_solvers(handler, serial_chains, solver_ids) -> list:
    """The arm solvers this handler commands, sliced to the motion driver it drives them with.

    No copy of the solver's own facts (`algorithm`, `gravity`, `chain.root`, ...) -- a
    template reaches them through `solver_id` into `resources.by_id`.
    """
    result = []
    for solver in serial_chains:
        if solver.id not in solver_ids or not solver.motion_drivers:
            continue
        owned = [driver for driver in solver.motion_drivers if driver.handler == handler.id]
        if len(owned) != 1:
            raise ConstraintViolation(
                "coordination",
                f"solver '{solver.id}' carries {len(owned)} motion drivers of handler "
                f"'{handler.id}' -- a handler drives a solver through one",
            )
        selected = owned[0]
        result.append(
            MotionSolverSlice(
                id=solver.id,
                solver_id=solver.id,
                output=solver.output,
                motion_driver=selected,
                read_only=solver.algorithm.read_only,
            )
        )

    return result


def _cartesian_force_nodes(model, chain_solvers, handler, computation, handler_node=None) -> list:
    """The authored Cartesian forces a handler's own controllers ultimately drive."""
    graph = model.graph
    outputs = {controller.control_signal.id for controller in handler.controllers}
    found = []
    driver_nodes = [
        node
        for node in (model.node_by_id.get(solver.motion_driver.id) for solver in chain_solvers)
        if node is not None
    ]
    # A force distribution's drivers hang off the solver, not off a chain.
    if handler_node is not None:
        driver_nodes.extend(
            driver
            for solver_node in graph.objects(handler_node, CSTR_HDL_EXT["runs-solver"])
            if (solver_node, RDF.type, SLV["ForceDistributionSolver"]) in graph
            for driver in graph[solver_node : SLV["motion-drivers"]]
        )
    for driver_node in driver_nodes:
        for node in graph[driver_node : SLV["cartesian-force"]]:
            force = graph.value(node, SLV["force"])
            if force is None:
                continue
            force_id = model.id(force)
            upstream = {force_id} | _upstream_dependencies(
                force_id, computation.indexes.function_input
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
    """The three call sequences one motion runs, in the order the loop runs them.

    `commanded_force` is held back rather than folded into `active`: a commanded wrench is built
    from control signals, so it can only run once the control laws have written them this tick.
    """

    def __init__(
        self,
        when: list,
        while_pre: list,
        active: list,
        until: list,
        commanded_force: list | None = None,
    ):
        self.when = when
        self.while_pre = while_pre
        self.active = active
        self.until = until
        self.commanded_force = commanded_force or []


def _when_schedule(model, phase: PhaseNodes) -> list:
    """The calls `monitor_when` runs, in their own scope: a step evaluated in both phases emits in
    both, so the when block never shares the active block's emitted-once set.
    """
    scope = Schedule(model)
    live = [
        node
        for node in phase.evaluators["when"]
        if not _is_elapsed_constraint(model, model.graph.value(node, CSTR_HDL["constraint"]))
    ]
    steps = scope.of(live, OPS_GENERIC + OPS_HANDLER)
    # Nothing downstream reaches the when evaluators, so they are scheduled explicitly.
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

    # Elapsed and goal-status conditions have no function to schedule: the monitor reads the clock
    # or the act's status slot directly.
    live = {"until": [], "while": []}
    for kind, nodes in live.items():
        for node in phase.evaluators[kind]:
            constraint = graph.value(node, CSTR_HDL["constraint"])
            if _is_elapsed_constraint(model, constraint):
                continue
            quantity = graph.value(constraint, CSTR["quantity"])
            if quantities.goal_status_act(model, quantity) is not None:
                continue
            nodes.append(node)

    # Until monitors run before control each tick, so build the until schedule first: a quantity
    # an until monitor consumes must be scheduled in the earlier phase, or the monitor reads the
    # previous tick's value on its first tick.
    until = scope.of(live["until"], OPS_GENERIC + OPS_HANDLER)
    # Until evaluators have no controller error signal to drive backward discovery, so append
    # them after their dependencies to keep the emitted call order right.
    for node in live["until"]:
        if scope.claim(model.id(node)):
            until.append(model.id(node))

    grouped_ids = {component.eval_id for group in groups for component in group.components}
    grouped_nodes = [node for node in phase.evaluators["while"] if model.id(node) in grouped_ids]
    # A grouped evaluator's error is emitted inline ahead of the schedule block, but whatever
    # produces its reference still has to run first -- so walk the grouped nodes before the main
    # pass rather than skipping those producers entirely.
    while_pre = [
        step
        for step in scope.of(grouped_nodes, OPS_GENERIC + OPS_HANDLER)
        if step not in grouped_ids
    ]

    active_plans = _active_plans(phase, derivation)
    by_constraint = {plan.constraint: plan for plan in active_plans}
    leading, trailing = [], []
    for node in live["while"]:
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
    _append_new(active, _alignment_chain_steps(model, phase))
    _append_new(active, _pose_command_steps(model, scope, active_plans))
    _append_new(active, _expression_gradient_steps(model, scope, active_plans))
    _append_new(active, [model.id(node) for node in leading])
    force_nodes = _cartesian_force_nodes(
        model, chain_solvers, handler, computation, phase.handler_node
    )
    # Claimed here so the walk still resolves shared prerequisites in this position, but emitted
    # after the control laws run: the wrench reads the control signal they write this tick.
    commanded_force = scope.of(force_nodes, OPS_GENERIC + OPS_SOLVER + OPS_HANDLER)
    active.extend(scope.of(trailing, OPS_GENERIC + OPS_HANDLER))
    _append_new(active, [model.id(node) for node in trailing])
    # A perturbation is reachable from no controller and no monitor, so nothing else pulls the
    # ops composing its wrench or evaluating its gate into a schedule.
    for node in perturbations.nodes(model, phase.handler_node):
        gate = perturbations.evaluator_nodes(model, node)
        _append_new(active, scope.of(gate, OPS_GENERIC + OPS_HANDLER))
        _append_new(
            active, [model.id(evaluator) for evaluator in gate if scope.claim(model.id(evaluator))]
        )
        _append_new(active, scope.of(perturbations.compose_ops(model, node), OPS_GENERIC))

    return MotionSchedules(_when_schedule(model, phase), while_pre, active, until, commanded_force)


def _alignment_chain_steps(model, phase) -> list:
    """Compute ops for every alignment this motion holds during: rotated direction, angle, then
    the direction the constraint is driven along -- a rotation vector onto the reference, or, off
    zero, the gradient axis. Both are read through shared state, by a moment controller's wrench
    or by a solver row, so the backward walk from either never reaches these; the ops are emitted
    once for the whole model, and every motion that holds the constraint has to name them itself.
    """
    graph = model.graph
    steps = []
    for constraint in phase.constraints["while"]:
        quantity = graph.value(constraint, CSTR.quantity)
        if quantity is None:
            continue
        drive_op = alignment_rotation_op(model, quantity) or alignment_gradient_op(model, quantity)
        if drive_op is None:
            continue
        _append_new(steps, [model.id(op) for op in alignment_chain_ops(model, quantity)])
        _append_new(steps, [model.id(drive_op)])
    return steps


def _pose_command_steps(model, scope, active_plans) -> list:
    """The interpolation and difference calls a per-axis pose command adds to the active block."""
    graph = model.graph
    steps = []
    for plan in active_plans:
        if len(plan.axes) <= 1:
            continue
        # An alignment has no pose pair to interpolate or difference; its own chain is scheduled.
        if alignment_rotation_op(model, plan.quantity) is not None:
            continue
        reference = graph.value(plan.constraint, CSTR["reference-value"])
        reference_view = quantities.view_of(graph, reference)
        interpolation = next(
            graph.subjects(GEOM_OP.out, graph.value(reference_view, MAP.superobject)), None
        )
        if interpolation is not None:
            steps.extend(scope.of([interpolation], OPS_GENERIC + OPS_HANDLER))
        steps.append(SolverIdFactory(model, plan.controller).pose_evaluator())

    return steps


def _expression_gradient_steps(model, scope, active_plans) -> list:
    """The calls computing each controlled expression's unit gradient and the norm it came from.

    A solver row and the control law read them through shared state, so the backward walk from
    either never reaches them; walking back from the ops writing the unit vectors reaches the norm
    and every term they combine.
    """
    graph = model.graph
    units = [
        unit
        for plan in active_plans
        for op in graph.subjects(ALGO_EXT.out, plan.quantity)
        for predicate in (GEOM_OP_EXT.gradient, GEOM_OP_EXT["gradient-moment"])
        if (unit := graph.value(op, predicate)) is not None
    ]
    return scope.of(
        [op for unit in units for op in graph.subjects(ALGO_EXT.out, unit)], OPS_GENERIC
    )


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
        model.id(model.graph.value(model.node_by_id[handler.id], CSTR_HDL["motion"]))
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
        groups = pose_axis_error_groups_for_motion(
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
        # Drop the calls another motion owns: the backward walk can reach its functions, and
        # running them here would recompute its outputs while it is inactive.
        token = model.id(motion_node)
        owner = computation.indexes.function_owner
        schedules.active = [step for step in schedules.active if owner.get(step, token) == token]
        schedules.while_pre = [
            step for step in schedules.while_pre if owner.get(step, token) == token
        ]
        _append_new(schedules.active, [c.id for c in reversed(active_controllers)])
        _append_new(
            schedules.active,
            [step for step in schedules.commanded_force if owner.get(step, token) == token],
        )
        # What state runs this motion, when the model says so rather than leaving it derived from
        # the event that ends the motion -- which a motion meant to keep running never fires.
        runs_in = model.graph.value(handler_node, CSTR_HDL_EXT["runs-in-state"])
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
        if runs_in is not None:
            motions[-1].runs_in_state = str(runs_in)
        unit = motions[-1]
        unit.perturbations = _read_perturbations(
            model,
            handler_node,
            [(slice_.id, solvers_by_id[slice_.solver_id]) for slice_ in chain_solvers],
        )
        unit.entry_snapshots = [s for s in unit.snapshots if s.scope == "entry"]
        unit.task_snapshots = [s for s in unit.snapshots if s.scope == "task"]

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
    when_elapsed = elapsed_coordinate_ids(evaluators["when"])
    active_elapsed = elapsed_coordinate_ids(evaluators["while"] + evaluators["until"])
    when_ages = observation_ages(evaluators["when"])
    active_ages = observation_ages(evaluators["while"] + evaluators["until"])

    return MotionUnit(
        id=handler.id,
        motion_id=handler.motion.id,
        name=handler.motion.name,
        description=(handler.motion.description or "").splitlines(),
        when_elapsed_ids=when_elapsed,
        active_elapsed_ids=active_elapsed,
        when_observation_ages=when_ages,
        active_observation_ages=active_ages,
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
        has_elapsed=bool(when_elapsed or active_elapsed or when_ages or active_ages),
        has_until_condition=bool(evaluators["until"]),
        serial_chain_solvers=chain_solvers,
        pose_axis_error_groups=groups,
        while_pre_schedule=schedules.while_pre,
        forwarded_commands=_forwarded_commands(
            model, phase, chain_solvers, runtime_solvers, derivation
        ),
        snapshots=snapshots_for_motion(
            all_evaluators,
            handler.motion.while_ + handler.motion.when + handler.motion.until,
            computation.indexes,
            computation.views,
            # while_pre too: a grouped row's setpoint generator runs there and may read a snapshot.
            schedules.while_pre + schedules.active + schedules.when + schedules.until,
            computation.functions,
            model.id(motion_node),
            tokens,
        ),
        path_projections=path_projections_for_motion(
            schedules.while_pre + schedules.active, computation.functions
        ),
    )


def _forwarded_commands(model, phase, chain_solvers, runtime_solvers, derivation) -> list:
    """The controller outputs written straight to a joint rather than through a solver."""
    graph = model.graph
    # Resolved from the joint, not the agent: a gripper's joint rides the arm's runtime.
    owned_trees = {solver.id: solver.runtime.owned_trees or () for solver in runtime_solvers}
    commands = []
    for plan in _active_plans(phase, derivation):
        if not derivation.algorithm_by_solver[plan.solver].forwards_commands:
            continue
        controller = derivation.controllers_for(plan)[0]
        quantity = graph.value(plan.constraint, CSTR.quantity)
        view = quantities.view_of(graph, quantity)
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
            raise ConstraintViolation(
                "coordination",
                "command forwarding: joint "
                f"'{model.label(target) if target is not None else target}' belongs to no "
                "kinematic tree this handler's runtimes own",
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
    """Fold on everything that needs the whole set of motions to be known."""
    order_by_handler = {handler.id: handler.order for handler in handlers}
    ordered = sorted(motions, key=lambda motion: order_by_handler[motion.id])
    for motion in ordered:
        set_motion_conditions(motion)
        for group in motion.pose_axis_error_groups:
            for flag, types in _GROUP_TYPE_FLAGS.items():
                setattr(group, flag, group.superobject_type in types)
        annotate_controller_signals(
            motion.controllers, computation.functions, motion.while_evaluators
        )
        motion.declared_pose_components = declared_pose_component_entries(
            model,
            computation.data_structures,
            computation.indexes.pose_components,
            collect_motion_references(motion, computation.functions),
        )
    # Ordered: the FSM wiring tags monitors and motions, then the capability booleans, then the
    # gate calls that read them.
    meta = apply_fsm_wiring(ordered, fsm, solvers_by_id.values())
    _add_motion_function_interfaces(ordered, solvers_by_id)
    apply_fsm_gate_calls(ordered, meta["cpp_namespace"])

    return ordered, meta


def annotate_sensor_dependencies(motions, computation) -> None:
    """Resolve which mounted sensor readings each motion's computations consume."""
    for motion in motions:
        references = collect_motion_input_references(motion, computation.functions)
        referenced_outputs = {
            view.superobject.id
            for view in computation.views.values()
            if view.subobject.id in references
        }
        for solver in motion.serial_chain_solvers:
            solver.required_sensors = [
                output.sensor_name
                for output in solver.output
                if (output.id in references or output.id in referenced_outputs)
                and isinstance(output, Wrench)
                and output.sensor_name
            ]


def evaluator_term(evaluator) -> dict:
    """The boolean term one evaluator contributes to a condition.

    An elapsed timing predicate or a solver constraint-satisfied check. Every term kind reads
    shared state and nothing else, so the same condition renders identically inside the motion and
    in the telemetry sample that runs outside it.
    """
    if evaluator.goal_status:
        return {
            "kind": "goal-status",
            "status_id": evaluator.constraint.quantity.id,
            "value": evaluator.goal_status,
        }

    if evaluator.is_elapsed:
        operator = evaluator.elapsed_op or ">="
        threshold = evaluator.elapsed_threshold_s or 0.0
        elapsed_id = elapsed_coordinate_id(evaluator)
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

    term = {
        "kind": "constraint",
        "error_id": evaluator.error.id if evaluator.error is not None else None,
    }
    # Omitted, not empty: ST4 reads an empty string as present, and would emit a bare access.
    if evaluator.tolerance is not None and evaluator.tolerance.id:
        term["tolerance_id"] = evaluator.tolerance.id

    return term


def _read_perturbations(model, handler_node, chains) -> list:
    """The handler's perturbations, each with the boolean terms its gate opens on.

    The terms are folded here rather than in the reader: they are the same rows a monitor's
    condition is built from, so both go through `evaluator_term` and render identically.
    """
    records = []
    for node in perturbations.nodes(model, handler_node):
        record = perturbations.read(model, node, chains)
        record.terms = [
            evaluator_term(constraint_evaluator(model, evaluator))
            for evaluator in perturbations.evaluator_nodes(model, node)
        ]
        if record.has_gate and not record.terms:
            raise ConstraintViolation(
                "perturbation",
                f"perturbation '{record.id}' states a when-gate that lowered to no terms, so it "
                "renders as a constant false and the disturbance can never be applied",
            )
        records.append(record)

    return records


def _stamp_terms(monitor, terms, any_flag, where: str) -> None:
    """Give a monitor the boolean terms its condition is built from.

    Raises:
        ConstraintViolation: the condition lowered to no terms at all, which renders as a
            constant false -- a monitor that can never fire, and an FSM that can never leave the
            state it watches.
    """
    if not terms:
        raise ConstraintViolation(
            "coordination",
            f"monitor '{monitor.id}' ({where}) watches a condition that lowered to no terms, so "
            "it renders as a constant false: it can never fire, and the FSM can never leave the "
            "state it runs in. Every constraint it names is one nothing evaluates -- give it a "
            "constraint with an error to watch, or an elapsed time.",
        )
    monitor.active_terms = terms
    monitor.active_any = any_flag
    monitor.has_active = True


def set_motion_conditions(motion) -> None:
    """Fold the until and when monitor terms onto a motion.

    Per phase, the terms go onto the aggregate monitor, and onto any monitor whose one constraint
    is read directly rather than through a solver error -- an elapsed clock or an action goal's
    status.
    """
    for phase, evaluators, monitors in (
        ("until", motion.until_evaluators, motion.until_monitors),
        ("when", motion.when_evaluators, motion.when_monitors),
    ):
        terms = [
            evaluator_term(evaluator)
            for evaluator in evaluators
            if evaluator.error or evaluator.is_elapsed
        ]
        # Keyed by constraint, not by error: two goal-status items on one act share the status
        # slot, so the error alone cannot say which status a monitor is watching for.
        direct_by_constraint = {
            evaluator.constraint.id: evaluator_term(evaluator)
            for evaluator in evaluators
            if (evaluator.is_elapsed or evaluator.goal_status) and evaluator.error
        }
        for monitor in monitors:
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
                    f"the '{phase}' group in motion '{motion.id}'",
                )
                continue
            # A whole-section monitor over a flat constraint list, from a graph minted before
            # sections carried an expression node. Archived generations vendor those graphs and
            # telemetry replay rebuilds the IR from them, so the join is spelled out here: a
            # section only ever linked flat when it meant a conjunction.
            if monitor.is_until_aggregate if phase == "until" else monitor.is_when_aggregate:
                _stamp_terms(monitor, terms, False, f"the whole '{phase}' section of '{motion.id}'")
                continue
            direct = [
                direct_by_constraint[constraint_id]
                for constraint_id in monitor.constraint_ids
                if constraint_id in direct_by_constraint
            ]
            if direct:
                _stamp_terms(monitor, direct, False, f"the '{phase}' phase of '{motion.id}'")


def _add_motion_function_interfaces(motions: list, solvers_by_id: dict) -> None:
    """Fold the capability booleans each generated function's signature is built from.

    Also assigns each motion its telemetry index, so the frame-log schema and the generated
    sample switch read one field rather than agreeing with a second generator.
    """
    for index, motion in enumerate(motions):
        motion.index = index
        when_mons = motion.when_monitors
        until_mons = motion.until_monitors
        # Only a when-elapsed clock lives in state; an observation age is read off shared.
        has_when_elapsed = bool(motion.when_elapsed_ids)
        when_sched = bool(motion.when_schedule)
        until_sched = bool(motion.until_schedule)
        when_fsm = any(monitor.fsm_namespace for monitor in when_mons)
        until_fsm = any(monitor.fsm_namespace for monitor in until_mons)
        has_chain = bool(motion.serial_chain_solvers)

        motion.when_needs_state = has_when_elapsed or bool(when_mons)
        motion.when_needs_data = (
            has_when_elapsed
            or bool(motion.declared_pose_components)
            or when_sched
            or bool(when_mons)
        )
        motion.when_needs_robot = when_fsm

        motion.until_needs_state = bool(until_mons)
        motion.until_needs_data = until_sched or bool(until_mons)
        motion.until_needs_robot = until_fsm

        motion.apply_needs_state = has_chain
        # Gate on the torque limit, not on the joint-space samples: this runs before the
        # telemetry artifact exists, so the sample list does not yet.
        motion.apply_needs_data = bool(motion.forwarded_commands) or any(
            solvers_by_id[solver.solver_id].torque_saturation
            for solver in motion.serial_chain_solvers
        )
        motion.apply_needs_robot = has_chain or bool(motion.forwarded_commands)
        if motion.perturbations:
            motion.apply_needs_state = True
            motion.apply_needs_data = True
            motion.apply_needs_robot = True

        # An edge is the occurrence: a flag monitor holds a level and never reaches the buffer,
        # and an FSM-namespaced edge is produced on the sequencer, not recorded in the buffer.
        when_events = any(
            monitor.is_edge_triggered and not monitor.fsm_namespace for monitor in when_mons
        )
        until_events = any(
            monitor.is_edge_triggered and not monitor.fsm_namespace for monitor in until_mons
        )
        control_events = any(
            monitor.is_edge_triggered and not monitor.fsm_namespace
            for monitor in motion.while_monitors
        )
        motion.when_needs_events = when_events
        motion.until_needs_events = until_events
        motion.control_needs_events = control_events
        motion.step_needs_events = until_events or control_events
