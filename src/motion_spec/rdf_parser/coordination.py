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
from motion_spec.rdf_parser.agents import scene_kinematics
from motion_spec.rdf_parser.constraint_handler import (
    SolverIdFactory,
    alignment_chain_ops,
    alignment_op,
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


def constraint_transition(model, node) -> ConstraintTransition:
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
        [constraint_transition(model, item) for item in graph[node : MOT["when"]]],
        [quantities.constraint(model, item) for item in graph[node : MOT["while"]]],
        [constraint_transition(model, item) for item in graph[node : MOT["until"]]],
        name=str(name),
        description=str(description) if description is not None else None,
    )


# no solver error: codegen compares the world clock to the threshold, in seconds
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

    types = get_node_types(graph, constraint_node) if constraint_node is not None else set()
    is_elapsed = CSTR_EXT["TimeConstraint"] in types
    operator, threshold, elapsed_tolerance, observed_at = None, None, None, None
    if is_elapsed:
        # from sosa:phenomenonTime when the interval starts at one, else from entry
        interval = graph.value(constraint_node, TIME.hasTime)
        begin = graph.value(interval, TIME.hasBeginning) if interval is not None else None
        observed = (
            graph.value(predicate=SOSA.phenomenonTime, object=begin) if begin is not None else None
        )
        if observed is not None:
            perceived = {
                row["pose_id"]
                for rows in quantities.perceived_written_poses(model).values()
                for row in rows
            }
            if model.id(observed) not in perceived:
                raise ConstraintViolation(
                    "coordination",
                    f"'{model.id(constraint_node)}' measures the time since "
                    f"'{model.id(observed)}' was observed, but nothing observes it: no subscriber "
                    "names it and no detect act writes it, so its observation instant can never "
                    "be filled.",
                )
            observed_at = model.id(begin)
        operator, predicate = next(
            ((op, pred) for type_, op, pred in _ELAPSED_RELATIONS if type_ in types),
            ("<", CSTR["threshold"]),
        )
        threshold = quantities.duration_seconds(model, graph.value(constraint_node, predicate))
        if operator == "==":
            band_node = graph.value(constraint_node, CSTR_EXT["tolerance"])
            elapsed_tolerance = quantities.duration_seconds(model, band_node)
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


@reader
def monitor_entry(model, node):
    """A monitor: a level flag its constraint sets continuously, or an edge event it fires once."""
    graph = model.graph
    handler = ensure_one_typed_subject_uri(
        graph, node, CSTR_HDL.monitors, CSTR_HDL.ConstraintHandler
    )
    motion = graph.value(handler, CSTR_HDL.motion) if handler is not None else None
    monitored = set(graph.objects(node, CSTR_HDL.constraint))
    expressions = [item for item in monitored if quantities.is_constraint_aggregate(model, item)]
    in_section = (len(monitored) > 1 or bool(expressions)) and motion is not None
    is_until_aggregate = in_section and monitored == set(graph[motion : MOT["until"]])
    is_when_aggregate = in_section and monitored == set(graph[motion : MOT["when"]])
    if len(expressions) > 1:
        raise ConstraintViolation(
            "coordination",
            f"a monitor watches {len(expressions)} constraint groups -- it watches one condition",
        )
    group_ids, group_uris, group_bands, group_any = [], [], [], False
    if expressions:
        (expression,) = expressions
        members = list(graph[expression : CSTR_EXT["has-constraint"]])
        bands = [graph.value(member, CSTR_EXT["tolerance"]) for member in members]
        group_ids = [model.id(member) for member in members]
        group_uris = [str(member) for member in members]
        group_bands = [model.id(band) if band is not None else "" for band in bands]
        group_any = CSTR_EXT.ConstraintDisjunction in get_node_types(graph, expression)

    error_node = graph.value(node, CSTR_HDL["error"])
    error = (
        None
        if is_until_aggregate or is_when_aggregate or group_ids or error_node is None
        else quantities.quantity(model, error_node)
    )
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
    publication = ros_publication(model, node)
    if CSTR_HDL["EdgeTriggeredMonitor"] not in get_node_types(graph, node):
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
    fallback = graph.value(node, CSTR_HDL_EXT["fallback-motion"])
    debounce = graph.value(node, CSTR_HDL_EXT["debounce-duration"])

    return EdgeMonitor(
        id=model.id(node),
        monitor_type="EdgeTriggeredMonitor",
        error=error,
        event=model.id(event_node),
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
    order_by_node = {
        node: int(graph.value(node, APP.order, default=Literal(0)).value)
        for node in graph.subjects(RDF.type, CSTR_HDL["ConstraintHandler"])
    }
    for node in sorted(order_by_node, key=order_by_node.get):
        handler = ConstraintHandler(
            model.id(node),
            guarded_motion(model, graph.value(node, CSTR_HDL["motion"])),
            [constraint_evaluator(model, item) for item in graph[node : CSTR_HDL["evaluators"]]],
            [],
            [monitor_entry(model, item) for item in graph[node : CSTR_HDL["monitors"]]],
            order_by_node[node],
        )
        plans = derivation.controllers_by_handler.get(node, ())
        handler.controllers = [
            controller for plan in plans for controller in derivation.controllers_for(plan)
        ]
        handlers.append(handler)
        steps.extend(
            schedule.of(list(graph.objects(node, CSTR_HDL.evaluators)), OPS_GENERIC + OPS_HANDLER)
        )
        for plan in plans:
            if len(plan.axes) > 1 or CSTR_HDL_EXT.FeedForwardController in get_node_types(
                graph, plan.controller
            ):
                continue
            error = graph.value(plan.controller, CSTR_HDL["error-signal"])
            evaluator = next(graph.subjects(CSTR_HDL.error, error), None)
            if evaluator is not None and model.id(evaluator) not in steps:
                steps.append(model.id(evaluator))
        # reversed, so the emitted order is the authored one
        steps.extend(
            controller.id
            for plan in reversed(plans)
            for controller in reversed(derivation.controllers_for(plan))
        )
        steps.extend(
            SolverIdFactory(model, plan.controller).pose_evaluator()
            for plan in reversed(plans)
            if len(plan.axes) > 1
            and alignment_op(model, GEOM_OP_EXT.RotationVectorFromDirections, plan.quantity) is None
        )

    return handlers, steps


def assign_event_indexes(handlers) -> None:
    """Give every edge monitor a stable index into the run's event buffer."""
    index = 0
    for handler in handlers:
        for monitor in handler.monitors:
            if monitor.is_edge_triggered:
                monitor.event_idx = index
                index += 1


class PhaseNodes:
    """Which constraints, evaluators and monitors of one handler belong to which phase.

    A monitor names the constraints it watches; one that names none is bucketed by the error
    signal it reads instead, which is the only other thing tying it to a phase.
    """

    def __init__(self, model, handler_node, motion_node):
        graph = model.graph
        self.raw = {phase: set(graph[motion_node : MOT[phase]]) for phase in _PHASES}
        self.constraints = {
            phase: expanded_constraints(model, nodes) if phase != "while" else nodes
            for phase, nodes in self.raw.items()
        }
        self.evaluators = {phase: [] for phase in _PHASES}
        for node in graph[handler_node : CSTR_HDL["evaluators"]]:
            constraint = graph.value(node, CSTR_HDL["constraint"])
            phase = next((p for p in _PHASES if constraint in self.constraints[p]), None)
            if phase is not None:
                self.evaluators[phase].append(node)

        self.watched = {
            phase: {graph.value(node, CSTR_HDL["constraint"]) for node in self.evaluators[phase]}
            for phase in _PHASES
        }
        self.errors = {
            phase: {graph.value(node, CSTR_HDL["error"]) for node in self.evaluators[phase]}
            - {None}
            for phase in _PHASES
        }
        self.monitors = {phase: [] for phase in _PHASES}
        for node in graph[handler_node : CSTR_HDL["monitors"]]:
            phase = self.monitor_phase(graph, node)
            if phase is not None:
                self.monitors[phase].append(node)

    def monitor_phase(self, graph, node) -> str | None:
        """The phase a monitor belongs to, by what it watches or, failing that, what it reads."""
        monitored = set(graph.objects(node, CSTR_HDL["constraint"]))
        if monitored:
            for phase in ("when", "until"):
                if monitored <= self.raw[phase]:
                    return phase

            return next((p for p in _PHASES if monitored & self.watched[p]), None)
        error = graph.value(node, CSTR_HDL["error"])

        return next((p for p in _PHASES if error in self.errors[p]), None)


def append_new(steps: list, candidates) -> None:
    """Append the calls this list does not already carry, keeping their order."""
    for step in candidates:
        if step not in steps:
            steps.append(step)


def build_motions(model, handlers, robots, computation, derivation, fsm):
    """Build one motion unit per handler, then fold on what only the whole set decides.

    Returns:
        `(motions, fsm_meta)`: the motions in the order the handlers declare them, and the FSM
        wiring codegen needs alongside the framed FSM
    """
    graph = model.graph
    motions = []
    tokens = {handler.motion.id for handler in handlers}
    solvers_by_id = robots.by_id
    owner = computation.indexes.function_owner
    function_input = computation.indexes.function_input
    _trees, holders = scene_kinematics(model)
    for handler in handlers:
        handler_node = model.node_by_id[handler.id]
        token = handler.motion.id
        phase = PhaseNodes(model, handler_node, graph.value(handler_node, CSTR_HDL["motion"]))
        plans = derivation.controllers_by_handler.get(handler_node, ())
        active_plans = [plan for plan in plans if plan.constraint in phase.constraints["while"]]
        active_controllers = [
            controller for plan in active_plans for controller in derivation.controllers_for(plan)
        ]
        solver_ids = {model.id(plan.solver) for plan in plans} | {
            model.id(solver) for solver in graph.objects(handler_node, CSTR_HDL_EXT["runs-solver"])
        }
        chain_solvers = []
        for solver in robots.serial_chains:
            if solver.id not in solver_ids or not solver.motion_drivers:
                continue
            owned = [driver for driver in solver.motion_drivers if driver.handler == handler.id]
            if len(owned) != 1:
                raise ConstraintViolation(
                    "coordination",
                    f"solver '{solver.id}' carries {len(owned)} motion drivers of handler "
                    f"'{handler.id}' -- a handler drives a solver through one",
                )
            chain_solvers.append(
                MotionSolverSlice(
                    id=solver.id,
                    solver_id=solver.id,
                    output=solver.output,
                    motion_driver=owned[0],
                    read_only=solver.algorithm.read_only,
                )
            )
        evaluators = {
            name: [constraint_evaluator(model, node) for node in phase.evaluators[name]]
            for name in _PHASES
        }
        groups = pose_axis_error_groups_for_motion(
            model, dict(zip(phase.evaluators["while"], evaluators["while"])), computation.views
        )

        scope = Schedule(model)
        live = {
            kind: [
                node
                for node, evaluator in zip(phase.evaluators[kind], evaluators[kind], strict=True)
                if not evaluator.is_elapsed and evaluator.goal_status is None
            ]
            for kind in ("until", "while")
        }

        # until first, or its monitor reads the previous tick's value
        until = scope.of(live["until"], OPS_GENERIC + OPS_HANDLER)
        until.extend(model.id(node) for node in live["until"] if scope.claim(model.id(node)))

        grouped_ids = {component.eval_id for group in groups for component in group.components}
        grouped_nodes = [
            node for node in phase.evaluators["while"] if model.id(node) in grouped_ids
        ]
        # producers of a grouped reference run before its inline error
        while_pre = [
            step
            for step in scope.of(grouped_nodes, OPS_GENERIC + OPS_HANDLER)
            if step not in grouped_ids
        ]

        by_constraint = {plan.constraint: plan for plan in active_plans}
        leading, trailing = [], []
        for node in live["while"]:
            if node in grouped_nodes:
                continue
            plan = by_constraint.get(graph.value(node, CSTR_HDL.constraint))
            if plan is None or CSTR_HDL_EXT.FeedForwardController in get_node_types(
                graph, plan.controller
            ):
                trailing.append(node)
            else:
                leading.append(node)

        controller_ids = {controller.id for controller in active_controllers} | {
            model.id(plan.controller) for plan in active_plans
        }
        active = [
            step
            for step in scope.of(leading, OPS_GENERIC + OPS_HANDLER)
            if step not in controller_ids
        ]
        # the backward walk never reaches alignment ops: each motion names them itself
        for constraint in phase.constraints["while"]:
            quantity = graph.value(constraint, CSTR.quantity)
            if quantity is None:
                continue
            drive_op = alignment_op(
                model, GEOM_OP_EXT.RotationVectorFromDirections, quantity
            ) or alignment_op(model, GEOM_OP_EXT.AngleGradientFromDirections, quantity)
            if drive_op is not None:
                append_new(
                    active,
                    [model.id(op) for op in alignment_chain_ops(model, quantity)]
                    + [model.id(drive_op)],
                )
        for plan in active_plans:
            if (
                len(plan.axes) <= 1
                or alignment_op(model, GEOM_OP_EXT.RotationVectorFromDirections, plan.quantity)
                is not None
            ):
                continue
            reference_view = quantities.view_of(
                graph, graph.value(plan.constraint, CSTR["reference-value"])
            )
            interpolation = next(
                graph.subjects(GEOM_OP.out, graph.value(reference_view, MAP.superobject)), None
            )
            if interpolation is not None:
                append_new(active, scope.of([interpolation], OPS_GENERIC + OPS_HANDLER))
            append_new(active, [SolverIdFactory(model, plan.controller).pose_evaluator()])
        gradient_units = [
            unit
            for plan in active_plans
            for op in graph.subjects(ALGO_EXT.out, plan.quantity)
            for predicate in (GEOM_OP_EXT.gradient, GEOM_OP_EXT["gradient-moment"])
            if (unit := graph.value(op, predicate)) is not None
        ]
        append_new(
            active,
            scope.of(
                [op for unit in gradient_units for op in graph.subjects(ALGO_EXT.out, unit)],
                OPS_GENERIC,
            ),
        )
        append_new(active, [model.id(node) for node in leading])
        control_outputs = {controller.control_signal.id for controller in handler.controllers}
        force_nodes = []
        driver_nodes = [
            model.node_by_id[solver.motion_driver.id]
            for solver in chain_solvers
            if solver.motion_driver.id in model.node_by_id
        ]
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
                feeding: set[str] = set()
                pending = list(function_input.get(force_id, ()))
                while pending:
                    item = pending.pop()
                    if item in feeding:
                        continue
                    feeding.add(item)
                    pending.extend(function_input.get(item, ()))
                if ({force_id} | feeding) & control_outputs:
                    force_nodes.append(node)
        # claimed here, emitted after the control laws it reads
        commanded_force = scope.of(force_nodes, OPS_GENERIC + OPS_SOLVER + OPS_HANDLER)
        active.extend(scope.of(trailing, OPS_GENERIC + OPS_HANDLER))
        append_new(active, [model.id(node) for node in trailing])
        perturbation_nodes = perturbations.nodes(model, handler_node)
        for node in perturbation_nodes:
            gate = perturbations.evaluator_nodes(model, node)
            append_new(active, scope.of(gate, OPS_GENERIC + OPS_HANDLER))
            append_new(
                active,
                [model.id(evaluator) for evaluator in gate if scope.claim(model.id(evaluator))],
            )
            append_new(active, scope.of(perturbations.compose_ops(model, node), OPS_GENERIC))
        when_scope = Schedule(model)
        when_live = [
            node
            for node, evaluator in zip(phase.evaluators["when"], evaluators["when"], strict=True)
            if not evaluator.is_elapsed
        ]
        when = when_scope.of(when_live, OPS_GENERIC + OPS_HANDLER)
        when.extend(model.id(node) for node in when_live if when_scope.claim(model.id(node)))

        # another motion's calls would recompute its outputs while it is inactive
        active = [step for step in active if owner.get(step, token) == token]
        while_pre = [step for step in while_pre if owner.get(step, token) == token]
        append_new(active, [controller.id for controller in reversed(active_controllers)])
        append_new(active, [step for step in commanded_force if owner.get(step, token) == token])
        when_elapsed = elapsed_coordinate_ids(evaluators["when"])
        active_elapsed = elapsed_coordinate_ids(evaluators["while"] + evaluators["until"])
        when_ages = observation_ages(evaluators["when"])
        active_ages = observation_ages(evaluators["while"] + evaluators["until"])
        monitors = {
            name: [monitor_entry(model, node) for node in phase.monitors[name]] for name in _PHASES
        }
        forwarded_commands = []
        for plan in active_plans:
            if not derivation.algorithm_by_solver[plan.solver].forwards_commands:
                continue
            quantity = graph.value(plan.constraint, CSTR.quantity)
            view = quantities.view_of(graph, quantity)
            target = graph.value(
                graph.value(view, MAP.superobject) if view is not None else quantity,
                KC_STAT["of-joint"],
            )
            runtime = next(
                (
                    solvers_by_id[slice_.solver_id]
                    for slice_ in chain_solvers
                    if any(
                        tree in holders.get(target, ())
                        for tree in solvers_by_id[slice_.solver_id].runtime.owned_trees or ()
                    )
                ),
                None,
            )
            if runtime is None:
                raise ConstraintViolation(
                    "coordination",
                    "command forwarding: joint "
                    f"'{model.label(target) if target is not None else target}' belongs to no "
                    "kinematic tree this handler's runtimes own",
                )
            forwarded_commands.append(
                ForwardedCommandStep(
                    f"cmd-fwd-{model.id(plan.controller)}",
                    derivation.controllers_for(plan)[0].control_signal,
                    f"{runtime.runtime.prefix}{model.label(target)}",
                    runtime.id,
                )
            )
        runs_in = graph.value(handler_node, CSTR_HDL_EXT["runs-in-state"])
        unit = MotionUnit(
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
            when_monitors=monitors["when"],
            while_monitors=monitors["while"],
            until_monitors=monitors["until"],
            when_schedule=when,
            while_schedule=active,
            until_schedule=until,
            has_elapsed=bool(when_elapsed or active_elapsed or when_ages or active_ages),
            has_until_condition=bool(evaluators["until"]),
            serial_chain_solvers=chain_solvers,
            pose_axis_error_groups=groups,
            while_pre_schedule=while_pre,
            forwarded_commands=forwarded_commands,
            snapshots=snapshots_for_motion(
                evaluators["while"] + evaluators["when"] + evaluators["until"],
                handler.motion.while_ + handler.motion.when + handler.motion.until,
                computation.indexes,
                computation.views,
                # while_pre too: its setpoint generators may read a snapshot
                while_pre + active + when + until,
                computation.functions,
                token,
                tokens,
            ),
            path_projections=path_projections_for_motion(
                while_pre + active, computation.functions
            ),
            runs_in_state=str(runs_in) if runs_in is not None else "",
        )
        motions.append(unit)
        chains = [(slice_.id, solvers_by_id[slice_.solver_id]) for slice_ in chain_solvers]
        records = []
        for node in perturbation_nodes:
            record = perturbations.read(model, node, chains)
            record.terms = [
                evaluator_term(constraint_evaluator(model, evaluator))
                for evaluator in perturbations.evaluator_nodes(model, node)
            ]
            if record.has_gate and not record.terms:
                raise ConstraintViolation(
                    "perturbation",
                    f"perturbation '{record.id}' states a when-gate that lowered to no terms, so "
                    "it renders as a constant false and the disturbance can never be applied",
                )
            records.append(record)
        unit.perturbations = records
        unit.entry_snapshots = [s for s in unit.snapshots if s.scope == "entry"]
        unit.task_snapshots = [s for s in unit.snapshots if s.scope == "task"]

    for motion in motions:
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
    meta = apply_fsm_wiring(motions, fsm, solvers_by_id.values())
    for index, motion in enumerate(motions):
        motion.index = index
        motion.when_needs_state = bool(motion.when_elapsed_ids or motion.when_monitors)
        motion.when_needs_data = bool(
            motion.when_elapsed_ids
            or motion.declared_pose_components
            or motion.when_schedule
            or motion.when_monitors
        )
        motion.when_needs_robot = any(monitor.fsm_namespace for monitor in motion.when_monitors)
        motion.until_needs_state = bool(motion.until_monitors)
        motion.until_needs_data = bool(motion.until_schedule or motion.until_monitors)
        motion.until_needs_robot = any(monitor.fsm_namespace for monitor in motion.until_monitors)
        motion.apply_needs_state = bool(motion.serial_chain_solvers or motion.perturbations)
        # not on the joint-space samples: they do not exist yet
        motion.apply_needs_data = (
            bool(motion.forwarded_commands)
            or any(
                solvers_by_id[solver.solver_id].torque_saturation
                for solver in motion.serial_chain_solvers
            )
            or bool(motion.perturbations)
        )
        motion.apply_needs_robot = bool(
            motion.serial_chain_solvers or motion.forwarded_commands or motion.perturbations
        )
        motion.when_needs_events, motion.until_needs_events, motion.control_needs_events = (
            any(monitor.is_edge_triggered and not monitor.fsm_namespace for monitor in monitors)
            for monitors in (motion.when_monitors, motion.until_monitors, motion.while_monitors)
        )
        motion.step_needs_events = motion.until_needs_events or motion.control_needs_events
    apply_fsm_gate_calls(motions, meta["cpp_namespace"])

    return motions, meta


_GROUP_TYPE_FLAGS = {
    "is_pose": ("Pose",),
    "is_twist": ("VelocityTwist", "AccelerationTwist"),
    "is_wrench": ("Wrench",),
}


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
        elapsed_id = elapsed_coordinate_id(evaluator)
        threshold = f"{evaluator.elapsed_threshold_s:.6f}"
        if evaluator.elapsed_op == "==":
            return {
                "kind": "elapsed-eq",
                "elapsed_id": elapsed_id,
                "threshold": threshold,
                "tolerance": f"{evaluator.elapsed_tolerance_s:.6f}",
            }

        return {
            "kind": "elapsed",
            "elapsed_id": elapsed_id,
            "op": evaluator.elapsed_op,
            "threshold": threshold,
        }

    term = {
        "kind": "constraint",
        "error_id": evaluator.error.id if evaluator.error is not None else None,
    }
    # omitted, not empty: ST4 reads "" as present
    if evaluator.tolerance is not None and evaluator.tolerance.id:
        term["tolerance_id"] = evaluator.tolerance.id

    return term


def stamp_terms(monitor, terms, any_flag, where: str) -> None:
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
        evaluable = [
            evaluator for evaluator in evaluators if evaluator.error or evaluator.is_elapsed
        ]
        terms = [evaluator_term(evaluator) for evaluator in evaluable]
        # by constraint: goal-status items on one act share a status slot
        direct_by_constraint = {
            evaluator.constraint.id: evaluator_term(evaluator)
            for evaluator in evaluators
            if (evaluator.is_elapsed or evaluator.goal_status) and evaluator.error
        }
        for monitor in monitors:
            group_ids = set(monitor.group_constraint_ids)
            if group_ids:
                stamp_terms(
                    monitor,
                    [
                        evaluator_term(evaluator)
                        for evaluator in evaluable
                        if evaluator.constraint.id in group_ids
                    ],
                    monitor.group_any,
                    f"the '{phase}' group in motion '{motion.id}'",
                )
                continue
            # a flat section in an archived graph means a conjunction
            is_aggregate = (
                monitor.is_until_aggregate if phase == "until" else monitor.is_when_aggregate
            )
            if is_aggregate:
                stamp_terms(monitor, terms, False, f"the whole '{phase}' section of '{motion.id}'")
                continue
            direct = [
                direct_by_constraint[constraint_id]
                for constraint_id in monitor.constraint_ids
                if constraint_id in direct_by_constraint
            ]
            if direct:
                stamp_terms(monitor, direct, False, f"the '{phase}' phase of '{motion.id}'")
