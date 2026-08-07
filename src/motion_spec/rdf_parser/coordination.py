# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Coordination: per-motion units, their monitors and conditions, the function-interface
capability flags, and the FSM framing and wiring."""

from __future__ import annotations

import rdflib
from rdf_utils.naming import get_valid_var_name
from rdf_utils.models.common import get_node_types
from rdf_utils.uri import iri_is_descendant, iri_parent
from rdflib.namespace import RDF
from motion_spec.classes.entities import ForwardedCommand, GuardedMotionBlock

# fmt: off
from motion_spec_dsl.rdf_parser.vocab import (
    CSTR, CSTR_HDL, CSTR_HDL_EXT, GEOM_OP, KC_STAT, MAP, MOT, SLV, SLV_EXT,
)
# fmt: on

from motion_spec.rdf_parser.records import _add_group_type_flags, _field, _set_field
from motion_spec.rdf_parser.graph import (
    Parser,
    _is_elapsed_constraint,
    ops_cstr_hdl,
    ops_generic,
    ops_slv,
)
from motion_spec.rdf_parser.controllers import (
    SolverIdFactory,
    _annotate_controller_signals,
    _derived_controllers,
    _motion_suffix,
)
from motion_spec.rdf_parser.solvers import (
    _serial_chain_solvers_for_handler,
    _tree_owns,
    _upstream_dependencies,
)

# fmt: off
from motion_spec.rdf_parser.computation import (
    _elapsed_coordinate_id, _elapsed_coordinate_ids, _expanded_constraints,
    _path_projections_for_motion, _pose_axis_error_groups_for_motion, _relative_poses_for_motion,
    _scene_relative_poses_for_motion, _snapshots_for_motion, collect_motion_references,
    declared_pose_component_entries,
)
# fmt: on


def build_motion_units(
    g,
    p,
    handlers,
    node_by_id,
    slv_chain,
    snapshot_source_map=None,
    view_map=None,
    closure_output_map=None,
    data_reference_map=None,
    closure_input_map=None,
    closures=None,
    data_structures=None,
    pose_components=None,
    fsm=None,
    derivation=None,
    snapshot_trigger_map=None,
    snapshot_owner_map=None,
    closure_owner_map=None,
):
    """Build the per-motion IR units (one per handler) with schedules, monitors, controllers,
    conditions, declared poses, FSM wiring and function-interface flags; returns (motions, fsm_meta).
    """
    snapshot_source_map = snapshot_source_map or {}
    view_map = view_map or {}
    closure_output_map = closure_output_map or {}
    data_reference_map = data_reference_map or {}
    closure_input_map = closure_input_map or {}
    motions = []
    snapshot_owner_map = snapshot_owner_map or {}
    closure_owner_map = closure_owner_map or {}
    motion_tokens = {
        _motion_suffix(p, g.value(node_by_id[h.id], CSTR_HDL["motion"])) for h in handlers
    }
    for handler in handlers:
        handler_node = node_by_id[handler.id]
        motion_node = g.value(handler_node, CSTR_HDL["motion"])
        motion = handler.motion
        handler_plans = derivation.controllers_by_handler.get(handler_node, ())
        # Declared solvers count even when no controller routes to them: a monitor-only
        # handler still owns its arm runtime for state reading, FK and command forwarding.
        handler_solver_ids = {p.id(plan.solver) for plan in handler_plans} | {
            p.id(solver) for solver in g.objects(handler_node, CSTR_HDL_EXT["runs-solver"])
        }

        _raw_when = set(g[motion_node : MOT["when"]])
        _raw_until = set(g[motion_node : MOT["until"]])
        when_constraint_nodes = _expanded_constraints(g, _raw_when)
        while_constraint_nodes = set(g[motion_node : MOT["while"]])
        until_constraint_nodes = _expanded_constraints(g, _raw_until)

        when_eval_nodes, while_eval_nodes, until_eval_nodes = [], [], []
        for eval_node in g[handler_node : CSTR_HDL["evaluators"]]:
            cstr_node = g.value(eval_node, CSTR_HDL["constraint"])
            if cstr_node in when_constraint_nodes:
                when_eval_nodes.append(eval_node)
            elif cstr_node in while_constraint_nodes:
                while_eval_nodes.append(eval_node)
            elif cstr_node in until_constraint_nodes:
                until_eval_nodes.append(eval_node)

        def _is_elapsed_eval(eval_node):
            return _is_elapsed_constraint(g, g.value(eval_node, CSTR_HDL["constraint"]))

        while_error_nodes = {g.value(n, CSTR_HDL["error"]) for n in while_eval_nodes}
        while_error_nodes.discard(None)
        active_plans = [plan for plan in handler_plans if plan.constraint in while_constraint_nodes]
        active_controllers = [
            controller
            for plan in active_plans
            for controller in _derived_controllers(g, p, derivation, plan)
        ]

        # Monitors without a cstr-hdl:constraint link fall back to error-signal bucketing.
        when_cstr_nodes = {g.value(n, CSTR_HDL["constraint"]) for n in when_eval_nodes}
        while_cstr_nodes = {g.value(n, CSTR_HDL["constraint"]) for n in while_eval_nodes}
        until_cstr_nodes = {g.value(n, CSTR_HDL["constraint"]) for n in until_eval_nodes}
        when_cstr_nodes.discard(None)
        while_cstr_nodes.discard(None)
        until_cstr_nodes.discard(None)

        when_error_nodes = {g.value(n, CSTR_HDL["error"]) for n in when_eval_nodes}
        until_error_nodes = {g.value(n, CSTR_HDL["error"]) for n in until_eval_nodes}
        when_error_nodes.discard(None)
        until_error_nodes.discard(None)

        when_mon_nodes, while_mon_nodes, until_mon_nodes = [], [], []
        for mon_node in g[handler_node : CSTR_HDL["monitors"]]:
            monitored = set(g.objects(mon_node, CSTR_HDL["constraint"]))
            if monitored and monitored == _raw_when:
                when_mon_nodes.append(mon_node)
                continue
            if monitored and monitored == _raw_until:
                until_mon_nodes.append(mon_node)
                continue
            # A group monitor names one of the section's nodes, not the whole section, and the
            # group node itself never appears in the expanded member sets below.
            if monitored and monitored <= _raw_when:
                when_mon_nodes.append(mon_node)
                continue
            if monitored and monitored <= _raw_until:
                until_mon_nodes.append(mon_node)
                continue
            if monitored:
                if monitored & when_cstr_nodes:
                    when_mon_nodes.append(mon_node)
                elif monitored & while_cstr_nodes:
                    while_mon_nodes.append(mon_node)
                elif monitored & until_cstr_nodes:
                    until_mon_nodes.append(mon_node)
            else:
                mon_error = g.value(mon_node, CSTR_HDL["error"])
                if mon_error in when_error_nodes:
                    when_mon_nodes.append(mon_node)
                elif mon_error in while_error_nodes:
                    while_mon_nodes.append(mon_node)
                elif mon_error in until_error_nodes:
                    until_mon_nodes.append(mon_node)

        # Validate classified sets are subsets of what the handler declares. Evaluators/controllers
        # not linked to any motion phase are silently excluded from all schedules (e.g. sc1's extras).
        all_eval_nodes = set(g[handler_node : CSTR_HDL["evaluators"]])
        classified_eval_nodes = set(when_eval_nodes) | set(while_eval_nodes) | set(until_eval_nodes)
        if not classified_eval_nodes <= all_eval_nodes:
            raise ValueError(
                f"Handler {handler.id}: classified evaluators not a subset of handler evaluators"
            )

        all_mon_nodes = set(g[handler_node : CSTR_HDL["monitors"]])
        classified_mon_nodes = set(when_mon_nodes) | set(while_mon_nodes) | set(until_mon_nodes)
        if not classified_mon_nodes <= all_mon_nodes:
            raise ValueError(
                f"Handler {handler.id}: classified monitors not a subset of handler monitors"
            )

        # when_schedule runs in can_start (own Parser + dedup set); while_/until share p_active
        # so steps evaluated in both phases emit once (see the _build_ir schedules note).
        p_when = Parser(g)
        when_schedule = p_when.schedule(
            [n for n in when_eval_nodes if not _is_elapsed_eval(n)], ops_generic + ops_cstr_hdl
        )

        handler_chain_solvers = _serial_chain_solvers_for_handler(
            handler, slv_chain, handler_solver_ids
        )
        handler_output_ids = {c.control_signal.id for c in handler.controllers}
        cartesian_force_nodes = []
        for solver in handler_chain_solvers:
            driver_node = node_by_id.get(solver.motion_driver.id)
            if driver_node is None:
                continue
            for cf_node in g[driver_node : SLV["cartesian-force"]]:
                cf_force = g.value(cf_node, SLV["force"])
                if cf_force is None:
                    continue
                cf_force_id = p.id(cf_force)
                upstream = {cf_force_id} | _upstream_dependencies(cf_force_id, closure_input_map)
                if upstream & handler_output_ids:
                    cartesian_force_nodes.append(cf_node)

        p_active = Parser(g)
        # Until monitors run before control each tick, so build until_schedule first: a
        # quantity an until monitor consumes must be scheduled in the earlier phase, or the
        # monitor reads the previous tick's value on the first tick.
        until_schedule = p_active.schedule(
            [n for n in until_eval_nodes if not _is_elapsed_eval(n)], ops_generic + ops_cstr_hdl
        )

        # Until evaluators have no controller error-signal to drive backward discovery, so
        # append them after their dependencies to keep the emitted call order right.
        for n in until_eval_nodes:
            if _is_elapsed_eval(n):
                continue
            eval_id = p.id(n)
            if eval_id not in p_active.sched:
                until_schedule.append(eval_id)
                p_active.sched.add(eval_id)

        pose_axis_error_groups = _pose_axis_error_groups_for_motion(
            while_eval_nodes, p_active, view_map
        )
        pose_axis_error_eval_ids = {
            component.eval_id for group in pose_axis_error_groups for component in group.components
        }
        grouped_while_eval_nodes = {
            node for node in while_eval_nodes if p_active.id(node) in pose_axis_error_eval_ids
        }
        # A grouped evaluator's error is emitted inline ahead of the schedule block, but
        # whatever produces its reference still has to run first -- so walk the grouped nodes
        # before the main pass rather than skipping those producers entirely.
        pre_group_schedule = [
            step
            for step in p_active.schedule(
                sorted(grouped_while_eval_nodes, key=str), ops_generic + ops_cstr_hdl
            )
            if step not in pose_axis_error_eval_ids
        ]
        plan_by_constraint = {plan.constraint: plan for plan in active_plans}
        pre_controller_evaluators = []
        trailing_evaluators = []
        for node in while_eval_nodes:
            if node in grouped_while_eval_nodes or _is_elapsed_eval(node):
                continue
            plan = plan_by_constraint.get(g.value(node, CSTR_HDL.constraint))
            if plan is not None and CSTR_HDL_EXT.FeedForwardController not in get_node_types(
                g, plan.controller
            ):
                pre_controller_evaluators.append(node)
            else:
                trailing_evaluators.append(node)

        while_schedule = p_active.schedule(pre_controller_evaluators, ops_generic + ops_cstr_hdl)
        derived_controller_ids = {controller.id for controller in active_controllers}
        authored_controller_ids = {p.id(plan.controller) for plan in active_plans}
        while_schedule = [
            step
            for step in while_schedule
            if step not in derived_controller_ids and step not in authored_controller_ids
        ]
        for plan in active_plans:
            if len(plan.axes) <= 1:
                continue
            reference = g.value(plan.constraint, CSTR["reference-value"])
            reference_view = next(g.subjects(MAP.subobject, reference), None)
            reference_pose = g.value(reference_view, MAP.superobject)
            interpolation = next(g.subjects(GEOM_OP.out, reference_pose), None)
            if interpolation is not None:
                for step in p_active.schedule([interpolation], ops_generic + ops_cstr_hdl):
                    if step not in while_schedule:
                        while_schedule.append(step)
            evaluator_id = SolverIdFactory(
                p.id(plan.controller), _motion_suffix(p, plan.motion)
            ).pose_evaluator()
            if evaluator_id not in while_schedule:
                while_schedule.append(evaluator_id)
        for node in pre_controller_evaluators:
            eval_id = p.id(node)
            if eval_id not in while_schedule:
                while_schedule.append(eval_id)
        force_schedule = p_active.schedule(
            cartesian_force_nodes, ops_generic + ops_slv + ops_cstr_hdl
        )
        while_schedule.extend(step for step in force_schedule if step not in while_schedule)
        while_schedule.extend(p_active.schedule(trailing_evaluators, ops_generic + ops_cstr_hdl))
        for node in trailing_evaluators:
            eval_id = p.id(node)
            if eval_id not in while_schedule:
                while_schedule.append(eval_id)

        # The backward walk can reach another motion's closures; running them here would
        # recompute its outputs while it is inactive.
        motion_token_for_closures = _motion_suffix(p, motion_node)

        def _owned_steps(steps):
            return [
                step
                for step in steps
                if closure_owner_map.get(step, motion_token_for_closures)
                == motion_token_for_closures
            ]

        while_schedule = _owned_steps(while_schedule)
        pre_group_schedule = _owned_steps(pre_group_schedule)
        while_schedule.extend(
            controller.id
            for controller in reversed(active_controllers)
            if controller.id not in while_schedule
        )
        # can_start inlines when evaluators via when_evaluators, but their prerequisite
        # generic ops still need scheduling.
        for n in when_eval_nodes:
            if _is_elapsed_eval(n):
                continue
            eval_id = p.id(n)
            if eval_id not in p_when.sched:
                when_schedule.append(eval_id)
                p_when.sched.add(eval_id)

        when_evaluators = [p.constraint_evaluator(n) for n in when_eval_nodes]
        while_evaluators = [p.constraint_evaluator(n) for n in while_eval_nodes]
        until_evaluators = [p.constraint_evaluator(n) for n in until_eval_nodes]
        controllers = active_controllers
        forwarding_solvers = set(g.subjects(RDF.type, SLV_EXT.CommandForwardingSolver))
        forwarded_commands = []
        for plan in active_plans:
            if plan.solver not in forwarding_solvers:
                continue
            controller = _derived_controllers(g, p, derivation, plan)[0]
            constraint = plan.constraint
            quantity = g.value(constraint, CSTR.quantity)
            view = next(g.subjects(MAP.subobject, quantity), None)
            target_quantity = g.value(view, MAP.superobject) if view is not None else quantity
            target = g.value(target_quantity, KC_STAT["of-joint"])
            # Resolved from the joint, not the agent: a gripper's joint rides the arm's runtime.
            trees_by_solver_id = {solver.id: (solver.owned_trees or ()) for solver in slv_chain}
            chain_solver = next(
                (
                    solver
                    for solver in handler_chain_solvers
                    if target is not None
                    and any(
                        _tree_owns(tree, target) for tree in trees_by_solver_id.get(solver.id, ())
                    )
                ),
                None,
            )
            if chain_solver is None:
                raise RuntimeError(
                    "command forwarding: joint "
                    f"'{p.label(target) if target is not None else target}' belongs to no "
                    "kinematic tree this handler's runtimes own"
                )
            runtime_solver = next(solver for solver in slv_chain if solver.id == chain_solver.id)
            forwarded_commands.append(
                ForwardedCommand(
                    f"cmd-fwd-{p.id(plan.controller)}",
                    controller.control_signal,
                    f"{runtime_solver.runtime_prefix}{p.label(target)}"
                    if target is not None
                    else "",
                    chain_solver.id,
                )
            )
        when_monitors = [p.monitor_entry(n) for n in when_mon_nodes]
        while_monitors = [p.monitor_entry(n) for n in while_mon_nodes]
        until_monitors = [p.monitor_entry(n) for n in until_mon_nodes]

        when_elapsed_ids = _elapsed_coordinate_ids(when_evaluators)
        active_elapsed_ids = _elapsed_coordinate_ids(while_evaluators + until_evaluators)
        has_when_elapsed = bool(when_elapsed_ids)
        has_active_elapsed = bool(active_elapsed_ids)
        has_elapsed = has_when_elapsed or has_active_elapsed

        motions.append(
            GuardedMotionBlock(
                id=handler.motion.id,
                handler=handler.id,
                name=handler.motion.name,
                description=(handler.motion.description or "").splitlines(),
                has_when_elapsed=has_when_elapsed,
                has_active_elapsed=has_active_elapsed,
                when_elapsed_ids=when_elapsed_ids,
                active_elapsed_ids=active_elapsed_ids,
                when_evaluators=when_evaluators,
                while_evaluators=while_evaluators,
                until_evaluators=until_evaluators,
                controllers=controllers,
                when_monitors=when_monitors,
                while_monitors=while_monitors,
                until_monitors=until_monitors,
                when_schedule=when_schedule,
                while_schedule=while_schedule,
                until_schedule=until_schedule,
                has_elapsed=has_elapsed,
                has_until_condition=bool(until_evaluators),
                until_any=handler.motion.until_any,
                when_any=handler.motion.when_any,
                serial_chain_solvers=handler_chain_solvers,
                relative_poses=_relative_poses_for_motion(
                    while_evaluators + when_evaluators + until_evaluators,
                    view_map,
                    handler_chain_solvers,
                ),
                scene_relative_poses=_scene_relative_poses_for_motion(
                    view_map,
                    handler_chain_solvers,
                    while_evaluators + when_evaluators + until_evaluators,
                ),
                pose_axis_error_groups=pose_axis_error_groups,
                while_pre_schedule=pre_group_schedule,
                forwarded_commands=forwarded_commands,
                snapshots=(
                    _snapshots_for_motion(
                        while_evaluators + when_evaluators + until_evaluators,
                        motion.while_ + motion.when + motion.until,
                        snapshot_source_map,
                        view_map,
                        closure_output_map,
                        data_reference_map,
                        while_schedule + when_schedule + until_schedule,
                        closures,
                        snapshot_trigger_map,
                        _motion_suffix(p, motion_node),
                        snapshot_owner_map,
                        motion_tokens,
                    )
                ),
                path_projections=_path_projections_for_motion(
                    pre_group_schedule + while_schedule, closures
                ),
            )
        )

    # One motion maps to exactly one constraint handler; a repeated motion id is a modelling
    # error and is rejected rather than silently merged.
    handler_by_motion: dict[str, str] = {}
    for motion in motions:
        if motion.id in handler_by_motion:
            raise ValueError(
                f"Motion '{motion.id}' is governed by more than one constraint handler "
                f"('{handler_by_motion[motion.id]}' and '{motion.handler}'). Each motion "
                f"must map to exactly one handler; split the motion or merge the handlers."
            )
        handler_by_motion[motion.id] = motion.handler

    ordered = sorted(
        motions, key=lambda motion: next(h.order for h in handlers if h.id == motion.handler)
    )
    for motion in ordered:
        _set_motion_conditions(motion)
        _add_group_type_flags(motion.pose_axis_error_groups)
        # Controller signal ids + this motion's declared-pose components.
        _annotate_controller_signals(motion.controllers, closures or {})
        motion_refs = collect_motion_references(motion, closures or {})
        motion.declared_pose_components = declared_pose_component_entries(
            data_structures or [], pose_components or {}, motion_refs
        )
    # Ordered: FSM wiring tags monitors/motions, then the capability booleans, then the gate
    # calls that read them.
    fsm_meta = _apply_fsm_wiring(ordered, fsm)
    add_motion_function_interfaces(ordered)
    _apply_fsm_gate_calls(ordered, fsm_meta["cpp_namespace"])
    return ordered, fsm_meta


def _assign_monitor_event_indexes(handlers) -> None:
    """Assign each monitor a stable per-handler event index."""
    event_idx = 0
    for handler in handlers:
        for monitor in handler.monitors:
            if monitor.monitor_type == "EdgeTriggeredMonitor":
                monitor.event_idx = event_idx
                event_idx += 1


def _apply_monitor_debounce(handlers, control_period_ns: int) -> None:
    """Convert each monitor's debounce duration to a step count from the control period."""
    for handler in handlers:
        for monitor in handler.monitors:
            if getattr(monitor, "debounce_duration_s", None) is not None:
                monitor.debounce_steps = round(
                    monitor.debounce_duration_s / (control_period_ns * 1e-9)
                )


def _motion_done_terms(motion) -> list:
    """Structured UNTIL-member terms that end a motion (edge monitors → event flag, level
    monitors → their boolean flag). Joined by until_any and rendered by bool-condition."""
    mid = _field(motion, "id")
    terms = []
    for monitor in _field(motion, "until_monitors", []):
        if _field(monitor, "is_edge_triggered"):
            terms.append({"kind": "event", "motion_id": mid, "monitor_id": _field(monitor, "id")})
        else:
            terms.append({"kind": "flag", "motion_id": mid, "flag": _field(monitor, "flag")})
    return terms


def _evaluator_term(e) -> dict:
    """Structured boolean term for an evaluator: an elapsed timing predicate or a solver
    constraint-satisfied check, rendered to C++ by the bool-condition template.

    Every term kind reads shared and nothing else, so the same condition renders identically inside
    the motion and in the introspection sample that runs outside it.
    """
    if _field(e, "is_elapsed"):
        op = _field(e, "elapsed_op") or ">="
        thr = _field(e, "elapsed_threshold_s") or 0.0
        # Pre-format the threshold (fixed 6-decimal) so the emitted literal is stable.
        if op == "==":
            tol = _field(e, "elapsed_tolerance_s") or 0.0
            return {
                "kind": "elapsed-eq",
                "elapsed_id": _elapsed_coordinate_id(e),
                "threshold": f"{thr:.6f}",
                "tolerance": f"{tol:.6f}",
            }
        return {
            "kind": "elapsed",
            "elapsed_id": _elapsed_coordinate_id(e),
            "op": op,
            "threshold": f"{thr:.6f}",
        }
    term = {"kind": "constraint", "error_id": _field(_field(e, "error"), "id")}
    # Omitted, not empty: ST4 reads an empty string as present, and would emit a bare `shared.`.
    tolerance_id = _field(_field(e, "tolerance"), "id")
    if tolerance_id:
        term["tolerance_id"] = tolerance_id
    return term


def _set_monitor_conditions(motion, evaluators_key: str, monitors_key: str, any_key: str) -> None:
    """Stamp the structured active-phase terms onto the aggregate monitor + any
    elapsed-error monitors. Rendered to C++ by the bool-condition template."""
    evaluators = _field(motion, evaluators_key, [])
    terms = [
        _evaluator_term(e) for e in evaluators if _field(e, "error") or _field(e, "is_elapsed")
    ]
    any_flag = bool(_field(motion, any_key))
    elapsed_terms_by_error = {
        _field(_field(e, "error"), "id"): _evaluator_term(e)
        for e in evaluators
        if _field(e, "is_elapsed") and _field(e, "error")
    }
    aggregate_key = "is_until_aggregate" if any_key == "until_any" else "is_when_aggregate"

    def _stamp(monitor, active_terms, active_any):
        _set_field(monitor, "active_terms", active_terms)
        _set_field(monitor, "active_terms_present", bool(active_terms))
        _set_field(monitor, "active_any", active_any)
        _set_field(monitor, "has_active", True)

    for monitor in _field(motion, monitors_key, []):
        group_ids = set(_field(monitor, "group_constraint_ids") or ())
        if group_ids:
            _stamp(
                monitor,
                [
                    _evaluator_term(e)
                    for e in evaluators
                    if _field(_field(e, "constraint"), "id") in group_ids
                    and (_field(e, "error") or _field(e, "is_elapsed"))
                ],
                bool(_field(monitor, "group_any")),
            )
            continue
        if _field(monitor, aggregate_key):
            _stamp(monitor, terms, any_flag)
            continue
        error_id = _field(_field(monitor, "error"), "id")
        if error_id in elapsed_terms_by_error:
            _stamp(monitor, [elapsed_terms_by_error[error_id]], False)


def _set_motion_conditions(motion) -> None:
    """Fold the UNTIL/WHEN/done structured boolean terms onto a motion (rendered to C++ by
    the bool-condition template). WHEN joins with when_any, done with until_any."""
    _set_monitor_conditions(motion, "until_evaluators", "until_monitors", "until_any")
    when_terms = [
        _evaluator_term(e)
        for e in _field(motion, "when_evaluators", [])
        if _field(e, "error") or _field(e, "is_elapsed")
    ]
    _set_field(motion, "when_terms", when_terms)
    _set_field(motion, "when_terms_present", bool(when_terms))
    _set_monitor_conditions(motion, "when_evaluators", "when_monitors", "when_any")
    done_terms = _motion_done_terms(motion)
    _set_field(motion, "done_terms", done_terms)
    _set_field(motion, "done_terms_present", bool(done_terms))


def _records_events(monitors: list) -> bool:
    """Whether any of these monitors records an event occurrence into the coordination buffer."""
    return any(_field(m, "is_edge_triggered") for m in monitors)


def add_motion_function_interfaces(motions: list) -> None:
    """Fold per-motion capability booleans (which context objects each generated function needs); the
    C++ signatures and call args are built from these by sig-params / sig-args.

    Also assigns each motion its introspection index, so the frame-log schema and the generated
    sample switch read one field rather than agreeing with a second generator.
    """
    for index, motion in enumerate(motions):
        _set_field(motion, "index", index)
        has_when_elapsed = any(
            _field(e, "is_elapsed") for e in _field(motion, "when_evaluators", [])
        )
        has_when_logic = bool(_field(motion, "when_schedule") or _field(motion, "when_evaluators"))
        when_mons = _field(motion, "when_monitors") or []
        until_mons = _field(motion, "until_monitors") or []
        while_mons = _field(motion, "while_monitors") or []
        has_pose = bool(_field(motion, "declared_pose_components"))
        when_sched = bool(_field(motion, "when_schedule"))
        until_sched = bool(_field(motion, "until_schedule"))
        when_fsm = any(_field(m, "fsm_namespace") for m in when_mons)
        until_fsm = any(_field(m, "fsm_namespace") for m in until_mons)
        has_forwarded_commands = bool(_field(motion, "forwarded_commands"))
        has_serial_chain = bool(_field(motion, "serial_chain_solvers"))

        _set_field(motion, "can_start_needs_state", has_when_elapsed)
        _set_field(motion, "can_start_needs_shared", has_when_logic)
        _set_field(motion, "can_start_needs_robot", False)

        _set_field(motion, "when_needs_state", has_when_elapsed or bool(when_mons))
        _set_field(
            motion,
            "when_needs_shared",
            has_when_elapsed or has_pose or when_sched or bool(when_mons),
        )
        _set_field(motion, "when_needs_robot", when_fsm)

        _set_field(motion, "until_needs_state", bool(until_mons))
        _set_field(motion, "until_needs_shared", until_sched or bool(until_mons))
        _set_field(motion, "until_needs_robot", until_fsm)

        _set_field(motion, "monitor_needs_state", bool(when_mons) or bool(until_mons))
        _set_field(
            motion,
            "monitor_needs_shared",
            when_sched or bool(when_mons) or until_sched or bool(until_mons),
        )
        _set_field(motion, "monitor_needs_robot", when_fsm or until_fsm)

        _set_field(motion, "apply_needs_state", has_serial_chain)
        # Gate on torque_saturation, not on joint_space_cmd_samples: this runs before
        # _build_introspection, so the sample list does not exist yet.
        logs_joint_cmd = any(
            _field(solver, "torque_saturation")
            for solver in (_field(motion, "serial_chain_solvers") or [])
        )
        _set_field(motion, "apply_needs_shared", has_forwarded_commands or logs_joint_cmd)
        _set_field(motion, "apply_needs_robot", has_serial_chain or has_forwarded_commands)

        # An edge is the occurrence: a flag monitor holds a level and never reaches the buffer.
        when_events = _records_events(when_mons)
        until_events = _records_events(until_mons)
        control_events = _records_events(while_mons)
        _set_field(motion, "when_needs_events", when_events)
        _set_field(motion, "until_needs_events", until_events)
        _set_field(motion, "monitor_needs_events", when_events or until_events)
        _set_field(motion, "control_needs_events", control_events)
        _set_field(motion, "step_needs_events", until_events or control_events)


_FSM_NS = "https://secorolab.github.io/metamodels/behaviour/fsm#"
_EL_NS = "https://secorolab.github.io/metamodels/behaviour/event-loop#"


def _fsm_from_graph(g) -> dict | None:
    """Frame the FSM named graph (states/events/transitions/reactions, folded into the model dataset
    by motion-spec-dsl) into the same dict shape the standalone .hpp uses, so codegen needs no
    fsm_ir.json read. None when the model imports no .fsm.
    """
    FSM = rdflib.Namespace(_FSM_NS)
    EL = rdflib.Namespace(_EL_NS)
    fsm_ref = next(iter(g.subjects(RDF["type"], FSM["FiniteStateMachine"])), None)
    if fsm_ref is None:
        return None

    def ident(uri):
        """FSM identifier token (upper-cased var name) for a graph URI."""
        return get_valid_var_name(g.compute_qname(uri)[2]).upper()

    # Graph iteration order is rdflib's: sort so two generations agree on state/event order,
    # since event indices are assigned from it and baked into the generated C++.
    state_uris = dict(sorted((ident(s), str(s)) for s in g.objects(fsm_ref, FSM["states"])))
    states = list(state_uris)
    event_loop_node = g.value(fsm_ref, EL["event-loop"])
    event_uris = dict(
        sorted((ident(e), str(e)) for e in g.objects(event_loop_node, EL["has-event"]))
    )
    events = list(event_uris)

    transitions_table = []
    for tr in g.objects(fsm_ref, FSM["transitions"]):
        transitions_table.append(
            {
                "id": ident(tr),
                "uri": str(tr),
                "from_state": ident(g.value(tr, FSM["transition-from"])),
                "to_state": ident(g.value(tr, FSM["transition-to"])),
            }
        )
    reactions_table = []
    for rx in g.objects(fsm_ref, FSM["reactions"]):
        fires = sorted(ident(ev) for ev in g.objects(rx, FSM["fires-events"]))
        reactions_table.append(
            {
                "id": ident(rx),
                "uri": str(rx),
                "when_event": ident(g.value(rx, EL["ref-event"])),
                "do_transition": ident(g.value(rx, FSM["do-transition"])),
                "fires_events": fires,
                "num_fires": len(fires),
            }
        )
    transitions_table.sort(key=lambda row: row["id"])
    reactions_table.sort(key=lambda row: row["id"])

    description_node = g.value(fsm_ref, FSM["description"])
    # Event/state IRIs share the FSM node's parent path (…/<model>/fsm/); is_fsm_event
    # matches monitor event IRIs against it.
    namespace_uri = str(rdflib.Namespace(f"{iri_parent(fsm_ref)}/"))
    return {
        "name": str(g.value(fsm_ref, FSM["name"])),
        "description": str(description_node) if description_node is not None else None,
        "start_state": ident(g.value(fsm_ref, FSM["start-state"])),
        "end_state": ident(g.value(fsm_ref, FSM["end-state"])),
        "states": states,
        "state_uris": state_uris,
        "events": events,
        "event_uris": event_uris,
        "transitions_table": transitions_table,
        "reactions_table": reactions_table,
        "namespace_uri": namespace_uri,
    }


def _event_to_state(fsm: dict) -> dict[str, str]:
    """Map each FSM event token to the state it transitions out of (the state the motion
    runs in): the from-state of the transition the event's reaction fires."""
    transition_from = {t["id"]: t["from_state"] for t in fsm["transitions_table"]}
    return {
        r["when_event"]: transition_from[r["do_transition"]]
        for r in fsm["reactions_table"]
        if r["do_transition"] in transition_from
    }


def is_fsm_event(monitor, fsm_ns_uri: str | None) -> bool:
    # A monitor fires the FSM only when its event lives in the FSM's namespace;
    # standalone (monitor-owned) events keep the existing warn stub.
    """True when a monitor fires an FSM event: edge-triggered with an event in the FSM namespace."""
    return bool(
        fsm_ns_uri
        and _field(monitor, "is_edge_triggered")
        and iri_is_descendant(fsm_ns_uri, _field(monitor, "event_uri") or "")
    )


def _apply_fsm_wiring(motions, fsm) -> dict:
    """Tag FSM-event monitors and their motions from the framed FSM, and return the codegen wiring
    (C++ namespace, header, heartbeat) folded into ``ir["fsm"]``. Runs during motion construction,
    before the function-interface pass so the FSM-added robot param is picked up.
    """
    fsm_namespace = fsm["name"].lower() if fsm else None
    events = fsm.get("events", []) if fsm else []
    fsm_event_index = {event: idx for idx, event in enumerate(events)}
    fsm_step_event = "E_STEP" if "E_STEP" in events else None
    # The heartbeat is the clock and is not logged every tick, but where a transition's guard is
    # the clock that occurrence caused the state change, so name those transitions.
    transitions_by_id = {t.get("id"): t for t in (fsm.get("transitions_table", []) if fsm else [])}
    fsm_step_transitions = [
        {"from": transition.get("from_state"), "to": transition.get("to_state")}
        for reaction in (fsm.get("reactions_table", []) if fsm else [])
        if reaction.get("when_event") == fsm_step_event
        for transition in [transitions_by_id.get(reaction.get("do_transition"))]
        if transition
    ]
    meta = {
        "cpp_namespace": fsm_namespace,
        "header": f"{fsm['name']}.hpp" if fsm else None,
        "step_event": fsm_step_event,
        "step_event_idx": fsm_event_index.get(fsm_step_event, -1),
        "step_transitions": fsm_step_transitions,
    }
    if fsm_namespace is None:
        # Without an FSM the sequencer advances on a motion's own `until`, so one declaring none
        # can never be left and every motion after it is unreachable.
        stuck = [_field(m, "id") for m in motions if not _field(m, "has_until_condition")]
        if stuck:
            raise ValueError(
                f"motions {sorted(stuck)} declare no 'until' condition and the model imports no "
                "FSM, so nothing can end them; add an 'until' condition or coordinate the model "
                "with an FSM"
            )
        return meta

    fsm_ns_uri = fsm.get("namespace_uri")
    event_state = _event_to_state(fsm)
    by_id = {_field(m, "id"): m for m in motions}

    def stamp_event(monitor):
        """Bind an FSM-event monitor to the FSM's namespace and event slot."""
        _set_field(monitor, "fsm_namespace", fsm_namespace)
        _set_field(
            monitor, "fsm_event_idx", fsm_event_index.get(_field(monitor, "event_name") or "", -1)
        )

    def tag_run_state(motion, monitors):
        """Tag FSM-event monitors and set their motion's run state."""
        for monitor in monitors:
            if is_fsm_event(monitor, fsm_ns_uri):
                stamp_event(monitor)
                state = event_state.get(_field(monitor, "event_name") or "")
                if state and not _field(motion, "fsm_state"):
                    _set_field(motion, "fsm_state", state)

    for motion in motions:
        # An event-triggered snapshot re-samples when its trigger is in the current event
        # buffer; that only compiles if the event is one this FSM declares.
        for snapshot in _field(motion, "snapshots", []) or []:
            trigger = _field(snapshot, "trigger_event")
            if not trigger:
                continue
            if trigger not in fsm_event_index:
                raise ValueError(
                    f"Snapshot '{_field(snapshot, 'target_id')}' in motion "
                    f"'{_field(motion, 'id')}' triggers on '{trigger}', which the FSM "
                    f"'{fsm_namespace}' does not declare."
                )
            _set_field(snapshot, "fsm_namespace", fsm_namespace)
        tag_run_state(
            motion, _field(motion, "until_monitors", []) + _field(motion, "while_monitors", [])
        )
        for monitor in _field(motion, "when_monitors", []):
            if not is_fsm_event(monitor, fsm_ns_uri):
                continue
            stamp_event(monitor)
            fallback_id = _field(monitor, "fallback_motion")
            if not fallback_id:
                raise ValueError(
                    f"WHEN monitor '{_field(monitor, 'id')}' on FSM-wired motion "
                    f"'{_field(motion, 'id')}' must declare a waiting hold motion "
                    f"(e.g. '... otherwise hold <hold-motion>'). A WHEN precondition "
                    f"without a fallback would leave the arm uncommanded while waiting."
                )
            fallback = by_id.get(fallback_id)
            if fallback is None:
                raise ValueError(
                    f"WHEN monitor '{_field(monitor, 'id')}' names unknown fallback motion "
                    f"'{fallback_id}'."
                )
            state = event_state.get(_field(monitor, "event_name") or "")
            if state and not _field(fallback, "fsm_state"):
                _set_field(fallback, "fsm_state", state)
            gates = _field(fallback, "fsm_when_gate_motions")
            if gates is None:
                gates = []
                _set_field(fallback, "fsm_when_gate_motions", gates)
            if _field(motion, "id") not in gates:
                gates.append(_field(motion, "id"))
    return meta


def _apply_fsm_gate_calls(motions, fsm_namespace) -> None:
    """Fold each fallback state's WHEN-evaluation gate calls: the gated motion id plus its
    when-signature capability booleans. Runs after function interfaces so when_needs_* exist; the
    C++ ``monitor_when_<id>(<args>)`` call is rendered by the template via sig-args.
    """
    if fsm_namespace is None:
        return
    by_id = {_field(m, "id"): m for m in motions}
    for fallback in motions:
        gate_ids = _field(fallback, "fsm_when_gate_motions")
        if not gate_ids:
            continue
        _set_field(
            fallback,
            "fsm_when_gate_calls",
            [
                {
                    "gid": gate_id,
                    "needs_state": _field(by_id[gate_id], "when_needs_state", False),
                    "needs_shared": _field(by_id[gate_id], "when_needs_shared", False),
                    "needs_robot": _field(by_id[gate_id], "when_needs_robot", False),
                    "needs_events": _field(by_id[gate_id], "when_needs_events", False),
                }
                for gate_id in gate_ids
                if gate_id in by_id
            ],
        )
