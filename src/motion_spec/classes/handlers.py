# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Evaluators, controllers, monitors and the handler that binds them to one motion.

`StateField` lives here rather than in `rdf_parser/constraint_handler.py`: that module is the
only one that *derives* a controller, and a derivation exports tables that `communication.py`
renders. `StateField` is the row type of one such table (`CONTROLLER_STATE_FIELDS`, which stays
in `constraint_handler.py`), not the table itself and not a derivation -- a future controller DSL
replaces the table and the derivation, with no reason to redefine the row type.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from motion_spec.classes.base import INTERNAL
from motion_spec.classes.constraints import Constraint, GuardedMotion
from motion_spec.classes.dynamics import Saturation
from motion_spec.classes.qudt import Quantity


class EvaluatorType(str, Enum):
    """Assignment vs error kind of a constraint evaluator."""

    AssignmentEvaluator = "AssignmentEvaluator"
    ErrorEvaluator = "ErrorEvaluator"


@dataclass
class ConstraintEvaluator:
    """Evaluates a constraint into an error signal (or an elapsed-timing predicate)."""

    id: str
    type_: EvaluatorType
    constraint: Constraint
    error: Quantity | None
    # The authored band this constraint is satisfied within; unset falls back to the global one.
    tolerance: Quantity | None = None
    is_elapsed: bool = False
    elapsed_op: str | None = None
    elapsed_threshold_s: float | None = None
    elapsed_tolerance_s: float | None = None
    # The action_msgs GoalStatus constant an action goal must reach for this to hold.
    goal_status: str | None = None
    type: str = field(default="ConstraintEvaluator")


@dataclass
class PIDController:
    """A proportional-integral-derivative controller."""

    id: str
    control_signal: Quantity
    error_signal: Quantity | None = None
    measured_derivative: Quantity | None = None
    proportional_gain: float | None = None
    integral_gain: float | None = None
    derivative_gain: float | None = None
    decay_rate: float | None = None
    output_saturation: Saturation | None = None
    integral_saturation: Saturation | None = None
    # Abstract signal ids folded from the error-evaluator closure; the C++ access
    # expression is rendered backend-side by access-expr (shared_data.stg).
    measured_signal: str | None = None
    setpoint_signal: str | None = None
    # The band its constraint is satisfied within, as the model authored it.
    tolerance_id: str = ""
    # The constraint this controller serves; per-axis controllers share the authored one.
    constraint: str | None = None
    constraint_uri: str | None = None
    type: str = "ProportionalIntegralDerivative"


@dataclass
class ImpedanceController:
    """An impedance controller."""

    id: str
    control_signal: Quantity
    error_signal: Quantity | None = None
    integral_gain: float | None = None
    stiffness: float | None = None
    damping: float | None = None
    output_saturation: Saturation | None = None
    measured_signal: str | None = None
    setpoint_signal: str | None = None
    # The band its constraint is satisfied within, as the model authored it.
    tolerance_id: str = ""
    # The constraint this controller serves; per-axis controllers share the authored one.
    constraint: str | None = None
    constraint_uri: str | None = None
    type: str = "ImpedanceController"


@dataclass
class FeedForwardController:
    """A feed-forward controller."""

    id: str
    control_signal: Quantity
    reference_signal: Quantity | None = None
    output_saturation: Saturation | None = None
    # Not consumed by the controller; folded from its constraint's evaluator so the logged
    # slot carries the real error instead of a zero.
    error_signal: str | None = None
    measured_signal: str | None = None
    setpoint_signal: str | None = None
    # The band its constraint is satisfied within, as the model authored it.
    tolerance_id: str = ""
    # The constraint this controller serves; per-axis controllers share the authored one.
    constraint: str | None = None
    constraint_uri: str | None = None
    type: str = "FeedForwardController"


Controller = PIDController | ImpedanceController | FeedForwardController


@dataclass
class LevelMonitor:
    """A monitor that continuously sets a boolean flag from its constraint."""

    id: str
    monitor_type: str
    error: Quantity | None
    flag: str | None
    # The band its constraint is satisfied within, when the model authored one.
    tolerance: Quantity | None = None
    is_edge_triggered: bool = False
    is_until_aggregate: bool = False
    is_when_aggregate: bool = False
    # Set when the monitor targets an expression node: the member constraint ids it
    # aggregates, and whether they combine with 'any' rather than 'all'.
    group_constraint_ids: list[str] = field(default_factory=list, metadata=INTERNAL)
    group_constraint_uris: list[str] = field(default_factory=list, metadata=INTERNAL)
    group_constraint_tolerances: list[str] = field(default_factory=list, metadata=INTERNAL)
    # The constraints this monitor watches, by id: how a term read directly off shared state --
    # an elapsed clock, an action goal's status -- is matched to the monitor that reads it.
    constraint_ids: list[str] = field(default_factory=list, metadata=INTERNAL)
    # The same constraints by URI, for joins against the model graph.
    constraint_uris: list[str] = field(default_factory=list, metadata=INTERNAL)
    group_any: bool = field(default=False, metadata=INTERNAL)
    # Structured active-phase boolean terms (rendered to C++ by the bool-condition template).
    has_active: bool = False
    active_terms: list | None = None
    active_terms_present: bool = False
    active_any: bool = False
    ros: RosPublication | None = None
    answer: RosGoalAnswer | None = None
    type: str = field(default="LevelMonitor")


@dataclass
class EdgeMonitor:
    """A monitor that fires an FSM event on a rising edge of its constraint."""

    id: str
    monitor_type: str
    error: Quantity | None
    event: str | None
    event_idx: int | None
    # The band its constraint is satisfied within, when the model authored one.
    tolerance: Quantity | None = None
    is_edge_triggered: bool = True
    is_until_aggregate: bool = field(default=False, metadata=INTERNAL)
    is_when_aggregate: bool = field(default=False, metadata=INTERNAL)
    # Set when the monitor targets an expression node: the member constraint ids it
    # aggregates, and whether they combine with 'any' rather than 'all'.
    group_constraint_ids: list[str] = field(default_factory=list, metadata=INTERNAL)
    group_constraint_uris: list[str] = field(default_factory=list, metadata=INTERNAL)
    group_constraint_tolerances: list[str] = field(default_factory=list, metadata=INTERNAL)
    # The constraints this monitor watches, by id: how a term read directly off shared state --
    # an elapsed clock, an action goal's status -- is matched to the monitor that reads it.
    constraint_ids: list[str] = field(default_factory=list, metadata=INTERNAL)
    # The same constraints by URI, for joins against the model graph.
    constraint_uris: list[str] = field(default_factory=list, metadata=INTERNAL)
    group_any: bool = field(default=False, metadata=INTERNAL)
    event_uri: str | None = None
    event_name: str | None = None
    fallback_motion: str | None = None
    # The authored duration the constraint must hold before the edge fires, by id: the runtime
    # accumulates measured cycle time against that shared value, so the model's bound has one
    # source of truth. None when absent.
    debounce_id: str | None = None
    # Structured active-phase boolean terms (rendered to C++ by the bool-condition template).
    has_active: bool = False
    active_terms: list | None = None
    active_terms_present: bool = False
    active_any: bool = False
    # FSM binding (folded when the monitor's event lives in the FSM namespace).
    fsm_namespace: str | None = None
    fsm_event_idx: int | None = None
    ros: RosPublication | None = None
    answer: RosGoalAnswer | None = None
    type: str = field(default="EdgeMonitor")


@dataclass
class RosPublication:
    """What a monitor publishes: the topic, and the fields each polarity writes.
    `include`/`cpp_type` come from the message class rosidl resolved, `pub_id` is the C++
    publisher member. `on_satisfied`/`on_violated` are the authored `{path, cpp_value}` rows
    live while the constraint holds and while it does not; either may be empty, so the
    presence flags say which branches exist. `auto_time`/`auto_context_id` are the fields
    nothing may author -- the node fills them from its clock and its scenario parameter.
    """

    channel: str | None = None
    type: str | None = field(default=None, metadata=INTERNAL)
    pkg: str | None = field(default=None, metadata=INTERNAL)
    include: str | None = field(default=None, metadata=INTERNAL)
    cpp_type: str | None = None
    pub_id: str | None = None
    on_satisfied: list[dict] = field(default_factory=list)
    on_violated: list[dict] = field(default_factory=list)
    has_satisfied: bool = False
    has_violated: bool = False
    auto_time: list[str] = field(default_factory=list)
    auto_context_id: list[str] = field(default_factory=list)
    # How often the verdict goes out, as the model states it and as the loop counts it. Without
    # a rate it goes out every cycle the motion is active, so `divider` stays unset.
    rate_hz: float | None = field(default=None, metadata=INTERNAL)
    divider: int | None = None
    # Occurrence form: the payload field an announced event's IRI is written into, instead of
    # authored field rows. `occurrence_events` are the events the monitor announces, resolved to
    # their FSM enum tokens; one message is published per event that fired this cycle.
    occurrence_path: str | None = None
    occurrence_events: list = field(default_factory=list)
    occurrence_namespace: str | None = None


@dataclass
class RosGoalAnswer:
    """How a monitor answers the goal in flight: the status it reports, the C++ result type it
    fills, and the authored `{path, cpp_value}` rows it fills it with.

    `satisfied` says which polarity of the monitor answers -- the state block the model wrote it
    in -- so the loop answers on the same condition the monitor's other actions run under.
    `auto_time`/`auto_context_id` are the result fields nothing may author: the node fills them
    from its clock and its scenario parameter.
    """

    outcome: str
    method: str
    result_cpp_type: str
    fields: list[dict] = field(default_factory=list)
    satisfied: bool = True
    auto_time: list[str] = field(default_factory=list)
    auto_context_id: list[str] = field(default_factory=list)


Monitor = LevelMonitor | EdgeMonitor


@dataclass
class ConstraintHandler:
    """Binds a motion to its progress policy, evaluators, controllers and monitors."""

    id: str
    motion: GuardedMotion
    evaluators: list[ConstraintEvaluator] = field(metadata=INTERNAL)
    controllers: list[Controller]
    monitors: list[Monitor]
    order: int = field(default=0, metadata=INTERNAL)
    type: str = field(default="ConstraintHandler")


@dataclass
class Perturbation:
    """A disturbance the simulator applies to one body while a state is active.

    `wrench_id` is what the authored magnitude and direction compose to, stated in the frame the
    direction is seen by; `applied_id` is that wrench rotated into the world frame MuJoCo's
    applied-force slots are read in, and is what the frame log records. `active_id` says whether
    the window was open on the cycle the log recorded.
    """

    id: str
    body: str
    robot_id: str
    wrench_id: str
    applied_id: str
    active_id: str
    # The authored window length, by id: the runtime accumulates measured cycle time against that
    # shared value. None when the window lasts until the state exits.
    duration_id: str | None = None
    has_gate: bool = False
    # Structured boolean terms, rendered by the same template a monitor's condition uses.
    terms: list = field(default_factory=list)
    terms_present: bool = False
    gate_any: bool = False
    type: str = field(default="Perturbation")


@dataclass(frozen=True)
class StateField:
    """One value a stateful controller keeps between ticks, and how its step call reads it."""

    name: str
    type: str
    getter: str
