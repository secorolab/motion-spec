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
    type: str = "ImpedanceController"


@dataclass
class FeedForwardController:
    """A feed-forward controller."""

    id: str
    control_signal: Quantity
    reference_signal: Quantity | None = None
    output_saturation: Saturation | None = None
    measured_signal: str | None = None
    setpoint_signal: str | None = None
    # The band its constraint is satisfied within, as the model authored it.
    tolerance_id: str = ""
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
    group_any: bool = field(default=False, metadata=INTERNAL)
    debounce_steps: int | None = None
    # Structured active-phase boolean terms (rendered to C++ by the bool-condition template).
    has_active: bool = False
    active_terms: list | None = None
    active_terms_present: bool = False
    active_any: bool = False
    ros: RosPublication | None = None
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
    group_any: bool = field(default=False, metadata=INTERNAL)
    event_uri: str | None = None
    event_name: str | None = None
    fallback_motion: str | None = None
    # Authored debounce duration (s); converted to debounce_steps once the loop period is known.
    # Stays None (not 0) when absent -- ST4's <if(x)> is true even for integer 0.
    debounce_duration_s: float | None = field(default=None, metadata=INTERNAL)
    debounce_steps: int | None = None
    # Structured active-phase boolean terms (rendered to C++ by the bool-condition template).
    has_active: bool = False
    active_terms: list | None = None
    active_terms_present: bool = False
    active_any: bool = False
    # FSM binding (folded when the monitor's event lives in the FSM namespace).
    fsm_namespace: str | None = None
    fsm_event_idx: int | None = None
    ros: RosPublication | None = None
    type: str = field(default="EdgeMonitor")


@dataclass
class RosPublication:
    """What a monitor publishes: the topic, and where the trinary verdict goes in the message.
    `include`/`cpp_type` come from the message class rosidl resolved, `pub_id` is the C++
    publisher member. `payload_path` is the one field the verdict is written to and
    `payload_cpp_type` the message class owning its TRUE/FALSE constants.
    `auto_time`/`auto_context_id` are the fields nothing may author -- the node fills them
    from its clock and its scenario parameter.
    """

    channel: str | None = None
    type: str | None = field(default=None, metadata=INTERNAL)
    pkg: str | None = field(default=None, metadata=INTERNAL)
    include: str | None = field(default=None, metadata=INTERNAL)
    cpp_type: str | None = None
    pub_id: str | None = None
    payload_path: str | None = None
    payload_cpp_type: str | None = None
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


@dataclass(frozen=True)
class StateField:
    """One value a stateful controller keeps between ticks, and how its step call reads it."""

    name: str
    type: str
    getter: str
