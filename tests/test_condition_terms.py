# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A condition that lowers to no terms renders as a constant, and the constant is the opposite of
what the model asked for in both directions: a monitor that can never fire, a precondition that is
always open. Neither may reach the generated program."""

from __future__ import annotations

import pytest
from rdf_utils.constraints import ConstraintViolation

from motion_spec.classes.constraints import Constraint
from motion_spec.classes.handlers import ConstraintEvaluator, EvaluatorType, LevelMonitor
from motion_spec.classes.motion import MotionUnit
from motion_spec.rdf_parser.coordination import _set_motion_conditions


def evaluator(cid: str) -> ConstraintEvaluator:
    """A constraint nothing evaluates: no error signal, no elapsed clock."""
    return ConstraintEvaluator(
        id=f"eval_{cid}",
        type_=EvaluatorType.AssignmentEvaluator,
        constraint=Constraint(id=cid, quantity=None, parameter=None),
        error=None,
    )


def motion(**kwargs) -> MotionUnit:
    fields = dict(
        id="motion_probe",
        motion_id="",
        name="probe",
        description=[],
        when_evaluators=[],
        while_evaluators=[],
        until_evaluators=[],
        controllers=[],
        when_monitors=[],
        while_monitors=[],
        until_monitors=[],
        when_schedule=[],
        while_schedule=[],
        until_schedule=[],
    )
    fields.update(kwargs)
    return MotionUnit(**fields)


def test_a_when_precondition_that_evaluates_to_nothing_is_rejected() -> None:
    # can_start would return a constant true and the motion would run as if the precondition
    # had been met.
    with pytest.raises(ConstraintViolation, match="start unconditionally"):
        _set_motion_conditions(motion(when_evaluators=[evaluator("c_ready")]))


def test_stating_no_when_precondition_still_means_always_ready() -> None:
    unit = motion()
    _set_motion_conditions(unit)
    assert unit.when_terms_present is False


def test_a_monitor_watching_nothing_evaluable_is_rejected() -> None:
    # It would render as a constant false: the event never fires and the FSM never leaves.
    watcher = LevelMonitor(
        id="mon_held", monitor_type="LevelMonitor", error=None, flag="held", is_until_aggregate=True
    )
    unit = motion(until_evaluators=[evaluator("c_held")], until_monitors=[watcher])
    with pytest.raises(ConstraintViolation, match="constant false"):
        _set_motion_conditions(unit)
