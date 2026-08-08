# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A constraint on a quantity, its four parameter shapes, and the guarded motion they compose
into a when/while/until schedule.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from motion_spec.classes.qudt import Quantity


class UnilateralConstraintType(str, Enum):
    """Greater-than vs less-than kind of a unilateral constraint."""

    GreaterThan = "GreaterThan"
    LessThan = "LessThan"


@dataclass
class EqualityConstraint:
    """Constraint parameter: equality to a reference value."""

    reference_value: Quantity
    type: str = field(default="EqualityConstraint")


@dataclass
class UnilateralConstraint:
    """Constraint parameter: a one-sided threshold."""

    type_: UnilateralConstraintType
    threshold: Quantity
    type: str = field(default="UnilateralConstraint")


@dataclass
class BilateralConstraint:
    """Constraint parameter: inside a lower/upper band."""

    lower_threshold: Quantity
    upper_threshold: Quantity
    type: str = field(default="BilateralConstraint")


@dataclass
class OutsideConstraint:
    """Constraint parameter: outside a lower/upper band."""

    lower_threshold: Quantity
    upper_threshold: Quantity
    type: str = field(default="OutsideConstraint")


@dataclass
class GoalStatus:
    """An action goal's terminal status: a slot the client writes, carrying no unit or kind."""

    id: str
    type: str = field(default="GoalStatus")


@dataclass
class Constraint:
    """A constraint on a quantity together with its parameter."""

    id: str
    quantity: Quantity | GoalStatus
    parameter: (
        EqualityConstraint | UnilateralConstraint | BilateralConstraint | OutsideConstraint | None
    )
    type: str = field(default="Constraint")


@dataclass(frozen=True)
class ConstraintTransition:
    """One condition a when or until phase can be met by.

    A lone constraint stands for itself; an expression node is one transition over its members,
    met when any of them holds or only when all do.
    """

    id: str
    any: bool
    constraints: tuple[Constraint, ...]


@dataclass
class GuardedMotion:
    """A guarded motion: the transitions its when and until phases turn on, and what it holds
    during.
    """

    id: str
    when_transitions: list[ConstraintTransition]
    while_: list[Constraint]
    until_transitions: list[ConstraintTransition]
    name: str = ""
    description: str | None = None
    type: str = field(default="GuardedMotion")

    @property
    def when(self) -> list[Constraint]:
        """Every constraint the when phase names, in graph order."""
        return [item for transition in self.when_transitions for item in transition.constraints]

    @property
    def until(self) -> list[Constraint]:
        """Every constraint the until phase names, in graph order."""
        return [item for transition in self.until_transitions for item in transition.constraints]
