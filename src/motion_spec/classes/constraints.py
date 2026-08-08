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
class Constraint:
    """A constraint on a quantity together with its parameter."""

    id: str
    quantity: Quantity
    parameter: EqualityConstraint | UnilateralConstraint | BilateralConstraint | OutsideConstraint
    type: str = field(default="Constraint")


@dataclass
class GuardedMotion:
    """A guarded motion: its when/while/until constraint sets."""

    id: str
    when: list[Constraint]
    while_: list[Constraint]
    until: list[Constraint]
    until_any: bool = False
    when_any: bool = False
    name: str = ""
    description: str | None = None
    type: str = field(default="GuardedMotion")
