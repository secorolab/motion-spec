# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Joint-space position and the signal-saturation limits applied on the force/torque side."""

from __future__ import annotations

from dataclasses import dataclass, field

from motion_spec.classes.qudt import Quantity


@dataclass
class JointPosition:
    """A joint-position quantity for a named joint."""

    id: str
    joint_name: str
    type: str = field(default="JointPosition")


@dataclass
class Saturation:
    """Input/output saturation limits applied to a signal."""

    id: str
    input_signal: Quantity
    output_signal: Quantity
    maximum: Quantity | None = None
    lower: Quantity | None = None
    upper: Quantity | None = None
    type: str = field(default="Saturation")
