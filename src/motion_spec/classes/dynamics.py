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
    # Where the joint sits in the chain's joint array, resolved while generating. None for a
    # joint the chain does not articulate, which only a simulated backend can read.
    joint_index: int | None = None
    # The interval this measurement is read into, as its world block states it; None when the
    # block states none and the reading is taken as the backend reports it.
    normalization: dict | None = None
    type: str = field(default="JointPosition")


@dataclass
class JointCurrent:
    """A joint motor-current quantity for a named joint."""

    id: str
    joint_name: str
    # Where the joint sits in the chain's joint array, resolved while generating. None for a
    # gripper joint, which the device reports on its own channel.
    joint_index: int | None = None
    type: str = field(default="JointCurrent")


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
