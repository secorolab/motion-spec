# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Joint-space position and the signal-saturation limits applied on the force/torque side."""

from __future__ import annotations

from dataclasses import dataclass, field

from motion_spec.classes.base import INTERNAL
from motion_spec.classes.qudt import Quantity


@dataclass
class JointQuantity:
    """A joint position, velocity or force for a named joint."""

    id: str
    joint_name: str
    type: str
    # resolve by identity: two grippers share local joint names
    joint_uri: str = field(default="", metadata=INTERNAL)
    joint_index: int | None = None
    on_chain: bool = False
    # read when not on_chain; slot 0 and index 0 are both falsy to ST4
    world_slot: int = 0
    normalization: dict | None = None


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
