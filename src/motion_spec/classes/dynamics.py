# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Joint-space position and the signal-saturation limits applied on the force/torque side."""

from __future__ import annotations

from dataclasses import dataclass, field

from motion_spec.classes.base import INTERNAL
from motion_spec.classes.qudt import Quantity


@dataclass
class JointQuantity:
    """A joint position, velocity or motor current for a named joint."""

    id: str
    joint_name: str
    # JointPosition, JointVelocity or JointCurrent.
    type: str
    # The scene joint itself, so a segment resolves by identity: two grippers on two arms
    # carry the same local joint name and must not resolve to one segment.
    joint_uri: str = field(default="", metadata=INTERNAL)
    # Where the joint sits in the chain's joint array, resolved while generating. None for a
    # joint the chain does not articulate, which is read through its own world port instead.
    joint_index: int | None = None
    on_chain: bool = False
    # The world port row a joint off the chain is read from; `on_chain` says which of the two
    # this output uses, since slot 0 and index 0 are both falsy to the template engine.
    world_slot: int = 0
    # The interval this measurement is read into, as its world block states it; None when the
    # block states none and the reading is taken as the backend reports it.
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
