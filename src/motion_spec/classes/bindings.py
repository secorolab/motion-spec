# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What a serial-chain solver runs on: the chain, the hardware bound to it, the runtime slot
it occupies, and what the scene mounts on it -- split out of the flat solver record so each
fact is named once, by the concept that owns it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from motion_spec.classes.base import INTERNAL


@dataclass
class ChainBinding:
    """The chain a solver runs on, as the scene declares it."""

    root: str
    end: str
    tip: str
    # Identifier-safe (sanitized in Python; ST4 cannot sanitize). `tree` names the kinematic
    # tree the chain is sliced from; `name` is the chain's own qualified name.
    tree: str
    name: str
    # Ordered revolute joint local names, unprefixed; the runtime prefix is `runtime.prefix`.
    joints: list[str]


@dataclass
class HardwareBinding:
    """The hardware asset a solver's chain is built from."""

    urdf: str = field(metadata=INTERNAL)
    model: str
    tool_body: str
    tcp_site: str


@dataclass
class RuntimeBinding:
    """Which runtime this solver's chain is driven by, and the deployment slot it fills."""

    id: str
    owner: bool
    # Scopes every scene name this runtime owns; backends and log channels apply it themselves.
    prefix: str
    owned_trees: list = field(metadata=INTERNAL)
    # Section name in the deployment config; empty under simulation.
    config_key: str


@dataclass
class SensorBinding:
    """A sensor mounted on this chain, as the runtime names it."""

    id: str
    type: str
    frame_site: str
    update_rate_hz: float | None
    observes: list[str] = field(default_factory=list)


@dataclass
class DeviceBinding:
    """Hardware bound on this chain: what it is, where it is configured, what it drives."""

    kind: str
    config_key: str
    drives: str = ""
    # kc-ext:JointCoupling mimic joints this device reports instead of the chain (was the
    # solver's own flat `gripper_joint_outputs`).
    joint_outputs: list = field(default_factory=list)


@dataclass(frozen=True)
class JointSpaceChannel:
    """One joint-space signal a runtime mirrors into the frame log.

    Declared in one place so the producer and the template's mirror expression cannot drift. All
    chain joints are revolute, so a position is an angle. `backends` of None means every backend
    carries the signal; naming backends restricts it to those that actually measure it.
    """

    name: str
    producer: str
    quantity_kind: str
    unit: str
    backends: tuple | None = None
