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
    # Where a scene frame or body sits on this chain: the index of the segment standing for it
    # and, for a frame, its constant pose on that segment. Keyed by IRI, filled from the scene.
    # Lowering resolves against these, so no name reaches the generated code to be searched for.
    frames: dict = field(default_factory=dict, metadata=INTERNAL)
    bodies: dict = field(default_factory=dict, metadata=INTERNAL)
    # The index the chain ends at, for generated code that means "the tip" without a frame.
    tip_segment: int = 0
    # The world model works in the built tree's own names, not in this chain's numbering: the
    # segment each chain joint moves, and the chain's endpoints as the whole tree names them.
    joint_segments: list[str] = field(default_factory=list)
    world_root: str = ""
    world_tip: str = ""
    # Every scene element this chain reaches, by IRI, named as the tree names it. Startup wiring
    # resolves an index from a name, so only lowering needs the whole lookup.
    world_segments: dict = field(default_factory=dict, metadata=INTERNAL)


@dataclass
class HardwareBinding:
    """The hardware asset a solver's chain is built from."""

    urdf: str = field(metadata=INTERNAL)
    model: str
    tool_body: str
    tcp_frame: str


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
    frame: str
    update_rate_hz: float | None
    # What `robot.toml` calls this sensor. The same key a bound device is configured under, so a
    # deployment property of the sensor -- its tare length -- is stated once for both platforms.
    config_key: str = ""
    observes: list[str] = field(default_factory=list)


@dataclass
class CameraBinding:
    """A camera declared in the scene: what it is called, what it renders, how often.

    Separate from `SensorBinding` because nothing a camera carries is a field a mounted sensor
    carries -- a resolution is not a tare length, and no solver reads a camera.
    """

    id: str
    width: int
    height: int
    rate_hz: float
    uri: str
    # The frame the camera reports in, named by the sensor the uri points at. A publisher stamps
    # its images with this, so it must name a frame in the ROS optical convention.
    frame_id: str = ""
    # Where a viewer reads this camera, as the model's subscription states it. A camera no
    # channel carries has no provider, and nothing may guess one from its name.
    topic: str | None = None
    message: str | None = None


@dataclass
class DeviceBinding:
    """Hardware bound on this chain: what it is, where it is configured, what it drives."""

    kind: str
    config_key: str
    drives: str = ""
    required_by_motion: list[int] = field(default_factory=list)
    has_required_motions: bool = False
    health_index: int | None = None
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
