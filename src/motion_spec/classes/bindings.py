# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What a serial-chain solver runs on: the chain, the hardware bound to it, the runtime slot
it occupies, and what the scene mounts on it -- split out of the flat solver record so each
fact is named once, by the concept that owns it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from rdflib import URIRef

from motion_spec.classes.base import INTERNAL


@dataclass
class ChainBinding:
    """The chain a solver runs on, as the scene declares it."""

    root: str
    end: str
    tip: str
    # identifier-safe: ST4 cannot sanitize
    tree: str
    namespace: str
    name: str
    joints: list[str]
    frames: dict = field(default_factory=dict, metadata=INTERNAL)
    bodies: dict = field(default_factory=dict, metadata=INTERNAL)
    tip_segment: int = 0
    joint_segments: list[str] = field(default_factory=list)
    world_root: str = ""
    world_tip: str = ""
    tree_root: str = ""
    world_segments: dict = field(default_factory=dict, metadata=INTERNAL)


@dataclass
class HardwareBinding:
    """The hardware asset a solver's chain is built from."""

    urdf: str = field(metadata=INTERNAL)
    model: str
    tool_body: str
    tcp_frame: str


@dataclass
class WorldPort:
    """One port of the world model's table: a physical variable, its provider mapping, its slot.

    `kind` is the variable; `segment` the world-model segment it moves or places, empty for a
    body-only port; `mapping` the name the backend resolves its provider by.
    """

    kind: str
    segment: str
    mapping: str
    slot: int
    owner_id: str = ""
    iri: str = field(default="", metadata=INTERNAL)
    type: str = field(default="WorldPort")


@dataclass
class RuntimeBinding:
    """Which runtime this solver's chain is driven by, and the deployment slot it fills."""

    id: str
    owner: bool
    prefix: str
    owned_trees: list = field(metadata=INTERNAL)
    config_key: str
    owner_id: str = ""
    command_mode: str | None = "torque"


@dataclass
class SensorBinding:
    """A sensor mounted on this chain, as the runtime names it."""

    id: str
    type: str
    frame: str
    local_id: str
    local_frame: str
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
    # must be a ROS optical frame
    frame_id: str = ""
    frame_uri: str = ""
    fovy_deg: float = 45.0
    # None without a channel; never guessed from the name
    topic: str | None = None
    message: str | None = None


@dataclass
class DeviceBinding:
    """Hardware bound on this chain: what it is, where it is configured, what it drives."""

    kind: str
    config_key: str
    drives: str = ""
    required_by_motion: list[int] = field(default_factory=list)
    health_index: int | None = None
    joint_outputs: list = field(default_factory=list)


@dataclass(frozen=True)
class JointSpaceChannel:
    """One joint-space signal a runtime mirrors into the frame log.

    Declared in one place so the writer and the template's mirror expression cannot drift. All
    chain joints are revolute, so a position is an angle. `backends` of None means every backend
    carries the signal; naming backends restricts it to those that actually measure it.
    """

    name: str
    writer: str
    quantity_kind: URIRef
    unit: URIRef
    backends: tuple | None = None
