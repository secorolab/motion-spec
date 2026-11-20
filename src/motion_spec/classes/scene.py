# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The MJCF scene the model runs in: robots, objects, attachments and the control timestep.

Renamed `Mjcf*`: these records carry MJCF asset paths, prefixes and scalar fan-out, and the
name should say so.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from motion_spec.classes.base import INTERNAL
from motion_spec.classes.bindings import CameraBinding


@dataclass
class MjcfSceneAttachment:
    """An asset attached to a robot or object in the scene."""

    id: str
    path: str
    attach_to: str
    attach_kind: str = "Body"
    prefix: str = ""
    pos: list[float] | None = None
    quat: list[float] | None = None
    actuator: str = ""
    pos_x: float = 0.0
    pos_y: float = 0.0
    pos_z: float = 0.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    quat_w: float = 1.0
    type: str = field(default="MjcfSceneAttachment")


@dataclass
class MjcfSceneRobot:
    """A robot placed in the scene, with its attachments."""

    id: str
    agent: str
    path: str
    prefix: str = ""
    attach_kind: str = "World"
    attach_name: str = ""
    pos: list[float] | None = None
    quat: list[float] | None = None
    attachments: list[MjcfSceneAttachment] = field(default_factory=list)
    pos_x: float = 0.0
    pos_y: float = 0.0
    pos_z: float = 0.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    quat_w: float = 1.0
    type: str = field(default="MjcfSceneRobot")


@dataclass
class MjcfSceneObject:
    """A scene object's placement and (procedural or asset) geometry."""

    id: str
    body: str
    path: str = ""
    attach_kind: str = "World"
    attach_name: str = ""
    pos: list[float] | None = None
    quat: list[float] | None = None
    draw: str | None = None
    draw_rotation: list[float] | None = None
    draw_base_pos: list[float] | None = None
    draw_base_quat: list[float] | None = None
    fixed: bool = False
    shape: str | None = None
    size: list[float] | None = None
    color: list[float] | None = None
    mass: float | None = None
    friction: list[float] | None = None
    pos_x: float = 0.0
    pos_y: float = 0.0
    pos_z: float = 0.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    quat_w: float = 1.0
    has_path: bool = False
    size_x: float | None = None
    size_y: float | None = None
    size_z: float | None = None
    color_r: float | None = None
    color_g: float | None = None
    color_b: float | None = None
    color_a: float | None = None
    friction_slide: float | None = None
    friction_torsion: float | None = None
    friction_roll: float | None = None
    # only the asset's root body is renamed to the object
    secondary_bodies: dict[str, str] = field(default_factory=dict, metadata=INTERNAL)
    body_iri: str = field(default="", metadata=INTERNAL)
    draw_uri: str = field(default="", metadata=INTERNAL)
    type: str = field(default="MjcfSceneObject")


@dataclass
class MjcfSceneFrame:
    """A frame the scene declares, marked on the body that carries it.

    The runtime draws and addresses these as sites, so what the model states about where
    things are is visible in the scene rather than only inside the placement arithmetic.
    """

    body: str
    name: str
    uri: str = field(default="", metadata=INTERNAL)
    pos_x: float = 0.0
    pos_y: float = 0.0
    pos_z: float = 0.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    quat_w: float = 1.0
    segment: str | None = None
    body_segment: str | None = None
    tree: str | None = None
    type: str = field(default="MjcfSceneFrame")


@dataclass
class MjcfSceneCamera:
    """A camera on a static scene frame, built into the composed scene on that frame's body."""

    name: str
    body: str
    fovy_deg: float
    pos_x: float = 0.0
    pos_y: float = 0.0
    pos_z: float = 0.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    quat_w: float = 1.0
    type: str = field(default="MjcfSceneCamera")


@dataclass
class MjcfSceneSpec:
    """The scene: robots, objects, cameras and the control timestep."""

    robots: list[MjcfSceneRobot] = field(default_factory=list)
    objects: list[MjcfSceneObject] = field(default_factory=list)
    cameras: list[CameraBinding] = field(default_factory=list)
    frames: list[MjcfSceneFrame] = field(default_factory=list)
    static_cameras: list[MjcfSceneCamera] = field(default_factory=list)
    # defaults to the backend interval when the model omits it
    timestep_s: float = 0.002
    type: str = field(default="MjcfSceneSpec")
