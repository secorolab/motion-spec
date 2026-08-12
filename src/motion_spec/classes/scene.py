# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The MJCF scene the model runs in: robots, objects, attachments and the control timestep.

Renamed `Mjcf*`: these records carry MJCF asset paths, prefixes and scalar fan-out, and the
name should say so.
"""

from __future__ import annotations

from dataclasses import dataclass, field

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
    pos_x: float | None = None
    pos_y: float | None = None
    pos_z: float | None = None
    quat_x: float | None = None
    quat_y: float | None = None
    quat_z: float | None = None
    quat_w: float | None = None
    type: str = field(default="MjcfSceneAttachment")


@dataclass
class MjcfSceneRobot:
    """A robot placed in the scene, with its attachments."""

    id: str
    path: str
    prefix: str = ""
    attach_kind: str = "World"
    attach_name: str = ""
    pos: list[float] | None = None
    quat: list[float] | None = None
    attachments: list[MjcfSceneAttachment] = field(default_factory=list)
    pos_x: float | None = None
    pos_y: float | None = None
    pos_z: float | None = None
    quat_x: float | None = None
    quat_y: float | None = None
    quat_z: float | None = None
    quat_w: float | None = None
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
    fixed: bool = False
    shape: str | None = None
    size: list[float] | None = None
    color: list[float] | None = None
    mass: float | None = None
    friction: list[float] | None = None
    # Folded scalar expansions (pos/quat always; geometry only for non-path objects).
    pos_x: float | None = None
    pos_y: float | None = None
    pos_z: float | None = None
    quat_x: float | None = None
    quat_y: float | None = None
    quat_z: float | None = None
    quat_w: float | None = None
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
    type: str = field(default="MjcfSceneObject")


@dataclass
class MjcfSceneSpec:
    """The scene: robots, objects, cameras and the control timestep."""

    robots: list[MjcfSceneRobot] = field(default_factory=list)
    objects: list[MjcfSceneObject] = field(default_factory=list)
    # Cameras are rendered against the composed scene, not read by a solver, so they belong here
    # and not on the agent that hosts them.
    cameras: list[CameraBinding] = field(default_factory=list)
    # Physics/control timestep from ENVIRONMENT.timestep; defaults to the backend
    # interval when the model omits it.
    timestep_s: float = 0.002
    # Ground height in the world frame: the lowest the scene places anything against the
    # world, so a world frame anchored above the ground still gets its floor.
    floor_z: float = 0.0
    type: str = field(default="MjcfSceneSpec")
