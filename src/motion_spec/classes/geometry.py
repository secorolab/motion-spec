# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Spatial entities and the quantities derived between them: points, frames, poses, twists,
wrenches, and the scalar/axis views onto their subspaces.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from motion_spec.classes.base import INTERNAL
from motion_spec.classes.qudt import Provenance, Quantity, QuantityKind, Unit


class Subspace(str, Enum):
    """The linear (translational) or angular (rotational) half of a 6D spatial quantity.

    The physical quantity it belongs to is carried by the View's superobject type, so the
    subspace only names the half -- matching the C++ runtime Subspace enum.
    """

    Linear = "Linear"
    Angular = "Angular"


class Axis(str, Enum):
    """A Cartesian axis: X, Y or Z, plus a quaternion's scalar component W."""

    X = "X"
    Y = "Y"
    Z = "Z"
    W = "W"


@dataclass
class Point:
    """A named point, such as a frame origin."""

    id: str
    uri: str = field(default="", metadata=INTERNAL)
    segment: int | None = None
    offset: dict | None = None
    type: str = field(default="Point")


@dataclass
class Frame:
    """A named reference frame."""

    id: str
    uri: str = field(default="", metadata=INTERNAL)
    segment: int | None = None
    offset: dict | None = None
    type: str = field(default="Frame")


@dataclass
class SimplicialComplex:
    """A geometric body."""

    id: str
    uri: str = field(default="", metadata=INTERNAL)
    segment: int | None = None
    type: str = field(default="SimplicialComplex")


@dataclass
class Direction:
    """A unit-direction quantity."""

    id: str
    quantity_kind: list[QuantityKind] = field(metadata=INTERNAL)
    as_seen_by: Frame
    unit: list[Unit] = field(metadata=INTERNAL)
    direction: list[float] | None
    type: str = field(default="Direction")


@dataclass
class Position:
    """A position quantity of a point with respect to another."""

    id: str
    of: Point | None
    with_respect_to: Point | None = field(metadata=INTERNAL)
    quantity_kind: QuantityKind = field(metadata=INTERNAL)
    as_seen_by: Frame
    unit: Unit = field(metadata=INTERNAL)
    position: list[float] | None
    type: str = field(default="Position")


@dataclass
class Orientation:
    """An orientation quantity of a frame/object with respect to another."""

    id: str
    of: Frame | None
    with_respect_to: Frame | None = field(metadata=INTERNAL)
    quantity_kind: QuantityKind = field(metadata=INTERNAL)
    as_seen_by: Frame | None
    unit: Unit = field(metadata=INTERNAL)
    euler_axes_sequence: str | None = field(default=None, metadata=INTERNAL)
    has_view: bool = field(default=False, metadata=INTERNAL)
    provenance: Provenance = field(default_factory=Provenance)
    type: str = field(default="Orientation")


@dataclass
class Pose:
    """A pose (position and orientation) quantity."""

    id: str
    of: SimplicialComplex | Frame | None
    with_respect_to: SimplicialComplex | Frame | None = field(metadata=INTERNAL)
    quantity_kind: list[QuantityKind] = field(metadata=INTERNAL)
    as_seen_by: Frame | None
    unit: list[str] = field(metadata=INTERNAL)
    position: list[float] | None
    euler_axes_sequence: str | None = field(default=None, metadata=INTERNAL)
    euler_intrinsic: bool = field(default=False, metadata=INTERNAL)
    orientation_representation: str = field(default="quaternion", metadata=INTERNAL)
    orientation_operands: list | None = None
    relative_to_frame: Frame | None = None
    seen_by_root: bool = False
    provenance: Provenance = field(default_factory=Provenance)
    type: str = field(default="Pose")


@dataclass(kw_only=True)
class SpatialCoordinate:
    """Shared shape of the four quantities derived per-view off a spatial pair: velocity,
    acceleration, pose-difference and wrench. Each subclass adds only what is genuinely its
    own (`kw_only` so a required subclass field can follow `provenance`'s default).
    """

    id: str
    quantity_kind: list[QuantityKind] = field(metadata=INTERNAL)
    reference_point: Point
    as_seen_by: Frame
    unit: list[str] = field(metadata=INTERNAL)
    provenance: Provenance = field(default_factory=Provenance)


@dataclass(kw_only=True)
class VelocityTwist(SpatialCoordinate):
    """A velocity-twist quantity."""

    of: SimplicialComplex
    with_respect_to: SimplicialComplex = field(metadata=INTERNAL)
    # chain FK answers in the root frame; elsewhere it is rotated
    seen_by_root: bool = True
    type: str = field(default="VelocityTwist")


@dataclass(kw_only=True)
class AccelerationTwist(SpatialCoordinate):
    """An acceleration-twist quantity."""

    type: str = field(default="AccelerationTwist")


@dataclass(kw_only=True)
class PoseDifference(SpatialCoordinate):
    """A pose-difference quantity."""

    type: str = field(default="PoseDifference")


@dataclass(frozen=True)
class WrenchEstimator:
    """The momentum observer an estimated wrench comes from, on the chain of one agent."""

    agent: str
    estimation_gain_hz: float
    filter_constant: float


@dataclass(kw_only=True)
class Wrench(SpatialCoordinate):
    """A wrench (force/torque) quantity, optionally read from a force/torque sensor."""

    estimator: WrenchEstimator | None = None
    sensor_frame: Frame | None = None
    # empty for computed or commanded wrenches
    sensor_name: str = ""
    retare_event_uris: tuple[str, ...] = ()
    retare_events: tuple[str, ...] = ()
    type: str = field(default="Wrench")


@dataclass
class View:
    """A scalar/axis view onto one subspace of a spatial superobject."""

    id: str
    superobject: Pose | VelocityTwist | AccelerationTwist | PoseDifference | Wrench | Direction
    subobject: Quantity = field(metadata=INTERNAL)
    # None for a 3-vector superobject
    subspace: Subspace | None
    axis: Axis | None
    direction: Direction | None = None
    type: str = field(default="View")
