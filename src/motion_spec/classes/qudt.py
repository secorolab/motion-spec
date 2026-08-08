# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""QUDT-typed scalar and free-vector quantities, and the provenance every one of them carries."""

from __future__ import annotations

from dataclasses import dataclass, field

from motion_spec.classes.base import INTERNAL


@dataclass
class QuantityKind:
    """A QUDT quantity kind."""

    id: str
    type: str = field(default="QuantityKind")


@dataclass
class Unit:
    """A QUDT unit."""

    id: str
    type: str = field(default="Unit")


@dataclass
class Provenance:
    """Origin of a quantity's value, shared by all quantity-like entities: whether it was
    authored by the user rather than computed. Whether it is a runtime snapshot capture is
    SnapshotCapture's fact, not this one's (F3).
    """

    authored: bool = False


@dataclass
class Quantity:
    """A scalar quantity with its kind, unit and optional value or view."""

    id: str
    quantity_kind: QuantityKind = field(metadata=INTERNAL)
    unit: Unit = field(metadata=INTERNAL)
    value: float | None
    has_view: bool = field(metadata=INTERNAL)
    provenance: Provenance = field(default_factory=Provenance)
    reference_value: str | None = None
    type: str = field(default="Quantity")


@dataclass
class FreeVector:
    """A free (un-anchored) vector quantity."""

    id: str
    quantity_kind: QuantityKind = field(metadata=INTERNAL)
    unit: Unit = field(metadata=INTERNAL)
    vector: list[float] | None
    has_view: bool = field(default=False, metadata=INTERNAL)
    provenance: Provenance = field(default_factory=Provenance)
    type: str = field(default="FreeVector")


@dataclass
class SetpointQuantity:
    """A setpoint-valued quantity produced by a path evaluator."""

    id: str
    quantity_kind: QuantityKind = field(metadata=INTERNAL)
    unit: Unit = field(metadata=INTERNAL)
    has_view: bool = field(metadata=INTERNAL)
    provenance: Provenance = field(default_factory=Provenance)
    value_kind: str | None = None
    type: str = field(default="SetpointQuantity")
