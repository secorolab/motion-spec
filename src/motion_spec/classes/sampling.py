# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A quantity the run draws at startup, stated as the distribution it comes from."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class SampledQuantity:
    """One draw: a scalar spec constant, or the position of a scene frame.

    `components` are the per-axis parameters of the C++ standard distribution `dist` names,
    `scale` takes the authored unit to SI. A scalar lands on `shared_member`; a position becomes
    the segment `segment` under `parent` of tree `tree`, with the authored `rotation`.
    """

    id: str
    uri: str
    distribution_uri: str
    dist: str
    components: list[dict]
    size: int
    scale: float
    shared_member: str | None = None
    segment: str | None = None
    parent: str | None = None
    rotation: list[float] | None = None
    tree: str | None = None
    type: str = field(default="SampledQuantity")
