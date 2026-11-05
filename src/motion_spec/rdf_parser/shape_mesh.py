# SPDX-License-Identifier: MPL-2.0
"""A unit cylinder mesh for elliptical cylinders: MuJoCo's cylinder primitive is circular."""

from __future__ import annotations

import math
from pathlib import Path

UNIT_CYLINDER_OBJ = "unit_cylinder.obj"


def write_unit_cylinder(path: Path, steps: int = 32) -> None:
    """Write a cylinder of radius 1 spanning Z in [-1, 1] as OBJ.

    MuJoCo's mesh `scale` sizes it with a geom's (X radius, Y radius, half height).
    """
    triangles = []
    for j in range(steps):
        phi, next_phi = 2 * math.pi * j / steps, 2 * math.pi * (j + 1) / steps
        a = (math.cos(phi), math.sin(phi), -1)
        b = (math.cos(next_phi), math.sin(next_phi), -1)
        c = (math.cos(next_phi), math.sin(next_phi), 1)
        d = (math.cos(phi), math.sin(phi), 1)
        triangles.extend(((a, b, c), (a, c, d), ((0, 0, -1), b, a), ((0, 0, 1), d, c)))

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as output:
        for index, triangle in enumerate(triangles):
            for vertex in triangle:
                output.write("v " + " ".join(format(float(v), ".17g") for v in vertex) + "\n")
            output.write(f"f {3 * index + 1} {3 * index + 2} {3 * index + 3}\n")
