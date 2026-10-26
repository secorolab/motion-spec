# SPDX-License-Identifier: MPL-2.0
"""Triangle geometry for non-uniformly scaled primitives."""

from __future__ import annotations

import math
from pathlib import Path


def _sphere(steps=32, rings=16):
    def point(i, j):
        theta, phi = math.pi * i / rings, 2 * math.pi * j / steps
        return (math.sin(theta) * math.cos(phi), math.sin(theta) * math.sin(phi), math.cos(theta))

    triangles = []
    for i in range(rings):
        for j in range(steps):
            a, b, c, d = point(i, j), point(i + 1, j), point(i + 1, j + 1), point(i, j + 1)
            if i != rings - 1:
                triangles.append((a, b, c))
            if i != 0:
                triangles.append((a, c, d))
    return triangles


def _cylinder(steps=32):
    triangles = []
    for j in range(steps):
        phi, next_phi = 2 * math.pi * j / steps, 2 * math.pi * (j + 1) / steps
        a = (math.cos(phi), math.sin(phi), -1)
        b = (math.cos(next_phi), math.sin(next_phi), -1)
        c = (math.cos(next_phi), math.sin(next_phi), 1)
        d = (math.cos(phi), math.sin(phi), 1)
        triangles.extend(((a, b, c), (a, c, d), ((0, 0, -1), b, a), ((0, 0, 1), d, c)))
    return triangles


def base_triangles(kind: str):
    if kind == "sphere":
        return _sphere()
    if kind == "cylinder":
        return _cylinder()
    raise ValueError(f"Unsupported scaled shape kind: {kind}")


def write_obj(path: Path, triangles, size=(1, 1, 1)) -> None:
    if len(size) != 3 or any(not math.isfinite(v) or v <= 0 for v in size):
        raise ValueError("Scaled shape needs three positive finite size factors")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as output:
        count = 0
        for triangle in triangles:
            for vertex in triangle:
                point = [factor * value for factor, value in zip(size, vertex)]
                if any(not math.isfinite(v) for v in point):
                    raise ValueError("Scaled shape vertex is not finite")
                output.write("v " + " ".join(format(float(value), ".17g") for value in point) + "\n")
            output.write(f"f {count + 1} {count + 2} {count + 3}\n")
            count += 3
