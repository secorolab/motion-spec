# SPDX-License-Identifier: MPL-2.0
"""Triangle geometry for shapes with a local linear transform."""

from __future__ import annotations

import math
import struct
from pathlib import Path

import numpy as np


def _box():
    vertices = [
        (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
        (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
    ]
    faces = [
        (0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4), (3, 7, 6), (3, 6, 2),
        (1, 2, 6), (1, 6, 5), (0, 4, 7), (0, 7, 3),
    ]
    return [tuple(vertices[index] for index in face) for face in faces]


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


def _stl(path: Path):
    data = path.read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if count and 84 + 50 * count <= len(data):
            triangles = []
            for offset in range(84, 84 + 50 * count, 50):
                values = struct.unpack_from("<9f", data, offset + 12)
                triangles.append((values[:3], values[3:6], values[6:9]))
            return triangles
    vertices = []
    for line in data.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0].lower() == b"vertex":
            vertices.append(tuple(float(value) for value in parts[1:]))
    if not vertices or len(vertices) % 3:
        raise ValueError(f"STL asset '{path}' has no complete triangles")
    return [tuple(vertices[i : i + 3]) for i in range(0, len(vertices), 3)]


def base_triangles(kind: str, source: Path | None = None):
    if kind == "box":
        return _box()
    if kind == "sphere":
        return _sphere()
    if kind == "cylinder":
        return _cylinder()
    if kind == "mesh" and source is not None:
        return _stl(source)
    raise ValueError(f"Unsupported transformed shape kind: {kind}")


def write_obj(path: Path, triangles, transform=None, size=(1, 1, 1)) -> None:
    matrix = np.eye(3) if transform is None else np.asarray(transform, dtype=float).reshape(3, 3)
    scale = np.asarray(size, dtype=float)
    if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0):
        raise ValueError("Transformed shape needs three positive finite size factors")
    reverse = np.linalg.det(matrix) < 0
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as output:
        count = 0
        for triangle in triangles:
            for vertex in triangle:
                point = matrix @ (scale * np.asarray(vertex, dtype=float))
                if not np.isfinite(point).all():
                    raise ValueError("Transformed shape vertex is not finite")
                output.write("v " + " ".join(format(float(value), ".17g") for value in point) + "\n")
            indexes = (count + 1, count + 3, count + 2) if reverse else (count + 1, count + 2, count + 3)
            output.write(f"f {indexes[0]} {indexes[1]} {indexes[2]}\n")
            count += 3
