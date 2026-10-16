# SPDX-License-Identifier: MPL-2.0
"""Scene-authored geometry shared by the viewer and generated MuJoCo startup."""

from __future__ import annotations

import json
import math
import random
from collections import Counter
from hashlib import sha256
from pathlib import Path
from urllib.parse import unquote, urlparse

import rdflib
from motion_spec_dsl.rdf_parser.vocab import QUDT_SCHEMA
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.distribution import DistributionModel
from rdf_utils.models.geom_coord import to_metres
from rdf_utils.models.vocab import (
    URI_DISTRIB_PRED_FROM_DISTRIB,
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_TYPE_JOINT,
    URI_QUDT_PRED_UNIT,
    URI_QUDT_PRED_VALUE,
)
from rdflib.namespace import RDF
from scene_dsl.rdf.ktree import ASTRO, COL, SHAPES
from scene_dsl.rdf_parser.kinematics import body_of_frame

from motion_spec.classes.scene import MjcfSceneSpec, MjcfShapeBody, MjcfShapeGeom, ShapeSize
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.model import identifier, local_name, si
from motion_spec.rdf_parser.sampling import _components, shape_sample_id


def _one(graph, subject, predicate, claim: str):
    values = list(graph.objects(subject, predicate))
    if len(values) != 1:
        raise ConstraintViolation("scene", f"{claim} needs exactly one {local_name(predicate)}.")
    return values[0]


def _color(graph, shape) -> list[float]:
    colors = list(graph.objects(shape, COL["has-color"]))
    if not colors:
        return [0.35, 0.65, 0.95, 1.0]
    if len(colors) != 1 or (colors[0], RDF.type, COL.RGBAScheme) not in graph:
        raise ConstraintViolation("scene", f"Shape '{shape}' needs one RGBA color.")
    value = str(_one(graph, colors[0], COL.value, f"Shape '{shape}'"))
    try:
        rgba = (
            [int(value[i : i + 2], 16) / 255 for i in (1, 3, 5, 7)]
            if value.startswith("#") and len(value) == 9
            else [float(channel) for channel in value.split()]
        )
    except ValueError as error:
        raise ConstraintViolation("scene", f"Shape '{shape}' has invalid RGBA color.") from error
    if len(rgba) != 4 or any(not math.isfinite(v) or not 0 <= v <= 1 for v in rgba):
        raise ConstraintViolation("scene", f"Shape '{shape}' needs four RGBA channels in [0, 1].")
    return rgba


def _size(graph, node, unit=None, index=0, factor=1.0) -> ShapeSize:
    if (node, RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY) in graph:
        return ShapeSize(
            draw=shape_sample_id(node), uri=str(node), index=index, factor=factor
        )
    value = float(_one(graph, node, URI_QUDT_PRED_VALUE, str(node)))
    metres = to_metres([value], unit, node)[0] if unit is not None else value
    if not math.isfinite(metres) or metres <= 0:
        raise ConstraintViolation("scene", f"Shape size '{node}' must be finite and positive.")
    return ShapeSize(value=metres * factor)


def _dimension(graph, shape, kind, index=0, factor=1.0) -> ShapeSize:
    nodes = [
        node for node in graph.objects(shape, SHAPES["has-dimension"])
        if (node, RDF.type, kind) in graph
    ]
    if len(nodes) != 1:
        raise ConstraintViolation("scene", f"Shape '{shape}' needs one {local_name(kind)}.")
    node = nodes[0]
    unit = _one(graph, node, URI_QUDT_PRED_UNIT, str(node))
    return _size(graph, node, unit, index, factor)


def _scale(graph, shape) -> list[ShapeSize]:
    sample = list(graph.objects(shape, SHAPES.scale))
    if sample:
        if len(sample) != 1 or (sample[0], RDF.type, SHAPES.ScaleXYZ) not in graph:
            raise ConstraintViolation("scene", f"Mesh '{shape}' needs one sampled scale.")
        return [_size(graph, sample[0], index=i) for i in range(3)]
    values = [list(graph.objects(shape, SHAPES[f"scale-{axis}"])) for axis in "xyz"]
    if not any(values):
        return [ShapeSize(value=1.0) for _ in range(3)]
    if any(len(value) != 1 for value in values):
        raise ConstraintViolation("scene", f"Mesh '{shape}' needs all three scale factors.")
    result = []
    for value in values:
        number = float(value[0])
        if not math.isfinite(number) or number <= 0:
            raise ConstraintViolation("scene", f"Mesh '{shape}' needs positive finite scale.")
        result.append(ShapeSize(value=number))
    return result


def _mesh_path(graph, shape) -> str:
    path = str(_one(graph, shape, SHAPES["source-path"], f"Mesh '{shape}'"))
    if Path(path).is_absolute():
        return path
    for _, _, _, source in graph.quads((shape, SHAPES["source-path"], None, None)):
        uri = urlparse(str(source))
        if uri.scheme == "file":
            candidate = Path(unquote(uri.path)).parent / path
            if candidate.is_file():
                return str(candidate.resolve())
    return path


def read_shape_bodies(model, scene: MjcfSceneSpec) -> list[MjcfShapeBody]:
    graph = model.graph
    anchor = quantities.anchor_frame(model)
    mapped = {obj.body_iri: obj.body for obj in scene.objects}
    joined = {
        body_of_frame(frame, graph)
        for joint in graph.subjects(RDF.type, URI_KC_TYPE_JOINT)
        for frame in graph.objects(joint, URI_KC_PRED_BETWEEN_ATTACHMENTS)
    }
    bodies: dict[rdflib.URIRef, MjcfShapeBody] = {}
    for shape in graph.subjects(SHAPES["attached-to-frame"], None):
        frame = _one(graph, shape, SHAPES["attached-to-frame"], f"Shape '{shape}'")
        body = body_of_frame(frame, graph)
        if body in joined and str(body) not in mapped:
            raise ConstraintViolation(
                "scene", f"Shape '{shape}' is on a joined body without a mapped scene object."
            )
        root = quantities.placement_frame(model, body)
        pos, quat = quantities.frame_placement(model, frame, root)
        if pos is None or quat is None:
            raise ConstraintViolation(
                "scene", f"Shape '{shape}' has no placement from its body's root frame."
            )
        if body not in bodies:
            if str(body) in mapped:
                name = mapped[str(body)] + "_shapes"
                attach_kind, attach_name = "Body", mapped[str(body)]
                body_pos, body_quat = [0.0] * 3, [0.0, 0.0, 0.0, 1.0]
            else:
                name = local_name(body)
                attach_kind, attach_name = "World", ""
                body_pos, body_quat = quantities.frame_placement(model, body, anchor)
                if body_pos is None or body_quat is None:
                    raise ConstraintViolation(
                        "scene", f"Shape body '{body}' has no placement from the scene anchor."
                    )
            bodies[body] = MjcfShapeBody(
                name, str(body), attach_kind, attach_name, body_pos, body_quat,
                cpp_name=f"{identifier(name)}_{sha256(str(body).encode()).hexdigest()[:8]}",
            )
        common = dict(
            name=local_name(shape),
            cpp_name=(
                f"{identifier(local_name(shape))}_{sha256(str(shape).encode()).hexdigest()[:8]}"
            ),
            pos=pos,
            quat_wxyz=[quat[3], *quat[:3]],
            rgba=_color(graph, shape),
        )
        if (shape, RDF.type, ASTRO.sphere) in graph:
            geom = MjcfShapeGeom(
                **common, kind="sphere", size=[_dimension(graph, shape, ASTRO.radius)]
            )
        elif (shape, RDF.type, ASTRO.cube) in graph:
            side = _dimension(graph, shape, ASTRO.length, factor=0.5)
            geom = MjcfShapeGeom(**common, kind="box", size=[side, side, side])
        elif (shape, RDF.type, ASTRO.rectangularPrism) in graph:
            if any(
                (node, RDF.type, SHAPES.DimensionsXYZ) in graph
                for node in graph.objects(shape, SHAPES["has-dimension"])
            ):
                size = [_dimension(graph, shape, SHAPES.DimensionsXYZ, i, 0.5) for i in range(3)]
            else:
                size = [
                    _dimension(graph, shape, SHAPES[f"{axis.upper()}Length"], factor=0.5)
                    for axis in "xyz"
                ]
            geom = MjcfShapeGeom(**common, kind="box", size=size)
        elif (shape, RDF.type, SHAPES.Cylinder) in graph:
            geom = MjcfShapeGeom(
                **common,
                kind="cylinder",
                size=[
                    _dimension(graph, shape, ASTRO.radius),
                    _dimension(graph, shape, ASTRO.height, factor=0.5),
                ],
            )
        elif (shape, RDF.type, SHAPES.MeshShape) in graph:
            format_ = str(_one(graph, shape, SHAPES["file-format"], f"Mesh '{shape}'"))
            if format_ != "stl":
                raise ConstraintViolation(
                    "scene", f"Mesh '{shape}' has unsupported format '{format_}'."
                )
            path = _mesh_path(graph, shape)
            geom = MjcfShapeGeom(
                **common, kind="mesh", size=_scale(graph, shape),
                path=path, cpp_path_literal=json.dumps(path, ensure_ascii=False),
                has_path=True,
            )
        else:
            raise ConstraintViolation("scene", f"Shape '{shape}' has no supported type.")
        bodies[body].geoms.append(geom)
    result = list(bodies.values())
    names = Counter([body.name for body in result] + [obj.body for obj in scene.objects])
    for body in result:
        if names[body.name] > 1:
            body.name = body.cpp_name
    return result


def draw_shape_sizes(
    model, bodies: list[MjcfShapeBody], seed: int | None = None
) -> dict[str, list[float]]:
    """One draw per sampled quantity, shared by each component that reads it."""
    rng = random.Random(seed)
    draws = {}
    for body in bodies:
        for geom in body.geoms:
            for size in geom.size:
                if not size.draw or size.uri in draws:
                    continue
                node = rdflib.URIRef(size.uri)
                distribution = model.graph.value(node, URI_DISTRIB_PRED_FROM_DISTRIB)
                kind, components = _components(DistributionModel(distribution, model.graph))
                if kind != "uniform_real" or any(
                    not math.isfinite(c["a"])
                    or not math.isfinite(c["b"])
                    or c["a"] <= 0
                    or c["b"] <= c["a"]
                    for c in components
                ):
                    raise ConstraintViolation(
                        "scene", f"Shape size '{node}' needs positive finite uniform bounds."
                    )
                unit = model.graph.value(node, QUDT_SCHEMA.unit)
                draws[size.uri] = [
                    si(rng.uniform(c["a"], c["b"]), unit) for c in components
                ]
    return draws


def size_value(size: ShapeSize, draws: dict[str, list[float]]) -> float:
    value = size.value if not size.draw else draws[size.uri][size.index] * size.factor
    if not math.isfinite(value) or value <= 0:
        raise ConstraintViolation("scene", "A sampled shape size must be finite and positive.")
    return value
