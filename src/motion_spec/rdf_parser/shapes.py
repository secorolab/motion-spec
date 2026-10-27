# SPDX-License-Identifier: MPL-2.0
"""Scene-authored geometry shared by the viewer and generated MuJoCo startup."""

from __future__ import annotations

import json
import math
import random
from collections import Counter
from hashlib import sha256
from pathlib import Path
from urllib.parse import unquote, urljoin, urlparse

import rdflib
from motion_spec_dsl.rdf_parser.vocab import QUDT_SCHEMA
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.distribution import DistributionModel
from rdf_utils.models.vocab import (
    URI_DISTRIB_PRED_FROM_DISTRIB,
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_GEOM_PRED_X,
    URI_GEOM_PRED_Y,
    URI_GEOM_PRED_Z,
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_TYPE_JOINT,
)
from rdflib.namespace import RDF
from scene_dsl.rdf.ktree import COL, PROV, PROV_EXT, QKIND, SHAPES
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


def _scale(graph, shape) -> list[ShapeSize]:
    predicates = (SHAPES["scale-x"], SHAPES["scale-y"], SHAPES["scale-z"])
    if (
        (shape, RDF.type, SHAPES.Scale) in graph
        or graph.value(shape, SHAPES["uniform-scale"]) is not None
        or any(
            graph.value(shape, predicate) is not None
            for predicate in (URI_GEOM_PRED_X, URI_GEOM_PRED_Y, URI_GEOM_PRED_Z)
        )
    ):
        raise ConstraintViolation("scene", f"Shape '{shape}' uses obsolete scale RDF; re-export the scene.")
    primitive = any(
        (shape, RDF.type, kind) in graph
        for kind in (
            SHAPES.UnitCube, SHAPES.UnitCylinder, SHAPES.UnitSphere,
        )
    )
    distributions = list(graph.objects(shape, URI_DISTRIB_PRED_FROM_DISTRIB))
    if (shape, RDF.type, SHAPES.ScaledShape) not in graph:
        if distributions or any(
            graph.value(shape, predicate) is not None for predicate in predicates
        ):
            raise ConstraintViolation("scene", f"Shape '{shape}' needs ScaledShape type for its factors.")
        if primitive:
            raise ConstraintViolation("scene", f"Primitive shape '{shape}' needs scale.")
        return [ShapeSize(value=1.0) for _ in range(3)]
    if distributions:
        raise ConstraintViolation(
            "scene", f"Shape '{shape}' uses obsolete sampled scale RDF; re-export the scene."
        )
    result = []
    for predicate in predicates:
        factor = _one(graph, shape, predicate, f"Shape scale '{shape}'")
        if not isinstance(factor, rdflib.URIRef) or (factor, RDF.type, QUDT_SCHEMA.Quantity) not in graph:
            raise ConstraintViolation(
                "scene", f"Shape '{shape}' needs scale factor quantity nodes; re-export the scene."
            )
        if graph.value(factor, QUDT_SCHEMA.hasQuantityKind) != QKIND.Dimensionless:
            raise ConstraintViolation("scene", f"Shape scale '{factor}' must be dimensionless.")
        if (factor, RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY) in graph:
            distribution = _one(
                graph, factor, URI_DISTRIB_PRED_FROM_DISTRIB, f"Shape scale '{factor}'"
            )
            if graph.value(factor, QUDT_SCHEMA.value) is not None:
                raise ConstraintViolation("scene", f"Sampled scale '{factor}' cannot also have a fixed value.")
            kind, components = _components(DistributionModel(distribution, graph))
            if kind != "uniform_real" or len(components) != 1:
                raise ConstraintViolation("scene", f"Shape scale '{factor}' needs a scalar uniform distribution.")
            bounds = components[0]
            if not all(math.isfinite(bounds[key]) for key in ("a", "b")) or not (
                0 < bounds["a"] < bounds["b"]
            ):
                raise ConstraintViolation("scene", f"Shape scale '{factor}' needs positive finite bounds.")
            result.append(ShapeSize(draw=shape_sample_id(factor), uri=str(factor)))
            continue
        if graph.value(factor, URI_DISTRIB_PRED_FROM_DISTRIB) is not None:
            raise ConstraintViolation("scene", f"Sampled scale '{factor}' needs SampledQuantity type.")
        value = _one(graph, factor, QUDT_SCHEMA.value, f"Shape scale '{factor}'")
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise ConstraintViolation("scene", f"Shape '{shape}' needs numeric scale factors.") from error
        if not math.isfinite(number) or number <= 0:
            raise ConstraintViolation("scene", f"Shape '{shape}' needs positive finite scale.")
        result.append(ShapeSize(value=number))
    return result


def _half_scale(scale: ShapeSize) -> ShapeSize:
    if scale.draw:
        return ShapeSize(
            draw=scale.draw, uri=scale.uri, index=scale.index,
            factor=0.5 * scale.factor,
        )
    return ShapeSize(value=0.5 * scale.value)


def _stl_path(graph, shape) -> str:
    source = _one(graph, shape, PROV.wasDerivedFrom, f"File shape '{shape}'")
    location = _one(graph, source, PROV.atLocation, f"File shape source '{source}'")
    relative = str(_one(graph, location, PROV_EXT["rel-path"], f"File location '{location}'"))
    base = str(_one(graph, location, PROV.atLocation, f"File location '{location}'"))
    ref, root = urlparse(relative), urlparse(base)
    if (
        ref.scheme or ref.netloc or ref.path.startswith("/") or ref.query or ref.fragment
        or root.scheme != "file" or root.netloc not in ("", "localhost")
        or not root.path.endswith("/")
    ):
        raise ConstraintViolation("scene", f"File shape '{shape}' needs a relative STL path at a file directory.")
    path = Path(unquote(urlparse(urljoin(base, relative)).path))
    if path.suffix.lower() != ".stl" or not path.is_file():
        raise ConstraintViolation("scene", f"File shape '{shape}' STL asset was not found: {path}.")
    return str(path.resolve())


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
        if graph.value(shape, SHAPES["linear-transform"]) is not None:
            raise ConstraintViolation("scene", f"Shape '{shape}' uses unsupported linear-transform.")
        frame = _one(graph, shape, SHAPES["attached-to-frame"], f"Shape '{shape}'")
        body = quantities.body_of(model, frame)
        free = body is None
        key = frame if free else body
        if body in joined and str(body) not in mapped:
            raise ConstraintViolation(
                "scene", f"Shape '{shape}' is on a joined body without a mapped scene object."
            )
        root = anchor if free else quantities.placement_frame(model, body)
        pos, quat = quantities.frame_placement(model, frame, root)
        if pos is None or quat is None:
            raise ConstraintViolation(
                "scene", f"Shape '{shape}' has no placement from its frame to the scene anchor."
                if free else f"Shape '{shape}' has no placement from its body's root frame."
            )
        if key not in bodies:
            if free:
                name = f"{local_name(frame)}_shapes"
                attach_kind, attach_name = "World", ""
                body_pos, body_quat = [0.0] * 3, [0.0, 0.0, 0.0, 1.0]
            elif str(body) in mapped:
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
            bodies[key] = MjcfShapeBody(
                name, str(key), attach_kind, attach_name, body_pos, body_quat,
                cpp_name=f"{identifier(name)}_{sha256(str(key).encode()).hexdigest()[:8]}",
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
        if (shape, RDF.type, SHAPES.UnitSphere) in graph:
            sizes = _scale(graph, shape)
            uniform = sizes[0] == sizes[1] == sizes[2]
            geom = MjcfShapeGeom(
                **common, kind="sphere", size=sizes[:1] if uniform else sizes,
                generated_mesh=not uniform,
            )
        elif (shape, RDF.type, SHAPES.UnitCube) in graph:
            sizes = [_half_scale(scale) for scale in _scale(graph, shape)]
            geom = MjcfShapeGeom(
                **common, kind="box", size=sizes,
            )
        elif (shape, RDF.type, SHAPES.UnitCylinder) in graph:
            sizes = _scale(graph, shape)
            sizes[2] = _half_scale(sizes[2])
            circular = sizes[0] == sizes[1]
            geom = MjcfShapeGeom(
                **common, kind="cylinder",
                size=[sizes[0], sizes[2]] if circular else sizes,
                generated_mesh=not circular,
            )
        elif (shape, RDF.type, SHAPES.FileBasedShape) in graph:
            path = _stl_path(graph, shape)
            geom = MjcfShapeGeom(
                **common, kind="mesh", size=_scale(graph, shape),
                path=path, cpp_path_literal=json.dumps(path, ensure_ascii=False),
                has_path=True,
            )
        else:
            raise ConstraintViolation("scene", f"Shape '{shape}' has no supported type.")
        bodies[key].geoms.append(geom)
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
