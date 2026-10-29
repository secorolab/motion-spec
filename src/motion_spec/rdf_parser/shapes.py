# SPDX-License-Identifier: MPL-2.0
"""Scene-authored geometry shared by the viewer and generated MuJoCo startup."""

from __future__ import annotations

import json
import math
from collections import Counter
from hashlib import sha256
from pathlib import Path
from urllib.parse import unquote, urljoin, urlparse

import rdflib
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_X,
    URI_GEOM_PRED_Y,
    URI_GEOM_PRED_Z,
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_TYPE_JOINT,
    URI_QUDT_PRED_UNIT,
    URI_QUDT_UNIT_CM,
    URI_QUDT_UNIT_M,
    URI_QUDT_UNIT_MM,
)
from rdflib.namespace import RDF, XSD
from scene_dsl.rdf.ktree import COL, PROV, PROV_EXT, SHAPES
from scene_dsl.rdf_parser.kinematics import body_of_frame

from motion_spec.classes.scene import MjcfSceneSpec, MjcfShapeBody, MjcfShapeGeom
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.model import identifier, local_name, si


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


def _scale(graph, shape) -> list[float]:
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
    if (shape, RDF.type, SHAPES.ScaledShape) not in graph:
        if any(graph.value(shape, predicate) is not None for predicate in predicates):
            raise ConstraintViolation("scene", f"Shape '{shape}' needs ScaledShape type for its factors.")
        if primitive:
            raise ConstraintViolation("scene", f"Primitive shape '{shape}' needs scale.")
        return [1.0] * 3
    result = []
    for predicate in predicates:
        value = _one(graph, shape, predicate, f"Shape scale '{shape}'")
        if not isinstance(value, rdflib.Literal) or value.datatype != XSD.double:
            raise ConstraintViolation(
                "scene", f"Shape '{shape}' needs xsd:double scale values; re-export the scene."
            )
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise ConstraintViolation("scene", f"Shape '{shape}' needs numeric scale factors.") from error
        if not math.isfinite(number) or number <= 0:
            raise ConstraintViolation("scene", f"Shape '{shape}' needs positive finite scale.")
        result.append(number)
    return result


def _half_scale(scale: float) -> float:
    return 0.5 * scale


def _unit_scale(graph, shape) -> float:
    unit = _one(graph, shape, URI_QUDT_PRED_UNIT, f"Shape '{shape}'")
    if unit not in (URI_QUDT_UNIT_M, URI_QUDT_UNIT_CM, URI_QUDT_UNIT_MM):
        raise ConstraintViolation("scene", f"Shape '{shape}' needs a supported length unit (m, cm, or mm).")
    return si(1.0, unit)


def _stl_path(graph, shape) -> str:
    source = _one(graph, shape, PROV.wasDerivedFrom, f"File shape '{shape}'")
    location = _one(graph, source, PROV.atLocation, f"File shape source '{source}'")
    if (location, RDF.type, PROV_EXT.RelativeLocation) in graph:
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
    else:
        ref = urlparse(str(location))
        if (
            ref.scheme != "file" or ref.netloc not in ("", "localhost")
            or not ref.path.startswith("/") or ref.params or ref.query or ref.fragment
        ):
            raise ConstraintViolation("scene", f"File shape '{shape}' needs a local absolute file location.")
        path = Path(unquote(ref.path))
    if path.suffix.lower() != ".stl" or not path.is_file():
        raise ConstraintViolation("scene", f"File shape '{shape}' STL asset was not found: {path}.")
    return str(path.resolve())


def read_shape_bodies(model, scene: MjcfSceneSpec) -> list[MjcfShapeBody]:
    graph = model.graph
    anchor = quantities.anchor_frame(model)
    anchor_body = quantities.body_of(model, anchor)
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
        # The scene anchor is stationary even when a joint names its body.
        if body in joined and body != anchor_body and str(body) not in mapped:
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
        unit_scale = _unit_scale(graph, shape)
        sizes = [factor * unit_scale for factor in _scale(graph, shape)]
        if (shape, RDF.type, SHAPES.UnitSphere) in graph:
            uniform = sizes[0] == sizes[1] == sizes[2]
            geom = MjcfShapeGeom(
                **common, kind="sphere", size=sizes[:1] if uniform else sizes,
                generated_mesh=not uniform,
            )
        elif (shape, RDF.type, SHAPES.UnitCube) in graph:
            sizes = [_half_scale(size) for size in sizes]
            geom = MjcfShapeGeom(
                **common, kind="box", size=sizes,
            )
        elif (shape, RDF.type, SHAPES.UnitCylinder) in graph:
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
                **common, kind="mesh", size=sizes,
                path=path, cpp_path_literal=json.dumps(path, ensure_ascii=False),
                has_path=True,
            )
        else:
            raise ConstraintViolation("scene", f"Shape '{shape}' has no supported type.")
        if any(not math.isfinite(size) or size <= 0 for size in geom.size):
            raise ConstraintViolation("scene", f"Shape '{shape}' has invalid dimensions after unit conversion.")
        bodies[key].geoms.append(geom)
    result = list(bodies.values())
    names = Counter([body.name for body in result] + [obj.body for obj in scene.objects])
    for body in result:
        if names[body.name] > 1:
            body.name = body.cpp_name
    return result
