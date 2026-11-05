# SPDX-License-Identifier: MPL-2.0
"""Scene-authored geometry shared by the viewer and generated MuJoCo startup."""

from __future__ import annotations

import colorsys
import json
import math
from collections import Counter
from hashlib import sha256

import rdflib
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.prov import get_file_path
from rdf_utils.models.vocab import (
    URI_COLOR_PRED_HAS_COLOR,
    URI_COLOR_PRED_VALUE,
    URI_GEOM_PRED_X,
    URI_GEOM_PRED_Y,
    URI_GEOM_PRED_Z,
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_TYPE_JOINT,
    URI_QUDT_PRED_UNIT,
    URI_QUDT_UNIT_CM,
    URI_QUDT_UNIT_M,
    URI_QUDT_UNIT_MM,
    URI_SHAPES_PRED_ATTACHED_TO_FRAME,
    URI_SHAPES_TYPE_FILE_BASED_SHAPE,
    URI_SHAPES_TYPE_SCALED_SHAPE,
)
from rdf_utils.namespace import NS_MM_COLOR, NS_MM_SHAPES
from rdflib.namespace import PROV, RDF, XSD
from scene_dsl.rdf_parser.kinematics import body_of_frame

from motion_spec.classes.scene import MjcfSceneSpec, MjcfShapeBody, MjcfShapeGeom
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.model import identifier, local_name, si


def _one(graph, subject, predicate, claim: str):
    values = list(graph.objects(subject, predicate))
    if len(values) != 1:
        raise ConstraintViolation("scene", f"{claim} needs exactly one {local_name(predicate)}.")
    return values[0]


# Channels in each scheme's value, as the color metamodel writes them.
_COLOR_CHANNELS = {NS_MM_COLOR.RGBScheme: 3, NS_MM_COLOR.RGBAScheme: 4, NS_MM_COLOR.HSVScheme: 3}


def _color(graph, shape) -> list[float]:
    """The shape's colour as MJCF rgba: RGB is taken as opaque, HSV converted."""
    colors = list(graph.objects(shape, URI_COLOR_PRED_HAS_COLOR))
    if not colors:
        return [0.35, 0.65, 0.95, 1.0]
    if len(colors) != 1:
        raise ConstraintViolation("scene", f"Shape '{shape}' needs at most one color.")
    [color] = colors
    schemes = [s for s in _COLOR_CHANNELS if (color, RDF.type, s) in graph]
    if len(schemes) != 1:
        raise ConstraintViolation(
            "scene", f"Shape '{shape}' color needs one scheme: RGB, RGBA, or HSV."
        )
    [scheme] = schemes
    name, count = local_name(scheme).removesuffix("Scheme"), _COLOR_CHANNELS[scheme]
    value = str(_one(graph, color, URI_COLOR_PRED_VALUE, f"Shape '{shape}'"))
    try:
        if value.startswith("#") and scheme != NS_MM_COLOR.HSVScheme and len(value) == 1 + 2 * count:
            channels = [int(value[i : i + 2], 16) / 255 for i in range(1, len(value), 2)]
        else:
            channels = [float(channel) for channel in value.split()]
    except ValueError as error:
        raise ConstraintViolation("scene", f"Shape '{shape}' has invalid {name} color.") from error
    if len(channels) != count or not all(math.isfinite(v) for v in channels):
        raise ConstraintViolation("scene", f"Shape '{shape}' needs {count} {name} channels.")
    if scheme == NS_MM_COLOR.HSVScheme:
        hue, saturation, brightness = channels
        if not 0 <= hue < 360:
            raise ConstraintViolation("scene", f"Shape '{shape}' needs HSV hue in [0, 360).")
        channels = [*colorsys.hsv_to_rgb(hue / 360, saturation, brightness)]
    if any(not 0 <= v <= 1 for v in channels):
        raise ConstraintViolation("scene", f"Shape '{shape}' needs {name} channels in [0, 1].")
    return channels if len(channels) == 4 else [*channels, 1.0]


def _scale(graph, shape) -> list[float]:
    predicates = (NS_MM_SHAPES["scale-x"], NS_MM_SHAPES["scale-y"], NS_MM_SHAPES["scale-z"])
    if (
        (shape, RDF.type, NS_MM_SHAPES.Scale) in graph
        or graph.value(shape, NS_MM_SHAPES["uniform-scale"]) is not None
        or any(
            graph.value(shape, predicate) is not None
            for predicate in (URI_GEOM_PRED_X, URI_GEOM_PRED_Y, URI_GEOM_PRED_Z)
        )
    ):
        raise ConstraintViolation("scene", f"Shape '{shape}' uses obsolete scale RDF; re-export the scene.")
    if (shape, RDF.type, URI_SHAPES_TYPE_SCALED_SHAPE) not in graph:
        if any(graph.value(shape, predicate) is not None for predicate in predicates):
            raise ConstraintViolation("scene", f"Shape '{shape}' needs ScaledShape type for its factors.")
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
    _one(graph, source, PROV.atLocation, f"File shape source '{source}'")
    # A path on this system, relative to a directory, or in a ROS or Python package.
    try:
        path = get_file_path(graph, source)
    except (ValueError, LookupError, ImportError) as error:
        raise ConstraintViolation("scene", f"File shape '{shape}' needs a local STL location: {error}") from error
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
    for shape in graph.subjects(URI_SHAPES_PRED_ATTACHED_TO_FRAME, None):
        if graph.value(shape, NS_MM_SHAPES["linear-transform"]) is not None:
            raise ConstraintViolation("scene", f"Shape '{shape}' uses unsupported linear-transform.")
        frame = _one(graph, shape, URI_SHAPES_PRED_ATTACHED_TO_FRAME, f"Shape '{shape}'")
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
        if (shape, RDF.type, NS_MM_SHAPES.UnitSphere) in graph:
            uniform = sizes[0] == sizes[1] == sizes[2]
            geom = MjcfShapeGeom(
                **common, kind="sphere" if uniform else "ellipsoid",
                size=sizes[:1] if uniform else sizes,
            )
        elif (shape, RDF.type, NS_MM_SHAPES.UnitCube) in graph:
            sizes = [_half_scale(size) for size in sizes]
            geom = MjcfShapeGeom(
                **common, kind="box", size=sizes,
            )
        elif (shape, RDF.type, NS_MM_SHAPES.UnitCylinder) in graph:
            sizes[2] = _half_scale(sizes[2])
            circular = sizes[0] == sizes[1]
            geom = MjcfShapeGeom(
                **common, kind="cylinder",
                size=[sizes[0], sizes[2]] if circular else sizes,
                generated_mesh=not circular,
            )
        elif (shape, RDF.type, URI_SHAPES_TYPE_FILE_BASED_SHAPE) in graph:
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
