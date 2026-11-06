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

from motion_spec.classes.scene import MjcfSceneShape, MjcfSceneSpec
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.model import identifier, local_name, si


def _one(graph, subject, predicate, claim: str):
    values = list(graph.objects(subject, predicate))
    if len(values) != 1:
        raise ConstraintViolation("scene", f"{claim} needs exactly one {local_name(predicate)}.")
    return values[0]


# Channels in each scheme's value, as the color metamodel writes them.
_COLOR_CHANNELS = {NS_MM_COLOR.RGBScheme: 3, NS_MM_COLOR.RGBAScheme: 4, NS_MM_COLOR.HSVScheme: 3}


def _color(graph, shape) -> list[float] | None:
    """The shape's colour as MJCF rgba: RGB is taken as opaque, HSV converted. None when the
    scene states no colour, which leaves MuJoCo's default."""
    colors = list(graph.objects(shape, URI_COLOR_PRED_HAS_COLOR))
    if not colors:
        return None
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


def _compose(outer_pos, outer_quat, inner_pos, inner_quat) -> tuple[list[float], list[float]]:
    """INNER, posed in a frame that OUTER poses, as one pose: metres and [x, y, z, w]."""
    ox, oy, oz, ow = outer_quat
    ix, iy, iz, iw = inner_quat
    quat = [
        ow * ix + ox * iw + oy * iz - oz * iy,
        ow * iy - ox * iz + oy * iw + oz * ix,
        ow * iz + ox * iy - oy * ix + oz * iw,
        ow * iw - ox * ix - oy * iy - oz * iz,
    ]
    # outer_quat rotates inner_pos: v + 2w(u x v) + 2u x (u x v), u the vector part.
    u, v = (ox, oy, oz), inner_pos
    t = [2 * (u[1] * v[2] - u[2] * v[1]), 2 * (u[2] * v[0] - u[0] * v[2]),
         2 * (u[0] * v[1] - u[1] * v[0])]
    rotated = [
        v[0] + ow * t[0] + (u[1] * t[2] - u[2] * t[1]),
        v[1] + ow * t[1] + (u[2] * t[0] - u[0] * t[2]),
        v[2] + ow * t[2] + (u[0] * t[1] - u[1] * t[0]),
    ]
    return [o + r for o, r in zip(outer_pos, rotated)], quat


def _mapped_bodies(scene: MjcfSceneSpec) -> dict[str, str]:
    return {obj.body_iri: obj.body for obj in scene.objects}


def world_bodies(model, objects) -> set[rdflib.URIRef]:
    """The bodies that stand in the world: they carry shapes and no scene object maps them.

    Nothing else builds such a body, and `read_shapes` only takes one that is static, so its
    shapes and frames go on MuJoCo's world body at their pose from the scene anchor.
    """
    graph = model.graph
    mapped = {obj.body_iri for obj in objects}
    return {
        body
        for frame in graph.objects(None, URI_SHAPES_PRED_ATTACHED_TO_FRAME)
        if (body := quantities.body_of(model, frame)) is not None and str(body) not in mapped
    }


def anchor_placement(model, frame, body):
    """Where FRAME on a world body sits from the scene anchor: its body's placement, then its
    own on the body, or (None, None) when either is missing."""
    anchor = quantities.anchor_frame(model)
    body_pos, body_quat = quantities.frame_placement(model, body, anchor)
    pos, quat = quantities.frame_placement(model, frame, quantities.placement_frame(model, body))
    if body_pos is None or body_quat is None or pos is None or quat is None:
        return None, None
    return _compose(body_pos, body_quat, pos, quat)


def read_shapes(model, scene: MjcfSceneSpec) -> list[MjcfSceneShape]:
    """Each scene-authored shape, as one fixed SceneObject on its mapped object or the world."""
    graph = model.graph
    anchor = quantities.anchor_frame(model)
    anchor_body = quantities.body_of(model, anchor)
    mapped = _mapped_bodies(scene)
    joined = {
        body_of_frame(frame, graph)
        for joint in graph.subjects(RDF.type, URI_KC_TYPE_JOINT)
        for frame in graph.objects(joint, URI_KC_PRED_BETWEEN_ATTACHMENTS)
    }
    shapes: list[MjcfSceneShape] = []
    for shape in graph.subjects(URI_SHAPES_PRED_ATTACHED_TO_FRAME, None):
        if graph.value(shape, NS_MM_SHAPES["linear-transform"]) is not None:
            raise ConstraintViolation("scene", f"Shape '{shape}' uses unsupported linear-transform.")
        frame = _one(graph, shape, URI_SHAPES_PRED_ATTACHED_TO_FRAME, f"Shape '{shape}'")
        body = quantities.body_of(model, frame)
        # The scene anchor is stationary even when a joint names its body.
        if body in joined and body != anchor_body and str(body) not in mapped:
            raise ConstraintViolation(
                "scene", f"Shape '{shape}' is on a joined body without a mapped scene object."
            )
        if body is None:
            # On a frame no body carries: placed from the scene anchor, on the world.
            attach_kind, attach_name = "World", ""
            pos, quat = quantities.frame_placement(model, frame, anchor)
            missing = f"Shape '{shape}' has no placement from its frame to the scene anchor."
        elif str(body) in mapped:
            # Rides the object's body, posed from the frame that body is built on.
            attach_kind, attach_name = "Body", mapped[str(body)]
            pos, quat = quantities.frame_placement(
                model, frame, quantities.placement_frame(model, body)
            )
            missing = f"Shape '{shape}' has no placement from its body's root frame."
        else:
            # On a static body nothing else builds: it stands in the world.
            attach_kind, attach_name = "World", ""
            pos, quat = anchor_placement(model, frame, body)
            missing = f"Shape '{shape}' has no placement from its body to the scene anchor."
        if pos is None or quat is None:
            raise ConstraintViolation("scene", missing)

        common = {
            "name": local_name(shape),
            "iri": str(shape),
            "attach_kind": attach_kind,
            "attach_name": attach_name,
            "pos": pos,
            "quat": quat,
        }
        rgba = _color(graph, shape)
        if rgba is not None:
            common.update(rgba=rgba, has_rgba=True)
        unit_scale = _unit_scale(graph, shape)
        sizes = [factor * unit_scale for factor in _scale(graph, shape)]
        if (shape, RDF.type, NS_MM_SHAPES.UnitSphere) in graph:
            uniform = sizes[0] == sizes[1] == sizes[2]
            entry = MjcfSceneShape(
                **common, shape="SPHERE" if uniform else "ELLIPSOID",
                size=[sizes[0], 0.0, 0.0] if uniform else sizes,
            )
        elif (shape, RDF.type, NS_MM_SHAPES.UnitCube) in graph:
            # MuJoCo sizes a box by its half extents.
            entry = MjcfSceneShape(**common, shape="BOX", size=[size / 2 for size in sizes])
        elif (shape, RDF.type, NS_MM_SHAPES.UnitCylinder) in graph:
            if sizes[0] != sizes[1]:
                raise ConstraintViolation(
                    "scene",
                    f"Shape '{shape}' is an elliptical cylinder (X scale != Y scale); MuJoCo's "
                    "cylinder is circular.",
                )
            # MuJoCo sizes a cylinder by its radius and half height.
            entry = MjcfSceneShape(**common, shape="CYLINDER", size=[sizes[0], sizes[2] / 2, 0.0])
        elif (shape, RDF.type, URI_SHAPES_TYPE_FILE_BASED_SHAPE) in graph:
            path = _stl_path(graph, shape)
            entry = MjcfSceneShape(
                **common, shape="MESH", size=sizes,
                path=path, cpp_path_literal=json.dumps(path, ensure_ascii=False), has_path=True,
            )
        else:
            raise ConstraintViolation("scene", f"Shape '{shape}' has no supported type.")
        used = {"SPHERE": 1, "CYLINDER": 2}.get(entry.shape, 3)
        if any(not math.isfinite(size) or size <= 0 for size in entry.size[:used]):
            raise ConstraintViolation(
                "scene", f"Shape '{shape}' has invalid dimensions after unit conversion."
            )
        shapes.append(entry)
    # Each shape is a body of its own: its name must not repeat another shape's or an object's.
    names = Counter([entry.name for entry in shapes] + [obj.body for obj in scene.objects])
    for entry in shapes:
        if names[entry.name] > 1:
            entry.name = f"{identifier(entry.name)}_{sha256(entry.iri.encode()).hexdigest()[:8]}"
    return shapes
