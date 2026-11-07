# SPDX-License-Identifier: MPL-2.0
"""Scene-authored geometry shared by the viewer and generated MuJoCo startup."""

from __future__ import annotations

import colorsys
import math

import rdflib
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.prov import get_file_path
from rdf_utils.models.vocab import (
    URI_COLOR_PRED_HAS_COLOR,
    URI_COLOR_PRED_VALUE,
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
from motion_spec.rdf_parser.model import local_name, si

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
    value = str(graph.value(color, URI_COLOR_PRED_VALUE))
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
    if (shape, RDF.type, URI_SHAPES_TYPE_SCALED_SHAPE) not in graph:
        if any(graph.value(shape, predicate) is not None for predicate in predicates):
            raise ConstraintViolation("scene", f"Shape '{shape}' needs ScaledShape type for its factors.")
        return [1.0] * 3
    result = []
    for predicate in predicates:
        value = graph.value(shape, predicate)
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
    unit = graph.value(shape, URI_QUDT_PRED_UNIT)
    if unit not in (URI_QUDT_UNIT_M, URI_QUDT_UNIT_CM, URI_QUDT_UNIT_MM):
        raise ConstraintViolation("scene", f"Shape '{shape}' needs a supported length unit (m, cm, or mm).")
    return si(1.0, unit)


def _stl_path(graph, shape) -> str:
    # A path on this system, relative to a directory, or in a ROS or Python package.
    try:
        path = get_file_path(graph, graph.value(shape, PROV.wasDerivedFrom))
    except (ValueError, LookupError, ImportError) as error:
        raise ConstraintViolation("scene", f"File shape '{shape}' needs a local STL location: {error}") from error
    if path is None:
        raise ConstraintViolation("scene", f"File shape '{shape}' has no STL location.")
    if path.suffix.lower() != ".stl" or not path.is_file():
        raise ConstraintViolation("scene", f"File shape '{shape}' STL asset was not found: {path}.")
    return str(path.resolve())


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


def read_shapes(model, scene: MjcfSceneSpec) -> list[MjcfSceneShape]:
    """Each scene-authored shape, as one fixed SceneObject on its mapped object or the world.

    A shape on a mapped body rides that object's body, posed from the frame the body is built
    on. Any other shape stands in the world, posed from the scene anchor.
    """
    graph = model.graph
    anchor = quantities.anchor_frame(model)
    anchor_body = quantities.body_of(model, anchor)
    mapped = {obj.body_iri: obj.body for obj in scene.objects}
    joined = {
        body_of_frame(frame, graph)
        for joint in graph.subjects(RDF.type, URI_KC_TYPE_JOINT)
        for frame in graph.objects(joint, URI_KC_PRED_BETWEEN_ATTACHMENTS)
    }
    shapes: list[MjcfSceneShape] = []
    for shape, frame in graph.subject_objects(URI_SHAPES_PRED_ATTACHED_TO_FRAME):
        body = quantities.body_of(model, frame)
        if body is not None and str(body) in mapped:
            attach_kind, attach_name = "Body", mapped[str(body)]
            root = quantities.placement_frame(model, body)
        # The world only takes a static body; the scene anchor is static even when a joint
        # names its body.
        elif body in joined and body != anchor_body:
            raise ConstraintViolation(
                "scene", f"Shape '{shape}' is on a joined body without a mapped scene object."
            )
        else:
            attach_kind, attach_name, root = "World", "", anchor
        pos, quat = quantities.frame_placement(model, frame, root)
        if pos is None or quat is None:
            raise ConstraintViolation(
                "scene", f"Shape '{shape}' has no placement from its frame to '{root}'."
            )

        common = {
            "name": local_name(shape),
            "attach_kind": attach_kind,
            "attach_name": attach_name,
            "pos": pos,
            "quat": quat,
            "rgba": _color(graph, shape),
        }
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
            entry = MjcfSceneShape(**common, shape="MESH", size=sizes, path=_stl_path(graph, shape))
        else:
            raise ConstraintViolation("scene", f"Shape '{shape}' has no supported type.")
        used = {"SPHERE": 1, "CYLINDER": 2}.get(entry.shape, 3)
        if any(not math.isfinite(size) or size <= 0 for size in entry.size[:used]):
            raise ConstraintViolation(
                "scene", f"Shape '{shape}' has invalid dimensions after unit conversion."
            )
        shapes.append(entry)
    return shapes
