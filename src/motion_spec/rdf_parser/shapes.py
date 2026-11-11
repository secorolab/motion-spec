# SPDX-License-Identifier: MPL-2.0
"""Scene-authored geometry shared by the viewer and generated MuJoCo startup.

scene-dsl reads each shape from the graph (`scene_dsl.rdf_parser.shapes`); this places it on a
MuJoCo body and sizes it the way MuJoCo takes each kind.
"""

from __future__ import annotations

import math
from collections import Counter

import rdflib
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import (
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_TYPE_JOINT,
    URI_SHAPES_PRED_ATTACHED_TO_FRAME,
)
from rdflib.namespace import RDF
from scene_dsl.classes.ktree import PrimitiveShapeType
from scene_dsl.rdf_parser.kinematics import body_of_frame
from scene_dsl.rdf_parser.shapes import ShapeModel, scene_shapes

from motion_spec.classes.scene import MjcfSceneShape, MjcfSceneSpec
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.model import local_name


def _stl_path(shape: ShapeModel) -> str:
    # scene-dsl takes any mesh file; the runtime builds a mesh from STL only.
    if shape.path.suffix.lower() != ".stl":
        raise ConstraintViolation("scene", f"File shape '{shape.id}' is no STL file: {shape.path}.")
    return str(shape.path.resolve())


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
    found = scene_shapes(graph)
    # A shape is named after its IRI's last segment, and carries its body only when a shape on
    # another body ends the same way -- as frame sites are named in `scene.py`: MuJoCo needs the
    # name unique, and a viewer needs it readable.
    counts = Counter(local_name(shape.id) for shape in found)
    shapes: list[MjcfSceneShape] = []
    for shape in found:
        body = quantities.body_of(model, shape.frame)
        if body is not None and str(body) in mapped:
            attach_kind, attach_name = "Body", mapped[str(body)]
            root = quantities.placement_frame(model, body)
        # The world only takes a static body; the scene anchor is static even when a joint
        # names its body.
        elif body in joined and body != anchor_body:
            raise ConstraintViolation(
                "scene", f"Shape '{shape.id}' is on a joined body without a mapped scene object."
            )
        else:
            attach_kind, attach_name, root = "World", "", anchor
        pos, quat = quantities.frame_placement(model, shape.frame, root)
        if pos is None or quat is None:
            raise ConstraintViolation(
                "scene", f"Shape '{shape.id}' has no placement from its frame to '{root}'."
            )

        name = local_name(shape.id)
        if counts[name] > 1 and body is not None:
            name = f"{local_name(body)}_{name}"
        common = {
            "name": name,
            "attach_kind": attach_kind,
            "attach_name": attach_name,
            "pos": pos,
            "quat": quat,
            # None leaves MuJoCo's default: the scene stated no colour.
            "rgba": None if shape.rgba is None else list(shape.rgba),
        }
        sizes = list(shape.scale)
        if shape.primitive is PrimitiveShapeType.SPHERE:
            uniform = sizes[0] == sizes[1] == sizes[2]
            entry = MjcfSceneShape(
                **common, shape="SPHERE" if uniform else "ELLIPSOID",
                size=[sizes[0], 0.0, 0.0] if uniform else sizes,
            )
        elif shape.primitive is PrimitiveShapeType.CUBE:
            # MuJoCo sizes a box by its half extents.
            entry = MjcfSceneShape(**common, shape="BOX", size=[size / 2 for size in sizes])
        elif shape.primitive is PrimitiveShapeType.CYLINDER:
            if sizes[0] != sizes[1]:
                raise ConstraintViolation(
                    "scene",
                    f"Shape '{shape.id}' is an elliptical cylinder (X scale != Y scale); MuJoCo's "
                    "cylinder is circular.",
                )
            # MuJoCo sizes a cylinder by its radius and half height.
            entry = MjcfSceneShape(**common, shape="CYLINDER", size=[sizes[0], sizes[2] / 2, 0.0])
        else:
            entry = MjcfSceneShape(**common, shape="MESH", size=sizes, path=_stl_path(shape))
        used = {"SPHERE": 1, "CYLINDER": 2}.get(entry.shape, 3)
        if any(not math.isfinite(size) or size <= 0 for size in entry.size[:used]):
            raise ConstraintViolation(
                "scene", f"Shape '{shape.id}' has invalid dimensions after unit conversion."
            )
        shapes.append(entry)
    return shapes
