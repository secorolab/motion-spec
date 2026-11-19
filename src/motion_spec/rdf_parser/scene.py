# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The scene the runtime assembles: every robot and object with its asset, placement, attachment,
frames and cameras.

Each reader validates what it reads, at the moment it reads it: procedural geometry a path-less
object omits, an attachment onto a site its asset does not declare.
"""

from __future__ import annotations

from collections import Counter
from pathlib import Path
from xml.etree import ElementTree

from motion_spec_dsl.rdf_parser.vocab import EXEC, GEOM_ENT, QUDT_SCHEMA
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdf_utils.models.prov import get_linked_file_path
from rdf_utils.models.vocab import URI_GEOM_TYPE_RIGID_BODY
from rdflib import URIRef
from rdflib.namespace import DCTERMS, RDF
from scene_dsl.rdf_parser.kinematics import body_of_frame, get_kinematic_mappings, kinematic_graphs

from motion_spec.classes.scene import (
    MjcfSceneCamera,
    MjcfSceneFrame,
    MjcfSceneObject,
    MjcfSceneRobot,
    MjcfSceneSpec,
)
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.agents import agent_assemblies, fixed_attachments, scene_graph
from motion_spec.rdf_parser.model import local_name, seconds
from motion_spec.rdf_parser.sampling import unplaced_frames


def read_scene(model, trees=()) -> MjcfSceneSpec:
    """The scene: every robot and object with its asset, placement and attachment.

    Geometry comes from the referenced mjcf assets, so procedural geometry fields stay unset and
    placement between attached frames is coincident.

    Raises:
        ConstraintViolation: a procedural (path-less) object omits geometry the model must declare.
    """
    graph = model.graph
    scene = MjcfSceneSpec()
    context = next(graph.subjects(RDF.type, EXEC.ExecutionContext), None)
    timestep = graph.value(context, EXEC.timestep) if context is not None else None
    value = graph.value(timestep, QUDT_SCHEMA.value) if timestep is not None else None
    if value is not None:
        scene.timestep_s = seconds(float(value.toPython()), graph.value(timestep, QUDT_SCHEMA.unit))

    attach_by_body, _root = fixed_attachments(model)
    name_object_attachments(model, attach_by_body)
    anchor = quantities.anchor_frame(model)

    parent_of = {}
    for obj, models in quantities.scene_instance(model).object_models.items():
        mapped = [
            (asset_id, mapping.target_id, mapping.entity or "")
            for asset_id, asset in models.items()
            for mapping in get_kinematic_mappings(asset, URI_GEOM_TYPE_RIGID_BODY)
        ]
        if not mapped:
            continue
        # The first mapping is the one the object is spawned as; the rest are further bodies of
        # the same asset, which the scene already carries -- they are named, not spawned again.
        asset, body, _entity = mapped[0]
        object_id = local_name(obj)
        attachment = attach_by_body.get(body, ("World", "", body, None, None))
        attach_kind, attach_name, _frame, parent, _host_frame = attachment
        drawn = quantities.drawn_placement_of(model, attachment, anchor)
        position, orientation = (
            (None, None) if drawn else quantities.placement_of(model, attachment, anchor)
        )
        parent_of[local_name(body)] = local_name(parent) if parent is not None else None
        # A file in a ROS package resolves against the package's share directory here, so the
        # runtime is handed a path it can open without knowing about ROS.
        try:
            asset_path = get_linked_file_path(graph, asset, DCTERMS.source)
        except ImportError as missing:
            # The one place that can say which asset needed ROS, instead of a bare import error.
            raise RuntimeError(
                f"asset '{asset}' is in a ROS package, which cannot be located without ROS on the "
                f"path: source /opt/ros/$ROS_DISTRO/setup.bash (and the workspace's own setup) in "
                f"the shell this is running from, then try again"
            ) from missing
        if asset_path is None:
            raise ConstraintViolation("scene", f"asset '{asset}' states no file")
        scene.objects.append(
            MjcfSceneObject(
                id=object_id,
                body=local_name(body),
                body_iri=str(body),
                path=str(asset_path),
                fixed=body in attach_by_body,
                attach_kind=attach_kind,
                attach_name=attach_name,
                pos=position,
                quat=orientation,
                draw=model.id(drawn.position) if drawn else None,
                draw_uri=str(drawn.position) if drawn else "",
                draw_rotation=drawn.rotation if drawn else None,
                draw_base_pos=drawn.base_pos if drawn else None,
                draw_base_quat=drawn.base_quat if drawn else None,
                secondary_bodies={
                    str(other): f"{object_id}_{entity}"
                    for _asset, other, entity in mapped[1:]
                    if entity
                },
            )
        )
    # The runtime resolves a parent site as it spawns, so a bolted-on object follows its host.
    scene.objects = hosts_first(scene.objects, parent_of)

    assemblies = agent_assemblies(model, attach_by_body)
    for assembly in assemblies:
        scene.robots.append(
            MjcfSceneRobot(
                id=local_name(assembly.agent),
                agent=str(assembly.agent),
                path=assembly.urdf,
                prefix=assembly.prefix,
                attach_kind=assembly.attach_kind,
                attach_name=assembly.attach_name,
                pos=assembly.pos,
                quat=assembly.quat,
                attachments=assembly.attachments,
            )
        )
        scene.cameras.extend(assembly.cameras)

    # A frame on a body an object maps beyond its root goes on that body under the name the
    # composed scene gives it, not under its kgraph name, which names nothing in the scene.
    body_names = {uri: name for obj in scene.objects for uri, name in obj.secondary_bodies.items()}
    # A body's own root frame is where the body is, so it needs no marker of its own; the rest are
    # posed against it, which is the frame the runtime builds each body on.
    # Every body the scene graph itself holds, as opposed to one a robot asset brings in.
    static_bodies = {
        body.id: body
        for kgraph in kinematic_graphs(scene_graph(model))
        for body in (
            *kgraph.free_bodies.values(),
            *(body for tree in kgraph.trees for body in tree.bodies.values()),
        )
    }
    marked = [
        (body.id, frame)
        for body in static_bodies.values()
        for frame in body.frames
        if frame != body.root_frame.id and GEOM_ENT.Frame in get_node_types(graph, frame)
    ]
    # A site is named after its frame, and carries its body only when another body has a frame
    # of the same name -- the name has to be unique, and it has to stay readable in a viewer.
    counts = Counter(local_name(frame) for _body, frame in marked)
    # A frame the tree could not place is marked from the tree that will place it at runtime.
    drawn_frames = {
        entry["iri"]: (tree["cpp_name"], entry) for tree in trees for entry in unplaced_frames(tree)
    }
    frames = []
    for body, frame in marked:
        name = local_name(frame)
        body_name = body_names.get(str(body), local_name(body))
        site = f"{body_name}_{name}" if counts[name] > 1 else name
        if str(frame) in drawn_frames:
            tree, entry = drawn_frames[str(frame)]
            frames.append(
                MjcfSceneFrame(
                    body=body_name,
                    name=site,
                    uri=str(frame),
                    segment=entry["name"],
                    body_segment=entry["body"],
                    tree=tree,
                )
            )
            continue
        position, orientation = quantities.frame_placement(
            model, frame, quantities.placement_frame(model, body)
        )
        # A frame no pose leads to is left out rather than placed at the body's origin, which
        # would invent a spot.
        if position is None:
            continue
        frames.append(
            MjcfSceneFrame(
                body=body_name,
                name=site,
                uri=str(frame),
                **dict(zip(("pos_x", "pos_y", "pos_z"), position)),
                **dict(zip(("quat_x", "quat_y", "quat_z", "quat_w"), orientation)),
            )
        )
    scene.frames = frames
    # A camera on a static scene frame is built into the composed scene at that frame's site
    # pose; a camera on a robot-asset frame rides the asset's own MJCF instead.
    frames_by_uri = {frame.uri: frame for frame in scene.frames}
    for camera in scene.cameras:
        frame = frames_by_uri.get(camera.frame_uri)
        if frame is not None:
            scene.static_cameras.append(
                MjcfSceneCamera(
                    name=camera.id,
                    body=frame.body,
                    fovy_deg=camera.fovy_deg,
                    pos_x=frame.pos_x,
                    pos_y=frame.pos_y,
                    pos_z=frame.pos_z,
                    quat_x=frame.quat_x,
                    quat_y=frame.quat_y,
                    quat_z=frame.quat_z,
                    quat_w=frame.quat_w,
                )
            )
            continue
        # A scene frame the placement search cannot reach from its own body is one the model
        # posed against the anchor instead: it stands in the world rather than riding the body
        # it is declared under, so it is built on the world body at the pose the anchor gives
        # it. A camera on a robot asset is not this case -- it rides that asset's own MJCF.
        camera_frame = URIRef(camera.frame_uri)
        if body_of_frame(camera_frame, graph) not in static_bodies:
            continue
        position, orientation = quantities.frame_placement(model, camera_frame, anchor)
        if position is None:
            continue
        scene.static_cameras.append(
            MjcfSceneCamera(
                name=camera.id,
                body="",
                fovy_deg=camera.fovy_deg,
                **dict(zip(("pos_x", "pos_y", "pos_z"), position)),
                **dict(zip(("quat_x", "quat_y", "quat_z", "quat_w"), orientation)),
            )
        )
    # A path-backed object takes its geometry from its asset, so only a procedural one expands.
    # What a procedural object leaves out is rejected below, so the expansion never depends on it.
    for robot in scene.robots:
        for item in (robot, *robot.attachments):
            for name, components, default in _PLACEMENT_VECTORS:
                expand_vector(item, name, components, default)
    for obj in scene.objects:
        for name, components, default in _PLACEMENT_VECTORS:
            expand_vector(obj, name, components, default)
        obj.has_path = bool(obj.path)
        if obj.has_path:
            continue
        for name, components, default in _PROCEDURAL_VECTORS:
            if getattr(obj, name) is not None:
                expand_vector(obj, name, components, default)

    # The runtime reaches a site on a scene object by the object's name and the site's, and only
    # the asset can say the site is there -- unchecked, the model builds and the scene fails to
    # assemble.
    for assembly, robot in zip(assemblies, scene.robots):
        if robot.attach_kind != "Site":
            continue
        parent = attach_by_body[assembly.root_body][3]
        obj = next((item for item in scene.objects if item.body_iri == str(parent)), None)
        if obj is None or not obj.path or not (asset := Path(obj.path)).is_file():
            continue
        site = local_name(quantities.placement_frame(model, parent))
        declared = [
            name
            for element in ElementTree.parse(asset).iter("site")
            if (name := element.get("name"))
        ]
        if site not in declared:
            raise ConstraintViolation(
                "scene",
                f"'{robot.id}' attaches to frame '{site}' of scene object '{obj.id}', which "
                f"'{obj.path}' states no site for. The asset holds an attachment by a site of "
                f"the same name; it declares {declared or 'none'}.",
            )
    # A procedural scene object that omits geometry is rejected: the model must declare it, and a
    # silent default would place a body the author never described.
    for obj in scene.objects:
        if obj.path:
            continue
        missing = [name for name in _PROCEDURAL_FIELDS if getattr(obj, name) is None]
        if missing:
            raise ConstraintViolation(
                "scene",
                f"Procedural scene object '{obj.id}' is missing required field "
                f"'{missing[0]}'. Add it to the .robmot model -- silent defaults are no "
                f"longer applied.",
            )

    return scene


def hosts_first(objects: list, parent_of: dict) -> list:
    """The objects with each one after the object it is bolted to, otherwise in the order given."""
    bodies = {obj.body for obj in objects}

    def hosts(body) -> int:
        parent = parent_of.get(body)
        return 1 + hosts(parent) if parent in bodies else 0

    return sorted(objects, key=lambda obj: hosts(obj.body))


def name_object_attachments(model, attach_by_body) -> None:
    """Rename an attachment onto a scene object after the object, as the runtime addresses it."""
    object_ids_by_body = {
        mapping.target_id: local_name(obj)
        for obj, models in quantities.scene_instance(model).object_models.items()
        for asset in models.values()
        for mapping in get_kinematic_mappings(asset, URI_GEOM_TYPE_RIGID_BODY)
    }
    for body, (kind, _name, _frame, parent_body, host_frame) in list(attach_by_body.items()):
        if kind != "Site" or parent_body not in object_ids_by_body:
            continue
        # The site is the object's root frame, which is the one an asset exposes and the one
        # the placement is composed against. Naming any other frame on the way up would
        # measure the offset from one frame and apply it at another.
        name = local_name(quantities.placement_frame(model, parent_body))
        attach_by_body[body] = (
            kind,
            # The runtime composes this as the object's name and the site, and it names the
            # object after the body it maps -- so the prefix has to be that body, not the object.
            f"{local_name(parent_body)}_{name}",
            host_frame,
            parent_body,
            host_frame,
        )


# Per scene item, the vector fields expanded into scalar components, with the value an omitted
# one falls back to. Env placement shorthand: no position means the origin, no rotation identity.
_PLACEMENT_VECTORS = (
    ("pos", ("x", "y", "z"), None),
    ("quat", ("x", "y", "z", "w"), [0.0, 0.0, 0.0, 1.0]),
)
# Procedural geometry a path-less object must declare, and the components each expands into.
_PROCEDURAL_VECTORS = (
    ("size", ("x", "y", "z"), None),
    ("color", ("r", "g", "b", "a"), None),
    ("friction", ("slide", "torsion", "roll"), None),
)
_PROCEDURAL_FIELDS = ("size", "color", "friction", "shape", "mass")


def expand_vector(item, name: str, components, default) -> None:
    """Expand a vector field into its `<name>_<component>` parts.

    Raises:
        ConstraintViolation: the value is present but does not have one number per component.
    """
    values = getattr(item, name)
    if values is None:
        values = list(default) if default is not None else [0.0] * len(components)
    if not isinstance(values, list) or len(values) != len(components):
        raise ConstraintViolation(
            "scene",
            f"Scene item '{item.id}' has invalid '{name}'; expected {len(components)} values.",
        )
    for component, value in zip(components, values):
        setattr(item, f"{name}_{component}", float(value))
