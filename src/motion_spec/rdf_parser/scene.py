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

from motion_spec_dsl.vocab import EXEC, GEOM_ENT, QUDT_SCHEMA
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
    scene.objects = scene_objects(model, attach_by_body, anchor)

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

    static_bodies = {
        body.id: body
        for kgraph in kinematic_graphs(scene_graph(model))
        for body in (
            *kgraph.free_bodies.values(),
            *(body for tree in kgraph.trees for body in tree.bodies.values()),
        )
    }
    scene.frames = scene_frames(model, scene.objects, static_bodies, trees)
    scene.static_cameras = static_cameras(model, scene, static_bodies, anchor)
    expand_vectors(scene)
    # unchecked, the model builds and the scene fails to assemble
    check_attachment_sites(model, assemblies, scene, attach_by_body)
    for obj in scene.objects:
        if obj.path:
            continue
        missing = [name for name in PROCEDURAL_FIELDS if getattr(obj, name) is None]
        if missing:
            raise ConstraintViolation(
                "scene",
                f"Procedural scene object '{obj.id}' is missing required field "
                f"'{missing[0]}'; add it to the .robmot model",
            )

    return scene


def scene_objects(model, attach_by_body: dict, anchor) -> list[MjcfSceneObject]:
    """The scene objects, each with its asset, placement and attachment, every host before the
    objects bolted to it.

    Raises:
        RuntimeError: an asset is in a ROS package and ROS is not on the path.
        ConstraintViolation: an asset states no file.
    """
    graph = model.graph
    objects = []
    parent_of = {}
    for obj, models in quantities.scene_instance(model).object_models.items():
        mapped = [
            (asset_id, mapping.target_id, mapping.entity or "")
            for asset_id, asset in models.items()
            for mapping in get_kinematic_mappings(asset, URI_GEOM_TYPE_RIGID_BODY)
        ]
        if not mapped:
            continue
        # further mappings are bodies of the same asset: named, not spawned again
        asset, body, _entity = mapped[0]
        object_id = local_name(obj)
        attachment = attach_by_body.get(body, ("World", "", body, None, None))
        attach_kind, attach_name, _frame, parent, _host_frame = attachment
        drawn = quantities.drawn_placement_of(model, attachment, anchor)
        position, orientation = (
            (None, None) if drawn else quantities.placement_of(model, attachment, anchor)
        )
        parent_of[local_name(body)] = local_name(parent) if parent is not None else None
        try:
            asset_path = get_linked_file_path(graph, asset, DCTERMS.source)
        except ImportError as missing:
            raise RuntimeError(
                f"asset '{asset}' is in a ROS package, which cannot be located without ROS on the "
                f"path: source /opt/ros/$ROS_DISTRO/setup.bash (and the workspace's own setup) in "
                f"the shell this is running from, then try again"
            ) from missing
        if asset_path is None:
            raise ConstraintViolation("scene", f"asset '{asset}' states no file")
        objects.append(
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
    # hosts first: the runtime resolves a parent site as it spawns
    return hosts_first(objects, parent_of)


def scene_frames(model, objects: list, static_bodies: dict, trees) -> list[MjcfSceneFrame]:
    """The marked frames of the static bodies, as the sites the runtime adds: placed on their
    body, or by the tree segment a drawn frame stands for.
    """
    graph = model.graph
    frames = []
    body_names = {uri: name for obj in objects for uri, name in obj.secondary_bodies.items()}
    marked = [
        (body.id, frame)
        for body in static_bodies.values()
        for frame in body.frames
        if frame != body.root_frame.id and GEOM_ENT.Frame in get_node_types(graph, frame)
    ]
    # a site name carries its body only on a clash
    counts = Counter(local_name(frame) for _body, frame in marked)
    drawn_frames = {
        entry["iri"]: (tree["cpp_name"], entry) for tree in trees for entry in unplaced_frames(tree)
    }
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
        placement = quantities.frame_placement(
            model, frame, quantities.placement_frame(model, body)
        )
        # left out rather than invent a spot at the body's origin
        if placement is None:
            continue
        position, orientation = placement
        frames.append(
            MjcfSceneFrame(
                body=body_name,
                name=site,
                uri=str(frame),
                **dict(zip(("pos_x", "pos_y", "pos_z"), position)),
                **dict(zip(("quat_x", "quat_y", "quat_z", "quat_w"), orientation)),
            )
        )

    return frames


def static_cameras(model, scene: MjcfSceneSpec, static_bodies: dict, anchor) -> list:
    """The cameras the scene mounts rather than a robot: on a marked frame's body, or in the world
    when the camera's frame is posed against the anchor.
    """
    cameras = []
    frames_by_uri = {frame.uri: frame for frame in scene.frames}
    for camera in scene.cameras:
        frame = frames_by_uri.get(camera.frame_uri)
        if frame is not None:
            cameras.append(
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
        # a frame posed against the anchor stands in the world, not on its body
        camera_frame = URIRef(camera.frame_uri)
        if body_of_frame(camera_frame, model.graph) not in static_bodies:
            continue
        placement = quantities.frame_placement(model, camera_frame, anchor)
        if placement is None:
            continue
        position, orientation = placement
        cameras.append(
            MjcfSceneCamera(
                name=camera.id,
                body="",
                fovy_deg=camera.fovy_deg,
                **dict(zip(("pos_x", "pos_y", "pos_z"), position)),
                **dict(zip(("quat_x", "quat_y", "quat_z", "quat_w"), orientation)),
            )
        )

    return cameras


def expand_vectors(scene: MjcfSceneSpec) -> None:
    """Expand the placement of every robot, attachment and object into its components, and the
    geometry a procedural object states.
    """
    for robot in scene.robots:
        for item in (robot, *robot.attachments):
            expand_placement(item)
    for obj in scene.objects:
        expand_placement(obj)
        obj.has_path = bool(obj.path)
        if obj.has_path:
            continue
        for name, components, default in PROCEDURAL_VECTORS:
            if getattr(obj, name) is not None:
                expand_vector(obj, name, components, default)


def check_attachment_sites(model, assemblies, scene: MjcfSceneSpec, attach_by_body: dict) -> None:
    """Check that each robot attached to a scene object's site attaches by a site the object's
    asset declares.

    Raises:
        ConstraintViolation: the asset declares no site of the attachment frame's name.
    """
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
    for body, (kind, _name, _frame, parent_body, host_frame) in attach_by_body.items():
        if kind != "Site" or parent_body not in object_ids_by_body:
            continue
        # the root frame: the one the placement is composed against
        name = local_name(quantities.placement_frame(model, parent_body))
        attach_by_body[body] = (
            kind,
            # the runtime names the object after the body it maps
            f"{local_name(parent_body)}_{name}",
            host_frame,
            parent_body,
            host_frame,
        )


PLACEMENT_VECTORS = (
    ("pos", ("x", "y", "z"), None),
    ("quat", ("x", "y", "z", "w"), [0.0, 0.0, 0.0, 1.0]),
)
PROCEDURAL_VECTORS = (
    ("size", ("x", "y", "z"), None),
    ("color", ("r", "g", "b", "a"), None),
    ("friction", ("slide", "torsion", "roll"), None),
)
PROCEDURAL_FIELDS = ("size", "color", "friction", "shape", "mass")


def expand_placement(item) -> None:
    """Expand an item's position and orientation into their components; unset, the origin and
    the identity.
    """
    for name, components, default in PLACEMENT_VECTORS:
        expand_vector(item, name, components, default)


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
