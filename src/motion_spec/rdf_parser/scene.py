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

from motion_spec_dsl.rdf_parser.vocab import AGN, ENV, EXEC, GEOM_ENT, QUDT_SCHEMA
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdf_utils.models.execution import get_path_of_node
from rdf_utils.models.vocab import (
    URI_GEOM_TYPE_KGRAPH,
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_TYPE_JOINT,
)
from rdflib import URIRef
from rdflib.namespace import RDF
from scene_dsl.rdf_parser.kinematics import body_of_frame, root_bodies
from scene_dsl.rdf_parser.vocab import URI_ROS_PRED_PACKAGE_NAME

from motion_spec.classes.scene import (
    MjcfSceneCamera,
    MjcfSceneFrame,
    MjcfSceneObject,
    MjcfSceneRobot,
    MjcfSceneSpec,
)
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.agents import (
    agent_assemblies,
    fixed_attachments,
    mapped_targets,
    model_mappings,
)
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

    bound_trees = mapped_targets(model, AGN["AgentModel"], GEOM_ENT.KinematicTree)
    attach_by_body, _root = fixed_attachments(model, bound_trees)
    name_object_attachments(model, attach_by_body)
    anchor = quantities.anchor_frame(model)

    parent_of = {}
    for modelled in graph.subjects(RDF.type, ENV["ModelledObject"]):
        obj = graph.value(modelled, ENV["of-object"])
        mapped = [
            (asset, body, entity)
            for asset in graph.objects(modelled, ENV["has-object-model"])
            for body, entity in model_mappings(model, asset, GEOM_ENT.RigidBody)
        ]
        if obj is None or not mapped:
            continue
        # The first mapping is the one the object is spawned as; the rest are further bodies of
        # the same asset, which the scene already carries -- they are named, not spawned again.
        asset, body, _entity = mapped[0]
        object_id = local_name(obj)
        attachment = attach_by_body.get(body, ("World", "", body, None))
        attach_kind, attach_name, _frame, parent = attachment
        drawn = quantities.drawn_placement_of(model, attachment, anchor)
        position, orientation = (
            (None, None) if drawn else quantities.placement_of(model, attachment, anchor)
        )
        parent_of[local_name(body)] = local_name(parent) if parent is not None else None
        scene.objects.append(
            MjcfSceneObject(
                id=object_id,
                body=local_name(body),
                body_iri=str(body),
                path=_asset_path(graph, asset),
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
    scene.objects = _hosts_first(scene.objects, parent_of)

    for assembly in agent_assemblies(model, attach_by_body):
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

    scene.frames = _scene_frames(model, scene.objects, trees)
    # A camera on a static scene frame is built into the composed scene at that frame's site
    # pose; a camera on a robot-asset frame rides the asset's own MJCF instead.
    frames_by_uri = {frame.uri: frame for frame in scene.frames}
    static_bodies = _static_body_uris(model)
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
        if str(body_of_frame(camera_frame, graph)) not in static_bodies:
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
    _expand_scene_geometry(scene)
    _validate_scene(scene, model.app_path)

    return scene


def _scene_frames(model, objects, trees=()) -> list:
    """Every frame the kgraph declares, placed on the body that carries it.

    A body's own root frame is where the body is, so it needs no marker of its own; the rest
    are posed against it, which is the frame the runtime builds each body on. A frame no pose
    leads to is left out rather than placed at the body's origin, which would invent a spot.

    A frame on a body an object maps beyond its root goes on that body under the name the
    composed scene gives it, not under its kgraph name, which names nothing in the scene.
    """
    graph = model.graph
    body_names = {uri: name for obj in objects for uri, name in obj.secondary_bodies.items()}
    marked = [
        (body, frame)
        for kgraph in graph.subjects(RDF.type, URI_GEOM_TYPE_KGRAPH)
        for body in _kgraph_bodies(model, kgraph)
        for frame in graph.objects(body, GEOM_ENT.simplices)
        if frame != quantities.placement_frame(model, body)
        and GEOM_ENT.Frame in get_node_types(graph, frame)
    ]
    # A site is named after its frame, and carries its body only when another body has a frame
    # of the same name -- the name has to be unique, and it has to stay readable in a viewer.
    counts = Counter(local_name(frame) for _body, frame in marked)
    # A frame the tree could not place is marked from the tree that will place it at runtime.
    drawn = {
        entry["iri"]: (tree["cpp_name"], entry) for tree in trees for entry in unplaced_frames(tree)
    }

    frames = []
    for body, frame in marked:
        name = local_name(frame)
        body_name = body_names.get(str(body), local_name(body))
        site = f"{body_name}_{name}" if counts[name] > 1 else name
        if str(frame) in drawn:
            tree, entry = drawn[str(frame)]
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
    return frames


def _hosts_first(objects: list, parent_of: dict) -> list:
    """The objects with each one after the object it is bolted to, otherwise in the order given."""
    bodies = {obj.body for obj in objects}

    def hosts(body) -> int:
        parent = parent_of.get(body)
        return 1 + hosts(parent) if parent in bodies else 0

    return sorted(objects, key=lambda obj: hosts(obj.body))


def _kgraph_bodies(model, kgraph) -> set:
    """Every body the graph holds: the ones it roots, and everything a joint reaches."""
    graph = model.graph
    bodies = set(root_bodies(kgraph, graph))
    for joint in graph.subjects(RDF.type, URI_KC_TYPE_JOINT):
        for frame in graph.objects(joint, URI_KC_PRED_BETWEEN_ATTACHMENTS):
            bodies.add(body_of_frame(frame, graph))
    return {body for body in bodies if body is not None}


def name_object_attachments(model, attach_by_body) -> None:
    """Rename an attachment onto a scene object after the object, as the runtime addresses it."""
    graph = model.graph
    object_ids_by_body = {
        body: local_name(obj)
        for modelled in graph.subjects(RDF.type, ENV["ModelledObject"])
        if (obj := graph.value(modelled, ENV["of-object"])) is not None
        for asset in graph.objects(modelled, ENV["has-object-model"])
        for body, _entity in model_mappings(model, asset, GEOM_ENT.RigidBody)
    }
    for body, (kind, name, frame, parent_body) in list(attach_by_body.items()):
        if kind != "Site" or parent_body not in object_ids_by_body:
            continue
        # The site is the object's root frame, which is the one an asset exposes and the one
        # the placement is composed against. Naming any other frame on the way up would
        # measure the offset from one frame and apply it at another.
        frame = model.child_node(parent_body, name)
        name = local_name(quantities.placement_frame(model, parent_body))
        attach_by_body[body] = (
            kind,
            # The runtime composes this as the object's name and the site, and it names the
            # object after the body it maps -- so the prefix has to be that body, not the object.
            f"{local_name(parent_body)}_{name}",
            frame,
            parent_body,
        )


def _asset_path(graph, asset) -> str:
    """Where an asset file is, from how the model says to find it.

    A path in a ROS package is only meaningful against that package's share directory, so it is
    resolved here; the runtime is handed a path it can open without knowing about ROS. Anything
    else keeps the path the model authored.
    """
    path = get_path_of_node(graph, asset)
    package = graph.value(asset, URI_ROS_PRED_PACKAGE_NAME)
    if package is None:
        return path
    # Imported here: a model with no ROS asset still generates without ROS on the path.
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as missing:
        # This is the one thing that made the asset need ROS at all, so it is the only place
        # that can say which asset, and what to do about it, instead of a bare import error.
        raise RuntimeError(
            f"asset '{path}' is in ROS package '{package}', which cannot be located without "
            f"ROS on the path: source /opt/ros/$ROS_DISTRO/setup.bash (and the workspace's "
            f"own setup) in the shell this is running from, then try again"
        ) from missing

    return str(Path(get_package_share_directory(str(package))) / path)


def _static_body_uris(model) -> set[str]:
    """Every body the scene graph itself holds, as opposed to one a robot asset brings in."""
    return {
        str(body)
        for kgraph in model.graph.subjects(RDF["type"], URI_GEOM_TYPE_KGRAPH)
        for body in _kgraph_bodies(model, kgraph)
    }


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


def _expand_vector(item, name: str, components, default) -> None:
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


def _expand_scene_geometry(scene: MjcfSceneSpec) -> None:
    """Expand placement vectors and procedural geometry onto the scene items themselves.

    A path-backed object takes its geometry from its asset, so only a procedural one expands. What
    a procedural object leaves out is `_validate_scene`'s to reject, so this never depends on it.
    """
    for robot in scene.robots:
        for item in (robot, *robot.attachments):
            for name, components, default in _PLACEMENT_VECTORS:
                _expand_vector(item, name, components, default)
    for obj in scene.objects:
        for name, components, default in _PLACEMENT_VECTORS:
            _expand_vector(obj, name, components, default)
        obj.has_path = bool(obj.path)
        if obj.has_path:
            continue
        for name, components, default in _PROCEDURAL_VECTORS:
            if getattr(obj, name) is not None:
                _expand_vector(obj, name, components, default)


def _asset_file(path: str, app_path: Path) -> Path | None:
    """The file the runtime would open for an asset, or None when the search comes up empty.

    Mirrors the generated controller's search so a check here reads the same file the run does:
    the path as given, or one below a directory the working directory or the generation sits in.
    """
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate if candidate.is_file() else None
    starts = (Path.cwd(), app_path.parent)
    return next(
        (
            root / candidate
            for start in starts
            for root in (start, *start.parents)
            if (root / candidate).is_file()
        ),
        None,
    )


def _validate_object_attachments(scene: MjcfSceneSpec, app_path: Path) -> None:
    """Reject an attachment onto a scene object whose asset states no site to hold it.

    The runtime reaches such a site by the object's name and the site's, and only the asset can
    say the site is there -- unchecked, the model builds and the scene fails to assemble.
    """
    for robot in scene.robots:
        if robot.attach_kind != "Site":
            continue
        obj = next(
            (item for item in scene.objects if robot.attach_name.startswith(f"{item.body}_")), None
        )
        if obj is None or not obj.path or (asset := _asset_file(obj.path, app_path)) is None:
            continue
        site = robot.attach_name[len(obj.body) + 1 :]
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


def _validate_scene(scene: MjcfSceneSpec, app_path: Path) -> None:
    """Reject a procedural scene object that omits geometry: the model must declare it, and a
    silent default would place a body the author never described.
    """
    _validate_object_attachments(scene, app_path)
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
