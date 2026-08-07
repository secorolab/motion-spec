# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Solvers, robots and the scene they are built from: chain setups, solver sections,
platform and device binding, and the joint-space channels mirrored into the frame log."""

from __future__ import annotations

import collections
from dataclasses import replace
from pathlib import Path
import rdflib
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.execution import get_path_of_node
from scene_dsl.rdf_parser.kinematics import (
    body_of_frame, get_kinematic_mapping,
)
from scene_dsl.rdf_parser.sensors import get_update_rate
from rdf_utils.models.geom_coord import (
    OrientCoordModel, PositionCoordModel, get_coord_vectorxyz, get_orientation_coord_vals,
)
from rdf_utils.models.geom_rel import (
    OrientationModel, PositionModel,
)
from rdf_utils.models.common import (
    ModelBase, get_node_types,
)
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_OF, URI_GEOM_TYPE_POSITION, URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_KC_TYPE_SERIAL,
)
from rdf_utils.namespace import NS_MM_KC_EXT
from rdf_utils.uri import (
    iri_is_descendant, iri_parent,
)
from rdflib.namespace import (
    RDF, SDO, split_uri,
)
from motion_spec.classes.entities import (
    HandlerSerialChainSolver, SceneAttachment, SceneObject, SceneObjectSpec, SceneRobot,
    SceneSpec,
)
# fmt: off
from motion_spec_dsl.rdf_parser.vocab import (
    AGN, APP, CSTR_HDL, CSTR_HDL_EXT, ENV, EXEC, GEOM_COORD, GEOM_ENT, GEOM_REL, KC, KC_STAT,
    QUDT_SCHEMA, RBDYN_COORD, RBDYN_ENT, SLV, SLV_EXT, SENSORS, SOSA,
)
# fmt: on

# fmt: off
from motion_spec.rdf_parser.records import (
    _cpp_identifier, _dedupe_by_id, _field, _set_field, _sole, expand_vector_fields,
    require_field,
)
# fmt: on
# fmt: off
from motion_spec.rdf_parser.graph import (
    DerivedIriRegistry, Parser, _length_unit, _seconds, _si_all, ops_cstr_hdl, ops_generic,
    ops_slv,
)
# fmt: on
from motion_spec.rdf_parser.controllers import (
    SolverDerivationContext, SolverIdFactory, _derived_controllers, _derived_motion_drivers,
    _motion_suffix,
)

def _upstream_dependencies(data_id: str, closure_input_map: dict[str, set[str]]) -> set[str]:
    """Transitive set of data ids that feed the given id through the closure input map."""
    result: set[str] = set()
    pending = list(closure_input_map.get(data_id, set()))
    while pending:
        item = pending.pop()
        if item in result:
            continue
        result.add(item)
        pending.extend(closure_input_map.get(item, set()))
    return result
def _body_name(name: str | None) -> str | None:
    """Strip a frame_/link_ prefix to the bare MuJoCo body name."""
    if name is None:
        return None
    for prefix in ("frame_", "frame-", "link_", "link-"):
        if name.startswith(prefix):
            return name[len(prefix) :]
    return name
def _mark_acceleration_constraint_frames(solver):
    """Flag each of a solver's acceleration constraints as base-aligned when its axis frame is the
    chain root (or unset).
    """
    root_body = _body_name(getattr(solver, "chain_root", None))
    for driver in solver.motion_drivers:
        for constraint in driver.acceleration_constraint:
            axis_frame = getattr(constraint.as_seen_by, "id", None)
            constraint.base_aligned = axis_frame is None or _body_name(axis_frame) == root_body
def _serial_chain_solvers_for_handler(handler, slv_chain, solver_ids):
    """Select the arm solvers explicitly referenced by a handler's controllers."""
    result = []
    motion_driver_id = f"driver_{handler.motion.id.removeprefix('motion_')}"
    for solver in slv_chain:
        if solver.id not in solver_ids:
            continue
        matched = list(solver.motion_drivers)
        if not matched:
            continue
        selected = next((driver for driver in matched if driver.id == motion_driver_id), matched[0])
        result.append(
            HandlerSerialChainSolver(
                id=solver.id,
                output=solver.output,
                motion_driver=selected,
                algorithm=solver.algorithm,
                gravity=solver.gravity,
                root_acc=solver.root_acc,
                chain_root=solver.chain_root,
                chain_end=solver.chain_end,
                torque_saturation=solver.torque_saturation,
            )
        )

    return result
def _frames_of(g, node):
    if GEOM_ENT.Frame in get_node_types(g, node):
        return [node]
    return [
        frame
        for frame in g.objects(node, GEOM_ENT.simplices)
        if GEOM_ENT.Frame in get_node_types(g, frame)
    ]
def _position_of(g, node):
    """Position of a body or frame from its authored scene-dsl pose, in metres, or None."""
    for frame in _frames_of(g, node):
        origin = g.value(frame, GEOM_ENT.origin) or frame
        for position_id in g.subjects(URI_GEOM_PRED_OF, origin):
            if URI_GEOM_TYPE_POSITION not in get_node_types(g, position_id):
                continue
            position = PositionModel(position_id=position_id, graph=g)
            for coordinate_id in position.coordinate_ids:
                coordinate = PositionCoordModel(
                    coord_id=coordinate_id, graph=g, position=position
                )
                if URI_DISTRIB_TYPE_SAMPLED_QUANTITY in coordinate.types:
                    raise ConstraintViolation(
                        "geometry",
                        f"Sampled placement coordinate '{coordinate.id}' is unsupported",
                    )
                if (value := get_coord_vectorxyz(coordinate, g)) is None:
                    continue
                return _si_all(value, _length_unit(coordinate))
    return None
def _orientation_of(g, node):
    """Rotation of a body or frame from its authored scene pose, as a quaternion
    [x, y, z, w], or None when the pose declares no orientation."""
    for frame in _frames_of(g, node):
        for orientation_node in g.subjects(GEOM_REL.of, frame):
            if GEOM_REL.Orientation not in get_node_types(g, orientation_node):
                continue
            orientation = OrientationModel(orn_id=orientation_node, graph=g)
            for coord_id in orientation.coordinate_ids:
                coord = OrientCoordModel(coord_id=coord_id, graph=g, orientation=orientation)
                if URI_DISTRIB_TYPE_SAMPLED_QUANTITY in coord.types:
                    raise ConstraintViolation(
                        "geometry", f"Sampled placement coordinate '{coord.id}' is unsupported"
                    )
                rotation = get_orientation_coord_vals(coord, g)
                if rotation is not None:
                    return list(rotation.as_quat())
    return None
def _optional_path_of_model(g, model_node):
    """Return an optional agent model path; pathless agent models are not runtime assets."""
    path = g.value(model_node, EXEC.path) if model_node is not None else None
    return str(path) if path is not None else ""
def _model_mappings(g, model, target_type):
    """Return (scene target, model entity) mappings of the requested RDF type. What a mapping means
    is its metamodel's to say, not ours.
    """
    mappings = (
        get_kinematic_mapping(mapping, g)
        for mapping in sorted(g.objects(model, EXEC["has-mapping"]), key=str)
    )
    return [
        (mapping.target_id, mapping.entity or "")
        for mapping in mappings
        if mapping.target_type == target_type
    ]
def _mapped_targets(g, model_type, target_type):
    """All scene targets of a given type mapped by models of model_type."""
    return {
        target
        for model in g.subjects(RDF.type, model_type)
        for target, _entity in _model_mappings(g, model, target_type)
    }
# The MuJoCo trajectory trace is a viewer-only overlay authored in the scene, not the motion
# spec; it is no longer part of this graph, so it stays disabled and costs nothing for headless
# / non-MuJoCo runtimes. Codegen guards on trace.enabled.
_TRACE_DISABLED = {
    "enabled": False,
    "length": 4096,
    "color_r": 1.0,
    "color_g": 0.5,
    "color_b": 0.1,
    "color_a": 1.0,
    "targets": [],
    "has_targets": False,
}
def _leaf(node):
    return split_uri(str(node))[1]
def _tree_owns(tree, node):
    return iri_is_descendant(tree, node)
def _kinematic_adjacency(g):
    adjacency = collections.defaultdict(list)
    fixed = []
    for joint in g.subjects(RDF.type, KC.Joint):
        frames = list(g.objects(joint, KC["between-attachments"]))
        if len(frames) != 2:
            continue
        body_a, body_b = (body_of_frame(f, g) for f in frames)
        if body_a == body_b:
            continue
        adjacency[body_a].append((body_b, frames[0], frames[1], joint))
        adjacency[body_b].append((body_a, frames[1], frames[0], joint))
        if get_node_types(g, joint) == {KC.Joint}:
            fixed.append((frames[0], frames[1]))
    return adjacency, fixed
def _distances(adjacency, source):
    distances = {source: 0}
    queue = collections.deque([source])
    while queue:
        node = queue.popleft()
        for neighbor, *_ in adjacency[node]:
            if neighbor not in distances:
                distances[neighbor] = distances[node] + 1
                queue.append(neighbor)
    return distances
def _body_path(adjacency, start, end):
    """Oriented body/frame edges on the shortest kinematic path from start to end."""
    parents = {start: None}
    queue = collections.deque([start])
    while queue and end not in parents:
        node = queue.popleft()
        for neighbor, frame, neighbor_frame, joint in adjacency[node]:
            if neighbor not in parents:
                parents[neighbor] = (node, frame, neighbor_frame, joint)
                queue.append(neighbor)
    if end not in parents:
        return []
    path = []
    node = end
    while parents[node] is not None:
        parent, parent_frame, child_frame, joint = parents[node]
        path.append((parent, node, parent_frame, child_frame, joint))
        node = parent
    return list(reversed(path))
def _fixed_attachments(g, bound_trees):
    """Fixed scene/model boundaries, oriented from the world's root toward robot tips."""
    adjacency, fixed = _kinematic_adjacency(g)
    if not fixed:
        return {}, None
    leaves = [body for body in adjacency if len(adjacency[body]) == 1]
    tip_distances = [
        _distances(adjacency, body_of_frame(tip, g))
        for tip in g.objects(None, NS_MM_KC_EXT["tip"])
    ]

    def distance_from_nearest_tip(body):
        distances = [distance[body] for distance in tip_distances if body in distance]
        return min(distances, default=-1)

    root = max(leaves, key=distance_from_nearest_tip) if leaves else None
    from_root = _distances(adjacency, root) if root is not None else {}

    def owner(body):
        return next(
            (
                tree
                for tree in sorted(bound_trees, key=lambda item: (-len(str(item)), str(item)))
                if _tree_owns(tree, body)
            ),
            None,
        )

    modelled_bodies = _mapped_targets(g, ENV["ObjectModel"], GEOM_ENT.RigidBody)
    attachments = {}
    for frame_a, frame_b in fixed:
        parent_frame, child_frame = (
            (frame_a, frame_b)
            if from_root.get(body_of_frame(frame_a, g), 1 << 30)
            <= from_root.get(body_of_frame(frame_b, g), 1 << 30)
            else (frame_b, frame_a)
        )
        parent_body, child_body = (body_of_frame(f, g) for f in (parent_frame, child_frame))
        if parent_body == root:
            attachments[child_body] = ("World", "", child_frame, parent_body)
        elif child_body in modelled_bodies or owner(parent_body) != owner(child_body):
            attachments[child_body] = (
                "Site",
                _leaf(parent_frame),
                child_frame,
                parent_body,
            )
    return attachments, root
def _sensor_kind(g, sensor) -> str:
    """The sensor's kind as the IR names it; empty for a kind codegen does not model."""
    types = get_node_types(g, sensor)
    return next((name for uri, name in SENSOR_KINDS.items() if uri in types), "")
def _device_of(g, element):
    """The deployed system that realizes `element`, or None. Read backwards from the device, which
    the execution context owns, because the element it stands for belongs to the scene.
    """
    return next(iter(g.subjects(EXEC["realizes"], element)), None)
def _device_kind(g, element) -> str:
    """The hardware kind realizing `element`, or empty when it is unbound."""
    device = _device_of(g, element)
    return str(g.value(device, SDO.model) or "") if device is not None else ""
def _config_key(g, element, agent, drives: str) -> str:
    """What `robot.toml` calls the device on `element`.

    A hosted sensor is named by its owning agent's leaf plus its own, since `runtime_prefix` inside
    `drives` is empty on a single-robot model. An agent is named by the scenex namespace it was
    referred to through, which survives in its IRI as the segment before the model's own.
    """
    if drives:
        return f"{_leaf(agent)}.{_leaf(element)}"
    # The set the scene declares the agent in, addressed as `<set>.<agent>`. Taken from the
    # graph, not from an IRI segment: the IRI path is namespace layout, not the authored name,
    # and the two disagree when the namespace is not named after the set.
    bdd = rdflib.Namespace("https://secorolab.github.io/metamodels/acceptance-criteria/bdd#")
    owner = next(g.subjects(bdd["elements"], element), None)
    if owner is None:
        raise ValueError(f"Agent '{element}' belongs to no declared agent set.")
    return f"{_leaf(owner)}.{_leaf(element)}"
def _bound_devices(g, agent, runtime_prefix, hosted, chain_bindings, agent_by_tree) -> list[dict]:
    """The hardware bound on this chain: what each device is, where it is configured, what it drives.

    The kind is the authored name passed through untouched -- only the backend's templates
    interpret it. `drives` names the sensor a sensor device reads, and is empty for a joint mover.
    """

    def entry(node, drives=""):
        device = _device_of(g, node)
        if device is None:
            return None
        return {
            "kind": str(g.value(device, SDO.model) or ""),
            "config_key": _config_key(g, node, agent, drives),
            "drives": drives,
        }

    owners = [agent]
    for binding in chain_bindings:
        owner = agent_by_tree.get(binding["tree"])
        # A tree may be bound by several models; the agent behind it is named once.
        if owner is not None and owner not in owners:
            owners.append(owner)
    found = [entry(owner) for owner in owners]
    found += [entry(sensor, f"{runtime_prefix}{_leaf(sensor)}") for sensor in hosted]
    return [device for device in found if device is not None]
def _agent_assemblies(g, attach_by_body):
    """Resolve model bindings into runtime robot assets, attachments, and chain bounds."""
    adjacency, _fixed = _kinematic_adjacency(g)
    bound_model_trees = _mapped_targets(g, AGN["AgentModel"], GEOM_ENT.KinematicTree)
    body_names_by_tree = {
        tree: {
            _leaf(body)
            for body in g.subjects(RDF.type, GEOM_ENT.RigidBody)
            if _tree_owns(tree, body)
        }
        for tree in bound_model_trees
    }
    serials = sorted(
        (
            (
                tree,
                g.value(tree, NS_MM_KC_EXT["root"]),
                g.value(tree, NS_MM_KC_EXT["tip"]),
            )
            for tree in g.subjects(RDF.type, URI_KC_TYPE_SERIAL)
        ),
        key=lambda item: str(item[0]),
    )
    # A chain may span several agents; the agent owning its root drives it.
    bindings_by_modelled = {}
    for modelled in sorted(g.subjects(RDF.type, AGN.ModelledAgent), key=str):
        rows = []
        for model in sorted(g.objects(modelled, AGN["has-agent-model"]), key=str):
            path = _optional_path_of_model(g, model)
            if not path:
                continue
            for tree, entity in _model_mappings(g, model, GEOM_ENT.KinematicTree):
                rows.append({"model": model, "tree": tree, "path": path, "entity": entity})
        bindings_by_modelled[modelled] = rows
    bindings = [row for rows in bindings_by_modelled.values() for row in rows]
    agent_by_tree = {
        row["tree"]: g.value(modelled, AGN["of-agent"])
        for modelled, rows in bindings_by_modelled.items()
        for row in rows
    }

    def binding_for(node):
        return next((binding for binding in bindings if _tree_owns(binding["tree"], node)), None)

    result = []
    for modelled in sorted(g.subjects(RDF.type, AGN.ModelledAgent), key=str):
        agent = g.value(modelled, AGN["of-agent"])
        own = bindings_by_modelled[modelled]
        if agent is None or not own:
            continue

        def own_binding_for(node, own=own):
            return next((binding for binding in own if _tree_owns(binding["tree"], node)), None)

        serial = next(
            (
                (tree, root, tip)
                for tree, root, tip in serials
                if root is not None
                and tip is not None
                and (
                    any(binding["tree"] == tree for binding in own)
                    or own_binding_for(root) is not None
                )
            ),
            None,
        )
        if serial is None:
            continue
        serial_tree, root_frame, tip_frame = serial
        root_binding = own_binding_for(root_frame) or next(
            binding for binding in own if binding["tree"] == serial_tree
        )
        tip_binding = binding_for(tip_frame) or root_binding
        root_body, tip_body = (body_of_frame(f, g) for f in (root_frame, tip_frame))
        duplicate_root = sum(
            _leaf(root_body) in names for names in body_names_by_tree.values()
        ) > 1
        runtime_prefix = f"{_leaf(root_binding['tree'])}_" if duplicate_root else ""
        runtime_root = f"{runtime_prefix}{_leaf(root_body)}"
        path = _body_path(adjacency, root_body, tip_body)
        # Scoped to this chain's path: two arms must not claim each other's models.
        chain_bodies = [root_body, tip_body, *(body for edge in path for body in edge[:2])]
        chain_bindings = [
            binding
            for binding in bindings
            if any(_tree_owns(binding["tree"], body) for body in chain_bodies)
        ]
        chain_tip_body = tip_body if root_binding["tree"] == serial_tree else root_body
        if root_binding["tree"] != serial_tree:
            for parent_body, child_body, *_ in path:
                if _tree_owns(root_binding["tree"], child_body):
                    chain_tip_body = child_body
                elif _tree_owns(root_binding["tree"], parent_body):
                    chain_tip_body = parent_body
                    break

        attachments = []
        for binding in chain_bindings:
            if binding is root_binding or binding["path"] == root_binding["path"]:
                continue
            boundary = next(
                (
                    edge
                    for edge in path
                    if _tree_owns(binding["tree"], edge[1])
                    and not _tree_owns(binding["tree"], edge[0])
                ),
                None,
            )
            if boundary is None:
                continue
            _parent_body, child_body, parent_frame, _child_frame, _joint = boundary
            entity = binding["entity"]
            child_name = _leaf(child_body)
            prefix = child_name[: -len(entity)] if entity and child_name.endswith(entity) else ""
            attachments.append(
                SceneAttachment(
                    id=_leaf(binding["tree"]),
                    path=binding["path"],
                    attach_to=_leaf(parent_frame),
                    attach_kind="Site",
                    prefix=prefix,
                )
            )

        attach_kind, attach_name, placement_frame, _parent_body = attach_by_body.get(
            root_body, ("World", "", root_frame, None)
        )
        hosted = sorted(g.objects(modelled, SOSA.hosts), key=str)
        sensors = [
            {
                "id": f"{runtime_prefix}{_leaf(sensor)}",
                "type": kind,
                "frame_site": f"{runtime_prefix}{_leaf(frame)}",
                "update_rate_hz": get_update_rate(g, ModelBase(node_id=sensor, graph=g)),
                "observes": sorted(_leaf(observed) for observed in g.objects(sensor, SOSA.observes)),
            }
            for sensor in hosted
            if (kind := _sensor_kind(g, sensor))
            and (frame := g.value(sensor, SENSORS.frame)) is not None
        ]
        agent_device = _device_of(g, agent)
        device = str(g.value(agent_device, SDO.model) or "") if agent_device else ""
        # An agent is named by the scenex alias it was referred to through, bound or not: a
        # simulated deployment addresses it the same way.
        config_key = _config_key(g, agent, agent, "")
        result.append(
            {
                "agent": agent,
                "device": device,
                "config_key": config_key,
                "sensors": sensors,
                "devices": _bound_devices(
                    g, agent, runtime_prefix, hosted, chain_bindings, agent_by_tree
                ),
                "path": root_binding["path"],
                "prefix": runtime_prefix,
                "trees": [binding["tree"] for binding in chain_bindings],
                "serial_chain": serial_tree,
                "root_body": root_body,
                "chain_root": runtime_root,
                "chain_tip": f"{runtime_prefix}{_leaf(chain_tip_body)}",
                "tool_body": (
                    f"{runtime_prefix}{_leaf(tip_body)}"
                    if tip_binding is not root_binding
                    else ""
                ),
                "tcp_site": (
                    f"{runtime_prefix}{_leaf(tip_frame)}"
                    if tip_binding is not root_binding
                    else ""
                ),
                "attach_kind": attach_kind,
                "attach_name": attach_name,
                "placement_frame": placement_frame,
                "attachments": attachments,
            }
        )
    return result
def _scene_from_graph(g):
    """Build the scene (robots + objects with model paths, placement and attachment) from the
    scene-dsl (`.scenex`) graph. Geometry comes from the referenced mjcf assets, so procedural
    geometry fields stay unset and placement between attached frames is coincident (identity).
    """
    scene = SceneSpec()
    context = next(g.subjects(RDF.type, EXEC.ExecutionContext), None)
    if context is not None:
        timestep = g.value(context, EXEC.timestep)
        value = g.value(timestep, QUDT_SCHEMA.value)
        if value is not None:
            scene.timestep_s = _seconds(
                float(value.toPython()), g.value(timestep, QUDT_SCHEMA.unit)
            )

    bound_trees = _mapped_targets(g, AGN["AgentModel"], GEOM_ENT.KinematicTree)
    attach_by_body, _root = _fixed_attachments(g, bound_trees)
    object_ids_by_body = {
        body: _leaf(obj)
        for modelled in g.subjects(RDF.type, ENV["ModelledObject"])
        if (obj := g.value(modelled, ENV["of-object"])) is not None
        for model in g.objects(modelled, ENV["has-object-model"])
        for body, _entity in _model_mappings(g, model, GEOM_ENT.RigidBody)
    }
    for body, (kind, name, frame, parent_body) in list(attach_by_body.items()):
        if kind == "Site" and parent_body in object_ids_by_body:
            parent_frame = rdflib.Namespace(f"{parent_body}/")[name]
            reference_frame = next(
                (
                    reference
                    for pose in g.subjects(GEOM_REL.of, parent_frame)
                    if (reference := g.value(pose, GEOM_REL["with-respect-to"])) is not None
                    and GEOM_ENT.Frame in get_node_types(g, reference)
                    and body_of_frame(reference, g) == parent_body
                ),
                None,
            )
            if reference_frame is not None:
                name = _leaf(reference_frame)
                frame = parent_frame
            name = f"{object_ids_by_body[parent_body]}_{name}"
        attach_by_body[body] = (kind, name, frame, parent_body)
    for modelled in sorted(g.subjects(RDF.type, ENV["ModelledObject"]), key=str):
        obj = g.value(modelled, ENV["of-object"])
        mapped = next(
            (
                (model, body)
                for model in sorted(g.objects(modelled, ENV["has-object-model"]), key=str)
                for body, _entity in _model_mappings(g, model, GEOM_ENT.RigidBody)
            ),
            None,
        )
        if obj is None or mapped is None:
            continue
        model, body = mapped
        path = get_path_of_node(g, model)
        attach_kind, attach_name, placement_frame, _parent_body = attach_by_body.get(
            body, ("World", "", body, None)
        )
        scene.objects.append(
            SceneObjectSpec(
                id=_leaf(obj),
                body=_leaf(body),
                path=path,
                fixed=body in attach_by_body,
                attach_kind=attach_kind,
                attach_name=attach_name,
                pos=_position_of(g, placement_frame),
                quat=_orientation_of(g, placement_frame),
            )
        )

    for assembly in _agent_assemblies(g, attach_by_body):
        scene.robots.append(
            SceneRobot(
                id=_leaf(assembly["agent"]),
                path=assembly["path"],
                prefix=assembly["prefix"],
                attach_kind=assembly["attach_kind"],
                attach_name=assembly["attach_name"],
                pos=_position_of(g, assembly["placement_frame"]),
                quat=_orientation_of(g, assembly["placement_frame"]),
                attachments=assembly["attachments"],
            )
        )

    _expand_scene_geometry(scene)
    return scene
def _expand_scene_geometry(scene) -> None:
    """Expand placement vectors and procedural geometry onto the native scene items so the scene is
    codegen-complete at construction. Path-backed objects take geometry from their MJCF/URDF asset.
    Missing required geometry is `_validate_scene`'s to raise, so building never depends on it.
    """
    for robot in scene.robots:
        expand_vector_fields(robot, "pos")
        expand_vector_fields(robot, "quat", ("x", "y", "z", "w"), default=[0.0, 0.0, 0.0, 1.0])
        for attachment in robot.attachments:
            expand_vector_fields(attachment, "pos")
            expand_vector_fields(
                attachment, "quat", ("x", "y", "z", "w"), default=[0.0, 0.0, 0.0, 1.0]
            )
    for obj in scene.objects:
        expand_vector_fields(obj, "pos")
        expand_vector_fields(obj, "quat", ("x", "y", "z", "w"), default=[0.0, 0.0, 0.0, 1.0])
        obj.has_path = bool(obj.path)
        if obj.has_path:
            continue
        if obj.size is not None:
            obj.size_x, obj.size_y, obj.size_z = (
                float(obj.size[0]),
                float(obj.size[1]),
                float(obj.size[2]),
            )
        if obj.color is not None:
            obj.color_r, obj.color_g, obj.color_b, obj.color_a = (
                float(obj.color[0]),
                float(obj.color[1]),
                float(obj.color[2]),
                float(obj.color[3]),
            )
        if obj.friction is not None:
            obj.friction_slide, obj.friction_torsion, obj.friction_roll = (
                float(obj.friction[0]),
                float(obj.friction[1]),
                float(obj.friction[2]),
            )
def _validate_scene(scene) -> None:
    """Every procedural (non-path) scene object must carry full geometry — the model must
    declare it; silent defaults are not applied. Runs at construction time in generate_ir."""
    for obj in scene.objects:
        if bool(obj.path):
            continue
        require_field(obj.id, "size", obj.size)
        require_field(obj.id, "color", obj.color)
        require_field(obj.id, "friction", obj.friction)
        require_field(obj.id, "shape", obj.shape)
        require_field(obj.id, "mass", obj.mass)
def _scene_chain(trees, assembly):
    """The agent assembly's declared serial chain, with MuJoCo runtime joint names."""
    from motion_spec.generation.scene_kdl import chain_for_iri

    name, tree, joints = chain_for_iri(trees, str(assembly["serial_chain"]))
    return name, tree, [f"{assembly['prefix']}{joint}" for joint in joints]
def _robot_setups_from_graph(g):
    """Per-robot solver chain setups, sourced from the scene-dsl (`.scenex`) graph.

    Returns ``(setups_by_node, ordered)`` where ``setups_by_node`` maps each robot's abstract agent
    node (the target of a solver's ``agn:of-agent``) to its setup tuple ``(urdf, chain_root,
    chain_end, chain_tip, robot_model, tool_body, tcp_site, sensors, devices, runtime_prefix,
    owned_trees, kdl_chain, kdl_tree, kdl_joints, config_key)``; ``kdl_joints`` is in KDL order.

    A scene-dsl frame URI is ``.../<robot>/<body>/<frame|site>``, so the body is the
    second-to-last path segment and the tip site is the last.
    """
    def _robot_model_from_path(path):
        low = str(path).lower()
        for hint, canonical in (("kinova_gen3", "KinovaGen3"), ("gen3", "KinovaGen3")):
            if hint in low:
                return canonical
        return ""

    def _robot_model_for(assembly):
        """The agent's device: authored when bound, else sniffed from the asset path."""
        # The arm-with-gripper pairing is wiring; the manipulator is still the arm.
        device = assembly.get("device") or ""
        if device:
            return "KinovaGen3" if device == "KinovaGen3-2F85" else device
        return _robot_model_from_path(assembly["path"])

    from scene_dsl.kdl_tree import build_kdl_trees

    try:
        trees = build_kdl_trees(g)
    except ConstraintViolation:
        # Graph-only consumers may use an incomplete scene fixture: assembly metadata survives,
        # but there is no KDL chain until Scene DSL can parse it.
        trees = []
    setups_by_node, ordered = {}, []
    bound_trees = _mapped_targets(g, AGN["AgentModel"], GEOM_ENT.KinematicTree)
    attach_by_body, _root = _fixed_attachments(g, bound_trees)
    for assembly in _agent_assemblies(g, attach_by_body):
        setup = (
            assembly["path"],
            assembly["chain_root"],
            assembly["chain_tip"],
            assembly["chain_tip"],
            _robot_model_for(assembly),
            assembly["tool_body"],
            assembly["tcp_site"],
            assembly["sensors"],
            assembly["devices"],
            assembly["prefix"],
            assembly["trees"],
            *_scene_chain(trees, assembly),
            assembly.get("config_key") or "",
        )
        setups_by_node[assembly["agent"]] = setup
        ordered.append(setup)
    return setups_by_node, ordered
def _world_solver_outputs(
    g, p: Parser, chain_root: str, runtime_prefix: str, owned_trees, scene_objects
):
    """Parse runtime observations in the solver's reference frame."""
    object_ids_by_body = {obj.body: obj.id for obj in scene_objects}
    outputs = []

    def _runtime_frame(frame_node):
        """Runtime body/site name for a scene frame owned by this solver."""
        frame = p.frame(frame_node)
        frame_tree = iri_parent(body_of_frame(frame_node, g))
        return replace(
            frame,
            id=f"{runtime_prefix}{frame.id}" if frame_tree in owned_trees else frame.id,
        )

    for type_, parse in (
        (GEOM_COORD.PoseCoordinate, p.pose),
        (GEOM_COORD.VelocityTwistCoordinate, p.velocity_twist),
        (KC_STAT.JointPositionCoordinate, p.joint_position),
        (RBDYN_COORD.WrenchCoordinate, p.wrench),
    ):
        for node in sorted(g.subjects(RDF.type, type_), key=str):
            scope = p._context_scope(node)
            if scope is None or scope[1] != "world":
                continue
            if type_ == KC_STAT.JointPositionCoordinate:
                joint = g.value(node, KC_STAT["of-joint"])
                if joint is None or not any(_tree_owns(tree, joint) for tree in owned_trees):
                    continue
            frame_node = None
            if type_ != KC_STAT.JointPositionCoordinate:
                seen_by_predicate = (
                    RBDYN_COORD["as-seen-by"]
                    if type_ == RBDYN_COORD.WrenchCoordinate
                    else GEOM_COORD["as-seen-by"]
                )
                frame_node = g.value(node, seen_by_predicate)
                if frame_node is None:
                    _of, _wrt, frame_node = p._derived_reference_frames(node)
            if type_ == RBDYN_COORD.WrenchCoordinate:
                sensor = g.value(node, SOSA.madeBySensor)
                sensor_frame_node = g.value(sensor, SENSORS.frame) if sensor is not None else None
                if sensor_frame_node is None or not any(
                    _tree_owns(tree, sensor_frame_node) for tree in owned_trees
                ):
                    continue
            elif frame_node is not None:
                frame_body = body_of_frame(frame_node, g)
                frame_tree = iri_parent(frame_body)
                runtime_frame = _leaf(frame_body)
                if frame_tree in owned_trees:
                    runtime_frame = f"{runtime_prefix}{_leaf(frame_body)}"
                if runtime_frame != chain_root:
                    continue
            output = parse(node)
            if type_ == KC_STAT.JointPositionCoordinate:
                output = replace(output, joint_name=f"{runtime_prefix}{output.joint_name}")
            elif type_ == RBDYN_COORD.WrenchCoordinate:
                relation = g.value(node, RBDYN_COORD["of-wrench"])
                reference_node = g.value(relation, RBDYN_ENT["reference-point"])
                output = replace(
                    output,
                    sensor_name=f"{runtime_prefix}{output.sensor_name}",
                    sensor_frame=_runtime_frame(sensor_frame_node),
                    reference_point=replace(
                        output.reference_point, id=_runtime_frame(reference_node).id
                    ),
                    as_seen_by=_runtime_frame(frame_node),
                )
            frame = getattr(output, "as_seen_by", None)
            if frame_node is None and frame is not None and frame.id != chain_root:
                continue
            of = getattr(output, "of", None)
            if getattr(of, "id", None) in object_ids_by_body:
                output.of = SceneObject(object_ids_by_body[of.id], of.id)
            outputs.append(output)
    return _dedupe_by_id(outputs)
def _solver_sections(
    g,
    p: Parser,
    setups_by_node: dict,
    default_setup,
    derivation: SolverDerivationContext,
    scene_objects,
):
    """Parse the solver/handler sections: base-velocity, arm and base-force solvers with their
    schedules.
    """
    sched1 = []
    slv_platform_vel = []
    sched2 = []
    hdl = []
    sched3 = []
    slv_chain = []
    sched4 = []
    slv_platform_frc = []

    for s in sorted(g.subjects(RDF.type, SLV["VelocityCompositionSolver"]), key=str):
        slv_platform_vel.append(p.velocity_composition_solver(s))
        sched1.extend(p.schedule([s], ops_generic + ops_slv))

    handler_nodes = sorted(
        g.subjects(RDF.type, CSTR_HDL["ConstraintHandler"]),
        key=lambda node: int(getattr(g.value(node, APP.order), "value", 0)),
    )
    for h in handler_nodes:
        handler = p.constraint_handler(h)
        handler.controllers = [
            controller
            for plan in derivation.controllers_by_handler.get(h, ())
            for controller in _derived_controllers(g, p, derivation, plan)
        ]
        hdl.append(handler)
        evaluator_nodes = list(g.objects(h, CSTR_HDL.evaluators))
        sched2.extend(p.schedule(evaluator_nodes, ops_generic + ops_cstr_hdl))
        plans = derivation.controllers_by_handler.get(h, ())
        for plan in plans:
            if len(plan.axes) > 1 or CSTR_HDL_EXT.FeedForwardController in get_node_types(
                g, plan.controller
            ):
                continue
            error = g.value(plan.controller, CSTR_HDL["error-signal"])
            evaluator = next(g.subjects(CSTR_HDL.error, error), None)
            if evaluator is not None:
                evaluator_id = p.id(evaluator)
                if evaluator_id not in sched2:
                    sched2.append(evaluator_id)
        sched2.extend(
            controller.id
            for plan in reversed(plans)
            for controller in reversed(_derived_controllers(g, p, derivation, plan))
        )
        sched2.extend(
            SolverIdFactory(p.id(plan.controller), _motion_suffix(p, plan.motion)).pose_evaluator()
            for plan in reversed(plans)
            if len(plan.axes) > 1
        )

    solver_nodes = []
    for handler in handler_nodes:
        for plan in derivation.controllers_by_handler.get(handler, ()):
            if plan.solver not in solver_nodes and SLV.SolverWithInputAndOutput in get_node_types(
                g, plan.solver
            ):
                solver_nodes.append(plan.solver)
    solver_nodes.extend(
        sorted(
            set(g.subjects(RDF.type, SLV.SolverWithInputAndOutput)) - set(solver_nodes),
            key=str,
        )
    )
    for s in solver_nodes:
        solver = p.solver_with_input_and_output(s)
        solver.motion_drivers = _derived_motion_drivers(g, p, derivation, s)
        robot_node = g.value(s, AGN["of-agent"])
        (
            solver.urdf,
            solver.chain_root,
            solver.chain_end,
            solver.chain_tip,
            solver.robot_model,
            solver.tool_body,
            solver.tcp_site,
            solver.sensors,
            solver.devices,
            solver.runtime_prefix,
            solver.owned_trees,
            solver.kdl_chain,
            solver.kdl_tree,
            solver.kdl_joints,
            solver.config_key,
        ) = setups_by_node.get(robot_node, default_setup)
        solver.output = _dedupe_by_id(
            [
                *solver.output,
                *_world_solver_outputs(
                    g,
                    p,
                    solver.chain_root,
                    solver.runtime_prefix,
                    solver.owned_trees,
                    scene_objects,
                ),
            ]
        )
        _mark_acceleration_constraint_frames(solver)
        slv_chain.append(solver)
        start = g[
            s
            : SLV["motion-drivers"]
            / ((SLV["cartesian-force"]) | (SLV["joint-force"]))
        ]
        sched3.extend(p.schedule(start, ops_generic + ops_slv))

    for s in sorted(g.subjects(RDF.type, SLV["ForceDistributionSolver"]), key=str):
        slv_platform_frc.append(p.force_distribution_solver(s))
        sched4.extend(p.schedule([s], ops_generic + ops_slv))

    # velocity-distribution and force-composition are authorable but have no template wiring:
    # the first needs a per-solver actuation-mode switch (kelo_cmd.ctrl_mode is hardcoded to
    # ROBIF2B_CTRL_MODE_FORCE), the second a measured wheel torque the kelo struct does not
    # expose. Fail loudly rather than drop them from codegen.
    for unsupported_type, label in (
        (SLV_EXT.VelocityDistributionSolver, "velocity-distribution"),
        (SLV_EXT.ForceCompositionSolver, "force-composition"),
    ):
        unsupported = sorted(g.subjects(RDF.type, unsupported_type), key=str)
        if unsupported:
            raise ValueError(
                f"Mobile-platform solver '{unsupported[0]}' uses algorithm '{label}', which "
                "the motion-spec code generator does not implement yet (no motion.stg/"
                "hddc2b.stg wiring exists for it). Use velocity-composition or "
                "force-distribution instead."
            )

    return slv_platform_vel, sched1, hdl, sched2, slv_chain, sched3, slv_platform_frc, sched4
# Which codegen backend serves an authored simulation platform. The platform is the model's; the
# backend is an implementation detail of running it, so the mapping lives here and nowhere else.
_SIMULATION_BACKENDS = {"mujoco": "mj_kdl"}
def _platform_from_graph(g) -> dict:
    """The execution platform the model declares, as one record every consumer reads: the authored
    node's IRI, its name, whether it is simulated, and the backend serving it. Platform identity
    comes from here, never re-derived from the backend token or from path substrings.
    """
    simulation = next(g.subjects(RDF.type, EXEC.Simulation), None)
    if simulation is None:
        real = next(g.subjects(RDF.type, EXEC.RealWorld), None)
        _reject_scene_objects_on_hardware(g, real)
        _reject_undriven_devices(g, real)
        _reject_unbound_sensors_on_hardware(g, real)
        return {
            "uri": str(real) if real is not None else None,
            "name": None,
            "simulated": False,
            "backend": "robif2b",
            "config": str(_config_path(g, real)) if real is not None else None,
        }
    name = str(g.value(simulation, SDO.name) or "")
    backend = _SIMULATION_BACKENDS.get(name.casefold())
    if backend is None:
        raise ValueError(f"Unsupported simulation platform '{name}'.")
    return {
        "uri": str(simulation),
        "name": name,
        "simulated": True,
        "backend": backend,
        "config": _config_path(g, simulation),
    }
def _config_path(g, context) -> str | None:
    """The deployment config's path, from exec:has-resource -> exec:path. The DSL resolves it against
    the model that declares it, so neither codegen nor the generated program needs the .robmot's directory.
    """
    config = g.value(context, EXEC["has-resource"])
    return str(g.value(config, EXEC.path)) if config is not None else None
def _reject_undriven_devices(g, context) -> None:
    """Reject a bound device the backend would silently ignore. The grammar decides what a model may
    name; this decides what the backend can drive, so an uncovered device fails here.
    """
    if context is None:
        return
    bound = sorted(
        {
            str(g.value(device, SDO.model) or "")
            for device in g.subjects(EXEC["realizes"], None)
        }
    )
    undriven = [name for name in bound if name not in DRIVEN_DEVICES]
    if undriven:
        raise ConstraintViolation(
            "platform",
            f"no backend support for device(s): {', '.join(undriven)}. "
            f"Driven: {', '.join(sorted(DRIVEN_DEVICES))}. Remove the binding, or add "
            "the driver templates before binding it.",
        )
def _reject_scene_objects_on_hardware(g, context) -> None:
    """Reject scene objects on hardware: nothing measures their pose without perception."""
    if context is None:
        return
    objects = sorted(_leaf(node) for node in g.subjects(RDF.type, ENV.ModelledObject))
    if objects:
        raise ConstraintViolation(
            "platform",
            f"real-world execution cannot use scene objects ({', '.join(objects)}): their poses "
            "come from a simulator, and nothing measures them on hardware. Remove them, or model "
            "the location as an authored frame.",
        )
def _reject_unbound_sensors_on_hardware(g, context) -> None:
    """Reject a sensor a model reads from but binds no device to. In simulation the simulator answers
    for every sensor; on hardware a reading comes from a device or from uninitialised memory.
    """
    if context is None:
        return
    unbound = sorted(
        _leaf(sensor)
        for sensor in set(g.objects(None, SOSA.madeBySensor))
        if _device_of(g, sensor) is None
    )
    if unbound:
        raise ConstraintViolation(
            "platform",
            f"real-world execution reads sensor(s) with no device bound: {', '.join(unbound)}. "
            "Bind one in the platform block, or stop reading the sensor.",
        )
def _shared_runtime_members(slv_chain, iris, control_period_ns: int, platform_uri) -> list[dict]:
    """Extra shared-data members for force/torque sensor state and the measured control period."""
    # The dt every integrator steps with, measured from the backend clock and nominal until the
    # first measurement. A contracted shared value, so it carries a producer and is logged.
    if not platform_uri:
        raise RuntimeError("measured dt: the execution platform has no IRI to derive a clock from")
    # Which clock it is is the platform's to say: the port derives from the exec context.
    clock_iri = iris.register("clock", platform_uri, "clock", DerivedIriRegistry.DERIVATION)
    iris.register("clock_time_s", clock_iri, "clock_time_s", DerivedIriRegistry.DERIVATION)
    iris.register("dt_measured_s", clock_iri, "dt_measured_s", DerivedIriRegistry.DERIVATION)
    # The reading is contracted too: written from the same port each tick, read like any value.
    members = [
        {"id": "clock_time_s", "type": "Quantity"},
        {"id": "dt_measured_s", "type": "Quantity", "value": control_period_ns * 1e-9},
    ]
    seen_ft_ids = set()
    for s in slv_chain:
        for out in s.output:
            if getattr(out, "type", None) == "Wrench" and getattr(out, "sensor_name", ""):
                if out.id in seen_ft_ids:
                    continue
                seen_ft_ids.add(out.id)
                # The tare state is computed from the reading, so it derives from that sensor
                # output's own node.
                sensor_iri = iris.iri_of(out.id)
                if sensor_iri is None:
                    raise RuntimeError(
                        f"ft tare state: sensor output '{out.id}' has no IRI to derive from"
                    )
                for suffix, member_type in (("ft_bias", "Wrench"), ("ft_settle", "IntCounter")):
                    member_id = f"{out.id}_{suffix}"
                    members.append({"id": member_id, "type": member_type})
                    iris.register(
                        member_id, sensor_iri, suffix, DerivedIriRegistry.DERIVATION
                    )

    return members
SUPPORTED_ROBOT_MODELS = {"KinovaGen3"}
# Devices with driver templates. A name the grammar accepts but that is missing here is rejected.
DRIVEN_DEVICES = {"KinovaGen3", "KinovaGen3-2F85", "Robotiq2F85", "RobotiqFT300s"}
# The sensor kinds the IR models, as the graph types them and as templates dispatch on them.
SENSOR_KINDS = {SENSORS.ForceTorqueSensor: "ForceTorque"}
def _validate_solvers(serial_chain_solvers, backend: str) -> None:
    """Reject unsupported robot models, and scene-object pose sync on the robif2b backend."""
    unsupported = {
        _field(s, "robot_model")
        for s in serial_chain_solvers
        if _field(s, "robot_model") and _field(s, "robot_model") not in SUPPORTED_ROBOT_MODELS
    }
    if unsupported:
        raise RuntimeError(
            f"Unsupported robot model(s): {', '.join(sorted(unsupported))}. "
            f"Supported: {', '.join(sorted(SUPPORTED_ROBOT_MODELS))}"
        )

    if backend != "robif2b":
        return

    for solver in serial_chain_solvers:
        for out in _field(solver, "output", []):
            if _field(out, "type") != "Pose":
                continue
            entity = _field(out, "of") or {}
            if _field(entity, "is_scene_object"):
                obj_id = _field(entity, "id") or _field(entity, "body") or _field(out, "id")
                raise RuntimeError(
                    "robif2b backend cannot sync scene-object pose output "
                    f"'{_field(out, 'id')}' for '{obj_id}'; world/scene object pose sync "
                    "is only implemented for mj_kdl."
                )
def _runtime_signature(solver, backend: str) -> tuple:
    """Identity tuple of a solver's runtime (backend, chain, tool) for deduplicating runtimes."""
    return (
        backend,
        _field(solver, "robot_model", ""),
        _field(solver, "urdf", ""),
        _field(solver, "chain_root", ""),
        _field(solver, "chain_tip") or _field(solver, "chain_end", ""),
        _field(solver, "tool_body", ""),
        _field(solver, "tcp_site", ""),
    )
def _annotate_rne_gravity(serial_chain_solvers, motions) -> None:
    """Derive the gravity vector an RNE solver is built with.

    The authored solver value is the Vereshchagin root acceleration, which ACHD takes as-is; KDL's
    inverse-dynamics solver wants the opposite sign, so the negation belongs wherever an RNE is
    constructed -- every backend that runs one, not just the simulated one.
    """
    for solver in list(serial_chain_solvers) + [
        s for motion in motions for s in _field(motion, "serial_chain_solvers", [])
    ]:
        root_acc = _field(solver, "root_acc")
        if root_acc:
            _set_field(solver, "gravity", [-component or 0.0 for component in root_acc])
def _annotate_runtime_robots(serial_chain_solvers, motions, backend: str) -> None:
    """Assign runtime_id/runtime_owner across solvers sharing a runtime and normalize empty tool
    fields.
    """
    runtime_by_signature: dict[tuple, str] = {}
    owner_by_runtime: dict[str, str] = {}
    solvers_by_id = {_field(solver, "id"): solver for solver in serial_chain_solvers}

    for solver in serial_chain_solvers:
        solver_id = _field(solver, "id", "")
        signature = _runtime_signature(solver, backend)
        runtime_id = runtime_by_signature.setdefault(signature, solver_id)
        owner_by_runtime.setdefault(runtime_id, solver_id)
        _set_field(solver, "runtime_id", runtime_id)
        _set_field(solver, "runtime_owner", solver_id == owner_by_runtime[runtime_id])
        # ST4's <if(x)> treats "" as truthy, so a bare robot's empty tool fields must be None.
        if not _field(solver, "tool_body"):
            _set_field(solver, "tool_body", None)
        if not _field(solver, "tcp_site"):
            _set_field(solver, "tcp_site", None)

    for motion in motions:
        for solver in _field(motion, "serial_chain_solvers", []):
            canonical = solvers_by_id.get(_field(solver, "id"))
            if canonical is None:
                continue
            _set_field(
                solver, "runtime_id", _field(canonical, "runtime_id") or _field(solver, "id", "")
            )
            _set_field(solver, "runtime_owner", _field(canonical, "runtime_owner", True))
        for command in _field(motion, "forwarded_commands", []):
            canonical = solvers_by_id.get(_field(command, "robot_id"))
            if canonical is not None:
                _set_field(command, "robot_id", _field(canonical, "runtime_id"))
# Producer and mirror expression declared in one place so they cannot drift; the template's
# joint-space-expr-<name> reads exactly what is named here. All chain joints are revolute
# (scene-dsl guarantees it), so position is an angle. `backends=None` means every backend carries
# the signal; naming backends restricts it to those that actually measure it.
JointSpaceChannel = collections.namedtuple(
    "JointSpaceChannel", ("name", "producer", "quantity_kind", "unit", "backends")
)
_JOINT_SPACE_CHANNELS = (
    JointSpaceChannel("q", "port", "Angle", "RAD", None),
    JointSpaceChannel("qd", "port", "AngularVelocity", "RAD_PER_SEC", None),
    JointSpaceChannel("qdd", "solver", "AngularAcceleration", "RAD_PER_SEC2", None),
    JointSpaceChannel("tau_ctrl", "solver", "Torque", "N_M", None),
    # robif2b reads eff_msr off the hardware; under mj_kdl torque control jnt_trq_msr mirrors
    # qfrc_actuator and is zero by construction, not by measurement.
    JointSpaceChannel("tau_msr", "sensor", "Torque", "N_M", ("robif2b",)),
)
# Emitted only where a torque limit is authored; that saturation is its producer, not the solver.
_JOINT_SPACE_CMD_CHANNEL = JointSpaceChannel("tau_cmd", "saturation", "Torque", "N_M", None)
def add_joint_space_logging(
    serial_chain_solvers, motions, shared_data: list, introspection: dict, backend: str, iris
) -> None:
    """Mirror each runtime's joint-space signals into shared_data so the frame log can carry them.

    Keyed by runtime, not by solver: these are the arm's ports and the command port is
    last-writer-wins, so a runtime-keyed slot records what the port received even when several
    solvers on one runtime run in the same tick. Keying by solver would multiply the field count
    by the number of motions to record the same ports.
    """
    quantities = introspection.setdefault("quantities", [])
    shared_ids = {_field(item, "id") for item in shared_data if _field(item, "id")}

    # The per-motion copies the templates render carry neither runtime_id nor kdl_joints:
    # derive from the top-level solvers, assign to both.
    copies_by_id: dict[str, list] = {}
    for motion in motions:
        for solver in _field(motion, "serial_chain_solvers", []) or []:
            copies_by_id.setdefault(_field(solver, "id"), []).append(solver)

    by_runtime: dict[str, list] = {}
    for solver in serial_chain_solvers:
        by_runtime.setdefault(_field(solver, "runtime_id") or _field(solver, "id"), []).append(
            solver
        )

    for runtime_id, solvers in by_runtime.items():
        joints = _field(solvers[0], "kdl_joints") or []
        if not joints:
            raise RuntimeError(
                f"joint-space logging: solver '{_field(solvers[0], 'id')}' has no kdl_joints; "
                "the shared ids are compile-time names, so a wrong joint count mislabels "
                "every channel"
            )
        # tau_cmd differs from tau_ctrl only where a limit clamps it, so that saturation is its
        # producer -- named only when the runtime carries exactly one.
        saturations = {
            _field(_field(solver, "torque_saturation"), "id")
            for solver in solvers
            if _field(solver, "torque_saturation")
        }
        available = tuple(
            channel
            for channel in _JOINT_SPACE_CHANNELS
            if channel.backends is None or backend in channel.backends
        )
        channels = available + ((_JOINT_SPACE_CMD_CHANNEL,) if saturations else ())
        producer_id = {
            "port": runtime_id,
            "sensor": runtime_id,
            "solver": _sole({_field(solver, "id") for solver in solvers}),
            "saturation": _sole(saturations),
        }

        ids_by_channel: dict[str, list] = {channel.name: [] for channel in channels}
        # Mirrors are keyed by runtime, so they derive from the runtime's solver node.
        runtime_iri = iris.iri_of(runtime_id) or iris.iri_of(_field(solvers[0], "id"))
        if runtime_iri is None:
            raise RuntimeError(
                f"joint-space logging: runtime '{runtime_id}' has no IRI to derive from"
            )
        for index, joint in enumerate(joints):
            for channel in channels:
                shared_id = f"{runtime_id}_{channel.name}_{_cpp_identifier(joint)}"
                if shared_id in shared_ids:
                    raise RuntimeError(
                        f"joint-space logging: id '{shared_id}' collides with an existing "
                        "shared value"
                    )
                shared_ids.add(shared_id)
                entry = {
                    "id": shared_id,
                    "type": "Quantity",
                    "quantity_kind": {"id": channel.quantity_kind, "type": "QuantityKind"},
                    "unit": {"id": channel.unit, "type": "Unit"},
                    "runtime": runtime_id,
                    "role": "joint_space",
                    "channel": channel.name,
                    "joint": joint,
                    # Declared at the mirror site, so the contract need not re-derive it.
                    "producer": {
                        "kind": channel.producer,
                        "id": producer_id[channel.producer],
                    },
                }
                shared_data.append(entry)
                quantities.append(dict(entry))
                iris.register(
                    shared_id,
                    runtime_iri,
                    f"{channel.name}-{_cpp_identifier(joint)}",
                    DerivedIriRegistry.DERIVATION,
                )
                ids_by_channel[channel.name].append({"id": shared_id, "index": index})

        # Every solver on the runtime mirrors the same ids: whichever motion is active writes them.
        samples = [
            {"id": sample["id"], "channel": channel.name, "index": sample["index"]}
            for channel in available
            for sample in ids_by_channel[channel.name]
        ]
        cmd_ids = ids_by_channel.get(_JOINT_SPACE_CMD_CHANNEL.name, [])
        for solver in solvers:
            cmd_samples = cmd_ids if _field(solver, "torque_saturation") else []
            for target in (solver, *copies_by_id.get(_field(solver, "id"), ())):
                _set_field(target, "joint_space_samples", samples)
                _set_field(target, "joint_space_cmd_samples", cmd_samples)
def _agent_home_positions(platform, solvers) -> dict:
    """Each agent's reset joint configuration, keyed the way `_config_key` names it. Authored beside
    the model rather than baked into a template, and with no default: a simulated deployment that
    states none is rejected, because a wrong home is silent and a missing one should not be.
    """
    import tomllib

    if not platform.get("simulated"):
        return {}
    owners = [s for s in solvers if _field(s, "runtime_owner")]
    if not owners:
        return {}
    config_path = platform.get("config")
    if not config_path:
        raise ValueError(
            "A simulated platform must declare `config: \"<file>.toml\"` in its exec-context, "
            "stating a [<agent>] home for every agent it drives."
        )
    resolved = Path(config_path)
    if not resolved.is_file():
        raise ValueError(f"{resolved} does not exist, but the exec-context declares it.")
    config = tomllib.loads(resolved.read_text())
    homes = {
        f"{alias}.{leaf}": [float(v) for v in entry["home"]]
        for alias, entries in config.items()
        if isinstance(entries, dict)
        for leaf, entry in entries.items()
        if isinstance(entry, dict) and entry.get("home")
    }
    missing = sorted(
        _field(s, "config_key") for s in owners if _field(s, "config_key") not in homes
    )
    if missing:
        raise ValueError(
            f"{resolved} states no home for {missing}. Every agent the model drives needs one; "
            "add a [<agent>] section with `home = [...]`."
        )
    return homes
