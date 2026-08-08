# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What the program commands, in C order.

Three chapters, in this order: **resources** -- the agents, their devices and sensors, the chain
setups and the solver records built on them; **composition** -- the scene, its bodies, frames,
kinematic adjacency and fixed attachments; **configuration** -- the execution platform, the
backend it implies, the trace settings and the agent home positions.

Configuration rides here rather than in a module of its own because nothing derives it: it is
read alongside the resources it configures. Each chapter validates what it reads, at the moment
it reads it -- a scene object on real hardware, an undriven device, an unbound sensor, a solver
the backend cannot run.
"""

from __future__ import annotations

import collections
from dataclasses import dataclass, replace
from pathlib import Path
from typing import NamedTuple

import tomllib
from motion_spec_dsl.rdf_parser.vocab import (
    AGN,
    ALGO_EXT,
    APP,
    CSTR_HDL,
    ENV,
    EXEC,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_REL,
    KC,
    KC_STAT,
    QUDT_SCHEMA,
    RBDYN_COORD,
    RBDYN_ENT,
    SENSORS,
    SLV,
    SLV_EXT,
    SOSA,
)
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import ModelBase, get_node_types
from rdf_utils.models.execution import URI_EXEC_PRED_PATH, get_path_of_node
from rdf_utils.models.vocab import URI_GEOM_PRED_OF, URI_GEOM_TYPE_POSITION, URI_KC_TYPE_SERIAL
from rdf_utils.namespace import NS_MM_KC_EXT, NS_MM_QUDT_QTY
from rdf_utils.uri import iri_is_descendant, iri_parent
from rdflib.namespace import PROV, RDF, SDO
from scene_dsl.kdl_tree import build_kdl_trees
from scene_dsl.rdf_parser.kinematics import body_of_frame, get_kinematic_mapping
from scene_dsl.rdf_parser.sensors import get_update_rate
from scene_dsl.rdf_parser.vocab import URI_BDD_PRED_ELEMS

from motion_spec.classes.base import dedupe_by_id
from motion_spec.classes.bindings import (
    ChainBinding,
    DeviceBinding,
    HardwareBinding,
    JointSpaceChannel,
    RuntimeBinding,
    SensorBinding,
)
from motion_spec.classes.geometry import SceneObject
from motion_spec.classes.motion import BlackboardValue
from motion_spec.classes.scene import (
    MjcfSceneAttachment,
    MjcfSceneObject,
    MjcfSceneRobot,
    MjcfSceneSpec,
)
from motion_spec.classes.solvers import (
    ForceDistributionSolver,
    SolverWithInputAndOutput,
    VelocityCompositionSolver,
)
from motion_spec.rdf_parser import constraint_handler, quantities
from motion_spec.rdf_parser.model import local_name, seconds
from motion_spec.rdf_parser.operations import OPS_GENERIC, OPS_SOLVER

SUPPORTED_ROBOT_MODELS = {"KinovaGen3"}
# Devices with driver templates. A name the grammar accepts but that is missing here is rejected.
DRIVEN_DEVICES = {"KinovaGen3", "KinovaGen3-2F85", "Robotiq2F85", "RobotiqFT300s"}
GRIPPER_DEVICES = {"KinovaGen3-2F85", "Robotiq2F85"}
# The sensor kinds the IR models, as the graph types them and as the templates dispatch on them.
SENSOR_KINDS = {SENSORS.ForceTorqueSensor: "ForceTorque"}
# Which codegen backend serves an authored simulation platform. The platform is the model's; the
# backend is an implementation detail of running it, so the mapping lives here and nowhere else.
SIMULATION_BACKENDS = {"mujoco": "mj_kdl"}
# The MuJoCo trajectory trace is a viewer-only overlay authored in the scene, not the motion spec;
# it is no longer part of this graph, so it stays disabled and costs nothing for headless or
# non-MuJoCo runtimes. Codegen guards on trace.enabled.
TRACE_DISABLED = {
    "enabled": False,
    "length": 4096,
    "color_r": 1.0,
    "color_g": 0.5,
    "color_b": 0.1,
    "color_a": 1.0,
    "targets": [],
    "has_targets": False,
}


def _model_mappings(model, asset, target_type) -> list:
    """The `(scene target, model entity)` mappings of one asset, of the requested RDF type.

    What a mapping means is its metamodel's to say, not ours.
    """
    mappings = (
        get_kinematic_mapping(mapping, model.graph)
        for mapping in sorted(model.graph.objects(asset, EXEC["has-mapping"]), key=str)
    )
    return [
        (mapping.target_id, mapping.entity or "")
        for mapping in mappings
        if mapping.target_type == target_type
    ]


def mapped_targets(model, model_type, target_type) -> set:
    """Every scene target of one type that models of `model_type` map onto."""
    return {
        target
        for asset in model.graph.subjects(RDF.type, model_type)
        for target, _entity in _model_mappings(model, asset, target_type)
    }


def _device_of(model, element):
    """The deployed system realizing an element, or None.

    Read backwards from the device, which the execution context owns, because the element it
    stands for belongs to the scene.
    """
    return next(iter(model.graph.subjects(EXEC["realizes"], element)), None)


def _config_key(model, element, agent, drives: str) -> str:
    """What `robot.toml` calls the device on an element.

    A hosted sensor is named by its owning agent's leaf plus its own, since `runtime_prefix`
    inside `drives` is empty on a single-robot model. An agent is named by the set the scene
    declares it in -- taken from the graph, not from an IRI segment, because the IRI path is
    namespace layout and the two disagree when the namespace is not named after the set.

    Raises:
        ConstraintViolation: an agent belongs to no declared agent set.
    """
    if drives:
        return f"{local_name(agent)}.{local_name(element)}"
    owner = next(model.graph.subjects(URI_BDD_PRED_ELEMS, element), None)
    if owner is None:
        raise ConstraintViolation("scene", f"Agent '{element}' belongs to no declared agent set.")
    return f"{local_name(owner)}.{local_name(element)}"


def _bound_devices(model, agent, runtime_prefix, hosted, chain_bindings, agent_by_tree) -> list:
    """The hardware bound on one chain: what each device is, where it is configured, what it drives.

    The kind is the authored name passed through untouched -- only the backend's templates
    interpret it. `drives` names the sensor a sensor device reads, and is empty for a joint mover.
    """

    def entry(node, drives=""):
        device = _device_of(model, node)
        if device is None:
            return None
        return DeviceBinding(
            kind=str(model.graph.value(device, SDO.model) or ""),
            config_key=_config_key(model, node, agent, drives),
            drives=drives,
        )

    owners = [agent]
    for binding in chain_bindings:
        owner = agent_by_tree.get(binding["tree"])
        # A tree may be bound by several models; the agent behind it is named once.
        if owner is not None and owner not in owners:
            owners.append(owner)
    found = [entry(owner) for owner in owners]
    found += [entry(sensor, f"{runtime_prefix}{local_name(sensor)}") for sensor in hosted]

    return [device for device in found if device is not None]


@dataclass(frozen=True)
class AgentAssembly:
    """One agent's runtime assets: the chain it drives, what is mounted on it, and where it sits.

    Builder-local machinery, not published -- but its fields are named after the bindings' own,
    so no third naming scheme survives: `urdf`, `owned_trees`, `tip`.
    """

    agent: object
    device: str
    config_key: str
    sensors: list
    devices: list
    urdf: str
    prefix: str
    owned_trees: list
    serial_chain: object
    root_body: object
    chain_root: str
    tip: str
    tool_body: str
    tcp_site: str
    attach_kind: str
    attach_name: str
    placement_frame: object
    attachments: list


def kinematic_adjacency(model):
    """The body graph the scene's joints induce, and the fixed joints among them.

    Returns:
        `(adjacency, fixed)`: per body, the `(neighbour, own frame, neighbour frame, joint)`
        edges it carries, and the frame pairs joined by a joint with no motion of its own
    """
    graph = model.graph
    adjacency = collections.defaultdict(list)
    fixed = []
    for joint in graph.subjects(RDF.type, KC.Joint):
        frames = list(graph.objects(joint, KC["between-attachments"]))
        if len(frames) != 2:
            continue
        body_a, body_b = (body_of_frame(frame, graph) for frame in frames)
        if body_a == body_b:
            continue
        adjacency[body_a].append((body_b, frames[0], frames[1], joint))
        adjacency[body_b].append((body_a, frames[1], frames[0], joint))
        if get_node_types(graph, joint) == {KC.Joint}:
            fixed.append((frames[0], frames[1]))

    return adjacency, fixed


def _distances(adjacency, source) -> dict:
    """Edge count from one body to every body reachable from it."""
    distances = {source: 0}
    queue = collections.deque([source])
    while queue:
        node = queue.popleft()
        for neighbor, *_ in adjacency[node]:
            if neighbor not in distances:
                distances[neighbor] = distances[node] + 1
                queue.append(neighbor)
    return distances


def body_path(adjacency, start, end) -> list:
    """The oriented body/frame edges on the shortest kinematic path from `start` to `end`.

    Returns:
        `(parent body, child body, parent frame, child frame, joint)` per step, empty when the
        two bodies are not connected
    """
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


def fixed_attachments(model, bound_trees):
    """Where the scene bolts one model to another, oriented from the world's root toward the tips.

    Returns:
        `(attachments, root)`: per attached body, the `(kind, name, frame, parent body)` the
        scene mounts it by, and the body the world is rooted at
    """
    graph = model.graph
    adjacency, fixed = kinematic_adjacency(model)
    if not fixed:
        return {}, None
    leaves = [body for body in adjacency if len(adjacency[body]) == 1]
    tip_distances = [
        _distances(adjacency, body_of_frame(tip, graph))
        for tip in graph.objects(None, NS_MM_KC_EXT["tip"])
    ]

    def distance_from_nearest_tip(body):
        return min((distance[body] for distance in tip_distances if body in distance), default=-1)

    root = max(leaves, key=distance_from_nearest_tip) if leaves else None
    from_root = _distances(adjacency, root) if root is not None else {}

    def owner(body):
        """The innermost bound tree owning a body: the longest IRI it descends from."""
        return next(
            (
                tree
                for tree in sorted(bound_trees, key=lambda item: (-len(str(item)), str(item)))
                if iri_is_descendant(tree, body)
            ),
            None,
        )

    modelled_bodies = mapped_targets(model, ENV["ObjectModel"], GEOM_ENT.RigidBody)
    attachments = {}
    for frame_a, frame_b in fixed:
        parent_frame, child_frame = (
            (frame_a, frame_b)
            if from_root.get(body_of_frame(frame_a, graph), 1 << 30)
            <= from_root.get(body_of_frame(frame_b, graph), 1 << 30)
            else (frame_b, frame_a)
        )
        parent_body, child_body = (body_of_frame(f, graph) for f in (parent_frame, child_frame))
        if parent_body == root:
            attachments[child_body] = ("World", "", child_frame, parent_body)
        elif child_body in modelled_bodies or owner(parent_body) != owner(child_body):
            attachments[child_body] = ("Site", local_name(parent_frame), child_frame, parent_body)

    return attachments, root


def _sensor_kind(model, sensor) -> str:
    """The sensor's kind as the IR names it; empty for a kind codegen does not model."""
    types = get_node_types(model.graph, sensor)
    return next((name for uri, name in SENSOR_KINDS.items() if uri in types), "")


def _agent_bindings(model):
    """Per modelled agent, the `(model, tree, path, entity)` rows its assets bind, and the agent
    each bound tree ultimately belongs to.
    """
    graph = model.graph
    bindings_by_modelled = {}
    for modelled in sorted(graph.subjects(RDF.type, AGN.ModelledAgent), key=str):
        rows = []
        for asset in sorted(graph.objects(modelled, AGN["has-agent-model"]), key=str):
            # Not `get_path_of_node`, which raises: a pathless agent model is metadata rather
            # than a runtime asset, and the assembly around it still reads.
            path = graph.value(asset, URI_EXEC_PRED_PATH)
            if not path:
                continue
            path = str(path)
            for tree, entity in _model_mappings(model, asset, GEOM_ENT.KinematicTree):
                rows.append({"model": asset, "tree": tree, "path": path, "entity": entity})
        bindings_by_modelled[modelled] = rows
    agent_by_tree = {
        row["tree"]: graph.value(modelled, AGN["of-agent"])
        for modelled, rows in bindings_by_modelled.items()
        for row in rows
    }

    return bindings_by_modelled, agent_by_tree


def _chain_attachments(model, path, root_binding, chain_bindings) -> list:
    """What the scene mounts partway along a chain, and where each boundary sits."""
    attachments = []
    for binding in chain_bindings:
        if binding is root_binding or binding["path"] == root_binding["path"]:
            continue
        boundary = next(
            (
                edge
                for edge in path
                if iri_is_descendant(binding["tree"], edge[1])
                and not iri_is_descendant(binding["tree"], edge[0])
            ),
            None,
        )
        if boundary is None:
            continue
        _parent_body, child_body, parent_frame, _child_frame, _joint = boundary
        entity = binding["entity"]
        child_name = local_name(child_body)
        prefix = child_name[: -len(entity)] if entity and child_name.endswith(entity) else ""
        attachments.append(
            MjcfSceneAttachment(
                id=local_name(binding["tree"]),
                path=binding["path"],
                attach_to=local_name(parent_frame),
                attach_kind="Site",
                prefix=prefix,
            )
        )

    return attachments


def _agent_assemblies(model, attach_by_body) -> list:
    graph = model.graph
    adjacency, _fixed = kinematic_adjacency(model)
    bound_model_trees = mapped_targets(model, AGN["AgentModel"], GEOM_ENT.KinematicTree)
    body_names_by_tree = {
        tree: {
            local_name(body)
            for body in graph.subjects(RDF.type, GEOM_ENT.RigidBody)
            if iri_is_descendant(tree, body)
        }
        for tree in bound_model_trees
    }
    serials = sorted(
        (
            (tree, graph.value(tree, NS_MM_KC_EXT["root"]), graph.value(tree, NS_MM_KC_EXT["tip"]))
            for tree in graph.subjects(RDF.type, URI_KC_TYPE_SERIAL)
        ),
        key=lambda item: str(item[0]),
    )
    bindings_by_modelled, agent_by_tree = _agent_bindings(model)
    bindings = [row for rows in bindings_by_modelled.values() for row in rows]

    result = []
    for modelled in sorted(graph.subjects(RDF.type, AGN.ModelledAgent), key=str):
        agent = graph.value(modelled, AGN["of-agent"])
        own = bindings_by_modelled[modelled]
        if agent is None or not own:
            continue

        def own_binding_for(node, own=own):
            return next(
                (binding for binding in own if iri_is_descendant(binding["tree"], node)), None
            )

        # A chain may span several agents; the agent owning its root drives it.
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
        tip_binding = next(
            (binding for binding in bindings if iri_is_descendant(binding["tree"], tip_frame)),
            root_binding,
        )
        root_body, tip_body = (body_of_frame(f, graph) for f in (root_frame, tip_frame))
        duplicate_root = (
            sum(local_name(root_body) in names for names in body_names_by_tree.values()) > 1
        )
        runtime_prefix = f"{local_name(root_binding['tree'])}_" if duplicate_root else ""
        path = body_path(adjacency, root_body, tip_body)
        # Scoped to this chain's path: two arms must not claim each other's models.
        chain_bodies = [root_body, tip_body, *(body for edge in path for body in edge[:2])]
        chain_bindings = [
            binding
            for binding in bindings
            if any(iri_is_descendant(binding["tree"], body) for body in chain_bodies)
        ]
        chain_tip_body = _chain_tip_body(root_binding, serial_tree, path, tip_body, root_body)

        hosted = sorted(graph.objects(modelled, SOSA.hosts), key=str)
        attach_kind, attach_name, placement_frame, _parent = attach_by_body.get(
            root_body, ("World", "", root_frame, None)
        )
        agent_device = _device_of(model, agent)
        result.append(
            AgentAssembly(
                agent=agent,
                device=str(graph.value(agent_device, SDO.model) or "") if agent_device else "",
                # An agent is named by the scenex alias it was referred to through, bound or not:
                # a simulated deployment addresses it the same way.
                config_key=_config_key(model, agent, agent, ""),
                sensors=[
                    SensorBinding(
                        id=f"{runtime_prefix}{local_name(sensor)}",
                        type=kind,
                        frame_site=f"{runtime_prefix}{local_name(frame)}",
                        update_rate_hz=get_update_rate(
                            model.graph, ModelBase(node_id=sensor, graph=model.graph)
                        ),
                        observes=sorted(
                            local_name(observed)
                            for observed in model.graph.objects(sensor, SOSA.observes)
                        ),
                    )
                    for sensor in hosted
                    if (kind := _sensor_kind(model, sensor))
                    and (frame := model.graph.value(sensor, SENSORS.frame)) is not None
                ],
                devices=_bound_devices(
                    model, agent, runtime_prefix, hosted, chain_bindings, agent_by_tree
                ),
                urdf=root_binding["path"],
                prefix=runtime_prefix,
                owned_trees=[binding["tree"] for binding in chain_bindings],
                serial_chain=serial_tree,
                root_body=root_body,
                chain_root=f"{runtime_prefix}{local_name(root_body)}",
                tip=f"{runtime_prefix}{local_name(chain_tip_body)}",
                tool_body=(
                    f"{runtime_prefix}{local_name(tip_body)}"
                    if tip_binding is not root_binding
                    else ""
                ),
                tcp_site=(
                    f"{runtime_prefix}{local_name(tip_frame)}"
                    if tip_binding is not root_binding
                    else ""
                ),
                attach_kind=attach_kind,
                attach_name=attach_name,
                placement_frame=placement_frame,
                attachments=_chain_attachments(model, path, root_binding, chain_bindings),
            )
        )

    return result


def _chain_tip_body(root_binding, serial_tree, path, tip_body, root_body):
    """Where this agent's own asset ends along the chain, which may be short of the chain's tip."""
    if root_binding["tree"] == serial_tree:
        return tip_body
    chain_tip_body = root_body
    for parent_body, child_body, *_ in path:
        if iri_is_descendant(root_binding["tree"], child_body):
            chain_tip_body = child_body
        elif iri_is_descendant(root_binding["tree"], parent_body):
            chain_tip_body = parent_body
            break

    return chain_tip_body


class _ChainSetup(NamedTuple):
    """One robot's chain setup, sourced from the scene-dsl graph: the bindings a solver's
    `chain`/`hardware`/`runtime`/`sensors`/`devices` are built from.
    """

    chain: ChainBinding
    hardware: HardwareBinding
    runtime: RuntimeBinding
    sensors: list
    devices: list


_EMPTY_SETUP = _ChainSetup(
    ChainBinding(root="", end="", tip="", tree="", name="", joints=[]),
    HardwareBinding(urdf="", model="", tool_body="", tcp_site=""),
    RuntimeBinding(id="", owner=False, prefix="", owned_trees=[], config_key=""),
    [],
    [],
)

# What an asset path says about the arm it holds, when no device is bound to say it outright.
_ROBOT_MODEL_HINTS = (("kinova_gen3", "KinovaGen3"), ("gen3", "KinovaGen3"))
# The arm-with-gripper pairing is wiring; the manipulator is still the arm.
_ROBOT_MODEL_BY_DEVICE = {"KinovaGen3-2F85": "KinovaGen3"}


def robot_setups(model):
    """Per-robot chain setups, sourced from the scene-dsl graph.

    Returns:
        `(setups_by_node, ordered)`: the setup for each robot's abstract agent node -- the target
        of a solver's `agn:of-agent` -- and the same setups in declaration order
    """
    from motion_spec.generation.scene_kdl import chain_for_iri

    try:
        trees = build_kdl_trees(model.graph)
    except ConstraintViolation:
        # Graph-only consumers may use an incomplete scene fixture: assembly metadata survives,
        # but there is no KDL chain until Scene DSL can parse it.
        trees = []
    bound_trees = mapped_targets(model, AGN["AgentModel"], GEOM_ENT.KinematicTree)
    attach_by_body, _root = fixed_attachments(model, bound_trees)

    setups_by_node, ordered = {}, []
    for assembly in _agent_assemblies(model, attach_by_body):
        chain_name, tree_name, joints = chain_for_iri(trees, str(assembly.serial_chain))
        # The arm is the authored device when one is bound, else sniffed from the asset path.
        if assembly.device:
            robot_model = _ROBOT_MODEL_BY_DEVICE.get(assembly.device, assembly.device)
        else:
            low = str(assembly.urdf).lower()
            robot_model = next((name for hint, name in _ROBOT_MODEL_HINTS if hint in low), "")
        setup = _ChainSetup(
            chain=ChainBinding(
                root=assembly.chain_root,
                end=assembly.tip,
                tip=assembly.tip,
                tree=tree_name,
                name=chain_name,
                joints=joints,
            ),
            hardware=HardwareBinding(
                urdf=assembly.urdf,
                model=robot_model,
                tool_body=assembly.tool_body,
                tcp_site=assembly.tcp_site,
            ),
            runtime=RuntimeBinding(
                id="",
                owner=False,
                prefix=assembly.prefix,
                owned_trees=assembly.owned_trees,
                config_key=assembly.config_key,
            ),
            sensors=assembly.sensors,
            devices=assembly.devices,
        )
        setups_by_node[assembly.agent] = setup
        ordered.append(setup)

    return setups_by_node, ordered


def _runtime_frame(model, frame_node, runtime_prefix, owned_trees):
    """A scene frame as the runtime names it, prefixed when this solver owns the tree it is in."""
    frame = quantities.frame(model, frame_node)
    tree = iri_parent(body_of_frame(frame_node, model.graph))
    return replace(frame, id=f"{runtime_prefix}{frame.id}" if tree in owned_trees else frame.id)


# The world-scoped coordinates a solver may observe, and the reader each is read with.
_WORLD_OUTPUTS = (
    (GEOM_COORD.PoseCoordinate, quantities.pose),
    (GEOM_COORD.VelocityTwistCoordinate, quantities.velocity_twist),
    (KC_STAT.JointPositionCoordinate, quantities.joint_position),
    (RBDYN_COORD.WrenchCoordinate, quantities.wrench),
)


def _observes_in_frame(model, node, type_, chain_root, runtime_prefix, owned_trees):
    """Whether an observation is stated in this solver's reference frame, and in which frame node."""
    graph = model.graph
    if type_ == KC_STAT.JointPositionCoordinate:
        joint = graph.value(node, KC_STAT["of-joint"])
        owned = joint is not None and any(iri_is_descendant(t, joint) for t in owned_trees)

        return owned, None

    seen_by = (
        RBDYN_COORD["as-seen-by"]
        if type_ == RBDYN_COORD.WrenchCoordinate
        else GEOM_COORD["as-seen-by"]
    )
    frame_node = graph.value(node, seen_by)
    if frame_node is None:
        frame_node = quantities.derived_reference_frames(model, node).as_seen_by
    if type_ == RBDYN_COORD.WrenchCoordinate:
        sensor = graph.value(node, SOSA.madeBySensor)
        sensor_frame = graph.value(sensor, SENSORS.frame) if sensor is not None else None
        owned = sensor_frame is not None and any(
            iri_is_descendant(tree, sensor_frame) for tree in owned_trees
        )

        return owned, frame_node
    if frame_node is not None:
        frame_body = body_of_frame(frame_node, graph)
        runtime_frame = local_name(frame_body)
        if iri_parent(frame_body) in owned_trees:
            runtime_frame = f"{runtime_prefix}{local_name(frame_body)}"

        return runtime_frame == chain_root, frame_node

    return True, frame_node


def _world_solver_outputs(model, setup: _ChainSetup, scene_objects) -> list:
    """The runtime observations stated in this solver's reference frame."""
    graph = model.graph
    object_ids_by_body = {obj.body: obj.id for obj in scene_objects}
    outputs = []
    for type_, read in _WORLD_OUTPUTS:
        for node in sorted(graph.subjects(RDF.type, type_), key=str):
            scope = model.context_scope(node)
            if scope is None or scope.section != "world":
                continue
            in_frame, frame_node = _observes_in_frame(
                model,
                node,
                type_,
                setup.chain.root,
                setup.runtime.prefix,
                setup.runtime.owned_trees,
            )
            if not in_frame:
                continue
            output = _runtime_output(model, read(model, node), type_, node, frame_node, setup)
            frame = getattr(output, "as_seen_by", None)
            if frame_node is None and frame is not None and frame.id != setup.chain.root:
                continue
            of = getattr(output, "of", None)
            if getattr(of, "id", None) in object_ids_by_body:
                output.of = SceneObject(object_ids_by_body[of.id], of.id)
            outputs.append(output)

    return dedupe_by_id(outputs)


def _runtime_output(model, output, type_, node, frame_node, setup: _ChainSetup):
    """One observation with its frames and names rewritten the way the runtime knows them."""
    if type_ == KC_STAT.JointPositionCoordinate:
        return replace(output, joint_name=f"{setup.runtime.prefix}{output.joint_name}")
    if type_ != RBDYN_COORD.WrenchCoordinate:
        return output

    graph = model.graph
    relation = graph.value(node, RBDYN_COORD["of-wrench"])
    reference_node = graph.value(relation, RBDYN_ENT["reference-point"])
    sensor = graph.value(node, SOSA.madeBySensor)
    sensor_frame_node = graph.value(sensor, SENSORS.frame)
    runtime = (setup.runtime.prefix, setup.runtime.owned_trees)

    return replace(
        output,
        sensor_name=f"{setup.runtime.prefix}{output.sensor_name}",
        sensor_frame=_runtime_frame(model, sensor_frame_node, *runtime),
        reference_point=replace(
            output.reference_point, id=_runtime_frame(model, reference_node, *runtime).id
        ),
        as_seen_by=_runtime_frame(model, frame_node, *runtime),
    )


def _body_name(name: str | None) -> str | None:
    """A frame or link name stripped to the bare body name the backend knows."""
    if name is None:
        return None
    for prefix in ("frame_", "frame-", "link_", "link-"):
        if name.startswith(prefix):
            return name[len(prefix) :]
    return name


@dataclass(frozen=True)
class Robots:
    """Every actuated resource the program commands, and the calls that drive them."""

    serial_chains: list
    platform_velocity: list
    platform_force: list
    schedule_steps: list

    @property
    def by_id(self) -> dict:
        """Every solver by its id: how a per-motion slice's `solver_id` is resolved."""
        return {
            solver.id: solver
            for solver in (*self.serial_chains, *self.platform_velocity, *self.platform_force)
        }


# Mobile-platform algorithms that are authorable but have no template wiring: the first needs a
# per-solver actuation-mode switch (the kelo command's control mode is fixed to force), the second
# a measured wheel torque the kelo struct does not expose. Rejected rather than silently dropped.
_UNIMPLEMENTED_PLATFORM_ALGORITHMS = (
    (SLV_EXT.VelocityDistributionSolver, "velocity-distribution"),
    (SLV_EXT.ForceCompositionSolver, "force-composition"),
)


def build_robots(model, schedule, setups, derivation, scene_objects, backend: str) -> Robots:
    """Build every robot the program commands, and schedule the calls that drive them.

    Parameters:
        schedule: the active scope; the steps join `Robots.schedule_steps`
        setups: the chain setup per agent node, from `robot_setups`
        scene_objects: the scene's objects, so an observation of one is named by the object

    Raises:
        ConstraintViolation: a mobile-platform solver names an algorithm with no codegen wiring.
        RuntimeError: a chain runs an unsupported robot model, or the backend cannot sync a
            scene-object pose the model asks for.
    """
    graph = model.graph
    steps = []
    # A solver whose agent the scene does not bind still needs a chain to talk about; the first
    # declared setup is the model's own answer to "which arm", and an empty one says there is none.
    default_setup = next(iter(setups.values()), _EMPTY_SETUP)

    platform_velocity = []
    for node in sorted(graph.subjects(RDF.type, SLV["VelocityCompositionSolver"]), key=str):
        model.expect_type(node, SLV["VelocityCompositionSolver"])
        platform_velocity.append(
            VelocityCompositionSolver(
                model.id(node),
                model.id(graph.value(node, SLV["configuration"])),
                quantities.velocity_twist(model, graph.value(node, SLV["velocity"])),
            )
        )
        steps.extend(schedule.of([node], OPS_GENERIC + OPS_SOLVER))

    serial_chains = []
    for node in _solver_nodes(model, derivation):
        setup = setups.get(graph.value(node, AGN["of-agent"]), default_setup)
        solver = _solver_with_input_and_output(model, node, setup)
        solver.motion_drivers = constraint_handler.motion_drivers(model, derivation, node)
        solver.output = dedupe_by_id(
            [*solver.output, *_world_solver_outputs(model, setup, scene_objects)]
        )
        # An acceleration constraint is base-aligned when its axis frame is the chain root.
        root_body = _body_name(solver.chain.root)
        for driver in solver.motion_drivers:
            for constraint in driver.acceleration_constraint:
                axis_frame = getattr(constraint.as_seen_by, "id", None)
                constraint.base_aligned = axis_frame is None or _body_name(axis_frame) == root_body
        serial_chains.append(solver)
        driven = graph[node : SLV["motion-drivers"] / (SLV["cartesian-force"] | SLV["joint-force"])]
        steps.extend(schedule.of(driven, OPS_GENERIC + OPS_SOLVER))

    platform_force = []
    for node in sorted(graph.subjects(RDF.type, SLV["ForceDistributionSolver"]), key=str):
        model.expect_type(node, SLV["ForceDistributionSolver"])
        platform_force.append(
            ForceDistributionSolver(
                model.id(node),
                model.id(graph.value(node, SLV["configuration"])),
                quantities.wrench(model, graph.value(node, SLV["force"])),
            )
        )
        steps.extend(schedule.of([node], OPS_GENERIC + OPS_SOLVER))

    for type_, label in _UNIMPLEMENTED_PLATFORM_ALGORITHMS:
        unsupported = sorted(graph.subjects(RDF.type, type_), key=str)
        if unsupported:
            raise ConstraintViolation(
                "solver",
                f"Mobile-platform solver '{unsupported[0]}' uses algorithm '{label}', which the "
                "motion-spec code generator does not implement yet. Use velocity-composition or "
                "force-distribution instead.",
            )
    _validate_solvers(serial_chains, backend)

    return Robots(serial_chains, platform_velocity, platform_force, steps)


def _solver_nodes(model, derivation) -> list:
    """Every chain solver, controllers' solvers first, in the order their handlers declare them."""
    graph = model.graph
    ordered = []
    for handler in sorted(
        graph.subjects(RDF.type, CSTR_HDL["ConstraintHandler"]),
        key=lambda node: int(getattr(graph.value(node, APP.order), "value", 0)),
    ):
        for plan in derivation.controllers_by_handler.get(handler, ()):
            if plan.solver not in ordered and SLV.SolverWithInputAndOutput in get_node_types(
                graph, plan.solver
            ):
                ordered.append(plan.solver)
    declared = set(graph.subjects(RDF.type, SLV.SolverWithInputAndOutput)) - set(ordered)

    return [*ordered, *sorted(declared, key=str)]


# The observation readers a chain solver's authored outputs dispatch over.
_SOLVER_OUTPUTS = (
    (GEOM_COORD["PoseCoordinate"], quantities.pose),
    (GEOM_COORD["VelocityTwistCoordinate"], quantities.velocity_twist),
    (KC_STAT["JointPositionCoordinate"], quantities.joint_position),
    (RBDYN_COORD["WrenchCoordinate"], quantities.wrench),
)


def _solver_with_input_and_output(model, node, setup: _ChainSetup) -> SolverWithInputAndOutput:
    """A chain solver as authored: its algorithm, its drivers, its outputs and its torque limit.

    `chain`/`hardware`/`runtime` have no default, so the bindings are constructor
    arguments rather than assigned after the fact; `replace()` gives each solver its own copies,
    since several solver nodes may share one agent's `setup`.
    """
    model.expect_type(node, SLV["SolverWithInputAndOutput"])
    graph = model.graph
    outputs = []
    for output_node in graph[node : SLV["output"]]:
        types = get_node_types(graph, output_node)
        read = next((func for type_, func in _SOLVER_OUTPUTS if type_ in types), None)
        if read is not None:
            outputs.append(read(model, output_node))

    family = constraint_handler.solver_algorithm(model, node)
    torque_limit = next(
        (
            limit
            for limit in graph.objects(node, ALGO_EXT.limits)
            if NS_MM_QUDT_QTY.Torque
            in graph[graph.value(limit, ALGO_EXT["in"]) : QUDT_SCHEMA.hasQuantityKind]
        ),
        None,
    )
    # The Vereshchagin root acceleration, passed to ACHD as-is; RNE's opposite-sign gravity is
    # derived in `annotate_runtime`.
    gravity_node = graph.value(node, SLV.gravity)

    return SolverWithInputAndOutput(
        id=model.id(node),
        motion_drivers=[
            constraint_handler.authored_motion_drivers(model, driver)
            for driver in graph[node : SLV["motion-drivers"]]
        ],
        output=outputs,
        chain=replace(setup.chain),
        hardware=replace(setup.hardware),
        runtime=replace(setup.runtime),
        sensors=setup.sensors,
        devices=setup.devices,
        algorithm=family,
        algorithm_name=family.codegen_name,
        derived_root_acceleration=quantities.parse_xyz(model, gravity_node)
        if gravity_node
        else None,
        torque_saturation=(
            constraint_handler.saturation(model, torque_limit) if torque_limit is not None else None
        ),
    )


def _validate_solvers(serial_chain_solvers, backend: str) -> None:
    """Reject unsupported robot models, and scene-object pose sync the backend cannot do."""
    unsupported = {
        solver.hardware.model
        for solver in serial_chain_solvers
        if solver.hardware.model and solver.hardware.model not in SUPPORTED_ROBOT_MODELS
    }
    if unsupported:
        raise RuntimeError(
            f"Unsupported robot model(s): {', '.join(sorted(unsupported))}. "
            f"Supported: {', '.join(sorted(SUPPORTED_ROBOT_MODELS))}"
        )
    if backend != "robif2b":
        return
    for solver in serial_chain_solvers:
        for out in solver.output:
            entity = getattr(out, "of", None)
            if getattr(out, "type", "") != "Pose" or not getattr(entity, "is_scene_object", False):
                continue
            raise RuntimeError(
                f"robif2b backend cannot sync scene-object pose output '{out.id}' for "
                f"'{entity.id or entity.body or out.id}'; world/scene object pose sync is only "
                "implemented for mj_kdl."
            )


def read_scene(model) -> MjcfSceneSpec:
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
    _name_object_attachments(model, attach_by_body)

    for modelled in sorted(graph.subjects(RDF.type, ENV["ModelledObject"]), key=str):
        obj = graph.value(modelled, ENV["of-object"])
        mapped = next(
            (
                (asset, body)
                for asset in sorted(graph.objects(modelled, ENV["has-object-model"]), key=str)
                for body, _entity in _model_mappings(model, asset, GEOM_ENT.RigidBody)
            ),
            None,
        )
        if obj is None or mapped is None:
            continue
        asset, body = mapped
        attach_kind, attach_name, placement_frame, _parent = attach_by_body.get(
            body, ("World", "", body, None)
        )
        scene.objects.append(
            MjcfSceneObject(
                id=local_name(obj),
                body=local_name(body),
                path=get_path_of_node(graph, asset),
                fixed=body in attach_by_body,
                attach_kind=attach_kind,
                attach_name=attach_name,
                pos=_placement_position(model, placement_frame),
                quat=_placement_orientation(model, placement_frame),
            )
        )

    for assembly in _agent_assemblies(model, attach_by_body):
        scene.robots.append(
            MjcfSceneRobot(
                id=local_name(assembly.agent),
                path=assembly.urdf,
                prefix=assembly.prefix,
                attach_kind=assembly.attach_kind,
                attach_name=assembly.attach_name,
                pos=_placement_position(model, assembly.placement_frame),
                quat=_placement_orientation(model, assembly.placement_frame),
                attachments=assembly.attachments,
            )
        )

    _expand_scene_geometry(scene)
    _validate_scene(scene)

    return scene


def _name_object_attachments(model, attach_by_body) -> None:
    """Rename an attachment onto a scene object after the object, as the runtime addresses it."""
    graph = model.graph
    object_ids_by_body = {
        body: local_name(obj)
        for modelled in graph.subjects(RDF.type, ENV["ModelledObject"])
        if (obj := graph.value(modelled, ENV["of-object"])) is not None
        for asset in graph.objects(modelled, ENV["has-object-model"])
        for body, _entity in _model_mappings(model, asset, GEOM_ENT.RigidBody)
    }
    for body, (kind, name, frame, parent_body) in list(attach_by_body.items()):
        if kind != "Site" or parent_body not in object_ids_by_body:
            continue
        parent_frame = model.child_node(parent_body, name)
        reference_frame = next(
            (
                reference
                for pose in graph.subjects(GEOM_REL.of, parent_frame)
                if (reference := graph.value(pose, GEOM_REL["with-respect-to"])) is not None
                and GEOM_ENT.Frame in get_node_types(graph, reference)
                and body_of_frame(reference, graph) == parent_body
            ),
            None,
        )
        if reference_frame is not None:
            name = local_name(reference_frame)
            frame = parent_frame
        attach_by_body[body] = (
            kind,
            f"{object_ids_by_body[parent_body]}_{name}",
            frame,
            parent_body,
        )


def _frames_of(model, node) -> list:
    """The frames a body carries, or the node itself when it already is one."""
    if GEOM_ENT.Frame in get_node_types(model.graph, node):
        return [node]
    return [
        frame
        for frame in model.graph.objects(node, GEOM_ENT.simplices)
        if GEOM_ENT.Frame in get_node_types(model.graph, frame)
    ]


def _placement_position(model, node) -> list[float] | None:
    """Where a body or frame is placed, in metres, from its authored scene pose."""
    for frame in _frames_of(model, node):
        origin = model.graph.value(frame, GEOM_ENT.origin) or frame
        for position_node in model.graph.subjects(URI_GEOM_PRED_OF, origin):
            if URI_GEOM_TYPE_POSITION not in get_node_types(model.graph, position_node):
                continue
            values = quantities.position_coordinate_values(model, position_node)
            if values is not None:
                return values
    return None


def _placement_orientation(model, node) -> list[float] | None:
    """How a body or frame is rotated, as [x, y, z, w], from its authored scene pose."""
    for frame in _frames_of(model, node):
        for orientation_node in model.graph.subjects(GEOM_REL.of, frame):
            if GEOM_REL.Orientation not in get_node_types(model.graph, orientation_node):
                continue
            rotation = quantities.orientation_relation_quaternion(model, orientation_node)
            if rotation is not None:
                return rotation
    return None


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
    values = getattr(item, name, None)
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


def _validate_scene(scene: MjcfSceneSpec) -> None:
    """Reject a procedural scene object that omits geometry: the model must declare it, and a
    silent default would place a body the author never described.
    """
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


def read_platform(model) -> dict:
    """The execution platform the model declares, as one record every consumer reads.

    The authored node's IRI, its name, whether it is simulated, the backend serving it and the
    deployment config beside it. Platform identity comes from here and is never re-derived from
    the backend token or from substrings of an id.

    Raises:
        ConstraintViolation: the model names a simulation platform with no backend, real-world
            execution uses scene objects, binds a device no backend drives, or reads a sensor with
            no device bound.
    """
    graph = model.graph
    simulation = next(graph.subjects(RDF.type, EXEC.Simulation), None)
    if simulation is None:
        real = next(graph.subjects(RDF.type, EXEC.RealWorld), None)
        _reject_scene_objects_on_hardware(model, real)
        _reject_undriven_devices(model, real)
        _reject_unbound_sensors_on_hardware(model, real)

        return {
            "uri": str(real) if real is not None else None,
            "name": None,
            "simulated": False,
            "backend": "robif2b",
            "config": str(_config_path(model, real)) if real is not None else None,
        }

    name = str(graph.value(simulation, SDO.name) or "")
    backend = SIMULATION_BACKENDS.get(name.casefold())
    if backend is None:
        raise ConstraintViolation("platform", f"Unsupported simulation platform '{name}'.")

    return {
        "uri": str(simulation),
        "name": name,
        "simulated": True,
        "backend": backend,
        "config": _config_path(model, simulation),
    }


def _config_path(model, context) -> str | None:
    """The deployment config's path, resolved by the DSL against the model that declares it."""
    config = model.graph.value(context, EXEC["has-resource"])
    return str(model.graph.value(config, EXEC.path)) if config is not None else None


def _reject_undriven_devices(model, context) -> None:
    """Reject a bound device the backend would silently ignore.

    The grammar decides what a model may name; this decides what the backend can drive.
    """
    if context is None:
        return
    bound = sorted(
        {
            str(model.graph.value(device, SDO.model) or "")
            for device in model.graph.subjects(EXEC["realizes"], None)
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


def _reject_scene_objects_on_hardware(model, context) -> None:
    """Reject scene objects on hardware: nothing measures their pose without perception."""
    if context is None:
        return
    objects = sorted(
        local_name(node) for node in model.graph.subjects(RDF.type, ENV.ModelledObject)
    )
    if objects:
        raise ConstraintViolation(
            "platform",
            f"real-world execution cannot use scene objects ({', '.join(objects)}): their poses "
            "come from a simulator, and nothing measures them on hardware. Remove them, or model "
            "the location as an authored frame.",
        )


def _reject_unbound_sensors_on_hardware(model, context) -> None:
    """Reject a sensor a model reads from but binds no device to.

    In simulation the simulator answers for every sensor; on hardware a reading comes from a
    device or from uninitialised memory.
    """
    if context is None:
        return
    unbound = sorted(
        local_name(sensor)
        for sensor in set(model.graph.objects(None, SOSA.madeBySensor))
        if _device_of(model, sensor) is None
    )
    if unbound:
        raise ConstraintViolation(
            "platform",
            f"real-world execution reads sensor(s) with no device bound: {', '.join(unbound)}. "
            "Bind one in the platform block, or stop reading the sensor.",
        )


def shared_runtime_members(model, serial_chains, control_period_ns: int, platform_uri) -> list:
    """The shared values the runtime writes that no model entity declares.

    The measured control period and the clock beside it, plus the tare state kept alongside every
    force/torque reading. Each is a contracted shared value: written from a port or computed from
    a reading, and read like any other.

    Raises:
        RuntimeError: the platform or a sensor output has no IRI to derive the value from.
    """
    if not platform_uri:
        raise RuntimeError("measured dt: the execution platform has no IRI to derive a clock from")
    # Which clock it is is the platform's to say: the port derives from the exec context.
    clock_iri = model.register_derived("clock", platform_uri, "clock", PROV.wasDerivedFrom)
    for name in ("clock_time_s", "dt_measured_s"):
        model.register_derived(name, clock_iri, name, PROV.wasDerivedFrom)
    members = [
        BlackboardValue(id="clock_time_s", type="Quantity"),
        BlackboardValue(id="dt_measured_s", type="Quantity", value=control_period_ns * 1e-9),
    ]

    seen = set()
    for solver in serial_chains:
        for out in solver.output:
            if getattr(out, "type", None) != "Wrench" or not out.sensor_name or out.id in seen:
                continue
            seen.add(out.id)
            # The tare state is computed from the reading, so it derives from that output's node.
            sensor_iri = model.iri_of(out.id)
            if sensor_iri is None:
                raise RuntimeError(
                    f"ft tare state: sensor output '{out.id}' has no IRI to derive from"
                )
            for suffix, member_type in (("ft_bias", "Wrench"), ("ft_settle", "IntCounter")):
                member_id = f"{out.id}_{suffix}"
                members.append(BlackboardValue(id=member_id, type=member_type))
                model.register_derived(member_id, sensor_iri, suffix, PROV.wasDerivedFrom)

    return members


def annotate_runtime(serial_chains, motions, backend: str) -> None:
    """Fold onto each solver what running it implies, once every solver is known.

    Which runtime it shares, whether it owns that runtime, which of its joint outputs a gripper
    device reports rather than the chain, and -- last, so it sees the runtime flags -- the gravity
    an RNE solver is built with.

    Raises:
        ConstraintViolation: a joint output falls outside the chain with no gripper bound to report
            it, or a motion declares a read-only solver on a runtime torque-commanded elsewhere.
    """
    runtime_by_signature: dict[tuple, str] = {}
    owner_by_runtime: dict[str, str] = {}
    for solver in serial_chains:
        # Two solvers are one runtime when they drive the same chain with the same tool.
        signature = (
            backend,
            solver.hardware.model,
            solver.hardware.urdf,
            solver.chain.root,
            solver.chain.tip or solver.chain.end,
            solver.hardware.tool_body,
            solver.hardware.tcp_site,
        )
        runtime_id = runtime_by_signature.setdefault(signature, solver.id)
        owner_by_runtime.setdefault(runtime_id, solver.id)
        solver.runtime.id = runtime_id
        solver.runtime.owner = solver.id == owner_by_runtime[runtime_id]
        # ST4's <if(x)> treats "" as truthy, so a bare robot's empty tool fields must be None.
        solver.hardware.tool_body = solver.hardware.tool_body or None
        solver.hardware.tcp_site = solver.hardware.tcp_site or None

    # Runtimes some driver torque-streams; declared-only solvers on any other runtime stage zeros.
    commanding = {
        solver.runtime.id
        for solver in serial_chains
        if any(
            driver.acceleration_constraint
            or driver.cartesian_force
            or driver.cartesian_acceleration
            or driver.joint_force
            for driver in solver.motion_drivers
        )
    }
    for solver in serial_chains:
        _split_gripper_outputs(solver, backend)
    _apply_runtime_to_motions(serial_chains, motions, commanding)
    # The authored solver value is the Vereshchagin root acceleration, which ACHD takes as-is;
    # KDL's inverse-dynamics solver wants the opposite sign, so every backend running an RNE
    # negates it here. Full solvers only: a slice resolves gravity through `solver_id`.
    for solver in serial_chains:
        if solver.derived_root_acceleration:
            solver.gravity = [-component or 0.0 for component in solver.derived_root_acceleration]


def _split_gripper_outputs(solver, backend: str) -> None:
    """Move a joint the chain does not articulate onto the gripper device that reports it.

    Only where a device is the reporter: the robif2b arm cannot measure a mimic joint, so the
    bound gripper answers for it and the joint lands on that device's own `joint_outputs`. A
    simulated backend measures every joint by name, so there the output stays an ordinary
    solver output and no device carries it.
    """
    if backend != "robif2b":
        return
    # Chain joints are published unprefixed; the outputs' joint names arrive runtime-scoped.
    chain_joints = {f"{solver.runtime.prefix}{joint}" for joint in solver.chain.joints}
    outputs, gripper_outputs = [], []
    for out in solver.output:
        joint = str(getattr(out, "joint_name", ""))
        if getattr(out, "type", "") != "JointPosition" or joint in chain_joints:
            outputs.append(out)
            continue
        gripper_outputs.append(out)
        if not any(device.kind in GRIPPER_DEVICES for device in solver.devices):
            raise ConstraintViolation(
                "solver",
                f"joint-position '{out.id}' reads joint '{joint}', which is outside solver "
                f"'{solver.id}'s chain and no gripper device is bound to report it",
            )
    solver.output = outputs
    for device in solver.devices:
        if device.kind in GRIPPER_DEVICES:
            device.joint_outputs = gripper_outputs


def _apply_runtime_to_motions(serial_chains, motions, commanding) -> None:
    """Copy each runtime's outputs onto the per-motion slices of it.

    The slice no longer copies `runtime_id`/`runtime_owner`: those are reached through
    `solver_id` once a template resolves the full solver.
    """
    by_id = {solver.id: solver for solver in serial_chains}
    for motion in motions:
        for solver in motion.serial_chain_solvers:
            canonical = by_id.get(solver.id)
            if canonical is None:
                continue
            solver.output = canonical.output
            solver.gripper_joint_outputs = [
                out
                for device in canonical.devices
                if device.kind in GRIPPER_DEVICES
                for out in device.joint_outputs
            ]
            # A read-only solver on a torque-streamed runtime would stage zero torques while
            # active (the arm drops) -- and skipping the stage would leave stale torques applied.
            if solver.read_only and canonical.runtime.id in commanding:
                raise ConstraintViolation(
                    "solver",
                    f"motion '{motion.id}' declares solver '{solver.id}' without any controller, "
                    f"but runtime '{canonical.runtime.id}' is torque-commanded elsewhere; drive "
                    "the solver in every motion or in none",
                )
        for command in motion.forwarded_commands:
            canonical = by_id.get(command.robot_id)
            if canonical is not None:
                command.robot_id = canonical.runtime.id


JOINT_SPACE_CHANNELS = (
    JointSpaceChannel("q", "port", "Angle", "RAD"),
    JointSpaceChannel("qd", "port", "AngularVelocity", "RAD_PER_SEC"),
    JointSpaceChannel("qdd", "solver", "AngularAcceleration", "RAD_PER_SEC2"),
    JointSpaceChannel("tau_ctrl", "solver", "Torque", "N_M"),
    # robif2b reads eff_msr off the hardware; under mj_kdl torque control jnt_trq_msr mirrors
    # qfrc_actuator and is zero by construction, not by measurement.
    JointSpaceChannel("tau_msr", "sensor", "Torque", "N_M", ("robif2b",)),
)
# Emitted only where a torque limit is authored; that saturation is its producer, not the solver.
JOINT_SPACE_COMMAND_CHANNEL = JointSpaceChannel("tau_cmd", "saturation", "Torque", "N_M")


def agent_home_positions(platform: dict, serial_chains) -> dict:
    """Each agent's reset joint configuration, keyed the way its config key names it.

    Authored beside the model rather than baked into a template, and with no default: a simulated
    deployment that states none is rejected, because a wrong home is silent and a missing one
    should not be.

    Raises:
        ConstraintViolation: a simulated platform declares no config, or the config states no home
            for an agent the model drives.
        RuntimeError: the declared config file does not exist.
    """
    if not platform.get("simulated"):
        return {}
    owners = [solver for solver in serial_chains if solver.runtime.owner]
    if not owners:
        return {}
    config_path = platform.get("config")
    if not config_path:
        raise ConstraintViolation(
            "platform",
            'A simulated platform must declare `config: "<file>.toml"` in its exec-context, '
            "stating a [<agent>] home for every agent it drives.",
        )
    resolved = Path(config_path)
    if not resolved.is_file():
        raise RuntimeError(f"{resolved} does not exist, but the exec-context declares it.")
    config = tomllib.loads(resolved.read_text())
    homes = {
        f"{alias}.{leaf}": [float(value) for value in entry["home"]]
        for alias, entries in config.items()
        if isinstance(entries, dict)
        for leaf, entry in entries.items()
        if isinstance(entry, dict) and entry.get("home")
    }
    missing = sorted(
        solver.runtime.config_key for solver in owners if solver.runtime.config_key not in homes
    )
    if missing:
        raise ConstraintViolation(
            "platform",
            f"{resolved} states no home for {missing}. Every agent the model drives needs one; "
            "add a [<agent>] section with `home = [...]`.",
        )

    return homes
