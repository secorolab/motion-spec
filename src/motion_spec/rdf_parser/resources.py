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
import sys
from collections import Counter
from dataclasses import dataclass, replace
from pathlib import Path
from typing import NamedTuple
from xml.etree import ElementTree

import tomllib
from motion_spec_dsl.rdf_parser.vocab import (
    AGN,
    ALGO_EXT,
    APP,
    CSTR,
    CSTR_HDL,
    ENV,
    EXEC,
    GEOM_COORD,
    GEOM_ENT,
    KC,
    KC_STAT,
    MAP,
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
from rdf_utils.models.geom_coord import (
    find_pose_path,
    get_pose_coords,
    get_transform_between_frames,
)
from rdf_utils.models.vocab import (
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_GEOM_TYPE_KGRAPH,
    URI_GEOM_TYPE_POSE,
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_TYPE_JOINT,
    URI_KC_TYPE_SERIAL,
)
from rdf_utils.namespace import NS_MM_KC_EXT, NS_MM_QUDT_QTY
from rdf_utils.uri import iri_is_descendant, iri_parent
from rdflib import Graph, URIRef
from rdflib.namespace import PROV, RDF, SDO
from scene_dsl.kdl_tree import build_kdl_trees
from scene_dsl.rdf.sensors import (
    CAMERA_TYPES,
    URI_SENS_PRED_CAMERA_KIND,
    URI_SENS_PRED_RESOLUTION_HEIGHT,
    URI_SENS_PRED_RESOLUTION_WIDTH,
    URI_SENS_TYPE_CAMERA,
)
from scene_dsl.rdf_parser.kinematics import body_of_frame, get_kinematic_mapping, root_bodies
from scene_dsl.rdf_parser.sensors import get_update_rate
from scene_dsl.rdf_parser.vocab import URI_BDD_PRED_ELEMS, URI_ROS_PRED_PACKAGE_NAME

from motion_spec.classes.base import dedupe_by_id
from motion_spec.classes.bindings import (
    CameraBinding,
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
    MjcfSceneFrame,
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
# What a quantity is ultimately stated on, and where a walk from a constraint stops.
_GEOMETRIC_ENTITY_TYPES = frozenset(
    {
        GEOM_ENT.Point,
        GEOM_ENT.Frame,
        GEOM_ENT.SimplicialComplex,
        GEOM_ENT.RigidBody,
        ENV.RigidObject,
    }
)
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
    cameras: list
    devices: list
    urdf: str
    prefix: str
    owned_trees: list
    serial_chain: object
    root_body: object
    chain_root: str
    tip: str
    tool_body: str
    tcp_frame: str
    attach_kind: str
    attach_name: str
    pos: list | None
    quat: list | None
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
    root = body_of_frame(anchor_frame(model), graph)
    from_root = _distances(adjacency, root)

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
        # A body no asset backs carries no site to bolt to -- it only says where its child
        # sits, which the placement composed against the anchor already accounts for.
        if parent_body not in modelled_bodies and owner(parent_body) is None:
            attachments[child_body] = ("World", "", child_frame, parent_body)
        elif child_body in modelled_bodies or owner(parent_body) != owner(child_body):
            attachments[child_body] = ("Site", local_name(parent_frame), child_frame, parent_body)

    return attachments, root


def _sensor_kind(model, sensor) -> str:
    """The sensor's kind as the IR names it; empty for a kind codegen does not model."""
    types = get_node_types(model.graph, sensor)
    return next((name for uri, name in SENSOR_KINDS.items() if uri in types), "")


def _cameras(model, hosted, runtime_prefix) -> list:
    """The cameras among an agent's hosted sensors, as the runtime renders them.

    Only rgb lowers: nothing downstream renders a depth image, so a depth camera is reported and
    dropped rather than generating code that cannot run.
    """
    graph = model.graph
    cameras = []
    for sensor in hosted:
        if URI_SENS_TYPE_CAMERA not in get_node_types(graph, sensor):
            continue
        if graph.value(sensor, URI_SENS_PRED_CAMERA_KIND) != CAMERA_TYPES["rgb"]:
            print(
                f"camera '{local_name(sensor)}' is not an rgb camera; not lowered", file=sys.stderr
            )
            continue
        cameras.append(
            CameraBinding(
                id=f"{runtime_prefix}{local_name(sensor)}",
                width=int(graph.value(sensor, URI_SENS_PRED_RESOLUTION_WIDTH).toPython()),
                height=int(graph.value(sensor, URI_SENS_PRED_RESOLUTION_HEIGHT).toPython()),
                rate_hz=get_update_rate(graph, ModelBase(node_id=sensor, graph=graph)),
                uri=str(sensor),
            )
        )
    return cameras


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
                (index, edge)
                for index, edge in enumerate(path)
                if iri_is_descendant(binding["tree"], edge[1])
                and not iri_is_descendant(binding["tree"], edge[0])
            ),
            None,
        )
        if boundary is None:
            continue
        index, boundary = boundary
        _parent_body, child_body, parent_frame, _child_frame, _joint = boundary
        entity = binding["entity"]
        child_name = local_name(child_body)
        prefix = child_name[: -len(entity)] if entity and child_name.endswith(entity) else ""
        attachments.append(
            (
                index,
                MjcfSceneAttachment(
                    id=local_name(binding["tree"]),
                    path=binding["path"],
                    attach_to=local_name(parent_frame),
                    attach_kind="Site",
                    prefix=prefix,
                ),
            )
        )

    return [attachment for _, attachment in sorted(attachments, key=lambda item: item[0])]


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
        attachment = attach_by_body.get(root_body, ("World", "", root_frame, None))
        attach_kind, attach_name, _frame, _parent = attachment
        position, orientation = _placement_of(model, attachment, anchor_frame(model))
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
                        frame=f"{runtime_prefix}{local_name(frame)}",
                        update_rate_hz=get_update_rate(
                            model.graph, ModelBase(node_id=sensor, graph=model.graph)
                        ),
                        config_key=_config_key(
                            model, sensor, agent, f"{runtime_prefix}{local_name(sensor)}"
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
                cameras=_cameras(model, hosted, runtime_prefix),
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
                tcp_frame=(
                    f"{runtime_prefix}{local_name(tip_frame)}"
                    if tip_binding is not root_binding
                    else ""
                ),
                attach_kind=attach_kind,
                attach_name=attach_name,
                pos=position,
                quat=orientation,
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
    HardwareBinding(urdf="", model="", tool_body="", tcp_frame=""),
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
        `(setups_by_node, ordered, trees)`: the setup for each robot's abstract agent node -- the
        target of a solver's `agn:of-agent` -- the same setups in declaration order, and every
        distinct scene tree, which the one world model is built from
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
        chain = chain_for_iri(trees, str(assembly.serial_chain))
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
                tree=chain["tree"],
                name=chain["name"],
                joints=chain["joints"],
                frames=chain["frames"],
                bodies=chain["bodies"],
                tip_segment=chain["tip_segment"],
                joint_segments=chain["joint_segments"],
                world_root=chain["world_root"],
                world_tip=chain["world_tip"],
                world_segments=chain["world_segments"],
            ),
            hardware=HardwareBinding(
                urdf=assembly.urdf,
                model=robot_model,
                tool_body=assembly.tool_body,
                tcp_frame=assembly.tcp_frame,
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

    return setups_by_node, ordered, trees


def tree_segments(setups) -> dict:
    """Every scene element the built trees carry, by IRI, named as the world model names it.

    A chain resolves the whole tree it is sliced from, not only the part it articulates, so a
    frame no chain reaches -- a fixed camera watching the scene -- resolves here just the same.
    """
    return {
        iri: segment
        for setup in setups.values()
        for iri, segment in setup.chain.world_segments.items()
    }


def _placed_on_chain(carrier, backend: str) -> tuple[tuple[str, bool], ...]:
    """What of one carrier the generated code asks forward kinematics for, and which authority
    answers: the one world model, or this chain's own numbering.

    Only these reach kinematics, so only these need placing. A record may name frames it never
    resolves through the chain -- a twist states the point it is taken about -- and placing
    those would reject a model over a frame nothing looks up.
    """
    if hasattr(carrier, "sensor_frame"):
        # A sensed wrench: read at the sensor, moved to the point the model asked for, and
        # expressed in the frame it asked to see it in. The simulator answers those by name from
        # its own scene, so only the driven arm resolves them through kinematics at all.
        if backend != "robif2b":
            return ()
        return (("sensor_frame", True), ("reference_point", True), ("as_seen_by", True))
    if hasattr(carrier, "attached_to"):
        # `f_ext` is indexed chain-relative, so this one stays the chain's own segment.
        return (("attached_to", False),)
    if hasattr(carrier, "subspace"):
        # An acceleration constraint whose direction is taken in a frame that moves with the arm.
        return (("as_seen_by", True),)
    if getattr(carrier, "type", "") == "VelocityTwist":
        # Chain FK answers the twist itself, in the chain's own root frame. A twist the model
        # asked to see in another frame is that same twist rotated, and the frame it rotates
        # into is read off the world model like any other posed frame.
        return (("of", False), ("as_seen_by", True))
    return (("of", getattr(carrier, "type", "") == "Pose"),)


def _place_on_chain(chain, item, owner: str, world_fk: bool) -> str | None:
    """Resolve where one frame, point or body sits, once, while generating.

    The generated code is handed a resolved index, never a name: a name would have to be matched
    at run time, and a frame that matched nothing would surface as a dead controller on the first
    tick instead of an error here. A world-model read resolves to the exact tree segment, which
    plan 04 gives every posed frame, so its constant offset needs no run-time math; a chain-local
    read still resolves to the chain's own segment index, which has no offset to compose onto.

    Returns:
        the full tree segment name for a world-model read, else None
    """
    # A scene object carries no segment to fill: where it is comes from the simulator's own
    # scene, not from this chain's kinematics.
    if item is None or not hasattr(item, "segment"):
        return None
    if getattr(item, "is_scene_object", False):
        return None
    placement = chain.frames.get(item.uri)
    if placement is None and item.uri in chain.bodies:
        placement = {"index": chain.bodies[item.uri], "offset": None}
    if placement is None:
        raise ConstraintViolation(
            "geometry",
            f"'{item.id}' is not on the chain '{chain.name}' that {owner} runs on, so no "
            f"forward kinematics reaches it. It has to be a frame or body the chain from "
            f"'{chain.root}' to '{chain.tip}' passes through.",
        )
    if not world_fk and placement["offset"] is not None:
        raise ConstraintViolation(
            "geometry",
            f"'{item.id}' sits at a pose on its body that no segment of '{chain.name}' stands "
            f"for. Composing that offset onto the forward kinematics is not implemented; give "
            f"the frame a segment of its own, by making it a chain endpoint or its body's root.",
        )
    if item.segment is None and placement["offset"] is None:
        item.segment = placement["index"]
    if not world_fk:
        return None
    segment = chain.world_segments.get(item.uri)
    if segment is None:
        raise ConstraintViolation(
            "geometry",
            f"'{item.id}' is on the chain '{chain.name}' that {owner} runs on, but the built "
            f"tree carries no segment standing for it, so the world model cannot be asked "
            f"where it is.",
        )

    return segment


def _place_solver_on_chain(solver, backend: str) -> list[dict]:
    """Place every frame, point and body the solver's generated code asks kinematics for.

    Returns:
        one `{solver_id, frame_id, segment_name}` record per world-model read this solver makes
    """
    carriers = [
        *solver.output,
        *(
            constraint
            for driver in solver.motion_drivers
            for constraint in (*driver.acceleration_constraint, *driver.cartesian_acceleration)
        ),
        *(force for driver in solver.motion_drivers for force in driver.cartesian_force),
    ]
    segment_by_frame: dict[str, str] = {}
    for carrier in carriers:
        for attribute, world_fk in _placed_on_chain(carrier, backend):
            item = getattr(carrier, attribute, None)
            segment = _place_on_chain(solver.chain, item, solver.id, world_fk)
            if segment is None:
                continue
            if segment_by_frame.setdefault(item.id, segment) != segment:
                raise ConstraintViolation(
                    "geometry",
                    f"'{item.id}' resolves to two segments of the tree solver '{solver.id}' "
                    f"runs on: '{segment_by_frame[item.id]}' and '{segment}'. One name cannot "
                    f"stand for two places in the world model.",
                )

    return [
        {"solver_id": solver.id, "frame_id": frame_id, "segment_name": segment}
        for frame_id, segment in sorted(segment_by_frame.items())
    ]


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
        on_this_chain = iri_parent(frame_body) in owned_trees
        runtime_frame = local_name(frame_body)
        if on_this_chain:
            runtime_frame = f"{runtime_prefix}{local_name(frame_body)}"
        # A twist seen by a frame this chain carries is this chain's to answer: it is the FK
        # twist rotated into a frame that moves with the arm, which only this chain can place.
        if runtime_frame != chain_root and type_ == GEOM_COORD.VelocityTwistCoordinate:
            return on_this_chain, frame_node

        return runtime_frame == chain_root, frame_node

    return True, frame_node


def _world_solver_outputs(model, setup: _ChainSetup, scene: MjcfSceneSpec) -> list:
    """The runtime observations stated in this solver's reference frame."""
    graph = model.graph
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
            scene_object = _scene_object_of(model, getattr(output, "of", None), scene)
            if scene_object is not None:
                output.of = scene_object
            outputs.append(output)

    return dedupe_by_id(outputs)


def _scene_object_of(model, of, scene: MjcfSceneSpec) -> SceneObject | None:
    """The scene object a pose is stated of, when the simulator's scene is what answers it.

    A pose of the object itself is the body's own frame; a pose of one of its other frames is
    the site the scene already marks that frame with. Either way the runtime reads it out of the
    scene rather than off a chain, which no forward kinematics of the arm reaches.

    Returns:
        the scene object with the marker to read, or None when the frame is not on one
    """
    if of is None or not getattr(of, "uri", ""):
        return None
    node = URIRef(of.uri)
    # The id a frame reports is its body's when it is that body's root, so the body is looked up
    # through the graph rather than by matching that id against a body name.
    body = (
        node
        if GEOM_ENT.RigidBody in get_node_types(model.graph, node)
        else quantities.body_of(model, node)
    )
    if body is None:
        return None
    object_id = next((obj.id for obj in scene.objects if obj.body == local_name(body)), None)
    if object_id is None:
        return None
    if quantities.placement_frame(model, body) == node:
        return SceneObject(object_id, local_name(body))
    site = next((frame.name for frame in scene.frames if frame.uri == str(node)), None)
    if site is None:
        raise ConstraintViolation(
            "geometry",
            f"'{model.id(node)}' is a frame of scene object '{object_id}', but the scene marks "
            f"no site for it, so the runtime cannot ask where it is. A frame is marked only "
            f"once a pose places it on its body.",
        )
    return SceneObject(object_id, local_name(body), site)


def _runtime_output(model, output, type_, node, frame_node, setup: _ChainSetup):
    """One observation with its frames and names rewritten the way the runtime knows them."""
    if type_ == KC_STAT.JointPositionCoordinate:
        return replace(output, joint_name=f"{setup.runtime.prefix}{output.joint_name}")
    if type_ == GEOM_COORD.VelocityTwistCoordinate:
        seen_by = _runtime_frame(model, frame_node, setup.runtime.prefix, setup.runtime.owned_trees)
        return replace(output, as_seen_by=seen_by, seen_by_root=seen_by.id == setup.chain.root)
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


def build_robots(
    model,
    schedule,
    setups,
    derivation,
    scene: MjcfSceneSpec,
    backend: str,
    detect_pose_ids=frozenset(),
) -> Robots:
    """Build every robot the program commands, and schedule the calls that drive them.

    Parameters:
        schedule: the active scope; the steps join `Robots.schedule_steps`
        setups: the chain setup per agent node, from `robot_setups`
        scene: the built scene, so an observation of one of its objects is named by that object
            and read off the marker the scene carries for it
        detect_pose_ids: the world poses a detect result writes; the kinematics do not compute
            them, so no chain lists them among its outputs

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
        solver.output = [
            out
            for out in dedupe_by_id([*solver.output, *_world_solver_outputs(model, setup, scene)])
            if out.id not in detect_pose_ids
        ]
        # An acceleration constraint is base-aligned when its axis frame is the chain root. Both
        # solver families ask: the row is a direction in the solver's frame either way, and one
        # taken in a frame that moves with the arm has to be turned into the root's axes.
        root_body = _body_name(solver.chain.root)
        for driver in solver.motion_drivers:
            for constraint in (*driver.acceleration_constraint, *driver.cartesian_acceleration):
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

    solver = SolverWithInputAndOutput(
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
        algorithm_name=family.codegen_name or None,
        derived_root_acceleration=quantities.parse_xyz(model, gravity_node)
        if gravity_node
        else None,
        torque_saturation=(
            constraint_handler.saturation(model, torque_limit) if torque_limit is not None else None
        ),
    )
    return solver


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
                "implemented for mj_kdl. A pose a detect act writes is exempt: it is produced "
                "by the perception result on every platform."
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
    anchor = anchor_frame(model)

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
        attachment = attach_by_body.get(body, ("World", "", body, None))
        attach_kind, attach_name, _frame, _parent = attachment
        position, orientation = _placement_of(model, attachment, anchor)
        scene.objects.append(
            MjcfSceneObject(
                id=local_name(obj),
                body=local_name(body),
                path=_asset_path(graph, asset),
                fixed=body in attach_by_body,
                attach_kind=attach_kind,
                attach_name=attach_name,
                pos=position,
                quat=orientation,
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
                pos=assembly.pos,
                quat=assembly.quat,
                attachments=assembly.attachments,
            )
        )
        scene.cameras.extend(assembly.cameras)

    scene.frames = _scene_frames(model)
    _expand_scene_geometry(scene)
    _validate_scene(scene, model.app_path)

    return scene


def _scene_frames(model) -> list:
    """Every frame the kgraph declares, placed on the body that carries it.

    A body's own root frame is where the body is, so it needs no marker of its own; the rest
    are posed against it, which is the frame the runtime builds each body on. A frame no pose
    leads to is left out rather than placed at the body's origin, which would invent a spot.
    """
    graph = model.graph
    marked = [
        (body, frame)
        for kgraph in sorted(graph.subjects(RDF.type, URI_GEOM_TYPE_KGRAPH), key=str)
        for body in sorted(_kgraph_bodies(model, kgraph), key=str)
        for frame in sorted(graph.objects(body, GEOM_ENT.simplices), key=str)
        if frame != quantities.placement_frame(model, body)
        and GEOM_ENT.Frame in get_node_types(graph, frame)
    ]
    # A site is named after its frame, and carries its body only when another body has a frame
    # of the same name -- the name has to be unique, and it has to stay readable in a viewer.
    counts = Counter(local_name(frame) for _body, frame in marked)

    frames = []
    for body, frame in marked:
        position, orientation = _placement(model, frame, quantities.placement_frame(model, body))
        if position is None:
            continue
        name = local_name(frame)
        frames.append(
            MjcfSceneFrame(
                body=local_name(body),
                name=f"{local_name(body)}_{name}" if counts[name] > 1 else name,
                uri=str(frame),
                **dict(zip(("pos_x", "pos_y", "pos_z"), position)),
                **dict(zip(("quat_x", "quat_y", "quat_z", "quat_w"), orientation)),
            )
        )
    return frames


def _kgraph_bodies(model, kgraph) -> set:
    """Every body the graph holds: the ones it roots, and everything a joint reaches."""
    graph = model.graph
    bodies = set(root_bodies(kgraph, graph))
    for joint in graph.subjects(RDF.type, URI_KC_TYPE_JOINT):
        for frame in graph.objects(joint, URI_KC_PRED_BETWEEN_ATTACHMENTS):
            bodies.add(body_of_frame(frame, graph))
    return {body for body in bodies if body is not None}


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
    from ament_index_python.packages import get_package_share_directory

    return str(Path(get_package_share_directory(str(package))) / path)


def anchor_frame(model):
    """The frame the scene stands on: its ground, and the origin every placement resolves into.

    The graph declares it, so nothing here guesses which of its roots the scene hangs from.
    """
    graph = model.graph
    anchors = {
        anchor
        for kgraph in graph.subjects(RDF.type, URI_GEOM_TYPE_KGRAPH)
        for anchor in graph.objects(kgraph, NS_MM_KC_EXT["anchor"])
    }
    if len(anchors) != 1:
        raise ConstraintViolation(
            "kinematics",
            f"the scene needs exactly one anchor to stand on, found {len(anchors)}"
            f"{': ' + ', '.join(sorted(map(str, anchors))) if anchors else ''}",
        )
    return anchors.pop()


def _frames_of(model, node) -> list:
    """The frames a body carries, or the node itself when it already is one."""
    if GEOM_ENT.Frame in get_node_types(model.graph, node):
        return [node]
    return [
        frame
        for frame in model.graph.objects(node, GEOM_ENT.simplices)
        if GEOM_ENT.Frame in get_node_types(model.graph, frame)
    ]


def _placement_of(model, attachment, anchor):
    """Where an attached body sits, in the frame of whatever it is bolted to.

    A body the scene places itself resolves against the anchor the runtime is built on. One
    bolted to another model resolves against that model's own root frame, which is where its
    site is: composing to the anchor instead would count its host's placement twice.
    """
    kind, _name, frame, parent = attachment
    if kind != "World":
        return _placement(model, frame, quantities.placement_frame(model, parent))
    # What the runtime places is the body, so its root frame -- not whichever of its frames a
    # joint happens to hang it by, which may sit anywhere on it.
    is_frame = GEOM_ENT.Frame in get_node_types(model.graph, frame)
    return _placement(model, body_of_frame(frame, model.graph) if is_frame else frame, anchor)


def _placement_graph(model):
    """The poses that place something, which is not every pose the graph relates.

    A context quantity relates two frames the scene has already placed: a world pose the run
    computes each cycle, a spec pose it aims at. Both are the shortest way between their frames,
    so a search left to walk them answers where a body sits with a target the arm is moving to,
    or with a coordinate that holds no value until the first cycle.
    """
    graph = model.cache.get("placement_graph")
    if graph is not None:
        return graph
    graph = Graph()
    for triple in model.graph.triples((None, None, None)):
        graph.add(triple)
    for relation in model.graph.subjects(RDF["type"], URI_GEOM_TYPE_POSE):
        if model.context_scope(relation) is not None:
            graph.remove((relation, None, None))
    model.cache["placement_graph"] = graph

    return graph


def _reject_sampled_placement(model, frame, wrt) -> None:
    """A sampled placement has no seed semantics here, so it must not resolve to one value.

    Composing reads whatever coordinates a sampled quantity happens to carry, which would put
    the body at one draw of a distribution and never say so.

    Raises:
        ConstraintViolation: a pose placing this frame is sampled.
    """
    graph = _placement_graph(model)
    for pose, coords in get_pose_coords(graph=graph, poses=find_pose_path(frame, wrt, graph) or []):
        for coord in coords:
            for node in (coord.id, coord.position_coord.id, coord.orientation_coord.id):
                if URI_DISTRIB_TYPE_SAMPLED_QUANTITY in get_node_types(model.graph, node):
                    raise ConstraintViolation(
                        "geometry",
                        f"sampled placement coordinate '{node}' places '{pose.id}': motion-spec "
                        f"has no seed for it, so it cannot be resolved to a single placement",
                    )


def _placement(model, node, wrt):
    """Where a body or frame sits in `wrt`, in metres and [x, y, z, w].

    Composed along the poses that place it, so a scene may author a placement against any
    frame it likes and still be read against the one it is assembled on. A body no pose
    leads to is placed by the joint that holds it, and comes back coincident.
    """
    frame = quantities.placement_frame(model, node)
    if frame is None:
        return None, None
    _reject_sampled_placement(model, frame, wrt)
    graph = _placement_graph(model)
    transform = get_transform_between_frames(frame, wrt, graph)
    if transform is None:
        # A pose reads one way, but it relates both frames: a scene that places a body's root
        # against one of its own frames still says where that frame is on the body.
        reverse = get_transform_between_frames(wrt, frame, graph)
        transform = reverse.inv() if reverse is not None else None
    if transform is None:
        return None, None
    return list(transform.translation), list(transform.rotation.as_quat())


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
        declared = sorted(
            name
            for element in ElementTree.parse(asset).iter("site")
            if (name := element.get("name"))
        )
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


def read_platform(model) -> dict:
    """The execution platform the model declares, as one record every consumer reads.

    The authored node's IRI, its name, whether it is simulated, the backend serving it and the
    deployment config beside it. Platform identity comes from here and is never re-derived from
    the backend token or from substrings of an id.

    Raises:
        ConstraintViolation: the model names a simulation platform with no backend, real-world
            execution constrains a scene object, binds a device no backend drives, or reads a
            sensor with no device bound.
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
    """Reject a scene object a constraint is stated on and nothing observes: on hardware its pose
    would come from a simulator that is not running.

    An object the scene only places -- the table the arm stands on, the furniture around it --
    is left alone: no run-time value is read off it. So is one a perception source observes,
    which is the case the simulator was standing in for.
    """
    if context is None:
        return
    graph = model.graph
    constrained = _constrained_entities(model)
    observed = set(graph.objects(None, SOSA.hasFeatureOfInterest))
    objects = sorted(
        local_name(modelled)
        for modelled in graph.subjects(RDF.type, ENV.ModelledObject)
        if (entities := _object_entities(model, modelled)) & constrained and not entities & observed
    )
    if objects:
        raise ConstraintViolation(
            "platform",
            f"real-world execution constrains scene object(s) ({', '.join(objects)}) that nothing "
            "observes: their poses come from a simulator, and nothing measures them on hardware. "
            "Subscribe to a perception channel that observes them, drop the constraint, or model "
            "the location as an authored frame.",
        )


def _constrained_entities(model) -> set:
    """Every geometric entity a constraint is stated on.

    A constraint names a quantity, and a quantity leads to its entities through the view,
    coordinate and relation nodes that stand between them -- so the walk follows every link and
    stops at the first entity, which is what keeps it out of the placement chain that puts that
    entity in the scene.
    """
    graph = model.graph
    queue = [
        quantity
        for constraint in graph.subjects(RDF.type, CSTR.Constraint)
        for quantity in graph.objects(constraint, CSTR.quantity)
    ]
    seen, entities = set(), set()
    while queue:
        node = queue.pop()
        if node in seen or not isinstance(node, URIRef):
            continue
        seen.add(node)
        if _GEOMETRIC_ENTITY_TYPES & get_node_types(graph, node):
            entities.add(node)
            continue
        queue.extend(graph.objects(node, None))
        # A view holds no frames of its own: they belong to the quantity it reads, which it
        # points at rather than being pointed at by.
        for view in graph.subjects(MAP.subobject, node):
            queue.extend(graph.objects(view, MAP.superobject))

    return entities


def _object_entities(model, modelled) -> set:
    """The nodes that stand for one scene object: the object, the bodies it maps, their frames."""
    graph = model.graph
    bodies = [
        body
        for asset in graph.objects(modelled, ENV["has-object-model"])
        for body, _entity in _model_mappings(model, asset, GEOM_ENT.RigidBody)
    ]

    return {
        graph.value(modelled, ENV["of-object"]),
        *bodies,
        *(frame for body in bodies for frame in graph.objects(body, GEOM_ENT.simplices)),
    }


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


# What answers an observation, per backend. A twist is derived by KDL from the solver's joint
# mirror on either backend; a joint position is read off the simulator on mj_kdl but off that
# mirror on robif2b. Everything else is answered by the world model or a sensor, so the loop can
# compute it whether or not a motion that reads it is running.
_STATE_ANSWERED = {"mj_kdl": {"VelocityTwist"}, "robif2b": {"VelocityTwist", "JointPosition"}}


def _split_outputs(solver, backend: str) -> None:
    """Split a solver's observations into the ones the loop answers and the ones its motion does."""
    from_state = _STATE_ANSWERED.get(backend, {"VelocityTwist"})
    solver.state_output = [out for out in solver.output if out.type in from_state]
    solver.world_output = [out for out in solver.output if out.type not in from_state]


def world_observations(serial_chains) -> list[dict]:
    """Every observation the loop answers, once, with the solver whose frame it is stated in.

    Two solvers driving one chain report the same value from the same root, so the first to
    claim an observation answers it for the run.
    """
    claimed: dict[str, dict] = {}
    for solver in serial_chains:
        for out in solver.world_output:
            claimed.setdefault(out.id, {"solver_id": solver.id, "out": out})
    return list(claimed.values())


def annotate_runtime(serial_chains, motions, backend: str) -> list[dict]:
    """Fold onto each solver what running it implies, once every solver is known.

    Which runtime it shares, whether it owns that runtime, which of its joint outputs a gripper
    device reports rather than the chain, where on its chain every frame it asks kinematics for
    sits, and -- last, so it sees the runtime flags -- the gravity an RNE solver is built with.

    Returns:
        every world-model read the program makes, as `{solver_id, frame_id, segment_name}`

    Raises:
        ConstraintViolation: a joint output falls outside the chain with no gripper bound to report
            it, a frame is named that the solver's chain never reaches, or a motion declares a
            read-only solver on a runtime torque-commanded elsewhere.
    """
    world_frames = []
    for solver in serial_chains:
        world_frames.extend(_place_solver_on_chain(solver, backend))

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
            solver.hardware.tcp_frame,
        )
        runtime_id = runtime_by_signature.setdefault(signature, solver.id)
        owner_by_runtime.setdefault(runtime_id, solver.id)
        solver.runtime.id = runtime_id
        solver.runtime.owner = solver.id == owner_by_runtime[runtime_id]
        # ST4's <if(x)> treats "" as truthy, so a bare robot's empty tool fields must be None.
        solver.hardware.tool_body = solver.hardware.tool_body or None
        solver.hardware.tcp_frame = solver.hardware.tcp_frame or None

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
        # Last, so it sees the outputs a gripper device took over: what the loop answers is
        # decided from the list as it finally stands.
        _split_outputs(solver, backend)
        _index_chain_joints(solver)
    _apply_runtime_to_motions(serial_chains, motions, commanding)
    health_index = 0
    for solver in serial_chains:
        if not solver.runtime.owner:
            continue
        for device in solver.devices:
            if device.kind == "RobotiqFT300s":
                device.health_index = health_index
                health_index += 1
    # What the model authored, passed to whatever solver it names exactly as written. The one
    # exception is the RNE pass a simulated ACHD run adds to gravity-compensate its command:
    # that pass wants the field, and the value beside it is the root acceleration ACHD takes,
    # so its opposite is derived here rather than being asked of the author twice.
    # Full solvers only: a slice resolves gravity through `solver_id`.
    for solver in serial_chains:
        if solver.derived_root_acceleration:
            solver.gravity = list(solver.derived_root_acceleration)
            solver.gravity_compensation = [
                -component or 0.0 for component in solver.derived_root_acceleration
            ]

    return world_frames


def annotate_device_dependencies(serial_chains, motions) -> None:
    """Fold each FT device's consuming motion indexes onto its runtime binding."""
    by_id = {solver.id: solver for solver in serial_chains}
    for solver in serial_chains:
        if not solver.runtime.owner:
            continue
        for device in solver.devices:
            if device.kind != "RobotiqFT300s":
                continue
            device.required_by_motion = [
                motion.index
                for motion in motions
                if any(
                    by_id[part.solver_id].runtime.id == solver.runtime.id
                    and device.drives in part.required_sensors
                    for part in motion.serial_chain_solvers
                )
            ]
            device.has_required_motions = bool(device.required_by_motion)


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


def _chain_joint_index(solver, joint_name: str) -> int | None:
    """Where a joint sits in the chain's joint array, or None when it is not on the chain.

    Outputs carry the joint runtime-scoped and an authored joint force carries it bare, so both
    spellings are tried; the exact name wins, so a prefix that is also part of a joint's own name
    cannot resolve to the wrong one.
    """
    joints = solver.chain.joints
    for candidate in (joint_name, joint_name.removeprefix(solver.runtime.prefix)):
        if candidate in joints:
            return joints.index(candidate)

    return None


def _index_chain_joints(solver) -> None:
    """Resolve every joint the generated code addresses to its index on this chain.

    The alternative is handing the name to the generated program and searching the built chain on
    the first tick, which can only fail where nothing can act on it -- and searching by name has
    to match loosely enough that it can answer with the wrong joint, which on the feed-forward
    path means torque on a joint the model never named.

    Raises:
        ConstraintViolation: a joint force names a joint the chain does not articulate, so there
            is no joint-array slot to add it to.
    """
    for out in solver.output:
        if getattr(out, "type", "") == "JointPosition":
            out.joint_index = _chain_joint_index(solver, out.joint_name)
    for driver in solver.motion_drivers:
        for force in driver.joint_force:
            force.joint_index = _chain_joint_index(solver, force.joint_name)
            if force.joint_index is None:
                raise ConstraintViolation(
                    "solver",
                    f"joint force '{force.id}' acts on joint '{force.joint_name}', which is not "
                    f"on solver '{solver.id}'s chain from '{solver.chain.root}' to "
                    f"'{solver.chain.tip}'. A joint force is added to that chain's joint torques, "
                    "so it has to name a joint the chain articulates.",
                )


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


def platform_config(platform: dict) -> dict:
    """The deployment config the exec-context declares, parsed once for every reader of it.

    Raises:
        RuntimeError: the declared config file does not exist.
    """
    config_path = platform.get("config")
    if not config_path:
        return {}
    resolved = Path(config_path)
    if not resolved.is_file():
        raise RuntimeError(f"{resolved} does not exist, but the exec-context declares it.")

    return tomllib.loads(resolved.read_text())


# The key an agent's reset configuration is stated under. Only a simulated run resets to it: a
# real arm is wherever it was left, so nothing reads this on hardware.
AGENT_HOME_KEY = "home"

# What a `[config.<key>]` pose section states. Checked at generation for presence and here for
# shape, and again before a run, which reads the numbers the deployment may have retuned since.
CONFIG_POSE_FIELDS = ("position", "orientation")


def agent_home_positions(platform: dict, serial_chains, config: dict) -> dict:
    """Each agent's reset joint configuration, keyed the way its config key names it.

    Authored beside the model rather than baked into a template, and with no default: a simulated
    deployment that states none is rejected, because a wrong home is silent and a missing one
    should not be.

    Raises:
        ConstraintViolation: a simulated platform declares no config, or the config states no home
            for an agent the model drives.
    """
    if not platform.get("simulated"):
        return {}
    owners = [solver for solver in serial_chains if solver.runtime.owner]
    if not owners:
        return {}
    if not platform.get("config"):
        raise ConstraintViolation(
            "platform",
            'A simulated platform must declare `config: "<file>.toml"` in its exec-context, '
            "stating a [<agent>] home for every agent it drives.",
        )
    homes = {
        f"{alias}.{leaf}": [float(value) for value in entry[AGENT_HOME_KEY]]
        for alias, entries in config.items()
        if isinstance(entries, dict)
        for leaf, entry in entries.items()
        if isinstance(entry, dict) and entry.get(AGENT_HOME_KEY)
    }
    missing = sorted(
        solver.runtime.config_key for solver in owners if solver.runtime.config_key not in homes
    )
    if missing:
        raise ConstraintViolation(
            "platform",
            f"{platform['config']} states no home for {missing}. Every agent the model drives "
            "needs one; add a [<agent>] section with `home = [...]`.",
        )

    return homes


def config_poses(model, config: dict) -> list[dict]:
    """Poses the deployment states rather than the model: what to fill and where to read it.

    A device's key is derived, because the element it realizes already names it. A pose's is
    authored -- the model writes `[config.<key>]`, and that key is the only statement of which
    section it means. Presence and shape are checked here and the numbers are not, so retuning
    a pose needs no regeneration.

    Raises:
        ConstraintViolation: the config states no section for a pose, or one that is not a
            position and an orientation of three numbers each.
    """
    entries = []
    for node in sorted(model.graph.subjects(EXEC["has-resource"], None), key=str):
        if EXEC.ExecutionContext in get_node_types(model.graph, node):
            continue
        key = str(model.graph.value(node, SDO.identifier))
        section = config
        for segment in key.split("."):
            section = section.get(segment, {}) if isinstance(section, dict) else {}
        if not section:
            raise ConstraintViolation(
                "platform",
                f"Pose '{local_name(node)}' reads [config.{key}], which the deployment config "
                f"does not state. Add a [{key}] section with `position = [x, y, z]` and "
                "`orientation = [rx, ry, rz]`.",
            )
        for field_name in CONFIG_POSE_FIELDS:
            values = section.get(field_name)
            if not isinstance(values, list) or len(values) != 3:
                raise ConstraintViolation(
                    "platform", f"[{key}] states no three-number `{field_name}`."
                )
        entries.append({"id": model.id(node), "config_key": key})

    return entries


# The deployment states the topic and the rate; the section's presence is what turns the
# publisher on, so retuning either needs no regeneration.
ROS_JOINT_STATES_KEY = "ros.joint_states"


def ros_joint_states(platform: dict, config: dict, serial_chains) -> dict | None:
    """The joint-state publisher the deployment asked for: the config key its topic and rate are
    read from at runtime, and per joint the blackboard values one message reports.

    Raises:
        ConstraintViolation: the config declares the section but the exec-context declares no
            config to read it from, or the model drives no serial chain to report.
    """
    # Presence is the switch; an empty section means "on, all defaults".
    if "joint_states" not in (config.get("ros") or {}):
        return None
    if not platform.get("config"):
        raise ConstraintViolation(
            "platform",
            f"[{ROS_JOINT_STATES_KEY}] needs the exec-context to declare `config:`; the topic "
            "and rate are read from that file at run time.",
        )
    joints = []
    for solver in serial_chains:
        if not solver.runtime.owner:
            continue
        by_channel: dict[str, dict[int, str]] = {}
        for sample in solver.joint_space_samples:
            by_channel.setdefault(sample["channel"], {})[sample["index"]] = sample["id"]
        # Chain joints are stored unprefixed; the runtime's own channels are scoped, so the
        # published names are too.
        for index, joint in enumerate(solver.chain.joints):
            joints.append(
                {
                    "name": f"{solver.runtime.prefix}{joint}",
                    "position": by_channel["q"][index],
                    "velocity": by_channel["qd"][index],
                    "effort": by_channel["tau_ctrl"][index],
                }
            )
        # A joint the chain does not articulate -- a gripper's driver joint -- is measured
        # elsewhere: under robif2b by the device `_split_gripper_outputs` moved it onto, under a
        # simulated backend by the simulator answering for its name. Its name is already
        # runtime-scoped, and only its position is read on either route: neither the serial nor
        # the interconnect gripper reports a joint velocity or effort, and a zero there would
        # claim the finger is standing still and unloaded.
        for out in [*solver.output, *(o for d in solver.devices for o in d.joint_outputs)]:
            if getattr(out, "type", "") == "JointPosition" and out.joint_index is None:
                joints.append({"name": out.joint_name, "position": out.id})
    if not joints:
        raise ConstraintViolation(
            "platform",
            f"[{ROS_JOINT_STATES_KEY}] is declared, but the model drives no serial chain whose "
            "joints it could report.",
        )

    return {"config_key": ROS_JOINT_STATES_KEY, "joints": joints}
