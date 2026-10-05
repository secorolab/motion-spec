# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The agents a program commands: their devices, sensors and cameras, the chain each is set up on,
and the solver records built on those chains.

Each reader validates what it reads, at the moment it reads it: an agent in no declared set, a
camera with two providers, a solver no handler runs, a robot model the backend cannot drive.
"""

from __future__ import annotations

import collections
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from typing import NamedTuple
from urllib.parse import unquote, urlsplit

from motion_spec_dsl.rdf_parser.vocab import (
    AGN,
    ALGO_EXT,
    APP,
    CSTR_HDL,
    CSTR_HDL_EXT,
    ENV,
    EST,
    EXEC,
    GEOM_COORD,
    GEOM_ENT,
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
from rdf_utils.models.execution import URI_EXEC_PRED_PATH
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_OF_POSE,
    URI_GEOM_PRED_WRT,
    URI_KC_TYPE_SERIAL,
    URI_QUDT_PRED_UNIT,
    URI_QUDT_PRED_VALUE,
)
from rdf_utils.namespace import NS_MM_KC_EXT, NS_MM_QUDT_QTY
from rdf_utils.naming import get_valid_var_name
from rdf_utils.uri import iri_is_descendant, iri_parent
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import RDF, SDO
from scene_dsl.kdl_tree import build_kdl_trees
from scene_dsl.rdf.sensors import (
    CAMERA_TYPES,
    URI_SENS_PRED_CAMERA_KIND,
    URI_SENS_PRED_FIELD_OF_VIEW,
    URI_SENS_PRED_FRAME,
    URI_SENS_PRED_RESOLUTION_HEIGHT,
    URI_SENS_PRED_RESOLUTION_WIDTH,
    URI_SENS_TYPE_CAMERA,
)
from scene_dsl.rdf_parser.common import ensure_one_typed_subject_uri
from scene_dsl.rdf_parser.kinematics import (
    body_of_frame,
    get_kinematic_mapping,
    kinematic_trees,
    root_frame_of,
)
from scene_dsl.rdf_parser.sensors import get_update_rate
from scene_dsl.rdf_parser.vocab import NS_MM_ROS, URI_BDD_PRED_ELEMS

from motion_spec.classes.base import unique_by_id
from motion_spec.classes.bindings import (
    CameraBinding,
    ChainBinding,
    DeviceBinding,
    HardwareBinding,
    RuntimeBinding,
    SensorBinding,
)
from motion_spec.classes.geometry import (
    Frame,
    Orientation,
    Point,
    Pose,
    Position,
    SimplicialComplex,
    SpatialCoordinate,
    VelocityTwist,
    Wrench,
)
from motion_spec.classes.scene import MjcfSceneAttachment
from motion_spec.classes.solvers import (
    AccelerationConstraint,
    CartesianForceSpecification,
    ForceDistributionSolver,
    SolverWithInputAndOutput,
    VelocityCompositionSolver,
)
from motion_spec.rdf_parser import constraint_handler, quantities
from motion_spec.rdf_parser.model import local_name, reader
from motion_spec.rdf_parser.operations import OPS_GENERIC, OPS_SOLVER

SUPPORTED_ROBOT_MODELS = {"KinovaGen3"}
# Devices with driver templates. A name the grammar accepts but that is missing here is rejected.
DRIVEN_DEVICES = {"KinovaGen3", "KinovaGen3-2F85", "Robotiq2F85", "RobotiqFT300s"}
GRIPPER_DEVICES = {"KinovaGen3-2F85", "Robotiq2F85"}
# The sensor kinds the IR models, as the graph types them and as the templates dispatch on them.
SENSOR_KINDS = {SENSORS.ForceTorqueSensor: "ForceTorque"}


def model_mappings(model, asset, target_type) -> list:
    """The `(scene target, model entity)` mappings of one asset, of the requested RDF type.

    What a mapping means is its metamodel's to say, not ours.
    """
    mappings = (
        get_kinematic_mapping(mapping, model.graph)
        for mapping in model.graph.objects(asset, EXEC["has-mapping"])
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
        for target, _entity in model_mappings(model, asset, target_type)
    }


def device_of(model, element):
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

    owners = [agent]
    for binding in chain_bindings:
        owner = agent_by_tree.get(binding["tree"])
        # A tree may be bound by several models; the agent behind it is named once.
        if owner is not None and owner not in owners:
            owners.append(owner)
    bound = [(owner, "") for owner in owners]
    bound += [(sensor, f"{runtime_prefix}{local_name(sensor)}") for sensor in hosted]
    devices = []
    for node, drives in bound:
        device = device_of(model, node)
        if device is not None:
            devices.append(
                DeviceBinding(
                    kind=str(model.graph.value(device, SDO.model) or ""),
                    config_key=_config_key(model, node, agent, drives),
                    drives=drives,
                )
            )
    return devices


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
    """Per body, the `(neighbour, own frame, neighbour frame, joint)` edges the scene's joints
    induce."""
    graph = model.graph
    adjacency = collections.defaultdict(list)
    for joint in graph.subjects(RDF.type, KC.Joint):
        frames = list(graph.objects(joint, KC["between-attachments"]))
        if len(frames) != 2:
            continue
        body_a, body_b = (body_of_frame(frame, graph) for frame in frames)
        if body_a == body_b:
            continue
        adjacency[body_a].append((body_b, frames[0], frames[1], joint))
        adjacency[body_b].append((body_a, frames[1], frames[0], joint))

    return adjacency


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
    """Where the scene bolts one model to another, oriented as scene-dsl's tree walk directs it.

    Returns:
        `(attachments, root)`: per attached body, the `(kind, name, frame, parent body)` the
        scene mounts it by, and the body the world is rooted at
    """
    graph = model.graph
    # (parent frame, child frame) per joint with no motion of its own.
    fixed = [
        (tree.joints[joint].frame_on(tree.parent[child]), tree.joints[joint].frame_on(child))
        for tree in kinematic_trees(scene_graph(model))
        for child, joint in tree.parent_joint.items()
        if get_node_types(graph, joint) == {KC.Joint}
    ]
    if not fixed:
        return {}, None
    root = body_of_frame(quantities.anchor_frame(model), graph)

    modelled_bodies = mapped_targets(model, ENV["ObjectModel"], GEOM_ENT.RigidBody)
    # Who holds whom, over the fixed joints alone, so a body can be followed up towards the root.
    held_by = {}
    for parent_frame, child_frame in fixed:
        held_by[body_of_frame(child_frame, graph)] = (
            body_of_frame(parent_frame, graph),
            parent_frame,
        )
    # The innermost bound tree owning a body: the longest IRI it descends from.
    owner = {
        body: max(
            (tree for tree in bound_trees if iri_is_descendant(tree, body)),
            key=lambda tree: len(str(tree)),
            default=None,
        )
        for body in {*held_by, *(parent for parent, _frame in held_by.values())}
    }

    attachments = {}
    for parent_frame, child_frame in fixed:
        parent_body, child_body = (body_of_frame(f, graph) for f in (parent_frame, child_frame))
        # A body no asset backs carries no site to bolt to, so follow what holds it until one
        # does. A wrapper's massless bracket is such a body: the arm hanging off it still belongs
        # on the tower, and stopping at the bracket would bolt the arm to the world instead --
        # leaving the simulation with an arm at the origin while the chain says it is mounted.
        host, host_frame = parent_body, parent_frame
        # An asset behind a body is what gives it a site to bolt to.
        while host is not None and host not in modelled_bodies and owner[host] is None:
            host, host_frame = held_by.get(host, (None, host_frame))
        # Nothing above it is backed either: the body places itself, and the placement composed
        # against the anchor already accounts for whatever said where it sits.
        if host is None:
            attachments[child_body] = ("World", "", child_frame, parent_body)
        elif child_body in modelled_bodies or owner[host] != owner[child_body]:
            attachments[child_body] = ("Site", local_name(host_frame), child_frame, host)

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
        topic, message = _camera_provider(graph, sensor)
        cameras.append(
            CameraBinding(
                id=f"{runtime_prefix}{local_name(sensor)}",
                width=int(graph.value(sensor, URI_SENS_PRED_RESOLUTION_WIDTH).toPython()),
                height=int(graph.value(sensor, URI_SENS_PRED_RESOLUTION_HEIGHT).toPython()),
                rate_hz=get_update_rate(graph, ModelBase(node_id=sensor, graph=graph)),
                uri=str(sensor),
                # Prefixed like id: two arms' wrist cameras must not share a frame name.
                frame_id=f"{runtime_prefix}{local_name(sensor)}",
                topic=topic,
                message=message,
                frame_uri=str(graph.value(sensor, URI_SENS_PRED_FRAME) or ""),
                fovy_deg=_camera_fovy_deg(graph, sensor),
            )
        )
    return cameras


def _camera_fovy_deg(graph, sensor) -> float:
    """The camera's field of view in degrees, whatever unit the model authored it in."""
    fov = graph.value(sensor, URI_SENS_PRED_FIELD_OF_VIEW)
    value = graph.value(fov, URI_QUDT_PRED_VALUE) if fov is not None else None
    if value is None:
        return 45.0
    degrees = float(value.toPython())
    unit = graph.value(fov, URI_QUDT_PRED_UNIT) if fov is not None else None
    if unit is not None and str(unit).endswith("RAD"):
        degrees = math.degrees(degrees)
    return degrees


def _camera_provider(graph, sensor) -> tuple[str | None, str | None]:
    """The channel this camera is read off, from the subscription that states it carries it.

    A camera nothing subscribes to has no provider: a viewer that cannot say where the images
    come from shows none, rather than guessing a channel from the camera's name.
    """
    channels = [
        node
        for node in graph.subjects(SOSA.hasFeatureOfInterest, sensor)
        if NS_MM_ROS["Topic"] in get_node_types(graph, node)
    ]
    if not channels:
        return None, None
    if len(channels) > 1:
        raise ConstraintViolation(
            "communication",
            f"camera '{local_name(sensor)}' is carried by {len(channels)} channels; a camera "
            "has one provider",
        )
    topic = channels[0]
    return (
        str(graph.value(topic, NS_MM_ROS["channel-name"]) or "") or None,
        str(graph.value(topic, NS_MM_ROS["type-name"]) or "") or None,
    )


def _agent_bindings(model):
    """Per modelled agent, the `(model, tree, path, entity)` rows its assets bind, and the agent
    each bound tree ultimately belongs to.
    """
    graph = model.graph
    bindings_by_modelled = {}
    for modelled in graph.subjects(RDF.type, AGN.ModelledAgent):
        rows = []
        for asset in graph.objects(modelled, AGN["has-agent-model"]):
            # Not `get_path_of_node`, which raises: a pathless agent model is metadata rather
            # than a runtime asset, and the assembly around it still reads.
            path = graph.value(asset, URI_EXEC_PRED_PATH)
            if not path:
                continue
            path = str(path)
            for tree, entity in model_mappings(model, asset, GEOM_ENT.KinematicTree):
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


def agent_assemblies(model, attach_by_body) -> list:
    graph = model.graph
    adjacency = kinematic_adjacency(model)
    bound_model_trees = mapped_targets(model, AGN["AgentModel"], GEOM_ENT.KinematicTree)
    body_names_by_tree = {
        tree: {
            local_name(body)
            for body in graph.subjects(RDF.type, GEOM_ENT.RigidBody)
            if iri_is_descendant(tree, body)
        }
        for tree in bound_model_trees
    }
    serials = [
        (tree, graph.value(tree, NS_MM_KC_EXT["root"]), graph.value(tree, NS_MM_KC_EXT["tip"]))
        for tree in graph.subjects(RDF.type, URI_KC_TYPE_SERIAL)
    ]
    bindings_by_modelled, agent_by_tree = _agent_bindings(model)
    bindings = [row for rows in bindings_by_modelled.values() for row in rows]

    result = []
    chainless = []
    for modelled in graph.subjects(RDF.type, AGN.ModelledAgent):
        agent = graph.value(modelled, AGN["of-agent"])
        own = bindings_by_modelled[modelled]
        if agent is None or not own:
            continue

        # A chain may span several agents; the agent owning its root drives it.
        serial = next(
            (
                (tree, root, tip)
                for tree, root, tip in serials
                if root is not None
                and tip is not None
                and (
                    any(
                        binding["tree"] == tree or iri_is_descendant(binding["tree"], root)
                        for binding in own
                    )
                )
            ),
            None,
        )
        if serial is None:
            # A mobile platform drives no serial chain: its castors branch off the base and
            # rejoin nothing, so there is no root-to-tip path to walk and no composition to
            # declare. The scene must still spawn and place the asset, which is all a chainless
            # assembly carries -- `robot_setups` skips it for want of a chain.
            root_binding = own[0]
            root_frame = graph.value(root_binding["tree"], NS_MM_KC_EXT["root"])
            if root_frame is None:
                continue
            root_body = body_of_frame(root_frame, graph)
            attachment = attach_by_body.get(root_body, ("World", "", root_frame, None))
            attach_kind, attach_name, _frame, _parent = attachment
            position, orientation = quantities.placement_of(
                model, attachment, quantities.anchor_frame(model)
            )
            hosted = list(graph.objects(modelled, SOSA.hosts))
            chainless.append(len(result))
            result.append(
                AgentAssembly(
                    agent=agent,
                    device="",
                    config_key=_config_key(model, agent, agent, ""),
                    sensors=[],
                    cameras=_cameras(model, hosted, ""),
                    devices=[],
                    urdf=root_binding["path"],
                    prefix="",
                    owned_trees=[binding["tree"] for binding in own],
                    serial_chain=None,
                    root_body=root_body,
                    chain_root="",
                    tip="",
                    tool_body="",
                    tcp_frame="",
                    attach_kind=attach_kind,
                    attach_name=attach_name,
                    pos=position,
                    quat=orientation,
                    attachments=[],
                )
            )
            continue
        serial_tree, root_frame, tip_frame = serial
        root_binding = next(
            (binding for binding in own if iri_is_descendant(binding["tree"], root_frame)), None
        ) or next(binding for binding in own if binding["tree"] == serial_tree)
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

        hosted = list(graph.objects(modelled, SOSA.hosts))
        attachment = attach_by_body.get(root_body, ("World", "", root_frame, None))
        attach_kind, attach_name, _frame, _parent = attachment
        position, orientation = quantities.placement_of(
            model, attachment, quantities.anchor_frame(model)
        )
        agent_device = device_of(model, agent)
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
                        local_id=local_name(sensor),
                        local_frame=local_name(frame),
                        update_rate_hz=get_update_rate(
                            model.graph, ModelBase(node_id=sensor, graph=model.graph)
                        ),
                        config_key=_config_key(
                            model, sensor, agent, f"{runtime_prefix}{local_name(sensor)}"
                        ),
                        observes=[
                            local_name(observed)
                            for observed in model.graph.objects(sensor, SOSA.observes)
                        ],
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

    # A gripper at a chain's tip drives no chain either, but that chain's assembly mounts it.
    carried = {
        tree
        for index, assembly in enumerate(result)
        if index not in chainless
        for tree in assembly.owned_trees
    }
    kept = [
        assembly
        for index, assembly in enumerate(result)
        if index not in chainless or not carried.intersection(assembly.owned_trees)
    ]

    def hosts(assembly) -> int:
        """How many assemblies this one is mounted on, through the sites the runtime resolves."""
        parent = attach_by_body.get(assembly.root_body, (None, None, None, None))[3]
        host = next(
            (
                other
                for other in kept
                if other is not assembly
                and parent is not None
                and any(iri_is_descendant(tree, parent) for tree in other.owned_trees)
            ),
            None,
        )
        return 1 + hosts(host) if host is not None else 0

    # The runtime resolves a parent site as it spawns, so a mounted agent follows its host.
    return sorted(kept, key=hosts)


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
    agent: object


_EMPTY_SETUP = _ChainSetup(
    ChainBinding(root="", end="", tip="", tree="", namespace="", name="", joints=[]),
    HardwareBinding(urdf="", model="", tool_body="", tcp_frame=""),
    RuntimeBinding(id="", owner=False, prefix="", owned_trees=[], config_key=""),
    [],
    [],
    None,
)

# What an asset path says about the arm it holds, when no device is bound to say it outright.
_ROBOT_MODEL_HINTS = (("kinova_gen3", "KinovaGen3"), ("gen3", "KinovaGen3"))
# The arm-with-gripper pairing is wiring; the manipulator is still the arm.
_ROBOT_MODEL_BY_DEVICE = {"KinovaGen3-2F85": "KinovaGen3"}


@reader
def scene_graph(model):
    """The imported scene documents on their own, which is what a KDL tree is built from.

    The builder walks every pose relation the graph holds and reads coordinates off each one. A
    motion-spec world quantity is a measurement: it states the frames it relates and carries no
    coordinates at all, so handing the builder the merged graph makes it demand numbers of a
    reading the run has yet to take.
    """
    graph = Graph()
    for document in model.graph.graphs():
        if (None, RDF["type"], GEOM_ENT.KinematicTree) in document:
            graph += document
    return graph


def robot_setups(model):
    """Per-robot chain setups, sourced from the scene-dsl graph.

    Returns:
        `(setups_by_node, ordered, trees)`: the setup for each robot's abstract agent node -- the
        target of a solver's `agn:of-agent` -- the same setups in declaration order, and every
        distinct scene tree, which the one world model is built from
    """
    from motion_spec.generation.scene_kdl import chain_for_iri

    # Graph-only consumers may use an incomplete scene fixture: assembly metadata survives, but
    # there is no chain to slice until the scene carries a tree. A scene that carries one and
    # still fails to build is a fault to report, not one to answer with an empty chain.
    trees = []
    for document in model.graph.graphs():
        if (None, RDF["type"], GEOM_ENT.KinematicTree) not in document:
            continue
        # scene-dsl names a scene's KDL header, and its namespace, after the scene's file.
        stem = Path(unquote(urlsplit(str(document.identifier)).path)).stem
        namespace = get_valid_var_name(stem)
        namespace = f"scene_{namespace}" if namespace[0].isdigit() else namespace
        trees.extend(
            {**tree, "namespace": namespace, "header": f"{stem}.kdl.hpp"}
            for tree in build_kdl_trees(document)
        )
    bound_trees = mapped_targets(model, AGN["AgentModel"], GEOM_ENT.KinematicTree)
    attach_by_body, _root = fixed_attachments(model, bound_trees)

    setups_by_node, ordered = {}, []
    for assembly in agent_assemblies(model, attach_by_body):
        # A chainless assembly is a platform: the scene spawns its asset, but there is no chain
        # to set up and nothing here to say about it.
        if assembly.serial_chain is None:
            continue
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
                namespace=chain["namespace"],
                name=chain["name"],
                joints=chain["joints"],
                frames=chain["frames"],
                bodies=chain["bodies"],
                tip_segment=chain["tip_segment"],
                joint_segments=chain["joint_segments"],
                world_root=chain["world_root"],
                world_tip=chain["world_tip"],
                tree_root=chain["tree_root"],
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
            agent=assembly.agent,
        )
        setups_by_node[assembly.agent] = setup
        ordered.append(setup)

    return setups_by_node, ordered, trees


def tree_segments(model, setups, trees=()) -> dict:
    """Every scene element the built trees carry, by IRI, named as the world model names it.

    A chain resolves the whole tree it is sliced from, not only the part it articulates, so a
    frame no chain reaches -- a fixed camera watching the scene -- resolves here just the same.
    A tree no chain is sliced from at all -- a free body, placed by what measures it -- is still
    added to the world model, so its segments resolve alongside them. A body's root frame is
    where the body's segment is, so it resolves to that segment as it does on a chain.
    """
    graph = model.graph
    # A tree's own segment is exact. A chain slice places what it reaches on its nearest
    # articulated body, so it only names what the trees leave unnamed.
    exact_names = []
    for tree in trees:
        exact_names.append((tree["root_iri"], tree["root"]))
        root = URIRef(tree["root_iri"])
        if GEOM_ENT.RigidBody in get_node_types(graph, root):
            exact_names.append((root_frame_of(root, graph).id, tree["root"]))
        for segment in tree["segments"]:
            exact_names.append((segment["iri"], segment["name"]))
            body = URIRef(segment["iri"])
            if GEOM_ENT.RigidBody in get_node_types(graph, body):
                exact_names.append((root_frame_of(body, graph).id, segment["name"]))
    placed_names = [
        (iri, segment)
        for setup in setups.values()
        for iri, segment in setup.chain.world_segments.items()
    ]
    exact: dict[str, str] = {}
    placed: dict[str, str] = {}
    for named, names in ((exact, exact_names), (placed, placed_names)):
        for iri, name in names:
            if named is placed and str(iri) in exact:
                continue
            if named.setdefault(str(iri), name) != name:
                raise ConstraintViolation(
                    "kinematics",
                    f"'{iri}' is named both '{named[str(iri)]}' and '{name}' in the world model -- "
                    "an element is one segment",
                )
    return {**placed, **exact}


def _placed_on_chain(carrier) -> tuple[tuple[object, bool], ...]:
    """What of one carrier the generated code asks forward kinematics for, and which authority
    answers: the one world model, or this chain's own numbering.

    Only these reach kinematics, so only these need placing. A record may name frames it never
    resolves through the chain -- a twist states the point it is taken about -- and placing
    those would reject a model over a frame nothing looks up.
    """
    if isinstance(carrier, Wrench) and carrier.estimator is not None:
        # No sensor frame to place: the transform frames are read off the world model.
        return ((carrier.reference_point, True), (carrier.as_seen_by, True))
    if isinstance(carrier, Wrench):
        # The tare's three frames all come off the world model, and so does the load hanging
        # off the sensor frame.
        return (
            (carrier.sensor_frame, True),
            (carrier.reference_point, True),
            (carrier.as_seen_by, True),
        )
    if isinstance(carrier, CartesianForceSpecification):
        # `f_ext` is indexed chain-relative, so the chain's own segment stays; the frame that
        # segment stands at is a world read like any other.
        return ((carrier.attached_to, False), (carrier.attached_to, True))
    if isinstance(carrier, AccelerationConstraint):
        # An acceleration constraint whose direction is taken in a frame that moves with the arm.
        return ((carrier.as_seen_by, True),)
    if isinstance(carrier, VelocityTwist):
        # The world pass carries the twist beside the pose, so both the segment it is taken of
        # and the frame it is seen in are world-model reads.
        return ((carrier.of, True), (carrier.as_seen_by, True))
    if isinstance(carrier, Pose) and carrier.relative_to_frame is not None:
        # A link-relative pose reads both of its ends off the world model.
        return ((carrier.of, True), (carrier.relative_to_frame, True))
    if isinstance(carrier, (Position, Orientation, Pose)):
        return ((carrier.of, isinstance(carrier, Pose)),)
    return ()


def place_on_chain(chain, item, owner: str, world_fk: bool, world_index: dict) -> str | None:
    """Resolve where one frame, point or body sits, once, while generating.

    The generated code is handed a resolved index, never a name: a name would have to be matched
    at run time, and a frame that matched nothing would surface as a dead controller on the first
    tick instead of an error here. A world-model read resolves to the exact tree segment of any
    tree the world model holds -- a body on a tree of its own included -- so its constant offset
    needs no run-time math; a chain-local read still resolves to the chain's own segment index,
    which has no offset to compose onto.

    Returns:
        the full tree segment name for a world-model read, else None
    """
    if not isinstance(item, (Point, Frame, SimplicialComplex)):
        return None
    if world_fk:
        segment = chain.world_segments.get(item.uri) or world_index.get(item.uri)
        if segment is None:
            raise ConstraintViolation(
                "geometry",
                f"'{item.id}' is absent from every tree the world model holds, so it cannot be "
                f"asked where solver '{owner}' should read it from.",
            )
        return segment
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
    return None


def place_solver_on_chain(solver, world_index: dict, root_by_segment: dict) -> list[dict]:
    """Place every frame, point and body the solver's generated code asks kinematics for.

    Returns:
        one `{solver_id, frame_id, segment_name, tree_root}` record per world-model read this
        solver makes; `tree_root` names the root of the tree carrying the segment when the chain
        root is not above it, which is where the joints placing it must be required from, and is
        None otherwise
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
    reads = [read for carrier in carriers for read in _placed_on_chain(carrier)]
    # A commanded wrench is moved onto its segment from its own frame and point.
    reads += [
        read
        for driver in solver.motion_drivers
        for force in driver.cartesian_force
        for read in ((force.force.reference_point, True), (force.force.as_seen_by, True))
    ]
    segment_by_frame: dict[str, str] = {}
    off_branch: set[str] = set()
    for item, world_fk in reads:
        segment = place_on_chain(solver.chain, item, solver.id, world_fk, world_index)
        if segment is None:
            continue
        if segment_by_frame.setdefault(item.id, segment) != segment:
            raise ConstraintViolation(
                "geometry",
                f"'{item.id}' resolves to two segments of the tree solver '{solver.id}' "
                f"runs on: '{segment_by_frame[item.id]}' and '{segment}'. One name cannot "
                f"stand for two places in the world model.",
            )
        if item.uri not in solver.chain.frames and item.uri not in solver.chain.bodies:
            off_branch.add(item.id)

    return [
        {
            "solver_id": solver.runtime.owner_id,
            "frame_id": frame_id,
            "segment_name": segment,
            "tree_root": (
                root_by_segment.get(segment, solver.chain.tree_root)
                if frame_id in off_branch
                else None
            ),
        }
        for frame_id, segment in segment_by_frame.items()
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
    *((rdf_type, quantities.joint_quantity) for rdf_type in quantities.JOINT_QUANTITY_TYPES),
    (RBDYN_COORD.WrenchCoordinate, quantities.wrench),
)


def _observes_in_frame(
    model, node, type_, chain, runtime_prefix, owned_trees, backend: str, agent=None
):
    """Whether an observation is stated in this solver's reference frame, and in which frame node."""
    graph = model.graph
    if type_ in quantities.JOINT_QUANTITY_TYPES:
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
        observer = graph.value(node, EST["estimated-by"])
        if observer is not None:
            # The observer runs on one agent's chain, so that agent's solver answers it.
            return graph.value(observer, AGN["of-agent"]) == agent, frame_node
        observation = ensure_one_typed_subject_uri(
            graph, node, SOSA.observedProperty, SOSA.Observation
        )
        sensor = graph.value(observation, SOSA.madeBySensor) if observation is not None else None
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
        # A pose likewise: wrt any frame the chain carries, it is two world reads composed.
        if runtime_frame != chain.root and type_ in (
            GEOM_COORD.VelocityTwistCoordinate,
            GEOM_COORD.PoseCoordinate,
        ):
            if not on_this_chain:
                # Off this chain, the read is still this chain's to answer whenever the thing
                # observed sits on it: the world model holds the observer's pose, so the FK
                # value is turned into its axes by one more world read. A twist composes the
                # same way a pose does -- the frame only supplies a rotation.
                endpoint = (
                    quantities.pose(model, node).of
                    if type_ == GEOM_COORD.PoseCoordinate
                    else quantities.velocity_twist(model, node).of
                )
                return (endpoint.uri in chain.frames or endpoint.uri in chain.bodies, frame_node)
            return on_this_chain, frame_node

        return runtime_frame == chain.root, frame_node

    return True, frame_node


def _world_solver_outputs(model, setup: _ChainSetup, backend: str) -> list:
    """The runtime observations stated in this solver's reference frame."""
    graph = model.graph
    outputs = []
    for type_, read in _WORLD_OUTPUTS:
        for node in graph.subjects(RDF.type, type_):
            if (node, RDF.type, SOSA.ObservableProperty) not in graph:
                continue
            in_frame, frame_node = _observes_in_frame(
                model,
                node,
                type_,
                setup.chain,
                setup.runtime.prefix,
                setup.runtime.owned_trees,
                backend,
                agent=setup.agent,
            )
            if not in_frame:
                continue
            output = _runtime_output(model, read(model, node), type_, node, frame_node, setup)
            frame = output.as_seen_by if isinstance(output, (Pose, SpatialCoordinate)) else None
            if frame_node is None and frame is not None and frame.id != setup.chain.root:
                continue
            outputs.append(output)

    return unique_by_id(outputs)


def _pose_wrt_node(model, node):
    """The frame a pose coordinate is stated with respect to, or None."""
    relation = model.graph.value(node, URI_GEOM_PRED_OF_POSE)

    return None if relation is None else model.graph.value(relation, URI_GEOM_PRED_WRT)


def _runtime_output(model, output, type_, node, frame_node, setup: _ChainSetup):
    """One observation with its frames and names rewritten the way the runtime knows them."""
    if type_ in quantities.JOINT_QUANTITY_TYPES:
        return replace(output, joint_name=f"{setup.runtime.prefix}{output.joint_name}")
    if type_ == GEOM_COORD.VelocityTwistCoordinate:
        seen_by = _runtime_frame(model, frame_node, setup.runtime.prefix, setup.runtime.owned_trees)
        return replace(output, as_seen_by=seen_by, seen_by_root=seen_by.id == setup.chain.root)
    if type_ == GEOM_COORD.PoseCoordinate and frame_node is not None:
        seen_by = _runtime_frame(model, frame_node, setup.runtime.prefix, setup.runtime.owned_trees)
        if seen_by.id == setup.chain.root:
            # Measured from a frame that moves, but stated in the chain root's axes: the
            # orientation is the subject's own and the translation is the gap between the two.
            wrt_node = _pose_wrt_node(model, node)
            if wrt_node is None:
                return output
            wrt = _runtime_frame(model, wrt_node, setup.runtime.prefix, setup.runtime.owned_trees)
            if wrt.id == setup.chain.root:
                return output
            return replace(output, relative_to_frame=wrt, seen_by_root=True)
        # Only wrt == as-seen-by composes as one relative pose; anything else stays unwritten.
        wrt_id = output.with_respect_to.id if output.with_respect_to is not None else None
        if wrt_id != (output.as_seen_by.id if output.as_seen_by is not None else None):
            return output
        return replace(output, as_seen_by=seen_by, relative_to_frame=seen_by)
    if type_ != RBDYN_COORD.WrenchCoordinate:
        return output

    graph = model.graph
    relation = graph.value(node, RBDYN_COORD["of-wrench"])
    reference_node = graph.value(relation, RBDYN_ENT["reference-point"])
    runtime = (setup.runtime.prefix, setup.runtime.owned_trees)
    if output.estimator is not None:
        # Nothing measures it, so there is no sensor or sensor frame to prefix.
        return replace(
            output,
            estimator=replace(
                output.estimator, agent=f"{setup.runtime.prefix}{output.estimator.agent}"
            ),
            reference_point=replace(
                output.reference_point, id=_runtime_frame(model, reference_node, *runtime).id
            ),
            as_seen_by=_runtime_frame(model, frame_node, *runtime),
        )
    observation = ensure_one_typed_subject_uri(graph, node, SOSA.observedProperty, SOSA.Observation)
    sensor_frame_node = graph.value(graph.value(observation, SOSA.madeBySensor), SENSORS.frame)

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


def build_robots(model, schedule, setups, derivation, backend: str, detect_pose_ids=()) -> Robots:
    """Build every robot the program commands, and schedule the calls that drive them.

    Parameters:
        schedule: the active scope; the steps join `Robots.schedule_steps`
        setups: the chain setup per agent node, from `robot_setups`
        detect_pose_ids: the world poses a detect result writes; the kinematics do not compute
            them, so no chain lists them among its outputs

    Raises:
        ConstraintViolation: a mobile-platform solver names an algorithm with no codegen wiring,
            or a chain runs an unsupported robot model.
    """
    graph = model.graph
    steps = []
    # A solver whose agent the scene does not bind still needs a chain to talk about; the first
    # declared setup is the model's own answer to "which arm", and an empty one says there is none.
    default_setup = next(iter(setups.values()), _EMPTY_SETUP)

    platform_velocity = []
    for node in graph.subjects(RDF.type, SLV["VelocityCompositionSolver"]):
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
        # An observation the solver also states as an output is that same output, named the way
        # the runtime knows it.
        observed = _world_solver_outputs(model, setup, backend)
        observed_ids = {out.id for out in observed}
        solver.output = [
            out
            for out in [*(out for out in solver.output if out.id not in observed_ids), *observed]
            if out.id not in detect_pose_ids
        ]
        # An acceleration constraint is base-aligned when its axis frame is the chain root. Both
        # solver families ask: the row is a direction in the solver's frame either way, and one
        # taken in a frame that moves with the arm has to be turned into the root's axes.
        root_body = _body_name(solver.chain.root)
        for driver in solver.motion_drivers:
            for constraint in (*driver.acceleration_constraint, *driver.cartesian_acceleration):
                constraint.base_aligned = (
                    constraint.as_seen_by is None
                    or _body_name(constraint.as_seen_by.id) == root_body
                )
        solver.constraint_rows = max(
            (
                len(driver.acceleration_constraint) + len(driver.cartesian_acceleration)
                for driver in solver.motion_drivers
            ),
            default=0,
        )
        serial_chains.append(solver)
        driven = graph[node : SLV["motion-drivers"] / (SLV["cartesian-force"] | SLV["joint-force"])]
        steps.extend(schedule.of(driven, OPS_GENERIC + OPS_SOLVER))

    platform_force = []
    for node in graph.subjects(RDF.type, SLV["ForceDistributionSolver"]):
        # The wrenches its controllers command, and the ops that build them. A serial chain gets
        # both from its drivers; a distribution is fed the same way, so it reads them the same
        # way -- otherwise the wrench is authored, never computed, and the platform is commanded
        # nothing at all.
        forces = tuple(
            constraint_handler.cartesian_force_specification(model, force)
            for driver in graph[node : SLV["motion-drivers"]]
            for force in graph[driver : SLV["cartesian-force"]]
        )
        platform_force.append(
            ForceDistributionSolver(
                model.id(node),
                model.id(graph.value(node, SLV["configuration"])),
                quantities.wrench(model, graph.value(node, SLV["force"])),
                forces=forces,
            )
        )
        driven = graph[node : SLV["motion-drivers"] / SLV["cartesian-force"]]
        steps.extend(schedule.of([node, *driven], OPS_GENERIC + OPS_SOLVER))

    for type_, label in _UNIMPLEMENTED_PLATFORM_ALGORITHMS:
        unsupported = list(graph.subjects(RDF.type, type_))
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
    """Every chain solver, in the order the handlers running it are declared: within a handler,
    the solvers its controllers drive first, then the ones it only runs."""
    graph = model.graph
    ordered = []
    for handler in sorted(
        graph.subjects(RDF.type, CSTR_HDL["ConstraintHandler"]),
        key=lambda node: int(graph.value(node, APP.order, default=Literal(0)).value),
    ):
        driven = [plan.solver for plan in derivation.controllers_by_handler.get(handler, ())]
        run = set(graph.objects(handler, CSTR_HDL_EXT["runs-solver"])) - set(driven)
        for solver in [*driven, *run]:
            if solver not in ordered and SLV.SolverWithInputAndOutput in get_node_types(
                graph, solver
            ):
                ordered.append(solver)
    unhandled = set(graph.subjects(RDF.type, SLV.SolverWithInputAndOutput)) - set(ordered)
    if unhandled:
        raise ConstraintViolation(
            "solver",
            f"solvers {', '.join(map(str, unhandled))} are run by no constraint handler, so nothing "
            "says when they run",
        )

    return ordered


# The observation readers a chain solver's authored outputs dispatch over.
_SOLVER_OUTPUTS = (
    (GEOM_COORD["PoseCoordinate"], quantities.pose),
    (GEOM_COORD["VelocityTwistCoordinate"], quantities.velocity_twist),
    *((rdf_type, quantities.joint_quantity) for rdf_type in quantities.JOINT_QUANTITY_TYPES),
    (RBDYN_COORD["WrenchCoordinate"], quantities.wrench),
)


def _solver_with_input_and_output(model, node, setup: _ChainSetup) -> SolverWithInputAndOutput:
    """A chain solver as authored: its algorithm, its drivers, its outputs and its torque limit.

    `chain`/`hardware`/`runtime` have no default, so the bindings are constructor
    arguments rather than assigned after the fact; `replace()` gives each solver its own copies,
    since several solver nodes may share one agent's `setup`.
    """
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
        algorithm_name=family.name,
        derived_root_acceleration=quantities.parse_xyz(model, gravity_node)
        if gravity_node
        else None,
        gravity_source=model.id(gravity_node) if gravity_node else None,
        torque_saturation=(
            constraint_handler.saturation(model, torque_limit) if torque_limit is not None else None
        ),
    )
    return solver


def _validate_solvers(serial_chain_solvers, backend: str) -> None:
    """Reject robot models the hardware backend has no driver for."""
    if backend != "robif2b":
        return
    unsupported = {
        solver.hardware.model or "(unrecognised)"
        for solver in serial_chain_solvers
        if solver.hardware.model not in SUPPORTED_ROBOT_MODELS
    }
    if unsupported:
        raise ConstraintViolation(
            "solver",
            f"Unsupported robot model(s) for robif2b: {', '.join(unsupported)}. "
            f"Supported: {', '.join(SUPPORTED_ROBOT_MODELS)}",
        )
