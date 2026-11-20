# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The agents a program commands: their devices, sensors and cameras, the chain each is set up on,
and the solver records built on those chains.

Each reader validates what it reads, at the moment it reads it: an agent in no declared set, a
camera with two providers, a solver no handler runs, a robot model the backend cannot drive.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, replace
from typing import NamedTuple

from motion_spec_dsl.vocab import (
    AGN,
    ALGO_EXT,
    APP,
    CSTR_HDL,
    CSTR_HDL_EXT,
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
from rdf_utils.models.geom_coord import PoseCoordModel
from rdf_utils.models.prov import get_linked_file_path
from rdf_utils.models.vocab import (
    URI_GEOM_TYPE_KTREE,
    URI_GEOM_TYPE_RIGID_BODY,
    URI_QUDT_PRED_UNIT,
    URI_QUDT_PRED_VALUE,
    URI_QUDT_UNIT_RAD,
)
from rdf_utils.namespace import NS_MM_QUDT_QTY
from rdflib import Graph, Literal, URIRef
from rdflib.namespace import DCTERMS, RDF, SDO
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
    get_kinematic_mappings,
    kinematic_trees,
    root_frame_of,
    typed,
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
from motion_spec.rdf_parser.operations import OPS_GENERIC, OPS_SOLVER, recorded_coord_policy

SUPPORTED_ROBOT_MODELS = {"KinovaGen3"}
# any other bound device is rejected
DRIVEN_DEVICES = {"KinovaGen3", "KinovaGen3-2F85", "Robotiq2F85", "RobotiqFT300s"}
GRIPPER_DEVICES = {"KinovaGen3-2F85", "Robotiq2F85"}
SENSOR_KINDS = {SENSORS.ForceTorqueSensor: "ForceTorque"}


def config_key(model, element, agent, drives: str) -> str:
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


def fixed_attachments(model):
    """Where the scene bolts one model to another, oriented as scene-dsl's tree walk directs it.

    Returns:
        `(attachments, root)`: per attached body, the `(kind, name, frame, parent body, host
        frame)` the scene mounts it by, and the body the world is rooted at
    """
    graph = model.graph
    trees, holders = scene_kinematics(model)
    fixed = [
        (
            tree.parent[child],
            child,
            tree.joints[joint].frame_on(tree.parent[child]),
            tree.joints[joint].frame_on(child),
        )
        for tree in trees
        for child, joint in tree.parent_joint.items()
        if get_node_types(graph, joint) == {KC.Joint}
    ]
    if not fixed:
        return {}, None
    root = body_of_frame(quantities.anchor_frame(model), graph)

    instance = quantities.scene_instance(model)
    modelled_bodies = {
        mapping.target_id
        for models in instance.object_models.values()
        for asset in models.values()
        for mapping in get_kinematic_mappings(asset, URI_GEOM_TYPE_RIGID_BODY)
    }
    bound_trees = {
        mapping.target_id
        for models in instance.agent_models.values()
        for asset in models.values()
        for mapping in get_kinematic_mappings(asset, URI_GEOM_TYPE_KTREE)
    }
    held_by = {child: (parent, parent_frame) for parent, child, parent_frame, _frame in fixed}
    owner = {
        body: next((tree for tree in holders[body] if tree in bound_trees), None)
        for body in {*held_by, *(parent for parent, _frame in held_by.values())}
    }

    attachments = {}
    for parent_body, child_body, parent_frame, child_frame in fixed:
        # an unbacked body (e.g. a massless bracket) has no site: climb to one that does
        host, host_frame = parent_body, parent_frame
        while host is not None and host not in modelled_bodies and owner[host] is None:
            host, host_frame = held_by.get(host, (None, host_frame))
        if host is None:
            attachments[child_body] = ("World", "", child_frame, parent_body, None)
        elif child_body in modelled_bodies or owner[host] != owner[child_body]:
            attachments[child_body] = (
                "Site",
                local_name(host_frame),
                child_frame,
                host,
                host_frame,
            )

    return attachments, root


def sensor_kind(model, sensor) -> str:
    """The sensor's kind as the IR names it; empty for a kind codegen does not model."""
    types = get_node_types(model.graph, sensor)
    return next((name for uri, name in SENSOR_KINDS.items() if uri in types), "")


def cameras(model, hosted, runtime_prefix) -> list:
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
        channels = [
            node
            for node in graph.subjects(SOSA.hasFeatureOfInterest, sensor)
            if NS_MM_ROS["Topic"] in get_node_types(graph, node)
        ]
        if len(channels) > 1:
            raise ConstraintViolation(
                "communication",
                f"camera '{local_name(sensor)}' is carried by {len(channels)} channels; a camera "
                "has one provider",
            )
        topic, message = None, None
        if channels:
            topic = str(graph.value(channels[0], NS_MM_ROS["channel-name"]) or "") or None
            message = str(graph.value(channels[0], NS_MM_ROS["type-name"]) or "") or None
        fov = graph.value(sensor, URI_SENS_PRED_FIELD_OF_VIEW)
        fov_value = graph.value(fov, URI_QUDT_PRED_VALUE) if fov is not None else None
        fovy_deg = 45.0
        if fov_value is not None:
            fovy_deg = float(fov_value.toPython())
            if graph.value(fov, URI_QUDT_PRED_UNIT) == URI_QUDT_UNIT_RAD:
                fovy_deg = math.degrees(fovy_deg)
        cameras.append(
            CameraBinding(
                id=f"{runtime_prefix}{local_name(sensor)}",
                width=int(graph.value(sensor, URI_SENS_PRED_RESOLUTION_WIDTH).toPython()),
                height=int(graph.value(sensor, URI_SENS_PRED_RESOLUTION_HEIGHT).toPython()),
                rate_hz=get_update_rate(graph, ModelBase(node_id=sensor, graph=graph)),
                uri=str(sensor),
                # two arms' wrist cameras must not share a frame name
                frame_id=f"{runtime_prefix}{local_name(sensor)}",
                topic=topic,
                message=message,
                frame_uri=str(graph.value(sensor, URI_SENS_PRED_FRAME) or ""),
                fovy_deg=fovy_deg,
            )
        )
    return cameras


def agent_assemblies(model, attach_by_body) -> list:
    graph = model.graph
    trees, holders = scene_kinematics(model)
    instance = quantities.scene_instance(model)
    bound_model_trees = {
        mapping.target_id
        for models in instance.agent_models.values()
        for asset in models.values()
        for mapping in get_kinematic_mappings(asset, URI_GEOM_TYPE_KTREE)
    }
    body_names_by_tree = {tree: set() for tree in bound_model_trees}
    for scene_tree in trees:
        for body in scene_tree.bodies:
            for tree in holders[body]:
                if tree in body_names_by_tree:
                    body_names_by_tree[tree].add(local_name(body))
    bindings_by_agent = {}
    for agent, models in instance.agent_models.items():
        rows = []
        for asset_id, asset in models.items():
            # no location: metadata, not a runtime asset
            asset_path = get_linked_file_path(graph, asset_id, DCTERMS.source)
            if asset_path is None:
                continue
            for mapping in get_kinematic_mappings(asset, URI_GEOM_TYPE_KTREE):
                rows.append(
                    {
                        "tree": mapping.target_id,
                        "path": str(asset_path),
                        "entity": mapping.entity or "",
                    }
                )
        bindings_by_agent[agent] = rows
    agent_by_tree = {
        row["tree"]: agent for agent, rows in bindings_by_agent.items() for row in rows
    }
    bindings = [row for rows in bindings_by_agent.values() for row in rows]

    result = []
    for agent, own in bindings_by_agent.items():
        if not own:
            continue
        hosted = [
            sensor
            for modelled in typed(graph.subjects(AGN["of-agent"], agent), AGN.ModelledAgent, graph)
            for sensor in graph.objects(modelled, SOSA.hosts)
        ]

        # the agent owning the chain's root drives it
        serial = next(
            (
                (scene_tree, chain)
                for scene_tree in trees
                for chain in scene_tree.chains
                if any(binding["tree"] in holders[chain.root_body] for binding in own)
            ),
            None,
        )
        if serial is None:
            # mobile platform: no serial chain, but the scene still spawns its asset
            root_binding = own[0]
            root_frame = root_frame_of(root_binding["tree"], graph).id
            root_body = body_of_frame(root_frame, graph)
        else:
            scene_tree, chain = serial
            root_binding = next(
                binding for binding in own if binding["tree"] in holders[chain.root_body]
            )
            root_frame, root_body = chain.root_frame, chain.root_body
        attachment = attach_by_body.get(root_body, ("World", "", root_frame, None, None))
        attach_kind, attach_name, _frame, _parent, _host_frame = attachment
        position, orientation = quantities.placement_of(
            model, attachment, quantities.anchor_frame(model)
        )
        if serial is None:
            result.append(
                AgentAssembly(
                    agent=agent,
                    device="",
                    config_key=config_key(model, agent, agent, ""),
                    sensors=[],
                    cameras=cameras(model, hosted, ""),
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
        tip_binding = next(
            (binding for binding in bindings if binding["tree"] in holders[chain.tip_body]),
            root_binding,
        )
        tip_frame, tip_body = chain.tip_frame, chain.tip_body
        duplicate_root = (
            sum(local_name(root_body) in names for names in body_names_by_tree.values()) > 1
        )
        runtime_prefix = f"{local_name(root_binding['tree'])}_" if duplicate_root else ""
        child_by_joint = {joint: body for body, joint in scene_tree.parent_joint.items()}
        path = []
        for joint in scene_tree.path(chain):
            child = child_by_joint[joint]
            parent = scene_tree.parent[child]
            between = scene_tree.joints[joint]
            path.append((parent, child, between.frame_on(parent), between.frame_on(child), joint))
        # two arms must not claim each other's models
        chain_bodies = [root_body, tip_body, *(body for edge in path for body in edge[:2])]
        chain_bindings = [
            binding
            for binding in bindings
            if any(binding["tree"] in holders[body] for body in chain_bodies)
        ]
        chain_tip_body = root_body
        for parent_body, child_body, *_ in path:
            if root_binding["tree"] in holders[child_body]:
                chain_tip_body = child_body
            elif root_binding["tree"] in holders[parent_body]:
                chain_tip_body = parent_body
                break

        agent_device = next(iter(graph.subjects(EXEC["realizes"], agent)), None)
        agent_key = config_key(model, agent, agent, "")
        hosted_cameras = cameras(model, hosted, runtime_prefix)
        owners = dict.fromkeys(
            [agent, *(agent_by_tree[binding["tree"]] for binding in chain_bindings)]
        )
        bound = [(owner, "") for owner in owners]
        bound += [(sensor, f"{runtime_prefix}{local_name(sensor)}") for sensor in hosted]
        devices = []
        for bound_node, drives in bound:
            device = next(iter(graph.subjects(EXEC["realizes"], bound_node)), None)
            if device is not None:
                devices.append(
                    DeviceBinding(
                        kind=str(graph.value(device, SDO.model) or ""),
                        config_key=config_key(model, bound_node, agent, drives),
                        drives=drives,
                    )
                )
        chain_attachments = []
        for binding in chain_bindings:
            if binding is root_binding or binding["path"] == root_binding["path"]:
                continue
            boundary = next(
                (
                    (index, edge)
                    for index, edge in enumerate(path)
                    if binding["tree"] in holders[edge[1]]
                    and binding["tree"] not in holders[edge[0]]
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
            chain_attachments.append(
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
        result.append(
            AgentAssembly(
                agent=agent,
                device=str(graph.value(agent_device, SDO.model) or "") if agent_device else "",
                config_key=agent_key,
                sensors=[
                    SensorBinding(
                        id=f"{runtime_prefix}{local_name(sensor)}",
                        type=kind,
                        frame=f"{runtime_prefix}{local_name(frame)}",
                        local_id=local_name(sensor),
                        local_frame=local_name(frame),
                        config_key=config_key(
                            model, sensor, agent, f"{runtime_prefix}{local_name(sensor)}"
                        ),
                        observes=[
                            local_name(observed)
                            for observed in model.graph.objects(sensor, SOSA.observes)
                        ],
                    )
                    for sensor in hosted
                    if (kind := sensor_kind(model, sensor))
                    and (frame := model.graph.value(sensor, SENSORS.frame)) is not None
                ],
                cameras=hosted_cameras,
                devices=devices,
                urdf=root_binding["path"],
                prefix=runtime_prefix,
                owned_trees=[binding["tree"] for binding in chain_bindings],
                serial_chain=chain.id,
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
                attachments=[
                    attachment
                    for _, attachment in sorted(chain_attachments, key=lambda item: item[0])
                ],
            )
        )

    carried = {
        tree
        for assembly in result
        if assembly.serial_chain is not None
        for tree in assembly.owned_trees
    }
    kept = [
        assembly
        for assembly in result
        if assembly.serial_chain is not None or not carried.intersection(assembly.owned_trees)
    ]

    def hosts(assembly) -> int:
        """How many assemblies this one is mounted on, through the sites the runtime resolves."""
        attachment = attach_by_body.get(assembly.root_body)
        if attachment is None or attachment[3] is None:
            return 0
        parent = attachment[3]
        host = next(
            (
                other
                for other in kept
                if other is not assembly
                and any(tree in holders[parent] for tree in other.owned_trees)
            ),
            None,
        )
        return 1 + hosts(host) if host is not None else 0

    # hosts first: the runtime resolves a parent site as it spawns
    return sorted(kept, key=hosts)


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

_ROBOT_MODEL_HINTS = (("kinova_gen3", "KinovaGen3"), ("gen3", "KinovaGen3"))
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


@reader
def scene_kinematics(model):
    """The scene's trees as scene-dsl reads them, and the trees holding each of their bodies,
    joints and chains, innermost first.

    A tree holds what it defines or declares and what the trees it composes hold, as the trees'
    own nesting states it -- not as their IRIs happen to nest.

    Returns:
        `(trees, holders)`: the scene-dsl trees, and per element the trees holding it
    """
    trees = kinematic_trees(scene_graph(model))
    holders = {}
    for tree in trees:
        holder_of = {
            inner: outer for outer, inners in tree.nested_trees.items() for inner in inners
        }
        for element, innermost in (
            *tree.defining_tree_by_body.items(),
            *tree.declaring_tree.items(),
        ):
            held_by = [innermost]
            while held_by[-1] in holder_of:
                held_by.append(holder_of[held_by[-1]])
            holders[element] = held_by
    return trees, holders


def agent_cameras(model) -> list:
    """Every camera the model's agents carry, as the runtime renders them: the scene's cameras
    the IR states, for a reader that chooses what to record before a run."""
    attach_by_body, _root = fixed_attachments(model)
    return [
        camera
        for assembly in agent_assemblies(model, attach_by_body)
        for camera in assembly.cameras
    ]


def robot_setups(model):
    """Per-robot chain setups, sourced from the scene-dsl graph.

    Returns:
        `(setups_by_node, trees)`: the setup for each robot's abstract agent node -- the target
        of a solver's `agn:of-agent` -- and every distinct scene tree, which the one world model
        is built from
    """
    from motion_spec.generation.scene_kdl import chain_for_iri

    # an incomplete scene fixture may carry no tree to slice
    trees = []
    for document in model.graph.graphs():
        if (None, RDF["type"], GEOM_ENT.KinematicTree) not in document:
            continue
        header, namespace = model.scene_headers.get(document.identifier, (None, None))
        trees.extend(
            {**tree, "namespace": namespace, "header": header} for tree in build_kdl_trees(document)
        )
    attach_by_body, _root = fixed_attachments(model)

    setups_by_node = {}
    for assembly in agent_assemblies(model, attach_by_body):
        if assembly.serial_chain is None:
            continue
        chain = chain_for_iri(trees, str(assembly.serial_chain))
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

    return setups_by_node, trees


def tree_segments(model, setups, trees=()) -> dict:
    """Every scene element the built trees carry, by IRI, named as the world model names it.

    A chain resolves the whole tree it is sliced from, not only the part it articulates, so a
    frame no chain reaches -- a fixed camera watching the scene -- resolves here just the same.
    A tree no chain is sliced from at all -- a free body, placed by what measures it -- is still
    added to the world model, so its segments resolve alongside them. A body's root frame is
    where the body's segment is, so it resolves to that segment as it does on a chain.
    """
    graph = model.graph
    exact = {tree["root_iri"]: tree["root"] for tree in trees}
    exact |= {segment["iri"]: segment["name"] for tree in trees for segment in tree["segments"]}
    exact |= {
        str(root_frame_of(URIRef(iri), graph).id): name
        for iri, name in exact.items()
        if GEOM_ENT.RigidBody in get_node_types(graph, URIRef(iri))
    }
    placed = {
        iri: segment
        for setup in setups.values()
        for iri, segment in setup.chain.world_segments.items()
    }
    return placed | exact


def placed_on_chain(carrier) -> tuple[tuple[object, bool], ...]:
    """What of one carrier the generated code asks forward kinematics for, and which authority
    answers: the one world model, or this chain's own numbering.

    Only these reach kinematics, so only these need placing. A record may name frames it never
    resolves through the chain -- a twist states the point it is taken about -- and placing
    those would reject a model over a frame nothing looks up.
    """
    if isinstance(carrier, Wrench) and carrier.estimator is not None:
        return ((carrier.reference_point, True), (carrier.as_seen_by, True))
    if isinstance(carrier, Wrench):
        return (
            (carrier.sensor_frame, True),
            (carrier.reference_point, True),
            (carrier.as_seen_by, True),
        )
    if isinstance(carrier, CartesianForceSpecification):
        # `f_ext` is indexed chain-relative
        return ((carrier.attached_to, False), (carrier.attached_to, True))
    if isinstance(carrier, AccelerationConstraint):
        return ((carrier.as_seen_by, True),)
    if isinstance(carrier, VelocityTwist):
        return ((carrier.of, True), (carrier.as_seen_by, True))
    if isinstance(carrier, Pose) and carrier.relative_to_frame is not None:
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
    if placement["offset"] is not None:
        raise ConstraintViolation(
            "geometry",
            f"'{item.id}' sits at a pose on its body that no segment of '{chain.name}' stands "
            f"for. Composing that offset onto the forward kinematics is not implemented; give "
            f"the frame a segment of its own, by making it a chain endpoint or its body's root.",
        )
    if item.segment is None:
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
            for constraint in (
                *driver.acceleration_constraint,
                *driver.cartesian_acceleration,
                *driver.cartesian_velocity,
            )
        ),
        *(force for driver in solver.motion_drivers for force in driver.cartesian_force),
    ]
    reads = [read for carrier in carriers for read in placed_on_chain(carrier)]
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


def runtime_frame(model, frame_node, runtime_prefix, owned_trees):
    """A scene frame as the runtime names it, prefixed when this solver owns the tree it is in."""
    frame = quantities.frame(model, frame_node)
    _trees, holders = scene_kinematics(model)
    owned = any(
        tree in owned_trees for tree in holders.get(body_of_frame(frame_node, model.graph), ())
    )
    return replace(frame, id=f"{runtime_prefix}{frame.id}" if owned else frame.id)


_WORLD_OUTPUTS = (
    (GEOM_COORD.PoseCoordinate, quantities.pose),
    (GEOM_COORD.VelocityTwistCoordinate, quantities.velocity_twist),
    *((rdf_type, quantities.joint_quantity) for rdf_type in quantities.JOINT_QUANTITY_TYPES),
    (RBDYN_COORD.WrenchCoordinate, quantities.wrench),
)


def observes_in_frame(model, node, type_, chain, runtime_prefix, owned_trees, agent=None):
    """Whether an observation is stated in this solver's reference frame, and in which frame node."""
    graph = model.graph
    _trees, holders = scene_kinematics(model)
    if type_ in quantities.JOINT_QUANTITY_TYPES:
        joint = graph.value(node, KC_STAT["of-joint"])
        owned = any(tree in owned_trees for tree in holders.get(joint, ()))

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
            return graph.value(observer, AGN["of-agent"]) == agent, frame_node
        observation = ensure_one_typed_subject_uri(
            graph, node, SOSA.observedProperty, SOSA.Observation
        )
        sensor = graph.value(observation, SOSA.madeBySensor) if observation is not None else None
        sensor_frame = graph.value(sensor, SENSORS.frame) if sensor is not None else None
        owned = sensor_frame is not None and any(
            tree in owned_trees for tree in holders.get(body_of_frame(sensor_frame, graph), ())
        )

        return owned, frame_node
    if frame_node is None:
        return True, frame_node
    frame_body = body_of_frame(frame_node, graph)
    on_this_chain = any(tree in owned_trees for tree in holders.get(frame_body, ()))
    prefix = runtime_prefix if on_this_chain else ""
    if f"{prefix}{local_name(frame_body)}" == chain.root:
        return True, frame_node
    if type_ not in (GEOM_COORD.VelocityTwistCoordinate, GEOM_COORD.PoseCoordinate):
        return False, frame_node
    # a twist or pose seen by a frame on this chain is this chain's to answer
    if on_this_chain:
        return True, frame_node
    # still ours when the observed thing sits on this chain
    endpoint = (
        quantities.pose(model, node).of
        if type_ == GEOM_COORD.PoseCoordinate
        else quantities.velocity_twist(model, node).of
    )
    return endpoint.uri in chain.frames or endpoint.uri in chain.bodies, frame_node


def runtime_output(model, output, type_, node, frame_node, setup: _ChainSetup):
    """One observation with its frames and names rewritten the way the runtime knows them."""
    runtime = (setup.runtime.prefix, setup.runtime.owned_trees)
    if type_ in quantities.JOINT_QUANTITY_TYPES:
        return replace(output, joint_name=f"{setup.runtime.prefix}{output.joint_name}")
    if type_ == GEOM_COORD.VelocityTwistCoordinate:
        seen_by = runtime_frame(model, frame_node, *runtime)
        return replace(output, as_seen_by=seen_by, seen_by_root=seen_by.id == setup.chain.root)
    if type_ == GEOM_COORD.PoseCoordinate and frame_node is not None:
        seen_by = runtime_frame(model, frame_node, *runtime)
        if seen_by.id == setup.chain.root:
            wrt_node = PoseCoordModel(
                node, model.graph, coord_policy=recorded_coord_policy
            ).relation.wrt_id
            wrt = runtime_frame(model, wrt_node, *runtime)
            if wrt.id == setup.chain.root:
                return output
            return replace(output, relative_to_frame=wrt, seen_by_root=True)
        # only wrt == as-seen-by composes as one relative pose
        wrt_id = output.with_respect_to.id if output.with_respect_to is not None else None
        if wrt_id != (output.as_seen_by.id if output.as_seen_by is not None else None):
            return output
        return replace(output, as_seen_by=seen_by, relative_to_frame=seen_by)
    if type_ != RBDYN_COORD.WrenchCoordinate:
        return output

    graph = model.graph
    relation = graph.value(node, RBDYN_COORD["of-wrench"])
    reference_node = graph.value(relation, RBDYN_ENT["reference-point"])
    if output.estimator is not None:
        renamed = {
            "estimator": replace(
                output.estimator, agent=f"{setup.runtime.prefix}{output.estimator.agent}"
            )
        }
    else:
        observation = ensure_one_typed_subject_uri(
            graph, node, SOSA.observedProperty, SOSA.Observation
        )
        sensor_frame_node = graph.value(graph.value(observation, SOSA.madeBySensor), SENSORS.frame)
        renamed = {
            "sensor_name": f"{setup.runtime.prefix}{output.sensor_name}",
            "sensor_frame": runtime_frame(model, sensor_frame_node, *runtime),
        }

    return replace(
        output,
        **renamed,
        reference_point=replace(
            output.reference_point, id=runtime_frame(model, reference_node, *runtime).id
        ),
        as_seen_by=runtime_frame(model, frame_node, *runtime),
    )


def body_name(name: str | None) -> str | None:
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


# authorable but without template wiring: rejected, not silently dropped
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
    # a solver on an unbound agent falls back to the first declared setup
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
    solver_nodes = []
    for handler in sorted(
        graph.subjects(RDF.type, CSTR_HDL["ConstraintHandler"]),
        key=lambda node: int(graph.value(node, APP.order, default=Literal(0)).value),
    ):
        driven = [plan.solver for plan in derivation.controllers_by_handler.get(handler, ())]
        run = set(graph.objects(handler, CSTR_HDL_EXT["runs-solver"])) - set(driven)
        for solver in [*driven, *run]:
            if solver not in solver_nodes and SLV.SolverWithInputAndOutput in get_node_types(
                graph, solver
            ):
                solver_nodes.append(solver)
    unhandled = set(graph.subjects(RDF.type, SLV.SolverWithInputAndOutput)) - set(solver_nodes)
    if unhandled:
        raise ConstraintViolation(
            "solver",
            f"solvers {', '.join(map(str, unhandled))} are run by no constraint handler, so "
            "nothing says when they run",
        )
    for node in solver_nodes:
        setup = setups.get(graph.value(node, AGN["of-agent"]), default_setup)
        outputs = []
        for output_node in graph[node : SLV["output"]]:
            types = get_node_types(graph, output_node)
            read = next((func for type_, func in _WORLD_OUTPUTS if type_ in types), None)
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
        # Vereshchagin root acceleration; RNE's opposite-sign gravity is derived later
        gravity_node = graph.value(node, SLV.gravity)
        # copies: several solver nodes may share one agent's setup
        chain = replace(setup.chain)
        solver = SolverWithInputAndOutput(
            id=model.id(node),
            motion_drivers=constraint_handler.motion_drivers(model, derivation, node, chain),
            output=outputs,
            chain=chain,
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
                constraint_handler.saturation(model, torque_limit)
                if torque_limit is not None
                else None
            ),
        )
        observations = []
        for type_, read in _WORLD_OUTPUTS:
            for observed_node in graph.subjects(RDF.type, type_):
                if (observed_node, RDF.type, SOSA.ObservableProperty) not in graph:
                    continue
                in_frame, frame_node = observes_in_frame(
                    model,
                    observed_node,
                    type_,
                    setup.chain,
                    setup.runtime.prefix,
                    setup.runtime.owned_trees,
                    agent=setup.agent,
                )
                if not in_frame:
                    continue
                output = runtime_output(
                    model, read(model, observed_node), type_, observed_node, frame_node, setup
                )
                frame = output.as_seen_by if isinstance(output, (Pose, SpatialCoordinate)) else None
                if frame_node is None and frame is not None and frame.id != setup.chain.root:
                    continue
                observations.append(output)
        observed = unique_by_id(observations)
        observed_ids = {out.id for out in observed}
        kept = [out for out in solver.output if out.id not in observed_ids]
        solver.output = [out for out in kept + observed if out.id not in detect_pose_ids]
        # a direction in a moving frame must be turned into the root's axes
        root_body = body_name(solver.chain.root)
        for driver in solver.motion_drivers:
            for constraint in (
                *driver.acceleration_constraint,
                *driver.cartesian_acceleration,
                *driver.cartesian_velocity,
            ):
                constraint.base_aligned = (
                    constraint.as_seen_by is None
                    or body_name(constraint.as_seen_by.id) == root_body
                )
        solver.constraint_rows = max(
            (
                len(driver.acceleration_constraint)
                + len(driver.cartesian_acceleration)
                + len(driver.cartesian_velocity)
                for driver in solver.motion_drivers
            ),
            default=0,
        )
        serial_chains.append(solver)
        driven = graph[node : SLV["motion-drivers"] / (SLV["cartesian-force"] | SLV["joint-force"])]
        steps.extend(schedule.of(driven, OPS_GENERIC + OPS_SOLVER))

    platform_force = []
    for node in graph.subjects(RDF.type, SLV["ForceDistributionSolver"]):
        # without its drivers' forces the platform is commanded nothing
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
        unsupported = next(graph.subjects(RDF.type, type_), None)
        if unsupported is not None:
            raise ConstraintViolation(
                "solver",
                f"Mobile-platform solver '{unsupported}' uses algorithm '{label}', which the "
                "motion-spec code generator does not implement yet. Use velocity-composition or "
                "force-distribution instead.",
            )
    if backend == "robif2b":
        unsupported = {
            solver.hardware.model or "(unrecognised)"
            for solver in serial_chains
            if solver.hardware.model not in SUPPORTED_ROBOT_MODELS
        }
        if unsupported:
            raise ConstraintViolation(
                "solver",
                f"Unsupported robot model(s) for robif2b: {', '.join(unsupported)}. "
                f"Supported: {', '.join(SUPPORTED_ROBOT_MODELS)}",
            )

    return Robots(serial_chains, platform_velocity, platform_force, steps)
