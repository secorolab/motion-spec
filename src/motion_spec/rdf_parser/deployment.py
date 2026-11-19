# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The execution platform: the backend it implies, the deployment config beside it, the agents'
home positions and config poses, and the ROS publishers the deployment turns on.

Nothing derives the platform: it is read as authored, and validated as it is read -- a scene
object on real hardware, an undriven device, an unbound sensor.
"""

from __future__ import annotations

import tomllib
from pathlib import Path

from motion_spec_dsl.rdf_parser.vocab import CSTR, ENV, EXEC, GEOM_ENT, MAP, SOSA
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdf_utils.models.geom_rel import relation_neighbors
from rdf_utils.models.prov import get_file_path
from rdf_utils.models.vocab import URI_GEOM_TYPE_POSE
from rdflib import URIRef
from rdflib.namespace import PROV, RDF, SDO, SSN

from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.agents import DRIVEN_DEVICES, model_mappings
from motion_spec.rdf_parser.model import local_name

# What a quantity is ultimately stated on, and where a walk from a constraint stops.
_GEOMETRIC_ENTITY_TYPES = {
    GEOM_ENT.Point,
    GEOM_ENT.Frame,
    GEOM_ENT.SimplicialComplex,
    GEOM_ENT.RigidBody,
    ENV.RigidObject,
}
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


def platform_facts(graph) -> dict:
    """The execution platform the model in GRAPH declares, as one record every consumer reads.

    The authored node's IRI, its name, whether it is simulated, the backend serving it (None for
    a simulation platform no backend serves) and the deployment config beside it, from the
    location the DSL states for the file. Platform identity comes from here and is never
    re-derived from the backend token or from substrings of an id. A reader of a generation reads
    it from the generation's model document; generation checks it with `read_platform`.
    """
    simulation = next(graph.subjects(RDF.type, EXEC.Simulation), None)
    context = simulation or next(graph.subjects(RDF.type, EXEC.RealWorld), None)
    config = graph.value(context, PROV.wasDerivedFrom) if context is not None else None
    path = get_file_path(graph, config) if config is not None else None
    name = (
        str(graph.value(graph.value(simulation, SSN.deployedOnPlatform), SDO.name) or "")
        if simulation is not None
        else None
    )
    return {
        "uri": str(context) if context is not None else None,
        "name": name,
        "simulated": simulation is not None,
        "backend": SIMULATION_BACKENDS.get(name.casefold()) if name is not None else "robif2b",
        "config": str(path) if path is not None else None,
    }


def read_platform(model) -> dict:
    """The execution platform the model declares, `platform_facts`, checked for generation.

    Raises:
        ConstraintViolation: the model names a simulation platform with no backend, real-world
            execution constrains a scene object, binds a device no backend drives, or reads a
            sensor with no device bound.
    """
    platform = platform_facts(model.graph)
    if not platform["simulated"]:
        real = URIRef(platform["uri"]) if platform["uri"] is not None else None
        reject_scene_objects_on_hardware(model, real)
        if real is not None:
            # The grammar decides what a model may name; this decides what the backend can drive.
            bound = {
                str(model.graph.value(device, SDO.model) or "")
                for device in model.graph.subjects(EXEC["realizes"], None)
            }
            undriven = [name for name in bound if name not in DRIVEN_DEVICES]
            if undriven:
                raise ConstraintViolation(
                    "platform",
                    f"no backend support for device(s): {', '.join(undriven)}. "
                    f"Driven: {', '.join(DRIVEN_DEVICES)}. Remove the binding, or add "
                    "the driver templates before binding it.",
                )
            # In simulation the simulator answers for every sensor; on hardware a reading comes
            # from a device or from uninitialised memory.
            unbound = [
                local_name(sensor)
                for sensor in set(model.graph.objects(None, SOSA.madeBySensor))
                if (None, EXEC["realizes"], sensor) not in model.graph
            ]
            if unbound:
                raise ConstraintViolation(
                    "platform",
                    "real-world execution reads sensor(s) with no device bound: "
                    f"{', '.join(unbound)}. Bind one in the platform block, or stop reading the "
                    "sensor.",
                )
    elif platform["backend"] is None:
        raise ConstraintViolation(
            "platform", f"Unsupported simulation platform '{platform['name']}'."
        )
    return platform


def reject_scene_objects_on_hardware(model, context) -> None:
    """Reject a scene object a constraint is stated on and nothing observes: on hardware its pose
    would come from a simulator that is not running.

    An object the scene only places -- the table the arm stands on, the furniture around it --
    is left alone: no run-time value is read off it. So is one a perception source observes,
    which is the case the simulator was standing in for.
    """
    if context is None:
        return
    graph = model.graph
    constrained = entities_from(
        graph,
        {
            quantity
            for constraint in graph.subjects(RDF.type, CSTR.Constraint)
            for quantity in graph.objects(constraint, CSTR.quantity)
        },
    )
    # A subscriber names the world quantity it observes; the object stands behind that
    # quantity, so the observation is walked down to entities the same way constraints are.
    observed = entities_from(graph, set(graph.objects(None, SOSA.hasFeatureOfInterest)))
    objects = []
    for modelled in graph.subjects(RDF.type, ENV.ModelledObject):
        bodies = [
            body
            for asset in graph.objects(modelled, ENV["has-object-model"])
            for body, _entity in model_mappings(model, asset, GEOM_ENT.RigidBody)
        ]
        entities = {
            graph.value(modelled, ENV["of-object"]),
            *bodies,
            *(frame for body in bodies for frame in graph.objects(body, GEOM_ENT.simplices)),
        }
        if entities & constrained and not entities & observed:
            objects.append(local_name(modelled))
    if objects:
        raise ConstraintViolation(
            "platform",
            f"real-world execution constrains scene object(s) ({', '.join(objects)}) that nothing "
            "observes: their poses come from a simulator, and nothing measures them on hardware. "
            "Subscribe to a perception channel that observes them, drop the constraint, or model "
            "the location as an authored frame.",
        )


def entities_from(graph, starts: set) -> set:
    """The geometric entities reachable from the given nodes, first entity per branch."""
    queue = list(starts)
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


def platform_config(platform: dict) -> dict:
    """The deployment config the exec-context declares, parsed once for every reader of it.

    Raises:
        RuntimeError: the declared config file does not exist.
    """
    if not platform.get("config"):
        return {}
    resolved = Path(platform["config"])
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
    missing = [
        solver.runtime.config_key for solver in owners if solver.runtime.config_key not in homes
    ]
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
    # Anchored at the context's config: provenance in the same dataset derives files too.
    derived = {
        node
        for context in model.graph.subjects(RDF.type, EXEC.ExecutionContext)
        for config_file in model.graph.objects(context, PROV.wasDerivedFrom)
        for node in model.graph.subjects(PROV.wasDerivedFrom, config_file)
    }
    for node in derived:
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
    read from at runtime, and per joint the data values one message reports.

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
        # elsewhere: under robif2b by the device `annotate_runtime` moved it onto, under a
        # simulated backend by the simulator answering for its name. Its name is already
        # runtime-scoped, and only its position is read on either route: neither the serial nor
        # the interconnect gripper reports a joint velocity or effort, and a zero there would
        # claim the finger is standing still and unloaded.
        for out in [*solver.output, *(o for d in solver.devices for o in d.joint_outputs)]:
            if out.type == "JointPosition" and out.joint_index is None:
                joints.append({"name": out.joint_name, "position": out.id})
    if not joints:
        raise ConstraintViolation(
            "platform",
            f"[{ROS_JOINT_STATES_KEY}] is declared, but the model drives no serial chain whose "
            "joints it could report.",
        )

    return {"config_key": ROS_JOINT_STATES_KEY, "joints": joints}


ROS_CLOCK_KEY = "ros.clock"


def ros_clock(platform: dict, config: dict) -> dict | None:
    """The simulation-clock publisher the deployment asked for: the config key its topic and rate
    are read from at runtime.

    Raises:
        ConstraintViolation: the config declares the section but the exec-context declares no
            config to read it from, or the platform is not a simulation.
    """
    # Presence is the switch; an empty section means "on, all defaults".
    if "clock" not in (config.get("ros") or {}):
        return None
    if not platform.get("config"):
        raise ConstraintViolation(
            "platform",
            f"[{ROS_CLOCK_KEY}] needs the exec-context to declare `config:`; the topic and rate "
            "are read from that file at run time.",
        )
    # On hardware the loop already runs on the wall clock every other node reads.
    if not platform.get("simulated", False):
        raise ConstraintViolation(
            "platform",
            f"[{ROS_CLOCK_KEY}] publishes the simulation clock, and this platform is not a "
            "simulation; its loop already runs on the wall clock other nodes read.",
        )
    return {"config_key": ROS_CLOCK_KEY}


ROS_TF_KEY = "ros.tf"


def ros_tf(
    model, platform: dict, config: dict, serial_chains, cameras, world_index: dict, ports: dict
) -> dict | None:
    """The tf publisher the deployment asked for: the config key its topic and rate are read
    from at run time, the anchor segment the forest hangs from, the segments the run observes
    and so never publishes, and the free roots the program measures itself and so does.

    Raises:
        ConstraintViolation: the section is declared but the exec-context declares no config to
            read it from; the model drives no serial chain whose tree it could report; a driven
            chain stands in a tree that is not the anchor's.
    """
    if "tf" not in (config.get("ros") or {}):
        return None
    if not platform.get("config"):
        raise ConstraintViolation(
            "platform",
            f"[{ROS_TF_KEY}] needs the exec-context to declare `config:`; the topic and rate are "
            "read from that file at run time.",
        )
    owners = [solver for solver in serial_chains if solver.runtime.owner]
    if not owners:
        raise ConstraintViolation(
            "platform",
            f"[{ROS_TF_KEY}] is declared, but the model drives no serial chain whose tree it "
            "could report.",
        )
    anchor = world_index[str(quantities.anchor_frame(model))]
    for solver in owners:
        if solver.chain.tree_root != anchor:
            raise ConstraintViolation(
                "platform",
                f"[{ROS_TF_KEY}]: solver '{solver.id}' stands in the tree rooted at "
                f"'{solver.chain.tree_root}', not at the scene's anchor '{anchor}'; the "
                "published forest would hang from two roots",
            )
    observed: dict[str, None] = {}
    for camera in cameras:
        if camera.topic:
            # Perception places the camera, so the tree publisher leaves out its segment and those
            # of every frame the scene poses against it -- its optical frame. The built tree hangs
            # each frame off its body, so what is posed against what is read off the scene.
            frames = [URIRef(camera.frame_uri)]
            for frame in frames:
                for posed, _pose in relation_neighbors(
                    frame, URI_GEOM_TYPE_POSE, model.graph, reverse=True
                ):
                    if posed not in frames:
                        frames.append(posed)
            for frame in frames:
                segment = world_index.get(str(frame))
                if segment is None:
                    raise ConstraintViolation(
                        "platform",
                        f"[{ROS_TF_KEY}]: camera '{camera.id}' is observed on '{camera.topic}' but "
                        f"'{frame}' is no segment of a built tree",
                    )
                observed.setdefault(segment, None)
    return {
        "config_key": ROS_TF_KEY,
        "anchor": anchor,
        "observed": list(observed),
        "free_roots": [port.segment for port in ports["free_roots"]],
    }
