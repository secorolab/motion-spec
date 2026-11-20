# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What running the solvers implies, folded on once every solver is known: the runtimes they
share, the world model's port table, the observations the loop answers, the runtime-written
D-blocks, and the joint-space channels each runtime reports.
"""

from __future__ import annotations

import collections

from motion_spec_dsl.rdf_parser.vocab import KC_OP
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import URI_GEOM_TYPE_KTREE
from rdf_utils.namespace import NS_MM_QUDT_QTY, NS_MM_QUDT_UNIT
from rdf_utils.naming import get_valid_var_name
from rdflib import URIRef
from rdflib.namespace import PROV
from scene_dsl.rdf_parser.kinematics import get_kinematic_mappings

from motion_spec.classes.bindings import JointSpaceChannel, WorldPort
from motion_spec.classes.dynamics import JointQuantity
from motion_spec.classes.geometry import VelocityTwist, Wrench
from motion_spec.classes.motion import DataValue
from motion_spec.classes.qudt import QuantityKind, Unit
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.agents import GRIPPER_DEVICES, place_solver_on_chain, scene_kinematics
from motion_spec.rdf_parser.constraint_handler import SOLVER_FAMILIES
from motion_spec.rdf_parser.model import identifier, local_name


def runtime_data(model, serial_chains, control_period_ns: int, platform_uri) -> tuple[list, dict]:
    """The D-blocks the runtime writes that no model entity declares.

    The measured control period and the clock beside it, plus, for every force/torque reading: the
    raw sample, the tare state computed from it, and the counters that report how taring went.
    Each is a contracted D-block: written from a port or computed from a reading, and read
    like any other.

    Returns:
        the members, and the ids of the tare companions minted per wrench reading id

    Raises:
        RuntimeError: the platform or a sensor output has no IRI to derive the value from.
    """
    if not platform_uri:
        raise RuntimeError("measured dt: the execution platform has no IRI to derive a clock from")
    clock_iri = model.register_derived("clock", platform_uri, "clock", PROV.wasDerivedFrom)
    for name in ("clock_time_s", "dt_measured_s"):
        model.register_derived(name, clock_iri, name, PROV.wasDerivedFrom)
    members = [
        DataValue(id="clock_time_s", type="Quantity"),
        DataValue(id="dt_measured_s", type="Quantity", value=control_period_ns * 1e-9),
    ]

    companions_by_reading: dict[str, list[str]] = {}
    for solver in serial_chains:
        for out in solver.output:
            if not isinstance(out, Wrench) or out.id in companions_by_reading:
                continue
            if out.sensor_name:
                companions = _FT_TARE_MEMBERS
            elif out.estimator is not None:
                companions = _ESTIMATE_TARE_MEMBERS
            else:
                continue
            companion_ids = companions_by_reading.setdefault(out.id, [])
            reading_iri = model.iri_of(out.id)
            if reading_iri is None:
                raise RuntimeError(
                    f"tare state: wrench output '{out.id}' has no IRI to derive from"
                )
            for suffix, member_type in companions:
                member_id = f"{out.id}_{suffix}"
                members.append(DataValue(id=member_id, type=member_type))
                companion_ids.append(member_id)
                model.register_derived(member_id, reading_iri, suffix, PROV.wasDerivedFrom)

    return members, companions_by_reading


_FT_TARE_MEMBERS = (
    ("ft_raw", "Wrench"),
    ("ft_bias", "Wrench"),
    ("ft_bias_new", "Wrench"),
    ("ft_bias_prev", "Wrench"),
    ("ft_load", "Wrench"),
    ("ft_payload", "Wrench"),
    ("ft_settle", "IntCounter"),
    ("ft_tares", "IntCounter"),
    ("ft_rejects", "IntCounter"),
    ("ft_confirming", "Bool"),
)

_ESTIMATE_TARE_MEMBERS = (
    ("est_payload", "Wrench"),
    ("est_payload_new", "Wrench"),
    ("est_settle", "IntCounter"),
    ("est_tares", "IntCounter"),
)


# per backend, read off the joint mirror; the world model answers the rest every tick
_STATE_ANSWERED = {
    "mj_kdl": {"VelocityTwist"},
    "robif2b": {"VelocityTwist", "JointPosition", "JointVelocity"},
}


def world_observations(serial_chains) -> list[dict]:
    """Every observation the loop answers, once, with the runtime owner whose frame it is stated in.

    Two solvers driving one chain report the same value from the same root, so the runtime that
    chain is answers it; a claim on it from another runtime, or of a different value, is refused.

    Raises:
        ConstraintViolation: two runtimes, or two different readings, claim one observation.
    """
    claimed: dict[str, dict] = {}
    for solver in serial_chains:
        owner = solver.runtime.owner_id
        for out in solver.world_output:
            claim = {"solver_id": owner, "owner_id": owner, "out": out}
            if claimed.setdefault(out.id, claim) != claim:
                raise ConstraintViolation(
                    "solver",
                    f"observation '{out.id}' is claimed by runtime '{claimed[out.id]['owner_id']}' "
                    f"and by runtime '{owner}' -- one runtime answers it",
                )
    return list(claimed.values())


def world_ports(
    model,
    world_trees,
    scene,
    serial_chains,
    motions,
    perturbation_bodies,
    subscriptions,
    backend: str,
) -> dict:
    """The world model's port table, split by kind: one row per physical variable a provider
    answers.

    `free_roots` and `joints` are measured -- what the kinematics and the world observations
    read; `aux_cmds` and `wrench_cmds` are commanded -- what a backend applies at the end of the
    tick. A chain joint's own measurements and its effort command ride the runtime's
    `ports_<runtime>` array instead, which the backend binds whole.

    Raises:
        ConstraintViolation: a free root nothing places, or a body wrench on a backend that has
            no way to apply one.
    """
    from motion_spec.generation.scene_kdl import segment_of_joint

    placed_by_perception = {
        row["observed_body_segment"]
        for subscription in subscriptions
        for row in subscription["written_poses"]
        if row.get("reframed")
    }
    object_by_body = {obj.body_iri: obj for obj in scene.objects if obj.body_iri}

    body_by_tree = {
        mapping.target_id: mapping.entity
        for models in quantities.scene_instance(model).agent_models.values()
        for asset in models.values()
        for mapping in get_kinematic_mappings(asset, URI_GEOM_TYPE_KTREE)
        if mapping.entity
    }
    _scene_trees, holders = scene_kinematics(model)
    mapped_by_root = {
        tree["root_iri"]: next(
            (
                entity
                for target, entity in body_by_tree.items()
                if target in holders.get(URIRef(tree["root_iri"]), ())
            ),
            None,
        )
        for tree in world_trees
    }
    anchored_root = next(
        (
            tree["root"]
            for tree in world_trees
            if any(segment["joint"] for segment in tree["segments"])
            and tree["root_iri"] not in object_by_body
            and mapped_by_root[tree["root_iri"]] is None
        ),
        None,
    )
    free_roots, joints, aux_cmds, wrench_cmds = [], [], [], []
    for tree in world_trees:
        # nothing measures a free body unless something places it
        if tree["root"] == anchored_root or tree["root"] in placed_by_perception:
            continue
        obj = object_by_body.get(tree["root_iri"])
        mapped_body = mapped_by_root[tree["root_iri"]]
        if obj is None and mapped_body is None:
            raise ConstraintViolation(
                "kinematics",
                f"'{tree['name']}' is a free body the scene does not place, so nothing can "
                "measure it",
            )
        if obj is not None and backend != "mj_kdl":
            raise ConstraintViolation(
                "kinematics",
                f"'{tree['name']}' is a free body nothing places on {backend}: only a "
                "subscription can place a free body on hardware",
            )
        free_roots.append(
            WorldPort(
                kind="free_root",
                segment=tree["root"],
                mapping=obj.body if obj is not None else mapped_body,
                slot=len(free_roots),
                iri=tree["root_iri"],
            )
        )

    joint_slots: dict[str, int] = {}
    for solver in serial_chains:
        outputs = [
            *solver.output,
            *(out for device in solver.devices for out in device.joint_outputs),
        ]
        for out in outputs:
            if out.type not in _JOINT_OUTPUT_KINDS or out.on_chain:
                continue
            slot = joint_slots.get(out.joint_name)
            if slot is None:
                slot = len(joint_slots)
                joint_slots[out.joint_name] = slot
                joints.append(
                    WorldPort(
                        kind="joint",
                        segment=segment_of_joint(world_trees, out.joint_uri),
                        mapping=out.joint_name,
                        slot=slot,
                        owner_id=solver.runtime.id,
                        iri=out.joint_uri,
                    )
                )
            out.world_slot = slot

    aux_slots: dict[str, int] = {}
    for motion in motions:
        for cmd in motion.forwarded_commands:
            if not cmd.target:
                continue
            if cmd.target not in aux_slots:
                aux_slots[cmd.target] = len(aux_slots)
                aux_cmds.append(
                    WorldPort(
                        kind="aux_cmd",
                        segment="",
                        mapping=cmd.target,
                        slot=aux_slots[cmd.target],
                        owner_id=cmd.robot_id,
                    )
                )
            cmd.world_slot = aux_slots[cmd.target]

    for slot, group in enumerate(perturbation_bodies):
        if backend != "mj_kdl":
            raise ConstraintViolation(
                "perturbation",
                f"perturbation on body '{group['body']}': a perturbation has no actuator on "
                f"{backend}",
            )
        group["slot"] = slot
        wrench_cmds.append(
            WorldPort(kind="wrench_cmd", segment="", mapping=group["body"], slot=slot)
        )

    return {
        "free_roots": free_roots,
        "joints": joints,
        "aux_cmds": aux_cmds,
        "wrench_cmds": wrench_cmds,
    }


def annotate_runtime(
    serial_chains, motions, backend: str, world_index=None, world_trees=(), force_solvers=()
) -> list[dict]:
    """Fold onto each solver what running it implies, once every solver is known.

    Which runtime it shares, whether it owns that runtime, which of its joint outputs a gripper
    device reports rather than the chain, where on its chain every frame it asks kinematics for
    sits, and -- last, so it sees the runtime flags -- the gravity an RNE solver is built with.

    Returns:
        every world-model read the program makes, as `{solver_id, frame_id, segment_name}`, once
        per runtime: the records sharing a chain read the same segment from the same root

    Raises:
        ConstraintViolation: a joint output falls outside the chain with no gripper bound to report
            it, a frame is named that the solver's chain never reaches, a motion declares a
            read-only solver on a runtime commanded elsewhere, or two solvers in one motion
            command one runtime in different modes.
    """
    runtime_by_signature: dict[tuple, str] = {}
    for solver in serial_chains:
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
        solver.runtime.id = runtime_id
        solver.runtime.owner = solver.id == runtime_id
        solver.runtime.owner_id = runtime_id
        # ST4 treats "" as truthy
        solver.hardware.tool_body = solver.hardware.tool_body or None
        solver.hardware.tcp_frame = solver.hardware.tcp_frame or None

    root_by_segment = {}
    for tree in world_trees:
        root_by_segment[tree["root"]] = tree["root"]
        for segment in tree["segments"]:
            root_by_segment[segment["name"]] = tree["root"]
    world_index = world_index or {}
    reads = [
        frame
        for solver in serial_chains
        for frame in place_solver_on_chain(solver, world_index, root_by_segment)
    ]
    # keyed by segment: two arms carry the same frame name
    for solver in force_solvers:
        for frame in [solver.force.as_seen_by, *(spec.force.as_seen_by for spec in solver.forces)]:
            if frame is None or not frame.uri:
                continue
            segment = world_index.get(frame.uri)
            if segment is None:
                raise ConstraintViolation(
                    "geometry",
                    f"'{frame.id}' is absent from every tree the world model "
                    f"holds, so force solver '{solver.id}' cannot read it.",
                )
            solver.world_keys[frame.uri] = f"{solver.id}_{get_valid_var_name(segment)}"
            reads.append(
                {
                    "solver_id": solver.id,
                    "frame_id": get_valid_var_name(segment),
                    "segment_name": segment,
                    "tree_root": root_by_segment.get(segment),
                }
            )

    claimed: dict[tuple[str, str], dict] = {}
    for frame in reads:
        key = (frame["solver_id"], frame["frame_id"])
        if claimed.setdefault(key, frame) != frame:
            raise ConstraintViolation(
                "geometry",
                f"frame '{key[1]}' of '{key[0]}' is read as both {claimed[key]} and {frame}",
            )
    world_frames = list(claimed.values())

    modes = collections.defaultdict(set)
    mode_by_driver = {}
    for solver in serial_chains:
        if solver.algorithm.read_only:
            continue
        for driver in solver.motion_drivers:
            joint_impedance = driver.pose_target is not None and driver.pose_target.stiffness
            mode = "torque" if joint_impedance else solver.algorithm.command_mode
            mode_by_driver[driver.id] = mode
            modes[solver.runtime.id].add(mode)
    commanding = set(modes)
    fpk = SOLVER_FAMILIES[KC_OP.ForwardPositionKinematics]
    from_state = _STATE_ANSWERED.get(backend, {"VelocityTwist"})
    for solver in serial_chains:
        found = modes[solver.runtime.id]
        solver.runtime.command_mode = (
            next(iter(found)) if len(found) == 1 else "position" if found else None
        )
        twist = next((out for out in solver.output if isinstance(out, VelocityTwist)), None)
        if solver.algorithm is fpk and twist is not None:
            raise ConstraintViolation(
                "solver",
                f"solver '{solver.id}' is forward position kinematics, but twist "
                f"'{twist.id}' is read through it; name 'fvk' to read twists.",
            )
        # robif2b cannot measure a mimic joint; the bound gripper answers it
        if backend == "robif2b":
            chain_joints = {f"{solver.runtime.prefix}{joint}" for joint in solver.chain.joints}
            outputs, gripper_outputs = [], []
            for out in solver.output:
                joint = str(out.joint_name) if isinstance(out, JointQuantity) else ""
                out_type = out.type
                if out_type in _GRIPPER_ONLY_OUTPUTS and joint in chain_joints:
                    raise ConstraintViolation(
                        "solver",
                        f"{_OUTPUT_KEYWORD[out_type]} '{out.id}' reads chain joint '{joint}'; "
                        "only a bound gripper reports it",
                    )
                if out_type not in _OUTPUT_KEYWORD or joint in chain_joints:
                    outputs.append(out)
                    continue
                gripper_outputs.append(out)
                if not any(device.kind in GRIPPER_DEVICES for device in solver.devices):
                    raise ConstraintViolation(
                        "solver",
                        f"{_OUTPUT_KEYWORD[out_type]} '{out.id}' reads joint '{joint}', which is "
                        f"outside solver '{solver.id}'s chain and no gripper device is bound to "
                        "report it",
                    )
            solver.output = outputs
            for device in solver.devices:
                if device.kind in GRIPPER_DEVICES:
                    device.joint_outputs = gripper_outputs
        solver.state_output = [out for out in solver.output if out.type in from_state]
        solver.world_output = [out for out in solver.output if out.type not in from_state]
        index_chain_joints(solver)
    by_id = {solver.id: solver for solver in serial_chains}
    for motion in motions:
        mode_by_runtime = {}
        for solver in motion.serial_chain_solvers:
            canonical = by_id[solver.id]
            solver.command_mode = mode_by_driver.get(solver.motion_driver.id)
            held = mode_by_runtime.get(canonical.runtime.id)
            if held and solver.command_mode and held != solver.command_mode:
                raise ConstraintViolation(
                    "solver",
                    f"motion '{motion.id}' commands arm '{canonical.runtime.id}' with both "
                    f"{held} and {solver.command_mode}; an arm takes one mode at a time",
                )
            if solver.command_mode:
                mode_by_runtime[canonical.runtime.id] = solver.command_mode
            solver.output = canonical.output
            solver.gripper_joint_outputs = [
                out
                for device in canonical.devices
                if device.kind in GRIPPER_DEVICES
                for out in device.joint_outputs
            ]
            # commanded or held, never both: a torque-streamed arm drops
            if solver.read_only and canonical.runtime.id in commanding:
                raise ConstraintViolation(
                    "solver",
                    f"motion '{motion.id}' only reads arm '{canonical.runtime.id}' through "
                    f"'{solver.id}' ({canonical.algorithm_name}), but another motion drives it "
                    f"with {canonical.runtime.command_mode}; name a driving algorithm in both or "
                    "forward kinematics in both",
                )
        for command in motion.forwarded_commands:
            command.robot_id = by_id[command.robot_id].runtime.id
    health_index = 0
    for solver in serial_chains:
        if not solver.runtime.owner:
            continue
        for device in solver.devices:
            if device.kind == "RobotiqFT300s":
                device.health_index = health_index
                health_index += 1
    for solver in serial_chains:
        if solver.derived_root_acceleration:
            solver.gravity = list(solver.derived_root_acceleration)
            solver.gravity_compensation = [
                -component or 0.0 for component in solver.derived_root_acceleration
            ]
        # without gravity the observer reports the arm's weight as a push
        if solver.gravity:
            continue
        for out in solver.world_output:
            if isinstance(out, Wrench) and out.estimator is not None:
                raise ConstraintViolation(
                    "solver",
                    f"wrench '{out.id}' is estimated on '{solver.id}', which declares no "
                    "gravity; the momentum observer needs the chain's gravity vector",
                )

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


_OUTPUT_KEYWORD = {
    "JointPosition": "joint-position",
    "JointVelocity": "joint-velocity",
    "JointForce": "joint-force",
}

_JOINT_OUTPUT_KINDS = set(_OUTPUT_KEYWORD)

_GRIPPER_ONLY_OUTPUTS = {"JointVelocity"}


def chain_joint_index(solver, joint_name: str) -> int | None:
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


def index_chain_joints(solver) -> None:
    """Resolve every joint the generated code addresses to its index on this chain.

    The alternative is handing the name to the generated program and searching the built chain on
    the first tick, which can only fail where nothing can act on it -- and searching by name has
    to match loosely enough that it can answer with the wrong joint, which on the feed-forward
    path means torque on a joint the model never named.

    Raises:
        ConstraintViolation: a joint force names a joint the chain does not articulate, so there
            is no joint-array slot to add it to.
    """
    joint_outputs = [
        *solver.output,
        *(out for device in solver.devices for out in device.joint_outputs),
    ]
    for out in joint_outputs:
        if out.type in _JOINT_OUTPUT_KINDS:
            out.joint_index = chain_joint_index(solver, out.joint_name)
            out.on_chain = out.joint_index is not None
        if out.type == "JointForce" and not out.on_chain:
            raise ConstraintViolation(
                "solver",
                f"joint-force '{out.id}' reads joint '{out.joint_name}', which is not on solver "
                f"'{solver.id}'s chain -- only a chain joint reports its torque",
            )
    for driver in solver.motion_drivers:
        for force in driver.joint_force:
            force.joint_index = chain_joint_index(solver, force.joint_name)
            if force.joint_index is None:
                raise ConstraintViolation(
                    "solver",
                    f"joint force '{force.id}' acts on joint '{force.joint_name}', which is not "
                    f"on solver '{solver.id}'s chain from '{solver.chain.root}' to "
                    f"'{solver.chain.tip}'. A joint force is added to that chain's joint torques, "
                    "so it has to name a joint the chain articulates.",
                )


JOINT_SPACE_CHANNELS = (
    JointSpaceChannel("q", "port", NS_MM_QUDT_QTY["Angle"], NS_MM_QUDT_UNIT["RAD"]),
    JointSpaceChannel(
        "qd", "port", NS_MM_QUDT_QTY["AngularVelocity"], NS_MM_QUDT_UNIT["RAD-PER-SEC"]
    ),
    JointSpaceChannel(
        "qdd", "solver", NS_MM_QUDT_QTY["AngularAcceleration"], NS_MM_QUDT_UNIT["RAD-PER-SEC2"]
    ),
    JointSpaceChannel("tau_ctrl", "solver", NS_MM_QUDT_QTY["Torque"], NS_MM_QUDT_UNIT["N-M"]),
    # under mj_kdl torque control jnt_trq_msr is zero by construction
    JointSpaceChannel(
        "tau_msr", "sensor", NS_MM_QUDT_QTY["Torque"], NS_MM_QUDT_UNIT["N-M"], ("robif2b",)
    ),
)
JOINT_SPACE_COMMAND_CHANNEL = JointSpaceChannel(
    "tau_cmd", "saturation", NS_MM_QUDT_QTY["Torque"], NS_MM_QUDT_UNIT["N-M"]
)


def add_joint_space_mirrors(model, robots, motions, algorithm_data: list, backend: str) -> None:
    """Mirror each runtime's joint-space signals into the algorithm data, as values its chains
    write every tick.

    Keyed by runtime, not by solver: these are the arm's ports, and the command port is
    last-writer-wins, so a runtime-keyed value holds what the port received even when several
    solvers on one runtime run in the same tick. Keying by solver would multiply the values by the
    number of motions to hold the same ports.
    """
    copies_by_id: dict[str, list] = {}
    for motion in motions:
        for solver in motion.serial_chain_solvers:
            copies_by_id.setdefault(solver.id, []).append(solver)

    by_runtime: dict[str, list] = {}
    for solver in robots.serial_chains:
        by_runtime.setdefault(solver.runtime.id or solver.id, []).append(solver)

    present = {item.id for item in algorithm_data}
    for runtime_id, solvers in by_runtime.items():
        joints = [f"{solvers[0].runtime.prefix}{joint}" for joint in solvers[0].chain.joints]
        if not joints:
            raise RuntimeError(
                f"joint-space mirror: solver '{solvers[0].id}' has no chain joints; the shared "
                "ids are compile-time names, so a wrong joint count mislabels every channel"
            )
        saturations = {
            solver.torque_saturation.id for solver in solvers if solver.torque_saturation
        }
        available = tuple(
            channel
            for channel in JOINT_SPACE_CHANNELS
            if channel.backends is None or backend in channel.backends
        )
        channels = available + ((JOINT_SPACE_COMMAND_CHANNEL,) if saturations else ())
        writer_id = {
            "port": runtime_id,
            "sensor": runtime_id,
            "solver": solvers[0].id if len(solvers) == 1 else None,
            "saturation": next(iter(saturations)) if len(saturations) == 1 else None,
        }
        parent = model.iri_of(runtime_id) or model.iri_of(solvers[0].id)
        if parent is None:
            raise RuntimeError(
                f"joint-space mirror: runtime '{runtime_id}' has no IRI to derive from"
            )

        ids_by_channel: dict[str, list] = {channel.name: [] for channel in channels}
        for index, joint in enumerate(joints):
            for channel in channels:
                member_id = f"{runtime_id}_{channel.name}_{identifier(joint)}"
                if member_id in present:
                    raise RuntimeError(
                        f"joint-space mirror: id '{member_id}' collides with an existing D-block"
                    )
                present.add(member_id)
                algorithm_data.append(
                    DataValue(
                        id=member_id,
                        type="Quantity",
                        role="joint_space",
                        writer={"kind": channel.writer, "id": writer_id[channel.writer]},
                        quantity_kind=QuantityKind(
                            identifier(local_name(channel.quantity_kind)),
                            str(channel.quantity_kind),
                        ),
                        unit=Unit(identifier(local_name(channel.unit)), str(channel.unit)),
                        runtime=runtime_id,
                        channel=channel.name,
                        joint=joint,
                    )
                )
                model.register_derived(
                    member_id, parent, f"{channel.name}-{identifier(joint)}", PROV.wasDerivedFrom
                )
                ids_by_channel[channel.name].append({"id": member_id, "index": index})

        samples = [
            {"id": entry["id"], "channel": channel.name, "index": entry["index"]}
            for channel in available
            for entry in ids_by_channel[channel.name]
        ]
        command_ids = ids_by_channel.get(JOINT_SPACE_COMMAND_CHANNEL.name, [])
        for solver in solvers:
            for target in (solver, *copies_by_id.get(solver.id, ())):
                target.joint_space_samples = samples
                target.joint_space_cmd_samples = command_ids if solver.torque_saturation else []
