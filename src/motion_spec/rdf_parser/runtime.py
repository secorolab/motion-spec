# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What running the solvers implies, folded on once every solver is known: the runtimes they
share, the world model's port table, the observations the loop answers, the runtime-written
D-blocks, and the joint-space channels each runtime reports.
"""

from __future__ import annotations

from motion_spec_dsl.rdf_parser.vocab import AGN, GEOM_ENT
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.namespace import NS_MM_QUDT_QTY, NS_MM_QUDT_UNIT
from rdf_utils.naming import get_valid_var_name
from rdf_utils.uri import iri_is_descendant
from rdflib.namespace import PROV, RDF

from motion_spec.classes.bindings import JointSpaceChannel, WorldPort
from motion_spec.classes.dynamics import JointQuantity
from motion_spec.classes.geometry import VelocityTwist, Wrench
from motion_spec.classes.motion import DataValue
from motion_spec.classes.qudt import QuantityKind, Unit
from motion_spec.rdf_parser.agents import GRIPPER_DEVICES, model_mappings, place_solver_on_chain
from motion_spec.rdf_parser.model import identifier, local_name


def runtime_data(model, serial_chains, control_period_ns: int, platform_uri) -> list:
    """The D-blocks the runtime writes that no model entity declares.

    The measured control period and the clock beside it, plus, for every force/torque reading: the
    raw sample, the tare state computed from it, and the counters that report how taring went.
    Each is a contracted D-block: written from a port or computed from a reading, and read
    like any other.

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
        DataValue(id="clock_time_s", type="Quantity"),
        DataValue(id="dt_measured_s", type="Quantity", value=control_period_ns * 1e-9),
    ]

    seen = set()
    for solver in serial_chains:
        for out in solver.output:
            if not isinstance(out, Wrench) or out.id in seen:
                continue
            if out.sensor_name:
                companions = _FT_TARE_MEMBERS
            elif out.estimator is not None:
                companions = _ESTIMATE_TARE_MEMBERS
            else:
                continue
            seen.add(out.id)
            # The tare state is computed from the reading, so it derives from that output's node.
            reading_iri = model.iri_of(out.id)
            if reading_iri is None:
                raise RuntimeError(
                    f"tare state: wrench output '{out.id}' has no IRI to derive from"
                )
            for suffix, member_type in companions:
                member_id = f"{out.id}_{suffix}"
                members.append(DataValue(id=member_id, type=member_type))
                model.register_derived(member_id, reading_iri, suffix, PROV.wasDerivedFrom)

    return members


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

# An estimate has no sensor zero or load to take out; its tare is the held weight it re-converges to.
_ESTIMATE_TARE_MEMBERS = (
    ("est_payload", "Wrench"),
    ("est_payload_new", "Wrench"),
    ("est_settle", "IntCounter"),
    ("est_tares", "IntCounter"),
)


# What answers an observation, per backend. A twist is derived by KDL from the solver's joint
# mirror on either backend; a joint position is read off the simulator on mj_kdl but off that
# mirror on robif2b. Everything else is answered by the world model or a sensor, so the loop can
# compute it whether or not a motion that reads it is running.
_STATE_ANSWERED = {
    "mj_kdl": {"VelocityTwist"},
    "robif2b": {"VelocityTwist", "JointPosition", "JointVelocity", "JointCurrent"},
}


def _split_outputs(solver, backend: str) -> None:
    """Split a solver's observations into the ones the loop answers and the ones its motion does."""
    from_state = _STATE_ANSWERED.get(backend, {"VelocityTwist"})
    solver.state_output = [out for out in solver.output if out.type in from_state]
    solver.world_output = [out for out in solver.output if out.type not in from_state]


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

    # A subscription binds the base of what it observes itself, so that root has its provider.
    placed_by_perception = {
        row["observed_body_segment"]
        for subscription in subscriptions
        for row in subscription["written_poses"]
        if row.get("reframed")
    }
    object_by_body = {obj.body_iri: obj for obj in scene.objects if obj.body_iri}

    # Which MJCF body a free-floating agent tree is spawned as, so a platform that articulates
    # can still be placed. Keyed by the ktree the agent's model maps.
    body_by_tree = {
        str(target): entity
        for agent_model in model.graph.subjects(RDF.type, AGN["AgentModel"])
        for target, entity in model_mappings(model, agent_model, GEOM_ENT.KinematicTree)
        if entity
    }
    anchored_root = next(
        (
            tree["root"]
            for tree in world_trees
            if any(segment["joint"] for segment in tree["segments"])
            and object_by_body.get(tree["root_iri"]) is None
            and not any(iri_is_descendant(target, tree["root_iri"]) for target in body_by_tree)
        ),
        None,
    )
    free_roots, joints, aux_cmds, wrench_cmds = [], [], [], []
    for tree in world_trees:
        # Trees are maximal joint-connected sets, so every tree but the anchored one stands on a
        # root no joint holds -- a mobile platform included, which articulates below that root.
        # Nothing measures a free body unless something places it.
        if tree["root"] == anchored_root:
            continue
        if tree["root"] in placed_by_perception:
            continue
        obj = object_by_body.get(tree["root_iri"])
        mapped_body = next(
            (
                entity
                for target, entity in body_by_tree.items()
                if iri_is_descendant(target, tree["root_iri"])
            ),
            None,
        )
        if obj is None and mapped_body is not None:
            free_roots.append(
                WorldPort(
                    kind="free_root",
                    segment=tree["root"],
                    mapping=mapped_body,
                    slot=len(free_roots),
                    iri=tree["root_iri"],
                )
            )
            continue
        if obj is None:
            raise ConstraintViolation(
                "kinematics",
                f"'{tree['name']}' is a free body the scene does not place, so nothing can "
                "measure it",
            )
        if backend != "mj_kdl":
            raise ConstraintViolation(
                "kinematics",
                f"'{tree['name']}' is a free body nothing places on {backend}: only a "
                "subscription can place a free body on hardware",
            )
        free_roots.append(
            WorldPort(
                kind="free_root",
                segment=tree["root"],
                mapping=obj.body,
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
            it, a frame is named that the solver's chain never reaches, or a motion declares a
            read-only solver on a runtime torque-commanded elsewhere.
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
            solver.hardware.tcp_frame,
        )
        runtime_id = runtime_by_signature.setdefault(signature, solver.id)
        owner_by_runtime.setdefault(runtime_id, solver.id)
        solver.runtime.id = runtime_id
        solver.runtime.owner = solver.id == owner_by_runtime[runtime_id]
        solver.runtime.owner_id = owner_by_runtime[runtime_id]
        # ST4's <if(x)> treats "" as truthy, so a bare robot's empty tool fields must be None.
        solver.hardware.tool_body = solver.hardware.tool_body or None
        solver.hardware.tcp_frame = solver.hardware.tcp_frame or None

    root_by_segment = {
        name: tree["root"]
        for tree in world_trees
        for name in (tree["root"], *(segment["name"] for segment in tree["segments"]))
    }
    reads = [
        frame
        for solver in serial_chains
        for frame in place_solver_on_chain(solver, world_index or {}, root_by_segment)
    ]
    # A force distribution sums commanded wrenches into its own frame, so it reads both frames
    # from the world model. Keyed by segment, not by frame id: two arms carry one `bracelet_link`.
    for solver in force_solvers:
        for frame in [solver.force.as_seen_by, *(spec.force.as_seen_by for spec in solver.forces)]:
            if frame is None or not frame.uri:
                continue
            segment = (world_index or {}).get(frame.uri)
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

    # Keyed by runtime owner and frame: all the records on one runtime read the same index.
    claimed: dict[tuple[str, str], dict] = {}
    for frame in reads:
        key = (frame["solver_id"], frame["frame_id"])
        if claimed.setdefault(key, frame) != frame:
            raise ConstraintViolation(
                "geometry",
                f"frame '{key[1]}' of '{key[0]}' is read as both {claimed[key]} and {frame}",
            )
    world_frames = list(claimed.values())

    # Runtimes a dynamics solver torque-streams; the rest are only read, so they hold position.
    commanding = {solver.runtime.id for solver in serial_chains if not solver.algorithm.read_only}
    for solver in serial_chains:
        solver.runtime.commanded = solver.runtime.id in commanding
        _refuse_twist_without_velocity_kinematics(solver)
        _refuse_unreportable_currents(solver, backend)
        _split_gripper_outputs(solver, backend)
        # Last, so it sees the outputs a gripper device took over: what the loop answers is
        # decided from the list as it finally stands.
        _split_outputs(solver, backend)
        index_chain_joints(solver)
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
    # The observer reconstructs the chain's momentum, which is gravity-dependent: without the
    # vector it would report the arm's own weight as an external push.
    for solver in serial_chains:
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


# Diagnostics name the world-block keyword the author wrote, not the parsed record's type.
_OUTPUT_KEYWORD = {
    "JointPosition": "joint-position",
    "JointVelocity": "joint-velocity",
    "JointCurrent": "joint-current",
}

# Every reading addressed by joint rather than by frame; each resolves to a chain index or a port.
_JOINT_OUTPUT_KINDS = set(_OUTPUT_KEYWORD)

# Readings the robif2b arm cannot take on a chain joint: only a bound gripper device answers them.
_GRIPPER_ONLY_OUTPUTS = {"JointVelocity", "JointCurrent"}

# Readings no simulated backend answers, and why; the simulator reads any joint's rate by name.
_SIMULATOR_UNREPORTED = {"JointCurrent": "the simulator reports no motor current"}


def _refuse_twist_without_velocity_kinematics(solver) -> None:
    """Forward position kinematics states poses only.

    Raises:
        ConstraintViolation: the model reads a twist through an FPK solver.
    """
    if solver.algorithm_name != "FPK":
        return
    for out in solver.output:
        if isinstance(out, VelocityTwist):
            raise ConstraintViolation(
                "solver",
                f"solver '{solver.id}' is forward position kinematics, but twist '{out.id}' is "
                "read through it; name 'fvk' to read twists.",
            )


def _refuse_unreportable_currents(solver, backend: str) -> None:
    """A gripper's motor current is a hardware reading no simulated backend answers.

    Raises:
        RuntimeError: the model reads one on a backend that does not measure it.
    """
    if backend == "robif2b":
        return
    for out in solver.output:
        reason = _SIMULATOR_UNREPORTED.get(out.type)
        if reason is not None:
            raise ConstraintViolation("solver", f"{_OUTPUT_KEYWORD[out.type]} '{out.id}': {reason}")


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
        joint = str(out.joint_name) if isinstance(out, JointQuantity) else ""
        out_type = out.type
        if out_type in _GRIPPER_ONLY_OUTPUTS and joint in chain_joints:
            raise ConstraintViolation(
                "solver",
                f"{_OUTPUT_KEYWORD[out_type]} '{out.id}' reads chain joint '{joint}'; only a "
                "bound gripper reports it",
            )
        if out_type not in _OUTPUT_KEYWORD or joint in chain_joints:
            outputs.append(out)
            continue
        gripper_outputs.append(out)
        if not any(device.kind in GRIPPER_DEVICES for device in solver.devices):
            raise ConstraintViolation(
                "solver",
                f"{_OUTPUT_KEYWORD[out_type]} '{out.id}' reads joint '{joint}', which is outside "
                f"solver '{solver.id}'s chain and no gripper device is bound to report it",
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
            out.joint_index = _chain_joint_index(solver, out.joint_name)
            out.on_chain = out.joint_index is not None
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
            # One arm is either torque-streamed or held: a kinematics-only motion on a
            # torque-streamed arm would stage zero torques while active, and the arm drops.
            if solver.read_only and canonical.runtime.id in commanding:
                raise ConstraintViolation(
                    "solver",
                    f"motion '{motion.id}' only reads arm '{canonical.runtime.id}' through "
                    f"'{solver.id}' ({canonical.algorithm_name}), but another motion drives it "
                    "with torque; name a dynamics algorithm in both or kinematics in both",
                )
        for command in motion.forwarded_commands:
            canonical = by_id.get(command.robot_id)
            if canonical is not None:
                command.robot_id = canonical.runtime.id


JOINT_SPACE_CHANNELS = (
    JointSpaceChannel("q", "port", NS_MM_QUDT_QTY["Angle"], NS_MM_QUDT_UNIT["RAD"]),
    JointSpaceChannel(
        "qd", "port", NS_MM_QUDT_QTY["AngularVelocity"], NS_MM_QUDT_UNIT["RAD-PER-SEC"]
    ),
    JointSpaceChannel(
        "qdd", "solver", NS_MM_QUDT_QTY["AngularAcceleration"], NS_MM_QUDT_UNIT["RAD-PER-SEC2"]
    ),
    JointSpaceChannel("tau_ctrl", "solver", NS_MM_QUDT_QTY["Torque"], NS_MM_QUDT_UNIT["N-M"]),
    # robif2b reads eff_msr off the hardware; under mj_kdl torque control jnt_trq_msr mirrors
    # qfrc_actuator and is zero by construction, not by measurement.
    JointSpaceChannel(
        "tau_msr", "sensor", NS_MM_QUDT_QTY["Torque"], NS_MM_QUDT_UNIT["N-M"], ("robif2b",)
    ),
)
# Emitted only where a torque limit is authored; that saturation is its writer, not the solver.
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
        # Channel ids name the runtime's own joints, so the chain's joints get its prefix here.
        joints = [f"{solvers[0].runtime.prefix}{joint}" for joint in solvers[0].chain.joints]
        if not joints:
            raise RuntimeError(
                f"joint-space mirror: solver '{solvers[0].id}' has no chain joints; the shared "
                "ids are compile-time names, so a wrong joint count mislabels every channel"
            )
        # tau_cmd differs from tau_ctrl only where a limit clamps it, so that saturation is its
        # writer -- named only when the runtime carries exactly one.
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
            # None where several instances write the value: no one of them is its writer.
            "solver": next(iter({s.id for s in solvers}), None) if len(solvers) == 1 else None,
            "saturation": next(iter(saturations)) if len(saturations) == 1 else None,
        }
        # Mirrors are keyed by runtime, so they derive from the runtime's own solver node.
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

        # Every solver on the runtime mirrors the same ids: whichever motion is active writes them.
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
