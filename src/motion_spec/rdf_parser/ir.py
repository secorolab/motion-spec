# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The lowering pipeline.

One forward pass from a model manifest to the published IR. Reading this file top to bottom is
reading the data flow; it holds no derivation of its own, and nothing else in the package is
reachable except through it.
"""

from __future__ import annotations

from motion_spec.rdf_parser import (
    communication,
    constraint_handler,
    coordination,
    operations,
    quantities,
    resources,
)
from rdflib.namespace import PROV

from motion_spec.classes.motion import BlackboardValue
from motion_spec.rdf_parser.model import load_model

_ALL_OPERATORS = operations.OPS_GENERIC + operations.OPS_SOLVER + operations.OPS_HANDLER


def generate_ir(manifest_path) -> dict:
    """Lower a model manifest to the IR codegen renders from.

    Parameters:
        manifest_path: path to the `<model>-app.ld.json` the DSL generated

    Returns:
        the IR, sectioned by the 5Cs plus the resources the program commands; complete by
        construction, so codegen loads it and renders with no derivation pass of its own
    """
    from motion_spec.generation.scene_kdl import model_stem

    model = load_model(manifest_path)
    operations.normalize(model)

    # The platform, the scene and the FSM are pure functions of the graph, and everything the
    # pass builds afterwards is shaped by them.
    platform = resources.read_platform(model)
    platform_config = resources.platform_config(platform)
    backend = platform["backend"]
    scene = resources.read_scene(model)
    fsm = coordination.read_fsm(model)
    derivation = constraint_handler.solver_derivation_context(model)
    setups, _ordered, world_trees = resources.robot_setups(model)

    # One scope for the whole active block: a step reachable from both a solver and a handler is
    # emitted once, and the four sections are only ever read as their union.
    schedule = operations.Schedule(model)
    # A pose perception writes has one producer -- the detect result or the subscription. The
    # kinematics must not also write it, so the chains are built knowing which poses are not theirs.
    perceived_pose_ids = frozenset(
        row["pose_id"]
        for rows in quantities.perceived_written_poses(model).values()
        for row in rows
    )
    robots = resources.build_robots(
        model, schedule, setups, derivation, scene.objects, backend, perceived_pose_ids
    )
    handlers, handler_steps = coordination.build_constraint_handlers(model, schedule, derivation)
    coordination.assign_event_indexes(handlers)

    closures = operations.build_closures(model, _ALL_OPERATORS)
    constraint_handler.augment_closures(model, derivation, closures)
    views = quantities.read_views(model)
    data_structures = quantities.read_data_structures(model)
    constraint_handler.augment_data(model, derivation, data_structures, views)
    computation = quantities.build_indexes(model, closures, data_structures, views)
    # After pose components exist, before motions are built: a path's goal is one of them.
    operations.resolve_closure_operands(closures, computation.indexes, data_structures)

    motions, fsm_meta = coordination.build_motions(
        model, handlers, robots, computation, derivation, fsm
    )
    world_frames = resources.annotate_runtime(robots.serial_chains, motions, backend)

    if scene.timestep_s <= 0:
        raise ValueError("ENVIRONMENT timestep must be positive.")
    control_period_ns = round(scene.timestep_s * 1e9)

    action_clients = communication.ros_action_clients(model)
    # A subscription places its detections through the world model, so it is built against the
    # same segment names the chains resolved against.
    subscriptions = communication.ros_subscriptions(model, resources.tree_segments(setups))
    standing = communication.ros_standing(model, data_structures, control_period_ns)
    clients_by_motion: dict[str, list] = {}
    for client in action_clients:
        clients_by_motion.setdefault(client["motion"], []).append(client)
    for motion in motions:
        motion.action_clients = clients_by_motion.get(motion.motion_id, [])
        motion.has_action_clients = bool(motion.action_clients)
    shared_data = quantities.filter_shared_data(
        data_structures,
        robots.schedule_steps + handler_steps,
        closures,
        views,
        {out.id for solver in robots.serial_chains for out in solver.output}
        | perceived_pose_ids
        # A standing publish is the only reader of what it reports, and it reads it off the
        # blackboard: without this the quantity drops out and the message has nothing to carry.
        | {publish["value_id"] for publish in standing},
    )
    shared_data += resources.shared_runtime_members(
        model, robots.serial_chains, control_period_ns, platform.get("uri")
    )
    # A snapshot the shared context declares is captured once for the run, so its guard belongs
    # to the run and not to whichever motion happened to reach it first.
    for target_id in sorted({s.target_id for motion in motions for s in motion.task_snapshots}):
        node = model.node_by_id.get(target_id)
        if node is None:
            raise RuntimeError(f"task snapshot '{target_id}' has no node to derive its guard from")
        model.register_derived(f"{target_id}_captured", node, "captured", PROV.wasDerivedFrom)
        # Carries its initial value, which is also what gives it a dataflow contract: a
        # member nothing produces is pruned as absent.
        shared_data.append(BlackboardValue(id=f"{target_id}_captured", type="Bool", value=False))

    config_poses = resources.config_poses(model, platform_config)
    introspection = communication.build_introspection(
        model,
        motions,
        computation,
        shared_data,
        robots,
        scene,
        platform,
        control_period_ns,
        backend,
        action_clients,
        subscriptions,
        config_poses,
    )

    return {
        "configuration": {
            "control_period_ns": control_period_ns,
            "backend": backend,
            # The model's own name, so a backend derives its artifact filenames instead of the
            # IR carrying one backend's spelling of them.
            "model_name": model_stem(model.app_path),
            # The authored execution platform, so provenance and the runtime graph read the
            # model's own answer instead of matching substrings of a derived id.
            "platform": platform,
            "agent_homes": resources.agent_home_positions(
                platform, robots.serial_chains, platform_config
            ),
            "config_poses": config_poses,
            "trace": resources.TRACE_DISABLED,
        },
        "resources": _resources_section(robots, world_trees, world_frames),
        "composition": {"scene": scene},
        "computation": _computation_section(
            closures, views, shared_data, motions, computation.indexes.pose_components
        ),
        "coordination": _coordination_section(motions, fsm, fsm_meta),
        "communication": _communication_section(
            introspection,
            motions,
            resources.ros_joint_states(platform, platform_config, robots.serial_chains),
            action_clients,
            communication.action_server(model, fsm),
            subscriptions,
            standing,
        ),
    }


def _resources_section(robots, world_trees, world_frames) -> dict:
    """Every actuated resource the program commands, plus the by-kind cuts of it.

    An arm and a wheeled base are both actuated resources with kinematics, solvers and devices, so
    they ride in one kind-tagged collection; the views are filtered here because ST4 cannot
    filter, and each is absent rather than empty when the model has none of that kind.

    `world_trees` is every distinct scene tree, once -- several robots may share one -- and
    `world_frames` every pose the one world model is asked for.
    """
    every = [*robots.serial_chains, *robots.platform_velocity, *robots.platform_force]
    by_kind = {}
    serial_chains = [robot for robot in every if robot.kind == "serial_chain"]
    if serial_chains:
        by_kind["serial_chain"] = serial_chains
    if any(robot.kind == "mobile_base" for robot in every):
        by_kind["mobile_base"] = {
            "velocity_solvers": robots.platform_velocity,
            "force_solvers": robots.platform_force,
        }

    # Every device and sensor kind the model binds, as a membership map: templates emit code for
    # a kind only when the platform block or the scene names it.
    device_kinds = sorted(
        {device.kind for robot in every for device in getattr(robot, "devices", [])}
        | {sensor.type for robot in every for sensor in getattr(robot, "sensors", [])}
    )

    # `by_id` is how a per-motion solver slice resolves everything the solver owns.
    section = {
        "robots": every,
        "by_kind": by_kind,
        "by_id": robots.by_id,
        "device_kinds": {kind: True for kind in device_kinds},
        # One entry per shared observation, not per solver that could answer it: several
        # solvers drive the same chain, and computing a value once per solver would repeat the
        # same reading -- ten times over, in a scene with ten motions.
        "world_observations": resources.world_observations(robots.serial_chains),
    }
    if world_trees:
        section["world_trees"] = [
            {"name": tree["name"], "cpp_name": tree["cpp_name"]} for tree in world_trees
        ]
    if world_frames:
        section["world_frames"] = world_frames

    return section


def _computation_section(closures, views, shared_data, motions, pose_components) -> dict:
    """What is computed each tick, and the blackboard it lives on."""
    section = {
        "shared_data": shared_data,
        "closures": closures,
        "views": quantities.views_for_access(
            views, shared_data, motions, closures, pose_components
        ),
    }
    # Elapsed constraints compare seconds from the runtime clock; naming the clock says what it
    # is, where a flag would only have asserted that one is wanted.
    if any(motion.has_elapsed for motion in motions):
        section["clock"] = {"id": "clock", "time_id": "clock_time_s"}

    return section


def _coordination_section(motions, fsm, fsm_meta) -> dict:
    """What runs when: the motions, and the FSM sequencing them when the model imports one."""
    section = {"motions": motions}
    if fsm:
        section["fsm"] = {**fsm, **fsm_meta}

    return section


def _communication_section(
    introspection,
    motions,
    joint_states,
    action_clients=(),
    server=None,
    subscriptions=(),
    standing=(),
) -> dict:
    """What leaves the loop: the frame log, and the ROS topics, goals and results the model asks for."""
    section = {"introspection": introspection}
    publishers = communication.ros_publishers(motions, standing)
    if (
        not publishers
        and joint_states is None
        and not action_clients
        and server is None
        and not subscriptions
    ):
        return section
    packages = {publisher["pkg"] for publisher in publishers}
    ros = {"publishers": publishers, "node_name": "motion_spec_monitor"}
    # A run belongs to a scenario where a message carries one, and always where a goal names it.
    if any(publisher["auto_context_id"] for publisher in publishers) or server is not None:
        ros["scenario_context_id"] = True
    if server is not None:
        ros["action_server"] = server
        packages.update(server["packages"])
    if joint_states is not None:
        ros["joint_states"] = joint_states
        packages.add("sensor_msgs")
    if action_clients:
        ros["action_clients"] = action_clients
        # Whatever the goal and the result reach into, not just the package the action lives in.
        packages.update(pkg for client in action_clients for pkg in client["packages"])
    if subscriptions:
        ros["subscriptions"] = subscriptions
        packages.update(pkg for sub in subscriptions for pkg in sub["packages"])
    if standing:
        ros["standing"] = standing
        packages.update(pkg for publish in standing for pkg in publish["packages"])
    ros["packages"] = sorted(packages)
    section["ros"] = ros

    return section
