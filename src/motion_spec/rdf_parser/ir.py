# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The lowering pipeline.

One forward pass from a loaded model to the published IR. Reading this file top to bottom is
reading the data flow; it holds no derivation of its own, and nothing else in the package is
reachable except through it.
"""

from __future__ import annotations

from rdflib.namespace import PROV

from motion_spec.classes.geometry import Orientation, Position
from motion_spec.classes.motion import DataValue
from motion_spec.classes.qudt import FreeVector, Quantity, SetpointQuantity
from motion_spec.rdf_parser import (
    agents,
    communication,
    constraint_handler,
    coordination,
    deployment,
    mobile_base,
    operations,
    quantities,
    runtime,
)
from motion_spec.rdf_parser.data_access import (
    analyse_data_access,
    build_data_access,
    build_indexes,
    filter_algorithm_data,
)
from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.sampling import sampled_quantities
from motion_spec.rdf_parser.scene import read_scene
from motion_spec.rdf_parser.views import read_data_structures, read_views, views_for_access

_ALL_OPERATORS = operations.OPS_GENERIC + operations.OPS_SOLVER + operations.OPS_HANDLER


def generate_ir(model: Model, fsm: dict | None = None) -> dict:
    """Lower a loaded model to the IR codegen renders from.

    Parameters:
        fsm: coord-dsl's framed FSM (`gen_json`), whose tokens name its states and events;
            None when the model imports no `.fsm`

    Returns:
        the IR, sectioned by the 5Cs plus the resources the program commands; complete by
        construction, so codegen loads it and renders with no derivation pass of its own
    """
    operations.normalize(model)

    # The platform, the scene and the FSM are pure functions of the graph, and everything the
    # pass builds afterwards is shaped by them.
    platform = deployment.read_platform(model)
    platform_config = deployment.platform_config(platform)
    backend = platform["backend"]
    setups, _ordered, world_trees = agents.robot_setups(model)
    scene = read_scene(model, world_trees)
    derivation = constraint_handler.solver_derivation_context(model)
    sampling = sampled_quantities(model, world_trees)

    # One scope for the whole active block: a step reachable from both a solver and a handler is
    # emitted once, and the four sections are only ever read as their union.
    schedule = operations.Schedule(model)
    # A pose perception writes has one writer -- the detect result or the subscription. The
    # kinematics must not also write it, so the chains are built knowing which poses are not theirs.
    perceived_pose_ids = {
        row["pose_id"]
        for rows in quantities.perceived_written_poses(model).values()
        for row in rows
    }
    robots = agents.build_robots(model, schedule, setups, derivation, backend, perceived_pose_ids)
    # Every scene element the world model holds, by IRI: a world read resolves against all of
    # them, not only the tree the reading solver's chain is sliced from.
    world_index = agents.tree_segments(model, setups, world_trees)
    handlers, handler_steps = coordination.build_constraint_handlers(model, schedule, derivation)
    coordination.assign_event_indexes(handlers)

    functions = operations.build_functions(model, _ALL_OPERATORS)
    constraint_handler.augment_functions(model, derivation, functions)
    views = read_views(model)
    data_structures = read_data_structures(model)
    constraint_handler.augment_data(model, derivation, data_structures, views)
    computation = build_indexes(model, functions, data_structures, views)
    # After pose components exist, before motions are built: a path's goal is one of them.
    operations.resolve_function_operands(functions, computation.indexes, data_structures)

    motions, fsm_meta = coordination.build_motions(
        model, handlers, robots, computation, derivation, fsm
    )
    world_frames = runtime.annotate_runtime(
        robots.serial_chains, motions, backend, world_index, world_trees, robots.platform_force
    )
    coordination.annotate_sensor_dependencies(motions, computation)
    runtime.annotate_device_dependencies(robots.serial_chains, motions)

    if scene.timestep_s <= 0:
        raise ValueError("ENVIRONMENT timestep must be positive.")
    control_period_ns = round(scene.timestep_s * 1e9)

    communication.annotate_publish_rates(motions, control_period_ns)

    action_clients = communication.ros_action_clients(model)
    # A subscription places its detections through the world model, so it is built against the
    # same segment names the chains resolved against.
    subscriptions = communication.ros_subscriptions(model, world_index)
    standing_publishes = communication.ros_standing(
        model, data_structures, control_period_ns, world_index
    )
    clients_by_motion: dict[str, list] = {}
    for client in action_clients:
        clients_by_motion.setdefault(client["motion"], []).append(client)
    for motion in motions:
        motion.action_clients = clients_by_motion.get(motion.motion_id, [])
    data_users = (
        robots.schedule_steps,
        handler_steps,
        functions,
        views,
        computation.indexes.pose_components,
        motions,
        robots.serial_chains,
        robots.platform_velocity,
        robots.platform_force,
        perceived_pose_ids,
        standing_publishes,
    )
    algorithm_data = filter_algorithm_data(data_structures, data_users)
    algorithm_data += runtime.runtime_data(
        model, robots.serial_chains, control_period_ns, platform.get("uri")
    )
    # A snapshot the shared context declares is captured once for the run, so its guard belongs
    # to the run and not to whichever motion happened to reach it first.
    latches = {s.target_id: s.captured_id for motion in motions for s in motion.task_snapshots}
    for target_id, captured_id in latches.items():
        node = model.node_by_id.get(target_id)
        if node is None:
            raise RuntimeError(f"task snapshot '{target_id}' has no node to derive its guard from")
        model.register_derived(captured_id, node, "captured", PROV.wasDerivedFrom)
        # Carries its initial value, which is also what gives it a writer: a
        # member nothing produces is pruned as absent.
        algorithm_data.append(DataValue(id=captured_id, type="Bool", value=False))

    # An observation instant is written by the channel that perceives its pose, from the executor
    # thread; until the first reading lands it is minus infinity, so the age read off it is
    # infinite and a freshness gate stays shut.
    observed_at_ids = {
        row["observed_at_id"]
        for channel in (*action_clients, *subscriptions)
        for row in channel["written_poses"]
        if row.get("observed_at_id")
    }
    for observed_at_id in observed_at_ids:
        algorithm_data.append(DataValue(id=observed_at_id, type="Quantity", unset=True))

    # A perturbation's applied wrench and its open/closed flag are runtime state no model entity
    # declares: the wrench the ops compose is stated in the robot's base frame, and what the run
    # has to answer for is the world-frame wrench the simulator was actually given.
    run_perturbations = [p for motion in motions for p in motion.perturbations]
    for perturbation in run_perturbations:
        node = model.node_by_id[perturbation.id]
        model.register_derived(perturbation.applied_id, node, "applied", PROV.wasDerivedFrom)
        model.register_derived(perturbation.active_id, node, "active", PROV.wasDerivedFrom)
        algorithm_data.append(DataValue(id=perturbation.applied_id, type="Wrench"))
        algorithm_data.append(DataValue(id=perturbation.active_id, type="Bool", value=False))
    # The values the computations keep between ticks, then the ones the chains and the action
    # channels write from outside: every D-block the data access constraints connect exists first.
    constraint_handler.add_controller_state(model, functions, algorithm_data, motions)
    constraint_handler.add_control_parameters(model, functions, algorithm_data, motions)
    runtime.add_joint_space_mirrors(model, robots, motions, algorithm_data, backend)
    communication.add_goal_status_values(model, algorithm_data, action_clients)
    perturbation_bodies = _perturbations_by_body(run_perturbations)
    ports = runtime.world_ports(
        model,
        world_trees,
        scene,
        robots.serial_chains,
        motions,
        perturbation_bodies,
        subscriptions,
        backend,
    )

    config_poses = deployment.config_poses(model, platform_config)
    # Which F-block writes and reads each D-block (Bruyninckx 2026, §2.3.4), then what an analysis
    # of that over the schedules concludes; what the run records taps both, and provenance states
    # where each id came from (Aho et al., *Compilers*, 2nd ed., §9.2: analysis precedes codegen).
    data_access = build_data_access(
        algorithm_data,
        functions,
        motions,
        robots.serial_chains,
        views,
        subscriptions,
        config_poses,
        robots.platform_velocity,
        robots.platform_force,
    )
    analysis = analyse_data_access(
        data_access, algorithm_data, motions, functions, robots.serial_chains, views
    )
    telemetry = communication.build_telemetry(
        model, motions, computation, algorithm_data, data_access, analysis, control_period_ns
    )

    ir = {
        "configuration": {
            "control_period_ns": control_period_ns,
            "backend": backend,
            # The authored execution platform, so provenance and the runtime graph read the
            # model's own answer instead of matching substrings of a derived id.
            "platform": platform,
            "agent_homes": deployment.agent_home_positions(
                platform, robots.serial_chains, platform_config
            ),
            "config_poses": config_poses,
            "trace": deployment.TRACE_DISABLED,
        },
        "resources": _resources_section(
            robots, world_trees, world_frames, sampling, ports, motions, views
        ),
        "composition": {"scene": scene, "sampling": sampling},
        "computation": _computation_section(
            functions,
            views,
            algorithm_data,
            data_access,
            motions,
            computation.indexes.pose_components,
        ),
        "coordination": _coordination_section(
            motions, fsm, fsm_meta, run_perturbations, perturbation_bodies
        ),
        "communication": _communication_section(
            telemetry,
            motions,
            deployment.ros_joint_states(platform, platform_config, robots.serial_chains),
            deployment.ros_clock(platform, platform_config),
            deployment.ros_tf(
                model,
                platform,
                platform_config,
                robots.serial_chains,
                scene.cameras,
                world_index,
                ports,
            ),
            action_clients,
            communication.action_server(model, fsm),
            subscriptions,
            standing_publishes,
        ),
    }
    # Last: every section above may mint a derived id, and each must be stated here.
    ir["provenance"] = _provenance_section(model, functions, algorithm_data, data_access, telemetry)
    return ir


def _resources_section(
    robots, world_trees, world_frames, sampling, ports, motions=(), views=None
) -> dict:
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
        drives = mobile_base.platform_drives(world_trees)
        by_kind["mobile_base"] = {
            "velocity_solvers": robots.platform_velocity,
            "force_solvers": robots.platform_force,
            # The platform's geometry as the scene tree states it, so the deployment config
            # carries none of it and the backend only adds joint names and indices.
            "drives": drives,
            "num_drives": len(drives),
            "wrench_by_motion": mobile_base.wrench_terms_by_motion(
                motions, views or {}, robots.platform_velocity, robots.platform_force
            ),
        }

    # Every device and sensor kind the model binds, as a membership map: templates emit code for
    # a kind only when the platform block or the scene names it.
    device_kinds = {device.kind for robot in robots.serial_chains for device in robot.devices} | {
        sensor.type for robot in robots.serial_chains for sensor in robot.sensors
    }

    # `by_id` is how a per-motion solver slice resolves everything the solver owns.
    section = {
        "robots": every,
        "by_kind": by_kind,
        # Whether the program drives anything at all, and whether a platform is the only thing it
        # drives. ST4 can only test one attribute, so the two questions the loop asks -- "is
        # there a scene to build" and "who steps it" -- are answered here rather than there.
        "driven": bool(by_kind) or None,
        "base_owns_runtime": ("mobile_base" in by_kind and not serial_chains) or None,
        "by_id": robots.by_id,
        "device_kinds": {kind: True for kind in device_kinds},
        # One entry per shared observation, not per solver that could answer it: several
        # solvers drive the same chain, and computing a value once per solver would repeat the
        # same reading -- ten times over, in a scene with ten motions.
        "world_observations": runtime.world_observations(robots.serial_chains),
    }
    if world_trees:
        # A drawn frame joins its tree after the draw, under the segment its pose is stated against.
        section["world_trees"] = [
            {
                "name": tree["name"],
                "cpp_name": tree["cpp_name"],
                "namespace": tree["namespace"],
                "header": tree["header"],
                "sampled_frames": [
                    {"name": s.segment, "parent": s.parent, "rotation": s.rotation, "draw": s.id}
                    for s in sampling
                    if s.tree == tree["cpp_name"]
                ],
            }
            for tree in world_trees
        ]
    if world_frames:
        section["world_frames"] = world_frames
    # Split by kind here, not in the templates: a template may iterate but not filter.
    section["world_ports"] = {kind: rows for kind, rows in ports.items() if rows}

    return section


def _provenance_section(model, functions, algorithm_data, data_access, telemetry) -> dict:
    """Where every published id comes from: its IRI, and for each derived value what it is and
    what it is computed from -- what its writer reads, or the value it is a view of.

    Raises:
        RuntimeError: a published id has no IRI -- a derived entity was minted without registering
            the node it came from.
    """
    uris = model.uri_rows()
    uri_by_id = {row["id"]: row["uri"] for row in uris if row.get("uri")}
    published = [(value_id, "data_access") for value_id in data_access]
    published += [(entry["write"].get("id"), "data_access.write") for entry in data_access.values()]
    for family in ("motions", "controllers", "monitors", "quantities", "quantity_samples"):
        published += [(row.get("source_id") or row.get("id"), family) for row in telemetry[family]]
    for pool, rows in telemetry["spatial_samples"].items():
        published += [(row["id"], f"spatial_samples.{pool}") for row in rows]
    unresolved: dict[str, set] = {}
    for value_id, origin in published:
        if isinstance(value_id, str) and value_id and value_id not in uri_by_id:
            unresolved.setdefault(value_id, set()).add(origin)
    if unresolved:
        details = "\n".join(
            f"  '{value_id}' (from {', '.join(origins)})"
            for value_id, origins in unresolved.items()
        )
        raise RuntimeError(
            "provenance: ids with no IRI -- a derived entity was minted without registering "
            f"the node it came from:\n{details}"
        )

    reads = operations.function_maps(functions).operands
    members = {item.id: item for item in algorithm_data}
    derivations = model.derivation_nodes()
    for row in derivations:
        member = members.get(row["id"])
        # Only a scalar names one kind and unit; a spatial record lists one per component.
        scalar = isinstance(
            member, (Quantity, FreeVector, SetpointQuantity, DataValue, Position, Orientation)
        )
        if scalar and member.quantity_kind is not None:
            row["kind"] = member.quantity_kind.iri
        if scalar and member.unit is not None:
            row["unit"] = member.unit.iri
        writer = (data_access.get(row["id"]) or {}).get("write") or {}
        written_from = [writer["id"]] if writer.get("kind") == "view" else reads.get(row["id"], ())
        sources = [uri_by_id[read] for read in written_from if read in uri_by_id]
        if sources:
            row["sources"] = sources

    return {"uris": uris, "derivations": derivations}


def _computation_section(
    functions, views, algorithm_data, data_access, motions, pose_components
) -> dict:
    """The algorithm: its F-blocks, the D-blocks they read and write, and the data access
    constraints that connect the two."""
    section = {
        "data": algorithm_data,
        "functions": functions,
        "data_access": data_access,
        "views": views_for_access(views, algorithm_data, motions, functions, pose_components),
    }
    # Elapsed constraints compare seconds from the runtime clock; naming the clock says what it
    # is, where a flag would only have asserted that one is wanted.
    if any(motion.has_elapsed for motion in motions):
        section["clock"] = {"id": "clock", "time_id": "clock_time_s"}

    return section


def _perturbations_by_body(perturbations) -> list[dict]:
    """The perturbations pushing each body, grouped.

    One body's applied-wrench slots hold the sum of everything pushing it. Written per
    perturbation instead, whichever ran last would be the only one the simulator ever saw --
    and a closed window writes zero, so an idle perturbation would erase a live one.
    """
    groups: dict[str, dict] = {}
    for perturbation in perturbations:
        group = groups.setdefault(
            perturbation.body,
            # Every solver on one simulation shares its model and data, so any member's
            # runtime resolves the body.
            {"body": perturbation.body, "robot_id": perturbation.robot_id, "members": []},
        )
        group["members"].append(perturbation)

    return list(groups.values())


def _coordination_section(motions, fsm, fsm_meta, perturbations=(), perturbation_bodies=()) -> dict:
    """What runs when: the motions, and the FSM sequencing them when the model imports one."""
    section = {"motions": motions}
    # Every perturbation in the run, not only the active motion's: the simulator holds an applied
    # wrench until something clears it, so the clearing pass has to reach the ones whose state
    # just exited, which by then selects no motion at all.
    if perturbations:
        section["perturbations"] = perturbations
        section["perturbation_bodies"] = perturbation_bodies
    if fsm:
        section["fsm"] = fsm_meta

    return section


def _communication_section(
    telemetry,
    motions,
    joint_states,
    clock=None,
    tf=None,
    action_clients=(),
    server=None,
    subscriptions=(),
    standing_publishes=(),
) -> dict:
    """What crosses the activity's boundary: the logging stream, and the ROS topics, goals and
    results the model asks for."""
    section = {"telemetry": telemetry}
    publishers = communication.ros_publishers(motions, standing_publishes)
    if (
        not publishers
        and joint_states is None
        and clock is None
        and tf is None
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
        packages.add(server["pkg"])
    if joint_states is not None:
        ros["joint_states"] = joint_states
        packages.add("sensor_msgs")
    if clock is not None:
        ros["clock"] = clock
        packages.add("rosgraph_msgs")
    if tf is not None:
        ros["tf"] = tf
        packages.update({"tf2_msgs", "geometry_msgs"})
    if action_clients:
        ros["action_clients"] = action_clients
        packages.update(client["pkg"] for client in action_clients)
    if subscriptions:
        ros["subscriptions"] = subscriptions
        packages.update(sub["pkg"] for sub in subscriptions)
    if standing_publishes:
        ros["standing"] = standing_publishes
        packages.update(publish["pkg"] for publish in standing_publishes)
    ros["packages"] = list(packages)
    section["ros"] = ros

    return section
