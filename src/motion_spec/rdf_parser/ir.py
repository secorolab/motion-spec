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

ALL_OPERATORS = operations.OPS_GENERIC + operations.OPS_SOLVER + operations.OPS_HANDLER


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

    platform = deployment.read_platform(model)
    platform_config = deployment.platform_config(platform)
    backend = platform["backend"]
    setups, world_trees = agents.robot_setups(model)
    scene = read_scene(model, world_trees)
    derivation = constraint_handler.solver_derivation_context(model)
    sampling = sampled_quantities(
        model, world_trees, {obj.draw_uri for obj in scene.objects if obj.draw_uri}
    )

    schedule = operations.Schedule(model)
    # a perceived pose has one writer; the chains must not write it
    perceived_pose_ids = {
        row["pose_id"]
        for rows in quantities.perceived_written_poses(model).values()
        for row in rows
    }
    robots = agents.build_robots(model, schedule, setups, derivation, backend, perceived_pose_ids)
    world_index = agents.tree_segments(model, setups, world_trees)
    handlers, handler_steps = coordination.build_constraint_handlers(model, schedule, derivation)
    coordination.assign_event_indexes(handlers)

    functions = operations.build_functions(model, ALL_OPERATORS)
    constraint_handler.augment_functions(model, derivation, functions)
    views = read_views(model)
    data_structures = read_data_structures(model)
    constraint_handler.add_pose_errors_and_control_signals(
        model, derivation, data_structures, views
    )
    computation = build_indexes(model, functions, data_structures, views)
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
    runtime_members, tare_companions = runtime.runtime_data(
        model, robots.serial_chains, control_period_ns, platform.get("uri")
    )
    algorithm_data += runtime_members
    coordination.add_snapshot_latches(model, motions, algorithm_data)

    observed_at_ids = {
        row["observed_at_id"]
        for channel in (*action_clients, *subscriptions)
        for row in channel["written_poses"]
        if row.get("observed_at_id")
    }
    for observed_at_id in observed_at_ids:
        algorithm_data.append(DataValue(id=observed_at_id, type="Quantity", unset=True))

    run_perturbations = [p for motion in motions for p in motion.perturbations]
    for perturbation in run_perturbations:
        node = model.node_by_id[perturbation.id]
        model.register_derived(perturbation.applied_id, node, "applied", PROV.wasDerivedFrom)
        model.register_derived(perturbation.active_id, node, "active", PROV.wasDerivedFrom)
        algorithm_data.append(DataValue(id=perturbation.applied_id, type="Wrench"))
        algorithm_data.append(DataValue(id=perturbation.active_id, type="Bool", value=False))
    constraint_handler.add_controller_state(model, functions, algorithm_data, motions)
    constraint_handler.add_control_parameters(model, functions, algorithm_data, motions)
    runtime.add_joint_space_mirrors(model, robots, motions, algorithm_data, backend)
    communication.add_goal_status_values(model, algorithm_data, action_clients)
    # summed per body: a closed window writes zero and would erase a live one
    perturbation_groups: dict[str, dict] = {}
    for perturbation in run_perturbations:
        group = perturbation_groups.setdefault(
            perturbation.body,
            {"body": perturbation.body, "robot_id": perturbation.robot_id, "members": []},
        )
        group["members"].append(perturbation)
    perturbation_bodies = list(perturbation_groups.values())
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
    # F/D-block access (Bruyninckx 2026, §2.3.4); analysis precedes codegen (Aho et al., §9.2)
    data_access = build_data_access(
        algorithm_data,
        functions,
        motions,
        robots.serial_chains,
        tare_companions,
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
            "platform": platform,
            "agent_homes": deployment.agent_home_positions(
                platform, robots.serial_chains, platform_config
            ),
            "config_poses": config_poses,
            "trace": deployment.TRACE_DISABLED,
        },
    }

    ir["resources"] = resources(robots, motions, views, world_trees, world_frames, sampling, ports)
    ir["composition"] = {"scene": scene, "sampling": sampling}

    ir["computation"] = {
        "data": algorithm_data,
        "functions": functions,
        "data_access": data_access,
        "views": views_for_access(
            views, algorithm_data, motions, functions, computation.indexes.pose_components
        ),
    }
    if any(motion.has_elapsed for motion in motions):
        ir["computation"]["clock"] = {"id": "clock", "time_id": "clock_time_s"}

    ir["coordination"] = {"motions": motions}
    # all of them: clearing must reach states that just exited
    if run_perturbations:
        ir["coordination"]["perturbations"] = run_perturbations
        ir["coordination"]["perturbation_bodies"] = perturbation_bodies
    if fsm:
        ir["coordination"]["fsm"] = fsm_meta

    joint_states = deployment.ros_joint_states(platform, platform_config, robots.serial_chains)
    clock = deployment.ros_clock(platform, platform_config)
    tf = deployment.ros_tf(
        model, platform, platform_config, robots.serial_chains, scene.cameras, world_index, ports
    )
    server = communication.action_server(model, fsm)
    ir["communication"] = {"telemetry": telemetry}
    publishers = communication.ros_publishers(motions, standing_publishes)
    ros = communication.ros_interface(
        publishers,
        server,
        joint_states,
        clock,
        tf,
        action_clients,
        subscriptions,
        standing_publishes,
    )
    if ros is not None:
        ir["communication"]["ros"] = ros

    ir["provenance"] = provenance(model, functions, algorithm_data, data_access, telemetry)

    return ir


def resources(robots, motions, views, world_trees, world_frames, sampling, ports) -> dict:
    """The resources section: the robots the program commands, split by kind, the devices and
    sensors they carry, and the world trees, frames and ports the runtime builds.
    """
    # split by kind here, ST4 cannot filter; absent rather than empty
    robots_by_kind = {}
    if robots.serial_chains:
        robots_by_kind["serial_chain"] = robots.serial_chains
    if robots.platform_velocity or robots.platform_force:
        drives = mobile_base.platform_drives(world_trees)
        robots_by_kind["mobile_base"] = {
            "velocity_solvers": robots.platform_velocity,
            "force_solvers": robots.platform_force,
            "drives": drives,
            "num_drives": len(drives),
            "wrench_by_motion": mobile_base.wrench_terms_by_motion(
                motions, views, robots.platform_velocity, robots.platform_force
            ),
        }

    device_kinds = {device.kind for robot in robots.serial_chains for device in robot.devices} | {
        sensor.type for robot in robots.serial_chains for sensor in robot.sensors
    }

    section = {
        "robots": [*robots.serial_chains, *robots.platform_velocity, *robots.platform_force],
        "by_kind": robots_by_kind,
        "driven": bool(robots_by_kind) or None,
        "base_owns_runtime": (
            "mobile_base" in robots_by_kind and not robots.serial_chains
        ) or None,
        "by_id": robots.by_id,
        "device_kinds": {kind: True for kind in device_kinds},
        "world_observations": runtime.world_observations(robots.serial_chains),
    }
    if world_trees:
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
    # ST4 cannot filter
    section["world_ports"] = {kind: rows for kind, rows in ports.items() if rows}

    return section


def provenance(model: Model, functions, algorithm_data, data_access: dict, telemetry: dict) -> dict:
    """The provenance section: every IRI the IR names, and each derived entity with its kind, unit
    and the entities it is computed from. Built last, since every section may mint a derived id.

    Raises:
        RuntimeError: a published id has no IRI, so a derived entity was minted without
            registering the node it came from.
    """
    uris = model.uri_rows()
    uri_by_id = {row["id"]: row["uri"] for row in uris if row.get("uri")}
    published = {
        "data_access": data_access,
        "data_access.write": [entry["write"].get("id") for entry in data_access.values()],
        **{
            family: [row.get("source_id") or row.get("id") for row in telemetry[family]]
            for family in ("motions", "controllers", "monitors", "quantities", "quantity_samples")
        },
        **{
            f"spatial_samples.{pool}": [row["id"] for row in rows]
            for pool, rows in telemetry["spatial_samples"].items()
        },
    }
    unresolved: dict[str, set] = {}
    for origin, value_ids in published.items():
        for value_id in value_ids:
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
