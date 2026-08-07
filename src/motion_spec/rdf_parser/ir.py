# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Assemble the code-generation IR for a motion specification model, and publish it.

The sections this orchestrates live beside it; the names re-exported below are the ones
other packages and the tests import from here."""

from __future__ import annotations

from rdf_utils.models.vocab import URI_GEOM_TYPE_POSE_COORD
from rdflib.namespace import RDF

from motion_spec.classes.entities import (
    GuardedMotionBlock, SceneRobot, SceneSpec,
)
# fmt: off
from motion_spec.rdf_parser.graph import (
    DerivedIriRegistry, Parser, SOLVER_SEMANTICS_BY_ALGORITHM, _load_graph, _node_indexes,
    ops_cstr_hdl, ops_generic, ops_slv,
)
# fmt: on
# fmt: off
from motion_spec.rdf_parser.controllers import (
    ANGULAR_AXES, ControllerDerivation, LINEAR_AXES, POSE_AXES, SolverDerivationContext,
    _annotate_controller_signals, _derive_solver_closures, _derive_solver_data,
    _derived_controllers, _derived_motion_drivers, _solver_derivation_context,
    add_controller_internal_state_logging, spatial_axes,
)
# fmt: on
# fmt: off
from motion_spec.rdf_parser.solvers import (
    _TRACE_DISABLED, _agent_home_positions, _annotate_rne_gravity, _annotate_runtime_robots,
    _fixed_attachments, _kinematic_adjacency, _mapped_targets, _orientation_of,
    _platform_from_graph, _position_of, _reject_scene_objects_on_hardware,
    _robot_setups_from_graph, _scene_from_graph, _shared_runtime_members, _solver_sections,
    _tree_owns, _validate_scene, _validate_solvers,
)
# fmt: on
# fmt: off
from motion_spec.rdf_parser.computation import (
    _closure_maps, _closure_owner_map, _data_reference_map, _filter_shared_data,
    _materialize_linear_distance_operations, _materialize_pose_reference_transforms,
    _snapshot_maps, _views_for_access, build_pose_components, resolve_arc_closures,
    resolve_lerp_closures,
)
# fmt: on
from motion_spec.rdf_parser.dataflow import annotate_dataflow
from motion_spec.rdf_parser.coordination import (
    _apply_monitor_debounce, _assign_monitor_event_indexes, _evaluator_term, _fsm_from_graph,
    build_motion_units,
)
from motion_spec.rdf_parser.introspection import (
    _assert_every_id_resolves, _build_introspection, add_quantity_samples, add_spatial_samples,
)

# The lowering's public surface. Everything else is a section's own business.
__all__ = [
    "ANGULAR_AXES", "ControllerDerivation", "DerivedIriRegistry", "GuardedMotionBlock",
    "LINEAR_AXES", "POSE_AXES", "Parser", "SOLVER_SEMANTICS_BY_ALGORITHM", "SceneRobot",
    "SceneSpec", "SolverDerivationContext", "_annotate_controller_signals",
    "_annotate_rne_gravity", "_assert_every_id_resolves", "_build_introspection",
    "_derived_controllers", "_derived_motion_drivers", "_evaluator_term", "_fixed_attachments",
    "_kinematic_adjacency", "_load_graph", "_mapped_targets",
    "_materialize_linear_distance_operations", "_materialize_pose_reference_transforms",
    "_orientation_of", "_position_of", "_reject_scene_objects_on_hardware",
    "_robot_setups_from_graph", "_solver_derivation_context", "_solver_sections", "_tree_owns",
    "_views_for_access", "add_controller_internal_state_logging", "add_quantity_samples",
    "add_spatial_samples", "annotate_dataflow", "generate_ir", "ops_generic", "spatial_axes",
]


def generate_ir(manifest_path):
    """Build the complete IR for a model manifest in one forward pass and return it as a dict."""
    app_model_path, g, imported_models, imported_provenance = _load_graph(manifest_path)
    from motion_spec.generation.scene_kdl import kdl_header_name

    kdl_header = kdl_header_name(app_model_path)
    _materialize_pose_reference_transforms(g)
    _materialize_linear_distance_operations(g)

    p = Parser(g)
    node_by_id, id_nodes = _node_indexes(g, p)
    iris = DerivedIriRegistry(id_nodes)
    setups_by_node, ordered_setups = _robot_setups_from_graph(g)
    default_setup = (
        ordered_setups[0]
        if ordered_setups
        else ("", "", "", "", "", "", "", [], [], "", [], "", "", [], "")
    )
    # Backend and FSM are pure functions of the graph and feed downstream construction.
    platform = _platform_from_graph(g)
    backend = platform["backend"]
    fsm = _fsm_from_graph(g)
    scene = _scene_from_graph(g)
    _validate_scene(scene)
    derivation = _solver_derivation_context(g, iris)

    (slv_platform_vel, sched1, hdl, sched2, slv_chain, sched3, slv_platform_frc, sched4) = _solver_sections(
        g, p, setups_by_node, default_setup, derivation, scene.objects
    )
    for solver in slv_chain:
        solver.kdl_header = kdl_header
    _assign_monitor_event_indexes(hdl)

    closures = p.closures(ops_generic + ops_slv + ops_cstr_hdl)
    _derive_solver_closures(g, p, derivation, closures)
    view_map = p.view()
    data_structures = p.data_structures()
    _derive_solver_data(g, p, derivation, data_structures, view_map)
    snapshot_source_map, snapshot_owner_map, snapshot_trigger_map = _snapshot_maps(g, p)
    closure_owner_map = _closure_owner_map(g, p, closures)
    data_reference_map = _data_reference_map(data_structures, closures)
    closure_output_map, closure_input_map = _closure_maps(closures)

    # Before motions are built: their declared poses reference these.
    pose_nodes = {
        p.id(node): node for node in g.subjects(RDF.type, URI_GEOM_TYPE_POSE_COORD)
    }
    pose_components = build_pose_components(view_map, data_structures, g, pose_nodes)
    resolve_lerp_closures(closures, pose_components)
    resolve_arc_closures(closures, data_structures)

    motions, fsm_meta = build_motion_units(
        g,
        p,
        hdl,
        node_by_id,
        slv_chain,
        snapshot_source_map=snapshot_source_map,
        view_map=view_map,
        closure_output_map=closure_output_map,
        data_reference_map=data_reference_map,
        closure_input_map=closure_input_map,
        closures=closures,
        data_structures=data_structures,
        pose_components=pose_components,
        fsm=fsm,
        derivation=derivation,
        snapshot_trigger_map=snapshot_trigger_map,
        snapshot_owner_map=snapshot_owner_map,
        closure_owner_map=closure_owner_map,
    )
    _validate_solvers(slv_chain, backend)
    _annotate_runtime_robots(slv_chain, motions, backend)
    _annotate_rne_gravity(slv_chain, motions)

    if scene.timestep_s <= 0:
        raise ValueError("ENVIRONMENT timestep must be positive.")
    control_period_ns = int(round(scene.timestep_s * 1e9))
    _apply_monitor_debounce(hdl, control_period_ns)

    # Safeguard: no two distinct URIs may collapse to one generated id (would silently merge).
    p.assert_no_id_collisions()

    shared_data = _filter_shared_data(
        data_structures,
        sched1 + sched2 + sched3 + sched4,
        closures,
        view_map=view_map,
        fk_output_ids={out.id for s in slv_chain for out in s.output},
    )
    shared_data = shared_data + _shared_runtime_members(
        slv_chain, iris, control_period_ns, platform.get("uri")
    )

    introspection, values = _build_introspection(
        app_model_path=app_model_path,
        imported_models=imported_models,
        imported_provenance=imported_provenance,
        id_nodes=id_nodes,
        node_by_id=node_by_id,
        motions=motions,
        data_structures=data_structures,
        control_period_ns=control_period_ns,
        backend=backend,
        scene=scene,
        closures=closures,
        views=view_map,
        shared_data=shared_data,
        serial_chain_solvers=slv_chain,
        platform=platform,
        iris=iris,
    )

    schedule = sched1 + sched2 + sched3 + sched4
    shared_schedule = sched1 + sched3 + sched4

    # Codegen links rclcpp/realtime_tools and sets up publishers only when publishing is present.
    _by_pub_id = {}
    for motion in motions:
        for phase in ("when_monitors", "until_monitors", "while_monitors"):
            for mon in getattr(motion, phase, []) or []:
                channel = getattr(mon, "ros_channel", None)
                if channel is None:
                    continue
                _by_pub_id.setdefault(
                    mon.ros_pub_id,
                    {
                        "pub_id": mon.ros_pub_id,
                        "channel": channel,
                        "cpp_type": mon.ros_cpp_type,
                        "include": mon.ros_include,
                        "pkg": mon.ros_pkg,
                    },
                )
    ros_publishers = list(_by_pub_id.values())
    ros_packages = sorted({p["pkg"] for p in ros_publishers})

    # Arm and wheeled base are both actuated resources, so they ride in one kind-tagged
    # collection. The by-kind views are filtered off `robots` here because ST4 cannot filter.
    robots = [*slv_chain, *slv_platform_vel, *slv_platform_frc]
    resources = {
        "robots": robots,
        "by_kind": {
            "serial_chain": [r for r in robots if r.kind == "serial_chain"],
            # ST4 treats an empty list as truthy, so the base rides as object-or-absent.
            "mobile_base": (
                {"velocity_solvers": slv_platform_vel, "force_solvers": slv_platform_frc}
                if any(r.kind == "mobile_base" for r in robots)
                else None
            ),
        },
    }

    ir = {
        # Read cross-package by motion-spec-dsl's solver-derivation tests.
        "cstr_hdl": hdl,
        "motions": motions,
        "closures": closures,
        "views": _views_for_access(view_map, shared_data, motions, closures),
        "shared_data": shared_data,
        # Layer-B projections of the dataflow contract, keyed by model role, not C++ construct.
        "values": values,
        "has_serial_chain": bool(slv_chain),
        # Every actuated resource the program commands, plus the by-kind views built above.
        "resources": resources,
        # One key per optional subsystem, absent when the model has none: ST4 treats an empty
        # list as truthy, so absence is the guard and no has_* flag has to track it.
        "ros": (
            {
                "publishers": ros_publishers,
                "packages": ros_packages,
                "node_name": "motion_spec_monitor",
            }
            if ros_publishers
            else None
        ),
        # Elapsed constraints compare seconds from the runtime clock (MuJoCo sim seconds /
        # real monotonic wall clock).
        "needs_clock_time": any(m.has_elapsed for m in motions),
        "control_period_ns": control_period_ns,
        "backend": backend,
        # The authored execution platform, so provenance and the runtime graph read the model's
        # own answer instead of matching substrings of a derived id.
        "platform": platform,
        "agent_homes": _agent_home_positions(platform, slv_chain),
        "scene": scene,
        "trace": _TRACE_DISABLED,
        "introspection": introspection,
        # Framed from the FSM named graph plus its derived codegen wiring; None without a .fsm.
        "fsm": {**fsm, **fsm_meta} if fsm else None,
    }
    # Complete by construction: codegen loads ir.json and renders, with no derivation pass.
    return ir
