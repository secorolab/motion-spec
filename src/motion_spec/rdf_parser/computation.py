# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Non-controller computation: views, poses, snapshots, derived pose operations and the
closure maps the schedules are built over."""

from __future__ import annotations

import collections
from rdf_utils.naming import get_valid_var_name
from rdf_utils.models.geom_coord import (
    PoseCoordModel,
    get_coord_vectorxyz,
    get_orientation_coord_vals,
)
from rdf_utils.models.geom_rel import PoseModel
from rdf_utils.models.common import get_node_types

# fmt: off
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_OF, URI_GEOM_PRED_OF_ORIENT, URI_GEOM_PRED_OF_POSE, URI_GEOM_PRED_OF_POSITION,
    URI_GEOM_PRED_ORIGIN, URI_GEOM_PRED_SEEN_BY, URI_GEOM_PRED_WRT, URI_GEOM_TYPE_POSE_COORD,
    URI_GEOM_TYPE_POSE, URI_GEOM_TYPE_POSITION, URI_GEOM_TYPE_POSITION_COORD,
    URI_GEOM_TYPE_ORIENT, URI_GEOM_TYPE_ORIENT_COORD, URI_GEOM_TYPE_ORIENT_REF,
    URI_GEOM_TYPE_FRAME, URI_GEOM_TYPE_POINT, URI_GEOM_TYPE_POSE_REF, URI_GEOM_TYPE_POSITION_REF,
    URI_GEOM_TYPE_VECTOR_XYZ, URI_QUDT_QK_LENGTH, URI_QUDT_UNIT_M, URI_QUDT_UNIT_RAD,
)
# fmt: on
from rdflib import URIRef
from rdflib.namespace import RDF

# fmt: off
from motion_spec.classes.entities import (
    EqualityConstraint, PoseAxisErrorComponent, PoseAxisErrorGroup, RelativePoseCapture,
    SceneRelativePose, SnapshotCapture, Subspace,
)
# fmt: on
from motion_spec.classes.closures import closure_output_ids

# fmt: off
from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT, CSTR, CSTR_EXT, CSTR_HDL, GEOM_OP, GEOM_REL, QUDT_SCHEMA,
)
# fmt: on

from motion_spec.rdf_parser.records import (
    _as_dict,
    _dedupe_by_id,
    _field,
    _index_by_id,
    _pose_component,
)
from motion_spec.rdf_parser.graph import Parser, _is_constraint_aggregate, _length_unit, _si_all
from motion_spec.rdf_parser.solvers import _leaf


def _views_by_subobject(view_map):
    indexed: dict[str, list] = {}
    for view in view_map.values():
        subobject_id = _field(_field(view, "subobject"), "id")
        if subobject_id:
            indexed.setdefault(subobject_id, []).append(view)
    return indexed


def _views_for_access(view_map, shared_data, motions, closures) -> dict:
    """Index unambiguous MAP views by subobject, for the template's access expressions.

    A subobject written directly -- an authored or literal shared value, a snapshot target, a
    closure output -- is an ordinary shared quantity and keeps its own field; one reused by views
    that disagree on how they access it must not silently pick one of them.
    """
    direct_ids = {
        _field(item, "id")
        for item in shared_data
        if _field(item, "id")
        and (
            _field(item, "value") is not None
            or _field(_field(item, "provenance"), "authored", False)
        )
    }
    direct_ids.update(
        _field(snapshot, "target_id")
        for motion in motions
        for snapshot in _field(motion, "snapshots", []) or []
    )
    direct_ids.update(
        output_id for closure in closures.values() for output_id in closure_output_ids(closure)
    )

    indexed: dict[str, object] = {}
    for view in view_map.values():
        subobject_id = _field(_field(view, "subobject"), "id")
        if not subobject_id or subobject_id in direct_ids:
            continue
        if subobject_id not in indexed:
            indexed[subobject_id] = view
            continue
        previous = indexed[subobject_id]
        if previous is None:
            continue
        if any(
            _field(previous, field) != _field(view, field)
            for field in ("superobject", "subspace", "axis", "direction")
        ):
            indexed[subobject_id] = None
    return {id_: view for id_, view in indexed.items() if view is not None}


def _unique_view_for_subobject(indexed_views, subobject_id, context):
    matches = indexed_views.get(subobject_id, ())
    if len(matches) > 1:
        raise ValueError(
            f"{context}: quantity '{subobject_id}' is the subobject of multiple MAP views"
        )
    return matches[0] if matches else None


def _relative_poses_for_motion(evaluators, view_map, serial_chain_solvers):
    """Detect Pose quantities whose wrt frame ends in _start and pair them with FK outputs."""
    fk_poses: dict[str, str] = {}  # of_id → FK pose id
    for solver in serial_chain_solvers:
        for out in solver.output:
            if getattr(out, "type", "") == "Pose":
                of_id = getattr(getattr(out, "of", None), "id", None)
                if of_id:
                    fk_poses[of_id] = out.id

    start_rel_poses: dict[str, object] = {}
    indexed_views = _views_by_subobject(view_map)
    for ev in evaluators:
        qty = getattr(getattr(ev, "constraint", None), "quantity", None)
        if qty is None or not getattr(qty, "has_view", False):
            continue
        view = _unique_view_for_subobject(indexed_views, qty.id, "relative pose lookup")
        if view is None:
            continue
        wrt = getattr(getattr(view, "superobject", None), "with_respect_to", None)
        if wrt and getattr(wrt, "id", "").endswith("_start"):
            pose_id = view.superobject.id
            start_rel_poses[pose_id] = view.superobject

    result = []
    for pose_id, pose in start_rel_poses.items():
        of_id = getattr(getattr(pose, "of", None), "id", None)
        fk_pose_id = fk_poses.get(of_id) if of_id else None
        if fk_pose_id:
            result.append(RelativePoseCapture(id=pose_id, fk_pose_id=fk_pose_id))
    return result


def _constraint_reference_value_id(constraint):
    """Id of a constraint's reference-value parameter, or None."""
    param = getattr(constraint, "parameter", None)
    ref = getattr(param, "reference_value", None) if param else None
    return getattr(ref, "id", None)


def _snapshot_reference_value_ids(evaluators, constraints):
    """Set of reference-value ids across the given evaluators and constraints."""
    ref_val_ids = set()
    for ev in evaluators:
        if ev.constraint is None:
            continue
        ref_id = _constraint_reference_value_id(ev.constraint)
        if ref_id:
            ref_val_ids.add(ref_id)
    for constraint in constraints:
        ref_id = _constraint_reference_value_id(constraint)
        if ref_id:
            ref_val_ids.add(ref_id)
    return ref_val_ids


def _snapshots_for_motion(
    evaluators,
    constraints,
    snapshot_source_map,
    view_map,
    closure_output_map,
    data_reference_map=None,
    schedule=None,
    closures=None,
    snapshot_trigger_map=None,
    motion_token=None,
    snapshot_owner_map=None,
    motion_tokens=(),
):
    """Build a motion's initial snapshot captures from its references."""
    snapshot_trigger_map = snapshot_trigger_map or {}
    snapshot_owner_map = snapshot_owner_map or {}
    ref_val_ids = _snapshot_reference_value_ids(evaluators, constraints)
    closures = closures or {}
    data_reference_map = data_reference_map or {}

    subobjects_by_super: dict[str, list[str]] = {}
    supers_by_subobject: dict[str, list[str]] = {}
    for view in view_map.values():
        super_id = _field(_field(view, "superobject"), "id")
        subobject_id = _field(_field(view, "subobject"), "id")
        if super_id and subobject_id:
            subobjects_by_super.setdefault(super_id, []).append(subobject_id)
            supers_by_subobject.setdefault(subobject_id, []).append(super_id)

    def add_reference(ref_id):
        """Record a snapshot capture for one referenced id."""
        if not isinstance(ref_id, str):
            return
        pending = [ref_id]
        expanded = set()
        while pending:
            current = pending.pop()
            if current in expanded:
                continue
            expanded.add(current)
            ref_val_ids.add(current)
            referenced = data_reference_map.get(current)
            if referenced:
                pending.append(referenced)
            # superobject -> subobject (forward decomposition), and subobject ->
            # superobject (a composite pose's snapshot is only referenced through its
            # scalar components, so climb back to capture the composite).
            pending.extend(subobjects_by_super.get(current, ()))
            pending.extend(supers_by_subobject.get(current, ()))

    for ref_id in list(ref_val_ids):
        add_reference(ref_id)
    for step in schedule or []:
        closure = closures.get(step) or {}
        for value in closure.values():
            if isinstance(value, str):
                add_reference(value)
            elif isinstance(value, list):
                for item in value:
                    add_reference(item)
    result = []
    seen = set()
    for target_id in sorted(ref_val_ids):
        if target_id not in snapshot_source_map or target_id in seen:
            continue
        # Capture only what this motion declares: re-capturing another motion's snapshot would
        # overwrite its value. Unowned (shared-context) snapshots stay everyone's to capture.
        owner = snapshot_owner_map.get(target_id)
        if owner in motion_tokens and owner != motion_token:
            continue
        seen.add(target_id)
        source_id = snapshot_source_map[target_id]
        source_closure_id = (
            None if source_id in supers_by_subobject else closure_output_map.get(source_id)
        )
        result.append(
            SnapshotCapture(
                target_id=target_id,
                source_id=source_id,
                source_closure_id=source_closure_id,
                trigger_event=snapshot_trigger_map.get((motion_token, target_id)),
            )
        )
    return result


def _scene_relative_poses_for_motion(view_map, serial_chain_solvers, evaluators=None):
    """For each view whose wrt-frame is a scene object, emit the requested relative pose."""
    fk_pose_by_frame: dict[str, str] = {}
    # Keyed by the scene-object's id; a wrt_id lookup asks whether that frame is the subject
    # of a tracked scene-object pose.
    scene_pose_by_id: dict[str, tuple[str, str | None]] = {}
    solver_output_ids: set[str] = set()
    for solver in serial_chain_solvers:
        for out in solver.output:
            if getattr(out, "type", "") != "Pose":
                continue
            solver_output_ids.add(out.id)
            of = getattr(out, "of", None)
            if of is None:
                chain_end = getattr(solver, "chain_end", None)
                if chain_end:
                    fk_pose_by_frame.setdefault(chain_end, out.id)
                continue
            if getattr(of, "is_scene_object", False):
                scene_pose_by_id[of.id] = (
                    out.id,
                    getattr(getattr(out, "with_respect_to", None), "id", None),
                )
                body = getattr(of, "body", None)
                if body:
                    scene_pose_by_id[body] = scene_pose_by_id[of.id]
            else:
                fk_pose_by_frame[of.id] = out.id

    for view in view_map.values():
        so = getattr(view, "superobject", None)
        if so is None or not hasattr(so, "of") or not hasattr(so, "with_respect_to"):
            continue
        if getattr(so, "id", None) not in solver_output_ids:
            continue
        of = getattr(so, "of", None)
        if of is None or getattr(of, "is_scene_object", False):
            continue
        fk_pose_by_frame.setdefault(getattr(of, "id", ""), so.id)

    candidate_poses = [
        getattr(view, "superobject", None)
        for view in view_map.values()
        if not getattr(getattr(view, "superobject", None), "authored", False)
    ]
    for evaluator in evaluators or []:
        constraint = getattr(evaluator, "constraint", None)
        quantity = getattr(constraint, "quantity", None)
        if (
            quantity is not None
            and hasattr(quantity, "of")
            and hasattr(quantity, "with_respect_to")
        ):
            candidate_poses.append(quantity)

    seen: set[str] = set()
    result: list[SceneRelativePose] = []
    for so in candidate_poses:
        if so is None:
            continue
        pose_id = getattr(so, "id", None)
        if not pose_id or pose_id in seen:
            continue
        of = getattr(so, "of", None)
        wrt = getattr(so, "with_respect_to", None)
        if of is None or wrt is None:
            continue
        of_id = getattr(of, "id", "")
        wrt_id = getattr(wrt, "id", "")
        if getattr(of, "is_scene_object", False) or of_id in scene_pose_by_id:
            continue
        scene_pose = scene_pose_by_id.get(wrt_id)
        if scene_pose is None and not getattr(wrt, "is_scene_object", False):
            continue
        fk_pose_id = fk_pose_by_frame.get(of_id)
        if fk_pose_id and scene_pose:
            scene_pose_id, scene_wrt_id = scene_pose
            as_seen_by_id = getattr(getattr(so, "as_seen_by", None), "id", None)
            seen.add(pose_id)
            result.append(
                SceneRelativePose(
                    id=pose_id,
                    fk_pose_id=fk_pose_id,
                    scene_pose_id=scene_pose_id,
                    base_seen=as_seen_by_id == scene_wrt_id,
                )
            )
    return result


_GROUPABLE_SO_TYPES = {"Pose", "VelocityTwist", "AccelerationTwist", "Wrench"}
_SUBSPACE_TO_GROUP_AXIS: dict[Subspace, tuple[str, bool]] = {
    Subspace.Linear: ("linear", False),
    Subspace.Angular: ("angular", True),
}


def _pose_axis_error_groups_for_motion(eval_nodes, p, view_map):
    """Group a motion's per-axis pose error evaluators into PoseAxisErrorGroups, one per superobject pose."""
    groups: dict[str, PoseAxisErrorGroup] = {}
    indexed_views = _views_by_subobject(view_map)
    for eval_node in eval_nodes:
        if CSTR_HDL["ErrorEvaluator"] not in get_node_types(p.g, eval_node):
            continue

        evaluator = p.constraint_evaluator(eval_node)
        if not isinstance(evaluator.constraint.parameter, EqualityConstraint):
            continue
        if evaluator.error is None:
            continue

        quantity = evaluator.constraint.quantity
        view = _unique_view_for_subobject(indexed_views, quantity.id, "pose-axis error grouping")
        if view is None:
            continue
        so_type = getattr(view.superobject, "type", None)
        if so_type not in _GROUPABLE_SO_TYPES:
            continue

        mapping = _SUBSPACE_TO_GROUP_AXIS.get(view.subspace)
        if mapping is None:
            qkind = getattr(getattr(quantity, "quantity_kind", None), "id", "")
            quantity_id = getattr(quantity, "id", "")
            if so_type == "Pose" and (
                "Angle" in qkind or "angle" in qkind.lower() or "rotation" in quantity_id
            ):
                mapping = ("angular", True)
            else:
                continue
        subspace, is_angular = mapping

        # A whole-subspace view has no per-axis component, so it cannot join a per-axis group;
        # its own equality-constraint controller drives it. Skip rather than deref a None axis.
        if view.axis is None:
            continue

        so_id = view.superobject.id
        group = groups.setdefault(
            so_id,
            PoseAxisErrorGroup(
                id=f"pose_axis_error_{so_id}", pose=so_id, components=[], superobject_type=so_type
            ),
        )
        if is_angular:
            group.has_angular = True
        group.components.append(
            PoseAxisErrorComponent(
                quantity=evaluator.constraint.quantity.id,
                error=evaluator.error.id,
                reference=evaluator.constraint.parameter.reference_value.id,
                subspace=subspace,
                axis=view.axis.value,
                eval_id=evaluator.id,
            )
        )
        setattr(
            group,
            f"{subspace}_{view.axis.value.lower()}",
            evaluator.constraint.parameter.reference_value.id,
        )

    return [group for group in groups.values() if len(group.components) > 1]


def _path_projections_for_motion(schedule: list, closures: dict) -> list[dict]:
    """The path projections a motion runs, with the measurements that re-arm on entry."""
    speeds = [
        closures[call]["along_speed"]
        for call in dict.fromkeys(schedule)
        if isinstance(closures.get(call), dict)
        and closures[call].get("type") == "TwistToLinearVelocityAlong"
    ]
    return [
        {"id": call, "parameter": closures[call]["path_parameter"], "along_speed": speed}
        for (call, speed) in zip(
            (
                call
                for call in dict.fromkeys(schedule)
                if isinstance(closures.get(call), dict)
                and closures[call].get("type") == "PathProjection"
            ),
            speeds,
        )
    ]


def _filter_shared_data(data_structures, schedule, closures, view_map=None, fk_output_ids=None):
    """Select data structures needed by scheduled calls, views, closures, or FK outputs."""
    referenced: set[str] = set(schedule)
    for c in closures.values():
        if isinstance(c, dict):
            for v in c.values():
                # An n-ary port (algo-ext:in) arrives as a list of operand ids.
                for item in v if isinstance(v, list) else [v]:
                    if isinstance(item, str):
                        referenced.add(item)
    if view_map:
        for view in view_map.values():
            for endpoint in (view.superobject, view.subobject):
                if endpoint:
                    referenced.add(endpoint.id)
    if fk_output_ids:
        referenced.update(fk_output_ids)

    result = []
    for item in data_structures:
        if item.type == "Quantity" and item.has_view and item.id not in referenced:
            continue
        if (
            item.type == "Quantity"
            and getattr(item, "value", None) is None
            and getattr(getattr(item, "quantity_kind", None), "id", "x") is None
            and item.id not in referenced
        ):
            continue
        if item.type in ("Pose", "VelocityTwist") and item.id not in referenced:
            continue
        result.append(item)
    return _dedupe_by_id(result)


def _expanded_constraints(g, raw_nodes) -> set:
    """The phase's constraints, with any when/until aggregate replaced by its members."""
    return {
        member
        for node in raw_nodes
        for member in (
            g[node : CSTR_EXT["has-constraint"]] if _is_constraint_aggregate(g, node) else (node,)
        )
    }


def _snapshot_maps(g, p: Parser) -> tuple[dict, dict, dict]:
    """One walk over the snapshots for the three maps codegen asks about.

    ``source``: each snapshot output to its source quantity. ``owner``: each output to the motion
    declaring it, taken from the motion segment of the quantity URI (<app>/<motion>/spec/<name>) --
    every motion captures each snapshot it references and they share one slot, so without an owner
    a motion silently retargets another's. ``trigger``: each event-triggered snapshot to its trigger
    event's local name, keyed by (declaring motion, output id) since only the owner re-samples.
    """
    source_map: dict[str, str] = {}
    owner_map: dict[str, str] = {}
    trigger_map: dict[tuple[str, str], str] = {}
    for snap_node in g.subjects(RDF.type, ALGO_EXT.Snapshot):
        output_node = g.value(snap_node, ALGO_EXT.out)
        if output_node is None:
            continue
        output_id = p.id(output_node)
        source_node = g.value(snap_node, ALGO_EXT["in"])
        if source_node is not None:
            source_map[output_id] = p.id(source_node)
        scope = p._context_scope(output_node)
        if scope is None:
            continue
        owner = get_valid_var_name(scope[0])
        owner_map[output_id] = owner
        trigger_node = g.value(snap_node, ALGO_EXT["trigger"])
        if trigger_node is not None:
            trigger_map[(owner, output_id)] = get_valid_var_name(_leaf(trigger_node)).upper()
    return source_map, owner_map, trigger_map


def _closure_owner_map(g, p: Parser, closures) -> dict[str, str]:
    """Map each closure to the motion whose context declares the quantities it reads.

    The backward schedule walk can reach another motion's closure, which would then run whenever
    that unrelated motion is active. Ownership is the motion segment of the quantities it
    references; a closure reading only shared context has no owner and stays available to all.
    """
    owner_map: dict[str, str] = {}
    for node in set(g.subjects()):
        if not isinstance(node, URIRef):
            continue
        closure_id = p.id(node)
        if closure_id not in closures:
            continue
        owners = set()
        for obj in g.objects(node, None):
            if not isinstance(obj, URIRef):
                continue
            scope = p._context_scope(obj)
            if scope is not None and scope[1] == "spec":
                owners.add(get_valid_var_name(scope[0]))
        if len(owners) == 1:
            owner_map[closure_id] = owners.pop()
    return owner_map


def _pose_frames(g, pose) -> tuple[URIRef, URIRef]:
    """Return a pose quantity's authored `(of, with-respect-to)` frames."""
    relation = (
        PoseModel(pose, g)
        if URI_GEOM_TYPE_POSE in get_node_types(g, pose)
        else PoseCoordModel(pose, g).relation
    )
    return relation.of_id, relation.wrt_id


def _emit_derived_pose(g, node: URIRef, of_frame: URIRef, wrt_frame: URIRef) -> None:
    """Materialize one runtime-derived pose in comp-rob2b relation/coordinate form."""
    origins = []
    for frame in (of_frame, wrt_frame):
        origin = g.value(frame, URI_GEOM_PRED_ORIGIN)
        if not isinstance(origin, URIRef):
            origin = URIRef(f"{frame}-origin")
            g.add((frame, URI_GEOM_PRED_ORIGIN, origin))
        g.add((frame, RDF.type, URI_GEOM_TYPE_FRAME))
        g.add((origin, RDF.type, URI_GEOM_TYPE_POINT))
        origins.append(origin)

    pose_relation = URIRef(f"{node}-pose-rel")
    position_relation = URIRef(f"{node}-position-rel")
    orientation_relation = URIRef(f"{node}-orientation-rel")
    for relation, relation_type, of_entity, wrt_entity in (
        (pose_relation, URI_GEOM_TYPE_POSE, of_frame, wrt_frame),
        (position_relation, URI_GEOM_TYPE_POSITION, origins[0], origins[1]),
        (orientation_relation, URI_GEOM_TYPE_ORIENT, of_frame, wrt_frame),
    ):
        g.add((relation, RDF.type, relation_type))
        g.add((relation, RDF.type, QUDT_SCHEMA.Quantity))
        g.add((relation, URI_GEOM_PRED_OF, of_entity))
        g.add((relation, URI_GEOM_PRED_WRT, wrt_entity))
    g.add((position_relation, QUDT_SCHEMA.hasQuantityKind, URI_QUDT_QK_LENGTH))
    for reference_type in (URI_GEOM_TYPE_POSITION_REF, URI_GEOM_TYPE_ORIENT_REF):
        g.add((pose_relation, RDF.type, reference_type))
    g.add((pose_relation, URI_GEOM_PRED_OF_POSITION, position_relation))
    g.add((pose_relation, URI_GEOM_PRED_OF_ORIENT, orientation_relation))

    g.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    for coordinate_type in (
        URI_GEOM_TYPE_POSE_COORD,
        URI_GEOM_TYPE_POSE_REF,
        URI_GEOM_TYPE_POSITION_COORD,
        URI_GEOM_TYPE_POSITION_REF,
        URI_GEOM_TYPE_ORIENT_COORD,
        URI_GEOM_TYPE_ORIENT_REF,
        URI_GEOM_TYPE_VECTOR_XYZ,
    ):
        g.add((node, RDF.type, coordinate_type))
    g.add((node, URI_GEOM_PRED_OF_POSE, pose_relation))
    g.add((node, URI_GEOM_PRED_OF_POSITION, position_relation))
    g.add((node, URI_GEOM_PRED_OF_ORIENT, orientation_relation))
    g.add((node, URI_GEOM_PRED_SEEN_BY, wrt_frame))
    g.add((node, QUDT_SCHEMA.unit, URI_QUDT_UNIT_M))
    g.add((node, QUDT_SCHEMA.unit, URI_QUDT_UNIT_RAD))


def _derived_node(node, suffix) -> URIRef:
    return URIRef(f"{node}.derived-{suffix}")


def _pose_edges(g) -> dict:
    """Frame adjacency over every authored pose, traversable in both directions."""
    edges = collections.defaultdict(list)
    for pose in g.subjects(RDF.type, GEOM_REL.Pose):
        try:
            of_frame, wrt_frame = _pose_frames(g, pose)
        except ValueError:
            continue
        edges[wrt_frame].append((of_frame, pose, False))
        edges[of_frame].append((wrt_frame, pose, True))
    return edges


def _pose_path(edges, start: URIRef, goal: URIRef) -> tuple:
    """Shortest `(pose, inverted)` chain from `start` to `goal`; empty when unreachable."""
    queue = collections.deque([(start, ())])
    seen = {start}
    while queue:
        frame, current_path = queue.popleft()
        for next_frame, pose, inverted in edges[frame]:
            if next_frame in seen:
                continue
            next_path = (*current_path, (pose, inverted))
            if next_frame == goal:
                return next_path
            seen.add(next_frame)
            queue.append((next_frame, next_path))
    return ()


def _compose_path(g, owner: URIRef, path, start_wrt: URIRef):
    """Emit the invert/compose operations along `path`; return the composed pose, or None."""
    current = None
    current_wrt = start_wrt
    for index, (pose, inverted) in enumerate(path):
        pose_of, pose_wrt = _pose_frames(g, pose)
        step = pose
        step_of, step_wrt = pose_of, pose_wrt
        if inverted:
            step = _derived_node(owner, f"path-{index}-inverse")
            _emit_derived_pose(g, step, pose_wrt, pose_of)
            operation = _derived_node(owner, f"path-{index}-invert")
            g.add((operation, RDF.type, GEOM_OP.InvertPose))
            g.add((operation, GEOM_OP.pose, pose))
            g.add((operation, GEOM_OP.out, step))
            step_of, step_wrt = pose_wrt, pose_of
        if current is None:
            current = step
            current_wrt = step_wrt
            continue
        composite = _derived_node(owner, f"path-{index}-pose")
        _emit_derived_pose(g, composite, step_of, current_wrt)
        operation = _derived_node(owner, f"path-{index}-compose")
        g.add((operation, RDF.type, GEOM_OP.ComposePose))
        g.add((operation, GEOM_OP.in1, current))
        g.add((operation, GEOM_OP.in2, step))
        g.add((operation, GEOM_OP.composite, composite))
        current = composite
    return current


def _materialize_linear_distance_operations(g) -> None:
    """Expand authored linear-distance relations into codegen operations.

    The DSL graph states only the two pose endpoints; the inverse/composition path between
    their reference frames is derivable from the complete RDF graph.
    """
    edges = _pose_edges(g)
    for distance in list(g.subjects(RDF.type, GEOM_REL.LinearDistance)):
        if next(g.subjects(CSTR.quantity, distance), None) is None:
            continue
        endpoints = list(dict.fromkeys(g.objects(distance, GEOM_REL["between-entities"])))
        if len(endpoints) != 2:
            raise ValueError(f"Linear distance {distance} needs exactly two pose endpoints.")
        start, end = endpoints
        start_of, start_wrt = _pose_frames(g, start)
        end_of, end_wrt = _pose_frames(g, end)

        path = ()
        if start_wrt != end_wrt:
            path = _pose_path(edges, start_wrt, end_wrt)
            if not path:
                raise ValueError(
                    f"Linear distance {distance} has no pose path from {start_wrt} to {end_wrt}."
                )
        current = _compose_path(g, distance, path, start_wrt)

        end_in_start_reference = end
        if current is not None:
            end_in_start_reference = _derived_node(distance, "end-in-start-reference")
            _emit_derived_pose(g, end_in_start_reference, end_of, start_wrt)
            operation = _derived_node(distance, "compose-reference-path")
            g.add((operation, RDF.type, GEOM_OP.ComposePose))
            g.add((operation, GEOM_OP.in1, current))
            g.add((operation, GEOM_OP.in2, end))
            g.add((operation, GEOM_OP.composite, end_in_start_reference))

        inverse_start = _derived_node(distance, "inverse-start")
        _emit_derived_pose(g, inverse_start, start_wrt, start_of)
        invert_start = _derived_node(distance, "invert-start")
        g.add((invert_start, RDF.type, GEOM_OP.InvertPose))
        g.add((invert_start, GEOM_OP.pose, start))
        g.add((invert_start, GEOM_OP.out, inverse_start))

        relative_pose = _derived_node(distance, "relative-pose")
        _emit_derived_pose(g, relative_pose, end_of, start_of)
        compose_relative = _derived_node(distance, "compose-relative-pose")
        g.add((compose_relative, RDF.type, GEOM_OP.ComposePose))
        g.add((compose_relative, GEOM_OP.in1, inverse_start))
        g.add((compose_relative, GEOM_OP.in2, end_in_start_reference))
        g.add((compose_relative, GEOM_OP.composite, relative_pose))

        operation = _derived_node(distance, "magnitude")
        g.add((operation, RDF.type, GEOM_OP.PoseToLinearDistance))
        g.add((operation, GEOM_OP.pose, relative_pose))
        g.add((operation, GEOM_OP.distance, distance))


def _materialize_pose_reference_transforms(g) -> None:
    """Re-express a full-pose equality reference into the constrained pose's frame.

    A full-pose EqualityConstraint states its reference in whatever frame it was authored; when
    that differs from the constrained quantity's `with-respect-to` frame the comparison first
    needs the reference re-expressed.
    """
    edges = _pose_edges(g)
    for constraint in list(g.subjects(RDF.type, CSTR.EqualityConstraint)):
        quantity = g.value(constraint, CSTR.quantity)
        reference = g.value(constraint, CSTR["reference-value"])
        if quantity is None or reference is None:
            continue
        if GEOM_REL.Pose not in get_node_types(g, quantity):
            continue
        if GEOM_REL.Pose not in get_node_types(g, reference):
            continue
        try:
            target_of, target_wrt = _pose_frames(g, quantity)
            source_of, source_wrt = _pose_frames(g, reference)
        except ValueError:
            # A coordinate-authored goal pose carries no of/with-respect-to frames; it is
            # already stated in the constrained pose's frame.
            continue
        if source_wrt == target_wrt:
            continue
        if source_of != target_of:
            raise ValueError(
                f"Equality constraint {constraint} compares a pose of {target_of} "
                f"to a reference of {source_of}."
            )

        path = _pose_path(edges, target_wrt, source_wrt)
        if not path:
            raise ValueError(
                f"Equality constraint {constraint} has no pose path from "
                f"{target_wrt} to {source_wrt}."
            )
        current = _compose_path(g, constraint, path, target_wrt)

        reference_in_target = _derived_node(constraint, "reference-in-target")
        _emit_derived_pose(g, reference_in_target, source_of, target_wrt)
        operation = _derived_node(constraint, "compose-reference")
        g.add((operation, RDF.type, GEOM_OP.ComposePose))
        g.add((operation, GEOM_OP.in1, current))
        g.add((operation, GEOM_OP.in2, reference))
        g.add((operation, GEOM_OP.composite, reference_in_target))

        g.remove((constraint, CSTR["reference-value"], reference))
        g.add((constraint, CSTR["reference-value"], reference_in_target))


def _data_reference_map(data_structures, closures: dict) -> dict[str, str]:
    """Map each data id to the ids it references through closures."""
    data_reference_map: dict[str, str] = {}
    for item in data_structures:
        ref = getattr(item, "reference_value", None)
        ref_id = ref if isinstance(ref, str) else getattr(ref, "id", None)
        if ref_id:
            data_reference_map[item.id] = ref_id
    for c in closures.values():
        if not isinstance(c, dict) or c.get("type") != "AssignmentEvaluator":
            continue
        quantity_id = c.get("quantity")
        ref_id = c.get("reference_value")
        if isinstance(quantity_id, str) and isinstance(ref_id, str):
            data_reference_map[quantity_id] = ref_id
    return data_reference_map


def _closure_maps(closures: dict) -> tuple[dict[str, str], dict[str, set[str]]]:
    """Build (closure_output_map, closure_input_map): data id to the closures producing/consuming it."""
    closure_output_map: dict[str, str] = {}
    closure_input_map: dict[str, set[str]] = {}
    for cid, c in closures.items():
        if not isinstance(c, dict):
            continue
        outputs = closure_output_ids(c)
        inputs = {
            value
            for key, value in c.items()
            if key not in {"id", "type"} and isinstance(value, str) and value not in outputs
        }
        for out_val in outputs:
            closure_output_map[out_val] = cid
            closure_input_map[out_val] = inputs
    return closure_output_map, closure_input_map


_ORIENTATION_COMPONENTS = {
    "quaternion": ("x", "y", "z", "w"),
    "euler": ("x", "y", "z"),  # symbolic only: the angles arrive at runtime
    "relative": (),
}


def _empty_pose_entry(representation: str, euler_axes: str | None = None) -> dict:
    """Blank component slots for a pose, sized to what will fill them. A symbolic Euler triple is
    filled one angle per authored axis -- `zyx` fills z, y and x -- so it is sized by the sequence
    the model wrote rather than by a fixed component list.
    """
    entry = {"representation": representation}
    entry.update({f"position_{axis}": None for axis in ("x", "y", "z")})
    names = tuple(euler_axes) if euler_axes else _ORIENTATION_COMPONENTS[representation]
    entry.update({f"orientation_{name}": None for name in names})
    return entry


def build_pose_components(views: dict, data: list, graph=None, pose_nodes=None) -> dict:
    """Resolve declared/inline poses into per-axis structured components (a literal value or a reference id)."""
    data_by_id = _index_by_id(data)
    components: dict[str, dict] = {}
    if graph is not None:
        for pose in data:
            if _field(pose, "type") != "Pose":
                continue
            pose_id = _field(pose, "id")
            coord_id = (pose_nodes or {}).get(pose_id)
            if coord_id is None:
                continue
            coordinate = PoseCoordModel(coord_id, graph)
            representation = _field(pose, "orientation_representation") or "quaternion"
            entry = _empty_pose_entry(representation, _field(pose, "euler_axes_sequence"))
            position = get_coord_vectorxyz(coordinate.position_coord, graph)
            if position is not None:
                length_unit = _length_unit(coordinate.position_coord)
                for axis, value in zip("xyz", _si_all(position, length_unit)):
                    entry[f"position_{axis}"] = {"value": str(value), "ref": None}
            if representation == "quaternion":
                # Euler angles, quaternion or direction cosines all resolve to one quaternion
                # here; anything sourced at runtime keeps its shape and is rendered, not resolved.
                rotation = get_orientation_coord_vals(coordinate.orientation_coord, graph)
                values = rotation.as_quat() if rotation is not None else None
                labels = "xyzw"
            else:
                values = None
                labels = ()
            if values is not None:
                for label, value in zip(labels, values):
                    entry[f"orientation_{label}"] = {"value": str(float(value)), "ref": None}
            if representation == "relative":
                entry["orientation_operands"] = _field(pose, "orientation_operands")
            if any(value is not None for key, value in entry.items() if key != "representation"):
                components[pose_id] = entry
    for view in views.values():
        superobject = _field(view, "superobject")
        so_type = _field(superobject, "type")
        so_prov = _field(superobject, "provenance") or {}
        is_declared_pose = bool(_field(so_prov, "authored") or _field(so_prov, "snapshot"))
        if so_type != "Pose":
            continue
        if not (is_declared_pose or _field(superobject, "euler_axes_sequence")):
            continue
        pose_id = _field(superobject, "id")
        representation = _field(superobject, "orientation_representation") or "quaternion"
        axis = str(_field(view, "axis") or "").lower()
        subobject = _field(_field(view, "subobject"), "id")
        if not subobject or axis not in {"x", "y", "z", "w"}:
            continue
        entry = components.setdefault(
            pose_id, _empty_pose_entry(representation, _field(superobject, "euler_axes_sequence"))
        )
        if representation == "relative":
            entry["orientation_operands"] = _field(superobject, "orientation_operands")
        prefix = "position" if _field(view, "subspace") == "Linear" else "orientation"
        entry[f"{prefix}_{axis}"] = _pose_component(subobject, data_by_id)
    for pose_id, parts in components.items():
        missing = [name for name, value in parts.items() if value is None]
        if missing:
            raise ValueError(
                f"Declared pose '{pose_id}' is missing required components: {', '.join(missing)}."
            )
        if parts["representation"] == "euler":
            parts["euler_factors"] = _euler_factors(pose_id, parts, data_by_id)
    return components


def _euler_factors(pose_id: str, parts: dict, data_by_id: dict) -> list[dict]:
    """A symbolic Euler triple as per-axis rotations, in the order they multiply.

    An extrinsic sequence turns about axes that stay put, so the rotation authored last multiplies
    on the left; an intrinsic one turns about axes carried along by the previous rotations, so the
    order reverses. Each component renders wherever its value comes from.
    """
    pose = data_by_id.get(pose_id)
    sequence = _field(pose, "euler_axes_sequence") or "xyz"
    factors = [
        {"axis": axis, "component": parts[f"orientation_{axis}"]}
        for axis in sequence
        if parts.get(f"orientation_{axis}") is not None
    ]
    if len(factors) != len(sequence):
        raise ValueError(f"Euler pose '{pose_id}' has no component for every axis of '{sequence}'.")
    return factors if _field(pose, "euler_intrinsic") else list(reversed(factors))


def resolve_lerp_closures(closures: dict, pose_components: dict) -> None:
    """Fold each linear-path goal into components or a shared-signal ref."""
    for closure in closures.values():
        if closure.get("shape") != "LinearPath":
            continue
        goal = closure.get("goal")
        if not isinstance(goal, str):
            continue
        if goal in pose_components and closure.get("type") == "PathProjection":
            # The template builds the pose frame. Only the projection assigns: it is scheduled
            # before the frame and the evaluator, which read the same shared goal.
            closure["goal_components"] = pose_components[goal]
            closure["assign_goal"] = True
        else:
            # goal is a shared signal id (already on the closure as closure["goal"]).
            closure["assign_goal"] = False


def resolve_arc_closures(closures: dict, data: list) -> None:
    """Validate that each Arc closure's end is a Pose (the template renders its position/orientation)."""
    data_by_id = _index_by_id(data)

    def is_pose(data) -> bool:
        """True when a data structure is a Pose quantity."""
        qkind = _field(data, "quantity_kind")
        qkind_ids = qkind if isinstance(qkind, list) else [qkind]
        return _field(data, "type") == "Pose" or any(
            _field(item, "id") == "Pose" for item in qkind_ids
        )

    for closure in closures.values():
        if closure.get("shape") != "Arc":
            continue
        end = closure.get("end")
        end_data = data_by_id.get(end)
        if not isinstance(end, str) or not is_pose(end_data):
            raise ValueError("Arc path end must be a Pose quantity.")


def declared_pose_component_entries(
    data: list, pose_components: dict, referenced_ids: set[str] | None = None
) -> list[dict]:
    """Authored declared-pose component entries, optionally restricted to the referenced ids."""
    data_by_id = _index_by_id(data)
    entries = []
    for pose_id, parts in pose_components.items():
        if referenced_ids is not None and pose_id not in referenced_ids:
            continue
        item = data_by_id.get(pose_id)
        item_prov = _field(item, "provenance") or {}
        if not _field(item_prov, "authored") or _field(item_prov, "snapshot"):
            continue
        entries.append({"id": pose_id, **parts})
    return entries


def collect_motion_references(motion, closures: dict) -> set[str]:
    """Every id a motion references, including through its scheduled closures."""
    refs: set[str] = set()

    def visit(value):
        """Recurse a value collecting every string id it references."""
        if isinstance(value, str):
            refs.add(value)
        elif isinstance(value, dict):
            for item in value.values():
                visit(item)
        elif isinstance(value, list):
            for item in value:
                visit(item)

    visit(_as_dict(motion))
    for schedule_name in ("when_schedule", "while_schedule", "until_schedule"):
        for step in _field(motion, schedule_name, []):
            closure = closures.get(step)
            if closure:
                visit(closure)
    return refs


def _elapsed_coordinate_id(e) -> str:
    """The shared value an elapsed constraint measures: its own authored duration coordinate. The
    error signal is the elapsed duration itself, so this is where the motion writes the seconds and
    where the condition, the introspection sample and the frame log all find them.
    """
    coordinate = _field(_field(e, "error"), "id")
    if not coordinate:
        raise ValueError(
            f"elapsed constraint '{_field(e, 'id')}' has no duration coordinate to measure into"
        )
    return coordinate


def _elapsed_coordinate_ids(evaluators) -> list[str]:
    """Each phase's elapsed coordinates, deduplicated, in authored order."""
    return list(
        dict.fromkeys(
            _elapsed_coordinate_id(e) for e in evaluators if getattr(e, "is_elapsed", False)
        )
    )
