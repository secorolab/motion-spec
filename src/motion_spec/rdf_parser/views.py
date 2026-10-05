# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What each motion reads through the quantities: the MAP views onto them, the data structures they
are, and per motion the references, snapshots, pose-error groups and pose components it reaches.
"""

from __future__ import annotations

from dataclasses import asdict

from motion_spec_dsl.rdf_parser.vocab import (
    CSTR_EXT,
    CSTR_HDL,
    GEOM_COORD,
    MAP,
    MAP_EXT,
    QUDT_SCHEMA,
    RBDYN_COORD,
)
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdf_utils.models.geom_coord import PoseCoordModel
from rdf_utils.models.geom_rel import OrientationModel, PositionModel
from rdf_utils.models.vocab import (
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_GEOM_TYPE_ORIENT,
    URI_GEOM_TYPE_ORIENT_COORD,
    URI_GEOM_TYPE_POSE_COORD,
    URI_GEOM_TYPE_POSITION,
    URI_GEOM_TYPE_POSITION_COORD,
)
from rdflib.namespace import RDF

from motion_spec.classes.constraints import EqualityConstraint
from motion_spec.classes.geometry import Orientation, Pose, SpatialCoordinate, Subspace, View
from motion_spec.classes.motion import (
    ComponentRef,
    DataValue,
    PoseComponents,
    PoseErrorComponent,
    PoseErrorRegroup,
    SnapshotCapture,
)
from motion_spec.classes.qudt import FreeVector, Quantity, SetpointQuantity
from motion_spec.rdf_parser.operations import function_output_ids, recorded_coord_policy
from motion_spec.rdf_parser.quantities import (
    acceleration_twist,
    axis,
    direction,
    is_constraint_aggregate,
    orientation,
    orientation_quaternion,
    pose,
    pose_difference,
    position,
    position_values,
    quantity,
    snapshot_target_ids,
    subspace,
    velocity_twist,
    wrench,
)

# The superobject reader each view type dispatches to.
_VIEW_SUPEROBJECTS = (
    (MAP_EXT["PoseCoordinateView"], pose),
    (MAP_EXT["VelocityTwistCoordinateView"], velocity_twist),
    (MAP_EXT["AccelerationTwistCoordinateView"], acceleration_twist),
    (MAP_EXT["PoseDifferenceView"], pose_difference),
    (MAP_EXT["WrenchCoordinateView"], wrench),
)
# A component view of a superobject that is itself a 3-vector carries no coordinate-view type of
# its own -- there is no subspace to cut -- so it is dispatched on what its superobject is.
_VECTOR_SUPEROBJECTS = ((GEOM_COORD["DirectionCoordinate"], direction),)
# A combined PoseCoordinate is also a PositionCoordinate and an OrientationCoordinate; dispatch
# the most specific type first, and dedupe by id afterwards.
_DATA_STRUCTURE_READERS = (
    (GEOM_COORD["DirectionCoordinate"], direction),
    (GEOM_COORD["PoseCoordinate"], pose),
    (GEOM_COORD["PositionCoordinate"], position),
    (GEOM_COORD["OrientationCoordinate"], orientation),
    (GEOM_COORD["VelocityTwistCoordinate"], velocity_twist),
    (GEOM_COORD["AccelerationTwistCoordinate"], acceleration_twist),
    (GEOM_COORD["PoseDifferenceCoordinate"], pose_difference),
    (RBDYN_COORD["WrenchCoordinate"], wrench),
    (QUDT_SCHEMA["Quantity"], quantity),
)


def read_views(model) -> dict:
    """Every MAP view in the graph, by view id.

    Returns:
        one `View` per `map:View` node, carrying the superobject it cuts, the subobject it names,
        the subspace and, when it selects one, the axis

    Raises:
        ConstraintViolation: a view's type matches no superobject reader.
    """
    graph = model.graph
    views = {}
    for node in graph[: RDF["type"] : MAP["View"]]:
        types = get_node_types(graph, node)
        superobject_node = graph.value(node, MAP["superobject"])
        read = next((func for type_, func in _VIEW_SUPEROBJECTS if type_ in types), None)
        if read is None:
            super_types = get_node_types(graph, superobject_node)
            read = next(
                (func for type_, func in _VECTOR_SUPEROBJECTS if type_ in super_types), quantity
            )
        superobject = read(model, superobject_node)
        if superobject is None:
            raise ConstraintViolation(
                "geometry", f"MAP view {node} has an unrecognized type; no view reader matched"
            )
        axis_node = graph.value(node, MAP["axis"])
        # A superobject that is itself a 3-vector has no half to name, so its component view
        # states only the axis.
        subspace_node = graph.value(node, MAP["subspace"])
        views[model.id(node)] = View(
            model.id(node),
            superobject,
            # The view itself, not its bare subobject: a pooled relation has several
            # coordinates and the view's superobject pins which sampling is meant.
            quantity(model, node),
            subspace(subspace_node) if subspace_node is not None else None,
            axis(axis_node) if axis_node is not None else None,
        )

    return views


def _is_pooled_relation(model, node) -> bool:
    """A relation sampled by several coordinates: the samplings are the data, not the relation."""
    types = get_node_types(model.graph, node)
    if URI_GEOM_TYPE_POSITION in types and URI_GEOM_TYPE_POSITION_COORD not in types:
        return len(PositionModel(node, model.graph).coordinate_ids) > 1
    if URI_GEOM_TYPE_ORIENT in types and URI_GEOM_TYPE_ORIENT_COORD not in types:
        return len(OrientationModel(node, model.graph).coordinate_ids) > 1
    return False


def read_data_structures(model) -> list:
    """Every data-structure entity the model itself declares or derives.

    A scene element's coordinates are scene-dsl's and reach the run through the world model, not
    the algorithm data, so only the model's own nodes are read.

    Returns:
        one record per node, read by the most specific reader its types match, so a combined
        pose coordinate is read as a pose rather than as its position half
    """
    own = model.node_by_id
    reader_by_node = {}
    for type_, read in _DATA_STRUCTURE_READERS:
        for node in model.graph[: RDF["type"] : type_]:
            # A pooled relation is no slot of its own: each of its samplings is one.
            if own.get(model.id(node)) != node or _is_pooled_relation(model, node):
                continue
            # The readers run most specific first; a node keeps the first that matches it.
            reader_by_node.setdefault(node, read)
    return [read(model, node) for node, read in reader_by_node.items()]


def views_by_subobject(views: dict) -> dict[str, list]:
    """Every view onto each subobject id."""
    indexed: dict[str, list] = {}
    for view in views.values():
        if view.subobject.id:
            indexed.setdefault(view.subobject.id, []).append(view)
    return indexed


def _unique_view(indexed, subobject_id, context):
    """The one view onto a subobject, or None; several disagreeing views is an error."""
    matches = indexed.get(subobject_id, ())
    if len(matches) > 1:
        raise ConstraintViolation(
            "geometry",
            f"{context}: quantity '{subobject_id}' is the subobject of multiple MAP views",
        )
    return matches[0] if matches else None


def views_for_access(
    views: dict, algorithm_data: list, motions, functions: dict, pose_components: dict
) -> dict:
    """Unambiguous MAP views by subobject, for the view's access expressions.

    A subobject written directly -- an authored or literal D-block, a snapshot target, a
    function output -- is an ordinary shared quantity and keeps its own field; one reused by views
    that disagree on how they access it must not silently pick one of them.

    Raises:
        ConstraintViolation: a subobject is left with neither a direct write nor one agreed
            reading, so nothing could compute it.
    """
    # A declared pose's components are bound *into* it. Reading one back off the pose it helps
    # define is circular -- and it is the quantity's own superobject that says how to read it --
    # so the binding is not a candidate reading, however much it looks like one.
    bound_into_pose = {
        (pose_id, component["ref"])
        for pose_id, parts in pose_components.items()
        for component in asdict(parts).values()
        if isinstance(component, dict) and component.get("ref")
    }
    direct_ids = {
        item.id
        for item in algorithm_data
        if item.id
        and (
            isinstance(item, (Quantity, DataValue))
            and item.value is not None
            or isinstance(
                item, (Quantity, FreeVector, SetpointQuantity, Orientation, Pose, SpatialCoordinate)
            )
            and item.provenance.authored
        )
    }
    direct_ids.update(snapshot.target_id for motion in motions for snapshot in motion.snapshots)
    direct_ids.update(
        output_id for function in functions.values() for output_id in function_output_ids(function)
    )

    indexed: dict[str, object] = {}
    for view in views.values():
        # A constraint may name the view itself (relations are pooled), so every view also
        # maps under its own id; view ids are unique, so this never conflicts.
        indexed.setdefault(view.id, view)
        subobject_id = view.subobject.id
        if not subobject_id or subobject_id in direct_ids:
            continue
        if subobject_id == view.superobject.id:
            # A whole-component view of a pooled relation reads the superobject's own
            # sampling; the superobject is computed elsewhere, not through this view.
            continue
        if (view.superobject.id, subobject_id) in bound_into_pose:
            continue
        previous = indexed.setdefault(subobject_id, view)
        if previous is view or previous is None:
            continue
        if (previous.superobject, previous.subspace, previous.axis, previous.direction) != (
            view.superobject,
            view.subspace,
            view.axis,
            view.direction,
        ):
            indexed[subobject_id] = None

    # Dropping the reading here used to leave the quantity to render as its own shared field --
    # a field nothing writes, which compiles to a zero and flies the robot at it. Nothing can
    # compute this quantity, so say so instead of emitting the zero.
    unreadable = [id_ for id_, view in indexed.items() if view is None]
    if unreadable:
        raise ConstraintViolation(
            "geometry",
            "no way to compute "
            + ", ".join(f"'{id_}'" for id_ in unreadable)
            + ": neither written directly nor read through a single agreed MAP view",
        )

    return dict(indexed)


def expanded_constraints(model, nodes) -> set:
    """A phase's constraints, with any when/until aggregate replaced by its members."""
    return {
        member
        for node in nodes
        for member in (
            model.graph[node : CSTR_EXT["has-constraint"]]
            if is_constraint_aggregate(model, node)
            else (node,)
        )
    }


_GROUPABLE_SUPEROBJECTS = {"Pose", "VelocityTwist", "AccelerationTwist", "Wrench"}
_GROUP_AXIS_BY_SUBSPACE = {Subspace.Linear: ("linear", False), Subspace.Angular: ("angular", True)}


def pose_axis_error_groups_for_motion(model, evaluators_by_node: dict, views: dict) -> list:
    """A motion's per-axis error evaluators, regrouped into one group per superobject.

    Parameters:
        evaluators_by_node: the phase's evaluator records, keyed by the node each was read from

    Returns:
        the groups with more than one component; a lone axis needs no regrouping, its own
        equality-constraint controller drives it
    """
    groups: dict[str, PoseErrorRegroup] = {}
    indexed = views_by_subobject(views)
    for node, evaluator in evaluators_by_node.items():
        if CSTR_HDL["ErrorEvaluator"] not in get_node_types(model.graph, node):
            continue
        if not isinstance(evaluator.constraint.parameter, EqualityConstraint):
            continue
        if evaluator.error is None:
            continue
        quantity_record = evaluator.constraint.quantity
        view = _unique_view(indexed, quantity_record.id, "pose-axis error grouping")
        if view is None:
            continue
        superobject_type = view.superobject.type
        if superobject_type not in _GROUPABLE_SUPEROBJECTS:
            continue
        # Which half of the superobject the view selects, or None when it joins no group.
        mapping = _GROUP_AXIS_BY_SUBSPACE.get(view.subspace)
        if mapping is None and superobject_type == "Pose":
            kind = quantity_record.quantity_kind.id if isinstance(quantity_record, Quantity) else ""
            if "Angle" in kind or "angle" in kind.lower() or "rotation" in quantity_record.id:
                mapping = ("angular", True)
        # A whole-subspace view has no per-axis component, so it cannot join a per-axis group;
        # its own equality-constraint controller drives it.
        if mapping is None or view.axis is None:
            continue
        half, is_angular = mapping

        superobject_id = view.superobject.id
        group = groups.setdefault(
            superobject_id,
            PoseErrorRegroup(
                id=f"pose_axis_error_{superobject_id}",
                pose=superobject_id,
                components=[],
                superobject_type=superobject_type,
            ),
        )
        group.has_angular = group.has_angular or is_angular
        reference_id = evaluator.constraint.parameter.reference_value.id
        group.components.append(
            PoseErrorComponent(
                quantity=quantity_record.id,
                error=evaluator.error.id,
                reference=reference_id,
                subspace=half,
                axis=view.axis.value,
                eval_id=evaluator.id,
            )
        )
        setattr(group, f"{half}_{view.axis.value.lower()}", reference_id)

    return [group for group in groups.values() if len(group.components) > 1]


def _reference_value_id(constraint_record) -> str | None:
    """The id of a constraint's reference-value parameter, or None."""
    parameter = constraint_record.parameter
    return parameter.reference_value.id if isinstance(parameter, EqualityConstraint) else None


def snapshots_for_motion(
    evaluators, constraints, indexes, views: dict, schedule, functions: dict, token, tokens
) -> list:
    """A motion's sample-and-hold captures, from every reference value it reaches.

    Parameters:
        token: the motion's own id, as `snapshot_owner` names declaring blocks
        tokens: every motion's id, so a shared snapshot can be told from another motion's

    Returns:
        one capture per snapshot target this motion may sample
    """
    referenced = {
        reference
        for record in [
            *(e.constraint for e in evaluators if e.constraint is not None),
            *constraints,
        ]
        if (reference := _reference_value_id(record))
    }
    subobjects_by_super: dict[str, list[str]] = {}
    supers_by_subobject: dict[str, list[str]] = {}
    for view in views.values():
        super_id = view.superobject.id
        subobject_id = view.subobject.id
        if super_id and subobject_id:
            subobjects_by_super.setdefault(super_id, []).append(subobject_id)
            supers_by_subobject.setdefault(subobject_id, []).append(super_id)

    # Every id reachable from a reference or a scheduled function's ids, in both view directions.
    pending = list(referenced)
    for step in schedule:
        for value in (functions.get(step) or {}).values():
            pending += [
                item
                for item in (value if isinstance(value, list) else [value])
                if isinstance(item, str)
            ]
    seen = set()
    while pending:
        current = pending.pop()
        if current in seen:
            continue
        seen.add(current)
        referenced.add(current)
        referred = indexes.data_reference.get(current)
        if referred:
            pending.append(referred)
        # superobject -> subobject (forward decomposition), and subobject -> superobject (a
        # composite pose's snapshot is only referenced through its scalar components, so
        # climb back to capture the composite).
        pending.extend(subobjects_by_super.get(current, ()))
        pending.extend(supers_by_subobject.get(current, ()))
        # A composed orientation names its base pose only through the compose operator.
        parts = indexes.pose_components.get(current)
        if parts is not None:
            pending.extend(
                operand["pose"]
                for operand in parts.orientation_operands or ()
                if isinstance(operand.get("pose"), str)
            )

    result = []
    for target_id in referenced:
        if target_id not in indexes.snapshot_source:
            continue
        # Capture only what this motion declares: re-capturing another motion's snapshot would
        # overwrite its value. A shared-context snapshot is owned by no motion, so every motion
        # that reads it emits the capture: one naming a trigger is re-sampled on that event by
        # whichever reading motion is active on the tick it is current, one naming none is
        # latched once for the run.
        owner = indexes.snapshot_owner.get(target_id)
        if owner in tokens and owner != token:
            continue
        trigger = indexes.snapshot_trigger.get((owner, target_id))
        source_id = indexes.snapshot_source[target_id]
        scope = "event" if trigger else ("entry" if owner in tokens else "task")
        writers = (
            set()
            if source_id in supers_by_subobject
            else indexes.function_output.get(source_id, set())
        )
        if len(writers) > 1:
            raise ConstraintViolation(
                "coordination",
                f"snapshot '{target_id}' captures '{source_id}', which {len(writers)} functions "
                "write -- a capture computes its source once",
            )
        result.append(
            SnapshotCapture(
                target_id=target_id,
                source_id=source_id,
                scope=scope,
                # Only a run-scoped capture is latched; an entry- or event-scoped one re-captures.
                captured_id=f"{target_id}_captured" if scope == "task" else None,
                source_function_id=next(iter(writers), None),
                trigger_event=trigger,
            )
        )

    return result


def collect_motion_references(motion, functions: dict) -> set[str]:
    """Every id a motion references, including through the functions its schedules run.

    Returns:
        every string the motion record or one of its functions holds -- a superset of the ids,
        which is what the callers restrict a global table down to
    """
    references: set[str] = set()

    def visit(value) -> None:
        """Collect every string a value holds, however deeply nested."""
        if isinstance(value, str):
            references.add(value)
        elif isinstance(value, dict):
            for item in value.values():
                visit(item)
        elif isinstance(value, list):
            for item in value:
                visit(item)

    visit(asdict(motion))
    for step in [*motion.when_schedule, *motion.while_schedule, *motion.until_schedule]:
        visit(functions.get(step))

    return references


def collect_motion_input_references(motion, functions: dict) -> set[str]:
    """Every value the motion's computations and captures read, excluding solver outputs."""
    references: set[str] = set()

    def visit(value) -> None:
        if isinstance(value, str):
            references.add(value)
        elif isinstance(value, dict):
            for item in value.values():
                visit(item)
        elif isinstance(value, list):
            for item in value:
                visit(item)

    for step in [
        *motion.when_schedule,
        *motion.while_pre_schedule,
        *motion.while_schedule,
        *motion.until_schedule,
    ]:
        visit(functions.get(step))
    visit([asdict(snapshot) for snapshot in motion.snapshots])
    return references


def elapsed_coordinate_id(evaluator) -> str:
    """The D-block an elapsed constraint measures: its own authored duration coordinate.

    The error signal is the elapsed duration itself, so this is where the motion writes the
    seconds and where the condition, the telemetry sample and the frame log all find them.
    """
    coordinate = evaluator.error.id if evaluator.error is not None else None
    if not coordinate:
        raise ConstraintViolation(
            "quantity",
            f"elapsed constraint '{evaluator.id}' has no duration coordinate to measure into",
        )
    return coordinate


def elapsed_coordinate_ids(evaluators) -> list[str]:
    """A phase's elapsed coordinates, each once."""
    return list(
        {
            elapsed_coordinate_id(evaluator)
            for evaluator in evaluators
            if evaluator.is_elapsed and not evaluator.observed_at_id
        }
    )


def observation_ages(evaluators) -> list[dict]:
    """A phase's observation-age clocks: the coordinate each writes and the instant it counts from."""
    seen = {}
    for evaluator in evaluators:
        if evaluator.is_elapsed and evaluator.observed_at_id:
            seen.setdefault(
                elapsed_coordinate_id(evaluator),
                {
                    "coordinate": elapsed_coordinate_id(evaluator),
                    "observed_at": evaluator.observed_at_id,
                },
            )
    return list(seen.values())


_POSITION_FIELDS = ("position_x", "position_y", "position_z")


def _required_pose_component_fields(representation: str, euler_axes_sequence: str | None) -> tuple:
    """The component fields a pose's `representation` requires: position always;
    `orientation_x/y/z/w` for `quaternion`; one `orientation_*` per character of the Euler
    sequence for `euler`; none for `relative`. Stated once; both the completeness check below
    and `_euler_factors` read it.
    """
    if representation == "quaternion":
        return _POSITION_FIELDS + (
            "orientation_x",
            "orientation_y",
            "orientation_z",
            "orientation_w",
        )
    if representation == "euler":
        return _POSITION_FIELDS + tuple(
            f"orientation_{name}" for name in (euler_axes_sequence or "")
        )

    return _POSITION_FIELDS


def _pose_component(component_id: str, data_by_id: dict) -> ComponentRef:
    """A pose component as either a literal `value` or a `ref` id the backend template renders
    via access-expr. Backend-agnostic -- no target syntax here.
    """
    component = data_by_id.get(component_id)
    if isinstance(component, Quantity) and component.reference_value:
        return ComponentRef(ref=component.reference_value)
    if isinstance(component, (Quantity, DataValue)) and component.value is not None:
        return ComponentRef(value=str(component.value))
    return ComponentRef(ref=component_id)


def _authored_pose_entry(model, pose_record, coordinate_node) -> PoseComponents | None:
    """The literal components a coordinate-authored pose carries, or None when it carries none."""
    coordinate = PoseCoordModel(coordinate_node, model.graph, coord_policy=recorded_coord_policy)
    # A position the run draws is no authored component; the tree it joins carries the pose.
    if URI_DISTRIB_TYPE_SAMPLED_QUANTITY in coordinate.position_coord.types:
        return None
    representation = pose_record.orientation_representation or "quaternion"
    entry = PoseComponents(representation)
    values = position_values(model, coordinate.position_coord)
    if values is not None:
        for name, value in zip("xyz", values):
            setattr(entry, f"position_{name}", ComponentRef(value=str(value)))
    if representation == "quaternion":
        # Euler angles, quaternion or direction cosines all resolve to one quaternion here;
        # anything sourced at runtime keeps its shape and is rendered, not resolved.
        rotation = orientation_quaternion(model, coordinate.orientation_coord)
        for name, value in zip("xyzw", rotation or ()):
            setattr(entry, f"orientation_{name}", ComponentRef(value=str(value)))
    if representation == "relative":
        entry.orientation_operands = pose_record.orientation_operands
    filled = any(
        value is not None for key, value in asdict(entry).items() if key != "representation"
    )

    return entry if filled else None


def build_pose_components(model, views: dict, data: list) -> dict:
    """Declared and inline poses as typed, fully-published `PoseComponents` (DECISION 10)."""
    data_by_id = {item.id: item for item in data if item.id}
    pose_nodes = {
        model.id(node): node for node in model.graph.subjects(RDF.type, URI_GEOM_TYPE_POSE_COORD)
    }
    components: dict[str, PoseComponents] = {}
    for item in data:
        if item.type != "Pose" or item.id not in pose_nodes:
            continue
        entry = _authored_pose_entry(model, item, pose_nodes[item.id])
        if entry is not None:
            components[item.id] = entry

    snapshot_ids = snapshot_target_ids(model)
    for view in views.values():
        superobject = view.superobject
        # A snapshot pose is captured whole at runtime, so a per-axis view of it is a reading
        # off the captured frame, never a component bound into the pose.
        if superobject.type != "Pose" or superobject.id in snapshot_ids:
            continue
        if not (superobject.provenance.authored or superobject.euler_axes_sequence):
            continue
        component_axis = str(view.axis.value if view.axis else "").lower()
        subobject_id = view.subobject.id
        if not subobject_id or component_axis not in {"x", "y", "z", "w"}:
            continue
        representation = superobject.orientation_representation or "quaternion"
        entry = components.setdefault(superobject.id, PoseComponents(representation))
        if representation == "relative":
            entry.orientation_operands = superobject.orientation_operands
        prefix = "position" if view.subspace == Subspace.Linear else "orientation"
        setattr(entry, f"{prefix}_{component_axis}", _pose_component(subobject_id, data_by_id))

    for pose_id, parts in components.items():
        pose_record = data_by_id.get(pose_id)
        euler_axes_sequence = (
            pose_record.euler_axes_sequence if isinstance(pose_record, Pose) else None
        )
        required = _required_pose_component_fields(parts.representation, euler_axes_sequence)
        missing = [name for name in required if getattr(parts, name) is None]
        if missing:
            raise ConstraintViolation(
                "geometry",
                f"Declared pose '{pose_id}' is missing required components: {', '.join(missing)}.",
            )
        if parts.representation == "euler":
            parts.euler_factors = _euler_factors(pose_id, parts, data_by_id)

    return components


def _euler_factors(pose_id: str, parts: PoseComponents, data_by_id: dict) -> list[dict]:
    """A symbolic Euler triple as per-axis rotations, in the order they multiply.

    An extrinsic sequence turns about axes that stay put, so the rotation authored last multiplies
    on the left; an intrinsic one turns about axes carried along by the previous rotations, so the
    order reverses. Each component renders wherever its value comes from.
    """
    pose_record = data_by_id.get(pose_id)
    is_pose = isinstance(pose_record, Pose)
    sequence = (pose_record.euler_axes_sequence if is_pose else None) or "xyz"
    factors = [
        {"axis": name, "component": getattr(parts, f"orientation_{name}")}
        for name in sequence
        if getattr(parts, f"orientation_{name}") is not None
    ]
    if len(factors) != len(sequence):
        raise ConstraintViolation(
            "geometry", f"Euler pose '{pose_id}' has no component for every axis of '{sequence}'."
        )

    return factors if is_pose and pose_record.euler_intrinsic else list(reversed(factors))


def declared_pose_component_entries(
    model, data: list, pose_components: dict, referenced=None
) -> list:
    """Authored declared-pose component entries, restricted to the ids a motion references."""
    data_by_id = {item.id: item for item in data if item.id}
    snapshot_ids = snapshot_target_ids(model)

    # A composed orientation names its base pose only through the compose operator, so the
    # motion's own references miss it; and the base has to be built before what composes it.
    ordered: list[str] = []
    visiting: set[str] = set()

    def schedule(pose_id) -> None:
        if pose_id in ordered or pose_id not in pose_components:
            return
        if pose_id in visiting:
            raise ValueError(f"pose {pose_id} composes its own orientation")
        visiting.add(pose_id)
        for operand in pose_components[pose_id].orientation_operands or ():
            if isinstance(operand.get("pose"), str):
                schedule(operand["pose"])
        visiting.discard(pose_id)
        ordered.append(pose_id)

    for pose_id in pose_components:
        if referenced is None or pose_id in referenced:
            schedule(pose_id)

    entries = []
    for pose_id in ordered:
        pose_record = data_by_id.get(pose_id)
        if (
            not isinstance(pose_record, Pose)
            or not pose_record.provenance.authored
            or pose_id in snapshot_ids
        ):
            continue
        entries.append({"id": pose_id, **asdict(pose_components[pose_id])})

    return entries
