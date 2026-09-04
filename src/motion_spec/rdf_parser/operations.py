# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What is computed, and in what order.

Sections, in order: ``normalize`` · the operator protocol · the operator tables ·
``build_closures`` · ``Schedule`` · the closure indexes.

This module never asks what a value *is* -- that is ``quantities.py``'s question. It asks which
computations the model implies, what each call reads and writes, and in what order they run.
"""

from __future__ import annotations

import collections
import itertools
from typing import NamedTuple

from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT,
    CSTR,
    CSTR_EXT,
    CSTR_HDL,
    GEOM_COORD,
    GEOM_OP,
    GEOM_OP_EXT,
    GEOM_PATH,
    GEOM_REL,
    KC_STAT,
    MAP,
    QUDT_SCHEMA,
    RBDYN_OP,
    RBDYN_OP_EXT,
    SLV,
)
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdf_utils.models.geom_coord import PoseCoordModel
from rdf_utils.models.geom_rel import PoseModel
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_OF,
    URI_GEOM_PRED_OF_ORIENT,
    URI_GEOM_PRED_OF_POSE,
    URI_GEOM_PRED_OF_POSITION,
    URI_GEOM_PRED_ORIGIN,
    URI_GEOM_PRED_SEEN_BY,
    URI_GEOM_PRED_WRT,
    URI_GEOM_TYPE_FRAME,
    URI_GEOM_TYPE_ORIENT,
    URI_GEOM_TYPE_ORIENT_COORD,
    URI_GEOM_TYPE_ORIENT_REF,
    URI_GEOM_TYPE_POINT,
    URI_GEOM_TYPE_POSE,
    URI_GEOM_TYPE_POSE_COORD,
    URI_GEOM_TYPE_POSE_REF,
    URI_GEOM_TYPE_POSITION,
    URI_GEOM_TYPE_POSITION_COORD,
    URI_GEOM_TYPE_POSITION_REF,
    URI_GEOM_TYPE_VECTOR_XYZ,
    URI_KC_EXT_PRED_OF_JOINT,
    URI_KC_EXT_TYPE_JOINT_LIMIT,
    URI_KC_STAT_JNT_POSITION,
    URI_KC_TYPE_REVOLUTE_JOINT,
    URI_KC_TYPE_REVOLUTE_JOINT_ORIENTED_AXIS,
    URI_QUDT_QK_LENGTH,
    URI_QUDT_UNIT_M,
    URI_QUDT_UNIT_RAD,
)
from rdf_utils.naming import get_valid_var_name
from rdflib import URIRef
from rdflib.namespace import PROV, RDF

from motion_spec.rdf_parser.model import identifier, local_name, reader


def normalize(model) -> None:
    """Materialize the operations the model implies but does not spell out.

    A distance between two points, a pose reference expressed in another frame: the DSL states the
    endpoints and leaves the arithmetic between them implied. This is the only writer of the
    graph, and it runs before any reader, so afterwards the graph is complete and immutable --
    which is what makes one shared read cache correct.

    Pose reference transforms run first: they rewrite the reference a distance may then be taken
    between.
    """
    _materialize_pose_reference_transforms(model)
    _materialize_linear_distance_operations(model)
    _materialize_link_pair_poses(model)


def recorded_coord_policy(candidates, graph=None, coord_id=None, component=None, **kwargs):
    """The coordinate the DSL recorded for this pose component.

    The writer runs the authored choice and minutes it as PROV under the deterministic
    activity name `{coordinate}-{component}-selection`; this is the read half of that pact.
    """
    activity = URIRef(f"{coord_id}-{component}-selection")
    for usage in graph.objects(activity, PROV.qualifiedUsage):
        chosen = graph.value(usage, PROV.entity)
        if chosen is not None:
            return chosen
    raise ConstraintViolation(
        "geometry",
        f"Pose coordinate '{coord_id}' records no {component} selection among: {candidates}",
    )


def _pose_frames(model, pose) -> tuple[URIRef, URIRef]:
    """A pose quantity's authored `(of, with-respect-to)` frames."""
    relation = (
        PoseModel(pose, model.graph)
        if URI_GEOM_TYPE_POSE in get_node_types(model.graph, pose)
        else PoseCoordModel(pose, model.graph, coord_policy=recorded_coord_policy).relation
    )
    return relation.of_id, relation.wrt_id


# The relation nodes a runtime-derived pose needs to read like an authored one: the role each is
# minted under, the type it carries, and which of the four endpoints it relates.
_DERIVED_POSE_RELATIONS = (
    ("pose", URI_GEOM_TYPE_POSE, "of_frame", "wrt_frame"),
    ("position", URI_GEOM_TYPE_POSITION, "of_origin", "wrt_origin"),
    ("orientation", URI_GEOM_TYPE_ORIENT, "of_frame", "wrt_frame"),
)
_DERIVED_POSE_COORDINATE_TYPES = (
    URI_GEOM_TYPE_POSE_COORD,
    URI_GEOM_TYPE_POSE_REF,
    URI_GEOM_TYPE_POSITION_COORD,
    URI_GEOM_TYPE_POSITION_REF,
    URI_GEOM_TYPE_ORIENT_COORD,
    URI_GEOM_TYPE_ORIENT_REF,
    URI_GEOM_TYPE_VECTOR_XYZ,
)


def _emit_derived_pose(model, node: URIRef, of_frame: URIRef, wrt_frame: URIRef) -> None:
    """Materialize one runtime-derived pose in comp-rob2b relation/coordinate form."""
    graph = model.graph
    parts = {"of_frame": of_frame, "wrt_frame": wrt_frame}
    for role, frame in (("of_origin", of_frame), ("wrt_origin", wrt_frame)):
        origin = graph.value(frame, URI_GEOM_PRED_ORIGIN)
        if not isinstance(origin, URIRef):
            origin = model.component_node(frame, "origin")
            graph.add((frame, URI_GEOM_PRED_ORIGIN, origin))
        graph.add((frame, RDF.type, URI_GEOM_TYPE_FRAME))
        graph.add((origin, RDF.type, URI_GEOM_TYPE_POINT))
        parts[role] = origin

    relations = {
        role: model.component_node(node, f"{role}-rel") for role, *_ in _DERIVED_POSE_RELATIONS
    }
    for role, relation_type, of_role, wrt_role in _DERIVED_POSE_RELATIONS:
        relation = relations[role]
        graph.add((relation, RDF.type, relation_type))
        graph.add((relation, RDF.type, QUDT_SCHEMA.Quantity))
        graph.add((relation, URI_GEOM_PRED_OF, parts[of_role]))
        graph.add((relation, URI_GEOM_PRED_WRT, parts[wrt_role]))
    graph.add((relations["position"], QUDT_SCHEMA.hasQuantityKind, URI_QUDT_QK_LENGTH))
    for reference_type in (URI_GEOM_TYPE_POSITION_REF, URI_GEOM_TYPE_ORIENT_REF):
        graph.add((relations["pose"], RDF.type, reference_type))
    graph.add((relations["pose"], URI_GEOM_PRED_OF_POSITION, relations["position"]))
    graph.add((relations["pose"], URI_GEOM_PRED_OF_ORIENT, relations["orientation"]))

    graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    for coordinate_type in _DERIVED_POSE_COORDINATE_TYPES:
        graph.add((node, RDF.type, coordinate_type))
    graph.add((node, URI_GEOM_PRED_OF_POSE, relations["pose"]))
    graph.add((node, URI_GEOM_PRED_OF_POSITION, relations["position"]))
    graph.add((node, URI_GEOM_PRED_OF_ORIENT, relations["orientation"]))
    graph.add((node, URI_GEOM_PRED_SEEN_BY, wrt_frame))
    graph.add((node, QUDT_SCHEMA.unit, URI_QUDT_UNIT_M))
    graph.add((node, QUDT_SCHEMA.unit, URI_QUDT_UNIT_RAD))


def _pose_edges(model) -> dict:
    """Frame adjacency over every authored pose, traversable in both directions."""
    edges = collections.defaultdict(list)
    for pose in model.graph.subjects(RDF.type, GEOM_REL.Pose):
        try:
            of_frame, wrt_frame = _pose_frames(model, pose)
        except ValueError:
            # A pose with no frame endpoints joins no path; it is stated in coordinates already.
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


def _compose(model, owner: URIRef, suffix: str, in1: URIRef, in2: URIRef, out: URIRef) -> None:
    operation = model.derived_node(owner, suffix)
    model.graph.add((operation, RDF.type, GEOM_OP.ComposePose))
    model.graph.add((operation, GEOM_OP.in1, in1))
    model.graph.add((operation, GEOM_OP.in2, in2))
    model.graph.add((operation, GEOM_OP.composite, out))


def _invert(model, owner: URIRef, suffix: str, pose: URIRef, out: URIRef) -> None:
    operation = model.derived_node(owner, suffix)
    model.graph.add((operation, RDF.type, GEOM_OP.InvertPose))
    model.graph.add((operation, GEOM_OP.pose, pose))
    model.graph.add((operation, GEOM_OP.out, out))


def _compose_path(model, owner: URIRef, path, start_wrt: URIRef):
    """Emit the invert/compose operations along `path`; return the composed pose, or None."""
    current = None
    current_wrt = start_wrt
    for index, (pose, inverted) in enumerate(path):
        pose_of, pose_wrt = _pose_frames(model, pose)
        step = pose
        step_of, step_wrt = pose_of, pose_wrt
        if inverted:
            step = model.derived_node(owner, f"path-{index}-inverse")
            _emit_derived_pose(model, step, pose_wrt, pose_of)
            _invert(model, owner, f"path-{index}-invert", pose, step)
            step_of, step_wrt = pose_wrt, pose_of
        if current is None:
            current = step
            current_wrt = step_wrt
            continue
        composite = model.derived_node(owner, f"path-{index}-pose")
        _emit_derived_pose(model, composite, step_of, current_wrt)
        _compose(model, owner, f"path-{index}-compose", current, step, composite)
        current = composite

    return current


def _recorded_distance_operand(graph, distance, role):
    """The pose coordinate the DSL recorded for one distance endpoint, or None if unrecorded."""
    usage = graph.value(URIRef(f"{distance}-{role}-selection"), PROV.qualifiedUsage)
    return graph.value(usage, PROV.entity) if usage is not None else None


def _materialize_linear_distance_operations(model) -> None:
    """Expand authored linear-distance relations into codegen operations.

    The DSL graph states only the two pose endpoints; the inverse/composition path between their
    reference frames is derivable from the complete RDF graph.

    The operations hang off the constraint's distance *coordinate*, not the relation it samples:
    motions measuring the same two poses share one relation, and each still computes its own
    value from its own operands.
    """
    graph = model.graph
    edges = _pose_edges(model)
    for distance in list(graph.subjects(RDF.type, GEOM_COORD.DistanceReference)):
        if next(graph.subjects(CSTR.quantity, distance), None) is None:
            continue
        relation = graph.value(distance, GEOM_COORD["of"])
        if relation is None:
            raise ConstraintViolation(
                "geometry", f"Distance coordinate {distance} states no linear distance."
            )
        # `between-entities` names frame-origin Points; which pose coordinate each endpoint
        # reads is the selection the DSL recorded per role (see `recorded_coord_policy`).
        start = _recorded_distance_operand(graph, distance, "start")
        end = _recorded_distance_operand(graph, distance, "end")
        if start is None or end is None:
            endpoints = list(dict.fromkeys(graph.objects(relation, GEOM_REL["between-entities"])))
            if len(endpoints) != 2:
                raise ConstraintViolation(
                    "geometry", f"Linear distance {relation} needs exactly two pose endpoints."
                )
            start, end = endpoints
        start_of, start_wrt = _pose_frames(model, start)
        end_of, end_wrt = _pose_frames(model, end)

        path = ()
        if start_wrt != end_wrt:
            path = _pose_path(edges, start_wrt, end_wrt)
            if not path:
                raise ConstraintViolation(
                    "geometry",
                    f"Linear distance {distance} has no pose path from {start_wrt} to {end_wrt}.",
                )
        current = _compose_path(model, distance, path, start_wrt)

        end_in_start_reference = end
        if current is not None:
            end_in_start_reference = model.derived_node(distance, "end-in-start-reference")
            _emit_derived_pose(model, end_in_start_reference, end_of, start_wrt)
            _compose(
                model, distance, "compose-reference-path", current, end, end_in_start_reference
            )

        inverse_start = model.derived_node(distance, "inverse-start")
        _emit_derived_pose(model, inverse_start, start_wrt, start_of)
        _invert(model, distance, "invert-start", start, inverse_start)

        relative_pose = model.derived_node(distance, "relative-pose")
        _emit_derived_pose(model, relative_pose, end_of, start_of)
        _compose(
            model,
            distance,
            "compose-relative-pose",
            inverse_start,
            end_in_start_reference,
            relative_pose,
        )

        magnitude = model.derived_node(distance, "magnitude")
        graph.add((magnitude, RDF.type, GEOM_OP.PoseToLinearDistance))
        graph.add((magnitude, GEOM_OP.pose, relative_pose))
        graph.add((magnitude, GEOM_OP.distance, distance))


def _materialize_pose_reference_transforms(model) -> None:
    """Re-express a full-pose equality reference into the constrained pose's frame.

    A full-pose EqualityConstraint states its reference in whatever frame it was authored; when
    that differs from the constrained quantity's `with-respect-to` frame the comparison first
    needs the reference re-expressed.
    """
    graph = model.graph
    edges = _pose_edges(model)
    for constraint in list(graph.subjects(RDF.type, CSTR.EqualityConstraint)):
        quantity = graph.value(constraint, CSTR.quantity)
        reference = graph.value(constraint, CSTR["reference-value"])
        if quantity is None or reference is None:
            continue
        if GEOM_REL.Pose not in get_node_types(graph, quantity):
            continue
        if GEOM_REL.Pose not in get_node_types(graph, reference):
            continue
        try:
            target_of, target_wrt = _pose_frames(model, quantity)
            source_of, source_wrt = _pose_frames(model, reference)
        except ValueError:
            # A coordinate-authored goal pose carries no of/with-respect-to frames; it is already
            # stated in the constrained pose's frame.
            continue
        if source_wrt == target_wrt:
            continue
        if source_of != target_of:
            raise ConstraintViolation(
                "geometry",
                f"Equality constraint {constraint} compares a pose of {target_of} "
                f"to a reference of {source_of}.",
            )

        path = _pose_path(edges, target_wrt, source_wrt)
        if not path:
            raise ConstraintViolation(
                "geometry",
                f"Equality constraint {constraint} has no pose path from "
                f"{target_wrt} to {source_wrt}.",
            )
        current = _compose_path(model, constraint, path, target_wrt)

        reference_in_target = model.derived_node(constraint, "reference-in-target")
        _emit_derived_pose(model, reference_in_target, source_of, target_wrt)
        _compose(model, constraint, "compose-reference", current, reference, reference_in_target)

        graph.remove((constraint, CSTR["reference-value"], reference))
        graph.add((constraint, CSTR["reference-value"], reference_in_target))


def _shared_reference_pair(frames, node, of_frame, wrt_frame):
    """Two other poses `(of_frame, C)` and `(wrt_frame, C)` sharing one reference frame C."""
    through_of = {
        reference: pose
        for pose, (pose_of, reference) in frames.items()
        if pose != node and pose_of == of_frame and reference not in (of_frame, wrt_frame)
    }
    for pose, (pose_of, reference) in frames.items():
        if pose != node and pose_of == wrt_frame and reference in through_of:
            return through_of[reference], pose

    return None


def _materialize_link_pair_poses(model) -> None:
    """Compose a pose stated between two links out of the two poses that share a reference frame.

    Forward kinematics publishes a link's pose only with respect to the chain root, so a pose of
    one link with respect to another has no producer. When both links are also stated in one
    common frame C, the link-pair pose is their composition:
    ``X_wrt_of = Inverse(X_C_wrt) * X_C_of``. With no such pair the pose stays unwritten, which
    the dataflow check reports -- and authoring the two common-frame poses is the fix.
    """
    graph = model.graph
    frames = {}
    for node in graph.subjects(RDF.type, URI_GEOM_TYPE_POSE_COORD):
        scope = model.context_scope(node)
        if scope is None or scope.section != "world":
            continue
        try:
            frames[node] = _pose_frames(model, node)
        except ValueError:
            # A pose stated in coordinates alone names no frames to triangulate between.
            continue

    for node, (of_frame, wrt_frame) in frames.items():
        if next(graph.subjects(GEOM_OP.composite, node), None) is not None:
            continue
        pair = _shared_reference_pair(frames, node, of_frame, wrt_frame)
        if pair is None:
            continue
        of_in_shared, wrt_in_shared = pair
        _, shared_frame = frames[wrt_in_shared]

        inverted = model.derived_node(node, "wrt-inverse")
        _emit_derived_pose(model, inverted, shared_frame, wrt_frame)
        _invert(model, node, "invert-wrt", wrt_in_shared, inverted)
        _compose(model, node, "compose-link-pair", inverted, of_in_shared, node)


# Every operator answers three questions, and each answers all three: what closure one of its
# calls renders to (`closure_step`), what data its inputs read (`operand_inputs`), and what calls
# produce a given output (`scheduler_step`). An operator with nothing to say returns None or an
# empty step rather than omitting the method, so `Schedule` never probes for one.


def _parse_argument(graph, closure_id, argument, to_id):
    """Resolve a closure argument (input/output/parameter) to id(s); a lone one collapses to a
    scalar. An authored bound is named, never copied: the closure carries the quantity's id and
    the call site reads it off the blackboard, so the model's number has one source of truth.
    """
    entry = list(graph[closure_id:argument])
    if len(entry) == 0:
        return None
    unique = list(dict.fromkeys(to_id(node) for node in entry))

    return unique[0] if len(unique) == 1 else unique


def _fill_closure_args(graph, closure, to_id, op, closure_id, input_subject) -> None:
    """Fill a closure's input/output/parameter slots; inputs may hang off another subject.
    Insertion order is the emitted JSON key order.
    """
    for input_ in op.input:
        closure[to_id(input_)] = _parse_argument(graph, input_subject, input_, to_id)
    for output in op.output:
        closure[to_id(output)] = _parse_argument(graph, closure_id, output, to_id)
    for param in op.parameters:
        closure[to_id(param)] = _parse_argument(graph, closure_id, param, to_id)


def _operator_inputs(graph, operator_id, inputs) -> set:
    """Data-structure nodes feeding an operator call's inputs, plus any path parameters."""
    data_structures = set()
    for in_ in inputs:
        for data_in in graph.objects(operator_id, in_):
            data_structures.add(data_in)
            # A path produces nothing, so the output-to-input walk never reaches its parameters
            # as a producer's outputs; they are contributed here instead.
            if GEOM_PATH.Path in get_node_types(graph, data_in):
                data_structures.update(
                    obj for pred, obj in graph.predicate_objects(data_in) if pred != RDF["type"]
                )

    return data_structures


def _constraint_types(graph, node) -> set:
    """Types of the constraint a handler call points at; empty when it points at nothing."""
    constraint_id = graph.value(node, CSTR_HDL["constraint"])
    return get_node_types(graph, constraint_id) if constraint_id is not None else set()


def _constraint_inputs(graph, operator_id, inputs) -> set:
    """Data-structure nodes reached through a handler call's constraint."""
    return {
        data_in
        for in_ in inputs
        for data_in in graph.objects(operator_id, CSTR_HDL["constraint"] / in_)
    }


class Operator:
    """A schedulable RDF computation: it maps a call's graph inputs, outputs and parameters to a
    closure, and participates in the output-to-input scheduling walk.
    """

    schedulable = True

    def __init__(self, type_, input, output, parameters=()):
        self.type_ = type_
        self.input = list(input)
        self.output = list(output)
        self.parameters = list(parameters)

    def closure_step(self, model, node):
        """The closure this operator's call at `node` renders to."""
        closure = {"id": model.id(node), "type": model.id(self.type_)}
        _fill_closure_args(model.graph, closure, model.id, self, node, node)
        return closure

    def operand_inputs(self, model, node):
        """Data-structure nodes feeding this call's inputs."""
        return _operator_inputs(model.graph, node, self.input)

    def scheduler_step(self, model, data_out):
        """Input data structures and schedulable calls producing `data_out`."""
        graph = model.graph
        data_structures = set()
        schedule = []
        for out in self.output:
            # sorted(): unordered here, and append order becomes the emitted schedule order.
            for call in sorted(graph[:out:data_out]):
                if self.type_ not in get_node_types(graph, call):
                    continue
                # A call with no input cannot be scheduled.
                inputs = _operator_inputs(graph, call, self.input)
                if inputs:
                    data_structures |= inputs
                    schedule.append(call)

        return {"data_structures": data_structures, "schedule": schedule}


class Specification(Operator):
    """Not a computation: no closure and no schedule entry, but it propagates outputs to inputs so
    further computations are found.
    """

    schedulable = False

    def closure_step(self, model, node):
        """None: a specification states a fact, so there is nothing to call."""
        return

    def scheduler_step(self, model, data_out):
        """Input data structures for this specification (never schedulable)."""
        graph = model.graph
        data_structures = set()
        for out in self.output:
            for call in graph.subjects(out, data_out):
                for in_ in self.input:
                    data_structures.update(graph.objects(call, in_))
        return {"data_structures": data_structures, "schedule": []}


def _authored_error_normalization(model, constraint_id) -> dict | None:
    """The interval a controller states its error is read into, for the controller driving this
    constraint. Authored beats derived: a model that says which turn is not second-guessed.
    """
    graph = model.graph
    for controller in graph.subjects(CSTR_HDL["constraint"], constraint_id):
        interval = graph.value(controller, ALGO_EXT["normalization"])
        if interval is None:
            continue
        bounds = {}
        for edge in ("lower", "upper"):
            bound = graph.value(interval, ALGO_EXT[f"{edge}-bound"])
            value = graph.value(bound, QUDT_SCHEMA.value) if bound is not None else None
            if value is None:
                return None
            bounds[edge] = float(value.toPython())
        return bounds
    return None


@reader
def continuous_joint_leaves(model) -> set[str]:
    """Leaf names of revolute joints with no authored position limit: continuous joints, whose
    position error lives on the circle. Read from absence -- scene-dsl emits a kc-ext:JointLimit
    per authored bound and nothing for a missing one.
    """
    graph = model.graph
    revolute = set(graph.subjects(RDF.type, URI_KC_TYPE_REVOLUTE_JOINT)) | set(
        graph.subjects(RDF.type, URI_KC_TYPE_REVOLUTE_JOINT_ORIENTED_AXIS)
    )
    position_limited = {
        graph.value(limit, URI_KC_EXT_PRED_OF_JOINT)
        for limit in graph.subjects(RDF.type, URI_KC_EXT_TYPE_JOINT_LIMIT)
        if URI_KC_STAT_JNT_POSITION in get_node_types(graph, limit)
    }

    return {local_name(joint) for joint in revolute - position_limited}


class ErrorEvaluator:
    """Constraint-handler operator emitting a constraint error signal, dispatching on the
    constraint type (equality/greater/less/bilateral/outside).
    """

    schedulable = False

    def __init__(self):
        self.type_ = CSTR_HDL["ErrorEvaluator"]
        self.cstr_op = [
            Operator(
                CSTR["EqualityConstraint"],
                [CSTR["quantity"], CSTR["reference-value"]],
                [CSTR_HDL["error"]],
            ),
            Operator(
                CSTR["GreaterThanConstraint"],
                [CSTR["quantity"], CSTR["threshold"]],
                [CSTR_HDL["error"]],
            ),
            Operator(
                CSTR["LessThanConstraint"],
                [CSTR["quantity"], CSTR["threshold"]],
                [CSTR_HDL["error"]],
            ),
            Operator(
                CSTR["BilateralConstraint"],
                [CSTR["quantity"], CSTR["lower-threshold"], CSTR["upper-threshold"]],
                [CSTR_HDL["error"]],
            ),
            Operator(
                CSTR_EXT["OutsideConstraint"],
                [CSTR["quantity"], CSTR["lower-threshold"], CSTR["upper-threshold"]],
                [CSTR_HDL["error"]],
            ),
        ]

    def closure_step(self, model, node):
        """The error-evaluator closure, dispatching on the constraint type."""
        from motion_spec.rdf_parser.quantities import goal_status_act

        graph = model.graph
        constraint_id = graph.value(node, CSTR_HDL["constraint"])
        # A goal status is runtime-written, never computed: its equality is read directly by
        # the monitor's goal-status term, so the evaluator yields no closure (elapsed precedent).
        if goal_status_act(model, graph.value(constraint_id, CSTR["quantity"])) is not None:
            return None
        for operator in self.cstr_op:
            if operator.type_ not in get_node_types(graph, constraint_id):
                continue
            closure = {
                "id": model.id(node),
                "type": "ErrorEvaluator",
                "constraint": model.id(operator.type_),
                # Which constraint this evaluates, not just what kind: a monitor watching an
                # aggregate reaches its members' errors through this.
                "constraint_id": model.id(constraint_id),
                "constraint_uri": str(constraint_id),
            }
            # An equality error on a continuous joint wraps to the shortest arc, unless the
            # controller driving it states the interval its error is read into.
            if operator.type_ == CSTR["EqualityConstraint"]:
                authored = _authored_error_normalization(model, constraint_id)
                if authored is not None:
                    closure["error_normalization"] = authored
                else:
                    quantity = graph.value(constraint_id, CSTR["quantity"])
                    joint = (
                        graph.value(quantity, KC_STAT["of-joint"]) if quantity is not None else None
                    )
                    if joint is not None and local_name(joint) in continuous_joint_leaves(model):
                        closure["angular_wrap"] = True
            _fill_closure_args(graph, closure, model.id, operator, node, constraint_id)

            return closure  # first matching constraint type wins

        # An authored bound that reaches no evaluator is never compared anywhere in the program,
        # so say which constraint states it rather than emit a program that ignores it.
        stated = sorted(
            local_name(type_)
            for type_ in get_node_types(graph, constraint_id)
            if type_ != CSTR["Constraint"] and local_name(type_).endswith("Constraint")
        )
        raise ConstraintViolation(
            "constraint-handler",
            f"Constraint '{model.id(constraint_id)}' relates its quantity as "
            f"{', '.join(stated) or 'nothing an evaluator reads'}; no evaluator compiles that, "
            "so its bound would never be compared.",
        )

    def operand_inputs(self, model, node):
        """Data-structure nodes feeding the matching constraint's inputs."""
        constraint_types = _constraint_types(model.graph, node)
        return {
            data_in
            for op in self.cstr_op
            if op.type_ in constraint_types
            for data_in in _constraint_inputs(model.graph, node, op.input)
        }

    def scheduler_step(self, model, data_out):
        """Input data structures and schedulable calls producing the error `data_out`."""
        graph = model.graph
        data_structures = set()
        schedule = []
        for op in self.cstr_op:
            for out in op.output:
                # sorted(): see Schedule.of -- append order becomes schedule order.
                for call in sorted(graph.subjects(out, data_out)):
                    if op.type_ not in _constraint_types(graph, call):
                        continue
                    # A call with no input cannot be scheduled.
                    inputs = _constraint_inputs(graph, call, op.input)
                    if inputs:
                        data_structures |= inputs
                        schedule.append(call)

        return {"data_structures": data_structures, "schedule": schedule}


class AssignmentEvaluator:
    """Constraint operator that assigns a reference value to a quantity (no error output)."""

    schedulable = True

    def __init__(self):
        self.type_ = CSTR_HDL["AssignmentEvaluator"]
        self.cstr_op = Operator(
            CSTR["EqualityConstraint"], [CSTR["quantity"], CSTR["reference-value"]], []
        )

    def closure_step(self, model, node):
        """The assignment-evaluator closure (equality constraint only)."""
        graph = model.graph
        constraint_id = graph.value(node, CSTR_HDL["constraint"])
        if self.cstr_op.type_ not in get_node_types(graph, constraint_id):
            return None
        closure = {
            "id": model.id(node),
            "type": "AssignmentEvaluator",
            "constraint": model.id(self.cstr_op.type_),
        }
        _fill_closure_args(graph, closure, model.id, self.cstr_op, node, constraint_id)

        return closure

    def operand_inputs(self, model, node):
        """Data-structure nodes feeding the assignment's inputs."""
        if self.cstr_op.type_ not in _constraint_types(model.graph, node):
            return set()
        return _constraint_inputs(model.graph, node, self.cstr_op.input)

    def scheduler_step(self, model, data_out):
        """Nothing: an assignment writes the quantity it is given and declares no output."""
        return {"data_structures": (), "schedule": ()}


def _output_predicates(op) -> set:
    """Output predicates an operator's scheduler queries; empty when it can never match, so its
    scheduler step can be skipped.
    """
    if isinstance(op, ErrorEvaluator):
        return {predicate for sub in op.cstr_op for predicate in sub.output}
    if isinstance(op, AssignmentEvaluator):
        return set()
    return set(op.output)


# A path is geometry: data with no output, so it is never found by the output-to-input walk and
# yields no closure of its own. The evaluator that traverses it is the computation, and folds the
# path's geometry into its call.
_OPS_PATH = [
    Specification(GEOM_PATH["LinearPath"], [GEOM_PATH["start"], GEOM_PATH["goal"]], []),
    Specification(
        GEOM_PATH["Circle"],
        [GEOM_PATH["start"], GEOM_PATH["center"], GEOM_PATH["plane-normal"]],
        [],
    ),
    Specification(
        GEOM_PATH["Arc"],
        [GEOM_PATH["start"], GEOM_PATH["end"], GEOM_PATH["amplitude"], GEOM_PATH["plane-normal"]],
        [],
    ),
    Specification(
        GEOM_PATH["Helix"],
        [
            GEOM_PATH["start"],
            GEOM_PATH["center"],
            GEOM_PATH["axis"],
            GEOM_PATH["pitch"],
            GEOM_PATH["revolutions"],
        ],
        [],
    ),
    Specification(
        GEOM_PATH["Figure8"],
        [GEOM_PATH["anchor"], GEOM_PATH["radius"], GEOM_PATH["plane-normal"]],
        [],
        [GEOM_PATH["form"]],
    ),
]

OPS_GENERIC = [
    Operator(
        GEOM_OP["RotateDirectionDistalToProximalWithPose"],
        [GEOM_OP["pose"], GEOM_OP["from"]],
        [GEOM_OP["to"]],
    ),
    Operator(GEOM_OP["ComposePose"], [GEOM_OP["in1"], GEOM_OP["in2"]], [GEOM_OP["composite"]]),
    Operator(GEOM_OP["InvertPose"], [GEOM_OP["pose"]], [GEOM_OP["out"]]),
    Operator(
        GEOM_OP["RotateVelocityTwistToProximalWithPose"],
        [GEOM_OP["pose"], GEOM_OP["from"]],
        [GEOM_OP["to"]],
    ),
    Operator(
        GEOM_OP["PoseToAngleAroundAxis"], [GEOM_OP["pose"]], [GEOM_OP["angle"]], [GEOM_OP["axis"]]
    ),
    Operator(GEOM_OP["PoseToLinearDistance"], [GEOM_OP["pose"]], [GEOM_OP["distance"]]),
    Operator(GEOM_OP["PoseToDirection"], [GEOM_OP["pose"]], [GEOM_OP["direction"]]),
    Operator(
        GEOM_OP["PlanarAngleFromDirections"], [GEOM_OP["from-directions"]], [GEOM_OP["angle"]]
    ),
    Operator(GEOM_OP["InvertAngle"], [GEOM_OP["in"]], [GEOM_OP["out"]]),
    Operator(
        GEOM_OP_EXT["RotationVectorFromDirections"],
        [GEOM_OP["in1"], GEOM_OP["in2"]],
        [GEOM_OP["out"]],
    ),
    Operator(
        GEOM_OP_EXT["PointPlaneToLinearDistance"],
        [GEOM_OP["in1"], GEOM_OP["in2"], GEOM_OP["direction"]],
        [GEOM_OP["distance"], GEOM_OP_EXT["gradient"]],
    ),
    Operator(
        GEOM_OP_EXT["PointLineToLinearDistance"],
        [GEOM_OP["in1"], GEOM_OP["in2"], GEOM_OP["direction"]],
        [GEOM_OP["distance"], GEOM_OP_EXT["gradient"]],
    ),
    Operator(
        GEOM_OP_EXT["PointBodyLineToLinearDistance"],
        [GEOM_OP["in1"], GEOM_OP["direction"]],
        [GEOM_OP["distance"], GEOM_OP_EXT["gradient"], GEOM_OP_EXT["gradient-moment"]],
    ),
    Operator(
        GEOM_OP_EXT["PointOnLineProjection"],
        [GEOM_OP["in1"], GEOM_OP["in2"], GEOM_OP["direction"]],
        [GEOM_OP["distance"], GEOM_OP_EXT["gradient"]],
    ),
    Operator(GEOM_OP_EXT["PoseDiffEvaluator"], [GEOM_OP["in1"], GEOM_OP["in2"]], [GEOM_OP["out"]]),
    Operator(
        GEOM_OP_EXT["LineLineToLinearDistance"],
        [GEOM_OP["in1"], GEOM_OP["in2"], GEOM_OP["pose"]],
        [GEOM_OP["distance"], GEOM_OP_EXT["gradient"]],
    ),
    Operator(
        GEOM_OP_EXT["LineOnLineProjection"],
        [GEOM_OP["in1"], GEOM_OP["in2"], GEOM_OP["pose"]],
        [GEOM_OP["distance"], GEOM_OP_EXT["gradient"]],
    ),
    Operator(
        GEOM_OP_EXT["DirectionPlaneToAngularDistance"],
        [GEOM_OP["in1"], GEOM_OP["in2"]],
        [GEOM_OP["angle"], GEOM_OP_EXT["gradient"]],
    ),
    Operator(
        GEOM_OP_EXT["AngleGradientFromDirections"],
        [GEOM_OP["in1"], GEOM_OP["in2"]],
        [GEOM_OP_EXT["gradient"]],
    ),
    Operator(RBDYN_OP["AddWrench"], [RBDYN_OP["in1"], RBDYN_OP["in2"]], [RBDYN_OP["out"]]),
    Operator(ALGO_EXT.Addition, [ALGO_EXT["in"]], [ALGO_EXT.out]),
    Operator(ALGO_EXT.Subtraction, [ALGO_EXT["minuend"], ALGO_EXT["subtrahend"]], [ALGO_EXT.out]),
    Operator(ALGO_EXT.Multiplication, [ALGO_EXT["in"]], [ALGO_EXT.out]),
    Operator(ALGO_EXT.Division, [ALGO_EXT["dividend"], ALGO_EXT["divisor"]], [ALGO_EXT.out]),
    Operator(
        RBDYN_OP["RotateWrenchToDistalWithPose"],
        [RBDYN_OP["pose"], RBDYN_OP["from"]],
        [RBDYN_OP["to"]],
    ),
    Operator(
        RBDYN_OP["RotateWrenchToProximalWithPose"],
        [RBDYN_OP["pose"], RBDYN_OP["from"]],
        [RBDYN_OP["to"]],
    ),
    Operator(
        RBDYN_OP["TransformWrenchToProximal"],
        [RBDYN_OP["pose"], RBDYN_OP["from"]],
        [RBDYN_OP["to"]],
    ),
    Operator(
        RBDYN_OP["WrenchFromPositionDirectionAndMagnitude"],
        [RBDYN_OP["magnitude"], RBDYN_OP["direction"], RBDYN_OP["position"]],
        [RBDYN_OP["wrench"]],
    ),
    Operator(
        RBDYN_OP_EXT["WrenchFromDirectionAndMoment"],
        [RBDYN_OP_EXT["moment"], RBDYN_OP["direction"]],
        [RBDYN_OP["wrench"]],
    ),
    Specification(MAP["View"], [MAP["superobject"]], [MAP["subobject"]]),
    *_OPS_PATH,
    Operator(
        GEOM_OP_EXT.PathProjection,
        [GEOM_OP_EXT.path, GEOM_OP["pose"]],
        [GEOM_OP_EXT["path-parameter"]],
    ),
    Operator(
        GEOM_OP_EXT.PathTangentFrame,
        [GEOM_OP_EXT.path, GEOM_OP_EXT["path-parameter"]],
        [GEOM_OP_EXT.tangent, GEOM_OP_EXT["normal-a"], GEOM_OP_EXT["normal-b"]],
    ),
    Operator(
        GEOM_OP_EXT.TwistToLinearVelocityAlong,
        [GEOM_OP["in"], GEOM_OP["direction"]],
        [GEOM_OP_EXT["along-speed"]],
    ),
    Operator(GEOM_OP_EXT.VectorNorm, [GEOM_OP["in"], GEOM_OP["direction"]], [GEOM_OP_EXT["norm"]]),
    Operator(
        GEOM_OP_EXT.PathEvaluator,
        [GEOM_OP_EXT.path, GEOM_OP_EXT["path-parameter"]],
        [GEOM_OP["out"]],
    ),
    Operator(
        ALGO_EXT["VelocityProfile"],
        [
            ALGO_EXT["target"],
            ALGO_EXT["in"],
            ALGO_EXT["maximum-velocity"],
            ALGO_EXT["maximum-acceleration"],
            ALGO_EXT["maximum-jerk"],
            GEOM_OP_EXT.path,
            GEOM_OP_EXT["path-parameter"],
        ],
        [ALGO_EXT["out"]],
        [ALGO_EXT["shape"]],
    ),
    Operator(
        ALGO_EXT["Admittance"],
        [ALGO_EXT["in"]],
        [ALGO_EXT["out"]],
        [
            ALGO_EXT["mass"],
            ALGO_EXT["damping"],
            ALGO_EXT["stiffness"],
            ALGO_EXT["maximum-velocity"],
            ALGO_EXT["maximum-absolute-value"],
            CSTR["lower-threshold"],
            CSTR["upper-threshold"],
        ],
    ),
]

OPS_HANDLER = [AssignmentEvaluator(), ErrorEvaluator()]

OPS_SOLVER = [
    Specification(SLV["CartesianForceSpecification"], [SLV["force"]], []),
    Specification(SLV["JointForceSpecification"], [SLV["force"]], []),
    Specification(SLV["ForceDistributionSolver"], [SLV["force"]], []),
]

# Closure kinds no operator fully registers: "Controller" and the pose-equality
# "PoseDiffEvaluator" closures are hand-built by
# constraint_handler.augment_closures; AssignmentEvaluator's write is one of its *inputs*
# (`quantity`, the value it assigns into), not its (empty) declared output; PathEvaluator's
# `setpoint` is folded on by the PathEvaluator closure hook, not declared as an operator output.
_EXTRA_CLOSURE_OUTPUTS = {
    "AssignmentEvaluator": ("quantity",),
    "PathEvaluator": ("setpoint",),
    "Controller": ("control_signal",),
    "PoseDiffEvaluator": ("out",),
}


def _closure_output_fields() -> dict[str, tuple[str, ...]]:
    """Per closure type, the field names it writes: derived from the operator registries' own
    declared outputs (a Specification writes no closure at all and is skipped), plus the kinds
    above that no operator fully accounts for.
    """
    table: dict[str, set[str]] = {}
    for op in [*OPS_GENERIC, *OPS_SOLVER, *OPS_HANDLER]:
        if isinstance(op, Specification):
            continue
        key = identifier(local_name(op.type_))
        table.setdefault(key, set()).update(
            identifier(local_name(pred)) for pred in _output_predicates(op)
        )
    for key, extra in _EXTRA_CLOSURE_OUTPUTS.items():
        table.setdefault(key, set()).update(extra)

    return {key: tuple(sorted(fields)) for key, fields in table.items()}


_CLOSURE_OUTPUT_FIELDS = _closure_output_fields()


def closure_output_ids(closure: dict) -> set[str]:
    """Data ids written by a generated closure call."""
    outputs = {
        value
        for field in _CLOSURE_OUTPUT_FIELDS.get(closure.get("type"), ())
        if isinstance((value := closure.get(field)), str)
    }
    outputs.update(
        sample["id"]
        for sample in closure.get("internal_state_samples", ())
        if isinstance(sample, dict) and isinstance(sample.get("id"), str)
    )
    if closure.get("assign_goal") and isinstance(closure.get("goal"), str):
        outputs.add(closure["goal"])

    return outputs


def _path_fields(model, path_node) -> dict:
    """The path's geometry, as fields of the evaluator call that traverses it: the shape decides
    the maths, so the closure takes the path's type and carries its parameters directly.
    """
    path_types = get_node_types(model.graph, path_node)
    spec = next((s for s in _OPS_PATH if s.type_ in path_types), None)
    if spec is None:
        return {}
    fields = {"type": model.id(spec.type_)}
    for input_ in spec.input:
        fields[model.id(input_)] = _parse_argument(model.graph, path_node, input_, model.id)
    for param in spec.parameters:
        fields[model.id(param)] = _parse_argument(model.graph, path_node, param, model.id)

    return fields


def _fold_path(model, node, closure) -> None:
    """The geometry decides the maths; each caller samples the same curve."""
    fields = _path_fields(model, model.graph.value(node, GEOM_OP_EXT.path))
    closure["shape"] = fields.pop("type")
    closure.update(fields)


def _fold_path_evaluator(model, node, closure) -> None:
    """A path evaluator writes a setpoint as well as sampling the curve."""
    reference = model.graph.value(node, GEOM_OP.out)
    if reference is not None:
        closure["setpoint"] = model.id(reference)
    _fold_path(model, node, closure)


class _FilterBinding(NamedTuple):
    """What an admittance or velocity-profile filter's output is bound to."""

    constraint: object
    controller: object


def _filter_controller(model, node, closure) -> _FilterBinding:
    """The constraint a filter's output drives, and the controller that drives it."""
    graph = model.graph
    reference = graph.value(node, ALGO_EXT.out)
    constraint = next(graph.subjects(CSTR["reference-value"], reference), None)
    if constraint is None:
        raise ConstraintViolation(
            "computation",
            f"{closure['type']} '{closure['id']}' output is not bound to a constraint.",
        )
    # Evaluators carry cstr-hdl:constraint too, and can win this lookup; filter state belongs to
    # the controller.
    controller = next(
        (
            candidate
            for candidate in graph.subjects(CSTR_HDL.constraint, constraint)
            if CSTR_HDL.ConstraintEvaluator not in get_node_types(graph, candidate)
        ),
        None,
    )
    if controller is None:
        raise ConstraintViolation(
            "computation", f"{closure['type']} '{closure['id']}' constraint has no controller."
        )

    return _FilterBinding(constraint, controller)


def _fold_admittance(model, node, closure) -> None:
    """An admittance filter keeps state, so it names the controller that owns it."""
    _constraint, controller = _filter_controller(model, node, closure)
    closure["controller"] = model.id(controller)


def _fold_velocity_profile(model, node, closure) -> None:
    """A physical profile keeps state and starts from the constraint's measured quantity."""
    constraint, controller = _filter_controller(model, node, closure)
    closure["controller"] = model.id(controller)
    closure["measured"] = model.id(model.graph.value(constraint, CSTR.quantity))
    closure["profile_target"] = closure.pop("target")
    if closure.get("path") is not None:
        closure["profile_shape"] = closure["shape"]
        _fold_path(model, node, closure)


# Per operator type, the post-processing its closure needs beyond its declared operands.
_CLOSURE_HOOKS = {
    GEOM_OP_EXT.PathProjection: _fold_path,
    GEOM_OP_EXT.PathTangentFrame: _fold_path,
    GEOM_OP_EXT.PathEvaluator: _fold_path_evaluator,
    ALGO_EXT.VelocityProfile: _fold_velocity_profile,
    ALGO_EXT.Admittance: _fold_admittance,
}


def build_closures(model, operators) -> dict:
    """The closure each call of these operators renders to.

    Parameters:
        operators: one of the operator tables, or a concatenation of them

    Returns:
        one closure dict per call, keyed by the call's generated id; an operator that yields no
        closure for a call (a specification, a goal-status evaluator the monitor reads directly)
        contributes nothing
    """
    closures = {}
    for operator in operators:
        for node in model.graph.subjects(RDF["type"], operator.type_):
            closure = operator.closure_step(model, node)
            if not closure:
                continue
            hook = _CLOSURE_HOOKS.get(operator.type_)
            if hook is not None:
                hook(model, node, closure)
            closures[model.id(node)] = closure

    return closures


class Schedule:
    """Dependency-ordered call ids, emitted at most once per instance.

    A new instance is a new scope: the `when` block and the active block each need one, because a
    step evaluated in both phases must be emitted in both.
    """

    def __init__(self, model):
        self._model = model
        self._emitted: set[str] = set()

    def claim(self, call_id: str) -> bool:
        """True the first time this scope is asked to emit a call id, and never again."""
        if call_id in self._emitted:
            return False
        self._emitted.add(call_id)
        return True

    def of(self, start_nodes, operators) -> list[str]:
        """The calls reachable backward from `start_nodes`, producers before consumers.

        Walks output-to-input from each start node, collecting every schedulable call that feeds
        it, then orders the result topologically. A call this scope has already emitted is not
        emitted again.

        Parameters:
            start_nodes: the graph nodes whose computation the block needs
            operators: the operator tables the walk may match against
        """
        model = self._model
        graph = model.graph
        pending = collections.deque()
        data_structures = set()
        sched = []
        nodes_by_call = {}

        for node in start_nodes:
            node_types = get_node_types(graph, node)
            for op in operators:
                if op.type_ not in node_types:
                    continue
                call = model.id(node)
                if op.schedulable and self.claim(call):
                    sched.append(call)
                    nodes_by_call[call] = node
                # sorted(): set order over rdflib nodes varies between processes, and traversal
                # order decides the emitted schedule order.
                for data_in in sorted(op.operand_inputs(model, node)):
                    pending.append(data_in)
                    data_structures.add(data_in)

        # data_in --(in)--> call --(out)--> data_out, traversed backward.
        by_predicates = [(_output_predicates(op), op) for op in operators]
        while pending:
            data_out = pending.pop()
            preds_into = set(graph.predicates(None, data_out))
            # Only ops whose output predicate points into data_out can match here.
            for out_set, op in by_predicates:
                if out_set.isdisjoint(preds_into):
                    continue
                step = op.scheduler_step(model, data_out)
                for call_node in step["schedule"]:
                    call = model.id(call_node)
                    if call and self.claim(call):
                        sched.append(call)
                        nodes_by_call[call] = call_node
                for data_in in sorted(step["data_structures"]):
                    if data_in not in data_structures:
                        pending.append(data_in)
                        data_structures.add(data_in)

            # An inline/declared Pose is no operator's output, so follow its per-axis views to
            # schedule the closures producing its components. sorted() for the reason above.
            view_subobjects = (
                graph.value(view, MAP["subobject"])
                for view in sorted(graph.subjects(MAP["superobject"], data_out))
                if graph.value(view, MAP["axis"]) is not None
            )
            for successor in itertools.chain(
                (node for node in view_subobjects if node is not None),
                sorted(graph.objects(data_out, CSTR["reference-value"])),
            ):
                if successor not in data_structures:
                    pending.append(successor)
                    data_structures.add(successor)

        sched.reverse()

        return self._ordered(sched, nodes_by_call, operators)

    def _ordered(self, sched, nodes_by_call, operators) -> list[str]:
        """Reorder scheduled calls so producers run before their consumers."""
        if len(sched) < 2:
            return sched
        model = self._model
        order = {call: index for index, call in enumerate(sched)}
        call_inputs: dict[str, set] = {}
        producer_of = {}

        for call in sched:
            node = nodes_by_call.get(call)
            if node is None:
                continue
            inputs, outputs = set(), set()
            node_types = get_node_types(model.graph, node)
            for op in operators:
                if op.type_ not in node_types:
                    continue
                inputs.update(op.operand_inputs(model, node))
                for out in getattr(op, "output", ()):
                    outputs.update(model.graph.objects(node, out))
            call_inputs[call] = inputs
            for output in outputs:
                producer_of.setdefault(output, call)

        deps = {
            call: {
                producer_of[data]
                for data in inputs
                if producer_of.get(data) is not None and producer_of[data] != call
            }
            for call, inputs in call_inputs.items()
        }
        result, visiting, done = [], set(), set()

        def visit(call):
            """Emit a call after every call it depends on."""
            if call in done or call in visiting:
                return
            visiting.add(call)
            for dep in sorted(deps.get(call, ()), key=lambda item: order.get(item, 0)):
                visit(dep)
            visiting.remove(call)
            done.add(call)
            result.append(call)

        for call in sched:
            visit(call)

        return result


class ClosureMaps(NamedTuple):
    """Per data id: the closure that writes it, and the ids that closure reads."""

    producer: dict
    operands: dict


def closure_maps(closures: dict) -> ClosureMaps:
    """Who writes each data id, and what that writer reads."""
    output_map: dict[str, str] = {}
    input_map: dict[str, set[str]] = {}
    for closure_id, closure in closures.items():
        outputs = closure_output_ids(closure)
        inputs = {
            value
            for key, value in closure.items()
            if key not in {"id", "type"} and isinstance(value, str) and value not in outputs
        }
        for output in outputs:
            output_map[output] = closure_id
            input_map[output] = inputs

    return ClosureMaps(output_map, input_map)


def closure_owner_map(model, closures: dict) -> dict[str, str]:
    """The motion whose context declares the quantities each closure reads.

    The backward schedule walk can reach another motion's closure, which would then run whenever
    that unrelated motion is active. Ownership is the motion segment of the quantities it
    references; a closure reading only shared context has no owner and stays available to all.
    """
    # Only a motion's own context names an owner. A shared context block sits at the same depth
    # in the IRI, so matching on the section alone would hand every motion-independent closure
    # the block's name and strand it: no motion answers to it.
    motions = {
        model.motion_suffix(node) for node in set(model.graph.objects(None, CSTR_HDL["motion"]))
    }
    owner_map: dict[str, str] = {}
    for node in set(model.graph.subjects()):
        if not isinstance(node, URIRef):
            continue
        closure_id = model.id(node)
        if closure_id not in closures:
            continue
        owners = set()
        for obj in model.graph.objects(node, None):
            if not isinstance(obj, URIRef):
                continue
            scope = model.context_scope(obj)
            if scope is not None and scope[1] == "spec":
                owner = get_valid_var_name(scope[0])
                if owner in motions:
                    owners.add(owner)
        if len(owners) == 1:
            owner_map[closure_id] = owners.pop()

    return owner_map


def data_reference_map(data_structures, closures: dict) -> dict[str, str]:
    """The id each data id takes its reference value from.

    Two sources say it: a record carrying `reference_value`, and an assignment evaluator binding
    a quantity to one. A snapshot walk follows these edges to find what it has to capture.
    """
    references: dict[str, str] = {}
    for item in data_structures:
        reference = getattr(item, "reference_value", None)
        reference_id = reference if isinstance(reference, str) else getattr(reference, "id", None)
        if reference_id:
            references[item.id] = reference_id
    for closure in closures.values():
        if closure.get("type") != "AssignmentEvaluator":
            continue
        quantity_id = closure.get("quantity")
        reference_id = closure.get("reference_value")
        if isinstance(quantity_id, str) and isinstance(reference_id, str):
            references[quantity_id] = reference_id

    return references


def path_projections_for_motion(schedule: list, closures: dict) -> list[dict]:
    """The path projections a motion runs, with the measurements that re-arm on entry.

    Returns:
        one entry per projection call, in schedule order, carrying its path parameter and the
        along-path speed measured beside it
    """

    def calls_of(type_):
        return [
            call
            for call in dict.fromkeys(schedule)
            if isinstance(closures.get(call), dict) and closures[call].get("type") == type_
        ]

    speeds = [closures[call]["along_speed"] for call in calls_of("TwistToLinearVelocityAlong")]

    return [
        {"id": call, "parameter": closures[call]["path_parameter"], "along_speed": speed}
        for call, speed in zip(calls_of("PathProjection"), speeds)
    ]


def _is_pose(item) -> bool:
    """True when a data structure is a Pose quantity, however its kind was stated."""
    if item is None:
        return False
    if getattr(item, "type", None) == "Pose":
        return True
    kind = getattr(item, "quantity_kind", None)
    kinds = kind if isinstance(kind, list) else [kind]
    return any(getattr(entry, "id", None) == "Pose" for entry in kinds)


def resolve_closure_operands(closures: dict, indexes, data_structures: list) -> None:
    """Fold in the operands a path closure can only resolve once poses exist.

    In place. A linear goal becomes per-axis components when the pose is declared, and stays a
    shared-signal reference otherwise; an arc's end is checked to be a pose, because the template
    renders its position and its orientation.

    Raises:
        ConstraintViolation: an arc path ends at something that is not a pose.
    """
    data_by_id = {item.id: item for item in data_structures if getattr(item, "id", None)}
    for closure in closures.values():
        shape = closure.get("shape")
        if shape == "LinearPath":
            goal = closure.get("goal")
            if not isinstance(goal, str):
                continue
            if goal in indexes.pose_components and closure.get("type") == "PathProjection":
                # The template builds the pose frame. Only the projection assigns: it is scheduled
                # before the frame and the evaluator, which read the same shared goal.
                closure["goal_components"] = indexes.pose_components[goal]
                closure["assign_goal"] = True
            else:
                # goal is a shared signal id (already on the closure as closure["goal"]).
                closure["assign_goal"] = False
        elif shape == "Arc":
            end = closure.get("end")
            if not isinstance(end, str) or not _is_pose(data_by_id.get(end)):
                raise ConstraintViolation("geometry", "Arc path end must be a Pose quantity.")
