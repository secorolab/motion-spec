# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What runs when.

In order: the readers for a handler and the evaluators, monitors and motion it binds; the
handlers themselves; the four steps a motion is built in -- which constraints belong to each
phase, the three schedules, the motion unit, and what is folded on once every motion exists; the
boolean terms a condition renders from; and the FSM, framed from its named graph and wired to the
monitors that fire it.

Nothing else in the package sequences anything.
"""

from __future__ import annotations

import re

from motion_spec_dsl.rdf_parser.vocab import (
    APP,
    CSTR,
    CSTR_EXT,
    CSTR_HDL,
    CSTR_HDL_EXT,
    GEOM_OP,
    KC_STAT,
    MAP,
    MOT,
    SENSORS,
    SLV,
)
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdf_utils.namespace import NS_MM_EL
from rdf_utils.naming import get_valid_var_name
from rdf_utils.uri import iri_is_descendant, iri_parent
from rdflib.namespace import RDF, RDFS, SDO
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.constraints import ConstraintTransition, GuardedMotion
from motion_spec.classes.handlers import (
    ConstraintEvaluator,
    ConstraintHandler,
    EdgeMonitor,
    EvaluatorType,
    LevelMonitor,
    RosGoalAnswer,
    RosPublication,
)
from motion_spec.classes.motion import ForwardedCommandStep, MotionSolverSlice, MotionUnit
from motion_spec.classes.solvers import CommandForwarding
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.constraint_handler import (
    SolverIdFactory,
    alignment_chain_ops,
    alignment_gradient_op,
    alignment_rotation_op,
    annotate_controller_signals,
)
from motion_spec.rdf_parser.model import reader
from motion_spec.rdf_parser.operations import (
    OPS_GENERIC,
    OPS_HANDLER,
    OPS_SOLVER,
    Schedule,
    path_projections_for_motion,
)
from motion_spec.rdf_parser.vocab import (
    URI_FSM_PRED_DESCRIPTION,
    URI_FSM_PRED_DO_TRANSITION,
    URI_FSM_PRED_END_STATE,
    URI_FSM_PRED_FIRES_EVENTS,
    URI_FSM_PRED_NAME,
    URI_FSM_PRED_REACTIONS,
    URI_FSM_PRED_START_STATE,
    URI_FSM_PRED_STATES,
    URI_FSM_PRED_TRANSITION_FROM,
    URI_FSM_PRED_TRANSITION_TO,
    URI_FSM_PRED_TRANSITIONS,
    URI_FSM_TYPE_FSM,
)

_PHASES = ("when", "while", "until")
_PHASE_PREDICATES = {"when": MOT["when"], "while": MOT["while"], "until": MOT["until"]}


def _is_elapsed_constraint(model, node) -> bool:
    """Whether a constraint is a timing constraint, measured against the clock rather than a solver."""
    return node is not None and CSTR_EXT["TimeConstraint"] in get_node_types(model.graph, node)


def _constraint_transition(model, node) -> ConstraintTransition:
    """One when/until object as a transition: an expression node over its members, or a lone
    constraint standing for itself.
    """
    if not quantities.is_constraint_aggregate(model, node):
        return ConstraintTransition(model.id(node), False, (quantities.constraint(model, node),))

    return ConstraintTransition(
        model.id(node),
        CSTR_EXT.ConstraintDisjunction in get_node_types(model.graph, node),
        tuple(
            quantities.constraint(model, member)
            for member in model.graph[node : CSTR_EXT["has-constraint"]]
        ),
    )


def _phase_any(transitions) -> bool:
    """Whether a phase is met by a disjunction: one transition, joined by `any`."""
    return len(transitions) == 1 and transitions[0].any


@reader
def guarded_motion(model, node) -> GuardedMotion:
    """A GuardedMotion: the constraints it starts on, holds during and ends on.

    Each when/until object is one transition, so a named group and a whole-section expression
    keep their own logic rather than collapsing into one flag for the phase.

    Raises:
        ConstraintViolation: the motion carries no `schema:name`, so nothing can name its step
            function.
    """
    graph = model.graph
    name = graph.value(node, SDO.name)
    if name is None:
        raise ConstraintViolation("coordination", f"GuardedMotion {node} has no schema:name triple")
    description = graph.value(node, SDO.description)

    return GuardedMotion(
        model.id(node),
        [_constraint_transition(model, item) for item in graph[node : MOT["when"]]],
        [quantities.constraint(model, item) for item in graph[node : MOT["while"]]],
        [_constraint_transition(model, item) for item in graph[node : MOT["until"]]],
        name=str(name),
        description=str(description) if description is not None else None,
    )


def constraint_handler(model, node) -> ConstraintHandler:
    """A ConstraintHandler: the motion it governs, and the evaluators and monitors it binds."""
    model.expect_type(node, CSTR_HDL["ConstraintHandler"])
    graph = model.graph

    return ConstraintHandler(
        model.id(node),
        guarded_motion(model, graph.value(node, CSTR_HDL["motion"])),
        [constraint_evaluator(model, item) for item in graph[node : CSTR_HDL["evaluators"]]],
        [],
        [monitor_entry(model, item) for item in graph[node : CSTR_HDL["monitors"]]],
        int(getattr(model.graph.value(node, APP.order), "value", 0)),
    )


# Timing relations, and how each states its threshold. An elapsed constraint has no solver error:
# codegen compares the world clock to the threshold, so the reader resolves both to seconds.
_ELAPSED_RELATIONS = (
    (CSTR["GreaterThanConstraint"], ">=", CSTR["threshold"]),
    (CSTR["EqualityConstraint"], "==", CSTR["reference-value"]),
)


@reader
def constraint_evaluator(model, node) -> ConstraintEvaluator:
    """A ConstraintEvaluator: the constraint it watches, and the error signal it writes."""
    model.expect_type(node, CSTR_HDL["ConstraintEvaluator"])
    graph = model.graph
    constraint_node = graph.value(node, CSTR_HDL["constraint"])
    assignment = CSTR_HDL["AssignmentEvaluator"] in get_node_types(graph, node)
    error_node = None if assignment else graph.value(node, CSTR_HDL["error"])

    status_slot = quantities.goal_status_act(model, graph.value(constraint_node, CSTR["quantity"]))
    goal_status = None
    if status_slot is not None:
        reference = graph.value(constraint_node, CSTR["reference-value"])
        goal_status = str(graph.value(reference, RDF.value))

    is_elapsed = _is_elapsed_constraint(model, constraint_node)
    operator, threshold, elapsed_tolerance = None, None, None
    if is_elapsed:
        types = get_node_types(graph, constraint_node)
        operator, predicate = next(
            ((op, pred) for type_, op, pred in _ELAPSED_RELATIONS if type_ in types),
            ("<", CSTR["threshold"]),
        )
        threshold = quantities.duration_seconds(model, graph.value(constraint_node, predicate))
        if operator == "==":
            band_node = graph.value(constraint_node, CSTR_EXT["tolerance"])
            elapsed_tolerance = quantities.duration_seconds(model, band_node)
    # An authored band on a spatial equality; the elapsed branch reads its own, in seconds,
    # because a duration's magnitude rides on qudt rather than on a shared value.
    band = None if is_elapsed else graph.value(constraint_node, CSTR_EXT["tolerance"])

    return ConstraintEvaluator(
        model.id(node),
        EvaluatorType.AssignmentEvaluator if assignment else EvaluatorType.ErrorEvaluator,
        quantities.constraint(model, constraint_node),
        quantities.quantity(model, error_node) if error_node is not None else None,
        tolerance=quantities.quantity(model, band) if band is not None else None,
        is_elapsed=is_elapsed,
        elapsed_op=operator,
        elapsed_threshold_s=threshold,
        elapsed_tolerance_s=elapsed_tolerance,
        goal_status=goal_status,
    )


def _monitored_expression(model, monitored):
    """The expression node a monitor targets, as its member ids and its logic.

    A named group and a whole-section conjunction/disjunction are the same thing here: one
    condition carrying its own members and join, which the monitor's terms are built from.
    """
    expression = next(
        (node for node in monitored if quantities.is_constraint_aggregate(model, node)), None
    )
    if expression is None:
        return [], False
    members = sorted(
        model.id(member) for member in model.graph[expression : CSTR_EXT["has-constraint"]]
    )

    return members, CSTR_EXT.ConstraintDisjunction in get_node_types(model.graph, expression)


@reader
def monitor_entry(model, node):
    """A monitor: a level flag its constraint sets continuously, or an edge event it fires once."""
    model.expect_type(node, CSTR_HDL["Monitor"])
    graph = model.graph
    handler = next(graph.subjects(CSTR_HDL.monitors, node), None)
    motion = graph.value(handler, CSTR_HDL.motion) if handler is not None else None
    monitored = set(graph.objects(node, CSTR_HDL.constraint))
    sections = {
        phase: set(graph.objects(motion, _PHASE_PREDICATES[phase])) if motion is not None else set()
        for phase in ("when", "until")
    }
    aggregate = len(monitored) > 1 or any(
        quantities.is_constraint_aggregate(model, item) for item in monitored
    )
    is_until_aggregate = aggregate and monitored == sections["until"]
    is_when_aggregate = aggregate and monitored == sections["when"]
    group_ids, group_any = _monitored_expression(model, monitored)

    error_node = graph.value(node, CSTR_HDL["error"])
    error = (
        None
        if is_until_aggregate or is_when_aggregate or group_ids or error_node is None
        else quantities.quantity(model, error_node)
    )
    # The band belongs to the constraint, so a monitor carries it only when it watches one.
    band = (
        graph.value(next(iter(monitored)), CSTR_EXT["tolerance"])
        if error is not None and len(monitored) == 1
        else None
    )
    shared = {
        "tolerance": quantities.quantity(model, band) if band is not None else None,
        "is_until_aggregate": is_until_aggregate,
        "is_when_aggregate": is_when_aggregate,
        "group_constraint_ids": group_ids,
        "group_any": group_any,
        "constraint_ids": sorted(model.id(item) for item in monitored),
    }
    types = get_node_types(graph, node)
    publication = _ros_publication(model, node)
    # A monitor that only publishes is neither edge- nor level-triggered: it names no signal,
    # it just reports the state of the constraint it watches.
    if CSTR_HDL["EdgeTriggeredMonitor"] not in types:
        flag_node = graph.value(node, CSTR_HDL["flag"])
        return LevelMonitor(
            model.id(node),
            "LevelTriggeredMonitor",
            error,
            model.id(flag_node) if flag_node is not None else None,
            **shared,
            **publication,
        )

    event_node = graph.value(node, CSTR_HDL["event"])
    event = model.id(event_node)
    fallback = graph.value(node, CSTR_HDL_EXT["fallback-motion"])

    return EdgeMonitor(
        model.id(node),
        "EdgeTriggeredMonitor",
        error,
        event,
        None,
        **shared,
        event_uri=str(event_node),
        event_name=event.upper(),
        fallback_motion=model.id(fallback) if fallback is not None else None,
        debounce_duration_s=quantities.optional_seconds(
            model, node, CSTR_HDL_EXT["debounce-duration"]
        ),
        **publication,
    )


# rosidl reports a nested field as `pkg/Type` and a repeated one wrapped in `sequence<>` or
# `[]`; everything else is a primitive.
_MANY = (re.compile(r"^sequence<(.+?)(?:,\s*\d+)?>$"), re.compile(r"^(.+?)\[\d*\]$"))
# Fields the node owns, never the model: the publish clock, and the scenario a run belongs to.
_AUTO_TIME_TYPE = "builtin_interfaces/Time"
_AUTO_CONTEXT_ID = ("scenario_context_id", "unique_identifier_msgs/UUID")
# rosidl spells these field types; anything else a model states must read as a number.
_STRING_TYPES = ("string", "wstring")
# What a detection states about itself rather than about the object: the frame it arrived in.
_HEADER_TYPE = "std_msgs/Header"
_FRAME_FIELD = "frame_id"
# What a detect act writes: a pose in the world. Reached by descent, so the model names the
# field that carries it rather than the whole path through it.
_POSE_TYPE = "geometry_msgs/Pose"
# The ROS type that carries a quantity published whole, per quantity type that has one.
_PAYLOAD_TYPES = {
    "Pose": _POSE_TYPE,
    "VelocityTwist": "geometry_msgs/Twist",
    "Wrench": "geometry_msgs/Wrench",
}


def _element_type(field_type: str) -> tuple[str, bool]:
    """The type one element carries, and whether the field holds many of them."""
    for pattern in _MANY:
        match = pattern.fullmatch(field_type)
        if match:
            return match.group(1), True
    return field_type, False


def _message_class(type_name: str):
    """The rosidl-generated Python class for `type_name`, or the error that names the fix."""
    try:
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise ConstraintViolation(
            "communication",
            "publishing a ROS topic needs rosidl_runtime_py; source the ROS distribution "
            "before generating",
        ) from error
    try:
        return get_message(type_name)
    except (ValueError, ModuleNotFoundError, AttributeError) as error:
        raise ConstraintViolation(
            "communication",
            f"message type '{type_name}' does not resolve; build and source the workspace so "
            "its interface package is on AMENT_PREFIX_PATH",
        ) from error


def action_shape(type_name: str) -> dict:
    """What an action type offers: the C++ type it instantiates, its header, and what its goal
    and result messages let a model state.

    A goal and a result are not message types of their own -- `get_message` cannot reach them and
    their own headers are not the ones a caller includes -- so both are walked from the class the
    action carries, under the action's include.

    Raises:
        ConstraintViolation: the ROS distribution is not sourced, or the action package is not
            on AMENT_PREFIX_PATH.
    """
    action = _action_class(type_name)
    package, cpp_type, include = _cpp_names(action)

    goal = _shape_of(action.Goal, f"{type_name} goal", package, include)
    result = _shape_of(action.Result, f"{type_name} result", package, include)

    return {
        "package": package,
        "cpp_type": cpp_type,
        "include": include,
        "goal": goal,
        "result": result,
        # A goal or a result may reach into other interface packages; the build needs every one
        # of them, not just the package the action itself lives in.
        "packages": sorted({package} | _leaf_packages(goal) | _leaf_packages(result)),
    }


def _leaf_packages(shape: dict) -> set:
    """The interface packages the message classes owning a shape's leaves come from."""
    return {owner.__module__.split(".")[0] for _element, owner in shape["leaves"].values()}


def _cpp_names(message) -> tuple[str, str, str]:
    """`(package, cpp type, include path)` read off the class rosidl generated, so the header
    name matches the generator's own case conversion by construction.
    """
    try:
        from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
    except ImportError:
        from rosidl_cmake import convert_camel_case_to_lower_case_underscore

    package, subfolder, _module = message.__module__.split(".")
    stem = convert_camel_case_to_lower_case_underscore(message.__name__)

    return (
        package,
        f"{package}::{subfolder}::{message.__name__}",
        f"{package}/{subfolder}/{stem}.hpp",
    )


def _action_class(type_name: str):
    """The rosidl-generated Python class for an action type, or the error that names the fix."""
    try:
        from rosidl_runtime_py.utilities import get_action
    except ImportError as error:
        raise ConstraintViolation(
            "communication",
            "sending a ROS action goal needs rosidl_runtime_py; source the ROS distribution "
            "before generating",
        ) from error
    try:
        return get_action(type_name)
    except (ValueError, ModuleNotFoundError, AttributeError) as error:
        raise ConstraintViolation(
            "communication",
            f"action type '{type_name}' does not resolve; build and source the workspace so "
            "its interface package is on AMENT_PREFIX_PATH",
        ) from error


def detect_shape(type_name: str, pose_path: str) -> dict:
    """How a detect act reads one action: where the goal names the objects it asks about, where
    the result holds its detections, and per detection which object it is, which frame it arrived
    in, and the pose it reports.

    Only the pose is stated by the model, because only it is ambiguous: a detection may reach
    several poses (a hypothesis carries one, a bounding box another), so which one answers the
    question is the model's to say. Everything else the message type settles on its own.

    Raises:
        ConstraintViolation: the action does not offer exactly one of what a detect act needs,
            or the fields the model named are not a repeated field and a pose within it.
    """
    action = _action_class(type_name)
    goal_targets = [
        name for name, element, many in _fields(action.Goal) if many and element in _STRING_TYPES
    ]
    detections_path, detection = _repeated_leaf(action.Result, f"{type_name} result")
    headers = [name for name, element, _many in _fields(detection) if element == _HEADER_TYPE]
    element_name = _type_name_of(detection)
    ids = [
        name for name, element, many in _fields(detection) if not many and element in _STRING_TYPES
    ]

    return {
        "targets_path": _sole(goal_targets, "repeated string fields", f"{type_name} goal"),
        "detections_path": detections_path,
        "id_path": _sole(ids, "string fields", element_name),
        "frame_path": f"{_sole(headers, f"'{_HEADER_TYPE}' fields", element_name)}.{_FRAME_FIELD}",
        "pose_path": _detection_pose(detection, element_name, pose_path),
        # The repeated field the pose is read out of, so a detection carrying none is skipped
        # rather than indexed into.
        "pose_container": pose_path.partition(".")[0],
    }


def observation_shape(type_name: str, pose_path: str) -> dict:
    """How a subscription reads one message: where it holds its detections, and per detection
    which object it is, which frame it arrived in, and the pose it reports.

    The same questions `detect_shape` asks of an action result, asked of a message that stands
    on its own -- there is no goal, so nothing names the targets in the payload.

    Raises:
        ConstraintViolation: the message does not offer exactly one of what an observation needs,
            or the fields the model named are not a repeated field and a pose within it.
    """
    root = _message_class(type_name)
    package, cpp_type, include = _cpp_names(root)
    detections_path, detection = _repeated_leaf(root, type_name)
    headers = [name for name, element, _many in _fields(detection) if element == _HEADER_TYPE]
    element_name = _type_name_of(detection)
    ids = [
        name for name, element, many in _fields(detection) if not many and element in _STRING_TYPES
    ]
    header = _sole(headers, f"'{_HEADER_TYPE}' fields", element_name)

    return {
        "package": package,
        "cpp_type": cpp_type,
        "include": include,
        # The message may reach into other interface packages; the build needs every one of them.
        "packages": sorted(
            {package} | _leaf_packages(_shape_of(root, type_name, package, include))
        ),
        "detections_path": detections_path,
        "id_path": _sole(ids, "string fields", element_name),
        "frame_path": f"{header}.{_FRAME_FIELD}",
        "pose_path": _detection_pose(detection, element_name, pose_path),
        # The repeated field the pose is read out of, so a detection carrying none is skipped
        # rather than indexed into.
        "pose_container": pose_path.partition(".")[0],
    }


def _detection_pose(detection, element_name: str, pose_path: str) -> str:
    """The accessor from one detection to the pose it reports, off the two fields the model named.

    Raises:
        ConstraintViolation: the model states no pose, names a field the detection does not
            repeat, or names one holding no single pose.
    """
    container, _, field = pose_path.partition(".")
    if not field:
        raise ConstraintViolation(
            "communication",
            f"'{element_name}' reaches more than one pose, so the act must say which one it "
            "reads: `<field> from <container>`",
        )
    entries = [
        element
        for name, element, many in _fields(detection)
        if name == container and many and "/" in element
    ]
    if not entries:
        raise ConstraintViolation(
            "communication",
            f"'{element_name}' has no repeated message field '{container}' for a pose to come from",
        )
    entry = _message_class(entries[0])
    carried = [element for name, element, many in _fields(entry) if name == field and not many]
    if not carried or "/" not in carried[0]:
        raise ConstraintViolation(
            "communication",
            f"'{_type_name_of(entry)}' has no message field '{field}' to read a pose from",
        )
    # The named field may carry the pose rather than be it -- a covariance wrapper does -- so the
    # last hop is derived, and it is only a hop when it is the one pose down there.
    descent = _descend_to(_message_class(carried[0]), _POSE_TYPE, f"{container}.{field}")

    return f"{container}[0].{field}{'.' + descent if descent else ''}"


def _sole(candidates: list, what: str, where: str):
    """The one candidate, or the error naming what was offered instead."""
    if len(candidates) != 1:
        offered = ", ".join(sorted(str(c) for c in candidates)) or "none"
        raise ConstraintViolation(
            "communication", f"'{where}' offers {len(candidates)} {what}: {offered}"
        )
    return candidates[0]


def _fields(message) -> list[tuple[str, str, bool]]:
    """`(name, element type, repeated)` per field the message declares."""
    return [
        (name, *_element_type(field_type))
        for name, field_type in message.get_fields_and_field_types().items()
    ]


def _repeated_leaves(message) -> list[tuple[str, object]]:
    """Every repeated field the message reaches, and the class one entry of each carries.

    Descends through nested messages so a result that wraps its list in an array message -- the
    `vision_msgs` convention -- is found at whatever depth it sits.
    """
    found = []

    def walk(node, prefix: str) -> None:
        for name, element, many in _fields(node):
            path = f"{prefix}{name}"
            if many and "/" in element:
                found.append((path, _message_class(element)))
            elif not many and "/" in element and element != _HEADER_TYPE:
                walk(_message_class(element), f"{path}.")

    walk(message, "")

    return found


def _repeated_leaf(message, type_name: str) -> tuple[str, object]:
    """The one repeated field the message reaches, and the class one entry carries."""
    return _sole(_repeated_leaves(message), "repeated message fields", type_name)


def _paths_to(message, wanted: str) -> list[str]:
    """Every path from `message` down to a field of type `wanted`.

    Repeated fields are not descended into: what one entry of an array holds is a question about
    the entry, asked of the entry.
    """
    found = []

    def walk(node, prefix: str) -> None:
        for name, element, many in _fields(node):
            if many or "/" not in element:
                continue
            path = f"{prefix}{name}"
            if element == wanted:
                found.append(path)
            else:
                walk(_message_class(element), f"{path}.")

    walk(message, "")

    return found


def _is_type(message, wanted: str) -> bool:
    """Whether the message is the wanted type itself rather than something reaching it."""
    return bool(getattr(message, "__module__", "").split(".")[0:2]) and (
        _type_name_of(message) == wanted
    )


def _descend_to(message, wanted: str, where: str) -> str:
    """The path from `message` down to the one field of type `wanted`, empty if it is already it."""
    if _is_type(message, wanted):
        return ""

    return _sole(_paths_to(message, wanted), f"'{wanted}' fields", where)


def _type_name_of(message) -> str:
    """`pkg/Type` for a rosidl class, as a field type spells it."""
    package, _subfolder, _module = message.__module__.split(".")

    return f"{package}/{message.__name__}"


def _shape_of(root, type_name: str, package: str, include: str) -> dict:
    """What a message class offers a model: the leaf fields it may state, the owning message
    class of each (its constants live there), and the fields the node auto-fills.
    """
    leaves: dict[str, tuple[str, object]] = {}
    auto: dict[str, str] = {}
    repeated: set[str] = set()

    def walk(message, prefix: str) -> None:
        for name, field_type in message.get_fields_and_field_types().items():
            path = f"{prefix}{name}"
            element, many = _element_type(field_type)
            if not many and element == _AUTO_TIME_TYPE:
                auto[path] = "time"
            elif not many and (name, element) == _AUTO_CONTEXT_ID:
                auto[path] = "context_id"
            elif not many and "/" in element:
                walk(_message_class(element), f"{path}.")
            else:
                leaves[path] = (element, message)
                if many:
                    repeated.add(path)

    walk(root, "")

    return {
        "type_name": type_name,
        "package": package,
        "cpp_type": _cpp_names(root)[1],
        "include": include,
        "leaves": leaves,
        "auto": auto,
        "repeated": repeated,
    }


def _message_shape(type_name: str) -> dict:
    """What a message type offers a publisher, walked from the type the model names."""
    root = _message_class(type_name)
    package, _cpp_type, include = _cpp_names(root)

    return _shape_of(root, type_name, package, include)


def standing_shape(type_name: str, quantity_type: str) -> dict:
    """What a message offers a publish that reports quantities whole.

    The quantity's own type decides which ROS type carries it and the message is descended to
    that type, so the model states the topic it publishes on rather than a path into its fields.
    A message that reaches it nowhere but inside a repeated field carries many of them, and
    `entry` says what one of them looks like; a message that reaches it directly carries one, and
    `entry` is None.

    Raises:
        ConstraintViolation: no ROS type carries that quantity whole, or the message reaches
            neither one of the type it maps to nor one array of something that does.
    """
    wanted = _PAYLOAD_TYPES.get(quantity_type)
    if wanted is None:
        raise ConstraintViolation(
            "communication",
            f"a '{quantity_type}' has no ROS type that carries it whole; a standing publish "
            f"reports {', '.join(sorted(_PAYLOAD_TYPES))}",
        )
    root = _message_class(type_name)
    package, _cpp_type, include = _cpp_names(root)
    shape = _shape_of(root, type_name, package, include)
    # A message reaching it nowhere and holding no array of anything reaches it nowhere: the
    # descent below says so about the quantity, which is what the model got wrong.
    carries_one = _is_type(root, wanted) or _paths_to(root, wanted) or not _repeated_leaves(root)
    entry = None if carries_one else _entry_shape(root, wanted, type_name)
    shape = {
        **shape,
        # The message may reach into other interface packages; the build needs every one of them.
        "packages": sorted({package} | _leaf_packages(shape)),
        # Dotted prefix, empty when the message is the quantity and nothing else.
        "payload_path": "" if entry else _prefix(_descend_to(root, wanted, type_name)),
        # The frame the quantity is stated against is the message's to carry, when it has a header.
        "frame_path": _frame_path(root),
        "auto_time": sorted(path for path, kind in shape["auto"].items() if kind == "time"),
        "auto_context_id": sorted(
            path for path, kind in shape["auto"].items() if kind == "context_id"
        ),
        "entry": entry,
    }
    if entry:
        shape["packages"] = sorted(set(shape["packages"]) | set(entry["packages"]))

    return shape


def _entry_shape(root, wanted: str, type_name: str) -> dict:
    """What one entry of the array a message carries offers: where its quantity goes, which
    entity it says it is about, and the frame it says that in.

    The same questions `observation_shape` asks of an arriving detection, asked of one the run
    writes -- so a topic can be published in the shape another model already reads.

    Raises:
        ConstraintViolation: the message holds no single array of entries, or one entry offers
            no single place for the quantity, or no single string to name what it is about.
    """
    entry_path, entry = _repeated_leaf(root, type_name)
    entry_name = _type_name_of(entry)
    package, cpp_type, include = _cpp_names(entry)
    shape = _shape_of(entry, entry_name, package, include)
    ids = [name for name, element, many in _fields(entry) if not many and element in _STRING_TYPES]

    return {
        "path": entry_path,
        "type_name": entry_name,
        "cpp_type": cpp_type,
        "payload_path": _prefix(_descend_to(entry, wanted, entry_name)),
        # What the entry says it is about. A run writes the model's own IRI here, which is what
        # the read side compares against.
        "id_path": _sole(ids, "string fields", entry_name),
        "frame_path": _frame_path(entry),
        "auto_time": sorted(path for path, kind in shape["auto"].items() if kind == "time"),
        "packages": sorted({package} | _leaf_packages(shape)),
    }


def _prefix(path: str) -> str:
    """A dotted path as a prefix to write fields under, empty when there is nothing to descend."""
    return f"{path}." if path else ""


def _frame_path(message) -> str | None:
    """Where the message states the frame it holds, when it carries a header at all."""
    headers = [
        name for name, element, many in _fields(message) if not many and element == _HEADER_TYPE
    ]

    return f"{headers[0]}.{_FRAME_FIELD}" if headers else None


def _sole_payload_path(shape: dict) -> str:
    """The one field the sugar form means, once the auto-filled ones are set aside."""
    leaves = sorted(shape["leaves"])
    if len(leaves) != 1:
        raise ConstraintViolation(
            "communication",
            f"message type '{shape['type_name']}' carries {len(leaves)} payload fields, so a "
            "publish must name the field it writes: `publish: to <topic> { path: value }`",
        )
    return leaves[0]


def _cpp_value(shape: dict, path: str, text: str) -> str:
    """The C++ the authored value renders to, against the field that owns it: a constant the
    message class defines, a quoted string, or a number the field's type accepts.
    """
    element, owner = shape["leaves"][path]
    if hasattr(owner, text) and text not in owner.get_fields_and_field_types():
        _package, owner_cpp, _include = _cpp_names(owner)
        return f"{owner_cpp}::{text}"
    if element in _STRING_TYPES:
        return '"{}"'.format(text.replace("\\", "\\\\").replace('"', '\\"'))
    try:
        float(text)
    except ValueError:
        raise ConstraintViolation(
            "communication",
            f"'{text}' is neither a constant of '{shape['type_name']}' field '{path}' nor a "
            f"value its type '{element}' accepts",
        ) from None
    return text


def publish_field(shape: dict, path: str, text: str) -> dict:
    """One authored assignment, resolved against the message type."""
    path = path or _sole_payload_path(shape)
    if path not in shape["leaves"]:
        raise ConstraintViolation(
            "communication",
            f"'{path}' is not a payload field of '{shape['type_name']}'; it offers "
            f"{', '.join(sorted(shape['leaves'])) or 'none'}",
        )
    return {"path": path, "cpp_value": _cpp_value(shape, path, text)}


def _occurrence_path(model, node, shape: dict) -> str:
    """The payload field an announced event writes its IRI into.

    Resolved the same way the sugar form resolves its field, so the message type decides what an
    occurrence looks like rather than the generator assuming a field name.

    Raises:
        ConstraintViolation: the field the message offers cannot hold an IRI.
    """
    path = _sole_payload_path(shape)
    element, _owner = shape["leaves"][path]
    if element not in _STRING_TYPES:
        raise ConstraintViolation(
            "communication",
            f"monitor '{model.id(node)}' publishes an occurrence on '{shape['type_name']}', "
            f"whose field '{path}' is a '{element}'; an occurrence carries the event's IRI, so "
            "the message must offer a string to hold it",
        )

    return path


def _ros_publication(model, node) -> dict:
    """What a monitor publishes, when the model asks it to publish at all.

    Each row states its field, its value, and the condition it holds under: a row conditioned on
    the constraint the monitor watches is satisfied; an unconditioned row is the otherwise.
    """
    graph = model.graph
    answer = _ros_answer(model, node)
    channel = graph.value(node, NS_MM_ROS["channel-name"])
    if channel is None:
        return answer
    type_name = str(graph.value(node, NS_MM_ROS["type-name"]) or "")
    shape = _message_shape(type_name)
    auto = shape["auto"]
    watched = set(graph.objects(node, CSTR_HDL["constraint"]))
    on_satisfied: list[dict] = []
    on_violated: list[dict] = []
    occurrence_path = None
    occurrence_events: list[str] = []

    for row in sorted(graph.objects(node, RDFS.member)):
        # An action member is the goal this monitor answers, which is nothing this topic carries.
        if (row, RDF.type, NS_MM_ROS["Action"]) in graph:
            continue
        # A member with no authored value is not a field row: it is one event this monitor
        # announces, published as an occurrence.
        if graph.value(row, RDF.value) is None:
            occurrence_path = _occurrence_path(model, node, shape)
            occurrence_events.append(str(row))
            continue
        conditions = set(graph.objects(row, CSTR_EXT["has-constraint"]))
        if not conditions:
            polarity = on_violated
        elif conditions == watched:
            polarity = on_satisfied
        else:
            raise ConstraintViolation(
                "communication",
                f"publish row '{model.id(row)}' states a condition that is not the constraint "
                "its monitor watches",
            )
        polarity.append(
            publish_field(
                shape,
                str(graph.value(row, NS_MM_ROS["field-path"]) or ""),
                str(graph.value(row, RDF.value)),
            )
        )

    return {
        **answer,
        "ros": RosPublication(
            str(channel),
            type_name,
            shape["package"],
            shape["include"],
            shape["cpp_type"],
            f"{model.id(node)}_pub".replace("-", "_"),
            on_satisfied=on_satisfied,
            on_violated=on_violated,
            has_satisfied=bool(on_satisfied),
            has_violated=bool(on_violated),
            auto_time=sorted(path for path, kind in auto.items() if kind == "time"),
            auto_context_id=sorted(path for path, kind in auto.items() if kind == "context_id"),
            occurrence_path=occurrence_path,
            occurrence_events=occurrence_events,
            rate_hz=_publish_rate(model, node),
        ),
    }


# The status a run may answer its own goal with, and the goal-handle call that reports it. A
# cancel is the client's to ask for, so the runtime reports it where it stops, never as an answer.
_ANSWER_METHODS = {"STATUS_SUCCEEDED": "succeed", "STATUS_ABORTED": "abort"}


def _ros_answer(model, node) -> dict:
    """How this monitor answers the goal in flight: the status it reports, and the result fields
    it fills, or nothing when it answers no goal.

    The answer is a member of the monitor rather than the monitor itself, so a monitor that
    publishes may answer too. Its own outcome member carries the status and, when the answering
    state is the satisfied one, the constraint that state holds under; every other member of it
    is one field of the result.

    Raises:
        ConstraintViolation: the monitor states a status a run cannot reach on its own, or none
            at all.
    """
    graph = model.graph
    answer = next(
        (
            member
            for member in sorted(graph.objects(node, RDFS.member))
            if (member, RDF.type, NS_MM_ROS["Action"]) in graph
        ),
        None,
    )
    if answer is None:
        return {}
    shape = action_shape(str(graph.value(answer, NS_MM_ROS["type-name"]) or ""))["result"]
    watched = set(graph.objects(node, CSTR_HDL["constraint"]))
    outcome, satisfied, fields = None, True, []
    for member in sorted(graph.objects(answer, RDFS.member)):
        path = graph.value(member, NS_MM_ROS["field-path"])
        if path is None:
            outcome = str(graph.value(member, RDF.value))
            satisfied = set(graph.objects(member, CSTR_EXT["has-constraint"])) == watched
            continue
        fields.append(publish_field(shape, str(path), str(graph.value(member, RDF.value))))
    if outcome not in _ANSWER_METHODS:
        raise ConstraintViolation(
            "communication",
            f"monitor '{model.id(node)}' answers its goal '{outcome}'; a run answers "
            f"{' or '.join(sorted(_ANSWER_METHODS))}",
        )

    return {
        "answer": RosGoalAnswer(
            outcome,
            _ANSWER_METHODS[outcome],
            shape["cpp_type"],
            fields=fields,
            satisfied=satisfied,
            auto_time=sorted(path for path, kind in shape["auto"].items() if kind == "time"),
            auto_context_id=sorted(
                path for path, kind in shape["auto"].items() if kind == "context_id"
            ),
        )
    }


def _publish_rate(model, node) -> float | None:
    """How often a monitor publishes, when the model says. Unstated is every cycle.

    Raises:
        ConstraintViolation: the rate does not say how often.
    """
    rate_node = model.graph.value(node, SENSORS["update-rate"])
    if rate_node is None:
        return None
    rate_hz = quantities.quantity(model, rate_node).value
    if not rate_hz or rate_hz <= 0.0:
        raise ConstraintViolation(
            "communication",
            f"monitor '{model.id(node)}' publishes at {rate_hz} Hz; a rate says how often, so "
            "it is positive",
        )

    return rate_hz


def build_constraint_handlers(model, schedule, derivation):
    """The constraint handlers, and the calls their evaluators and controllers imply.

    The controllers are attached here rather than read: an authored controller becomes one record
    per axis, and `constraint_handler.py` is the only module that decides how many.

    Returns:
        `(handlers, steps)`: the handler records in declaration order, and every call the
        active scope emitted for them
    """
    graph = model.graph
    handlers, steps = [], []
    for node in sorted(
        graph.subjects(RDF.type, CSTR_HDL["ConstraintHandler"]),
        key=lambda item: int(getattr(graph.value(item, APP.order), "value", 0)),
    ):
        handler = constraint_handler(model, node)
        plans = derivation.controllers_by_handler.get(node, ())
        handler.controllers = [
            controller for plan in plans for controller in derivation.controllers_for(plan)
        ]
        handlers.append(handler)
        steps.extend(
            schedule.of(list(graph.objects(node, CSTR_HDL.evaluators)), OPS_GENERIC + OPS_HANDLER)
        )
        # A single-axis controller's evaluator is not reachable from its own error signal, so
        # append it once its dependencies are scheduled.
        for plan in plans:
            if len(plan.axes) > 1 or CSTR_HDL_EXT.FeedForwardController in get_node_types(
                graph, plan.controller
            ):
                continue
            error = graph.value(plan.controller, CSTR_HDL["error-signal"])
            evaluator = next(graph.subjects(CSTR_HDL.error, error), None)
            if evaluator is not None and model.id(evaluator) not in steps:
                steps.append(model.id(evaluator))
        # Controllers run after what feeds them, and a pose command's difference evaluator after
        # every controller reading it; both in reverse, so the emitted order is the authored one.
        steps.extend(
            controller.id
            for plan in reversed(plans)
            for controller in reversed(derivation.controllers_for(plan))
        )
        steps.extend(
            SolverIdFactory(
                model.id(plan.controller), model.motion_suffix(plan.motion)
            ).pose_evaluator()
            for plan in reversed(plans)
            if len(plan.axes) > 1 and alignment_rotation_op(model, plan.quantity) is None
        )

    return handlers, steps


def assign_event_indexes(handlers) -> None:
    """Give every edge monitor a stable index into the run's event buffer."""
    index = 0
    for handler in handlers:
        for monitor in handler.monitors:
            if monitor.monitor_type == "EdgeTriggeredMonitor":
                monitor.event_idx = index
                index += 1


class PhaseNodes:
    """Which constraints, evaluators and monitors of one handler belong to which phase.

    A monitor names the constraints it watches; one that names none is bucketed by the error
    signal it reads instead, which is the only other thing tying it to a phase.
    """

    def __init__(self, model, handler, handler_node, motion_node):
        graph = model.graph
        self.handler_node = handler_node
        self.raw = {phase: set(graph[motion_node : _PHASE_PREDICATES[phase]]) for phase in _PHASES}
        self.constraints = {
            phase: quantities.expanded_constraints(model, self.raw[phase])
            if phase != "while"
            else set(self.raw[phase])
            for phase in _PHASES
        }
        self.evaluators = {phase: [] for phase in _PHASES}
        for node in graph[handler_node : CSTR_HDL["evaluators"]]:
            constraint = graph.value(node, CSTR_HDL["constraint"])
            phase = next((p for p in _PHASES if constraint in self.constraints[p]), None)
            if phase is not None:
                self.evaluators[phase].append(node)

        self.watched = {
            phase: {graph.value(node, CSTR_HDL["constraint"]) for node in self.evaluators[phase]}
            - {None}
            for phase in _PHASES
        }
        self.errors = {
            phase: {graph.value(node, CSTR_HDL["error"]) for node in self.evaluators[phase]}
            - {None}
            for phase in _PHASES
        }
        self.monitors = {phase: [] for phase in _PHASES}
        for node in graph[handler_node : CSTR_HDL["monitors"]]:
            phase = self._monitor_phase(graph, node)
            if phase is not None:
                self.monitors[phase].append(node)
        self._check(model, handler, handler_node)

    def _monitor_phase(self, graph, node) -> str | None:
        """The phase a monitor belongs to, by what it watches or, failing that, what it reads."""
        monitored = set(graph.objects(node, CSTR_HDL["constraint"]))
        if monitored:
            # A group monitor names one of the section's nodes, not the whole section, and the
            # group node itself never appears in the expanded member sets.
            for phase in ("when", "until"):
                if monitored == self.raw[phase] or monitored <= self.raw[phase]:
                    return phase

            return next((p for p in _PHASES if monitored & self.watched[p]), None)
        error = graph.value(node, CSTR_HDL["error"])

        return next((p for p in _PHASES if error in self.errors[p]), None)

    def _check(self, model, handler, handler_node) -> None:
        """Every classified node must be one the handler declares.

        An evaluator or controller bound to no phase is silently excluded from all schedules,
        which is intended; inventing one that the handler never declared is not.
        """
        for kind, declared_predicate in (
            ("evaluators", CSTR_HDL["evaluators"]),
            ("monitors", CSTR_HDL["monitors"]),
        ):
            declared = set(model.graph[handler_node:declared_predicate])
            classified = {node for phase in _PHASES for node in getattr(self, kind)[phase]}
            if not classified <= declared:
                raise ValueError(
                    f"Handler {handler.id}: classified {kind} not a subset of handler {kind}"
                )


def _upstream_dependencies(data_id: str, closure_inputs: dict) -> set:
    """Every data id that feeds one id, however many closures deep."""
    result: set[str] = set()
    pending = list(closure_inputs.get(data_id, set()))
    while pending:
        item = pending.pop()
        if item in result:
            continue
        result.add(item)
        pending.extend(closure_inputs.get(item, set()))
    return result


def _handler_chain_solvers(handler, serial_chains, solver_ids) -> list:
    """The arm solvers this handler commands, sliced to the motion driver it drives them with.

    No copy of the solver's own facts (`algorithm`, `gravity`, `chain.root`, ...) -- a
    template reaches them through `solver_id` into `resources.by_id`.
    """
    result = []
    driver_id = f"driver_{handler.id}"
    for solver in serial_chains:
        if solver.id not in solver_ids or not solver.motion_drivers:
            continue
        drivers = solver.motion_drivers
        selected = next((driver for driver in drivers if driver.id == driver_id), drivers[0])
        result.append(
            MotionSolverSlice(
                id=solver.id,
                solver_id=solver.id,
                output=solver.output,
                motion_driver=selected,
                read_only=not (
                    selected.acceleration_constraint
                    or selected.cartesian_force
                    or selected.cartesian_acceleration
                    or selected.joint_force
                ),
            )
        )

    return result


def _cartesian_force_nodes(model, chain_solvers, handler, computation) -> list:
    """The authored Cartesian forces a handler's own controllers ultimately drive."""
    graph = model.graph
    outputs = {controller.control_signal.id for controller in handler.controllers}
    found = []
    for solver in chain_solvers:
        driver_node = model.node_by_id.get(solver.motion_driver.id)
        if driver_node is None:
            continue
        for node in graph[driver_node : SLV["cartesian-force"]]:
            force = graph.value(node, SLV["force"])
            if force is None:
                continue
            force_id = model.id(force)
            upstream = {force_id} | _upstream_dependencies(
                force_id, computation.indexes.closure_input
            )
            if upstream & outputs:
                found.append(node)

    return found


def _append_new(steps: list, candidates) -> None:
    """Append the calls this list does not already carry, keeping their order."""
    for step in candidates:
        if step not in steps:
            steps.append(step)


class MotionSchedules:
    """The three call sequences one motion runs, in the order the loop runs them.

    `commanded_force` is held back rather than folded into `active`: a commanded wrench is built
    from control signals, so it can only run once the control laws have written them this tick.
    """

    def __init__(
        self,
        when: list,
        while_pre: list,
        active: list,
        until: list,
        commanded_force: list | None = None,
    ):
        self.when = when
        self.while_pre = while_pre
        self.active = active
        self.until = until
        self.commanded_force = commanded_force or []


def _when_schedule(model, phase: PhaseNodes) -> list:
    """The calls `can_start` runs, in their own scope: a step evaluated in both phases emits in
    both, so the when block never shares the active block's emitted-once set.
    """
    scope = Schedule(model)
    live = [
        node
        for node in phase.evaluators["when"]
        if not _is_elapsed_constraint(model, model.graph.value(node, CSTR_HDL["constraint"]))
    ]
    steps = scope.of(live, OPS_GENERIC + OPS_HANDLER)
    # can_start inlines the when evaluators, but their prerequisite generic ops still need
    # scheduling, and the evaluators themselves are not reachable from anything.
    for node in live:
        if scope.claim(model.id(node)):
            steps.append(model.id(node))

    return steps


def _motion_schedules(
    model, phase, handler, chain_solvers, derivation, groups, computation
) -> MotionSchedules:
    """The three schedules a motion runs, in loop order."""
    graph = model.graph
    scope = Schedule(model)

    def live(nodes):
        return [
            node
            for node in nodes
            if not _is_elapsed_constraint(model, graph.value(node, CSTR_HDL["constraint"]))
        ]

    # Until monitors run before control each tick, so build the until schedule first: a quantity
    # an until monitor consumes must be scheduled in the earlier phase, or the monitor reads the
    # previous tick's value on its first tick.
    until = scope.of(live(phase.evaluators["until"]), OPS_GENERIC + OPS_HANDLER)
    # Until evaluators have no controller error signal to drive backward discovery, so append
    # them after their dependencies to keep the emitted call order right.
    for node in live(phase.evaluators["until"]):
        if scope.claim(model.id(node)):
            until.append(model.id(node))

    grouped_ids = {component.eval_id for group in groups for component in group.components}
    grouped_nodes = {node for node in phase.evaluators["while"] if model.id(node) in grouped_ids}
    # A grouped evaluator's error is emitted inline ahead of the schedule block, but whatever
    # produces its reference still has to run first -- so walk the grouped nodes before the main
    # pass rather than skipping those producers entirely.
    while_pre = [
        step
        for step in scope.of(sorted(grouped_nodes, key=str), OPS_GENERIC + OPS_HANDLER)
        if step not in grouped_ids
    ]

    active_plans = _active_plans(phase, derivation)
    by_constraint = {plan.constraint: plan for plan in active_plans}
    leading, trailing = [], []
    for node in live(phase.evaluators["while"]):
        if node in grouped_nodes:
            continue
        plan = by_constraint.get(graph.value(node, CSTR_HDL.constraint))
        feed_forward = plan is not None and CSTR_HDL_EXT.FeedForwardController in get_node_types(
            graph, plan.controller
        )
        (trailing if plan is None or feed_forward else leading).append(node)

    active = scope.of(leading, OPS_GENERIC + OPS_HANDLER)
    # The controller calls themselves are appended in authored order below, so drop whatever the
    # backward walk found of them.
    controller_ids = {
        controller.id for plan in active_plans for controller in derivation.controllers_for(plan)
    } | {model.id(plan.controller) for plan in active_plans}
    active = [step for step in active if step not in controller_ids]
    _append_new(active, _alignment_chain_steps(model, phase))
    _append_new(active, _pose_command_steps(model, scope, active_plans))
    _append_new(active, [model.id(node) for node in leading])
    force_nodes = _cartesian_force_nodes(model, chain_solvers, handler, computation)
    # Claimed here so the walk still resolves shared prerequisites in this position, but emitted
    # after the control laws run: the wrench reads the control signal they write this tick.
    commanded_force = scope.of(force_nodes, OPS_GENERIC + OPS_SOLVER + OPS_HANDLER)
    active.extend(scope.of(trailing, OPS_GENERIC + OPS_HANDLER))
    _append_new(active, [model.id(node) for node in trailing])

    return MotionSchedules(_when_schedule(model, phase), while_pre, active, until, commanded_force)


def _alignment_chain_steps(model, phase) -> list:
    """Compute ops for every alignment this motion holds during: rotated direction, angle, then
    the direction the constraint is driven along -- a rotation vector onto the reference, or, off
    zero, the gradient axis. Both are read through shared state, by a moment controller's wrench
    or by a solver row, so the backward walk from either never reaches these; the ops are emitted
    once for the whole model, and every motion that holds the constraint has to name them itself.
    """
    graph = model.graph
    steps = []
    for constraint in phase.constraints["while"]:
        quantity = graph.value(constraint, CSTR.quantity)
        if quantity is None:
            continue
        drive_op = alignment_rotation_op(model, quantity) or alignment_gradient_op(model, quantity)
        if drive_op is None:
            continue
        _append_new(steps, [model.id(op) for op in alignment_chain_ops(model, quantity)])
        _append_new(steps, [model.id(drive_op)])
    return steps


def _pose_command_steps(model, scope, active_plans) -> list:
    """The interpolation and difference calls a per-axis pose command adds to the active block."""
    graph = model.graph
    steps = []
    for plan in active_plans:
        if len(plan.axes) <= 1:
            continue
        # An alignment has no pose pair to interpolate or difference; its own chain is scheduled.
        if alignment_rotation_op(model, plan.quantity) is not None:
            continue
        reference = graph.value(plan.constraint, CSTR["reference-value"])
        reference_view = quantities.view_of(graph, reference)
        interpolation = next(
            graph.subjects(GEOM_OP.out, graph.value(reference_view, MAP.superobject)), None
        )
        if interpolation is not None:
            steps.extend(scope.of([interpolation], OPS_GENERIC + OPS_HANDLER))
        steps.append(
            SolverIdFactory(
                model.id(plan.controller), model.motion_suffix(plan.motion)
            ).pose_evaluator()
        )

    return steps


def _active_plans(phase: PhaseNodes, derivation) -> list:
    """The authored controllers whose constraints this motion holds during."""
    plans = derivation.controllers_by_handler.get(phase.handler_node, ())
    return [plan for plan in plans if plan.constraint in phase.constraints["while"]]


def build_motions(model, handlers, robots, computation, derivation, fsm):
    """Build one motion unit per handler, then fold on what only the whole set decides.

    Returns:
        `(motions, fsm_meta)`: the motions in the order the handlers declare them, and the FSM
        wiring codegen needs alongside the framed FSM
    """
    motions = []
    tokens = {
        model.motion_suffix(model.graph.value(model.node_by_id[handler.id], CSTR_HDL["motion"]))
        for handler in handlers
    }
    # A per-motion slice copies nothing off its solver: `solver_id` resolves through the
    # resources index, exactly as templates do through `resources.by_id`.
    solvers_by_id = robots.by_id
    for handler in handlers:
        handler_node = model.node_by_id[handler.id]
        motion_node = model.graph.value(handler_node, CSTR_HDL["motion"])
        phase = PhaseNodes(model, handler, handler_node, motion_node)
        # A declared solver counts even when no controller routes to it: a monitor-only handler
        # still owns its arm runtime for state reading, FK and command forwarding.
        solver_ids = {
            model.id(plan.solver)
            for plan in derivation.controllers_by_handler.get(handler_node, ())
        } | {
            model.id(solver)
            for solver in model.graph.objects(handler_node, CSTR_HDL_EXT["runs-solver"])
        }
        chain_solvers = _handler_chain_solvers(handler, robots.serial_chains, solver_ids)
        evaluators = {
            name: [constraint_evaluator(model, node) for node in phase.evaluators[name]]
            for name in _PHASES
        }
        groups = quantities.pose_axis_error_groups_for_motion(
            model, dict(zip(phase.evaluators["while"], evaluators["while"])), computation.views
        )
        schedules = _motion_schedules(
            model, phase, handler, chain_solvers, derivation, groups, computation
        )
        active_controllers = [
            controller
            for plan in _active_plans(phase, derivation)
            for controller in derivation.controllers_for(plan)
        ]
        # Drop the calls another motion owns: the backward walk can reach its closures, and
        # running them here would recompute its outputs while it is inactive.
        token = model.motion_suffix(motion_node)
        owner = computation.indexes.closure_owner
        schedules.active = [step for step in schedules.active if owner.get(step, token) == token]
        schedules.while_pre = [
            step for step in schedules.while_pre if owner.get(step, token) == token
        ]
        _append_new(schedules.active, [c.id for c in reversed(active_controllers)])
        _append_new(
            schedules.active,
            [step for step in schedules.commanded_force if owner.get(step, token) == token],
        )
        # What state runs this motion, when the model says so rather than leaving it derived from
        # the event that ends the motion -- which a motion meant to keep running never fires.
        runs_in = model.graph.value(handler_node, CSTR_HDL_EXT["runs-in-state"])
        motions.append(
            _motion_unit(
                model,
                handler,
                phase,
                chain_solvers,
                robots.serial_chains,
                evaluators,
                groups,
                schedules,
                active_controllers,
                derivation,
                computation,
                motion_node,
                tokens,
                solvers_by_id,
            )
        )
        if runs_in is not None:
            motions[-1].runs_in_state = str(runs_in)
        unit = motions[-1]
        unit.entry_snapshots = [s for s in unit.snapshots if s.scope == "entry"]
        unit.task_snapshots = [s for s in unit.snapshots if s.scope == "task"]
        unit.has_entry_snapshots = bool(unit.entry_snapshots)

    return _finish_motions(model, motions, handlers, computation, fsm, solvers_by_id)


def _motion_unit(
    model,
    handler,
    phase,
    chain_solvers,
    runtime_solvers,
    evaluators,
    groups,
    schedules,
    active_controllers,
    derivation,
    computation,
    motion_node,
    tokens,
    solvers_by_id,
):
    """One motion's IR unit: its evaluators, monitors, schedules and everything it captures."""
    all_evaluators = evaluators["while"] + evaluators["when"] + evaluators["until"]
    when_elapsed = quantities.elapsed_coordinate_ids(evaluators["when"])
    active_elapsed = quantities.elapsed_coordinate_ids(evaluators["while"] + evaluators["until"])

    return MotionUnit(
        id=handler.id,
        motion_id=handler.motion.id,
        name=handler.motion.name,
        description=(handler.motion.description or "").splitlines(),
        has_when_elapsed=bool(when_elapsed),
        has_active_elapsed=bool(active_elapsed),
        when_elapsed_ids=when_elapsed,
        active_elapsed_ids=active_elapsed,
        when_evaluators=evaluators["when"],
        while_evaluators=evaluators["while"],
        until_evaluators=evaluators["until"],
        controllers=active_controllers,
        when_monitors=[monitor_entry(model, node) for node in phase.monitors["when"]],
        while_monitors=[monitor_entry(model, node) for node in phase.monitors["while"]],
        until_monitors=[monitor_entry(model, node) for node in phase.monitors["until"]],
        when_schedule=schedules.when,
        while_schedule=schedules.active,
        until_schedule=schedules.until,
        has_elapsed=bool(when_elapsed or active_elapsed),
        has_until_condition=bool(evaluators["until"]),
        when_any=_phase_any(handler.motion.when_transitions),
        serial_chain_solvers=chain_solvers,
        relative_poses=quantities.relative_poses_for_motion(
            all_evaluators, computation.views, chain_solvers
        ),
        scene_relative_poses=quantities.scene_relative_poses_for_motion(
            computation.views, chain_solvers, solvers_by_id, all_evaluators
        ),
        pose_axis_error_groups=groups,
        while_pre_schedule=schedules.while_pre,
        forwarded_commands=_forwarded_commands(
            model, phase, chain_solvers, runtime_solvers, derivation
        ),
        snapshots=quantities.snapshots_for_motion(
            all_evaluators,
            handler.motion.while_ + handler.motion.when + handler.motion.until,
            computation.indexes,
            computation.views,
            schedules.active + schedules.when + schedules.until,
            computation.closures,
            model.motion_suffix(motion_node),
            tokens,
        ),
        path_projections=path_projections_for_motion(
            schedules.while_pre + schedules.active, computation.closures
        ),
    )


def _forwarded_commands(model, phase, chain_solvers, runtime_solvers, derivation) -> list:
    """The controller outputs written straight to a joint rather than through a solver."""
    graph = model.graph
    # Resolved from the joint, not the agent: a gripper's joint rides the arm's runtime.
    owned_trees = {solver.id: solver.runtime.owned_trees or () for solver in runtime_solvers}
    commands = []
    for plan in _active_plans(phase, derivation):
        if not issubclass(derivation.algorithm_by_solver[plan.solver], CommandForwarding):
            continue
        controller = derivation.controllers_for(plan)[0]
        quantity = graph.value(plan.constraint, CSTR.quantity)
        view = quantities.view_of(graph, quantity)
        target_quantity = graph.value(view, MAP.superobject) if view is not None else quantity
        target = graph.value(target_quantity, KC_STAT["of-joint"])
        chain_solver = next(
            (
                solver
                for solver in chain_solvers
                if target is not None
                and any(iri_is_descendant(tree, target) for tree in owned_trees.get(solver.id, ()))
            ),
            None,
        )
        if chain_solver is None:
            raise RuntimeError(
                "command forwarding: joint "
                f"'{model.label(target) if target is not None else target}' belongs to no "
                "kinematic tree this handler's runtimes own"
            )
        runtime = next(s for s in runtime_solvers if s.id == chain_solver.id)
        commands.append(
            ForwardedCommandStep(
                f"cmd-fwd-{model.id(plan.controller)}",
                controller.control_signal,
                f"{runtime.runtime.prefix}{model.label(target)}" if target is not None else "",
                chain_solver.id,
            )
        )

    return commands


# Per superobject type, the branch flags a pose-axis error group renders through.
_GROUP_TYPE_FLAGS = {
    "is_pose": ("Pose",),
    "is_twist": ("VelocityTwist", "AccelerationTwist"),
    "is_wrench": ("Wrench",),
}


def _finish_motions(model, motions, handlers, computation, fsm, solvers_by_id):
    """Fold on everything that needs the whole set of motions to be known."""
    order_by_handler = {handler.id: handler.order for handler in handlers}
    ordered = sorted(motions, key=lambda motion: order_by_handler[motion.id])
    for motion in ordered:
        _set_motion_conditions(motion)
        for group in motion.pose_axis_error_groups:
            for flag, types in _GROUP_TYPE_FLAGS.items():
                setattr(group, flag, group.superobject_type in types)
        annotate_controller_signals(motion.controllers, computation.closures)
        motion.declared_pose_components = quantities.declared_pose_component_entries(
            model,
            computation.data_structures,
            computation.indexes.pose_components,
            quantities.collect_motion_references(motion, computation.closures),
        )
    # Ordered: the FSM wiring tags monitors and motions, then the capability booleans, then the
    # gate calls that read them.
    meta = _apply_fsm_wiring(ordered, fsm, solvers_by_id.values())
    _add_motion_function_interfaces(ordered, solvers_by_id)
    _apply_fsm_gate_calls(ordered, meta["cpp_namespace"])

    return ordered, meta


def annotate_sensor_dependencies(motions, computation) -> None:
    """Resolve which mounted sensor readings each motion's computations consume."""
    for motion in motions:
        references = quantities.collect_motion_input_references(motion, computation.closures)
        referenced_outputs = {
            view.superobject.id
            for view in computation.views.values()
            if view.subobject.id in references
        }
        for solver in motion.serial_chain_solvers:
            solver.required_sensors = sorted(
                output.sensor_name
                for output in solver.output
                if (output.id in references or output.id in referenced_outputs)
                and getattr(output, "sensor_name", "")
            )


def evaluator_term(evaluator) -> dict:
    """The boolean term one evaluator contributes to a condition.

    An elapsed timing predicate or a solver constraint-satisfied check. Every term kind reads
    shared state and nothing else, so the same condition renders identically inside the motion and
    in the introspection sample that runs outside it.
    """
    if evaluator.goal_status:
        return {
            "kind": "goal-status",
            "status_id": evaluator.constraint.quantity.id,
            "value": evaluator.goal_status,
        }

    if evaluator.is_elapsed:
        operator = evaluator.elapsed_op or ">="
        threshold = evaluator.elapsed_threshold_s or 0.0
        elapsed_id = quantities.elapsed_coordinate_id(evaluator)
        # Pre-format the threshold so the emitted literal is stable.
        if operator == "==":
            tolerance = evaluator.elapsed_tolerance_s or 0.0

            return {
                "kind": "elapsed-eq",
                "elapsed_id": elapsed_id,
                "threshold": f"{threshold:.6f}",
                "tolerance": f"{tolerance:.6f}",
            }

        return {
            "kind": "elapsed",
            "elapsed_id": elapsed_id,
            "op": operator,
            "threshold": f"{threshold:.6f}",
        }

    term = {"kind": "constraint", "error_id": getattr(evaluator.error, "id", None)}
    # Omitted, not empty: ST4 reads an empty string as present, and would emit a bare access.
    tolerance_id = getattr(evaluator.tolerance, "id", None)
    if tolerance_id:
        term["tolerance_id"] = tolerance_id

    return term


def _stamp_terms(monitor, terms, any_flag, where: str) -> None:
    """Give a monitor the boolean terms its condition is built from.

    Raises:
        ConstraintViolation: the condition lowered to no terms at all, which renders as a
            constant false -- a monitor that can never fire, and an FSM that can never leave the
            state it watches.
    """
    if not terms:
        raise ConstraintViolation(
            "coordination",
            f"monitor '{monitor.id}' ({where}) watches a condition that lowered to no terms, so "
            "it renders as a constant false: it can never fire, and the FSM can never leave the "
            "state it runs in. Every constraint it names is one nothing evaluates -- give it a "
            "constraint with an error to watch, or an elapsed time.",
        )
    monitor.active_terms = terms
    monitor.active_terms_present = bool(terms)
    monitor.active_any = any_flag
    monitor.has_active = True


def _set_monitor_conditions(motion, phase: str) -> None:
    """Stamp one phase's terms onto the aggregate monitor, and onto any monitor whose one
    constraint is read directly rather than through a solver error -- an elapsed clock or an
    action goal's status."""
    evaluators = getattr(motion, f"{phase}_evaluators")
    aggregate_field = f"is_{phase}_aggregate"
    terms = [
        evaluator_term(evaluator)
        for evaluator in evaluators
        if evaluator.error or evaluator.is_elapsed
    ]
    # Keyed by constraint, not by error: two goal-status items on one act share the status slot,
    # so the error alone cannot say which status a monitor is watching for.
    direct_by_constraint = {
        evaluator.constraint.id: evaluator_term(evaluator)
        for evaluator in evaluators
        if (evaluator.is_elapsed or evaluator.goal_status) and evaluator.error
    }
    for monitor in getattr(motion, f"{phase}_monitors"):
        group_ids = set(monitor.group_constraint_ids or ())
        if group_ids:
            _stamp_terms(
                monitor,
                [
                    evaluator_term(evaluator)
                    for evaluator in evaluators
                    if evaluator.constraint.id in group_ids
                    and (evaluator.error or evaluator.is_elapsed)
                ],
                bool(monitor.group_any),
                f"the '{phase}' group in motion '{motion.id}'",
            )
            continue
        # A whole-section monitor over a flat constraint list, from a graph minted before
        # sections carried an expression node. Archived generations vendor those graphs and
        # introspection replay rebuilds the IR from them, so the join is spelled out here: a
        # section only ever linked flat when it meant a conjunction.
        if getattr(monitor, aggregate_field):
            _stamp_terms(monitor, terms, False, f"the whole '{phase}' section of '{motion.id}'")
            continue
        direct = [
            direct_by_constraint[constraint_id]
            for constraint_id in monitor.constraint_ids
            if constraint_id in direct_by_constraint
        ]
        if direct:
            _stamp_terms(monitor, direct, False, f"the '{phase}' phase of '{motion.id}'")


def _set_motion_conditions(motion) -> None:
    """Fold the until and when boolean terms onto a motion.

    Raises:
        ConstraintViolation: the motion states a `when` precondition none of whose conditions
            lowered to a term. Declaring none is fine and means the motion is always ready;
            stating one that evaluates to nothing renders as `can_start` returning a constant
            true, so the motion starts as if the precondition had been met.
    """
    _set_monitor_conditions(motion, "until")
    motion.when_terms = [
        evaluator_term(evaluator)
        for evaluator in motion.when_evaluators
        if evaluator.error or evaluator.is_elapsed
    ]
    if motion.when_evaluators and not motion.when_terms:
        raise ConstraintViolation(
            "coordination",
            f"motion '{motion.id}' states a 'when' precondition that lowered to no terms, so it "
            "would start unconditionally -- the gate reads as always open, not as the condition "
            "the model states. Every condition it names is one nothing evaluates.",
        )
    motion.when_terms_present = bool(motion.when_terms)
    _set_monitor_conditions(motion, "when")


def _add_motion_function_interfaces(motions: list, solvers_by_id: dict) -> None:
    """Fold the capability booleans each generated function's signature is built from.

    Also assigns each motion its introspection index, so the frame-log schema and the generated
    sample switch read one field rather than agreeing with a second generator.
    """
    for index, motion in enumerate(motions):
        motion.index = index
        when_mons = motion.when_monitors
        until_mons = motion.until_monitors
        has_when_elapsed = any(evaluator.is_elapsed for evaluator in motion.when_evaluators)
        when_sched = bool(motion.when_schedule)
        until_sched = bool(motion.until_schedule)
        when_fsm = any(monitor.fsm_namespace for monitor in when_mons)
        until_fsm = any(monitor.fsm_namespace for monitor in until_mons)
        has_chain = bool(motion.serial_chain_solvers)

        motion.can_start_needs_state = has_when_elapsed
        motion.can_start_needs_shared = bool(when_sched or motion.when_evaluators)
        motion.can_start_needs_robot = False

        motion.when_needs_state = has_when_elapsed or bool(when_mons)
        motion.when_needs_shared = (
            has_when_elapsed
            or bool(motion.declared_pose_components)
            or when_sched
            or bool(when_mons)
        )
        motion.when_needs_robot = when_fsm

        motion.until_needs_state = bool(until_mons)
        motion.until_needs_shared = until_sched or bool(until_mons)
        motion.until_needs_robot = until_fsm

        motion.monitor_needs_state = bool(when_mons) or bool(until_mons)
        motion.monitor_needs_shared = (
            when_sched or bool(when_mons) or until_sched or bool(until_mons)
        )
        motion.monitor_needs_robot = when_fsm or until_fsm

        motion.apply_needs_state = has_chain
        # Gate on the torque limit, not on the joint-space samples: this runs before the
        # introspection artifact exists, so the sample list does not yet.
        motion.apply_needs_shared = bool(motion.forwarded_commands) or any(
            solvers_by_id[solver.solver_id].torque_saturation
            for solver in motion.serial_chain_solvers
        )
        motion.apply_needs_robot = has_chain or bool(motion.forwarded_commands)

        # An edge is the occurrence: a flag monitor holds a level and never reaches the buffer.
        when_events = any(monitor.is_edge_triggered for monitor in when_mons)
        until_events = any(monitor.is_edge_triggered for monitor in until_mons)
        control_events = any(monitor.is_edge_triggered for monitor in motion.while_monitors)
        motion.when_needs_events = when_events
        motion.until_needs_events = until_events
        motion.monitor_needs_events = when_events or until_events
        motion.control_needs_events = control_events
        motion.step_needs_events = until_events or control_events


def read_fsm(model) -> dict | None:
    """The FSM named graph, framed the way codegen reads it.

    States, events, transitions and reactions, in the same shape the standalone header uses, so
    codegen needs no second read of the FSM document. None when the model imports no `.fsm`.

    Returns:
        the framed FSM, with every table sorted -- event indices are assigned from this order and
        baked into the generated C++, so two generations must agree on it
    """
    graph = model.graph
    fsm_node = next(iter(graph.subjects(RDF["type"], URI_FSM_TYPE_FSM)), None)
    if fsm_node is None:
        return None

    def token(uri):
        return get_valid_var_name(graph.compute_qname(uri)[2]).upper()

    state_uris = dict(
        sorted((token(s), str(s)) for s in graph.objects(fsm_node, URI_FSM_PRED_STATES))
    )
    event_loop = graph.value(fsm_node, NS_MM_EL["event-loop"])
    event_uris = dict(
        sorted((token(e), str(e)) for e in graph.objects(event_loop, NS_MM_EL["has-event"]))
    )
    transitions = sorted(
        (
            {
                "id": token(node),
                "uri": str(node),
                "from_state": token(graph.value(node, URI_FSM_PRED_TRANSITION_FROM)),
                "to_state": token(graph.value(node, URI_FSM_PRED_TRANSITION_TO)),
            }
            for node in graph.objects(fsm_node, URI_FSM_PRED_TRANSITIONS)
        ),
        key=lambda row: row["id"],
    )
    reactions = sorted(
        (
            {
                "id": token(node),
                "uri": str(node),
                "when_event": token(graph.value(node, NS_MM_EL["ref-event"])),
                "do_transition": token(graph.value(node, URI_FSM_PRED_DO_TRANSITION)),
                "fires_events": sorted(
                    token(event) for event in graph.objects(node, URI_FSM_PRED_FIRES_EVENTS)
                ),
                "num_fires": len(list(graph.objects(node, URI_FSM_PRED_FIRES_EVENTS))),
            }
            for node in graph.objects(fsm_node, URI_FSM_PRED_REACTIONS)
        ),
        key=lambda row: row["id"],
    )
    description = graph.value(fsm_node, URI_FSM_PRED_DESCRIPTION)

    return {
        "name": str(graph.value(fsm_node, URI_FSM_PRED_NAME)),
        "description": str(description) if description is not None else None,
        "start_state": token(graph.value(fsm_node, URI_FSM_PRED_START_STATE)),
        "end_state": token(graph.value(fsm_node, URI_FSM_PRED_END_STATE)),
        "states": list(state_uris),
        "state_uris": state_uris,
        "events": list(event_uris),
        "event_uris": event_uris,
        "transitions_table": transitions,
        "reactions_table": reactions,
        # Event and state IRIs share the FSM node's parent path; a monitor's event is matched
        # against it to tell an FSM event from a monitor-owned one.
        "namespace_uri": str(model.child_node(iri_parent(fsm_node), "")),
    }


def _apply_fsm_wiring(motions, fsm, solvers) -> dict:
    """Tag the monitors that fire the FSM, and return the wiring codegen needs beside it.

    Raises:
        ConstraintViolation: a motion declares no `until` and the model imports no FSM, so nothing
            can end it; a snapshot triggers on an event the FSM does not declare; or a WHEN-gated
            motion names no hold motion, or an unknown one.
    """
    namespace = fsm["name"].lower() if fsm else None
    events = fsm.get("events", []) if fsm else []
    index_by_event = {event: index for index, event in enumerate(events)}
    step_event = "E_STEP" if "E_STEP" in events else None
    # The heartbeat is the clock and is not logged every tick, but where a transition's guard is
    # the clock, that occurrence caused the state change -- so name those transitions.
    transitions = {row["id"]: row for row in (fsm.get("transitions_table", []) if fsm else [])}
    meta = {
        "cpp_namespace": namespace,
        "header": f"{fsm['name']}.hpp" if fsm else None,
        "step_event": step_event,
        "step_event_idx": index_by_event.get(step_event, -1),
        "step_transitions": [
            {"from": transition["from_state"], "to": transition["to_state"]}
            for reaction in (fsm.get("reactions_table", []) if fsm else [])
            if reaction["when_event"] == step_event
            for transition in [transitions.get(reaction["do_transition"])]
            if transition
        ],
    }
    if namespace is None:
        # Without an FSM the sequencer advances on a motion's own `until`, so one declaring none
        # can never be left and every motion after it is unreachable.
        stuck = [motion.id for motion in motions if not motion.has_until_condition]
        if stuck:
            raise ConstraintViolation(
                "coordination",
                f"motions {sorted(stuck)} declare no 'until' condition and the model imports no "
                "FSM, so nothing can end them; add an 'until' condition or coordinate the model "
                "with an FSM",
            )
        _resolve_occurrence_events(motions, fsm, namespace)

        return meta

    namespace_uri = fsm.get("namespace_uri")
    state_by_event = {
        reaction["when_event"]: transitions[reaction["do_transition"]]["from_state"]
        for reaction in fsm["reactions_table"]
        if reaction["do_transition"] in transitions
    }
    # A fallback names the motion specification, which several handlers may realize.
    units_by_motion: dict[str, list] = {}
    for motion in motions:
        units_by_motion.setdefault(motion.motion_id, []).append(motion)
    state_by_uri = {uri: name for name, uri in fsm.get("state_uris", {}).items()}
    for motion in motions:
        if not motion.runs_in_state:
            continue
        state = state_by_uri.get(motion.runs_in_state)
        if state is None:
            raise ConstraintViolation(
                "coordination",
                f"motion '{motion.id}' says it runs in '{motion.runs_in_state}', which the FSM "
                f"'{namespace}' does not declare as a state.",
            )
        motion.fsm_state = state

    def fires_fsm_event(monitor) -> bool:
        """A monitor fires the FSM only when its event lives in the FSM's namespace; a
        standalone, monitor-owned event keeps its own stub."""
        return bool(
            monitor.is_edge_triggered
            and iri_is_descendant(namespace_uri or "", monitor.event_uri or "")
        )

    def stamp(monitor):
        monitor.fsm_namespace = namespace
        monitor.fsm_event_idx = index_by_event.get(monitor.event_name or "", -1)

    # A sensor re-tares on occurrences the same way a snapshot re-samples on them.
    for solver in solvers:
        for out in getattr(solver, "output", ()):
            if not getattr(out, "retare_event_uris", ()):
                continue
            names = []
            for uri in out.retare_event_uris:
                name = uri.rsplit("/", 1)[-1].rsplit("#", 1)[-1]
                if not iri_is_descendant(namespace_uri or "", uri) or name not in index_by_event:
                    raise ConstraintViolation(
                        "coordination",
                        f"Wrench '{out.id}' re-tares on '{name}', which '{namespace}' does not "
                        "declare.",
                    )
                names.append(f"{namespace}::{name}")
            out.retare_events = tuple(names)
            out.retare_events_present = bool(names)

    for motion in motions:
        # An event-triggered snapshot only compiles when the FSM declares the event it waits on.
        for snapshot in motion.snapshots:
            if not snapshot.trigger_event:
                continue
            if snapshot.trigger_event not in index_by_event:
                raise ConstraintViolation(
                    "coordination",
                    f"Snapshot '{snapshot.target_id}' in motion '{motion.id}' triggers on "
                    f"'{snapshot.trigger_event}', which the FSM '{namespace}' does not declare.",
                )
            snapshot.fsm_namespace = namespace
        for monitor in [*motion.until_monitors, *motion.while_monitors]:
            if not fires_fsm_event(monitor):
                continue
            stamp(monitor)
            state = state_by_event.get(monitor.event_name or "")
            if state and not motion.fsm_state:
                motion.fsm_state = state
        for monitor in motion.when_monitors:
            if not fires_fsm_event(monitor):
                continue
            stamp(monitor)
            fallback = _when_gate_fallback(motion, monitor, units_by_motion)
            state = state_by_event.get(monitor.event_name or "")
            if state and not fallback.fsm_state:
                fallback.fsm_state = state
            if motion.id not in fallback.fsm_when_gate_motions:
                fallback.fsm_when_gate_motions.append(motion.id)

    _apply_reentry_events(motions, fsm)
    _resolve_occurrence_events(motions, fsm, namespace)
    _check_every_commanding_motion_runs(motions, fsm, meta)

    return meta


def _reachable_states(fsm) -> set:
    """The states the FSM can reach from its start state.

    A transition fires only when a reaction names it, so one no reaction refers to is not an
    edge: following it would call a state reachable that the generated FSM can never enter.
    """
    transitions = {row["id"]: row for row in fsm["transitions_table"]}
    outgoing: dict = {}
    for reaction in fsm["reactions_table"]:
        transition = transitions.get(reaction["do_transition"])
        if transition is not None:
            outgoing.setdefault(transition["from_state"], set()).add(transition["to_state"])

    reached = {fsm["start_state"]}
    pending = [fsm["start_state"]]
    while pending:
        for state in outgoing.get(pending.pop(), ()):
            if state not in reached:
                reached.add(state)
                pending.append(state)

    return reached


def _check_every_commanding_motion_runs(motions, fsm, meta) -> None:
    """Motions and the states that run them have to cover each other.

    A motion reaches its state by firing the event that leaves it, so a motion that fires
    nothing is bound to nothing: the dispatch gets no case, the motion never steps, and the
    generated program runs its loop commanding whatever the drivers start at -- zero torque on
    a torque-controlled arm, which is an arm that falls.

    The same hole opens from the other side. A state the FSM can sit in with no motion bound to
    it renders no `case` either, and the loop keeps calling the driver every tick with whatever
    was staged last, so the arm is uncommanded for exactly as long as the FSM stays there. Only
    two states are allowed to run nothing: the end state, which the loop breaks on before it
    dispatches, and a state the heartbeat leaves, which the FSM does not dwell in.
    """
    namespace = meta["cpp_namespace"]
    orphaned = [motion.id for motion in motions if motion.controllers and not motion.fsm_state]
    if orphaned:
        raise ConstraintViolation(
            "coordination",
            f"{', '.join(orphaned)} command a robot but no state of the FSM '{namespace}' runs "
            "them. A motion is bound to the state its monitor's event leaves, so a motion that "
            "declares no monitor firing an FSM event is never stepped.",
        )

    passed_through = {transition["from"] for transition in meta["step_transitions"]}
    idle = sorted(
        _reachable_states(fsm)
        - {motion.fsm_state for motion in motions}
        - passed_through
        - {fsm["end_state"]}
    )
    if idle:
        raise ConstraintViolation(
            "coordination",
            f"the FSM '{namespace}' can be in {', '.join(idle)}, but no motion runs there. The "
            "dispatch gets no case for a state nothing is bound to, so the loop steps no motion "
            "while the FSM sits in it and the robot keeps whatever command was staged last -- "
            "zero torque, if nothing has run yet. Bind a motion to it with 'runs-in', or give "
            "the state a transition the heartbeat takes so the FSM passes straight through.",
        )


def _resolve_occurrence_events(motions, fsm, namespace) -> None:
    """Resolve each announced event to the enum token the generated FSM names it by.

    A monitor announces events it need not fire itself, so the tokens come from the FSM's own
    table rather than from the monitor's trigger.

    Raises:
        ConstraintViolation: a monitor announces an event no imported FSM declares, so there is
            no table to read the IRI from.
    """
    token_by_uri = {uri: token for token, uri in (fsm or {}).get("event_uris", {}).items()}
    for motion in motions:
        for phase in ("when", "while", "until"):
            for monitor in getattr(motion, f"{phase}_monitors"):
                ros = getattr(monitor, "ros", None)
                if ros is None or ros.occurrence_path is None:
                    continue
                tokens = []
                for uri in ros.occurrence_events:
                    token = token_by_uri.get(uri)
                    if token is None:
                        raise ConstraintViolation(
                            "coordination",
                            f"monitor '{monitor.id}' announces '{uri}', which no imported FSM "
                            "declares; an event with no IRI table has nothing to publish from",
                        )
                    tokens.append(token)
                ros.occurrence_events = tokens
                ros.occurrence_namespace = namespace


def _apply_reentry_events(motions, fsm) -> None:
    """A self-transition's fired events re-enter the state's motion.

    The model opts in by authoring `fires` on the retry reaction; the loop consumes the fired
    event and deactivates the motion, so entry runs again (snapshots re-capture, goals re-send).
    """
    tables = fsm or {}
    self_state = {
        row["id"]: row["from_state"]
        for row in tables.get("transitions_table", [])
        if row["from_state"] == row["to_state"]
    }
    fired_by_state: dict = {}
    for row in tables.get("reactions_table", []):
        state = self_state.get(row["do_transition"])
        if state is not None:
            fired_by_state.setdefault(state, set()).update(row["fires_events"])
    for motion in motions:
        motion.reentry_events = sorted(fired_by_state.get(motion.fsm_state, ()))
        motion.has_reentry_events = bool(motion.reentry_events)


def _when_gate_fallback(motion, monitor, units_by_motion):
    """The hold motion that runs while a WHEN-gated motion waits for its event.

    Raises:
        ConstraintViolation: the fallback names a motion two handlers realize, so nothing says
            which of them holds while the gated motion waits.
    """
    if not monitor.fallback_motion:
        raise ConstraintViolation(
            "coordination",
            f"WHEN monitor '{monitor.id}' on FSM-wired motion '{motion.id}' must declare a "
            "waiting hold motion (e.g. '... otherwise hold <hold-motion>'). A WHEN precondition "
            "without a fallback would leave the arm uncommanded while waiting.",
        )
    candidates = units_by_motion.get(monitor.fallback_motion, [])
    if not candidates:
        raise ConstraintViolation(
            "coordination",
            f"WHEN monitor '{monitor.id}' names unknown fallback motion "
            f"'{monitor.fallback_motion}'.",
        )
    if len(candidates) > 1:
        raise ConstraintViolation(
            "coordination",
            f"WHEN monitor '{monitor.id}' on motion '{motion.id}' falls back to "
            f"'{monitor.fallback_motion}', which is realized by more than one constraint "
            f"handler ({', '.join(sorted(unit.id for unit in candidates))}), so nothing says "
            "which of them holds while the gated motion waits. Name the handler's own motion.",
        )

    return candidates[0]


def _apply_fsm_gate_calls(motions, namespace) -> None:
    """Fold each hold motion's WHEN-evaluation gate calls.

    The gated motion's id plus its when-signature capability booleans, so the generated call is
    built from the same flags the function's own signature is.
    """
    if namespace is None:
        return
    by_id = {motion.id: motion for motion in motions}
    for fallback in motions:
        if not fallback.fsm_when_gate_motions:
            continue
        fallback.fsm_when_gate_calls = [
            {
                "gid": gate_id,
                "needs_state": by_id[gate_id].when_needs_state,
                "needs_shared": by_id[gate_id].when_needs_shared,
                "needs_robot": by_id[gate_id].when_needs_robot,
                "needs_events": by_id[gate_id].when_needs_events,
            }
            for gate_id in fallback.fsm_when_gate_motions
            if gate_id in by_id
        ]
