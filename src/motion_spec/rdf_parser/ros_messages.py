# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What the ROS interfaces a model names offer it, read off the rosidl classes: the fields an
action goal, result or message lets a model state, where a detection holds its pose, and what a
monitor publishes and answers with.
"""

from __future__ import annotations

import re

from motion_spec_dsl.rdf_parser.vocab import CSTR_EXT, CSTR_HDL, SENSORS
from rdf_utils.constraints import ConstraintViolation
from rdflib.namespace import RDF, RDFS
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.handlers import RosGoalAnswer, RosPublication
from motion_spec.rdf_parser import quantities

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
# The ROS types that carry a quantity published whole, per quantity type that has any.
_PAYLOAD_TYPES = {
    "Pose": (_POSE_TYPE, "geometry_msgs/Transform"),
    "VelocityTwist": ("geometry_msgs/Twist",),
    "Wrench": ("geometry_msgs/Wrench",),
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
    }


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
        "frame_path": f"{_sole(headers, f'{_HEADER_TYPE!r} fields', element_name)}.{_FRAME_FIELD}",
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
        offered = ", ".join(str(c) for c in candidates) or "none"
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


def _carried(root, carriers: tuple[str, ...]) -> str:
    """The one of a quantity's ROS types the message reaches -- itself, by descent, or in one
    entry of an array it holds -- or the first when it reaches none, so the descent names it."""
    holders = [root, *(entry for _path, entry in _repeated_leaves(root))]
    for carrier in carriers:
        if any(
            _type_name_of(holder) == carrier or _paths_to(holder, carrier) for holder in holders
        ):
            return carrier

    return carriers[0]


def _descend_to(message, wanted: str, where: str) -> str:
    """The path from `message` down to the one field of type `wanted`, empty if it is already it."""
    if _type_name_of(message) == wanted:
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
    repeated: list[str] = []

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
                    repeated.append(path)

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
    carriers = _PAYLOAD_TYPES.get(quantity_type)
    if carriers is None:
        raise ConstraintViolation(
            "communication",
            f"a '{quantity_type}' has no ROS type that carries it whole; a standing publish "
            f"reports {', '.join(_PAYLOAD_TYPES)}",
        )
    root = _message_class(type_name)
    wanted = _carried(root, carriers)
    package, _cpp_type, include = _cpp_names(root)
    shape = _shape_of(root, type_name, package, include)
    # A message reaching it nowhere and holding no array of anything reaches it nowhere: the
    # descent below says so about the quantity, which is what the model got wrong.
    carries_one = (
        _type_name_of(root) == wanted or _paths_to(root, wanted) or not _repeated_leaves(root)
    )
    entry = None if carries_one else _entry_shape(root, wanted, type_name)
    shape = {
        **shape,
        # Dotted prefix, empty when the message is the quantity and nothing else.
        "payload_path": "" if entry else _prefix(_descend_to(root, wanted, type_name)),
        # The frame the quantity is stated against is the message's to carry, when it has a header.
        "frame_path": _frame_path(root),
        "auto_time": [path for path, kind in shape["auto"].items() if kind == "time"],
        "auto_context_id": [path for path, kind in shape["auto"].items() if kind == "context_id"],
        "entry": entry,
        "carrier": _message_class(wanted).__name__,
    }

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
        "auto_time": [path for path, kind in shape["auto"].items() if kind == "time"],
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
    leaves = list(shape["leaves"])
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
            f"{', '.join(shape['leaves']) or 'none'}",
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


def ros_publication(model, node) -> dict:
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

    for row in graph.objects(node, RDFS.member):
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
            auto_time=[path for path, kind in auto.items() if kind == "time"],
            auto_context_id=[path for path, kind in auto.items() if kind == "context_id"],
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
            for member in graph.objects(node, RDFS.member)
            if (member, RDF.type, NS_MM_ROS["Action"]) in graph
        ),
        None,
    )
    if answer is None:
        return {}
    shape = action_shape(str(graph.value(answer, NS_MM_ROS["type-name"]) or ""))["result"]
    watched = set(graph.objects(node, CSTR_HDL["constraint"]))
    outcome, satisfied, fields = None, True, []
    for member in graph.objects(answer, RDFS.member):
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
            f"{' or '.join(_ANSWER_METHODS)}",
        )

    return {
        "answer": RosGoalAnswer(
            outcome,
            _ANSWER_METHODS[outcome],
            shape["cpp_type"],
            fields=fields,
            satisfied=satisfied,
            auto_time=[path for path, kind in shape["auto"].items() if kind == "time"],
            auto_context_id=[path for path, kind in shape["auto"].items() if kind == "context_id"],
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
