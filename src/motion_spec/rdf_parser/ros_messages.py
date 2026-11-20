# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What the ROS interfaces a model names offer it, read off the rosidl classes: the fields an
action goal, result or message lets a model state, where a detection holds its pose, and what a
monitor publishes and answers with.
"""

from __future__ import annotations

from motion_spec_dsl.vocab import CSTR_EXT, CSTR_HDL, SENSORS
from rdf_utils.constraints import ConstraintViolation
from rdflib.namespace import RDF, RDFS
from scene_dsl.rdf_parser.vocab import NS_MM_ROS

from motion_spec.classes.handlers import RosGoalAnswer, RosPublication
from motion_spec.rdf_parser import quantities

# node-owned fields, never authored
AUTO_TIME_TYPE = "builtin_interfaces/Time"
AUTO_CONTEXT_ID = ("scenario_context_id", "unique_identifier_msgs/UUID")
# any other stated field type must read as a number
STRING_TYPES = ("string", "wstring")
HEADER_TYPE = "std_msgs/Header"
FRAME_FIELD = "frame_id"
POSE_TYPE = "geometry_msgs/Pose"
PAYLOAD_TYPES = {
    "Pose": (POSE_TYPE, "geometry_msgs/Transform"),
    "VelocityTwist": ("geometry_msgs/Twist",),
    "Wrench": ("geometry_msgs/Wrench",),
}


def message_class(type_name: str):
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
    action = action_class(type_name)
    package, cpp_type, include = cpp_names(action)

    return {
        "package": package,
        "cpp_type": cpp_type,
        "include": include,
        "goal": shape_of(action.Goal, f"{type_name} goal", package, include),
        "result": shape_of(action.Result, f"{type_name} result", package, include),
    }


def cpp_names(message) -> tuple[str, str, str]:
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


def action_class(type_name: str):
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
    action = action_class(type_name)
    goal_targets = [
        name
        for name, element, many, nested in fields(action.Goal)
        if many and element in STRING_TYPES
    ]
    detections_path, detection = sole(
        repeated_leaves(action.Result), "repeated message fields", f"{type_name} result"
    )

    return {
        "targets_path": sole(goal_targets, "repeated string fields", f"{type_name} goal"),
        "detections_path": detections_path,
        **detection_paths(detection, pose_path),
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
    root = message_class(type_name)
    package, cpp_type, include = cpp_names(root)
    detections_path, detection = sole(repeated_leaves(root), "repeated message fields", type_name)

    return {
        "package": package,
        "cpp_type": cpp_type,
        "include": include,
        "detections_path": detections_path,
        **detection_paths(detection, pose_path),
    }


def detection_paths(detection, pose_path: str) -> dict:
    """Per detection, which object it is, which frame it arrived in, and the accessor to the pose
    it reports, off the two fields the model named.

    Raises:
        ConstraintViolation: the detection offers not exactly one string field or one header, or
            the model states no pose, names a field the detection does not repeat, or names one
            holding no single pose.
    """
    element_name = type_name_of(detection)
    detection_fields = fields(detection)
    ids = [
        name
        for name, element, many, _nested in detection_fields
        if not many and element in STRING_TYPES
    ]
    id_path = sole(ids, "string fields", element_name)
    headers = [
        name for name, element, _many, _nested in detection_fields if element == HEADER_TYPE
    ]
    header = sole(headers, f"'{HEADER_TYPE}' fields", element_name)
    container, _, field = pose_path.partition(".")
    if not field:
        raise ConstraintViolation(
            "communication",
            f"'{element_name}' reaches more than one pose, so the act must say which one it "
            "reads: `<field> from <container>`",
        )
    entries = [
        element
        for name, element, many, nested in detection_fields
        if name == container and many and nested
    ]
    if not entries:
        raise ConstraintViolation(
            "communication",
            f"'{element_name}' has no repeated message field '{container}' for a pose to come from",
        )
    entry = message_class(entries[0])
    carried = [
        element
        for name, element, many, nested in fields(entry)
        if name == field and not many and nested
    ]
    if not carried:
        raise ConstraintViolation(
            "communication",
            f"'{type_name_of(entry)}' has no message field '{field}' to read a pose from",
        )
    # the named field may wrap the pose, e.g. with a covariance
    descent = descend_to(message_class(carried[0]), POSE_TYPE, f"{container}.{field}")

    return {
        "id_path": id_path,
        "frame_path": f"{header}.{FRAME_FIELD}",
        "pose_path": f"{container}[0].{field}{'.' + descent if descent else ''}",
        "pose_container": container,
    }


def sole(candidates: list, what: str, where: str):
    """The one candidate, or the error naming what was offered instead."""
    if len(candidates) != 1:
        offered = ", ".join(str(c) for c in candidates) or "none"
        raise ConstraintViolation(
            "communication", f"'{where}' offers {len(candidates)} {what}: {offered}"
        )
    return candidates[0]


def fields(message) -> list[tuple[str, str, bool, bool]]:
    """`(name, element type, repeated, a message)` per field the message declares, read off the
    types rosidl parsed it into: a message element is named `pkg/Type`, a string `string` or
    `wstring` whatever its bound, any other primitive by its rosidl type name.
    """
    from rosidl_parser.definition import (
        AbstractNestedType,
        AbstractString,
        AbstractWString,
        NamespacedType,
    )

    found = []
    for name, slot in zip(message.get_fields_and_field_types(), message.SLOT_TYPES, strict=True):
        many = isinstance(slot, AbstractNestedType)
        element = slot.value_type if many else slot
        nested = isinstance(element, NamespacedType)
        if nested:
            type_name = f"{element.namespaced_name()[0]}/{element.name}"
        elif isinstance(element, AbstractString):
            type_name = "string"
        elif isinstance(element, AbstractWString):
            type_name = "wstring"
        else:
            type_name = element.typename
        found.append((name, type_name, many, nested))
    return found


def repeated_leaves(message) -> list[tuple[str, object]]:
    """Every repeated field the message reaches, and the class one entry of each carries.

    Descends through nested messages so a result that wraps its list in an array message -- the
    `vision_msgs` convention -- is found at whatever depth it sits.
    """
    found = []

    def walk(node, prefix: str) -> None:
        for name, element, many, nested in fields(node):
            path = f"{prefix}{name}"
            if many and nested:
                found.append((path, message_class(element)))
            elif not many and nested and element != HEADER_TYPE:
                walk(message_class(element), f"{path}.")

    walk(message, "")

    return found


def paths_to(message, wanted: str) -> list[str]:
    """Every path from `message` down to a field of type `wanted`.

    Repeated fields are not descended into: what one entry of an array holds is a question about
    the entry, asked of the entry.
    """
    found = []

    def walk(node, prefix: str) -> None:
        for name, element, many, nested in fields(node):
            if many or not nested:
                continue
            path = f"{prefix}{name}"
            if element == wanted:
                found.append(path)
            else:
                walk(message_class(element), f"{path}.")

    walk(message, "")

    return found


def descend_to(message, wanted: str, where: str) -> str:
    """The path from `message` down to the one field of type `wanted`, empty if it is already it."""
    if type_name_of(message) == wanted:
        return ""

    return sole(paths_to(message, wanted), f"'{wanted}' fields", where)


def type_name_of(message) -> str:
    """`pkg/Type` for a rosidl class, as a field type spells it."""
    package, _subfolder, _module = message.__module__.split(".")

    return f"{package}/{message.__name__}"


def shape_of(root, type_name: str, package: str, include: str) -> dict:
    """What a message class offers a model: the leaf fields it may state, the owning message
    class of each (its constants live there), and the fields the node auto-fills.
    """
    leaves: dict[str, tuple[str, object]] = {}
    auto: dict[str, str] = {}
    repeated: list[str] = []

    def walk(message, prefix: str) -> None:
        for name, element, many, nested in fields(message):
            path = f"{prefix}{name}"
            if not many and element == AUTO_TIME_TYPE:
                auto[path] = "time"
            elif not many and (name, element) == AUTO_CONTEXT_ID:
                auto[path] = "context_id"
            elif not many and nested:
                walk(message_class(element), f"{path}.")
            else:
                leaves[path] = (element, message)
                if many:
                    repeated.append(path)

    walk(root, "")

    return {
        "type_name": type_name,
        "package": package,
        "cpp_type": cpp_names(root)[1],
        "include": include,
        "leaves": leaves,
        "auto": auto,
        "repeated": repeated,
    }


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
    carriers = PAYLOAD_TYPES.get(quantity_type)
    if carriers is None:
        raise ConstraintViolation(
            "communication",
            f"a '{quantity_type}' has no ROS type that carries it whole; a standing publish "
            f"reports {', '.join(PAYLOAD_TYPES)}",
        )
    root = message_class(type_name)
    arrays = repeated_leaves(root)
    holders = [root, *(entry for _path, entry in arrays)]
    for carrier in carriers:
        if any(type_name_of(holder) == carrier or paths_to(holder, carrier) for holder in holders):
            wanted = carrier
            break
    else:
        wanted = carriers[0]
    package, _cpp_type, include = cpp_names(root)
    shape = shape_of(root, type_name, package, include)
    if type_name_of(root) == wanted or paths_to(root, wanted) or not arrays:
        entry = None
        payload_descent = descend_to(root, wanted, type_name)
        payload_path = f"{payload_descent}." if payload_descent else ""
    else:
        payload_path = ""
        entry_path, entry_class = sole(arrays, "repeated message fields", type_name)
        entry_name = type_name_of(entry_class)
        entry_package, entry_cpp_type, entry_include = cpp_names(entry_class)
        entry_fields = shape_of(entry_class, entry_name, entry_package, entry_include)
        ids = [
            name
            for name, element, many, nested in fields(entry_class)
            if not many and element in STRING_TYPES
        ]
        entry_descent = descend_to(entry_class, wanted, entry_name)
        entry = {
            "path": entry_path,
            "type_name": entry_name,
            "cpp_type": entry_cpp_type,
            "payload_path": f"{entry_descent}." if entry_descent else "",
            "id_path": sole(ids, "string fields", entry_name),
            "frame_path": frame_path(entry_class),
            "auto_time": [path for path, kind in entry_fields["auto"].items() if kind == "time"],
        }

    return {
        **shape,
        "payload_path": payload_path,
        "frame_path": frame_path(root),
        "auto_time": [path for path, kind in shape["auto"].items() if kind == "time"],
        "auto_context_id": [path for path, kind in shape["auto"].items() if kind == "context_id"],
        "entry": entry,
        "carrier": message_class(wanted).__name__,
    }


def frame_path(message) -> str | None:
    """Where the message states the frame it holds, when it carries a header at all."""
    headers = [
        name
        for name, element, many, nested in fields(message)
        if not many and element == HEADER_TYPE
    ]

    return f"{headers[0]}.{FRAME_FIELD}" if headers else None


def sole_payload_path(shape: dict) -> str:
    """The one field the sugar form means, once the auto-filled ones are set aside."""
    leaves = list(shape["leaves"])
    if len(leaves) != 1:
        raise ConstraintViolation(
            "communication",
            f"message type '{shape['type_name']}' carries {len(leaves)} payload fields, so a "
            "publish must name the field it writes: `publish: to <topic> { path: value }`",
        )
    return leaves[0]


def cpp_value(shape: dict, path: str, text: str) -> str:
    """The C++ the authored value renders to, against the field that owns it: a constant the
    message class defines, a quoted string, or a number the field's type accepts.
    """
    element, owner = shape["leaves"][path]
    if hasattr(owner, text) and text not in owner.get_fields_and_field_types():
        _package, owner_cpp, _include = cpp_names(owner)
        return f"{owner_cpp}::{text}"
    if element in STRING_TYPES:
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
    path = path or sole_payload_path(shape)
    if path not in shape["leaves"]:
        raise ConstraintViolation(
            "communication",
            f"'{path}' is not a payload field of '{shape['type_name']}'; it offers "
            f"{', '.join(shape['leaves']) or 'none'}",
        )
    return {"path": path, "cpp_value": cpp_value(shape, path, text)}


def publish_rate(model, rate_node, publisher: str) -> float:
    """The rate in Hz a publish states, PUBLISHER naming it in the error.

    Raises:
        ConstraintViolation: the rate is not positive.
    """
    rate_hz = quantities.quantity(model, rate_node).value
    if not rate_hz or rate_hz <= 0.0:
        raise ConstraintViolation(
            "communication",
            f"{publisher} publishes at {rate_hz} Hz; a rate says how often, so it is positive",
        )

    return rate_hz


def goal_answer(model, node, answer_node, watched: set) -> RosGoalAnswer:
    """How a monitor answers the goal the run serves: the outcome, whether it answers when the
    WATCHED constraint is satisfied, and the result fields it sets.

    Raises:
        ConstraintViolation: the outcome is not one a run answers with.
    """
    graph = model.graph
    result_shape = action_shape(str(graph.value(answer_node, NS_MM_ROS["type-name"]) or ""))[
        "result"
    ]
    outcome, satisfied, result_fields = None, True, []
    for member in graph.objects(answer_node, RDFS.member):
        result_path = graph.value(member, NS_MM_ROS["field-path"])
        if result_path is None:
            outcome = str(graph.value(member, RDF.value))
            satisfied = set(graph.objects(member, CSTR_EXT["has-constraint"])) == watched
            continue
        result_fields.append(
            publish_field(result_shape, str(result_path), str(graph.value(member, RDF.value)))
        )
    if outcome not in ANSWER_METHOD_BY_STATUS:
        raise ConstraintViolation(
            "communication",
            f"monitor '{model.id(node)}' answers its goal '{outcome}'; a run answers "
            f"{' or '.join(ANSWER_METHOD_BY_STATUS)}",
        )

    return RosGoalAnswer(
        outcome,
        ANSWER_METHOD_BY_STATUS[outcome],
        result_shape["cpp_type"],
        fields=result_fields,
        satisfied=satisfied,
        auto_time=[path for path, kind in result_shape["auto"].items() if kind == "time"],
        auto_context_id=[
            path for path, kind in result_shape["auto"].items() if kind == "context_id"
        ],
    )


def ros_publication(model, node) -> dict:
    """What a monitor publishes, when the model asks it to publish at all.

    Each row states its field, its value, and the condition it holds under: a row conditioned on
    the constraint the monitor watches is satisfied; an unconditioned row is the otherwise.
    """
    graph = model.graph
    answer_node = next(
        (
            member
            for member in graph.objects(node, RDFS.member)
            if (member, RDF.type, NS_MM_ROS["Action"]) in graph
        ),
        None,
    )
    watched = set(graph.objects(node, CSTR_HDL["constraint"]))
    answer = {}
    if answer_node is not None:
        answer = {"answer": goal_answer(model, node, answer_node, watched)}
    channel = graph.value(node, NS_MM_ROS["channel-name"])
    if channel is None:
        return answer
    type_name = str(graph.value(node, NS_MM_ROS["type-name"]) or "")
    root = message_class(type_name)
    package, _cpp_type, include = cpp_names(root)
    shape = shape_of(root, type_name, package, include)
    auto = shape["auto"]
    on_satisfied: list[dict] = []
    on_violated: list[dict] = []
    occurrence_path = None
    occurrence_events: list[str] = []

    for row in graph.objects(node, RDFS.member):
        if (row, RDF.type, NS_MM_ROS["Action"]) in graph:
            continue
        # no authored value: an announced occurrence, not a field row
        if graph.value(row, RDF.value) is None:
            occurrence_path = sole_payload_path(shape)
            element, _owner = shape["leaves"][occurrence_path]
            if element not in STRING_TYPES:
                raise ConstraintViolation(
                    "communication",
                    f"monitor '{model.id(node)}' publishes an occurrence on "
                    f"'{shape['type_name']}', whose field '{occurrence_path}' is a '{element}'; an "
                    "occurrence carries the event's IRI, so the message must offer a string to "
                    "hold it",
                )
            occurrence_events.append(str(row))
            continue
        conditions = set(graph.objects(row, CSTR_EXT["has-constraint"]))
        if conditions and conditions != watched:
            raise ConstraintViolation(
                "communication",
                f"publish row '{model.id(row)}' states a condition that is not the constraint "
                "its monitor watches",
            )
        polarity = on_satisfied if conditions else on_violated
        polarity.append(
            publish_field(
                shape,
                str(graph.value(row, NS_MM_ROS["field-path"]) or ""),
                str(graph.value(row, RDF.value)),
            )
        )

    # unstated is every cycle
    rate_hz = None
    rate_node = graph.value(node, SENSORS["update-rate"])
    if rate_node is not None:
        rate_hz = publish_rate(model, rate_node, f"monitor '{model.id(node)}'")

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
            rate_hz=rate_hz,
        ),
    }


# a cancel is the client's to ask for, never an answer
ANSWER_METHOD_BY_STATUS = {"STATUS_SUCCEEDED": "succeed", "STATUS_ABORTED": "abort"}
