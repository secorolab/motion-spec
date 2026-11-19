# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The algorithm's data, its data access constraints, and what an analysis of them concludes.

In the terms of Bruyninckx, *Situational aware robotic and cyber-physical multi-agent systems*
(2026), §2.3: the computation indexes and the algorithm's D-blocks come first. Then the data access
constraints -- for every D-block, the F-block that writes it and those that read it, each through
a port -- as the model states them, built from the records. Last the analysis over those and the
schedules that run them: when each D-block is written and the storage that follows (Aho et al.,
*Compilers*, 2nd ed., §9.2). It never asks in what order a value is written -- that is
``operations.py``'s question.
"""

from __future__ import annotations

from dataclasses import dataclass, fields, is_dataclass
from enum import Enum
from typing import NamedTuple

from rdf_utils.models.vocab import (
    URI_TIME_PRED_AFTER_EVT,
    URI_TIME_PRED_OF_CONSTRAINT,
    URI_TIME_TYPE_AFTER_EVT,
)
from rdflib.namespace import PROV, RDF

from motion_spec.classes.geometry import Direction, Pose, Position, Wrench
from motion_spec.classes.handlers import EdgeMonitor
from motion_spec.classes.motion import DataValue
from motion_spec.classes.qudt import FreeVector, Quantity
from motion_spec.rdf_parser.operations import (
    data_reference_map,
    declaring_block,
    function_maps,
    function_output_ids,
    function_owner_map,
)
from motion_spec.rdf_parser.views import build_pose_components


@dataclass(frozen=True)
class ComputationIndexes:
    """Everything a motion asks about the computation, resolved once before any motion is built."""

    snapshot_source: dict
    snapshot_owner: dict
    snapshot_trigger: dict
    function_owner: dict
    function_output: dict
    function_input: dict
    data_reference: dict
    pose_components: dict


@dataclass(frozen=True)
class Computation:
    """What is computed this run: the functions, the values they read and write, the views onto
    those values, and every lookup a motion makes over the three.
    """

    functions: dict
    data_structures: list
    views: dict
    indexes: ComputationIndexes


def build_indexes(model, functions: dict, data_structures: list, views: dict) -> Computation:
    """Resolve every per-motion lookup once, before any motion is built.

    Raises:
        ConstraintViolation: a declared pose is missing a component, or an Euler pose has no
            component for every axis of its sequence.
    """
    graph = model.graph
    source: dict[str, str] = {}
    # snapshots share one slot: without an owner a motion retargets another's
    owner: dict[str, str] = {}
    trigger: dict[tuple[str, str], str] = {}
    for schedule in graph.subjects(RDF.type, URI_TIME_TYPE_AFTER_EVT):
        output_node = graph.value(schedule, URI_TIME_PRED_OF_CONSTRAINT)
        if output_node is None:
            continue
        source_node = graph.value(output_node, PROV.wasDerivedFrom)
        if source_node is None:
            continue
        output_id = model.id(output_node)
        source[output_id] = model.id(source_node)
        block = declaring_block(model, output_node)
        if block is None:
            continue
        owner[output_id] = model.id(block)
        trigger_node = graph.value(schedule, URI_TIME_PRED_AFTER_EVT)
        if trigger_node is not None:
            trigger[(owner[output_id], output_id)] = str(trigger_node)
    function_output, function_input = function_maps(functions)
    indexes = ComputationIndexes(
        snapshot_source=source,
        snapshot_owner=owner,
        snapshot_trigger=trigger,
        function_owner=function_owner_map(model, functions),
        function_output=function_output,
        function_input=function_input,
        data_reference=data_reference_map(data_structures, functions),
        pose_components=build_pose_components(model, views, data_structures),
    )

    return Computation(functions, data_structures, views, indexes)


def filter_algorithm_data(data_structures, users) -> list:
    """The data structures that are the algorithm's D-blocks: exactly those it reads or writes.

    Read off `users` -- the IR parts built before the algorithm data: schedules, functions, views,
    motions, solvers and publishes -- at any depth, and closed over what a kept record names in
    turn, so a value reached only through another (a reference value, a composed orientation's
    base pose) stays. A value nothing names, a scene element's own coordinates among them, is no
    runtime value at all.
    """
    by_id = {item.id: item for item in data_structures}
    referenced: set[str] = set()
    walked: set[int] = set()

    def visit(value) -> None:
        """Mark every data id named anywhere in `value`, and what each one names in turn."""
        if isinstance(value, str):
            if value in by_id and value not in referenced:
                referenced.add(value)
                visit(by_id[value])
            return
        if id(value) in walked:
            return
        walked.add(id(value))
        if isinstance(value, dict):
            for item in value.values():
                visit(item)
        elif isinstance(value, (list, tuple, set, frozenset)):
            for item in value:
                visit(item)
        elif is_dataclass(value) and not isinstance(value, type):
            for item in fields(value):
                visit(getattr(value, item.name))

    visit(users)

    return [item for item in data_structures if item.id in referenced]


# their initial literal is a fallback, not an authored constant
PORT_WRITERS = {
    "clock_time_s": {"kind": "port", "id": "clock"},
    "dt_measured_s": {"kind": "port", "id": "clock"},
}
_STATIC_WRITERS = {"authored", "sampled", "none"}
_TICK_WRITERS = {"port", "config", "action", "subscription"}
_STORAGE_BY_CADENCE = {"never": "absent", "init": "record", "tick": "log"}


def bound_id(value):
    """The id a binding names: the id itself, a record, a serialized record or an enum."""
    if value is None or isinstance(value, str):
        return value
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return value.get("id")
    return value.id


class _ValueOwners(NamedTuple):
    """Which motions write each value, and which values a per-motion block is responsible for."""

    owners: dict
    block_ids: dict


def owners_by_value(motions, functions: dict, serial_chain_solvers) -> _ValueOwners:
    """Which motions write each value, and which values the per-motion blocks -- rather than any
    schedule -- are responsible for.

    A union, never last-writer-wins: several motions can write one value, and attributing it to a
    single motion would gate away live data.

    The outputs in a solver's `world_output` are the ones the loop answers rather than a motion,
    written once per tick before the FSM is dispatched, so no motion owns them.
    """
    world_output_ids = {out.id for solver in serial_chain_solvers for out in solver.world_output}
    block_ids = {"pose": set(), "snapshot": set(), "decomposition": set(), "perturbation": set()}
    owned = []  # (data id, motion id, block kind or None)
    for motion in motions:
        for function_id in [
            *motion.when_schedule,
            *motion.while_pre_schedule,
            *motion.while_schedule,
            *motion.until_schedule,
        ]:
            for out_id in function_output_ids(functions.get(function_id) or {}):
                owned.append((out_id, motion.id, None))
        for solver in motion.serial_chain_solvers:
            for out in [*solver.output, *solver.gripper_joint_outputs]:
                # written every tick before the FSM; a motion owner would blank it elsewhere
                if out.id in world_output_ids:
                    continue
                owned.append((out.id, motion.id, None))
            for sample in [*solver.joint_space_samples, *solver.joint_space_cmd_samples]:
                owned.append((sample["id"], motion.id, None))
        for entry in motion.declared_pose_components:
            owned.append((entry["id"] if isinstance(entry, dict) else entry.id, motion.id, "pose"))
        for snapshot in motion.snapshots:
            owned.append((snapshot.target_id, motion.id, "snapshot"))
            for out_id in function_output_ids(functions.get(snapshot.source_function_id) or {}):
                owned.append((out_id, motion.id, None))
        for group in motion.pose_axis_error_groups:
            for component in group.components:
                owned.append((component.error, motion.id, "decomposition"))
        # cleared every tick, even with no motion selected
        for perturbation in motion.perturbations:
            block_ids["perturbation"].update((perturbation.applied_id, perturbation.active_id))

    owners: dict[str, set] = {}
    for data_id, motion_id, kind in owned:
        if not isinstance(data_id, str):
            continue
        owners.setdefault(data_id, set()).add(motion_id)
        if kind is not None:
            block_ids[kind].add(data_id)
    return _ValueOwners(owners, block_ids)


def superobjects_by_view(views) -> dict[str, set]:
    """Each value a view selects, to the values it is selected from: one subobject can MAP into
    several superobjects, and it is rewritten whenever any of them is."""
    superobjects_of: dict[str, set] = {}
    for view in (views or {}).values():
        if view.subobject.id == view.superobject.id:
            continue
        if view.subobject.id and view.superobject.id:
            superobjects_of.setdefault(view.subobject.id, set()).add(view.superobject.id)
    return superobjects_of


def build_data_access(
    algorithm_data: list,
    functions: dict,
    motions,
    serial_chain_solvers,
    tare_companions: dict,
    views,
    subscriptions=(),
    config_poses=(),
    platform_velocity_solvers=(),
    platform_force_solvers=(),
) -> dict:
    """The data access constraints: for every D-block, the block that writes it and the blocks
    that read it, each through a port.

    What the model states, built from its records; when a D-block is written, and the storage that
    follows, is `analyse_data_access`'s. `tare_companions` is `runtime_data`'s: the tare state ids
    per wrench reading id.
    """
    function_by_output: dict[str, set] = {}
    for function_id, function in functions.items():
        for out_id in function_output_ids(function):
            function_by_output.setdefault(out_id, set()).add(function_id)
    solver_by_output: dict[str, set] = {}
    sensor_outputs: set = set()
    for solver in serial_chain_solvers:
        gripper_outputs = [out for device in solver.devices for out in device.joint_outputs]
        for out in [*solver.output, *gripper_outputs]:
            solver_by_output.setdefault(out.id, set()).add(solver.id)
            if not isinstance(out, Wrench):
                continue
            if out.sensor_name:
                sensor_outputs.add(out.id)
            for companion_id in tare_companions.get(out.id, ()):
                solver_by_output.setdefault(companion_id, set()).add(solver.id)
                if out.sensor_name:
                    sensor_outputs.add(companion_id)
    for solver in platform_velocity_solvers:
        if solver.velocity.id:
            solver_by_output.setdefault(solver.velocity.id, set()).add(solver.id)
    for solver in platform_force_solvers:
        if solver.force.id:
            solver_by_output.setdefault(solver.force.id, set()).add(solver.id)
    block_ids = owners_by_value(motions, functions, serial_chain_solvers).block_ids
    perceived_by_output = {
        **{
            written_id: {"kind": "action", "id": client["act_id"]}
            for motion in motions
            for client in motion.action_clients
            for written_id in (
                client["status_id"],
                *(row["pose_id"] for row in client["written_poses"]),
                *(
                    row["observed_at_id"]
                    for row in client["written_poses"]
                    if row.get("observed_at_id")
                ),
            )
        },
        **{
            written_id: {"kind": "subscription", "id": sub["sub_id"]}
            for sub in subscriptions
            for row in sub["written_poses"]
            for written_id in (
                row["pose_id"],
                *((row["observed_at_id"],) if row.get("observed_at_id") else ()),
            )
        },
    }
    config_pose_ids = {entry["id"] for entry in config_poses}

    data_access = {}
    for item in algorithm_data:
        if not item.id:
            continue
        block = next((kind for kind, ids in block_ids.items() if item.id in ids), None)
        if item.id in config_pose_ids:
            writer = {"kind": "config", "id": item.id}
        elif item.id in PORT_WRITERS:
            writer = PORT_WRITERS[item.id]
        elif item.id in perceived_by_output:
            writer = perceived_by_output[item.id]
        elif isinstance(item, DataValue) and item.role == "joint_space":
            writer = item.writer
        elif item.id in function_by_output:
            writers = function_by_output[item.id]
            types = {(functions.get(cid) or {}).get("type") for cid in writers}
            kind = "controller" if types == {"Controller"} else "function"
            writer = {"kind": kind, "id": next(iter(writers)) if len(writers) == 1 else None}
            if writer["id"] is not None:
                writer["port"] = next(
                    port
                    for value, port in bound_ids(functions[writer["id"]], "")
                    if value == item.id
                )
        elif item.id in solver_by_output:
            kind = "sensor" if item.id in sensor_outputs else "solver"
            solvers = solver_by_output[item.id]
            writer = {"kind": kind, "id": next(iter(solvers)) if len(solvers) == 1 else None}
        elif block is not None:
            writer = {"kind": block, "id": item.id}
        elif isinstance(item, Quantity) and item.sampled:
            writer = {"kind": "sampled", "id": item.id}
        # an authored 0.0 is a value
        elif (
            isinstance(item, (Position, Pose))
            and item.position is not None
            or isinstance(item, Direction)
            and item.direction is not None
            or isinstance(item, (Quantity, DataValue))
            and item.value is not None
            or isinstance(item, FreeVector)
            and item.vector is not None
        ):
            writer = {"kind": "authored", "id": None}
        else:
            writer = {"kind": "none", "id": None}
        data_access[item.id] = {"write": writer}

    for subobject_id, superobject_ids in superobjects_by_view(views).items():
        entry = data_access.get(subobject_id)
        kinds = [data_access[sid]["write"]["kind"] for sid in superobject_ids if sid in data_access]
        if entry is None or not kinds:
            continue
        if entry["write"]["kind"] == "none" or any(kind not in _STATIC_WRITERS for kind in kinds):
            entry["write"] = {
                "kind": "view",
                "id": next(iter(superobject_ids)) if len(superobject_ids) == 1 else None,
            }

    # by IRI: two motions' until terms may share a short id
    errors_by_constraint = {
        function["constraint_uri"]: bound_id(function.get("error"))
        for function in functions.values()
        if function.get("type") == "ErrorEvaluator" and function.get("constraint_uri")
    }
    reads = []  # (data id, reader kind, reader id, port)
    for motion in motions:
        for monitor in (*motion.when_monitors, *motion.while_monitors, *motion.until_monitors):
            reads.append((bound_id(monitor.error), "monitor", monitor.id, "error"))
            # else the band is a D-block nothing reads
            reads.append((bound_id(monitor.tolerance), "monitor", monitor.id, "tolerance"))
            for _member, uri, band in zip(
                monitor.group_constraint_ids,
                monitor.group_constraint_uris,
                monitor.group_constraint_tolerances or [""] * len(monitor.group_constraint_ids),
                strict=False,
            ):
                reads.append((errors_by_constraint.get(uri), "monitor", monitor.id, "error"))
                reads.append((band or None, "monitor", monitor.id, "tolerance"))
            if isinstance(monitor, EdgeMonitor):
                reads.append((monitor.debounce_id, "monitor", monitor.id, "debounce"))
        for controller in motion.controllers:
            for port, value in (
                ("error_signal", controller.error_signal),
                ("measured_signal", controller.measured_signal),
                ("setpoint_signal", controller.setpoint_signal),
                ("tolerance_signal", controller.tolerance_id or None),
            ):
                reads.append((bound_id(value), "controller", controller.id, port))
        for snapshot in motion.task_snapshots:
            reads.append((snapshot.target_id, "motion", motion.id, "snapshot.target"))
            reads.append((snapshot.captured_id, "motion", motion.id, "snapshot.captured"))
        # no function binds a composed orientation's base pose
        for entry in motion.declared_pose_components:
            reads += [
                (operand.get("pose"), "motion", motion.id, "pose.orientation.base")
                for operand in entry.get("orientation_operands") or ()
            ]
    for function_id, function in functions.items():
        outputs = function_output_ids(function) | set(function.get("errors") or ())
        reads += [
            (value, "function", function_id, port)
            for value, port in bound_ids(function, "")
            if value not in outputs
        ]
    for solver in serial_chain_solvers:
        reads.append((solver.gravity_source, "solver", solver.id, "gravity"))
        saturation = solver.torque_saturation
        if saturation is not None:
            for bound, quantity in (
                ("maximum", saturation.maximum),
                ("lower", saturation.lower),
                ("upper", saturation.upper),
            ):
                if quantity is not None:
                    reads.append((quantity.id, "solver", solver.id, f"torque_saturation.{bound}"))
    readers_by_id: dict[str, list] = {}
    for data_id, kind, reader_id, port in reads:
        reader = {"kind": kind, "id": reader_id, "port": port}
        if isinstance(data_id, str) and reader_id:
            readers = readers_by_id.setdefault(data_id, [])
            if reader not in readers:
                readers.append(reader)

    for data_id, readers in readers_by_id.items():
        if data_id in data_access:
            data_access[data_id]["read"] = readers

    return data_access


def analyse_data_access(
    data_access: dict, algorithm_data: list, motions, functions: dict, serial_chain_solvers, views
) -> dict:
    """When each value is written, and the storage that follows: a liveness analysis of the data
    flow over the motions that run it (Aho et al., *Compilers*, 2nd ed., §9.2).

    Applied, not only reported: a value never written leaves the algorithm data. The
    returned `{id: {cadence, storage}}` is what a reader of the run taps -- a value written once is
    recorded once, a value written per tick is logged, gated by the motions that write it.

    Raises:
        RuntimeError: a value is read but never written -- a broken binding that would feed its
            reader a zero forever.
    """
    owners = owners_by_value(motions, functions, serial_chain_solvers).owners
    items = {item.id: item for item in algorithm_data if item.id}
    analysis = {}
    for value_id, entry in data_access.items():
        kind = entry["write"]["kind"]
        if kind == "view":
            continue
        motion_ids = owners.get(value_id)
        # motions' order keeps the gating deterministic
        owned = (
            {"motions": [motion.id for motion in motions if motion.id in motion_ids]}
            if motion_ids
            else "tick"
        )
        item = items[value_id]
        if isinstance(item, DataValue) and item.role == "joint_space":
            cadence = owned
        elif kind in _TICK_WRITERS:
            cadence = "tick"
        elif kind in {"authored", "sampled"}:
            cadence = "init"
        elif kind == "none":
            cadence = "never"
        else:
            cadence = owned
        analysis[value_id] = {"cadence": cadence}

    superobjects_of = superobjects_by_view(views)
    pending = [vid for vid, entry in data_access.items() if entry["write"]["kind"] == "view"]
    while pending:
        waiting = []
        for value_id in pending:
            sources = [
                analysis[sid]["cadence"] for sid in superobjects_of[value_id] if sid in analysis
            ]
            if not sources:
                waiting.append(value_id)
                continue
            live = [
                cadence for cadence in sources if isinstance(cadence, dict) or cadence == "tick"
            ]
            if "tick" in live:
                cadence = "tick"
            elif live:
                cadence = {
                    "motions": [
                        motion.id
                        for motion in motions
                        if any(motion.id in source["motions"] for source in live)
                    ]
                }
            else:
                cadence = sources[0]
            analysis[value_id] = {"cadence": cadence}
        if len(waiting) == len(pending):
            raise RuntimeError(f"data access: views with no written superobject: {waiting}")
        pending = waiting

    for entry in analysis.values():
        cadence = entry["cadence"]
        entry["storage"] = "log" if isinstance(cadence, dict) else _STORAGE_BY_CADENCE[cadence]

    orphans = {
        value_id: entry["read"]
        for value_id, entry in data_access.items()
        if analysis[value_id]["storage"] == "absent" and entry.get("read")
    }
    if orphans:
        raise RuntimeError(
            "data access: read but never written: "
            + "; ".join(
                f"{value_id} (read by {', '.join(reader['id'] for reader in readers)})"
                for value_id, readers in orphans.items()
            )
        )

    dead = {value_id for value_id, entry in analysis.items() if entry["storage"] == "absent"}
    algorithm_data[:] = [item for item in algorithm_data if item.id not in dead]
    for value_id in dead:
        del data_access[value_id]
        del analysis[value_id]
    return analysis


def bound_ids(value, path: str):
    """Every string a reader record binds, with the dotted field path that named it.

    Nested, because a record does not keep all its bindings at the top: a controller function
    holds its gains in a `gains` sub-dict and its integral bounds as whole quantity records
    under `integral_saturation`, and a flat scan reports both as read by nobody. A nested
    record's own `id` is the value the field binds; the record's own `id` and `type` are not.

    A function holds some of those sub-records as the dataclass the parser built (a controller's
    saturation bounds), so a record reads the same here whether or not it has been serialized.
    """
    if is_dataclass(value) and not isinstance(value, type):
        value = vars(value)
    if isinstance(value, str):
        yield value, path
    elif isinstance(value, dict):
        if path and isinstance(value.get("id"), str):
            yield value["id"], path
        for key, item in value.items():
            if key not in {"id", "type"}:
                yield from bound_ids(item, f"{path}.{key}" if path else key)
    elif isinstance(value, (list, tuple)):
        for item in value:
            yield from bound_ids(item, path)
