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


class _SnapshotMaps(NamedTuple):
    """The three things a snapshot output is looked up by."""

    source: dict
    owner: dict
    trigger: dict


def _snapshot_maps(model) -> _SnapshotMaps:
    """One walk over the snapshots for the three maps codegen asks about.

    ``source``: each snapshot output to its source quantity. ``owner``: each output to the block
    whose context declares it (`prov:hadMember`) -- every motion captures each snapshot it
    references and they share one slot, so without an owner a motion silently retargets another's.
    ``trigger``: each event-triggered snapshot to its trigger event's IRI, keyed by
    (declaring block, output id) -- a motion's own re-samples only for it, a shared one for
    whichever motion reads it.
    """
    graph = model.graph
    source: dict[str, str] = {}
    owner: dict[str, str] = {}
    trigger: dict[tuple[str, str], str] = {}
    for schedule in graph.subjects(RDF.type, URI_TIME_TYPE_AFTER_EVT):
        output_node = graph.value(schedule, URI_TIME_PRED_OF_CONSTRAINT)
        if output_node is None:
            continue
        source_node = graph.value(output_node, PROV.wasDerivedFrom)
        if source_node is None:
            # A sensor tare: scheduled the same way, but nothing is derived.
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

    return _SnapshotMaps(source, owner, trigger)


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
    source, owner, trigger = _snapshot_maps(model)
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
    # Records are shared between motions; one walk through each is enough.
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


# D-blocks the control loop writes from a backend port, not from any model entity. Their
# initial literal is a fallback, not an authored constant, so their writer is stated here rather
# than inferred from the value being present.
PORT_WRITERS = {
    "clock_time_s": {"kind": "port", "id": "clock"},
    "dt_measured_s": {"kind": "port", "id": "clock"},
}
# Producers that write before the loop starts, or never: a view onto such a value is not rewritten.
_STATIC_WRITERS = {"authored", "sampled", "none"}
# Producers that write on every tick whichever motion runs: a backend port, the deployment config,
# and the channels a perception arrives on.
_TICK_WRITERS = {"port", "config", "action", "subscription"}
# Storage follows from cadence, in one place. A value written once says nothing new when repeated
# per tick, and one never written is not a runtime value at all.
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


def _sole(ids) -> str | None:
    """The one id in a set, or None when several instances write the same value."""
    return next(iter(ids)) if len(ids) == 1 else None


class _SolverWrites(NamedTuple):
    """Which solver writes each output, and which of those are sensor readings."""

    by_output: dict
    sensor_outputs: set


def _writers_by_output(serial_chain_solvers) -> _SolverWrites:
    """Which solver writes each output, and which of those are sensor readings (the tare
    companions are written alongside the reading, not measured).
    """
    by_output: dict[str, set] = {}
    sensor_outputs: set = set()
    for solver in serial_chain_solvers:
        # Full solvers carry gripper outputs on their gripper device(s), not as a flat list.
        gripper_outputs = [out for device in solver.devices for out in device.joint_outputs]
        for out in [*solver.output, *gripper_outputs]:
            by_output.setdefault(out.id, set()).add(solver.id)
            if not isinstance(out, Wrench):
                continue
            if out.estimator is not None:
                # payload tare, written alongside the estimate (runtime.runtime_data)
                for suffix in ("est_payload", "est_payload_new", "est_settle", "est_tares"):
                    by_output.setdefault(f"{out.id}_{suffix}", set()).add(solver.id)
            if not out.sensor_name:
                continue
            sensor_outputs.add(out.id)
            # tare state, written alongside the reading (runtime.runtime_data)
            for companion in (
                f"{out.id}_ft_raw",
                f"{out.id}_ft_bias",
                f"{out.id}_ft_bias_new",
                f"{out.id}_ft_bias_prev",
                f"{out.id}_ft_load",
                f"{out.id}_ft_payload",
                f"{out.id}_ft_settle",
                f"{out.id}_ft_tares",
                f"{out.id}_ft_rejects",
                f"{out.id}_ft_confirming",
            ):
                by_output.setdefault(companion, set()).add(solver.id)
                sensor_outputs.add(companion)

    return _SolverWrites(by_output, sensor_outputs)


class _ValueOwners(NamedTuple):
    """Which motions write each value, and which values a per-motion block is responsible for."""

    owners: dict
    block_ids: dict


def _owners_by_value(motions, functions: dict, serial_chain_solvers) -> _ValueOwners:
    """Which motions write each value, and which values the per-motion blocks -- rather than any
    schedule -- are responsible for.

    A union, never last-writer-wins: several motions can write one value, and attributing it to a
    single motion would gate away live data.

    The outputs in a solver's `world_output` are the ones the loop answers rather than a motion,
    written once per tick before the FSM is dispatched, so no motion owns them.
    """
    world_output_ids = {out.id for solver in serial_chain_solvers for out in solver.world_output}
    # Poses, snapshots and per-axis errors are written by the pose-composition, snapshot and
    # error-decomposition blocks, which are emitted per motion rather than scheduled as functions.
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
                # A world observation is not a motion's work: _split_outputs sorts every output
                # the loop answers into solver.world_output, and world-state-block writes those
                # once per tick, before the FSM is dispatched. Attributing one to whichever
                # motions happen to read it gates its frame-log slot behind active_motion and
                # blanks it in every other state -- for an FT wrench, exactly where a phantom
                # bias has to be read.
                if out.id in world_output_ids:
                    continue
                owned.append((out.id, motion.id, None))
            # Joint-space mirrors are written by whichever motion's solver ran, so the runtime's
            # channels are live in every motion that drives it.
            for sample in [*solver.joint_space_samples, *solver.joint_space_cmd_samples]:
                owned.append((sample["id"], motion.id, None))
        for entry in motion.declared_pose_components:
            owned.append((entry["id"] if isinstance(entry, dict) else entry.id, motion.id, "pose"))
        for snapshot in motion.snapshots:
            owned.append((snapshot.target_id, motion.id, "snapshot"))
            # The source function runs inside the snapshot block, not from a schedule.
            for out_id in function_output_ids(functions.get(snapshot.source_function_id) or {}):
                owned.append((out_id, motion.id, None))
        for group in motion.pose_axis_error_groups:
            for component in group.components:
                owned.append((component.error, motion.id, "decomposition"))
        # Not owned by the motion: the applied wrench is cleared every tick by the run, including
        # the ticks after its state has exited and no motion is selected at all.
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


def _superobjects_by_view(views) -> dict[str, set]:
    """Each value a view selects, to the values it is selected from: one subobject can MAP into
    several superobjects, and it is rewritten whenever any of them is."""
    superobjects_of: dict[str, set] = {}
    for view in (views or {}).values():
        if view.subobject.id and view.superobject.id:
            superobjects_of.setdefault(view.subobject.id, set()).add(view.superobject.id)
    return superobjects_of


def build_data_access(
    algorithm_data: list,
    functions: dict,
    motions,
    serial_chain_solvers,
    views,
    subscriptions=(),
    config_poses=(),
    platform_velocity_solvers=(),
    platform_force_solvers=(),
) -> dict:
    """The data access constraints: for every D-block, the block that writes it and the blocks
    that read it, each through a port.

    What the model states, built from its records; when a D-block is written, and the storage that
    follows, is `analyse_data_access`'s.
    """
    function_by_output: dict[str, set] = {}
    for function_id, function in functions.items():
        for out_id in function_output_ids(function):
            function_by_output.setdefault(out_id, set()).add(function_id)
    solver_by_output, sensor_outputs = _writers_by_output(serial_chain_solvers)
    # A platform's twist is composed from the measured hub rates every tick, not forwarded along
    # a chain, so its writer is the composition solver itself. The component views the
    # constraints read inherit that from the twist, as every other view does.
    for solver in platform_velocity_solvers:
        if solver.velocity.id:
            solver_by_output.setdefault(solver.velocity.id, set()).add(solver.id)
    # The commanded wrench is the distribution's own input, assembled from the running motion's
    # controllers and mirrored back by the base cycle, so the solver is what answers for it.
    for solver in platform_force_solvers:
        if solver.force.id:
            solver_by_output.setdefault(solver.force.id, set()).add(solver.id)
    block_ids = _owners_by_value(motions, functions, serial_chain_solvers).block_ids
    # Named by the mechanism that produced the value, so the artifact says whether a pose was
    # asked for once or arrived on a standing channel.
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
            # Read from the deployment config before the loop, by the run that names the file.
            writer = {"kind": "config", "id": item.id}
        elif item.id in PORT_WRITERS:
            writer = PORT_WRITERS[item.id]
        elif item.id in perceived_by_output:
            writer = perceived_by_output[item.id]
        elif isinstance(item, DataValue) and item.role == "joint_space":
            # Declared at the mirror site, which is the only place that knows what it reads.
            writer = item.writer
        elif item.id in function_by_output:
            writers = function_by_output[item.id]
            types = {(functions.get(cid) or {}).get("type") for cid in writers}
            kind = "controller" if types == {"Controller"} else "function"
            writer = {"kind": kind, "id": _sole(writers)}
            if writer["id"] is not None:
                writer["port"] = next(
                    port
                    for value, port in _bound_ids(functions[writer["id"]], "")
                    if value == item.id
                )
        elif item.id in solver_by_output:
            kind = "sensor" if item.id in sensor_outputs else "solver"
            writer = {"kind": kind, "id": _sole(solver_by_output[item.id])}
        elif block is not None:
            writer = {"kind": block, "id": item.id}
        elif isinstance(item, Quantity) and item.sampled:
            # Drawn once at startup, by the run that drew it.
            writer = {"kind": "sampled", "id": item.id}
        # `is not None`, not truthiness: an authored 0.0 is a value, not a missing one.
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

    # A view's value is its superobject's, read through the view: whatever rewrites the
    # superobject at run time writes it, and so does any superobject of a value nothing else writes.
    for subobject_id, superobject_ids in _superobjects_by_view(views).items():
        entry = data_access.get(subobject_id)
        kinds = [data_access[sid]["write"]["kind"] for sid in superobject_ids if sid in data_access]
        if entry is None or not kinds:
            continue
        if entry["write"]["kind"] == "none" or any(kind not in _STATIC_WRITERS for kind in kinds):
            entry["write"] = {"kind": "view", "id": _sole(superobject_ids)}

    for data_id, readers in _readers_by_id(functions, motions, serial_chain_solvers).items():
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
    owners = _owners_by_value(motions, functions, serial_chain_solvers).owners
    items = {item.id: item for item in algorithm_data if item.id}
    analysis = {}
    for value_id, entry in data_access.items():
        kind = entry["write"]["kind"]
        if kind == "view":
            continue
        motion_ids = owners.get(value_id)
        # In the motions' own order, so the same model gates the same way every time.
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

    # A view is written exactly when its superobject is; a view onto a view waits for that one.
    superobjects_of = _superobjects_by_view(views)
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


def _bound_ids(value, path: str):
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
                yield from _bound_ids(item, f"{path}.{key}" if path else key)
    elif isinstance(value, (list, tuple)):
        for item in value:
            yield from _bound_ids(item, path)


def _readers_by_id(functions: dict, motions, serial_chain_solvers) -> dict[str, list]:
    """The blocks that read each D-block, and the port each reads it through: the monitors,
    controllers, functions, solvers and motion entry code bound to it, from their own records."""
    # By IRI: two motions' until terms may share a short id.
    errors_by_constraint = {
        function["constraint_uri"]: bound_id(function.get("error"))
        for function in functions.values()
        if function.get("type") == "ErrorEvaluator" and function.get("constraint_uri")
    }
    reads = []  # (data id, reader kind, reader id, port)
    for motion in motions:
        for monitor in (*motion.when_monitors, *motion.while_monitors, *motion.until_monitors):
            reads.append((bound_id(monitor.error), "monitor", monitor.id, "error"))
            # Without this the band is a D-block nothing reads, and the analysis drops it.
            reads.append((bound_id(monitor.tolerance), "monitor", monitor.id, "tolerance"))
            # An aggregate monitor evaluates each member constraint against its own tolerance, so
            # the terms bind values the monitor's own two signals never name.
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
        # The pose materializer reads a composed orientation's base pose; no function binds it.
        for entry in motion.declared_pose_components:
            reads += [
                (operand.get("pose"), "motion", motion.id, "pose.orientation.base")
                for operand in entry.get("orientation_operands") or ()
            ]
    for function_id, function in functions.items():
        # A pose evaluator copies its difference's per-axis views into `errors`: it writes them.
        outputs = function_output_ids(function) | set(function.get("errors") or ())
        reads += [
            (value, "function", function_id, port)
            for value, port in _bound_ids(function, "")
            if value not in outputs
        ]
    for solver in serial_chain_solvers:
        # The gravity field and the torque bound are the D-blocks a solver binds; everything
        # else on the record is a chain, a device or an output it writes.
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

    return readers_by_id
