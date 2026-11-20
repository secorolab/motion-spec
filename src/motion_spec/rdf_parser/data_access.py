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
            children = value.values()
        elif isinstance(value, (list, tuple, set, frozenset)):
            children = value
        elif is_dataclass(value) and not isinstance(value, type):
            children = [getattr(value, item.name) for item in fields(value)]
        else:
            return
        for item in children:
            visit(item)

    visit(users)

    return [item for item in data_structures if item.id in referenced]


# their initial literal is a fallback, not an authored constant
PORT_WRITERS = {
    "clock_time_s": {"kind": "port", "id": "clock"},
    "dt_measured_s": {"kind": "port", "id": "clock"},
}
STATIC_WRITER_KINDS = {"authored", "sampled", "none"}
CADENCE_BY_WRITER_KIND = {
    "port": "tick",
    "config": "tick",
    "action": "tick",
    "subscription": "tick",
    "authored": "init",
    "sampled": "init",
    "none": "never",
}
# an authored 0.0 is a value
AUTHORED_VALUE_FIELDS = {
    Position: "position",
    Pose: "position",
    Direction: "direction",
    Quantity: "value",
    DataValue: "value",
    FreeVector: "vector",
}
STORAGE_BY_CADENCE = {"never": "absent", "init": "record", "tick": "log"}


def bound_id(value):
    """The id a binding names: the id itself, a record, a serialized record or an enum."""
    if value is None or isinstance(value, str):
        return value
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return value.get("id")
    return value.id


def owners_by_value(motions, functions: dict, serial_chain_solvers) -> tuple[dict, dict]:
    """Which motions write each value, and which values the per-motion blocks -- rather than any
    schedule -- are responsible for.

    A union, never last-writer-wins: several motions can write one value, and attributing it to a
    single motion would gate away live data.

    The outputs in a solver's `world_output` are the ones the loop answers rather than a motion,
    written once per tick before the FSM is dispatched, so no motion owns them.
    """
    world_output_ids = {out.id for solver in serial_chain_solvers for out in solver.world_output}
    block_ids = {"pose": set(), "snapshot": set(), "decomposition": set(), "perturbation": set()}
    owners: dict[str, set] = {}
    for motion in motions:
        for data_id, kind in motion_written_values(motion, functions, world_output_ids):
            if not isinstance(data_id, str):
                continue
            owners.setdefault(data_id, set()).add(motion.id)
            if kind is not None:
                block_ids[kind].add(data_id)
        # cleared every tick, even with no motion selected
        for perturbation in motion.perturbations:
            block_ids["perturbation"].update((perturbation.applied_id, perturbation.active_id))
    return owners, block_ids


def motion_written_values(motion, functions: dict, world_output_ids: set):
    """Every value MOTION's own blocks write, with the per-motion block kind writing it, or None
    for a function or solver output.
    """
    function_ids = [
        *motion.when_schedule,
        *motion.while_pre_schedule,
        *motion.while_schedule,
        *motion.until_schedule,
        *(snapshot.source_function_id for snapshot in motion.snapshots),
    ]
    for function_id in function_ids:
        yield from (
            (out_id, None) for out_id in function_output_ids(functions.get(function_id) or {})
        )
    for solver in motion.serial_chain_solvers:
        # written every tick before the FSM; a motion owner would blank it elsewhere
        yield from (
            (out.id, None)
            for out in [*solver.output, *solver.gripper_joint_outputs]
            if out.id not in world_output_ids
        )
        yield from (
            (sample["id"], None)
            for sample in [*solver.joint_space_samples, *solver.joint_space_cmd_samples]
        )
    yield from (
        (entry["id"] if isinstance(entry, dict) else entry.id, "pose")
        for entry in motion.declared_pose_components
    )
    yield from ((snapshot.target_id, "snapshot") for snapshot in motion.snapshots)
    yield from (
        (component.error, "decomposition")
        for group in motion.pose_axis_error_groups
        for component in group.components
    )


def superobjects_by_subobject(views) -> dict[str, set]:
    """Each value a view selects, to the values it is selected from: one subobject can MAP into
    several superobjects, and it is rewritten whenever any of them is."""
    superobjects_of: dict[str, set] = {}
    for view in (views or {}).values():
        sub_id, super_id = view.subobject.id, view.superobject.id
        if sub_id and super_id and sub_id != super_id:
            superobjects_of.setdefault(sub_id, set()).add(super_id)
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
    solver_by_output, sensor_outputs = solver_writers(serial_chain_solvers, tare_companions)
    for solver in platform_velocity_solvers:
        if solver.velocity.id:
            solver_by_output.setdefault(solver.velocity.id, set()).add(solver.id)
    for solver in platform_force_solvers:
        if solver.force.id:
            solver_by_output.setdefault(solver.force.id, set()).add(solver.id)
    _owners, block_ids = owners_by_value(motions, functions, serial_chain_solvers)
    upstream = upstream_writers(motions, subscriptions, config_poses)

    data_access: dict[str, dict] = {
        item.id: {
            "write": value_writer(
                item,
                next((kind for kind, ids in block_ids.items() if item.id in ids), None),
                upstream,
                functions,
                function_by_output,
                solver_by_output,
                sensor_outputs,
            )
        }
        for item in algorithm_data
        if item.id
    }
    add_view_writers(data_access, views)

    for data_id, kind, reader_id, port in reader_bindings(motions, functions, serial_chain_solvers):
        if not isinstance(data_id, str) or not reader_id or data_id not in data_access:
            continue
        readers = data_access[data_id].setdefault("read", [])
        reader = {"kind": kind, "id": reader_id, "port": port}
        if reader not in readers:
            readers.append(reader)

    return data_access


def solver_writers(serial_chain_solvers, tare_companions: dict) -> tuple[dict, set]:
    """Which serial-chain solvers write each value, a wrench reading's tare companions included,
    and which of those values a sensor measures.
    """
    solver_by_output: dict[str, set] = {}
    sensor_outputs: set = set()
    for solver in serial_chain_solvers:
        gripper_outputs = [out for device in solver.devices for out in device.joint_outputs]
        for out in [*solver.output, *gripper_outputs]:
            written = [out.id]
            if isinstance(out, Wrench):
                written += tare_companions.get(out.id, ())
            for out_id in written:
                solver_by_output.setdefault(out_id, set()).add(solver.id)
            if isinstance(out, Wrench) and out.sensor_name:
                sensor_outputs.update(written)
    return solver_by_output, sensor_outputs


def add_view_writers(data_access: dict, views) -> None:
    """Make a view the writer of each subobject whose superobjects are written at run time, or
    that nothing else writes.
    """
    for subobject_id, superobject_ids in superobjects_by_subobject(views).items():
        entry = data_access.get(subobject_id)
        kinds = [data_access[sid]["write"]["kind"] for sid in superobject_ids if sid in data_access]
        if entry is None or not kinds:
            continue
        if entry["write"]["kind"] == "none" or any(
            kind not in STATIC_WRITER_KINDS for kind in kinds
        ):
            entry["write"] = {
                "kind": "view",
                "id": next(iter(superobject_ids)) if len(superobject_ids) == 1 else None,
            }


def upstream_writers(motions, subscriptions, config_poses) -> dict:
    """The writers outside the computation, by the value they write: the action client or the
    subscription perceiving it, the clock port, or the config for a configured pose -- the later
    of those winning.
    """
    perceived_by_output: dict[str, dict] = {}
    for motion in motions:
        for client in motion.action_clients:
            writer = {"kind": "action", "id": client["act_id"]}
            perceived_by_output[client["status_id"]] = writer
            record_perceived_poses(perceived_by_output, writer, client["written_poses"])
    for sub in subscriptions:
        writer = {"kind": "subscription", "id": sub["sub_id"]}
        record_perceived_poses(perceived_by_output, writer, sub["written_poses"])
    config_writers = {entry["id"]: {"kind": "config", "id": entry["id"]} for entry in config_poses}
    return {**perceived_by_output, **PORT_WRITERS, **config_writers}


def record_perceived_poses(perceived_by_output: dict, writer: dict, rows) -> None:
    """Record WRITER as the writer of every pose in ROWS, and of the stamp it was observed at."""
    for row in rows:
        perceived_by_output[row["pose_id"]] = writer
        if row.get("observed_at_id"):
            perceived_by_output[row["observed_at_id"]] = writer


def value_writer(
    item,
    block,
    upstream: dict,
    functions: dict,
    function_by_output: dict,
    solver_by_output: dict,
    sensor_outputs: set,
) -> dict | None:
    """The block writing ITEM, in precedence order: an upstream writer, a joint-space mirror's
    own, a function, a solver, a per-motion BLOCK, a sampler, the model's literal, or none.
    """
    if item.id in upstream:
        return upstream[item.id]
    if isinstance(item, DataValue) and item.role == "joint_space":
        return item.writer
    if item.id in function_by_output:
        writers = function_by_output[item.id]
        types = {(functions.get(cid) or {}).get("type") for cid in writers}
        writer = {
            "kind": "controller" if types == {"Controller"} else "function",
            "id": next(iter(writers)) if len(writers) == 1 else None,
        }
        if writer["id"] is not None:
            writer["port"] = next(
                port for value, port in bound_ids(functions[writer["id"]], "") if value == item.id
            )
        return writer
    if item.id in solver_by_output:
        solvers = solver_by_output[item.id]
        return {
            "kind": "sensor" if item.id in sensor_outputs else "solver",
            "id": next(iter(solvers)) if len(solvers) == 1 else None,
        }
    if block is not None:
        return {"kind": block, "id": item.id}
    if isinstance(item, Quantity) and item.sampled:
        return {"kind": "sampled", "id": item.id}
    if any(
        isinstance(item, cls) and getattr(item, field) is not None
        for cls, field in AUTHORED_VALUE_FIELDS.items()
    ):
        return {"kind": "authored", "id": None}
    return {"kind": "none", "id": None}


def reader_bindings(motions, functions: dict, serial_chain_solvers) -> list:
    """Every binding a reader makes, as `(data id, reader kind, reader id, port)`."""
    # by IRI: two motions' until terms may share a short id
    errors_by_constraint = {
        function["constraint_uri"]: bound_id(function.get("error"))
        for function in functions.values()
        if function.get("type") == "ErrorEvaluator" and function.get("constraint_uri")
    }
    reads = []
    for motion in motions:
        reads += motion_reader_bindings(motion, errors_by_constraint)
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
        if saturation is None:
            continue
        for bound, quantity in (
            ("maximum", saturation.maximum),
            ("lower", saturation.lower),
            ("upper", saturation.upper),
        ):
            if quantity is not None:
                reads.append((quantity.id, "solver", solver.id, f"torque_saturation.{bound}"))
    return reads


def motion_reader_bindings(motion, errors_by_constraint: dict) -> list:
    """The bindings MOTION's monitors, controllers, snapshots and composed poses read."""
    reads = []
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
    return reads


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
    owners, _block_ids = owners_by_value(motions, functions, serial_chain_solvers)
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
        joint_space = isinstance(item, DataValue) and item.role == "joint_space"
        cadence = owned if joint_space else CADENCE_BY_WRITER_KIND.get(kind, owned)
        analysis[value_id] = {"cadence": cadence}

    add_view_cadences(analysis, data_access, motions, views)

    for entry in analysis.values():
        cadence = entry["cadence"]
        entry["storage"] = "log" if isinstance(cadence, dict) else STORAGE_BY_CADENCE[cadence]

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


def add_view_cadences(analysis: dict, data_access: dict, motions, views) -> None:
    """Add to ANALYSIS the cadence of each value a view writes, inherited from its superobjects
    once they have one: every tick if any is written per tick, else while their motions run.

    Raises:
        RuntimeError: views whose superobjects nothing writes.
    """
    superobjects_of = superobjects_by_subobject(views)
    pending = [vid for vid, entry in data_access.items() if entry["write"]["kind"] == "view"]
    while pending:
        waiting = []
        for value_id in pending:
            sources = [
                analysis[sid]["cadence"] for sid in superobjects_of[value_id] if sid in analysis
            ]
            if sources:
                analysis[value_id] = {"cadence": inherited_cadence(sources, motions)}
            else:
                waiting.append(value_id)
        if len(waiting) == len(pending):
            raise RuntimeError(f"data access: views with no written superobject: {waiting}")
        pending = waiting


def inherited_cadence(sources: list, motions):
    """The cadence a view takes from its superobjects' SOURCES cadences."""
    if "tick" in sources:
        return "tick"
    live = [cadence for cadence in sources if isinstance(cadence, dict)]
    if not live:
        return sources[0]
    return {
        "motions": [
            motion.id
            for motion in motions
            if any(motion.id in source["motions"] for source in live)
        ]
    }


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
        return
    if isinstance(value, (list, tuple)):
        for item in value:
            yield from bound_ids(item, path)
        return
    if not isinstance(value, dict):
        return
    if path and isinstance(value.get("id"), str):
        yield value["id"], path
    prefix = f"{path}." if path else ""
    for key, item in value.items():
        if key not in {"id", "type"}:
            yield from bound_ids(item, f"{prefix}{key}")
