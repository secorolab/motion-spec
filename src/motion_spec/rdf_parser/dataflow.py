# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The dataflow contract: every shared value's producer, write cadence and storage, plus the
layer-B role projections built off that one classification."""

from __future__ import annotations

from motion_spec.classes.closures import closure_output_ids

from motion_spec.rdf_parser.records import _field, _sole

# Storage follows from write cadence, in one place. A value written once says nothing new when
# repeated per tick, and one never written is not a runtime value at all.
_STORAGE_BY_CADENCE = {"never": "absent", "init": "record", "tick": "log"}
# Shared values the control loop writes from a backend port, not from any model entity. Their
# initial literal is a fallback, not an authored constant, so the contract is stated here rather
# than inferred from the value being present.
_PORT_PRODUCERS = {
    "clock_time_s": {"kind": "port", "id": "clock"},
    "dt_measured_s": {"kind": "port", "id": "clock"},
}
_MOTION_SCHEDULES = ("when_schedule", "while_pre_schedule", "while_schedule", "until_schedule")
_LITERAL_FIELDS = ("position", "direction", "orientation", "value", "vector")
# Fields on a shared-data entry that carry the numbers behind a `vec` sample descriptor.
_LITERAL_VECTORS = ("position", "direction", "vector")


def _storage_for(cadence) -> str:
    """Derive where a value belongs from when it is written (never override this per value)."""
    return "log" if isinstance(cadence, dict) else _STORAGE_BY_CADENCE[cadence]


def _constant_value(item, desc: dict):
    """The authored number a `cadence: init` sample row carries, for the schema header."""
    kind = desc.get("kind")
    if kind == "literal":
        return float(desc["value"])
    if kind in {"shared", "access", "bool", "int"}:
        return _field(item, "value")
    if kind == "vec":
        for field in _LITERAL_VECTORS:
            values = _field(item, field)
            if values is not None:
                return values[desc["axis"]]
    return None


def annotate_dataflow(
    introspection: dict, shared_data: list, closures: dict, motions, serial_chain_solvers, views
) -> dict:
    """Give every shared value its producer, its write cadence, and the storage those imply, then
    apply that contract: drop what nothing writes, move what is written once into the header, and
    project the roles the templates ask about (``values``) off the same classification.

    Cadence -- not motion membership -- decides gating: a value written by several motions carries
    all of them, and one no motion's step function writes falls back to ``tick``.
    """
    closure_by_output: dict[str, set] = {}
    for closure_id, closure in closures.items():
        if isinstance(closure, dict):
            for out_id in closure_output_ids(closure):
                closure_by_output.setdefault(out_id, set()).add(closure_id)

    solver_by_output: dict[str, set] = {}
    sensor_outputs: set = set()
    # The readings themselves, without the tare companions below: only these are supplied by the
    # platform, and only they get an external-measurement pointer.
    measured_outputs: set = set()
    for solver in serial_chain_solvers:
        for out in (_field(solver, "output", []) or []) + (
            _field(solver, "gripper_joint_outputs", []) or []
        ):
            out_id = _field(out, "id")
            solver_by_output.setdefault(out_id, set()).add(_field(solver, "id"))
            if _field(out, "sensor_name"):
                sensor_outputs.add(out_id)
                measured_outputs.add(out_id)
                # tare state, written alongside the reading (_shared_runtime_members)
                for companion in (f"{out_id}_ft_bias", f"{out_id}_ft_settle"):
                    solver_by_output.setdefault(companion, set()).add(_field(solver, "id"))
                    sensor_outputs.add(companion)

    # A union, never last-writer-wins: several motions can write one value, and attributing it
    # to a single motion would gate away live data.
    owners: dict[str, set] = {}
    # Poses, snapshots and per-axis errors are written by the pose-composition, snapshot and
    # error-decomposition blocks, which are emitted per motion rather than scheduled as closures.
    pose_ids: set = set()
    snapshot_ids: set = set()
    axis_error_ids: set = set()

    def own(data_id, motion_id) -> None:
        if isinstance(data_id, str):
            owners.setdefault(data_id, set()).add(motion_id)

    for motion in motions:
        motion_id = _field(motion, "id")
        for schedule in _MOTION_SCHEDULES:
            for closure_id in _field(motion, schedule, []) or []:
                for out_id in closure_output_ids(closures.get(closure_id) or {}):
                    own(out_id, motion_id)
        for solver in _field(motion, "serial_chain_solvers", []) or []:
            for out in (_field(solver, "output", []) or []) + (
                _field(solver, "gripper_joint_outputs", []) or []
            ):
                out_id = _field(out, "id")
                own(out_id, motion_id)
                if _field(out, "sensor_name"):
                    own(f"{out_id}_ft_bias", motion_id)
                    own(f"{out_id}_ft_settle", motion_id)
            # Joint-space mirrors are written by whichever motion's solver ran (add_joint_space_
            # logging), so the runtime's channels are live in every motion that drives it.
            for sample in (_field(solver, "joint_space_samples", []) or []) + (
                _field(solver, "joint_space_cmd_samples", []) or []
            ):
                own(_field(sample, "id"), motion_id)
        for group in (
            _field(motion, "declared_pose_components", []) or [],
            _field(motion, "relative_poses", []) or [],
        ):
            for entry in group:
                own(_field(entry, "id"), motion_id)
                pose_ids.add(_field(entry, "id"))
        for snapshot in _field(motion, "snapshots", []) or []:
            own(_field(snapshot, "target_id"), motion_id)
            snapshot_ids.add(_field(snapshot, "target_id"))
            # The source closure runs inside the snapshot block, not from a schedule.
            for out_id in closure_output_ids(
                closures.get(_field(snapshot, "source_closure_id")) or {}
            ):
                own(out_id, motion_id)
        for group in _field(motion, "pose_axis_error_groups", []) or []:
            for component in _field(group, "components", []) or []:
                own(_field(component, "error"), motion_id)
                axis_error_ids.add(_field(component, "error"))

    def contract(item) -> tuple[dict, object]:
        """(producer, cadence) for one shared-data member."""
        item_id = _field(item, "id")
        if item_id in _PORT_PRODUCERS:
            return _PORT_PRODUCERS[item_id], "tick"
        motion_ids = owners.get(item_id)
        cadence = {"motions": sorted(motion_ids)} if motion_ids else "tick"
        if _field(item, "role") == "joint_space":
            # Declared at the mirror site (add_joint_space_logging), which is the only place that
            # knows what the expression reads.
            return _field(item, "producer"), cadence
        if item_id in closure_by_output:
            producer_ids = closure_by_output[item_id]
            types = {(closures.get(cid) or {}).get("type") for cid in producer_ids}
            kind = "controller" if types == {"Controller"} else "closure"
            return {"kind": kind, "id": _sole(producer_ids)}, cadence
        if item_id in solver_by_output:
            kind = "sensor" if item_id in sensor_outputs else "solver"
            return {"kind": kind, "id": _sole(solver_by_output[item_id])}, cadence
        for kind, ids in (
            ("pose", pose_ids),
            ("snapshot", snapshot_ids),
            ("decomposition", axis_error_ids),
        ):
            if item_id in ids:
                return {"kind": kind, "id": item_id}, cadence
        # `is not None`, not truthiness: an authored 0.0 is a value, not a missing one.
        if any(_field(item, field) is not None for field in _LITERAL_FIELDS):
            return {"kind": "authored", "id": None}, "init"
        return {"kind": "none", "id": None}, "never"

    # A view is written exactly when its superobject is. One subobject can MAP into several,
    # so collect them all: the value is live whenever any is recomputed.
    superobjects_of: dict[str, set] = {}
    for view in (views or {}).values():
        subobject_id = _field(_field(view, "subobject"), "id")
        superobject_id = _field(_field(view, "superobject"), "id")
        if subobject_id and superobject_id:
            superobjects_of.setdefault(subobject_id, set()).add(superobject_id)

    # One artifact holds the contract, so storage is derived from cadence in exactly one place.
    dataflow = {}
    items_by_id = {}
    for item in shared_data:
        item_id = _field(item, "id")
        if not item_id:
            continue
        items_by_id[item_id] = item
        producer, cadence = contract(item)
        dataflow[item_id] = {
            "producer": producer,
            "cadence": cadence,
            "storage": _storage_for(cadence),
        }

    for subobject_id, superobject_ids in superobjects_of.items():
        entry = dataflow.get(subobject_id)
        sources = [dataflow[sid] for sid in sorted(superobject_ids) if sid in dataflow]
        if entry is None or not sources:
            continue
        live = [source["cadence"] for source in sources if source["storage"] == "log"]
        if live:
            # Live even when the view was authored with a literal: that is only the start value.
            cadence = (
                "tick"
                if any(not isinstance(source, dict) for source in live)
                else {"motions": sorted({m for source in live for m in source["motions"]})}
            )
        elif entry["producer"]["kind"] == "none":
            cadence = sources[0]["cadence"]
        else:
            continue
        entry["producer"] = {"kind": "view", "id": _sole(superobject_ids)}
        entry["cadence"] = cadence
        entry["storage"] = _storage_for(cadence)

    for member_id, consumers in _consumers_by_id(introspection, closures).items():
        entry = dataflow.get(member_id)
        if entry is not None:
            entry["consumers"] = consumers

    # A value nothing writes but something reads is a broken binding, not metadata: dropping it
    # would feed the reader a zero forever.
    orphans = {
        member_id: entry["consumers"]
        for member_id, entry in dataflow.items()
        if entry["cadence"] == "never" and entry.get("consumers")
    }
    if orphans:
        raise RuntimeError(
            "dataflow: read but never written: "
            + "; ".join(
                f"{member_id} (read by {', '.join(c['id'] for c in consumers)})"
                for member_id, consumers in sorted(orphans.items())
            )
        )

    introspection["dataflow"] = dataflow
    _apply_dataflow(introspection, shared_data, items_by_id, dataflow)

    # Layer-B projections of the same contract, answered from the producer classification above
    # rather than by a second scan. Externally measured = a solver output the platform supplies.
    # Returned, not stored: the projection is published once, at the top level.
    return {
        "externally_measured": sorted(
            (item for item in shared_data if _field(item, "id") in measured_outputs),
            key=lambda item: _field(item, "id"),
        )
    }


def _consumers_by_id(introspection: dict, closures: dict) -> dict[str, list]:
    """Who reads each shared value: the monitors, controllers and closures bound to it."""
    consumers: dict[str, list] = {}

    def add(member_id, kind: str, reader_id, role: str) -> None:
        if isinstance(member_id, str) and reader_id:
            consumers.setdefault(member_id, []).append(
                {"kind": kind, "id": reader_id, "role": role}
            )

    for monitor in introspection.get("monitors", []):
        add(monitor.get("error_signal"), "monitor", monitor.get("id"), "error")
        # Without this the band is a shared value nothing reads, and the contract drops it.
        add(monitor.get("tolerance_signal"), "monitor", monitor.get("id"), "tolerance")
    for controller in introspection.get("controllers", []):
        for role in ("error_signal", "measured_signal", "setpoint_signal"):
            add(controller.get(role), "controller", controller.get("id"), role)
    for closure_id, closure in closures.items():
        if not isinstance(closure, dict):
            continue
        outputs = closure_output_ids(closure)
        for key, value in closure.items():
            if key not in {"id", "type"} and isinstance(value, str) and value not in outputs:
                add(value, "closure", closure_id, key)
    # Readers are collected from dicts whose order is the graph's; the list is an artifact.
    for readers in consumers.values():
        readers.sort(key=lambda reader: (reader["kind"], reader["id"], reader["role"]))
    return consumers


def _apply_dataflow(introspection: dict, shared_data: list, items_by_id: dict, dataflow: dict):
    """Act on the contract: absent values leave the program, init values leave the per-tick frame."""
    shared_data[:] = [
        item
        for item in shared_data
        if dataflow.get(_field(item, "id"), {}).get("storage") != "absent"
    ]

    logged, constants, unattributed = [], [], []
    for sample in introspection.get("quantity_samples", []):
        entry = dataflow.get(sample.get("source_id"))
        if entry is None:
            unattributed.append(sample.get("id"))
            continue
        sample.update(entry)
        if entry["storage"] == "log":
            logged.append(sample)
        elif entry["storage"] == "record":
            value = _constant_value(items_by_id[sample["source_id"]], sample["sample_desc"])
            if value is None:
                unattributed.append(sample.get("id"))
                continue
            constants.append({"id": sample["id"], "source_id": sample["source_id"], "value": value})
    if unattributed:
        raise RuntimeError(f"dataflow: samples with no resolvable contract: {sorted(unattributed)}")
    introspection["quantity_samples"] = logged
    introspection["constants"] = constants

    spatial = introspection.get("spatial_samples") or {}
    for pool, rows in spatial.items():
        kept = [row for row in rows if dataflow.get(row["id"], {}).get("storage") == "log"]
        spatial[pool] = [dict(row, index=idx) for idx, row in enumerate(kept)]
