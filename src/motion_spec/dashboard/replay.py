# SPDX-License-Identifier: MPL-2.0

"""A finished run, read back: what it constrained, what it recorded, what it plots."""

from __future__ import annotations

from pathlib import Path, PurePosixPath
from urllib.parse import urlparse

from rdflib import Dataset

from motion_spec.dashboard import roots
from motion_spec.dashboard.frames import slot_signals
from motion_spec.dashboard.sources import authored_lines
from motion_spec.runs.provenance import (
    GENERATION_DOCUMENT,
    kept_sources,
    read_generation_dataset,
    run_generation,
)
from motion_spec.runs.replay import resolve_run, validate_header
from motion_spec.runs.verify import RecordError
from motion_spec.telemetry import frame_log_pb
from motion_spec.telemetry.frame_log_pb import read_health

GAIN_LABELS = {
    "proportional_gain": "Kp",
    "integral_gain": "Ki",
    "derivative_gain": "Kd",
    "decay_rate": "decay",
}


def slot_ids(slots: list, field: str) -> list[str]:
    """The ids these slots name in one role, in order, without repeats or blanks."""
    ids = []
    for slot in slots:
        value = getattr(slot, field)
        if value and value not in ids:
            ids.append(value)
    return ids


def authored_key(slot, motion, name: str) -> tuple[str, str]:
    """The (motion, constraint) a slot was authored as, from the IRI the header carries.

    A constraint IRI reads `<model>/<motion>/<phase>/<constraint>`; a name alone does not say
    the motion when one constraint is authored in several.
    """
    # a generated conjunction is placed by what it watches
    watched = slot.watched if slot.DESCRIPTOR.name == "Monitor" else ()
    constraint = (name or "").replace("-", "_")
    for iri in (slot.constraint_iri, *(member.iri for member in watched)):
        segments = PurePosixPath(urlparse(iri or "").path).parts
        if len(segments) >= 3 and segments[-2] in ("while", "until", "when"):
            return (segments[-3].replace("-", "_"), constraint)
    return ((motion.id or "").replace("-", "_"), constraint)


def source_constraints(run_dir: Path, contract) -> list[dict]:
    """Return one row per constraint this run recorded, read off the log's own header.

    The join is on the slot identities the header records, not on names from the source.

    The row shape is a contract with the frontend: motion, handler, line, name, expression,
    kind, evaluator, between, tracking, error, control, monitors, setpoints, gains, tolerance.
    """
    generation = run_generation(run_dir)
    provenance = (
        read_generation_dataset(generation / "generated" / GENERATION_DOCUMENT)
        if generation
        else Dataset()
    )
    source = next(
        (path for path in kept_sources(provenance) if path.suffix == ".robmot" and path.is_file()),
        None,
    )
    authored = authored_lines(source) if source else {}
    constants = {
        key: row.value
        for row in contract.header.constants
        for key in (row.id, row.source_id)
        if key
    }
    spelled = {(name or "").replace("-", "_"): name for _, _, name in authored.values()}
    rows = []
    for motion in contract.header.motions:
        for kind, slots in (("controlled", motion.controllers), ("monitored", motion.monitors)):
            # an aggregate monitor names no constraint and groups under its own id
            groups: dict[str, list] = {}
            for slot in slots:
                groups.setdefault(slot.constraint_iri or slot.id, []).append(slot)
            for group in groups.values():
                first = group[0]
                name = (
                    first.constraint_id
                    or str(first.constraint_iri).rsplit("/", 1)[-1].rsplit("#", 1)[-1]
                    or first.id
                )
                key = authored_key(first, motion, name)
                line, expression, authored_motion = authored.get(key, (None, None, None))
                operands = list(
                    dict.fromkeys(value for slot in group for value in slot.operand_ids)
                )
                compared = operands or (
                    [*slot_ids(group, "measured_id"), *slot_ids(group, "setpoint_id")]
                    if kind == "controlled"
                    else []
                )
                evaluator = next(iter(slot_ids(group, "evaluator_id")), None)
                rows.append(
                    {
                        "motion": authored_motion or spelled.get(key[0]) or key[0] or motion.id,
                        "handler": motion.id,
                        "line": line,
                        "name": name,
                        "expression": expression,
                        "kind": kind,
                        "evaluator": evaluator or first.iri or first.id or None,
                        "between": compared,
                        "tracking": [value for value in compared if value not in constants],
                        "members": list(
                            {
                                member.id: {
                                    "id": member.id,
                                    "iri": member.iri,
                                    "error": member.error_id,
                                    "tolerance": constants.get(member.tolerance_id),
                                }
                                for slot in group
                                for member in slot.watched
                                if member.error_id
                            }.values()
                        )
                        if kind == "monitored"
                        else [],
                        "error": slot_ids(group, "error_id"),
                        "control": slot_ids(group, "output_id") if kind == "controlled" else [],
                        "monitors": [slot.id for slot in group] if kind == "monitored" else [],
                        "setpoints": [
                            {"label": value, "value": constants[value]}
                            for value in slot_ids(group, "setpoint_id")
                            if value in constants
                        ]
                        if kind == "controlled"
                        else [],
                        "gains": {
                            GAIN_LABELS.get(gain.role, gain.role): gain.value
                            for slot in group
                            for gain in slot.gains
                        }
                        if kind == "controlled"
                        else {},
                        "tolerance": next(
                            (
                                constants[slot.tolerance_id]
                                for slot in group
                                if slot.tolerance_id in constants
                            ),
                            None,
                        ),
                    }
                )
    return rows


SLOT_KEYS = {
    "constraints": ("error", "output", "measured", "setpoint", "satisfied"),
    "monitors": ("value", "satisfied"),
}

SLOT_ROLES = {
    "constraints": (
        ("error_id", "error"),
        ("output_id", "output"),
        ("measured_id", "measured"),
        ("setpoint_id", "setpoint"),
        ("tolerance_id", "tolerance"),
    ),
    "monitors": (("error_id", "error"), ("tolerance_id", "tolerance")),
}


def replay_data(run_dir: Path) -> dict:
    """Return replay metadata without decoding the complete frame log."""
    pending = False
    try:
        run_dir, log, contract = resolve_run(run_dir)
    except RecordError:
        # not writing yet: the generation's header record is a zero-frame log
        generation = run_generation(run_dir)
        record = generation / roots.HEADER_REL if generation else None
        if record is None or not record.is_file():
            raise
        log, contract = record, frame_log_pb.read_contract(record)
        pending = True
    constraints = source_constraints(run_dir, contract)
    health = read_health(log) or {}
    frame_count = health.get("written_frames", 0)
    signals = ["timing.compute_ms", "timing.period_ms"]
    mirrored = {
        f"{contract.fields['constraints'][slot.number]['id']}.{role}"
        for motion in contract.header.motions
        for slot in motion.controllers
        if slot.number < len(contract.fields["constraints"])
        for attribute, role in SLOT_ROLES["constraints"]
        if getattr(slot, attribute)
    }
    signals.extend(
        name
        for field in contract.fields["constraints"]
        for key in SLOT_KEYS["constraints"]
        if (name := f"{field['id']}.{key}") not in mirrored
    )
    signals.extend(
        f"{slot.id}.{key}"
        for motion in contract.header.motions
        for slot in motion.monitors
        for key in SLOT_KEYS["monitors"]
    )
    signals.extend(slot_signals(contract))
    indices = {motion.id: motion.index for motion in contract.header.motions}
    spelling: dict[str, str] = {}
    scanned = log_events(log, contract)
    logged = set(signals) | {field["id"] for field in contract.fields["quantities"]}
    parts = {}
    for name in logged:
        parts.setdefault(name.partition(".")[0], []).append(name)
    for constraint in constraints:
        constraint["monitors"] = [
            f"{name}.{key}" for name in constraint["monitors"] for key in SLOT_KEYS["monitors"]
        ]
        for key in ("tracking", "control", "monitors", "error"):
            constraint[key] = [
                signal
                for name in constraint[key]
                for signal in ([name] if name in logged else parts.get(name, ()))
            ]
        handler = constraint.pop("handler")
        spelling.setdefault(handler, constraint["motion"])
        constraint["window"] = scanned["windows"].get(indices.get(handler))
    signals.extend(
        signal
        for constraint in constraints
        for signal in (*constraint["tracking"], *constraint["control"], *constraint["monitors"])
    )
    offered = list(dict.fromkeys(signals))
    index: dict[str, dict] = {}
    for motion in contract.header.motions:
        for pool, slots in (("constraints", motion.controllers), ("monitors", motion.monitors)):
            fields = contract.fields[pool]
            for slot in slots:
                owner = (
                    slot.constraint_id
                    or str(slot.constraint_iri).rsplit("/", 1)[-1].rsplit("#", 1)[-1]
                    or slot.id
                )
                field = fields[slot.number]["id"] if slot.number < len(fields) else None
                named = {role for attribute, role in SLOT_ROLES[pool] if getattr(slot, attribute)}
                for key in SLOT_KEYS[pool]:
                    where = {"motion": motion.id, "constraint": owner, "role": key, "slot": slot.id}
                    # skip a pooled signal mirroring one the header names
                    if field and key not in named:
                        index[f"{field}.{key}"] = where
                    if pool == "monitors":
                        index[f"{slot.id}.{key}"] = where
                for attribute, role in SLOT_ROLES[pool]:
                    signal = getattr(slot, attribute)
                    if signal:
                        index.setdefault(
                            signal,
                            {
                                "motion": motion.id,
                                "constraint": owner,
                                "role": role,
                                "slot": slot.id,
                            },
                        )
                for member in slot.watched if pool == "monitors" else ():
                    if member.error_id:
                        index.setdefault(
                            member.error_id,
                            {
                                "motion": motion.id,
                                "constraint": member.id,
                                "role": "error",
                                "slot": slot.id,
                            },
                        )
    generation = run_generation(run_dir)
    return {
        "generation": (
            str(generation.relative_to(roots.GENERATIONS))
            if generation and roots.GENERATIONS.resolve() in generation.resolve().parents
            else None
        ),
        "frames": frame_count,
        "duration": frame_count * contract.header.nominal_period_ns / 1e9,
        "states": [state.id for state in contract.header.fsm_states],
        "events": scanned["events"],
        "signals": offered,
        "signal_index": {
            name: {**where, "motion": spelling.get(where["motion"], where["motion"])}
            for name, where in index.items()
        },
        "constraints": constraints,
        "motion_names": spelling,
        # a `when` monitor runs in its predecessor motion, not where it was authored
        "monitor_owners": {
            slot.id: spelling.get(motion.id, motion.id)
            for motion in contract.header.motions
            for slot in motion.monitors
        },
        "videos": [path.stem for path in (run_dir / "logs").glob("*.mp4")],
        "header": validate_header(log, contract),
        "health": health,
        "pending": pending,
    }


_EVENTS: dict[str, dict] = {}


def log_events(log: Path, contract) -> dict:
    """Every FSM state entry, fired event and satisfied edge in one log, scanned incrementally.

    Events come off the frame's trigger ring, since `last_event` holds only one of a tick's
    fires. A satisfied edge is a rise and a fall for a goal constraint, a rise only for a
    monitor. A growing log resumes where the last call stopped; a finished one is not rescanned.
    """
    size = log.stat().st_size
    scan = _EVENTS.get(str(log))
    if scan is None or scan["size"] > size:
        if len(_EVENTS) > 32:
            _EVENTS.pop(next(iter(_EVENTS)))
        fired = [event.id for event in contract.header.fsm_events]
        scan = _EVENTS[str(log)] = {
            "size": 0,
            "offset": 0,
            "index": 0,
            "states": [state.id for state in contract.header.fsm_states],
            "fired": fired,
            # slot indices are motion-local
            "by_motion": {
                motion.index: (
                    {slot.number: slot for slot in motion.controllers},
                    {slot.number: slot for slot in motion.monitors},
                )
                for motion in contract.header.motions
            },
            "state_was": None,
            "event_was": None,
            "motion_was": None,
            "csat_was": None,
            "msat_was": None,
            "result": {"events": [], "windows": {}, "tally": {}, "exits": {}},
        }
    if size <= scan["size"]:
        return scan["result"]
    scan["size"] = size
    states, fired = scan["states"], scan["fired"]
    constraint_pool = len(contract.fields["constraints"])
    monitor_pool = len(contract.fields["monitors"])
    events, windows = scan["result"]["events"], scan["result"]["windows"]
    tally, exits = scan["result"]["tally"], scan["result"]["exits"]
    index = scan["index"]
    state_was, event_was, motion_was = scan["state_was"], scan["event_was"], scan["motion_was"]
    csat_was, msat_was = scan["csat_was"], scan["msat_was"]
    with frame_log_pb.open_log(log) as fh:
        for frame, offset in frame_log_pb.raw_frames(fh, contract, scan["offset"]):
            scan["offset"] = offset
            fired_now: list[str] = []
            if contract.trigger_pool:
                for entry in frame.triggers:
                    if 0 <= entry.idx < len(fired):
                        events.append({"frame": index, "kind": "event", "label": fired[entry.idx]})
                        fired_now.append(fired[entry.idx])
            elif frame.last_event != event_was:
                event_was = frame.last_event
                if 0 <= event_was < len(fired):
                    events.append({"frame": index, "kind": "event", "label": fired[event_was]})
                    fired_now.append(fired[event_was])
            controllers, monitors = scan["by_motion"].get(frame.active_motion, ({}, {}))
            csat = [
                slot in controllers
                and bool(
                    (entry := frame_log_pb.slot(frame, "constraints", slot)) and entry.satisfied
                )
                for slot in range(constraint_pool)
            ]
            msat = [
                slot in monitors
                and bool((entry := frame_log_pb.slot(frame, "monitors", slot)) and entry.satisfied)
                for slot in range(monitor_pool)
            ]
            # across a state or motion change slot i is another controller
            held = frame.fsm_state == state_was and frame.active_motion == motion_was
            _, monitors_was = scan["by_motion"].get(motion_was, ({}, {}))
            if state_was is not None and frame.fsm_state != state_was:
                exits.setdefault(motion_was, []).append(
                    {
                        "frame": index,
                        "to_state": states[frame.fsm_state]
                        if 0 <= frame.fsm_state < len(states)
                        else str(frame.fsm_state),
                        "events": list(fired_now),
                        "monitors": [
                            monitors_was[slot].id
                            for slot, up in enumerate(msat_was or [])
                            if up and slot in monitors_was
                        ],
                    }
                )
            for slot, controller in controllers.items():
                if not controller.constraint_iri or slot >= len(csat):
                    continue
                goal = tally.setdefault(
                    (frame.active_motion, "goal", slot),
                    {
                        "id": controller.id,
                        "active": 0,
                        "satisfied": 0,
                        "first_satisfied": None,
                        "losses": 0,
                    },
                )
                goal["active"] += 1
                if csat[slot]:
                    goal["satisfied"] += 1
                    if goal["first_satisfied"] is None:
                        goal["first_satisfied"] = index
                elif held and csat_was[slot]:
                    goal["losses"] += 1
            for slot, monitor in monitors.items():
                if slot >= len(msat):
                    continue
                watch = tally.setdefault(
                    (frame.active_motion, "monitor", slot),
                    {"id": monitor.id, "active": 0, "fired": None},
                )
                watch["active"] += 1
                if msat[slot] and watch["fired"] is None:
                    watch["fired"] = index
            if frame.fsm_state != state_was:
                state_was = frame.fsm_state
                label = states[state_was] if 0 <= state_was < len(states) else str(state_was)
                events.append({"frame": index, "kind": "state", "label": label})
            if held:
                for slot, (now, before) in enumerate(zip(csat, csat_was)):
                    # only goal constraints, not pure regulation
                    if now == before or not controllers[slot].constraint_iri:
                        continue
                    events.append(
                        {
                            "frame": index,
                            "kind": "satisfied" if now else "unsatisfied",
                            "label": controllers[slot].id,
                        }
                    )
                for slot, (now, before) in enumerate(zip(msat, msat_was)):
                    if now and not before:
                        events.append(
                            {"frame": index, "kind": "monitor", "label": monitors[slot].id}
                        )
            motion_was, csat_was, msat_was = frame.active_motion, csat, msat
            window = windows.setdefault(frame.active_motion, [index, index])
            window[1] = index
            index += 1
    scan["index"] = index
    scan["state_was"], scan["event_was"], scan["motion_was"] = state_was, event_was, motion_was
    scan["csat_was"], scan["msat_was"] = csat_was, msat_was
    return scan["result"]


def run_verdict(run_dir: Path) -> dict:
    """Per motion: whether each goal was reached and held, when each monitor fired, and how the
    motion was left, read off the cached marker scan."""
    run_dir, log, contract = resolve_run(run_dir)
    scanned = log_events(log, contract)
    tally, exits, windows = scanned["tally"], scanned["exits"], scanned["windows"]
    spelling: dict[str, str] = {}
    for row in source_constraints(run_dir, contract):
        spelling.setdefault(row["handler"], row["motion"])
    ran = sorted(
        (motion for motion in contract.header.motions if motion.index in windows),
        key=lambda motion: windows[motion.index][0],
    )
    motions = [
        {
            "motion": spelling.get(motion.id, motion.id),
            "window": windows[motion.index],
            "constraints": [
                {"kind": kind, **counts}
                for (index, kind, _slot), counts in tally.items()
                if index == motion.index
            ],
            "exits": exits.get(motion.index, []),
        }
        for motion in ran
    ]
    return {"period_s": contract.header.nominal_period_ns / 1e9, "motions": motions}


def signal_reader(contract):
    """Map a signal name onto a raw frame, the way the plots read one."""
    constraint_slots = {
        field["id"]: index for index, field in enumerate(contract.fields["constraints"])
    }
    slots = slot_signals(contract)
    monitor_slots = {
        slot.id: (contract.fields["monitors"][slot.number], motion.index)
        for motion in contract.header.motions
        for slot in motion.monitors
        if slot.number < len(contract.fields["monitors"])
    }
    quantities = {field["id"]: field for field in contract.fields["quantities"]}

    def value(frame, name: str):
        if name == "timing.compute_ms":
            return frame.compute_ns / 1e6
        if name == "timing.period_ms":
            return frame.period_ns / 1e6
        if name in quantities:
            field = quantities[name]
            gate = contract.gate.get("quantities")
            if gate is not None and field["index"] not in gate.get(frame.active_motion, ()):
                return None
            for index, value in zip(frame.quantity_slots, frame.quantities):
                if index == field["index"]:
                    return float(value)
            return 0.0
        if name in slots:
            kind, field, attribute = slots[name]
            gate = contract.gate.get(kind)
            if gate is not None and field["index"] not in gate.get(frame.active_motion, ()):
                return None
            return float(getattr(getattr(frame, kind)[field["index"]], attribute))
        prefix, _, key = name.rpartition(".")
        if prefix in constraint_slots:
            entry = frame_log_pb.slot(frame, "constraints", constraint_slots[prefix])
            return getattr(entry, key) if entry else 0
        if prefix in monitor_slots:
            field, owner = monitor_slots[prefix]
            if frame.active_motion != owner:
                return None
            entry = frame_log_pb.slot(frame, "monitors", field["index"])
            return getattr(entry, key) if entry else 0
        raise ValueError(f"unknown signal: {name}")

    return value


def plot_data(run_dir: Path, names: list[str], window: tuple | None = None) -> dict:
    """Stream and downsample requested fields without shaping complete frames.

    Sampling follows the window asked for, so a short motion is not missed between samples.
    """
    _, log, contract = resolve_run(run_dir)
    value = signal_reader(contract)
    health = read_health(log) or {}
    first, last = window or (0, max(0, health.get("written_frames", 0) - 1))
    step = max(1, (last - first + 1) // 1600)
    series = {name: [] for name in names}
    with frame_log_pb.open_log(log) as fh:
        for index, (frame, _) in enumerate(frame_log_pb.raw_frames(fh, contract)):
            if index > last:
                break
            if index >= first and (index - first) % step == 0:
                for name in names:
                    series[name].append(value(frame, name))
    return {
        "signals": series,
        "events": log_events(log, contract)["events"],
        "sample_step": step,
        "first_frame": first,
    }
