# SPDX-License-Identifier: MPL-2.0

"""A finished run, read back: what it constrained, what it recorded, what it plots."""

from __future__ import annotations

from pathlib import Path, PurePosixPath
from urllib.parse import urlparse

from motion_spec.dashboard import roots
from motion_spec.dashboard.catalog import rdf_name, run_videos
from motion_spec.dashboard.frames import slot_signals
from motion_spec.dashboard.sources import _key, authored_lines
from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.archive import ArchiveError
from motion_spec.introspection.replay import read_health, resolve_archive, validate_header


def downsample(values: list[float], target: int = 1600) -> list[float]:
    step = max(1, len(values) // target)
    return values[::step]


# How the dashboard labels the gain roles a controller slot carries; others keep their role name.
GAIN_LABELS = {
    "proportional_gain": "Kp",
    "integral_gain": "Ki",
    "derivative_gain": "Kd",
    "decay_rate": "decay",
}


def run_source_text(run_dir: Path, manifest: dict | None) -> str:
    """The authored `.robmot` this run was generated from, vendored in the run or beside it."""
    vendored = [
        run_dir / rel
        for rel in (manifest or {}).get("files", {}).get("sources", ())
        if rel.endswith(".robmot")
    ]
    generated = sorted((run_dir.parent.parent / "generated/source").glob("*.robmot"))
    source = next((path for path in (*vendored, *generated) if path.is_file()), None)
    return source.read_text() if source else ""


def _slot_ids(slots: list, field: str) -> list[str]:
    """The ids these slots name in one role, in order, without repeats or blanks."""
    return list(dict.fromkeys(value for slot in slots if (value := getattr(slot, field))))


def _by_constraint(slots) -> dict:
    """Slots grouped by the constraint they serve -- the axis controllers of one share it.

    An aggregate monitor watches several constraints and names none of them singly, so it
    groups under its own id and stays one row.
    """
    groups: dict[str, list] = {}
    for slot in slots:
        groups.setdefault(slot.constraint_iri or slot.id, []).append(slot)
    return groups


def authored_key(slot, motion, name: str) -> tuple[str, str]:
    """The (motion, constraint) a slot was authored as, from the IRI the header carries.

    A constraint IRI reads `<model>/<motion>/<phase>/<constraint>`, so it says which motion the
    slot serves. A name alone does not: one authored in several motions -- an elbow held
    everywhere -- would otherwise take the first motion's line for all of them.
    """
    # A generated conjunction has no motion of its own in its IRI, but what it watches does:
    # it is the motion's `until`, so it belongs in that motion, not in a block beside it.
    for iri in (slot.constraint_iri, *(member.iri for member in slot.watched)):
        segments = PurePosixPath(urlparse(iri or "").path).parts
        if len(segments) >= 3 and segments[-2] in ("while", "until", "when"):
            return (_key(segments[-3]), _key(name))
    return (_key(motion.id), _key(name))


def _constraint_row(motion, kind: str, group: list, constants: dict, authored: dict) -> dict:
    spelled = {_key(name): name for _, _, name in authored.values()}
    """One constraint's row: its identity and signals from the header, its line from the source."""
    first = group[0]
    name = first.constraint_id or rdf_name(first.constraint_iri) or first.id
    key = authored_key(first, motion, name)
    line, expression, authored_motion = authored.get(key, (None, None, None))
    # The header names the pair the evaluator compares; measured/setpoint only where it does not.
    operands = list(dict.fromkeys(value for slot in group for value in slot.operand_ids))
    compared = operands or [*_slot_ids(group, "measured_id"), *_slot_ids(group, "setpoint_id")]
    evaluator = next(iter(_slot_ids(group, "evaluator_id")), None)
    return {
        # As the source spells it, so a generated row lands in the motion's own block
        "motion": authored_motion or spelled.get(key[0]) or key[0] or motion.id,
        "handler": motion.id,
        "line": line,
        "name": name,
        "expression": expression,
        "kind": kind,
        # The closure the header names, or the slot computing the error where it names none.
        "evaluator": evaluator or first.iri or first.id or None,
        "between": compared,
        "tracking": [value for value in compared if value not in constants],
        # An aggregate monitor's scalar says only that every member holds; each member says why,
        # and they are not one plot: a velocity and a distance share no axis.
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
        ),
        "error": _slot_ids(group, "error_id"),
        "control": _slot_ids(group, "output_id"),
        "monitors": [slot.id for slot in group] if kind == "monitored" else [],
        "setpoints": [
            {"label": value, "value": constants[value]}
            for value in _slot_ids(group, "setpoint_id")
            if value in constants
        ],
        "gains": {
            GAIN_LABELS.get(gain.role, gain.role): gain.value
            for slot in group
            for gain in slot.gains
        },
        "tolerance": next(
            (constants[slot.tolerance_id] for slot in group if slot.tolerance_id in constants), None
        ),
    }


def source_constraints(run_dir: Path, manifest: dict | None, contract) -> list[dict]:
    """Return one row per constraint this run recorded, read off the log's own header.

    The header says which constraint every controller and monitor slot serves, which quantity
    ids carry its error, output, measurement and setpoint, which constant holds its tolerance
    and which gains the controller runs -- so the join is on identity the run wrote down rather
    than on names recovered from the source, and a run plots wherever it is stored.

    The row shape is a contract with the frontend: motion, handler, line, name, expression,
    kind, evaluator, between, tracking, error, control, monitors, setpoints, gains, tolerance.
    """
    authored = authored_lines(run_source_text(run_dir, manifest))
    constants = {
        key: row.value
        for row in contract.header.constants
        for key in (row.id, row.source_id)
        if key
    }
    return [
        _constraint_row(motion, kind, group, constants, authored)
        for motion in contract.header.motions
        for kind, slots in (("controlled", motion.controllers), ("monitored", motion.monitors))
        for group in _by_constraint(slots).values()
    ]


SLOT_ROLES = (
    ("error_id", "error"),
    ("output_id", "output"),
    ("measured_id", "measured"),
    ("setpoint_id", "setpoint"),
    ("tolerance_id", "tolerance"),
)


def signal_index(contract) -> dict:
    """signal id -> what it is: the motion and constraint it belongs to, and its role there.

    A picker offering `constraint_0.error` beside `eacc_ctrl_home_x` asks the reader to know
    the schema. The header says which constraint each slot serves, so say that instead.
    """
    index: dict[str, dict] = {}
    for motion in contract.header.motions:
        # a controller writes a constraint slot; a monitor writes a monitor slot
        for pool, slots, keys in (
            (
                "constraints",
                motion.controllers,
                ("error", "output", "measured", "setpoint", "satisfied"),
            ),
            ("monitors", motion.monitors, ("value", "satisfied")),
        ):
            for slot in slots:
                owner = slot.constraint_id or rdf_name(slot.constraint_iri) or slot.id
                fields = contract.fields.get(pool) or ()
                field = fields[slot.number]["id"] if slot.number < len(fields) else None
                named = {role for attribute, role in SLOT_ROLES if getattr(slot, attribute, "")}
                for key in keys:
                    where = {"motion": motion.id, "constraint": owner, "role": key, "slot": slot.id}
                    # The pooled slot mirrors a signal the header already names: offering both
                    # is the same series twice, under a name nobody can read.
                    if field and key not in named:
                        index[f"{field}.{key}"] = where
                    if pool == "monitors":
                        index[f"{slot.id}.{key}"] = where
                for attribute, role in SLOT_ROLES:
                    signal = getattr(slot, attribute, "")
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
                for member in slot.watched:
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
    return index


def replay_data(run_dir: Path) -> dict:
    """Return replay metadata without decoding the complete frame log."""
    pending = False
    try:
        run_dir, log, manifest, contract = resolve_archive(run_dir)
    except ArchiveError:
        # A run named but not yet writing: the generation carries the same header record the
        # runtime will put at the front of the log, and that record is itself a zero-frame
        # log -- so the page is built from it and follows the real log when it begins.
        record = run_dir.parent.parent / "generated/contract/frame_log_header.pb"
        if not record.is_file():
            raise
        log, manifest, contract = record, None, frame_log_pb.read_contract(record)
        pending = True
    constraints = source_constraints(run_dir, manifest, contract)
    health = read_health(log) or {}
    frame_count = health.get("written_frames", 0)
    signals = ["timing.compute_ms", "timing.period_ms"]
    mirrored = {
        f"{contract.fields['constraints'][slot.number]['id']}.{role}"
        for motion in contract.header.motions
        for slot in motion.controllers
        if slot.number < len(contract.fields["constraints"])
        for attribute, role in SLOT_ROLES
        if getattr(slot, attribute, "")
    }
    signals.extend(
        name
        for field in contract.fields["constraints"]
        for key in ("error", "output", "measured", "setpoint", "satisfied")
        if (name := f"{field['id']}.{key}") not in mirrored
    )
    signals.extend(
        f"{slot.id}.{key}"
        for motion in contract.header.motions
        for slot in motion.monitors
        for key in ("value", "satisfied")
    )
    signals.extend(slot_signals(contract))
    indices = {motion.id: motion.index for motion in contract.header.motions}
    spelling: dict[str, str] = {}  # gate id -> the motion name the source uses
    windows = log_events(log, contract)["windows"]
    logged = set(signals) | {field["id"] for field in contract.fields["quantities"]}
    parts = {}
    for name in logged:
        prefix, _, _ = name.partition(".")
        parts.setdefault(prefix, []).append(name)
    for constraint in constraints:
        constraint["monitors"] = [
            f"{name}.{key}" for name in constraint["monitors"] for key in ("value", "satisfied")
        ]
        for key in ("tracking", "control", "monitors", "error"):
            constraint[key] = [
                signal
                for name in constraint[key]
                for signal in ([name] if name in logged else sorted(parts.get(name, ())))
            ]
        handler = constraint.pop("handler")
        spelling.setdefault(handler, constraint["motion"])
        constraint["window"] = windows.get(indices.get(handler))
    signals.extend(
        signal
        for constraint in constraints
        for signal in (*constraint["tracking"], *constraint["control"], *constraint["monitors"])
    )
    # A run copied out of its generation has none to go back to; the log says everything else.
    generation = run_dir.parent.parent
    return {
        "generation": (
            str(generation.relative_to(roots.GENERATIONS))
            if roots.GENERATIONS.resolve() in generation.resolve().parents
            else None
        ),
        "frames": frame_count,
        "duration": frame_count * contract.header.nominal_period_ns / 1e9,
        "states": [state.id for state in contract.header.fsm_states],
        "events": log_events(log, contract)["events"],
        "signals": list(dict.fromkeys(signals)),
        "signal_index": {
            name: {**where, "motion": spelling.get(where["motion"], where["motion"])}
            for name, where in signal_index(contract).items()
        },
        "constraints": constraints,
        # The live poll names the motion by its gate; the panel is headed by the authored name.
        "motion_names": spelling,
        # A `when` guard's monitor runs while the PREDECESSOR motion is active: the contract's
        # owner, not the block it was authored in, says whose window carries its data.
        "monitor_owners": {
            slot.id: spelling.get(motion.id, motion.id)
            for motion in contract.header.motions
            for slot in motion.monitors
        },
        "videos": run_videos(run_dir),
        "header": validate_header(log, contract),
        "health": health,
        "pending": pending,
    }


_EVENTS: dict[str, dict] = {}


def log_events(log: Path, contract) -> dict:
    """Every FSM state entry, fired event and satisfied edge in one log, scanned incrementally.

    Events come off the frame's trigger ring, not `last_event`: several events fire on one
    transition and `last_event` holds only one of them, while the ring carries every fire of
    that tick (`trigger_count` entries, each naming its event by index). The satisfied edges are
    the same ones runtime_graph projects into occurrences -- rise and fall for a goal
    constraint, rise only for a monitor -- so a marker and an occurrence say the same thing
    about the same tick. A log the runtime is still writing resumes at the byte the last call
    stopped at, latches intact, so the live page pays for its new frames only; a finished log is
    never rescanned.
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
            "trigger_names": [field["name"] for field in contract.fields.get("triggers", [])],
            # Slot indices are motion-local: the active motion says which controller and
            # monitor slot i is, and a slot that motion does not claim is not written at all.
            "by_motion": {
                motion.index: (
                    {slot.number: slot for slot in motion.controllers},
                    {slot.number: slot for slot in motion.monitors},
                )
                for motion in contract.header.motions
            },
            "constraint_names": [field["name"] for field in contract.fields["constraints"]],
            "monitor_names": [field["name"] for field in contract.fields["monitors"]],
            "state_was": None,
            "event_was": None,
            "motion_was": None,
            "csat_was": None,
            "msat_was": None,
            "result": {"events": [], "windows": {}},
        }
    if size > scan["size"]:
        scan["size"] = size
        _extend_events(log, contract, scan)
    return scan["result"]


def _extend_events(log: Path, contract, scan: dict) -> None:
    """Append the markers of the frames written since the scan's offset, advancing it."""
    states, fired = scan["states"], scan["fired"]
    trigger_names = scan["trigger_names"]
    events, windows = scan["result"]["events"], scan["result"]["windows"]
    index = scan["index"]
    state_was, event_was, motion_was = scan["state_was"], scan["event_was"], scan["motion_was"]
    csat_was, msat_was = scan["csat_was"], scan["msat_was"]
    with frame_log_pb.open_log(log) as fh:
        while True:
            data, next_offset = frame_log_pb._read_delimited_at(fh, scan["offset"])
            if data is None:
                break
            scan["offset"] = next_offset
            record = contract.record_cls()
            record.ParseFromString(data)
            if record.WhichOneof("record") != "frame":
                continue
            frame = record.frame
            if trigger_names:
                for slot in range(min(int(frame.trigger_count), len(trigger_names))):
                    entry = getattr(frame, trigger_names[slot])
                    if 0 <= entry.idx < len(fired):
                        events.append({"frame": index, "kind": "event", "label": fired[entry.idx]})
            elif frame.last_event != event_was:
                event_was = frame.last_event
                if 0 <= event_was < len(fired):
                    events.append({"frame": index, "kind": "event", "label": fired[event_was]})
            controllers, monitors = scan["by_motion"].get(frame.active_motion, ({}, {}))
            csat = [
                slot in controllers and bool(getattr(frame, name).satisfied)
                for slot, name in enumerate(scan["constraint_names"])
            ]
            msat = [
                slot in monitors and bool(getattr(frame, name).satisfied)
                for slot, name in enumerate(scan["monitor_names"])
            ]
            # The latch is only valid within one state and motion: across a change slot i is
            # a different controller, so the projection compares nothing there either.
            held = frame.fsm_state == state_was and frame.active_motion == motion_was
            if frame.fsm_state != state_was:
                state_was = frame.fsm_state
                label = states[state_was] if 0 <= state_was < len(states) else str(state_was)
                events.append({"frame": index, "kind": "state", "label": label})
            if held:
                for slot, (now, before) in enumerate(zip(csat, csat_was)):
                    # only goal constraints, not pure regulation -- runtime_graph's own rule
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


def signal_reader(contract):
    """Map a signal name onto a raw frame, the way the plots read one.

    The plot history and the live increments read the same log; sharing the lookup means a
    signal cannot mean one thing while the run writes and another once it is finished.
    """
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
            return float(getattr(frame, field["name"]))
        if name in slots:
            kind, field, attribute = slots[name]
            gate = contract.gate.get(kind)
            if gate is not None and field["index"] not in gate.get(frame.active_motion, ()):
                return None
            return float(getattr(getattr(frame, field["name"]), attribute))
        prefix, _, key = name.rpartition(".")
        if prefix in constraint_slots:
            field = contract.fields["constraints"][constraint_slots[prefix]]
            return getattr(getattr(frame, field["name"]), key)
        if prefix in monitor_slots:
            field, owner = monitor_slots[prefix]
            if frame.active_motion != owner:
                return None
            return getattr(getattr(frame, field["name"]), key)
        raise ValueError(f"unknown signal: {name}")

    return value


def plot_data(run_dir: Path, names: list[str], window: tuple | None = None) -> dict:
    """Stream and downsample requested fields without shaping complete frames.

    Sampling follows the window asked for, so a motion that lasted a handful of frames is
    drawn from those frames rather than missed between two samples of the whole run.
    """
    _, log, _manifest, contract = resolve_archive(run_dir)
    value = signal_reader(contract)
    health = read_health(log) or {}
    first, last = window or (0, max(0, health.get("written_frames", 0) - 1))
    step = max(1, (last - first + 1) // 1600)
    series = {name: [] for name in names}
    index = 0
    with frame_log_pb.open_log(log) as fh:
        frame_log_pb._read_delimited(fh)
        while data := frame_log_pb._read_delimited(fh, partial_ok=True):
            record = contract.record_cls()
            record.ParseFromString(data)
            if record.WhichOneof("record") != "frame":
                continue
            frame = record.frame
            if first <= index <= last and (index - first) % step == 0:
                for name in names:
                    series[name].append(value(frame, name))
            index += 1
    return {
        "signals": series,
        "events": log_events(log, contract)["events"],
        "sample_step": step,
        "first_frame": first,
    }
