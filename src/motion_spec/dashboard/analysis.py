# SPDX-License-Identifier: MPL-2.0

"""Statistics over a recorded run: coherent oscillation, contact, torque saturation.

Time comes from `step * nominal_period_ns`, never the frame's `t`. Two runs of the same
model, both reporting `dt_measured_s == 0.001`, disagreed by 2.9x on `t` per step, which
inverts any duration comparison built on it. Why `t` drifts is unknown and out of scope --
nothing here reads it.

The three reports share one sweep: a finished run's log is ~160 MB and opening it three
times is the cost that matters. So the builders take the series that sweep produced rather
than a log path, and `run_reports` is the one place a log is opened.

What a signal means is discovered from the run's own contract -- the quantity ids it
records, the wrench and twist slots it declares, the constants its monitors are judged by --
never from a model's names.
"""

from __future__ import annotations

import math
import re
import statistics
from pathlib import Path

from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.replay import resolve_archive

PERIOD_FALLBACK_S = 1e-3


def period_seconds(header) -> float:
    """Control period. Falls back to 1 ms for archives written before the header carried one;
    a caller shows durations derived from the fallback as approximate rather than measured."""
    return (header.nominal_period_ns / 1e9) if header.nominal_period_ns else PERIOD_FALLBACK_S


def run_seconds(step: int, header) -> float:
    return step * period_seconds(header)


# The frame says what was active as an FSM state index. A behaviour-tree target is coming and
# will carry its own field; scoping goes through these two so that is one edit, not a sweep.
def _scope(frame) -> int:
    return frame.fsm_state


def _scope_labels(contract) -> list[str]:
    return [state.id for state in contract.header.fsm_states]


def sweep(log, contract, ids=(), *, states=None, stride: int = 1):
    """One pass over the log: the spans it occupied and the named series, positionally aligned.

    A slot the active motion does not write is a hole rather than a decoded zero, exactly as
    `replay.signal_reader` gates it -- proto3 elides a genuine 0.0 the same way it elides
    "never written".
    """
    quantities = {field["id"]: field for field in contract.fields["quantities"]}
    wanted = [quantities[name] for name in ids if name in quantities]
    gate = contract.gate.get("quantities")
    labels = _scope_labels(contract)
    period = period_seconds(contract.header)
    series: dict[str, list] = {field["id"]: [] for field in wanted}
    spans: list[dict] = []
    scope = motion = None
    allowed: set | None = None
    open_span: dict | None = None
    index = seen = 0
    with frame_log_pb.open_log(log) as fh:
        frame_log_pb._read_delimited(fh)
        while data := frame_log_pb._read_delimited(fh, partial_ok=True):
            record = contract.record_cls()
            record.ParseFromString(data)
            if record.WhichOneof("record") != "frame":
                continue
            frame = record.frame
            if frame.active_motion != motion:
                motion = frame.active_motion
                allowed = None if gate is None else set(gate.get(motion, ()))
            if _scope(frame) != scope:
                scope = _scope(frame)
                # A re-entry is its own row: two occupancies of one state are two spans.
                open_span = {
                    "state_index": len(spans),
                    "state_id": labels[scope] if 0 <= scope < len(labels) else str(scope),
                    "entered_step": frame.step,
                    "exited_step": frame.step,
                    "first": index,
                    "last": index - 1,
                    "motions": [],
                }
                spans.append(open_span)
            open_span["exited_step"] = frame.step
            if motion not in open_span["motions"]:
                open_span["motions"].append(motion)
            seen += 1
            if (states is not None and open_span["state_index"] not in states) or (
                (seen - 1) % stride
            ):
                continue
            open_span["last"] = index
            index += 1
            for field in wanted:
                series[field["id"]].append(
                    getattr(frame, field["name"])
                    if allowed is None or field["index"] in allowed
                    else None
                )
    for span in spans:
        span["ticks"] = span["exited_step"] - span["entered_step"] + 1
        span["seconds"] = span["ticks"] * period
    if states is not None:
        spans = [span for span in spans if span["state_index"] in states]
    return spans, series


def state_spans(log, contract) -> list[dict]:
    """Contiguous occupancies of the scoping key, in order, a re-entry its own row.

    Private to signal scoping. The runtime graph is the record of what happened when; if the
    two ever disagree on a boundary the graph wins and the disagreement is a bug.
    """
    return sweep(log, contract)[0]


def signal_series(log, contract, ids, *, states=None, stride: int = 1) -> dict[str, list]:
    """Named series. An id the run does not record is omitted rather than filled with None,
    so a caller can tell "not recorded" from "recorded as zero"."""
    return sweep(log, contract, ids, states=states, stride=stride)[1]


def detrend(values, window: int = 201) -> list[float]:
    """Centred moving-average residual. Over a whole motion the motion's own sweep dominates
    any spectrum taken of it, and buries the mode that is worth seeing."""
    half = window // 2
    total = [0.0]
    for value in values:
        total.append(total[-1] + value)
    count = len(values)
    return [
        value
        - (total[min(count, i + half + 1)] - total[max(0, i - half)])
        / (min(count, i + half + 1) - max(0, i - half))
        for i, value in enumerate(values)
    ]


def dominant_frequency(values, period_s: float, lo=0.5, hi=10.0, step=0.25) -> tuple[float, float]:
    """The probed frequency carrying the most amplitude, and that amplitude.

    A direct probe of ~40 bands, not an FFT: the band is what the question is about and the
    whole sweep is milliseconds, so this costs nothing a dependency would buy back.
    """
    count = len(values)
    best = (0.0, 0.0)
    for probe in range(round((hi - lo) / step) + 1):
        hz = lo + probe * step
        turn = 2 * math.pi * hz * period_s
        real = imaginary = 0.0
        for i, value in enumerate(values):
            angle = turn * i
            real += value * math.cos(angle)
            imaginary += value * math.sin(angle)
        amplitude = 2 * math.hypot(real, imaginary) / count
        if amplitude > best[1]:
            best = (hz, amplitude)
    return best


JOINT_SIGNAL = re.compile(r"(?P<solver>.+)_(?P<role>qd|tau_cmd|tau_ctrl)_joint_(?P<joint>\d+)$")


def joint_signals(contract) -> dict[str, dict[str, dict[str, str]]]:
    """solver -> role -> joint -> quantity id, from the ids the run actually recorded.

    Partitioned by solver because a model may drive more than one chain: two arms have no
    mechanical reason to agree, so pooling their joints would read an accidental agreement as
    a mode that does not exist.
    """
    found: dict[str, dict[str, dict[str, str]]] = {}
    for name in contract.quantity_ids:
        match = JOINT_SIGNAL.fullmatch(name)
        if match:
            solver = found.setdefault(match["solver"], {})
            solver.setdefault(match["role"], {})[match["joint"]] = name
    return found


def component_signals(contract, category: str, kind: str) -> dict[str, dict[str, str]]:
    """Slot id -> axis -> quantity id, for the slots whose three components the run records.

    A slot recorded on one axis is a commanded push, not a measurement: an external wrench or
    a body twist is logged whole.
    """
    recorded = set(contract.quantity_ids)
    found = {}
    for slot in contract.fields[category]:
        axes = {axis: f"{slot['id']}_{kind}_{axis}" for axis in "xyz"}
        if recorded.issuperset(axes.values()):
            found[slot["id"]] = axes
    return found


def monitor_bands(contract) -> dict[int, dict[str, dict]]:
    """Motion -> quantity id -> the gate watching it there and the tolerance it is judged by.

    The header names each monitor's operands and its tolerance constant, so the band a signal
    is held to is a join on identity the run wrote down. Same constants map as
    `replay.source_constraints`: a constant answers to its own id and to the source's.

    Per motion, because one quantity is held to different bands in different motions: a
    placement's settling band is not the band the same velocity was touched down under.
    """
    constants = {
        key: row.value
        for row in contract.header.constants
        for key in (row.id, row.source_id)
        if key
    }
    bands: dict[int, dict[str, dict]] = {}
    for motion in contract.header.motions:
        for slot in motion.monitors:
            band = constants.get(slot.tolerance_id)
            if band is None:
                continue
            for operand in slot.operand_ids:
                bands.setdefault(motion.index, {}).setdefault(
                    operand, {"band": band, "gate": slot.id, "motion": motion.id}
                )
    return bands


def _slice(series, name, span):
    return series[name][span["first"] : span["last"] + 1]


def _values(series, name, span):
    return [value for value in _slice(series, name, span) if value is not None]


# The slowest band a spectrum is probed at; a span too short to hold two of its cycles has
# no spectrum to take.
SPECTRUM_FLOOR_HZ = 0.5


def coherent_modes(spans, series, period_s, joints, *, min_fraction=0.5, window=201) -> list[dict]:
    """Per state and per chain, the frequency several joints agree on.

    Coherence is the finding, not amplitude: a tray carried through an arc wobbled visibly
    while six of seven joints peaked at one frequency; the same arc without the tray had
    scattered peaks fifty times smaller. `mode_hz` is None when fewer than `min_fraction` of
    the chain's joints land in one probe band.
    """
    rows = []
    for solver, roles in sorted(joints.items()):
        ids = [name for _joint, name in sorted(roles.get("qd", {}).items())]
        for span in spans:
            if not ids or span["seconds"] < 2 / SPECTRUM_FLOOR_HZ:
                continue
            peaks = {}
            for name in ids:
                values = _values(series, name, span)
                if len(values) > window:
                    peaks[name] = dominant_frequency(detrend(values, window), period_s)
            if not peaks:
                continue
            counts = {}
            for hz, _amplitude in peaks.values():
                counts[hz] = counts.get(hz, 0) + 1
            mode, agreeing = max(counts.items(), key=lambda item: (item[1], -item[0]))
            amplitudes = [amplitude for _hz, amplitude in peaks.values()]
            rows.append(
                {
                    "solver": solver,
                    "state_index": span["state_index"],
                    "state_id": span["state_id"],
                    "seconds": span["seconds"],
                    "mode_hz": mode if agreeing >= min_fraction * len(peaks) else None,
                    "agreeing": agreeing,
                    "joints": len(peaks),
                    "mean_amplitude": sum(amplitudes) / len(amplitudes),
                    "peaks": {name: hz for name, (hz, _amplitude) in peaks.items()},
                }
            )
    return rows


def contact_events(
    spans, series, period_s, forces, velocities, bands, *, threshold_n=10.0
) -> list[dict]:
    """Per state, the impact the external wrench recorded and what the body did after it.

    A contact is a rise in the wrench, not a level: an arm already pressing carries the force
    from the state before. The plan's 15 N would have missed a measured 13.6 N placement, and
    this arm's touchdown lands at ~11 N, so the rise that counts is 10 N.

    Settling is judged against the band the run's own gate holds that velocity to, so ripple
    is reported against what was declared rather than against a number chosen here.
    """
    debounce = max(1, round(0.1 / period_s))
    rows = []
    for span in spans:
        wrench = [_slice(series, forces[axis], span) for axis in "xyz"]
        # A tick any axis did not write is not a wrench; dropping it keeps the axes aligned.
        recorded = [
            (i, components) for i, components in enumerate(zip(*wrench)) if None not in components
        ]
        if not recorded:
            continue
        magnitude = [math.hypot(math.hypot(x, y), z) for _i, (x, y, z) in recorded]
        baseline = statistics.median(magnitude[:50] or magnitude)
        peak = max(range(len(magnitude)), key=magnitude.__getitem__)
        if magnitude[peak] - baseline < threshold_n:
            continue
        impact, components = recorded[peak]
        # The axis the impact came in on, so the rebound is read where the arm was pushed.
        axis = "xyz"[max(range(3), key=lambda a: abs(components[a]))]
        after = [
            value for value in _slice(series, velocities[axis], span)[impact:] if value is not None
        ]
        # Only a gate that was actually running in this span says what the band was.
        band = next(
            (
                held
                for motion in span["motions"]
                if (held := bands.get(motion, {}).get(velocities[axis]))
            ),
            None,
        )
        settle = ripple = None
        if band and len(after) > debounce:
            limit = band["band"]
            settle = next(
                (
                    i
                    for i in range(len(after) - debounce)
                    if all(abs(value) < limit for value in after[i : i + debounce])
                ),
                None,
            )
            if settle is not None:
                tail = after[settle:]
                ripple = max(tail) - min(tail)
        rows.append(
            {
                "state_index": span["state_index"],
                "state_id": span["state_id"],
                "peak_n": magnitude[peak],
                "baseline_n": baseline,
                "axis": axis,
                "impact_step": span["entered_step"] + impact,
                "rebound": max(after, key=abs) if after else None,
                "settle_s": None if settle is None else settle * period_s,
                "ripple": ripple,
                "band": band["band"] if band else None,
                "gate": band["gate"] if band else None,
            }
        )
    return rows


def saturation_report(spans, series, joints, limits=()) -> list[dict]:
    """Where the torque a controller asked for is not the torque that was applied.

    Self-sufficient by construction: the requested/applied pair says it was clipped without
    any declared limit, which matters because `Constant.consumers[]` is empty for most
    constants and nothing may be built on it. A declared limit only names the bound.
    """
    rows = []
    for solver, roles in sorted(joints.items()):
        requested, applied = roles.get("tau_ctrl", {}), roles.get("tau_cmd", {})
        for joint in sorted(requested.keys() & applied.keys()):
            clipped = {"ticks": 0, "limit_nm": 0.0, "requested_nm": 0.0, "states": []}
            for span in spans:
                for want, have in zip(
                    _slice(series, requested[joint], span), _slice(series, applied[joint], span)
                ):
                    if want is None or have is None or abs(want) <= abs(have) + 1e-9:
                        continue
                    clipped["ticks"] += 1
                    clipped["limit_nm"] = max(clipped["limit_nm"], abs(have))
                    clipped["requested_nm"] = max(clipped["requested_nm"], abs(want))
                    if span["state_id"] not in clipped["states"]:
                        clipped["states"].append(span["state_id"])
            if clipped["ticks"]:
                named = [name for name, value in limits if value == clipped["limit_nm"]]
                rows.append(
                    {
                        "solver": solver,
                        "joint": joint,
                        "signal": applied[joint],
                        # The bound is unattributed unless exactly one constant carries it.
                        "limit_id": named[0] if len(named) == 1 else None,
                        **clipped,
                    }
                )
    return rows


def watched_signals(spans, series, bands) -> list[dict]:
    """Per occupancy, what each signal a gate judges there actually did.

    Restricted to signals a monitor references, which is the restriction `monitor_bands`
    already draws: every other recorded quantity moves for reasons nothing in the model has
    an opinion about, and comparing all of them buries the few that were judged.

    Per occupancy and per motion, because one quantity is held to different bands in
    different motions -- the same restriction, and the same reason, as the contact bands.
    """
    rows = []
    for span in spans:
        watched = {
            signal: band
            for motion in span["motions"]
            for signal, band in bands.get(motion, {}).items()
        }
        for signal, band in sorted(watched.items()):
            # A gate may watch an operand the run records no quantity for, exactly as `sweep`
            # drops an id it cannot find: omitted, never filled in with a zero.
            values = _values(series, signal, span) if signal in series else []
            if not values:
                continue
            rows.append(
                {
                    "state_index": span["state_index"],
                    "state_id": span["state_id"],
                    "signal": signal,
                    "gate": band["gate"],
                    "band": band["band"],
                    "peak": max(values, key=abs),
                    "pk_pk": max(values) - min(values),
                    "rms": math.sqrt(sum(value * value for value in values) / len(values)),
                }
            )
    return rows


def run_reports(run_dir: Path | str) -> dict:
    """All four reports off one sweep of the run's frame log."""
    _run_dir, log, _manifest, contract = resolve_archive(Path(run_dir))
    period = period_seconds(contract.header)
    joints = joint_signals(contract)
    wrenches = component_signals(contract, "wrenches", "force")
    twists = component_signals(contract, "twists", "linear")
    # A run recording several whole wrenches or twists is read on the first of each.
    forces = next(iter(wrenches.values()), {})
    velocities = next(iter(twists.values()), {})
    bands = monitor_bands(contract)
    ids = [
        name
        for roles in joints.values()
        for role in ("qd", "tau_cmd", "tau_ctrl")
        for name in roles.get(role, {}).values()
    ]
    ids += [*forces.values(), *velocities.values()]
    ids += sorted({signal for motion in bands.values() for signal in motion} - set(ids))
    spans, series = sweep(log, contract, ids)
    limits = [(row.id, row.value) for row in contract.header.constants]
    return {
        "period_s": period,
        "period_exact": bool(contract.header.nominal_period_ns),
        "states": [
            {key: span[key] for key in ("state_index", "state_id", "entered_step", "seconds")}
            for span in spans
        ],
        "oscillation": coherent_modes(spans, series, period, joints),
        "contact": (
            contact_events(spans, series, period, forces, velocities, bands)
            if forces and velocities
            else []
        ),
        "saturation": saturation_report(spans, series, joints, limits),
        "signals": watched_signals(spans, series, bands),
    }
