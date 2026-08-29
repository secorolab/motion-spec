# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

"""Rank by when a candidate left the reference, not by how far it ended up outside it.

v1 ranks on the excess a run finished with, and an overdriven gain drives its neighbours further
than itself: what tops that ranking is a row the mutation disturbed, not the row it changed. The
cause moves first, so v2 orders candidates by the tick each one first left the reference envelope
and keeps the excess only to separate candidates that left on the same tick.

The envelope is read tick by tick from the entry of the state a tick belongs to, so a run that only
runs late stays inside it. What such a run does show is its events firing late, which is the
monitors' feature: the candidates are the constraints the controllers drive *and* the monitors that
gate the transitions, ranked against each other on one clock.

Nothing here reads the mutation. The site's identity is used afterwards and only to say where in
the finished ranking the mutated element landed.
"""

import math
from dataclasses import dataclass, field
from pathlib import Path

from motion_spec.mutation import metric, scorer

# What a frame reports per constraint row. A mutated gain shows in `output` on the tick it runs, a
# mutated constant in `setpoint`, and a flipped direction in `measured`, all before the robot has
# moved far enough for `error` to answer for any of them.
CHANNELS = ("setpoint", "measured", "error", "output")

REL_TOL = 0.05  # of the channel's own reference scale, so its unit drops out
ABS_TOL = 1e-6
EVENT_TICKS = 4  # firing-time slack, in control periods, on top of the reference spread
DEPTH = 4  # dataflow hops from an authored value to the elements that consume it


@dataclass
class Reference:
    """What the unmutated runs say a run of this model looks like."""

    period: float
    slots: dict[int, list[dict]]
    # (state, slot) -> channel -> (low, high) per tick since that state was entered, and the
    # largest magnitude the channel reached, which is the scale its tolerance is read against.
    bands: dict[tuple[int, int], dict[str, tuple[list[float], list[float]]]] = field(
        default_factory=dict
    )
    scales: dict[tuple[int, int], dict[str, float]] = field(default_factory=dict)
    events: list[str] = field(default_factory=list)
    event_band: dict[str, tuple[float, float]] = field(default_factory=dict)
    event_monitors: dict[str, list[dict]] = field(default_factory=dict)
    rows: dict[str, dict] = field(default_factory=dict)
    monitors: dict[str, dict] = field(default_factory=dict)
    closures: dict[str, dict] = field(default_factory=dict)
    dataflow: dict[str, dict] = field(default_factory=dict)
    produced: dict[str, list[str]] = field(default_factory=dict)
    candidates: list[str] = field(default_factory=list)


def load(campaign: Path) -> Reference:
    """Build the reference from a campaign directory: its unmutated generation and its runs."""
    generation = _generation(campaign)
    ir = scorer.introspection(generation)
    introspection = ir["communication"]["introspection"]
    rows = {row["id"]: row for row in introspection["controllers"]}
    monitors = {row["id"]: row for row in introspection["monitors"]}
    slots = scorer.state_controllers(generation, ir)
    reference = Reference(
        period=float(ir["configuration"]["control_period_ns"]) / 1e9,
        slots=slots,
        events=scorer.enum_order(generation, ir, "e_events"),
        rows=rows,
        monitors=monitors,
        closures=ir["computation"]["closures"],
        dataflow=introspection["dataflow"],
    )
    for name, node in reference.dataflow.items():
        producer = (node.get("producer") or {}).get("id")
        if producer:
            reference.produced.setdefault(producer, []).append(name)
    for row in monitors.values():
        reference.event_monitors.setdefault(row["event_name"], []).append(row)

    # Every element a mutation can reach, whatever site it was made at: the constraint each
    # controller drives, and every monitor. Enumerated from the model, not from the mutation.
    seen: dict[str, None] = {}
    for state in sorted(slots):
        for row in slots[state]:
            seen.setdefault(row["constraint_uri"])
    for row in monitors.values():
        seen.setdefault(row["uri"])
    reference.candidates = list(seen)

    runs = sorted(campaign.glob("reference/reference*.jsonl"))
    if not runs:
        raise RuntimeError(f"{campaign}: no reference runs to build an envelope from")
    firings: dict[str, list[float]] = {}
    for run in runs:
        frames = metric.load_frames(run)  # one run at a time: they are large
        _widen(frames, reference)
        for event, when in _event_times(frames, reference.events).items():
            firings.setdefault(event, []).append(when)
    for event, times in firings.items():
        if len(times) == len(runs):  # an event some reference run never fired has no band
            reference.event_band[event] = (min(times), max(times))
    return reference


def score(frames: list[dict], reference: Reference, name: str) -> dict:
    """The v2 ranking, and where the mutated element sits in it."""
    ranked = rank(frames, reference)
    targets = target_uris(name, reference)
    result = {
        "target_rank": None,
        "target_uris": sorted(targets),
        "target_deviated": False,
        "n_ranked": len(ranked),
        "n_deviating": sum(1 for _uri, onset, _worst in ranked if math.isfinite(onset)),
        "top": [
            [uri, None if math.isinf(onset) else onset, worst] for uri, onset, worst in ranked[:3]
        ],
    }
    for position, (uri, onset, _worst) in enumerate(ranked, start=1):
        if uri in targets:
            result["target_rank"] = position
            result["target_deviated"] = math.isfinite(onset)
            break
    return result


def rank(frames: list[dict], reference: Reference) -> list[tuple[str, float, float]]:
    """Every candidate as (uri, onset, excess), earliest departure from the reference first.

    Onset is the ranking; excess only separates candidates that left on the same tick. A candidate
    that never left is carried with an infinite onset, so it ranks last but is still ranked.
    """
    onset, worst = _constraints(frames, reference)
    for uri, when, excess in _monitors(frames, reference):
        onset[uri] = min(onset.get(uri, math.inf), when)
        worst[uri] = max(worst.get(uri, 0.0), excess)
    return sorted(
        (
            (uri, onset.get(uri, math.inf), worst.get(uri, 0.0))
            for uri in reference.candidates  # a stable order behind the two features
        ),
        key=lambda item: (item[1], -item[2]),
    )


def target_uris(name: str, reference: Reference) -> set[str]:
    """The candidates the mutated element reaches. Evaluation only -- never used to rank.

    A site names a controller, a monitor, or an authored value, and the last of those reaches its
    constraints and monitors only through what consumes it, so the dataflow graph is walked out
    from it until every branch has landed on something a run reports.
    """
    sanitized = name.replace("-", "_")
    seeds = {row["id"] for row in reference.rows.values() if _named(row["id"], sanitized)}
    seeds |= {row["id"] for row in reference.monitors.values() if row["id"] == sanitized}
    for node, flow in reference.dataflow.items():
        # A direction is authored as one term and flows as its components.
        if node == sanitized or node.startswith(f"{sanitized}."):
            seeds |= {consumer["id"] for consumer in flow.get("consumers", ()) if consumer["id"]}
    return _resolve(seeds, reference)


def _resolve(ids: set[str], reference: Reference) -> set[str]:
    """Walk ids forward through the dataflow until each lands on a candidate."""
    found: set[str] = set()
    seen: set[str] = set()
    frontier = set(ids)
    for _hop in range(DEPTH):
        following = set()
        for identifier in frontier - seen:
            seen.add(identifier)
            if identifier in reference.rows:
                found.add(reference.rows[identifier]["constraint_uri"])
                continue
            if identifier in reference.monitors:
                found.add(reference.monitors[identifier]["uri"])
                continue
            closure = reference.closures.get(identifier)
            constraint = (closure or {}).get("constraint_uri")
            if constraint is not None:
                found |= _consumers_of(constraint, reference)
                continue
            # A closure that evaluates no constraint of its own -- a geometry computation -- is
            # followed through the quantities it produces to whoever reads them.
            for quantity in reference.produced.get(identifier, ()):
                node = reference.dataflow.get(quantity) or {}
                following |= {
                    consumer["id"] for consumer in node.get("consumers", ()) if consumer["id"]
                }
        frontier = following - seen
        if not frontier:
            break
    return found


def _consumers_of(constraint: str, reference: Reference) -> set[str]:
    """The candidates that answer for one constraint: its controllers, else its monitors."""
    driven = {
        row["constraint_uri"]
        for row in reference.rows.values()
        if row["constraint_uri"] == constraint
    }
    watching = {
        row["uri"]
        for row in reference.monitors.values()
        if constraint in row.get("constraint_uris", ())
        or any(watched["uri"] == constraint for watched in row.get("watched", ()))
    }
    return driven | watching


def _constraints(
    frames: list[dict], reference: Reference
) -> tuple[dict[str, float], dict[str, float]]:
    """Per constraint: when it first left the band, and the widest excess it reached."""
    onset: dict[str, float] = {}
    worst: dict[str, float] = {}
    for when, state, tick, rows in _walk(frames):
        for slot, row in enumerate(rows):
            if not row.get("active"):
                continue
            band = reference.bands.get((state, slot))
            controller = _controller(reference, state, slot)
            if band is None or controller is None:
                continue  # a state and slot the reference never ran: no envelope to leave
            scale = reference.scales[(state, slot)]
            uri = controller["constraint_uri"]
            for channel, (low, high) in band.items():
                if tick >= len(low) or low[tick] > high[tick]:
                    continue  # the reference never ran this state this long
                value = float(row.get(channel) or 0.0)
                slack = REL_TOL * scale[channel] + ABS_TOL
                excess = max(low[tick] - value, value - high[tick]) - slack
                if excess <= 0.0:
                    continue
                onset.setdefault(uri, when)
                worst[uri] = max(worst.get(uri, 0.0), excess / (scale[channel] or 1.0))
    return onset, worst


def _monitors(frames: list[dict], reference: Reference) -> list[tuple[str, float, float]]:
    """Monitors whose event fired outside the reference's firing band, and when that showed.

    A late event is answered for from the moment it was due, not from the moment it finally fired:
    that is the tick a reader watching this run could first say it was late, and it is what puts
    the monitor that delayed a transition ahead of every monitor downstream of it.
    """
    times = _event_times(frames, reference.events)
    end = float(frames[-1].get("t") or 0.0) if frames else 0.0
    found = []
    for event, (low, high) in reference.event_band.items():
        gating = reference.event_monitors.get(event)
        if not gating:
            continue  # an event a reaction fires has no monitor to hold responsible
        slack = max(high - low, EVENT_TICKS * reference.period)
        fired = times.get(event)
        if fired is None:
            when, shift = high + slack, end - high
        elif fired > high + slack:
            when, shift = high + slack, fired - high
        elif fired < low - slack:
            when, shift = fired, low - fired
        else:
            continue
        excess = abs(shift) / max(high, reference.period)
        found += [(monitor["uri"], when, excess) for monitor in gating]
    return found


def _widen(frames: list[dict], reference: Reference) -> None:
    """Widen the per-tick band and the per-channel scale with one reference run."""
    for _when, state, tick, rows in _walk(frames):
        for slot, row in enumerate(rows):
            if not row.get("active"):
                continue
            band = reference.bands.setdefault((state, slot), {})
            scale = reference.scales.setdefault((state, slot), {})
            for channel in CHANNELS:
                value = float(row.get(channel) or 0.0)
                low, high = band.setdefault(channel, ([], []))
                while len(low) <= tick:
                    low.append(math.inf)
                    high.append(-math.inf)
                low[tick] = min(low[tick], value)
                high[tick] = max(high[tick], value)
                scale[channel] = max(scale.get(channel, 0.0), abs(value))


def _walk(frames: list[dict]):
    """Each frame as (t, state, ticks since that state was entered, constraint rows).

    Ticks are counted from the state's entry rather than from the run's start, so a run that
    reaches the same state later is still compared against the reference tick for tick.
    """
    state, tick = None, 0
    for frame in frames:
        if frame["fsm_state"] != state:
            state, tick = frame["fsm_state"], 0
        yield float(frame.get("t") or 0.0), state, tick, frame.get("constraints") or []
        tick += 1


def _event_times(frames: list[dict], events: list[str]) -> dict[str, float]:
    """When each event first fired."""
    first: dict[str, float] = {}
    for frame in frames:
        index = frame.get("last_event", -1)
        if index is not None and 0 <= index < len(events):
            first.setdefault(events[index], float(frame.get("event_t") or frame.get("t") or 0.0))
    return first


def _controller(reference: Reference, state: int, slot: int) -> dict | None:
    rows = reference.slots.get(state, [])
    return rows[slot] if slot < len(rows) else None


def _named(identifier: str, sanitized: str) -> bool:
    """One authored controller expands into a row per axis, named after the block it came from."""
    return identifier == sanitized or identifier.startswith(f"{sanitized}_")


def _generation(campaign: Path) -> Path:
    for candidate in sorted(campaign.glob("reference/gen/*/*")):
        if (candidate / "generated" / "model" / "ir.json").is_file():
            return candidate
    raise RuntimeError(f"{campaign}: no unmutated generation under reference/gen")
