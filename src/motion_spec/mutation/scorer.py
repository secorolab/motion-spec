# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

"""Rank the constraints a deviation points at, and say where the mutated one landed in that rank.

The slot a frame carries is positional; the reference generation's IR is what turns it back into a
constraint URI. Every mutant is scored against the same reference IR, so the ranks are comparable.
"""

import json
import re
from pathlib import Path

SLOT = re.compile(r"s(\d+)/c(\d+)")
ENUM_ENTRY = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)")


def introspection(generation: Path) -> dict:
    """The reference generation's IR."""
    return json.loads((generation / "generated" / "model" / "ir.json").read_text())


def enum_order(generation: Path, ir: dict, enum: str) -> list[str]:
    """One generated FSM enum, in the order its integers were assigned.

    Read off the generated enum, because that is what the integer in a frame means. The IR's own
    lists are sorted by name and say nothing about which state or event is which number.
    """
    header = generation / "generated" / "controller" / ir["coordination"]["fsm"]["header"]
    pattern = re.compile(rf"enum {enum}\s*\{{(.*?)\}}", re.DOTALL)
    match = pattern.search(header.read_text())
    if match is None:
        raise ValueError(f"{header}: no {enum} enum to read the numbering from")
    return [name for name in ENUM_ENTRY.findall(match.group(1)) if not name.startswith("NUM_")]


def state_order(generation: Path, ir: dict) -> list[str]:
    """The FSM's states in the order their integers were assigned."""
    return enum_order(generation, ir, "e_states")


def state_controllers(generation: Path, ir: dict) -> dict[int, list[dict]]:
    """Controller rows per FSM state index, in slot order.

    The state a frame reports is the generated enum's integer; the handler that runs in that state
    names its controllers in the order the frame's constraint slots follow.
    """
    states = state_order(generation, ir)
    rows = {row["id"]: row for row in ir["communication"]["introspection"]["controllers"]}
    handlers = {
        handler["id"]: handler for handler in ir["communication"]["introspection"]["motions"]
    }
    slots = {}
    for motion in ir["coordination"]["motions"]:
        handler = handlers.get(motion["id"])
        if handler is None or motion["fsm_state"] not in states:
            continue
        slots[states.index(motion["fsm_state"])] = [
            rows[name] for name in handler["controllers"] if name in rows
        ]
    return slots


def score(
    deviation: dict,
    slots: dict[int, list[dict]],
    operator: str,
    name: str,
    element_uri: str | None = None,
) -> dict:
    """Rank the constraints by excess, and report where the mutated one sits in that ranking.

    A site that already knows the element it damaged is evaluated against that element instead of
    against what its operator tag implies. v1 ranks constraints, so an authored value that is not
    itself a constraint stays unranked here -- which is v1's answer, not a missing one.
    """
    ranked = rank_constraints(deviation["excess"], slots)
    deviated = (
        any(value > 0.0 for value in deviation["excess"].values())
        or deviation["sequence_differs"]
        or deviation["incomplete"]
    )
    targets = {element_uri} if element_uri else target_uris(operator, name, slots)
    result = {
        "deviated": deviated,
        "target_rank": None,
        "n_ranked": len(ranked),
        "top": [[uri, excess] for uri, excess in ranked[:3]],
        # The same ranking with the constraints stripped back out: what a reader who only has the
        # frame layout can say. It is the baseline the attributable ranking is worth more than.
        "signal_baseline": {
            "ranking": [key for key, _excess in rank_slots(deviation["excess"], slots)],
            "attributable": False,
        },
    }
    if targets is None:
        result["monitor_target"] = True
        return result
    for position, (uri, _excess) in enumerate(ranked, start=1):
        if uri in targets:
            result["target_rank"] = position
            break
    return result


def rank_constraints(excess: dict[str, float], slots: dict[int, list[dict]]) -> list:
    """Constraint URIs by their worst excess across states, worst first."""
    worst: dict[str, float] = {}
    for key, value in excess.items():
        row = _row(key, slots)
        if row is None:
            continue
        uri = row["constraint_uri"]
        worst[uri] = max(worst.get(uri, 0.0), value)
    return sorted(worst.items(), key=lambda item: -item[1])


def rank_slots(excess: dict[str, float], slots: dict[int, list[dict]]) -> list:
    """The same ranking over the same keys, named by (state, slot) alone.

    A state with no handler -- the start and end states -- carries no controller to rank, so it is
    left out of both rankings and the two stay comparable.
    """
    ranked = [(key, value) for key, value in excess.items() if _row(key, slots) is not None]
    return sorted(ranked, key=lambda item: -item[1])


def target_uris(operator: str, name: str, slots: dict[int, list[dict]]) -> set[str] | None:
    """The constraints the mutation touched, or None when it touched a monitor instead.

    A monitor mutation moves when a transition fires, not what a controller drives, so there is no
    constraint for it to be ranked against.
    """
    if operator.startswith("debounce"):
        return None
    sanitized = name.replace("-", "_")
    found = set()
    for rows in slots.values():
        for row in rows:
            if operator.startswith("gain"):
                hit = row["id"] == sanitized
            elif operator.startswith("scale_constant"):
                hit = sanitized in (row.get("setpoint_signal"), row.get("tolerance_signal"))
            else:
                # A direction is not a signal of its own: it reaches the row through the constraint
                # it names and the quantity measured along it.
                hit = sanitized in row.get("constraint", "") or sanitized in row.get(
                    "measured_signal", ""
                )
            if hit:
                found.add(row["constraint_uri"])
    return found


def _row(key: str, slots: dict[int, list[dict]]) -> dict | None:
    match = SLOT.fullmatch(key)
    if match is None:
        return None
    rows = slots.get(int(match.group(1)), [])
    slot = int(match.group(2))
    return rows[slot] if slot < len(rows) else None
