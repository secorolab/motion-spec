# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

"""What a decoded run looks like, and how far a mutant's run falls outside the reference envelope.

A run is reduced to the peak error each constraint reached in each state, the states it passed
through, and where it ended. The reference runs of the unmutated model set the envelope: anything
above what an unmutated run ever reached is the mutation showing through the noise.
"""

import json
from pathlib import Path

# Peak error per (state, slot). Slots are positional: slot n is the nth controller of the motion
# that state runs, which is what the frame carries and what the IR names.
KEY = "s{state}/c{slot}"


def load_frames(path: Path) -> list[dict]:
    """The decoded frames of one run."""
    with path.open() as source:
        return [json.loads(line) for line in source if line.strip()]


def features(frames: list[dict]) -> dict:
    """Peak error per (state, slot), the state sequence, and where the run ended."""
    peak: dict[str, float] = {}
    sequence: list[int] = []
    final_state, final_t = None, 0.0
    for frame in frames:
        state = frame["fsm_state"]
        if not sequence or sequence[-1] != state:
            sequence.append(state)
        for slot, constraint in enumerate(frame.get("constraints") or []):
            if not constraint.get("active"):
                continue
            error = abs(float(constraint.get("error") or 0.0))
            key = KEY.format(state=state, slot=slot)
            if error > peak.get(key, 0.0):
                peak[key] = error
        final_state, final_t = state, float(frame.get("t") or 0.0)
    return {"peak": peak, "sequence": sequence, "final_state": final_state, "final_t": final_t}


def envelope(reference: list[dict]) -> dict:
    """The widest each key got across the reference runs, and the ends they reached."""
    peak: dict[str, float] = {}
    for feature in reference:
        for key, value in feature["peak"].items():
            peak[key] = max(peak.get(key, 0.0), value)
    return {
        "peak": peak,
        "sequences": [feature["sequence"] for feature in reference],
        "final_states": [feature["final_state"] for feature in reference],
        "final_t": max((feature["final_t"] for feature in reference), default=0.0),
    }


def deviation(feature: dict, reference: dict) -> dict:
    """How far each key exceeds the envelope, and whether the run went somewhere else entirely.

    A key the envelope never saw is excess in full: the reference never reached that state and
    slot at all, so there is nothing to subtract.
    """
    excess = {}
    for key, value in feature["peak"].items():
        bound = reference["peak"].get(key)
        excess[key] = value if bound is None else max(0.0, value - bound)
    return {
        "excess": excess,
        "sequence_differs": feature["sequence"] not in reference["sequences"],
        "incomplete": feature["final_state"] not in reference["final_states"],
    }
