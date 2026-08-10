# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

import pytest
from rdf_utils.constraints import ConstraintViolation

from motion_spec.classes.motion import MotionUnit
from motion_spec.rdf_parser.coordination import _apply_fsm_wiring

NS = "https://example.org/fsm/"


def fsm(states, transitions, reactions):
    """An FSM framed the way read_fsm hands it to the wiring."""
    return {
        "name": "probe_fsm",
        "start_state": "S_START",
        "end_state": "S_DONE",
        "states": states,
        "state_uris": {state: f"{NS}{state}" for state in states},
        "events": ["E_STEP", "E_HELD"],
        "transitions_table": [
            {"id": tid, "uri": f"{NS}{tid}", "from_state": source, "to_state": target}
            for tid, source, target in transitions
        ],
        "reactions_table": [
            {
                "id": rid,
                "uri": f"{NS}{rid}",
                "when_event": event,
                "do_transition": transition,
                "fires_events": [],
                "num_fires": 0,
            }
            for rid, event, transition in reactions
        ],
        "namespace_uri": NS,
    }


def motion(mid, state):
    unit = MotionUnit(
        id=mid,
        handler="",
        name=mid,
        description=[],
        when_evaluators=[],
        while_evaluators=[],
        until_evaluators=[],
        controllers=[],
        when_monitors=[],
        while_monitors=[],
        until_monitors=[],
        when_schedule=[],
        while_schedule=[],
        until_schedule=[],
    )
    unit.fsm_state = state
    return unit


def test_a_state_the_fsm_can_sit_in_with_no_motion_is_rejected() -> None:
    # S_WAIT is entered and never left: the dispatch would render no case for it, so the loop
    # steps nothing while the FSM is there and the arm keeps the last staged command.
    document = fsm(
        ["S_START", "S_WAIT", "S_HOLD", "S_DONE"],
        [("T_START_WAIT", "S_START", "S_WAIT"), ("T_WAIT_HOLD", "S_WAIT", "S_HOLD")],
        [("R_STEP", "E_STEP", "T_START_WAIT")],
    )
    with pytest.raises(ConstraintViolation, match="S_WAIT"):
        _apply_fsm_wiring([motion("m_hold", "S_HOLD")], document)


def test_the_start_and_end_states_need_no_motion() -> None:
    # The heartbeat leaves S_START before the FSM dwells there, and the loop breaks on S_DONE
    # ahead of the dispatch, so neither has to run anything.
    document = fsm(
        ["S_START", "S_HOLD", "S_DONE"],
        [("T_START_HOLD", "S_START", "S_HOLD"), ("T_HOLD_DONE", "S_HOLD", "S_DONE")],
        [("R_STEP", "E_STEP", "T_START_HOLD"), ("R_HELD", "E_HELD", "T_HOLD_DONE")],
    )
    _apply_fsm_wiring([motion("m_hold", "S_HOLD")], document)


def test_a_state_no_reaction_transitions_into_is_not_reachable() -> None:
    # S_ORPHAN is named by a transition, but no reaction fires it, so the FSM can never be there
    # and demanding a motion for it would reject a model that is fine.
    document = fsm(
        ["S_START", "S_HOLD", "S_ORPHAN", "S_DONE"],
        [("T_START_HOLD", "S_START", "S_HOLD"), ("T_HOLD_ORPHAN", "S_HOLD", "S_ORPHAN")],
        [("R_STEP", "E_STEP", "T_START_HOLD")],
    )
    _apply_fsm_wiring([motion("m_hold", "S_HOLD")], document)
