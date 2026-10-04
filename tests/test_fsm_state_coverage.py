# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

import pytest
from rdf_utils.constraints import ConstraintViolation

from motion_spec.classes.handlers import EdgeMonitor
from motion_spec.classes.motion import MotionUnit
from motion_spec.rdf_parser.fsm import apply_fsm_wiring

NS = "https://example.org/fsm/"

SEEN_MONITOR = EdgeMonitor(
    id="mon",
    monitor_type="EdgeTriggeredMonitor",
    error=None,
    event="e_seen",
    event_idx=None,
    event_uri=f"{NS}E_SEEN",
    event_name="E_SEEN",
    fallback_motion="hold",
)


@pytest.mark.parametrize(
    ("states", "transitions", "reactions", "events", "motions", "rejection"),
    [
        (
            ["S_START", "S_LOOK", "S_DONE"],
            [("T_START_LOOK", "S_START", "S_LOOK"), ("T_LOOK_DONE", "S_LOOK", "S_DONE")],
            [("R_STEP", "E_STEP", "T_START_LOOK"), ("R_HELD", "E_HELD", "T_LOOK_DONE")],
            ["E_STEP", "E_HELD", "E_SEEN"],
            [("m_look", None, "", [SEEN_MONITOR]), ("m_hold", "S_LOOK", "hold", [])],
            "names no\n?\\s*'runs-in' state",
        ),
        # S_WAIT is entered and never left: the dispatch would render no case for it, so the loop
        # steps nothing while the FSM is there and the arm keeps the last staged command.
        (
            ["S_START", "S_WAIT", "S_HOLD", "S_DONE"],
            [("T_START_WAIT", "S_START", "S_WAIT"), ("T_WAIT_HOLD", "S_WAIT", "S_HOLD")],
            [("R_STEP", "E_STEP", "T_START_WAIT")],
            ["E_STEP", "E_HELD"],
            [("m_hold", "S_HOLD", "", [])],
            "S_WAIT",
        ),
    ],
    ids=["in-state-gate-without-a-state", "state-with-no-motion"],
)
def test_every_state_the_fsm_can_hold_in_has_a_motion(
    states, transitions, reactions, events, motions, rejection
) -> None:
    """The FSM is framed the way coord-dsl hands it to the wiring."""
    document = {
        "name": "probe_fsm",
        "start_state": "S_START",
        "end_state": "S_DONE",
        "states": states,
        "state_uris": {state: f"{NS}{state}" for state in states},
        "events": events,
        "event_uris": {event: f"{NS}{event}" for event in events},
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
    units = [
        MotionUnit(
            id=mid,
            motion_id=motion_id,
            name=mid,
            description=[],
            when_evaluators=[],
            while_evaluators=[],
            until_evaluators=[],
            controllers=[],
            when_monitors=when_monitors,
            while_monitors=[],
            until_monitors=[],
            when_schedule=[],
            while_schedule=[],
            until_schedule=[],
            fsm_state=state,
        )
        for mid, state, motion_id, when_monitors in motions
    ]
    with pytest.raises(ConstraintViolation, match=rejection):
        apply_fsm_wiring(units, document, [])
