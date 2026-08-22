# SPDX-License-Identifier: MPL-2.0
"""Timeline markers: state entries, fired events and the satisfied edges under them."""

from __future__ import annotations

from motion_spec.dashboard import replay
from motion_spec.introspection import frame_log_pb

from dashboard_fixture import CONSTRAINT, CTRL, schema
from frame_log_fixture import flat_frame, write_frame_log_pb
from support import _hash_doc

SETTLE = "https://example.test/S_SETTLE"


def _two_motion_schema() -> dict:
    """Two motions sharing constraint slot 0, so a slot means what the active motion says.

    Slot 1 is a regulation controller: it names no constraint, so its satisfied bit is not a goal
    reached and carries no marker.
    """
    doc = schema()
    doc["pools"]["constraints"] = 2
    doc["fsm"]["states"].append({"index": 1, "id": "S_SETTLE", "uri": SETTLE})
    doc["by_motion"]["move"]["controllers"] = [
        {"index": 0, "id": "ctrl_x", "uri": CTRL, "constraint_uri": CONSTRAINT},
        {"index": 1, "id": "regulate_x", "uri": "https://example.test/regulate_x"},
    ]
    doc["by_motion"]["settle"] = {
        "index": 1,
        "id": "settle",
        "uri": "https://example.test/settle",
        "controllers": [
            {
                "index": 0,
                "id": "ctrl_settle",
                "uri": "https://example.test/ctrl_settle",
                "constraint_uri": "https://example.test/constraint_settle",
            }
        ],
        "monitors": [],
    }
    doc["schema_hash"] = _hash_doc(doc)
    return doc


# fsm_state, active_motion, and the three satisfied bits the frame carries.
FRAMES = [
    (0, 0, 0, 0, 0),  # 0: entering S_MOVE
    (0, 0, 1, 1, 0),  # 1: the goal is reached; the regulation slot is not a goal
    (0, 0, 0, 1, 0),  # 2: and lost again
    (0, 0, 1, 1, 1),  # 3: reached, and the monitor fires on the same tick
    (0, 0, 1, 1, 1),  # 4: nothing moved
    (0, 1, 0, 0, 0),  # 5: another motion owns slot 0 now -- no edge across the change
    (0, 1, 1, 0, 0),  # 6: that motion's own goal
    (1, 1, 0, 0, 0),  # 7: S_SETTLE; no edge across a state change either
    (1, 1, 1, 0, 0),  # 8: reached again, under the new state
]


def _log(tmp_path):
    doc = _two_motion_schema()
    flats = [
        flat_frame(
            doc,
            t=index / 1000,
            step=index,
            fsm_state=state,
            active_motion=motion,
            last_event=-1,
            **{"c0.satisfied": c0, "c1.satisfied": c1, "m0.satisfied": m0},
        )
        for index, (state, motion, c0, c1, m0) in enumerate(FRAMES)
    ]
    log = tmp_path / "frame_log.pb"
    write_frame_log_pb(log, doc, flats)
    return log, frame_log_pb.read_contract(log)


def test_the_scan_marks_state_entries_and_satisfied_edges(tmp_path):
    log, contract = _log(tmp_path)
    marks = [
        (event["frame"], event["kind"], event["label"])
        for event in replay.log_events(log, contract)["events"]
    ]
    assert marks == [
        (0, "state", "S_MOVE"),
        (1, "satisfied", "ctrl_x"),
        (2, "unsatisfied", "ctrl_x"),
        (3, "satisfied", "ctrl_x"),
        (3, "monitor", "done_mon"),
        (6, "satisfied", "ctrl_settle"),
        (7, "state", "S_SETTLE"),
        (8, "satisfied", "ctrl_settle"),
    ]


def test_the_motion_windows_and_the_cache_survive_the_extra_edges(tmp_path):
    log, contract = _log(tmp_path)
    scanned = replay.log_events(log, contract)
    assert scanned["windows"] == {0: [0, 4], 1: [5, 8]}
    # One scan per (path, size): the second read is the first read's answer, not another pass.
    assert replay.log_events(log, contract) is scanned
