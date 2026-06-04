# FSM integration — open items

The namespaced-monitor-event → coord2b-FSM wiring is implemented and verified end-to-end (pick_place
walks `S_START … S_DONE`, one event per state). A WHEN precondition is never an empty guard/wait
state: its monitor names an explicit `fallback <hold-motion>`, and the FSM enters that hold motion's
state (full `update + monitor + control + apply`) where it holds pose and re-evaluates the WHEN each
tick, advancing to the gated motion when the success event fires. Codegen rejects an FSM-wired WHEN
with no fallback. The fallback can be the *preceding motion itself* (it holds its goal pose), so a whole
task can be fully WHEN-gated: every transition is driven by the successor's WHEN precondition rather than
the predecessor's UNTIL (pick_place does this end-to-end). As-built reference lives in the code and the
`fsm-event-wiring` memory note. Remaining open items:

## Constant poses in a WHEN are not materialized for the standalone precondition check
A WHEN evaluated in the fallback state runs as `monitor_when_<gated>` only (no `update_<gated>`), so its
schedule must be self-contained. Scalar/`distance between` comparisons are (their compute graph is in the
when_schedule). But a constructed constant `Pose` (e.g. `pre-place-pose` built from spec scalars) is
materialized in `update_<motion>` via `declared_pose_components`, *not* in the when_schedule — so inside
`monitor_when_` it is identity and `ee equal to <that-pose>` never fires. Workaround in models: gate on a
single discriminating axis (`ee.position.y greater than …`) instead of a full-pose equality. Real fix:
emit the when-referenced (snapshot-free) declared pose materializations inside `monitor_when_` too.

## Monitor rising-edge on state re-entry
`mon_*_previous` (rising_edge state) is no longer reset on entry, so a looping FSM that re-enters a
state could miss or stale an edge. Correct for linear FSMs (pick_place); looping states need an entry
reset (same `_fsm_prev_state` hook).

## Dead legacy fields
`mon_*_event_triggered` is write-only everywhere; `active`/`active_steps` are now only used by the
legacy non-FSM `switch(current_motion)` path. Remove together with that path if/when it is retired.
