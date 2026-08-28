# Plans

Implementation plans written by `/improve`, each self-contained for an executor with no context
from the session that produced it. Every plan stamps the commit it was written against; check
`git rev-parse --short HEAD` before executing and re-read excerpted files if it has moved.

| # | Plan | Depends on | Effort | Status |
|---|------|-----------|--------|--------|
| 001 | [Fold the Views page into the transport timeline](001-fold-views-into-transport.md) | — | L | TODO |
| 002 | [Declare the camera's provider instead of guessing its topic](002-declare-the-camera-provider.md) | — | M | DONE |

## 001 in one line

The run page draws the same time axis twice — instants on the transport, intervals in a Views
table. Motion spans become bars on the transport's own axis, gates become wait-segments joined
to the monitor markers already there, the Views page is deleted, and its run-vs-run Compare view
moves to a Reports tab that also un-parks the existing oscillation/contact/saturation reports.

The plan's load-bearing constraints, in case they get lost in a summary:

- **The transport must not grow.** `#content`'s `padding-bottom: 130px` (twice) and `.replay`'s
  `calc(100dvh - 139px)` are hardcoded against its height, and `--transport-h` does not drive
  them. Motion bars go *in* the existing 26 px strip; the only height-varying part — the
  expanded constraint lane — is an overlay anchored above it.
- **No JS measurement.** Layout is pure CSS by repo rule.
- **The transport keeps rendering without a fetch.** Markers come from the frame log and must
  paint immediately; spans and gates arrive afterwards and are an enhancement, never a
  precondition.
- **Live runs report `projected`, not `archive`**, and in that state `waited_s` and
  `rearm_count` are `None` by design. The tooltip must not invent them.
- **`CONSTRAINTS_DURING_ACTIVITY` returns names, not spans**, so the expand lane needs a new
  per-occurrence query. It must stay bound to one occurrence — unbound it is a cross join that
  costs ten seconds on a real run.

## 002 in one line

The run page invents the ROS topic a real robot's camera is published on — `/${camera.id}/color`,
built in JavaScript. A `.robmot` subscription names the scene's camera instead, the topic reaches
the dashboard through the generation's contract, and a camera with no declared provider gets no
live pane.

The plan's load-bearing constraints, in case they get lost in a summary:

- **No new RDF term.** The link rides the existing `sosa:hasFeatureOfInterest`, decided
  explicitly over the semantically-better `sosa:madeBySensor`. That overloads the predicate:
  every future reader of it must branch on the target's type.
- **A camera subscription is defined by absence.** It carries no `pose from` clause, so
  `perceived_written_poses` yields it no rows, so `ros_subscriptions` skips it, so **no C++
  subscriber is generated**. No guard makes that true — it falls out. A refactor that iterates
  topics directly would silently put an image subscriber in the control loop.
- **`quantities.py:880` is the one thing that breaks.** It resolves every `ros:Topic` feature of
  interest as a pose; a camera target must be skipped there by its type in the graph.
- **`OBJECT` in the grammar is a textX built-in**, not a rule in this tree — `DeviceBinding`
  already points at a scene sensor with it. Widening to it removes the grammar-level type check,
  so validation has to carry it.
- **`src/bdd_collab_bhv_cpp` is not ours.** `collab_real.robmot` is the obvious first model to
  gain a `wrist-view` subscription; the plan stops at describing that diff.

## Considered and rejected

- **A `views` clause beside `publishers`/`subscribers`** (plan 002). It would have left the
  pose-observing subscriber and its validation completely untouched, at the cost of one more
  keyword. Rejected: a camera provider *is* a subscription, and saying so once is worth the
  conditional `pose from` clause.
- **Declaring the camera's topic in `robot.toml`, as `[ros.joint_states]` already does**
  (plan 002). Deployment-side, no regeneration, no new terms, and the dashboard already reads
  that file for the devices panel. Rejected: the graph would never learn the camera is observed.
- **Deleting the Reports surface** (`reports.js`, `/api/reports`, `analysis.py`, its tests —
  759 lines). Proposed and reverted: `analysis.py` is signal analysis, not report plumbing —
  oscillation modes via detrend + `dominant_frequency`, contact from wrench against twist,
  torque saturation against the contract's constants, all off one sweep because a finished log
  is ~160 MB. None of it is replaceable by a saved query. Plan 001 makes it reachable again
  instead.
