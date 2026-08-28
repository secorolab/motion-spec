# Plans

Implementation plans written by `/improve`, each self-contained for an executor with no context
from the session that produced it. Every plan stamps the commit it was written against; check
`git rev-parse --short HEAD` before executing and re-read excerpted files if it has moved.

A plan is deleted once its work has landed -- the commits are the record. What survives it is
below: the designs that were weighed and turned down, so they are not proposed again.

| # | Plan | Depends on | Effort | Status |
|---|------|-----------|--------|--------|
| — | none open | | | |

## Considered and rejected

- **A `views` clause beside `publishers`/`subscribers`**, for naming where a camera is
  published. It would have left the pose-observing subscriber and its validation completely
  untouched, at the cost of one more keyword. Rejected: a camera provider *is* a subscription,
  and saying so once is worth the conditional `pose from` clause.
- **Declaring the camera's topic in `robot.toml`, as `[ros.joint_states]` already does.**
  Deployment-side, no regeneration, no new terms, and the dashboard already reads that file for
  the devices panel. Rejected: the graph would never learn the camera is observed.
- **`sosa:madeBySensor` for the camera a channel carries.** The correct SOSA sense -- the camera
  is the sensor, not the feature being observed. Rejected in favour of overloading
  `sosa:hasFeatureOfInterest`, so a subscription lowers to one shape. The consequence is that
  every reader of that predicate must branch on the target's type.
- **Deleting the Reports surface** (`reports.js`, `/api/reports`, `analysis.py`, its tests --
  759 lines). Proposed and reverted: `analysis.py` is signal analysis, not report plumbing --
  oscillation modes via detrend + `dominant_frequency`, contact from wrench against twist,
  torque saturation against the contract's constants, all off one sweep because a finished log
  is ~160 MB. None of it is replaceable by a saved query.
