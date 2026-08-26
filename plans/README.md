# Plans

Implementation plans written by `/improve`, each self-contained for an executor with no context
from the session that produced it. Every plan stamps the commit it was written against; check
`git rev-parse --short HEAD` before executing and re-read excerpted files if it has moved.

| # | Plan | Depends on | Effort | Status |
|---|------|-----------|--------|--------|
| 001 | [Fold the Views page into the transport timeline](001-fold-views-into-transport.md) | — | L | TODO |

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

## Considered and rejected

- **Deleting the Reports surface** (`reports.js`, `/api/reports`, `analysis.py`, its tests —
  759 lines). Proposed and reverted: `analysis.py` is signal analysis, not report plumbing —
  oscillation modes via detrend + `dominant_frequency`, contact from wrench against twist,
  torque saturation against the contract's constants, all off one sweep because a finished log
  is ~160 MB. None of it is replaceable by a saved query. Plan 001 makes it reachable again
  instead.
