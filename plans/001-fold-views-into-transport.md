# 001 — Fold the Views page into the transport timeline

**Written against commit** `7a9ed20` (branch `dev`; a branch `feat/reports` exists, identical to `dev`).
**Repo** `/home/batsy/work/ms/src/motion-spec`
**Status** TODO

If `git rev-parse --short HEAD` is not `7a9ed20`, re-read every file this plan excerpts before trusting
the excerpts. Line numbers will have moved; the surrounding code should still match.

---

## Why

The run page shows the same time axis twice. The transport at the bottom of the page draws
**instants** — one marker per event at a position along the scrubber, label in a `title` tooltip.
The Views panel draws **intervals** — a table of occupancies with entry time and duration.

A span with a start and a duration is a bar. Rendering it as a table row is the weak form: the
reader compares durations by reading numbers instead of by looking at lengths, and the two
renderings of the same run sit on different pages so neither can be read against the other.

The change: motion spans become bars on the transport's own axis, gates become wait-segments on
that axis, the Views page is deleted, and its run-vs-run Compare view moves to a reachable
Reports page.

## What already exists (read this before designing anything)

### The axis

`src/motion_spec/dashboard/frontend/run.js:554`

```js
export function trackLeft(frame) {
  // the thumb travels inset by half its width, so markers must follow the same geometry
  return `calc(var(--thumb) / 2 + ${trackFraction(frame)} * (100% - var(--thumb)))`;
}
```

`trackFraction(frame)` is `frame / max(1, state.replay.frames - 1)`. **Every new element must be
positioned with these two functions and nothing else.** A bar spanning `[begin, end]` gets
`left: trackLeft(begin)` and a width computed from the same expression — see "Bar width" below.

### The strip

`src/motion_spec/dashboard/frontend/style.css:328`

```css
.markers { position: relative; height: 26px; margin-top: 8px; }
.markers::before, .markers::after { content: ""; position: absolute; left: calc(var(--thumb) / 2); right: calc(var(--thumb) / 2); height: 1px; background: var(--line); }
.markers::before { top: 1px; }
.markers::after { top: 15px; }
```

26 px tall with two hairlines, at `top: 1px` and `top: 15px`. Markers sit relative to those:
`.marker` (state) at `top: -5px`, `.marker-event` at `top: 12px`, `.marker-satisfied` /
`.marker-unsatisfied` at `top: 4px`, `.marker-monitor` at `top: 4px`.

`renderMarkers()` at `run.js:525` rebuilds the strip from `state.replay.events` — the frame log,
**no API call** — and caps satisfied/unsatisfied at `SATISFIED_MARKER_CAP = 12` per slot so a
chattering constraint cannot bury the state markers.

### The layout coupling — read this twice

The transport is `position: fixed; bottom: 0`. Three numbers are hardcoded against its height:

- `style.css:114` — `#content { overflow: auto; padding: 9px 22px 130px; }`
- `style.css:358` — the same `130px` inside a media query
- `style.css:~256` — `.replay { height: calc(100dvh - 139px); ... }`

`--transport-h` is set in `reserveVideoSpace()` (`run.js:126`) but is consumed **only** by
`.videos`. It does not drive `#content`.

**Therefore: the transport's own box must not change height.** If it grows, all three numbers
are wrong and the page ends up with content hidden behind the transport or a dead band above it.
The design below keeps the transport exactly 26 px in the strip and puts the only height-varying
element in an overlay that does not participate in the transport's box.

The repo rule is **pure CSS, no JS measurement**. Do not add `offsetHeight` reads to solve this.

### The data

`GET /api/run/timeline?path=<run>[&iri=<design-iri>]` → `queries.timeline()` at
`queries.py:421`:

```
{ "period_s": float|None,
  "runtime_source": "archive"|"projected"|"unavailable",
  "spans": [ { "occurrence", "element", "name", "begin_step", "end_step",
               "entered_s", "duration_s", "transition", "event", "satisfied" } ] }
```

**`ACTIVITY_TIMELINE` (queries.py:280) selects `?occ a ms-prov:MotionExecution` only.** The
timeline route already returns motions and nothing else, so the default lane needs no filtering.

`end_step` is `None` for a span that never closed. `duration_s` is `None` when `period_s` is
unknown or the span is open.

`GET /api/run/gates?path=<run>` → `queries.gates()` at `queries.py:463`:

```
{ "period_s", "runtime_source",
  "gates": [ { "monitor", "monitor_name", "event", "event_name", "members",
               "first_held_step", "fired_step", "waited_s", "rearm_count",
               "declared_dwell_s" } ] }
```

`waited_s` and `rearm_count` are **`None` unless `runtime_source == "archive"`** — see
`queries.py:472`, `archived = service.runtime_source == "archive"`. A projection has only
strided frames behind it, so waits are reported unavailable rather than as numbers a reader
would trust. `fired_step` is `None` for a gate that never fired.

`GET /api/run/compare?left=<run>&right=<run>` → `queries.compare()`. **This must survive.**

Both timeline and gates go through `views_graph()` and `_rows()`, which serialise on a
module-level `threading.Lock` (`queries.py:~355`) because rdflib parses SPARQL with pyparsing,
whose parser state is process-global. Two concurrent handlers killed both connections. Do not
remove or work around that lock.

### The gap you must fill

`CONSTRAINTS_DURING_ACTIVITY` (`queries.py:300`) returns **constraint names only, no steps**:

```sparql
SELECT DISTINCT ?constraint WHERE {
    ?occ time:hasBeginning/time:inTimePosition/time:numericPosition ?from ;
         time:hasEnd/time:inTimePosition/time:numericPosition ?to .
    ?held a ms-prov:ConstraintMaintenance ; prov:used ?constraint ;
          time:hasBeginning/time:inTimePosition/time:numericPosition ?heldFrom ;
          time:hasEnd/time:inTimePosition/time:numericPosition ?heldTo .
    ?constraint a cstr:Constraint .
    FILTER(?heldFrom >= ?from && ?heldTo <= ?to)
}
```

The expand-on-demand lane needs **spans**, so this query cannot serve it. Its own comment
records why `?occ` must stay bound:

> `?occ` is bound by the caller -- left open it is a cross join of every span against every
> other, which costs ten seconds on a real run to answer a question nobody asked of all eleven
> rows at once.

Honour that: the new query is **per-occurrence**, never for the whole run.

---

## Design

### Lane 1 — motion spans, always, in-band

Motion bars are drawn **behind** the existing markers inside the same 26 px strip, not above it.
A motion span is the context the markers occurred in, so it reads correctly as a background
stripe, and — decisively — it costs no height, so none of the three hardcoded numbers move.

- One `<button class="span span-motion">` per entry in `timeline.spans`.
- `left: trackLeft(begin_step)`.
- **Bar width**: the two `calc()` expressions subtract, so width is
  `calc((${trackFraction(end) - trackFraction(begin)}) * (100% - var(--thumb)))`. An open span
  (`end_step === null`) runs to `trackFraction(state.replay.frames - 1)`.
- `top: 0; bottom: 0;` inside `.markers`, `z-index: 0`, with the existing markers given
  `z-index: 1` so they stay on top. The bar is a low-opacity tint, alternating between two
  values by index so adjacent motions are distinguishable without a palette.
- The motion name is rendered inside the bar, clipped with `overflow: hidden`, so a wide span is
  labelled and a narrow one is not — no JS measurement, the browser clips.
- `title` carries `name`, `entered_s`, `duration_s` (or `open`), and `event`, which is the entry
  event the deleted table had a column for.
- Click seeks to `begin_step` (`seek()` from `run.js`), matching what the table's `seekable` rows
  did.

### Lane 2 — constraints, on demand, in an overlay

Clicking a motion bar expands **that motion's** constraint-maintenance spans.

- Rendered into a `<div class="span-expand">` that is `position: absolute; bottom: 100%; left: 0;
  right: 0;` **inside `.markers`**. It floats above the transport without being part of its box,
  so the transport's height never changes and the layout coupling is untouched.
- Its rows use the same `trackLeft` geometry, so a constraint bar sits directly above the part of
  the axis it covers.
- One row per constraint, capped — see "Caps" below.
- Fetched on first expand of that span and cached on the bar's dataset; a second click collapses.
- Only one span is expanded at a time. Expanding another collapses the first.

### Lane 3 — gates, in-band, joined to the marker that already exists

A gate is an interval (`first_held_step → fired_step`, the wait) plus an instant (`fired_step`).
`.marker-monitor` **already draws the instant** at `top: 4px`. Do not draw a second dot.

- Draw only the wait: a 2 px-tall segment at `top: 11px` (between the two hairlines) from
  `trackLeft(first_held_step)` to `trackLeft(fired_step)`, terminating at the existing monitor
  marker.
- `title`: the sentence the deleted `gateStory()` built — reuse its wording verbatim, including
  the `SLOW_GATE = 1.2` threshold for "waited N s beyond its dwell" and the "condition broke N×
  before firing" clause. Copy that function across rather than paraphrasing it; it is the one
  piece of Views worth keeping word for word.
- **A gate whose `fired_step` is `null` gets no segment** — there is no interval to draw.
- **When `runtime_source !== "archive"`, `waited_s` and `rearm_count` are `null` by design.**
  Draw the segment if both steps are present, but the tooltip must not invent a wait; fall back
  to the `first_held_step`/`fired_step` numbers and say the source is a projection.

### Live vs archived

The transport must keep working during a run, and must not block on the graph.

1. `renderMarkers()` keeps rendering from `state.replay.events` immediately, exactly as now. **Do
   not make the frame-log markers wait on a fetch.** This is the property that makes the
   transport usable the instant the page opens.
2. Spans and gates are fetched *after*, and painted when they arrive. A failed fetch leaves the
   strip exactly as it is today — bars are an enhancement, never a precondition.
3. The strip carries the provenance note as a `title` on `.markers` itself, reusing the three
   strings from the deleted `sourceNote()`: `"from the archived runtime graph"`, `"projected from
   a run still going — waits and re-arms not yet known"`, `"no runtime graph recorded yet"`.
4. During a live run the newest motion span is open (`end_step === null`) and its bar runs to the
   current end of the log. Refetch when the run settles: `settle(false)` is called in
   `live.js` when the archive lands — hook the refetch there, not on a timer.

### Caps

`renderMarkers()` already caps satisfied markers at 12 per slot with a comment explaining why. A
run with many motions or a motion with many constraints needs the same discipline:

- Motion bars: no cap needed (a pick-place run has ~8–12; they do not overlap).
- Constraint rows in the expand overlay: cap at **12 rows**, and `log`-style say so in the
  overlay ("showing 12 of 31") rather than silently truncating. Silent truncation reads as "that
  is all there was".

---

## Steps

### Step 1 — backend: constraint spans for one occurrence

**File** `src/motion_spec/dashboard/queries.py`

Add a query beside `CONSTRAINTS_DURING_ACTIVITY`, modelled on it but selecting the spans, and a
function beside `timeline()`. Match the file's conventions: module-level SPARQL constant built
from `VIEW_PREFIXES`, a comment above it explaining the *why* not the *what*, and results read
through `_rows(service, QUERY, occ=...)` with `_step()` / `_seconds()` for positions.

```python
CONSTRAINT_SPANS_DURING_ACTIVITY = (
    VIEW_PREFIXES
    + """
SELECT ?constraint ?heldFrom ?heldTo WHERE {
    ?occ time:hasBeginning/time:inTimePosition/time:numericPosition ?from ;
         time:hasEnd/time:inTimePosition/time:numericPosition ?to .
    ?held a ms-prov:ConstraintMaintenance ; prov:used ?constraint ;
          time:hasBeginning/time:inTimePosition/time:numericPosition ?heldFrom ;
          time:hasEnd/time:inTimePosition/time:numericPosition ?heldTo .
    ?constraint a cstr:Constraint .
    FILTER(?heldFrom >= ?from && ?heldTo <= ?to)
} ORDER BY ?heldFrom
"""
)


def activity_constraints(run_dir: Path, occurrence: str) -> dict:
    """The constraint spans held inside one occupancy, for the lane that expands under it.

    Bound to one occurrence, never left open: unbound this is a cross join of every span
    against every other, which costs ten seconds on a real run.
    """
```

Return `{"period_s", "runtime_source", "spans": [{"constraint", "name", "begin_step",
"end_step", "entered_s", "duration_s"}]}` — the same key names the motion spans use, so the
frontend can render both with one function.

**STOP and report back** if `occurrence` cannot be passed as an `rdflib.URIRef` binding the way
`timeline()` binds `element` — do not fall back to string interpolation into the query text.

**File** `src/motion_spec/dashboard/server.py`

Add `/api/run/constraints` next to the other `run/` routes (they are at lines ~316–326), taking
`path` and `occ`. Add it to `LAN_GET_ALLOWED` (line ~120) beside `/api/run/timeline` — it is a
read, like its neighbours.

**Verify**

```
cd /home/batsy/work/ms/src/motion-spec
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MOTION_SPEC_GENERATIONS=/home/batsy/work/ms/generations \
  /home/batsy/work/ms/.venv/bin/python -m pytest tests/test_dashboard_views.py -q
```

Expected: still passes (this step adds, changes nothing).

### Step 2 — frontend: the three lanes

**File** `src/motion_spec/dashboard/frontend/run.js`

`renderMarkers()` keeps its current body unchanged. Add alongside it:

- `renderSpans(timeline)` — motion bars, per "Lane 1".
- `renderGateWaits(gates)` — wait segments, per "Lane 3".
- `expandSpan(bar, span)` — the overlay, per "Lane 2".
- `loadTimelineOverlay(path)` — fetches `/api/run/timeline` and `/api/run/gates` **after** the
  page renders, `.catch()`-ing into a no-op so a failed graph query never breaks the transport.

Call `loadTimelineOverlay(path)` from `loadReplay()` after `bindTransport()`, not before, and do
not `await` it in a way that delays the rest of the page.

Copy `gateStory()` and `sourceNote()` out of `views.js` before deleting that file — they are the
two functions worth keeping.

**File** `src/motion_spec/dashboard/frontend/style.css`

Add `.span`, `.span-motion`, `.span-expand`, `.gate-wait` near the existing `.marker` rules
(~line 332). Give the existing `.marker*` rules `z-index: 1` so bars sit behind them.

**Do not change** `#content`'s `130px`, the `130px` in the media query, or `.replay`'s
`calc(100dvh - 139px)`. If your design needs them changed, the design is wrong — go back to
"Lane 2" and put the varying part in the overlay.

### Step 3 — Reports page, holding Compare

**File** `src/motion_spec/dashboard/frontend/reports.js` (currently orphaned — nothing imports it)

Move `renderCompare()` and `renderComparePicker()` here from `views.js`, plus the `table()`,
`section()`, `seconds()` and `signed()` helpers they need. `reports.js` has its own `section()`
at line 78 with a different signature — reconcile them into one rather than shipping two.

**File** `src/motion_spec/dashboard/frontend/run.js`

In `replayShell()`, replace the `views` tab with `reports` and `#panel-views` with
`#panel-reports`. In `bindPanels()`, replace the `loadViews()` call with `showReports()`, and
update the panel whitelist — it currently reads:

```js
show(["plots", "views", "explore", "console"].includes(stored) ? stored : "plots", false);
```

**Decision — the three analysis reports.** `reports.js` already renders oscillation, contact and
saturation from `/api/reports` → `analysis.py`, all of it live and tested but unreachable since
the tab was parked. Bring it back **in the same panel, below Compare**: it is the only reachable
home that surface has, and it costs nothing beyond re-showing what is already written and
covered by `tests/test_dashboard_analysis.py`. If the panel feels overloaded once you see it,
STOP and report rather than redesigning it yourself.

### Step 4 — delete the Views page

Delete `src/motion_spec/dashboard/frontend/views.js` (178 lines) **after** Steps 2 and 3 have
taken what they need from it.

Remove from `style.css`: `.views`, `.view`, `.view-head`, `.view-note`, `.tinted`, `.seekable`
and `#panel-views` rules — but **check each one for other users first** (`grep -rn "\.tinted"`
etc.); `.compare-bar` and `.compare-pick` move to Reports and must survive.

**Keep in `queries.py`**: `timeline()`, `gates()`, `compare()`, `views_graph()` and every SPARQL
constant. The page goes; the queries are what the transport and Reports now read. Renaming
`views_graph` is out of scope.

**Keep** `/api/run/timeline`, `/api/run/gates`, `/api/run/compare` in `server.py`.

### Step 5 — tests

**File** `tests/test_dashboard_views.py` (368 lines, 11 tests)

Every test in it exercises `timeline()` / `gates()` / `compare()` at the query or HTTP level, not
the page. **They all still apply** — do not delete them with the page. Two changes:

- Add a test for `activity_constraints()` from Step 1, modelled on
  `test_the_timeline_narrows_to_one_design_iri_with_what_held_throughout` (line 331), asserting
  the returned spans carry `begin_step`/`end_step` and fall inside the occurrence.
- `test_the_three_routes_answer_over_http` (line 360) becomes four routes.

Renaming the file to `test_dashboard_timeline.py` is reasonable — if you do, `git mv` it so the
history follows, and change nothing else in the same commit.

**Verify**

```
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MOTION_SPEC_GENERATIONS=/home/batsy/work/ms/generations \
  /home/batsy/work/ms/.venv/bin/python -m pytest tests/test_dashboard_views.py tests/test_dashboard_api.py -q
/home/batsy/work/ms/.venv/bin/ruff format src/motion_spec/dashboard tests
/home/batsy/work/ms/.venv/bin/ruff check src/motion_spec/dashboard
```

`ruff check` has pre-existing failures elsewhere in the tree (e.g. `BLE001` in `ros_camera.py`);
the files you touch must be clean.

---

## Done criteria

1. `grep -rn "panel-views\|views.js\|loadViews" src/motion_spec/dashboard/` returns nothing.
2. `git show HEAD:src/motion_spec/dashboard/frontend/style.css | grep -c "130px"` and the same
   grep on the working tree return the **same** count — the layout numbers did not move.
3. `grep -n "offsetHeight\|getBoundingClientRect" src/motion_spec/dashboard/frontend/run.js`
   returns only the pre-existing `reserveVideoSpace` hit at line ~126 — no new measurement.
4. The four routes answer: `/api/run/timeline`, `/api/run/gates`, `/api/run/compare`,
   `/api/run/constraints`.
5. Both suites in Step 5 pass.
6. With the dashboard running (`serve(port=8087, logs=Path("/home/batsy/work/ms/generations"))`,
   from the repo root with `source setup-grc.zsh` first), opening an archived run shows motion
   bars on the transport; clicking one expands its constraints above the strip; the transport's
   height is unchanged with the overlay open and closed.

## Out of scope

- Renaming `views_graph()` or the `queries.py` SPARQL constants.
- Touching `analysis.py` itself. It is signal analysis (oscillation via detrend +
  `dominant_frequency`, contact from wrench vs twist, torque saturation) with hard-won decisions
  in its module docstring — in particular that time comes from `step * nominal_period_ns`, never
  the frame's `t`, because two runs of the same model disagreed 2.9× on `t` per step. Read it,
  do not edit it.
- The `?iri=` narrowing path. `explore.js`'s "occurrences" action links to
  `#panel=views&iri=...`; it must be repointed at the new panel name, but redesigning what
  narrowing means on a bar strip is a separate question — if the obvious repoint does not work,
  STOP and report.
- Any change to the `threading.Lock` in `queries.py`.

## Maintenance note

The three hardcoded transport heights (`130px` twice, `139px` once) are the trap this design
routes around rather than fixes. Anything that later adds a permanent row to the transport must
either change all three or use an overlay as Lane 2 does. That is worth its own plan.

The gate lane assumes `.marker-monitor` keeps drawing the fired instant. If marker rendering is
ever changed to drop monitor markers, the wait segments will terminate at nothing.
