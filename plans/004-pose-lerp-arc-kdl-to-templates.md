# Plan 004: Move pose-component / Lerp / Arc `KDL::Frame` construction into templates

> **Executor instructions**: Follow step by step; run every verification and
> confirm the expected result before continuing. Honor STOP conditions. Update
> this plan's row in `plans/README.md` when done.
>
> **Drift check (run first)**:
> `git -C src/motion-spec diff --stat 41ca73d..HEAD -- src/motion_spec/ir_gen.py code-generator/motion.stg code-generator/closures.stg`
> On any change to these files, compare the "Current state" excerpts to live
> code; mismatch = STOP.

## Status

- **Priority**: P1
- **Effort**: M
- **Risk**: MED
- **Depends on**: none (independent of 003)
- **Category**: tech-debt
- **Planned at**: commit `41ca73d`, 2026-07-11

## Why this matters

`ir_gen.py` (the RDF/DSL layer, which AGENTS.md forbids from knowing C++/KDL)
constructs literal `KDL::Frame(KDL::Rotation::RPY(...), KDL::Vector(...))` strings
for declared poses and Lerp goals, and `shared.X.p` / `shared.X.M` strings for
Arc endpoints. The templates already wrap these (`closures.stg:44` does
`= <closure.goal_expr>`), so the KDL is *interpolated* from Python. Move the KDL
construction into the templates; ir_gen emits the abstract component ids and a
per-axis expr map that the template renders. The per-axis component values are
themselves either literals or `access-expr` targets, so the template already has
everything it needs.

## Current state

**`ir_gen.py`** three functions bake KDL:

`build_pose_components` (around line 3982) fills a per-pose dict of six
`*_expr` strings via `component_expr` (which calls `cpp_access_expr` → KDL):

```python
entry[f"{prefix}_{axis}_expr"] = component_expr(subobject, data_by_id, views)
```

`resolve_lerp_closures` (around line 4028):

```python
closure["goal_expr"] = (
    "KDL::Frame(KDL::Rotation::RPY("
    f"{parts['orientation_x_expr']}, {parts['orientation_y_expr']}, {parts['orientation_z_expr']}), "
    "KDL::Vector("
    f"{parts['position_x_expr']}, {parts['position_y_expr']}, {parts['position_z_expr']}))"
)
closure["assign_goal"] = True
# else branch:
closure["goal_expr"] = f"shared.{goal}"
closure["assign_goal"] = False
```

`resolve_arc_closures` (around line 4053):

```python
closure["end_position_expr"] = f"shared.{end}.p"
closure["end_orientation_expr"] = f"shared.{end}.M"
```

`component_expr` (line 3780) returns either a literal `str(value)`, or
`cpp_access_expr(reference_value, views)` (KDL), or `cpp_access_expr(component_id, views)`.

**Consumer templates** already emit the KDL scaffold and interpolate the baked
inner strings:

`motion.stg:94` `materialize-pose-component`:
```
materialize-pose-component(pose) ::= <<
    shared.<pose.id> = KDL::Frame(
        KDL::Rotation::RPY(<pose.orientation_x_expr>, <pose.orientation_y_expr>, <pose.orientation_z_expr>),
        KDL::Vector(<pose.position_x_expr>, <pose.position_y_expr>, <pose.position_z_expr>));
>>
```

`closures.stg:42` `emit-call-Lerp`:
```
    const KDL::Frame _lerp_goal_<closure.trajectory> = <closure.goal_expr>;
<if(closure.assign_goal)>    shared.<closure.goal> = _lerp_goal_<closure.trajectory>;
```

`closures.stg:91` `emit-call-Arc` uses `<closure.end_position_expr>` /
`<closure.end_orientation_expr>` (lines 99, 158).

**Existing template renderer to reuse** — `access-expr(id, views)`
(`shared_data.stg:128`) renders both literals-as-shared and view/axis access.
There is also `component-expr`-style precedent: `closures.stg:240` renders pose
axis targets directly with `<access-expr(g.linear_x, views)>`.

### The mechanics

The six `*_expr` values on a pose are each one of: a literal number, or an
`access-expr(id)`. `component_expr` decides which. That decision (literal vs
reference) must move to the template. Emit, per component, a structured pair:
`{value: <literal-or-null>, ref: <id-or-null>}`. The template renders
`<if(comp.ref)><access-expr(comp.ref, views)><else><comp.value><endif>`. This is
already the exact idiom used at `closures.stg:246` (`<if(g.angular_x)>…<else>0.0<endif>`).

## Commands you will need

Identical to Plan 003 ("Commands you will need" + "Golden C++ diff"). The golden
diff over all 10 models is the primary gate. `cd src/motion-spec && pytest -q`.

## Scope

**In scope:**
- `src/motion-spec/src/motion_spec/ir_gen.py` — `build_pose_components`,
  `component_expr`, `declared_pose_component_entries`, `resolve_lerp_closures`,
  `resolve_arc_closures`.
- `src/motion-spec/code-generator/motion.stg` — `materialize-pose-component`.
- `src/motion-spec/code-generator/closures.stg` — `emit-call-Lerp`, `emit-call-Arc`.

**Out of scope:**
- `cpp_access_expr` (delete in Plan 008 after all callers gone). `component_expr`
  becomes dead after this plan — you MAY delete it here since this plan removes
  its only callers, but verify with `grep -rn 'component_expr' src/motion_spec`
  first; if any other caller exists, leave it and note it.
- Any `access-expr` sub-template (it already works; do not modify).

## Steps

### Step 1: Capture golden baseline
Run part (1) of Plan 003's "Golden C++ diff".

### Step 2: ir_gen — emit structured pose components (value/ref), not KDL

Rework `build_pose_components` so each pose entry holds, per axis, a structured
component instead of a baked expr string. Target shape per pose:

```python
{
  "id": pose_id,
  "position": [ {"value": <num or None>, "ref": <id or None>}, ...x,y,z ],
  "orientation": [ ...x,y,z ],
}
```

Replace the `component_expr(...)` call with the raw decision it made: read the
subobject data; if it has a `value`, set `{"value": value, "ref": None}`; if it
has a `reference_value`, set `{"value": None, "ref": reference_value}`. Do NOT
call `cpp_access_expr`. Keep the missing-component validation
(`raise ValueError(... missing required components ...)`) — adapt it to the new
shape.

`declared_pose_component_entries` must pass this structured shape through
unchanged (it currently spreads `**parts`).

### Step 3: motion.stg — render KDL::Frame from structured components

Rewrite `materialize-pose-component` to build the frame from the structured
lists, rendering each component via the value/ref idiom:

```
component-value(comp, views) ::= <<
<if(comp.ref)><access-expr(comp.ref, views)><else><comp.value><endif>
>>

materialize-pose-component(pose, views) ::= <<
    shared.<pose.id> = KDL::Frame(
        KDL::Rotation::RPY(<component-value(pose.orientation.0, views)>, <component-value(pose.orientation.1, views)>, <component-value(pose.orientation.2, views)>),
        KDL::Vector(<component-value(pose.position.0, views)>, <component-value(pose.position.1, views)>, <component-value(pose.position.2, views)>));
>>
```

Thread `views` into the `materialize-pose-component` call site (it renders inside
motion codegen where `views` is already available — confirm by grepping the call
site in `motion.stg` / `main.stg`).

### Step 4: ir_gen — Lerp goal as ids + flag, not KDL::Frame string

In `resolve_lerp_closures`, replace the `goal_expr` KDL string with a flag +
reference to the declared-pose components. When the goal is a declared pose, set
`closure["goal_is_pose"] = True` and leave the pose lookup to the template via
the existing `pose_components`/`declared_pose_components` the template can index;
when it is a shared signal, set `closure["goal_is_pose"] = False` and
`closure["goal"] = goal` (already present). Keep `assign_goal` semantics.

Then in `emit-call-Lerp` (`closures.stg:42`), build the `KDL::Frame` inline using
the same `materialize-pose-component` frame expression when `goal_is_pose`, else
`shared.<closure.goal>`. Reuse the `component-value` sub-template from Step 3 so
the KDL construction lives in exactly one place.

### Step 5: ir_gen — Arc endpoints as ids, not `shared.X.p` strings

In `resolve_arc_closures`, replace:
```python
closure["end_position_expr"] = f"shared.{end}.p"
closure["end_orientation_expr"] = f"shared.{end}.M"
```
with just the abstract id `closure["end"] = end` (already present) and let
`emit-call-Arc` render `shared.<closure.end>.p` / `shared.<closure.end>.M`
directly in the template (`closures.stg:99,158`). `end` is validated to be a
Pose, so `.p`/`.M` are always valid — the template is the right place for that
KDL member access.

### Step 6: Golden diff + tests
Run parts (2)+(3) of the golden diff and `pytest -q`.

**Verify**: no `DIFF …` lines; `pytest -q` all pass.

## Test plan

- Golden diff over all 10 models is the regression gate — Lerp appears in
  `pick_place_*`, Arc in `arc_demo` / `admittance_arc_single`, declared poses in
  `pick_place_*`. All must stay byte-identical.
- `pytest -q`; update any `tests/` assertion that checks `goal_expr` /
  `*_position_expr` string shapes to the new structured shape (in-scope test edit).

## Done criteria

- [ ] `grep -n 'KDL::' src/motion-spec/src/motion_spec/ir_gen.py` returns only the
      line ~1324 comment (`# KDL::Frame is filled at runtime…`) — no code.
- [ ] `grep -n 'goal_expr\|end_position_expr\|end_orientation_expr\|_x_expr\|_y_expr\|_z_expr' src/motion-spec/src/motion_spec/ir_gen.py` returns nothing.
- [ ] Golden C++ diff: no `DIFF …` / `GEN FAIL …`.
- [ ] `cd src/motion-spec && pytest -q` exits 0.
- [ ] No out-of-scope files modified.
- [ ] `plans/README.md` row updated.

## STOP conditions

- "Current state" excerpts don't match live code (drift).
- Golden diff shows a C++ change you can't eliminate — most likely the RPY/Vector
  argument order or a literal's float formatting differs (Python `str(value)` vs
  template rendering of the same number). If a pure numeric-formatting diff
  appears (e.g. `1.0` vs `1`), STOP and report — do not "fix" by reformatting in
  Python, that re-introduces backend concerns; the fix belongs in how the number
  is carried in the IR.
- `component_expr` turns out to have a caller outside the three functions listed.

## Maintenance notes

- The KDL frame-construction idiom now lives once, in `component-value` +
  `materialize-pose-component`. Reuse it if a new trajectory type needs a pose
  goal; do not re-bake in Python.
- Reviewer: check Lerp `assign_goal` still gates the `shared.<goal> = …`
  assignment, and Arc still validates `end` is a Pose in ir_gen (validation stays;
  only the C++ member access moved).
