# Plan 008: FSM gate calls render in templates + delete the now-dead C++-baking helpers

> **Executor instructions**: Follow step by step; verify each step. Honor STOP
> conditions. Update this plan's row in `plans/README.md` when done.
>
> **Drift check (run first)**:
> `git -C src/motion-spec diff --stat 41ca73d..HEAD -- src/motion_spec/ir_gen.py code-generator/motion.stg`
> Mismatch against "Current state" = STOP.

## Status

- **Priority**: P2
- **Effort**: S (gate calls) + S (sweep)
- **Risk**: LOW-MED
- **Depends on**: plans 003, 004, 005 (their callers must be gone before the sweep
  can delete `cpp_access_expr` / `component_expr`). The FSM-gate-call part depends
  on **007** (`when_args` is replaced by capability booleans there).
- **Category**: tech-debt
- **Planned at**: commit `41ca73d`, 2026-07-11

## Why this matters

Two loose ends close the refactor: (1) `_apply_fsm_gate_calls` bakes C++ call
statements `monitor_when_<gate>(<args>);` in `ir_gen.py` — the last string-of-C++
in the derivations section; (2) once every caller of `cpp_access_expr` /
`component_expr` has been migrated by plans 003–005, those helpers (and any other
now-orphaned expr helper) are dead code that must be deleted so no future edit
re-introduces backend leakage through them. After this plan,
`grep -n 'KDL::\|shared\.\|_state &\|robot_io' ir_gen.py` finds nothing but the
one runtime-comment at ~line 1324.

## Current state

**`ir_gen.py` `_apply_fsm_gate_calls`** (around line 4414):

```python
_set_field(fallback, "fsm_when_gate_calls", [
    f"monitor_when_{gate_id}({_field(by_id[gate_id], 'when_args', '')});"
    for gate_id in gate_ids
    if gate_id in by_id
])
```

It reads `when_args` (a baked C++ arg string). After Plan 007, `when_args` no
longer exists — the args come from capability booleans. So this must emit the
gate *ids* and let the template build the call via Plan 007's `sig-args`.

**Consumer** — grep `fsm_when_gate_calls` in `motion.stg` to find where the calls
are spliced (it renders each string on its own line inside the fallback state's
hold step).

**Helpers to delete after 003–005 land**: `cpp_access_expr` (line 3748),
`component_expr` (line 3780, if Plan 004 didn't already delete it), and check for
any other expr-only helper left with no callers.

## Commands you will need

Same as Plan 003 (env, `pytest -q`, golden diff over all 10 models). FSM gate
calls appear only in models with FSM WHEN-gated motions — `pick_place_dual`,
`collab_pickplace`-style; the sweep affects all models via the deleted helpers
(should be a no-op on output).

## Scope

**In scope:**
- `ir_gen.py` — `_apply_fsm_gate_calls`; deletion of `cpp_access_expr`,
  `component_expr`, and any other orphaned expr helper.
- `code-generator/motion.stg` — the `fsm_when_gate_calls` render site.

**Out of scope:**
- FSM *wiring* (`_apply_fsm_wiring`, `_fsm_from_graph`, `is_fsm_event`) — those are
  semantic/graph derivations, not C++ emission; they stay in ir_gen.
- Any helper that still has a caller (the pre-deletion grep gate below protects
  this).

## Steps

### Step 1: Capture golden baseline
Run part (1) of Plan 003's "Golden C++ diff".

### Step 2: ir_gen — gate calls emit ids, not baked call strings

Replace `fsm_when_gate_calls` (list of C++ statements) with the structured gate
list already available — `fsm_when_gate_motions` (the gate ids) is set in
`_apply_fsm_wiring`. You likely do not need `_apply_fsm_gate_calls` at all
anymore: the template can iterate `fallback.fsm_when_gate_motions` and render
`monitor_when_<gate>(<sig-args(...)>);` per id. If `_apply_fsm_gate_calls` only
existed to bake the string, delete it and remove its call in
`derive_codegen_fields`. If it also filters `if gate_id in by_id`, preserve that
filter by keeping only valid gate ids in `fsm_when_gate_motions`.

### Step 3: motion.stg — render the gate call

At the `fsm_when_gate_calls` site, iterate the gate motions and emit the call
using Plan 007's `sig-args` for the gated motion's `when` signature:

```
<fallback.fsm_when_gate_motions:{gid | monitor_when_<gid>(<sig-args(motions.(gid), motions.(gid).when_needs_state, motions.(gid).when_needs_shared, motions.(gid).when_needs_robot)>);}; separator="\n">
```

Confirm the motions are indexable by id in the template scope (grep how other
by-id lookups are done in `motion.stg`; if there is no id-indexed map, emit the
already-rendered `when_args` equivalent — but prefer the id path to stay
backend-free).

### Step 4: Golden diff (gate calls)
Run parts (2)+(3) of the golden diff. FSM-gated models must be byte-identical.

**Verify**: no `DIFF …`; `pytest -q` all pass.

### Step 5: Dead-code sweep — delete orphaned helpers (GATED)

Only after plans 003, 004, 005 are DONE (check `plans/README.md`). Run the gate:

```bash
grep -rn 'cpp_access_expr\|component_expr' src/motion-spec/src/motion_spec/ | grep -v 'def cpp_access_expr\|def component_expr'
```

- If this prints **nothing**, delete `def cpp_access_expr` and `def component_expr`
  (and their now-unused imports if any). Then re-run the golden diff — it must
  still be clean (deleting dead code changes no output).
- If it prints **any** caller, STOP: a plan among 003–005 is incomplete. Do not
  delete; report which caller remains.

Also sweep for other orphans introduced by the migration:
```bash
grep -n 'def _evaluator_term\|def _set_monitor_conditions\|def _motion_done_condition' src/motion-spec/src/motion_spec/ir_gen.py
```
Only delete one if Plan 006 replaced it AND it has zero callers.

**Verify**: golden diff clean; `pytest -q` pass.

## Test plan

- Golden diff over all 10 models — gate calls (FSM models) byte-identical; sweep
  is a pure no-op on generated C++.
- `pytest -q`.

## Done criteria

- [ ] `grep -n 'monitor_when_.*when_args\|fsm_when_gate_calls' src/motion-spec/src/motion_spec/ir_gen.py` returns nothing.
- [ ] `grep -rn 'cpp_access_expr\|component_expr' src/motion-spec/src/motion_spec/` returns nothing (helpers deleted; gate passed).
- [ ] `grep -n 'KDL::\|shared\.' src/motion-spec/src/motion_spec/ir_gen.py` returns only the runtime comment at ~line 1324 — no code emitting C++.
- [ ] Golden C++ diff: no `DIFF …` / `GEN FAIL …`.
- [ ] `cd src/motion-spec && pytest -q` exits 0.
- [ ] `plans/README.md` row updated (and, if all of 003–008 are DONE, note the
      refactor complete in the index).

## STOP conditions

- Drift vs "Current state".
- The dead-code gate (Step 5) finds a remaining caller — a prerequisite plan is
  incomplete; report and stop, do not force the deletion.
- The template cannot look up a gated motion by id — report; do not fall back to
  reading a removed `when_args` field.
- Golden diff shows any change from the sweep (it should be impossible; a change
  means a "dead" helper wasn't actually dead).

## Maintenance notes

- After this, `ir_gen.py` is C++/KDL-free (verified by the `KDL::`/`shared.`
  greps). Any future change that reintroduces a `KDL::`/`shared.`/`<type> &`
  string in ir_gen is a regression of this whole effort — worth a CI grep guard
  (see the index's follow-up note).
- Reviewer: confirm the three completion greps in Done criteria actually return
  empty, and that FSM semantic wiring was left intact.
