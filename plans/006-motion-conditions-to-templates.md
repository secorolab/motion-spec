# Plan 006: Motion boolean conditions (active / when / done) render in templates

> **Executor instructions**: Follow step by step; verify each step. Honor STOP
> conditions. Update this plan's row in `plans/README.md` when done.
>
> **Drift check (run first)**:
> `git -C src/motion-spec diff --stat 41ca73d..HEAD -- src/motion_spec/ir_gen.py src/motion_spec/codegen_artifacts.py code-generator/motion.stg`
> Mismatch against "Current state" = STOP.

## Status

- **Priority**: P2
- **Effort**: M
- **Risk**: MED-HIGH
- **Depends on**: none (independent, but harder — schedule after 003–005)
- **Category**: tech-debt
- **Planned at**: commit `41ca73d`, 2026-07-11

## Why this matters

`ir_gen.py` builds C++ boolean expressions for motion sequencing —
`active_condition`, `when_condition`, `done_condition` — out of `&&`/`||`,
`state.<field>`, `<motion>_state_instance.<flag>`,
`motion_spec::runtime::constraint_satisfied(shared.<err>)`, and
`(shared.clock_time_s - state.<start> >= <thr>)`. These are pure C++/runtime
constructs the RDF/DSL layer should not know. Move the boolean assembly into the
templates; ir_gen emits the structured term list (per-evaluator: error-id or
elapsed spec) plus the join semantics (`any` vs `all`), and the template folds
them into the C++ condition.

## Current state

**`ir_gen.py` `_evaluator_term`** (around line 4126) — the atom:

```python
def _evaluator_term(e, start_field: str) -> str:
    if _field(e, "is_elapsed"):
        op = _field(e, "elapsed_op") or ">="
        thr = _field(e, "elapsed_threshold_s") or 0.0
        return f"(shared.clock_time_s - state.{start_field} {op} {thr:.6f})"
    return f"motion_spec::runtime::constraint_satisfied(shared.{_field(_field(e, 'error'), 'id')})"
```

**`_set_monitor_conditions` / `_set_motion_conditions`** (lines 4136, 4165) join
terms with `" || "` / `" && "` (chosen by the motion's `*_any` flag), wrap in
parens when >1 term, and stamp `active_condition` / `when_condition` onto
monitors + motion. `_motion_done_condition` (line 3709) builds `done_condition`
from `until_monitors` as `<mid>_state_instance.<flag>` /
`…_event_triggered` terms.

**Consumers** — `motion.stg`:
- `:19` `<if(monitor.active_condition)> … const bool active = <monitor.active_condition>;`
- `:91` `<motion.done_condition>`
- `:439` `return <motion.when_condition>;`

**Also `codegen_artifacts.py`** `build_introspection_model` (line 508) reuses
`active_condition`:
```python
value_expr = slot.get("active_condition") or _shared_expr(slot.get("error_signal"), shared_ids)
"satisfied_expr": value_expr if slot.get("active_condition") else f"constraint_satisfied({value_expr})",
```
so `active_condition` is consumed in two places — keep both working.

## Commands you will need

Same as Plan 003 (env, `pytest -q`, golden diff over all 10 models). Conditions
appear in every model with WHEN/UNTIL logic — `pick_place_*`, `admittance_arc_single`.

## Scope

**In scope:**
- `ir_gen.py` — `_evaluator_term`, `_set_monitor_conditions`,
  `_set_motion_conditions`, `_motion_done_condition`, `_apply_monitor_debounce`
  (only if it reads the removed fields — check).
- `code-generator/motion.stg` — `update-monitor`, `motion-done-condition`,
  `can_start_<motion.id>` body (line ~439).
- `codegen_artifacts.py` — the `active_condition` reuse at line ~508.

**Out of scope:**
- Monitor edge/debounce machinery in `update-monitor` (lines 22-45) — only the
  `active_condition` atom changes; the `rising_edge`/`sustained_edge` logic stays.
- FSM wiring fields (`fsm_event_idx`, etc.) — Plan 008.

## Steps

### Step 1: Capture golden baseline
Run part (1) of Plan 003's "Golden C++ diff".

### Step 2: ir_gen — emit structured terms, not C++ strings

For each motion, instead of `active_condition` / `when_condition` strings, emit a
structured list per phase. Each term is one of:

```python
{"kind": "constraint", "error_id": <id>}
{"kind": "elapsed", "op": <op>, "threshold_s": <float>, "start_field": <field>}
```

Emit on the motion: `when_terms`, `when_any` (already exists);
`until_terms` + on the aggregate monitor `active_terms` + `active_any`. For
`done_condition`, emit `done_terms` where each is
`{"kind": "flag", "motion_id": mid, "flag": <flag>}` or
`{"kind": "event", "motion_id": mid, "id": <monitor_id>}` (edge-triggered), plus
`done_any`.

Keep the SAME grouping the current code produces (aggregate monitor gets the full
active list; per-error elapsed monitors get their single elapsed term) — the
structure carries what `_set_monitor_conditions` computed, minus the string join.

### Step 3: motion.stg — a `bool-condition` sub-template that folds terms

Add sub-templates that render a term list into the exact C++ the Python produced:

```
cond-term(t) ::= <<
<if(t.kind_constraint)>motion_spec::runtime::constraint_satisfied(shared.<t.error_id>)<endif><if(t.kind_elapsed)>(shared.clock_time_s - state.<t.start_field> <t.op> <t.threshold_s>)<endif><if(t.kind_flag)><t.motion_id>_state_instance.<t.flag><endif><if(t.kind_event)><t.motion_id>_state_instance.<t.id>_event_triggered<endif>
>>

bool-condition(terms, any, empty_default) ::= <<
<if(!terms)><empty_default><elseif(rest(terms))>(<terms:{t | <cond-term(t)>}; separator={ <if(any)>||<else>&&<endif> }>)<else><cond-term(first(terms))><endif>
>>
```

Then:
- `update-monitor` line 21: `const bool active = <bool-condition(monitor.active_terms, monitor.active_any, "false")>;`
  guarded by `<if(monitor.active_terms)>`.
- `motion-done-condition` (line 90): `<bool-condition(motion.done_terms, motion.done_any, "true")>`.
- `can_start` (line 439): `return <bool-condition(motion.when_terms, motion.when_any, "true")>;`.

**Critical float formatting**: the Python elapsed term uses `{thr:.6f}` (e.g.
`2.000000`). The template must reproduce the SAME 6-decimal formatting or the
golden diff fails. Carry the threshold **pre-formatted as a string** in the IR
term (`"threshold_s": f"{thr:.6f}"`) — this is a numeric-formatting detail, not
backend knowledge, so formatting it in Python is acceptable. Do the same for any
literal that has fixed formatting today.

### Step 4: codegen_artifacts — rebuild `active_condition` for its introspection use

`build_introspection_model` (line 508) still needs a monitor `value_expr`. Since
`active_condition` is gone, render the same string here from `active_terms` using
a small Python helper that mirrors `bool-condition` — OR, preferred, have the
monitor introspection sample call the template `bool-condition` too. Choose the
path that keeps `build_introspection_model` free of new C++ if possible; if a
tiny Python join is unavoidable here, isolate it in one helper and mark it
`# ponytail: monitor value_expr mirrors bool-condition; keep in sync`.

### Step 5: Golden diff + tests
Run parts (2)+(3) of the golden diff and `pytest -q`.

**Verify**: no `DIFF …`; `pytest -q` all pass.

## Test plan

- Golden diff over all 10 models — WHEN/UNTIL/done conditions in every sequenced
  model. Byte-identical required, including paren nesting and `&&`/`||` choice.
- `pytest -q`; update `tests/` assertions on `when_condition`/`active_condition`
  strings to the term-list shape.

## Done criteria

- [ ] `grep -n 'constraint_satisfied\|clock_time_s\|_event_triggered\|state_instance\.\| && \| || ' src/motion-spec/src/motion_spec/ir_gen.py` returns nothing in the derivations section (lines >3599). (Matches in the RDF-parsing section above 3599 are unrelated — verify the hits are all <3599.)
- [ ] Golden C++ diff: no `DIFF …` / `GEN FAIL …`.
- [ ] `cd src/motion-spec && pytest -q` exits 0.
- [ ] No out-of-scope files modified.
- [ ] `plans/README.md` row updated.

## STOP conditions

- Drift vs "Current state".
- Golden diff shows a condition change you can't eliminate. The two highest-risk
  causes: (a) paren wrapping — Python wraps only when `len(terms) > 1`; the
  template `<if(rest(terms))>` must match exactly (single term → no parens).
  (b) float formatting of thresholds — must be identical 6-decimal strings.
  Report the model + diff.
- `_apply_monitor_debounce` or another function reads `active_condition` /
  `when_condition` — if so it is another consumer; STOP and report rather than
  guessing.

## Maintenance notes

- The C++ boolean grammar now lives once in `cond-term`/`bool-condition`. A new
  evaluator kind (e.g. a new timing predicate) adds a `<if(t.kind_X)>` branch
  there, and a term-emitter in ir_gen — never a new f-string of C++.
- Reviewer: scrutinize the empty-list defaults (`when`→`true`, `until/active`→
  `false`/`true` per current code) and the `any`/`all` join — these are the
  behavioral heart and a wrong default silently changes sequencing.
