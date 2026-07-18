# Plan 007: Motion function signatures render from capability flags, not C++ param strings

> **Executor instructions**: Follow step by step; verify each step. Honor STOP
> conditions. Update this plan's row in `plans/README.md` when done.
>
> **Drift check (run first)**:
> `git -C src/motion-spec diff --stat 41ca73d..HEAD -- src/motion_spec/ir_gen.py code-generator/motion.stg`
> Mismatch against "Current state" = STOP.

## Status

- **Priority**: P2
- **Effort**: L
- **Risk**: HIGH
- **Depends on**: none (hardest layer — schedule last of the mechanical ones)
- **Category**: tech-debt
- **Planned at**: commit `41ca73d`, 2026-07-11

## Why this matters

`add_motion_function_interfaces` in `ir_gen.py` assembles literal C++ function
signatures and argument lists — `"{mid}_state &state"`, `"shared_data &shared"`,
`"const robot_io &robot"` — and joins them into `*_params` / `*_args` strings the
templates splice into `inline bool can_start_…(…)`, `apply_…(…)`, etc. These are
the most blatant "C++ in the DSL layer" strings in the whole file: the type names
`shared_data`, `robot_io`, `<mid>_state` are pure backend. Replace the baked
strings with **capability booleans** on each motion; the template builds the
signature and call from those booleans, owning the C++ type names.

## Current state

**`ir_gen.py` `add_motion_function_interfaces`** (around line 4183). For each
motion it computes booleans then bakes strings:

```python
state_type = f"{mid}_state &state"
has_when_elapsed = any(_field(e, "is_elapsed") for e in _field(motion, "when_evaluators", []))
has_when_logic   = bool(_field(motion, "when_schedule") or _field(motion, "when_evaluators"))
# can_start:
if has_when_elapsed: can_start_params.append(state_type); can_start_args.append(f"{mid}_state_instance")
if has_when_logic:   can_start_params.append("shared_data &shared"); can_start_args.append("shared")
_set_field(motion, "can_start_params", join_params(can_start_params))
_set_field(motion, "can_start_args", ", ".join(can_start_args))
# monitor_sig(use_state, use_shared, use_robot) similarly builds when_/until_/monitor_ params+args
# apply: state (if arm_solvers) + shared (if forwarded_commands) + robot (if arm_solvers or forwarded)
```

`join_params(params)` formats as `"\n    " + ",\n    ".join(params) + "\n"` (the
multiline signature style) or `""` when empty.

**Consumers** — `motion.stg`:
```
:149  if (!<motion.id>_state_instance.active && can_start_<motion.id>(<motion.can_start_args>)) {
:163              apply_<motion.id>(<motion.apply_args>);
:433  inline bool can_start_<motion.id>(<motion.can_start_params>) {
:442  inline void monitor_when_<motion.id>(<motion.when_params>) {
:454  inline void monitor_until_<motion.id>(<motion.until_params>) {
:461  inline void monitor_<motion.id>(<motion.monitor_params>) {
:489  inline void apply_<motion.id>(<motion.apply_params>) {
```

**FSM note**: `derive_codegen_fields` runs `_apply_fsm_wiring` BEFORE
`add_motion_function_interfaces` so the FSM-added `robot` param is included
(`when_fsm`/`until_fsm` come from monitors tagged with `fsm_namespace`). The
docstring of `derive_codegen_fields` says the FSM-dependent bits of the
signatures are *recomputed at codegen time* — confirm whether `codegen_artifacts`
or a template already recomputes any `*_params`. Grep both before starting.

## Commands you will need

Same as Plan 003 (env, `pytest -q`, golden diff over all 10 models). Signatures
appear in EVERY motion in EVERY model — this is the widest-blast-radius layer.

## Scope

**In scope:**
- `ir_gen.py` — `add_motion_function_interfaces` (replace string outputs with
  capability booleans; keep the boolean computation).
- `code-generator/motion.stg` — the 5 signature sites + 2 call sites above, plus
  new signature/arg sub-templates.

**Out of scope:**
- The boolean *predicates* (`has_when_elapsed`, `when_fsm`, …) — keep their exact
  logic; only stop turning them into strings.
- FSM wiring (`_apply_fsm_wiring`, `_apply_fsm_gate_calls`) — Plan 008. But note
  the ordering dependency (below).

## Steps

### Step 1: Capture golden baseline
Run part (1) of Plan 003's "Golden C++ diff".

### Step 2: ir_gen — emit capability booleans per motion

Replace the `*_params` / `*_args` string fields with the booleans that drive
them. For each motion emit (names illustrative — match to the four signatures):

```python
# can_start
_set_field(motion, "can_start_needs_state", has_when_elapsed)
_set_field(motion, "can_start_needs_shared", has_when_logic)
# when / until / monitor  (each: needs_state, needs_shared, needs_robot)
_set_field(motion, "when_needs_state", has_when_elapsed or bool(when_mons))
_set_field(motion, "when_needs_shared", has_when_elapsed or has_pose or when_sched or bool(when_mons))
_set_field(motion, "when_needs_robot", when_fsm)
# ... until_needs_*, monitor_needs_*  (from monitor_sig args)
# apply
_set_field(motion, "apply_needs_state", bool(_field(motion, "arm_solvers")))
_set_field(motion, "apply_needs_shared", has_forwarded_commands)
_set_field(motion, "apply_needs_robot", bool(_field(motion, "arm_solvers")) or has_forwarded_commands)
```

Delete `join_params`, the `*_params`/`*_args` `_set_field` calls, and `monitor_sig`
(its logic becomes the boolean assignments). Keep every predicate exactly as-is.

### Step 3: motion.stg — signature + arg sub-templates own the C++ types

Add two sub-templates that build the multiline param list and the arg list from
the three needs-booleans, reproducing `join_params` formatting **exactly**
(leading `\n    `, `,\n    ` separators, trailing `\n`; empty → nothing):

```
sig-params(motion, needs_state, needs_shared, needs_robot) ::= <%
<if(needs_state)>
    <motion.id>_state &state<endif><if(needs_shared)><if(needs_state)>,<endif>
    shared_data &shared<endif><if(needs_robot)><if(anyprev)>,<endif>
    const robot_io &robot<endif>...
%>
```

(StringTemplate whitespace is fiddly — build it so the rendered text is
byte-for-byte what `join_params` produced. The reliable approach: assemble a list
of the present params via nested `<if>` and join with the ST list separator,
matching `",\n    "` and the `"\n    "` prefix / `"\n"` suffix. Test against the
baseline early and often.) Provide a matching `sig-args` that joins the present
arg tokens (`<motion.id>_state_instance`, `shared`, `robot`) with `", "`.

Then replace each consumer:
- `:433` `inline bool can_start_<motion.id>(<sig-params(motion, motion.can_start_needs_state, motion.can_start_needs_shared, false)>) {`
- `:149` `… can_start_<motion.id>(<sig-args(motion, motion.can_start_needs_state, motion.can_start_needs_shared, false)>) …`
- and likewise `when` / `until` / `monitor` / `apply` for their sites (442, 454,
  461, 489, 163).

### Step 4: Golden diff — iterate on whitespace until clean

Run parts (2)+(3) of the golden diff. Signatures are multiline; expect
whitespace mismatches on the first try. Fix the sub-template whitespace, not the
IR. Repeat until zero `DIFF`.

**Verify**: no `DIFF …`; `pytest -q` all pass.

## Test plan

- Golden diff over all 10 models is the whole test — every motion's five
  functions must be byte-identical, including the multiline `(\n    a,\n    b\n)`
  layout and the empty `()` case.
- `pytest -q`; update `tests/` assertions on `*_params`/`*_args` to the boolean
  fields.

## Done criteria

- [ ] `grep -n 'shared_data &shared\|robot_io &robot\|_state &state\|_state_instance\|join_params\|monitor_sig\|_params\|_args' src/motion-spec/src/motion_spec/ir_gen.py` returns nothing in the derivations section (>3599). (Confirm any remaining hits are above line 3599 and unrelated.)
- [ ] Golden C++ diff: no `DIFF …` / `GEN FAIL …`.
- [ ] `cd src/motion-spec && pytest -q` exits 0.
- [ ] No out-of-scope files modified.
- [ ] `plans/README.md` row updated.

## STOP conditions

- Drift vs "Current state".
- Golden diff shows a signature diff you cannot eliminate after two honest
  whitespace-fix attempts — report the exact model, function, and the byte diff
  (`diff -u` of the two `motion.stg`-rendered functions). Whitespace in ST4 is a
  known trap; if it proves intractable, report rather than approximating.
- **Ordering trap**: if Plan 008 has already run, its `_apply_fsm_gate_calls`
  depends on `when_args` existing. Since this plan removes `when_args`, coordinate:
  either run this plan before 008, or ensure 008's gate calls were also migrated
  to booleans. If `when_args` is referenced anywhere after your change
  (`grep -rn 'when_args'`), STOP and reconcile with 008.

## Maintenance notes

- The C++ type names `shared_data`, `robot_io`, `<mid>_state` now appear ONLY in
  `motion.stg`. A new capability (e.g. a motion needing a new context object) adds
  a needs-boolean in ir_gen + an `<if>` branch in `sig-params`/`sig-args`.
- Reviewer: this is the whitespace-sensitive plan — insist the golden diff output
  is shown in full (all 10 models), not summarized.
- This layer and Plan 008 share the `when_args` seam; note in the PR which landed
  first.
