# Plan 003: Controller signal metadata emits abstract signal ids, not baked C++ exprs

> **Executor instructions**: Follow this plan step by step. Run every
> verification command and confirm the expected result before moving to the
> next step. If anything in "STOP conditions" occurs, stop and report — do not
> improvise. When done, update the status row for this plan in
> `plans/README.md`.
>
> **Drift check (run first)**:
> `git -C src/motion-spec diff --stat 41ca73d..HEAD -- src/motion_spec/ir_gen.py src/motion_spec/codegen_artifacts.py code-generator/introspection.stg`
> If any in-scope file changed since this plan was written, compare the "Current
> state" excerpts against the live code before proceeding; on a mismatch, treat
> it as a STOP condition.

## Status

- **Priority**: P1
- **Effort**: S
- **Risk**: LOW
- **Depends on**: none
- **Category**: tech-debt
- **Planned at**: commit `41ca73d`, 2026-07-11

## Why this matters

`src/motion-spec` is the RDF/DSL layer. AGENTS.md forbids it from knowing the
downstream C++ library: *"Keep the RDF/DSL layer implementation-agnostic …
never the downstream C++ library (no `KDL::` … )."* Yet `ir_gen.py` bakes C++
access strings (`shared.X.p[0]`, `KDL::diff(KDL::Rotation::Identity(), …)`) into
the IR via `cpp_access_expr`. The StringTemplate backend **already** has an
`access-expr(id, views)` sub-template (`code-generator/shared_data.stg:128`)
that renders exactly the same C++ from the abstract signal id plus the `views`
map. So the Python duplicates backend knowledge that lives — correctly — in the
templates. This plan removes the first caller: controller signal metadata.
After it lands, controller introspection samples render from abstract ids, and
the C++ is produced only in the template.

## Current state

**`src/motion-spec/src/motion_spec/ir_gen.py`**, `add_controller_signal_metadata`
(around line 3790) sets both abstract ids AND baked C++ exprs onto each
controller:

```python
def annotate(controller) -> None:
    error_id = signal_id(_field(controller, "error_signal"))
    source = error_sources.get(error_id) or {}
    measured_id = source.get("quantity")
    setpoint_id = signal_id(_field(controller, "reference_signal")) or source.get("reference_value")
    measured_derivative_id = signal_id(_field(controller, "measured_derivative"))
    if measured_id:
        _set_field(controller, "measured_signal", measured_id)
        _set_field(controller, "measured_expr", cpp_access_expr(measured_id, views))   # <-- C++ baked here
    if setpoint_id:
        _set_field(controller, "setpoint_signal", setpoint_id)
        _set_field(controller, "setpoint_expr", cpp_access_expr(setpoint_id, views))   # <-- C++ baked here
    if measured_derivative_id:
        _set_field(controller, "measured_derivative_expr",
                   cpp_access_expr(measured_derivative_id, views))                     # <-- C++ baked here
```

Note `measured_signal` / `setpoint_signal` (abstract ids) are **already**
emitted alongside the exprs. There is no `measured_derivative_signal` id emitted
yet — only its expr.

**`src/motion-spec/src/motion_spec/codegen_artifacts.py`** (build-side, runs at
codegen time, sees `ir.json`) consumes these. `_controller_slot` (around line
138):

```python
"measured_expr": controller.get("measured_expr"),
"setpoint_expr": controller.get("setpoint_expr"),
"measured_derivative_expr": controller.get("measured_derivative_expr"),
```

and `build_introspection_model` (around line 498) already falls back to a
shared-access helper when the baked expr is absent:

```python
"measured_expr": slot.get("measured_expr")
    or _shared_expr(slot.get("measured_signal"), shared_ids),
"setpoint_expr": slot.get("setpoint_expr")
    or _shared_expr(slot.get("setpoint_signal"), shared_ids),
```

where `_shared_expr(id, shared_ids)` (line 468) returns `shared.{id}` — it does
**not** handle the view/axis case (`shared.X.p[0]`). That is why the baked expr
was needed: for signals that are *views* (a single axis of a Pose/Twist/Wrench),
`_shared_expr` is wrong and `measured_expr` carries the correct KDL access.

**Consumer template** — `code-generator/introspection.stg:481`:

```
introspection-controller-sample(slot) ::= <<
        pub.constraint_sample(<slot.uri>, <slot.error_expr>, <slot.output_expr>, <slot.measured_expr>, <slot.setpoint_expr>, constraint_satisfied(<slot.error_expr>));
>>
```

**The template that already does this correctly** —
`code-generator/shared_data.stg:128`:

```
access-expr(id, views) ::= <<
<if(views.(id))><view-access-expr(views.(id))><else>shared.<id><endif>
>>
```

`access-expr` handles BOTH the plain-shared case (`shared.X`) and every view/axis
case (`shared.X.p[0]`, `KDL::diff(KDL::Rotation::Identity(), shared.X.M)[i]`, …).
It is the complete, correct replacement for both `cpp_access_expr` (Python) and
`_shared_expr` (codegen_artifacts).

### The seam

`introspection-controller-sample` does not currently receive `views`. To call
`access-expr` it must. The introspection samples are rendered from
`ir["introspection_artifacts"]` which `codegen.py` builds via
`write_introspection_artifacts` / `build_introspection_model`. The cleanest
behavior-preserving move: keep `codegen_artifacts` computing the final expr
string, but compute it from the abstract id via the SAME logic as the template
(a full view-aware access), so ir_gen no longer bakes it. **However** the goal is
to move C++ into templates, not into a second Python module. So the correct end
state is: the template calls `access-expr`, and `codegen_artifacts` passes the
abstract signal id + views through instead of a baked string.

## Commands you will need

| Purpose | Command | Expected on success |
|---|---|---|
| Activate env | `source .venv/bin/activate` (from workspace root `/home/batsy/work/ms`) | prompt shows venv |
| Unit tests | `cd src/motion-spec && pytest -q` | all pass |
| Golden regen | see "Golden C++ diff" below | no `DIFF in …` lines |

**Golden C++ diff** (the primary gate — the generated C++ must stay
byte-identical). Run from workspace root with venv active, `METAMODELS_PATH` set
to your metamodels checkout, and the STSTv4 binary on PATH (the models `Makefile`
uses it):

```bash
cd src/bdd_collab_bhv_cpp/models
MODELS="admittance_arc_single arc_demo circle_demo figure8_demo helix_demo pick_place_dual pick_place_relative pick_place_single_rnea pick_place_single pick_place_single_wait"
# 1) BEFORE touching any code, capture the baseline:
for m in $MODELS; do make -s MODEL=$m codegen DIR=/tmp/gold_before || echo "GEN FAIL $m"; done
# 2) AFTER your changes:
for m in $MODELS; do make -s MODEL=$m codegen DIR=/tmp/gold_after || echo "GEN FAIL $m"; done
# 3) Compare generated C++ only (ir.json/schema.json/*.jsonld are EXPECTED to change):
for m in $MODELS; do for f in ref_main.cpp introspection_runtime.hpp introspect_model.hpp headers/runtime.hpp; do
  [ -f /tmp/gold_before/$m/$f ] && diff -q /tmp/gold_before/$m/$f /tmp/gold_after/$m/$f | sed "s|^|DIFF $m/$f: |";
done; done
```

Expected: no `DIFF …` and no `GEN FAIL …` lines. If a model legitimately has no
FSM/controllers the file may be absent on both sides — that is fine (the `-f`
guard skips it).

## Scope

**In scope:**
- `src/motion-spec/src/motion_spec/ir_gen.py` — `add_controller_signal_metadata` only.
- `src/motion-spec/src/motion_spec/codegen_artifacts.py` — `_controller_slot`, `build_introspection_model`, `_shared_expr`.
- `src/motion-spec/code-generator/introspection.stg` — `introspection-controller-sample` and its one call site.

**Out of scope (do NOT touch):**
- `cpp_access_expr` itself — it has other callers (quantity samples, pose
  components) removed by later plans; deleting it here breaks them. Leave it.
- `error_expr` / `output_expr` (line 498-499 of codegen_artifacts) — those derive
  from `_shared_expr(error_signal)` which is a plain shared signal, handled in a
  later plan. Do not change them here.
- Any other `.stg` file.

## Steps

### Step 1: Capture the golden baseline

Run part (1) of "Golden C++ diff" above. Confirm no `GEN FAIL` lines. This is
your reference; do not skip it.

### Step 2: ir_gen — stop baking controller exprs, emit ids only

In `add_controller_signal_metadata.annotate`, replace the three
`_set_field(controller, "*_expr", cpp_access_expr(...))` calls with the abstract
ids only:

```python
if measured_id:
    _set_field(controller, "measured_signal", measured_id)
if setpoint_id:
    _set_field(controller, "setpoint_signal", setpoint_id)
if measured_derivative_id:
    _set_field(controller, "measured_derivative_signal", measured_derivative_id)
```

(Note the new `measured_derivative_signal` id replacing `measured_derivative_expr`.)

### Step 3: codegen_artifacts — pass ids + views, render via access-expr in the template

The introspection template must call `access-expr`, which needs `views`. Two
sub-changes:

3a. In `_controller_slot`, replace the three `*_expr` reads with the abstract
signal ids:

```python
"measured_signal": controller.get("measured_signal"),
"setpoint_signal": controller.get("setpoint_signal"),
"measured_derivative_signal": controller.get("measured_derivative_signal"),
```

3b. In `build_introspection_model`, drop the `slot.get("measured_expr") or …`
fallback and pass the ids straight through to the model so the template renders
them. The `_shared_expr` fallback for `measured_signal`/`setpoint_signal` must be
removed for controllers (it was the *incomplete* renderer). Emit
`measured_signal` / `setpoint_signal` / `measured_derivative_signal` on the
controller slot, and ensure the introspection model dict carries the `views` map
so the template can resolve them. Look at how `views` reaches other rendered
sub-templates in this module and thread it through the controller sample the
same way.

### Step 4: introspection.stg — render via access-expr

Change the call site so `views` is in scope for `introspection-controller-sample`
and rewrite it to call `access-expr`:

```
introspection-controller-sample(slot, views) ::= <<
        pub.constraint_sample(<slot.uri>, <access-expr(slot.error_signal, views)>, <access-expr(slot.output_signal, views)>, <access-expr(slot.measured_signal, views)>, <access-expr(slot.setpoint_signal, views)>, constraint_satisfied(<access-expr(slot.error_signal, views)>));
>>
```

Adjust `error_signal`/`output_signal` naming to whatever the slot actually
carries — **but** keep `error_expr`/`output_expr` behavior identical for this
plan if they are still baked upstream (out of scope). If `error_expr` is still a
pre-baked plain-shared string, keep using `<slot.error_expr>` for it and only
convert `measured`/`setpoint` here. The rule: convert only the two fields this
plan owns; leave the rest byte-identical.

### Step 5: Golden diff + tests

Run part (2)+(3) of "Golden C++ diff" and `pytest -q`. Both must be clean.

**Verify**: golden diff → no `DIFF …` lines; `pytest -q` → all pass.

## Test plan

- No new C++ behavior, so the golden diff IS the regression test (byte-identical
  generated C++ across all 10 example models).
- `cd src/motion-spec && pytest -q` — the existing `tests/test_codegen_artifacts.py`
  covers the artifact builder; it must still pass. If it asserts on
  `measured_expr`, update those assertions to the new `measured_signal` id shape
  (this is an expected, in-scope test change).

## Done criteria

ALL must hold:

- [ ] `grep -n 'measured_expr\|setpoint_expr\|measured_derivative_expr' src/motion-spec/src/motion_spec/ir_gen.py` returns nothing.
- [ ] Golden C++ diff prints no `DIFF …` and no `GEN FAIL …` lines.
- [ ] `cd src/motion-spec && pytest -q` exits 0.
- [ ] No files outside the in-scope list are modified (`git -C src/motion-spec status`).
- [ ] `plans/README.md` status row updated.

## STOP conditions

Stop and report (do not improvise) if:
- The "Current state" excerpts don't match the live code (drift).
- The golden diff shows any C++ change you cannot eliminate after one honest fix
  attempt — a diff here means the template's `access-expr` renders a controller
  signal differently than the old baked expr (likely a view whose axis handling
  differs). Report the exact model + diff.
- Threading `views` into the controller sample requires touching a `.stg` or
  Python file outside the scope list.

## Maintenance notes

- After this plan, `cpp_access_expr` still exists (other callers). Its deletion is
  Plan 008's final sweep, gated on all callers gone.
- Reviewer: confirm the golden diff was actually run over all 10 models, not just
  one, and that `error_expr`/`output_expr` were intentionally left untouched.
- This establishes the pattern (emit id → template `access-expr`) that plans
  004/005 reuse.
