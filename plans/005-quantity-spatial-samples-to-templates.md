# Plan 005: Quantity & spatial introspection samples render from structured descriptors

> **Executor instructions**: Follow step by step; verify each step. Honor STOP
> conditions. Update this plan's row in `plans/README.md` when done.
>
> **Drift check (run first)**:
> `git -C src/motion-spec diff --stat 41ca73d..HEAD -- src/motion_spec/ir_gen.py src/motion_spec/codegen_artifacts.py code-generator/introspection.stg code-generator/shared_data.stg`
> Mismatch against the "Current state" excerpts = STOP.

## Status

- **Priority**: P1
- **Effort**: M
- **Risk**: MED
- **Depends on**: none (but do 003 first to avoid churn on shared code)
- **Category**: tech-debt
- **Planned at**: commit `41ca73d`, 2026-07-11

## Why this matters

`add_quantity_samples` and `add_spatial_samples` in `ir_gen.py` bake C++ sample
expressions with full KDL knowledge:
`KDL::diff(KDL::Rotation::Identity(), shared.X.M)`, `shared.X.rot`,
`shared.X.force`, `static_cast<double>(shared.X)`, `shared.X ? 1.0 : 0.0`. These
are the frame-log sampling expressions read by the introspection publisher. They
belong in the template. The per-quantity type + component the sample needs is
already structured; only the final C++ string must move.

## Current state

**`ir_gen.py` `add_quantity_samples`** (around line 3890) builds rows with a
baked `sample_expr`, dispatching on quantity type:

```python
elif qtype == "Orientation" and qid in shared_ids:
    add_axes(quantity, "", f"KDL::diff(KDL::Rotation::Identity(), shared.{qid})")
elif qtype in {"Pose", "Trajectory"} and qid in shared_ids:
    add_axes(quantity, "position", f"shared.{qid}.p")
    add_axes(quantity, "orientation", f"KDL::diff(KDL::Rotation::Identity(), shared.{qid}.M)")
elif qtype in {"VelocityTwist", "AccelerationTwist", "PoseDifference"} and qid in shared_ids:
    add_axes(quantity, "angular", f"shared.{qid}.rot")
    add_axes(quantity, "linear", f"shared.{qid}.vel")
elif qtype == "Wrench" and qid in shared_ids:
    add_axes(quantity, "torque", f"shared.{qid}.torque")
    add_axes(quantity, "force", f"shared.{qid}.force")
# ... plus for scalar shared items:
add(item, "", f"shared.{item_id} ? 1.0 : 0.0")     # Bool
add(item, "", f"static_cast<double>(shared.{item_id})")  # IntCounter
# ... and view/plain scalar quantities via cpp_access_expr / shared.{qid}
```

`add(source, component, expr)` stores the row with `sample_expr=expr`, plus
already-structured fields: `source_id`, `component`, `source_type`, `type`
(`"Scalar"`). `add_axes` produces one row per x/y/z with `expr=f"{base}[{idx}]"`.

**`ir_gen.py` `add_spatial_samples`** (around line 3958) buckets shared Pose /
VelocityTwist / Wrench items into `poses/twists/wrenches` with
`expr: f"shared.{iid}"`.

**`codegen_artifacts.py` `build_introspection_model`** (line 480) copies the
baked expr to the template field name:

```python
quantities = [
    {"index": ..., "expr": quantity.get("sample_expr", "0.0"), ...}
    for quantity in schema.get("quantities", [])
]
```

**Consumer templates** — `introspection.stg`:
```
introspection-quantity-sample(slot) ::= << pub.quantity_sample(<slot.index>, <slot.expr>); >>
introspection-pose-sample(slot)    ::= << … <slot.expr>.M.GetQuaternion(…) … <slot.expr>.p.x() … >>
introspection-twist-sample(slot)   ::= << … <slot.expr>.vel.x() … <slot.expr>.rot.x() … >>
introspection-wrench-sample(slot)  ::= << … <slot.expr>.force.x() … <slot.expr>.torque.x() … >>
```

Note the pose/twist/wrench sample templates take a whole-object `<slot.expr>` and
apply `.M`/`.p`/`.vel`/`.rot`/`.force`/`.torque` **in the template already**. So
for `add_spatial_samples`, the template only needs `shared.<id>` — it already
knows the KDL members. That half is nearly free.

## Commands you will need

Same as Plan 003 (env, `pytest -q`, and the "Golden C++ diff" over all 10 models).

## Scope

**In scope:**
- `ir_gen.py` — `add_quantity_samples`, `add_spatial_samples`.
- `codegen_artifacts.py` — `build_introspection_model` quantity/spatial mapping.
- `code-generator/introspection.stg` — the 5 `introspection-*-sample` templates
  and their call sites.
- `code-generator/shared_data.stg` — MAY add a `sample-expr` sub-template here
  (next to `access-expr`) if the scalar-quantity dispatch is cleaner there.

**Out of scope:**
- `cpp_access_expr` (Plan 008 deletes it). This plan removes its
  `add_quantity_samples` caller; leave the function.
- Constraint / monitor samples (`introspection-controller-sample`,
  `introspection-monitor-sample`) — those are Plan 003 / 006.

## Steps

### Step 1: Capture golden baseline
Run part (1) of Plan 003's "Golden C++ diff".

### Step 2: spatial samples — emit id, template renders members

In `add_spatial_samples`, replace `"expr": f"shared.{iid}"` with the abstract
`"id": iid` (already present) and nothing else. In `build_introspection_model`
spatial mapping, pass the id through as the field the template reads. In
`introspection.stg`, change `introspection-pose-sample` /
`introspection-twist-sample` / `introspection-wrench-sample` to render
`shared.<slot.id>` in place of `<slot.expr>`:

```
introspection-pose-sample(slot) ::= <<
    { double _qx, _qy, _qz, _qw; shared.<slot.id>.M.GetQuaternion(_qx, _qy, _qz, _qw); pub.pose_sample(<slot.index>, shared.<slot.id>.p.x(), shared.<slot.id>.p.y(), shared.<slot.id>.p.z(), _qx, _qy, _qz, _qw); \}
>>
```
(and analogously for twist/wrench). The `.M/.p/.vel/.rot/.force/.torque` members
already live in the template — you are only replacing the pre-baked
`shared.<id>` prefix with a template-built one.

### Step 3: quantity samples — carry a structured descriptor

The quantity sample dispatch has three families:
1. **view / scalar quantity** (single value) → today `cpp_access_expr(qid, views)`
   or `shared.{qid}` or a literal → replace with a descriptor
   `{kind: "access", ref: qid}` (or `{kind: "literal", value: v}`), rendered by
   `access-expr(ref, views)` / the literal.
2. **vector quantity** (Position/Direction/FreeVector/Orientation/Pose/Twist/
   Wrench broken into x/y/z rows) → today baked with KDL member + `[idx]`. Carry
   `{kind: "<member-kind>", id: qid, axis: idx}` where `<member-kind>` is one of
   `linear|angular|position|orientation|force|torque|rot|vel` matching the old
   member. Add a `sample-expr(desc, views)` sub-template that dispatches on
   `desc.kind` to emit the exact same C++ (`shared.<id>.p[<axis>]`,
   `KDL::diff(KDL::Rotation::Identity(), shared.<id>.M)[<axis>]`, etc.).
3. **scalar shared Bool / IntCounter** → carry `{kind: "bool", id}` /
   `{kind: "int", id}`; template renders `shared.<id> ? 1.0 : 0.0` /
   `static_cast<double>(shared.<id>)`.

Emit the descriptor on each row instead of `sample_expr`. `build_introspection_model`
passes the descriptor through; `introspection-quantity-sample` calls
`sample-expr(slot.desc, views)`.

The `sample-expr` sub-template belongs next to `access-expr` in
`shared_data.stg` (it is the same category of "render access to a shared datum").
Its dispatch mirrors the removed Python branches one-to-one — keep them in the
same order so a reviewer can diff old-Python-branch ↔ new-template-branch.

### Step 4: thread `views` into the quantity sample call site

`introspection-quantity-sample` now needs `views` (for the access/literal
family). Confirm the call site in `introspection.stg` (grep
`introspection-quantity-sample`) has `views` in scope and pass it.

### Step 5: Golden diff + tests
Run parts (2)+(3) of the golden diff and `pytest -q`.

**Verify**: no `DIFF …`; `pytest -q` all pass.

## Test plan

- Golden diff over all 10 models — quantity/spatial samples appear in every
  model that logs quantities (all of them). Byte-identical required.
- `pytest -q`; `tests/test_codegen_artifacts.py` likely asserts on `sample_expr`
  / `expr` — update to the descriptor shape (in-scope).

## Done criteria

- [ ] `grep -n 'sample_expr\|static_cast<double>\|? 1.0 : 0.0\|KDL::diff\|\.rot\b\|\.vel\b\|\.force\|\.torque' src/motion-spec/src/motion_spec/ir_gen.py` returns nothing (the only KDL/member C++ for samples is gone).
- [ ] Golden C++ diff: no `DIFF …` / `GEN FAIL …`.
- [ ] `cd src/motion-spec && pytest -q` exits 0.
- [ ] No out-of-scope files modified.
- [ ] `plans/README.md` row updated.

## STOP conditions

- Drift vs "Current state".
- Golden diff shows a sample-expr change you can't eliminate — the descriptor→C++
  dispatch missed a branch. Report the model + diff; do NOT re-bake in Python.
- The `add_axes` component-naming (`position.x`, `angular.y`, …) is load-bearing
  for frame-log field names elsewhere (`codegen_artifacts` proto/field builder).
  If changing the row shape breaks `build_frame_log_proto_fields` or
  `field_names_and_format`, STOP — the row's `id`/`component`/`source_id` fields
  must stay identical; only `sample_expr` becomes `desc`.

## Maintenance notes

- `sample-expr` and `access-expr` now both live in `shared_data.stg`. A new shared
  datum type needs one branch added there, never in `ir_gen`.
- Reviewer: the frame-log schema (field names/offsets in `codegen_artifacts.py`)
  must be untouched — only the render expression moved. Diff `frame_layout.json`
  before/after (it should also be byte-identical).
