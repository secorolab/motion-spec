# Plan 009: Eliminate the `derive_codegen_fields` post-pass — compute every field during construction

> **Executor instructions**: Follow step by step. Run the golden diff + pytest
> after every step (not just at the end) — this refactor moves *where* fields are
> computed without changing *what* they are, so generated C++ and `ir.json`
> content must stay identical throughout. Honor STOP conditions. Update this
> plan's row in `plans/README.md` when done.
>
> **Drift check (run first)**:
> `git -C src/motion-spec diff --stat f2f14ee..HEAD -- src/motion_spec/ir_gen.py`
> If `ir_gen.py` changed since this plan was written, re-map the line references
> below before proceeding.

## Status

- **Priority**: P1
- **Effort**: L
- **Risk**: MED
- **Depends on**: 003–008 (DONE — those made the derivations emit clean structured IR; this plan relocates *where* they run)
- **Category**: tech-debt / architecture
- **Planned at**: commit `f2f14ee`, 2026-07-11

## Why this matters

`ir_gen.generate_ir()` builds all the model pieces, assembles them into the
`ir = {…}` dict, and **then** calls `derive_codegen_fields(ir)` — which takes that
finished dict back as input and runs 16 mutation passes over it. The IR is the
final artifact of ir_gen; re-reading and re-processing it after assembly is the
architectural smell. It forces the awkward `_field`/`_set_field`/`_as_dict`
"works on a dict OR a dataclass" shim (because the pass runs over the assembled
structure), splits each piece's logic across two distant places, and makes the
data flow a two-phase build-then-patch instead of a single forward pass.

**Target:** `generate_ir` computes every codegen-facing field *while constructing
the piece it belongs to*, in dependency order, and assembles a **complete** `ir`
dict once at the end. `derive_codegen_fields` and the entire
`# Codegen-facing derivations` section header are deleted. No function takes the
assembled `ir` dict as a parameter.

This is behavior-preserving: the same fields end up in `ir.json`, so generated
C++ stays byte-identical. Only the *location and timing* of the computation move.

## Current state

`generate_ir` (ir_gen.py ~4438–4561) today:

```python
def generate_ir(manifest_path):
    ... load graph, parser, indexes ...
    setups_by_node, ordered_setups = _robot_setups_from_graph(g)
    scene = _scene_from_graph(g)
    (slv_base_vel, sched1, hdl, sched2, slv_arm, sched3, slv_base_frc, sched4) = _solver_sections(...)
    _assign_monitor_event_indexes(hdl)
    closures = p.closures(...); view_map = p.view(); data_structures = p.data_structures()
    ... snapshot maps, reference maps, wrench_outputs ...
    motions = build_motion_units(g, p, hdl, node_by_id, slv_arm, ... view_map, ... closures, data_structures)
    _apply_solver_control_modes(slv_arm, motions)
    backend = _backend_from_graph(g)
    control_period_ns = int(round(scene.timestep_s * 1e9))
    ... rne_damping, debounce, id-collision guard ...
    shared_data = _filter_shared_data(...) + _shared_runtime_members(slv_arm, motions)
    introspection = _build_introspection(..., motions, data_structures, control_period_ns, backend, scene)
    ir = { ...big dict... "fsm": _fsm_from_graph(g) }
    derive_codegen_fields(ir)     # <-- THE POST-PASS TO DELETE
    return ir
```

`derive_codegen_fields(ir)` (ir_gen.py ~4364–4435) runs these 16 operations, all
reading/mutating the assembled `ir` dict via `_field`/`_set_field`:

| # | Operation (current fn) | Reads | Writes | Natural construction home |
|---|---|---|---|---|
| 1 | `_validate_ir(ir)` | `arm_solvers`, `backend`, solver outputs | (raises) | `generate_ir`, right after `slv_arm` + `backend` known — call as `_validate_solvers(slv_arm, backend)` |
| 2 | scene vector/geometry expansion (inline loop + `expand_vector_fields`/`require_field`) | `scene.robots`, `scene.objects` | `pos_x`, `euler_x`, `size_x`, `color_r`, `friction_*`, `has_path` on scene items | **inside `_scene_from_graph`** |
| 3 | `_annotate_runtime_robots(ir, backend)` | `arm_solvers`, `motions[].arm_solvers`, `backend` | `runtime_id`, `runtime_owner`, `tool_body`/`tcp_site` None-normalize | `generate_ir` after `motions` built — `_annotate_runtime_robots(slv_arm, motions, backend)` |
| 4 | `build_pose_components(ir)` | `views`, `data` | returns `pose_components` | `generate_ir` after `view_map`+`data_structures` — `build_pose_components(view_map, data_structures)` |
| 5 | `declared_pose_component_entries(ir, pc)` (top-level) | `pose_components`, `data` | `ir["declared_pose_components"]` | same place as #4 (from `pose_components`, `data_structures`) |
| 6 | `resolve_lerp_closures(ir, pc)` | `closures`, `pose_components` | closure fields | `generate_ir` after closures + `pose_components` — `resolve_lerp_closures(closures, pose_components)` |
| 7 | `resolve_arc_closures(ir)` | `closures`, `data` | closure fields | same place — `resolve_arc_closures(closures, data_structures)` |
| 8 | per-motion `declared_pose_components` | `motions`, `closures`, `pose_components`, `data` | `motion.declared_pose_components` | **inside `build_motion_units`** (pass `pose_components` in) |
| 9 | `needs_clock_time` | `motions` | `ir["needs_clock_time"]` | `generate_ir` inline at assembly: `any(m.has_elapsed for m in motions)` |
| 10 | `_apply_fsm_wiring(ir)` | `fsm`, `motions` | monitor/motion fsm fields | **inside `build_motion_units`** (pass `fsm` in) — or a `_apply_fsm_wiring(motions, fsm)` call right after |
| 11 | `add_motion_function_interfaces(motions)` | `motions` | `*_needs_*` booleans | **inside `build_motion_units`** (per motion, after conditions/poses/fsm set) |
| 12 | `_apply_fsm_gate_calls(ir)` | `motions`, `when_needs_*` | `fsm_when_gate_calls` | after #10+#11 — `_apply_fsm_gate_calls(motions, fsm_namespace)` |
| 13 | `add_controller_signal_metadata(ir)` | `motions[].controllers`, `closures`, `views` | `measured_signal`, `setpoint_signal` | **inside `build_motion_units`** (controllers are built there) |
| 14 | `add_controller_internal_state_logging(ir)` | `closures`, `motions`, mutates `shared_data`+`introspection` | shared_data items, quantities, closure samples | **inside `_build_introspection`** (pass `shared_data`, `closures`; mutate same list object) |
| 15 | `add_quantity_samples(ir)` | `introspection.quantities`, `shared_data`, `views` | `introspection.quantity_samples` | **inside `_build_introspection`** (pass `views`, `shared_data`) |
| 16 | `add_spatial_samples(ir)` | `shared_data` | `introspection.spatial_samples` | **inside `_build_introspection`** (pass `shared_data`) |

The `_field`/`_set_field`/`_as_dict` shims (ir_gen.py ~3630–3660) exist so those
16 ops can run on the assembled dict-or-dataclass structure. After this refactor,
each op runs on a piece of known type — native dataclasses (`Motion`, `Solver`,
`Scene`…) or genuine dicts (`closures`, `views`, `introspection`). Keep `_field`/
`_set_field` only where a piece is genuinely a dict (closures, views,
introspection quantities); use direct attribute access on the dataclasses.

Builder signatures today:
- `build_motion_units(g, p, hdl, node_by_id, slv_arm, *, snapshot_source_map, snapshot_clock_map, view_map, closure_output_map, data_reference_map, closure_input_map, closures, data_structures)` (ir_gen.py:2220) — already has views/closures/data; add `pose_components` and `fsm`.
- `_build_introspection(*, app_model_path, imported_models, imported_provenance, id_nodes, node_by_id, motions, data_structures, control_period_ns, backend, scene)` (ir_gen.py:3142) — add `views` and `shared_data`.
- `_scene_from_graph(g)` (ir_gen.py:2924) — fold the expansion in.

Comments in the current code confirm this direction is already partly done:
`build_motion_units` already folds group flags, trajectory progress, elapsed
flags, and the when/until/done **conditions** (`_set_motion_conditions`). This
plan finishes the same job for the remaining 16 ops.

## Ordering (the dependency-correct forward pass)

`generate_ir` must be reordered so each piece is built after its inputs:

1. graph, parser, indexes, setups
2. **`backend = _backend_from_graph(g)`** — moved UP (needed by #1 validation and #3)
3. **`fsm = _fsm_from_graph(g)`** — moved up (needed by #10 inside build_motion_units)
4. `scene = _scene_from_graph(g)` — now returns fully-expanded scene (#2)
5. `_solver_sections(...)` → `slv_*`, `hdl`, `sched*`; then `_validate_solvers(slv_arm, backend)` (#1)
6. `closures`, `view_map`, `data_structures`, snapshot/reference maps, `wrench_outputs`
7. `pose_components = build_pose_components(view_map, data_structures)` (#4); top-level `declared_pose_components` (#5); `resolve_lerp_closures(closures, pose_components)` (#6); `resolve_arc_closures(closures, data_structures)` (#7)
8. `motions = build_motion_units(..., pose_components=pose_components, fsm=fsm)` — folds per-motion declared poses (#8), fsm wiring (#10), function interfaces (#11), controller signal metadata (#13), and (already) conditions
9. `_apply_solver_control_modes(slv_arm, motions)`; `_annotate_runtime_robots(slv_arm, motions, backend)` (#3); `_apply_fsm_gate_calls(motions, fsm)` (#12)
10. `control_period_ns`, `rne_damping`, `_apply_monitor_debounce`, `p.assert_no_id_collisions()`
11. `shared_data = _filter_shared_data(...) + _shared_runtime_members(...)`
12. `introspection = _build_introspection(..., views=view_map, shared_data=shared_data)` — folds controller-internal-state logging (#14, appends to the SAME `shared_data` list), quantity samples (#15), spatial samples (#16)
13. assemble the **complete** `ir` dict, with `needs_clock_time = any(m.has_elapsed for m in motions)` (#9) inline and `fsm` from step 3
14. `return ir` — **no** `derive_codegen_fields` call

### Sharp edges to get right

- **#14 mutates `shared_data`**: `add_controller_internal_state_logging` appends
  controller-internal-state items to `shared_data` and quantities to
  `introspection`. `shared_data` is built (step 11) *before* `_build_introspection`
  (step 12) and the SAME list object is placed in the `ir` dict (step 13), so the
  appends must happen on that object inside `_build_introspection` and be visible
  in the final dict. Order inside `_build_introspection`: internal-state logging
  first (it grows shared_data + quantities), then quantity samples (reads them),
  then spatial samples.
- **#8 needs `pose_components`** which is built in step 7 (after views+data,
  before motions) — thread it through `build_motion_units`.
- **#10 fsm wiring needs `fsm`**: `_fsm_from_graph` is cheap and pure; call it in
  step 3 and pass `fsm` into `build_motion_units`. Do the wiring per motion (or in
  one pass at the end of `build_motion_units` over the built motions).
- **#11 function interfaces** must run *after* a motion's conditions,
  `declared_pose_components`, monitors, and fsm wiring are set (they read
  `has_pose`, `when_monitors`, `fsm_namespace`). So within `build_motion_units`,
  interfaces come last per motion (or in a final loop over all motions).
- **#12 gate calls** read `when_needs_*` (set by #11) — must run after motions are
  fully built. Keep it as a small pass in `generate_ir` step 9, taking `motions`.

## Commands you will need

| Purpose | Command | Expected |
|---|---|---|
| Env | `source .venv/bin/activate` (from `/home/batsy/work/ms`); `export METAMODELS_PATH=$PWD/src/metamodels` | — |
| Unit tests | `cd src/motion-spec && pytest -q` | 27 passed |
| Golden C++ + ir.json | see below | no diffs |

**Golden diff (the gate — run after EVERY step).** This refactor must not change
`ir.json` content *or* generated C++. Capture a baseline once from HEAD before any
edit, then compare after each step. Regenerate all 8 buildable models and diff
**both** the generated C++ (`.cpp/.hpp/.h`, recursive) **and** `ir.json` (which
must now also be identical, since only the computation location moves):

```bash
cd src/bdd_collab_bhv_cpp/models
MODELS="admittance_arc_single arc_demo circle_demo figure8_demo helix_demo pick_place_single_rnea pick_place_single pick_place_single_wait"
# baseline (before editing): PHASE=before ; after each step: PHASE=after
for m in $MODELS; do make -s MODEL=$m codegen DIR=/tmp/dcf_$PHASE >/dev/null || echo "GEN FAIL $m"; done
# compare (run once in an 'after' phase):
for m in $MODELS; do
  for f in $(cd /tmp/dcf_before/$m && find . -name '*.cpp' -o -name '*.hpp' -o -name '*.h'); do
    diff -q /tmp/dcf_before/$m/$f /tmp/dcf_after/$m/$f >/dev/null 2>&1 || echo "C++ DIFF $m/${f#./}"; done
  # ir.json must be identical too now (unlike plans 003-008, nothing about its content changes)
  python3 -c "import json,sys; a=json.load(open('/tmp/dcf_before/$m/ir.json')); b=json.load(open('/tmp/dcf_after/$m/ir.json')); sys.exit(0 if a==b else print(f'IR DIFF $m'))"
done
```

Expected after every step: no `C++ DIFF`, no `IR DIFF`, no `GEN FAIL`. **Note the
ir.json equality check** — this is a stronger gate than plans 003–008 used, and it
is the whole point: moving computation must not change the artifact.
`frame_layout.h` hashes should now also be **unchanged** (schema content is
identical), unlike plans 003–008.

## Scope

**In scope:** `src/motion-spec/src/motion_spec/ir_gen.py` only. Specifically:
`generate_ir`, `_scene_from_graph`, `build_motion_units`, `_build_introspection`,
the 16 helper functions, and deletion of `derive_codegen_fields` + the
`# Codegen-facing derivations` banner. Adjust `tests/` only if a test imports or
calls `derive_codegen_fields` or a helper whose signature changes.

**Out of scope:**
- Any `.stg` template (they consume the same fields — untouched).
- `codegen_artifacts.py`, `codegen.py` (they read the finished `ir.json` — the
  contract is unchanged).
- Renaming or changing any emitted field name/shape (that would break templates).
- The pre-existing broken models `pick_place_dual` / `pick_place_relative`.

## Steps

Do these in order; each ends green (golden + pytest). Move ONE operation at a
time so a regression is localized.

### Step 0: Baseline
Capture the golden baseline (PHASE=before). Confirm no `GEN FAIL`.

### Step 1: Move `backend` and `fsm` derivation up
Move `backend = _backend_from_graph(g)` and `fsm = _fsm_from_graph(g)` to the top
of `generate_ir` (after the parser/indexes, before solver sections). Use these
locals throughout; keep `"fsm": fsm` in the dict. Nothing else changes yet.
**Verify:** golden + pytest clean.

### Step 2: Fold scene expansion into `_scene_from_graph` (op #2)
Move the scene robots/objects/attachments vector + size/color/friction/has_path
expansion loop out of `derive_codegen_fields` into the end of `_scene_from_graph`,
operating on the native scene dataclasses. Delete that loop from
`derive_codegen_fields`. (`expand_vector_fields`/`require_field` move with it or
become module helpers called there.)
**Verify:** golden + pytest clean.

### Step 3: Fold pose components + lerp/arc into `generate_ir` construction (ops #4–7)
After `view_map`/`data_structures` are built, call
`pose_components = build_pose_components(view_map, data_structures)` and set
`declared_pose_components` from `(pose_components, data_structures)`. After
closures are built, call `resolve_lerp_closures(closures, pose_components)` and
`resolve_arc_closures(closures, data_structures)`. Change those four functions'
signatures from `(ir_payload)` to the explicit pieces. Remove them from
`derive_codegen_fields`. Store `ir["pose_components"]` / `ir["declared_pose_components"]`
at final assembly.
**Verify:** golden + pytest clean.

### Step 4: Fold per-motion derived fields into `build_motion_units` (ops #8, #10, #11, #13)
Pass `pose_components=` and `fsm=` into `build_motion_units`. Inside it, for each
built motion, run (in this order): controller signal metadata (#13), per-motion
`declared_pose_components` (#8), fsm wiring (#10), then function-interface
booleans (#11). Change those helpers to take the motion/closures/views/fsm pieces
instead of the `ir` dict. Remove them from `derive_codegen_fields`.
**Verify:** golden + pytest clean. (FSM path: check `pick_place_single` gate
calls + `admittance_arc_single` fsm state.)

### Step 5: Fold runtime-robot annotation, validation, gate calls (ops #1, #3, #12)
In `generate_ir`: after solvers+backend, `_validate_solvers(slv_arm, backend)`
(#1); after motions, `_annotate_runtime_robots(slv_arm, motions, backend)` (#3)
and `_apply_fsm_gate_calls(motions, fsm)` (#12). Change signatures off the `ir`
dict. Remove from `derive_codegen_fields`.
**Verify:** golden + pytest clean.

### Step 6: Fold introspection samples into `_build_introspection` (ops #14–16)
Pass `views=view_map` and `shared_data=shared_data` into `_build_introspection`.
Inside, after building the base introspection dict, run internal-state logging
(#14, mutating the passed `shared_data` list + quantities), then quantity samples
(#15), then spatial samples (#16). Change those helpers off the `ir` dict. Remove
from `derive_codegen_fields`. Ensure the `shared_data` list object mutated here is
the same one placed in the `ir` dict.
**Verify:** golden + pytest clean — pay attention to the pool sizes / frame layout
(internal-state items must still land in `shared_data`).

### Step 7: Delete the post-pass and inline `needs_clock_time` (op #9)
`derive_codegen_fields` should now be empty except `needs_clock_time`. Inline
`ir["needs_clock_time"] = any(m.has_elapsed for m in motions)` at dict assembly,
delete `derive_codegen_fields`, delete its call in `generate_ir`, and delete the
`# ----- Codegen-facing derivations … -----` banner comment block.
**Verify:** golden + pytest clean. `grep -n "derive_codegen_fields\|Codegen-facing derivations" src/motion_spec/ir_gen.py` → nothing.

### Step 8: Simplify the dict-or-dataclass shims
Where a moved helper now operates on a known-native dataclass, replace
`_field(x, "k")`/`_set_field(x, "k", v)` with `x.k`/`x.k = v`. Keep `_field`/
`_set_field` only for the genuinely-dict pieces (closures, views, introspection
quantities). If `_as_dict` becomes unused, delete it. Do NOT force this where a
helper still legitimately handles both — correctness/byte-identity first.
**Verify:** golden + pytest clean.

## Test plan

- The golden diff (C++ **and** `ir.json` equality across 8 models) after every
  step is the regression gate — this refactor's contract is "identical artifact".
- `pytest -q` after every step. If a test imports `derive_codegen_fields` or a
  renamed/re-signatured helper, update the import/call to the new construction
  entry point (in-scope). Do not weaken an assertion to make it pass — if a value
  actually changed, that's a real regression → STOP.
- Add no new tests unless a moved helper loses its only existing coverage; if so,
  add one `assert`-based check next to the existing codegen tests.

## Done criteria

- [ ] `grep -n "derive_codegen_fields" src/motion-spec/src/motion_spec/ir_gen.py` returns nothing.
- [ ] `grep -n "Codegen-facing derivations" src/motion-spec/src/motion_spec/ir_gen.py` returns nothing.
- [ ] No function in ir_gen.py takes a parameter named `ir`/`ir_payload` that is the assembled output dict (grep `def .*(ir[_:) ]`; the only dict named `ir` is the local built at the end of `generate_ir`).
- [ ] Golden: no `C++ DIFF`, no `IR DIFF`, no `GEN FAIL` across all 8 models.
- [ ] `frame_layout.h` hashes are unchanged vs baseline (schema content identical).
- [ ] `cd src/motion-spec && pytest -q` → 27 passed.
- [ ] `plans/README.md` row updated.

## STOP conditions

Stop and report (do not improvise) if:
- Any step produces a `C++ DIFF` or `IR DIFF` you cannot eliminate — the move
  reordered a dependency (e.g. a field read before it is written). Report the
  model + the diff; the fix is ordering, never changing a value.
- A moved operation needs a piece that isn't available at its new location
  (a genuine ordering cycle). Report it; the resolution is to move the *input's*
  construction earlier, not to keep the post-pass.
- `add_controller_internal_state_logging`'s appends to `shared_data` don't appear
  in the final `ir` (list-object identity broken) — this shows up as changed
  frame-layout pool sizes. STOP and fix the aliasing.
- The id-collision guard `p.assert_no_id_collisions()` or `_apply_monitor_debounce`
  turns out to depend on a field a moved op sets — re-check ordering, report.

## Maintenance notes

- After this, `generate_ir` is a single dependency-ordered forward pass and the
  `ir` dict it returns is final by construction. New codegen-facing fields must be
  computed in the builder for their piece (scene → `_scene_from_graph`, motion →
  `build_motion_units`, introspection → `_build_introspection`), never in a
  post-assembly pass. A CI grep guard for `def derive` / re-reading the assembled
  dict would prevent regressing to two-phase build.
- Reviewer: verify the golden `ir.json` equality was actually run (not just C++),
  and that `build_motion_units` didn't grow a dependency on a piece built after
  it (which would force a partial re-introduction of the post-pass).
- The `_field`/`_set_field` shims survive only for the dict-typed pieces; if those
  pieces (closures, views, introspection) are later converted to dataclasses, the
  shims can be deleted entirely.
