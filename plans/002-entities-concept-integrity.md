# 002 — entities.py concept-integrity refactor

Make every IR dataclass model **one concept**: its fields are that concept's own
properties; other concepts appear as typed references, never copies, derived
restatements, or discriminator-and-optionals bags. Source: whole-repo ponytail
audit of `src/motion_spec/entities.py` (findings A–G below).

The file already shows the target pattern: `Constraint.parameter:
Equality|Unilateral|Bilateral|Outside` delegates a variant to a typed reference
instead of a `type:str` + optionals. Replicate that; delete the rest.

## Guiding principle
One dataclass = one concept. No god-objects, no projections/copies of another
dataclass, no fields that restate other fields, no foreign concerns, no
discriminator strings standing in for subtypes, no unions/lists broader than the
parser ever produces.

## Verification protocol (every phase)
Codegen is behavior-critical (generates the robot C++). Harness in scratchpad:
- `regen.sh <label>` — DSL→jsonld→ir→codegen for 5 models into a fixed DIR, snapshots
  artifacts. Models: pick_place_single, _rnea, _wait, circle_demo, admittance_arc_single.
- `vdiff.sh base after` — **codegen artifacts (`*.hpp/*.cpp/schema.json/frame_layout.json/
  fsm_ir.json`) must be byte-identical**; `ir.json` diffs are inspected (dead-key drops OK).
- `pytest` (27 tests, `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`) stays green.
- Reference run for behavior-changing phases: `$GRC_SCRIPT_RUN pick_place_single --headless`.

Commit + push after each phase.

## Findings → phases

### Phase 1 — safe cleanups, no template changes (byte-identical codegen)
- **View.subobject**: `Quantity | Position | Orientation` → `Quantity`. Parser
  (`view()`, ir_gen ~1657) always builds a `Quantity`; the other two are dead. Annotation only.
- **Trajectory.value**: field typed `None = None`. Dead placeholder — remove.
- **AccelerationConstraintSpecification.attached_to**: never read (templates read only
  `force.attached_to` on the *cartesian* spec). Remove field + parser lookup.
- Keep `SceneObject.is_scene_object` (load-bearing: duck-typed `getattr(of,"is_scene_object")`).
- Keep `MotionDrivers.has_cartesian_force`, `PoseAxisErrorGroup.linear_*/angular_*`,
  `CartesianForceSpecification.attached_to` — all read by `.stg` templates.

### Phase 2 — MotionArmSolver rename (finding A/#5)
`MotionArmSolver` is a per-handler **projection** of `SolverWithInputAndOutput` (ir_gen ~1937)
and the name lies (it's not "the solver", it's this handler's slice). Rename → `HandlerArmSolver`
(class + `type`). Verify templates don't branch on the `type` string ⇒ codegen identical.

### Phase 3 — AccelerationConstraintSpecification flatten (finding F)
Templates already assume one spec (`first(motion_driver.acceleration_constraint).constraints`,
solver.stg:98/278). Collapse `MotionDrivers.acceleration_constraint: list[Spec]` →
`list[AccelerationConstraint]`; adjust solver.stg accordingly. Delete the wrapper dataclass.
Verify codegen identical after template edit.

### Phase 4 — Position/Orientation endpoints narrow (finding #4, user flagship)
`Position.of/with_respect_to` accept `SimplicialComplex|Point|Frame|SceneObject`; semantically
a Position is *of a Point wrt a Point*. Narrow to `Point`; `Orientation` to `Frame`. Requires
`rdf.py`/`ir_gen` to resolve frame→origin-point, body→ref-point at parse. Behavior-changing IR ⇒
reference run + codegen review (of/wrt ids may change; keep generated behavior equivalent).

### Phase 5 — GuardedMotionBlock decompose (finding A, headline) — NEEDS REFERENCE RUN
Measured consumption surface (do these together):
- Templates: `motion.control_mode` (motion.stg:491), `motion.controllers` (motion.stg:386),
  and the loop var `motion` throughout motion.stg. Only motion.stg reads the duplicated fields.
- `codegen.py`: `ir["motions"]` is post-processed (add_until/when_monitor_conditions,
  add_motion_done_conditions, …) then **merged into `ir["unique_motions"]`** (codegen.py ~999-1012),
  which is what templates render. Any block restructure must update the merge + the dict-key reads.
- Because block structure (hence ir.json) changes, codegen is NOT byte-identical here; oracle is
  `$GRC_SCRIPT_RUN pick_place_single --headless` (behavior) + inspecting the codegen diff is intended.
Minimal step: block holds `handler: ConstraintHandler`; drop the 4 copied fields
(motion/control_mode/controllers + `handler:str`); rewrite the ~4 template/codegen reads to go
through `handler`. Full step: peel the solver/data-flow group (arm_solvers, snapshots,
relative_poses, scene_relative_poses, pose_axis_error_groups, forwarded_commands,
has_entry_snapshot) into a sibling `MotionCodegen` the emitter zips by id.
30-field god-object keyed by `handler.motion.id`, pretending to be the GuardedMotion. Every group
is already a proper dataclass; the block is a materialized per-handler join. Target: the block
holds `handler: ConstraintHandler` (drop the 4 copied fields: motion/control_mode/controllers +
the `handler:str` backref) and the solver/data-flow group (`arm_solvers, snapshots, relative_poses,
scene_relative_poses, pose_axis_error_groups, forwarded_commands, has_entry_snapshot`) moves to a
sibling `MotionCodegen`/`SolverPlan` the emitter zips by id. Codegen is its only consumer ⇒ large
codegen.py + template refactor; reference run is the oracle.

### Phase 6 — discriminator-strings → typed subtypes (finding C/D)
- **MonitorEntry**: `monitor_type:str` + level-only/edge-only optionals → `LevelMonitor`/`EdgeMonitor`
  referenced (follow `Constraint.parameter`).
- **Controller**: P/PID/impedance flattened into 8 optional gains → typed variants.
- **Wrench.sensor_name / debounce_steps / is_elapsed**: foreign concerns; lift out.
- **authored/snapshot/has_view**: cross-cutting role-flags repeated on ~8 quantity types → one
  `role`/provenance the quantities reference.
- **PoseDifference ≡ AccelerationTwist ≡ Wrench−sensor_name**: identical structs + copy-paste
  constructors. Keep the distinct concepts, share the construction via one helper (code-dup, not
  concept-dup). Optionally a shared base for the common coordinate fields.

Phase 6 is the largest; sequence subtypes first (MonitorEntry, then Controller), role-flags last.

Sub-status / measured surfaces:
- **6a DONE** — twist/pose-diff/wrench constructor dedup via `_spatial_coordinate_fields`
  (3 distinct concepts kept). Byte-identical, pytest 27.
- **6b MonitorEntry → LevelMonitor/EdgeMonitor** (needs run): template reads are
  `monitor.flag` (level), `monitor.event/event_idx/event_name` (edge), plus `is_edge_triggered`,
  `error`, `debounce_steps`; codegen branches on `monitor_type == "EdgeTriggeredMonitor"`
  (ir_gen:3528) and `codegen_artifacts` emits `monitor.get("monitor_type")`. Disjoint-field split
  changes ir.json shape → verify every `<monitor.event*>`/`<monitor.flag>` is guarded by trigger
  type, then confirm reference run reaches S_DONE @ ~22308.
- **6c Controller → P/PID/impedance variants** (needs run): codegen + codegen_artifacts read the
  optional gains directly for the introspection contract.
- **6d role-flag extraction** (authored/snapshot/has_view): most invasive — every quantity type +
  every consumer; do last, own session.
- **6e foreign-concern lifts**: Wrench.sensor_name, MonitorEntry.debounce_steps (derived from
  debounce_duration_s), ConstraintEvaluator.is_elapsed/elapsed_*.

## Status
- [x] Phase 1 — View.subobject narrowed; Trajectory.value + AccelerationConstraintSpecification.attached_to removed. Codegen byte-identical across 5 models, ir.json drops only dead keys, pytest 27 green.
- [x] Phase 2 — MotionArmSolver → HandlerArmSolver. Codegen byte-identical (no template branches on the type string), ir.json shows only the 10 renames, pytest 27 green.
- [x] Phase 3 — AccelerationConstraintSpecification flattened away; MotionDrivers.acceleration_constraint is now list[AccelerationConstraint]. solver.stg adjusted (first().constraints → the list). All 111 drivers had exactly 1 spec, so byte-equivalent. Codegen identical, wrapper gone from ir.json (30→0), pytest 27 green.
- [x] Phase 4 — Position.of/wrt narrowed to `Point | None`; Orientation.of/wrt to `Frame | SceneObject | None` (dead SimplicialComplex dropped). Measured across golden models: Position endpoints are ALWAYS Point (never Frame/SC/SceneObject); Orientation endpoints are Frame or SceneObject (never SC, never Point) — so my original "narrow to Frame" was wrong, SceneObject is real. position_reference now enforces Point-only (metamodel: a Position is of a Point wrt a Point). Pure model correction: codegen byte-identical, zero of/wrt value changes in ir.json, pytest 27 green.
- [x] Phase 5 — removed the dead aggregation from GuardedMotionBlock: the embedded `motion: GuardedMotion` (block never re-read it — the purest god-object violation) and the three unread `*_events` lists (computed at construction, consumed nowhere). block.handler stays as a legit id reference (used for sort order). Verified: codegen byte-identical + pytest 27 + reference run reaches S_DONE at steps=22308 with identical FSM sequence and cube pose. Deferred: peeling the solver/data-flow group (arm_solvers/snapshots/…) into a sibling struct is pure reorg of *consumed* data (116+ template refs, no behavior change) — low ROI/high churn; left for a focused pass if desired.
- [ ] Phase 6 — staged; subtypes + role-flag extraction + twist constructor dedup (steps above)
