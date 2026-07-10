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

### Phase 5 — GuardedMotionBlock decompose (finding A, headline)
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

## Status
- [x] Phase 1 — View.subobject narrowed; Trajectory.value + AccelerationConstraintSpecification.attached_to removed. Codegen byte-identical across 5 models, ir.json drops only dead keys, pytest 27 green.
- [x] Phase 2 — MotionArmSolver → HandlerArmSolver. Codegen byte-identical (no template branches on the type string), ir.json shows only the 10 renames, pytest 27 green.
- [ ] Phase 3
- [ ] Phase 4
- [ ] Phase 5
- [ ] Phase 6
