<!-- SPDX-License-Identifier: MPL-2.0 -->
# motion-spec codegen architecture

Why the current codegen is shaped the way it is, what is wrong with it, and the architecture
that replaces it. Every rule below is grounded in literature so the shape is settled once
rather than re-litigated per refactor.

Diagrams: `current-templates`, `current-ir`, `proposed-templates`, `proposed-ir`,
`proposed-runtime` (`.dot` + `.png`). Measurements are from `pick_place_single` unless stated,
re-measured at `7b9a291` (2026-08-06); cross-model claims verified on all three maintained models.

The sim/real fork surface, the external-wrench law and the accepted platform seams are a
separate docs page: [../sphinx/source/sim-real-parity.rst](../sphinx/source/sim-real-parity.rst).

---

# Do first

Ordered. Each is grounded in a section below; the section says *why*, this says *what is left*.

## A. Evict what has no contract from the blackboard — §9

`shared_data` still carries five things the model never declared:

| member | what it is | belongs in |
|---|---|---|
| `_motion_spec_events[]`, `_motion_spec_event_count`, `_motion_spec_event_buffer_size` | the coordination event buffer | **coordination**, with the FSM |
| `_motion_spec_record_event()` | a **method on a blackboard** — its callers never appear in any `consumers` list, so the read/write contract cannot be checked | not on the blackboard at all |
| `clock_time_s` | genuinely shared and read by elapsed monitors, but unmodelled: no producer, no cadence, no storage, and so not loggable like every other value | **computation**, as a modelled value |

All five are hardcoded in `entry_program.stg`'s `shared_state_header` with no IR entry —
verified at `7b9a291` on all three maintained models (the orphan set is identical in each). The
rule they violate — *a blackboard member must have a producer/consumer contract* — is already
obeyed on the IR side: the contract now lives centrally in `introspection.dataflow`, and it
covers every `shared_data` member — 606/606 (single), 1144/1144 (dual), 333/333 (arc) — each
with producer, cadence and storage.

**Cost:** this changes the generated motion-function signatures, because the event buffer has to
reach `update_*`/`monitor_*` by some route other than `shared`. Not a tidy-up. Verify by running
the three maintained models, not by diffing text.

**Checkable when done:** every member of the generated `struct shared_data` corresponds to an
entry in `ir["shared_data"]`. Today four members and one method do not.

Separately, and larger: `mobile_base_runtime_state` adds 17 more members mixing configuration
(magic geometry), solver tuning and pure hddc2b scratch. The serial chain already does this
correctly — its scratch lives in `<solver>_solver_state` inside the motion's own state struct.
Fold this into §8's "a mobile base is a robot" rather than treating it separately.

## B. ~~Fold the plan-014 audit back into plan 014~~ — done (§10c, §10d)

`plans/014-frame-log-delta-encoding.md` §8/§8b opened an audit and left it open. The answers
are now recorded in the plan itself (2026-08-06); kept here for the architecture argument:

- **§8.1 "where does each gain live?"** — they *are* authored (all present in `ir.json`, 60
  `PIDControl` instances built from them), but baked into constructors rather than blackboard
  values, so they can neither vary nor be logged. §8's own conclusion holds: the contract needs
  no extension, the gains need to *be* values. → §10d
- **§8b "values that decide behaviour and are invisible"** — completed, and mostly **resolved**
  since. Of §8b's table: `kConstraintTolerance` gone (tolerances are modelled shared values),
  `kMonitorArmSteps` gone (authored `after active for <time>` debounce), `kRneDampingLambda`
  gone with the solver change, `kPathSearchForward/Backward/Samples/RefineSteps` gone (the
  projection window is derived, `7b9a291`). What remains invisible at HEAD: `kPathTangentStep`,
  the FT settle count, the MuJoCo solver triplet, and the `ImpedanceControl` dt default — §10c,
  including one live defect.

Delta encoding itself stays out of scope, and its verification and tests are untouched: it
changes the wire format, and mixing that into a structural refactor destroys the one check that
scales. Do it against a clean baseline afterwards.

## C. ~~Refresh the stale parts of this document~~ — done

Re-measured at `7b9a291` (2026-08-06); §3, §4, §5, §9 and §10 below now carry the measured
numbers. Two of the earlier refresh claims were themselves wrong and are corrected in place:
the IR is 19 keys / 5.26 MiB (not 17 / 2.7 — `cstr_hdl` is still published unread, and
`introspection` doubled with the IRI work), and reproducible generation is **not** achieved —
see item D.

## D. Generation is nondeterministic — again, or still

Two identical `motion-spec gen` runs at `7b9a291` differ in most generated files. Two distinct
sources, both implicit structural order (§5):

- **FSM state and event lists** come out of `ir_gen` in set-iteration order. Event indices are
  baked into the C++ (`_motion_spec_record_event(5)` vs `(1)` for the same model), so two
  builds of one model record differently-numbered events. `codegen._adopt_fsm_state_order`
  already exists to patch the *state* order back from `fsm_ir.json` — the event order it
  deliberately leaves alone is exactly the unstable one.
- **`shared_data` ordering** — the blackboard struct's member order changes run to run
  (layout-irrelevant, but it defeats any artifact diff and proves the traversal is unordered).

The earlier claim that `sorted()` on the schedule traversal fixed this was tested before the
FSM and IRI work landed, or tested the wrong artifact. §11 stands — reproducibility is not the
verification gate — but a build that differs run to run cannot be explained, and run artifacts
(event tables) are not comparable across two builds of the same model.

---

## Terminology

Two distinct things were previously conflated under the word "bus". They are not the same
structure and not even the same lifetime.

| term | what it is | where |
|---|---|---|
| **intermediate representation** (`ir.json`) | the **interface** between two stages of a Pipes-and-Filters pipeline [[9]](#r9) — a value handed from `ir_gen` to `codegen`. Nothing broadcasts; nothing subscribes. | generation time |
| **blackboard** (`shared_data`) | the shared structure that components genuinely read from and write to: solvers, closures and snapshots write; monitors, controllers, introspection and ROS read [[9]](#r9)[[10]](#r10) | run time |

There is **no bus** in this system. `ir.json` is not one, and calling it one invited exactly the
wrong intuition — that components subscribe to it. What it actually is: a *description of the
blackboard*, computed ahead of time. `annotate_dataflow.contract()` and the blackboard's
read/write contract are the same thing — producer, consumers, cadence.

That equivalence is why §2's duplicate derivation is a defect and not merely redundancy:
`wrench_outputs` answers a question about the blackboard from a weaker premise than the
contract that governs the blackboard itself.

## 0. The premise that decides everything else

StringTemplate is **logic-less by construction**, not by convention. Parr's separation rules
[[1]](#r1) permit a view to test the *presence or absence* of a value and nothing more: it may
not compute over data, may not compare data values, and may not assume types.

That single constraint decides the whole architecture. Any question of the form *"which values
are X?"* is a **query**, and a query cannot live in the view. It has to be formulated and
solved before rendering — which is the book's representation / query-formulation /
query-solving trinity [[2, §1.2.3]](#r2), and the classic model-to-text arrangement where a
prepared model is handed to a logic-free template [[3]](#r3).

So `wrench_outputs` existing as its own pre-computed collection **is correct**. The mistake was
never that it is separate. It is that nothing in the architecture said *what kind of thing it
is*, so each such query was invented ad hoc — its own name, its own derivation rule, its own
spot on a flat interface — and one of them was derived from the wrong premise.

## 1. Two layers in the IR: mechanism and policy

The book's primary separation is **mechanism** ("how something works, or what that something
does") versus **policy** ("how best to use a mechanism in a particular application context")
[[2, §1.1.9]](#r2). Applied here:

| layer | is | contains | may mention C++? |
|---|---|---|---|
| **A. model** — mechanism | what the robot program *means* | sectioned by the **5Cs** [[2, ch.10]](#r2): configuration, resources, computation, coordination, communication, composition | **no** |
| **B. resolved queries** — policy | which values play which **role** for this target | one collection per role, each a solved query over A — **structure only** | **no** |
| **C. view** — templates | text | rendering only; presence tests only | it *is* C++ |

### The boundary that keeps B honest

Layer B says *which values, in which role*. Layer C alone says *what text*. **`ir_gen` must
never build target code to ease the template's job** — that relocates the logic instead of
removing it, and it is exactly what finding 1 of the audit undid (`_shared_expr` returning
`"shared.<id>"`, `_escape_for_line_comment` producing `\n` for a C++ comment).

This is why the projections are named for the model role and not the construct:

| ✗ names a construct | ✓ names a role |
|---|---|
| `emit.io_pointers` | `values.externally_measured` |
| `emit.pose_compositions` | `poses.declared` |

`externally_measured` states a fact about the model — *the platform supplies this value, the
program does not compute it*. That an externally-measured Wrench becomes a pointer member plus
a measurement local is the **view's** decision, and stays in the view.

**The rule is checkable, and currently passes:** no string literal in `motion_spec/*.py`
contains target-language syntax (`KDL::`, `std::`, `->`, `shared.`, `#include`, `nullptr`, …).
That check belongs in the test suite as the layer-B guard.

This is the same split as MDA's platform-independent versus platform-specific model
[[4]](#r4), and as the *presentation model* prepared for a passive view [[5]](#r5). Layer B is
what the generator community calls the generator/transformation model that sits between the
domain model and the templates [[3]](#r3).

The project's own rule already says the model layer must stay implementation-agnostic
(`AGENTS.md`: *"Keep the RDF/DSL layer implementation-agnostic … never the downstream C++
library"*). Layer B is where that rule stops applying, and naming it is what lets the rule be
enforced rather than remembered.

## 2. One contract, N projections

Every projection in layer B derives from **one** answer to *"who writes this value?"* —
`annotate_dataflow.contract()`, which consults closures, solvers and sensors — and nothing
re-derives it. Single source of truth [[6]](#r6).

The current code violates this exactly once, and it is instructive:

| | rule | inputs |
|---|---|---|
| `contract()` | classify every value's producer | closures **+ solvers + sensors** |
| `wrench_outputs` (`ir.py:7853` at `7b9a291`) | `type == "Wrench" and id not in closure_output_map` | closures only |

Both answer the same question. On all three maintained models they agree — and they agree *by
luck, not by construction*: `ext_force` is `producer.kind == "sensor"`, which the second rule
never looks at. A Wrench written by a solver that is not a sensor would be classified
`kind: "solver"` by one and handed an external-measurement pointer by the other.

Under one contract, adding a construct adds a projection. It cannot add a second opinion.

## 3. The interface starts at the model, not above it

`ir.json` is the interface between `ir_gen` and codegen. L0 graph facts and L1 resolution
indexes are `ir_gen`'s **secrets** and must not cross it — Parnas's criterion for module
decomposition [[7]](#r7): the interface exposes what callers need, not how the module works.

Publishing internals is not a stylistic complaint here; it is measurable, and most of it has
been fixed: `data`, `pose_components`, the global `declared_pose_components`, `schedule`,
`shared_schedule` and the top-level `uris` duplicate are gone. At `7b9a291` the IR is **19 keys
/ 5.26 MiB**. One near-dead key survives: `cstr_hdl` (207 KiB, `ir.py:7848`), read by no
template and no motion-spec consumer — its sole reader is a motion-spec-dsl contract test
(`tests/test_solver_derivation_contract.py:123`), which iterates `handler.controllers` and
needs none of the other 200 KiB. The same rule — *publish a key only if something downstream reads it* —
now applies one level deeper: retiring `schema.json` (`3cb48f3`) silently orphaned the
per-record metadata fields that schema was the last reader of (§10a).

## 4. Layers are a DAG, enforced by ST4 itself

**Correction to an earlier draft of this document.** I previously recorded that ST4 `import`
declarations are inert because `stst -t <dir>` loads the directory as one flat namespace. That
was wrong, and the test behind it was invalid: it rendered a template with a degenerate payload
that never reached the cross-group calls, then compared line counts.

Re-tested with a full payload:

| rendered via | output | `no such template` errors |
|---|---|---|
| `main.introspect_model_header` (main imports all groups) | 1209 lines | **0** |
| `introspection.introspect_model_header` (declared no imports) | 1209 lines | **1078** |

Both emit 1209 lines because ST4 substitutes empty for an unresolved template and continues —
so line count proves nothing, and the earlier conclusion was drawn from exactly that.

So imports are real: a group sees its own rules plus its transitive imports. What was true is
weaker and more useful — **because everything renders through `main`, which imported all ten
groups, the per-group imports were unenforced in practice**, and 5 cross-group calls were
unsatisfied by their own group's declared imports.

That means the layering does not need a bespoke test. ST4 enforces it: a cyclic import makes
`stst` recurse until it stack-overflows, which is how the `backend_robot` ↔ backend cycle
surfaced during the restructure. The dispatch shim now imports no backend, because backend
leaves are reached by **dynamic** dispatch, resolved at render time rather than through imports.

Post-restructure: **0 unsatisfied cross-group calls, 0 import cycles** — re-verified at
`7b9a291` by walking every group's transitive import closure against every cross-group call.
One deviation from `proposed-templates.dot` remains: backends are not reached *only* by dynamic
dispatch. Four static imports survive — `entry_program` → `backend_kelo` + `backend_robot`,
`assembly_motion` → `backend_robif2b`, `entry_motion` → `backend_robif2b`. All point downward,
so the DAG holds; they are drift from the diagram, not a layering violation.

## 5. Two defects the book names directly

- **Magic number** [[2, §1.5.8]](#r2) — *"property values for which the cause of that value is
  not identified explicitly"*. The 750-step motion timeout was exactly this. The book's advice
  is to identify the relations that determine the value, or make the influence explicit. Here
  the honest answer was that no such relation exists: a motion with no `until` in a model with
  no FSM cannot be sequenced at all, so it is now rejected rather than capped.
- **Implicit structural order** [[2, §1.3.6]](#r2) — order that comes from lexical or
  iteration accident rather than from a modelled relation. Generation is nondeterministic:
  the schedule traversal was sorted at one point, but at `7b9a291` two identical runs still
  differ in FSM state/event numbering and blackboard member order — see item D. Schedule,
  state, event and member order must all come from a modelled (or at least stable) order, not
  from set iteration; fixing one traversal at a time is how the claim went stale.

## 6. What the view is allowed to do

Presence tests, iteration, and dispatch. Nothing else. Concretely, from probing `stst`:

- `<if(x)>` — only null/absent is falsy. An empty list, an empty string and `0` are **truthy**,
  so a guard over a collection does nothing. Optional subsystems therefore ride as one nested
  key each (`fsm`, `ros`, `mobile_base`), absent when the model has none.
- `&&`, `||`, `!` are *accepted* by ST4 but are computation over data values — forbidden by
  [[1]](#r1). If a template needs them, layer B is missing a projection.
- Multi-variant hooks dispatch through a dictionary with a `default`, so only variants that
  emit something need a rule.
- Dispatch passes **one context object**, not N positional arguments. 66 formal parameters are
  currently never referenced, because positional dispatch forces every variant to declare the
  union of what any sibling needs.

## 7. Not solved by this architecture

The per-motion payload re-serializes `views` + `closures` once per motion (1.4 MiB for a
10-motion model). `stst` reads one JSON file per render and JSON has no references, so the only
cure is a different render strategy. It is a cost, not a defect, and it is recorded here so it
is not rediscovered as one.

## 8. Sections follow the 5Cs, not "is it optional?"

The model is sectioned by the book's 5Cs [[2, ch.10]](#r2) — Computation, Communication,
Coordination, Configuration, Composition — plus the **resources** the program commands. This
is not imposed: the generated `ref_main` loop already conforms to the 4C loop template
[[2, §3.4.4]](#r2) — clock read (communicate) → `produce_event`/`fsm_dispatch` (coordinate) →
`step_<motion>` update/control/apply (compute) → `fsm_step_nbx`/`reconfig_event_buffers`
(coordinate + configure) → `sample_model` (communicate).

Two groupings in the current IR are wrong by this criterion:

**A mobile base is a robot, not an optional subsystem.** Today `serial_chain_solvers` sits at
top level while `mobile_base` sits beside `fsm` and `ros`. That groups by *"might be absent"* —
a property of the instance, not of the type, and the same error as the `has_*` flags. A KELO
base and a Kinova arm are both actuated resources with kinematics, solvers and devices. One
`resources.robots[]` collection with a `kind` field, and dual-arm and arm-on-base stop being
special cases.

**Coordination and ROS are different concerns.** The FSM sequences the behaviour — it is what
the program *is*. ROS publishing moves data out of the loop, which is Communication. A
consequence worth having: introspection and ROS become one concern, so the frame log, the shm
publisher and the ROS topics are one story rather than three.


## 9. What may live on the blackboard

**Rule: a blackboard member must have a producer/consumer contract.** If nothing in the model
writes it and nothing reads it, it is not a shared value — it is scratch, configuration, or
another concern borrowing the struct because the struct is reachable.

The IR side already obeys this. The contract is now one central table, `introspection.dataflow`
(670 entries at `7b9a291`), and every `shared_data` member has an entry — 606/606, each with
producer, cadence and storage. Producer kinds: controller 278, closure 93, view 91, authored 89,
`none` 64 (pose sub-components written by pose sampling), solver 19, snapshot 17, port 14,
pose 5. The 232 members added after parsing are role-tagged (`controller_internal_state` 204,
`joint_space` 28). Verified likewise on dual (1144/1144) and arc (333/333).

The **generated** struct does not. Four members and one method are hardcoded in
`entry_program.stg` with no IR entry:

| generated member | what it is | concern it belongs to |
|---|---|---|
| `_motion_spec_events[]`, `_motion_spec_event_count`, `_motion_spec_event_buffer_size` | the coordination event buffer | **Coordination**, not Computation |
| `_motion_spec_record_event()` | a **method on a blackboard** | behaviour, not data |
| `clock_time_s` | genuinely shared, read by elapsed monitors | Computation — but unmodelled, so no producer/cadence/storage, and not loggable like every other value |

A method is the sharpest violation: behaviour on the blackboard means the read/write contract
cannot be checked, because the callers never appear in any `consumers` list.

### The mobile base makes the same mistake at scale

With a base present, `mobile_base_runtime_state` adds 17 further members that are three
unrelated things at once:

- **configuration, as magic numbers** — `drive_attachment` (0.195, 0.21), `wheel_diameter`
  (0.115), `wheel_distance` (0.0775), `castor_offset` (0.01): robot geometry hardcoded in a
  template. Book §1.5.8 [[2]](#r2).
- **solver tuning** — `w_platform`, `w_drive`, `w_align`.
- **solver scratch** — `g`, `f_drive_ref`, `f_prim`, `f_scnd`, `f_wheel`, `xd_ground`,
  `xd_drive`: pure intermediates of the hddc2b solve.

The serial chain proves none of this is necessary. It keeps `q`, `qd`, `qdd`, `tau_ff`,
`f_ext`, `spatial_directions` in `<solver>_solver_state` **inside the motion's own state
struct** — verified: none of them appear in `shared_data`. The base puts the equivalent on the
blackboard only so its cycle functions can reach it.

This is §8's correction arriving from the other side. Once both are `resources.robots[]`, the
rule is forced rather than remembered:

> **solver scratch lives in solver state · geometry lives in configuration · coordination
> state lives in coordination · the blackboard holds only values with a contract.**

Checkable: every member of the generated `struct shared_data` must correspond to an entry in
`ir["shared_data"]`. Today four do not.

## 10. Complete audit — hardcoded values and information loss

### 10a. Unconsumed IR fields — 33, up from 9

Per record type at `7b9a291`, fields referenced by no template and no Python consumer
(token scan over every `.stg` expression and every non-`ir_gen` Python module):

| record | fields | unconsumed |
|---|---|---|
| `shared_data[]` (606 records) | 28 | 8 — `unit`, `quantity_kind`, `producer`, `with_respect_to`, `orientation_representation`, `euler_axes_sequence`, `euler_intrinsic`, `has_view` |
| `closures{}` (164) | 42 | **0** |
| `views{}` (76) | 7 | 1 — `subobject` |
| `introspection.quantities` (650) | 16 | 4 — `unit`, `quantity_kind`, `producer`, `reference_frame` |
| `introspection.*` (15 keys) | — | 1 — `contract_version` |
| `motions[]` (10) | 55 | 6 — `handler`, `has_elapsed`, `has_until_condition`, `until_evaluators`, `when_evaluators`, `while_evaluators` |
| `serial_chain_solvers[]` (10) | 28 | 5 — `chain_tip`, `motion_drivers`, `owned_trees`, `runtime_prefix`, `urdf` |
| `motions[].until_monitors[]` (4) | 29 | 8 — `debounce_duration_s`, `group_any`, `group_constraint_ids`, `is_until_aggregate`, `is_when_aggregate`, `ros_include`, `ros_pkg`, `ros_type` |

(`command_robot_id`, the field read nowhere at all, has since been removed.)

The count grew from 9 to 32 for two distinct reasons, and they need different fixes:

- **The old pattern, spread wider** — consumed *inside* `ir_gen` during construction, then
  published anyway (`debounce_duration_s` becomes `debounce_steps`, the `*_evaluators` lists
  become the schedule): the same Parnas violation as §3, one level deeper.
- **A new pattern: orphaned by schema retirement.** When the audit first ran, `schema.json` was
  the consumer of the per-record metadata (`unit`, `quantity_kind`, `producer`,
  `with_respect_to`, the orientation fields). Retiring it (`3cb48f3`) made the frame log
  self-describing by IRI — and left the copied-into-IR metadata with no reader. The metadata is
  not lost (the model graph in the archive still carries it); the IR copies are now dead weight.

**No information is lost between IR and templates.** The leak is the other way: internals
crossing the interface.

### 10b. Hardcoded values — most of the audit's list is resolved

Re-scanned at `7b9a291`: numeric literals in template *text* (outside every `<…>` expression),
SPDX years, comments and pure arithmetic excluded.

**Resolved since the first scan:**

| was | now |
|---|---|
| EtherCAT product code, device name, interface, slave indices (`backend_kelo`) | `robot.toml` deployment config (`8209898`, `704d617`) |
| the arm's home pose in a reset lambda (`backend_mj_kdl`) | modelled — `ir["agent_homes"]`, rendered per solver |
| gripper speed/force raw bytes (`backend_robif2b`) | named `kGripperSpeedByte`/`kGripperForceByte` constexprs (still literal `128`, but named and in one place) |
| FT bias samples `200` | config-overridable, `bias_samples` with a named default (`entry_build`) |
| `kMonitorArmSteps`, `kRneDampingLambda`, `kPathSearchForward/Backward/Samples/RefineSteps` | gone — authored debounce, solver change, derived projection window |

**Still hardcoded — physical facts that belong in the robot model or config:**

| value | file | what it is |
|---|---|---|
| `0.195, 0.21` ×4, `0.115`, `0.0775`, `0.01` | `entry_program` (moved from `backend_kelo` by the split) | KELO drive attachment geometry, wheel diameter, wheel distance, castor offset — §9's mobile-base finding, unchanged |
| `0.8` rad | `backend_robif2b` | 2F-85 travel (named template rule, commented as a device fact — acceptable as is) |

**Still hardcoded — control policy that decides behaviour:** MuJoCo's `iterations = 100`,
`tolerance = 1e-10`, `impratio = 20.0` (`backend_mj_kdl`); the FT settle count
`< 200` (`domain_solver:299`, commented "author it if startup transients vary"); and the
`dt = 0.002` constructor defaults in `runtime.stg` — see §10c for the live defect behind the
last one.

**Infrastructure** (defensible, but unnamed): ring capacity `8192`, shm mode `0666`, shm-name
truncation `16`, ROS QoS depth `10`.

Pure arithmetic (`0.5`, `2.0`, `M_PI`, `1e-9` guards, `kInvPhi`, LEB128's `128`s) is not a
finding.

### 10c. Runtime constants — down from eight to one unexplained, plus one live defect

Plan 014 §8b's finding, re-measured at `7b9a291`. Of the eight behaviour-deciding `k*`
constants, the generated runtime now declares two: `kControlPeriodS` (modelled — derives from
`control_period_ns`, and every log header carries `nominal_period_ns`) and `kPathTangentStep`
(`1e-4`, still in no artifact — but since `7b9a291` it is only the finite-difference seed of a
*derived* search window, no longer a tuned policy value). The other six are gone, not renamed
(§10b). What still decides behaviour from template text: the MuJoCo solver triplet and the FT
settle count (§10b).

`introspection.constants` carries 201 authored literals (the `cadence: init` values), so the
mechanism for recording such values exists — it just does not reach values that live in
template text rather than in the model.

**Zero tolerances are resolved.** `constraint_satisfied` takes its tolerance from modelled
shared values (`default_tolerance_Distance`/`_Angle`, `satisfied_band(_rot)`). The calls that
pass a literal `0.0` — 1 (single), 2 (dual), 9 (arc) — all wrap inequality or band evaluators
(`greater_than`, `less_than`, `outside`, `bilateral`) whose violation value is exactly `0.0`
when satisfied, so the zero is deliberate. Exact float equality on a converging axis — the
`S_PLACE`-stall shape — no longer occurs in any maintained model.

**New defect, same family:** the model's period is 1 ms and `PIDControl` instances receive
`kControlPeriodS` explicitly — but every `ImpedanceControl` instance is constructed with three
arguments (`{800.0, 80.0, 0.0}`), so it silently keeps the constructor default `dt = 0.002`
(`runtime.stg`). Every impedance controller integrates and differentiates at **twice the actual
control period**. `959bf63` fixed this for PID and missed impedance; the fix is to pass
`kControlPeriodS` there too — or better, delete the `dt` defaults so the omission cannot
compile.

### 10d. Gains are authored, but not loggable

Plan 014 §8 asked where gains live. Answer (unchanged at `7b9a291`): they *are* in the model —
`200.0, 100.0, 40.0` and the rest all appear in `ir.json`, and 60 `PIDControl` plus the
`ImpedanceControl` instances are constructed from them. But they are baked into constructors,
not shared values, so they cannot vary and cannot be logged. §8's own conclusion holds: the
contract needs no extension, the gains simply need to *be* values on the blackboard. (The
impedance `dt` defect in §10c is one more argument: a constructor argument is invisible
precisely when it is wrong.)

## 11. How this is verified

**Not by byte-diffing generated C++.** `AGENTS.md` says it plainly: *"Verify behaviour, not
generated text — a rename or IR restructuring legitimately moves the C++, so comparing it
against a pre-change baseline fights the change instead of checking it."* Chasing byte-identity
pushes you either to contort an implementation to preserve output, or to skip a correct change
because it moves text. Every remaining fix in this document — evicting non-contract members from
the blackboard, moving hardcoded values into models — legitimately changes generated code.

The gate is:

1. `motion-spec check` on each maintained model's `-app.ld.json` → Conforms
2. `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest`
3. `motion-spec run` on `pick_place_single`, `admittance_arc_single`, `pick_place_dual` → each
   reaches `s_done`, with no step or time bound

Reproducible generation (§5) is worth having on its own merits — a build that differs run to run
cannot be explained — but it is not a verification prerequisite.


---

## References

<a id="r1"></a>[1] T. J. Parr. *Enforcing Strict Model–View Separation in Template Engines.*
WWW '04. [doi:10.1145/988672.988703](https://doi.org/10.1145/988672.988703)

<a id="r2"></a>[2] H. Bruyninckx et al. *Composable and Explainable Systems of Systems.*
`src/composable-and-explainable-systems-of-systems.pdf` — §1.1.9 mechanism/policy,
§1.2.3 representation–query–solver, §1.3.2 DAG for partial order, §1.3.6 implicit structural
order, §1.5.8 magic number.

<a id="r3"></a>[3] T. Stahl and M. Völter. *Model-Driven Software Development: Technology,
Engineering, Management.* Wiley, 2006. ISBN 978-0-470-02570-3.

<a id="r4"></a>[4] Object Management Group. *MDA Guide rev. 2.0*, 2014, OMG document
ormsc/2014-06-01 — platform-independent vs platform-specific model.

<a id="r5"></a>[5] M. Fowler. *Presentation Model*, 2004; and *Patterns of Enterprise
Application Architecture*, Addison-Wesley, 2002. ISBN 978-0-321-12742-6.

<a id="r6"></a>[6] A. Hunt and D. Thomas. *The Pragmatic Programmer.* Addison-Wesley, 1999.
ISBN 978-0-201-61622-4 — DRY: every piece of knowledge has one authoritative representation.

<a id="r7"></a>[7] D. L. Parnas. *On the Criteria To Be Used in Decomposing Systems into
Modules.* CACM 15(12), 1972. [doi:10.1145/361598.361623](https://doi.org/10.1145/361598.361623)

<a id="r8"></a>[8] R. C. Martin. *Agile Software Development: Principles, Patterns, and
Practices.* Prentice Hall, 2002. ISBN 978-0-13-597444-5 — Acyclic Dependencies Principle.

<a id="r9"></a>[9] F. Buschmann, R. Meunier, H. Rohnert, P. Sommerlad, M. Stal.
*Pattern-Oriented Software Architecture, Volume 1: A System of Patterns.* Wiley, 1996.
ISBN 978-0-471-95869-7 — Pipes and Filters; Blackboard; Layers.

<a id="r10"></a>[10] B. Hayes-Roth. *A blackboard architecture for control.* Artificial
Intelligence 26(3), 1985. [doi:10.1016/0004-3702(85)90063-3](https://doi.org/10.1016/0004-3702(85)90063-3)
