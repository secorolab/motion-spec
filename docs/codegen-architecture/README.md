<!-- SPDX-License-Identifier: MPL-2.0 -->
# motion-spec codegen architecture

Why the current codegen is shaped the way it is, what is wrong with it, and the architecture
that replaces it. Every rule below is grounded in literature so the shape is settled once
rather than re-litigated per refactor.

Diagrams: `current-templates`, `current-ir`, `proposed-templates`, `proposed-ir`,
`proposed-runtime` (`.dot` + `.png`). Measurements are from `pick_place_single` unless stated.

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

All five are hardcoded in `entry_program.stg`'s `shared_state_header` with no IR entry. The rule
they violate — *a blackboard member must have a producer/consumer contract* — is already obeyed
on the IR side: 602 members, **zero** without one.

**Cost:** this changes the generated motion-function signatures, because the event buffer has to
reach `update_*`/`monitor_*` by some route other than `shared`. Not a tidy-up. Verify by running
the three maintained models, not by diffing text.

**Checkable when done:** every member of the generated `struct shared_data` corresponds to an
entry in `ir["shared_data"]`. Today four do not.

Separately, and larger: `mobile_base_runtime_state` adds 17 more members mixing configuration
(magic geometry), solver tuning and pure hddc2b scratch. The serial chain already does this
correctly — its scratch lives in `<solver>_solver_state` inside the motion's own state struct.
Fold this into §8's "a mobile base is a robot" rather than treating it separately.

## B. Fold the plan-014 audit back into plan 014 — §10c, §10d

`plans/014-frame-log-delta-encoding.md` §8/§8b opened an audit and left it open. It is now
answered, but the answers live here rather than in the plan:

- **§8.1 "where does each gain live?"** — they *are* authored (all present in `ir.json`, 60
  `PIDControl` instances built from them), but baked into constructors rather than blackboard
  values, so they can neither vary nor be logged. §8's own conclusion holds: the contract needs
  no extension, the gains need to *be* values. → §10d
- **§8b "values that decide behaviour and are invisible"** — completed. Of the 8 runtime
  constants, **7 appear in no run artifact by name**; only `kControlPeriodS` is explicable,
  deriving from the modelled `control_period_ns`. → §10c
- **`kConstraintTolerance`** — §8b's worst finding is **gone**; `constraint_satisfied` now takes
  an explicit tolerance from the model. §8b should be marked resolved.
- **`kMonitorArmSteps`** — also gone, replaced by authored `after active for <time>` debounce.

Delta encoding itself stays out of scope, and its verification and tests are untouched: it
changes the wire format, and mixing that into a structural refactor destroys the one check that
scales. Do it against a clean baseline afterwards.

## C. Refresh the stale parts of this document

Written before the work landed, so several sections describe a tree that no longer exists:

| section | says | actually |
|---|---|---|
| §3 | 6 unread keys + the `uris` duplicate | removed; IR is 17 keys / 2.7 MiB |
| §4 | 3 call-graph cycles, 5 unsatisfied cross-group calls | both **0** after the restructure |
| §5 | nondeterminism "should be fixed first" | fixed — `sorted()` on the schedule traversal |
| §10b, §10c | "31 of 122 exact-equality tolerances" | **wrong.** 9 of 10 zero-tolerance verdicts are inequalities, where `0.0` is deliberate. The real count was 1 (2 in dual), now 0 |

§9's findings are still accurate — that work is item A above.

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
| `wrench_outputs` (`ir.py:7704`) | `type == "Wrench" and id not in closure_output_map` | closures only |

Both answer the same question. On all three maintained models they agree — and they agree *by
luck, not by construction*: `ext_force` is `producer.kind == "sensor"`, which the second rule
never looks at. A Wrench written by a solver that is not a sensor would be classified
`kind: "solver"` by one and handed an external-measurement pointer by the other.

Under one contract, adding a construct adds a projection. It cannot add a second opinion.

## 3. The interface starts at the model, not above it

`ir.json` is the interface between `ir_gen` and codegen. L0 graph facts and L1 resolution
indexes are `ir_gen`'s **secrets** and must not cross it — Parnas's criterion for module
decomposition [[7]](#r7): the interface exposes what callers need, not how the module works.

Publishing internals is not a stylistic complaint here; it is measurable. Six top-level keys
are read by no template and no Python consumer — `cstr_hdl` (203 KiB), `data` (171 KiB),
`pose_components`, the global `declared_pose_components`, `schedule`, `shared_schedule` — and
every one is an L0/L1 internal. A seventh, `uris` (1054 KiB), is an L4 field copied to the
top, where its only reader prefers the nested original (`artifacts.py:97`). One rule —
*publish a key only if something downstream reads it* — removes 1.6 MiB of a 4.2 MiB IR and
prevents the eighth.

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

Post-restructure: **0 unsatisfied cross-group calls, 0 import cycles.**

## 5. Two defects the book names directly

- **Magic number** [[2, §1.5.8]](#r2) — *"property values for which the cause of that value is
  not identified explicitly"*. The 750-step motion timeout was exactly this. The book's advice
  is to identify the relations that determine the value, or make the influence explicit. Here
  the honest answer was that no such relation exists: a motion with no `until` in a model with
  no FSM cannot be sequenced at all, so it is now rejected rather than capped.
- **Implicit structural order** [[2, §1.3.6]](#r2) — order that comes from lexical or
  iteration accident rather than from a modelled relation. Generation is currently
  nondeterministic: three runs of identical code on `admittance_arc_single` produced two
  different `motion_arc_motion.hpp` files, with `end_*_add` statements moving relative to
  their neighbours. Schedule order must come from a modelled partial order, not from set
  iteration. **Fixed** — see below.

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

The IR side already obeys this. All 602 members of `shared_data` carry a dataflow producer
kind (controller 278, closure 93, view 91, authored 85, solver 19, snapshot 17, port 14,
pose 5) — **zero without one** — and the 232 added after parsing are role-tagged
(`controller_internal_state` 204, `joint_space` 28).

The **generated** struct does not. Four members and one method are hardcoded in `main.stg`
with no IR entry:

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

### 10a. Every IR field is consumed, bar nine

Per record type, fields referenced by no template and no Python:

| record | fields | unconsumed |
|---|---|---|
| `shared_data[]` (602 records) | 28 | **0** |
| `closures{}` (164) | 42 | **0** |
| `views{}` (76) | 7 | **0** |
| `introspection.quantities` (646) | 16 | **0** |
| `introspection.*` (15 keys) | — | **0** |
| `motions[]` (10) | 56 | 3 — `command_robot_id`, `has_elapsed`, `while_evaluators` |
| `serial_chain_solvers[]` (10) | 28 | 3 — `motion_drivers`, `owned_trees`, `runtime_prefix` |
| `motions[].until_monitors[]` (4) | 29 | 3 — `ros_include`, `ros_pkg`, `ros_type` |

Eight of the nine are consumed *inside* `ir_gen` during construction and then published anyway
— the same Parnas violation as §3's six dead top-level keys, one level deeper.
`command_robot_id` is written at `ir.py:3923` and read nowhere at all.

**No information is lost between IR and templates.** The leak is the other way: internals
crossing the interface.

### 10b. Hardcoded values — 101 literals, three categories

Numeric literals in template *text* (outside every `<…>` expression):

**Physical facts that belong in the robot model or config:**

| value | file | what it is |
|---|---|---|
| `0.195, 0.21` ×4 | `backend_kelo` | KELO drive attachment geometry |
| `0.115`, `0.0775`, `0.01` | `backend_kelo` | wheel diameter, wheel distance, castor offset |
| `10.0` A, `0.29` Nm/A | `backend_kelo` | drive current limit, torque constant |
| `0x02001001`, `"KELOD105"`, `"net0"`, slave idx `3,4,6,7` | `backend_kelo` | EtherCAT product code, device name, interface |
| `{0.0, 0.2618, 3.1416, -2.2689, 0.0, 0.9599, 1.5708}` | `backend_mj_kdl` | **the arm's home pose**, hardcoded in a reset lambda |
| `0.8` rad | `backend_robif2b` | 2F-85 travel (commented as a device fact) |
| `128`, `100.0`, `255.0`, `200` | `backend_robif2b` | gripper speed/force defaults, unit scaling, FT bias samples |

**Control policy that decides behaviour:** `kMonitorArmSteps = 100`, `kRneDampingLambda = 0.05`,
`kPathSearchForward/Backward/Samples/RefineSteps/TangentStep`, and MuJoCo's
`iterations = 100`, `tolerance = 1e-10`, `impratio = 20.0`.

**Infrastructure** (defensible, but unnamed): ring capacity `8192`, shm mode `0666`, shm-name
truncation `16`, `-ftemplate-depth=2048`.

Pure arithmetic (`0.5`, `2.0`, `M_PI`, `1e-9` guards, `kInvPhi`) is not a finding.

### 10c. Seven of eight runtime constants appear in no artifact

Completing plan 014 §8b. Of the eight `k*` constants the generated runtime declares, only
`kControlPeriodS` is explicable from a run's own artifacts — it derives from the modelled
`control_period_ns`. The other seven are named in **zero** files under `model/`, `contract/`
or `provenance/`. A run cannot explain why a monitor armed when it did, or why the path search
converged where it did.

`introspection.constants` already carries 197 authored literals (the `cadence: init` values),
so the mechanism exists — it just does not reach values that live in template text rather than
in the model.

**Good news since plan 014 was written:** `kConstraintTolerance` is gone. `constraint_satisfied`
now takes an explicit tolerance fed from the model via `band(tolerance_id)`. But the default
moved rather than vanished — **31 of 122 generated calls (25%) pass `0.0`**, i.e. exact float
equality, for constraints that authored no tolerance. That is the same shape of implicit
assumption, and it has bitten before (the `S_PLACE` stall was a too-tight equality gate).

### 10d. Gains are authored, but not loggable

Plan 014 §8 asked where gains live. Answer: they *are* in the model — `200.0, 100.0, 40.0` and
the rest all appear in `ir.json`, and 60 `PIDControl` instances are constructed from them. But
they are baked into constructors, not shared values, so they cannot vary and cannot be logged.
§8's own conclusion holds: the contract needs no extension, the gains simply need to *be*
values on the blackboard.

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
