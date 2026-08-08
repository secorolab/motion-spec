<!-- SPDX-License-Identifier: MPL-2.0 -->
<!-- SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de) -->
# motion-spec codegen architecture

How a model becomes C++: what each stage does, what each module owns, and what the two
interfaces between the stages contain. Read this, then read `rdf_parser/ir.py` top to bottom —
it *is* the pipeline, in order, and nothing else in the package is reachable except through it.

Diagrams: `proposed-ir`, `proposed-templates`, `proposed-runtime` (`.dot` + `.png`).
The sim/real fork surface and the external-wrench law are a separate page:
[../sphinx/source/sim-real-parity.rst](../sphinx/source/sim-real-parity.rst).

---

## 1. The pipeline

`ir.py` is one forward pass. Reading it top to bottom is reading the data flow:

```text
<model>-app.ld.json                          the manifest: the app graph plus its imports
  model.load_model                           one merged graph, the identity rules, the read cache
  operations.normalize                       materialize the operations the model implies
  operations.build_closures / Schedule       the computations and the order they run in
  quantities.read_*                          poses, views, snapshots — the values that exist
  controllers.*                              authored controllers → per-axis control laws
  resources.*                                the scene, the robots, the platform they run on
  coordination.*                             handlers, motions, monitors, the FSM
  quantities.annotate_dataflow               the blackboard contract
  communication.build_introspection          the frame log, the model samples, the run graph
  → the published IR, sectioned by the 5Cs
generated/model/ir.json
  → generation/codegen.py + templates/*.stg
generated/controller/*.cpp|hpp
```

Four properties hold across the whole pass. They are why it can be one pass.

- **Resolve, then emit.** Every graph question is answered before records are assembled, into
  Python indexes. Nothing reads the graph back once emission has begun, and idempotency is
  tracked with Python `set`/`dict`, never with graph-membership checks.
- **The graph has exactly one writer.** `operations.normalize` runs before anything reads. After
  it the graph is immutable, which is what makes one shared read cache correct.
- **No stage re-derives what an earlier stage decided.** Where a later stage needs an earlier
  answer it receives it as an argument or reads it off the record.
- **Stable order everywhere.** Every traversal that decides emitted order is `sorted()` or
  follows a modelled order (`app:order`). Set iteration never reaches an output list — rdflib
  node order varies between processes, and generation must not.

## 2. The module set

Eight modules. Seven own a concern; one is the graph the other seven read through. There is no
parser module, no orchestrator façade and no helper module: graph readers live **with the concern
that consumes what they read**, the entry point is the pipeline itself, and a helper with one
consumer lives next to it.

Nothing imports a name that starts with `_` from a sibling: the underscore convention **is** the
public-surface contract, enforced by `tests/test_layer_boundary.py`, and everything private is a
module's own business (Parnas, *On the Criteria To Be Used in Decomposing Systems into Modules*,
CACM 15(12), 1972). There is no `__all__` anywhere in the package: it duplicated the same rule
and drifted silently.

| module | ~lines | 5C | why it is not folded into a sibling |
|---|---|---|---|
| `model.py` | 300 | — | every other module reads the graph through it; folding it into one concern would make the other six depend on that concern |
| `operations.py` | 800 | computation | it never asks what a value *is*; folding it into `quantities.py` would put the scheduler inside the geometry readers |
| `quantities.py` | 1320 | computation | it never asks in what order a value is written; it is the only module that knows what is on the blackboard |
| `constraint_handler.py` | 910 | computation | the declared seam for a controller DSL: its inputs and outputs are frozen so that a future DSL replaces this module and nothing else |
| `resources.py` | 1040 | resources · composition · configuration | the only module that reads the scene and agent graphs; the platform, the trace and the homes are read alongside the resources they configure |
| `coordination.py` | 860 | coordination | the only module that decides what runs when; nothing else may sequence |
| `communication.py` | 495 | communication | the only module that decides what leaves the loop; folding it into `quantities.py` would make the blackboard responsible for its own reporting |
| `ir.py` | 135 | — | it is the pipeline; it holds no derivation of its own |

### Three rules that hold in every module

**A record that a derivation mutates is a typed entity.** Every such record is a dataclass in
one of [`classes/`](../../src/motion_spec/classes/)'s domain modules, whose shared
[`base`](../../src/motion_spec/classes/base.py) carries the
`INTERNAL` convention that keeps a construction input out of the published document — entities,
data structures, views, motions, monitors, evaluators, controllers, robots, solvers, scene
records, constraint handlers, and every member of `shared_data` including the runtime values.
A published payload that nothing mutates after it is built may stay a plain dict: the `platform`,
the trace settings, the agent homes, the `by_kind` views. Two mutated structures stay dicts
because a fixed set of fields cannot describe them — a **closure**, whose keys are operand names
taken from RDF predicates, and an **introspection row**, whose shape is the artifact's. Both are
read and written as dicts.

There is therefore no dict-or-dataclass access helper. Attributes are read as attributes and keys
as keys, because at every site the shape is known. A site that genuinely receives both is a
design error, not a reason to reintroduce one.

**Each concern validates what it reads, at the moment it reads it.** There is no validator module
and no `check_*` entry point: a scene object on real hardware is rejected inside `read_scene`, a
solver the backend cannot run inside `build_robots`, an unresolvable id inside
`build_introspection`. Fail fast, in the one place that still holds the context needed for the
message. A module never validates another module's records.

**A concern exposes what it knows as data; only `communication.py` turns data into rows.** No
module outside it appends to the introspection artifact or to the frame log. When a concern has
something to contribute — the internal state a PID keeps, the gains a controller carries, the
joint-space channels a serial chain mirrors — it publishes that as a table, and
`communication.py` reads the table and builds every member and every row itself.

Each module's docstring lists its top-level sections, in order. A load-bearing invariant gets a
named section rather than a comment in the middle of a long file.

### `model.py` — the loaded model

`load_model(manifest_path) -> Model` resolves the manifest's imports, installs the IRI-to-file
resolver and merges one graph. `Model` then carries everything every reader needs:

- the graph, the app-model path, and the imported model/provenance sources;
- **identity** — `id(node)` is the single id-minting rule, `label(node)`, and
  `assert_no_id_collisions()`, which rejects two distinct IRIs collapsing onto one generated id;
- **derived IRIs** — `register_derived(id_, parent_iri, suffix, relation)` mints
  `<parent>/<suffix>` for an entity the model implies but does not author, so every published id
  resolves to something a run graph can make a statement about. `uri_rows()` and
  `derivation_nodes()` publish the table;
- **one read cache**, and the `@reader` decorator that uses it. A reader is a free function
  `f(model, node)`; the decorator memoizes it on the model, keyed by the function and the node.

Also here because they are identity and unit questions, not concerns of their own:
`identifier`, `kebab`, `local_name`, and the QUDT→SI conversions `si`, `si_all`, `seconds`,
`length_unit`, `si_unit`.

### `operations.py` — what is computed, and in what order

Three sections, in this order:

1. `normalize(model)` — the operations the model implies but does not spell out: a distance
   between two points, a pose reference expressed in another frame. Materialized as triples
   **before** anything reads, so one graph serves the whole pass.
2. the operator tables — `OPS_GENERIC`, `OPS_SOLVER`, `OPS_HANDLER` — and `build_closures`, which
   turns each call of an operator into the closure the templates render.
3. `Schedule` — dependency-ordered call ids, emitted at most once per instance. A new instance
   is a new scope: the `when` block and the active block each need one, because a step evaluated
   in both phases must be emitted in both.

Public: `normalize`, `OPS_GENERIC`, `OPS_SOLVER`, `OPS_HANDLER`, `ErrorEvaluator`,
`AssignmentEvaluator`, `Schedule`, `build_closures`, `resolve_closure_operands`, `closure_maps`,
`closure_owner_map`, `data_reference_map`, `path_projections_for_motion`,
`continuous_joint_leaves`.

### `quantities.py` — what values exist, and who writes them

The model's data layer. Four sections, named as such in the file and listed in its docstring:

1. **readers** — every quantity and geometry node the DSL emits: positions, orientations, poses,
   twists, wrenches, durations, joint positions, views, data structures;
2. **derived values** — pose components, snapshots, scene-relative poses, pose-axis error groups;
3. **the blackboard** — which values reach `shared_data`;
4. **the dataflow contract** — the architecture's central invariant, and the reason it is a named
   section rather than a trailing clause of a long file.

`annotate_dataflow` is the single answer to *"who writes this value?"* — it consults closures,
solvers and sensors, and every projection the templates ask for is read off that one
classification. **One contract, N projections:** adding a construct adds a projection, and it
cannot add a second opinion. Any other module answering the same question from a weaker premise
is a defect, not redundancy.

Public, in three groups:

- readers — `quantity`, `pose`, `position`, `orientation`, `velocity_twist`,
  `acceleration_twist`, `wrench`, `joint_position`, `frame`, `position_values`,
  `orientation_quaternion`, `read_views`, `read_data_structures`;
- per-motion queries — `relative_poses_for_motion`, `scene_relative_poses_for_motion`,
  `snapshots_for_motion`, `pose_axis_error_groups_for_motion`, `declared_pose_component_entries`,
  `collect_motion_references`, `elapsed_coordinate_ids`, `expanded_constraints`;
- blackboard — `build_indexes`, `ComputationIndexes`, `filter_shared_data`, `views_for_access`,
  `views_by_subobject`, `annotate_dataflow`, `PORT_PRODUCERS`.

### `constraint_handler.py` — the control law

Everything that turns an authored controller into derived per-axis controller records: the axis
decision table, the derived signal/error/energy ids and the IRIs they register, saturations,
motion drivers, the closure and data augmentation those imply, and the invariants that only hold
once authored RDF has resolved to solver plans. It also exports, as tables, what internal state and
which gains a controller type carries; `communication.py` reads those tables and builds the rows,
so nothing here appends to the frame log.

A colleague builds a controller DSL later. This module is where it lands: its inputs are
`(Model, closures, data structures)` and its outputs are the `SolverDerivationContext` and the
controller/driver records. No other module derives a controller.

Public: `SpatialAxis`, `LINEAR_AXES`, `ANGULAR_AXES`, `POSE_AXES`, `spatial_axes`,
`SolverIdFactory`, `ControllerDerivation`, `SolverDerivationContext`, `solver_derivation_context`,
`motion_drivers`, `augment_closures`, `augment_data`, `annotate_controller_signals`,
`CONTROLLER_STATE_FIELDS`, `CONTROLLER_GAIN_FIELDS`, `ADMITTANCE_PARAMETERS`,
`CONTROLLER_SIGNAL_ROLES`, `SOLVER_SEMANTICS_BY_ALGORITHM`.

### `resources.py` — what the program commands

Three chapters, in C order, named as such in the file and listed in its docstring:

1. **resources** — the agents: assemblies, devices, sensors, config keys, robot setups, and the
   solver records built on them;
2. **composition** — the scene: bodies, frames, kinematic adjacency, body paths, fixed
   attachments, the `SceneSpec`;
3. **configuration** — the execution platform, the backend it implies, the trace settings and the
   agent home positions.

Configuration rides here rather than in a module of its own because nothing derives it: it is
read alongside the resources it configures. Each chapter validates what it reads — a scene object
on real hardware, an undriven device, an unbound sensor, a solver the backend cannot run.

Public: `Robots`, `read_scene`, `kinematic_adjacency`, `body_path`, `fixed_attachments`,
`mapped_targets`, `robot_setups`, `read_platform`, `build_robots`, `annotate_runtime`,
`shared_runtime_members`, `agent_home_positions`, `JOINT_SPACE_CHANNELS`,
`JOINT_SPACE_COMMAND_CHANNEL`, `TRACE_DISABLED`.

### `coordination.py` — what runs when

The constraint handlers, and the per-motion unit: which constraints belong to `when` / `while` /
`until`, the three schedules in the order the loop runs them, the monitors and the conditions
they evaluate, the FSM framed from its named graph, and the wiring that gives each monitor its
event, its debounce duration and its gate call.

Public: `build_constraint_handlers`, `build_motions`, `read_fsm`, `evaluator_term`.

### `communication.py` — what leaves the loop

The introspection artifact — uris, motion/controller/monitor/quantity/signal rows, provenance —
the frame-log samples built from the blackboard, and the ROS publishers collected off the
monitors. It reads the tables each concern exports — the internal state a PID keeps, the gains a
controller carries, the joint-space channels a chain mirrors — and builds every member and every
row itself. It also checks that every published id resolves to an IRI: one that does not is a
derived entity minted without registering where it came from.

Public: `build_introspection`, `ros_publishers`.

### `ir.py` — the pipeline

`generate_ir(manifest_path)` calls the stages above in order and assembles the six sections. It
holds no derivation. Public: `generate_ir` — a caller that wants the graph itself asks
`model.load_model`, not a re-export from here.

## 3. The two interfaces

### `ir.json` — the IR

`ir.json` is the interface between `ir_gen` and codegen: a value handed from one stage of a
pipes-and-filters pipeline to the next. Nothing broadcasts and nothing subscribes; it is a
*description of the blackboard*, computed ahead of time.

Everything above it — the graph, the `Model`, the resolution indexes — is `rdf_parser`'s secret
and never crosses. **Publish a key only if something downstream reads it.**

The IR is sectioned by the 5Cs plus the resources the program commands. This is not imposed: the
generated `main.cpp` loop already conforms to the 4C loop — clock read (communicate) →
`fsm_dispatch` (coordinate) → `step_<motion>` (compute) → `fsm_step_nbx` (coordinate + configure)
→ `sample_model` (communicate).

| section | keys | what belongs here |
|---|---|---|
| `configuration` | `control_period_ns`, `backend`, `platform`, `agent_homes`, `trace` | fixed for the whole run; nothing computes it |
| `resources` | `robots`, `by_kind` | every actuated thing the program commands. A wheeled base and an arm are both robots with a `kind`; neither is an optional subsystem |
| `composition` | `scene` | how bodies are placed and attached into one world |
| `computation` | `shared_data`, `values`, `closures`, `views`, `clock` | what is computed each tick, and the blackboard it lives on |
| `coordination` | `motions`, `fsm` | what runs when |
| `communication` | `introspection`, `ros` | what leaves the loop: the frame log, the shm publisher, the ROS topics — one story, not three |

Two rules keep the table from drifting:

- **A projection lives in the section of what it projects.** `resources.by_kind` and
  `computation.values.externally_measured` are solved queries, not new concerns, so they sit
  inside the section whose data they re-cut.
- **Anything optional is absent, never empty, and never flagged.** ST4 treats an empty list as
  truthy, so `<if(x)>` can only distinguish *absent* from *present* — but that is enough for
  every case, because a key emitted only when it has content is absent exactly when a flag would
  have been false. `coordination.fsm`, `communication.ros`, `resources.by_kind.mobile_base`,
  `resources.by_kind.serial_chain` and `computation.clock` all follow this, and no `has_*` or
  `needs_*` key exists. An absent key iterates to nothing, so a template may guard on it and
  iterate it at the same site.

The per-motion payload codegen writes for `motion_header` is **not** the IR — it is codegen's own
projection (`motion`, `closures`, `views`, `values`, `backend`) and stays flat.

### `shared_data` — the blackboard

**A blackboard member must have a producer/consumer contract.** If nothing in the model writes it
and nothing reads it, it is not a shared value — it is scratch, configuration, or another concern
borrowing the struct because the struct is reachable.

Storage follows from cadence in exactly one place: `never → absent`, `init → record`,
`tick → log`. Consequences that are rules, not preferences: solver scratch lives in solver state,
geometry lives in configuration, coordination state lives in coordination, and the blackboard
holds only values with a contract.

**The rule holds today and must keep holding: every member of the generated `struct shared_data`
corresponds to an entry in `computation.shared_data`, one for one.** The coordination event
buffer is its own `struct motion_spec_event_buffer`, passed explicitly to the step and monitor
functions rather than riding the blackboard, and `clock_time_s` is a modelled value with a `port`
producer. A member that appears in the generated struct without an IR entry is a regression.

## 4. The layer boundary

| layer | is | may mention C++? |
|---|---|---|
| **A. model** | what the robot program means — the 5C sections above | no |
| **B. resolved queries** | which values play which role for this target: one collection per role, each a solved query over A, structure only | no |
| **C. view** — `templates/*.stg` | text | it *is* C++ |

StringTemplate is logic-less by construction (Parr, *Enforcing Strict Model–View Separation in
Template Engines*, WWW '04): a view may test the *presence* of a value and nothing more. So any
question of the form *"which values are X?"* is a query, and a query cannot live in the view — it
is formulated and solved in layer B, before rendering.

**`rdf_parser` must never build target code to ease a template's job.** That relocates the logic
instead of removing it. Projections are named for the model role, not the construct:
`values.externally_measured`, not `emit.io_pointers` — the first states a fact about the model
(the platform supplies this value, the program does not compute it); that it becomes a pointer
member is the view's decision and stays in the view.

The rule is a test, not a convention. `tests/test_layer_boundary.py` asserts two things over
every module in `rdf_parser/`:

1. no non-docstring string literal contains target syntax (`KDL::`, `std::`, `->`, `shared.`,
   `#include`, `nullptr`);
2. no module imports a name beginning with `_` from a sibling module.

What the view is allowed to do: presence tests, iteration, and dispatch. `&&`/`||`/`!` are
accepted by ST4 but are computation over data values — if a template needs one, layer B is
missing a projection. Multi-variant hooks dispatch through a dictionary with a `default`, and
dispatch passes **one context object**, never N positional arguments.

Template layering needs no bespoke test: ST4 enforces it. A group sees its own rules plus its
transitive imports, and a cyclic import makes `stst` recurse until it overflows. Backend leaves
are reached by dynamic dispatch, resolved at render time.

## 5. How this is verified

**Not by byte-diffing generated C++.** A rename or an IR restructuring legitimately moves the
C++, so comparing it against a pre-change baseline fights the change instead of checking it.

The gate:

1. `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest` in `motion-spec` and `motion-spec-dsl`, from the
   workspace `.venv`;
2. `motion-spec check <generation>/generated/model/<name>-app.ld.json` → `Conforms: True`;
3. `motion-spec run` on `pick_place_single`, `pick_place_dual` and `admittance_arc_single` →
   each reaches `s_done`, with no step or time bound.

A truncated run proves only that the model started. Reproducible generation is worth having on
its own merits — a build that differs run to run cannot be explained — but it is not a
prerequisite of the gate.

Tests follow the repo rule: minimum count, none for trivial code, and none that pins incidental
structure. A test earns its place by proving a behaviour that would otherwise fail silently —
the dataflow contract, the layer boundary, id resolution, the solver-derivation shape.
