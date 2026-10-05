<!-- SPDX-License-Identifier: MPL-2.0 -->
<!-- SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de) -->
# motion-spec codegen architecture

How a model becomes C++: what each stage does, what each module owns, and what the two
interfaces between the stages contain. Read this, then read `rdf_parser/ir.py` top to bottom —
it *is* the pipeline, in order, and nothing else in the package is reachable except through it.

The sim/real fork surface and the external-wrench law are a separate page:
[sim-real parity](sim-real-parity.rst).

## 1. The pipeline

```{graphviz} _static/pipeline.dot
:caption: Commands, ownership, and the reusable generation boundary.
```

`ir.py` is one forward pass. Reading it top to bottom is reading the data flow:

```text
<model>.robmot
  motion_spec_dsl.gens.generate              writes the JSON-LD; returns the dataset, one graph per document
  model.Model                                that dataset, the identity rules, the read cache
  operations.normalize                       materialize the operations the model implies
  operations.build_functions / Schedule      the F-blocks and the S-blocks that order them
  quantities.read_*                          poses, views, snapshots — the D-blocks that exist
  controllers.*                              authored controllers → per-axis control laws
  resources.*                                the scene, the robots, the platform they run on
  coordination.*                             handlers, motions, monitors, the FSM
  data_access.build_data_access              which F-block writes and reads each D-block
  data_access.analyse_data_access            when each is written, and the storage that follows
  communication.build_telemetry              the logging stream that taps the algorithm data
  provenance                                 every id's IRI, and what each derived value comes from
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

Eighteen modules, each owning one concern of the 5Cs; `model.py` is the graph the others read
through. There is no parser module, no orchestrator façade and no helper module: graph readers
live **with the concern that consumes what they read**, the entry point is the pipeline itself,
and a helper with one consumer lives next to it. Imports point one way, from a concern to the
ones it builds on, so no two modules import each other.

Nothing imports a name that starts with `_` from a sibling: the underscore convention **is** the
public-surface contract, and everything private is a module's own business (Parnas, *On the
Criteria To Be Used in Decomposing Systems into Modules*, CACM 15(12), 1972). There is no
`__all__` anywhere in the package: it duplicated the same rule and drifted silently.

| module | ~lines | 5C | why it is not folded into a sibling |
|---|---|---|---|
| `model.py` | 340 | — | every other module reads the graph through it; folding it into one concern would make the others depend on that concern |
| `operations.py` | 1450 | computation | it never asks what a value *is*; folding it into `quantities.py` would put the scheduler inside the geometry readers |
| `quantities.py` | 1370 | computation | the readers: what every quantity, pose, frame, constraint and placement *is* |
| `views.py` | 720 | computation | what each motion reads through the quantities: views, data structures, per-motion references, snapshots, pose components |
| `data_access.py` | 670 | computation | the single answer to *who writes and who reads this D-block*: the computation indexes, the algorithm data, its data access constraints and their analysis |
| `constraint_handler.py` | 1600 | computation | the declared seam for a controller DSL: its inputs and outputs are frozen so that a future DSL replaces this module and nothing else |
| `sampling.py` | 120 | computation | what the run draws at startup, read as its distribution |
| `agents.py` | 1405 | resources | the agents: devices, sensors, cameras, chain setups, and the solver records built on them |
| `runtime.py` | 640 | resources | what running the solvers implies once all are known: shared runtimes, ports, world observations, joint-space channels |
| `mobile_base.py` | 270 | resources | a platform's drive geometry, read off the scene's tree |
| `scene.py` | 430 | composition | the assembled scene: robots, objects, frames and cameras, with their placements and attachments |
| `deployment.py` | 495 | configuration | the execution platform, its backend and config, the agents' homes, the ROS publishers a deployment turns on |
| `coordination.py` | 1190 | coordination | the only module that decides what runs when; nothing else may sequence |
| `fsm.py` | 375 | coordination | the FSM wired to the monitors that fire it |
| `perturbations.py` | 180 | coordination | the disturbances a simulation applies, and the window each is open in |
| `ros_messages.py` | 695 | communication | what a ROS interface offers a model, read off its rosidl classes, and what a monitor publishes |
| `communication.py` | 1050 | communication | the only module that decides what crosses the activity's boundary; folding it into `data_access.py` would make the algorithm data responsible for its own logging |
| `ir.py` | 415 | — | it is the pipeline; it holds no derivation of its own |

### Three rules that hold in every module

**A record that a derivation mutates is a typed entity.** Every such record is a dataclass in
one of `classes/`'s domain modules, whose shared `base` carries the
`INTERNAL` convention that keeps a construction input out of the published document — entities,
data structures, views, motions, monitors, evaluators, controllers, robots, solvers, scene
records, constraint handlers, and every D-block of the algorithm data including the runtime values.
A published payload that nothing mutates after it is built may stay a plain dict: the `platform`,
the trace settings, the agent homes, the `by_kind` views. Two mutated structures stay dicts
because a fixed set of fields cannot describe them — a **function** (an F-block with its closure:
its ports, whose names are operand names taken from RDF predicates, bound to D-blocks), and a
**telemetry row**, whose shape is the artifact's. Both are read and written as dicts.

There is therefore no dict-or-dataclass access helper. Attributes are read as attributes and keys
as keys, because at every site the shape is known. A site that genuinely receives both is a
design error, not a reason to reintroduce one.

**Each concern validates what it reads, at the moment it reads it.** There is no validator module
and no `check_*` entry point: a scene object on real hardware is rejected inside `read_scene`, a
solver the backend cannot run inside `build_robots`, an unresolvable id inside
`build_telemetry`. Fail fast, in the one place that still holds the context needed for the
message. A module never validates another module's records.

**A concern declares its own D-blocks; only `communication.py` turns them into rows.** The
internal state a PID keeps and the gains it carries are `constraint_handler.py`'s, the joint-space
channels a serial chain mirrors are `runtime.py`'s, and a sent goal's status is the action
channel's: each adds its D-blocks to the algorithm data before the data access constraints are
built. No module outside `communication.py` appends to the telemetry artifact or the frame log.

Each module's docstring lists its top-level sections, in order. A load-bearing invariant gets a
named section rather than a comment in the middle of a long file.

### `model.py` — the loaded model

`Model` wraps the dataset `motion_spec_dsl.gens.generate` returns, read in process rather than
re-parsed from the files it wrote: the manifest in the default graph, each imported document as
the graph its import IRI names, read as their union. It carries everything every reader needs:

- the graph and the app-model path;
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
2. the operator tables — `OPS_GENERIC`, `OPS_SOLVER`, `OPS_HANDLER` — and `build_functions`, which
   turns each call of an operator into the F-block, bound to its D-blocks, that the templates
   render.
3. `Schedule` — dependency-ordered call ids, emitted at most once per instance. A new instance
   is a new scope: the `when` block and the active block each need one, because a step evaluated
   in both phases must be emitted in both.

Public: `normalize`, `OPS_GENERIC`, `OPS_SOLVER`, `OPS_HANDLER`, `ErrorEvaluator`,
`AssignmentEvaluator`, `Schedule`, `build_functions`, `resolve_function_operands`,
`function_maps`, `function_owner_map`, `data_reference_map`, `path_projections_for_motion`,
`continuous_joint_leaves`.

### `quantities.py`, `views.py`, `data_access.py` — what D-blocks exist, and who accesses them

The model's data layer, in three modules that build on each other in this order. The terms are
Bruyninckx's (*Situational aware robotic and cyber-physical multi-agent systems*, 2026, §2.3): an
algorithm composes F-blocks (functions) over D-blocks (data) under S-blocks (schedules), and a
data access constraint connects an F-block's port to a D-block, for reading or writing.

1. **`quantities.py`, the readers** — every quantity and geometry node the DSL emits: positions,
   orientations, poses, twists, wrenches, durations, joint quantities, frames, constraints, and
   where a body or frame is placed (`anchor_frame`, `frame_placement`, `placement_of`, and
   `drawn_placement_of` for a free scene object whose position the run draws);
2. **`views.py`, what a motion reads** — the MAP views and data structures, and per motion its
   references, snapshots, pose-axis error groups and pose components;
3. **`data_access.py`, who accesses it** — the computation indexes, which data structures are the
   algorithm's D-blocks, their data access constraints, and the analysis of those.

`build_data_access` is the single answer to *"which block writes this D-block, and which read
it, through which port?"* — it consults functions, controllers, monitors, solvers, sensors and
channels, from their records. `analyse_data_access` then answers *when* each D-block is written,
over the schedules that run them, and the storage that follows (`never → absent`, `init →
record`, `tick` or a set of motions `→ log`); a D-block never written leaves the algorithm data,
and one read but never written is an error. Every projection the templates and the logging
stream ask for is read off those. **One answer, N projections:** adding a construct adds a
projection, and it cannot add a second opinion. Any other module answering the same question
from a weaker premise is a defect, not redundancy.

Public:

- `quantities.py` — `quantity`, `pose`, `position`, `orientation`, `velocity_twist`,
  `acceleration_twist`, `wrench`, `joint_quantity`, `frame`, `position_values`,
  `orientation_quaternion`, `anchor_frame`, `frame_placement`, `placement_of`,
  `drawn_placement_of`;
- `views.py` — `read_views`, `read_data_structures`, `views_for_access`, `views_by_subobject`,
  `snapshots_for_motion`, `pose_axis_error_groups_for_motion`, `declared_pose_component_entries`,
  `collect_motion_references`, `elapsed_coordinate_ids`, `expanded_constraints`,
  `build_pose_components`;
- `data_access.py` — `build_indexes`, `Computation`, `ComputationIndexes`,
  `filter_algorithm_data`, `build_data_access`, `analyse_data_access`, `bound_id`, `PORT_WRITERS`.

### `constraint_handler.py` — the control law

Everything that turns an authored controller into derived per-axis controller records: the axis
decision table, the derived signal/error/energy ids and the IRIs they register, saturations,
motion drivers, the function and data augmentation those imply, and the invariants that only hold
once authored RDF has resolved to solver plans. It also declares each controller's D-blocks: the
internal state it keeps between ticks and its gains, which the book calls dependency injection —
configuration of the F-block, kept as data so a run can report it.

A colleague builds a controller DSL later. This module is where it lands: its inputs are
`(Model, functions, data structures)` and its outputs are the `SolverDerivationContext` and the
controller/driver records. No other module derives a controller.

Public: `SpatialAxis`, `LINEAR_AXES`, `ANGULAR_AXES`, `POSE_AXES`, `spatial_axes`,
`SolverIdFactory`, `ControllerDerivation`, `SolverDerivationContext`, `solver_derivation_context`,
`motion_drivers`, `augment_functions`, `augment_data`, `annotate_controller_signals`,
`add_controller_state`, `add_control_parameters`, `CONTROLLER_STATE_FIELDS`,
`CONTROLLER_GAIN_FIELDS`, `SOLVER_SEMANTICS_BY_ALGORITHM`.

### `agents.py`, `runtime.py`, `scene.py`, `deployment.py` — what the program commands

In the order they build on each other:

1. **`agents.py`** — the agents: assemblies, devices, sensors, cameras, config keys, robot
   setups, and the solver records built on them;
2. **`runtime.py`** — what running those solvers implies once every one is known: the runtimes
   they share, the world model's ports, the observations the loop answers, the runtime-written
   D-blocks, the joint-space mirrors;
3. **`scene.py`** — the scene the runtime assembles: robots, objects, frames, static cameras,
   with their placements and attachments;
4. **`deployment.py`** — the execution platform, the backend it implies, the deployment config,
   the agents' homes and config poses, and the ROS publishers a deployment turns on.

Each validates what it reads — a scene object on real hardware, an undriven device, an unbound
sensor, a solver the backend cannot run.

Public:

- `agents.py` — `Robots`, `AgentAssembly`, `build_robots`, `robot_setups`, `tree_segments`,
  `agent_assemblies`, `fixed_attachments`, `kinematic_adjacency`, `body_path`,
  `mapped_targets`, `model_mappings`, `device_of`, `place_on_chain`, `place_solver_on_chain`;
- `runtime.py` — `annotate_runtime`, `annotate_device_dependencies`, `runtime_data`,
  `add_joint_space_mirrors`, `world_ports`, `world_observations`, `index_chain_joints`,
  `JOINT_SPACE_CHANNELS`, `JOINT_SPACE_COMMAND_CHANNEL`;
- `scene.py` — `read_scene`, `name_object_attachments`;
- `deployment.py` — `read_platform`, `platform_config`, `agent_home_positions`, `config_poses`,
  `ros_joint_states`, `ros_clock`, `ros_tf`, `reject_scene_objects_on_hardware`,
  `TRACE_DISABLED`, `AGENT_HOME_KEY`, `CONFIG_POSE_FIELDS`.

### `coordination.py`, `fsm.py` — what runs when

`coordination.py` holds the constraint handlers and the per-motion unit: which constraints
belong to `when` / `while` / `until`, the three schedules in the order the loop runs them, and
the monitors and the conditions they evaluate. `fsm.py` wires the FSM to the monitors that fire
it: the state each motion runs in, the events snapshots and re-tares wait on, and the holds a
gated motion runs.

Public: `build_constraint_handlers`, `build_motions`, `evaluator_term`, `set_motion_conditions`;
`apply_fsm_wiring`, `apply_fsm_gate_calls`.

### `communication.py` — what crosses the activity's boundary

Communication is exchange between activities (Bruyninckx 2026, §2.5, §10.4.5); the D-blocks the
controller's F-blocks share are inside one activity, so their data access is computation's. What
crosses is a **logging stream** (§11): the per-tick frames are its data stream, and the header —
the motion/controller/monitor/quantity rows that say how to read a slot, and the constants
recorded once — its metadata stream (§2.5.15). `build_telemetry` builds it as a tap on the
algorithm data and its analysis, adding nothing to either. The ROS interface — what the model
publishes, the goals it sends and the one it answers, and the goal status a sent goal writes
back — is the other exchange.

What a ROS interface offers is `ros_messages.py`'s: the fields an action goal, result or message
lets a model state, read off its rosidl classes, and what a monitor publishes and answers with.

Public: `build_telemetry`, `add_goal_status_values`, `ros_publishers`; `action_shape`, `detect_shape`,
`observation_shape`, `standing_shape`, `publish_field`, `ros_publication`.

### `ir.py` — the pipeline

`generate_ir(model)` calls the stages above in order and assembles the sections; last, it states
every published id's IRI and what each derived value comes from, and refuses an id with no IRI.
Public: `generate_ir`.

## 3. The two interfaces

### `ir.json` — the IR

`ir.json` is the interface between `ir_gen` and codegen: a value handed from one stage of a
pipes-and-filters pipeline to the next. Nothing broadcasts and nothing subscribes; it is a
*description of the algorithm*, computed ahead of time.

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
| `computation` | `data`, `functions`, `data_access`, `views`, `clock` | the algorithm: its D-blocks, its F-blocks, the data access constraints between them |
| `coordination` | `motions`, `fsm` | what runs when: each motion's S-blocks, and the FSM that dispatches them |
| `communication` | `telemetry`, `ros` | what crosses the activity's boundary: the logging stream (frame log and shm), the ROS topics |
| `provenance` | `uris`, `derivations` | every published id's IRI, and what each derived value is computed from |

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
projection (`motion`, `functions`, `views`, `solvers`) and stays flat.

### `computation.data` — the algorithm data

```{graphviz} _static/runtime.dot
:caption: The generated program at run time.
```

**A D-block must have a data access constraint that writes it.** If nothing in the model writes
it, it is not algorithm data — it is scratch, configuration, or another concern borrowing the
struct because the struct is reachable — and the analysis removes it; if something reads it
anyway, that is a broken binding and generation stops.

Storage follows from when a D-block is written, in exactly one place: `never → absent`,
`init → record`, `tick` or a set of motions `→ log`. Consequences that are rules, not
preferences: solver scratch lives in solver state, geometry lives in configuration, coordination
state lives in coordination, and the algorithm data holds only D-blocks something writes.

**The rule holds today and must keep holding: every member of the generated `struct
algorithm_data` corresponds to an entry in `computation.data`, one for one.** The coordination
event buffer is its own `struct motion_spec_event_buffer`, passed explicitly to the step and
monitor functions rather than riding the algorithm data, and `clock_time_s` is a modelled value
written by a `port`. A member that appears in the generated struct without an IR entry is a
regression.

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

For every module in `rdf_parser/` the rule means two things:

1. no non-docstring string literal contains target syntax (`KDL::`, `std::`, `->`, `shared.`,
   `#include`, `nullptr`);
2. no module imports a name beginning with `_` from a sibling module.

What the view is allowed to do: presence tests, iteration, and dispatch. `&&`/`||`/`!` are
accepted by ST4 but are computation over data values — if a template needs one, layer B is
missing a projection. Multi-variant hooks dispatch through a dictionary with a `default`, and
dispatch passes **one context object**, never N positional arguments.

Template layering needs no bespoke test: ST4 enforces it. A cyclic import makes `stst` recurse
until it overflows. Every artifact renders through a per-backend root, `backend/<name>/main.stg`,
which imports its backend's groups before the entry groups; ST4 resolves a name against the
root's imports in order, so a shared group's call reaches the backend's rule with no dispatch.

```{graphviz} _static/templates.dot
:caption: Template groups and their imports.
```

## 5. How this is verified

**Not by byte-diffing generated C++.** A rename or an IR restructuring legitimately moves the
C++, so comparing it against a pre-change baseline fights the change instead of checking it.

The gate:

1. `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest` in `motion-spec` and `motion-spec-dsl`, from the
   workspace `.venv`;
2. `motion-spec check <generation>/generated/model/<name>-app.ld.json` → `Conforms: True`;
3. `motion-spec run` on `pick_and_place`, `dual_arm_pick_and_place` and `arc_tracing_with_admittance` →
   each reaches `s_done`, with no step or time bound.

A truncated run proves only that the model started. Reproducible generation is worth having on
its own merits — a build that differs run to run cannot be explained — but it is not a
prerequisite of the gate.

Tests follow the repo rule: minimum count, none for trivial code, and none that pins incidental
structure. A test earns its place by proving a behaviour that would otherwise fail silently —
the data access constraints, the layer boundary, id resolution, the solver-derivation shape.
