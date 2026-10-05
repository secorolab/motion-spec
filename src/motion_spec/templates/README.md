# templates

StringTemplate v4 (`.stg`) groups that motion-spec codegen renders into the generated C++
controller, its telemetry, and CMake.

Architecture, measurements and citations: `docs/sphinx/source/architecture.md`.

## Rules

**All target-language text is rendered here, and only here.** The IR is the interface: codegen
loads `ir.json`, writes artifacts and renders — it never reshapes the payload, and no C++
fragment is assembled in Python.

**The view performs no operations.** StringTemplate is logic-less by design (Parr, WWW '04):
a template may test the *presence or absence* of a value, iterate, and dispatch. It may not
compute over data, compare data values, or assume types. A predicate over a collection is a
**query**, and a query is solved in `ir_gen` — if a template wants `&&`, the IR is missing a
resolved collection.

**Imports are acyclic.** A shared group imports only shared groups; a cyclic import makes `stst`
recurse until it stack-overflows. Imports are relative to the importing file and precede the
rules.

**The backend is chosen by the root, not by dispatch.** Codegen renders every artifact through
`backend/<backend>/main.stg`. A root imports its backend's groups first and the entry groups
after; ST4 resolves a name against the root's imports in order, so the first definition wins.
A shared group calls a rule such as `solver-run-finalize` or `world-port-address` by its plain
name and gets the backend's definition; a shared default in a later import applies only where
the backend defines none.

**Dictionaries are not polymorphic.** ST4 binds a dictionary to the group that defines it, so a
table that differs per backend lives in the backend group, beside the rules it names.

**Multi-variant hooks dispatch through a dictionary with a `default`**, so only variants that
emit something need a rule.

**Sections the spec does not use are not rendered.** Runtime classes and overloads are gated on
`uses` (controller kinds, operators, `CompositeError`), which codegen derives from the IR.

## Layout

Folders follow the IR sections.

| folder | group | holds |
|---|---|---|
| `entry/` | `program.stg` | `main_source`, `algorithm_data_header`, shared hook defaults |
| | `motion.stg` | `motion_header` |
| | `telemetry.stg` | `frame_layout.h`, the frame-log and shared-memory writer, model samples |
| | `build.stg` | `cmake_project`, `robot_config.hpp` |
| | `runtime.stg` | `runtime_header`: the loop core, then each section's runtime the spec uses |
| `resources/` | `serial_chain.stg` | Vereshchagin/RNE solver state, init, run stages, outputs |
| | `world.stg` | the world model every chain reads its poses from |
| | `mobile_base.stg` | the hddc2b platform cycle and its runtime constants |
| `computation/` | `values.stg` | access expressions, data members, saturation, lookups |
| | `functions.stg` | function library: trajectories, geometry, expression operators |
| | `controllers.stg` | controller classes, state, tuning, per-tick call |
| | `monitors.stg` | conditions, edges, flags |
| | `poses.stg` | pose composition: rotation, position, deltas |
| `coordination/` | `fsm.stg` | FSM dispatch and per-state step functions |
| | `motion.stg` | schedules, motion cycle, chain and device members |
| `communication/` | `ros.stg` | the ROS node, executor, publishers, per-cycle tick |
| `backend/mj_kdl/` | `main.stg` | root: imports `robot.stg`, then the entry groups |
| | `robot.stg` | MuJoCo+KDL robot, camera, simulated mobile base |
| `backend/robif2b/` | `main.stg` | root: imports `robot.stg`, `kelo.stg`, then the entry groups |
| | `robot.stg` | robif2b robot; imports `devices.stg` |
| | `devices.stg` | bound devices |
| | `kelo.stg` | KELO mobile base over EtherCAT |
