# templates

StringTemplate v4 (`.stg`) groups that motion-spec codegen renders into the generated C++
controller, introspection artifacts, and CMake.

Architecture, measurements and citations: `docs/codegen-architecture/`.

## Rules

**All target-language text is rendered here, and only here.** The IR is the interface: codegen
loads `ir.json`, writes artifacts and renders — it never reshapes the payload, and no C++
fragment is assembled in Python.

**The view performs no operations.** StringTemplate is logic-less by design (Parr, WWW '04):
a template may test the *presence or absence* of a value, iterate, and dispatch. It may not
compute over data, compare data values, or assume types. A predicate over a collection is a
**query**, and a query is solved in `ir_gen` — if a template wants `&&`, the IR is missing a
resolved collection.

**Layers are enforced by the import graph.** A group may call only what it imports; the import
graph is acyclic and every group declares exactly what it needs. This is real enforcement, not
convention: ST4 resolves a name against the group and its imports, and a cyclic import makes
`stst` recurse until it stack-overflows. `main.stg` holds no rules — it imports the entry
groups so `main.<entry>` resolves.

Backend leaves are reached by **dynamic** dispatch (`<({rule-<backend>})(…)>`), which ST4
resolves at render time against the rendering group. That is why `backend_robot.stg` — the
dispatch shim — imports no backend: a static import there would close a cycle.

**Multi-variant hooks dispatch through a dictionary with a `default`**, so only variants that
emit something need a rule.

## Layout

| layer | group | holds |
|---|---|---|
| root | `main.stg` | imports the entry groups; no rules |
| L4 entry | `entry_program.stg` | `ref_main`, `shared_state_header` |
| | `entry_motion.stg` | `motion_header` |
| | `entry_introspection.stg` | `frame_layout.h`, the frame-log writer, model samples, `frame_log.proto` |
| | `entry_build.stg` | `cmake_mj_kdl`, `cmake_robif2b`, `robot_config.hpp` |
| L3 assembly | `assembly_loop.stg` | chain setup, clock, telemetry, ROS wiring |
| | `assembly_coordination.stg` | FSM dispatch and per-state step functions |
| | `assembly_motion.stg` | schedules, motion cycle, chain and device members |
| L2 domain | `domain_solver.stg` | Vereshchagin/RNE solver state, init, run stages |
| | `domain_closures.stg` | closure library: trajectories, controllers, geometry |
| | `domain_monitors.stg` | conditions, edges, flags, ROS publish |
| | `domain_poses.stg` | pose composition: rotation, position, deltas |
| L1 expression | `expr_values.stg` | access expressions, shared members, saturation, lookups |
| L0 backend | `backend_robot.stg` | backend-neutral dispatch shim |
| | `backend_mj_kdl.stg` | MuJoCo+KDL robot impl |
| | `backend_robif2b.stg` | robif2b robot impl and bound devices |
| | `backend_kelo.stg` | KELO mobile base (nothing renders it today) |
| — | `runtime.stg` | 625 lines of static C++ with 1% templating; a shipped header wearing a template's clothes |
