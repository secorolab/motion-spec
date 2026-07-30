# code-generator

StringTemplate v4 (`.stg`) templates that motion-spec codegen renders into the
generated C++ controller, introspection artifacts, and CMake.

### `main.stg`
- Root group: imports every other template group.
- Holds the top-level program — shared_state header, `app_main`/main loop, clock/telemetry, FSM dispatch, and `ref_main`.

### `shared_data.stg`
- Lookup maps and scalar saturation helpers.
- Shared-data struct member declarations.
- View access expressions for reading typed subspaces of shared data.

### `closures.stg`
- Closure/evaluator library (`emit-call-*`): trajectories, controllers, geometry/wrench ops.
- Pose-axis error groups.

### `solver.stg`
- Modular Vereshchagin/RNEA dynamics solver.
- Solver state, init, input sync, outputs, and run stages.

### `robot.stg`
- Backend-neutral robot driver dispatch: include, state, init, chain, configure, shutdown.

### `motion.stg`
- Per-step schedule and monitor blocks.
- Arm and mobile-base cycle assembly.
- Per-motion and mobile-base cycle headers.

### `introspection.stg`
- `frame_layout.h` and the runtime frame-log writer.
- Model sample/state templates.
- `frame_log.proto` wire contract.

### `runtime.stg`
- Runtime control-loop math header: progress/admittance/profile helpers and monitor helpers.

### `mj_kdl_backend.stg`
- MuJoCo+KDL simulation robot impl (KinovaGen3).
- CMake generation.

### `robif2b_backend.stg`
- EtherCAT/KELO driver bindings.
- robif2b robot impl.

### `hddc2b.stg`
- KELO mobile-base solver: platform/drive/wheel force distribution.
