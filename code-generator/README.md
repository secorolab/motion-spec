# code-generator

StringTemplate v4 (`.stg`) templates that motion-spec codegen renders into the
generated C++ controller, introspection artifacts, and CMake. Rendered by
`motion_spec.codegen.render_template` (`stst -t <this dir> main.<template>`).

- **`main.stg`** — root group: imports every other group and holds the top-level program (shared_state header, `app_main`/main loop, clock/telemetry, FSM dispatch, `ref_main`).
- **`shared_data.stg`** — shared data model: lookup maps, saturation, member declarations, and view access expressions.
- **`closures.stg`** — closure/evaluator library (`emit-call-*`) and pose-axis error groups.
- **`solver.stg`** — modular Vereshchagin/RNEA dynamics solver: state, init, sync, outputs, run stages.
- **`robot.stg`** — backend-neutral robot driver dispatch (include/state/init/chain/shutdown).
- **`motion.stg`** — motion assembly: schedule/monitor blocks, arm/base cycle, per-motion and mobile-base headers.
- **`introspection.stg`** — introspection artifacts: `frame_layout.h`, runtime writer, model samples, `frame_log.proto`.
- **`runtime.stg`** — runtime control-loop math header (easing/admittance/spring-damper, monitors).
- **`mj_kdl_backend.stg`** — MuJoCo+KDL simulation robot impl (KinovaGen3) and CMake generation.
- **`robif2b_backend.stg`** — EtherCAT/KELO driver bindings and robif2b robot impl.
- **`hddc2b.stg`** — KELO mobile-base solver (platform/drive/wheel force distribution).
