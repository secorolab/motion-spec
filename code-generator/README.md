# code-generator

StringTemplate v4 (`.stg`) templates that motion-spec codegen renders into the
generated C++ controller, introspection artifacts, and CMake. Rendered by
`motion_spec.codegen.render_template`, which runs `stst -t <this dir> main.<template>`.

## Files

| File | Concern |
|------|---------|
| `main.stg` | Root group: imports every group **and** holds the top-level program — shared_state header, `app_main`/main loop, clock/telemetry, FSM dispatch, `ref_main`. |
| `shared_data.stg` | Shared data model (leaf): lookup maps, saturation, member declarations, view access expressions. |
| `closures.stg` | Closure/evaluator library (`emit-call-*`) and pose-axis error groups. |
| `solver.stg` | Modular Vereshchagin/RNEA dynamics solver: state, init, sync, outputs, run stages. |
| `robot.stg` | Backend-neutral robot driver dispatch (include/state/init/chain/shutdown). |
| `motion.stg` | Motion assembly: schedule/monitor blocks, arm/base cycle, per-motion + mobile-base headers. |
| `introspection.stg` | Introspection artifacts: `frame_layout.h`, runtime writer, model samples, `frame_log.proto`. |
| `runtime.stg` | Runtime control-loop math header (easing/admittance/spring-damper, monitors). |
| `mj_kdl_backend.stg` | **Tool**: MuJoCo+KDL simulation robot impl (KinovaGen3) + CMake generation. |
| `robif2b_backend.stg` | **Tool**: EtherCAT/KELO driver bindings + robif2b robot impl. |
| `hddc2b.stg` | **Tool**: KELO mobile-base solver (platform/drive/wheel force distribution). |

The three tool files are per-backend implementations; the neutral groups
dispatch into them (e.g. `robot.stg`'s `robot-init` → `robot-init-mj_kdl-KinovaGen3`).

## Resolution model

`-t <dir>` loads the whole directory as one flat namespace via `main.stg`'s
imports, so every template resolves. When moving or adding templates:

- **Static refs** — `<foo()>`, `<map.(key)>` — are import-scoped: the file must
  `import` the group that defines `foo`.
- **Dynamic dispatch** — `<({foo-<backend>})>` — resolves globally (no import
  required), but a group still imports the backends it dispatches to, for clarity.

Imports form an acyclic DAG; leaves (`shared_data`, `introspection`, `runtime`)
import nothing.

## Rules

- Put a template in the group matching its concern; add the `import` only if it
  **statically** references another group.
- Codegen output must stay **byte-identical** for unchanged models. Verify by
  diffing `gen/<model>/` before and after a template change.
