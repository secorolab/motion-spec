# Namespaced monitor events → fire coord2b FSM events

## Context

A `.robmot` monitor currently triggers a **monitor-owned** event (`rdf.py:2791`
`URIRef(f"{mon.uri}.{signal_name}")`) and emits a dead C++ stub
(`warn_produce_event_not_implemented(...)`, module.stg:530/543). To drive a coord-dsl/coord2b FSM, a
monitor's event must **be** an event the FSM defines (same namespace+name, so URIs align), and the
generated C++ must call `produce_event(events*, <fsm_ns>::<EVENT>)` — the FSM's enum value is the
index coord2b wants (`test_fsm.cpp:33`).

Because the generated code calls the FSM enum directly, **a `.fsm` — compiled by coord-dsl into its
C++ header — becomes a required input of the motion-spec pipeline whenever FSM wiring is used.** The
pipeline must generate that header and the motion-spec build must include it; coord-dsl and coord2b
become pipeline dependencies.

## Design

Monitor event = **optionally namespace-qualified name** (no FSM concept in the `.robmot`):
- `trigger event home-settled` → standalone ⇒ the model's own namespace (today's default),
  URI = `<model_ns>home-settled`.
- `trigger event ns2.E_OBJ_REACHED` → qualified ⇒ URI = `<ns2_uri>E_OBJ_REACHED` (`ns ns2 = "..."`
  declared in the `.robmot`, matching the FSM's namespace); local name = the FSM enum token.

The event URI propagates RDF → IR (already surfaces in `uris.hpp`). The FSM's C++ namespace + header
are derived by codegen from the `.fsm`, not `.robmot` concerns. The monitor calls the FSM enum
through an injected, null-guarded `robot.fsm_events`.

## Changes

### A. motion-spec-dsl — optionally-namespaced event reference
- Grammar `metamodels/motion_spec.tx`, `MonitorEntry` (line 528): `event=IRI_TRUNK` →
  `event=EventName`, `EventName: (ns=[NamespaceDeclare|FQN] ".")? name=IRI_TRUNK` (reuses existing
  namespace-ref machinery).
- `domain.py`: `MonitorEntry.event` gains `uri` = `Namespace((self.ns or <default model ns>).uri)[self.name]`
  and an `event_name` accessor.
- `rdf.py` (~2789-2791, both branches): `signal_node = URIRef(mon.event.uri)`; keep `EL.Event`
  typing + event-loop attachment.

### B. motion-spec — IR (semantic only)
`ir_gen.py` `monitor_entry` (~1245): add `event_uri`, `event_name` to `MonitorEntry`. **Nothing FSM-
namespace/header-related goes in the IR** — the IR is the motion model; the FSM C++ binding is a
codegen/build concern (see C).

### C. motion-spec-codegen — take the `.fsm` as input, orchestrate both codegens
`motion-spec-codegen` gains a `--fsm <file.fsm>` input. When given, it **first runs coord-dsl on the
`.fsm`** (subprocess `textx generate <file.fsm> --target cpp -o <out>/headers/<name>.hpp`, the same
way it already shells out to `stst`), then derives the FSM C++ namespace + header from the generated
output and **proceeds to the motion-spec codegen** against them. The namespace/header are derived
render parameters (merged into the in-memory `.stst/` render payload only) — never written into
ir_gen's semantic `ir.json`.
- `codegen.py` `main()`: add `--fsm <file.fsm>`; `generate_code` runs the coord-dsl step, reads the
  FSM namespace (from the generated header / FSM name) and header path, and renders the FSM-wired
  templates with them.
- `code-generator/module.stg`:
  - `update-monitor` edge branch (~530): replace the warn stub with
    `if (robot.fsm_events) produce_event(robot.fsm_events, <fsm_namespace>::<monitor.event_name>);`.
  - `shared_state_header` (~1377): under `<if(fsm_namespace)>` — `#include "coord2b/functions/event_loop.h"`,
    `#include` the `<fsm_header>`, add `struct events *fsm_events = nullptr;` to `robot_io`.
  - `ref_main` (~1884): under `<if(fsm_namespace)>` — `<fsm_namespace>::create_fsm()`,
    `robot.fsm_events = fsm->eventData;`, `fsm_step_nbx` + `reconfig_event_buffers` in the loop,
    `destroy_fsm` at end.
- coord2b in the generated CMake — both `code-generator/CMakeLists.txt` (robif2b) and `cmake_mj_kdl`
  (`mj_kdl_backend.stg:301`): `find_package(coord2b REQUIRED)`, link `coord2b`, add the FSM header
  include dir.

### D. Pipeline — pass the `.fsm` through
- `bdd_collab_bhv_cpp/models/Makefile` `codegen` rule: pass `--fsm <name>.fsm` to
  `motion-spec-codegen` (codegen runs the coord-dsl FSM step itself — no separate Makefile rule).
  A model that wires FSM events won't build without its `.fsm` present — the intended dependency.
- coord-dsl (`textx` + `coord_dsl`) and coord2b (in the `INSTALL` tree) become documented pipeline
  build deps (`motion-spec/README.md`); `check-deps` gains a coord-dsl/`stst` check.

## Intentionally NOT in scope
- No edits to `bdd_collab_bhv_cpp` C++ sources or to coord-dsl (the FSM header already exposes the
  event enum).
- No FSM-driven motion *selection* yet (FSM steps alongside; dispatch unchanged).
- Per-model monitor→FSM-event mapping and choice of `.fsm` are author-supplied.

## Verification
1. Standalone form unchanged: existing `.robmot` monitors parse/emit; `pytest src/motion-spec-dsl/tests`
   + `src/motion-spec/tests` green.
2. Qualified form end-to-end: a small `.robmot` (`ns ns2` + `trigger event ns2.E_STEP`) and a small
   `.fsm`; `make run MODEL=<model>` (env `INSTALL`, `METAMODELS_PATH`) — `motion-spec-codegen --fsm <name>.fsm`
   runs coord-dsl on the `.fsm` first, then the motion-spec codegen; the standalone `main`
   creates+steps the FSM; firing the monitor produces the FSM transition; coord2b links.
3. Missing `.fsm` when FSM-wired → build fails at the FSM-gen step (the intended dependency).
4. Null-guard: model generated without `--fsm` → no `fsm_events`; `produce_event` skipped;
   `make run MODEL=pick_place_relative.robmot` still builds.
