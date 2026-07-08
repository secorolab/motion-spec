# Plan: drop `.bin`, make `.mcap` first-class, use Foxglove well-known schemas

## Outcome
`.mcap` is the sole log format. `.bin` (and its header/struct reader) is deleted.
Python reads the run back from the `.mcap`. FSM states/events are also emitted on a
`foxglove.Log` channel so Foxglove shows them natively (named), not as bare integers.

## Channel design (multi-channel, right schema per concern)
All in one `frame_log.mcap`:
1. `/motion_spec/frame` — **custom** `motion_spec.introspection.Frame` (jsonschema).
   The dense telemetry (constraints/monitors/quantities/timing/fsm_state index). This is
   the canonical record Python/RDF read. No Foxglove well-known schema fits scalar
   controller telemetry, so custom is correct — authored in Foxglove style
   (title/description/$comment).
2. `/motion_spec/log` — **`foxglove.Log`** (well-known schema, verbatim). One message per
   FSM **state change** and per **event**, with the resolved id/IRI as `message`. Native
   Log panel; solves "states are numbers".
- `foxglove.JointStates` / `foxglove.PoseInFrame` — **deferred**: the introspection frame
  does not capture joint/pose data today. Additive later if we log those signals.

## C++ (`code-generator/module.stg`, `frame_layout.h` template)
- **Remove `.bin`:** delete `FrameLogHeader` write, `kFrameLogMagic`, the `fwrite(&frame)`
  in `drain()`, and the `file_`/`written_` bin bookkeeping. Ring buffer + background writer
  stay; they now feed only the mcap writer. `Frame` struct + pools stay (in-memory ring).
- **Log channel:** in `drain()` (writer thread, off the realtime path), track previous
  `fsm_state`; on change emit a `foxglove.Log` message `{timestamp:{sec,nsec}, level:INFO,
  message:"<state id/iri>", name:"motion_spec", file:"", line:0}`. On `trigger_count`
  increase, emit a Log per new trigger using the event id. All derived from frames already
  draining — no realtime-path change.
- **Naming tables:** generate `kFsmStateIds[]` and `kEventIds[]` (`const char*` arrays)
  into `frame_layout.h` from `schema.fsm.states`/`schema.fsm.events`. Add
  `kFsmStateBaseIri` so Log messages can carry the full model IRI.
- Embed the `foxglove.Log` schema verbatim as a second `kFoxgloveLogSchema` constant.
- `health.json`: keep `mcap_written_frames`/`mcap_write_errors`/`dropped_frames`; drop
  `.bin`-only `written_frames` (or alias to mcap count).

## Python — read the run from `.mcap`
- **New hard dependency:** `mcap` (pip). Add to `pyproject` (introspection extra). Already
  installed (1.4.0).
- `replay.py`: replace struct/`.bin` reader.
  - `frames()`/`decode_frames()`: open the mcap, iterate `/motion_spec/frame`, `json.loads`
    each message — **already the `to_record` shape**, so `to_record` is no longer needed on
    read (kept only if other callers use it). `sampled_frames`, `summarize`, `runtime_frames`
    reduce to iterating mcap messages.
  - `read_header`/`validate_header`: read `schema_hash`/`frame_layout_hash`/producer/activity
    from the **channel metadata** (already written there) and compare to
    `schema.json`/`frame_layout.json`. Drop `MAGIC`/`HEADER` struct.
- `runtime_graph.py`: unchanged logic — it consumes `decode_frames()` records; only the
  source changes.
- `frame_layout_spec.py`: `frame_json_schema` + field lists stay (define the format).
  `frame_struct`/`fields_with_offsets` byte-offset machinery no longer used for reading;
  keep `frame_layout.json` as the versioned **field/hash descriptor** (offsets become
  vestigial metadata, harmless).
- `archive.py`: `HASHED_ARTIFACTS` — drop `frame_log`(.bin) + `frame_log_health`(.bin.*);
  `frame_log` role → `logs/frame_log.mcap`; health → `frame_log.mcap.health.json`. Manifest
  `files.frame_log` → `.mcap`, drop `frame_log_mcap` duplicate. Update REC file records.
- `runner.py`: `frame_log = run_dir/"logs"/"frame_log.mcap"` (lines 55, 124); env
  `MOTION_SPEC_FRAME_LOG` now names the `.mcap`; health path `.mcap.health.json`.

## Tests
- Update `.bin`-based tests (`test_runtime_graph.py` builds a `.bin` via `HEADER`/`MAGIC`;
  `replay` tests) to synthesize a `.mcap` fixture instead (write frames + channel metadata).
- New: assert the `foxglove.Log` channel exists, uses schema `foxglove.Log`, and its
  messages carry resolved state/event names; assert round-trip `mcap frames == expected`.
- Keep the `frame_json_schema` vs `to_record` guard.

## Verify
1. Codegen + build `pick_place_single` introspection=ON; run headless.
2. `mcap list channels` shows `/motion_spec/frame [jsonschema]` + `/motion_spec/log
   [foxglove.Log]`; `mcap doctor` clean.
3. `replay.py --jsonl` (now mcap-backed) + `runtime.ttl` recovery reproduce prior output.
4. Foxglove Log panel shows named states; Plot panels unchanged.

## Notes / trade-offs
- Reproducibility: `.mcap` isn't byte-identical across runs (timestamps/chunk order), same
  as `.bin` already wasn't (wall_ns). Archive hashing still gives integrity for the file.
- `.bin`'s zero-dependency Python read is gone by design — `.mcap` first-class means the
  `mcap` reader dep is accepted.

---

# DEFERRED (revisit later): log joint-space internal state

**Status:** not implemented; must be added. Design captured here so it's not lost.

## Why
Everything logged today is the *specification's* view — task-space control signals,
constraint error/measured/setpoint, monitor values (the 827 quantities / 157 signals, all
model-declared). The robot's **physical state is not logged**. Gaps that constraint
`measured` values cannot cover:
- only scalar task-space projections are logged, never the joint configuration `q` → cannot
  reconstruct the robot, no 3D replay, cannot independently check the kinematics that
  produced `measured`;
- null-space / redundant-DOF motion (7-DOF arm) is invisible while every constraint stays
  satisfied;
- state no *currently-active* controller observes (e.g. object pose, contact during grasp).

Minimal sufficient state = **joint-space only**: `q`, `qd`, effort/`tau`. Do **not** log TCP
pose/twist — those are FK/Jacobian of `q`,`qd` (derived); Foxglove computes FK from a URDF.
`qdd` also derivable → skip. `tau` is the one non-derivable signal (dynamics/contact output)
→ keep.

## Semantics — where joint state enters (the important part)
It does **not** enter through the motion-spec quantity/signal layer. It already exists one
layer down, in models the codegen already consumes — reuse these terms, invent nothing:
- **`src/metamodels/kinematic-chain/structural-entities.json` → `Joint`** — structural
  identity (the IRI) each logged joint references; `q`/`qd` are coordinates of these.
- **`src/metamodels/task/solver-specification.shacl.ttl` → `JointAccelerationCoordinate`,
  `JointForceCoordinate`** (+ `JointTorque` in `constraint-handler-extension`) — the
  joint-space coordinates the Vereshchagin/RNE solver already computes each tick. `qdd` *is*
  a `JointAccelerationCoordinate`; `tau` *is* a `JointForceCoordinate`/`JointTorque`.

So joint state is already-modeled coordinates at the kinematic-chain ⇄ solver-specification
boundary; the log simply isn't tapping them yet. TODO before implementing: trace where
codegen consumes the solver spec (where `qdd`/`tau` coordinates are in hand) — that's the tap
point.

## Mechanism (when we do it)
- Generated joint pool derived from the **kinematic chain + solver spec the codegen already
  parses** (not a hand-invented pool). `JointSlot = {position, velocity, effort}`, sized by
  the chain's joint count.
- Sample per tick from the solver/robot object (`q`,`qd` inputs, `tau`/`qdd` solver outputs).
- Two sinks, same pattern as frame/Log: (a) custom `Frame` carries the joint slots for the
  RDF/replay path, referencing `kinematic-chain:Joint` IRIs and the coordinate types above;
  (b) a `/motion_spec/joints` channel using the fixed well-known **`foxglove.JointStates`**
  schema, with each message's `name` = the `Joint` IRI (keeps the semantic link, not just a
  label). Foxglove then does FK from a URDF → 3D replay for free.

## Open question
Multi-robot (dual-arm): one flat joint pool with `Joint` IRIs disambiguating, vs one
`foxglove.JointStates` channel per robot (`/motion_spec/joints/r1`, `/r2`). Decide at
implementation.
