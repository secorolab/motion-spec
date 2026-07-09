# Plan: `.mcap` first-class, `.bin` deleted, proper well-known + custom schemas

## Outcome
`frame_log.mcap` is the sole introspection log. `.bin` and its struct reader/header are
deleted. Python reads runs back from the `.mcap` (new hard dep: `mcap`). Every concern
rides the right schema on its own channel, so the log is natively inspectable in Foxglove.

## Channels (one `frame_log.mcap`)
1. `/motion_spec/frame` — **custom** `motion_spec.introspection.Frame` (jsonschema/json).
   Dense per-tick telemetry (constraints/monitors/quantities/timing/fsm indices). The
   canonical record Python/RDF read. **Quantities are now a named object** keyed by
   quantity id (`{"pose_ee.position.x": …}`), not a positional array — every signal is a
   named, typed, plottable Foxglove path.
2. `/motion_spec/log` — **`foxglove.Log`** (well-known, verbatim). One message per FSM
   **state change** and per **event**, `message` = resolved IRI (fallback id). Native Log
   panel; solves "states are bare integers".
3. `/motion_spec/pose/<id>` — **`foxglove.PoseInFrame`** (well-known) per `Pose` data
   object. `frame_id` = the pose's reference frame (`as_seen_by` ‖ `with_respect_to`),
   position from `KDL::Frame.p`, orientation = exact quaternion `M.GetQuaternion()`.
4. `/motion_spec/twist/<id>` — **`motion_spec.TwistInFrame`** (custom, authored in the
   Foxglove pattern) per `VelocityTwist`. `{linear, angular}` (Vector3 each) from
   `KDL::Twist.vel/.rot`.
5. `/motion_spec/wrench/<id>` — **`motion_spec.WrenchInFrame`** (custom, Foxglove pattern)
   per `Wrench`. `{force, torque}` from `KDL::Wrench.force/.torque`.

Foxglove has no well-known Twist/Wrench schema → we define them (schemas 4/5) mirroring
`foxglove.PoseInFrame` (timestamp `{sec,nsec}` + `frame_id` + nested Vector3s). Joint-space
`foxglove.JointState` stays **deferred** (needs solver q/qd/tau taps, not sampled today).

## Schema source of truth — `src/motion-spec/schemas/jsonschema/`
Self-contained (codegen embeds these verbatim; no dependency on the sibling foxglove-sdk
checkout at build time):
- `Log.json`, `PoseInFrame.json` — vendored from foxglove-sdk (the two we embed).
- `TwistInFrame.json`, `WrenchInFrame.json` — **authored**, same pattern.

## Frame layout (`frame_layout_spec.py` + C++ `Frame`)
Pools gain `poses`, `twists`, `wrenches`. New slots ride in the in-memory `Frame`/ring so the
writer thread can emit their channels; they count toward `frame_size_bytes`/hash but are
**not** in the `/motion_spec/frame` JSON (they have their own channels):
- `PoseSlot`  = px,py,pz, qx,qy,qz,qw  (7 doubles)
- `TwistSlot` = lx,ly,lz, ax,ay,az     (6 doubles)  [linear=vel, angular=rot]
- `WrenchSlot`= fx,fy,fz, tx,ty,tz     (6 doubles)  [force, torque]
`frame_json_schema(quantities)` now takes the ordered quantity list → builds the `quantities`
object with per-id typed properties (`["number","null"]`, title=id, description=unit/kind/uri).

## Codegen — thread the reference frame (no faking)
- `ir_gen._build_introspection`: each spatial quantity carries `reference_frame` =
  `_id_ref(item.as_seen_by) or _id_ref(item.with_respect_to)`.
- `codegen.add_quantity_samples`: `reference_frame` propagates into `quantity_samples`
  (rows already copy source keys). Build spatial sample lists (pose/twist/wrench) with
  `{index, topic, frame_id, sample block}`; the block extracts `.p`+`M.GetQuaternion()` /
  `.vel/.rot` / `.force/.torque` in generated code (KDL stays in `introspect_model.hpp`;
  `RunPublisher` takes plain doubles → backend-agnostic).
- `artifacts.py`: pass `schema["quantities"]` to `frame_json_schema`; add poses/twists/
  wrenches to pools; emit naming tables into the `frame_layout.h` context — `kQuantityIds[]`,
  `kFsmStateIds[]`/`kFsmStateIris[]`, `kEventIds[]`/`kEventIris[]`, `k{Pose,Twist,Wrench}Topics[]`
  + `…FrameIds[]`; embed `kFoxgloveLogSchema`, `kPoseInFrameSchema`, `kTwistInFrameSchema`,
  `kWrenchInFrameSchema` (loaded from the schemas dir).

## C++ (`module.stg`, `mj_kdl_backend.stg`)
- **Delete `.bin`:** `FrameLogHeader`, `kFrameLogMagic`, `mcap_path_for_log`, the header/frame
  `fwrite`, `file_`/`written_` bin bookkeeping. Ring + writer thread stay; feed only mcap.
  `MOTION_SPEC_FRAME_LOG` names the `.mcap`; health = `<log>.mcap.health.json`.
- `open_mcap`: register the 5 schemas/channels (frame + log + per-pose/twist/wrench).
- `encode_frame_json`: `quantities` → object keyed by `kQuantityIds[i]`.
- `drain()` (writer thread, off RT path): emit `foxglove.Log` on fsm_state change and per new
  trigger; emit PoseInFrame/TwistInFrame/WrenchInFrame from the frame's spatial slots.
- `RunPublisher`: `pose_sample/twist_sample/wrench_sample` (plain doubles). `sample_model`
  calls them with `shared.<id>` (KDL extraction in the generated block).

## Python read — `.mcap`-backed
- `replay.py`: drop `MAGIC`/`HEADER`/struct. `frames()`/`decode_frames()` open the mcap,
  iterate `/motion_spec/frame`, `json.loads` each (already the record shape). Header/provenance
  (`schema_hash`/`frame_layout_hash`/producer/activity) read from the channel metadata and
  checked against schema.json/frame_layout.json. `summarize`/`sampled_frames`/`runtime_frames`
  iterate mcap messages. `runtime_graph.py` unchanged (consumes records; never read quantities).
- `archive.py`: `HASHED_ARTIFACTS` `frame_log` → `logs/frame_log.mcap`, health →
  `logs/frame_log.mcap.health.json`; drop `frame_log_mcap` duplicate + `.bin` copies.
- `runner.py`: `frame_log = logs/frame_log.mcap`; health path `.mcap.health.json`.
- `pyproject`: `mcap` in the introspection extra (installed 1.4.0).

## Tests
- New `.mcap` fixture (via `mcap.writer.Writer`) exercising all 5 channels; assert replay
  round-trips frames, channels/schemas present, Log carries names, Pose/Twist/Wrench validate.
- Update `.bin`-based fixtures in `test_runtime_graph.py`, `test_introspection_archive.py`,
  `test_introspection_runner.py` to synthesize a `.mcap`.
- Keep the `frame_json_schema` ↔ record drift guard (keyed quantities); add spatial-channel
  spec assertions in `test_introspection_artifacts.py`.

## Staging
1. **Core:** drop `.bin`; named quantities; `foxglove.Log`; Python mcap read; tests. (fully
   satisfies first-class/clean)
2. **Spatial:** PoseInFrame + TwistInFrame + WrenchInFrame channels + custom schemas.

## Verify
1. Codegen + build `pick_place_single` introspection=ON; run headless.
2. `mcap doctor` clean; `mcap list channels` shows all 5 with the right schemas.
3. `replay.py --jsonl` + `runtime.ttl` recovery reproduce prior output.
4. Foxglove: Log panel named; Plot panels name every signal; 3D shows poses/twists/wrenches.
