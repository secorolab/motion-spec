# Plan 001: Generate semantic protobuf frame logs per run

> **Executor instructions**: Follow this plan step by step. Run every
> verification command and confirm the expected result before moving to the
> next step. If anything in the "STOP conditions" section occurs, stop and
> report. Do not improvise. When done, update the status row for this plan
> in `plans/README.md`.
>
> **Drift check (run first)**:
> `git -C src/motion-spec diff --stat c2bcfb3..HEAD -- src/motion_spec/introspection/artifacts.py src/motion_spec/introspection/frame_log_pb.py src/motion_spec/introspection/archive.py src/motion_spec/introspection/replay.py src/motion_spec/introspection/runner.py src/motion_spec/introspection/runtime_graph.py code-generator/module.stg tests`
>
> If any in-scope file changed since this plan was written, compare the
> "Current state" excerpts against the live code before proceeding. On a
> mismatch, stop and report.

## Status

- **Priority**: P1
- **Effort**: L
- **Risk**: HIGH
- **Depends on**: none
- **Category**: dx
- **Planned at**: commit `c2bcfb3`, 2026-07-10

## Why this matters

The current `logs/frame_log.pb` is protobuf, but generic protobuf tooling only
sees repeated arrays such as `quantities`, `poses`, and `wrenches`. The model
semantics live in `contract/schema.json`, so a human using `protoc --decode`
still has to map `quantities[0]` to a schema entry. The target state is that
each run archives a generated `contract/frame_log.proto`, and the actual
`logs/frame_log.pb` uses semantic field names derived from that run's model
schema.

Do not add a second converted log file. This plan changes the real runtime log
wire format produced by the generated C++ writer.

## Current state

Relevant files:

- `src/motion_spec/introspection/artifacts.py` builds `schema.json`,
  `frame_layout.json`, and the codegen payload consumed by StringTemplate.
- `src/motion_spec/introspection/frame_log_pb.py` manually encodes and decodes
  length-delimited protobuf records for tests and replay.
- `code-generator/module.stg` contains the generated C++ writer that writes
  `logs/frame_log.pb` during the run.
- `src/motion_spec/introspection/archive.py` archives `contract/frame_log.proto`.
- `tests/test_introspection_archive.py`, `tests/test_runtime_graph.py`, and
  `tests/frame_log_fixture.py` exercise archive/replay behavior.

Current archive behavior copies a static proto from the Python package:

```python
# src/motion_spec/introspection/archive.py:28
HASHED_ARTIFACTS = {
    "schema": "contract/schema.json",
    "frame_log_proto": "contract/frame_log.proto",
    ...
}

# src/motion_spec/introspection/archive.py:215
copies = {
    "schema.json": "contract/schema.json",
    str(Path(__file__).with_name("frame_log.proto")): "contract/frame_log.proto",
    ...
}
```

Current schema generation already has the model-specific ordered slots:

```python
# src/motion_spec/introspection/artifacts.py:365
quantities = [
    {
        "index": idx,
        **quantity,
    }
    for idx, quantity in enumerate(
        introspection.get("quantity_samples") or introspection.get("quantities", [])
    )
]

# src/motion_spec/introspection/artifacts.py:377
spatial = introspection.get("spatial_samples") or {"poses": [], "twists": [], "wrenches": []}
pools = {
    "constraints": max_controllers,
    "monitors": max_monitors,
    "quantities": len(quantities),
    "triggers": max(TRIGGER_POOL_SIZE, len(fsm.get("events", [])), max_monitors + heartbeat_events),
    "poses": len(spatial["poses"]),
    "twists": len(spatial["twists"]),
    "wrenches": len(spatial["wrenches"]),
}
```

Current Python fixture encoder still writes repeated array fields:

```python
# src/motion_spec/introspection/frame_log_pb.py:142
for idx in range(pools["quantities"]):
    payload.append(_double_field(15, flat[f"q{idx}"]))

# src/motion_spec/introspection/frame_log_pb.py:160
for idx in range(pools.get("poses", 0)):
    payload.append(_bytes_field(18, _slot(...)))
```

Current generated C++ writer also writes repeated array fields:

```cpp
// code-generator/module.stg:1776
for (std::size_t i = 0; i \< kNumConstraints; ++i) append_constraint(frame.constraints[i]);
for (std::size_t i = 0; i \< kNumMonitors; ++i) append_monitor(frame.monitors[i]);
for (std::size_t i = 0; i \< kNumQuantities; ++i) append_double(payload_, 15, frame.quantities[i]);
for (std::size_t i = 0; i \< kNumTriggers; ++i) append_trigger(frame.triggers[i]);
append_i64(payload_, 17, frame.trigger_count);
for (std::size_t i = 0; i \< kNumPoses; ++i) append_pose(frame.poses[i]);
for (std::size_t i = 0; i \< kNumTwists; ++i) append_twist(frame.twists[i]);
for (std::size_t i = 0; i \< kNumWrenches; ++i) append_wrench(frame.wrenches[i]);
```

Repo conventions to follow:

- Source files start with SPDX headers.
- Codegen changes live in `src/motion-spec/code-generator/module.stg` and
  Python helpers under `src/motion-spec/src/motion_spec/introspection/`.
- Manual protobuf encoding is already used. Do not add generated Python/C++
  protobuf runtime dependencies unless the maintainer explicitly approves.
- StringTemplate uses `<...>` delimiters. C++ `<` and `>=` operators inside
  templates must be escaped as `\<` and `\>=`.

## Commands you will need

| Purpose | Command | Expected on success |
|---|---|---|
| Unit tests | `cd src/motion-spec && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest tests` | exit 0, all tests pass |
| Build generated controller | `cd src/bdd_collab_bhv_cpp/models && make build MODEL=pick_place_single` | exit 0, CMake build succeeds |
| Full run | `cd src/bdd_collab_bhv_cpp/models && $GRC_SCRIPT_RUN pick_place_single` | exit 0, run reaches `S_DONE` |
| Replay summary | `cd src/motion-spec && python -m motion_spec.introspection.replay ../bdd_collab_bhv_cpp/models/runs/<run-id>` | prints frame count and final state |
| MCAP absence | `cd src/motion-spec && grep -R "mcap\\|MCAP\\|MOTION_SPEC_MCAP\\|mcap/writer" -n code-generator src/motion_spec tests pyproject.toml || true` | no matches |

## Scope

**In scope**:

- `src/motion-spec/src/motion_spec/introspection/artifacts.py`
- `src/motion-spec/src/motion_spec/introspection/frame_log_pb.py`
- `src/motion-spec/src/motion_spec/introspection/archive.py`
- `src/motion-spec/src/motion_spec/introspection/replay.py`
- `src/motion-spec/src/motion_spec/introspection/runner.py`
- `src/motion-spec/src/motion_spec/introspection/runtime_graph.py`
- `src/motion-spec/code-generator/module.stg`
- `src/motion-spec/tests/*`
- Generated ignored output under `src/bdd_collab_bhv_cpp/models/gen/pick_place_single/`
  and run archives under `src/bdd_collab_bhv_cpp/models/runs/` for verification only.

**Out of scope**:

- Do not reintroduce MCAP.
- Do not add `protoc` or protobuf runtime dependencies to the controller.
- Do not remove `schema.json`; it still carries RDF/provenance/semantic metadata
  beyond protobuf field names.
- Do not change the control loop/ring-buffer policy.
- Do not change DSL, metamodels, RDF emission, or model syntax.
- Do not create a second `frame_log_view.pb`; the actual `frame_log.pb` must use
  the generated semantic proto.

## Git workflow

- Work in the `src/motion-spec` git repository.
- Branch suggestion: `advisor/001-semantic-frame-log-protobuf`.
- Commit message style in recent history is imperative sentence case, for example:
  `Stop archiving frame layout JSON`.
- Do not push or open a PR unless instructed.

## Design decisions

Use deterministic field-number ranges. Keep core fields stable:

| Range | Meaning |
|---|---|
| `1..99` | core timing/FSM fields and `FrameLogHeader`/`FrameLogRecord` oneof tags |
| `1000 + index` | constraint slots |
| `2000 + index` | monitor slots |
| `3000 + index` | quantity slots |
| `4000 + index` | trigger slots |
| `5000 + index` | pose slots |
| `6000 + index` | twist slots |
| `7000 + index` | wrench slots |

Generate field names by sanitizing schema IDs to valid protobuf identifiers:

- lower-case ASCII
- replace every non-alphanumeric character with `_`
- collapse repeated `_`
- strip leading/trailing `_`
- prefix with the category if needed to avoid invalid names or ambiguity
- if the first character is a digit, prefix with `field_`
- if a duplicate occurs, append `_2`, `_3`, etc.

Examples:

- `direction_ctrl_cg_support_z.x` -> `direction_ctrl_cg_support_z_x`
- `home_pose` -> `home_pose`
- `wrench_force_ctrl_pk_support_z` -> `wrench_force_ctrl_pk_support_z`

STOP if any generated field number exceeds protobuf's maximum allowed field
number `536870911`, or if a generated field name cannot be made unique.

## Steps

### Step 1: Add semantic protobuf field metadata generation

In `src/motion-spec/src/motion_spec/introspection/artifacts.py`, add helpers
near `build_frame_layout`:

- `PROTO_FIELD_BASES = {"constraints": 1000, "monitors": 2000, ...}`
- `_proto_field_name(value: str, used: set[str], fallback: str) -> str`
- `build_frame_log_proto_fields(schema: dict) -> dict`

The returned mapping should be stored in `schema["protobuf"]` before
`schema_hash` is computed, so the frame-log header hash changes when the wire
field mapping changes.

Target shape:

```json
"protobuf": {
  "runtime_frame": "RuntimeFrame",
  "fields": {
    "constraints": [
      {"index": 0, "id": "constraint_0", "name": "constraint_0", "number": 1000}
    ],
    "quantities": [
      {"index": 0, "id": "direction_ctrl_cg_support_z.x", "name": "direction_ctrl_cg_support_z_x", "number": 3000}
    ],
    "poses": [
      {"index": 2, "id": "home_pose", "name": "home_pose", "number": 5002}
    ]
  }
}
```

For constraints and monitors, prefer semantic IDs from `schema["by_state"]`
when available. Because those slots are reused per state and only have a fixed
slot index in the runtime frame, the field name should be slot-stable, such as
`constraint_0` and `monitor_0`, unless there is a globally unambiguous ID for
that slot. Do not generate state-specific field names for reused slot indexes.

**Verify**:

```bash
cd src/motion-spec
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest tests/test_introspection_artifacts.py
```

Expected: tests pass, or fail only because assertions need to be updated for
the new `schema["protobuf"]` key.

### Step 2: Generate `frame_log.proto` from schema

In `artifacts.py`, add `build_frame_log_proto(schema: dict) -> str` and call it
from `write_introspection_artifacts()` to write:

```text
<output_dir>/frame_log.proto
```

Keep common message definitions:

```proto
message FrameLogHeader { ... }
message ConstraintSlot { ... }
message MonitorSlot { ... }
message Trigger { ... }
message PoseSlot { ... }
message TwistSlot { ... }
message WrenchSlot { ... }
message FrameLogRecord { oneof record { FrameLogHeader header = 1; RuntimeFrame frame = 2; } }
```

Generate `RuntimeFrame` with core fields plus semantic slot fields:

```proto
message RuntimeFrame {
  double t = 1;
  uint64 step = 2;
  sfixed64 fsm_state = 3;
  sfixed64 active_motion = 4;
  sfixed64 last_event = 5;
  double state_since_t = 6;
  sfixed64 state_since_wall_ns = 7;
  double event_t = 8;
  sfixed64 event_wall_ns = 9;
  sfixed64 wall_ns = 10;
  sfixed64 period_ns = 11;
  sfixed64 compute_ns = 12;
  sfixed64 trigger_count = 17;
  ConstraintSlot constraint_0 = 1000;
  MonitorSlot monitor_0 = 2000;
  double direction_ctrl_cg_support_z_x = 3000;
  Trigger trigger_0 = 4000;
  PoseSlot home_pose = 5002;
  TwistSlot twist_ee_base = 6000;
  WrenchSlot wrench_force_ctrl_pk_support_z = 7003;
}
```

Important: singular fields are acceptable here because each generated field
number corresponds to one slot. Do not use `repeated` for semantic slots.

Remove `motion_spec = ["introspection/frame_log.proto"]` from package-data only
after confirming no code path still needs the static proto. If keeping the
static file temporarily reduces risk, leave it in package-data but ensure
archives copy the generated proto from `source_dir / "frame_log.proto"`.

**Verify**:

```bash
cd src/bdd_collab_bhv_cpp/models
make build MODEL=pick_place_single
test -f gen/pick_place_single/frame_log.proto
grep -n "direction_ctrl_cg_support_z_x" gen/pick_place_single/frame_log.proto
grep -n "home_pose" gen/pick_place_single/frame_log.proto
```

Expected: build succeeds, proto exists, and semantic field names appear.

### Step 3: Archive the generated proto, not the static package proto

In `src/motion-spec/src/motion_spec/introspection/archive.py`, change
`create_archive_manifest()` so `copies` maps:

```python
"frame_log.proto": "contract/frame_log.proto"
```

and no longer uses:

```python
str(Path(__file__).with_name("frame_log.proto"))
```

Keep `HASHED_ARTIFACTS["frame_log_proto"] = "contract/frame_log.proto"` and
`manifest["files"]["frame_log_proto"] = "contract/frame_log.proto"`.

Update `_validate_new_run()` in `runner.py` to require `source_dir / "frame_log.proto"`.

**Verify**:

```bash
cd src/motion-spec
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest tests/test_introspection_archive.py tests/test_introspection_runner.py
```

Expected: tests pass after updating fixtures to create a generated proto in
their source tree.

### Step 4: Update the Python protobuf encoder and decoder

In `frame_log_pb.py`, update `frame_record(flat, schema)` to use
`schema["protobuf"]["fields"]`:

- core fields stay `1..12` and `17`.
- constraints use each entry's generated `number`.
- monitors use each entry's generated `number`.
- quantities use each entry's generated `number`.
- triggers use each entry's generated `number`.
- poses/twists/wrenches use each entry's generated `number`.

For decode, replace hardcoded field checks `13`, `14`, `15`, `16`, `18`, `19`,
`20` with reverse maps from generated field number to category/index.

The public decoded record shape should remain compatible:

```json
{
  "timing": {"wall_ns": 0, "period_ns": 0, "compute_ns": 0},
  "constraints": [...],
  "monitors": [...],
  "quantities": {"semantic_id": 0.0},
  "triggers": [...],
  "poses": [...],
  "twists": [...],
  "wrenches": [...]
}
```

Backwards compatibility with old array-field archives is optional. If it is
cheap, support both by falling back to old field tags when `schema["protobuf"]`
is missing. If it complicates the implementation, do not support old archives,
but update tests and docs to make the migration explicit.

**Verify**:

```bash
cd src/motion-spec
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest tests/test_introspection_archive.py tests/test_runtime_graph.py
```

Expected: tests pass and `decode_frames()` still returns named quantities.

### Step 5: Update the generated C++ writer

In `code-generator/module.stg`, change `write_frame()` so it no longer writes
model-specific slots with array tags `13`, `14`, `15`, `16`, `18`, `19`, `20`.

The generated output should look conceptually like:

```cpp
append_constraint(1000, frame.constraints[0]);
append_monitor(2000, frame.monitors[0]);
append_double(payload_, 3000, frame.quantities[0]);
append_trigger(4000, frame.triggers[0]);
append_pose(5002, frame.poses[2]);
append_twist(6000, frame.twists[0]);
append_wrench(7003, frame.wrenches[3]);
```

Change helper signatures from fixed payload field numbers:

```cpp
void append_pose(const PoseSlot &slot) { ... append_bytes(payload_, 18, ...); }
```

to:

```cpp
void append_pose(std::uint32_t field, const PoseSlot &slot) {
    ...
    append_bytes(payload_, field, slot_.data(), slot_.size());
}
```

Then generate one line per schema field using
`introspection_artifacts.frame_layout.protobuf.fields.<category>`.

If StringTemplate access to nested dictionaries becomes awkward, adjust the
payload returned by `write_introspection_artifacts()` to include a flat list
such as:

```json
"protobuf_writes": [
  {"kind": "quantity", "number": 3000, "index": 0},
  {"kind": "pose", "number": 5002, "index": 2}
]
```

**Verify**:

```bash
cd src/bdd_collab_bhv_cpp/models
make build MODEL=pick_place_single
grep -n "append_double(payload_, 3000" gen/pick_place_single/introspection_runtime.hpp
grep -n "append_pose(500" gen/pick_place_single/introspection_runtime.hpp
grep -n "append_double(payload_, 15" gen/pick_place_single/introspection_runtime.hpp && false || true
```

Expected: build succeeds, generated writer contains semantic field numbers,
and old quantity tag `15` is gone from slot writing. It may still appear in
comments or unrelated code only if justified.

### Step 6: Update tests for semantic protobuf

Add or update tests in `src/motion-spec/tests/`:

- `test_introspection_artifacts.py`: assert `schema["protobuf"]` exists, field
  names are valid protobuf identifiers, field numbers are unique, and generated
  `frame_log.proto` contains semantic names.
- `test_introspection_archive.py`: assert the archived `contract/frame_log.proto`
  is copied from source output and contains a test quantity field name.
- `test_runtime_graph.py`: keep runtime graph recovery passing from the semantic
  wire format.
- `tests/frame_log_fixture.py`: write fixture logs through the new semantic
  `frame_record(flat, schema)`.

Use existing fixture helpers rather than creating a separate test-only proto.

**Verify**:

```bash
cd src/motion-spec
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest tests
```

Expected: all tests pass.

### Step 7: Verify end-to-end archive self-explanation

Run a full model and inspect the archive:

```bash
cd src/bdd_collab_bhv_cpp/models
$GRC_SCRIPT_RUN pick_place_single
latest=$(ls -td runs/pick_place_single-* | head -1)
test -f "$latest/contract/frame_log.proto"
test -f "$latest/logs/frame_log.pb"
grep -n "direction_ctrl_cg_support_z_x" "$latest/contract/frame_log.proto"
grep -n "home_pose" "$latest/contract/frame_log.proto"
python -m motion_spec.introspection.replay "$latest"
```

Expected:

- run exits 0
- replay summary reports final state `S_DONE`
- health reports `dropped 0`
- generated proto contains semantic field names
- archive has no `contract/frame_layout.json`

If `protoc` is installed, also verify one delimited record using a splitter.
Because `frame_log.pb` is a stream of length-delimited protobuf messages,
plain `protoc --decode ... < logs/frame_log.pb` will not decode the whole file
as-is. Add a small documented helper only if necessary, for example a Python
CLI option to emit the first raw `FrameLogRecord` message without the length
prefix. Do not make this helper part of the hot path.

## Test plan

New/updated test cases:

- Generated proto field naming:
  - IDs with dots become underscores.
  - duplicate sanitized names receive suffixes.
  - numeric-leading names receive `field_` prefix.
  - field numbers are unique and in the intended ranges.
- Python encode/decode:
  - fixture frame encoded with semantic field numbers decodes to the same
    canonical record shape as before.
  - quantities are still keyed by semantic IDs in `decode_frames()`.
- Archive:
  - `contract/frame_log.proto` is present.
  - archived proto contains a semantic field from the model fixture.
  - `contract/frame_layout.json` is absent.
- Generated C++:
  - `pick_place_single` generated `introspection_runtime.hpp` writes semantic
    field numbers.
  - old repeated array tags are not used for model-specific slots.

Full verification:

```bash
cd src/motion-spec
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest tests
cd ../bdd_collab_bhv_cpp/models
make build MODEL=pick_place_single
$GRC_SCRIPT_RUN pick_place_single
```

Expected: tests pass, build succeeds, run reaches `S_DONE`, replay works.

## Done criteria

All must hold:

- [ ] `gen/<model>/frame_log.proto` is generated from that model's schema.
- [ ] `runs/<run-id>/contract/frame_log.proto` is the generated proto, not the
      static package proto.
- [ ] `logs/frame_log.pb` uses semantic field numbers for quantities,
      constraints, monitors, triggers, poses, twists, and wrenches.
- [ ] `python -m motion_spec.introspection.replay <run>` still prints a valid
      summary and can recover `runtime.ttl`.
- [ ] `contract/frame_layout.json` is not archived.
- [ ] `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest tests` exits 0 in `src/motion-spec`.
- [ ] `make build MODEL=pick_place_single` exits 0.
- [ ] A full `$GRC_SCRIPT_RUN pick_place_single` exits 0 and reports zero drops.
- [ ] `grep -R "mcap\\|MCAP\\|MOTION_SPEC_MCAP\\|mcap/writer" -n code-generator src/motion_spec tests pyproject.toml || true` returns no matches.
- [ ] `plans/README.md` status row updated.

## STOP conditions

Stop and report if:

- A generated field name collision cannot be resolved deterministically.
- Any generated field number would exceed protobuf's legal field-number range.
- The StringTemplate writer cannot access the generated protobuf mapping without
  adding brittle string parsing.
- The change requires adding C++ protobuf runtime dependencies.
- The control-loop hot path would need dynamic schema lookup, heap allocation
  per field beyond current `std::string` payload construction, or file I/O.
- Existing replay/runtime graph tests require changing the public decoded
  record shape instead of only the wire mapping.
- `pick_place_single` log size or period max regresses materially compared with
  the previous full-protobuf run: previous reference was 22,308 frames, zero
  drops, mean period about 1.073 ms, max period about 2.462 ms.

## Maintenance notes

- Protobuf field numbers are now part of the run's wire contract. Reviewers
  should scrutinize the field-number generation and ensure it is deterministic.
- `schema.json` remains necessary for RDF/provenance, FSM state metadata, units,
  and semantic IDs. Do not delete it just because field names are present in
  `frame_log.proto`.
- If future work adds a new frame slot category, it must reserve a new field
  number range and update both the proto generator and C++ writer generator.
- If external users start relying on generated field names, changing the
  sanitizer becomes a compatibility break for new runs.
