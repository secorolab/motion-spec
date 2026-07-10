# Plan 002: Serialize frame logs with a real protobuf library

> **Executor instructions**: Follow this plan step by step. Run every
> verification command and confirm the expected result before moving on. If a
> STOP condition occurs, stop and report. When done, update the status row in
> `plans/README.md`.

## Status

- **Priority**: P2
- **Effort**: L
- **Risk**: HIGH
- **Depends on**: none (supersedes the hand-rolled codec from plan 001)
- **Category**: tech-debt / correctness

## Why this matters

The frame log is standard protobuf **wire format**, but both the encoder (the
generated C++ writer) and the decoder (`frame_log_pb.py`) are hand-rolled —
~230 lines reimplementing varints, wire types, length-delimiting, and
field-by-field parsing. That is a reinvented standard library: a maintenance
and correctness liability that grows every time a frame slot category is added.
The per-run `frame_log.proto` already exists as the source of truth; nothing but
inertia stops `protoc` from owning the wire format on both sides.

The real-time argument for hand-rolling does **not** hold: the control loop only
`push`es a `Frame` struct into a ring buffer (a memcpy); serialization runs on a
separate writer thread (`FrameLogger::drain` → `write_frame`), off the hot path.
A protobuf library on that thread is fine.

This plan replaces the hand-rolled encode/decode with a protobuf library, keeps
the generated semantic `.proto` as the source of truth, and preserves the
decoded record shape that replay/runtime-graph depend on.

## Current state

- `src/motion_spec/introspection/artifacts.py` — `build_frame_log_proto_fields`
  derives semantic field names/numbers; the `frame_log_proto` StringTemplate
  renders the per-model `frame_log.proto`. **Keep both** — they feed `protoc`.
- `src/motion_spec/introspection/frame_log_pb.py` — hand-rolled varint/field
  encode (`frame_record`) and decode (`_fields`/`_parse_*`/`_parse_frame`).
- `code-generator/introspection.stg` — the `FrameLogger` class: `append_varint`,
  `append_key`, `append_bytes`, `append_string`, `append_uint`, `append_i64`,
  `append_double`, and `write_header`/`write_frame`/`append_<slot>` assembly.
- `code-generator/mj_kdl_backend.stg` (`cmake_mj_kdl`) — the generated CMake; no
  protobuf today.
- `src/motion_spec/codegen.py` — renders artifacts; would gain a `protoc` step.
- Consumers of the decoded record shape (MUST stay compatible): `replay.py`,
  `runtime_graph.py`, `tests/frame_log_fixture.py`, `test_introspection_*`.

## Design decisions

**Source of truth**: the generated `contract/frame_log.proto` stays. `protoc`
compiles it for C++ (writer) and Python (reader). `build_frame_log_proto_fields`
and the `.proto` template are unchanged.

**C++ writer — full libprotobuf** (decided). At codegen time, `protoc --cpp_out`
compiles the per-model `frame_log.proto` → `frame_log.pb.{h,cc}`; the generated
controller compiles it and links `protobuf::libprotobuf`. `write_frame`
populates a `RuntimeFrame` message and calls `SerializeToString`. Full (not
`-lite`) for the complete, familiar API; the extra binary size is acceptable for
now. `protobuf-lite` or nanopb remain a later optimization if a lean generated
binary becomes a requirement — deferred, not chosen.

**Python reader**: parse the archived `frame_log.proto` into a descriptor with
`protoc --descriptor_set_out` (or `grpcio-tools`), build the message class
dynamically via `descriptor_pool` + `message_factory` — no generated `_pb2.py`
to vendor. Decode length-delimited `FrameLogRecord`s and map to the **same
canonical record** (`timing`, `constraints`, `monitors`, `quantities` keyed by
semantic id, `triggers`, `poses`, `twists`, `wrenches`).

**Wire compatibility**: same field numbers + wire types (`sfixed64`/`double` =
wire type 1, `uint64` = varint) ⇒ output stays decodable by `protoc --decode`
and any protobuf library. Field **order** may differ from the hand-rolled writer
(protobuf serializes ascending by number), so new logs are valid protobuf but
**not byte-identical** to old ones — acceptable for new runs; note it.

**Dependencies (new)**: Python `protobuf`; C++ `protoc` + `libprotobuf` (full).
These must be added to `grc_meta` (Dockerfile apt `protobuf-compiler
libprotobuf-dev`, and `script-setup`'s package check + pip list) — this is the
protobuf setup that is intentionally absent today.

## Scope

**In scope**: `frame_log_pb.py`, `codegen.py` (protoc invocation), `introspection.stg`
(writer), `mj_kdl_backend.stg` (`cmake_mj_kdl` link/compile), `code-generator/CMakeLists.txt`,
`tests/*`, and `grc_meta` (Dockerfile + `script-setup`).

**Out of scope**: the semantic field-name/number scheme (keep), the ring-buffer /
drop policy, the `.proto` template output, schema.json.

## Steps

1. **Add the toolchain deps.** Python `protobuf` to `pyproject.toml`; `grc_meta`
   Dockerfile + `script-setup` gain `protobuf-compiler`/`libprotobuf-dev`.
   Verify `protoc --version` and `python -c "import google.protobuf"`.
2. **Codegen: run protoc.** In `codegen.py`, after writing `frame_log.proto`, run
   `protoc --cpp_out=<gen>` (option A) on it. Verify `gen/<model>/frame_log.pb.{h,cc}`.
3. **C++ writer.** Replace `FrameLogger`'s hand-rolled `append_*`/`write_frame`
   with: build a `motion_spec::introspection::RuntimeFrame`, set each semantic
   field from the `Frame` struct, `SerializeToString`, length-delimit (keep the
   varint length prefix helper OR use `SerializeDelimitedToOstream`). Drop
   `append_varint`/`append_key`/`append_bytes`/`append_string`/`append_uint`/
   `append_i64`/`append_double`. Update `cmake_mj_kdl` +
   `code-generator/CMakeLists.txt` to compile `frame_log.pb.cc` and link
   `protobuf::libprotobuf`.
4. **Python reader.** Replace `frame_log_pb.py` encode/decode: load the run's
   `.proto` descriptor, parse `FrameLogRecord`s, emit the canonical record.
   Delete `_varint`/`_read_varint`/`_key`/`_*_field`/`_slot`/`_fields`/`_parse_*`.
5. **Fixtures/tests.** `frame_log_fixture.py` builds fixture logs via the message
   class (skip if `protoc` absent, like the `stst` skip). Update assertions.
6. **Verify end-to-end.** Full `pick_place_single` run: reaches `S_DONE`, **zero
   drops**, replay recovers `runtime.ttl`, `protoc --decode` reads a record.

## Test plan

- Round-trip: a fixture `RuntimeFrame` encodes (C++/Python) and decodes to the
  same canonical record; quantities keyed by semantic id.
- Archive/replay/runtime-graph tests pass unchanged (record shape preserved).
- `protoc --decode motion_spec.introspection.FrameLogRecord < first-record` works.

## Done criteria

- [ ] No hand-rolled varint/wire code remains in `frame_log_pb.py` or `introspection.stg`.
- [ ] Generated controller compiles `frame_log.pb.cc` and links protobuf.
- [ ] `pytest tests` green; `make build MODEL=pick_place_single` green.
- [ ] Full run reaches `S_DONE` with **zero drops**; period max does not regress
      materially vs the plan-001 baseline (mean ~1.0 ms, drops 0, 22,308 frames).
- [ ] `grc_meta` Dockerfile + `script-setup` install `protobuf-compiler`/`libprotobuf-dev`.
- [ ] `plans/README.md` row updated.

## STOP conditions

- The writer thread can't keep up (drops appear) with library serialization —
  reconsider (reuse a single `RuntimeFrame` with `Clear()`, arena allocation)
  before proceeding.
- protoc is unavailable in CI/build and cannot be added — report; do not silently
  fall back to the hand-rolled codec.
- The decoded record shape would have to change (replay/runtime-graph break) —
  stop; only the wire mechanism should change, not the record contract.
