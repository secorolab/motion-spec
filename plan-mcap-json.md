# mcap-JSON introspection log — bake-off implementation (DONE)

## Purpose
Evaluate the **existing standard (mcap)** vs the **bespoke `.bin`** format. `.bin` is
the control; `.mcap` is the candidate, written **live** in the generated controller so
mcap's live-write reliability is tested under the real path. Both coexist *for the test* —
the eval decides whether mcap replaces `.bin`.

## Decisions (final)
- **Container:** mcap, vendored **header-only** (`thirdparty/mcap/include/mcap/`, v2.0.0).
- **Encoding:** JSON messages (`message_encoding=json`) + JSON Schema
  (`schema_encoding=jsonschema`). Chosen because it keeps the controller **dependency-free**
  (mcap is header-only; JSON needs only `snprintf`), and it is natively decodable by
  Foxglove / `mcap` CLI. Raw-bytes-in-mcap is undecodable by any tool → rejected.
  Protobuf (compact, canonical) was rejected: not header-only, needs libprotobuf +
  protoc in every generated build — breaks the vendored/dep-free constraint. Revisit
  only if the bake-off shows JSON's size/speed actually hurts.
- **Message shape:** nested record, byte-for-byte matching `replay.to_record()`, so a
  decoded `.mcap` message == the decoded `.bin` frame (exact cross-check for the eval).
- **Single source of truth:** the JSON Schema is derived from the same field spec
  (`frame_layout_spec.HEADER/CSLOT/MSLOT/TSLOT`) that drives `.bin`.

## What changed
- `thirdparty/mcap/include/mcap/*` — vendored v2.0.0 headers (13 files) + `VERSION`.
- `frame_layout_spec.py` — `frame_json_schema()` (nested, pool-independent, `d`→number|null,
  `q`/`Q`→integer).
- `artifacts.py` — emits `frame_json_schema` (compact JSON string) into the frame_layout
  template context.
- `module.stg`
  - `frame_layout_header`: `kFrameJsonSchema` raw-string constant.
  - `introspection_runtime_header`: `#include <cmath>`; `json_double/i64/u64` emitters
    (non-finite doubles → `null`); `encode_frame_json()` builds the nested record into a
    reused `json_` buffer; `open_mcap` uses `jsonschema`/`json` + `Compression::None`
    (default is Zstd, compiled out) with chunking/index left on for a seekable file;
    `write_mcap` sends the JSON buffer. Trigger window mirrors `to_record` exactly.
- `mj_kdl_backend.stg` — corrected the FATAL_ERROR message to the vendored path.
- `tests/test_introspection_artifacts.py` — schema-vs-`to_record` drift guard.

## Verified
- `frame_json_schema()` is a valid draft-2020-12 schema; matches `to_record` output.
- `frame_layout_header` renders through StringTemplate with the schema as a raw string;
  generated header compiles as valid C++; embedded schema round-trips as JSON.
- Standalone probe compiled against the vendored headers, wrote a `.mcap`:
  `mcap doctor` clean, `mcap info` shows the `jsonschema` channel, `mcap cat` decodes it;
  NaN→null and Inf→null confirmed.
- `pytest` introspection suites: 13 passed.
- `mcap` CLI 0.2.0 installed at `~/.local/bin/mcap`.

## Remaining — the actual bake-off (needs a real run)
1. Codegen an example with `MOTION_SPEC_ENABLE_INTROSPECTION=ON`, build (`cbps`), run
   headless (`$GRC_SCRIPT_RUN --headless <model>`).
2. Cross-check reliability: decode `frame_log.mcap` and `frame_log.bin`, assert identical
   frame data; compare health.json `mcap_written_frames` vs `written_frames` (drops).
3. Compare size + write cost `.mcap` (JSON) vs `.bin` → the evidence to decide whether
   mcap replaces `.bin`, and whether JSON is enough or protobuf's dep is warranted.
