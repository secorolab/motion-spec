# TODO

## Motion-side IRI → slot lookup table (revisit — not needed now)

The FSM is table-driven (`enum` index ↔ `STATE_URIS[]`/`EVENT_URIS[]`), and that's
fine — it's a self-contained single header and the interpreter needs those tables to
run/report. The motion/controller code is deliberately *not* table-driven: variables are
named typed fields bound at codegen time, and the id/uri ↔ slot mapping lives in the
contract (`schema.json` / `frame_layout.json`, plus the generated `frame_log.proto` that
names each wire slot), consumed offline by replay/introspection/provenance tooling. That split is intentional (single source of truth, hot-path stays
integer-slot-keyed, no type erasure over heterogeneous controller types). We dropped the
old `uris.hpp` (a `#include`d-but-unused, ~5k-entry / <1k-unique id→uri table) accordingly.

**Revisit only if** a *runtime, in-process, IRI-keyed* need appears in C++ — e.g. an
external planner / RPC / live tuner that says "act on controller `<iri>`" and the C++ must
resolve it during the run (nothing does this today; everything is compile-time bound).

**If that happens**, generate a **deduped `id/uri → slot` table** (a few hundred entries),
NOT `IRI → typed variable`: map to the frame slot so the table stays homogeneous and
type-safe, and let the existing slot → variable binding do the rest. Guard it against the
dedup / dead-code rot that bit `uris.hpp` (emit unique entries only; only generate it when
a real C++ consumer exists — YAGNI until then).
