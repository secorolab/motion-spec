# Template — adding a feature to the motion-spec pipeline

Use this when adding **any** new authoring construct (a constraint kind, view, quantity, handler option, controller, trajectory, monitor, …). It encodes the rules so they don't have to be re-explained each time. Copy the **Decision sheet** + **Layer checklist** into the feature's `plans/NNN-*.md` and fill them in.

Use the current numbered plans as examples when they exist; do not depend on a
specific feature plan being present.

---

## 0. Principles (the non-negotiables)

1. **Reuse first, add only if required.** Before inventing vocabulary, grep the metamodels and the DSL for an existing concept. New vocab is the last resort; when added, it lives in the **secorolab** layer (`src/metamodels/...`, `*-ext`), never in vendored `src/comp-rob2b/**`. Override upstream SHACL via `src/metamodels` + the prefix-redirect map in `rdf.py`.
2. **Flow through the existing machinery — don't bypass it.** A new construct that is "like X" must take the *same* path X takes (parse → RDF node → SHACL → ir → the same monitor/event/controller wiring). If you find yourself emitting a one-off literal and then *filtering it out* at every layer, stop — that's the wrong shape. Diverge only at the one point where the construct genuinely differs, and keep that divergence as small as possible.
3. **No hardcoded values in codegen.** Anything tunable (rates, periods, thresholds, gains, durations) is **authored in the model** and flows through the graph. If a system constant seems unavoidable, author it in the natural place (e.g. the `CONSTRAINT_HANDLER`) and make codegen read it.
4. **Pick the correct runtime source, not the convenient proxy.** Read from the real source (sensor, backend API, world state, sequencer state) rather than deriving it indirectly. Hardware/sim never matches the ideal on paper; leave the calibration knob.
5. **Use standard/native domain types before primitives.** In generated C++, prefer existing project/runtime types and standard/platform types for domain concepts. Raw primitives are allowed only for authored scalar values or when the source API itself is scalar, and the plan must say why.
6. **Keep the IR semantic and minimal.** No codegen concerns in the IR. Prefer one derived input over many flags. Reuse the namespace/aggregation machinery.
7. **No silent legacy fallback.** If the feature makes an option required, make it required in grammar, RDF/SHACL, IR, fixtures, and models in the same plan. Do not keep a magic default "for compatibility" unless the user explicitly chooses legacy support.
8. **Generated code is part of the contract.** The plan must state the C++ shape you expect (field, helper, condition, loop call) and the grep/excerpt that proves it. A green build is not enough if the generated code is semantically wrong.
9. **Match the surrounding code.** Naming concrete (name the real target, not an abstraction the tooling resolves), comment density minimal, group related scalars into value nodes. Shortest working diff.
10. **Every non-trivial change leaves one runnable check.** A DSL unit test or the `make` gate that fails if the logic breaks.

---

## 1. Decision sheet (answer before coding)

Fill these in the plan. If an answer is genuinely the user's call and you can't derive it, ask **with concrete previews** — getting surface syntax wrong is the most expensive mistake.

- **What existing construct is this most like?** (the analog whose path you'll reuse) → ____
- **Surface syntax** in `.robmot` (show the exact line, symmetric with the analog) → ____
- **Which block(s)/scope** does it live in (WHEN/WHILE/UNTIL, CONTEXT, ENVIRONMENT, handler)? Don't bind it narrower than needed → ____
- **Where does its value/config come from?** declared quantity, inline literal, both, or handler option? (no hardcodes) → ____
- **What is the runtime source** that produces its signal? (sensor via solver, backend state, world state, FSM flag, etc.) → ____
- **What exact runtime type/unit crosses the C++ boundary?** Prefer existing project/runtime types or native/standard types. If using a primitive, justify why the source is scalar and name the unit → ____
- **Is any new authoring field mandatory?** If yes, list every existing model/fixture that must be migrated; no fallback default → ____
- **What generated C++ excerpt should exist?** Field/helper/condition/loop snippet, not just "build succeeds" → ____
- **What existing helper/type/pattern are we reusing?** Name the local function/template/backend hook or standard API; don't invent a wrapper if one exists → ____
- **Reuse vs add** — see §2.
- **Failure/edge semantics** — what happens at the boundary (timeout, loss, missing value)?
- **Backend semantics** — what differs between `mj_kdl` and `robif2b`/real? (source availability, units, update cadence, failure behavior)
- **Open semantic questions / escape hatches** — anything to STOP-and-ask on rather than guess.

---

## 2. Metamodel verification (do this, then write a table in the plan)

Read the relevant shapes/contexts and classify every vocabulary term you need:

- Constraint shapes: `src/comp-rob2b/metamodels/task/constraint.{json,ttl}` (what a `cstr:Constraint`/`UnilateralConstraint`/… *requires*).
- Motion shapes: `src/metamodels/task/motion-specification.{json,shacl.ttl}`.
- Handler shapes: `src/comp-rob2b/.../constraint-handler.ttl` + `src/metamodels/task/constraint-handler-extension.{json,shacl.ttl}`.
- Quantities/roles: `qudt.json`, `src/metamodels/task/value-role.json`, and `QUDT_QKIND`/`QUDT_UNIT`/`MOT_EXT`/`CSTR*` in `src/motion-spec/.../namespace.py`.
- Don't conflate layers: vocabularies from validation/BDD/analysis layers are not automatically runtime motion concepts.

Produce three lists in the plan:

| Reused as-is | Must add (where) | Deliberately NOT added (why) |
|---|---|---|

Rules: reuse `value-role:` (`Measured`/`Declared`/`Reference`/`Computed`) to distinguish runtime-filled vs constant nodes — usually removes the need for a new marker. Standard external vocab (QUDT kinds/units) counts as "reuse" even if you must *declare* the term in the Python `DefinedNamespace` shim.

If a new SHACL property is required, do **one** of these explicitly:

- make it optional (`sh:minCount 0`) and document why legacy models are supported; or
- make it required and migrate **every** model/fixture plus add a missing-field test.

Do not add an optional shape and then make IR/codegen secretly assume a default.

---

## 3. Layer checklist (the pipeline, in order)

Data flow: `.robmot → textX (motion_spec.tx + domain.py + registration.py) → rdf.py → RDF graph (+SHACL) → ir_gen.py → codegen.py + module.stg → C++`. (See memory `motion-spec-codegen-pipeline`; C++ gen lives in motion-spec, not motion-spec-dsl.)

- [ ] **Grammar** `motion-spec-dsl/.../metamodels/motion_spec.tx` — add the rule/alternative. Mind PEG ordering (keywords before `<`/`[`; bare-literal alternatives last). Add units to `Unit`, types to `QuantityType`, etc.
- [ ] **Domain** `domain.py` — the `@dataclass` for the new rule (subclass `NamedNamespaceObject` if it has a name/namespace). Add enum members; update `_SCALAR_TYPES` for scalar quantity types.
- [ ] **Registration** `registration.py` — add any new Python class to `LANGUAGE_CLASSES` (textX won't bind it otherwise).
- [ ] **RDF emission** `rdf.py` — emit the graph node(s) through the analog's path. Add unit→QUDT in `DSL_UNIT`, type→kind/unit in the `QUDT_KIND_BY_QUANTITY_TYPE` / unit maps. Tag `value-role`. Branch only where the construct truly differs.
- [ ] **Validation** `motion-spec-dsl/.../validation/*.py` — handle the new shape in the view/quantity/ref checks; add to `_types_match` compatibility; don't let it trip generic resolution rules.
- [ ] **Metamodels** `src/metamodels/...` — JSON-LD `@context` term(s) + SHACL property/shape (secorolab/`-ext` layer; optional unless migrating all models). Never edit `comp-rob2b`.
- [ ] **Namespace** `motion-spec/.../namespace.py` — declare new predicates/kinds/units in the `DefinedNamespace` (`_extras`).
- [ ] **IR** `ir_gen.py` — read the graph; carry only semantic fields (no C++ strings). Detect the new construct from the graph (kind/role/type), not from DSL-only state.
- [ ] **Codegen** `codegen.py` — turn IR into the C++ condition/struct/call strings. Reuse the existing term builders (`add_*_monitor_conditions`, `add_motion_done_conditions`, schedule). Thread any authored config (no magic numbers). If a construct appears in multiple blocks (WHEN/WHILE/UNTIL), generate all of them or reject unsupported blocks explicitly with a test.
- [ ] **Template** `code-generator/module.stg` (+ `*_backend.stg`) — struct fields, entry/reset hooks, loop. Reuse existing runtime patterns and helper templates before adding new generated code. Per-backend (`mj_kdl`/`robif2b`) branches where runtime differs.
- [ ] **Model + FSM** `src/bdd_collab_bhv_cpp/models/*.{robmot,fsm}` — a test model exercising **every** authoring form you added. Don't edit vendored menagerie `.xml`; change code to match assets.
- [ ] **Migration pass** — if grammar/SHACL now requires a field, update all in-repo `.robmot` fixtures/examples/models in the same change and add one negative test for omission.

---

## 4. Verification gates

```bash
cd src/bdd_collab_bhv_cpp/models
make MODEL=pick_place_single.robmot                 # existing models still build (baseline)
make MODEL=pick_place_dual.robmot
make MODEL=<new_model>.robmot FSM=<new>.fsm          # new feature builds (SHACL runs here)
cd ../../motion-spec-dsl && python -m pytest tests/ -q
cd ../motion-spec        && python -m pytest tests/ -q
# then grep the generated C++ to confirm the intended code is present and old artifacts are gone
```
Run requires `INSTALL` + `METAMODELS_PATH` env (see memory `bdd-collab-bhv-run`).

Every feature plan must add a **Generated C++ inspection** block with commands like:

```bash
grep -R "<new helper/field/condition>" src/bdd_collab_bhv_cpp/models/gen/<model>/
grep -R "<old bad artifact>" src/bdd_collab_bhv_cpp/models/gen/<model>/ && false
```

If the feature is observable in simulation, add one run gate:

```bash
make MODEL=<new_model>.robmot FSM=<new>.fsm run-headless STEPS=<small smoke count>
# or, for visual behavior:
timeout 60s make MODEL=<new_model>.robmot FSM=<new>.fsm run
```

The plan should name the expected runtime trace/event/observable output, not just "viewer opens".

---

## 5. Anti-patterns (caught us before)

- A new construct emitted as a one-off literal that's then *filtered out* of the normal path at every layer. → Make it a first-class node on the existing path.
- Hardcoding a system rate/constant in `codegen.py`/`module.stg`. → Author it in the model (handler), flow it through.
- Adding a "temporary" default for a required authored field. → Make the field mandatory and migrate every model/fixture.
- Deriving runtime state from a convenient proxy when the backend exposes the real source. → Read the real source.
- Representing domain concepts as naked primitives without justification. → Use existing project/runtime types or native standard types; convert scalar authored quantities only at the boundary.
- Reimplementing a type/helper already in the codebase or standard library. → Reuse the local helper or stdlib/native API.
- Treating a green compile as proof. → Inspect generated C++ and run the representative model long enough to see the intended event/state.
- Editing `src/comp-rob2b/**` or vendored `.xml`. → Override in `src/metamodels`; change code, not assets.
- Adding a new metamodel class when `value-role` + an existing kind already distinguishes the case.
- Leaving a feature half-wired or silently downscoping when blocked. → Finish it, or surface the fork and ask (see memory `discuss-before-workarounds`).
