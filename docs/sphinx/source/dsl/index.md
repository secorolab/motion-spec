# DSL authoring

[`motion-spec-dsl`](https://github.com/secorolab/motion-spec-dsl) defines the
`.robmot` authoring language and emits the RDF dataset consumed by `motion-spec`.
The DSL package also owns its model classes, RDF emission, vocabulary, manifest
handling, and semantic validation. `motion-spec` begins at RDF-to-compiler-IR
lowering.

Read [DSL concepts](concepts.md) for the complete language surface, [quantity
expressions](expressions.md) for arithmetic over quantity refs, then work through the
[`pick_place_single` tutorial](tutorials/single-arm.md).

```{toctree}
:maxdepth: 2

concepts
expressions
tutorials/index
```
