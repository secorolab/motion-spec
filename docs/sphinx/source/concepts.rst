========
Concepts
========

Pipeline
========

The high-level pipeline crosses an explicit project boundary:

.. graphviz:: _static/pipeline.dot
   :caption: Commands, ownership, and the reusable generation boundary.

`motion-spec-dsl <https://github.com/secorolab/motion-spec-dsl>`_ defines the
``.robmot`` language, composes its scene and FSM imports, resolves DSL semantics,
owns the model classes, RDF vocabulary and manifest validation, and emits the
JSON-LD RDF dataset. ``motion-spec`` consumes that boundary: its RDF parser lowers
the combined dataset into compiler IR, after which code generation and runtime
tooling take over. It does not define or parse the authoring language itself.

The high-level ``motion-spec gen`` and ``motion-spec run MODEL`` commands invoke
the installed DSL frontend before continuing through their own stages.
``motion-spec build`` compiles one generation. ``motion-spec run`` takes either a
model, which it generates and builds first, or an existing generation directory.
``motion-spec replay`` consumes the resulting run.

Generations and runs
====================

A generation is immutable input plus reusable generated and compiled artifacts.
It receives its own directory because separate generations may differ even when
they came from the same model. Without ``-o``, ``gen`` and ``run`` create a unique
directory under ``generation/``.

A generation may be executed many times. Each execution belongs under that
generation's ``runs/<run-id>/`` directory. Runs contain runtime data only; they
reference generation-owned model, controller, contract, and provenance files
instead of copying them.

.. code-block:: text

   generation/<generation-id>/
     generated/
       source/       authored DSL snapshot
       model/        application JSON-LD, imported graphs, FSM IR, motion IR
       controller/   generated C++ and CMake project
       contract/     schema, frame layout, and frame-log protocol
       provenance/   DSL, coordinate, and motion-spec provenance
     build/           reusable compiled controller
     runs/<run-id>/
       logs/          frame_log.pb and health information
       runtime/       recovered runtime.ttl
       rec.ld.json    REC lifecycle and runtime artifact provenance
       manifest.json consumer-facing paths into the run and generation

JSON-LD artifacts use the ``.ld.json`` suffix and are loaded as RDF datasets,
not treated as application JSON.

High-level commands
===================

.. list-table:: Model workflow
   :header-rows: 1

   * - Command
     - Contract
   * - ``health``
     - Report Python, executable, and target-native dependencies
   * - ``gen [code] MODEL``
     - Generate JSON-LD, IR, contracts, provenance, and C++
   * - ``gen ir MODEL``
     - Stop after JSON-LD and IR
   * - ``build GENERATION``
     - Configure and compile into ``GENERATION/build``
   * - ``run MODEL``
     - Create a generation, build it, execute it, recover runtime RDF, and archive the run
   * - ``run GENERATION``
     - Execute an existing generation again, as a new run beside the earlier ones
   * - ``replay RUN``
     - Summarize, verify, decode, or recover a recorded run

Use ``--prefix`` on ``build`` or ``run`` to add CMake package prefixes. It is
repeatable for ``build``. ``-o``, ``--prefix``, and ``-j`` configure generation
and build, so they require a ``.robmot`` model; ``--headless``, ``--steps``, and
``--run-id`` apply to both forms, and ``--steps`` requires ``--headless``.

Low-level commands
==================

The individual stages remain available for automation and debugging:

.. list-table:: Stage commands
   :header-rows: 1

   * - Command
     - Input and output
   * - ``check MANIFEST``
     - Validate an application JSON-LD manifest and its imported graphs with SHACL
   * - ``ir MANIFEST``
     - Lower an application manifest to IR; write with ``-o`` or print with ``--console``
   * - ``codegen IR -o DIR``
     - Render C++ and runtime contracts using STST
   * - ``archive RUN_DIR``
     - Create a standalone archive from existing generated artifacts and a frame log, or verify one

The high-level model commands call these capabilities through the same Python
implementation; they do not maintain a second pipeline.

Validation
==========

``check`` resolves the application manifest's RDF imports and SHACL shapes. Pass
``--meta-shacl`` to validate the shapes themselves against SHACL-of-SHACL. A
successful generation does not replace an explicit validation step when validation
is part of a release or CI contract.

Runtime recording
=================

``run`` creates ``rec.ld.json`` before launching the executable and exports the
run ID, REC path, and frame-log path to it. The log starts with a header containing
the schema hash, followed by one protobuf frame per control tick. On completion,
the CLI catalogs run-owned files, records their provenance in REC, optionally
recovers ``runtime.ttl``, and verifies the archive.

The manifest is deliberately small: its ``files`` map identifies what replay and
other consumers need. Runtime artifact hashes and lifecycle provenance live in
``rec.ld.json``. Static code-generation provenance is consolidated in
``generated/provenance/motion-spec.ld.json``.

Replay
======

With no option, ``replay`` prints a run summary. ``--verify`` checks manifest
files and the frame-log header against the generation contract.
``--recover-runtime-ttl`` rebuilds runtime RDF from the frame log, and ``--jsonl``
streams decoded frames for external analysis.
