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
they came from the same model. A new generation goes to
``$MOTION_SPEC_GEN/<model stem or --name>/<timestamp>/``; without
``$MOTION_SPEC_GEN``, ``gen`` and ``run`` stop. ``gen`` prints the
generation's path on stdout, and ``$MOTION_SPEC_GEN/latest`` points at the newest.

A generation may be executed many times. Each execution belongs under that
generation's ``runs/<run-id>/`` directory. Runs contain runtime data only; they
reference generation-owned model, controller, contract, and provenance files
instead of copying them.

.. code-block:: text

   <base>/<model>/<timestamp>/
     generated/
       source/       authored DSL snapshot
       model/        application JSON-LD, imported graphs, scene and FSM graphs, motion IR
       controller/   generated C++ and CMake project
       contract/     schema, frame layout, and frame-log protocol
       provenance/
         generation.ld.json  one document, one named graph per tool
         derived.ld.json     what IR generation derived, and from what
     build/           reusable compiled controller
     logs/            console.log: what the DSL, stst and cmake said
     runs/<run-id>/
       logs/          frame_log.pb (zstd-packed to frame_log.pb.zst once archived): its
                      contract, the frames, and last the writer's health record;
                      console.log
       files/         the environment file the run was given
       provenance/
         <run-id>.ld.json    REC record: lifecycle and host
         execution.ld.json   who ran it, the files it used and generated, sampling draws

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
   * - ``rerun [GENERATION]``
     - ``run`` for the generation ``latest`` points at (or a named one); generates and builds nothing
   * - ``replay RUN``
     - Summarize, verify, decode, or recover a recorded run
   * - ``view SCENEX``
     - Compose a scene in MuJoCo and view it, without generating or building anything

Use ``--prefix`` on ``build`` or ``run`` to add CMake package prefixes. It is
repeatable for ``build``. ``--name``, ``--prefix``, and ``-j`` configure
generation and build, so they require a ``.robmot`` model; ``--run-id`` applies
to both forms. How a simulation runs -- headless, its steps, its pace, whether it
starts paused -- is the ``[run]`` section of the robot config its deployment declares.

Low-level commands
==================

The individual stages remain available for automation and debugging:

.. list-table:: Stage commands
   :header-rows: 1

   * - Command
     - Input and output
   * - ``check MANIFEST``
     - Validate an application JSON-LD manifest and its imported graphs with SHACL
   * - ``gen ir MODEL``
     - Stop after lowering to IR

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

``run`` creates the REC record ``provenance/<run-id>.ld.json`` before launching the executable
and exports the frame-log path to it. The run is recorded there as a
``prov-ext:Execution`` with an OSLC Automation state and, once complete, a verdict;
the agents that ran it and the files it used and generated go to
``provenance/execution.ld.json`` on the same run node. The log starts with a header containing the schema hash;
a model that samples follows it with the seed and every draw (``[sampling] seed`` in the robot
config fixes the seed, otherwise the OS draws one). One protobuf frame per control tick follows. On completion, the CLI catalogs
run-owned files, records their provenance, and verifies the archive; REC records the run's end
only after that, so an ended run is an archived one.

The provenance is the run's record: lifecycle in the REC record, the files the run used and
generated, with their hashes, in the execution document, and a run recorded without a frame
log is one whose execution document records none. Everything generation recorded lives in one document,
``generated/provenance/generation.ld.json``, with a ``prov:Bundle`` per tool that wrote into
it, attributed to that tool's package URL; the DSL's bundle stays in the model document it
generated. motion-spec's bundle records the generation as one activity, each tool acting on
its behalf in it (a qualified ``prov:Delegation``), and its own steps. A generated or run file
is a ``prov-ext:RelativeLocation`` under the generation directory, such as
``generated/model/ir.json``, with its hash as ``dcterms:identifier``. Each tool records the
authored sources it read where they are, with their hash and git revision; the generation keeps
a copy of each in ``generated/source/``, generated by the generation and ``prov:alternateOf``
the original.

Replay
======

With no option, ``replay`` prints a run summary. ``--verify`` checks every file the
run's provenance records against its hash, the provenance against the PROV and REC
shapes, the log's schema hash against the generation's ``frame_layout.json``, and the
log's frame count against what the writer reported. A run killed before it was
archived can still be summarized and decoded, but not verified. ``--jsonl`` streams decoded
frames for external analysis.
