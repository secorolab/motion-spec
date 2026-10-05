========================
Generate, build, and run
========================

This page executes each stage separately first. That makes the input, output,
and ownership boundary of every command visible before using the combined
``motion-spec run MODEL`` shortcut.

Choose the model and generation
===============================

Run from the workspace root. ``motion-spec examples`` copies the models that ship
with motion-spec-dsl into ``src/ms-examples``, which is where ``MODEL_PATH``
points:

.. code-block:: console

   $ motion-spec examples
   $ MODEL_PATH=src/ms-examples/01_pick_and_place/pick_and_place.robmot

``MODEL_PATH`` is authored input. The generation directory, made in the next
step, will own the generated model, controller, contracts, provenance, build, and
every run made from that build.

Check the required toolchain
============================

.. code-block:: console

   $ motion-spec health --target mujoco

Resolve missing ``dsl`` or ``codegen`` checks before generation. Resolve MuJoCo
``build`` and ``runtime`` checks before the corresponding later stages.

1. Generate
===========

.. code-block:: console

   $ GENERATION_DIR=$(motion-spec gen "$MODEL_PATH" -o generation/pick-place)

``gen`` parses the ``.robmot`` model and its scene/FSM imports, emits JSON-LD,
lowers the RDF dataset to motion-spec IR, and generates C++ plus runtime contracts.
It does not compile or execute the controller. ``-o`` is a base directory: the
generation lands at ``generation/pick-place/pick_and_place/<timestamp>``, and
``gen`` prints that path on stdout.

The important outputs are:

.. code-block:: text

   generation/pick-place/pick_and_place/<timestamp>/
     generated/source/       authored input snapshot
     generated/model/        JSON-LD, FSM artifacts, and ir.json
     generated/controller/   generated C++ and CMake project
     generated/contract/     schema and frame-log contract
     generated/provenance/   generation and derivation provenance

To stop after RDF and IR, use a different base directory:

.. code-block:: console

   $ motion-spec gen ir "$MODEL_PATH" -o generation/pick-place-ir

An IR-only generation cannot be built because it has no generated controller.

2. Build
========

.. code-block:: console

   $ motion-spec build "$GENERATION_DIR" \
       --prefix /path/to/workspace/install

``build`` consumes ``generated/controller/`` and creates the reusable
``build/main`` executable. It does not regenerate the model and does not create a
run. Add another ``--prefix`` for each additional CMake package prefix when
required.

After this step, the same executable can be run any number of times without
regenerating or rebuilding.

3. Run the existing build
=========================

Point ``run`` at the generation directory:

.. code-block:: console

   $ motion-spec run "$GENERATION_DIR" --run-id run-1 --headless

For an existing build:

- the positional input is the generation directory, which already holds the
  model, contracts, and provenance under ``generated/`` and the built controller
  at ``build/main``;
- the run is created under ``runs/``, named by ``--run-id`` or by a timestamp;
- arguments after ``--`` are passed to the executable.

``run`` creates the REC lifecycle record, launches the controller with an
archive-local frame-log path, catalogs runtime outputs, and verifies the result.
It refuses to overwrite a run that already contains a REC record or frame log.

To run that same build again, without naming it:

.. code-block:: console

   $ motion-spec rerun

``gen`` and ``run`` point one ``latest`` symlink, under ``$MOTION_SPEC_GEN``, at
the generation they make, the way a colcon workspace carries ``log/latest``:

.. code-block:: text

   $MOTION_SPEC_GEN/latest  -> base_rotation/20260815T155542360467Z

``rerun`` follows it: it is ``run`` for the generation already built, under a
run id of its own, taking the same options. It generates and builds nothing, so
it is the command to reach for while tuning a deployment config or a scene the
controller reads at startup.

The link is an ordinary path, so it works with every other command too. To go
back to one model after another has been generated since, name its generation
directory instead.

4. Inspect the run
==================

.. code-block:: console

   $ motion-spec replay "$GENERATION_DIR/runs/run-1"
   $ motion-spec replay "$GENERATION_DIR/runs/run-1" --verify

The first command prints a summary. The second checks the files the manifest
names, the log's schema hash against the generation's frame layout, and its frame
count against what the writer reported.

Combined shortcut
=================

Once the separate stages are clear, the high-level form performs generation,
build, run, runtime RDF recovery, and verification in one command:

.. code-block:: console

   $ motion-spec run "$MODEL_PATH" \
       -o generation/pick-place-direct \
       --prefix /path/to/workspace/install \
       --run-id run-1 \
       --headless

Use this form for a new model generation. Pass the generation directory instead
when running one unchanged generation repeatedly.
