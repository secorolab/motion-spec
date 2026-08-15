========================
Generate, build, and run
========================

This tutorial executes each stage separately first. That makes the input, output,
and ownership boundary of every command visible before using the combined
``motion-spec run MODEL`` shortcut.

Choose the model and generation
===============================

Run from the workspace root and define two paths used throughout the tutorial:

.. code-block:: console

   $ MODEL_PATH=src/motion-spec-dsl/models/pick_place_single/pick_place_single.robmot
   $ GENERATION_DIR=generation/pick-place

``MODEL_PATH`` is authored input. ``GENERATION_DIR`` will own the generated model,
controller, contracts, provenance, build, and every run made from that build. It
must not already exist.

Check the required toolchain
============================

.. code-block:: console

   $ motion-spec health --target mujoco

Resolve missing ``dsl`` or ``codegen`` checks before generation. Resolve MuJoCo
``build`` and ``runtime`` checks before the corresponding later stages.

1. Generate
===========

.. code-block:: console

   $ motion-spec gen "$MODEL_PATH" -o "$GENERATION_DIR"

``gen`` parses the ``.robmot`` model and its scene/FSM imports, emits JSON-LD,
lowers the RDF dataset to motion-spec IR, and generates C++ plus runtime contracts.
It does not compile or execute the controller.

The important outputs are:

.. code-block:: text

   generation/pick-place/
     generated/source/       authored input snapshot
     generated/model/        JSON-LD, FSM artifacts, and ir.json
     generated/controller/   generated C++ and CMake project
     generated/contract/     schema and frame-log contract
     generated/provenance/   generation provenance

To stop after RDF and IR, use a different generation directory:

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

To do the same run again -- same arguments, same working directory -- without
restating them:

.. code-block:: console

   $ motion-spec rerun "$GENERATION_DIR"

Each run records how it was launched in ``runs/RUN/invocation.json``, and
``rerun`` repeats the most recent one into a run of its own. It generates and
builds nothing, so it is the command to reach for while tuning a deployment
config or a scene the controller reads at startup.

4. Inspect the run
==================

.. code-block:: console

   $ motion-spec replay "$GENERATION_DIR/runs/run-1"
   $ motion-spec replay "$GENERATION_DIR/runs/run-1" --verify

The first command prints a summary. The second verifies the manifest and the
frame-log header against the generation contract.

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
