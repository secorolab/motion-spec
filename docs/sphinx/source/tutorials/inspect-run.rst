====================
Validate and inspect
====================

This page continues from :doc:`end-to-end` and uses the generation it made, at
``$GENERATION_DIR``.

Validate the RDF dataset
========================

The application manifest imports the motion, scene, FSM, provenance, and SHACL
graphs. Validate the complete dataset before treating it as a release artifact:

.. code-block:: console

   $ motion-spec check \
       "$GENERATION_DIR/generated/model/pick_and_place-app.ld.json"

Add ``--meta-shacl`` when the shape graph itself must also be validated against
SHACL-of-SHACL.

Stop at one lowering boundary
=============================

The default ``gen`` stage produces IR and C++. The ``ir`` stage stops after the
RDF and the IR, which is the boundary to inspect when the IR looks wrong:

.. code-block:: console

   $ motion-spec gen ir "$MODEL_PATH" -o scratch

It writes ``generated/model/ir.json`` and nothing under ``generated/controller/``.
Neither stage configures CMake, runs an executable, or creates a recorded run.

Summarize and verify a run
==========================

.. code-block:: console

   $ RUN_DIR="$GENERATION_DIR/runs/run-1"
   $ motion-spec replay "$RUN_DIR"
   $ motion-spec replay "$RUN_DIR" --verify

Summary mode reports the run without rewriting it. Verification requires the
run's ``manifest.json``: it checks that every file the manifest names exists, that
the log's schema hash matches the generation's frame layout, and that the log
holds as many frames as the writer reported. A run recorded before
``manifest.json`` was written (e.g. one killed mid-run) can still be summarized
and decoded, but not verified.

Stream decoded frames
=====================

.. code-block:: console

   $ motion-spec replay "$RUN_DIR" --jsonl > scratch/frames.jsonl

JSON Lines is intended for external whole-frame analysis. Normal summary,
verification, and runtime-RDF recovery do not require exporting this duplicate.
