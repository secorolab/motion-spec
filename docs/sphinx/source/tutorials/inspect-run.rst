====================
Validate and inspect
====================

This page continues from :doc:`end-to-end` and uses the artifacts under
``generation/pick-place``.

Validate the RDF dataset
========================

The application manifest imports the motion, scene, FSM, provenance, and SHACL
graphs. Validate the complete dataset before treating it as a release artifact:

.. code-block:: console

   $ motion-spec check \
       generation/pick-place/generated/model/pick_place_single-app.ld.json

Add ``--meta-shacl`` when the shape graph itself must also be validated against
SHACL-of-SHACL.

Re-run one lowering boundary
============================

The high-level ``gen`` command already produced IR and C++. The low-level commands
are for inspecting or debugging those boundaries independently:

.. code-block:: console

   $ mkdir -p scratch/controller
   $ motion-spec ir \
       generation/pick-place/generated/model/pick_place_single-app.ld.json \
       -o scratch/ir.json
   $ motion-spec codegen scratch/ir.json -o scratch/controller

``ir`` consumes an RDF application manifest and writes motion-spec IR. ``codegen``
consumes that IR and renders C++ plus runtime contracts. Neither command configures
CMake, runs an executable, or creates a recorded run.

Summarize and verify a run
==========================

.. code-block:: console

   $ RUN_DIR=generation/pick-place/runs/run-1
   $ motion-spec replay "$RUN_DIR"
   $ motion-spec replay "$RUN_DIR" --verify

Summary mode reports the run without rewriting it. Verification checks required
files and the manifest's PROV/REC provenance; the frame log carries its own
schema hash, so there is nothing external left to cross-check it against. A run
recorded before ``manifest.json`` was written (e.g. one killed mid-run) still
replays and verifies -- verification just falls back to the frame-log header.

Recover runtime RDF
===================

.. code-block:: console

   $ motion-spec replay "$RUN_DIR" --recover-runtime-ttl

This decodes the archived frame log and writes ``runtime/runtime.ttl``. The source
frame log and REC record remain the authoritative runtime record.

Stream decoded frames
=====================

.. code-block:: console

   $ motion-spec replay "$RUN_DIR" --jsonl > scratch/frames.jsonl

JSON Lines is intended for external whole-frame analysis. Normal summary,
verification, and runtime-RDF recovery do not require exporting this duplicate.
