=============================
Run one generation repeatedly
=============================

This page continues from :doc:`end-to-end`. It assumes
``generation/pick-place`` already contains ``generated/`` and ``build/main``.

Why generation and run IDs differ
=================================

A generation identifies one authored source snapshot and its derived RDF, IR,
C++, contracts, provenance, and build. A run identifies one execution of that
build. Changing the model requires a new generation; repeating an experiment with
the same build requires only a new run ID.

.. graphviz::
   :caption: One immutable generation owns multiple runtime records.

   digraph reuse {
     rankdir=LR;
     graph [bgcolor="white", nodesep=0.35];
     node [shape=box, style="rounded", fontname="sans-serif"];

     generation [label="generation/pick-place\ngenerated/ + build/"];
     run1 [label="runs/run-1\nlogs + REC + runtime RDF"];
     run2 [label="runs/run-2\nlogs + REC + runtime RDF"];
     run3 [label="runs/run-3\nlogs + REC + runtime RDF"];

     generation -> run1;
     generation -> run2;
     generation -> run3;
   }

Create another run
==================

.. code-block:: console

   $ GENERATION_DIR=generation/pick-place
   $ motion-spec run "$GENERATION_DIR" --run-id run-2 --headless

Only ``runs/run-2`` is new. Its manifest references the existing generation
artifacts relatively; the source, model, controller, contracts, provenance, and
executable are not copied into every run.

Pass runtime arguments
======================

Everything after ``--`` belongs to the executable, not to the CLI:

.. code-block:: console

   $ motion-spec run "$GENERATION_DIR" --run-id run-3 -- --headless --steps 20000

Each invocation creates a fresh run directory. This preserves each REC lifecycle
and prevents accidental log replacement.

When to generate again
======================

Create a new generation when the ``.robmot``, scene, FSM, code-generation inputs,
or target configuration changes. Do not create a new generation merely to repeat
an unchanged controller execution.
