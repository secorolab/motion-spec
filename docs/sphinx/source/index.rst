===========
motion-spec
===========

``motion-spec`` validates, compiles, builds, runs, and inspects guarded robot
motion specifications. It accepts a ``.robmot`` model for the complete workflow
and retains lower-level RDF, IR, code-generation, and archive commands for tools
that need individual stages.

Quick start
===========

.. code-block:: console

   $ python -m pip install /path/to/motion-spec
   $ motion-spec install all
   $ motion-spec setup
   $ motion-spec health --target mujoco
   $ motion-spec run model.robmot -o generation/demo --run-id run-1 --headless
   $ motion-spec replay generation/demo/runs/run-1
   $ motion-spec run generation/demo --run-id run-2 --headless

Run ``motion-spec --help`` or ``motion-spec COMMAND --help`` for the exact
options supported by the installed version.

Where to go next
================

Start with :doc:`setup`, read :doc:`concepts` for the pipeline and artifact
contracts, then work through :doc:`tutorials/index` to generate, build, and run
one. Read :doc:`dsl/index` when you author a model of your own rather than
running an existing one.

.. toctree::
   :maxdepth: 2
   :caption: Guide

   setup
   concepts
   sim-real-parity
   loop-timing
   tutorials/index

.. toctree::
   :maxdepth: 2
   :caption: Authoring language

   dsl/index

.. toctree::
   :maxdepth: 2
   :caption: JSON-LD tutorial

   tutorial

.. toctree::
   :maxdepth: 1
   :caption: About

   acknowledgement
