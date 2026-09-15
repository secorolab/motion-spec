===========
motion-spec
===========

``motion-spec`` validates, compiles, builds, runs, and inspects guarded robot
motion specifications. It accepts a ``.robmot`` model for the complete workflow
and retains lower-level RDF, IR, code-generation, and archive commands for tools
that need individual stages.

Quick start
===========

``motion-spec`` needs the rest of what it builds against — the authoring DSLs,
the kinematics fork, the simulator wrapper — and installs all of it itself:

.. code-block:: console

   $ mkdir -p ws/src
   $ git clone git@github.com:secorolab/motion-spec.git ws/src/motion-spec
   $ python3 -m venv ws/.venv && source ws/.venv/bin/activate
   $ pip install -e ws/src/motion-spec            # the CLI and the DSL compilers
   $ motion-spec install all                      # every optional Python feature
   $ motion-spec health                           # the one apt line for what is missing
   $ motion-spec setup --workspace ws             # STST and the C++ libraries
   $ source ws/setup-motion-spec.bash             # or .zsh, whichever setup wrote

ROS is optional, and the robot hardware drivers are installed only if a model
binds them. Then run a model, replay the run it recorded, and run the same
generation again:

.. code-block:: console

   $ motion-spec run model.robmot --run-id run-1 --headless
   $ motion-spec replay "$MOTION_SPEC_GEN/model/latest/runs/run-1"
   $ motion-spec run "$MOTION_SPEC_GEN/model/latest" --run-id run-2 --headless

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
   dashboard
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
