===========
motion-spec
===========

``motion-spec`` validates, compiles, builds, runs, and inspects guarded robot
motion specifications. It accepts a ``.robmot`` model for the complete workflow
and retains lower-level RDF, IR, code-generation, and archive commands for tools
that need individual stages.

Quick start
===========

``motion-spec`` needs the rest of its workspace — the authoring DSLs, the
metamodels, the kinematics fork, the simulator wrapper — so install it through
``grc_meta``, which sets all of them up in one command:

.. code-block:: console

   $ mkdir -p ws/src
   $ git clone git@github.com:secorolab/grc_meta.git ws/src/grc_meta
   $ ws/src/grc_meta/script-setup --check ws     # what is missing, changing nothing
   $ ws/src/grc_meta/script-setup ws             # import, build, verify
   $ source ws/setup-grc.bash                    # or .zsh

ROS is optional (``--no-ros``), and the robot hardware backends are off until
asked for (``--with-hardware``). Then run a model, replay the run it recorded,
and run the same generation again:

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
