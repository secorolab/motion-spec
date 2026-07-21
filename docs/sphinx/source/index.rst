===========
motion-spec
===========

``motion-spec`` validates, compiles, builds, runs, and inspects guarded robot
motion specifications. It accepts a ``.robmot`` model for the complete workflow
and retains lower-level RDF, IR, code-generation, and archive commands for tools
that need individual stages.

Start with :doc:`setup`, read :doc:`concepts` for the pipeline and artifact
contracts, then follow the :doc:`tutorials/index`.

.. toctree::
   :maxdepth: 2
   :caption: Guide

   setup
   concepts
   tutorials/index

.. toctree::
   :maxdepth: 2
   :caption: Authoring language

   dsl/index

.. toctree::
   :maxdepth: 2
   :caption: Original RAL documentation

   tutorial

.. toctree::
   :maxdepth: 1
   :caption: About

   acknowledgement

Pipeline
========

.. graphviz:: _static/pipeline.dot
   :caption: ``motion-spec-dsl`` owns authoring and RDF emission; ``motion-spec`` owns compilation, execution, and inspection.

The `motion-spec-dsl project <https://github.com/secorolab/motion-spec-dsl>`_
provides the ``.robmot`` frontend. This project consumes its RDF dataset and owns
every stage from SHACL validation and IR generation onward.

Quick start
===========

.. code-block:: console

   $ motion-spec install dsl all
   $ motion-spec setup
   $ motion-spec health --target mujoco
   $ motion-spec run model.robmot -o generation/demo --run-id run-1 --headless
   $ motion-spec replay generation/demo/runs/run-1
   $ motion-spec run generation/demo --run-id run-2 --headless

Run ``motion-spec --help`` or ``motion-spec COMMAND --help`` for the exact
options supported by the installed version.
