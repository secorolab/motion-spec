=====
Setup
=====

Installation
============

The base package provides the CLI, RDF-to-IR lowering, and C++ generation.
Install optional features only where they are needed:

.. code-block:: console

   $ python -m venv .venv
   $ source .venv/bin/activate
   $ python -m pip install /path/to/motion-spec
   $ motion-spec install validation
   $ motion-spec install introspection
   $ motion-spec install dsl

``motion-spec install dsl all`` installs the authoring DSL and every optional
Python feature. The package extras are ``validation``, ``introspection``, and
``all``; the sibling ``motion-spec-dsl`` package is installed by the ``dsl``
feature.

Dependencies
============

Python profiles
---------------

.. list-table:: Python dependencies installed by package profile
   :header-rows: 1
   :width: 100%

   * - Profile
     - Dependencies
     - Required for
   * - Base
     - `Python 3.10+ <https://www.python.org/>`_,
       `Click <https://click.palletsprojects.com/>`_,
       `RDFLib <https://github.com/RDFLib/rdflib>`_, and
       `rdf-utils <https://github.com/secorolab/rdf-utils>`_
     - CLI, RDF loading, IR, and code generation
   * - Validation
     - `pySHACL <https://github.com/RDFLib/pySHACL>`_
     - ``motion-spec check``
   * - Introspection
     - `pySHACL <https://github.com/RDFLib/pySHACL>`_,
       `REC <https://github.com/secorolab/rec>`_, and
       `Protocol Buffers <https://github.com/protocolbuffers/protobuf>`_
     - Recording, archive verification, replay, and runtime RDF
   * - DSL
     - `motion-spec-dsl <https://github.com/secorolab/motion-spec-dsl>`_,
       `textX <https://github.com/textX/textX>`_,
       `coord-dsl <https://github.com/secorolab/coord-dsl>`_, and
       `scene-dsl <https://github.com/secorolab/scene-dsl>`_
     - Accepting ``.robmot`` models as high-level command input

Generation and common runtime
-----------------------------

.. list-table:: External toolchain dependencies
   :header-rows: 1
   :width: 100%

   * - Dependency
     - Required for
   * - `STSTv4 <https://github.com/jsnyders/STSTv4>`_
     - Rendering generated C++
   * - `Protocol Buffers compiler <https://protobuf.dev/>`_
     - Generating the C++ frame-log codec
   * - `Git <https://git-scm.com/>`_, `Java <https://openjdk.org/>`_, and
       `Apache Ant <https://ant.apache.org/>`_
     - Building the managed STST installation
   * - `CMake <https://cmake.org/>`_ and `GCC <https://gcc.gnu.org/>`_ or another C++ compiler
     - Configuring and compiling generated controllers
   * - `coord2b <https://github.com/rosym-project/coord2b>`_
     - Building and running generated controller state machines

Target dependencies
-------------------

.. list-table:: Target-specific build and runtime dependencies
   :header-rows: 1
   :width: 100%

   * - Target
     - Dependencies
   * - MuJoCo
     - `Orocos KDL <https://github.com/orocos/orocos_kinematics_dynamics>`_,
       `kdl_parser <https://github.com/ros/kdl_parser>`_, and
       `mj_kdl_wrapper <https://github.com/vamsikalagaturu/mj_kdl_wrapper>`_
   * - robif2b
     - `robif2b <https://github.com/secorolab/robif2b>`_,
       `Eigen <https://eigen.tuxfamily.org/>`_,
       `Orocos KDL <https://github.com/orocos/orocos_kinematics_dynamics>`_,
       `urdfdom_headers <https://github.com/ros/urdfdom_headers>`_,
       `urdfdom <https://github.com/ros/urdfdom>`_, and
       `kdl_parser <https://github.com/ros/kdl_parser>`_

These tables mirror ``motion-spec health``. ``hddc2b`` is optional for base
solvers and is not a core health check.

Code generation
===============

C++ generation requires STST and ``protoc``. Install the pinned STST version
with:

.. code-block:: console

   $ motion-spec setup

The default launcher is ``~/.local/bin/stst``. A local installation prefix and
its matching cleanup are:

.. code-block:: console

   $ motion-spec setup --prefix /path/to/workspace
   $ motion-spec setup --prefix /path/to/workspace --clean

STST setup requires Git, Java, and Ant.

Health checks
=============

``health`` reports every profile and target by default:

.. code-block:: console

   $ motion-spec health
   $ motion-spec health --profile dsl
   $ motion-spec health --profile codegen
   $ motion-spec health --target mujoco
   $ motion-spec health --target robif2b

The profiles are ``base``, ``validation``, ``introspection``, ``dsl``,
``codegen``, ``build``, and ``runtime``. Build and runtime are evaluated per
target: MuJoCo requires the KDL stack and ``mj_kdl_wrapper``, the real-robot
target requires ``robif2b``. Missing optional profiles do not invalidate a
base-only installation; select the profile required by the command you intend to
run.

Development checkout
====================

Use the workspace virtual environment:

.. code-block:: console

   $ cd /path/to/workspace
   $ source .venv/bin/activate
   $ python -m pip install --no-deps -e src/motion-spec

Build these docs locally with:

.. code-block:: console

   $ python -m pip install -e "src/motion-spec[docs]"
   $ sphinx-build -W -b html src/motion-spec/docs/sphinx/source \
       src/motion-spec/docs/_build/html
