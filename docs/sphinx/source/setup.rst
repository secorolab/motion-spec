=====
Setup
=====

Installation
============

The base package provides the CLI, the authoring DSL, RDF-to-IR lowering, and C++
generation.
Install optional features only where they are needed:

.. code-block:: console

   $ python -m venv .venv
   $ source .venv/bin/activate
   $ python -m pip install /path/to/motion-spec
   $ motion-spec install validation
   $ motion-spec install introspection

``motion-spec install all`` installs every optional Python feature. The package
extras are ``validation``, ``introspection``, and ``all``. ``motion-spec install
dsl`` installs the compilers: none of them are published on PyPI, so each is
taken from its sibling checkout when the workspace has one and from its
secorolab repository otherwise.

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
       `RDFLib <https://github.com/RDFLib/rdflib>`_,
       `rdf-utils <https://github.com/secorolab/rdf-utils>`_, and
       `Jinja <https://github.com/pallets/jinja>`_
     - CLI, RDF loading, IR, and the scene's KDL headers
   * - DSL
     - `motion-spec-dsl <https://github.com/secorolab/motion-spec-dsl>`_,
       `textX <https://github.com/textX/textX>`_,
       `coord-dsl <https://github.com/secorolab/coord-dsl>`_, and
       `scene-dsl <https://github.com/secorolab/scene-dsl>`_
     - Compiling ``.robmot``, ``.fsm``, and ``.scenex`` sources
   * - Validation
     - `pySHACL <https://github.com/RDFLib/pySHACL>`_
     - ``motion-spec check``
   * - Introspection
     - `pySHACL <https://github.com/RDFLib/pySHACL>`_,
       `REC <https://github.com/secorolab/rec>`_, and
       `Protocol Buffers <https://github.com/protocolbuffers/protobuf>`_
     - Recording, archive verification, replay, and runtime RDF

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
   * - `coord2b <https://github.com/rosym-project/coord2b>`_,
       `Eigen <https://eigen.tuxfamily.org/>`_,
       `Orocos KDL <https://github.com/secorolab/orocos_kinematics_dynamics>`_,
       `kdl_parser <https://github.com/ros/kdl_parser>`_, and
       `toml++ <https://github.com/marzer/tomlplusplus>`_
     - Every generated controller, whichever target it drives

Orocos KDL must be the secorolab fork: generated controllers call the
Vereshchagin solvers with fixed joints, which ``liborocos-kdl-dev`` does not
carry. ``grc_meta`` provides it with the other workspace packages.

Target dependencies
-------------------

.. list-table:: Target-specific build and runtime dependencies
   :header-rows: 1
   :width: 100%

   * - Target
     - Dependencies
   * - MuJoCo
     - `mj_kdl_wrapper <https://github.com/vamsikalagaturu/mj_kdl_wrapper>`_,
       at the version the generated ``CMakeLists.txt`` pins
   * - robif2b
     - `robif2b <https://github.com/secorolab/robif2b>`_,
       `urdfdom_headers <https://github.com/ros/urdfdom_headers>`_, and
       `urdfdom <https://github.com/ros/urdfdom>`_
   * - robif2b devices (optional)
     - `serial <https://github.com/secorolab/serial>`_ and
       `robotiq_driver_noros <https://github.com/secorolab/robotiq_driver_noros>`_,
       which drives both the gripper and the force-torque sensor

These tables mirror ``motion-spec health``. The device drivers are optional:
robif2b builds each wrapper only when its flag is on. A model's ROS interface
packages are not listed — the generated ``CMakeLists.txt`` asks for whichever
ones that model declares. ``hddc2b`` is optional for base solvers and is not a
core health check.

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

The health profiles are ``base``, ``validation``, ``introspection``, ``dsl``,
``codegen``, ``build``, and ``runtime``. Build and runtime are evaluated per
target: everything both targets share is reported under ``build``, and only
what a target adds appears under ``build[mujoco]`` or ``build[robif2b]``.
Missing optional profiles do not invalidate a base-only installation; the
``dsl`` profile reports the transitive authoring dependencies separately so
installation problems are actionable.

Every check names what its dependency is for, where it comes from, and the
command that installs it. A module resolved from a source tree rather than
``site-packages`` is reported as editable. Ones absent by build option report
``ABSENT`` and do not set a non-zero exit status; only missing ones do.

The dashboard shows the same report under **Health**. Installing into a new
location while it runs needs a restart — an editable install adds its path in a
``.pth`` file, which Python reads only at interpreter startup.

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
