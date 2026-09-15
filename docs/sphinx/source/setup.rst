=====
Setup
=====

``motion-spec`` is one package in a workspace: the authoring DSLs, the RDF
metamodels, the kinematics fork its generated controllers link, and the
simulator wrapper are separate repositories. ``grc_meta`` sets all of them up in
one command, and that is the supported installation.

.. code-block:: console

   $ mkdir -p ws/src
   $ git clone git@github.com:secorolab/grc_meta.git ws/src/grc_meta
   $ ws/src/grc_meta/script-setup --check ws     # what is missing, changing nothing
   $ ws/src/grc_meta/script-setup ws             # import, build, verify
   $ source ws/setup-grc.bash                    # or .zsh

``--check`` reports every missing prerequisite at once, with the ``apt-get``
line that installs them; the only steps needing ``sudo`` are that line and
``rosdep init``. Setup then imports the repositories, creates ``ws/.venv``,
installs the Python packages, builds the workspace, installs the STST code
generator, and finishes by running :ref:`motion-spec health <health-checks>`.

Use HTTPS repository URLs on a machine without GitHub SSH configured:

.. code-block:: console

   $ GRC_GIT_TRANSPORT=https ws/src/grc_meta/script-setup ws

Choosing what to build
======================

.. list-table::
   :header-rows: 1
   :width: 100%

   * - Option
     - What it does
   * - *(none)*
     - Build against the ROS distribution in ``$ROS_DISTRO``, or the single one
       installed under ``/opt/ros``. Simulation backends only.
   * - ``--ros <distro>``
     - Build against that distribution, failing if it is not installed. Needed
       when several are.
   * - ``--no-ros``
     - Build without ROS at all: plain CMake, no colcon, no rosdep.
   * - ``--with-hardware``
     - Also build the robot device backends: Kinova Kortex, EtherCat, Kelo and
       Robotiq. Off by default — a simulation workspace does not need them, and
       the Kortex API is a proprietary download.
   * - ``--smoke``
     - After setup, generate, build and run one model headless, as proof the
       installation works end to end.

Without ROS
-----------

ROS is optional. The generated controller asks for ``rclcpp`` only when the
model communicates over ROS, and the simulator wrapper builds its camera
publisher only when ROS is present, so a ROS-free workspace generates, builds
and runs every model that talks to nothing outside itself.

What a ROS-free workspace cannot do is generate a model that publishes a topic,
sends an action goal or answers one: reading a ROS message's shape needs
``rosidl_runtime_py``, which comes with the distribution. ``motion-spec health
--profile ros`` reports those dependencies as absent rather than missing, and
the generator says so plainly if a model asks for them anyway.

The two modes build differently, and the scripts remember which one a workspace
was set up as (in ``ws/.grc-mode``):

.. list-table::
   :header-rows: 1
   :width: 100%

   * -
     - With ROS
     - Without ROS
   * - Build driver
     - ``colcon build``, options from ``colcon.meta``
     - ``cmake`` per package, in dependency order, options from ``grc-lib.sh``
   * - System dependencies
     - ``rosdep``
     - the ``apt-get`` line from ``--check``
   * - Environment
     - sources ``/opt/ros/<distro>/setup.*`` and the colcon overlay
     - exports ``CMAKE_PREFIX_PATH``, ``LD_LIBRARY_PATH`` and ``PATH``

Python
------

``uv`` is used when it is installed, because it is much faster, and
``python3 -m venv`` with ``pip`` when it is not. Neither is required to be
present in advance and both produce the same workspace virtual environment at
``ws/.venv``.

Keeping a workspace up to date
==============================

.. code-block:: console

   $ "$GRC" sync     # update every repo to what grc_meta.repos declares, rebuild what moved
   $ "$GRC" build    # rebuild everything and refresh the editable Python installs
   $ "$GRC" mj       # rebuild mj_kdl_wrapper alone (it compiles twice)

``$GRC`` is exported by the generated environment file, so a sourced shell never
needs the script paths, the workspace path or the distribution. See the
`grc_meta README <https://github.com/secorolab/grc_meta>`_ for the update rules
these follow — fast-forward only, never a merge commit, and no repository ever
loses local work.

The generated environment exports:

.. list-table::
   :header-rows: 1
   :width: 100%

   * - Variable
     - Meaning
   * - ``GRC_WS``
     - The workspace root.
   * - ``INSTALL``
     - The install prefix generated controllers are built against.
   * - ``MOTION_SPEC_GEN``
     - Where ``motion-spec gen`` and ``run`` write a generation given no ``-o``.
   * - ``METAMODELS_PATH``
     - The metamodels checkout, for validation.
   * - ``GRC``
     - The workspace entry point described above.

Docker
======

``grc_meta``'s ``script-docker`` builds a development image with the system
packages preinstalled and runs it with the host display and GPU forwarded, so
the MuJoCo GUI renders on your screen. Inside the container:

.. code-block:: console

   $ GRC_GIT_TRANSPORT=https /grc_meta/script-setup ws
   $ source ws/setup-grc.bash

Dependencies
============

These tables mirror what ``motion-spec health`` checks.

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
   * - ROS *(optional)*
     - ``rosidl_runtime_py``, ``ament_index_python``, and ``rosidl_pycommon``
       or ``rosidl_cmake``, all from the ROS distribution
     - Generating a model that publishes a topic or drives a ROS action

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
   * - `Git <https://git-scm.com/>`_, a JDK, and `Apache Ant <https://ant.apache.org/>`_
     - Building the managed STST installation. A JRE is not enough: STST is
       compiled from source, and the ``ant`` package depends only on a runtime.
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
   * - robif2b devices (``--with-hardware``)
     - `serial <https://github.com/secorolab/serial>`_ and
       `robotiq_driver_noros <https://github.com/secorolab/robotiq_driver_noros>`_,
       which drives both the gripper and the force-torque sensor

The device drivers are optional: robif2b builds each wrapper only when its flag
is on. A model's ROS interface packages are not listed — the generated
``CMakeLists.txt`` asks for whichever ones that model declares. ``hddc2b`` is
optional for base solvers and is not a core health check.

Code generation
===============

C++ generation requires STST and ``protoc``. ``script-setup`` installs the
pinned STST itself; to install or repair it by hand:

.. code-block:: console

   $ motion-spec setup --prefix "$GRC_WS"
   $ motion-spec setup --prefix "$GRC_WS" --force   # rebuild a broken installation
   $ motion-spec setup --prefix "$GRC_WS" --clean   # remove it

The launcher is written to ``PREFIX/bin/stst``, which the workspace environment
puts on ``PATH``; with no ``--prefix`` it goes to ``~/.local/bin``. STST setup
requires Git, a JDK and Ant.

.. _health-checks:

Health checks
=============

``health`` reports every profile and target by default:

.. code-block:: console

   $ motion-spec health
   $ motion-spec health --profile dsl
   $ motion-spec health --profile codegen
   $ motion-spec health --profile ros
   $ motion-spec health --target mujoco
   $ motion-spec health --target robif2b

The health profiles are ``base``, ``validation``, ``introspection``, ``dsl``,
``codegen``, ``ros``, ``build``, and ``runtime``. Build and runtime are
evaluated per target: everything both targets share is reported under ``build``,
and only what a target adds appears under ``build[mujoco]`` or
``build[robif2b]``.

Every check names what its dependency is for, where it comes from, and the
command that installs it. A module resolved from a source tree rather than
``site-packages`` is reported as editable. Dependencies that are absent by
choice — a device wrapper whose flag is off, ROS on a ROS-free workspace —
report ``ABSENT`` and do not set a non-zero exit status; only missing ones do.

The dashboard shows the same report under **Health**. Installing into a new
location while it runs needs a restart — an editable install adds its path in a
``.pth`` file, which Python reads only at interpreter startup.

As a library
============

``motion-spec`` can be installed on its own, for a tool that reads a generation
or drives the IR. This installs no DSL compiler, no kinematics and no simulator,
so it cannot build or run a controller — use the workspace above for that.

.. code-block:: console

   $ python -m pip install /path/to/motion-spec
   $ motion-spec install validation
   $ motion-spec install introspection

``motion-spec install all`` installs every optional Python feature. The package
extras are ``validation``, ``introspection``, ``dashboard``, ``replay`` and
``all``. ``motion-spec install dsl`` adds the compilers: none of them are
published on PyPI, so each is taken from its sibling checkout when the workspace
has one and from its secorolab repository otherwise.

Development checkout
====================

The workspace virtual environment already has every package installed editable.
To reinstall one by hand:

.. code-block:: console

   $ cd "$GRC_WS"
   $ source .venv/bin/activate
   $ python -m pip install --no-deps -e src/motion-spec

Build these docs locally with:

.. code-block:: console

   $ python -m pip install -e "src/motion-spec[docs]"
   $ sphinx-build -W -b html src/motion-spec/docs/sphinx/source \
       src/motion-spec/docs/_build/html
