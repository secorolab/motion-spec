=====
Setup
=====

``motion-spec`` is installed on its own, and installs the rest of what it builds
against itself: the authoring DSLs arrive with the Python package, and
``motion-spec setup`` builds the kinematics fork, the simulator wrapper and the
other C++ libraries from source into one prefix.

.. code-block:: console

   $ mkdir -p ws/src
   $ git clone git@github.com:secorolab/motion-spec.git ws/src/motion-spec
   $ python3 -m venv ws/.venv && source ws/.venv/bin/activate
   $ pip install -e ws/src/motion-spec            # the CLI and the DSL compilers
   $ motion-spec install all                      # every optional Python feature
   $ motion-spec health                           # the one apt line for what is missing
   $ motion-spec setup --workspace ws             # STST and the C++ libraries
   $ source ws/setup-motion-spec.bash             # or .zsh

:ref:`health <health-checks>` gathers everything apt provides into a single
``sudo apt-get install -y`` line — that line is the only step needing ``sudo``.
Run it before ``setup``, which needs ``git``, ``cmake``, a C++ compiler, a JDK
and Ant to build what it installs.

Installing the external dependencies
====================================

``motion-spec setup`` installs each dependency at the version
``src/motion_spec/motion_spec.repos`` pins. With no components named it installs
the ones every model needs, in dependency order; name components to do fewer, or
to add a device driver.

The CMake components have to be built, so their sources are fetched into
``WORKSPACE/.ms-sources``. The Python components do not: pip installs them
straight from the pinned ref. ``--dev`` changes both — everything is checked out
into ``WORKSPACE/src`` and the Python components are installed editable.

.. list-table::
   :header-rows: 1
   :width: 100%

   * - Where
     - What
   * - ``WORKSPACE/.ms-sources/<repository>``
     - The sources a plain install must build, fetched and removed by ``setup``
       — ``orocos_kdl`` carries a ``package.xml``, the others are plain CMake
       projects.
   * - ``WORKSPACE/src/<repository>``
     - The same sources under ``--dev``, as ordinary workspace packages, plus
       the Python components installed editable. Yours to edit; ``setup``
       adopts a checkout already there and never moves it.
   * - ``<sources>/thirdparty/STSTv4``
     - The exception: an Ant project colcon cannot identify, in a subtree
       carrying a ``COLCON_IGNORE``.
   * - ``WORKSPACE/build/<component>``
     - The CMake build directories.
   * - ``WORKSPACE/install``
     - The launcher, libraries, headers and CMake packages.

In a ROS workspace
------------------

``[ros] workspace = true`` in the config makes this a colcon workspace, and
``setup`` stops building by hand: it writes ``colcon.meta`` with the cmake
options each package needs, then runs ``colcon build --packages-select`` once
per component, in the order motion-spec knows — colcon cannot derive it, since
``coord2b`` and ``mj_kdl_wrapper`` carry no ``package.xml``. ``[ros] distro``
says which ``/opt/ros`` to build against, and ``$ROS_DISTRO`` overrides it.

The environment file then sources rather than exports: the distribution, then
the workspace's own ``install/setup.<shell>``, plus ``PATH`` for the prefix's
``bin`` — ``stst`` is Ant-built into it and belongs to no colcon package.
``stst`` is built the same way either way; only the CMake components change.

The device drivers — ``serial``, ``robotiq_driver_noros`` and ``robif2b`` — are
pinned like the rest but built only when named: ``motion-spec setup robif2b``.
Each robif2b device wrapper stays off until its flag is passed, with
``--cmake-arg`` or the config file's ``[setup.cmake_args]``; ``health`` names the
flag each missing wrapper needs.

A source directory that is already there belongs to whoever made it. ``setup``
fetches it — which touches no working tree — and builds it only when it is clean
and already on the pinned commit. A checkout on another ref, or with uncommitted
changes, is reported and skipped, never moved onto the pin; ``setup`` exits
non-zero so a caller knows those components were not built. Only a checkout
``setup`` cloned itself is ever removed by ``--clean``.

The workspace is ``$MOTION_SPEC_WS``, or ``--workspace`` when the variable is
unset; with neither, ``setup`` stops and says so rather than picking a location.
What it installs is a toolchain — four C++ libraries, their headers and a Python
extension — so where that lands is a choice the command never makes on its own.
``--prefix`` overrides the install location for a caller who wants one somewhere
other than ``WORKSPACE/install``; the environment files are written to the
workspace root either way.

.. list-table::
   :header-rows: 1
   :width: 100%

   * - Component
     - What it is
   * - ``stst``
     - The StringTemplate tool the C++ generator drives, built with Ant and
       launched from ``PREFIX/bin/stst``.
   * - ``orocos_kdl``
     - The secorolab fork of Orocos KDL — chains, solvers and frames.
   * - ``coord2b``
     - The FSM event loop the generated controller dispatches through.
   * - ``mj_kdl_wrapper``
     - The MuJoCo simulation, built twice: the CMake package the controller
       links, and the Python bindings, installed into the active environment.
       It consumes the ``orocos_kdl`` installed above it
       (``MJ_KDL_OROCOS_KDL_FROM_PACKAGE``) instead of cloning and building the
       same fork a second time into its own tree, which is also what keeps one
       ``liborocos-kdl`` in the controller's process rather than two.

.. code-block:: console

   $ export MOTION_SPEC_WS="$PWD/ws"      # or pass --workspace to each command below
   $ motion-spec setup                    # the core set, into $MOTION_SPEC_WS/install
   $ motion-spec setup robif2b            # a device driver, only when named
   $ motion-spec setup mj_kdl_wrapper     # one of them
   $ motion-spec setup --force            # rebuild regardless
   $ motion-spec setup --clean            # remove what it installed
   $ motion-spec setup --build-type Debug

Each installation records the ref it was built from, so running ``setup`` again
is a no-op until a pin moves and a rebuild once it does — ``--force`` is for
repairing a broken build, not for picking up a new version. ``--clean`` removes
only what ``setup`` installed, through the build's own install manifest, and
refuses an installation it did not make.

Nothing motion-spec removes is unrecoverable: ``--clean`` moves the sources,
builds, installed files and environment files to the desktop trash rather than
deleting them, the same way the dashboard clears a generation. The installed
files go as one entry per component rather than as a page of loose headers. This
needs ``gio``; without it ``--clean`` says so and removes nothing.

The manifest is `vcstool <https://github.com/dirk-thomas/vcstool>`_'s format, so
the same sources can be imported by hand into the same place:

.. code-block:: console

   $ vcs import ws/src < ws/src/motion-spec/src/motion_spec/motion_spec.repos

The environment file
====================

``setup`` writes one ``setup-motion-spec.<shell>`` to the workspace root, for the
shell in force — ``$SHELL``, or ``[workspace] shell`` when the config names one.
Sourcing it is what makes an installation usable from a fresh shell, and
``motion-spec mutate`` finds a workspace by looking for it above the model.

.. list-table::
   :header-rows: 1
   :width: 100%

   * - Variable
     - Meaning
   * - ``MOTION_SPEC_PREFIX``
     - The prefix everything was installed into.
   * - ``PATH``
     - Gains ``PREFIX/bin``, so ``stst`` is found during generation.
   * - ``CMAKE_PREFIX_PATH``
     - Gains the prefix, so a generated ``CMakeLists.txt`` finds its packages.
   * - ``LD_LIBRARY_PATH``
     - Gains ``PREFIX/lib``, so a generated controller runs.

A workspace can keep its settings in a ``motion-spec.config.toml`` at its root
instead of in the shell. ``motion-spec setup`` writes a commented sample the
first time, ``motion-spec config --init`` writes one on demand, and
``motion-spec config`` prints what is in force and where each value came from:

.. code-block:: console

   $ motion-spec config
   file: /home/you/ws/motion-spec.config.toml
     workspace.root         /home/you/ws            (this file's directory)
     workspace.generations  /home/you/ws/gen-out    (file)
     setup.build_type       Debug                   (file)
     setup.components       ['stst', 'coord2b']     (default)

A command-line option overrides an environment variable, which overrides the
file, which overrides the built-in default. The file also answers the workspace
question on its own: with one at the root, ``setup`` needs no ``--workspace``
and no ``$MOTION_SPEC_WS``. It carries ``[workspace]`` (``root``,
``generations``, ``environment``) and ``[setup]`` (``prefix``, ``build_type``,
``components``, and ``[setup.cmake_args]`` per component — which is where the
robif2b device flags belong). A component's ``cmake_args`` is the whole list it
is built with, not an addition to a hidden one: the sample writes out what
motion-spec uses, so removing an option from the list removes it from the build.
``--cmake-arg`` adds to whichever list applies, for one invocation. An unknown
section or key is an error rather than
a setting that silently does nothing.

Two variables are yours to set, and sourcing the file sets the first:

.. list-table::
   :header-rows: 1
   :width: 100%

   * - Variable
     - Meaning
   * - ``MOTION_SPEC_WS``
     - The workspace. ``setup`` installs into its ``install/``, generations go
       to its ``generations/``, and ``stst`` is looked for in its ``install/bin``
       when it is not on ``PATH``. Required by ``setup`` unless ``--workspace``
       is passed.
   * - ``MOTION_SPEC_GEN``
     - Generations somewhere other than ``$MOTION_SPEC_WS/generations``.
       With neither this nor ``MOTION_SPEC_WS`` set, ``gen``, ``run``, ``rerun``
       and ``dashboard`` stop and say so rather than writing to the working
       directory, where generations accumulate unnoticed and ``rerun`` cannot
       find them again.

Everything else is optional: ``MOTION_SPEC_ENV`` (the file to source, below),
``MOTION_SPEC_BUILD_TYPE`` (``CMAKE_BUILD_TYPE`` for a generated build),
``MOTION_SPEC_JOURNAL`` (a journal somewhere other than
``$MOTION_SPEC_WS/.motion-spec/journal.jsonl``) and ``ROS_DISTRO``.

Every command appends one JSON line to that journal — when, what was asked, and
from where — written before the work, so a command that never returned is still
in the record. It is a workspace's history or nothing: outside a workspace,
nothing is written. ``motion-spec journal`` prints it as columns, ``-n`` for the
last few and ``--json`` for the lines themselves.

What the tools themselves printed is kept too, beside the journal or with the
generation it was about:

.. list-table::
   :header-rows: 1
   :width: 100%

   * - Command
     - Console kept in
   * - ``setup`` (git, cmake, ant, pip)
     - ``$MOTION_SPEC_WS/.motion-spec/logs/<stamp>-setup.log``
   * - ``gen`` (the DSL)
     - ``<generation>/logs/gen.log``
   * - ``build`` (cmake)
     - ``<generation>/logs/build.log``
   * - ``run`` (the controller)
     - ``<generation>/runs/<run-id>/logs/console.log``

Each runs under a pty, so a tool still sees a terminal: git's progress counter
and cmake's colour look on screen exactly as they do without motion-spec in the
way, and the log keeps those same bytes. A failed build is therefore something
to read afterwards rather than something that scrolled past. Nothing reads ``MOTION_SPEC_PREFIX``: where a workspace installs
is not a separate fact from the workspace, so it is derived rather than
configured, and exported only so a shell and a build can see it.

Sourcing it by hand is not the only way to use it. ``build``, ``run``, ``rerun``
and ``health`` source a file themselves and run everything under what it left:

.. code-block:: console

   $ motion-spec build gen-1 --env ws/setup-motion-spec.bash
   $ MOTION_SPEC_ENV=ws/setup-motion-spec.bash motion-spec run model.robmot
   $ motion-spec run gen-1 --no-env        # inherit this shell, whatever files sit above

With no ``--env`` the file is found the same way twice: ``$MOTION_SPEC_ENV``
first, then the nearest ``setup-motion-spec.bash`` above the generation, then
above the working directory. So an installation made with ``--workspace ws``
works from a fresh shell with no flags and nothing sourced. Each command says on
stderr which file it used; ``--no-env`` turns the whole mechanism off.

This matters most with ROS, where the distribution's overlay decides which
``rclcpp`` a build links and whether a run can reach its topics. ``health``
takes the same option and probes under that environment — module imports in an
interpreter started in it, ``find_package`` and the runtime library loads in
processes started in it — so its report is about the environment the build will
actually get, not about the terminal it was typed in.

A run records the environment it happened in: the file's path and the variables
a build and a run depend on (``PATH``, ``CMAKE_PREFIX_PATH``,
``LD_LIBRARY_PATH``, ``PYTHONPATH``, ``MOTION_SPEC_PREFIX`` and the ``ROS_*``
ones) are archived with the run's host information, so a recorded result names
the toolchain and the prefixes that produced it rather than only the hostname.

Without ROS
===========

ROS is optional. The generated controller asks for ``rclcpp`` only when the
model communicates over ROS, and the simulator wrapper builds its camera
publisher only when ROS is present, so an installation without ROS generates,
builds and runs every model that talks to nothing outside itself.

What it cannot do is generate a model that publishes a topic, sends an action
goal or answers one: reading a ROS message's shape needs ``rosidl_runtime_py``,
which comes with the distribution. ``motion-spec health --profile ros`` reports
those dependencies as absent rather than missing, names the distributions
installed under ``/opt/ros`` and says whether one is sourced, and the generator
says so plainly if a model asks for them anyway.

With a distribution sourced, ``setup`` and the generated builds pick it up from
the environment; nothing needs to be told which one.

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
     - `Python 3.11+ <https://www.python.org/>`_,
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
       `Orocos KDL <https://github.com/secorolab/orocos_kinematics_dynamics>`_, and
       `toml++ <https://github.com/marzer/tomlplusplus>`_
     - Every generated controller, whichever target it drives

Orocos KDL must be the secorolab fork: generated controllers call the
Vereshchagin solvers with fixed joints, which ``liborocos-kdl-dev`` does not
carry. ``motion-spec setup`` installs that fork, along with ``coord2b`` and
``mj_kdl_wrapper``; Eigen and toml++ come from apt.

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

C++ generation requires STST and ``protoc``. ``motion-spec setup`` installs the
pinned STST with everything else; to install or repair only it:

.. code-block:: console

   $ motion-spec setup stst
   $ motion-spec setup stst --force   # rebuild a broken one
   $ motion-spec setup stst --clean   # remove it

The launcher is written to ``WORKSPACE/install/bin/stst``, which the environment
file puts on ``PATH``. It is a two-line script naming the jar it runs, so a
launcher left behind by an installation that has since been deleted keeps
resolving on ``PATH`` and fails at the jar — ``setup stst --force`` replaces it.
STST setup requires Git, a JDK and Ant.

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

The report opens with the variables motion-spec reads and what each is set to,
and names the ROS distribution in use — ``jazzy sourced (ROS 2)``, with the others
installed beside it. Several checks configure a CMake project, so each one is
announced as it runs; the dashboard's Health page shows the same progress and
offers the workspace's environment files to check under.

The health profiles are ``base``, ``validation``, ``introspection``, ``dsl``,
``codegen``, ``ros``, ``build``, and ``runtime``. Build and runtime are
evaluated per target: everything both targets share is reported under ``build``,
and only what a target adds appears under ``build[mujoco]`` or
``build[robif2b]``.

Every check names what its dependency is for and where it comes from. What apt
provides is gathered into one ``sudo apt-get install -y`` line under the summary,
rather than a command per row to assemble by hand; anything apt cannot serve — a
workspace package, the STST installation, a device wrapper whose flag is off —
keeps its own ``fix:`` line where it was reported. A module resolved from a
source tree rather than ``site-packages`` is reported as editable. Dependencies that are absent by
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

The installation above is already editable — edits to ``src/motion-spec`` take
effect without reinstalling. To reinstall it by hand:

.. code-block:: console

   $ cd ws
   $ source .venv/bin/activate
   $ python -m pip install --no-deps -e src/motion-spec

Build these docs locally with:

.. code-block:: console

   $ python -m pip install -e "src/motion-spec[docs]"
   $ sphinx-build -W -b html src/motion-spec/docs/sphinx/source \
       src/motion-spec/docs/_build/html
