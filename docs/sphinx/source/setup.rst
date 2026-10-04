=====
Setup
=====

``motion-spec`` itself is the one thing installed by hand. Everything it builds
against — rdf-utils, the DSL compilers, rec, the kinematics fork, the simulator
wrapper and STST — is listed in a ``.repos`` manifest, and ``motion-spec setup``
imports and installs exactly that list.

.. code-block:: console

   $ mkdir -p ws/src
   $ git clone git@github.com:secorolab/motion-spec.git ws/src/motion-spec
   $ python3 -m venv ws/.venv && source ws/.venv/bin/activate
   $ pip install -e ws/src/motion-spec            # the CLI and its PyPI dependencies
   $ motion-spec health                           # the one apt line for what is missing
   $ motion-spec setup --workspace ws --dev       # everything motion_spec.repos lists
   $ source ws/setup-motion-spec.bash             # or .zsh

``pip install`` of motion-spec pulls in only what PyPI has. The packages that are
on no index (rdf-utils, coord-dsl, scene-dsl, motion-spec-dsl, rec) are not
dependencies of the Python package at all: ``setup`` installs them from the
manifest, into the same environment.

:ref:`health <health-checks>` gathers everything apt provides into a single
``sudo apt-get install -y`` line — that line is the only step needing ``sudo``.
Run it before ``setup``, which needs ``vcs`` (``python3-vcstool``), ``cmake``, a
C++ compiler, a JDK and Ant to build what it installs.

The Python environment
======================

Every Python package ``setup`` installs goes into one environment: the active
virtual environment when ``$VIRTUAL_ENV`` is set, else ``WORKSPACE/.venv``, which
``setup`` creates (``--system-site-packages`` under ``--ros``) and installs
motion-spec into from the checkout it runs from. The environment file activates
that environment, so one ``source`` gives the whole toolchain.

With uv, create and activate the environment yourself; ``setup`` finds no
``pip`` in it and installs with ``uv pip install --python`` instead:

.. code-block:: console

   $ uv venv ws/.venv
   $ source ws/.venv/bin/activate
   $ uv pip install -e ws/src/motion-spec

For a ROS workspace, build the environment on the system interpreter:

.. code-block:: console

   $ uv venv --python /usr/bin/python3 --system-site-packages ws/.venv

``--python /usr/bin/python3`` is the part that matters. Without it uv builds the
environment on its own CPython, whose system packages are not the
distribution's, so ``--system-site-packages`` reaches nothing from apt.

What setup installs
===================

``motion-spec setup`` reads two manifests that ship inside the package, both in
`vcstool <https://github.com/dirk-thomas/vcstool>`_'s format:

.. list-table::
   :header-rows: 1
   :width: 100%

   * - Manifest
     - Lists
   * - ``motion_spec.repos``
     - Everything a model needs: ``thirdparty/rec``,
       ``thirdparty/motion-spec-dsl``, ``thirdparty/coord-dsl``,
       ``thirdparty/scene-dsl``, ``thirdparty/rdf-utils``,
       ``orocos_kinematics_dynamics``, ``coord2b``, ``mjkdl`` and
       ``thirdparty/STSTv4``. Always installed.
   * - ``motion_spec.real.repos``
     - The device drivers a real platform needs: ``serial``,
       ``robotiq_driver_noros`` and ``robif2b``. Installed after the first with
       ``--real``.

The manifest is the whole statement: what it lists is installed, in the order it
lists it, and nothing else; ``--real`` layers the device drivers on top. A path
listed in both manifests is an error.

The order is the install order. The Python packages come first and go
*dependents first*: their own ``pyproject.toml`` files pin each other by git URL,
so each install pulls its dependencies from git, and the local checkout installed
after it replaces that copy. ``rdf-utils``, which all of them pin, is last. The
CMake packages follow in link order — ``mjkdl`` links ``orocos_kdl``.

A run goes through the same steps every time:

#. The Python environment, as above.
#. ``vcs import --skip-existing`` of each manifest into the source directory.
   A checkout already at a manifest path is left exactly as it is.
#. For each entry in order: what the checkout holds decides how it is built.
   A ``CMakeLists.txt`` is a CMake package, a ``pyproject.toml`` or ``setup.py``
   a Python one, both (``mjkdl``) means CMake and then its bindings.
   A checkout with neither at its root is searched one level down, as colcon
   does: ``orocos_kinematics_dynamics`` yields ``orocos_kdl`` and then
   ``python_orocos_kdl``, in ``package.xml`` dependency order.
#. ``thirdparty/STSTv4`` is the one special case: an Ant project, built and given
   a launcher in ``PREFIX/bin/stst``.
#. The environment file.

``motion-spec setup mjkdl`` narrows the build to the named entries, by
manifest path or by the last component of it; the import still brings in any
listed repository that is missing.

Normal and ``--dev``
--------------------

Sources are in ``WORKSPACE/src/<path>`` either way; the modes differ only in how
the Python packages are installed:

.. list-table::
   :header-rows: 1
   :width: 100%

   * - Mode
     - Python packages
   * - ``motion-spec setup``
     - installed as snapshots
   * - ``motion-spec setup --dev``
     - installed editable (``pip install -e``)

A checkout you already have is used when it sits at its manifest path — the
Python ones under ``src/thirdparty/`` — and a checkout anywhere else is not seen,
so ``vcs import`` clones a fresh copy there. motion-spec itself is in no
manifest: it is the code running, cloned by hand into ``src/motion-spec``.

.. list-table::
   :header-rows: 1
   :width: 100%

   * - Where
     - What
   * - ``WORKSPACE/src/<path>``
     - The CMake packages.
   * - ``WORKSPACE/src/thirdparty/<path>``
     - The Python packages and STSTv4: what colcon must not build.
   * - ``WORKSPACE/build/<package>``
     - The CMake build directories.
   * - ``WORKSPACE/install``
     - The launcher, libraries, headers and CMake packages.

A source directory that is already there belongs to whoever made it, and
``setup`` treats it that way. It never fetches into it or checks anything out;
it builds what is in the tree, whatever ref that is:

.. list-table::
   :header-rows: 1
   :width: 100%

   * - The checkout
     - What ``setup`` does
   * - Clean, on the pinned commit
     - Builds it, and records that commit, so the next run is a no-op.
   * - Clean, on some other ref
     - Builds it as it stands and warns, naming the ref it found and the pin it
       is not. A checkout is the operator's answer to which version this
       workspace wants; ``setup`` never moves it onto the pin, which would lose
       the branch you were on.
   * - Uncommitted changes
     - Builds them, and warns. Nothing recorded can describe work that is in no
       commit, so it is rebuilt on every run until you commit it.
   * - Not a git checkout
     - Skipped with a warning, and ``setup`` exits non-zero once it has built
       everything else, so a caller knows which repositories it did not get.

``--clean`` never removes a source tree, whoever cloned it.

cmake arguments: ``colcon.meta``
--------------------------------

Per-package cmake arguments live in ``WORKSPACE/colcon.meta``, colcon's own
format, and nowhere else. The first ``setup`` seeds it from the shipped one; from
then on it is yours to edit, and ``setup`` never rewrites it:

.. code-block:: json

   {
       "names": {
           "mjkdl": {
               "cmake-args": ["-DMJKDL_OROCOS_KDL_FROM_PACKAGE=ON"]
           },
           "robif2b": {
               "cmake-args": ["-DENABLE_INSTALL_TARGETS=ON", "-DENABLE_KORTEX=ON"]
           }
       }
   }

colcon reads it through ``--metas``; a plain CMake build reads the same entry, so
a package gets the same arguments either way. That is where the robif2b device
wrappers are turned on: each stays off until its ``-DENABLE_*`` flag is in the
list, and ``health`` names the flag a missing wrapper needs. One kind of
argument is not in it: the interpreter, pybind11 and site-packages of the
Python environment, which ``setup`` passes to every CMake package because they
are paths on this machine.

The workspace
-------------

The workspace is ``--workspace``, else ``$MOTION_SPEC_WS``, else the one the
config file names or sits in; with none, ``setup`` stops and says so rather than
picking a location. Everything installs into ``WORKSPACE/install``.

.. code-block:: console

   $ export MOTION_SPEC_WS="$PWD/ws"      # or pass --workspace to each command below
   $ motion-spec setup --dev              # motion_spec.repos, into $MOTION_SPEC_WS/install
   $ motion-spec setup --dev --real       # plus the device drivers
   $ motion-spec setup mjkdl     # build only that entry
   $ motion-spec setup --force            # rebuild regardless, from a cleared CMake cache
   $ motion-spec setup --clean            # trash build/, install/, log/ and the env files
   $ motion-spec setup --build-type Debug

Each package records the commit it was built from, so running ``setup`` again
is a no-op until that checkout moves — ``--force`` is for repairing a broken
build, not for picking up a new version.

``--clean`` moves the workspace's ``build/``, ``install/`` and ``log/`` trees and
its environment files to the desktop trash rather than deleting them, the same
way the dashboard clears a generation; sources and generations stay. This needs
``gio``; without it ``--clean`` says so and removes nothing.

The manifests import by hand too, into the same place ``--dev`` uses:

.. code-block:: console

   $ vcs import ws/src < ws/src/motion-spec/src/motion_spec/motion_spec.repos

How many compilers at once
--------------------------

No build is given an unbounded job count. Left to itself, ``cmake --build
--parallel`` emits a bare ``make -j``, which is unlimited and overrides
``CMAKE_BUILD_PARALLEL_LEVEL`` and ``MAKEFLAGS`` as well — enough to swap a
machine to death on an Eigen or pybind11 translation unit. ``setup`` decides the
number instead: the lesser of the usable cores and one job per 2 GiB of RAM.

.. code-block:: console

   $ motion-spec setup -j 4                 # or --jobs 4
   $ CMAKE_BUILD_PARALLEL_LEVEL=4 motion-spec setup

``-j`` wins, then ``CMAKE_BUILD_PARALLEL_LEVEL``, then the computed default. In a
colcon workspace the same number
is passed as ``MAKEFLAGS=-jN -lN``, which colcon-cmake honours in place of its
own ``-j$(nproc)``. ``motion-spec build`` uses the same default.

In a ROS workspace
------------------

``[ros] workspace = true`` in the config makes this a colcon workspace. Each
CMake package is then built with ``colcon build --packages-select``, one per
call in the manifest's order — colcon cannot derive it, since ``coord2b`` and
``robif2b`` carry no ``package.xml`` — with the workspace ``colcon.meta`` passed
as ``--metas``. ``setup`` puts a ``COLCON_IGNORE`` in ``src/thirdparty``,
so a bare ``colcon build`` never tries the Python packages or STSTv4 either. The
build and install bases are named, so colcon writes where the environment file
points. ``[ros] distro`` says which ``/opt/ros`` to build
against, and ``$ROS_DISTRO`` overrides it.

The environment file then sources rather than exports: the distribution, then
the workspace's own ``install/setup.<shell>``, plus ``PATH`` for the prefix's
``bin`` — ``stst`` is Ant-built into it and belongs to no colcon package. The
Python packages and STST are installed the same way either way; only the CMake
build changes.

``--ros`` and ``--no-ros`` apply to one run; ``[ros] workspace`` is what keeps
the choice. Before anything is imported, ``setup`` checks that a
distribution is resolvable, that ``colcon`` is on ``PATH`` and that the
environment can see the distribution's Python packages.

The environment file
====================

``setup`` writes one ``setup-motion-spec.<shell>`` to the workspace root, for
``$SHELL``'s shell (bash or zsh, else bash).
Sourcing it is what makes an installation usable from a fresh shell.

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

Nothing reads ``MOTION_SPEC_PREFIX`` back: where a workspace installs is not a
separate fact from the workspace, so it is derived rather than configured, and
exported only so a shell and a build can see it.

A workspace can keep its settings in a ``motion-spec.config.toml`` at its root
instead of in the shell. ``motion-spec setup`` writes a commented sample the
first time, ``motion-spec config --init`` writes one on demand, and
``motion-spec config`` prints what is in force and where each value came from:

.. code-block:: console

   $ motion-spec config
   file: /home/you/ws/motion-spec.config.toml
     workspace.root         /home/you/ws            (default)
     workspace.generations  /home/you/ws/gen-out    (file)
     ros.workspace          True                    (file)

A command-line option overrides an environment variable, which overrides the
file, which overrides the built-in default. The file also answers the workspace
question on its own: with one at the root, ``setup`` needs no ``--workspace``
and no ``$MOTION_SPEC_WS``. An unknown section or key is an error rather than a
setting that silently does nothing. Paths are relative to the file.

.. list-table::
   :header-rows: 1
   :width: 100%

   * - Key
     - Meaning
   * - ``[workspace] root``, ``generations``, ``environment``
     - The workspace, where generations go, and the environment file.
   * - ``[ros] workspace``, ``distro``
     - Build CMake packages with colcon (the default for ``--ros``), and
       against which ``/opt/ros``.

``setup`` itself has no section: how it installs is said on its command line,
every time, where it can be seen.

.. list-table::
   :header-rows: 1
   :width: 100%

   * - Flag
     - Default
   * - ``--dev``
     - off: Python packages installed as snapshots
   * - ``--real``
     - off: ``motion_spec.real.repos`` is not installed
   * - ``--build-type``
     - ``$MOTION_SPEC_BUILD_TYPE``, else ``RelWithDebInfo``
   * - ``-j``/``--jobs``
     - ``$CMAKE_BUILD_PARALLEL_LEVEL``, else from cores and memory
   * - ``--ros``/``--no-ros``
     - ``[ros] workspace``

What is installed is what the manifests list, and cmake arguments are in
``colcon.meta``.

Two variables are yours to set, and sourcing the file sets the first:

.. list-table::
   :header-rows: 1
   :width: 100%

   * - Variable
     - Meaning
   * - ``MOTION_SPEC_WS``
     - The workspace. ``setup`` installs into its ``install/``, its setup script points
       ``MOTION_SPEC_GEN`` at its ``generations/``, and ``stst`` is looked for in its ``install/bin``
       when it is not on ``PATH``. Required by ``setup`` unless ``--workspace``
       is passed.
   * - ``MOTION_SPEC_GEN``
     - Where generations are written, and the root every provenance location is a path
       from. The setup script sets it to the workspace's ``generations/``. Unset, ``gen``,
       ``run``, ``rerun`` and ``dashboard`` stop and say so rather than writing to the
       working directory, where generations accumulate unnoticed and ``rerun`` cannot
       find them again.

Everything else is optional: ``MOTION_SPEC_ENV`` (the file to source, below),
``MOTION_SPEC_BUILD_TYPE`` (``CMAKE_BUILD_TYPE`` for a generated build) and
``ROS_DISTRO``.

What the tools themselves printed is kept, in the workspace or with the
generation it was about:

.. list-table::
   :header-rows: 1
   :width: 100%

   * - Command
     - Console kept in
   * - ``setup`` (vcs, cmake, colcon, ant, pip)
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
to read afterwards rather than something that scrolled past. A ``setup`` log
opens with the command line, the workspace, the prefix, the manifests and
repositories, the
build type, the job count and the machine's cores and RAM, and closes with the
exit status — or with the signal, which is what identifies a build the kernel
killed for memory.

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

Example models
==============

The example models ship inside ``motion_spec_dsl``, so ``setup`` already
installed them. ``motion-spec examples`` copies them out of the package and into
the workspace, where they are yours to edit:

.. code-block:: console

   $ motion-spec examples                     # into WORKSPACE/src/ms-examples
   $ motion-spec examples --into ~/scratch    # or anywhere else

It adds only. A file already at the destination is kept and reported, never
overwritten, so running it again after an edit brings in what is new and leaves
your work alone.

They are read from wherever ``motion_spec_dsl`` is installed: under ``--dev``
that is your tree, so the models you copy are the models you are editing.

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

These tables mirror what ``motion-spec health`` checks. What ``setup`` provides —
rdf-utils, the DSL compilers, rec, STSTv4, coord2b, Orocos KDL, mjkdl and
the device drivers — is fetched from the URL its manifest pins, which ``health``
also names as its source.

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
       `RDFLib <https://github.com/RDFLib/rdflib>`_, and
       `rdf-utils <https://github.com/minhnh/rdf-utils>`_
     - CLI, RDF loading, and IR
   * - DSL
     - `motion-spec-dsl <https://github.com/secorolab/motion-spec-dsl>`_,
       `textX <https://github.com/textX/textX>`_,
       `coord-dsl <https://github.com/secorolab/coord-dsl>`_, and
       `scene-dsl <https://github.com/secorolab/scene-dsl>`_
     - Compiling ``.robmot``, ``.fsm``, and ``.scenex`` sources
   * - Validation
     - `pySHACL <https://github.com/RDFLib/pySHACL>`_
     - ``motion-spec check``
   * - Telemetry
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
   * - `vcstool <https://github.com/dirk-thomas/vcstool>`_
     - Importing what the manifests list (``python3-vcstool``)
   * - A JDK and `Apache Ant <https://ant.apache.org/>`_
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
``mjkdl``; Eigen and toml++ come from apt.

Target dependencies
-------------------

.. list-table:: Target-specific build and runtime dependencies
   :header-rows: 1
   :width: 100%

   * - Target
     - Dependencies
   * - MuJoCo
     - `mjkdl <https://github.com/vamsikalagaturu/mjkdl>`_,
       at the version the generated ``CMakeLists.txt`` pins, with what its own
       build asks the system for: `GLFW <https://www.glfw.org/>`_ and OpenGL
       for the viewer, `EGL <https://www.khronos.org/egl>`_ for the headless
       video recorder, and `ffmpeg <https://ffmpeg.org/>`_ on ``PATH`` at run
       time, which encodes the recorder's frames and the ROS camera recordings.
       From apt: ``libglfw3-dev libgl-dev libegl-dev ffmpeg``
   * - robif2b
     - `robif2b <https://github.com/secorolab/robif2b>`_,
       `urdfdom_headers <https://github.com/ros/urdfdom_headers>`_, and
       `urdfdom <https://github.com/ros/urdfdom>`_
   * - robif2b devices
     - `serial <https://github.com/secorolab/serial>`_ and
       `robotiq_driver_noros <https://github.com/secorolab/robotiq_driver_noros>`_,
       which drives both the gripper and the force-torque sensor. Installed
       with ``motion-spec setup --real``, and each robif2b wrapper is turned on
       with its own flag in ``colcon.meta``.

The device drivers are optional: robif2b builds each wrapper only when its flag
is on. A model's ROS interface packages are not listed — the generated
``CMakeLists.txt`` asks for whichever ones that model declares. ``hddc2b`` is
optional for base solvers and is not a core health check.

Code generation
===============

C++ generation requires STST and ``protoc``. ``motion-spec setup`` installs the
pinned STSTv4 with everything else; to install or repair only it:

.. code-block:: console

   $ motion-spec setup STSTv4
   $ motion-spec setup STSTv4 --force   # rebuild a broken one

The launcher is written to ``WORKSPACE/install/bin/stst``, which the environment
file puts on ``PATH``. It is a two-line script naming the jar it runs, so a
launcher left behind by an installation that has since been deleted keeps
resolving on ``PATH`` and fails at the jar — ``setup STSTv4 --force`` replaces
it. STST setup requires a JDK and Ant.

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

The health profiles are ``base``, ``telemetry``, ``dsl``, ``codegen``, ``ros``,
``build``, and ``runtime``. Build and runtime are
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

``pip install`` of motion-spec on its own gives the CLI and its PyPI
dependencies, and nothing that is on no index: no rdf-utils, no DSL compiler,
no rec. It cannot load a model, build or run a controller until ``motion-spec
setup`` has installed the manifest — use the workspace above for that.

.. code-block:: console

   $ python -m pip install /path/to/motion-spec
   $ motion-spec install dashboard

``motion-spec install`` adds the package's optional features:
``motion-spec install all`` installs every one of them.

Development checkout
====================

The installation above is already editable — edits to ``src/motion-spec`` and,
under ``--dev``, to everything in ``src/thirdparty/`` take effect without
reinstalling. To reinstall motion-spec by hand:

.. code-block:: console

   $ cd ws
   $ source .venv/bin/activate
   $ python -m pip install -e src/motion-spec

Build these docs locally with:

.. code-block:: console

   $ python -m pip install -e "src/motion-spec[docs]"
   $ sphinx-build -W -b html src/motion-spec/docs/sphinx/source \
       src/motion-spec/docs/_build/html
