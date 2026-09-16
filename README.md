# motion-spec

[![test](https://github.com/secorolab/motion-spec/actions/workflows/test.yml/badge.svg?branch=dev)](https://github.com/secorolab/motion-spec/actions/workflows/test.yml)
[![docs](https://github.com/secorolab/motion-spec/actions/workflows/gh-pages.yml/badge.svg?branch=dev)](https://github.com/secorolab/motion-spec/actions/workflows/gh-pages.yml)

Validate, compile, build, run, and inspect guarded robot motion specifications.

`motion-spec` consumes the RDF dataset that
[motion-spec-dsl](https://github.com/secorolab/motion-spec-dsl) emits from a `.robmot`
model, validates it against the [metamodels](https://github.com/secorolab/metamodels) with
SHACL, lowers it to an intermediate representation, and generates the C++ controller that
runs it. Each execution is archived as a run that carries its own provenance: the model and
contracts it came from, and a full-rate frame log that can be replayed or recovered back
into RDF.

Companion code for the RAL paper *From Composable Models to Correct-by-Construction Software
for Contact-Rich Robotic Mobile-Manipulation Tasks*.

## Installation

`motion-spec` installs itself and everything it builds against — no second repository, no
workspace tool. Two choices decide the commands: whether you are **using** it or **working on
it** (`--dev`), and whether the workspace is a **colcon** one (`--ros`).

### Use it

The Python components come from their pinned git refs and the C++ ones are built in
`WORKSPACE/.ms-sources`; nothing lands in `src/`.

```bash
python3 -m venv ~/ws/.venv && source ~/ws/.venv/bin/activate
pip install "motion_spec @ git+https://github.com/secorolab/motion-spec.git@dev"
export MOTION_SPEC_WS=~/ws
motion-spec setup
source ~/ws/setup-motion-spec.bash                               # .zsh under zsh
motion-spec health
```

### Work on it

`--dev` checks every component out into `WORKSPACE/src` and installs the Python ones editable.
Clone motion-spec itself there too — `setup` cannot check out the code it is running, and says
so if you do not.

```bash
mkdir -p ~/ws/src
git clone -b dev https://github.com/secorolab/motion-spec.git ~/ws/src/motion-spec
python3 -m venv ~/ws/.venv && source ~/ws/.venv/bin/activate
pip install -e ~/ws/src/motion-spec
export MOTION_SPEC_WS=~/ws
motion-spec setup --dev
source ~/ws/setup-motion-spec.bash
```

### With ROS

Add `--ros` to either. The environment must be able to reach both ROS and the distribution's
Python packages, so build the venv on the system interpreter — `setup` refuses before cloning
anything if it cannot:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages ~/ws/.venv && source ~/ws/.venv/bin/activate
pip install "motion_spec @ git+https://github.com/secorolab/motion-spec.git@dev"
export MOTION_SPEC_WS=~/ws
motion-spec setup --ros                                          # add --dev to work on it
source ~/ws/setup-motion-spec.bash
```

`--ros` writes a `colcon.meta`, builds each component with `colcon build --packages-select` in
motion-spec's dependency order, and writes an environment file that sources the distro and the
overlay instead of exporting paths. Set `[ros] workspace = true` to keep it; the flag applies
to one run. The distribution comes from `$ROS_DISTRO`, then `[ros] distro`, then the only one
installed.

### With uv

Same commands with `uv venv` and `uv pip`. `uv venv` does not activate anything, so source the
activation yourself; after that `uv pip install` needs no `--python`.

```bash
uv venv ~/ws/.venv                                     # plain
source ~/ws/.venv/bin/activate
uv pip install "motion_spec @ git+https://github.com/secorolab/motion-spec.git@dev"
```

For ROS, build it on the system interpreter:

```bash
uv venv --python /usr/bin/python3 --system-site-packages ~/ws/.venv
```

`--python /usr/bin/python3` is the part that matters: without it uv builds the environment on
its own CPython, whose system packages are not the distribution's, and `--system-site-packages`
reaches nothing from apt. A uv environment has no `pip` in it either — `setup` notices and uses
`uv pip install --python` for the components it installs.

### What setup does

It clones each dependency at the version
[`motion_spec.repos`](src/motion_spec/motion_spec.repos) pins, builds it in `WORKSPACE/build`
and installs it into `WORKSPACE/install`; run it again after a pin moves and only what changed
is rebuilt. Name components to do fewer: `motion-spec setup stst mj_kdl_wrapper`. The workspace
comes from `$MOTION_SPEC_WS` or `--workspace`, and naming neither is an error rather than a
guess — nothing is ever installed into a location you did not choose. A source checkout that is
already there is built only when it is clean and on the pinned commit, and otherwise reported
and skipped rather than moved. Before the first clone it checks what it cannot install itself
and stops with the one apt line that fixes it. The environment file it writes activates the
environment setup ran in, so sourcing it is the only step between a new shell and a working
workspace. Anything `--clean` removes goes to the desktop trash, not away.
Use `motion-spec setup mj_kdl_wrapper --clear-cache` to discard that component's
CMake configuration and rebuild it after a compiler or dependency change.

`build`, `run`, `rerun` and `health` can source that file themselves — `--env <file>`, or
`$MOTION_SPEC_ENV`, or the nearest one above the generation — and run every subprocess under
what it left, so a ROS overlay reaches the build and the controller without a sourced shell.
`--no-env` turns it off. A run archives the file it used and the variables a build depends on.

ROS is optional, and the robot hardware drivers are not installed by `setup` — a model that
binds none of them never needs them. Full instructions: **[Setup](https://secorolab.github.io/motion-spec/setup.html)**.

## Requirements

| | |
|---|---|
| Python | 3.11+, with Click, RDFLib, rdf-utils, and Jinja |
| Authoring | motion-spec-dsl, coord-dsl, scene-dsl, textX |
| Generation | STSTv4 (needs Git, a JDK, Ant) and `protoc` |
| Build | CMake, a C++20 compiler, coord2b, Eigen, Orocos KDL, toml++ |
| MuJoCo target | mj_kdl_wrapper |
| Real-robot target | robif2b, urdfdom, urdfdom_headers |
| ROS (optional) | rclcpp, realtime_tools, rosidl_runtime_py, ament_package, PyYAML, colcon — only for a model that publishes a topic or drives an action |

Orocos KDL must be the [secorolab fork](https://github.com/secorolab/orocos_kinematics_dynamics):
generated controllers call the Vereshchagin solvers with fixed joints, which a distro
`liborocos-kdl-dev` configures against and then fails to build. `motion-spec setup` installs
that fork, coord2b and mj_kdl_wrapper from source; Eigen, toml++ and urdfdom come
from apt; the Robotiq and serial device drivers are needed only by a model that binds them.

`motion-spec health` checks all of the above and, for anything missing, names what it is for
and the command that installs it — including which ROS distributions are installed under
`/opt/ros` and whether one is sourced. Optional Python features install with
`motion-spec install`, and the external tools and libraries with `motion-spec setup`.

## Documentation

Full documentation: **<https://secorolab.github.io/motion-spec/>**

- [Setup](docs/sphinx/source/setup.rst) — dependencies, installation, health checks
- [Pipeline and artifact concepts](docs/sphinx/source/concepts.rst) — commands, generation layout, archives
- [Dashboard](docs/sphinx/source/dashboard.rst) — browsing generations, live runs, replay
- [CLI tutorials](docs/sphinx/source/tutorials/index.rst)
- [DSL concepts and tutorials](docs/sphinx/source/dsl/index.md)
- [Original RAL tutorial](docs/sphinx/source/tutorial.rst)

## Contributors

* [Sven Schneider](https://github.com/svenschneider)
* [Vamsi Kalaagaturu](https://github.com/vamsikalagaturu)

## License

All Python scripts are licensed under the Mozilla Public License 2.0 (see [`LICENSE.MPL-2.0`](LICENSE.MPL-2.0))

The models are licensed under the MIT No Attribution (see [`LICENSE.MIT-0`](LICENSE.MIT-0)) License.

## Acknowledgement

This work is part of a project that has received funding from the European Union's Horizon 2020 research and innovation programme SESAME under grant agreement No 101017258.
