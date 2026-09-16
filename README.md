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
workspace tool:

```bash
mkdir -p ws/src
git clone git@github.com:secorolab/motion-spec.git ws/src/motion-spec
python3 -m venv ws/.venv && source ws/.venv/bin/activate
pip install -e ws/src/motion-spec            # the CLI and the DSL compilers
motion-spec install all                      # validation, recording, dashboard, replay
motion-spec health                           # prints the one apt line for what is missing
motion-spec setup --workspace ws             # STST and the C++ libraries, into ws/install
source ws/setup-motion-spec.bash             # or .zsh, whichever setup wrote
```

`setup` clones each dependency at the version
[`motion_spec.repos`](src/motion_spec/motion_spec.repos) pins into `WORKSPACE/src` as ordinary
workspace packages (only STSTv4, which colcon cannot build, goes under `src/thirdparty` with a
`COLCON_IGNORE`), builds it in
`WORKSPACE/build` and installs it into `WORKSPACE/install`; run it again after the pin moves
and only what changed is rebuilt. Name components to do fewer:
`motion-spec setup stst mj_kdl_wrapper`. The workspace comes from `$MOTION_SPEC_WS` or
`--workspace`, and naming neither is an error rather than a guess — nothing is ever installed
into a location you did not choose. A source checkout that is already there is built only when
it is clean and on the pinned commit, and otherwise reported and skipped rather than moved.
One environment file is written to the workspace root, for the shell in force, putting the
install prefix on `PATH`, `CMAKE_PREFIX_PATH` and `LD_LIBRARY_PATH` — which is how a generated
controller finds its libraries. Anything `--clean` removes goes to the desktop trash, not away.

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
| ROS (optional) | rclcpp, realtime_tools, rosidl_runtime_py — only for a model that publishes a topic or drives an action |

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
