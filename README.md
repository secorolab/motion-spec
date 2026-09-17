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

## Install

```bash
python3 -m venv ~/ws/.venv && source ~/ws/.venv/bin/activate
pip install "motion_spec @ git+https://github.com/secorolab/motion-spec.git@dev"
export MOTION_SPEC_WS=~/ws
motion-spec setup
source ~/ws/setup-motion-spec.bash                               # .zsh under zsh
motion-spec health
motion-spec examples                                             # models to run, in src/ms-examples
```

`setup` installs everything motion-spec builds against — no second repository, no workspace
tool — into `$MOTION_SPEC_WS/install`, and writes the environment file that is the one step
between a new shell and a working workspace. Naming no workspace is an error rather than a
guess: nothing is installed into a location you did not choose.

| | |
|---|---|
| `--dev` | check every component out into `WORKSPACE/src` and install the Python ones editable |
| `--ros` | build with colcon, and source the distro and the overlay instead of exporting paths |
| uv | `uv venv` and `uv pip` in place of venv and pip |

Each is one flag and a couple of lines of setup:
**[Setup](https://secorolab.github.io/motion-spec/setup.html)**.

### In a workspace you already have

`setup` adopts whatever is already in `ws/src/<repository>` — from an earlier `--dev` run, from
your own clones, or from a `vcs import` of the pins, which the manifest's format supports. It
never moves a checkout and never touches a working tree: it fetches, then builds what is there.

| In `ws/src` | `motion-spec setup` | `motion-spec setup --dev` |
|---|---|---|
| a C++ package, clean, on the pinned commit | built where it stands; the next run is a no-op | same |
| a C++ package on some other ref | built as it stands, with a warning naming the ref | same |
| a C++ package with uncommitted changes | your edits are built, with a warning — and rebuilt on every run, since no commit describes them | same |
| nothing for that package | cloned into `WORKSPACE/.ms-sources`, or pip-installed from the pin for the DSLs | cloned into `ws/src` |
| a motion-spec-dsl or scene-dsl checkout | installed editable from your checkout | same |
| a directory that is not a git checkout | skipped with a warning, and `setup` exits 1 | same |

So `--dev` decides only where a *missing* source is cloned. Whatever is already in `ws/src` is
what gets built and installed, in either mode — including a DSL you are editing, which is
installed editable so your edits are live. `--clean` removes builds, installed files and
markers, never a source tree.

## Requirements

What every model needs, whichever target it drives:

| Stage | Needs | For |
|---|---|---|
| Python | 3.11+, with Click, RDFLib, rdf-utils, Jinja, pySHACL and protobuf | the CLI, RDF loading, validation, the IR |
| Authoring | motion-spec-dsl, scene-dsl, coord-dsl, textX | compiling `.robmot`, `.fsm` and `.scenex` sources |
| Generation | STSTv4 and `protoc` | rendering the C++, and the frame-log codec |
| | Git, a JDK and Ant | building STSTv4 itself — a JRE is not enough |
| Build | CMake and a C++20 compiler | configuring and compiling a generated controller |
| | coord2b, Eigen, Orocos KDL, toml++ | what every generated controller links |
| MuJoCo | mj_kdl_wrapper, GLFW, OpenGL, EGL, `ffmpeg` | the simulation, its viewer, and the video recorder |

The Python and authoring packages arrive with `pip install`; `motion-spec setup` builds STSTv4,
coord2b, Orocos KDL and mj_kdl_wrapper; Eigen, toml++ and `protoc` come from apt. Orocos KDL
must be the [secorolab fork](https://github.com/secorolab/orocos_kinematics_dynamics) —
generated controllers call the Vereshchagin solvers with fixed joints, which
`liborocos-kdl-dev` does not carry.

Per-target and ROS dependencies are installed only when a model asks for them: mj_kdl_wrapper
for MuJoCo, robif2b and urdfdom for a real robot, `rclcpp` and friends for a model that
publishes a topic. `motion-spec health` checks all of it and, for anything missing, names what
it is for and the command that installs it.

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

## Citation

This is the companion code for *From Composable Models to Correct-by-Construction Software for
Contact-Rich Robotic Mobile-Manipulation Tasks*, IEEE Robotics and Automation Letters 10(10),
2025 — [10.1109/LRA.2025.3597864](https://doi.org/10.1109/LRA.2025.3597864). Please cite it if
you use this software.

```bibtex
@ARTICLE{11122613,
  author={Schneider, Sven and Kalagaturu, Vamsi and Bruyninckx, Herman and Hochgeschwender, Nico},
  journal={IEEE Robotics and Automation Letters},
  title={From Composable Models to Correct-by-Construction Software for Contact-Rich Robotic Mobile-Manipulation Tasks},
  year={2025},
  volume={10},
  number={10},
  pages={9894-9901},
  doi={10.1109/LRA.2025.3597864}}
```

## Acknowledgement

This work is part of a project that has received funding from the European Union's Horizon 2020 research and innovation programme SESAME under grant agreement No 101017258.
