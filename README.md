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
mkdir -p ~/ws/src && cd ~/ws
git clone https://github.com/secorolab/motion-spec.git src/motion-spec
vcs import src < src/motion-spec/motion_spec.repos
vcs import src < src/motion-spec/motion_spec.real.repos             # with --real only
python3 -m venv .venv && source .venv/bin/activate
pip install -e src/motion-spec                                   # the CLI, PyPI dependencies only
motion-spec setup --workspace . --dev                            # builds and installs what src/ holds
source setup-motion-spec.bash                                    # .zsh under zsh
motion-spec health
motion-spec examples                                             # models to run, in src/ms-examples
```

With ROS, `--ros` builds the CMake packages with colcon, and the environment file sources the
distribution and the overlay; the venv has to see the distribution's Python packages:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv && source .venv/bin/activate
pip install -e src/motion-spec
motion-spec setup --workspace . --dev --ros
source setup-motion-spec.bash
```

What motion-spec builds against is listed in one place: the vcstool manifest
`motion_spec.repos` — rdf-utils, the DSL compilers and rec under `thirdparty/`,
then Orocos KDL, coord2b, mj_kdl_wrapper and STSTv4 — with the device drivers in
`motion_spec.real.repos`. `setup` fetches nothing: it pip-installs the Python packages the
imported checkouts hold into the active environment (or a `.venv` it creates), builds the CMake ones
into `install/` with the arguments in the workspace's `colcon.meta`, and writes the environment
file that is the one step between a new shell and a working workspace.

| | |
|---|---|
| `--dev` | Python packages installed editable (`pip install -e`); without it they install as snapshots. Sources are in `WORKSPACE/src` either way |
| `--real` | also install `motion_spec.real.repos`: serial, robotiq_driver_noros, robif2b |
| `--ros` | build with colcon, and source the distro and the overlay instead of exporting paths |
| `--force` | rebuild regardless, from a cleared CMake cache |
| `--build-type`, `-j` | CMake build type; parallel compile jobs |
| `REPOSITORIES...` | narrow the run to these manifest entries, by path or by name |

A checkout already at a manifest path is never moved: it is built as it stands, with a warning
when it is not on the pinned commit, and rebuilt on every run while it has uncommitted changes.
`--clean` removes builds, installed files and markers, never a source tree. Details:
**[Setup](https://secorolab.github.io/motion-spec/setup.html)**.

## Requirements

What every model needs, whichever target it drives:

| Stage | Needs | For |
|---|---|---|
| Python | 3.11+, with Click, RDFLib, rdf-utils, pySHACL, protobuf, PyYAML, zstandard and NumPy | the CLI, RDF loading, validation, the IR |
| Authoring | motion-spec-dsl, scene-dsl, coord-dsl, textX | compiling `.robmot`, `.fsm` and `.scenex` sources |
| Generation | STSTv4 and `protoc` | rendering the C++, and the frame-log codec |
| | Git, a JDK and Ant | building STSTv4 itself — a JRE is not enough |
| Build | CMake and a C++20 compiler | configuring and compiling a generated controller |
| | coord2b, Eigen, Orocos KDL, toml++ | what every generated controller links |
| MuJoCo | mj_kdl_wrapper, GLFW, OpenGL, EGL, `ffmpeg` | the simulation, its viewer, and the video recorder |

PyPI packages arrive with `pip install`; `motion-spec setup` installs rdf-utils, the authoring
packages and rec, and builds STSTv4, coord2b, Orocos KDL and mj_kdl_wrapper; vcstool, Eigen,
toml++ and `protoc` come from apt. Orocos KDL
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
* [Vamsi Kalagaturu](https://github.com/vamsikalagaturu)

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
