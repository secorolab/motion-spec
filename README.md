# motion-spec

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

## Requirements

| | |
|---|---|
| Python | 3.10+, with Click, RDFLib, rdf-utils, and Jinja |
| Authoring | motion-spec-dsl, coord-dsl, scene-dsl, textX |
| Generation | STSTv4 (needs Git, Java, Ant) and `protoc` |
| Build | CMake, a C++20 compiler, coord2b, Eigen, Orocos KDL, kdl_parser, toml++ |
| MuJoCo target | mj_kdl_wrapper |
| Real-robot target | robif2b, urdfdom, urdfdom_headers |

Orocos KDL must be the [secorolab fork](https://github.com/secorolab/orocos_kinematics_dynamics):
generated controllers call the Vereshchagin solvers with fixed joints. The workspace packages
come from `grc_meta`; the Robotiq and serial device drivers are optional and needed only by a
model that binds them.

`motion-spec health` checks all of the above and, for anything missing, names what it is for
and the command that installs it. Optional features install with `motion-spec install`, and
the pinned STST with `motion-spec setup`.

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
