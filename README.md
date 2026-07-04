# motion-spec

Code base for RAL paper: *From Composable Models to Correct-by-Construction Software for Contact-Rich Robotic Mobile-Manipulation Tasks*

## Installation

```bash
pip install -e .
```

## Usage

`motion-spec` is the code-generation half of the motion-spec toolchain. Its input is an
application manifest — a JSON-LD motion-spec graph plus its SHACL/ontology references —
produced by [`motion-spec-dsl`](../motion-spec-dsl) from a `.robmot` file. Its output is
C++: runtime/shared headers, one header per motion, and a `ref_main.cpp`.

The pipeline is three console scripts run in order.

```bash
# 1. SHACL-validate the manifest
motion-spec-check models/sc0a-right-arm.json

# 2. Lower the graph to an intermediate representation (IR)
motion-spec-ir-gen models/sc0a-right-arm.json -o gen/ir.json

# 3. Render C++ from the IR
motion-spec-codegen gen/ir.json --output-dir gen
```

`motion-spec-ir-gen` writes to a file with `-o FILE` (use `-o -` for stdout) or prints
with `--console`. `motion-spec-codegen` takes a previously generated IR JSON and renders
the headers via the `stst` StringTemplate engine (`--stst-bin` to point at a custom
binary). Pass `--help` to any script for the full flag set.

FSM wiring (monitor events bound to a coord2b state machine) is picked up automatically
from the `fsm_ir.json` that `textx generate --target jsonld` writes alongside the
manifest when the `.robmot` imports a `.fsm`.

## Documentation

The documentation comprises

* the [setup instructions](https://secorolab.github.io/motion-spec-ral/setup.html)
* a [tutorial](https://secorolab.github.io/motion-spec-ral/tutorial.html).

## Third-party software

* [`orocos-kdl`](thirdparty/orocos-kdl/) is a modified version that retains the GNU Lesser General Public License version 2.1 and originates from the OROCOS [Kinematics and Dynamics Library](https://github.com/orocos/orocos_kinematics_dynamics).
* [`GEN3_URDF_V12.urdf`](thirdparty/kinova/GEN3_URDF_V12.urdf) is licensed under the BSD-3-Clause license and originates from Kinova's [ros_kortex](https://github.com/Kinovarobotics/ros_kortex).

## Contributors

* [Sven Schneider](https://github.com/svenschneider)
* [Vamsi Kalaagaturu](https://github.com/vamsikalagaturu)

## License

All Python scripts are licensed under the Mozilla Public License 2.0 (see [`LICENSE.MPL-2.0`](LICENSE.MPL-2.0))

The models are licensed under the MIT No Attribution (see [`LICENSE.MIT-0`](LICENSE.MIT-0)) License.

## Acknowledgement

This work is part of a project that has received funding from the European Union's Horizon 2020 research and innovation programme SESAME under grant agreement No 101017258.
