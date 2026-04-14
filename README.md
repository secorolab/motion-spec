# motion-spec

Code base for RAL paper: *From Composable Models to Correct-by-Construction Software for Contact-Rich Robotic Mobile-Manipulation Tasks*

## Installation

```bash
pip install -e .
```

## Usage

The motion-spec package provides console scripts for validating and generating intermediate representations from motion specification models:

### Validation

Validate motion specification models against SHACL constraints:

```bash
motion-spec-check manifest.json
```

### IR Generation

Generate intermediate representation (IR) JSON from motion specification models:

```bash
# Print IR to console
motion-spec-ir-gen manifest.json --console

# Save IR to file
motion-spec-ir-gen manifest.json -o output.json

# Print IR to console (alternative)
motion-spec-ir-gen manifest.json --output -

# Get help
motion-spec-ir-gen --help
```

### C++ Code Generation

Generate C++ code through StringTemplate:

```bash
# Generate IR first
motion-spec-ir-gen models/sc0a-right-arm.json -o gen/ir.json

# Header-only mode: runtime/shared headers + one header per motion
motion-spec-codegen gen/ir.json --mode headers --output-dir gen

# Demo app mode
motion-spec-ir-gen models/sc1.json -o gen/ir.json
motion-spec-codegen gen/ir.json --mode app --output-dir gen
```

`motion-spec-codegen` takes a previously generated IR JSON file as input.
Header mode is intended for integration with an external FSM such as
`coord-dsl`; app mode emits a generated application.

### Examples

```bash
# Validate a model
motion-spec-check models/sc0a-right-arm.json

# Generate IR and save to file
motion-spec-ir-gen models/sc0a-right-arm.json -o ir-output.json

# Generate IR and view in console
motion-spec-ir-gen models/sc0a-right-arm.json --console
```

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
