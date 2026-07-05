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

## Standalone postmortem analysis

Generated controllers with `MOTION_SPEC_ENABLE_INTROSPECTION=ON` write a full-rate
binary frame log named `frame_log.bin`. Treat that file as data, not as a standalone
format: postmortem analysis needs the generated `schema.json`, `frame_layout.json`,
`provenance.jsonld`, model graph, and generated source bundle that describe how to
decode and attribute the log.

For new runs, launch the generated executable through `motion-spec-run`. It starts a
REC record before the executable is launched, sets `MOTION_SPEC_FRAME_LOG` to the
archive-local `frame_log.bin`, records the executable/source inputs as provenance,
packages the generated bundle, and verifies the archive after the process exits:

```bash
motion-spec-run logs/run-001 \
  --source-dir gen/controller \
  --run-id run-001 \
  --executable gen/controller/build-introspection/main \
  --recover-runtime-ttl
```

Pass executable arguments after `--`:

```bash
motion-spec-run logs/run-001 \
  --source-dir gen/controller \
  --executable gen/controller/build-introspection/main \
  -- --scenario pick-place-single
```

For an already completed run, create a self-contained archive from the generated
controller directory and the frame log:

```bash
motion-spec-archive logs/run-001 \
  --source-dir gen/controller \
  --run-id run-001 \
  --frame-log gen/controller/logs/frame_log.bin \
  --log-producer-executable gen/controller/build-introspection/main
```

The archive layout is:

```text
logs/run-001/
  manifest.json
  rec.json
  contract/
    schema.json
    frame_layout.json
  controller/
    executable/
    source/
  logs/
    frame_log.bin
  model/
    model.jsonld
    ir.json
  provenance/
    static.jsonld
  runtime/
    runtime.ttl
  media/
```

`manifest.json` records hashes for the required artifacts. The frame log header is
checked against `schema.json` and `frame_layout.json` before any frames are decoded;
`provenance.jsonld` is parsed as JSON-LD and validated against the local PROV SHACL
shape.

Verify and summarize a copied archive without the original checkout:

```bash
motion-spec-replay logs/run-001/logs/frame_log.bin --verify
motion-spec-replay logs/run-001/logs/frame_log.bin
```

Recover runtime provenance RDF from the frame log and archive metadata:

```bash
motion-spec-replay logs/run-001/logs/frame_log.bin --recover-runtime-ttl
motion-spec-archive logs/run-001 --verify
```

For custom analysis, load the archive metadata and stream decoded frames in Python.
The generated `schema.json` tells you which controller/monitor slot is active in
each FSM state; `frame_log.bin` supplies the samples. This example plots whichever
controller and fields you choose:

```python
import matplotlib.pyplot as plt

from motion_spec.introspection.replay import frames, load_archive, to_record

log = "logs/run-001/logs/frame_log.bin"
target_controller = "https://example.test/controller-uri"
fields = ("error", "output", "measured", "setpoint")

_run_dir, _manifest, schema, _layout = load_archive(log)
states = schema["fsm"]["states"]

series = {name: [] for name in fields}
time = []

for flat, n_constraints, n_monitors, n_quantities, n_triggers in frames(log):
    record = to_record(flat, n_constraints, n_monitors, n_quantities, n_triggers)
    state_index = record["fsm_state"]
    if not 0 <= state_index < len(states):
        continue

    state_id = states[state_index]["id"]
    controllers = schema.get("by_state", {}).get(state_id, {}).get("controllers", [])

    for slot, controller in enumerate(controllers):
        if controller.get("uri") != target_controller:
            continue
        sample = record["constraints"][slot]
        if not sample["active"]:
            continue
        time.append(record["t"])
        for name in fields:
            series[name].append(sample[name])
        break

for name, values in series.items():
    plt.plot(time, values, label=name)
plt.xlabel("time [s]")
plt.legend()
plt.show()
```

Use the same pattern for other analyses: select a state from `schema["fsm"]`, inspect
`schema["by_state"][state_id]["controllers"]` or `["monitors"]`, then read the
matching slot from each decoded frame. Global quantity samples are in
`record["quantities"]`; their metadata is in `schema["quantities"]`.

Export all decoded frames as JSON Lines only when you need whole-frame analysis:

```bash
motion-spec-replay logs/run-001/logs/frame_log.bin --jsonl > frames.jsonl
```

The binary header magic is `MSFRMBIN`. A bad magic value, mismatched frame size, hash
mismatch, missing artifact, or invalid RDF provenance is a hard error.

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
