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

## Installation

```bash
pip install -e .
```

Install only the features you use:

```bash
pip install -e ".[validation]"       # motion-spec check
pip install -e ".[introspection]"    # archive, replay, and run tooling
pip install -e ".[all]"              # all end-user features
pip install -e ".[test]"             # test suite
pip install -e ".[docs]"             # documentation build
```

The base install provides RDF-to-IR and C++ generation. C++ generation still requires
the external `stst` executable; install its pinned version after installing motion-spec:

```bash
motion-spec setup
```

This installs under `~/.local` by default and requires `git`, `ant`, and Java. Use
`motion-spec setup --clean` to remove the managed STST installation. Generated protobuf
artifacts require `protoc`.

## Usage

`motion-spec` generates, builds, runs, and inspects motion models. The high-level commands
accept a `.robmot` model directly:

```bash
motion-spec gen model.robmot                   # DSL through generated C++
motion-spec gen ir model.robmot                # stop after IR generation
motion-spec gen code model.robmot              # explicit form of the first command
motion-spec build generation/<generation-id>   # configure and compile generated C++
motion-spec run model.robmot                    # generate, build, run, and archive
```

Use `-o generation/demo` with `gen` or `run` to choose a generation directory. Without
`-o`, each command creates a unique directory under `./generation/`. Runs are stored under
that generation's `runs/<run-id>/` directory.

The low-level commands remain available for individual pipeline stages:

```bash
motion-spec check generated/model-app.ld.json
motion-spec ir generated/model-app.ld.json -o generated/ir.json
motion-spec codegen generated/ir.json --output-dir generated
```

`motion-spec ir` writes to a file with `-o FILE` (use `-o -` for stdout) or prints
with `--console`. `motion-spec codegen` takes a previously generated IR JSON and renders
the headers via the `stst` StringTemplate engine (`--stst-bin` to point at a custom
binary). Pass `--help` to any script for the full flag set.

FSM wiring (monitor events bound to a coord2b state machine) is picked up automatically
from the `fsm_ir.json` that `textx generate --target jsonld` writes alongside the
manifest when the `.robmot` imports a `.fsm`.

## Standalone postmortem analysis

Generated controllers with `MOTION_SPEC_ENABLE_INTROSPECTION=ON` write a full-rate
protobuf frame log named `frame_log.pb` — a stream of length-delimited
`FrameLogRecord` messages (one `FrameLogHeader`, then one `RuntimeFrame` per tick).
Each run also archives `contract/frame_log.proto`, generated from that run's
`schema.json`: its `RuntimeFrame` uses semantic field names derived from the model
(e.g. `direction_ctrl_cg_support_z_x = 3000`, `home_pose = 5002`) instead of generic
`quantities`/`poses` arrays, so `protoc --decode` and other protobuf tooling read the
log without the motion-spec package. `schema.json` stays required: it carries the
RDF/provenance, units, and FSM/state metadata that field names alone do not replace.

For new runs, launch the generated executable through `motion-spec run`. It starts a
REC record before the executable is launched, sets `MOTION_SPEC_FRAME_LOG` to the
archive-local `frame_log.pb`, records the executable/source inputs as provenance,
packages the generated bundle, and verifies the archive after the process exits:

```bash
motion-spec run generation/pick-place --run-id run-001
```

Pass executable arguments after `--`:

```bash
motion-spec run generation/pick-place -- --scenario pick-place-single
```

For an already completed run, create a self-contained archive from the generated
controller directory and the frame log:

```bash
motion-spec archive logs/run-001 \
  --source-dir gen/controller \
  --run-id run-001 \
  --frame-log gen/controller/logs/frame_log.pb \
  --log-producer-executable gen/controller/build-introspection/main
```

Each generation owns its immutable inputs and build products once:

```text
generation/<generation-id>/
  generated/
    contract/       # schema, frame layout, frame-log protocol
    controller/     # generated C++ and CMake project
    model/          # JSON-LD graphs, FSM artifacts, and IR
    provenance/     # dsl.ld.json, coord-dsl.ld.json, motion-spec.ld.json
    source/         # authored DSL inputs
  build/
  runs/<run-id>/
    logs/           # frame log and health
    runtime/        # recovered runtime.ttl
    rec.ld.json
    manifest.json
```

Run manifests hash only run-owned artifacts and reference their generation's static
artifacts relatively. The frame log header carries `schema_hash`, checked against
`generated/contract/schema.json` before any frames are decoded; `motion-spec.ld.json`
is parsed as JSON-LD and validated against the local PROV SHACL shape.

Verify and summarize a copied archive without the original checkout:

```bash
motion-spec replay generation/demo/runs/run-001 --verify
motion-spec replay generation/demo/runs/run-001
```

Recover runtime provenance RDF from the frame log and archive metadata:

```bash
motion-spec replay logs/run-001/logs/frame_log.pb --recover-runtime-ttl
motion-spec archive logs/run-001 --verify
```

For custom analysis, load the archive metadata and stream decoded frames in Python.
`frames()` yields one decoded record dict per tick; the generated `schema.json` tells
you which controller/monitor slot is active in each FSM state. This example plots
whichever controller and fields you choose:

```python
import matplotlib.pyplot as plt

from motion_spec.introspection.replay import frames, load_archive

log = "logs/run-001/logs/frame_log.pb"
target_controller = "https://example.test/controller-uri"
fields = ("error", "output", "measured", "setpoint")

_run_dir, _manifest, schema, _layout = load_archive(log)
states = schema["fsm"]["states"]

series = {name: [] for name in fields}
time = []

for record in frames(log):
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
`record["quantities"]`, a dict keyed by semantic id; their metadata is in
`schema["quantities"]`.

Export all decoded frames as JSON Lines only when you need whole-frame analysis:

```bash
motion-spec replay logs/run-001/logs/frame_log.pb --jsonl > frames.jsonl
```

A `schema_hash` mismatch, a field number the schema does not define, a missing
artifact, or invalid RDF provenance is a hard error.

## Documentation

- [Setup](docs/sphinx/source/setup.rst)
- [Pipeline and artifact concepts](docs/sphinx/source/concepts.rst)
- [CLI tutorials](docs/sphinx/source/tutorials/index.rst)
- [DSL concepts and tutorials](docs/sphinx/source/dsl/index.md)
- [Original RAL tutorial](docs/sphinx/source/tutorial.rst)

## Third-party software

* [`GEN3_URDF_V12.urdf`](thirdparty/kinova/GEN3_URDF_V12.urdf) is licensed under the BSD-3-Clause license and originates from Kinova's [ros_kortex](https://github.com/Kinovarobotics/ros_kortex).

## Contributors

* [Sven Schneider](https://github.com/svenschneider)
* [Vamsi Kalaagaturu](https://github.com/vamsikalagaturu)

## License

All Python scripts are licensed under the Mozilla Public License 2.0 (see [`LICENSE.MPL-2.0`](LICENSE.MPL-2.0))

The models are licensed under the MIT No Attribution (see [`LICENSE.MIT-0`](LICENSE.MIT-0)) License.

## Acknowledgement

This work is part of a project that has received funding from the European Union's Horizon 2020 research and innovation programme SESAME under grant agreement No 101017258.
