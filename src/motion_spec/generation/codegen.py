# SPDX-License-Identifier: MPL-2.0
"""C++ code generation for motion specification models via StringTemplate."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

from motion_spec.classes.base import DataclassJSONEncoder
from motion_spec.generation.artifacts import write_introspection_artifacts

PACKAGE_ROOT = Path(__file__).resolve().parents[3]
MAIN_TEMPLATE = "main"
# The templates ship inside the package, so they sit beside it however it was installed.
TEMPLATES = Path(__file__).resolve().parents[1] / "templates"

# A render that reaches the end of the group still exits 0; anything it could not resolve is
# reported on stderr under this prefix. A group that fails to parse or a template that does not
# exist exits non-zero instead, so those never reach here.
ST_RUNTIME_ERROR = "Runtime Error:"
# ST4 reports a read of an absent JSON property this way while still returning null, and the
# templates rely on exactly that to ask whether an optional section was emitted at all
# (`<if(resources.by_kind.mobile_base)>`). It is the one runtime error that means nothing.
ST_OPTIONAL_READ = "no such property or can't access"


def write_json(path: Path, payload):
    """Write payload as pretty, dataclass-aware JSON, creating parent directories.

    ``sort_keys`` so dict-insertion order can never make two generations of the same model differ."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, cls=DataclassJSONEncoder, indent=4, sort_keys=True) + "\n")


def render_template(
    stst_bin: str,
    template_name: str,
    payload_path: Path,
    output_path: Path,
    module_template: str = MAIN_TEMPLATE,
):
    """Render a StringTemplate group template over a JSON payload to output_path via the STSTv4 runner."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    stst_path = Path(stst_bin)
    run_cwd = stst_path.resolve().parent if stst_path.parent != Path(".") else PACKAGE_ROOT
    command = [
        stst_bin,
        "-s",
        "<>",
        "-t",
        str(TEMPLATES),
        f"{module_template}.{template_name}",
        str(payload_path),
    ]
    env = os.environ.copy()
    java_tool_options = env.get("JAVA_TOOL_OPTIONS", "").strip()
    perfdata_opt = "-XX:-UsePerfData"
    if perfdata_opt not in java_tool_options.split():
        env["JAVA_TOOL_OPTIONS"] = f"{java_tool_options} {perfdata_opt}".strip()

    try:
        result = subprocess.run(
            command, cwd=run_cwd, env=env, check=True, capture_output=True, text=True
        )
    except FileNotFoundError as exc:
        raise RuntimeError(
            f"StringTemplate runner '{stst_bin}' was not found. Install STSTv4 or pass --stst-bin."
        ) from exc
    except subprocess.CalledProcessError as exc:
        error_text = exc.stderr.strip() or exc.stdout.strip()
        if "NoClassDefFoundError: org/antlr/runtime/Token" in error_text:
            raise RuntimeError(
                "StringTemplate runner is missing the ANTLR runtime "
                "(NoClassDefFoundError: org/antlr/runtime/Token). "
                "Install/fix STSTv4 so its launcher includes the required ANTLR jars, "
                "or pass a working --stst-bin."
            ) from exc
        raise RuntimeError(error_text) from exc

    _reject_dropped_output(template_name, output_path, result.stderr)
    output_path.write_text(_collapse_blank_lines(result.stdout))


def _reject_dropped_output(template_name: str, output_path: Path, stderr: str) -> None:
    """Fail on the ST4 errors that silently shorten the generated C++.

    ST4 exits 0 after a dispatch that found no template, a call with the wrong arity, or an
    undefined attribute: it renders the fragment as nothing and reports the reason on stderr.
    Written out, that is C++ which compiles and does less than the model asked -- the empty
    `switch` that stepped no motion and left a torque-controlled arm commanding zero. Reading
    the reason is the whole check.

    Raises:
        RuntimeError: the render reported anything other than a read of an absent property.
    """
    dropped = [
        line.strip()
        for line in stderr.splitlines()
        if line.startswith(ST_RUNTIME_ERROR) and ST_OPTIONAL_READ not in line
    ]
    if dropped:
        raise RuntimeError(
            f"{template_name}: StringTemplate emitted nothing where it meant to emit code, so "
            f"{output_path.name} would be written incomplete:\n  " + "\n  ".join(dropped)
        )


def compile_frame_log_proto(proto_path: Path) -> None:
    """Compile the generated frame_log.proto to C++ (frame_log.pb.{h,cc}) with protoc.

    The controller links libprotobuf and includes the generated header; protoc owns the
    wire format on the C++ side (the Python reader builds its message classes from schema).
    """
    proto_path = Path(proto_path)
    try:
        subprocess.run(
            [
                "protoc",
                f"--proto_path={proto_path.parent}",
                f"--cpp_out={proto_path.parent}",
                proto_path.name,
            ],
            check=True,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(
            "protoc was not found. Install the protobuf compiler (apt: protobuf-compiler) "
            "to generate the frame-log C++ codec."
        ) from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(exc.stderr.strip() or exc.stdout.strip()) from exc


def _collapse_blank_lines(text: str) -> str:
    """Normalize StringTemplate output: at most one blank line between blocks, single trailing newline.

    ST4 emits stray blank lines for empty conditional/list items (the literal newlines survive even when
    the rendered value is empty), which is impractical to eliminate per-template — collapse them here."""
    text = re.sub(r"[ \t]+\n", "\n", text)  # drop trailing whitespace on each line
    text = re.sub(r"\n{3,}", "\n\n", text)  # collapse runs of blank lines to a single blank
    return text.strip("\n") + "\n"


def load_ir(input_path: Path):
    """Load an IR JSON file into a dict."""
    with input_path.open() as handle:
        return json.load(handle)


def _adopt_fsm_state_order(ir: dict, *candidates: Path) -> None:
    """Index FSM states the way the runtime does.

    A frame's ``fsm_state`` is ``fsm->currentStateIndex`` -- coord-dsl's ``e_states`` enum, built
    from ``fsm_ir.json``. ir_gen re-derives its own state list by iterating the merged app graph,
    and rdflib hands it back in a different order, so the schema's indices (and every
    ``case <index>:`` generated from them) named the wrong state. Take the order from the artifact
    that defines the enum rather than re-deriving it.

    Events deliberately do NOT get the same treatment: a trigger's ``idx`` is ir_gen's own event
    index (``fsm_event_idx``, emitted into the C++ as ``events.record(N)``), so the
    recorded value and ``schema["fsm"]["events"]`` already share one index space. Reordering
    events here would desynchronise the schema from numbers already baked into the generated code.
    """
    fsm = ir["coordination"].get("fsm")
    fsm_ir_path = next((path for path in candidates if path.is_file()), None)
    if not fsm or fsm_ir_path is None:
        return
    states = json.loads(fsm_ir_path.read_text()).get("states")
    if not states:
        return
    if set(states) != set(fsm.get("states", [])):
        raise RuntimeError(
            f"{fsm_ir_path.name} states {sorted(states)} do not match the model's "
            f"{sorted(fsm.get('states', []))}"
        )
    fsm["states"] = list(states)


def generate_code(ir_path: Path, output_dir: Path, stst_bin: str):
    """Render every C++/artifact file for an IR: introspection headers, runtime and
    shared-state headers, the frame-log proto (compiled to C++), per-motion headers and
    main.cpp.

    ir.json is complete by construction in ir_gen (every codegen-facing field, incl. FSM
    wiring); codegen only loads it, writes artifacts, and renders.

    Raises:
        RuntimeError: the IR declares no FSM. The generated program is an FSM dispatcher.
    """
    ir = load_ir(ir_path)
    if not ir["coordination"].get("fsm"):
        raise RuntimeError(
            f"{ir_path}: the model declares no FSM; FSM-less execution is not supported. "
            "Coordinate the model with an FSM to generate C++."
        )
    # The pipeline moves fsm_ir.json into the controller dir before calling codegen; the
    # standalone `gen code <ir.json>` path leaves it beside the IR.
    _adopt_fsm_state_order(
        ir, Path(output_dir) / "fsm_ir.json", Path(ir_path).parent / "fsm_ir.json"
    )

    ir["communication"]["introspection_artifacts"] = write_introspection_artifacts(
        ir, ir_path=ir_path, output_dir=output_dir
    )

    headers_dir = output_dir / "headers"
    headers_dir.mkdir(parents=True, exist_ok=True)
    # The DSL-generated FSM header stays at the source root (like frame_layout.h); the bare
    # include in shared_state.hpp resolves it via the root include dir, so no headers/ copy
    # is needed — copying it there just duplicated the file in the tree and the archive.

    payload_dir = output_dir / ".stst"
    payload_dir.mkdir(parents=True, exist_ok=True)
    ir_payload_path = payload_dir / "ir.json"
    write_json(ir_payload_path, ir)

    render_template(
        stst_bin,
        "introspection_runtime_header",
        ir_payload_path,
        output_dir / "introspection_runtime.hpp",
    )
    render_template(
        stst_bin, "introspect_model_header", ir_payload_path, output_dir / "introspect_model.hpp"
    )
    render_template(stst_bin, "frame_layout_header", ir_payload_path, output_dir / "frame_layout.h")
    render_template(stst_bin, "frame_log_proto", ir_payload_path, output_dir / "frame_log.proto")
    compile_frame_log_proto(output_dir / "frame_log.proto")
    render_template(stst_bin, "runtime_header", ir_payload_path, headers_dir / "runtime.hpp")
    render_template(
        stst_bin, "controller_runtime_header", ir_payload_path, headers_dir / "controllers.hpp"
    )
    render_template(
        stst_bin, "shared_state_header", ir_payload_path, headers_dir / "shared_state.hpp"
    )
    if ir["resources"]["by_kind"].get("mobile_base"):
        render_template(
            stst_bin,
            "mobile_base_cycle_header",
            ir_payload_path,
            headers_dir / "mobile_base_cycle.hpp",
        )

    for motion in ir["coordination"]["motions"]:
        payload = {
            "motion": motion,
            "closures": ir["computation"]["closures"],
            "views": ir["computation"]["views"],
            "values": ir["computation"]["values"],
            "backend": ir["configuration"]["backend"],
            "solvers": ir["resources"]["by_id"],
        }
        payload_path = payload_dir / f"{motion['id']}.json"
        write_json(payload_path, payload)
        render_template(
            stst_bin, "motion_header", payload_path, headers_dir / f"{motion['id']}.hpp"
        )

    render_template(stst_bin, "main_source", ir_payload_path, output_dir / "main.cpp")
    cmake = "cmake_mj_kdl" if ir["configuration"]["backend"] == "mj_kdl" else "cmake_robif2b"
    render_template(stst_bin, cmake, ir_payload_path, output_dir / "CMakeLists.txt")
    # Both backends read deployment properties (the FT tare length) from the same config.
    render_template(
        stst_bin, "robot_config_header", ir_payload_path, output_dir / "robot_config.hpp"
    )
    # Only real hardware has serial devices the loop must not block on: the simulator's are
    # function calls. Which kinds those are is the backend template's decision
    # (backend_robif2b_io.stg); this mirror only decides whether the file exists at all.
    serial_device_kinds = {"Robotiq2F85", "RobotiqFT300s"}
    if ir["configuration"]["backend"] == "robif2b" and serial_device_kinds & set(
        ir["resources"]["device_kinds"]
    ):
        render_template(stst_bin, "device_io_header", ir_payload_path, output_dir / "device_io.hpp")


def main(argv: list[str] | None = None):
    """CLI entry point: render C++ from a previously generated IR JSON."""
    parser = argparse.ArgumentParser(
        prog="motion-spec codegen", description="Generate C++ header files from motion-spec IR."
    )
    parser.add_argument("input", help="Previously generated IR JSON path")
    parser.add_argument(
        "-o", "--output-dir", required=True, help="Directory for generated C++ files"
    )
    parser.add_argument(
        "--stst-bin",
        default="stst",
        help="Path to the STSTv4 executable used to render StringTemplate groups",
    )
    args = parser.parse_args(argv)

    try:
        generate_code(
            ir_path=Path(args.input).resolve(),
            output_dir=Path(args.output_dir).resolve(),
            stst_bin=args.stst_bin,
        )
    except RuntimeError as exc:
        print(f"Code generation failed: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
