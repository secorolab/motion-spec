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

from motion_spec.classes.entities import DataclassJSONEncoder
from motion_spec.classes.closures import closure_output_ids
from motion_spec.generation.artifacts import write_introspection_artifacts


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
MAIN_TEMPLATE = "main"
# The templates ship inside the package, so they sit beside it however it was installed.
TEMPLATES = Path(__file__).resolve().parents[1] / "templates"


def write_json(path: Path, payload):
    """Write payload as pretty, dataclass-aware JSON, creating parent directories."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, cls=DataclassJSONEncoder, indent=4) + "\n")


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

    output_path.write_text(_collapse_blank_lines(result.stdout))


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


def _escape_for_line_comment(text: str) -> str:
    """Escape newlines and comment terminators so authored text can't break out of a `///` comment."""
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    return text.replace("\n", "\\n").replace("*/", "*\\/")


def load_ir(input_path: Path):
    """Load an IR JSON file into a dict."""
    with input_path.open() as handle:
        return json.load(handle)


def _views_for_access(views: dict, direct_ids: set[str]) -> dict:
    """Index unambiguous MAP views by subobject for template access expressions.

    The IR keeps every view by its own identity. Templates instead resolve a quantity ID
    to its superobject expression; a subobject reused by several views is an ordinary
    shared quantity and must not select one of those views arbitrarily.
    """
    indexed: dict[str, dict | None] = {}
    for view in views.values():
        subobject_id = (view.get("subobject") or {}).get("id")
        if subobject_id and subobject_id not in direct_ids:
            previous = indexed.get(subobject_id)
            if previous is None and subobject_id in indexed:
                continue
            if previous is None:
                indexed[subobject_id] = view
                continue
            same_access = all(
                previous.get(field) == view.get(field)
                for field in ("superobject", "subspace", "axis", "direction")
            )
            if not same_access:
                indexed[subobject_id] = None
    return {id_: view for id_, view in indexed.items() if view is not None}


def generate_code(ir_path: Path, output_dir: Path, stst_bin: str):
    """Render every C++/artifact file for an IR: introspection headers, runtime and
    shared-state headers, the frame-log proto (compiled to C++), per-motion headers and
    ref_main.cpp.

    ir.json is complete by construction in ir_gen (every codegen-facing field, incl. FSM
    wiring); codegen only loads it, writes artifacts, and renders.
    """
    ir = load_ir(ir_path)

    ir["introspection_artifacts"] = write_introspection_artifacts(
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
    template_ir = dict(ir)
    direct_ids = {
        item["id"]
        for item in ir.get("shared_data", [])
        if item.get("id")
        and (
            item.get("value") is not None
            or (item.get("provenance") or {}).get("authored", False)
        )
    }
    direct_ids.update(
        snapshot["target_id"]
        for motion in ir.get("motions", [])
        for snapshot in motion.get("snapshots", [])
    )
    direct_ids.update(
        output_id
        for closure in ir.get("closures", {}).values()
        for output_id in closure_output_ids(closure)
    )
    template_ir["views"] = _views_for_access(ir["views"], direct_ids)
    write_json(ir_payload_path, template_ir)

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
        stst_bin, "shared_state_header", ir_payload_path, headers_dir / "shared_state.hpp"
    )
    if ir.get("has_mobile_base"):
        render_template(
            stst_bin,
            "mobile_base_cycle_header",
            ir_payload_path,
            headers_dir / "mobile_base_cycle.hpp",
        )

    for motion in ir.get("motions", []):
        motion_payload = dict(motion)
        if motion_payload.get("description"):
            motion_payload["description"] = _escape_for_line_comment(motion_payload["description"])
        payload = {
            "motion": motion_payload,
            "closures": ir["closures"],
            "views": template_ir["views"],
            "wrench_outputs": ir["wrench_outputs"],
            "platform_velocity_solvers": ir["platform_velocity_solvers"],
            "platform_force_solvers": ir["platform_force_solvers"],
            "has_mobile_base": ir["has_mobile_base"],
            "backend": ir["backend"],
            "declared_pose_components": motion.get("declared_pose_components", []),
            "pose_axis_error_groups": motion.get("pose_axis_error_groups", []),
        }
        payload_path = payload_dir / f"{motion['id']}.json"
        write_json(payload_path, payload)
        render_template(
            stst_bin, "motion_header", payload_path, headers_dir / f"{motion['id']}.hpp"
        )

    render_template(stst_bin, "ref_main", ir_payload_path, output_dir / "ref_main.cpp")
    if ir["backend"] == "mj_kdl":
        render_template(stst_bin, "cmake_mj_kdl", ir_payload_path, output_dir / "CMakeLists.txt")


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
