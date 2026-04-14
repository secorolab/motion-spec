# SPDX-License-Identifier: MPL-2.0
"""C++ code generation for motion specification models via StringTemplate."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

from motion_spec.ir_gen import JSONEncoder


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
TEMPLATE_GROUP = PACKAGE_ROOT / "code-generator"
MODULE_TEMPLATE = "module"


def write_json(path: Path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, cls=JSONEncoder, indent=4) + "\n")


def render_template(
    stst_bin: str,
    template_name: str,
    payload_path: Path,
    output_path: Path,
    module_template: str = MODULE_TEMPLATE,
):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    stst_path = Path(stst_bin)
    run_cwd = stst_path.resolve().parent if stst_path.parent != Path(".") else PACKAGE_ROOT
    command = [
        stst_bin,
        "-s",
        "<>",
        "-t",
        str(TEMPLATE_GROUP),
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
            command,
            cwd=run_cwd,
            env=env,
            check=True,
            capture_output=True,
            text=True,
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

    output_path.write_text(result.stdout)


def copy_demo_support_files(output_dir: Path):
    for relative_path in [
        Path("code-generator/CMakeLists.txt"),
        Path("thirdparty/orocos-kdl/chainhdsolver_vereshchagin_fext.hpp"),
        Path("thirdparty/orocos-kdl/chainhdsolver_vereshchagin_fext.cpp"),
        Path("thirdparty/kinova/GEN3_URDF_V12.urdf"),
    ]:
        source = PACKAGE_ROOT / relative_path
        target = output_dir / relative_path.name
        shutil.copy2(source, target)


def load_ir(input_path: Path):
    with input_path.open() as handle:
        return json.load(handle)


def generate_code(ir_path: Path, output_dir: Path, mode: str, stst_bin: str):
    ir = load_ir(ir_path)

    payload_dir = output_dir / ".stst"
    payload_dir.mkdir(parents=True, exist_ok=True)

    output_ir_path = output_dir / "ir.json"
    write_json(output_ir_path, ir)

    render_template(stst_bin, "runtime_header", output_ir_path, output_dir / "runtime.hpp")
    render_template(stst_bin, "shared_state_header", output_ir_path, output_dir / "shared_state.hpp")
    if ir.get("has_mobile_base"):
        render_template(stst_bin, "mobile_base_cycle_header", output_ir_path, output_dir / "mobile_base_cycle.hpp")

    for motion in ir["motions"]:
        payload = {
            "motion": motion,
            "closures": ir["closures"],
            "views": ir["views"],
            "wrench_outputs": ir["wrench_outputs"],
            "base_velocity_solvers": ir["base_velocity_solvers"],
            "base_force_solvers": ir["base_force_solvers"],
            "has_mobile_base": ir["has_mobile_base"],
        }
        payload_path = payload_dir / f"motion_{motion['id']}.json"
        write_json(payload_path, payload)
        render_template(stst_bin, "motion_header", payload_path, output_dir / f"motion_{motion['id']}.hpp")

    if mode == "app":
        render_template(stst_bin, "app_main", output_ir_path, output_dir / "main.cpp")
        copy_demo_support_files(output_dir)


def main():
    parser = argparse.ArgumentParser(
        description="Generate C++ code from motion-spec IR using StringTemplate.",
    )
    parser.add_argument("input", help="Previously generated IR JSON path")
    parser.add_argument(
        "-m",
        "--mode",
        choices=["headers", "app"],
        default="headers",
        help="Generate header-only motion modules or a sequential demo application",
    )
    parser.add_argument("-o", "--output-dir", required=True, help="Directory for generated C++ files")
    parser.add_argument(
        "--stst-bin",
        default="stst",
        help="Path to the STSTv4 executable used to render StringTemplate groups",
    )

    args = parser.parse_args()

    try:
        generate_code(
            ir_path=Path(args.input).resolve(),
            output_dir=Path(args.output_dir).resolve(),
            mode=args.mode,
            stst_bin=args.stst_bin,
        )
    except RuntimeError as exc:
        print(f"Code generation failed: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
