# SPDX-License-Identifier: MPL-2.0
"""C++ code generation for motion specification models via StringTemplate."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from importlib.metadata import PackageNotFoundError, distribution
from pathlib import Path
from urllib.parse import urlparse

from motion_spec.ir_gen import JSONEncoder


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
MODULE_TEMPLATE = "module"
DIST_NAME = "motion_spec"


def _path_endswith(path, relative_path: Path) -> bool:
    path_parts = Path(path).parts
    relative_parts = relative_path.parts
    return len(path_parts) >= len(relative_parts) and path_parts[-len(relative_parts) :] == relative_parts


def _distribution_path(relative_path: Path) -> Path | None:
    try:
        dist = distribution(DIST_NAME)
    except PackageNotFoundError:
        return None

    for package_path in dist.files or []:
        if _path_endswith(package_path, relative_path):
            candidate = Path(str(dist.locate_file(package_path)))
            if candidate.exists():
                return candidate
    return None


def _source_root_from_distribution() -> Path | None:
    try:
        direct_url = distribution(DIST_NAME).read_text("direct_url.json")
    except (PackageNotFoundError, FileNotFoundError):
        return None
    if not direct_url:
        return None
    try:
        url = json.loads(direct_url).get("url", "")
    except json.JSONDecodeError:
        return None
    parsed = urlparse(url)
    if parsed.scheme != "file":
        return None
    return Path(parsed.path)


def _source_path(relative_path: Path) -> Path | None:
    roots = [PACKAGE_ROOT, Path.cwd() / "src" / "motion-spec", Path.cwd()]
    source_root = _source_root_from_distribution()
    if source_root is not None:
        roots.insert(0, source_root)

    for root in roots:
        candidate = root / relative_path
        if candidate.exists():
            return candidate
    return None


def resource_path(relative_path: Path) -> Path:
    path = _distribution_path(relative_path) or _source_path(relative_path)
    if path is not None:
        return path

    raise RuntimeError(
        f"Could not locate required motion_spec resource '{relative_path}'. "
        "Reinstall the motion_spec package from this repository."
    )


def template_group() -> Path:
    return resource_path(Path("code-generator/module.stg")).parent


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
    templates = template_group()
    command = [
        stst_bin,
        "-s",
        "<>",
        "-t",
        str(templates),
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


def load_ir(input_path: Path):
    with input_path.open() as handle:
        return json.load(handle)


SUPPORTED_ROBOT_MODELS = {"KinovaGen3"}


def _validate_ir(ir: dict) -> None:
    unsupported = {
        s["robot_model"]
        for s in ir.get("arm_solvers", [])
        if s.get("robot_model") and s["robot_model"] not in SUPPORTED_ROBOT_MODELS
    }
    if unsupported:
        raise RuntimeError(
            f"Unsupported robot model(s): {', '.join(sorted(unsupported))}. "
            f"Supported: {', '.join(sorted(SUPPORTED_ROBOT_MODELS))}"
        )


def generate_code(ir_path: Path, output_dir: Path, stst_bin: str, backend: str = "robif2b"):
    ir = load_ir(ir_path)
    _validate_ir(ir)
    ir["backend"] = backend

    if backend == "mj_kdl":
        for solver in ir.get("arm_solvers", []):
            if solver.get("root_acc"):
                solver["gravity"] = [-v for v in solver["root_acc"]]
        for motion in ir.get("motions", []):
            for solver in motion.get("arm_solvers", []):
                if solver.get("root_acc"):
                    solver["gravity"] = [-v for v in solver["root_acc"]]

    headers_dir = output_dir / "headers"
    headers_dir.mkdir(parents=True, exist_ok=True)

    payload_dir = output_dir / ".stst"
    payload_dir.mkdir(parents=True, exist_ok=True)
    ir_payload_path = payload_dir / "ir.json"
    write_json(ir_payload_path, ir)

    render_template(stst_bin, "runtime_header", ir_payload_path, headers_dir / "runtime.hpp")
    render_template(stst_bin, "shared_state_header", ir_payload_path, headers_dir / "shared_state.hpp")
    if ir.get("has_mobile_base"):
        render_template(stst_bin, "mobile_base_cycle_header", ir_payload_path, headers_dir / "mobile_base_cycle.hpp")

    for motion in ir["motions"]:
        payload = {
            "motion": motion,
            "closures": ir["closures"],
            "views": ir["views"],
            "wrench_outputs": ir["wrench_outputs"],
            "base_velocity_solvers": ir["base_velocity_solvers"],
            "base_force_solvers": ir["base_force_solvers"],
            "has_mobile_base": ir["has_mobile_base"],
            "backend": backend,
        }
        payload_path = payload_dir / f"{motion['id']}.json"
        write_json(payload_path, payload)
        render_template(stst_bin, "motion_header", payload_path, headers_dir / f"{motion['id']}.hpp")

    render_template(stst_bin, "ref_main", ir_payload_path, output_dir / "ref_main.cpp")
    if backend == "mj_kdl":
        render_template(stst_bin, "cmake_mj_kdl", ir_payload_path, output_dir / "CMakeLists.txt")


def main():
    parser = argparse.ArgumentParser(
        description="Generate C++ header files from motion-spec IR.",
    )
    parser.add_argument("input", help="Previously generated IR JSON path")
    parser.add_argument("-o", "--output-dir", required=True, help="Directory for generated C++ files")
    parser.add_argument(
        "--stst-bin",
        default="stst",
        help="Path to the STSTv4 executable used to render StringTemplate groups",
    )
    parser.add_argument(
        "--backend",
        choices=("robif2b", "mj-kdl"),
        default="robif2b",
        help="Robot runtime backend for generated reference code.",
    )

    args = parser.parse_args()

    try:
        generate_code(
            ir_path=Path(args.input).resolve(),
            output_dir=Path(args.output_dir).resolve(),
            stst_bin=args.stst_bin,
            backend=args.backend.replace("-", "_"),
        )
    except RuntimeError as exc:
        print(f"Code generation failed: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
