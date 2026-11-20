# SPDX-License-Identifier: MPL-2.0
"""C++ code generation for motion specification models via StringTemplate."""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
from collections.abc import Iterator
from pathlib import Path

from motion_spec.classes.base import DataclassJSONEncoder
from motion_spec.generation.artifacts import write_telemetry_artifacts
from motion_spec.rdf_parser.deployment import AGENT_HOME_KEY, CONFIG_POSE_FIELDS
from motion_spec.telemetry import frame_log_pb

CONFIG_CHECK = "check_config.py"

PACKAGE_ROOT = Path(__file__).resolve().parents[3]
TEMPLATES = Path(__file__).resolve().parents[1] / "templates"

# stderr prefix of what a render that still exits 0 could not resolve
ST_RUNTIME_ERROR = "Runtime Error:"
# absent optional property: templates test sections this way, harmless
ST_OPTIONAL_READ = "no such property or can't access"


def for_templates(o):
    """Every empty list as null: StringTemplate's `<if()>` holds a JSON list true even when empty."""
    if isinstance(o, dict):
        return {key: for_templates(value) for key, value in o.items()}
    if isinstance(o, list):
        return [for_templates(value) for value in o] or None
    return o


def write_payload(path: Path, payload) -> None:
    """Write a template payload, with empty lists read as absent."""
    readable = for_templates(json.loads(json.dumps(payload, cls=DataclassJSONEncoder)))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(readable, indent=4) + "\n")


def render_template(
    stst_bin: str, group: str, template_name: str, payload_path: Path, output_path: Path
) -> Path:
    """Render GROUP's template over a JSON payload to output_path via the STSTv4 runner.

    GROUP is a path under the templates, without `.stg`: a backend's root, `backend/<name>/main`,
    for everything the program is generated from.
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    stst_path = Path(stst_bin)
    run_cwd = stst_path.resolve().parent if stst_path.parent != Path(".") else PACKAGE_ROOT
    command = [
        stst_bin,
        "-s",
        "<>",
        "-t",
        str(TEMPLATES),
        f"{group}.{template_name}",
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

    # ST4 renders an unresolved fragment as nothing and still exits 0
    dropped = [
        line.strip()
        for line in result.stderr.splitlines()
        if line.startswith(ST_RUNTIME_ERROR) and ST_OPTIONAL_READ not in line
    ]
    if dropped:
        raise RuntimeError(
            f"{template_name}: StringTemplate emitted nothing where it meant to emit code, so "
            f"{output_path.name} would be written incomplete:\n  " + "\n  ".join(dropped)
        )
    output_path.write_text(result.stdout)
    return output_path


class MissingAssets(RuntimeError):
    """The MJCF files a model names that are not on this machine.

    Carries both, so a caller can report them its own way; the message is what the CLI prints,
    and needs no stack beside it.
    """

    def __init__(self, ir_path: Path, paths: list[str]) -> None:
        self.ir_path = ir_path
        self.paths = paths
        super().__init__(
            f"{ir_path}: the scene names MJCF assets that are not there:\n"
            + "\n".join(f"  {path}" for path in paths)
        )


def asset_nodes(node) -> Iterator[dict]:
    """Every mapping in the IR that names an MJCF file, wherever it sits."""
    if isinstance(node, dict):
        if isinstance(node.get("path"), str) and node["path"].endswith(".xml"):
            yield node
        for value in node.values():
            yield from asset_nodes(value)
    elif isinstance(node, list):
        for value in node:
            yield from asset_nodes(value)


def generate_code(
    ir_path: Path,
    output_dir: Path,
    contract_dir: Path,
    stst_bin: str,
    fsm: dict | None,
    program: str,
) -> list[Path]:
    """Render every C++/artifact file for an IR: telemetry headers, runtime and
    shared-state headers, the frame-log proto (compiled to C++), per-motion headers and
    main.cpp into OUTPUT_DIR, the frame-log contract into CONTRACT_DIR, and return the files
    written. PROGRAM names the executable, so the process list shows it.

    ir.json carries every codegen-facing field but the FSM's own tables, which come from
    coord-dsl's framed FSM `fsm`.

    Raises:
        RuntimeError: the IR declares no FSM. The generated program is an FSM dispatcher.
    """
    ir = json.loads(ir_path.read_text())
    if not ir["coordination"].get("fsm"):
        raise RuntimeError(
            f"{ir_path}: the model declares no FSM; FSM-less execution is not supported. "
            "Coordinate the model with an FSM to generate C++."
        )
    missing = sorted(
        {node["path"] for node in asset_nodes(ir) if not Path(node["path"]).is_file()}
    )
    if missing:
        raise MissingAssets(Path(ir_path), missing)

    ir["communication"]["telemetry_artifacts"] = write_telemetry_artifacts(
        ir, ir_path=ir_path, output_dir=contract_dir, fsm_ir=fsm
    )
    written = [contract_dir / frame_log_pb.HEADER_FILE]

    headers_dir = output_dir / "headers"
    headers_dir.mkdir(parents=True, exist_ok=True)

    # runtime.hpp carries only these
    uses = set()
    for function in ir["computation"]["functions"].values():
        kind = function["type"]
        uses.add(kind)
        if kind == "ErrorEvaluator":
            uses.add(f"ErrorEvaluator-{function['constraint']}")
        if kind == "VelocityProfile":
            uses.add(
                "PathVelocityProfile" if function.get("path_parameter") else "TargetVelocityProfile"
            )
        if function.get("error_normalization"):
            uses.add("JointNormalization")
        if function.get("controller_type"):
            uses.add(function["controller_type"])
    for solver in ir["resources"]["by_id"].values():
        if solver.get("algorithm_name"):
            uses.add(f"Solver{solver['algorithm_name']}")
        if any(out.get("normalization") for out in solver.get("output") or ()):
            uses.add("JointNormalization")
    composite = {
        item["id"]
        for item in ir["computation"]["data"]
        if item.get("type") in {"Pose", "VelocityTwist"}
    }
    for motion in ir["coordination"]["motions"]:
        for phase in ("when", "while", "until"):
            for monitor in motion[f"{phase}_monitors"]:
                if (monitor.get("error") or {}).get("id") in composite or any(
                    term.get("error_id") in composite for term in monitor["active_terms"]
                ):
                    uses.add("CompositeError")
                if monitor.get("is_edge_triggered"):
                    uses.add("SustainedEdge" if monitor.get("debounce_id") else "RisingEdge")
                    if not monitor.get("fsm_namespace"):
                        uses.add("ProduceEvent")
                elif monitor.get("flag"):
                    uses.add("Flag")
    ir["computation"]["uses"] = dict.fromkeys(uses, True)
    ir["configuration"]["name"] = program
    ir["configuration"]["config_check"] = {
        "home_key": AGENT_HOME_KEY,
        "pose_fields": list(CONFIG_POSE_FIELDS),
    }
    with tempfile.TemporaryDirectory(prefix="motion-spec-stst-") as scratch:
        payload_dir = Path(scratch)
        ir_payload_path = payload_dir / "ir.json"
        write_payload(ir_payload_path, ir)
        root = f"backend/{ir['configuration']['backend']}/main"

        frame_log_pb.run_protoc(f"--cpp_out={output_dir}")
        written += [output_dir / "frame_log.pb.h", output_dir / "frame_log.pb.cc"]
        renders = [
            ("telemetry_header", ir_payload_path, output_dir / "telemetry.hpp"),
            ("telemetry_model_header", ir_payload_path, output_dir / "telemetry_model.hpp"),
            ("frame_layout_header", ir_payload_path, output_dir / "frame_layout.h"),
            ("runtime_header", ir_payload_path, headers_dir / "runtime.hpp"),
            ("controller_runtime_header", ir_payload_path, headers_dir / "controllers.hpp"),
            ("algorithm_data_header", ir_payload_path, headers_dir / "algorithm_data.hpp"),
        ]
        if ir["resources"]["by_kind"].get("mobile_base"):
            renders.append(
                ("mobile_base_cycle_header", ir_payload_path, headers_dir / "mobile_base_cycle.hpp")
            )
        for motion in ir["coordination"]["motions"]:
            payload_path = payload_dir / f"{motion['id']}.json"
            write_payload(
                payload_path,
                {
                    "motion": motion,
                    "functions": ir["computation"]["functions"],
                    "views": ir["computation"]["views"],
                    "solvers": ir["resources"]["by_id"],
                },
            )
            renders.append(("motion_header", payload_path, headers_dir / f"{motion['id']}.hpp"))
        renders += [
            ("main_source", ir_payload_path, output_dir / "main.cpp"),
            ("cmake_project", ir_payload_path, output_dir / "CMakeLists.txt"),
            ("robot_config_header", ir_payload_path, output_dir / "robot_config.hpp"),
        ]
        if ir["configuration"]["backend"] == "robif2b":
            renders += [
                ("device_io_header", ir_payload_path, output_dir / "device_io.hpp"),
                ("config_check", ir_payload_path, output_dir / CONFIG_CHECK),
            ]
        written += [
            render_template(stst_bin, root, template_name, payload_path, output_path)
            for template_name, payload_path, output_path in renders
        ]
    return written
