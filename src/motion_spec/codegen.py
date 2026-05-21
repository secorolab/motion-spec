# SPDX-License-Identifier: MPL-2.0
"""C++ code generation for motion specification models via StringTemplate."""

from __future__ import annotations

import argparse
import copy
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


def _add_group_type_flags(groups: list) -> list:
    for g in groups:
        so_type = g.get("superobject_type", "Pose")
        g["is_pose"] = so_type == "Pose"
        g["is_twist"] = so_type in ("VelocityTwist", "AccelerationTwist")
        g["is_wrench"] = so_type == "Wrench"
    return groups


def generate_code(ir_path: Path, output_dir: Path, stst_bin: str):
    ir = load_ir(ir_path)
    _validate_ir(ir)
    backend = ir.get("backend", "robif2b")

    def expand_vector_fields(item: dict, field: str) -> None:
        values = item.get(field) or [0.0, 0.0, 0.0]
        item[f"{field}_x"] = values[0] if len(values) > 0 else 0.0
        item[f"{field}_y"] = values[1] if len(values) > 1 else 0.0
        item[f"{field}_z"] = values[2] if len(values) > 2 else 0.0

    scene = ir.get("scene") or {}
    for robot in scene.get("robots", []):
        expand_vector_fields(robot, "pos")
        expand_vector_fields(robot, "euler")
        for attachment in robot.get("attachments", []):
            expand_vector_fields(attachment, "pos")
            expand_vector_fields(attachment, "euler")
    for obj in scene.get("objects", []):
        expand_vector_fields(obj, "pos")
        expand_vector_fields(obj, "euler")
        expand_vector_fields(obj, "size")
        friction = obj.get("friction") or [0.5, 0.005, 0.0001]
        obj["friction_slide"] = friction[0] if len(friction) > 0 else 0.5
        obj["friction_torsion"] = friction[1] if len(friction) > 1 else 0.005
        obj["friction_roll"] = friction[2] if len(friction) > 2 else 0.0001
        obj["has_path"] = bool(obj.get("path"))

    def merge_list_by_id(dst: list, src: list) -> None:
        seen = {item.get("id") for item in dst if isinstance(item, dict)}
        for item in src:
            item_id = item.get("id") if isinstance(item, dict) else None
            if item_id in seen:
                continue
            dst.append(copy.deepcopy(item))
            seen.add(item_id)

    def cpp_access_expr(data_id: str, views: dict) -> str:
        view = views.get(data_id)
        if not view:
            return f"shared.{data_id}"
        superobject = view["superobject"]
        axis_index = {"X": 0, "Y": 1, "Z": 2, "x": 0, "y": 1, "z": 2}[view["axis"]]
        if superobject["type"] == "Pose" and view["subspace"] == "Position":
            return f"shared.{superobject['id']}.p[{axis_index}]"
        if superobject["type"] == "Pose" and view["subspace"] == "Rotation":
            return (
                "KDL::diff(KDL::Rotation::Identity(), "
                f"shared.{superobject['id']}.M)[{axis_index}]"
            )
        if superobject["type"] == "VelocityTwist":
            member = "rot" if view["subspace"] == "AngularVelocity" else "vel"
            return f"shared.{superobject['id']}.{member}[{axis_index}]"
        if superobject["type"] == "AccelerationTwist":
            member = "rot" if view["subspace"] == "AngularAcceleration" else "vel"
            return f"shared.{superobject['id']}.{member}[{axis_index}]"
        if superobject["type"] == "Wrench":
            member = "torque" if view["subspace"] == "Torque" else "force"
            return f"shared.{superobject['id']}.{member}[{axis_index}]"
        return f"shared.{data_id}"

    def component_expr(component_id: str, data_by_id: dict, views: dict) -> str:
        component = data_by_id.get(component_id) or {}
        if component.get("reference_value"):
            return cpp_access_expr(component["reference_value"], views)
        if component.get("value") is not None:
            return str(component["value"])
        return cpp_access_expr(component_id, views)

    def build_pose_components(ir_payload: dict) -> dict:
        views = ir_payload.get("views", {})
        data_by_id = {
            item.get("id"): item
            for item in ir_payload.get("data", [])
            if isinstance(item, dict) and item.get("id")
        }
        components: dict[str, dict] = {}
        for view in views.values():
            superobject = view.get("superobject") or {}
            so_type = superobject.get("type")
            roles = set(superobject.get("roles") or [])
            is_declared_pose = bool({"Declared", "Snapshot"} & roles)
            is_legacy_pose_quantity = so_type == "PoseQuantity"
            if so_type not in ("Pose", "PoseQuantity"):
                continue
            if so_type == "Pose" and not (is_declared_pose or superobject.get("euler_axes_sequence")):
                continue
            if is_legacy_pose_quantity:
                is_declared_pose = True
            # Only include inline-defined poses (those where components have values/references)
            # FK poses have all components computed from the solver with no stored value/reference
            subobject_id = (view.get("subobject") or {}).get("id")
            subobject_data = data_by_id.get(subobject_id) or {}
            if not subobject_data.get("reference_value") and subobject_data.get("value") is None:
                continue
            pose_id = superobject["id"]
            entry = components.setdefault(
                pose_id,
                {
                    "position_x_expr": "0.0",
                    "position_y_expr": "0.0",
                    "position_z_expr": "0.0",
                    "orientation_x_expr": "0.0",
                    "orientation_y_expr": "0.0",
                    "orientation_z_expr": "0.0",
                },
            )
            axis = str(view.get("axis", "")).lower()
            if axis not in {"x", "y", "z"}:
                continue
            subobject = (view.get("subobject") or {}).get("id")
            if not subobject:
                continue
            prefix = "position" if view.get("subspace") == "Position" else "orientation"
            entry[f"{prefix}_{axis}_expr"] = component_expr(subobject, data_by_id, views)
        return components

    def enrich_lerp_closures(ir_payload: dict, pose_components: dict) -> None:
        for closure in ir_payload.get("closures", {}).values():
            if closure.get("type") != "Lerp":
                continue
            goal = closure.get("goal")
            if not isinstance(goal, str):
                continue
            if goal in pose_components:
                parts = pose_components[goal]
                closure["goal_expr"] = (
                    "KDL::Frame("
                    "KDL::Rotation::RPY("
                    f"{parts['orientation_x_expr']}, "
                    f"{parts['orientation_y_expr']}, "
                    f"{parts['orientation_z_expr']}), "
                    "KDL::Vector("
                    f"{parts['position_x_expr']}, "
                    f"{parts['position_y_expr']}, "
                    f"{parts['position_z_expr']}))"
                )
                closure["assign_goal"] = True
            else:
                closure["goal_expr"] = f"shared.{goal}"
                closure["assign_goal"] = False

    def declared_pose_component_entries(
        ir_payload: dict,
        pose_components: dict,
        referenced_ids: set[str] | None = None,
    ) -> list[dict]:
        data_by_id = {
            item.get("id"): item
            for item in ir_payload.get("data", [])
            if isinstance(item, dict) and item.get("id")
        }
        entries = []
        for pose_id, parts in pose_components.items():
            if referenced_ids is not None and pose_id not in referenced_ids:
                continue
            item = data_by_id.get(pose_id) or {}
            roles = set(item.get("roles") or [])
            if "Declared" not in roles or "Snapshot" in roles:
                continue
            entries.append({"id": pose_id, **parts})
        return entries

    def collect_motion_references(motion: dict, closures: dict) -> set[str]:
        refs: set[str] = set()

        def visit(value):
            if isinstance(value, str):
                refs.add(value)
            elif isinstance(value, dict):
                for item in value.values():
                    visit(item)
            elif isinstance(value, list):
                for item in value:
                    visit(item)

        visit(motion)
        for schedule_name in ("when_schedule", "while_schedule", "until_schedule"):
            for step in motion.get(schedule_name, []):
                closure = closures.get(step)
                if closure:
                    visit(closure)
        return refs

    def add_motion_trajectory_progress(ir_payload: dict) -> None:
        data_by_id = {
            item.get("id"): item
            for item in ir_payload.get("data", [])
            if isinstance(item, dict) and item.get("id")
        }
        closures = ir_payload.get("closures", {})
        for motion in ir_payload.get("motions", []):
            progress_ids: list[str] = []
            for step in motion.get("while_schedule", []):
                closure = closures.get(step)
                if not closure or closure.get("type") != "Lerp":
                    continue
                alpha_id = closure.get("alpha")
                alpha_data = data_by_id.get(alpha_id) or {}
                qkind = (alpha_data.get("quantity_kind") or {}).get("id")
                if qkind == "Progress" and alpha_id not in progress_ids:
                    progress_ids.append(alpha_id)
            motion["trajectory_progress_ids"] = progress_ids

    def add_until_monitor_conditions(motions: list[dict]) -> None:
        for motion in motions:
            terms = [
                f"motion_spec::runtime::constraint_satisfied(shared.{e['error']['id']})"
                for e in motion.get("until_evaluators", [])
                if e.get("error")
            ]
            joiner = " || " if motion.get("until_any") else " && "
            active_condition = joiner.join(terms) if terms else "false"
            if len(terms) > 1:
                active_condition = f"({active_condition})"
            for monitor in motion.get("until_monitors", []):
                if monitor.get("is_until_aggregate"):
                    monitor["active_condition"] = active_condition

    def add_motion_done_conditions(motions: list[dict]) -> None:
        for motion in motions:
            terms = [
                f"shared.{alpha_id} >= 1.0"
                for alpha_id in motion.get("trajectory_progress_ids", [])
            ]
            until_monitors = motion.get("until_monitors", [])
            if until_monitors:
                event_joiner = " || " if motion.get("until_any") else " && "
                event_terms = [
                    (
                        f"{motion['id']}_state_instance.{monitor['id']}_event_triggered"
                        if monitor.get("is_edge_triggered")
                        else f"{motion['id']}_state_instance.{monitor['flag']}"
                    )
                    for monitor in until_monitors
                ]
                if event_terms:
                    event_condition = event_joiner.join(event_terms)
                    if len(event_terms) > 1:
                        event_condition = f"({event_condition})"
                    terms.append(event_condition)
            motion["done_condition"] = " && ".join(terms) if terms else "true"

    def add_motion_function_interfaces(motions: list[dict]) -> None:
        def join_params(params: list[str]) -> str:
            if not params:
                return ""
            return "\n    " + ",\n    ".join(params) + "\n"

        def join_args(args: list[str]) -> str:
            return ", ".join(args)

        for motion in motions:
            state_type = f"{motion['id']}_state &state"
            has_when_logic = bool(motion.get("when_schedule") or motion.get("when_evaluators"))
            motion["can_start_params"] = "shared_data &shared" if has_when_logic else ""
            motion["can_start_args"] = "shared" if has_when_logic else ""

            has_monitor_logic = bool(
                motion.get("when_schedule")
                or motion.get("until_schedule")
                or motion.get("when_monitors")
                or motion.get("until_monitors")
            )
            motion["monitor_params"] = join_params([state_type, "shared_data &shared"]) if has_monitor_logic else ""
            motion["monitor_args"] = join_args([f"{motion['id']}_state_instance", "shared"]) if has_monitor_logic else ""

            has_apply_state = bool(motion.get("arm_solvers"))
            has_command_forwarding = bool(motion.get("command_forwarding"))
            has_apply_shared = has_command_forwarding
            has_apply_robot = bool(motion.get("arm_solvers") or has_command_forwarding)
            apply_params = []
            apply_args = []
            if has_apply_state:
                apply_params.append(state_type)
                apply_args.append(f"{motion['id']}_state_instance")
            if has_apply_shared:
                apply_params.append("shared_data &shared")
                apply_args.append("shared")
            if has_apply_robot:
                apply_params.append("const robot_io &robot")
                apply_args.append("robot")
            motion["apply_params"] = join_params(apply_params)
            motion["apply_args"] = join_args(apply_args)

    unique_motions = []
    motion_by_id = {}
    for motion in ir.get("motions", []):
        motion_id = motion.get("id")
        if motion_id in motion_by_id:
            existing = motion_by_id[motion_id]
            merge_list_by_id(existing.setdefault("controllers", []), motion.get("controllers", []))
            merge_list_by_id(existing.setdefault("monitors", []), motion.get("monitors", []))
            merge_list_by_id(existing.setdefault("arm_solvers", []), motion.get("arm_solvers", []))
            continue
        merged_motion = copy.deepcopy(motion)
        motion_by_id[motion_id] = merged_motion
        unique_motions.append(merged_motion)
    ir["unique_motions"] = unique_motions

    if backend == "mj_kdl":
        for solver in ir.get("arm_solvers", []):
            if solver.get("root_acc"):
                solver["gravity"] = [-v for v in solver["root_acc"]]
        for motion_list in (ir.get("motions", []), unique_motions):
            for motion in motion_list:
                for solver in motion.get("arm_solvers", []):
                    if solver.get("root_acc"):
                        solver["gravity"] = [-v for v in solver["root_acc"]]

    pose_components = build_pose_components(ir)
    ir["pose_components"] = pose_components
    ir["declared_pose_components"] = declared_pose_component_entries(ir, pose_components)
    enrich_lerp_closures(ir, pose_components)
    for motion in unique_motions:
        motion_refs = collect_motion_references(motion, ir.get("closures", {}))
        motion["declared_pose_components"] = declared_pose_component_entries(
            ir, pose_components, motion_refs
        )
    add_motion_trajectory_progress(ir)
    add_motion_trajectory_progress(
        {
            "motions": unique_motions,
            "data": ir.get("data", []),
            "closures": ir.get("closures", {}),
        }
    )
    primary_robot_id = next((solver.get("id") for solver in ir.get("arm_solvers", []) if solver.get("id")), "")
    for motion in ir.get("motions", []):
        motion["command_robot_id"] = primary_robot_id
    for motion in unique_motions:
        motion["command_robot_id"] = primary_robot_id

    add_until_monitor_conditions(ir.get("motions", []))
    add_until_monitor_conditions(unique_motions)
    add_motion_done_conditions(ir.get("motions", []))
    add_motion_done_conditions(unique_motions)
    add_motion_function_interfaces(ir.get("motions", []))
    add_motion_function_interfaces(unique_motions)

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

    for motion in unique_motions:
        payload = {
            "motion": motion,
            "closures": ir["closures"],
            "views": ir["views"],
            "wrench_outputs": ir["wrench_outputs"],
            "base_velocity_solvers": ir["base_velocity_solvers"],
            "base_force_solvers": ir["base_force_solvers"],
            "has_mobile_base": ir["has_mobile_base"],
            "backend": ir["backend"],
            "declared_pose_components": motion.get("declared_pose_components", []),
            "pose_axis_error_groups": _add_group_type_flags(motion.get("pose_axis_error_groups", [])),
        }
        payload_path = payload_dir / f"{motion['id']}.json"
        write_json(payload_path, payload)
        render_template(stst_bin, "motion_header", payload_path, headers_dir / f"{motion['id']}.hpp")

    render_template(stst_bin, "ref_main", ir_payload_path, output_dir / "ref_main.cpp")
    if ir["backend"] == "mj_kdl":
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
    args = parser.parse_args()

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
