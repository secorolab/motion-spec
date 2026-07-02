# SPDX-License-Identifier: MPL-2.0
"""C++ code generation for motion specification models via StringTemplate."""

from __future__ import annotations

import argparse
import copy
import json
import os
import re
import shutil
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
    return (
        len(path_parts) >= len(relative_parts)
        and path_parts[-len(relative_parts) :] == relative_parts
    )


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

    output_path.write_text(_collapse_blank_lines(result.stdout))


def _collapse_blank_lines(text: str) -> str:
    """Normalize StringTemplate output: at most one blank line between blocks, single trailing newline.

    ST4 emits stray blank lines for empty conditional/list items (the literal newlines survive even when
    the rendered value is empty), which is impractical to eliminate per-template — collapse them here."""
    text = re.sub(r"[ \t]+\n", "\n", text)  # drop trailing whitespace on each line
    text = re.sub(r"\n{3,}", "\n\n", text)  # collapse runs of blank lines to a single blank
    return text.strip("\n") + "\n"


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

    backend = ir.get("backend", "robif2b")
    if backend != "robif2b":
        return

    for solver in ir.get("arm_solvers", []):
        for out in solver.get("output", []):
            if out.get("type") != "Pose":
                continue
            entity = out.get("of") or {}
            if entity.get("is_scene_object"):
                obj_id = entity.get("id") or entity.get("body") or out.get("id")
                raise RuntimeError(
                    "robif2b backend cannot sync scene-object pose output "
                    f"'{out.get('id')}' for '{obj_id}'; world/scene object pose sync "
                    "is only implemented for mj_kdl."
                )


def _runtime_signature(solver: dict, backend: str) -> tuple:
    return (
        backend,
        solver.get("robot_model", ""),
        solver.get("urdf", ""),
        solver.get("chain_root", ""),
        solver.get("chain_tip") or solver.get("chain_end", ""),
        solver.get("tool_body", ""),
        solver.get("tcp_site", ""),
    )


def _annotate_runtime_robots(ir: dict, unique_motions: list, backend: str) -> None:
    runtime_by_signature: dict[tuple, str] = {}
    owner_by_runtime: dict[str, str] = {}
    solvers_by_id = {solver.get("id"): solver for solver in ir.get("arm_solvers", [])}

    for solver in ir.get("arm_solvers", []):
        solver_id = solver.get("id", "")
        signature = _runtime_signature(solver, backend)
        runtime_id = runtime_by_signature.setdefault(signature, solver_id)
        owner_by_runtime.setdefault(runtime_id, solver_id)
        solver["runtime_id"] = runtime_id
        solver["runtime_owner"] = solver_id == owner_by_runtime[runtime_id]
        # ST4's <if(x)> treats "" as truthy. Convert empty strings to None so
        # the template's <if(solver.tool_body)> branch is correctly skipped
        # for bare robots (no gripper / tool attached).
        if not solver.get("tool_body"):
            solver["tool_body"] = None
        if not solver.get("tcp_site"):
            solver["tcp_site"] = None

    for motion_list in (ir.get("motions", []), unique_motions):
        for motion in motion_list:
            for solver in motion.get("arm_solvers", []):
                canonical = solvers_by_id.get(solver.get("id"))
                if canonical is None:
                    continue
                solver["runtime_id"] = canonical.get("runtime_id", solver.get("id", ""))
                solver["runtime_owner"] = canonical.get("runtime_owner", True)


def _add_group_type_flags(groups: list) -> list:
    for g in groups:
        so_type = g.get("superobject_type", "Pose")
        g["is_pose"] = so_type == "Pose"
        g["is_twist"] = so_type in ("VelocityTwist", "AccelerationTwist")
        g["is_wrench"] = so_type == "Wrench"
    return groups


_FSM_NS_RE = re.compile(r"FSM\s*\(\s*ns\s*=\s*([^)\s]+)\s*\)")


def _fsm_namespace_uri(fsm_path: Path) -> str:
    """URI of the FSM's namespace; FSM event URIs (which monitor events must match) live under it."""
    text = fsm_path.read_text()
    ns_match = _FSM_NS_RE.search(text)
    if not ns_match:
        raise RuntimeError(
            f"Could not read the FSM namespace from '{fsm_path}'. Expected 'FSM (ns=<prefix>) ...'."
        )
    prefix = ns_match.group(1)
    uri_match = re.search(rf'ns\s+{re.escape(prefix)}\s*=\s*"([^"]+)"', text)
    if not uri_match:
        raise RuntimeError(
            f"FSM namespace prefix '{prefix}' has no 'ns {prefix} = \"...\"' declaration in '{fsm_path}'."
        )
    return uri_match.group(1)


def _load_fsm(fsm_path: Path) -> tuple[dict, str]:
    """Parse a .fsm via coord-dsl: return its structured IR and the rendered C++ header."""
    try:
        from coord_dsl.generators.registration import fsm_metamodel
        from coord_dsl.generators.fsm_graph import get_fsm_graph, gen_json, gen_cpp_header
    except ImportError as exc:
        raise RuntimeError(
            "coord-dsl is required for --fsm. Install it, e.g. pip install -e src/coord-dsl."
        ) from exc
    model = fsm_metamodel().model_from_file(str(fsm_path))
    graph, _, fsm_ref = get_fsm_graph(model)
    fsm_ir = gen_json(graph, fsm_ref)
    return fsm_ir, gen_cpp_header(fsm_ir)


def _event_to_state(fsm_ir: dict) -> dict[str, str]:
    """Map each FSM event enum token to the state it transitions out of (the state the motion runs in)."""
    transition_from = {t["id"]: t["from_state"] for t in fsm_ir["transitions_table"]}
    return {
        r["when_event"]: transition_from[r["do_transition"]]
        for r in fsm_ir["reactions_table"]
        if r["do_transition"] in transition_from
    }


def _load_fsm_ir(output_dir: Path, fsm_path: Path | None) -> tuple[dict | None, str | None]:
    """Return (fsm_ir, fsm_header_text).

    Resolution order:
    1. ``fsm_ir.json`` in *output_dir* — written by ``textx generate --target jsonld``
       when the .robmot imports a .fsm.  The C++ header is already on disk; header
       text is not returned in this case (None).
    2. ``--fsm <path>`` legacy flag — parses the .fsm directly (backward compat).
    3. Neither present → (None, None); no FSM wiring.
    """
    fsm_ir_path = output_dir / "fsm_ir.json"
    if fsm_ir_path.exists():
        return json.loads(fsm_ir_path.read_text()), None
    if fsm_path is not None:
        return _load_fsm(fsm_path)
    return None, None


def generate_code(ir_path: Path, output_dir: Path, stst_bin: str, fsm_path: Path | None = None):
    ir = load_ir(ir_path)
    _validate_ir(ir)
    backend = ir.get("backend", "robif2b")

    # FSM wiring (codegen/build concern; derived, not part of the semantic IR).
    fsm_ir, fsm_header_text = _load_fsm_ir(output_dir, fsm_path)
    fsm_namespace = fsm_ir["name"].lower() if fsm_ir else None
    fsm_header = f"{fsm_ir['name']}.hpp" if fsm_ir else None
    fsm_ns_uri = fsm_ir.get("namespace_uri") if fsm_ir else None
    if fsm_ns_uri is None and fsm_path is not None:
        fsm_ns_uri = _fsm_namespace_uri(fsm_path)
    event_state = _event_to_state(fsm_ir) if fsm_ir else {}
    # Heartbeat event produced every tick (drives the start-state kick / self-loops), if present.
    fsm_step_event = "E_STEP" if (fsm_ir and "E_STEP" in fsm_ir["events"]) else None
    ir["fsm_namespace"] = fsm_namespace
    ir["fsm_header"] = fsm_header
    ir["fsm_step_event"] = fsm_step_event

    def is_fsm_event(monitor: dict) -> bool:
        # A monitor fires the FSM only when its event lives in the FSM's namespace;
        # standalone (monitor-owned) events keep the existing warn stub.
        return bool(
            fsm_ns_uri
            and monitor.get("is_edge_triggered")
            and (monitor.get("event_uri") or "").startswith(fsm_ns_uri)
        )

    def expand_vector_fields(item: dict, field: str) -> None:
        values = item.get(field)
        if values is None:
            # Env placement shorthand: omitted position/orientation means zero/identity.
            values = [0.0, 0.0, 0.0]
        if not isinstance(values, list) or len(values) != 3:
            item_id = item.get("id", "<unknown>")
            raise ValueError(
                f"Scene item '{item_id}' has invalid '{field}'; expected three values."
            )
        item[f"{field}_x"] = values[0]
        item[f"{field}_y"] = values[1]
        item[f"{field}_z"] = values[2]

    def normalize_controller_gains(controller: dict) -> None:
        ctrl_type = controller.get("type")
        ctrl_id = controller.get("id", "<unknown>")
        if ctrl_type == "ProportionalIntegralDerivative":
            missing = [
                name
                for name, key in (
                    ("Kp", "proportional_gain"),
                    ("Ki", "integral_gain"),
                    ("Kd", "derivative_gain"),
                )
                if controller.get(key) is None
            ]
            if missing:
                raise ValueError(
                    f"PID controller '{ctrl_id}' is missing required gain(s) {missing}; "
                    f"all of Kp, Ki, Kd must be specified in the model."
                )
        elif ctrl_type == "ImpedanceController":
            if controller.get("stiffness") is None or controller.get("damping") is None:
                raise ValueError(
                    f"Impedance controller '{ctrl_id}' requires stiffness and damping."
                )

    def _require(obj_id: str, field: str, value):
        if value is None:
            raise ValueError(
                f"Procedural scene object '{obj_id}' is missing required field "
                f"'{field}'. Add it to the .robmot model — silent defaults are no "
                f"longer applied."
            )
        return value

    for handler in ir.get("cstr_hdl", []):
        for controller in handler.get("controllers", []):
            normalize_controller_gains(controller)
    for motion in ir.get("motions", []):
        for controller in motion.get("controllers", []):
            normalize_controller_gains(controller)

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
        obj["has_path"] = bool(obj.get("path"))
        if obj["has_path"]:
            # Geometry comes from the MJCF/URDF asset; do not fabricate flat
            # size/color/friction fields. The template skips this block when
            # has_path is true.
            continue
        obj_id = obj.get("id", "<unknown>")
        size = _require(obj_id, "size", obj.get("size"))
        color = _require(obj_id, "color", obj.get("color"))
        friction = _require(obj_id, "friction", obj.get("friction"))
        _require(obj_id, "shape", obj.get("shape"))
        _require(obj_id, "mass", obj.get("mass"))
        obj["size_x"], obj["size_y"], obj["size_z"] = (
            float(size[0]),
            float(size[1]),
            float(size[2]),
        )
        obj["color_r"], obj["color_g"], obj["color_b"], obj["color_a"] = (
            float(color[0]),
            float(color[1]),
            float(color[2]),
            float(color[3]),
        )
        obj["friction_slide"], obj["friction_torsion"], obj["friction_roll"] = (
            float(friction[0]),
            float(friction[1]),
            float(friction[2]),
        )

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
                f"KDL::diff(KDL::Rotation::Identity(), shared.{superobject['id']}.M)[{axis_index}]"
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
        if superobject["type"] == "ExternalForce":
            return f"shared.{superobject['id']}[{axis_index}]"
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
            if so_type == "Pose" and not (
                is_declared_pose or superobject.get("euler_axes_sequence")
            ):
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
                    "position_x_expr": None,
                    "position_y_expr": None,
                    "position_z_expr": None,
                    "orientation_x_expr": None,
                    "orientation_y_expr": None,
                    "orientation_z_expr": None,
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
        for pose_id, parts in components.items():
            missing = [name for name, value in parts.items() if value is None]
            if missing:
                raise ValueError(
                    f"Declared pose '{pose_id}' is missing required components: {', '.join(missing)}."
                )
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

    def enrich_arc_closures(ir_payload: dict) -> None:
        data_by_id = {
            item.get("id"): item
            for item in ir_payload.get("data", [])
            if isinstance(item, dict) and item.get("id")
        }
        closures = ir_payload.get("closures", {})
        pose_diff_by_target = {
            closure.get("in2"): closure
            for closure in closures.values()
            if isinstance(closure, dict) and closure.get("type") == "PoseDiffEvaluator"
        }

        def is_pose(data: dict) -> bool:
            qkind = data.get("quantity_kind")
            qkind_ids = qkind if isinstance(qkind, list) else [qkind]
            return data.get("type") == "Pose" or any(
                isinstance(item, dict) and item.get("id") == "Pose" for item in qkind_ids
            )

        for closure in closures.values():
            if closure.get("type") != "Arc":
                continue
            end = closure.get("end")
            end_data = data_by_id.get(end) or {}
            if not isinstance(end, str) or not is_pose(end_data):
                raise ValueError("Arc trajectory end must be a Pose quantity.")
            closure["end_position_expr"] = f"shared.{end}.p"
            closure["end_orientation_expr"] = f"shared.{end}.M"
            # current_pose_expr was used by the old closed-loop pose-projection block.
            # The arc template now uses time-based alpha and no longer reads this field.

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
            time_progress_ids: list[str] = []
            for step in motion.get("while_schedule", []):
                closure = closures.get(step)
                if not closure or closure.get("type") not in {
                    "Lerp",
                    "Circle",
                    "Arc",
                    "Helix",
                    "Figure8",
                }:
                    continue
                alpha_id = closure.get("alpha")
                alpha_data = data_by_id.get(alpha_id) or {}
                qkind = (alpha_data.get("quantity_kind") or {}).get("id")
                if qkind == "Progress" and alpha_id not in progress_ids:
                    progress_ids.append(alpha_id)
                if (
                    qkind == "Progress"
                    and not (closure.get("type") == "Arc")
                    and alpha_id not in time_progress_ids
                ):
                    time_progress_ids.append(alpha_id)
            motion["trajectory_progress_ids"] = progress_ids
            motion["time_trajectory_progress_ids"] = time_progress_ids

    def _evaluator_term(e: dict, start_field: str) -> str:
        # A timing evaluator has no solver error: compare the world clock
        # against the threshold, measured from the selected state timestamp.
        if e.get("is_elapsed"):
            op = e.get("elapsed_op") or ">="
            thr = e.get("elapsed_threshold_s") or 0.0
            return f"(shared.clock_time_s - state.{start_field} {op} {thr:.6f})"
        return f"motion_spec::runtime::constraint_satisfied(shared.{e['error']['id']})"

    def _evaluator_active_term(e: dict) -> str:
        return _evaluator_term(e, "motion_start_time")

    def _evaluator_when_term(e: dict) -> str:
        return _evaluator_term(e, "when_start_time")

    def add_until_monitor_conditions(motions: list[dict]) -> None:
        for motion in motions:
            terms = [
                _evaluator_active_term(e)
                for e in motion.get("until_evaluators", [])
                if e.get("error") or e.get("is_elapsed")
            ]
            elapsed_terms_by_error = {
                e["error"]["id"]: _evaluator_active_term(e)
                for e in motion.get("until_evaluators", [])
                if e.get("is_elapsed") and e.get("error")
            }
            joiner = " || " if motion.get("until_any") else " && "
            active_condition = joiner.join(terms) if terms else "false"
            if len(terms) > 1:
                active_condition = f"({active_condition})"
            for monitor in motion.get("until_monitors", []):
                if monitor.get("is_until_aggregate"):
                    monitor["active_condition"] = active_condition
                    continue
                error_id = (monitor.get("error") or {}).get("id")
                if error_id in elapsed_terms_by_error:
                    monitor["active_condition"] = elapsed_terms_by_error[error_id]

    def add_when_monitor_conditions(motions: list[dict]) -> None:
        for motion in motions:
            terms = [
                _evaluator_when_term(e)
                for e in motion.get("when_evaluators", [])
                if e.get("error") or e.get("is_elapsed")
            ]
            elapsed_terms_by_error = {
                e["error"]["id"]: _evaluator_when_term(e)
                for e in motion.get("when_evaluators", [])
                if e.get("is_elapsed") and e.get("error")
            }
            joiner = " || " if motion.get("when_any") else " && "
            motion["when_condition"] = joiner.join(terms) if terms else "true"
            if len(terms) > 1:
                motion["when_condition"] = f"({motion['when_condition']})"
            active_condition = joiner.join(terms) if terms else "false"
            if len(terms) > 1:
                active_condition = f"({active_condition})"
            for monitor in motion.get("when_monitors", []):
                if monitor.get("is_when_aggregate"):
                    monitor["active_condition"] = active_condition
                    continue
                error_id = (monitor.get("error") or {}).get("id")
                if error_id in elapsed_terms_by_error:
                    monitor["active_condition"] = elapsed_terms_by_error[error_id]

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
            has_when_elapsed = any(e.get("is_elapsed") for e in motion.get("when_evaluators", []))
            has_when_logic = bool(motion.get("when_schedule") or motion.get("when_evaluators"))
            can_start_params = []
            can_start_args = []
            if has_when_elapsed:
                can_start_params.append(state_type)
                can_start_args.append(f"{motion['id']}_state_instance")
            if has_when_logic:
                can_start_params.append("shared_data &shared")
                can_start_args.append("shared")
            motion["can_start_params"] = join_params(can_start_params)
            motion["can_start_args"] = join_args(can_start_args)

            # Each monitor fn (when / until / combined) takes exactly the params its
            # generated body uses, so no parameter is ever emitted unused. state is
            # touched by edge/level monitors and the elapsed-clock latch; shared by
            # any view closure, monitor, or pose materialisation; robot only when a
            # monitor fires an FSM event (produce_event needs robot.fsm_events).
            when_mons = motion.get("when_monitors") or []
            until_mons = motion.get("until_monitors") or []
            has_pose = bool(motion.get("declared_pose_components"))
            when_sched = bool(motion.get("when_schedule"))
            until_sched = bool(motion.get("until_schedule"))
            when_fsm = any(m.get("fsm_namespace") for m in when_mons)
            until_fsm = any(m.get("fsm_namespace") for m in until_mons)

            def monitor_sig(use_state, use_shared, use_robot):
                params, args = [], []
                if use_state:
                    params.append(state_type)
                    args.append(f"{motion['id']}_state_instance")
                if use_shared:
                    params.append("shared_data &shared")
                    args.append("shared")
                if use_robot:
                    params.append("const robot_io &robot")
                    args.append("robot")
                return join_params(params), join_args(args)

            # monitor_when_<id>: elapsed latch + pose materialise + when schedule/monitors
            motion["when_params"], motion["when_args"] = monitor_sig(
                has_when_elapsed or bool(when_mons),
                has_when_elapsed or has_pose or when_sched or bool(when_mons),
                when_fsm,
            )
            # monitor_until_<id>: until schedule/monitors
            motion["until_params"], motion["until_args"] = monitor_sig(
                bool(until_mons),
                until_sched or bool(until_mons),
                until_fsm,
            )
            # monitor_<id> (combined): when + until schedules/monitors (no pose/elapsed)
            motion["monitor_params"], motion["monitor_args"] = monitor_sig(
                bool(when_mons) or bool(until_mons),
                when_sched or bool(when_mons) or until_sched or bool(until_mons),
                when_fsm or until_fsm,
            )

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

    _annotate_runtime_robots(ir, unique_motions, backend)

    pose_components = build_pose_components(ir)
    ir["pose_components"] = pose_components
    ir["declared_pose_components"] = declared_pose_component_entries(ir, pose_components)
    enrich_lerp_closures(ir, pose_components)
    enrich_arc_closures(ir)
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
    primary_robot_id = next(
        (solver.get("id") for solver in ir.get("arm_solvers", []) if solver.get("id")), ""
    )
    for motion in ir.get("motions", []):
        motion["command_robot_id"] = primary_robot_id
    for motion in unique_motions:
        motion["command_robot_id"] = primary_robot_id
    for motion in ir.get("motions", []) + unique_motions:
        motion["has_when_elapsed"] = any(
            e.get("is_elapsed") for e in motion.get("when_evaluators", [])
        )
        motion["has_active_elapsed"] = any(
            e.get("is_elapsed")
            for e in motion.get("while_evaluators", []) + motion.get("until_evaluators", [])
        )
        motion["has_elapsed"] = motion["has_when_elapsed"] or motion["has_active_elapsed"]

    add_until_monitor_conditions(ir.get("motions", []))
    add_until_monitor_conditions(unique_motions)
    add_when_monitor_conditions(ir.get("motions", []))
    add_when_monitor_conditions(unique_motions)
    add_motion_done_conditions(ir.get("motions", []))
    add_motion_done_conditions(unique_motions)

    # Elapsed constraints compare seconds from the runtime clock. MuJoCo supplies sim
    # seconds; real backends use a monotonic wall clock.
    ir["needs_clock_time"] = any(m.get("has_elapsed") for m in ir.get("motions", []))
    if fsm_namespace is not None:
        # Tag FSM-event monitors so update-monitor emits produce_event(...) (and the monitor fn takes
        # robot); also map each motion to the state it *runs* in (fsm_state): the from-state of the
        # transition its UNTIL/WHILE event fires.
        #
        # A WHEN precondition is never an entry guard with no controller (that would leave the arm
        # uncommanded). Instead it must name an explicit fallback hold motion: the FSM enters that
        # fallback motion's state, holds pose there, and re-evaluates the precondition each tick; when
        # it becomes satisfied the WHEN monitor's event advances to the gated motion. So the fallback
        # motion runs in the state the WHEN (success) event exits, and the gated motion's WHEN is
        # evaluated inside that fallback state.
        def tag_run_state(motion, monitors):
            for monitor in monitors:
                if is_fsm_event(monitor):
                    monitor["fsm_namespace"] = fsm_namespace
                    state = event_state.get(monitor.get("event_name") or "")
                    if state and not motion.get("fsm_state"):
                        motion["fsm_state"] = state

        for motion_list in (ir.get("motions", []), unique_motions):
            by_id = {m["id"]: m for m in motion_list}
            for motion in motion_list:
                tag_run_state(
                    motion, motion.get("until_monitors", []) + motion.get("while_monitors", [])
                )
                for monitor in motion.get("when_monitors", []):
                    if not is_fsm_event(monitor):
                        continue
                    monitor["fsm_namespace"] = fsm_namespace
                    fallback_id = monitor.get("fallback_motion")
                    if not fallback_id:
                        raise ValueError(
                            f"WHEN monitor '{monitor.get('id')}' on FSM-wired motion "
                            f"'{motion['id']}' must declare a fallback hold motion "
                            f"(e.g. '... when active fallback <hold-motion>'). A WHEN precondition "
                            f"without a fallback would leave the arm uncommanded while waiting."
                        )
                    fallback = by_id.get(fallback_id)
                    if fallback is None:
                        raise ValueError(
                            f"WHEN monitor '{monitor.get('id')}' names unknown fallback motion "
                            f"'{fallback_id}'."
                        )
                    state = event_state.get(monitor.get("event_name") or "")
                    if state and not fallback.get("fsm_state"):
                        fallback["fsm_state"] = state
                    gates = fallback.setdefault("fsm_when_gate_motions", [])
                    if motion["id"] not in gates:
                        gates.append(motion["id"])

    add_motion_function_interfaces(ir.get("motions", []))
    add_motion_function_interfaces(unique_motions)

    if fsm_namespace is not None:
        # Now that monitor_args are computed, materialize the WHEN-evaluation calls that each fallback
        # state must run (the gated motion's precondition, dispatched alongside the hold step).
        for motion_list in (ir.get("motions", []), unique_motions):
            by_id = {m["id"]: m for m in motion_list}
            for fallback in motion_list:
                gate_ids = fallback.get("fsm_when_gate_motions")
                if not gate_ids:
                    continue
                fallback["fsm_when_gate_calls"] = [
                    f"monitor_when_{gate_id}({by_id[gate_id].get('when_args', '')});"
                    for gate_id in gate_ids
                    if gate_id in by_id
                ]

    headers_dir = output_dir / "headers"
    headers_dir.mkdir(parents=True, exist_ok=True)

    if fsm_header_text is not None:
        (headers_dir / fsm_header).write_text(fsm_header_text)
    elif fsm_ir is not None:
        shutil.copy2(output_dir / fsm_header, headers_dir / fsm_header)

    payload_dir = output_dir / ".stst"
    payload_dir.mkdir(parents=True, exist_ok=True)
    ir_payload_path = payload_dir / "ir.json"
    write_json(ir_payload_path, ir)

    render_template(stst_bin, "runtime_header", ir_payload_path, headers_dir / "runtime.hpp")
    render_template(
        stst_bin, "shared_state_header", ir_payload_path, headers_dir / "shared_state.hpp"
    )
    render_template(stst_bin, "uris_header", ir_payload_path, headers_dir / "uris.hpp")
    if ir.get("has_mobile_base"):
        render_template(
            stst_bin,
            "mobile_base_cycle_header",
            ir_payload_path,
            headers_dir / "mobile_base_cycle.hpp",
        )

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
            "pose_axis_error_groups": _add_group_type_flags(
                motion.get("pose_axis_error_groups", [])
            ),
        }
        payload_path = payload_dir / f"{motion['id']}.json"
        write_json(payload_path, payload)
        render_template(
            stst_bin, "motion_header", payload_path, headers_dir / f"{motion['id']}.hpp"
        )

    render_template(stst_bin, "ref_main", ir_payload_path, output_dir / "ref_main.cpp")
    if ir["backend"] == "mj_kdl":
        render_template(stst_bin, "cmake_mj_kdl", ir_payload_path, output_dir / "CMakeLists.txt")


def main():
    parser = argparse.ArgumentParser(
        description="Generate C++ header files from motion-spec IR.",
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
    parser.add_argument(
        "--fsm",
        default=None,
        help="(deprecated) Path to a coord-dsl .fsm; auto-detected from fsm_ir.json in the output directory",
    )
    args = parser.parse_args()

    try:
        generate_code(
            ir_path=Path(args.input).resolve(),
            output_dir=Path(args.output_dir).resolve(),
            stst_bin=args.stst_bin,
            fsm_path=Path(args.fsm).resolve() if args.fsm else None,
        )
    except RuntimeError as exc:
        print(f"Code generation failed: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
