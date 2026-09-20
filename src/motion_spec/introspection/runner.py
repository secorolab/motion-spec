# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Launch generated introspection executables under REC cataloging."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone
from pathlib import Path

from rdflib import Graph
from rec import State, Verdict

from motion_spec.introspection.archive import create_archive_manifest, verify_manifest
from motion_spec.introspection.frame_log_pb import ctrl_shm_name, shm_name_for
from motion_spec.introspection.lifecycle_events import publish_lifecycle
from motion_spec.introspection.provenance import (
    CONTROLLER_PROCESS,
    EXECUTION_DOCUMENT,
    GENERATION_DOCUMENT,
    GRAPH_EXECUTION,
    RUN_IRI_BASE,
    ensure_local_rec_importable,
    host_info,
    parse_rec_time,
    rec_document,
    rec_run_lifecycle_from_file,
    record_draw,
    record_execution,
    record_software,
    record_used_file,
    uri,
    write_generation_graph,
)
from motion_spec.introspection.ros_video import RosImageRecorder, real_camera_recordings


class RunnerError(RuntimeError):
    """A cataloged generated executable run could not be completed."""


def run_cataloged(
    run_dir: Path | str,
    *,
    source_dir: Path | str,
    executable: Path | str,
    executable_args: list[str] | None = None,
    run_id: str | None = None,
    cwd: Path | str | None = None,
    record: list[str] | None = None,
    record_log: bool = True,
    env: dict[str, str] | None = None,
    environment_provenance: dict | None = None,
) -> int:
    """Run a generated executable with REC lifecycle and archive provenance.

    ENV is what the executable runs under; ENVIRONMENT_PROVENANCE is what the run records of it.
    """
    run_dir = Path(run_dir)
    source_dir = Path(source_dir)
    executable = Path(executable).resolve()
    executable_args = [str(arg) for arg in (executable_args or [])]
    run_id = run_id or run_dir.name
    frame_log = run_dir / "logs" / "frame_log.pb"
    rec_path = rec_document(run_dir, run_id)

    _validate_new_run(run_dir, source_dir, executable, run_id, Path(cwd).resolve() if cwd else None)
    # frame_layout.json, not the log: the run is recorded before the log exists.
    schema_path = (
        source_dir / "contract" / "frame_layout.json"
        if (source_dir / "contract").is_dir()
        else source_dir / "frame_layout.json"
    )
    schema = json.loads(schema_path.read_text())
    run_dir.mkdir(parents=True, exist_ok=True)
    _start_rec_run(
        run_dir,
        run_id,
        source_dir,
        executable,
        schema,
        executable_args,
        cwd=Path(cwd).resolve() if cwd else None,
        environment=environment_provenance,
    )

    try:
        with (
            _rosbag_recorder(run_dir, *_rosbag_settings(source_dir)),
            _real_camera_recorders(schema, record, frame_log.parent),
        ):
            returncode = _run_executable(
                executable,
                executable_args,
                cwd=Path(cwd).resolve() if cwd else None,
                frame_log=frame_log,
                run_id=run_id,
                rec_path=rec_path,
                record=record,
                record_log=record_log,
                base_env=env,
                schema_hash=schema.get("schema_hash"),
            )
    except Exception:
        _finish_rec_run(rec_path, run_id, Verdict.FAILED)
        raise
    if (frame_log.parent / "sampling.json").exists():
        _record_sampling(rec_path, run_id, frame_log.parent / "sampling.json")
    if rec_run_lifecycle_from_file(rec_path)["verdict"] is not Verdict.ERROR:
        # 130 is the program leaving its loop on SIGINT/SIGTERM, which it reports rather than
        # dying from: the run stopped early but its artifacts are complete, so it is not a
        # failure. Reached when the signal went to the child alone and never raised here.
        if returncode == 130:
            _finish_rec_run(rec_path, run_id, Verdict.ERROR)
        elif returncode != 0:
            _finish_rec_run(rec_path, run_id, Verdict.FAILED)

    # A run that died before its first frame has nothing to catalogue. The rec run is already
    # FAILED, and what the caller needs to see is the executable's own error -- not a missing
    # frame log raised from the manifest on top of it.
    # A log-less run has no such signal -- catalogue it anyway, or an interrupted --no-log run
    # would leave nothing saying it ran.
    if returncode != 0 and record_log and not frame_log.exists():
        return returncode

    try:
        create_archive_manifest(
            run_dir,
            source_dir=source_dir,
            run_id=run_id,
            frame_log=frame_log,
            log_producer_executable=executable,
            complete_rec=False,
            recorded=record_log,
        )
        if returncode == 0:
            _finish_rec_run(rec_path, run_id, Verdict.PASSED)
            # A recording nobody checked is not worth the disk it sits on.
            verify_manifest(run_dir)
    except Exception:
        if returncode == 0:
            _finish_rec_run(rec_path, run_id, Verdict.FAILED)
        raise
    return returncode


_ROBOT_CONFIG_KEYS = (
    "ip",
    "user",
    "password",
    "port",
    "port_real_time",
    "session_timeout_ms",
    "connection_timeout_ms",
)

# What each device kind's reader in robot_config.hpp demands of its section. Optional keys
# (timeout_ms, bias_samples, poll_interval_ms) have documented defaults there and are not
# required here.
_DEVICE_CONFIG_KEYS = {
    "KinovaGen3": _ROBOT_CONFIG_KEYS,
    "KinovaGen3-2F85": _ROBOT_CONFIG_KEYS,
    "Robotiq2F85": ("port", "baudrate", "slave_address"),
    "RobotiqFT300s": ("port", "baudrate", "slave_address"),
}


def _config_sections(table: dict, prefix: str = "") -> list[str]:
    """Every dotted path in the file that carries values -- what a device's config_key names.

    A table holding only tables is the namespace an authored FQN passes through
    (`agents` in `[agents.arm1]`), not a section anything is configured in.
    """
    found = []
    for name, value in table.items():
        if not isinstance(value, dict):
            continue
        path = f"{prefix}{name}"
        if any(not isinstance(entry, dict) for entry in value.values()):
            found.append(path)
        found += _config_sections(value, f"{path}.")
    return found


def _config_section(table: dict, key: str) -> dict | None:
    """The table a dotted config key names, or None when the file states no such path."""
    section = table
    for part in key.split("."):
        section = section.get(part) if isinstance(section, dict) else None
    return section if isinstance(section, dict) else None


def _validate_robot_config(source_dir: Path, cwd: Path | None = None) -> None:
    """Check the deployment config before launching, so a typo fails here, not against hardware."""
    import tomllib

    from motion_spec.rdf_parser.resources import AGENT_HOME_KEY, CONFIG_POSE_FIELDS

    ir_path = source_dir / "model" / "ir.json"
    if not ir_path.exists():
        return
    ir = json.loads(ir_path.read_text())
    if (ir["configuration"].get("platform") or {}).get("simulated", True):
        return
    declared = (ir["configuration"]["platform"] or {}).get("config") or ""
    if not declared:
        raise RunnerError(
            "real-world run declares no config; it has nowhere to read addresses from"
        )
    # Resolved exactly as the executable will resolve it: against the working directory the run
    # gets. Checking any other file would clear a config the run never opens.
    config_path = Path(declared)
    if not config_path.is_absolute():
        config_path = (Path(cwd) if cwd else Path.cwd()) / declared
    if not config_path.exists():
        raise RunnerError(f"{config_path}: robot config not found")
    try:
        config = tomllib.loads(config_path.read_text())
    except tomllib.TOMLDecodeError as error:
        raise RunnerError(f"{config_path}: {error}") from error
    # A chain that shares another's runtime repeats its owner's devices; the pair is the fact.
    bound = {
        (device["config_key"], device["kind"])
        for solver in ir.get("resources", {}).get("robots") or ()
        if solver.get("kind") == "serial_chain"
        for device in solver.get("devices") or ()
        if device.get("config_key")
    }
    for key, kind in sorted(bound):
        section = _config_section(config, key)
        if section is None:
            raise RunnerError(f"{config_path}: no [{key}] section for the bound {kind}")
        missing = [field for field in _DEVICE_CONFIG_KEYS.get(kind, ()) if field not in section]
        if missing:
            raise RunnerError(f"{config_path}: [{key}] is missing {', '.join(missing)}")
    # Only a simulated run resets an agent to a home; on hardware the arm is wherever it was left,
    # so a home here is a number the deployment believes in and nothing acts on.
    homed = sorted(
        key for key, _ in bound if AGENT_HOME_KEY in (_config_section(config, key) or {})
    )
    if homed:
        raise RunnerError(
            f"{config_path}: `{AGENT_HOME_KEY}` in [{'], ['.join(homed)}] is read only by a "
            "simulated run.\n"
            "  This run drives the real devices, which start wherever they were left, so nothing "
            "resets to it.\n"
            f"  Comment the `{AGENT_HOME_KEY}` line out -- keeping the numbers for the simulated "
            "platform -- or run that platform instead."
        )
    # A pose the model reads with `[config.<key>]` binds its section as much as a device does:
    # generation refuses a model whose pose section is absent, so the run must not refuse it for
    # being present. Its numbers are re-read here because they may change without regeneration.
    poses = {
        entry["config_key"]
        for entry in (ir["configuration"].get("config_poses") or ())
        if entry.get("config_key")
    }
    for key in sorted(poses):
        section = _config_section(config, key) or {}
        for field in CONFIG_POSE_FIELDS:
            values = section.get(field)
            if not isinstance(values, list) or len(values) != 3:
                raise RunnerError(f"{config_path}: [{key}] states no three-number `{field}`")

    def offers_a_pose(key: str) -> bool:
        """Whether a section states a pose rather than a device.

        A deployment keeps as many poses as it likes and a model reads the ones it names, so an
        unread one is a pose on offer, not a mistake. The shape of the ones actually read is
        checked above; nothing reads this one, so nothing here has an opinion on it.
        """
        section = _config_section(config, key) or {}
        return any(field in section for field in CONFIG_POSE_FIELDS)

    # A section for nothing bound is a mis-key or a stale device: it would connect to hardware
    # this run never commands. Under KinovaGen3-2F85 a separate gripper section lands here.
    # [ros.*] configures the generated publishers and [rosbag] the run's recording, not a
    # device this run binds.
    sections = {
        key for key in _config_sections(config) if key.split(".")[0] not in ("ros", "rosbag")
    }
    unbound = sorted(
        key for key in sections - {key for key, _ in bound} - poses if not offers_a_pose(key)
    )
    if unbound:
        binds = ", ".join(f"[{key}]" for key in sorted({key for key, _ in bound} | poses))
        raise RunnerError(
            f"{config_path}: [{'], ['.join(unbound)}] configures nothing this run binds.\n"
            f"  This run binds {binds or 'no sections'}, and a device section is named by the "
            "agent its solver realizes -- so this one reaches hardware the run never commands.\n"
            f"  Comment it out, or correct its name. This run was generated at {ir_path}."
        )


def _validate_new_run(
    run_dir: Path, source_dir: Path, executable: Path, run_id: str, cwd: Path | None = None
) -> None:
    if not source_dir.exists():
        raise RunnerError(f"{source_dir}: source directory does not exist")
    required = (
        (source_dir / "contract" / "frame_log.proto", source_dir / GENERATION_DOCUMENT)
        if (source_dir / "contract").is_dir()
        else tuple(source_dir / rel for rel in ("frame_log.proto", GENERATION_DOCUMENT))
    )
    for path in required:
        if not path.exists():
            raise RunnerError(f"{path}: required generated artifact is missing")
    if not executable.exists():
        raise RunnerError(f"{executable}: executable does not exist")
    _validate_robot_config(source_dir, cwd)
    if run_dir.exists() and rec_document(run_dir, run_id).exists():
        raise RunnerError(f"{run_dir}: already records a run; choose a fresh run directory")
    frame_log = run_dir / "logs" / "frame_log.pb"
    if frame_log.exists():
        raise RunnerError(f"{frame_log}: refusing to overwrite an existing frame log")


def _start_rec_run(
    run_dir: Path,
    run_id: str,
    source_dir: Path,
    executable: Path,
    schema: dict,
    executable_args: list[str],
    cwd: Path | None = None,
    environment: dict | None = None,
) -> None:
    ensure_local_rec_importable()
    from rec.run import Run

    # One run, one node: rec describes the same IRI the generation provenance describes, so
    # the documents union instead of standing side by side.
    run = Run(observers=[_rec_observer(run_dir)], run_id=run_id)
    run._emit_started()
    run.log_host_info(host_info(environment))
    record_software(run, run_dir)
    _record_execution_inputs(run, run_dir, executable, schema, cwd, environment)
    run.observers[0].close()
    # rec has already stamped the start; a second `now()` would date the same run twice.
    record_execution(
        run_dir,
        run_id,
        source_dir / GENERATION_DOCUMENT,
        schema.get("platform") or {},
        executable_args,
        run.start_time,
    )
    _publish(run_dir, run_id)


def _rec_observer(run_dir: Path):
    from rec.observers.file_observer import FileObserver

    return FileObserver(run_dir, base=RUN_IRI_BASE)


def _record_execution_inputs(
    run, run_dir: Path, executable: Path, schema: dict, cwd: Path | None, environment: dict | None
) -> None:
    """What the execution ran and under what: the executable, its deployment, its environment."""
    inputs = [(executable, "log_producer_executable")]
    platform = schema.get("platform") or {}
    declared = platform.get("config") or ""
    if declared and not platform.get("simulated"):
        # Resolved exactly as the executable resolves it: against the working directory it gets.
        config = Path(declared)
        if not config.is_absolute():
            config = (cwd or Path.cwd()) / declared
        inputs.append((config, "deployment_config"))
    script = (environment or {}).get("script")
    if script:
        inputs.append((Path(script), "environment"))
    for path, role in inputs:
        if path.exists():
            record_used_file(run, path, role, os.path.relpath(path, run_dir))


def _rosbag_settings(source_dir: Path) -> tuple[list[str], bool]:
    """The topics the deployment config asks the run to bag, and whether it stamps them in
    simulation time: [ros.clock] means the run publishes /clock and every stamp reads from it."""
    import tomllib

    ir_path = source_dir / "model" / "ir.json"
    if not ir_path.exists():
        return [], False
    ir = json.loads(ir_path.read_text())
    declared = (ir["configuration"].get("platform") or {}).get("config") or ""
    if not declared:
        return [], False
    config_path = Path(declared)
    if not config_path.exists():
        return [], False
    try:
        config = tomllib.loads(config_path.read_text())
    except tomllib.TOMLDecodeError as error:
        raise RunnerError(f"{config_path}: {error}") from error
    topics = (config.get("rosbag") or {}).get("topics") or []
    if not isinstance(topics, list) or any(not isinstance(topic, str) for topic in topics):
        raise RunnerError(f"{config_path}: [rosbag] `topics` must be a list of topic names")
    return topics, "clock" in (config.get("ros") or {})


@contextmanager
def _rosbag_recorder(run_dir: Path, topics: list[str], use_sim_time: bool = False):
    """Bag the named topics for as long as the run lasts."""
    if not topics:
        yield
        return
    try:
        import rclpy
        import rosbag2_py
        from rclpy.signals import SignalHandlerOptions
        from rclpy.utilities import ok as rclpy_ok
    except ImportError as exc:
        raise RunnerError(
            f"[rosbag] names topics to record, but rosbag2 is not importable: {exc}.\n"
            "  Source the ROS workspace, or drop the section to run without a bag."
        ) from exc
    bag_dir = run_dir / "bag"
    if bag_dir.exists():
        raise RunnerError(f"{bag_dir}: bag directory already exists")
    options = rosbag2_py.RecordOptions()
    options.topics = topics
    options.rmw_serialization_format = "cdr"
    options.disable_keyboard_controls = True
    options.use_sim_time = use_sim_time
    # The run's own topics appear only once it starts, so this is how much of the first cycle
    # discovery can miss.
    options.topic_polling_interval = timedelta(milliseconds=50)
    # NO: an rclpy handler would take SIGINT ahead of the run's own, which reports the stop.
    started_rclpy = not rclpy_ok()
    if started_rclpy:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    recorder = rosbag2_py.Recorder(rosbag2_py.StorageOptions(uri=str(bag_dir)), options)
    spinner = threading.Thread(target=recorder.record, daemon=True)
    spinner.start()
    try:
        _await_rosbag_ready(bag_dir, spinner)
        # record() subscribes; without the spin the bag lists every topic and holds no message.
        recorder.start_spin()
        yield
    finally:
        recorder.stop()
        spinner.join(timeout=15)
        if started_rclpy:
            rclpy.shutdown()
        if not (bag_dir / "metadata.yaml").exists():
            print(f"rosbag: {bag_dir} has no metadata.yaml; the bag is unreadable", file=sys.stderr)


def _await_rosbag_ready(bag_dir: Path, spinner: threading.Thread, timeout_s: float = 30.0) -> None:
    """Block until the recorder has opened the bag, so the run does not start without it."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if bag_dir.exists():
            return
        if not spinner.is_alive():
            raise RunnerError(f"{bag_dir}: the rosbag2 recorder stopped before it opened the bag")
        time.sleep(0.01)
    raise RunnerError(f"{bag_dir}: the rosbag2 recorder did not open the bag in {timeout_s:g}s")


@contextmanager
def _real_camera_recorders(schema: dict, record: list[str] | None, output_dir: Path):
    """Record selected real RGB camera topics while the generated controller runs."""
    recorders = [
        RosImageRecorder(camera, output_dir / f"{camera.id}.mp4")
        for camera in real_camera_recordings(schema, record)
    ]
    for recorder in recorders:
        recorder.start()
    try:
        yield
    finally:
        for recorder in recorders:
            recorder.close()
            if recorder.error:
                wrote = recorder.output.exists() and recorder.output.stat().st_size > 0
                outcome = f"kept {recorder.output.name}" if wrote else "not recorded"
                print(
                    f"camera '{recorder.recording.id}': {recorder.error}; {outcome}",
                    file=sys.stderr,
                )


def _run_executable(
    executable: Path,
    executable_args: list[str],
    *,
    cwd: Path | None,
    frame_log: Path,
    run_id: str,
    rec_path: Path,
    record: list[str] | None = None,
    record_log: bool = True,
    base_env: dict[str, str] | None = None,
    schema_hash: str | None = None,
) -> int:
    # logs/ holds the console tee and any camera videos too, so it is made whether or not the
    # frame log goes in it.
    frame_log.parent.mkdir(parents=True, exist_ok=True)
    # The run's own variables are set on top of whatever the caller's environment file left,
    # so a sourced ROS overlay reaches the controller and the frame log still lands here.
    env = dict(base_env) if base_env is not None else os.environ.copy()
    # Blocks named per run, so runs of one generation can go side by side; an operator's own
    # names win.
    if schema_hash:
        env.setdefault("MOTION_SPEC_SHM_NAME", shm_name_for(schema_hash, run_id))
        env.setdefault("MOTION_SPEC_CTRL_SHM_NAME", ctrl_shm_name(schema_hash, run_id))
    # An empty path is how the runtime is told to record nothing (--no-log).
    env["MOTION_SPEC_FRAME_LOG"] = str(frame_log.resolve()) if record_log else ""
    # A camera to record, and where the video goes: the runtime renders the frame, so it
    # writes the file, beside the log of the same run.
    if record:
        env["MOTION_SPEC_RECORD_CAMERAS"] = ",".join(record)
        env["MOTION_SPEC_RECORD_DIR"] = str(frame_log.parent.resolve())
    # Where the runtime writes the seed and the draw of every sampled quantity, if it has any.
    env["MOTION_SPEC_SAMPLING_PATH"] = str((frame_log.parent / "sampling.json").resolve())
    command = [str(executable), *executable_args]
    # Why a run died is otherwise only on the operator's terminal: tee it into the run dir.
    console = (frame_log.parent / "console.log").open("w", encoding="utf-8", errors="replace")
    try:
        process = subprocess.Popen(
            command,
            cwd=str(cwd) if cwd else None,
            env=env,
            stdout=subprocess.PIPE,
            # Merged so one pump drains both and no half-read pipe can block the exit.
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
            bufsize=1,
        )
    except OSError as exc:
        console.close()
        raise RunnerError(f"{executable}: failed to launch: {exc}") from exc
    pump = threading.Thread(target=_tee, args=(process.stdout, console), daemon=True)
    pump.start()
    try:
        return process.wait()
    except KeyboardInterrupt:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        _finish_rec_run(rec_path, run_id, Verdict.ERROR)
        return 130
    finally:
        # A grandchild holding the pipe open must not stall the run's bookkeeping.
        pump.join(timeout=5)
        process.stdout.close()
        console.close()


def _tee(stream, sink) -> None:
    """Mirror the child's merged output to the operator's terminal and the run's console log."""
    try:
        for line in stream:
            sys.stdout.write(line)
            sys.stdout.flush()
            sink.write(line)
            sink.flush()
    except ValueError:
        return  # the run gave up waiting for this pump and closed the log under it


def _finish_rec_run(rec_path: Path, run_id: str, verdict: Verdict) -> None:
    """Complete the run with VERDICT: passed, or error for a run stopped early, else failed."""
    ensure_local_rec_importable()
    from rec.run import Run

    run_dir = rec_path.parent
    lifecycle = rec_run_lifecycle_from_file(rec_path)
    if (
        lifecycle["state"] is State.COMPLETE
        and lifecycle["verdict"] is verdict
        and (verdict is not Verdict.PASSED or lifecycle["completed_time"])
    ):
        return
    run = Run(observers=[_rec_observer(run_dir)], run_id=run_id)
    run.start_time = (
        parse_rec_time(lifecycle["started_time"])
        if lifecycle["started_time"]
        else datetime.now(timezone.utc)
    )
    if verdict is Verdict.PASSED:
        run._emit_completed()
    elif verdict is Verdict.ERROR:
        run._emit_interrupted()
    else:
        run._emit_failed()
    run.observers[0].close()
    _publish(run_dir, run_id)


def _record_sampling(rec_path: Path, run_id: str, path: Path) -> None:
    """The seed as a metric of the run, and each drawn value as an entity the run generated."""
    ensure_local_rec_importable()
    from rec.run import Run

    run_dir = rec_path.parent
    sampling = json.loads(path.read_text())
    run = Run(observers=[_rec_observer(run_dir)], run_id=run_id)
    run.log_scalar("sampling/seed", sampling["seed"])
    run.observers[0].close()
    drawn_at = parse_rec_time(sampling["drawn_at"])
    graph = Graph()
    for quantity, draw in sorted(sampling["draws"].items()):
        record_draw(graph, run_id, uri(CONTROLLER_PROCESS), quantity, draw["values"], drawn_at)
    write_generation_graph(run_dir / EXECUTION_DOCUMENT, GRAPH_EXECUTION, graph)


def _publish(run_dir: Path, run_id: str) -> None:
    lifecycle = rec_run_lifecycle_from_file(rec_document(run_dir, run_id))
    publish_lifecycle(run_dir, run_id, lifecycle["state"], lifecycle["verdict"])
