# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Launch generated controllers under REC cataloging."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from datetime import UTC, datetime, timedelta
from pathlib import Path

from rdf_utils.models.prov import (
    add_directory_location,
    capture_file_prov,
    load_transformation_prov,
)
from rdflib import Literal, URIRef
from rdflib.namespace import PROV
from rec import State, Verdict

from motion_spec.generation.codegen import CONFIG_CHECK
from motion_spec.rdf_parser.agents import agent_cameras
from motion_spec.rdf_parser.deployment import platform_facts
from motion_spec.runs.lifecycle_events import publish_lifecycle
from motion_spec.runs.provenance import (
    GENERATION_DOCUMENT,
    add_package,
    generation_model,
    rec_run,
    record_draw,
    record_execution,
    record_files,
    record_run_file,
    run_bundle,
)
from motion_spec.runs.ros_video import RosImageRecorder, real_camera_recordings
from motion_spec.runs.verify import FRAME_LOG, verify_run
from motion_spec.telemetry.frame_log_pb import (
    HEADER_FILE,
    LOG_SUFFIX,
    ctrl_shm_name,
    read_contract,
    read_sampling,
    shm_name_for,
)

# most of the ratio at a fraction of the top levels' time
COMPRESSION_LEVEL = 10


class RunnerError(RuntimeError):
    """A cataloged generated executable run could not be completed."""


def run_cataloged(
    run_dir: Path | str,
    *,
    source_dir: Path | str,
    executable: Path | str,
    run_id: str | None = None,
    cwd: Path | str | None = None,
    record: list[str] | None = None,
    record_log: bool = True,
    env: dict[str, str] | None = None,
    environment_script: Path | None = None,
) -> int:
    """Run a generated executable and record it: rec's lifecycle and motion-spec's provenance.

    ENV is what the executable runs under; ENVIRONMENT_SCRIPT is the file it was sourced
    from, which the run keeps.
    """
    run_dir = Path(run_dir)
    source_dir = Path(source_dir)
    executable = Path(executable).resolve()
    cwd = Path(cwd).resolve() if cwd else None
    run_id = run_id or run_dir.name
    frame_log = run_dir / FRAME_LOG

    if not source_dir.exists():
        raise RunnerError(f"{source_dir}: source directory does not exist")
    for path in (source_dir / "contract" / HEADER_FILE, source_dir / GENERATION_DOCUMENT):
        if not path.exists():
            raise RunnerError(f"{path}: required generated artifact is missing")
    if not executable.exists():
        raise RunnerError(f"{executable}: executable does not exist")
    validate_robot_config(source_dir, cwd)
    if rec_run(run_dir, run_id).store.load(run_id) is not None:
        raise RunnerError(f"{run_dir}: already records a run; choose a fresh run directory")
    if frame_log.exists():
        raise RunnerError(f"{frame_log}: refusing to overwrite an existing frame log")
    schema_hash = read_contract(source_dir / "contract" / HEADER_FILE).header.schema_hash
    model = generation_model(source_dir)
    platform = platform_facts(model.graph)
    config = Path(platform["config"]) if platform["config"] else None
    run_dir.mkdir(parents=True, exist_ok=True)
    run, bundle_id = start_rec_run(
        run_dir,
        run_id,
        source_dir,
        executable,
        platform,
        config=config,
        environment_script=environment_script,
    )

    try:
        # [ros.clock]: the run publishes /clock, so bag in sim time
        topics, use_sim_time = [], False
        if config is not None and config.exists():
            import tomllib

            try:
                deployment = tomllib.loads(config.read_text())
            except tomllib.TOMLDecodeError as error:
                raise RunnerError(f"{config}: {error}") from error
            topics = (deployment.get("rosbag") or {}).get("topics") or []
            if not isinstance(topics, list) or any(not isinstance(topic, str) for topic in topics):
                raise RunnerError(f"{config}: [rosbag] `topics` must be a list of topic names")
            use_sim_time = "clock" in (deployment.get("ros") or {})
        with (
            rosbag_recorder(run_dir, topics, use_sim_time),
            real_camera_recorders(
                [] if platform["simulated"] else agent_cameras(model), record, frame_log.parent
            ),
        ):
            returncode = run_executable(
                executable,
                cwd=cwd,
                frame_log=frame_log,
                run_id=run_id,
                run=run,
                record=record,
                record_log=record_log,
                base_env=env,
                schema_hash=schema_hash,
            )
    except Exception:
        finish_rec_run(run_dir, run, Verdict.FAILED)
        raise
    sampling = read_sampling(frame_log) if record_log and frame_log.exists() else None
    if sampling:
        drawn_at = datetime.fromtimestamp(sampling["drawn_at_ns"] / 1e9, tz=UTC)
        with run_bundle(run_dir, run_id, bundle_id) as bundle:
            process = next(bundle.subjects(PROV.qualifiedDelegation), None)
            for quantity, values in sampling["draws"].items():
                record_draw(bundle, process, quantity, values, drawn_at)
    # 130: stopped by SIGINT/SIGTERM with complete artifacts, not a failure
    verdict = {0: Verdict.PASSED, 130: Verdict.ERROR}.get(returncode, Verdict.FAILED)

    # died before its first frame: surface the executable's error, not a missing log
    if returncode != 0 and record_log and not frame_log.exists():
        finish_rec_run(run_dir, run, verdict)
        return returncode

    try:
        record_outputs(run_dir, run, bundle_id, recorded=record_log)
        if returncode == 0:
            verify_run(run_dir)
    except Exception:
        finish_rec_run(run_dir, run, Verdict.FAILED if returncode == 0 else verdict)
        raise
    finish_rec_run(run_dir, run, verdict)
    return returncode


def validate_robot_config(source_dir: Path, cwd: Path | None = None) -> None:
    """Check the deployment config before launching, so a typo fails here, not against hardware.

    A real-hardware generation ships the check beside its program, with the sections the program
    reads baked in, so it reads the config file alone; run in CWD, which a relative config path
    is resolved against, as the program resolves it. A simulation ships none.
    """
    check = source_dir / "controller" / CONFIG_CHECK
    if not check.is_file():
        return
    done = subprocess.run(
        [sys.executable, str(check)], cwd=cwd, capture_output=True, text=True, check=False
    )
    if done.returncode:
        raise RunnerError(done.stderr.strip())


def start_rec_run(
    run_dir: Path,
    run_id: str,
    source_dir: Path,
    executable: Path,
    platform: dict,
    config: Path | None = None,
    environment_script: Path | None = None,
):
    """Start the run in rec and record in motion-spec's bundle what it runs and under what:
    the deployment CONFIG, the model's, and the ENVIRONMENT_SCRIPT it was given.

    Returns:
        rec's handle on the run and the identifier of motion-spec's bundle, which the run's
        later steps record into
    """
    run = rec_run(run_dir, run_id)
    run.start()
    with run_bundle(run_dir, run_id) as bundle:
        inputs = [config] if config is not None else []
        if environment_script is not None and environment_script.is_file():
            kept = run_dir / "files" / f"environment{environment_script.suffix}"
            kept.parent.mkdir(exist_ok=True)
            shutil.copy2(environment_script, kept)
            inputs.append(kept)
        for path in inputs:
            if path.exists():
                record_run_file(bundle, run.node, run_dir.parents[1], path, generated=False)
        record_execution(bundle, run.node, source_dir, executable, platform)
    publish_lifecycle(run_dir, run_id, *run.lifecycle())
    return run, bundle.identifier


@contextmanager
def rosbag_recorder(run_dir: Path, topics: list[str], use_sim_time: bool = False):
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
    options.topic_polling_interval = timedelta(milliseconds=50)
    # SIGINT belongs to the run's own handler
    started_rclpy = not rclpy_ok()
    if started_rclpy:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    recorder = rosbag2_py.Recorder(rosbag2_py.StorageOptions(uri=str(bag_dir)), options)
    spinner = threading.Thread(target=recorder.record, daemon=True)
    spinner.start()
    try:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if bag_dir.exists():
                break
            if not spinner.is_alive():
                raise RunnerError(
                    f"{bag_dir}: the rosbag2 recorder stopped before it opened the bag"
                )
            time.sleep(0.01)
        else:
            raise RunnerError(f"{bag_dir}: the rosbag2 recorder did not open the bag in 30s")
        # without the spin the bag lists topics but holds no message
        recorder.start_spin()
        yield
    finally:
        recorder.stop()
        spinner.join(timeout=15)
        if started_rclpy:
            rclpy.shutdown()
        if not (bag_dir / "metadata.yaml").exists():
            print(f"rosbag: {bag_dir} has no metadata.yaml; the bag is unreadable", file=sys.stderr)


@contextmanager
def real_camera_recorders(cameras: list, record: list[str] | None, output_dir: Path):
    """Record selected real RGB camera topics while the generated controller runs."""
    recorders = [
        RosImageRecorder(camera, output_dir / f"{camera.id}.mp4")
        for camera in real_camera_recordings(cameras, record)
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


def run_executable(
    executable: Path,
    *,
    cwd: Path | None,
    frame_log: Path,
    run_id: str,
    run,
    record: list[str] | None = None,
    record_log: bool = True,
    base_env: dict[str, str] | None = None,
    schema_hash: str | None = None,
) -> int:
    frame_log.parent.mkdir(parents=True, exist_ok=True)
    env = dict(base_env) if base_env is not None else os.environ.copy()
    if schema_hash:
        env.setdefault("MOTION_SPEC_SHM_NAME", shm_name_for(schema_hash, run_id))
        env.setdefault("MOTION_SPEC_CTRL_SHM_NAME", ctrl_shm_name(schema_hash, run_id))
    # empty: record nothing
    env["MOTION_SPEC_FRAME_LOG"] = str(frame_log.resolve()) if record_log else ""
    if record:
        env["MOTION_SPEC_RECORD_CAMERAS"] = ",".join(record)
        env["MOTION_SPEC_RECORD_DIR"] = str(frame_log.parent.resolve())
    console = (frame_log.parent / "console.log").open("w", encoding="utf-8", errors="replace")
    try:
        process = subprocess.Popen(
            [str(executable)],
            cwd=str(cwd) if cwd else None,
            env=env,
            stdout=subprocess.PIPE,
            # one pump drains both, so no half-read pipe blocks the exit
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
            bufsize=1,
        )
    except OSError as exc:
        console.close()
        raise RunnerError(f"{executable}: failed to launch: {exc}") from exc
    pump = threading.Thread(target=tee, args=(process.stdout, console), daemon=True)
    pump.start()
    canceled_at = None
    try:
        while True:
            try:
                return process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass
            if canceled_at is None:
                if run.lifecycle()[0] is State.CANCELING:
                    process.terminate()
                    canceled_at = time.monotonic()
            elif time.monotonic() - canceled_at > 5:
                process.kill()
    except KeyboardInterrupt:
        if run.lifecycle()[0] is State.IN_PROGRESS:
            run.cancel()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        return 130
    finally:
        # a grandchild may hold the pipe open
        pump.join(timeout=5)
        process.stdout.close()
        console.close()


def tee(stream, sink) -> None:
    """Mirror the child's merged output to the operator's terminal and the run's console log."""
    try:
        for line in stream:
            sys.stdout.write(line)
            sys.stdout.flush()
            sink.write(line)
            sink.flush()
    except ValueError:
        return  # log closed after the join timed out


def record_outputs(run_dir: Path, run, bundle_id: URIRef, *, recorded: bool = True) -> None:
    """Record in motion-spec's bundle of the run what it left in RUN_DIR once the program exited.

    The frame log of a RECORDED run is packed with zstd, as nothing appends to it any more and
    neighbouring frames nearly repeat: the run generated the raw log, and the packing derived
    the packed one from it and invalidated the raw one. Every other file it generated -- the
    console log, camera videos, the rosbag -- is recorded as generated by the run.
    """
    import zstandard

    frame_log = run_dir / FRAME_LOG
    generation = run_dir.parents[1]
    with run_bundle(run_dir, run.id, bundle_id) as bundle:
        if recorded and frame_log.is_file():
            raw = record_run_file(bundle, run.node, generation, frame_log, generated=True)
            packed = frame_log.with_name(frame_log.name + LOG_SUFFIX)
            started = datetime.now(UTC)
            with frame_log.open("rb") as raw_log, packed.open("wb") as out:
                zstandard.ZstdCompressor(level=COMPRESSION_LEVEL).copy_stream(raw_log, out)
            frame_log.unlink()
            ended = datetime.now(UTC)
            roots = {generation.resolve(): add_directory_location(bundle, generation)}
            packing = load_transformation_prov(
                bundle,
                [raw],
                [capture_file_prov(bundle, packed, roots=roots)],
                add_package(bundle, "motion_spec"),
                started=started,
                ended=ended,
            )
            bundle.add((raw, PROV.wasInvalidatedBy, packing))
            bundle.add((raw, PROV.invalidatedAtTime, Literal(ended)))
        record_files(bundle, run.node, run_dir)


def finish_rec_run(run_dir: Path, run, verdict: Verdict) -> None:
    """End RUN with VERDICT: passed, or error for a run stopped early, else failed.

    A run that has already ended keeps its first end: rec refuses to end a run twice. A run
    cancelled while in progress that stopped as asked ends passed, which rec records canceled.
    """
    state = run.lifecycle()[0]
    if state not in (State.IN_PROGRESS, State.CANCELING):
        return
    run.end(Verdict.PASSED if state is State.CANCELING and verdict is Verdict.ERROR else verdict)
    publish_lifecycle(run_dir, run.id, *run.lifecycle())
