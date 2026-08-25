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
from datetime import datetime, timezone
from pathlib import Path

from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.archive import (
    consolidate_provenance,
    create_archive_manifest,
    verify_manifest,
)
from motion_spec.introspection.lifecycle_events import publish_lifecycle
from motion_spec.introspection.provenance import (
    artifact_sha256,
    artifact_size,
    dependencies,
    ensure_local_rec_importable,
    host_info,
    parse_rec_time,
    prov_uri,
    rec_run_lifecycle,
    rec_run_lifecycle_from_file,
    rec_types,
    record_activities,
    record_agents,
    repositories,
)


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
    recover_runtime_ttl: bool = False,
    record: list[str] | None = None,
    record_log: bool = True,
) -> int:
    """Run a generated executable with REC lifecycle and archive provenance."""
    run_dir = Path(run_dir)
    source_dir = Path(source_dir)
    executable = Path(executable).resolve()
    executable_args = [str(arg) for arg in (executable_args or [])]
    run_id = run_id or run_dir.name
    frame_log = run_dir / "logs" / "frame_log.pb"
    rec_path = run_dir / "rec.ld.json"

    _validate_new_run(run_dir, source_dir, executable, Path(cwd).resolve() if cwd else None)
    # frame_layout.json, not the log: the run is recorded before the log exists.
    schema_path = (
        source_dir / "contract" / "frame_layout.json"
        if (source_dir / "contract").is_dir()
        else source_dir / "frame_layout.json"
    )
    schema = json.loads(schema_path.read_text())
    run_dir.mkdir(parents=True, exist_ok=True)
    _start_rec_run(run_dir, run_id, source_dir, executable, schema)

    try:
        returncode = _run_executable(
            executable,
            executable_args,
            cwd=Path(cwd).resolve() if cwd else None,
            frame_log=frame_log,
            run_id=run_id,
            rec_path=rec_path,
            record=record,
            record_log=record_log,
        )
    except Exception:
        _finish_rec_run(rec_path, run_id, "FAILED")
        raise
    if _rec_status(rec_path) != "INTERRUPTED":
        # 130 is the program leaving its loop on SIGINT/SIGTERM, which it reports rather than
        # dying from: the run stopped early but its artifacts are complete, so it is not a
        # failure. Reached when the signal went to the child alone and never raised here.
        if returncode == 130:
            _finish_rec_run(rec_path, run_id, "INTERRUPTED")
        elif returncode != 0:
            _finish_rec_run(rec_path, run_id, "FAILED")

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
        # Archiving has just packed the log, so ask after it by whichever name it now has --
        # by the one it was written under, an interrupted run looks like it recorded nothing.
        archived_log = frame_log_pb.log_path(frame_log)
        if recover_runtime_ttl and record_log and (returncode == 0 or archived_log.exists()):
            from motion_spec.introspection.replay import runtime_frames
            from motion_spec.introspection.runtime_graph import write_runtime_ttl

            # Recovery reads the whole frame log: report how long it took, and never let a
            # failure vanish behind whatever the caller does with the raise.
            started = time.monotonic()
            try:
                records, _frame_count = runtime_frames(archived_log)
                write_runtime_ttl(run_dir, records)
            except Exception as exc:
                print(f"runtime.ttl recovery failed: {exc!r}", file=sys.stderr)
                raise
            print(f"runtime.ttl recovered in {time.monotonic() - started:.1f}s")
        if returncode == 0:
            _finish_rec_run(rec_path, run_id, "COMPLETED")
            # The lifecycle is terminal, so the run's documents are final and join into one
            # dataset -- the thing every cross-layer question is asked of.
            consolidate_provenance(run_dir)
            # A recording nobody checked is not worth the disk it sits on.
            verify_manifest(run_dir)
    except Exception:
        if returncode == 0:
            _finish_rec_run(rec_path, run_id, "FAILED")
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
    # [ros.*] configures the generated publishers, not a device this run binds.
    sections = {key for key in _config_sections(config) if key.split(".")[0] != "ros"}
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
    run_dir: Path, source_dir: Path, executable: Path, cwd: Path | None = None
) -> None:
    if not source_dir.exists():
        raise RunnerError(f"{source_dir}: source directory does not exist")
    required = (
        (
            source_dir / "contract" / "frame_log.proto",
            source_dir / "provenance" / "motion-spec.ld.json",
        )
        if (source_dir / "contract").is_dir()
        else tuple(source_dir / rel for rel in ("frame_log.proto", "provenance.ld.json"))
    )
    for path in required:
        if not path.exists():
            raise RunnerError(f"{path}: required generated artifact is missing")
    if not executable.exists():
        raise RunnerError(f"{executable}: executable does not exist")
    _validate_robot_config(source_dir, cwd)
    if run_dir.exists() and (run_dir / "rec.ld.json").exists():
        raise RunnerError(f"{run_dir}: already contains rec.ld.json; choose a fresh run directory")
    frame_log = run_dir / "logs" / "frame_log.pb"
    if frame_log.exists():
        raise RunnerError(f"{frame_log}: refusing to overwrite an existing frame log")


def _start_rec_run(
    run_dir: Path, run_id: str, source_dir: Path, executable: Path, schema: dict
) -> None:
    ensure_local_rec_importable()
    from rec import Run
    from rec.observers import FileObserver

    # One run, one node: rec describes the same IRI the runtime graph and the generation
    # provenance describe, so the three documents union instead of standing side by side.
    observer = FileObserver(run_dir / "rec.ld.json", run_iri=prov_uri(f"run:{run_id}"))
    run = Run(observers=[observer], run_id=run_id)
    run._emit_started()
    run.log_host_info(host_info())
    run.log_repositories(repositories(run_dir))
    run.log_dependencies(dependencies())
    record_agents(run, run_dir, schema)
    record_activities(run, schema)
    run.add_agent(
        prov_uri("agent:motion_spec_runner"),
        rec_types(["prov:SoftwareAgent", "obs:ObservationProvider"]),
    )
    run.add_activity(
        prov_uri("activity:run_cataloging"),
        rec_types(["prov:Activity"]),
        associated_with=prov_uri("agent:motion_spec_runner"),
    )
    _record_execution_inputs(run, run_dir, source_dir, executable, schema)
    observer.close()
    publish_lifecycle(
        run_dir, run_id, rec_run_lifecycle_from_file(run_dir / "rec.ld.json")["status"]
    )


def _record_execution_inputs(
    run, run_dir: Path, source_dir: Path, executable: Path, schema: dict
) -> None:
    activity = prov_uri(
        schema.get("runtime_provenance", {}).get("activity_id") or "activity:controller_execution"
    )
    inputs = (
        (("provenance/motion-spec.ld.json", "provenance"), ("model/ir.json", "ir"))
        if (source_dir / "contract").is_dir()
        else (("provenance.ld.json", "provenance"), ("model.ld.json", "model"), ("ir.json", "ir"))
    )
    for rel, role in inputs:
        path = source_dir / rel
        if path.exists():
            # archivePath = where this input lands in the bundle, so the rec reference is
            # portable and dedupes with the archive's own record of the same file.
            run.add_resource(
                path,
                usage_activity=activity,
                title=role,
                archive_path=os.path.relpath(path, run_dir),
                sha256=artifact_sha256(path),
                size_bytes=artifact_size(path),
            )
    run.add_resource(
        executable,
        usage_activity=activity,
        title="log_producer_executable",
        archive_path=os.path.relpath(executable, run_dir),
        sha256=artifact_sha256(executable),
        size_bytes=artifact_size(executable),
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
) -> int:
    # logs/ holds the console tee and any camera videos too, so it is made whether or not the
    # frame log goes in it.
    frame_log.parent.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    # An empty path is how the runtime is told to record nothing (--no-log).
    env["MOTION_SPEC_FRAME_LOG"] = str(frame_log.resolve()) if record_log else ""
    env["MOTION_SPEC_RUN_ID"] = run_id
    env["MOTION_SPEC_REC_PATH"] = str(rec_path.resolve())
    # A camera to record, and where the video goes: the runtime renders the frame, so it
    # writes the file, beside the log of the same run.
    if record:
        env["MOTION_SPEC_RECORD_CAMERAS"] = ",".join(record)
        env["MOTION_SPEC_RECORD_DIR"] = str(frame_log.parent.resolve())
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
        _finish_rec_run(rec_path, run_id, "INTERRUPTED")
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


def _finish_rec_run(rec_path: Path, run_id: str, status: str) -> None:
    ensure_local_rec_importable()
    from rec import Run
    from rec.observers import FileObserver

    observer = FileObserver(rec_path, run_iri=prov_uri(f"run:{run_id}"))
    lifecycle = rec_run_lifecycle(observer.graph)
    if lifecycle.get("status") == status and (
        status != "COMPLETED" or lifecycle.get("completed_time")
    ):
        observer.close()
        return
    run = Run(observers=[observer], run_id=run_id)
    run._id = run_id
    run.start_time = (
        parse_rec_time(lifecycle["started_time"])
        if lifecycle.get("started_time")
        else datetime.now(timezone.utc)
    )
    if status == "COMPLETED":
        run._emit_completed()
    elif status == "INTERRUPTED":
        run._emit_interrupted()
    else:
        run._emit_failed()
    observer.close()
    publish_lifecycle(rec_path.parent, run_id, rec_run_lifecycle_from_file(rec_path)["status"])


def _rec_status(rec_path: Path) -> str | None:
    return rec_run_lifecycle_from_file(rec_path).get("status")
