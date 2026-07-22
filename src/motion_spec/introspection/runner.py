# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Launch generated introspection executables under REC cataloging."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from motion_spec.introspection.archive import (
    ArchiveError,
    create_archive_manifest,
    verify_manifest,
)
from motion_spec.provenance import (
    artifact_sha256,
    artifact_size,
    dependencies,
    ensure_local_rec_importable,
    host_info,
    parse_rec_time,
    rec_types,
    rec_run_lifecycle,
    rec_run_lifecycle_from_file,
    prov_uri,
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
    verify: bool = True,
) -> int:
    """Run a generated executable with REC lifecycle and archive provenance."""
    run_dir = Path(run_dir)
    source_dir = Path(source_dir)
    executable = Path(executable).resolve()
    executable_args = [str(arg) for arg in (executable_args or [])]
    run_id = run_id or run_dir.name
    frame_log = run_dir / "logs" / "frame_log.pb"
    rec_path = run_dir / "rec.ld.json"

    _validate_new_run(run_dir, source_dir, executable)
    schema_path = (
        source_dir / "contract" / "schema.json"
        if (source_dir / "contract").is_dir()
        else source_dir / "schema.json"
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
        )
    except Exception:
        _finish_rec_run(rec_path, run_id, "FAILED")
        raise
    if returncode != 0 and _rec_status(rec_path) != "INTERRUPTED":
        _finish_rec_run(rec_path, run_id, "FAILED")

    try:
        create_archive_manifest(
            run_dir,
            source_dir=source_dir,
            run_id=run_id,
            frame_log=frame_log,
            log_producer_executable=executable,
            complete_rec=False,
        )
        if recover_runtime_ttl and (returncode == 0 or frame_log.exists()):
            from motion_spec.introspection.replay import runtime_frames
            from motion_spec.introspection.runtime_graph import write_runtime_ttl

            records, frame_count = runtime_frames(frame_log)
            write_runtime_ttl(run_dir, records, frame_count=frame_count)
        if returncode == 0:
            _finish_rec_run(rec_path, run_id, "COMPLETED")
            if verify:
                verify_manifest(run_dir)
    except Exception:
        if returncode == 0:
            _finish_rec_run(rec_path, run_id, "FAILED")
        raise
    return returncode


def _validate_new_run(run_dir: Path, source_dir: Path, executable: Path) -> None:
    if not source_dir.exists():
        raise RunnerError(f"{source_dir}: source directory does not exist")
    required = (
        (
            source_dir / "contract" / "schema.json",
            source_dir / "contract" / "frame_log.proto",
            source_dir / "provenance" / "motion-spec.ld.json",
        )
        if (source_dir / "contract").is_dir()
        else tuple(source_dir / rel for rel in ("schema.json", "frame_log.proto", "provenance.ld.json"))
    )
    for path in required:
        if not path.exists():
            raise RunnerError(f"{path}: required generated artifact is missing")
    if not executable.exists():
        raise RunnerError(f"{executable}: executable does not exist")
    if run_dir.exists() and (run_dir / "rec.ld.json").exists():
        raise RunnerError(f"{run_dir}: already contains rec.ld.json; choose a fresh run directory")
    frame_log = run_dir / "logs" / "frame_log.pb"
    if frame_log.exists():
        raise RunnerError(f"{frame_log}: refusing to overwrite an existing frame log")


def _start_rec_run(
    run_dir: Path,
    run_id: str,
    source_dir: Path,
    executable: Path,
    schema: dict,
) -> None:
    ensure_local_rec_importable()
    from rec import Run
    from rec.observers import FileObserver

    observer = FileObserver(run_dir / "rec.ld.json")
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


def _record_execution_inputs(
    run, run_dir: Path, source_dir: Path, executable: Path, schema: dict
) -> None:
    activity = prov_uri(schema.get("runtime_provenance", {}).get("activity_id") or "activity:controller_execution")
    inputs = (
        (
            ("contract/schema.json", "schema"),
            ("provenance/motion-spec.ld.json", "provenance"),
            ("model/ir.json", "ir"),
        )
        if (source_dir / "contract").is_dir()
        else (
            ("schema.json", "schema"),
            ("provenance.ld.json", "provenance"),
            ("model.ld.json", "model"),
            ("ir.json", "ir"),
        )
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
) -> int:
    frame_log.parent.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["MOTION_SPEC_FRAME_LOG"] = str(frame_log.resolve())
    env["MOTION_SPEC_RUN_ID"] = run_id
    env["MOTION_SPEC_REC_PATH"] = str(rec_path.resolve())
    command = [str(executable), *executable_args]
    try:
        process = subprocess.Popen(command, cwd=str(cwd) if cwd else None, env=env)
    except OSError as exc:
        raise RunnerError(f"{executable}: failed to launch: {exc}") from exc
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


def _finish_rec_run(rec_path: Path, run_id: str, status: str) -> None:
    ensure_local_rec_importable()
    from rec import Run
    from rec.observers import FileObserver

    observer = FileObserver(rec_path)
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


def _rec_status(rec_path: Path) -> str | None:
    return rec_run_lifecycle_from_file(rec_path).get("status")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="motion-spec run")
    parser.add_argument("run_dir", help="fresh directory for the cataloged run archive")
    parser.add_argument("--source-dir", required=True, help="generated controller directory")
    parser.add_argument("--executable", required=True, help="generated executable to launch")
    parser.add_argument("--run-id", default=None, help="stable run id; defaults to run_dir name")
    parser.add_argument("--cwd", default=None, help="working directory for the generated executable")
    parser.add_argument(
        "--recover-runtime-ttl",
        action="store_true",
        help="recover runtime.ttl from the captured frame log after the run",
    )
    parser.add_argument(
        "--no-verify",
        action="store_true",
        help="skip archive verification after a successful run",
    )
    raw_args = list(sys.argv[1:] if argv is None else argv)
    if "--" in raw_args:
        separator = raw_args.index("--")
        parser_args = raw_args[:separator]
        executable_args = raw_args[separator + 1 :]
    else:
        parser_args = raw_args
        executable_args = []
    args = parser.parse_args(parser_args)
    try:
        return run_cataloged(
            args.run_dir,
            source_dir=args.source_dir,
            executable=args.executable,
            executable_args=executable_args,
            run_id=args.run_id,
            cwd=args.cwd,
            recover_runtime_ttl=args.recover_runtime_ttl,
            verify=not args.no_verify,
        )
    except (ArchiveError, RunnerError) as exc:
        parser.exit(2, f"{exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
