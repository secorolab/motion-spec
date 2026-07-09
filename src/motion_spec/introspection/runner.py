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
    HASHED_ARTIFACTS,
    ArchiveError,
    _artifact_size,
    _dependencies,
    _ensure_local_rec_importable,
    _host_info,
    _parse_rec_time,
    _record_activities,
    _record_agents,
    _repositories,
    create_archive_manifest,
    sha256_file,
    verify_manifest,
)
from motion_spec.introspection.artifacts import prov_uri


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
    rec_path = run_dir / "rec.jsonld"

    _validate_new_run(run_dir, source_dir, executable)
    schema = json.loads((source_dir / "schema.json").read_text())
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
    if returncode != 0:
        if _rec_status(rec_path) != "INTERRUPTED":
            _finish_rec_run(rec_path, run_id, "FAILED")
        create_archive_manifest(
            run_dir,
            source_dir=source_dir,
            run_id=run_id,
            frame_log=frame_log,
            log_producer_executable=executable,
            complete_rec=False,
        )
        return returncode

    try:
        create_archive_manifest(
            run_dir,
            source_dir=source_dir,
            run_id=run_id,
            frame_log=frame_log,
            log_producer_executable=executable,
            complete_rec=False,
        )
        if recover_runtime_ttl:
            from motion_spec.introspection.replay import runtime_frames
            from motion_spec.introspection.runtime_graph import write_runtime_ttl

            records, frame_count = runtime_frames(frame_log)
            write_runtime_ttl(run_dir, records, frame_count=frame_count)
        _finish_rec_run(rec_path, run_id, "COMPLETED")
        _refresh_rec_hash(run_dir)
        if verify:
            verify_manifest(run_dir)
    except Exception:
        _finish_rec_run(rec_path, run_id, "FAILED")
        _refresh_rec_hash(run_dir)
        raise
    return returncode


def _validate_new_run(run_dir: Path, source_dir: Path, executable: Path) -> None:
    if not source_dir.exists():
        raise RunnerError(f"{source_dir}: source directory does not exist")
    for rel in ("schema.json", "frame_layout.json", "provenance.jsonld"):
        if not (source_dir / rel).exists():
            raise RunnerError(f"{source_dir / rel}: required generated artifact is missing")
    if not executable.exists():
        raise RunnerError(f"{executable}: executable does not exist")
    if run_dir.exists() and (run_dir / "rec.jsonld").exists():
        raise RunnerError(f"{run_dir}: already contains rec.jsonld; choose a fresh run directory")
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
    _ensure_local_rec_importable()
    from rec import Run
    from rec.observers import FileObserver

    observer = FileObserver(run_dir / "rec.jsonld", run_id=run_id)
    run = Run(observers=[observer], run_id=run_id)
    run._emit_started()
    run.log_host_info(_host_info())
    run.log_repositories(_repositories(run_dir))
    run.log_dependencies(_dependencies())
    _record_agents(run, run_dir, schema)
    _record_activities(run, schema)
    run.add_agent(
        prov_uri("agent:motion_spec_runner"),
        ["prov:SoftwareAgent", "obs:ObservationProvider"],
        role="run_cataloguer",
    )
    run.add_activity(
        prov_uri("activity:run_cataloging"),
        ["prov:Activity"],
        role="run_cataloging",
        wasAssociatedWith=prov_uri("agent:motion_spec_runner"),
    )
    _record_execution_inputs(run, source_dir, executable, schema)
    observer.close()


def _record_execution_inputs(run, source_dir: Path, executable: Path, schema: dict) -> None:
    activity = prov_uri(schema.get("runtime_provenance", {}).get("activity_id") or "activity:controller_execution")
    for rel, role in (
        ("schema.json", "schema"),
        ("frame_layout.json", "frame_layout"),
        ("provenance.jsonld", "provenance"),
        ("model.jsonld", "model"),
        ("ir.json", "ir"),
    ):
        path = source_dir / rel
        if path.exists():
            # archivePath = where this input lands in the bundle, so the rec reference is
            # portable and dedupes with the archive's own record of the same file.
            run.add_resource(
                path,
                usage_activity=activity,
                role=role,
                archivePath=HASHED_ARTIFACTS.get(role),
                sha256=sha256_file(path),
                size_bytes=_artifact_size(path),
            )
    run.add_resource(
        executable,
        usage_activity=activity,
        role="log_producer_executable",
        archivePath=f"controller/executable/{executable.name}",
        sha256=sha256_file(executable),
        size_bytes=_artifact_size(executable),
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
    _ensure_local_rec_importable()
    from rec import Run
    from rec.observers import FileObserver

    observer = FileObserver(rec_path, run_id=run_id)
    lifecycle = observer.snapshot.get("run", {})
    if lifecycle.get("status") == status and (
        status != "COMPLETED" or lifecycle.get("completed_time")
    ):
        observer.close()
        return
    run = Run(observers=[observer], run_id=run_id)
    run._id = run_id
    run.start_time = (
        _parse_rec_time(lifecycle["started_time"])
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
    if not rec_path.exists():
        return None
    doc = json.loads(rec_path.read_text())
    if "run" in doc:
        return doc.get("run", {}).get("status")
    return doc.get("status")


def _refresh_rec_hash(run_dir: Path) -> None:
    manifest_path = run_dir / "manifest.json"
    rec_path = run_dir / "rec.jsonld"
    if not manifest_path.exists() or not rec_path.exists():
        return
    manifest = json.loads(manifest_path.read_text())
    manifest.setdefault("artifacts", {})["rec.jsonld"] = {"role": "rec", "sha256": sha256_file(rec_path)}
    manifest.setdefault("files", {})["rec"] = "rec.jsonld"
    manifest.setdefault("rec", {"path": "rec.jsonld", "run_id": manifest.get("run_id", run_dir.name)})
    manifest_path.write_text(json.dumps(manifest, indent=4) + "\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
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
