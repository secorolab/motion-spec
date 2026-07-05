# SPDX-License-Identifier: MPL-2.0
"""Self-contained run archive manifest helpers."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import platform
import shutil
import socket
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import rdflib
from pyshacl import validate

from motion_spec.manifest import metamodel_url_map

MANIFEST_VERSION = 1
PROVENANCE_DOCUMENT_VERSION = 1
HASHED_ARTIFACTS = {
    "schema": "contract/schema.json",
    "frame_layout": "contract/frame_layout.json",
    "provenance": "provenance/codegen.jsonld",
    "runtime": "runtime/runtime.ttl",
    "frame_log": "logs/frame_log.bin",
    "model": "model/model.jsonld",
    "ir": "model/ir.json",
    "controller": "controller/source",
}
GENERATED_BUNDLE_FILES = (
    "CMakeLists.txt",
    "ref_main.cpp",
    "frame_layout.h",
    "introspection_runtime.hpp",
    "introspect_model.hpp",
    "fsm_ir.json",
    "headers",
)


class ArchiveError(ValueError):
    """Archive verification failed."""


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def short_hash(path: Path) -> str:
    return sha256_file(path)[:16]


def hash_tree(path: Path) -> str:
    h = hashlib.sha256()
    for item in sorted(p for p in path.rglob("*") if p.is_file()):
        h.update(item.relative_to(path).as_posix().encode())
        h.update(b"\0")
        h.update(item.read_bytes())
        h.update(b"\0")
    return h.hexdigest()


def _copy_file(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def create_archive_manifest(
    run_dir: Path | str,
    *,
    source_dir: Path | str | None = None,
    run_id: str | None = None,
    streams: list[dict] | None = None,
    frame_log: Path | str | None = None,
    log_producer_executable: Path | str | None = None,
    rec: Path | str | None = None,
    complete_rec: bool = True,
) -> dict:
    """Create or refresh a local replay manifest for a run folder."""
    run_dir = Path(run_dir)
    source_dir = Path(source_dir) if source_dir else run_dir
    run_dir.mkdir(parents=True, exist_ok=True)

    copies = {
        "schema.json": "contract/schema.json",
        "frame_layout.json": "contract/frame_layout.json",
        "provenance.jsonld": "provenance/codegen.jsonld",
    }
    if frame_log and Path(frame_log).exists():
        copies[str(Path(frame_log).resolve())] = "logs/frame_log.bin"
    elif (source_dir / "frame_log.bin").exists():
        copies["frame_log.bin"] = "logs/frame_log.bin"
    schema = json.loads((source_dir / "schema.json").read_text())
    if (source_dir / "model.jsonld").exists():
        copies["model.jsonld"] = "model/model.jsonld"
    elif schema.get("graph") and Path(schema["graph"]).exists():
        copies[str(Path(schema["graph"]))] = "model/model.jsonld"
    if schema.get("ir_path") and Path(schema["ir_path"]).exists():
        copies[str(Path(schema["ir_path"]))] = "model/ir.json"

    for src_name, dst_name in copies.items():
        src = Path(src_name) if Path(src_name).is_absolute() else source_dir / src_name
        if src.exists() and src.resolve() != (run_dir / dst_name).resolve():
            _copy_file(src, run_dir / dst_name)

    controller_dir = run_dir / "controller" / "source"
    controller_dir.mkdir(parents=True, exist_ok=True)
    for rel in GENERATED_BUNDLE_FILES:
        src = source_dir / rel
        dst = controller_dir / rel
        if src.is_file():
            _copy_file(src, dst)
        elif src.is_dir():
            if dst.exists():
                shutil.rmtree(dst)
            shutil.copytree(src, dst)
    for header in sorted(source_dir.glob("*_fsm.hpp")):
        _copy_file(header, controller_dir / header.name)
    if log_producer_executable and Path(log_producer_executable).exists():
        executable = Path(log_producer_executable)
        _copy_file(executable, run_dir / "controller" / "executable" / executable.name)

    artifacts = {}
    for key, rel in HASHED_ARTIFACTS.items():
        path = run_dir / rel
        if path.is_file():
            artifacts[rel] = {"role": key, "sha256": sha256_file(path)}
        elif path.is_dir():
            artifacts[rel] = {"role": key, "sha256": hash_tree(path)}
    if log_producer_executable:
        executable = (
            run_dir
            / "controller"
            / "executable"
            / Path(log_producer_executable).name
        )
        if executable.exists():
            artifacts[f"controller/executable/{executable.name}"] = {
                "role": "log_producer_executable",
                "sha256": sha256_file(executable),
            }

    manifest = {
        "manifest_version": MANIFEST_VERSION,
        "run_id": run_id or run_dir.name,
        "files": {
            "schema": "contract/schema.json",
            "frame_layout": "contract/frame_layout.json",
            "provenance": "provenance/codegen.jsonld",
            "runtime_ttl": "runtime/runtime.ttl",
            "frame_log": "logs/frame_log.bin",
            "model": "model/model.jsonld",
            "ir": "model/ir.json",
            "controller": "controller/source",
            "log_producer_executable": (
                f"controller/executable/{Path(log_producer_executable).name}"
                if log_producer_executable
                else None
            ),
            "rec": "rec.json",
        },
        "versions": {
            "manifest": MANIFEST_VERSION,
            "schema": schema.get("schema_version"),
            "frame_layout": schema.get("frame_layout_version"),
            "runtime_rdf_contract": schema.get("runtime_rdf_contract_version"),
            "provenance_document": PROVENANCE_DOCUMENT_VERSION,
        },
        "contexts": schema.get("provenance_contexts", []),
        "artifacts": artifacts,
        "provenance": {
            "document": "provenance/codegen.jsonld",
            "runtime": "runtime/runtime.ttl",
            "rec": "rec.json",
        },
        "streams": streams or [],
    }
    if rec and Path(rec).exists():
        _copy_file(Path(rec), run_dir / "rec.json")
    else:
        _write_rec_snapshot(run_dir, manifest, schema, complete_lifecycle=complete_rec)
    manifest["artifacts"]["rec.json"] = {"role": "rec", "sha256": sha256_file(run_dir / "rec.json")}
    manifest["rec"] = {"path": "rec.json", "run_id": manifest["run_id"]}
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=4) + "\n")
    return manifest


def load_manifest(run_dir_or_manifest: Path | str) -> tuple[Path, dict]:
    path = Path(run_dir_or_manifest)
    manifest_path = path if path.name == "manifest.json" else path / "manifest.json"
    if not manifest_path.exists():
        raise ArchiveError(f"{manifest_path}: missing manifest.json")
    return manifest_path.parent, json.loads(manifest_path.read_text())


def _artifact_path(run_dir: Path, rel: str) -> Path:
    path = run_dir / rel
    if not path.exists():
        raise ArchiveError(f"{rel}: missing required archive artifact")
    return path


def verify_manifest(run_dir_or_manifest: Path | str) -> dict:
    run_dir, manifest = load_manifest(run_dir_or_manifest)
    errors = []
    for rel, meta in manifest.get("artifacts", {}).items():
        path = run_dir / rel
        if not path.exists():
            errors.append(f"{rel}: missing")
            continue
        actual = hash_tree(path) if path.is_dir() else sha256_file(path)
        if actual != meta.get("sha256"):
            errors.append(f"{rel}: sha256 mismatch")
    for key in ("schema", "frame_layout", "provenance", "frame_log", "model", "ir", "controller", "rec"):
        rel = manifest.get("files", {}).get(key)
        if not rel and key in ("schema", "frame_layout", "provenance", "frame_log", "rec"):
            errors.append(f"files.{key}: missing")
        elif rel and rel in manifest.get("artifacts", {}) and not (run_dir / rel).exists():
            errors.append(f"{rel}: missing")
    for stream in manifest.get("streams", []):
        for field in ("id", "kind", "mode", "url"):
            if not stream.get(field):
                errors.append(f"stream {stream.get('id', '<unknown>')}: missing {field}")
    if errors:
        raise ArchiveError("; ".join(errors))
    provenance_graph = _parse_rdf(run_dir / manifest["files"]["provenance"], "json-ld")
    _require_provenance(provenance_graph, "provenance.jsonld")
    _validate_prov_shacl(run_dir / manifest["files"]["provenance"])
    model_rel = manifest.get("files", {}).get("model")
    if model_rel and (run_dir / model_rel).exists():
        _parse_rdf(run_dir / model_rel, "json-ld")
    runtime_rel = manifest.get("files", {}).get("runtime_ttl")
    if runtime_rel and (run_dir / runtime_rel).exists():
        runtime_graph = _parse_rdf(run_dir / runtime_rel, "turtle")
        _require_runtime_provenance(runtime_graph, "runtime.ttl")
    rec_rel = manifest.get("files", {}).get("rec")
    if rec_rel and (run_dir / rec_rel).exists():
        rec_graph = _parse_rdf(run_dir / rec_rel, "json-ld")
        _require_rec_provenance(rec_graph, "rec.json")
        _validate_prov_shacl(run_dir / rec_rel)
    return manifest


def _parse_rdf(path: Path, fmt: str) -> rdflib.Graph:
    try:
        from rdf_utils.resolver import IriToFileResolver, install_resolver

        install_resolver(IriToFileResolver(metamodel_url_map(), download=False))
    except Exception:
        pass
    try:
        graph = rdflib.Graph().parse(path, format=fmt)
    except Exception as exc:
        raise ArchiveError(f"{path.name}: RDF parse failed: {exc}") from exc
    return graph


def _require_provenance(graph: rdflib.Graph, label: str) -> None:
    prov = rdflib.Namespace("http://www.w3.org/ns/prov#")
    required = (prov.Entity, prov.Activity, prov.Agent)
    missing = [str(item) for item in required if (None, rdflib.RDF.type, item) not in graph]
    if missing:
        raise ArchiveError(f"{label}: missing required PROV types {', '.join(missing)}")


def _require_runtime_provenance(graph: rdflib.Graph, label: str) -> None:
    prov = rdflib.Namespace("http://www.w3.org/ns/prov#")
    checks = {
        "generated entity": (None, prov.wasGeneratedBy, None),
        "associated activity": (None, prov.wasAssociatedWith, None),
        "used entity": (None, prov.used, None),
    }
    missing = [name for name, triple in checks.items() if triple not in graph]
    if missing:
        raise ArchiveError(f"{label}: missing runtime provenance relationship(s): {', '.join(missing)}")


def _require_rec_provenance(graph: rdflib.Graph, label: str) -> None:
    prov = rdflib.Namespace("http://www.w3.org/ns/prov#")
    bdd = rdflib.Namespace("https://secorolab.github.io/metamodels/acceptance-criteria/bdd#")
    obs = rdflib.Namespace("https://secorolab.github.io/metamodels/observation#")
    checks = {
        "BDD execution activity": (None, rdflib.RDF.type, bdd.SimulatedExecution),
        "observation provider agent": (None, rdflib.RDF.type, obs.ObservationProvider),
        "generated artifact entity": (None, prov.wasGeneratedBy, None),
        "activity-agent association": (None, prov.wasAssociatedWith, None),
        "activity resource usage": (None, prov.used, None),
    }
    missing = [name for name, triple in checks.items() if triple not in graph]
    if missing:
        raise ArchiveError(f"{label}: missing REC provenance relationship(s): {', '.join(missing)}")


def _write_rec_snapshot(
    run_dir: Path, manifest: dict, schema: dict, *, complete_lifecycle: bool = True
) -> None:
    try:
        _ensure_local_rec_importable()
        from rec import Run
        from rec.observers import FileObserver
    except ImportError as exc:
        raise ArchiveError(
            "REC is required to create introspection archives. Install the sibling "
            "package with `pip install -e /home/batsy/work/ms/src/rec` or install "
            "`motion-spec[introspection]` once REC is published."
        ) from exc

    run_id = manifest["run_id"]
    observer = FileObserver(run_dir / "rec.json", run_id=run_id)
    run = Run(observers=[observer], run_id=run_id)
    lifecycle = observer.snapshot.get("run", {})
    started_time = lifecycle.get("started_time")
    completed_time = lifecycle.get("completed_time")
    terminal_status = lifecycle.get("status") in {
        "COMPLETED",
        "FAILED",
        "INTERRUPTED",
        "CANCELLED",
        "TIMED_OUT",
        "DEAD",
    }
    if started_time:
        run._id = run_id
        run.start_time = _parse_rec_time(started_time)
    else:
        run._emit_started()
    run.log_host_info(_host_info())
    run.log_repositories(_repositories(run_dir))
    run.log_dependencies(_dependencies())
    _record_agents(run, schema)
    _record_activities(run, schema)
    _record_files(run, run_dir, manifest, schema)
    run.log_scalar("archive_artifact_count", len(manifest.get("artifacts", {})), step=0)
    if complete_lifecycle and not completed_time and not terminal_status:
        if run.start_time is None:
            run.start_time = datetime.now(timezone.utc)
        run._emit_completed()
    observer.close()


def _parse_rec_time(value: str) -> datetime:
    parsed = datetime.fromisoformat(value)
    if parsed.tzinfo is None:
        return parsed.replace(tzinfo=timezone.utc)
    return parsed


def _record_agents(run, schema: dict) -> None:
    runtime = schema.get("runtime_provenance", {})
    runtime_agent = runtime.get("runtime_agent_id") or "agent:runtime"
    runtime_type = "rt:MuJoCoRuntime" if str(runtime_agent).endswith(":mujoco") else "prov:SoftwareAgent"
    run.add_agent(runtime_agent, ["prov:SoftwareAgent", runtime_type], role="runtime")
    run.add_agent(
        runtime.get("producer_agent_id") or "agent:controller_process",
        ["prov:SoftwareAgent", "obs:ObservationProvider"],
        role="log_producer",
        actedOnBehalfOf=runtime_agent,
    )
    run.add_agent(
        "agent:motion_spec_archive",
        ["prov:SoftwareAgent", "obs:ObservationProvider"],
        role="archive_writer",
    )
    for agent in _provenance_nodes(schema, "agn:ModelledAgent"):
        run.add_agent(
            agent.get("@id", "agent:modelled"),
            agent.get("@type", ["prov:Agent", "agn:ModelledAgent"]),
            role=agent.get("role", "modelled_agent"),
        )


def _ensure_local_rec_importable() -> None:
    try:
        import rec  # noqa: F401

        return
    except ImportError:
        pass
    rec_root = _local_rec_root()
    if rec_root:
        sys.path.insert(0, str(rec_root))


def _record_activities(run, schema: dict) -> None:
    runtime = schema.get("runtime_provenance", {})
    run.add_activity(
        runtime.get("activity_id") or "activity:controller_execution",
        ["prov:Activity", "bdd:SimulatedExecution"],
        role="controller_execution",
        wasAssociatedWith=runtime.get("producer_agent_id") or "agent:controller_process",
    )
    run.add_activity(
        "activity:archive_creation",
        ["prov:Activity"],
        role="archive_creation",
        wasAssociatedWith="agent:motion_spec_archive",
    )


def _record_files(run, run_dir: Path, manifest: dict, schema: dict) -> None:
    resource_roles = {"schema", "frame_layout", "provenance", "model", "ir"}
    runtime_activity = (
        schema.get("runtime_provenance", {}).get("activity_id") or "activity:controller_execution"
    )
    for rel, meta in sorted(manifest.get("artifacts", {}).items()):
        path = run_dir / rel
        if not path.exists():
            continue
        row = {
            "path": rel,
            "role": meta.get("role"),
            "sha256": meta.get("sha256"),
            "size_bytes": _artifact_size(path),
        }
        if meta.get("role") in resource_roles:
            run.add_resource(rel, **row)
        else:
            gen_activity = runtime_activity if meta.get("role") == "frame_log" else "activity:archive_creation"
            run.add_artefact(rel, gen_activity=gen_activity, **row)
    for stream in manifest.get("streams", []):
        row = {
            "path": stream.get("url"),
            "role": f"stream_{stream.get('kind', 'unknown')}",
            "id": stream.get("id"),
            "label": stream.get("label"),
        }
        if stream.get("mode") in {"mp4", "file"}:
            run.add_artefact(stream.get("url"), **row)
        else:
            run.add_resource(stream.get("url"), **row)


def _artifact_size(path: Path) -> int:
    if path.is_file():
        return path.stat().st_size
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def _host_info() -> dict:
    return {
        "hostname": socket.gethostname(),
        "os": platform.platform(),
        "python": sys.version,
        "executable": sys.executable,
    }


def _dependencies() -> list[dict]:
    rows = []
    for name in ("motion_spec", "rec", "rdflib", "pyshacl"):
        try:
            rows.append({"name": name, "hasVersion": importlib.metadata.version(name)})
        except importlib.metadata.PackageNotFoundError:
            continue
    return rows


def _repositories(run_dir: Path) -> list[dict]:
    rows = []
    for path in (run_dir, Path(__file__).resolve()):
        root = _git_root(path)
        if root and str(root) not in {row.get("path") for row in rows}:
            rows.append({"name": root.name, "path": str(root), "commit": _git(root, "rev-parse", "HEAD")})
    rec_root = _local_rec_root()
    if rec_root and str(rec_root) not in {row.get("path") for row in rows}:
        rows.append({"name": "rec", "path": str(rec_root), "commit": _git(rec_root, "rev-parse", "HEAD")})
    return rows


def _local_rec_root() -> Path | None:
    for root in (Path.cwd(), *Path.cwd().parents, *Path(__file__).resolve().parents):
        candidate = root / "src" / "rec"
        if (candidate / "rec" / "__init__.py").exists():
            return candidate
    return None


def _git_root(path: Path) -> Path | None:
    start = path if path.is_dir() else path.parent
    root = _git(start, "rev-parse", "--show-toplevel")
    return Path(root) if root else None


def _git(cwd: Path, *args: str) -> str | None:
    try:
        return subprocess.check_output(["git", *args], cwd=cwd, text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return None


def _provenance_nodes(schema: dict, type_id: str) -> list[dict]:
    path = Path(schema.get("output_dir", "")) / "provenance.jsonld"
    if not path.exists():
        return []
    try:
        nodes = json.loads(path.read_text()).get("@graph", [])
    except Exception:
        return []
    return [
        node
        for node in nodes
        if type_id in (node.get("@type") if isinstance(node.get("@type"), list) else [node.get("@type")])
    ]


def _metamodels_root() -> Path:
    for root in (Path.cwd(), *Path.cwd().parents):
        candidate = root / "src" / "metamodels"
        if (candidate / "prov.shacl.ttl").exists():
            return candidate
    for root in Path(__file__).resolve().parents:
        candidate = root / "metamodels"
        if (candidate / "prov.shacl.ttl").exists():
            return candidate
    raise ArchiveError("could not locate src/metamodels/prov.shacl.ttl")


def _validate_prov_shacl(path: Path) -> None:
    root = _metamodels_root()
    conforms, _graph, text = validate(
        data_graph=str(path),
        shacl_graph=str(root / "prov.shacl.ttl"),
        data_graph_format="json-ld",
        shacl_graph_format="turtle",
        inference="rdfs",
    )
    if not conforms:
        raise ArchiveError(f"{path.name}: PROV SHACL validation failed: {text}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--source-dir", default=None)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--frame-log", default=None)
    parser.add_argument("--log-producer-executable", default=None)
    parser.add_argument("--rec", default=None)
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.verify:
            verify_manifest(args.run_dir)
            print("archive OK")
        else:
            create_archive_manifest(
                args.run_dir,
                source_dir=args.source_dir,
                run_id=args.run_id,
                frame_log=args.frame_log,
                log_producer_executable=args.log_producer_executable,
                rec=args.rec,
            )
            print(Path(args.run_dir) / "manifest.json")
    except ArchiveError as exc:
        parser.exit(2, f"{exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
