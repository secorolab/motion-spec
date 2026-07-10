# SPDX-License-Identifier: MPL-2.0
"""Self-contained run archive manifest helpers."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import urllib.parse
from datetime import datetime, timezone
from pathlib import Path

import rdflib
from pyshacl import validate

from motion_spec.provenance import (
    dependencies,
    ensure_local_rec_importable,
    host_info,
    parse_rec_time,
    record_activities,
    record_agents,
    record_files,
    record_frame_log_health,
    repositories,
)
from motion_spec.manifest import build_url_map, metamodel_url_map, metamodels_root

MANIFEST_VERSION = 1
PROVENANCE_DOCUMENT_VERSION = 1
HASHED_ARTIFACTS = {
    "schema": "contract/schema.json",
    "frame_log_proto": "contract/frame_log.proto",
    "provenance": "provenance/codegen.jsonld",
    "dsl_provenance": "provenance/dsl.jsonld",
    "runtime": "runtime/runtime.ttl",
    "frame_log": "logs/frame_log.pb",
    "frame_log_health": "logs/frame_log.pb.health.json",
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


def _manifest_imports(manifest_path: Path) -> list[str]:
    """Relative graph files an app manifest declares via ``app:import``.

    Returns [] for plain provenance/model documents that carry no import list.
    """
    try:
        doc = json.loads(manifest_path.read_text())
    except (OSError, json.JSONDecodeError):
        return []
    imports: list[str] = []
    for node in doc.get("@graph", []) if isinstance(doc, dict) else []:
        if isinstance(node, dict):
            value = node.get("import")
            if isinstance(value, str):
                imports.append(value)
            elif isinstance(value, list):
                imports.extend(str(item) for item in value)
    return list(dict.fromkeys(imports))


def _file_uri_to_path(value: str) -> Path | None:
    """Resolve a ``file://`` atLocation to a local path (None for other IRIs)."""
    if not isinstance(value, str) or not value.startswith("file://"):
        return None
    parsed = urllib.parse.urlparse(value)
    return Path(urllib.parse.unquote(parsed.path))


def _iter_file_uris(obj) -> "list[str]":
    """Every ``file://`` string value anywhere in a JSON structure."""
    found: list[str] = []
    if isinstance(obj, dict):
        for value in obj.values():
            found.extend(_iter_file_uris(value))
    elif isinstance(obj, list):
        for value in obj:
            found.extend(_iter_file_uris(value))
    elif isinstance(obj, str) and obj.startswith("file://"):
        found.append(obj)
    return found


def _archive_referenced_sources(
    prov_docs: list[Path], run_dir: Path, location_map: dict[str, str]
) -> list[str]:
    """Vendor authored source inputs referenced by the given provenance doc(s).

    Pass only the DSL source provenance here: a referenced-but-unmapped file that still
    exists on disk (the .robmot/.fsm models) is copied into ``source/`` and registered so
    the rewrite can point at it. Third-party/vendor assets (MuJoCo scene xml, etc.) are not
    in the DSL source provenance and are intentionally left as references, not archived.
    """
    sources: list[str] = []
    for doc in prov_docs:
        if not doc.is_file():
            continue
        for uri in _iter_file_uris(json.loads(doc.read_text())):
            path = _file_uri_to_path(uri)
            if path is None or not path.is_file():
                continue
            key = str(path.resolve())
            if key in location_map:
                continue
            rel = f"source/{path.name}"
            _copy_file(path, run_dir / rel)
            location_map[key] = rel
            sources.append(rel)
    return sources


def _relativize_paths(obj, location_map: dict[str, str], start: str):
    """Repoint every archived reference (``file://`` URI or absolute path) at its archive
    copy, relative to ``start``. Non-archived references are left untouched."""
    if isinstance(obj, dict):
        return {k: _relativize_paths(v, location_map, start) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_relativize_paths(v, location_map, start) for v in obj]
    if isinstance(obj, str) and (obj.startswith("file://") or obj.startswith("/")):
        path = _file_uri_to_path(obj) if obj.startswith("file://") else Path(obj)
        rel = location_map.get(str(path.resolve())) if path else None
        if rel is not None:
            return os.path.relpath(rel, start)
    return obj


def _rewrite_archived_locations(doc: Path, location_map: dict[str, str]) -> None:
    """Repoint every archived file reference in a JSON(-LD) doc, relative to the doc's dir.

    Covers the provenance graphs' ``atLocation`` (file:// IRIs) and the IR's provenance
    entity ``path``/``source`` (absolute paths) so no build-tree location survives in the
    archived model.
    """
    if not doc.is_file():
        return
    data = json.loads(doc.read_text())
    rewritten = _relativize_paths(data, location_map, doc.parent.name)
    if rewritten != data:
        doc.write_text(json.dumps(rewritten, indent=2) + "\n")


def _rewrite_model_imports(model_manifest: Path, imports: list[str]) -> None:
    """Point the archived app manifest at the vendored graphs (archive-root-relative).

    Imports become root-relative archive paths and the model-root IRI maps to '..' (the
    archive root, one up from model/), so each import resolves to its single archived
    copy — no duplicate dsl.jsonld, no path into the build tree.
    """
    if not model_manifest.is_file() or not imports:
        return
    doc = json.loads(model_manifest.read_text())
    for node in doc.get("@graph", []):
        if isinstance(node, dict) and "import" in node:
            node["import"] = imports
            node["iri-map"] = {"https://secorolab.github.io/": {"path": ".."}}
    model_manifest.write_text(json.dumps(doc, indent=2) + "\n")


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

    # Contract inputs the archive cannot be self-explanatory without. Fail fast at the source
    # rather than emit an archive that only trips verify_manifest later.
    for required in ("schema.json", "frame_log.proto"):
        if not (source_dir / required).is_file():
            raise ArchiveError(f"{source_dir / required}: required generated artifact is missing")

    copies = {
        "schema.json": "contract/schema.json",
        "frame_log.proto": "contract/frame_log.proto",
        "provenance.jsonld": "provenance/codegen.jsonld",
        "provenance/dsl.jsonld": "provenance/dsl.jsonld",
    }
    frame_log_rel = "logs/frame_log.pb"
    frame_log_health_rel = "logs/frame_log.pb.health.json"
    if frame_log and Path(frame_log).exists():
        frame_log_path = Path(frame_log)
        frame_log_rel = f"logs/{frame_log_path.name}"
        frame_log_health_rel = f"logs/{frame_log_path.name}.health.json"
        copies[str(frame_log_path.resolve())] = frame_log_rel
        health = Path(str(frame_log_path) + ".health.json")
        if health.exists():
            copies[str(health.resolve())] = frame_log_health_rel
    elif (source_dir / "frame_log.pb").exists():
        copies["frame_log.pb"] = frame_log_rel
        if (source_dir / "frame_log.pb.health.json").exists():
            copies["frame_log.pb.health.json"] = frame_log_health_rel
    schema = json.loads((source_dir / "schema.json").read_text())
    # schema.graph/ir_path are portable basenames; resolve them against source_dir.
    if (source_dir / "model.jsonld").exists():
        copies["model.jsonld"] = "model/model.jsonld"
    elif schema.get("graph") and (source_dir / Path(schema["graph"]).name).exists():
        copies[Path(schema["graph"]).name] = "model/model.jsonld"
    if schema.get("ir_path") and (source_dir / Path(schema["ir_path"]).name).exists():
        copies[Path(schema["ir_path"]).name] = "model/ir.json"

    # Track where each source artifact lands in the archive so provenance atLocations
    # can be rewritten to point at the archived copy (keyed by resolved source path).
    location_map: dict[str, str] = {}

    def _register(src: Path, rel: str) -> None:
        if src.is_file():
            location_map[str(src.resolve())] = rel

    for src_name, dst_name in copies.items():
        src = Path(src_name) if Path(src_name).is_absolute() else source_dir / src_name
        if src.exists() and src.resolve() != (run_dir / dst_name).resolve():
            _copy_file(src, run_dir / dst_name)
        _register(src, dst_name)

    # Vendor the model graph(s) the app manifest imports. Graphs already archived under
    # another role (e.g. the dsl provenance -> provenance/dsl.jsonld) are reused in place
    # rather than duplicated; the manifest's import list is rewritten to the single copies.
    model_manifest = run_dir / "model" / "model.jsonld"
    model_imports: list[str] = []
    if model_manifest.exists():
        for imp in _manifest_imports(model_manifest):
            src = source_dir / imp
            key = str(src.resolve())
            if key in location_map:
                model_imports.append(location_map[key])
                continue
            dst_rel = f"model/{imp}"
            dst = run_dir / dst_rel
            if src.is_file() and src.resolve() != dst.resolve():
                _copy_file(src, dst)
            if dst.is_file():
                _register(src, dst_rel)
                model_imports.append(dst_rel)

    controller_dir = run_dir / "controller" / "source"
    controller_dir.mkdir(parents=True, exist_ok=True)
    for rel in GENERATED_BUNDLE_FILES:
        src = source_dir / rel
        dst = controller_dir / rel
        if src.is_file():
            _copy_file(src, dst)
            _register(src, f"controller/source/{rel}")
        elif src.is_dir():
            if dst.exists():
                shutil.rmtree(dst)
            shutil.copytree(src, dst)
            for item in sorted(p for p in src.rglob("*") if p.is_file()):
                _register(item, f"controller/source/{rel}/{item.relative_to(src).as_posix()}")
    for header in sorted(source_dir.glob("*_fsm.hpp")):
        _copy_file(header, controller_dir / header.name)
        _register(header, f"controller/source/{header.name}")
    if log_producer_executable and Path(log_producer_executable).exists():
        executable = Path(log_producer_executable)
        _copy_file(executable, run_dir / "controller" / "executable" / executable.name)
        _register(executable, f"controller/executable/{executable.name}")

    # Vendor the DSL's authored source inputs (.robmot/.fsm — referenced by dsl.jsonld and
    # still outside the archive) so the model provenance is self-contained. Third-party /
    # vendor assets the model merely points at (e.g. MuJoCo scene xml from the menagerie
    # submodule) are left as references, not copied in. Then rewrite every archived file
    # reference — and the model manifest's import/iri-map — to resolve inside the bundle.
    source_artifacts = _archive_referenced_sources(
        [run_dir / "provenance" / "dsl.jsonld"], run_dir, location_map
    )
    for doc in (
        run_dir / "provenance" / "codegen.jsonld",
        run_dir / "provenance" / "dsl.jsonld",
        run_dir / "model" / "ir.json",
    ):
        _rewrite_archived_locations(doc, location_map)
    _rewrite_model_imports(model_manifest, model_imports)

    artifacts = {}
    for key, rel in HASHED_ARTIFACTS.items():
        path = run_dir / rel
        if path.is_file():
            artifacts[rel] = {"role": key, "sha256": sha256_file(path)}
        elif path.is_dir():
            artifacts[rel] = {"role": key, "sha256": hash_tree(path)}
    for rel in model_imports:
        path = run_dir / rel
        if rel not in artifacts and path.is_file():
            artifacts[rel] = {"role": "imported_model_graph", "sha256": sha256_file(path)}
    for rel in source_artifacts:
        path = run_dir / rel
        if rel not in artifacts and path.is_file():
            artifacts[rel] = {"role": "source_model", "sha256": sha256_file(path)}
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
            "frame_log_proto": "contract/frame_log.proto",
            "provenance": "provenance/codegen.jsonld",
            "dsl_provenance": (
                "provenance/dsl.jsonld"
                if (run_dir / "provenance" / "dsl.jsonld").exists()
                else None
            ),
            "runtime_ttl": "runtime/runtime.ttl",
            "frame_log": frame_log_rel,
            "frame_log_health": frame_log_health_rel,
            "model": "model/model.jsonld",
            "model_imports": model_imports or None,
            "sources": source_artifacts or None,
            "ir": "model/ir.json",
            "controller": "controller/source",
            "log_producer_executable": (
                f"controller/executable/{Path(log_producer_executable).name}"
                if log_producer_executable
                else None
            ),
            "rec": "rec.jsonld",
        },
        "versions": {
            "manifest": MANIFEST_VERSION,
            "schema": schema.get("schema_version"),
            "runtime_rdf_contract": schema.get("runtime_rdf_contract_version"),
            "provenance_document": PROVENANCE_DOCUMENT_VERSION,
        },
        "contexts": schema.get("provenance_contexts", []),
        "artifacts": artifacts,
        "provenance": {
            "document": "provenance/codegen.jsonld",
            "dsl": (
                "provenance/dsl.jsonld"
                if (run_dir / "provenance" / "dsl.jsonld").exists()
                else None
            ),
            "runtime": "runtime/runtime.ttl",
            "rec": "rec.jsonld",
        },
        "streams": streams or [],
    }
    if rec and Path(rec).exists():
        _copy_file(Path(rec), run_dir / "rec.jsonld")
    else:
        _write_rec_snapshot(run_dir, manifest, schema, complete_lifecycle=complete_rec)
    manifest["artifacts"]["rec.jsonld"] = {"role": "rec", "sha256": sha256_file(run_dir / "rec.jsonld")}
    manifest["rec"] = {"path": "rec.jsonld", "run_id": manifest["run_id"]}
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
    for key in ("schema", "frame_log_proto", "provenance", "frame_log", "model", "ir", "controller", "rec"):
        rel = manifest.get("files", {}).get(key)
        if not rel and key in ("schema", "frame_log_proto", "provenance", "frame_log", "rec"):
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
    dsl_provenance_rel = manifest.get("files", {}).get("dsl_provenance")
    if dsl_provenance_rel and (run_dir / dsl_provenance_rel).exists():
        dsl_provenance_graph = _parse_rdf(run_dir / dsl_provenance_rel, "json-ld")
        _require_provenance(dsl_provenance_graph, "dsl.jsonld")
        _validate_prov_shacl(run_dir / dsl_provenance_rel)
    model_rel = manifest.get("files", {}).get("model")
    if model_rel and (run_dir / model_rel).exists():
        _parse_rdf(run_dir / model_rel, "json-ld")
        _verify_model_imports(run_dir / model_rel)
    runtime_rel = manifest.get("files", {}).get("runtime_ttl")
    if runtime_rel and (run_dir / runtime_rel).exists():
        runtime_graph = _parse_rdf(run_dir / runtime_rel, "turtle")
        _require_runtime_provenance(runtime_graph, "runtime.ttl")
        _validate_runtime_shacl(run_dir / runtime_rel)
    rec_rel = manifest.get("files", {}).get("rec")
    if rec_rel and (run_dir / rec_rel).exists():
        rec_graph = _parse_rdf(run_dir / rec_rel, "json-ld")
        _require_rec_provenance(rec_graph, "rec.jsonld")
        _validate_prov_shacl(run_dir / rec_rel)
        _validate_rec_shacl(run_dir / rec_rel)
    return manifest


def _verify_model_imports(model_path: Path) -> None:
    """Assert every app:import in the archived model resolves offline within the bundle.

    Metamodel prefixes resolve through the local checkout; the model's own iri-map
    resolves its imported graphs to model/'s directory (where they were vendored).
    A dangling import — the graph left behind in the build tree — fails here.
    """
    from motion_spec.namespace import APP

    try:
        from rdf_utils.resolver import IriToFileResolver, install_resolver
    except Exception:
        return
    dataset = rdflib.Dataset()
    install_resolver(IriToFileResolver(metamodel_url_map(), download=False))
    try:
        dataset.parse(str(model_path), format="json-ld")
    except Exception as exc:
        raise ArchiveError(f"{model_path.name}: model RDF parse failed: {exc}") from exc
    imports = {str(o) for _, _, o, _ in dataset.quads((None, APP["import"], None, None))}
    if not imports:
        return
    url_map = {**metamodel_url_map(), **build_url_map(dataset, model_path)}
    install_resolver(
        IriToFileResolver(
            dict(sorted(url_map.items(), key=lambda x: len(x[0]), reverse=True)),
            download=False,
        )
    )
    for iri in sorted(imports):
        try:
            rdflib.Graph().parse(location=iri, format="json-ld")
        except Exception as exc:
            raise ArchiveError(
                f"{model_path.name}: import '{iri}' does not resolve within the archive: {exc}"
            ) from exc


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
        ensure_local_rec_importable()
        from rec import Run
        from rec.observers import FileObserver
    except ImportError as exc:
        raise ArchiveError(
            "REC is required to create introspection archives. Install the sibling "
            "package with `pip install -e /home/batsy/work/ms/src/rec` or install "
            "`motion-spec[introspection]` once REC is published."
        ) from exc

    run_id = manifest["run_id"]
    observer = FileObserver(run_dir / "rec.jsonld", run_id=run_id)
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
        run.start_time = parse_rec_time(started_time)
    else:
        run._emit_started()
    run.log_host_info(host_info())
    run.log_repositories(repositories(run_dir))
    run.log_dependencies(dependencies())
    record_agents(run, run_dir, schema)
    record_activities(run, schema)
    record_files(run, run_dir, manifest, schema)
    record_frame_log_health(run, run_dir, manifest)
    run.log_scalar("archive_artifact_count", len(manifest.get("artifacts", {})), step=0)
    if complete_lifecycle and not completed_time and not terminal_status:
        if run.start_time is None:
            run.start_time = datetime.now(timezone.utc)
        run._emit_completed()
    observer.close()


def _metamodels_root() -> Path:
    root = metamodels_root()
    if root is None or not (root / "prov.shacl.ttl").exists():
        raise ArchiveError("could not locate metamodels/prov.shacl.ttl")
    return root


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


def _validate_rec_shacl(path: Path) -> None:
    root = _metamodels_root()
    shape = root / "rec" / "rec.shacl.ttl"
    if not shape.exists():
        raise ArchiveError(f"{shape}: missing REC SHACL shape")
    conforms, _graph, text = validate(
        data_graph=str(path),
        shacl_graph=str(shape),
        data_graph_format="json-ld",
        shacl_graph_format="turtle",
        inference="rdfs",
    )
    if not conforms:
        raise ArchiveError(f"{path.name}: REC SHACL validation failed: {text}")


def _validate_runtime_shacl(path: Path) -> None:
    root = _metamodels_root()
    shape = root / "motion-spec" / "runtime.shacl.ttl"
    if not shape.exists():
        raise ArchiveError(f"{shape}: missing runtime SHACL shape")
    conforms, _graph, text = validate(
        data_graph=str(path),
        shacl_graph=str(shape),
        data_graph_format="turtle",
        shacl_graph_format="turtle",
        inference="rdfs",
    )
    if not conforms:
        raise ArchiveError(f"{path.name}: runtime SHACL validation failed: {text}")


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
