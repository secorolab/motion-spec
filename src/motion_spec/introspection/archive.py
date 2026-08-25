# SPDX-License-Identifier: MPL-2.0
"""Self-contained run archive manifest helpers."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import urllib.parse
from datetime import datetime, timezone
from pathlib import Path

import rdflib
from motion_spec_dsl.rdf_parser.manifest import build_url_map, metamodel_url_map, metamodels_root
from motion_spec_dsl.rdf_parser.vocab import APP
from pyshacl import validate

from motion_spec.introspection.provenance import (
    artifact_sha256,
    dependencies,
    ensure_local_rec_importable,
    host_info,
    parse_rec_time,
    rec_run_lifecycle,
    record_activities,
    record_agents,
    record_files,
    record_frame_log_health,
    repositories,
)

PROV = rdflib.Namespace("http://www.w3.org/ns/prov#")

MANIFEST_VERSION = 1
GENERATED_BUNDLE_FILES = (
    "CMakeLists.txt",
    "main.cpp",
    "frame_layout.h",
    "introspection_runtime.hpp",
    "introspect_model.hpp",
    "fsm_ir.json",
    "headers",
)


class ArchiveError(ValueError):
    """Archive verification failed."""


def sha256_file(path: Path) -> str:
    return artifact_sha256(path)


def _copy_file(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def _existing(run_dir: Path, rel: str) -> str | None:
    """A manifest entry for a file written outside the manifest builder, or None."""
    return rel if (run_dir / rel).is_file() else None


def _parse_jsonld(path: Path) -> rdflib.Dataset:
    """Parse JSON-LD with the workspace resolver, without network fallback."""
    from rdf_utils.resolver import IriToFileResolver, install_resolver

    install_resolver(IriToFileResolver(metamodel_url_map(), download=False))
    return rdflib.Dataset().parse(path, format="json-ld")


def _mapped_iri_path(iri: str, url_map: dict[str, str]) -> Path | None:
    """Resolve an IRI through the same prefix map used by rdf-utils."""
    for prefix, directory in sorted(url_map.items(), key=lambda item: len(item[0]), reverse=True):
        if iri.startswith(prefix):
            suffix = urllib.parse.unquote(urllib.parse.urlsplit(iri.removeprefix(prefix)).path)
            return Path(directory) / suffix
    return None


def _manifest_imports(manifest_path: Path) -> list[Path]:
    """Resolve app:import entities in a JSON-LD manifest to local graph files."""
    dataset = _parse_jsonld(manifest_path)
    url_map = build_url_map(dataset, manifest_path)
    imports = {
        path
        for _, _, iri, _ in dataset.quads((None, APP["import"], None, None))
        if (path := _mapped_iri_path(str(iri), url_map)) is not None
    }
    return sorted(imports)


def _file_uri_to_path(value: str) -> Path | None:
    """Resolve a ``file://`` atLocation to a local path (None for other IRIs)."""
    if not isinstance(value, str) or not value.startswith("file://"):
        return None
    parsed = urllib.parse.urlparse(value)
    return Path(urllib.parse.unquote(parsed.path))


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
        graph = _parse_jsonld(doc)
        for _, _, location, _ in graph.quads((None, PROV.atLocation, None, None)):
            path = _file_uri_to_path(str(location))
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
    if doc.name.endswith(".ld.json"):
        dataset = _parse_jsonld(doc)
        changed = False
        for subject, predicate, location, context in list(
            dataset.quads((None, PROV.atLocation, None, None))
        ):
            path = _file_uri_to_path(str(location))
            rel = location_map.get(str(path.resolve())) if path else None
            if rel is None:
                continue
            graph = dataset.graph(context)
            graph.remove((subject, predicate, location))
            graph.add((subject, predicate, rdflib.URIRef(os.path.relpath(rel, doc.parent.name))))
            changed = True
        if changed:
            dataset.serialize(doc, format="json-ld", indent=2)
        return

    data = json.loads(doc.read_text())
    rewritten = _relativize_paths(data, location_map, doc.parent.name)
    if rewritten != data:
        doc.write_text(json.dumps(rewritten, indent=2) + "\n")


def _rewrite_model_imports(model_manifest: Path, imports: list[str]) -> None:
    """Point the archived app manifest at the vendored graphs (archive-root-relative).

    Imports become root-relative archive paths and the model-root IRI maps to '..' (the
    archive root, one up from model/), so each import resolves to its single archived
    copy — no duplicate dsl.ld.json, no path into the build tree.
    """
    if not model_manifest.is_file() or not imports:
        return
    dataset = _parse_jsonld(model_manifest)
    import_quads = list(dataset.quads((None, APP["import"], None, None)))
    iri_map_quads = list(dataset.quads((None, APP["iri-map"], None, None)))
    for subject, predicate, value, context in import_quads:
        dataset.graph(context).remove((subject, predicate, value))
    for subject, _, _, context in import_quads:
        graph = dataset.graph(context)
        base = str(iri_map_quads[0][2])
        for rel in imports:
            graph.add((subject, APP["import"], rdflib.URIRef(urllib.parse.urljoin(base, rel))))
    for _, _, iri, context in iri_map_quads:
        graph = dataset.graph(context)
        graph.set((iri, APP.path, rdflib.Literal("..")))
    dataset.serialize(model_manifest, format="json-ld", indent=2)


def _create_generation_run_manifest(
    run_dir: Path,
    generated: Path,
    *,
    run_id: str | None,
    frame_log: Path | str | None,
    log_producer_executable: Path | str | None,
    rec: Path | str | None,
    complete_rec: bool,
    recorded: bool = True,
) -> dict:
    """Catalog a run while referencing immutable artifacts owned by its generation."""
    run_dir.mkdir(parents=True, exist_ok=True)
    proto_path = generated / "contract" / "frame_log.proto"
    provenance_path = generated / "provenance" / "motion-spec.ld.json"
    model_manifests = list((generated / "model").glob("*-app.ld.json"))
    for required in (proto_path, provenance_path, generated / "model" / "ir.json"):
        if not required.is_file():
            raise ArchiveError(f"{required}: required generated artifact is missing")
    if len(model_manifests) != 1:
        raise ArchiveError(f"{generated / 'model'}: expected exactly one application manifest")

    def relative(path: Path) -> str:
        return os.path.relpath(path, run_dir)

    if recorded:
        frame_log_path = Path(frame_log) if frame_log else run_dir / "logs" / "frame_log.pb"
        if (
            frame_log_path.exists()
            and frame_log_path.resolve() != (run_dir / "logs/frame_log.pb").resolve()
        ):
            _copy_file(frame_log_path, run_dir / "logs/frame_log.pb")
        frame_log_path = run_dir / "logs" / "frame_log.pb"
        # The run states its own contract; nothing is read back out of a generated file.
        from motion_spec.introspection import frame_log_pb

        schema = frame_log_pb.read_contract(frame_log_path).summary()
    else:
        # No log to state the contract, so read the one it would have carried -- the same
        # frame_layout.json the runner records the run from before any frame exists.
        schema = json.loads((generated / "contract" / "frame_layout.json").read_text())
    # The authored source travels with the run: it is kilobytes next to a gigabyte log, and a run
    # moved out of its generation still shows the lines its constraints were written on.
    sources = []
    for authored in sorted((generated / "source").glob("*")):
        if authored.is_file():
            _copy_file(authored, run_dir / "source" / authored.name)
            sources.append(f"source/{authored.name}")
    files = {
        "frame_log_proto": relative(proto_path),
        "provenance": relative(provenance_path),
        "dsl_provenance": (
            relative(generated / "provenance" / "dsl.ld.json")
            if (generated / "provenance" / "dsl.ld.json").is_file()
            else None
        ),
        "model": relative(model_manifests[0]),
        # Without it the log's derived slot IRIs resolve to nothing.
        "derived": next(
            (relative(path) for path in (generated / "model").glob("*-derived.ld.json")), None
        ),
        "ir": relative(generated / "model" / "ir.json"),
        "sources": sources or None,
        "controller": relative(generated / "controller"),
        "log_producer_executable": (
            relative(Path(log_producer_executable)) if log_producer_executable else None
        ),
        "frame_log": "logs/frame_log.pb" if recorded else None,
        "frame_log_health": "logs/frame_log.pb.health.json" if recorded else None,
        # Listed only when written: a manifest never promises a file the run dir lacks.
        "runtime_ttl": _existing(run_dir, "runtime/runtime.ttl"),
        "console": _existing(run_dir, "logs/console.log"),
        "rec": "rec.ld.json",
    }
    files = {key: value for key, value in files.items() if value is not None}
    manifest = {
        "manifest_version": MANIFEST_VERSION,
        "run_id": run_id or run_dir.name,
        "files": files,
    }
    # The single marker for "no frame log by choice": replay, live plots and verify all read it
    # rather than guessing from a file that is merely absent.
    if not recorded:
        manifest["recorded"] = False
    if rec and Path(rec).exists():
        _copy_file(Path(rec), run_dir / "rec.ld.json")
    else:
        _write_rec_snapshot(run_dir, manifest, schema, complete_lifecycle=complete_rec)
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=4) + "\n")
    return manifest


def _compress_frame_log(run_dir: Path, rel: str, location_map: dict[str, str]) -> str:
    """Pack the archived frame log with zstd, and say where it went.

    Frames are numbers, one row per control cycle, and rows next to each other say nearly the
    same thing -- which is why this is worth about three to one. Streamed rather than read
    whole: a long run's log is bigger than it needs to be held in memory to shrink.

    Left alone if zstandard is not installed, and the archive is still a valid archive: the
    log is named by the manifest, and both names are ones a reader accepts.
    """
    try:
        import zstandard
    except ImportError:
        return rel
    # Locally, as everywhere else here: frame_log_pb reads ArchiveError back out of this module.
    from motion_spec.introspection import frame_log_pb

    source = run_dir / rel
    if not source.is_file():
        return rel
    packed = source.with_name(source.name + frame_log_pb.LOG_SUFFIX)
    with source.open("rb") as raw, packed.open("wb") as out:
        zstandard.ZstdCompressor(level=COMPRESSION_LEVEL).copy_stream(raw, out)
    source.unlink()
    packed_rel = f"{rel}{frame_log_pb.LOG_SUFFIX}"
    # Provenance points at where each artifact landed; this one landed somewhere else.
    for key, value in list(location_map.items()):
        if value == rel:
            location_map[key] = packed_rel
    return packed_rel


# Measured on a real log: every level from 3 to 15 lands on the same 2.3x, and only 19 finds
# more (2.9x) by searching harder -- thirteen times the time, and it does not thread. So this
# sits in the range that is effectively free rather than paying half a minute at the end of
# every run for a quarter more.
COMPRESSION_LEVEL = 10


def create_archive_manifest(
    run_dir: Path | str,
    *,
    source_dir: Path | str | None = None,
    run_id: str | None = None,
    frame_log: Path | str | None = None,
    log_producer_executable: Path | str | None = None,
    rec: Path | str | None = None,
    complete_rec: bool = True,
    recorded: bool = True,
) -> dict:
    """Create or refresh a local replay manifest for a run folder."""
    run_dir = Path(run_dir)
    source_dir = Path(source_dir) if source_dir else run_dir
    if (source_dir / "contract" / "frame_log.proto").is_file():
        return _create_generation_run_manifest(
            run_dir,
            source_dir,
            run_id=run_id,
            frame_log=frame_log,
            log_producer_executable=log_producer_executable,
            rec=rec,
            complete_rec=complete_rec,
            recorded=recorded,
        )
    run_dir.mkdir(parents=True, exist_ok=True)

    # Contract inputs the archive cannot be self-explanatory without. Fail fast at the source
    # rather than emit an archive that only trips verify_manifest later.
    for required in ("frame_log.proto",):
        if not (source_dir / required).is_file():
            raise ArchiveError(f"{source_dir / required}: required generated artifact is missing")

    copies = {
        "frame_log.proto": "contract/frame_log.proto",
        "provenance.ld.json": "provenance/motion-spec.ld.json",
        "provenance/dsl.ld.json": "provenance/dsl.ld.json",
    }
    frame_log_rel = "logs/frame_log.pb"
    frame_log_health_rel = "logs/frame_log.pb.health.json"
    if not recorded:
        # No log to state the contract: read the layout the run would have written.
        schema = json.loads((source_dir / "frame_layout.json").read_text())
    else:
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
        from motion_spec.introspection import frame_log_pb

        schema = frame_log_pb.read_contract(
            source_dir / "frame_log.pb"
            if (source_dir / "frame_log.pb").exists()
            else run_dir / frame_log_rel
        ).summary()
    # graph/ir_path are portable basenames; resolve them against source_dir.
    model_source = source_dir / "model.ld.json"
    if (source_dir / "model.ld.json").exists():
        copies["model.ld.json"] = "model/model.ld.json"
    if (source_dir / "ir.json").exists():
        copies["ir.json"] = "model/ir.json"
    # Without it the archived log's derived slot IRIs resolve to nothing.
    for derived in source_dir.glob("*-derived.ld.json"):
        copies[derived.name] = "model/derived.ld.json"

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
    # another role (e.g. the dsl provenance -> provenance/dsl.ld.json) are reused in place
    # rather than duplicated; the manifest's import list is rewritten to the single copies.
    model_manifest = run_dir / "model" / "model.ld.json"
    model_imports: list[str] = []
    if model_manifest.exists():
        for src in _manifest_imports(model_source):
            key = str(src.resolve())
            if key in location_map:
                model_imports.append(location_map[key])
                continue
            dst_rel = f"model/{src.relative_to(source_dir)}"
            dst = run_dir / dst_rel
            if src.is_file() and src.resolve() != dst.resolve():
                _copy_file(src, dst)
            if dst.is_file():
                _register(src, dst_rel)
                model_imports.append(dst_rel)

    # The run wrote its log uncompressed, at the speed the control loop produced it. Nothing
    # is appending to it now, so this is where it stops costing what a live file has to cost.
    if recorded:
        frame_log_rel = _compress_frame_log(run_dir, frame_log_rel, location_map)

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

    # Vendor the DSL's authored source inputs (.robmot/.fsm — referenced by dsl.ld.json and
    # still outside the archive) so the model provenance is self-contained. Third-party /
    # vendor assets the model merely points at (e.g. MuJoCo scene xml from the menagerie
    # submodule) are left as references, not copied in. Then rewrite every archived file
    # reference — and the model manifest's import/iri-map — to resolve inside the bundle.
    source_artifacts = _archive_referenced_sources(
        [run_dir / "provenance" / "dsl.ld.json"], run_dir, location_map
    )
    for doc in (
        run_dir / "provenance" / "motion-spec.ld.json",
        run_dir / "provenance" / "dsl.ld.json",
        run_dir / "model" / "ir.json",
    ):
        _rewrite_archived_locations(doc, location_map)
    _rewrite_model_imports(model_manifest, model_imports)

    files = {
        "frame_log_proto": "contract/frame_log.proto",
        "provenance": "provenance/motion-spec.ld.json",
        "dsl_provenance": (
            "provenance/dsl.ld.json" if (run_dir / "provenance" / "dsl.ld.json").exists() else None
        ),
        # Listed only when written: a manifest never promises a file the run dir lacks.
        "runtime_ttl": _existing(run_dir, "runtime/runtime.ttl"),
        "console": _existing(run_dir, "logs/console.log"),
        "frame_log": frame_log_rel if recorded else None,
        "frame_log_health": frame_log_health_rel if recorded else None,
        "model": "model/model.ld.json",
        "model_imports": model_imports or None,
        "sources": source_artifacts or None,
        "ir": "model/ir.json",
        "controller": "controller/source",
        "log_producer_executable": (
            f"controller/executable/{Path(log_producer_executable).name}"
            if log_producer_executable
            else None
        ),
        "rec": "rec.ld.json",
    }
    manifest = {
        "manifest_version": MANIFEST_VERSION,
        "run_id": run_id or run_dir.name,
        "files": {key: value for key, value in files.items() if value is not None},
    }
    if not recorded:
        manifest["recorded"] = False
    if rec and Path(rec).exists():
        _copy_file(Path(rec), run_dir / "rec.ld.json")
    else:
        _write_rec_snapshot(run_dir, manifest, schema, complete_lifecycle=complete_rec)
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
    files = manifest.get("files", {})
    # `recorded: false` is the run stating it kept no frame log; only that opt-in marker excuses
    # it. A manifest that merely lost its log still fails here.
    recorded = manifest.get("recorded") is not False
    required = (
        ("frame_log_proto", "provenance", "frame_log", "rec")
        if recorded
        else ("frame_log_proto", "provenance", "rec")
    )
    for key in required:
        if not files.get(key):
            errors.append(f"files.{key}: missing")
    for key, value in files.items():
        for rel in value if isinstance(value, list) else [value]:
            if (
                key in {"frame_log_health", "runtime_ttl", "console"}
                and not (run_dir / rel).exists()
            ):
                continue
            if not (run_dir / rel).exists():
                errors.append(f"{rel}: missing")
    if errors:
        raise ArchiveError("; ".join(errors))
    provenance_graph = _parse_rdf(run_dir / manifest["files"]["provenance"], "json-ld")
    _require_provenance(provenance_graph, "provenance.ld.json")
    _validate_prov_shacl(run_dir / manifest["files"]["provenance"])
    dsl_provenance_rel = manifest.get("files", {}).get("dsl_provenance")
    if dsl_provenance_rel and (run_dir / dsl_provenance_rel).exists():
        dsl_provenance_graph = _parse_rdf(run_dir / dsl_provenance_rel, "json-ld")
        _require_provenance(dsl_provenance_graph, "dsl.ld.json")
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
        rec = rdflib.Namespace("https://secorolab.github.io/metamodels/rec#")
        for entity, expected in rec_graph.subject_objects(rec.sha256):
            location = rec_graph.value(entity, PROV.atLocation)
            rel = rec_graph.value(location, rec.path) if location else None
            path = run_dir / str(rel) if rel else None
            if not path or not path.exists():
                errors.append(f"{rel or entity}: missing")
            elif artifact_sha256(path) != str(expected):
                errors.append(f"{rel}: sha256 mismatch")
        if errors:
            raise ArchiveError("; ".join(errors))
        simulated = None
        if recorded:
            from motion_spec.introspection import frame_log_pb

            run_schema = frame_log_pb.read_contract(run_dir / files["frame_log"]).summary()
            simulated = (run_schema.get("platform") or {}).get("simulated")
        _require_rec_provenance(rec_graph, "rec.ld.json", simulated=simulated)
        _validate_prov_shacl(run_dir / rec_rel)
        _validate_rec_shacl(run_dir / rec_rel)
    return manifest


def _verify_model_imports(model_path: Path) -> None:
    """Assert every app:import in the archived model resolves offline within the bundle.

    Metamodel prefixes resolve through the local checkout; the model's own iri-map
    resolves its imported graphs to model/'s directory (where they were vendored).
    A dangling import — the graph left behind in the build tree — fails here.
    """
    from motion_spec_dsl.rdf_parser.vocab import APP

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
            dict(sorted(url_map.items(), key=lambda x: len(x[0]), reverse=True)), download=False
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
        raise ArchiveError(
            f"{label}: missing runtime provenance relationship(s): {', '.join(missing)}"
        )


def _require_rec_provenance(graph: rdflib.Graph, label: str, simulated: bool | None = None) -> None:
    prov = rdflib.Namespace("http://www.w3.org/ns/prov#")
    bdd = rdflib.Namespace("https://secorolab.github.io/metamodels/acceptance-criteria/bdd#")
    obs = rdflib.Namespace("https://secorolab.github.io/metamodels/observation#")
    # A real-hardware run is a ScenarioExecution. Requiring SimulatedExecution unconditionally
    # would make the archive enforce a claim the model never made. None means the archive predates
    # the platform record: assert that *an* execution activity is typed, not which kind.
    checks = {
        "observation provider agent": (None, rdflib.RDF.type, obs.ObservationProvider),
        "generated artifact entity": (None, prov.wasGeneratedBy, None),
        "activity-agent association": (None, prov.wasAssociatedWith, None),
        "activity resource usage": (None, prov.used, None),
    }
    if simulated is not None:
        checks["BDD execution activity"] = (
            None,
            rdflib.RDF.type,
            bdd.SimulatedExecution if simulated else bdd.ScenarioExecution,
        )
    elif (None, rdflib.RDF.type, bdd.SimulatedExecution) not in graph and (
        None,
        rdflib.RDF.type,
        bdd.ScenarioExecution,
    ) not in graph:
        raise ArchiveError(
            f"{label}: missing REC provenance relationship(s): BDD execution activity"
        )
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
    observer = FileObserver(run_dir / "rec.ld.json")
    run = Run(observers=[observer], run_id=run_id)
    lifecycle = rec_run_lifecycle(observer.graph)
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
    if complete_lifecycle and not completed_time and not terminal_status:
        if run.start_time is None:
            run.start_time = datetime.now(timezone.utc)
        run._emit_completed()
    observer.close()


# pyshacl reads a non-absolute source shorter than 140 characters as a filename and anything
# longer as inline RDF, so a relative archive path fails with a parser error once the run
# directory name grows. Always hand it an absolute path.
def _graph_source(path: Path) -> str:
    return str(Path(path).resolve())


def _metamodels_root() -> Path:
    root = metamodels_root()
    if root is None or not (root / "prov.shacl.ttl").exists():
        raise ArchiveError("could not locate metamodels/prov.shacl.ttl")
    return root


def _validate_prov_shacl(path: Path) -> None:
    root = _metamodels_root()
    conforms, _graph, text = validate(
        data_graph=_graph_source(path),
        shacl_graph=_graph_source(root / "prov.shacl.ttl"),
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
        data_graph=_graph_source(path),
        shacl_graph=_graph_source(shape),
        data_graph_format="json-ld",
        shacl_graph_format="turtle",
        inference="rdfs",
    )
    if not conforms:
        raise ArchiveError(f"{path.name}: REC SHACL validation failed: {text}")


def _validate_runtime_shacl(path: Path) -> None:
    root = _metamodels_root()
    # The ms-prov shape covers the run, its motions and their maintenances; the tick rate is a
    # sensors update-rate, so its frequency shape comes from the metamodel that defines it
    # rather than being restated. The W3C prov shape is deliberately not loaded: it requires
    # every prov:used object to be a typed prov:Entity, and design IRIs are not.
    shapes = rdflib.Graph()
    for shape in (root / "motion-spec" / "prov.shacl.ttl", root / "robot" / "sensors.shacl.ttl"):
        if not shape.exists():
            raise ArchiveError(f"{shape}: missing runtime SHACL shape")
        shapes.parse(_graph_source(shape), format="turtle")
    conforms, _graph, text = validate(
        data_graph=_graph_source(path),
        shacl_graph=shapes,
        data_graph_format="turtle",
        inference="rdfs",
    )
    if not conforms:
        raise ArchiveError(f"{path.name}: runtime SHACL validation failed: {text}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="motion-spec archive")
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
