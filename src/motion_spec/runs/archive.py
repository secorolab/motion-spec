# SPDX-License-Identifier: MPL-2.0
"""Self-contained run archive manifest helpers."""

from __future__ import annotations

import json
import logging
import os
import shutil
from datetime import UTC, datetime
from pathlib import Path

import rdflib
from motion_spec_dsl.rdf_parser.manifest import (
    build_url_map,
    install_metamodel_resolver,
    metamodels_root,
)
from pyshacl import validate
from rdflib.namespace import PROV
from rec import State

from motion_spec.runs.provenance import (
    DERIVED_DOCUMENT,
    EXECUTION_DOCUMENT,
    GENERATION_DOCUMENT,
    GRAPH_EXECUTION,
    PROVENANCE_DIR,
    RUN_IRI_BASE,
    artifact_sha256,
    file_path,
    parse_rec_time,
    prov_uri,
    rec_document,
    rec_run_lifecycle_from_file,
    record_execution,
    record_files,
    record_frame_log_health,
    record_run_file,
    record_software,
    write_generation_graph,
)

log = logging.getLogger(__name__)

COMPRESSION_LEVEL = 10
PROV_SHAPES = (("prov.shacl.ttl",), ("prov-extension.shacl.ttl",))


class ArchiveError(ValueError):
    """Archive verification failed."""


def _copy_file(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def _existing(run_dir: Path, rel: str) -> str | None:
    """A manifest entry for a file written outside the manifest builder, or None."""
    return rel if (run_dir / rel).is_file() else None


def _camera_videos(run_dir: Path) -> list[str] | None:
    """The videos a run recorded, whichever side wrote them."""
    videos = [f"logs/{path.name}" for path in (run_dir / "logs").glob("*.mp4")]
    return videos or None


def create_archive_manifest(
    run_dir: Path | str,
    *,
    source_dir: Path | str,
    run_id: str | None = None,
    frame_log: Path | str | None = None,
    log_producer_executable: Path | str | None = None,
    rec: Path | str | None = None,
    complete_rec: bool = True,
    recorded: bool = True,
) -> dict:
    """Catalog a run while referencing immutable artifacts owned by its generation."""
    run_dir, generated = Path(run_dir), Path(source_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    proto_path = generated / "contract" / "frame_log.proto"
    provenance_path = generated / GENERATION_DOCUMENT
    model_manifests = list((generated / "model").glob("*-app.ld.json"))
    for required in (proto_path, provenance_path, generated / "model" / "ir.json"):
        if not required.is_file():
            raise ArchiveError(f"{required}: required generated artifact is missing")
    if len(model_manifests) != 1:
        raise ArchiveError(f"{generated / 'model'}: expected exactly one application manifest")
    if recorded:
        frame_log_path = Path(frame_log) if frame_log else run_dir / "logs" / "frame_log.pb"
        if (
            frame_log_path.exists()
            and frame_log_path.resolve() != (run_dir / "logs/frame_log.pb").resolve()
        ):
            _copy_file(frame_log_path, run_dir / "logs/frame_log.pb")
        frame_log_path = run_dir / "logs" / "frame_log.pb"
        # The run states its own contract; nothing is read back out of a generated file.
        from motion_spec.telemetry import frame_log_pb

        schema = frame_log_pb.read_contract(frame_log_path).summary()
        # Nothing appends to the log any more; neighbouring frames nearly repeat, so it packs.
        if frame_log_path.is_file():
            import zstandard

            packed = frame_log_path.with_name(frame_log_path.name + frame_log_pb.LOG_SUFFIX)
            with frame_log_path.open("rb") as raw, packed.open("wb") as out:
                zstandard.ZstdCompressor(level=COMPRESSION_LEVEL).copy_stream(raw, out)
            frame_log_path.unlink()
    else:
        # No log to state the contract, so read the one it would have carried -- the same
        # frame_layout.json the runner records the run from before any frame exists.
        schema = json.loads((generated / "contract" / "frame_layout.json").read_text())
    # The authored source travels with the run: it is kilobytes next to a gigabyte log, and a run
    # moved out of its generation still shows the lines its constraints were written on.
    sources = []
    for authored in (generated / "source").glob("*"):
        if authored.is_file():
            _copy_file(authored, run_dir / "source" / authored.name)
            sources.append(f"source/{authored.name}")
    files = {
        "frame_log_proto": os.path.relpath(proto_path, run_dir),
        "provenance": os.path.relpath(provenance_path, run_dir),
        "model": os.path.relpath(model_manifests[0], run_dir),
        # Without it the log's derived slot IRIs resolve to nothing.
        "derived": (
            os.path.relpath(generated / DERIVED_DOCUMENT, run_dir)
            if (generated / DERIVED_DOCUMENT).is_file()
            else None
        ),
        "ir": os.path.relpath(generated / "model" / "ir.json", run_dir),
        # The scene and FSM graphs as their tools built them; the IR names their nodes.
        "graphs": [
            os.path.relpath(path, run_dir)
            for pattern in ("*.scenex.ld.json", "*.fsm.ld.json")
            for path in (generated / "model").glob(pattern)
        ]
        or None,
        "sources": sources or None,
        "controller": os.path.relpath(generated / "controller", run_dir),
        "log_producer_executable": (
            os.path.relpath(log_producer_executable, run_dir) if log_producer_executable else None
        ),
        "frame_log": "logs/frame_log.pb.zst" if recorded else None,
        "frame_log_health": "logs/frame_log.pb.health.json" if recorded else None,
        # Listed only when written: a manifest never promises a file the run dir lacks.
        "console": _existing(run_dir, "logs/console.log"),
        "environment": next(
            (f"files/{path.name}" for path in (run_dir / "files").glob("environment*")), None
        ),
        "videos": _camera_videos(run_dir),
        # metadata.yaml is what says rosbag2 closed the bag.
        "bag": "bag" if (run_dir / "bag" / "metadata.yaml").is_file() else None,
        "rec": os.path.relpath(rec_document(run_dir, run_id), run_dir),
    }
    files = {key: value for key, value in files.items() if value is not None}
    manifest = {"run_id": run_id or run_dir.name, "files": files}
    # The single marker for "no frame log by choice": replay, live plots and verify all read it
    # rather than guessing from a file that is merely absent.
    if not recorded:
        manifest["recorded"] = False
    if rec and Path(rec).exists():
        _copy_file(Path(rec), rec_document(run_dir, run_id))
    else:
        _write_rec_snapshot(run_dir, manifest, schema, complete_lifecycle=complete_rec)
    _name_execution_document(run_dir, manifest)
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=4) + "\n")
    return manifest


def _name_execution_document(run_dir: Path, manifest: dict) -> None:
    """The execution document is written by the run or the snapshot, so it is named last."""
    if (run_dir / EXECUTION_DOCUMENT).is_file():
        manifest["files"]["execution"] = EXECUTION_DOCUMENT


def load_manifest(run_dir_or_manifest: Path | str) -> tuple[Path, dict]:
    path = Path(run_dir_or_manifest)
    manifest_path = path if path.name == "manifest.json" else path / "manifest.json"
    if not manifest_path.exists():
        raise ArchiveError(f"{manifest_path}: missing manifest.json")
    return manifest_path.parent, json.loads(manifest_path.read_text())


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
            if key in {"frame_log_health", "console"} and not (run_dir / rel).exists():
                continue
            if not (run_dir / rel).exists():
                errors.append(f"{rel}: missing")
    if errors:
        raise ArchiveError("; ".join(errors))
    provenance_path = run_dir / manifest["files"]["provenance"]
    provenance_graph = _parse_rdf(provenance_path, "json-ld")
    _require_provenance(provenance_graph, provenance_path.name)
    _validate_shacl(provenance_graph, provenance_path.name, *PROV_SHAPES)
    model_rel = manifest.get("files", {}).get("model")
    if model_rel and (run_dir / model_rel).exists():
        _parse_rdf(run_dir / model_rel, "json-ld")
        _verify_model_imports(run_dir / model_rel)
    rec_rel = manifest.get("files", {}).get("rec")
    if rec_rel and (run_dir / rec_rel).exists():
        rec_graph = _parse_rdf(run_dir / rec_rel, "json-ld")
        # The run's files are in the execution document, on the run node rec's record describes.
        recorded_files = rdflib.Graph() + rec_graph
        execution_rel = manifest.get("files", {}).get("execution")
        if execution_rel and (run_dir / execution_rel).exists():
            recorded_files += _parse_rdf(run_dir / execution_rel, "json-ld")
        _verify_checksums(recorded_files, run_dir, errors)
        if errors:
            raise ArchiveError("; ".join(errors))
        _require_rec_provenance(
            rec_graph, recorded_files, rec_rel, prov_uri(f"run:{manifest['run_id']}")
        )
        _validate_shacl(recorded_files, rec_rel, *PROV_SHAPES, ("rec", "rec.shacl.ttl"))
    return manifest


def _verify_checksums(rec_graph: rdflib.Graph, run_dir: Path, errors: list[str]) -> None:
    """Every file rec hashed still hashes to what rec recorded, where rec said it is."""
    spdx = rdflib.Namespace("http://spdx.org/rdf/terms#")
    for entity, checksum in rec_graph.subject_objects(spdx.checksum):
        expected = rec_graph.value(checksum, spdx.checksumValue)
        location = rec_graph.value(entity, PROV.atLocation)
        path = file_path(location) if location else None
        if path is None and location is not None:
            path = run_dir / str(location)
        if path is None or not path.exists():
            errors.append(f"{location or entity}: missing")
        elif expected is not None and artifact_sha256(path) != str(expected):
            errors.append(f"{location}: sha256 mismatch")


def _verify_model_imports(model_path: Path) -> None:
    """Assert every app:import in the archived model resolves offline within the bundle.

    Metamodel prefixes resolve through the local checkout; the model's own iri-map
    resolves its imported graphs to model/'s directory (where they were vendored).
    A dangling import — the graph left behind in the build tree — fails here.
    """
    from motion_spec_dsl.rdf_parser.vocab import APP

    install_metamodel_resolver()
    dataset = rdflib.Dataset()
    try:
        dataset.parse(str(model_path), format="json-ld")
    except Exception as exc:
        raise ArchiveError(f"{model_path.name}: model RDF parse failed: {exc}") from exc
    imports = {str(o) for _, _, o, _ in dataset.quads((None, APP["import"], None, None))}
    if not imports:
        return
    install_metamodel_resolver(build_url_map(dataset, model_path))
    for iri in imports:
        try:
            rdflib.Graph().parse(location=iri, format="json-ld")
        except Exception as exc:
            raise ArchiveError(
                f"{model_path.name}: import '{iri}' does not resolve within the archive: {exc}"
            ) from exc


def _parse_rdf(path: Path, fmt: str) -> rdflib.Graph:
    """One document as a single graph; a named-graph document is read as its union.

    The generation provenance is written as one named graph per tool, and a plain Graph parse
    of it comes back empty -- every triple belongs to a graph.
    """
    install_metamodel_resolver()
    try:
        dataset = rdflib.Dataset(default_union=True).parse(path, format=fmt)
    except Exception as exc:
        raise ArchiveError(f"{path.name}: RDF parse failed: {exc}") from exc
    graph = rdflib.Graph()
    for triple in dataset.triples((None, None, None)):
        graph.add(triple)
    return graph


def _require_provenance(graph: rdflib.Graph, label: str) -> None:
    prov = rdflib.Namespace("http://www.w3.org/ns/prov#")
    required = (prov.Entity, prov.Activity, prov.Agent)
    missing = [str(item) for item in required if (None, rdflib.RDF.type, item) not in graph]
    if missing:
        raise ArchiveError(f"{label}: missing required PROV types {', '.join(missing)}")


def _require_rec_provenance(
    rec_graph: rdflib.Graph, recorded_files: rdflib.Graph, label: str, run_iri: str
) -> None:
    prov_ext = rdflib.Namespace("https://secorolab.github.io/metamodels/prov#")
    run = rdflib.URIRef(run_iri)
    # rec records the run itself; the files it used are recorded beside it.
    checks = {
        "execution": (rec_graph, (run, rdflib.RDF.type, prov_ext.Execution)),
        "activity-agent association": (rec_graph, (run, PROV.wasAssociatedWith, None)),
        "activity resource usage": (recorded_files, (run, PROV.used, None)),
    }
    missing = [name for name, (graph, triple) in checks.items() if triple not in graph]
    if missing:
        raise ArchiveError(f"{label}: missing REC provenance relationship(s): {', '.join(missing)}")


def _write_rec_snapshot(
    run_dir: Path, manifest: dict, schema: dict, *, complete_lifecycle: bool = True
) -> None:
    try:
        from rec.observers.file_observer import FileObserver
        from rec.run import Run, host_info
    except ImportError as exc:
        raise ArchiveError("REC is required to archive a run: `motion-spec setup rec`.") from exc

    run_id = manifest["run_id"]
    observer = FileObserver(run_dir / PROVENANCE_DIR, base=RUN_IRI_BASE)
    run = Run(observers=[observer], run_id=run_id)
    lifecycle = rec_run_lifecycle_from_file(rec_document(run_dir, run_id))
    started_time = lifecycle["started_time"]
    completed_time = lifecycle["completed_time"]
    terminal_status = lifecycle["state"] in (State.COMPLETE, State.CANCELED)
    if started_time:
        run.start_time = parse_rec_time(started_time)
    else:
        run._emit_started()
        # Archiving without a prior catalogued run: the caller names the executable that ran,
        # and nothing knows the command line it was given.
        generation_document = run_dir / manifest["files"].get("provenance", GENERATION_DOCUMENT)
        record_execution(
            run_dir, run_id, generation_document, schema.get("platform") or {}, [], run.start_time
        )
    run.log_host_info(host_info())
    record_software(run, run_dir)
    files = rdflib.Graph()
    executable = manifest.get("files", {}).get("log_producer_executable")
    if executable and (run_dir / executable).exists():
        record_run_file(
            files, run_id, run_dir / executable, "log_producer_executable", generated=False
        )
    record_files(files, run_id, run_dir, manifest)
    write_generation_graph(run_dir / EXECUTION_DOCUMENT, GRAPH_EXECUTION, files)
    record_frame_log_health(run, run_dir, manifest)
    if complete_lifecycle and not completed_time and not terminal_status:
        if run.start_time is None:
            run.start_time = datetime.now(UTC)
        run._emit_completed()
    observer.close()


# pyshacl reads a non-absolute source shorter than 140 characters as a filename and anything
# longer as inline RDF, so a relative archive path fails with a parser error once the run
# directory name grows. Always hand it an absolute path.
def _graph_source(path: Path) -> str:
    return str(Path(path).resolve())


_warned_about_shapes = False


def _shapes_missing() -> None:
    """Say it once, not once per document the archive holds."""
    global _warned_about_shapes

    if _warned_about_shapes:
        return
    _warned_about_shapes = True
    log.warning(
        "no metamodels checkout: the archive's SHACL checks are skipped, everything else is "
        "still checked; set METAMODELS_PATH to run them"
    )


def _metamodels_root() -> Path | None:
    """The metamodels checkout the SHACL shapes live in, or None when there is none.

    A checkout is a dev convenience; the shapes are not part of an installation, so a run is
    archived without these checks rather than failed for want of them.
    """
    root = metamodels_root()
    return root if root is not None and (root / "prov.shacl.ttl").exists() else None


def _validate_shacl(graph: rdflib.Graph, label: str, *shape_names: tuple[str, ...]) -> None:
    root = _metamodels_root()
    if root is None or not all((root.joinpath(*name)).exists() for name in shape_names):
        _shapes_missing()
        return
    shapes = rdflib.Graph()
    for name in shape_names:
        shapes.parse(_graph_source(root.joinpath(*name)), format="turtle")
    conforms, _graph, text = validate(data_graph=graph, shacl_graph=shapes, inference="rdfs")
    if not conforms:
        raise ArchiveError(f"{label}: SHACL validation failed: {text}")
