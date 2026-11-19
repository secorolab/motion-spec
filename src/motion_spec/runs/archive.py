# SPDX-License-Identifier: MPL-2.0
"""Archive a run in its directory and verify it from its provenance."""

from __future__ import annotations

import json
import shutil
from datetime import UTC, datetime
from pathlib import Path

import rdflib
from motion_spec_dsl.rdf_parser.manifest import install_metamodel_resolver
from rdf_utils.constraints import SHACLViolation, check_shacl_constraints
from rdf_utils.models.prov import (
    add_directory_location,
    add_file_hash,
    capture_file_prov,
    get_file_path,
    load_transformation_prov,
)
from rdf_utils.models.vocab import URI_PROV_EXT_TYPE_EXECUTION
from rdf_utils.namespace import URL_MM_PROV_EXT_SHACL, URL_MM_PROV_SHACL, URL_SECORO_MM
from rdflib import Literal, URIRef
from rdflib.namespace import DCTERMS, PROV, RDF
from rec import State, Verdict

from motion_spec.runs.provenance import (
    EXECUTION_DOCUMENT,
    GENERATION_DOCUMENT,
    RUN_IRI_BASE,
    add_package,
    execution_node,
    kept_dsl_document,
    motion_spec_bundle,
    rec_document,
    rec_run,
    record_execution,
    record_files,
    record_run_file,
    run_generation,
    write_generation_graph,
)

COMPRESSION_LEVEL = 10
FRAME_LOG = "logs/frame_log.pb"
PROV_TYPES = (PROV.Entity, PROV.Activity, PROV.Agent)
PROV_SHAPES = {URL_MM_PROV_SHACL: "turtle", URL_MM_PROV_EXT_SHACL: "turtle"}
REC_SHAPES = {**PROV_SHAPES, f"{URL_SECORO_MM}/rec/rec.shacl.ttl": "turtle"}


class ArchiveError(ValueError):
    """A run's archive is incomplete, or disagrees with its provenance or its generation.

    Attributes:
        location: the run directory, document or frame log the problems were found in; None
            when the reader was handed bytes, not a file
        problems: what is wrong there, one entry per failed check
    """

    location: Path | None
    problems: tuple[str, ...]

    def __init__(self, location: Path | str | None, *problems: str):
        super().__init__(("" if location is None else f"{location}: ") + "; ".join(problems))
        self.location = None if location is None else Path(location)
        self.problems = problems


def archive_run(
    run_dir: Path | str,
    *,
    source_dir: Path | str,
    run_id: str | None = None,
    frame_log: Path | str | None = None,
    log_producer_executable: Path | str | None = None,
    rec: Path | str | None = None,
    complete_rec: bool = True,
    recorded: bool = True,
) -> None:
    """Archive a finished run in RUN_DIR, a run of the generation whose `generated/` is SOURCE_DIR.

    Copies FRAME_LOG into `logs/` when it lies elsewhere and packs it with zstd (RECORDED runs),
    then either copies REC's record from REC or writes the snapshot: the execution, unless the run was
    catalogued, and every file the run generated, into the execution document, and the run's
    completion into REC when COMPLETE_REC.

    Raises:
        ArchiveError: the generation lacks its log protocol, its provenance, its IR or the DSL's
            document; or REC is not installed.
    """
    run_dir, generated = Path(run_dir), Path(source_dir)
    run_id = run_id or run_dir.name
    run_dir.mkdir(parents=True, exist_ok=True)
    provenance_path = generated / GENERATION_DOCUMENT
    proto_path = generated / "contract" / "frame_log.proto"
    for required in (proto_path, provenance_path, generated / "model" / "ir.json"):
        if not required.is_file():
            raise ArchiveError(required, "required generated artifact is missing")
    model_document = kept_dsl_document(generated)
    if model_document is None or not model_document.is_file():
        raise ArchiveError(provenance_path, "records no model document the DSL generated")

    if recorded:
        frame_log_path = run_dir / FRAME_LOG
        source = Path(frame_log) if frame_log else frame_log_path
        if source.exists() and source.resolve() != frame_log_path.resolve():
            frame_log_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, frame_log_path)
        # The run states its own contract; nothing is read back out of a generated file.
        from motion_spec.telemetry import frame_log_pb

        schema = frame_log_pb.read_contract(frame_log_path).summary()
        # Nothing appends to the log any more; neighbouring frames nearly repeat, so it packs.
        if frame_log_path.is_file():
            import zstandard

            packed = frame_log_path.with_name(frame_log_path.name + frame_log_pb.LOG_SUFFIX)
            # The run generated the log; packing derives the packed log from it and ends it.
            files = motion_spec_bundle(run_dir / EXECUTION_DOCUMENT)
            generation = run_dir.parents[1]
            raw = record_run_file(
                files, URIRef(f"{RUN_IRI_BASE}{run_id}"), generation, frame_log_path, generated=True
            )
            started = datetime.now(UTC)
            with frame_log_path.open("rb") as raw_log, packed.open("wb") as out:
                zstandard.ZstdCompressor(level=COMPRESSION_LEVEL).copy_stream(raw_log, out)
            frame_log_path.unlink()
            ended = datetime.now(UTC)
            roots = {generation.resolve(): add_directory_location(files, generation)}
            log = capture_file_prov(files, packed, roots=roots)
            packing = load_transformation_prov(
                files, [raw], [log], add_package(files, "motion_spec"), started=started, ended=ended
            )
            files.add((raw, PROV.wasInvalidatedBy, packing))
            files.add((raw, PROV.invalidatedAtTime, Literal(ended)))
            write_generation_graph(run_dir / EXECUTION_DOCUMENT, files)
    else:
        # No log to state the contract, so read the one it would have carried -- the same
        # frame_layout.json the runner records the run from before any frame exists.
        schema = json.loads((generated / "contract" / "frame_layout.json").read_text())

    if rec and Path(rec).exists():
        rec_document(run_dir, run_id).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(rec, rec_document(run_dir, run_id))
    else:
        _write_rec_snapshot(
            run_dir,
            run_id,
            provenance_path,
            Path(log_producer_executable) if log_producer_executable else None,
            schema,
            complete_lifecycle=complete_rec,
        )


def verify_run(run_dir: Path | str) -> None:
    """Check the archived run in RUN_DIR against its provenance, reading only.

    The generation is the one the run's execution document names the provenance of, by its
    location under the generations root. Its log protocol and provenance and REC's record must
    exist; every file the REC
    record and the execution document hash must still be where they say, with that hash; the
    generation's provenance must state an entity, an activity and an agent, and REC must record
    the run as a `prov-ext:Execution` associated with an agent that used something; and the
    generation's provenance, with the DSL's record, must conform to the PROV shapes, and the run's
    records together with it to the REC shapes.

    Raises:
        ArchiveError: naming every failed check of the first stage that failed.
    """
    run_dir = Path(run_dir)
    generation = run_generation(run_dir)
    if generation is None:
        raise ArchiveError(run_dir, "the execution document names no generation's provenance")
    generated = generation / "generated"
    provenance_path = generated / GENERATION_DOCUMENT
    rec_path = rec_document(run_dir)
    required = (generated / "contract" / "frame_log.proto", provenance_path, rec_path)
    errors = [f"{path}: missing" for path in required if not path.is_file()]
    if errors:
        raise ArchiveError(run_dir, *errors)

    install_metamodel_resolver()
    provenance = _parse_rdf(rdflib.Dataset(default_union=True), provenance_path)
    model_document = kept_dsl_document(generated)
    if model_document is not None and model_document.is_file():
        model = _parse_rdf(rdflib.Dataset(), model_document)
        # The generation's record names the DSL's nodes, which only the DSL's own bundle describes.
        for bundle in model.graphs():
            if (None, RDF.type, PROV.Bundle) in bundle:
                provenance.add_graph(bundle)
    rec = _parse_rdf(rdflib.Dataset(default_union=True), rec_path)
    # The run's files are in the execution document, on the run node rec's record describes.
    recorded_files = rdflib.Dataset(default_union=True)
    for graph in rec.graphs():
        recorded_files.add_graph(graph)
    execution = rdflib.Dataset(default_union=True)
    if (run_dir / EXECUTION_DOCUMENT).is_file():
        _parse_rdf(execution, run_dir / EXECUTION_DOCUMENT)
        for graph in execution.graphs():
            recorded_files.add_graph(graph)

    # Each document's graph is read alone, so a file is checked at the place that document
    # states, against the hash it recorded; rdf-utils hashes it again into a copy without them.
    for graph in recorded_files.graphs():
        recorded = dict(graph.subject_objects(DCTERMS.identifier))
        rehashed = rdflib.Graph()
        rehashed += (triple for triple in graph if triple[1] != DCTERMS.identifier)
        for entity, digest in recorded.items():
            path = get_file_path(graph, entity)
            # An invalidated file, the raw frame log, is gone by the time of the record.
            if (entity, PROV.wasInvalidatedBy, None) in graph or path is None:
                continue
            if not path.exists():
                errors.append(f"{path}: missing")
                continue
            add_file_hash(rehashed, entity)
            if rehashed.value(entity, DCTERMS.identifier) != digest:
                errors.append(f"{path}: content differs from the recorded one")
    if errors:
        raise ArchiveError(run_dir, *errors)

    run = execution_node(rec)
    if run is None:
        raise ArchiveError(rec_path, "records no prov-ext:Execution")
    checks = [
        *((provenance_path.name, provenance, (None, RDF.type, kind)) for kind in PROV_TYPES),
        # rec records the run itself; the execution document, the files it used, on that node.
        (rec_path.name, rec, (run, RDF.type, URI_PROV_EXT_TYPE_EXECUTION)),
        (rec_path.name, rec, (run, PROV.wasAssociatedWith, None)),
        (EXECUTION_DOCUMENT, execution, (run, PROV.used, None)),
    ]
    missing = [
        f"{label}: missing {' '.join('?' if term is None else term.n3() for term in triple)}"
        for label, graph, triple in checks
        if triple not in graph
    ]
    if missing:
        raise ArchiveError(run_dir, *missing)

    # What the run used was generated, so the generation's provenance says what each of its nodes
    # is; only that, as the run may describe a package at another version than the generation did.
    mentioned = {node for triple in recorded_files for node in (triple[0], triple[2])}
    recorded_files.default_graph += (
        triple for triple in provenance.triples((None, RDF.type, None)) if triple[0] in mentioned
    )
    for label, graph, shapes in (
        (provenance_path.name, provenance, PROV_SHAPES),
        (rec_path.name, recorded_files, REC_SHAPES),
    ):
        try:
            check_shacl_constraints(graph, shapes)
        except SHACLViolation as exc:
            raise ArchiveError(run_dir, f"{label}: {exc}") from exc


def _parse_rdf(dataset: rdflib.Dataset, path: Path) -> rdflib.Dataset:
    """Parse the document at PATH into DATASET and return it; its named graphs keep their names,
    its default graph goes into a graph named by the file's IRI.

    Raises:
        ArchiveError: the document does not parse.
    """
    try:
        dataset.graph(rdflib.URIRef(path.resolve().as_uri())).parse(path)
    except Exception as exc:
        raise ArchiveError(path, f"RDF parse failed: {exc}") from exc
    return dataset


def _write_rec_snapshot(
    run_dir: Path,
    run_id: str,
    generation_document: Path,
    executable: Path | None,
    schema: dict,
    *,
    complete_lifecycle: bool = True,
) -> None:
    """Bring REC's record and the execution document up to the archived run.

    A run REC has not seen is started and its execution recorded, with EXECUTABLE and the
    generation's robots, host and software. Then EXECUTABLE as used and every generated file go
    into the execution document, and the run is completed when COMPLETE_LIFECYCLE and still
    running.
    """
    try:
        run = rec_run(run_dir, run_id)
    except ImportError as exc:
        raise ArchiveError(
            run_dir, "REC is required to archive a run: `motion-spec setup rec`."
        ) from exc

    if run.lifecycle()[0] in (State.NEW, State.QUEUED):
        # rec records the host it starts the run on.
        run.start()
        # Archiving without a prior catalogued run: the caller names the executable that ran.
        record_execution(
            run_dir, run.node, generation_document, executable, schema.get("platform") or {}
        )
    files = motion_spec_bundle(run_dir / EXECUTION_DOCUMENT)
    record_files(files, run.node, run_dir)
    write_generation_graph(run_dir / EXECUTION_DOCUMENT, files)
    if complete_lifecycle and run.lifecycle()[0] in (State.IN_PROGRESS, State.CANCELING):
        run.end(Verdict.PASSED)
