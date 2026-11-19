# SPDX-License-Identifier: MPL-2.0
"""Verify a recorded run from its provenance."""

from __future__ import annotations

from pathlib import Path

import rdflib
from motion_spec_dsl.rdf_parser.manifest import install_metamodel_resolver
from rdf_utils.constraints import SHACLViolation, check_shacl_constraints
from rdf_utils.models.prov import add_file_hash, get_file_path
from rdf_utils.models.vocab import URI_PROV_EXT_TYPE_EXECUTION
from rdf_utils.namespace import URL_MM_PROV_EXT_SHACL, URL_MM_PROV_SHACL, URL_SECORO_MM
from rdflib.namespace import DCTERMS, PROV, RDF

from motion_spec.runs.provenance import (
    GENERATION_DOCUMENT,
    kept_dsl_document,
    rec_run,
    run_generation,
)

FRAME_LOG = "logs/frame_log.pb"
PROV_TYPES = (PROV.Entity, PROV.Activity, PROV.Agent)
PROV_SHAPES = {URL_MM_PROV_SHACL: "turtle", URL_MM_PROV_EXT_SHACL: "turtle"}
REC_SHAPES = {**PROV_SHAPES, f"{URL_SECORO_MM}/rec/rec.shacl.ttl": "turtle"}


class RecordError(ValueError):
    """A run's record is incomplete, or disagrees with its provenance or its generation.

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


def verify_run(run_dir: Path | str) -> None:
    """Check the recorded run in RUN_DIR against its provenance, reading only.

    The generation is the one the run's document names the provenance of, by its location. Its
    log protocol and provenance and the run's document must exist; every file the run's bundles
    hash must still be where they say, with that hash; the generation's provenance must state an
    entity, an activity and an agent; rec's bundle must record the run as a `prov-ext:Execution`
    associated with an agent, and motion-spec's what it used; and the generation's provenance,
    with the DSL's record, must conform to the PROV shapes, and the run's document together with
    it to the REC shapes.

    Raises:
        RecordError: naming every failed check of the first stage that failed.
    """
    run_dir = Path(run_dir)
    generation = run_generation(run_dir)
    if generation is None:
        raise RecordError(run_dir, "the run's document names no generation's provenance")
    generated = generation / "generated"
    provenance_path = generated / GENERATION_DOCUMENT
    recorded_run = rec_run(run_dir)
    rec_path = recorded_run.store.path(recorded_run.id)
    required = (generated / "contract" / "frame_log.proto", provenance_path, rec_path)
    errors = [f"{path}: missing" for path in required if not path.is_file()]
    if errors:
        raise RecordError(run_dir, *errors)

    install_metamodel_resolver()
    provenance = parse_rdf(rdflib.Dataset(default_union=True), provenance_path)
    model_document = kept_dsl_document(generated)
    if model_document is not None and model_document.is_file():
        model = parse_rdf(rdflib.Dataset(), model_document)
        for bundle in model.graphs():
            if (None, RDF.type, PROV.Bundle) in bundle:
                provenance.add_graph(bundle)
    rec = parse_rdf(rdflib.Dataset(default_union=True), rec_path)

    # per bundle: each states its own file locations and hashes
    for graph in rec.graphs():
        recorded = dict(graph.subject_objects(DCTERMS.identifier))
        rehashed = rdflib.Graph()
        rehashed += (triple for triple in graph if triple[1] != DCTERMS.identifier)
        for entity, digest in recorded.items():
            path = get_file_path(graph, entity)
            # an invalidated file is already gone
            if (entity, PROV.wasInvalidatedBy, None) in graph or path is None:
                continue
            if not path.exists():
                errors.append(f"{path}: missing")
                continue
            add_file_hash(rehashed, entity)
            if rehashed.value(entity, DCTERMS.identifier) != digest:
                errors.append(f"{path}: content differs from the recorded one")
    if errors:
        raise RecordError(run_dir, *errors)

    run = next(iter(rec.subjects(RDF.type, URI_PROV_EXT_TYPE_EXECUTION)), None)
    if run is None:
        raise RecordError(rec_path, "records no prov-ext:Execution")
    if run != recorded_run.node:
        raise RecordError(rec_path, f"records the run {run}, not the one in {run_dir}")
    checks = [
        *((provenance_path.name, provenance, (None, RDF.type, kind)) for kind in PROV_TYPES),
        (rec_path.name, rec, (run, RDF.type, URI_PROV_EXT_TYPE_EXECUTION)),
        (rec_path.name, rec, (run, PROV.wasAssociatedWith, None)),
        (rec_path.name, rec, (run, PROV.used, None)),
    ]
    missing = [
        f"{label}: missing {' '.join('?' if term is None else term.n3() for term in triple)}"
        for label, graph, triple in checks
        if triple not in graph
    ]
    if missing:
        raise RecordError(run_dir, *missing)

    # types only: the run may name a package at another version than the generation
    mentioned = {node for triple in rec for node in (triple[0], triple[2])}
    rec.default_graph += (
        triple for triple in provenance.triples((None, RDF.type, None)) if triple[0] in mentioned
    )
    for label, graph, shapes in (
        (provenance_path.name, provenance, PROV_SHAPES),
        (rec_path.name, rec, REC_SHAPES),
    ):
        try:
            check_shacl_constraints(graph, shapes)
        except SHACLViolation as exc:
            raise RecordError(run_dir, f"{label}: {exc}") from exc


def parse_rdf(dataset: rdflib.Dataset, path: Path) -> rdflib.Dataset:
    """Parse the document at PATH into DATASET and return it; its named graphs keep their names,
    its default graph goes into a graph named by the file's IRI.

    Raises:
        RecordError: the document does not parse.
    """
    try:
        dataset.graph(rdflib.URIRef(path.resolve().as_uri())).parse(path)
    except Exception as exc:
        raise RecordError(path, f"RDF parse failed: {exc}") from exc
    return dataset


