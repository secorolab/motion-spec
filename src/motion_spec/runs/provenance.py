# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Provenance for generated artifacts and recorded executions."""

from __future__ import annotations

import re
import subprocess
from collections.abc import Iterator
from contextlib import contextmanager
from datetime import UTC, datetime
from functools import lru_cache
from pathlib import Path
from uuid import uuid4

from packageurl import PackageURL
from rdf_utils import DatasetFormat
from rdf_utils.models.prov import (
    PackageInfo,
    add_agent,
    add_directory_location,
    add_entity,
    add_file_entity,
    capture_file_prov,
    create_bundle,
    get_file_path,
    get_py_pkg_info,
    load_pkg_prov,
    load_sampling_prov,
    load_transformation_prov,
)
from rdf_utils.models.vocab import URI_AGN_TYPE_MOD_AGN, URI_PROV_EXT_TYPE_EXECUTION
from rdf_utils.namespace import NS_MM_QUDT
from rdflib import Dataset, Graph, Literal, URIRef
from rdflib.namespace import PROV, RDF, SDO

PROVENANCE_DIR = "provenance"
GENERATION_DOCUMENT = f"{PROVENANCE_DIR}/generation.ld.json"
DERIVED_DOCUMENT = f"{PROVENANCE_DIR}/derived.ld.json"
FRAME_LOG_PACKED = "frame_log.pb.zst"


def rec_run(run_dir: Path, run_id: str | None = None):
    """rec's handle on the run in RUN_DIR, its record kept in the run's `provenance/`."""
    from rec.run import Run
    from rec.stores.file_store import FileStore

    return Run(FileStore(Path(run_dir) / PROVENANCE_DIR), run_id or Path(run_dir).name)


@contextmanager
def run_bundle(
    run_dir: Path, run_id: str | None = None, bundle_id: URIRef | None = None
) -> Iterator[Graph]:
    """motion-spec's bundle in the run's document, beside rec's: what rec does not record of the
    run, edited under rec's store lock so neither tool overwrites the other.

    BUNDLE_ID is the bundle an earlier step of the run created; None creates it, and the caller
    keeps the yielded graph's identifier for the steps after.
    """
    run = rec_run(run_dir, run_id)
    with run.store.edit(run.id) as dataset:
        yield dataset.graph(
            bundle_id or create_bundle(dataset, get_py_pkg_info("motion_spec").purl)
        )


def kept_dsl_document(generated: Path) -> Path | None:
    """The DSL's document of the generation: the model document attributed to the DSL."""
    graph = read_generation_dataset(generated / GENERATION_DOCUMENT)
    dsl = next(graph.subjects(SDO.name, Literal("motion_spec_dsl")), None)
    document = graph.value(predicate=PROV.wasAttributedTo, object=dsl) if dsl else None
    return get_file_path(graph, document) if document is not None else None


def built_executable(generated: Path) -> Path | None:
    """The controller executable of the generation: what the build, the transformation cmake
    is associated with, generated. None when no build is recorded."""
    graph = read_generation_dataset(generated / GENERATION_DOCUMENT)
    cmake = next(graph.subjects(SDO.name, Literal("cmake")), None)
    build = graph.value(predicate=PROV.wasAssociatedWith, object=cmake) if cmake else None
    executable = graph.value(predicate=PROV.wasGeneratedBy, object=build) if build else None
    return get_file_path(graph, executable) if executable is not None else None


@lru_cache(maxsize=64)
def generation_model(generated: Path):
    """The model a generation was made from, as the RDF its tools wrote: the documents its
    provenance attributes, each graph a named graph, read as a `Model` for the `rdf_parser`
    readers. Kept per generation, which no step changes once written.

    Raises:
        ValueError: the generation's provenance records no model document.
    """
    from rdf_utils.resolver import install_resolver

    from motion_spec.rdf_parser.model import Model

    graph = read_generation_dataset(generated / GENERATION_DOCUMENT)
    paths = (
        get_file_path(graph, entity)
        for entity in set(graph.subjects(PROV.wasAttributedTo))
        if (entity, PROV.atLocation, None) in graph
    )
    documents = [path for path in paths if path is not None]
    if not documents:
        raise ValueError(f"{generated}: its provenance records no model document")
    install_resolver()
    dataset = Dataset(default_union=True)
    for path in documents:
        dataset.parse(path, format="json-ld")
    return Model(graph=dataset)


def add_package(graph: Graph, key: str) -> URIRef:
    """One package as the agent an activity is associated with, named by its purl, the version
    and revision that ran stated on it in this graph. stst and cmake are not Python packages."""
    if key == "stst":
        from motion_spec.setup import STST_REPOSITORY, shipped_pin

        pin = shipped_pin(STST_REPOSITORY)
        info = PackageInfo(
            name="stst",
            purl=URIRef(PackageURL(type="generic", name="stst", version=pin.version).to_string()),
            version=pin.version,
            repository=pin.url.removesuffix(".git"),
        )
    elif key == "cmake":
        try:
            output = subprocess.check_output(
                ("cmake", "--version"), text=True, stderr=subprocess.DEVNULL
            )
        except (OSError, subprocess.CalledProcessError):
            output = None
        match = re.search(r"cmake version (\S+)", output or "")
        version = match.group(1) if match else None
        info = PackageInfo(
            name="cmake",
            purl=URIRef(PackageURL(type="generic", name="cmake", version=version).to_string()),
            version=version,
        )
    else:
        info = get_py_pkg_info(key)
    load_pkg_prov(graph, info)
    return info.purl


def add_delegation(graph: Graph, delegate: URIRef, responsible: URIRef, activity: URIRef) -> None:
    """DELEGATE acted on behalf of RESPONSIBLE in ACTIVITY: a qualified `prov:Delegation`."""
    # not blank: rdflib's JSON-LD compaction drops blank nodes behind `@id` terms
    delegation = URIRef(uuid4().urn)
    graph.add((delegate, PROV.qualifiedDelegation, delegation))
    graph.add((delegation, RDF.type, PROV.Delegation))
    graph.add((delegation, PROV.agent, responsible))
    graph.add((delegation, PROV.hadActivity, activity))


def read_generation_dataset(document: Path) -> Dataset:
    """The generation document as a dataset; empty when it has not been written yet."""
    from rdf_utils.resolver import install_resolver

    dataset = Dataset(default_union=True)
    if Path(document).is_file():
        install_resolver()
        dataset.parse(document, format="json-ld")
    return dataset


def record_ir_generation(
    bundle: Graph,
    generation: Path,
    inputs: dict[URIRef, URIRef],
    activity: URIRef,
    ir: dict,
    ir_path: Path,
    derived_path: Path,
    *,
    started: datetime,
    ended: datetime,
) -> URIRef:
    """The IR's transformation from the files INPUTS maps to the agents that wrote them.

    Returns:
        URI of the IR
    """
    # relative to the generation directory, so the record moves with it
    roots = {generation.resolve(): add_directory_location(bundle, generation)}
    targets = [capture_file_prov(bundle, path, roots=roots) for path in (ir_path, derived_path)]
    load_transformation_prov(
        bundle,
        set(inputs),
        targets,
        add_package(bundle, "motion_spec"),
        activity_id=activity,
        started=started,
        ended=ended,
    )
    for source, agent in inputs.items():
        bundle.add((source, PROV.wasAttributedTo, agent))

    for robot in (ir["composition"]["scene"] or {}).get("robots") or ():
        add_agent(bundle, URIRef(robot["agent"]), (URI_AGN_TYPE_MOD_AGN,), robot["id"])
    return targets[0]


def record_code_generation(
    bundle: Graph,
    generation: Path,
    inputs: list[URIRef],
    written: list[Path],
    *,
    started: datetime,
    ended: datetime,
) -> None:
    """The transformation that turned INPUTS, the IR and the FSM graph, into the controller
    sources and the contract."""
    roots = {generation.resolve(): add_directory_location(bundle, generation)}
    targets = [capture_file_prov(bundle, path, roots=roots) for path in written]
    activity = load_transformation_prov(
        bundle, inputs, targets, add_package(bundle, "motion_spec"), started=started, ended=ended
    )
    bundle.add((activity, PROV.wasAssociatedWith, add_package(bundle, "stst")))


def record_build(
    generated: Path, controller: Path, executable: Path, *, started: datetime, ended: datetime
) -> None:
    """The transformation that compiled the generated sources into the controller executable,
    in a bundle of its own in the generation's document."""
    document = generated / GENERATION_DOCUMENT
    dataset = read_generation_dataset(document)
    # sources from any tool, as long as they are in the controller tree
    sources = {
        entity
        for entity in dataset.subjects(PROV.wasGeneratedBy)
        if (path := get_file_path(dataset, entity)) is not None
        and path.is_relative_to(controller.resolve())
    }
    bundle = dataset.graph(create_bundle(dataset, get_py_pkg_info("motion_spec").purl))
    roots = {generated.parent.resolve(): add_directory_location(bundle, generated.parent)}
    target = capture_file_prov(bundle, executable, roots=roots)
    activity = load_transformation_prov(
        bundle, sources, [target], add_package(bundle, "cmake"), started=started, ended=ended
    )
    bundle.add((activity, PROV.wasAssociatedWith, add_package(bundle, "motion_spec")))
    document.write_text(
        dataset.serialize(format=DatasetFormat.JSON_LD, indent=2, auto_compact=True)
    )


def build_derivation_document(
    ir: dict, derived: Graph, authored: list[Graph], activity: URIRef
) -> Graph:
    """Declare what IR generation added: every derived entity the IR uses, with what it is and
    what it is computed from, and every node it materialized in DERIVED.

    A frame-log slot names its value by IRI, and most of those values have no node in the
    authored model; without this graph a run graph could make no statement about them. ACTIVITY
    is the IR's generation, as the generation document names it.
    """
    provenance = ir["provenance"]
    # the provenance section itself is not a use
    used: set[str] = set()
    pending = [ir]
    while pending:
        value = pending.pop()
        if value is provenance:
            continue
        if isinstance(value, dict):
            pending.extend(value.values())
        elif isinstance(value, list):
            pending.extend(value)
        elif isinstance(value, str):
            used.add(value)

    graph = Graph()
    for entry in provenance["derivations"]:
        if entry["id"] not in used:
            continue
        node = URIRef(entry["uri"])
        add_entity(graph, node)
        graph.add((node, PROV.wasGeneratedBy, activity))
        for type_ in entry["types"]:
            graph.add((node, RDF.type, URIRef(type_)))
        if entry["relation"] == "specializationOf":
            add_entity(graph, URIRef(entry["parent"]))
            graph.add((node, PROV.specializationOf, URIRef(entry["parent"])))
        sources = entry.get("sources") or (
            [entry["parent"]] if entry["relation"] == "wasDerivedFrom" else []
        )
        for source in sources:
            add_entity(graph, URIRef(source))
            graph.add((node, PROV.wasDerivedFrom, URIRef(source)))
        if entry.get("kind"):
            graph.add((node, RDF.type, NS_MM_QUDT["Quantity"]))
            graph.add((node, NS_MM_QUDT["hasQuantityKind"], URIRef(entry["kind"])))
        if entry.get("unit"):
            graph.add((node, NS_MM_QUDT["unit"], URIRef(entry["unit"])))

    graph += derived
    for subject in set(derived.subjects()):
        if isinstance(subject, URIRef) and not any(
            (subject, None, None) in document for document in authored
        ):
            add_entity(graph, subject)
            graph.add((subject, PROV.wasGeneratedBy, activity))
    return graph


def record_run_file(
    bundle: Graph, run: URIRef, generation: Path, path: Path, *, generated: bool
) -> URIRef:
    """Add to BUNDLE the file at PATH, with its hash, as an entity RUN used, or GENERATED, located
    under the GENERATION directory when it is in it, with a qualified usage or generation at the
    file's modification time, and return the entity."""
    roots = {generation.resolve(): add_directory_location(bundle, generation)}
    entity = capture_file_prov(bundle, path, roots=roots)
    # not blank, see add_delegation
    qualified = URIRef(uuid4().urn)
    if generated:
        bundle.add((entity, PROV.wasGeneratedBy, run))
        bundle.add((entity, PROV.qualifiedGeneration, qualified))
        bundle.add((qualified, RDF.type, PROV.Generation))
        bundle.add((qualified, PROV.activity, run))
    else:
        bundle.add((run, PROV.used, entity))
        bundle.add((run, PROV.qualifiedUsage, qualified))
        bundle.add((qualified, RDF.type, PROV.Usage))
        bundle.add((qualified, PROV.entity, entity))
    bundle.add(
        (qualified, PROV.atTime, Literal(datetime.fromtimestamp(Path(path).stat().st_mtime, UTC)))
    )
    return entity


def record_execution(
    bundle: Graph,
    run: URIRef,
    generated: Path,
    executable: Path,
    platform_facts: dict,
) -> None:
    """Add to motion-spec's BUNDLE in the run's document what rec does not record of RUN.

    The run, on the node rec's record describes, is an execution by motion-spec that used
    the executable the generation's build generated -- which has its provenance in GENERATED's
    generation document -- and the platform IRI PLATFORM_FACTS names. It is associated with the
    process EXECUTABLE ran as and with every robot that document models. The process acted, in this run, on behalf of motion-spec, which launched it, and of
    the runtime it drove (mj_kdl_wrapper in simulation, else the named backend): a qualified
    `prov:Delegation` each, as the generation's tools acted for motion-spec in the generation.
    """
    generation_document = generated / GENERATION_DOCUMENT
    generation = read_generation_dataset(generation_document)
    motion_spec = add_package(bundle, "motion_spec")
    if platform_facts.get("simulated"):
        runtime = add_package(bundle, "mj_kdl_wrapper")
    else:
        backend = platform_facts.get("backend") or "runtime"
        runtime = URIRef(PackageURL(type="generic", name=backend).to_string())
        load_pkg_prov(bundle, PackageInfo(name=backend, purl=runtime))
    process = URIRef(uuid4().urn)
    add_agent(bundle, process, (PROV.SoftwareAgent,), executable.name)
    add_delegation(bundle, process, motion_spec, run)
    add_delegation(bundle, process, runtime, run)
    bundle.add((run, RDF.type, PROV.Activity))
    bundle.add((run, RDF.type, URI_PROV_EXT_TYPE_EXECUTION))
    bundle.add((run, PROV.wasAssociatedWith, motion_spec))
    bundle.add((run, PROV.wasAssociatedWith, process))
    for agent in set(generation.subjects(RDF.type, URI_AGN_TYPE_MOD_AGN)):
        add_agent(bundle, agent, (URI_AGN_TYPE_MOD_AGN,), str(generation.value(agent, SDO.name)))
        bundle.add((run, PROV.wasAssociatedWith, agent))
    # rdf-utils names one file entity per location
    executable_path = executable.resolve()
    built = next(
        (
            entity
            for entity in generation.subjects(PROV.atLocation)
            if (entity, RDF.type, PROV.Location) not in generation
            and get_file_path(generation, entity) == executable_path
        ),
        None,
    )
    # located, not hashed: later generation steps add to the record
    generation_dir = generated.resolve().parent
    record = add_file_entity(
        bundle,
        add_directory_location(bundle, generation_dir),
        generation_document.resolve().relative_to(generation_dir),
    )
    if built is not None:
        bundle.add((built, PROV.has_provenance, record))
        bundle.add((run, PROV.used, built))
    if platform_facts.get("uri"):
        platform = URIRef(platform_facts["uri"])
        add_entity(bundle, platform)
        bundle.add((run, PROV.used, platform))


def record_draw(bundle: Graph, agent: URIRef, quantity: str, values, drawn_at) -> None:
    """One draw of the quantity the generation declared, recorded as rdf-utils records a sample:
    the drawn values on the quantity, in this run's bundle, and a sampling by AGENT that
    generated an entity specializing it.

    The run's bundle is its own sample graph, so every run of a generation keeps its own values.
    The program drew them at run time, a position as a VectorXYZ and anything else as one number.
    """
    from motion_spec_dsl.vocab import QUDT_SCHEMA
    from rdf_utils.models.common import ModelBase
    from rdf_utils.models.geom_coord import set_coord_vectorxyz
    from rdf_utils.models.vocab import URI_GEOM_TYPE_VECTOR_XYZ

    quantity_id = URIRef(quantity)
    if len(values) == 3:
        coordinate = ModelBase(quantity_id, types={URI_GEOM_TYPE_VECTOR_XYZ})
        set_coord_vectorxyz(coordinate, tuple(float(value) for value in values), bundle)
    else:
        bundle.set((quantity_id, URIRef(QUDT_SCHEMA["value"]), Literal(float(values[0]))))
    load_sampling_prov(
        bundle, None, [], [URIRef(uuid4().urn)], quantity_id, agent, drawn_at, drawn_at
    )


def record_files(bundle: Graph, run: URIRef, run_dir: Path) -> None:
    """Add to BUNDLE, as entities RUN generated, each output found in RUN_DIR: the console log,
    camera videos, and every file of the ROS bag once rosbag2 closed it. The packed frame log is
    the runner's, recorded where it packs it.
    """
    logs = run_dir / "logs"
    generated = [logs / "console.log", *logs.glob("*.mp4")]
    # rosbag2 writes metadata.yaml on close
    if (run_dir / "bag" / "metadata.yaml").is_file():
        generated.extend(path for path in (run_dir / "bag").rglob("*") if path.is_file())
    for path in generated:
        if path.exists():
            record_run_file(bundle, run, run_dir.parents[1], path, generated=True)


def run_outputs(run_dir: Path) -> list[Path]:
    """The local paths of the files the run's document in RUN_DIR records the run as having
    generated, or derived from one, like the packed frame log; empty before its outputs are recorded
    or when the document does not exist."""
    run = rec_run(run_dir)
    graph = run.store.load(run.id)
    if graph is None:
        return []
    graph.default_union = True
    generated = set(graph.subjects(PROV.wasGeneratedBy, run.node))
    entities = generated | {
        entity for source in generated for entity in graph.subjects(PROV.wasDerivedFrom, source)
    }
    paths = [get_file_path(graph, entity) for entity in entities]
    return [path for path in paths if path is not None]


def run_generation(run_dir: Path) -> Path | None:
    """The generation directory of the run in RUN_DIR: the one whose provenance the run's
    document names for the executable the run used, at the location it states; None before
    motion-spec has recorded the run."""
    run = rec_run(run_dir)
    graph = run.store.load(run.id)
    if graph is None:
        return None
    graph.default_union = True
    for used in graph.objects(run.node, PROV.used):
        for record in graph.objects(used, PROV.has_provenance):
            return get_file_path(graph, graph.value(record, PROV.atLocation))
    return None


def artifact_size(path: Path) -> int:
    if path.is_file():
        return path.stat().st_size
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())
