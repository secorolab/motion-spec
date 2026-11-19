# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Provenance for generated artifacts, archives and recorded executions."""

from __future__ import annotations

import json
import re
import subprocess
from datetime import UTC, datetime
from pathlib import Path
from uuid import uuid4

from packageurl import PackageURL
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
from rdf_utils.namespace import (
    NS_MM_QUDT,
    URL_MM_PROV_EXT_JSON,
    URL_MM_PROV_JSON,
    URL_SECORO_UNI,
)
from rdflib import Dataset, Graph, Literal, Namespace, URIRef
from rdflib.namespace import PROV, RDF, SDO

MSPROV = Namespace(f"{URL_SECORO_UNI}/motion-spec/provenance/")
URL_MM_AGENT_JSON = "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/agent.json"
# agent.json first: it and prov.json both define the term `Agent`, and the later definition wins,
# so with it last every prov:Agent would expand back as agn:Agent.
PROV_CONTEXT = [
    URL_MM_AGENT_JSON,
    URL_MM_PROV_JSON,
    URL_MM_PROV_EXT_JSON,
    {"msprov": str(MSPROV)},
]

# A generation's and a run's provenance documents sit in its `provenance/`.
PROVENANCE_DIR = "provenance"
# One generation, one provenance document: a bundle per tool that wrote into it.
# One run, one execution document beside rec's record: what rec does not say about the run.
GENERATION_DOCUMENT = f"{PROVENANCE_DIR}/generation.ld.json"
DERIVED_DOCUMENT = f"{PROVENANCE_DIR}/derived.ld.json"
EXECUTION_DOCUMENT = f"{PROVENANCE_DIR}/execution.ld.json"
# What rec is told to mint run nodes under, so its record and the execution document share the node.
RUN_IRI_BASE = str(MSPROV["run/"])
# A run's frame log once archived.
FRAME_LOG_PACKED = "frame_log.pb.zst"


def rec_document(run_dir: Path, run_id: str | None = None) -> Path:
    """Where rec keeps the run's record: ``<run_dir>/provenance/<run_id>.ld.json``, the run dir
    named after the run."""
    return Path(run_dir) / PROVENANCE_DIR / f"{run_id or Path(run_dir).name}.ld.json"


def rec_run(run_dir: Path, run_id: str | None = None):
    """rec's handle on the run in RUN_DIR, its record kept in the run's `provenance/`."""
    from rec.run import Run
    from rec.stores.file_store import FileStore

    return Run(
        FileStore(Path(run_dir) / PROVENANCE_DIR), run_id or Path(run_dir).name, base=RUN_IRI_BASE
    )


def located_entities(graph: Graph, path: Path) -> list[URIRef]:
    """The entities GRAPH locates at PATH, the generated ones first."""
    path = Path(path).resolve()
    located = [
        entity
        for entity in set(graph.subjects(PROV.atLocation))
        if (entity, RDF.type, PROV.Location) not in graph and get_file_path(graph, entity) == path
    ]
    return sorted(located, key=lambda entity: (entity, PROV.wasGeneratedBy, None) not in graph)


def kept_dsl_document(generated: Path) -> Path | None:
    """Where the generation keeps the DSL's document: the IR's source attributed to the DSL."""
    graph = read_generation_dataset(generated / GENERATION_DOCUMENT)
    for ir in located_entities(graph, generated / "model" / "ir.json"):
        for source in graph.objects(ir, PROV.wasDerivedFrom):
            for agent in graph.objects(source, PROV.wasAttributedTo):
                if str(graph.value(agent, SDO.name)) == "motion_spec_dsl":
                    return get_file_path(graph, source)
    return None


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
        match = re.search(r"cmake version (\S+)", run_tool("cmake", "--version") or "")
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


def _document_nodes(graph: Graph) -> list[dict]:
    """One graph as the JSON-LD node list that goes inside a named graph of the document."""
    from motion_spec_dsl.rdf_parser.manifest import install_metamodel_resolver

    install_metamodel_resolver()
    document = json.loads(
        graph.serialize(format="json-ld", context=PROV_CONTEXT, auto_compact=True)
    )
    nodes = document.get("@graph")
    if nodes is not None:
        return nodes
    return [{key: value for key, value in document.items() if key != "@context"}]


def write_generation_graph(document: Path, bundle: Graph) -> None:
    """Add everything BUNDLE states to its named graph of the document, leaving the rest as written."""
    data = (
        json.loads(document.read_text())
        if document.is_file()
        else {"@context": PROV_CONTEXT, "@graph": []}
    )
    nodes = _document_nodes(bundle)
    for entry in data["@graph"]:
        if entry.get("@id") == str(bundle.identifier):
            entry["@graph"].extend(nodes)
            break
    else:
        data["@graph"].append({"@id": str(bundle.identifier), "@graph": nodes})
    document.parent.mkdir(parents=True, exist_ok=True)
    document.write_text(json.dumps(data, indent=2) + "\n")


def read_generation_dataset(document: Path) -> Dataset:
    """The generation document as a dataset; empty when it has not been written yet."""
    from motion_spec_dsl.rdf_parser.manifest import install_metamodel_resolver

    dataset = Dataset(default_union=True)
    if Path(document).is_file():
        install_metamodel_resolver()
        dataset.parse(document, format="json-ld")
    return dataset


def motion_spec_bundle(document: Path) -> Graph:
    """The bundle of DOCUMENT attributed to motion-spec, to add to; a new one when it has none."""
    purl = get_py_pkg_info("motion_spec").purl
    dataset = read_generation_dataset(document)
    for bundle in dataset.subjects(PROV.wasAttributedTo, purl):
        if (bundle, RDF.type, PROV.Bundle) in dataset:
            return Graph(identifier=bundle)
    created = Dataset()
    return created.graph(create_bundle(created, purl))


def write_generation_document(document: Path, bundles: list[Graph]) -> None:
    """The generation's provenance, written once: each bundle a named graph of it."""
    data = {
        "@context": PROV_CONTEXT,
        "@graph": [
            {"@id": str(bundle.identifier), "@graph": _document_nodes(bundle)}
            for bundle in bundles
        ],
    }
    document.parent.mkdir(parents=True, exist_ok=True)
    document.write_text(json.dumps(data, indent=2) + "\n")


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
    # Located under the generation directory, so the record moves with it.
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

    # The robots the scene models, by the scene's own nodes for them.
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
    # motion-spec drives it; stst renders every template.
    bundle.add((activity, PROV.wasAssociatedWith, add_package(bundle, "stst")))


def record_build(
    generated: Path, controller: Path, executable: Path, *, started: datetime, ended: datetime
) -> None:
    """The transformation that compiled the generated sources into the controller executable."""
    document = generated / GENERATION_DOCUMENT
    dataset = read_generation_dataset(document)
    # Whichever tool wrote a source, it is a file in the controller tree cmake compiled.
    sources = {
        entity
        for entity in set(dataset.subjects(PROV.wasGeneratedBy))
        if (path := get_file_path(dataset, entity)) is not None
        and path.is_relative_to(controller.resolve())
    }
    bundle = motion_spec_bundle(document)
    roots = {generated.parent.resolve(): add_directory_location(bundle, generated.parent)}
    target = capture_file_prov(bundle, executable, roots=roots)
    activity = load_transformation_prov(
        bundle, sources, [target], add_package(bundle, "cmake"), started=started, ended=ended
    )
    # motion-spec drives it, as the bundle's generation already describes; cmake compiles.
    bundle.add((activity, PROV.wasAssociatedWith, add_package(bundle, "motion_spec")))
    write_generation_graph(document, bundle)


def build_derivation_document(
    ir: dict, derived: Graph, authored: list[Graph], activity: URIRef
) -> dict:
    """Declare what IR generation added: every derived entity the IR uses, with what it is and
    what it is computed from, and every node it materialized in DERIVED.

    A frame-log slot names its value by IRI, and most of those values have no node in the
    authored model; without this graph a run graph could make no statement about them. ACTIVITY
    is the IR's generation, as the generation document names it.
    """
    provenance = ir["provenance"]
    # The provenance section names every id and its derivation once; that is not a use of it.
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
        # What it is computed from when the IR says; otherwise only the node it derives from.
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

    return {"@context": PROV_CONTEXT, "@graph": _document_nodes(graph)}


def rec_run_lifecycle_from_file(path) -> dict:
    """The run's OSLC state and verdict, from rec's document on disk.

    Both are None when there is no document.
    """
    path = Path(path)
    if not path.is_file():
        return {"state": None, "verdict": None}
    state, verdict = rec_run(path.parent.parent, path.name.removesuffix(".ld.json")).lifecycle()
    return {"state": state, "verdict": verdict}


def record_run_file(
    bundle: Graph, run: URIRef, generation: Path, path: Path, *, generated: bool
) -> URIRef:
    """Add to BUNDLE the file at PATH, with its hash, as an entity RUN used, or GENERATED, located
    under the GENERATION directory when it is in it, with a qualified usage or generation at the
    file's modification time, and return the entity."""
    roots = {generation.resolve(): add_directory_location(bundle, generation)}
    entity = capture_file_prov(bundle, path, roots=roots)
    # Not blank: rdflib's JSON-LD compaction drops a blank node behind an `@id` term.
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
    run_dir: Path,
    run: URIRef,
    generation_document: Path,
    executable: Path,
    platform_facts: dict,
) -> None:
    """Write into RUN_DIR's execution document what rec does not record of RUN.

    The run, on the node rec's record describes, is an execution by motion-spec that used
    the executable the generation's build generated -- which has its
    provenance in GENERATION_DOCUMENT -- and the platform IRI PLATFORM_FACTS names. It is
    associated with the process EXECUTABLE ran as, which acted for the runtime (mj_kdl_wrapper
    in simulation, else the named backend), and with every robot GENERATION_DOCUMENT models.
    """
    document = run_dir / EXECUTION_DOCUMENT
    bundle = motion_spec_bundle(document)
    generation = read_generation_dataset(generation_document)
    # A simulated run steps the wrapper's physics; a real one runs on the platform it names.
    if platform_facts.get("simulated"):
        runtime = add_package(bundle, "mj_kdl_wrapper")
    else:
        backend = platform_facts.get("backend") or "runtime"
        runtime = URIRef(PackageURL(type="generic", name=backend).to_string())
        load_pkg_prov(bundle, PackageInfo(name=backend, purl=runtime))
    process = URIRef(uuid4().urn)
    add_agent(bundle, process, (PROV.SoftwareAgent,), executable.name, acted_on_behalf_of=runtime)
    for agent in set(generation.subjects(RDF.type, URI_AGN_TYPE_MOD_AGN)):
        add_agent(bundle, agent, (URI_AGN_TYPE_MOD_AGN,), str(generation.value(agent, SDO.name)))
    # The executable is the node the generation's build generated, found by where it is.
    used = located_entities(generation, executable)[:1]
    # PROV-AQ's link from what the run ran to the record that describes it, the generation's.
    # Located, not hashed: later steps of the generation add to that record.
    generation_dir = generation_document.resolve().parents[len(Path(GENERATION_DOCUMENT).parts)]
    record = add_file_entity(
        bundle,
        add_directory_location(bundle, generation_dir),
        generation_document.resolve().relative_to(generation_dir),
    )
    for entity in used:
        bundle.add((entity, PROV.has_provenance, record))
    if platform_facts.get("uri"):
        used.append(URIRef(platform_facts["uri"]))
        add_entity(bundle, used[-1])
    # rec records when the run began and ended.
    bundle.add((run, RDF.type, PROV.Activity))
    bundle.add((run, RDF.type, URI_PROV_EXT_TYPE_EXECUTION))
    for entity in used:
        bundle.add((run, PROV.used, entity))
    # motion-spec ran it; the process carried it out, driving the robots the scene models.
    for agent in (
        add_package(bundle, "motion_spec"),
        process,
        *bundle.subjects(RDF.type, URI_AGN_TYPE_MOD_AGN),
    ):
        bundle.add((run, PROV.wasAssociatedWith, agent))
    write_generation_graph(document, bundle)


def record_draw(bundle: Graph, agent: URIRef, quantity: str, values, drawn_at) -> None:
    """One draw, as the coordinate this run made of the quantity the generation declared.

    The unit, the kind and the distribution stay on the declared quantity, which every run of
    the generation shares; `prov:specializationOf` is how a reader gets from one to the other.
    """
    from motion_spec_dsl.rdf_parser.vocab import GEOM_COORD, QUDT_SCHEMA
    from rdf_utils.models.common import ModelBase
    from rdf_utils.models.geom_coord import set_coord_vectorxyz
    from rdf_utils.models.vocab import URI_GEOM_TYPE_VECTOR_XYZ

    draw = URIRef(uuid4().urn)
    load_sampling_prov(bundle, None, [], [draw], URIRef(quantity), agent, drawn_at, drawn_at)
    if len(values) == 3:
        model = ModelBase(draw, types={URI_GEOM_TYPE_VECTOR_XYZ})
        bundle.add((draw, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
        bundle.add((draw, RDF.type, URIRef(GEOM_COORD["PositionCoordinate"])))
        set_coord_vectorxyz(model, tuple(float(value) for value in values), bundle)
    else:
        bundle.set((draw, URIRef(QUDT_SCHEMA["value"]), Literal(float(values[0]))))
    bundle.set((draw, PROV.generatedAtTime, Literal(drawn_at)))


def record_files(bundle: Graph, run: URIRef, run_dir: Path) -> None:
    """Add to BUNDLE, as entities RUN generated, each output found in RUN_DIR: the console log,
    camera videos, and every file of the ROS bag once rosbag2 closed it. The packed frame log is
    the archive's, recorded where it packs.
    """
    logs = run_dir / "logs"
    generated = [logs / "console.log", *sorted(logs.glob("*.mp4"))]
    # metadata.yaml is what says rosbag2 closed the bag.
    if (run_dir / "bag" / "metadata.yaml").is_file():
        generated.extend(sorted(path for path in (run_dir / "bag").rglob("*") if path.is_file()))
    for path in generated:
        if path.exists():
            record_run_file(bundle, run, run_dir.parents[1], path, generated=True)


def execution_node(graph: Graph) -> URIRef | None:
    """The run an execution document or rec's record describes: its one `prov-ext:Execution`."""
    return next(iter(graph.subjects(RDF.type, URI_PROV_EXT_TYPE_EXECUTION)), None)


def run_outputs(run_dir: Path) -> list[Path]:
    """The local paths of the files the execution document in RUN_DIR records the run as having
    generated, or derived from one, like the packed frame log; empty before the run is archived
    or when the document does not exist."""
    graph = read_generation_dataset(run_dir / EXECUTION_DOCUMENT)
    run = execution_node(graph)
    if run is None:
        return []
    generated = set(graph.subjects(PROV.wasGeneratedBy, run))
    entities = generated | {
        entity for source in generated for entity in graph.subjects(PROV.wasDerivedFrom, source)
    }
    paths = [get_file_path(graph, entity) for entity in entities]
    return [path for path in paths if path is not None]


def run_generation(run_dir: Path) -> Path | None:
    """The generation directory of the run in RUN_DIR: the one whose provenance its execution
    document names for the executable the run used, at the location it states; None before
    that document is written."""
    graph = read_generation_dataset(run_dir / EXECUTION_DOCUMENT)
    run = execution_node(graph)
    if run is None:
        return None
    for used in graph.objects(run, PROV.used):
        for record in graph.objects(used, PROV.has_provenance):
            path = get_file_path(graph, record)
            if path is not None:
                # The record is <generation>/generated/GENERATION_DOCUMENT.
                return path.parents[len(Path(GENERATION_DOCUMENT).parts)]
    return None


def artifact_size(path: Path) -> int:
    if path.is_file():
        return path.stat().st_size
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def run_tool(*command: str) -> str | None:
    try:
        return subprocess.check_output(command, text=True, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.CalledProcessError):
        return None
