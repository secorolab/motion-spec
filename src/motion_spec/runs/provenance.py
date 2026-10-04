# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Provenance for generated artifacts, archives and recorded executions."""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import urllib.parse
from datetime import UTC, datetime, timezone
from pathlib import Path

from rdf_utils.models.prov import (
    add_agent,
    add_entity,
    add_file_entity,
    get_git_info,
    get_pkg_info,
    load_pkg_prov,
    load_sampling_prov,
    load_transformation_prov,
)
from rdf_utils.models.vocab import URI_AGN_TYPE_MOD_AGN
from rdf_utils.namespace import NS_MM_QUDT, URL_MM_PROV_EXT_JSON, URL_MM_PROV_JSON
from rdflib import Dataset, Graph, Literal, Namespace, URIRef
from rdflib.namespace import PROV, RDF, SDO

MSPROV = "https://secorolab.github.io/motion-spec/provenance/"
MSPROV_PREFIX = "msprov:"
DSLPROV = "https://secorolab.github.io/motion-spec-dsl/provenance/"
URL_MM_AGENT_JSON = "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/agent.json"
# agent.json first: it and prov.json both define the term `Agent`, and the later definition wins,
# so with it last every prov:Agent would expand back as agn:Agent.
PROV_CONTEXT = [
    URL_MM_AGENT_JSON,
    URL_MM_PROV_JSON,
    URL_MM_PROV_EXT_JSON,
    {"msprov": MSPROV, "dslprov": DSLPROV},
]

# One generation, one provenance document: three named graphs, one per tool that wrote into it.
# One run, one execution document beside rec's record: what rec does not say about the run.
GENERATION_DOCUMENT = "provenance.ld.json"
EXECUTION_DOCUMENT = "execution.ld.json"
GRAPH_DSL = URIRef(f"{MSPROV}graph/dsl")
GRAPH_COORD_DSL = URIRef(f"{MSPROV}graph/coord-dsl")
GRAPH_MOTION_SPEC = URIRef(f"{MSPROV}graph/motion-spec")
GRAPH_EXECUTION = URIRef(f"{MSPROV}graph/execution")
# What rec is told to mint run nodes under, so its record and these graphs share the node.
RUN_IRI_BASE = f"{MSPROV}run/"


def _slug(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "item"


def _prov_iri(identifier: str) -> str:
    kind, _, name = identifier.partition(":")
    if not name:
        kind, name = "id", identifier
    return f"{MSPROV_PREFIX}{_slug(kind)}/{_slug(name)}"


def prov_uri(identifier: str) -> str:
    """Canonical full provenance IRI for an agent/activity/run id.

    `run:<run-id>` is what makes the rec and generation graphs describe one run rather than
    two, so every document mints the run through here.
    """
    if identifier.startswith(("http://", "https://")):
        return identifier
    if identifier.startswith(MSPROV_PREFIX):
        return MSPROV + identifier[len(MSPROV_PREFIX) :]
    return MSPROV + _prov_iri(identifier)[len(MSPROV_PREFIX) :]


def uri(identifier: str) -> URIRef:
    """`prov_uri` as the node the rdf-utils writers take."""
    return URIRef(prov_uri(identifier))


def rec_document(run_dir: Path, run_id: str | None = None) -> Path:
    """Where rec keeps the run's record: ``<run_dir>/<run_id>.ld.json``, the run dir named after the run."""
    return Path(run_dir) / f"{run_id or Path(run_dir).name}.ld.json"


def run_entity_uri(run_id: str, slug: str) -> str:
    """Canonical IRI for a file entity belonging to one run.

    Run-scoped, so unioning two runs of one generation keeps their artefacts apart instead of
    collapsing them onto one node with conflicting locations and hashes.
    """
    return f"{MSPROV}entity/run/{_slug(run_id)}/{_slug(slug)}"


def generation_scope(generation: Path) -> Namespace:
    """Where one generation names its steps and files, so two generations never merge into one."""
    generation = Path(generation).resolve()
    return Namespace(
        f"{MSPROV}generation/{_slug(generation.parent.name)}/{_slug(generation.name)}/"
    )


def package_agent_uri(name: str, version: str | None, commit: str | None) -> URIRef:
    """A package as one agent per release it ran as: another version or revision is another one."""
    parts = [name, version or "unversioned", *([commit] if commit else [])]
    return URIRef(f"{MSPROV}agent/{'/'.join(_slug(part) for part in parts)}")


def file_path(location) -> Path | None:
    """The local path a `file:` IRI names, or None for any other IRI."""
    parsed = urllib.parse.urlparse(str(location))
    return Path(urllib.parse.unquote(parsed.path)) if parsed.scheme == "file" else None


def _mtime(path: Path) -> datetime:
    return datetime.fromtimestamp(Path(path).stat().st_mtime, UTC)


def add_package(graph: Graph, key: str) -> URIRef:
    """One package as the agent a step is associated with; stst and cmake are not Python ones."""
    if key == "stst":
        from motion_spec.setup import STST_REPOSITORY, shipped_pin

        pin = shipped_pin(STST_REPOSITORY)
        name, version, commit, repository = "stst", None, pin.version, pin.url.removesuffix(".git")
    elif key == "cmake":
        match = re.search(r"cmake version (\S+)", run_tool("cmake", "--version") or "")
        name, version, commit, repository = "cmake", match.group(1) if match else None, None, None
    else:
        name, version, commit, repository = get_pkg_info(key)
    agent = package_agent_uri(name, version, commit)
    load_pkg_prov(graph, agent, name, version, commit, repository)
    return agent


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


def append_generation_graph(document: Path, graph_id: URIRef, nodes: list[dict]) -> None:
    """Merge nodes into one named graph of the document, leaving the rest as written.

    Edited as JSON rather than round-tripped through rdflib, which would resolve the relative
    `prov:atLocation` values back into absolute file IRIs.
    """
    data = (
        json.loads(document.read_text())
        if document.is_file()
        else {"@context": PROV_CONTEXT, "@graph": []}
    )
    for entry in data["@graph"]:
        if entry.get("@id") == str(graph_id):
            entry["@graph"].extend(nodes)
            break
    else:
        data["@graph"].append({"@id": str(graph_id), "@graph": nodes})
    document.write_text(json.dumps(data, indent=2) + "\n")


def write_generation_graph(document: Path, graph_id: URIRef, graph: Graph) -> None:
    """Add everything `graph` states to one named graph of the generation document."""
    append_generation_graph(document, graph_id, _document_nodes(graph))


def read_generation_dataset(document: Path) -> Dataset:
    """The generation document as a dataset; empty when it has not been written yet."""
    from motion_spec_dsl.rdf_parser.manifest import install_metamodel_resolver

    dataset = Dataset(default_union=True)
    if Path(document).is_file():
        install_metamodel_resolver()
        dataset.parse(document, format="json-ld")
    return dataset


def write_generation_document(document: Path, graphs: dict[URIRef, Graph]) -> None:
    """The generation's provenance, written once: each tool's graph a named graph of it."""
    data = {
        "@context": PROV_CONTEXT,
        "@graph": [
            {"@id": str(graph_id), "@graph": _document_nodes(graph)}
            for graph_id, graph in graphs.items()
        ],
    }
    document.write_text(json.dumps(data, indent=2) + "\n")


def generated_file(graph: Graph, generated: Path, path: Path) -> URIRef:
    """One generation-scoped file entity, located relative to GENERATED so the tree can move."""
    entity = generation_scope(generated.parent)[f"entity/{os.path.relpath(path, generated.parent)}"]
    add_file_entity(
        graph, entity, location=URIRef(os.path.relpath(path, generated)), generated_at=_mtime(path)
    )
    return entity


def record_ir_generation(
    graph: Graph,
    generated: Path,
    manifest: URIRef,
    inputs: list[URIRef],
    ir: dict,
    ir_path: Path,
    derived_path: Path,
    *,
    started: datetime,
    ended: datetime,
) -> None:
    """The IR's transformation from the manifest and its documents, by the DSL's nodes for them."""
    activity = generation_scope(generated.parent)["activity/motion_spec_ir_generation"]
    package = add_package(graph, "motion_spec")
    ir_entity = generated_file(graph, generated, ir_path)
    load_transformation_prov(
        graph,
        activity,
        [manifest, *inputs],
        [ir_entity, generated_file(graph, generated, derived_path)],
        package,
        started,
        ended,
    )
    graph.add((ir_entity, PROV.wasDerivedFrom, manifest))

    # The robots the scene models, by the scene's own nodes for them.
    for robot in (ir["composition"]["scene"] or {}).get("robots") or ():
        add_agent(graph, URIRef(robot["agent"]), (URI_AGN_TYPE_MOD_AGN,), robot["id"])


def record_code_generation(
    graph: Graph,
    generated: Path,
    ir_path: Path,
    written: list[Path],
    *,
    started: datetime,
    ended: datetime,
) -> None:
    """The transformation that turned the IR into the controller sources and the contract."""
    activity = generation_scope(generated.parent)["activity/code_generation"]
    package = add_package(graph, "motion_spec")
    # motion-spec drives it; stst renders every template.
    graph.add((activity, PROV.wasAssociatedWith, add_package(graph, "stst")))
    targets = [generated_file(graph, generated, path) for path in written]
    load_transformation_prov(
        graph,
        activity,
        [generated_file(graph, generated, ir_path)],
        targets,
        package,
        started,
        ended,
    )


def record_tool_generation(
    graph: Graph,
    generated: Path,
    step: str,
    package: str,
    sources: list[URIRef],
    written: list[Path],
    *,
    started: datetime,
    ended: datetime,
) -> None:
    """A transformation a DSL package ran for motion-spec, from the DSL's nodes for its sources."""
    activity = generation_scope(generated.parent)[f"activity/{step}"]
    targets = [generated_file(graph, generated, path) for path in written]
    load_transformation_prov(
        graph, activity, sources, targets, add_package(graph, package), started, ended
    )


def record_build(
    generated: Path, controller: Path, executable: Path, *, started: datetime, ended: datetime
) -> None:
    """The transformation that compiled the generated sources into the controller executable."""
    document = generated / GENERATION_DOCUMENT
    dataset = read_generation_dataset(document)
    # Whichever tool wrote a source, it is a file in the controller tree cmake compiled.
    sources = {
        entity
        for entity, location in dataset.subject_objects(PROV.atLocation)
        if (path := file_path(location)) is not None and path.is_relative_to(controller.resolve())
    }

    graph = Graph()
    activity = generation_scope(generated.parent)["activity/build"]
    package = add_package(graph, "cmake")
    target = generated_file(graph, generated, executable)
    load_transformation_prov(graph, activity, sources, [target], package, started, ended)
    write_generation_graph(document, GRAPH_MOTION_SPEC, graph)


def build_derivation_document(
    ir: dict, derived: Graph, authored: list[Graph], generated: Path
) -> dict:
    """Declare what IR generation added: every derived entity the IR uses, with what it is and
    what it is computed from, and every node it materialized in DERIVED.

    A frame-log slot names its value by IRI, and most of those values have no node in the
    authored model; without this graph a run graph could make no statement about them.
    """
    # The step the generation document names as the IR's generator.
    activity = generation_scope(generated.parent)["activity/motion_spec_ir_generation"]
    telemetry = ir["communication"]["telemetry"]
    # The tables of every id and its derivation name each id once; that is not a use of it.
    tables = {id(telemetry.get("uris")), id(telemetry.get("derivations"))}
    used: set[str] = set()
    pending = [ir]
    while pending:
        value = pending.pop()
        if id(value) in tables:
            continue
        if isinstance(value, dict):
            pending.extend(value.values())
        elif isinstance(value, list):
            pending.extend(value)
        elif isinstance(value, str):
            used.add(value)

    graph = Graph()
    for entry in telemetry.get("derivations") or ():
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

    return {
        "@context": PROV_CONTEXT,
        "@graph": _document_nodes(graph),
    }


def rec_run_lifecycle_from_file(path) -> dict:
    """The run's OSLC state and verdict and its timestamps, from rec's document on disk.

    Read back through rec's own mapping, as JSON: an rdflib parse resolves the document's
    contexts, which a run list would pay per row. Every key is None when there is no document
    or it was written before the OSLC terms.
    """
    from rec import jsonld

    empty = {"state": None, "verdict": None, "started_time": None, "completed_time": None}
    path = Path(path)
    if not path.exists():
        return empty
    try:
        record = jsonld.record(json.loads(path.read_text()))
    except (KeyError, StopIteration, ValueError):
        return empty
    info = record.get("run_info") or {}
    return {
        "state": record["state"],
        "verdict": record["verdict"],
        "started_time": info.get("start_time"),
        "completed_time": info.get("end_time"),
    }


def parse_rec_time(value: str) -> datetime:
    parsed = datetime.fromisoformat(value)
    if parsed.tzinfo is None:
        return parsed.replace(tzinfo=timezone.utc)
    return parsed


# The process one run started, which produced its logs: another run is another process.
CONTROLLER_PROCESS = RUN_IRI_BASE + "{run_id}/controller_process"


def record_used_file(run, path: Path, role: str, archive_path: str) -> None:
    """One file the execution used; rec owns its checksum, size and qualified usage.

    Named by where it lands in the bundle, so the reference is portable and dedupes with the
    archive's own record of the same file.
    """
    run.add_resource(
        archive_path,
        usage_time=_mtime(path),
        title=role,
        sha256=artifact_sha256(path),
        size_bytes=artifact_size(path),
    )


def record_execution(
    run_dir: Path,
    run_id: str,
    generation_document: Path,
    platform_facts: dict,
    arguments: list[str],
    started: datetime,
) -> None:
    """What rec does not record of the run: who ran it and the command line it was given.

    Written as the run's execution document, on the same run node rec's record describes.
    The modelled robots are read from the generation document, wherever the caller keeps it.
    """
    from rdf_utils.models.prov import load_execution_prov

    graph = Graph()
    generation = read_generation_dataset(generation_document)
    # A simulated run steps the wrapper's physics; a real one runs on the platform it names.
    if platform_facts.get("simulated"):
        runtime = add_package(graph, "mj_kdl_wrapper")
    else:
        runtime = uri(f"agent:runtime_{_slug(platform_facts.get('backend') or 'runtime')}")
        add_agent(graph, runtime, (PROV.SoftwareAgent,), platform_facts.get("backend") or "runtime")
    controller = URIRef(CONTROLLER_PROCESS.format(run_id=run_id))
    add_agent(graph, controller, (PROV.SoftwareAgent,), "controller", acted_on_behalf_of=runtime)
    for agent in set(generation.subjects(RDF.type, URI_AGN_TYPE_MOD_AGN)):
        add_agent(graph, agent, (URI_AGN_TYPE_MOD_AGN,), str(generation.value(agent, SDO.name)))
    run = uri(f"run:{run_id}")
    # The executable is the node the generation's build generated, when it was built.
    executables = [
        entity
        for entity, activity in generation.subject_objects(PROV.wasGeneratedBy)
        if str(activity).endswith("/activity/build")
    ]
    load_execution_prov(
        graph,
        run,
        [record_arguments(graph, run_id, arguments), *executables],
        add_package(graph, "motion_spec"),
        started,
    )
    # The controller process carried the run out, driving the robots the scene models.
    for agent in (controller, *graph.subjects(RDF.type, URI_AGN_TYPE_MOD_AGN)):
        graph.add((run, PROV.wasAssociatedWith, agent))
    write_generation_graph(run_dir / EXECUTION_DOCUMENT, GRAPH_EXECUTION, graph)


def record_arguments(graph: Graph, run_id: str, arguments: list[str]) -> URIRef:
    """The command line the run was given, as the entity the execution used."""
    from rdflib.namespace import RDFS

    entity = URIRef(run_entity_uri(run_id, "arguments"))
    add_entity(graph, entity)
    graph.set((entity, RDFS.label, Literal(" ".join(arguments))))
    return entity


def record_draw(graph: Graph, run_id: str, agent: URIRef, quantity: str, values, drawn_at) -> None:
    """One draw, as the coordinate this run made of the quantity the generation declared.

    The unit, the kind and the distribution stay on the declared quantity, which every run of
    the generation shares; `prov:specializationOf` is how a reader gets from one to the other.
    """
    from motion_spec_dsl.rdf_parser.vocab import GEOM_COORD, QUDT_SCHEMA
    from rdf_utils.models.common import ModelBase
    from rdf_utils.models.geom_coord import set_coord_vectorxyz
    from rdf_utils.models.vocab import URI_GEOM_TYPE_VECTOR_XYZ

    draw = URIRef(run_entity_uri(run_id, f"draw/{quantity}"))
    quantity_id = URIRef(quantity)
    add_entity(graph, quantity_id)
    load_sampling_prov(
        graph,
        URIRef(f"{prov_uri(f'run:{run_id}')}/sampling/{_slug(quantity)}"),
        [],
        [draw],
        quantity_id,
        agent,
        drawn_at,
        drawn_at,
    )
    if len(values) == 3:
        model = ModelBase(draw, types={URI_GEOM_TYPE_VECTOR_XYZ})
        graph.add((draw, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
        graph.add((draw, RDF.type, URIRef(GEOM_COORD["PositionCoordinate"])))
        set_coord_vectorxyz(model, tuple(float(value) for value in values), graph)
    else:
        graph.set((draw, URIRef(QUDT_SCHEMA["value"]), Literal(float(values[0]))))
    graph.set((draw, PROV.specializationOf, quantity_id))
    graph.set((draw, PROV.generatedAtTime, Literal(drawn_at)))


# What the run produced, as opposed to what it read. Camera videos are a list.
GENERATED_ROLES = ("frame_log", "frame_log_health", "console", "bag", "videos")


def record_files(run, run_dir: Path, manifest: dict) -> None:
    """Record the files the run generated, with their integrity metadata, as PROV entities."""
    for role in GENERATED_ROLES:
        value = manifest.get("files", {}).get(role)
        if not value:
            continue
        for rel in value if isinstance(value, list) else [value]:
            path = run_dir / rel
            if not path.exists():
                continue
            run.add_artefact(
                rel,
                generated_time=_mtime(path),
                title=role,
                sha256=artifact_sha256(path),
                size_bytes=artifact_size(path),
            )


def record_frame_log_health(run, run_dir: Path, manifest: dict) -> None:
    rel = manifest.get("files", {}).get("frame_log_health")
    if not rel:
        return
    path = run_dir / rel
    if not path.exists():
        return
    health = json.loads(path.read_text())
    for name in (
        "attempted_frames",
        "accepted_frames",
        "written_frames",
        "write_errors",
        "dropped_frames",
    ):
        if name in health:
            run.log_scalar(f"frame_log_{name}", health[name], step=0)
    if "complete" in health:
        run.log_scalar("frame_log_complete", int(bool(health["complete"])), step=0)


def artifact_size(path: Path) -> int:
    if path.is_file():
        return path.stat().st_size
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def artifact_sha256(path: Path) -> str:
    """Hash a file or directory tree deterministically."""
    digest = hashlib.sha256()
    items = [path] if path.is_file() else sorted(item for item in path.rglob("*") if item.is_file())
    for item in items:
        if path.is_dir():
            digest.update(item.relative_to(path).as_posix().encode())
            digest.update(b"\0")
        with item.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        if path.is_dir():
            digest.update(b"\0")
    return digest.hexdigest()


def record_software(run, run_dir: Path) -> None:
    """The recorder, and the checkout the run sits in; the execution document names motion-spec."""
    name, version, _, _ = get_pkg_info("rec")
    run.log_dependencies([{"name": name, **({"version": version} if version else {})}])
    commit, url = get_git_info(run_dir)
    if commit:
        name = url.rsplit("/", 1)[-1] if url else Path(run_dir).name
        run.log_repositories([{"name": name, "commit": commit, **({"url": url} if url else {})}])


def run_tool(*command: str) -> str | None:
    try:
        return subprocess.check_output(command, text=True, stderr=subprocess.DEVNULL)
    except Exception:
        return None
