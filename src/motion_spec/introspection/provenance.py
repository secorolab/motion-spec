# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Introspection provenance for generated artifacts, archives and recorded executions."""

from __future__ import annotations

import hashlib
import importlib.metadata
import json
import platform
import re
import socket
import subprocess
import sys
from datetime import UTC, datetime, timezone
from pathlib import Path

from rdf_utils.models.prov import (
    add_agent,
    add_entity,
    add_file_entity,
    load_pkg_prov,
    load_sampling_prov,
    load_transformation_prov,
)
from rdf_utils.models.vocab import URI_AGN_TYPE_MOD_AGN
from rdf_utils.namespace import (
    URL_MM_PROV_EXT_JSON,
    URL_MM_PROV_JSON,
)
from rdflib import Dataset, Graph, Literal, URIRef
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
GENERATION_DOCUMENT = "provenance.ld.json"
GRAPH_DSL = URIRef(f"{MSPROV}graph/dsl")
GRAPH_COORD_DSL = URIRef(f"{MSPROV}graph/coord-dsl")
GRAPH_MOTION_SPEC = URIRef(f"{MSPROV}graph/motion-spec")
SCHEMA_VERSION = 1

# What each package is, for the one agent node it gets. The commit is read from the checkout
# (motion-spec) or from the pinned repos file (stst); a wheel has neither.
_PACKAGES = {
    "motion_spec": ("motion-spec", "motion_spec", "https://github.com/secorolab/motion-spec"),
    "rdf_utils": ("rdf-utils", "rdf_utils", "https://github.com/secorolab/rdf-utils"),
    "rdflib": ("rdflib", "rdflib", "https://github.com/RDFLib/rdflib"),
    "stst": ("stst", None, "https://github.com/jsnyders/STSTv4"),
    "cmake": ("cmake", None, None),
}


def _slug(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "item"


def _prov_iri(identifier: str) -> str:
    kind, _, name = identifier.partition(":")
    if not name:
        kind, name = "id", identifier
    return f"{MSPROV_PREFIX}{_slug(kind)}/{_slug(name)}"


def prov_uri(identifier: str) -> str:
    """Canonical full provenance IRI for an agent/activity/run id.

    `run:<run-id>` is what makes the rec and consolidated graphs describe one run rather than
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


def run_entity_uri(run_id: str, slug: str) -> str:
    """Canonical IRI for a file entity belonging to one run.

    Run-scoped, so unioning two runs of one generation keeps their artefacts apart instead of
    collapsing them onto one node with conflicting locations and hashes.
    """
    return f"{MSPROV}entity/run/{_slug(run_id)}/{_slug(slug)}"


def _mtime(path: Path) -> datetime:
    return datetime.fromtimestamp(Path(path).stat().st_mtime, UTC)


def _version(distribution: str | None) -> str | None:
    if distribution is None:
        return None
    try:
        return importlib.metadata.version(distribution)
    except importlib.metadata.PackageNotFoundError:
        return None


def _stst_commit() -> str | None:
    """The STSTv4 revision motion_spec.repos pins."""
    repos = Path(__file__).resolve().parent.parent / "motion_spec.repos"
    match = re.search(r"STSTv4:.*?version:\s*(\S+)", repos.read_text(), re.DOTALL)
    return match.group(1) if match else None


def _cmake_version() -> str | None:
    text = run_tool("cmake", "--version")
    match = re.search(r"cmake version (\S+)", text or "")
    return match.group(1) if match else None


def add_package(graph: Graph, key: str) -> URIRef:
    """Declare one software package as the agent a step is associated with."""
    name, distribution, repository = _PACKAGES[key]
    agent = uri(f"agent:{key}")
    commit = None
    if key == "motion_spec":
        commit = git(Path(__file__).resolve().parent, "rev-parse", "HEAD")
    elif key == "stst":
        commit = _stst_commit()
    version = _cmake_version() if key == "cmake" else _version(distribution)
    load_pkg_prov(graph, agent, name, version=version, commit=commit, repository=repository)
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
    """Merge nodes into one named graph of the generation document, leaving the rest as written.

    The document is edited as JSON rather than round-tripped through rdflib: a re-serialization
    resolves the archive-relative `prov:atLocation` values `_organize_generation` writes back
    into absolute file IRIs.
    """
    data = (
        json.loads(document.read_text())
        if document.is_file()
        else {"schema_version": SCHEMA_VERSION, "@context": PROV_CONTEXT, "@graph": []}
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


def record_ir_generation(
    generated: Path,
    manifest: Path,
    ir: dict,
    targets: dict[str, Path],
    *,
    started: datetime,
    ended: datetime,
) -> None:
    """The transformation that turned the app manifest and its model graphs into the IR."""
    from motion_spec.rdf_parser.model import imported_models

    graph = Graph()
    activity = uri("activity:motion_spec_ir_generation")
    package = add_package(graph, "motion_spec")

    app_manifest = uri("entity:app_manifest")
    add_file_entity(graph, app_manifest, location=str(Path(manifest).resolve()))
    sources = [app_manifest]
    for index, imported in enumerate(imported_models(manifest)):
        source = uri(f"entity:imported_graph:{index}")
        add_file_entity(graph, source, location=imported)
        sources.append(source)

    generated_ids = []
    for identifier, path in targets.items():
        entity = uri(identifier)
        add_file_entity(graph, entity, location=str(path.resolve()), generated_at=_mtime(path))
        generated_ids.append(entity)

    load_transformation_prov(graph, activity, sources, generated_ids, package, started, ended)
    graph.add((uri("entity:motion_spec_ir"), PROV.wasDerivedFrom, app_manifest))

    for robot in (ir["composition"]["scene"] or {}).get("robots") or ():
        if robot.get("id"):
            agent = uri(f"agent:modelled:{robot['id']}")
            add_agent(graph, agent, (URI_AGN_TYPE_MOD_AGN,), robot["id"])
    write_generation_graph(generated / GENERATION_DOCUMENT, GRAPH_MOTION_SPEC, graph)


def record_code_generation(
    generated: Path, ir_path: Path, written: list[Path], *, started: datetime, ended: datetime
) -> None:
    """The transformation that turned the IR into the controller sources."""
    graph = Graph()
    activity = uri("activity:code_generation")
    package = add_package(graph, "motion_spec")
    for key in ("stst", "rdf_utils", "rdflib"):
        add_package(graph, key)

    source = uri("entity:motion_spec_ir")
    add_file_entity(graph, source, location=str(Path(ir_path).resolve()))
    controller = Path(written[0]).parent if written else generated / "controller"
    targets = []
    for path in written:
        entity = uri(f"entity:generated_{_relative_name(path, controller)}")
        add_file_entity(graph, entity, location=str(path.resolve()), generated_at=_mtime(path))
        targets.append(entity)
    load_transformation_prov(graph, activity, [source], targets, package, started, ended)
    write_generation_graph(generated / GENERATION_DOCUMENT, GRAPH_MOTION_SPEC, graph)


def _relative_name(path: Path, root: Path) -> str:
    path = Path(path)
    return path.relative_to(root).as_posix() if path.is_relative_to(root) else path.name


def record_build(
    generated: Path, controller: Path, executable: Path, *, started: datetime, ended: datetime
) -> None:
    """The transformation that compiled the generated sources into the controller executable."""
    document = generated / GENERATION_DOCUMENT
    dataset = read_generation_dataset(document)
    code_generation = uri("activity:code_generation")
    declared = set(dataset.subjects(PROV.wasGeneratedBy, code_generation))

    graph = Graph()
    activity = uri("activity:build")
    package = add_package(graph, "cmake")
    sources = [
        entity
        for path in sorted(item for item in controller.rglob("*") if item.is_file())
        if (entity := uri(f"entity:generated_{_relative_name(path, controller)}")) in declared
    ]
    target = uri("entity:controller_executable")
    add_file_entity(
        graph, target, location=str(executable.resolve()), generated_at=_mtime(executable)
    )
    load_transformation_prov(graph, activity, sources, [target], package, started, ended)
    write_generation_graph(document, GRAPH_MOTION_SPEC, graph)


# The activity that mints these -- the same one the generation provenance already names as the
# generator of the IR they are part of.
_DERIVATION_ACTIVITY = "activity:motion_spec_ir_generation"


def build_derivation_document(ir: dict) -> dict:
    """Declare every codegen-derived entity, and what it was derived from.

    A frame-log slot names its value by IRI. Two thirds of those values are minted during IR
    generation and have no node in the authored model, so without this graph their IRIs resolve
    to nothing and a run graph cannot make a statement about what the log recorded.
    """
    derivations = ir["communication"]["introspection"].get("derivations") or []
    return {
        "schema_version": SCHEMA_VERSION,
        "@context": PROV_CONTEXT,
        "@graph": [
            {
                "@id": entry["id"],
                "@type": entry["types"],
                f"prov:{entry['relation']}": {"@id": entry["parent"]},
                "prov:wasGeneratedBy": prov_uri(_DERIVATION_ACTIVITY),
            }
            for entry in derivations
        ],
    }


# OSLC Automation is where rec records a run's lifecycle: a state, and once complete, a verdict.
_OSLC_AUTO = "http://open-services.net/ns/auto#"
_RUN_STATUS = {
    ("queued", "unavailable"): "QUEUED",
    ("inProgress", "unavailable"): "RUNNING",
    ("complete", "passed"): "COMPLETED",
    ("complete", "failed"): "FAILED",
    ("complete", "error"): "INTERRUPTED",
    ("canceled", "unavailable"): "CANCELLED",
}


def rec_run_lifecycle(graph) -> dict:
    """The observed run's status and timestamps, read from a REC graph.

    REC exposes lifecycle only as RDF, so a consumer has to project it. Every key is None when
    the document states nothing -- a document written before the OSLC terms names no run here.
    """
    from rec.observers.graph_observer import run_node

    empty = {"status": None, "started_time": None, "completed_time": None}
    run = run_node(graph)
    if run is None:
        return empty
    state, verdict = (
        graph.value(run, URIRef(_OSLC_AUTO + "state")),
        graph.value(run, URIRef(_OSLC_AUTO + "verdict")),
    )
    key = tuple(
        str(term).removeprefix(_OSLC_AUTO) if term is not None else None
        for term in (state, verdict)
    )
    started = graph.value(run, PROV.startedAtTime)
    ended = graph.value(run, PROV.endedAtTime)
    return {
        "status": _RUN_STATUS.get(key),
        "started_time": str(started) if started is not None else None,
        "completed_time": str(ended) if ended is not None else None,
    }


def rec_run_lifecycle_from_file(path) -> dict:
    """`rec_run_lifecycle` for an archive on disk; empty when it does not exist.

    Through the workspace resolver, as every other JSON-LD read here: the REC context is a
    github.io URL, so an unresolved parse fetches it over the network -- twice per document,
    which a run list pays per row and an offline reader waits out.
    """
    from motion_spec_dsl.rdf_parser.manifest import install_metamodel_resolver

    path = Path(path)
    if not path.exists():
        return {"status": None, "started_time": None, "completed_time": None}
    install_metamodel_resolver()
    graph = Graph()
    graph.parse(path, format="json-ld")
    return rec_run_lifecycle(graph)


def parse_rec_time(value: str) -> datetime:
    parsed = datetime.fromisoformat(value)
    if parsed.tzinfo is None:
        return parsed.replace(tzinfo=timezone.utc)
    return parsed


def ensure_local_rec_importable() -> None:
    try:
        import rec

        # A bare `<ws>/src/rec` on sys.path imports as a namespace package (__file__ is
        # None) and has no Run; drop it so the real package below wins.
        if getattr(rec, "__file__", None):
            return
        del sys.modules["rec"]
    except ImportError:
        pass
    rec_root = local_rec_root()
    if rec_root:
        sys.path.insert(0, str(rec_root))


def runtime_agent_uri(platform_facts: dict) -> URIRef:
    """The runtime the controller process acts for, named by the platform the model authored."""
    return uri(f"agent:runtime_{_slug(platform_facts.get('name') or 'runtime').casefold()}")


CONTROLLER_PROCESS = "agent:controller_process"


def record_run_agents(graph: Graph, run_dir: Path, platform_facts: dict) -> URIRef:
    """The process that produced the run's logs, the runtime it ran on, and the modelled robots."""
    runtime = runtime_agent_uri(platform_facts)
    add_agent(graph, runtime, (PROV.SoftwareAgent,), platform_facts.get("name") or "runtime")
    controller = uri(CONTROLLER_PROCESS)
    add_agent(graph, controller, (PROV.SoftwareAgent,), "controller", acted_on_behalf_of=runtime)
    for agent_id, name in modelled_agents(run_dir):
        add_agent(graph, URIRef(agent_id), (URI_AGN_TYPE_MOD_AGN,), name)
    return controller


def record_used_file(run, run_iri: URIRef, path: Path, role: str, archive_path: str) -> URIRef:
    """One file the execution used; rec owns its checksum, size and qualified usage."""
    return run.add_resource(
        path,
        usage_activity=str(run_iri),
        usage_time=_mtime(path),
        title=role,
        archive_path=archive_path,
        sha256=artifact_sha256(path),
        size_bytes=artifact_size(path),
    )


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
GENERATED_ROLES = ("frame_log", "frame_log_health", "console", "sampling", "bag", "videos")


def record_files(run, run_dir: Path, manifest: dict, run_iri: str) -> None:
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
                gen_activity=run_iri,
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


def host_info(environment: dict | None = None) -> dict:
    # The interpreter identity that matters for reproducibility is its version (python);
    # sys.executable is just the local venv path — machine-specific and provenance-free.
    info = {"hostname": socket.gethostname(), "os": platform.platform(), "python": sys.version}
    # The environment file this run was launched under, and the variables it set that a build
    # and a run depend on: without them, "it worked on that host" names the host but not the
    # toolchain, the prefixes or the ROS distribution that produced the result.
    if environment:
        info["environment"] = environment
    return info


def dependencies() -> list[dict]:
    rows = []
    for name in ("motion_spec", "rec", "rdflib", "pyshacl"):
        try:
            rows.append({"name": name, "hasVersion": importlib.metadata.version(name)})
        except importlib.metadata.PackageNotFoundError:
            continue
    return rows


def repositories(run_dir: Path) -> list[dict]:
    """Portable repo provenance: name + commit + remote url."""
    rows = []
    seen: set[str] = set()

    def add(root: Path | None, name: str | None = None) -> None:
        if root is None or str(root) in seen:
            return
        seen.add(str(root))
        row = {"name": name or root.name, "commit": git(root, "rev-parse", "HEAD")}
        url = git(root, "remote", "get-url", "origin")
        if url:
            row["url"] = url
        rows.append(row)

    for path in (run_dir, Path(__file__).resolve()):
        add(git_root(path))
    add(local_rec_root(), name="rec")
    return rows


def local_rec_root() -> Path | None:
    for root in (Path.cwd(), *Path.cwd().parents, *Path(__file__).resolve().parents):
        candidate = root / "src" / "rec"
        if (candidate / "rec" / "__init__.py").exists():
            return candidate
    return None


def git_root(path: Path) -> Path | None:
    start = path if path.is_dir() else path.parent
    root = git(start, "rev-parse", "--show-toplevel")
    return Path(root) if root else None


def git(cwd: Path, *args: str) -> str | None:
    try:
        return subprocess.check_output(
            ["git", *args], cwd=cwd, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except Exception:
        return None


def run_tool(*command: str) -> str | None:
    try:
        return subprocess.check_output(command, text=True, stderr=subprocess.DEVNULL)
    except Exception:
        return None


def modelled_agents(run_dir: Path) -> list[tuple[str, str]]:
    """Every robot the generation modelled, as its agent IRI and name."""
    manifest_path = run_dir / "manifest.json"
    path = run_dir / GENERATION_DOCUMENT
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text())
        path = run_dir / manifest.get("files", {}).get("provenance", path)
    if not path.exists():
        return []
    try:
        dataset = read_generation_dataset(path)
    except Exception:
        return []
    return [
        (str(subject), str(dataset.value(subject, SDO.name) or ""))
        for subject in set(dataset.subjects(RDF.type, URI_AGN_TYPE_MOD_AGN))
    ]
