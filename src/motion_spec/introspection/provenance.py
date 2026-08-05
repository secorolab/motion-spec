# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Introspection provenance for generated artifacts, archives and recorded executions."""

from __future__ import annotations

import importlib.metadata
import hashlib
import json
import platform
import re
import socket
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

MSPROV = "https://secorolab.github.io/motion-spec/provenance/"
MSPROV_PREFIX = "msprov:"
PROV_AGENT = "http://www.w3.org/ns/prov#Agent"
PROV_SOFTWARE_AGENT = "http://www.w3.org/ns/prov#SoftwareAgent"
TYPE_PREFIXES = {
    "http://www.w3.org/ns/prov#": "prov:",
    "https://secorolab.github.io/metamodels/acceptance-criteria/bdd#": "bdd:",
    "https://secorolab.github.io/metamodels/agent#": "agn:",
    "https://secorolab.github.io/metamodels/observation#": "obs:",
    "https://secorolab.github.io/metamodels/execution-context#": "exec:",
}
METAMODEL_CONTEXTS = [
    "https://secorolab.github.io/metamodels/prov.json",
    "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/agent.json",
    "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/bdd.json",
    "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/observation.json",
    "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/execution-context.json",
]
TOOL_METADATA = {
    "agent:motion_spec_codegen": {
        "package": "motion-spec",
        "repository": "https://github.com/secorolab/motion-spec",
    },
    "agent:motion_spec_ir_gen": {
        "package": "motion-spec",
        "repository": "https://github.com/secorolab/motion-spec",
    },
    "agent:rdf_utils": {
        "package": "rdf-utils",
        "repository": "https://github.com/secorolab/rdf-utils",
    },
    "agent:rdflib": {
        "package": "rdflib",
        "repository": "https://github.com/RDFLib/rdflib",
    },
    "agent:pyshacl": {
        "package": "pyshacl",
        "repository": "https://github.com/RDFLib/pySHACL",
    },
    "agent:stst": {
        "version": "0.4.1",
        "repository": "https://github.com/jsnyders/STSTv4",
    },
}


def _prov_iri(identifier: str) -> str:
    kind, _, name = identifier.partition(":")
    if not name:
        kind, name = "id", identifier
    kind_slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", kind).strip("_") or "item"
    name_slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("_") or "item"
    return f"{MSPROV_PREFIX}{kind_slug}/{name_slug}"


# REC expands only the `rec:` and `prov:` prefixes it owns; every other CURIE would be stored
# verbatim as a bogus URIRef. Expand ours before handing types across the boundary.
_TYPE_IRI_BY_PREFIX = {curie: iri for iri, curie in TYPE_PREFIXES.items()}


def rec_types(types) -> list[str]:
    """Expand motion-spec CURIEs to full IRIs for a REC agent/activity type list."""
    expanded = []
    for value in [types] if isinstance(types, str) else list(types or ()):
        text = str(value)
        prefix, _, local = text.partition(":")
        base = _TYPE_IRI_BY_PREFIX.get(f"{prefix}:")
        expanded.append(f"{base}{local}" if base and local else text)
    return expanded


def prov_uri(identifier: str) -> str:
    """Canonical full provenance IRI for an agent/activity id."""
    if identifier.startswith(("http://", "https://")):
        return identifier
    if identifier.startswith(MSPROV_PREFIX):
        return MSPROV + identifier[len(MSPROV_PREFIX):]
    return MSPROV + _prov_iri(identifier)[len(MSPROV_PREFIX):]


def _location_iri(value: str | None) -> str | None:
    if not value:
        return None
    if value.startswith(("http://", "https://", "file://")):
        return value
    path = Path(value)
    if path.is_absolute():
        return path.resolve().as_uri()
    if path.parts[:1] == ("src",):
        for root in (Path.cwd(), *Path.cwd().parents):
            if (root / "src" / "motion-spec").is_dir():
                return (root / path).resolve().as_uri()
    for root in (Path.cwd(), *Path.cwd().parents):
        candidate = root / path
        if candidate.exists():
            return candidate.resolve().as_uri()
    return path.resolve().as_uri()


_VENDOR_MARKERS = (
    ("third_party/menagerie/", "menagerie"),
    ("src/mj_kdl_wrapper/assets/", "assets"),
    ("src/examples/assets/", "assets"),
)


def _vendor_model_ref(path: str) -> str:
    text = Path(path).as_posix()
    for marker, vendor in _VENDOR_MARKERS:
        pos = text.find(marker)
        if pos != -1:
            return f"{vendor}:{text[pos + len(marker):]}"
    return text


def _agent_types(types: list[str]) -> list[str]:
    result = list(types)
    has_agent = PROV_AGENT in result or "prov:Agent" in result
    has_software_agent = PROV_SOFTWARE_AGENT in result or "prov:SoftwareAgent" in result
    if has_software_agent and not has_agent:
        result.append(PROV_AGENT)
    return result


def _compact_type(type_id: str) -> str:
    for base, prefix in TYPE_PREFIXES.items():
        if type_id.startswith(base):
            return prefix + type_id[len(base) :]
    return type_id


def _tool_properties(agent_id: str) -> dict:
    metadata = TOOL_METADATA.get(agent_id, {})
    package = metadata.get("package")
    version = metadata.get("version")
    if version is None and package:
        try:
            version = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            version = None
    return {"hasVersion": version, "references": metadata.get("repository")}


def build_provenance_document(ir: dict, output_dir: Path) -> dict:
    prov = (ir.get("introspection") or {}).get("provenance", {})
    graph = []

    def add_node(identifier: str, types: list[str], **properties) -> str:
        node_id = _prov_iri(identifier)
        node = {"@id": node_id, "@type": [_compact_type(type_id) for type_id in types]}
        node.update({k: v for k, v in properties.items() if v is not None and v != []})
        graph.append(node)
        return node_id

    input_entity_ids = []
    for entity in prov.get("entities", []):
        properties = {
            "role": entity.get("role"),
            "atLocation": _location_iri(entity.get("path") or entity.get("source")),
            "wasGeneratedBy": _prov_iri(entity["wasGeneratedBy"])
            if entity.get("wasGeneratedBy")
            else None,
        }
        if entity.get("role") == "imported_model_graph":
            properties["references"] = (
                _prov_iri(entity["wasDerivedFrom"]) if entity.get("wasDerivedFrom") else None
            )
        elif entity.get("wasDerivedFrom"):
            properties["wasDerivedFrom"] = _prov_iri(entity["wasDerivedFrom"])
        entity_id = add_node(
            entity.get("id", "entity"),
            entity.get("types") or ["prov:Entity"],
            **properties,
        )
        if entity.get("role") != "motion_spec_ir":
            input_entity_ids.append(entity_id)

    artifact_names = [
        "frame_layout.json",
        "frame_layout.h",
        "frame_log.proto",
        "provenance.ld.json",
        "introspection_runtime.hpp",
        "introspect_model.hpp",
        "CMakeLists.txt",
        "ref_main.cpp",
        "headers/runtime.hpp",
        "headers/shared_state.hpp",
    ]
    artifact_names.extend(
        f"headers/{motion['id']}.hpp"
        for motion in (ir.get("unique_motions") or ir.get("motions", []))
        if motion.get("id")
    )
    if (output_dir / "fsm_ir.json").exists():
        artifact_names.append("fsm_ir.json")
    artifact_names.extend(path.name for path in sorted(output_dir.glob("*_fsm.hpp")))
    artifact_entities = {
        name: add_node(
            f"entity:generated_{name}",
            ["prov:Entity"],
            role=f"generated_{name}",
            atLocation=_location_iri(str(output_dir / name)),
            wasGeneratedBy=_prov_iri("activity:code_generation"),
        )
        for name in dict.fromkeys(artifact_names)
    }

    required_agents = {
        activity["wasAssociatedWith"]
        for activity in prov.get("activities", [])
        if activity.get("wasAssociatedWith")
    }
    emitted_agents = set()

    for activity in prov.get("activities", []):
        add_node(
            activity.get("id", "activity"),
            activity.get("types") or ["prov:Activity"],
            role=activity.get("role"),
            used=[_prov_iri(item) for item in activity.get("used", [])],
            wasAssociatedWith=_prov_iri(activity["wasAssociatedWith"])
            if activity.get("wasAssociatedWith")
            else None,
        )
    codegen_activity = add_node(
        "activity:code_generation",
        ["prov:Activity"],
        role="code_generation",
        used=input_entity_ids,
        wasAssociatedWith=_prov_iri("agent:motion_spec_codegen"),
    )
    add_node(
        "activity:build",
        ["prov:Activity"],
        role="build",
        used=list(artifact_entities.values()),
        wasInformedBy=codegen_activity,
        wasAssociatedWith=_prov_iri("agent:build_toolchain"),
    )

    for agent in prov.get("agents", []):
        emitted_agents.add(agent.get("id", "agent"))
        agent_id = agent.get("id", "agent")
        add_node(
            agent_id,
            _agent_types(agent.get("types") or [PROV_AGENT]),
            role=agent.get("role"),
            **({"has-agn-model": _vendor_model_ref(agent["model"])} if agent.get("model") else {}),
            actedOnBehalfOf=_prov_iri(agent["actedOnBehalfOf"])
            if agent.get("actedOnBehalfOf")
            else None,
            **_tool_properties(agent_id),
        )
    for agent_id in sorted(required_agents - emitted_agents):
        add_node(agent_id, [PROV_SOFTWARE_AGENT, PROV_AGENT])
    add_node(
        "agent:motion_spec_codegen",
        [PROV_SOFTWARE_AGENT, PROV_AGENT, "obs:ObservationProvider"],
        role="code_generator",
        **_tool_properties("agent:motion_spec_codegen"),
    )
    add_node(
        "agent:stst",
        [PROV_SOFTWARE_AGENT, PROV_AGENT],
        role="template_renderer",
        **_tool_properties("agent:stst"),
    )
    add_node(
        "agent:rdf_utils",
        [PROV_SOFTWARE_AGENT, PROV_AGENT],
        role="rdf_resolver",
        **_tool_properties("agent:rdf_utils"),
    )
    add_node(
        "agent:rdflib",
        [PROV_SOFTWARE_AGENT, PROV_AGENT],
        role="rdf_graph_parser",
        **_tool_properties("agent:rdflib"),
    )
    add_node("agent:build_toolchain", [PROV_SOFTWARE_AGENT, PROV_AGENT], role="build_toolchain")
    add_node(
        "agent:replay_process",
        [PROV_SOFTWARE_AGENT, PROV_AGENT],
        role="expected_replay_process",
    )
    add_node(
        "agent:dashboard_process",
        [PROV_SOFTWARE_AGENT, PROV_AGENT],
        role="expected_dashboard_process",
    )

    return {
        "schema_version": 1,
        "runtime_rdf_contract_version": 1,
        "@context": [*METAMODEL_CONTEXTS, {"msprov": MSPROV, "role": "msprov:role"}],
        "@graph": [
            {"@id": "msprov:bundle/static-provenance", "@type": "prov:Bundle"},
            *graph,
        ],
    }


# The activity that mints these -- the same one the static provenance already names as the
# generator of the IR they are part of.
_DERIVATION_ACTIVITY = "activity:motion_spec_ir_generation"


def build_derivation_document(ir: dict) -> dict:
    """Declare every codegen-derived entity, and what it was derived from.

    A frame-log slot names its value by IRI. Two thirds of those values are minted during IR
    generation and have no node in the authored model, so without this graph their IRIs resolve
    to nothing and a run graph cannot make a statement about what the log recorded.
    """
    derivations = (ir.get("introspection") or {}).get("derivations") or []
    return {
        "schema_version": 1,
        "@context": [*METAMODEL_CONTEXTS, {"msprov": MSPROV}],
        "@graph": [
            {
                "@id": entry["id"],
                "@type": [_compact_type(type_id) for type_id in entry["types"]],
                f"prov:{entry['relation']}": {"@id": entry["parent"]},
                "prov:wasGeneratedBy": _prov_iri(_DERIVATION_ACTIVITY),
            }
            for entry in derivations
        ],
    }


# REC records a run's state as an rdf:type on the run node, not as a status string. These
# are the terminal-and-transient states rec.observers.graph_observer.RUN_TYPES defines.
_REC_RUN_STATUS = {
    "QueuedRun": "QUEUED",
    "RunningRun": "RUNNING",
    "CompletedRun": "COMPLETED",
    "FailedRun": "FAILED",
    "InterruptedRun": "INTERRUPTED",
    "CancelledRun": "CANCELLED",
}
_REC_NS = "https://secorolab.github.io/metamodels/rec#"
_PROV_NS = "http://www.w3.org/ns/prov#"


def rec_run_lifecycle(graph) -> dict:
    """The observed run's status and timestamps, read from a REC graph.

    REC exposes lifecycle only as RDF -- an rdf:type drawn from its run-state vocabulary plus
    prov:startedAtTime / prov:endedAtTime -- so a consumer has to project it. Returns the keys
    callers need: `status`, `started_time`, `completed_time`; each is None when absent.
    """
    import rdflib

    run = next(graph.subjects(rdflib.URIRef(_REC_NS + "run-id"), None), None)
    if run is None:
        return {}
    status = next(
        (
            _REC_RUN_STATUS[local]
            for type_ in graph.objects(run, rdflib.RDF.type)
            if (local := str(type_).rsplit("#", 1)[-1]) in _REC_RUN_STATUS
        ),
        None,
    )
    started = graph.value(run, rdflib.URIRef(_PROV_NS + "startedAtTime"))
    ended = graph.value(run, rdflib.URIRef(_PROV_NS + "endedAtTime"))
    return {
        "status": status,
        "started_time": str(started) if started is not None else None,
        "completed_time": str(ended) if ended is not None else None,
    }


def rec_run_lifecycle_from_file(path) -> dict:
    """`rec_run_lifecycle` for an archive on disk; empty when it does not exist."""
    import rdflib

    path = Path(path)
    if not path.exists():
        return {}
    graph = rdflib.Graph()
    graph.parse(path, format="json-ld")
    return rec_run_lifecycle(graph)


def parse_rec_time(value: str) -> datetime:
    parsed = datetime.fromisoformat(value)
    if parsed.tzinfo is None:
        return parsed.replace(tzinfo=timezone.utc)
    return parsed


def ensure_local_rec_importable() -> None:
    try:
        import rec  # noqa: F401

        return
    except ImportError:
        pass
    rec_root = local_rec_root()
    if rec_root:
        sys.path.insert(0, str(rec_root))


def record_agents(run, run_dir: Path, schema: dict) -> None:
    """Register the run's agents.

    REC identifies an agent by IRI and describes it by rdf:type, so the kind of agent is
    carried in the type list rather than a separate role label.
    """
    runtime = schema.get("runtime_provenance", {})
    raw_runtime = runtime.get("runtime_agent_id") or "agent:runtime"
    runtime_agent = prov_uri(raw_runtime)
    # The model declares whether this platform is simulated; do not infer it from the agent id.
    runtime_type = (
        "exec:Simulation"
        if (schema.get("platform") or {}).get("simulated")
        else "prov:SoftwareAgent"
    )
    run.add_agent(runtime_agent, rec_types(["prov:SoftwareAgent", runtime_type]))
    run.add_agent(
        prov_uri(runtime.get("producer_agent_id") or "agent:controller_process"),
        rec_types(["prov:SoftwareAgent", "obs:ObservationProvider"]),
    )
    run.add_agent(
        prov_uri("agent:motion_spec_archive"),
        rec_types(["prov:SoftwareAgent", "obs:ObservationProvider"]),
    )
    for agent_id, agent_types in provenance_nodes(run_dir, "agn:ModelledAgent"):
        run.add_agent(
            agent_id,
            agent_types,
        )


def record_activities(run, schema: dict) -> None:
    """Register the run's activities and the agent each is associated with."""
    runtime = schema.get("runtime_provenance", {})
    execution_type = (
        "bdd:SimulatedExecution"
        if (schema.get("platform") or {}).get("simulated")
        else "bdd:ScenarioExecution"
    )
    run.add_activity(
        prov_uri(runtime.get("activity_id") or "activity:controller_execution"),
        rec_types(["prov:Activity", execution_type]),
        associated_with=prov_uri(
            runtime.get("producer_agent_id") or "agent:controller_process"
        ),
    )
    run.add_activity(
        prov_uri("activity:archive_creation"),
        rec_types(["prov:Activity"]),
        associated_with=prov_uri("agent:motion_spec_archive"),
    )


def record_files(run, run_dir: Path, manifest: dict, schema: dict) -> None:
    """Record manifest files and their integrity metadata as PROV entities."""
    generated_roles = {"frame_log", "frame_log_health"}
    labels = {"model_imports": "imported_model_graph", "sources": "source_model"}
    runtime_activity = prov_uri(
        schema.get("runtime_provenance", {}).get("activity_id") or "activity:controller_execution"
    )
    for role, value in sorted(manifest.get("files", {}).items()):
        if role == "rec" or not value:
            continue
        for rel in value if isinstance(value, list) else [value]:
            path = run_dir / rel
            if not path.exists():
                continue
            common = {
                "title": labels.get(role, role),
                "sha256": artifact_sha256(path),
                "size_bytes": artifact_size(path),
            }
            if role in generated_roles:
                run.add_artefact(rel, gen_activity=runtime_activity, **common)
            else:
                run.add_resource(rel, usage_activity=runtime_activity, **common)
def record_frame_log_health(run, run_dir: Path, manifest: dict) -> None:
    rel = manifest.get("files", {}).get("frame_log_health")
    if not rel:
        return
    path = run_dir / rel
    if not path.exists():
        return
    health = json.loads(path.read_text())
    for name in ("attempted_frames", "accepted_frames", "written_frames", "dropped_frames"):
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


def host_info() -> dict:
    # The interpreter identity that matters for reproducibility is its version (python);
    # sys.executable is just the local venv path — machine-specific and provenance-free.
    return {
        "hostname": socket.gethostname(),
        "os": platform.platform(),
        "python": sys.version,
    }


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


def provenance_nodes(run_dir: Path, type_id: str) -> list[tuple[str, list[str]]]:
    manifest_path = run_dir / "manifest.json"
    path = run_dir / "provenance" / "motion-spec.ld.json"
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text())
        path = run_dir / manifest.get("files", {}).get("provenance", path)
    if not path.exists():
        return []
    try:
        import rdflib

        graph = rdflib.Graph().parse(path, format="json-ld")
    except Exception:
        return []
    target = rdflib.URIRef(rec_types(type_id)[0])
    return [
        (str(subject), [str(value) for value in graph.objects(subject, rdflib.RDF.type)])
        for subject in graph.subjects(rdflib.RDF.type, target)
    ]
