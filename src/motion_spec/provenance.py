# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# SPDX-FileContributor: Vamsi Kalagaturu <vamsikalagaturu@gmail.com>
"""Provenance helpers for generated artifacts, run archives and cataloged executions."""

from __future__ import annotations

import importlib.metadata
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
    "https://secorolab.github.io/metamodels/runtime#": "rt:",
}
METAMODEL_CONTEXTS = [
    "https://secorolab.github.io/metamodels/prov.json",
    "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/agent.json",
    "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/bdd.json",
    "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/observation.json",
    "https://secorolab.github.io/metamodels/runtime/runtime.json",
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


def _slug(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "item"


def _prov_iri(identifier: str) -> str:
    kind, _, name = identifier.partition(":")
    if not name:
        kind, name = "id", identifier
    return f"{MSPROV_PREFIX}{_slug(kind)}/{_slug(name)}"


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


def _compact_types(types: list[str]) -> list[str]:
    return [_compact_type(type_id) for type_id in types]


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
        node = {"@id": node_id, "@type": _compact_types(types)}
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
        "schema.json",
        "frame_layout.json",
        "frame_layout.h",
        "frame_log.proto",
        "provenance.jsonld",
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
    runtime = schema.get("runtime_provenance", {})
    raw_runtime = runtime.get("runtime_agent_id") or "agent:runtime"
    runtime_agent = prov_uri(raw_runtime)
    runtime_type = "rt:MuJoCoRuntime" if raw_runtime.endswith(":mujoco") else "prov:SoftwareAgent"
    run.add_agent(runtime_agent, ["prov:SoftwareAgent", runtime_type], role="runtime")
    run.add_agent(
        prov_uri(runtime.get("producer_agent_id") or "agent:controller_process"),
        ["prov:SoftwareAgent", "obs:ObservationProvider"],
        role="log_producer",
        actedOnBehalfOf=runtime_agent,
    )
    run.add_agent(
        prov_uri("agent:motion_spec_archive"),
        ["prov:SoftwareAgent", "obs:ObservationProvider"],
        role="archive_writer",
    )
    for agent in provenance_nodes(run_dir, "agn:ModelledAgent"):
        run.add_agent(
            prov_uri(agent.get("@id", "agent:modelled")),
            agent.get("@type", ["prov:Agent", "agn:ModelledAgent"]),
            role=agent.get("role", "modelled_agent"),
        )


def record_activities(run, schema: dict) -> None:
    runtime = schema.get("runtime_provenance", {})
    run.add_activity(
        prov_uri(runtime.get("activity_id") or "activity:controller_execution"),
        ["prov:Activity", "bdd:SimulatedExecution"],
        role="controller_execution",
        wasAssociatedWith=prov_uri(runtime.get("producer_agent_id") or "agent:controller_process"),
    )
    run.add_activity(
        prov_uri("activity:archive_creation"),
        ["prov:Activity"],
        role="archive_creation",
        wasAssociatedWith=prov_uri("agent:motion_spec_archive"),
    )


def record_files(run, run_dir: Path, manifest: dict, schema: dict) -> None:
    resource_roles = {"schema", "frame_log_proto", "provenance", "dsl_provenance", "model", "ir"}
    runtime_activity = prov_uri(
        schema.get("runtime_provenance", {}).get("activity_id") or "activity:controller_execution"
    )
    for rel, meta in sorted(manifest.get("artifacts", {}).items()):
        path = run_dir / rel
        if not path.exists():
            continue
        row = {
            "path": str(path.resolve()),
            "archivePath": rel,
            "role": meta.get("role"),
            "sha256": meta.get("sha256"),
            "size_bytes": artifact_size(path),
        }
        if meta.get("role") in resource_roles:
            run.add_resource(rel, **row)
        else:
            gen_activity = (
                runtime_activity
                if meta.get("role") in {"frame_log", "frame_log_health"}
                else prov_uri("activity:archive_creation")
            )
            run.add_artefact(rel, gen_activity=gen_activity, **row)
    for stream in manifest.get("streams", []):
        row = {
            "path": stream.get("url"),
            "role": f"stream_{stream.get('kind', 'unknown')}",
            "id": stream.get("id"),
            "label": stream.get("label"),
        }
        if stream.get("mode") in {"mp4", "file"}:
            run.add_artefact(stream.get("url"), **row)
        else:
            run.add_resource(stream.get("url"), **row)


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


def provenance_nodes(run_dir: Path, type_id: str) -> list[dict]:
    path = run_dir / "provenance" / "codegen.jsonld"
    if not path.exists():
        return []
    try:
        nodes = json.loads(path.read_text()).get("@graph", [])
    except Exception:
        return []
    return [
        node
        for node in nodes
        if type_id
        in (node.get("@type") if isinstance(node.get("@type"), list) else [node.get("@type")])
    ]
