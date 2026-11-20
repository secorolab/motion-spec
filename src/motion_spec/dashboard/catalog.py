# SPDX-License-Identifier: MPL-2.0

"""What a generation is, read off the bundle: its facts, its sources, its runs.

Everything here comes from what the generation itself carries -- its contract, its model
graph, the files it vendored. The IR is the generator's, and is never read here.
"""

from __future__ import annotations

import re
from collections import Counter
from datetime import UTC, datetime
from pathlib import Path

from coord_dsl.rdf.vocab import URI_FSM_PRED_DESCRIPTION
from google.protobuf.message import DecodeError
from rdf_utils.models.prov import get_file_path
from rdflib import RDF, BNode, Dataset, Literal
from rdflib.namespace import PROV, TIME
from rdflib.term import Node
from rec import Verdict

from motion_spec.dashboard import roots
from motion_spec.dashboard.graph import deployed_devices
from motion_spec.dashboard.metadata import annotations, model_folder
from motion_spec.dashboard.roots import STATE_REL, directory_size, json_file, trace
from motion_spec.dashboard.runs import ENDED, GenerationInfo, RunInfo
from motion_spec.dashboard.sources import aligned_rows, authored_lines
from motion_spec.rdf_parser.agents import agent_cameras
from motion_spec.rdf_parser.deployment import platform_facts
from motion_spec.runs.provenance import (
    FRAME_LOG_PACKED,
    GENERATION_DOCUMENT,
    generation_model,
    kept_sources,
    read_generation_dataset,
    rec_run,
    run_outputs,
)
from motion_spec.runs.replay import resolve_run
from motion_spec.runs.verify import RecordError


def generation_info(path: Path) -> dict:
    layout = GenerationInfo(path, roots.GENERATIONS).layout
    provenance = read_generation_dataset(path / "generated" / GENERATION_DOCUMENT)
    began = provenance.value(provenance.value(predicate=PROV.hadActivity), TIME.hasBeginning)
    created = provenance.value(began, TIME.inXSDDateTimeStamp)
    platform = generation_platform(path)
    source = next((path for path in kept_sources(provenance) if path.suffix == ".robmot"), None)
    generation = GenerationInfo(path, roots.GENERATIONS)
    runs = generation.runs
    return {
        **annotations(path),
        "note_tags": run_notes(path),
        "baseline": json_file(model_folder(path) / "dashboard-model.json").get("baseline"),
        "path": str(path.relative_to(roots.GENERATIONS)),
        "name": generation.model,
        "created": str(created) if created is not None else None,
        "variant": (path.parent.name if path.parent.parent != roots.GENERATIONS else None),
        "built_at": datetime.fromtimestamp(generation.built_at, UTC).isoformat(),
        "source": source.name if source else None,
        "backend": platform.get("backend"),
        "platform": platform.get("name"),
        "simulated": platform.get("simulated"),
        "schema_hash": layout.schema_hash,
        "size_bytes": directory_size(path),
        "pools": layout.pools,
        "runs": len(runs),
        "cameras": generation_cameras(path),
        "last_run": (
            {
                "id": runs[0].run_id,
                "state": runs[0].state,
                "verdict": runs[0].verdict,
                "live": runs[0].is_live(),
            }
            if runs
            else None
        ),
    }


def build_toolchain(generation_dir: Path) -> dict:
    """What this generation was actually built against, read out of its own CMake cache.

    A generation never built reports nothing.
    """
    cache = generation_dir / "build" / "CMakeCache.txt"
    if not cache.is_file():
        return {}
    found = re.search(r"^mj_kdl_wrapper_DIR:PATH=(.+)$", cache.read_text(), re.MULTILINE)
    if not found:
        return {}
    config_dir = Path(found.group(1).strip())
    toolchain = {}
    version_file = config_dir / "mj_kdl_wrapperConfigVersion.cmake"
    if version_file.is_file():
        version = re.search(r'set\(PACKAGE_VERSION\s+"([^"]+)"', version_file.read_text())
        if version:
            toolchain["wrapper"] = version.group(1)
    # the linked MuJoCo's version is in the wrapper config's paths
    for exported in config_dir.glob("*.cmake"):
        mujoco = re.search(r"mujoco-(\d+\.\d+\.\d+)", exported.read_text())
        if mujoco:
            toolchain["mujoco"] = mujoco.group(1)
            break
    return toolchain


def drift_summary(generation_dir: Path) -> dict:
    """Every source this generation archived, and whether the working tree still matches it."""
    generated = generation_dir / "generated"
    provenance = read_generation_dataset(generated / GENERATION_DOCUMENT)
    files = []
    for archived, copy in kept_sources(provenance).items():
        if not archived.is_file():
            continue
        twin = workspace_twin(generated, provenance, copy)
        if twin is None:
            status = "missing"
        else:
            status = (
                "same"
                if (roots.WORKSPACE / twin).read_bytes() == archived.read_bytes()
                else "changed"
            )
        files.append(
            {
                "name": archived.name,
                "workspace": twin,
                "status": status,
                "model": archived.suffix == ".robmot",
            }
        )
    return {"files": files}


def source_drift(generation_dir: Path, name: str | None = None) -> dict:
    """What one archived source has been edited into since this generation was made.

    The sides are aligned by `aligned_rows`, so the page only draws rows.
    """
    generated = generation_dir / "generated"
    provenance = read_generation_dataset(generated / GENERATION_DOCUMENT)
    kept = kept_sources(provenance)
    archived = next(
        (
            path
            for path in kept
            if path.is_file()
            and (path.name == Path(name).name if name else path.suffix == ".robmot")
        ),
        None,
    )
    if archived is None:
        raise ValueError("this generation archived no such source")
    twin = workspace_twin(generated, provenance, kept[archived])
    if twin is None:
        return {
            "name": archived.name,
            "workspace": None,
            "archived": str(archived),
            "same": False,
            "rows": [],
        }
    return {
        "workspace": twin,
        "workspace_path": str(roots.WORKSPACE / twin),
        "archived": str(archived),
        "name": archived.name,
        **aligned_rows(
            archived.read_text().splitlines(), (roots.WORKSPACE / twin).read_text().splitlines()
        ),
    }


def workspace_twin(generated: Path, provenance: Dataset, copy: Node) -> str | None:
    """The working-tree file under the sources root that the kept source COPY is an alternate
    of, or None if it is not there any more.

    Each original is located by the bundle of the tool that used it: scene-dsl's and
    coord-dsl's are in PROVENANCE, the generation's document; the DSL's is in the DSL's own
    document, one of the model documents of GENERATED.
    """
    model = generation_model(generated).graph
    for original in provenance.objects(copy, PROV.alternateOf):
        for graph in (provenance, model):
            path = get_file_path(graph, original)
            if path is not None and path.is_file() and path.is_relative_to(roots.WORKSPACE):
                return str(path.relative_to(roots.WORKSPACE))
    return None


def generation_details(path: Path) -> dict:
    """Add authored motion metadata without slowing the generation sidebar."""
    details = generation_info(path)
    generated = path / "generated"
    provenance = read_generation_dataset(generated / GENERATION_DOCUMENT)
    kept = kept_sources(provenance)
    robmot = next((source for source in kept if source.suffix == ".robmot"), None)
    try:
        description = generation_model(generated).graph.value(predicate=URI_FSM_PRED_DESCRIPTION)
    except ValueError:
        description = None
    constraints = authored_lines(robmot) if robmot else {}
    details["platform"] = details["platform"] or " · ".join(deployed_devices(path)) or None
    details["authored_constraints"] = len(constraints)
    details["motions"] = len({motion for motion, _name in constraints})
    details["folder"] = str(path)
    details["spec_name"] = Path(details["source"] or path.name).stem
    details["description"] = str(description) if description is not None else None
    details["source_files"] = [
        {
            "name": source.name,
            "path": str(source),
            "model": source == robmot,
            "workspace": workspace_twin(generated, provenance, copy) if source == robmot else None,
        }
        for source, copy in kept.items()
        if source.is_file()
    ]
    details["toolchain"] = build_toolchain(path)
    details["generated_files"] = [
        {"name": str(source.relative_to(generated)), "path": str(source)}
        for source in generated.rglob("*")
        if source.is_file() and source.resolve() not in kept
    ]
    return details


# a larger file locks the browser's editor
GENERATED_MAX_BYTES = 2_000_000

_TEXT_CONTROL = {9, 10, 13}


def read_generated(path: Path) -> dict:
    """One generated artifact as text, for reading in the dashboard's editor."""
    size = path.stat().st_size
    with path.open("rb") as handle:
        head = handle.read(GENERATED_MAX_BYTES)
    # a protobuf log has few NULs, so count every control byte
    block = head[:8192]
    control = sum(1 for byte in block if byte < 32 and byte not in _TEXT_CONTROL)
    if b"\0" in block or control > len(block) * 0.02:
        return {"absolute": str(path), "size": size, "binary": True, "text": ""}
    return {
        "absolute": str(path),
        "size": size,
        "binary": False,
        # the cut can land mid-character
        "text": head.decode("utf-8", "replace"),
        "truncated": size > GENERATED_MAX_BYTES,
    }


def graph_name(context) -> str:
    """A quad's named graph, named as compactly as it can be."""
    identifier = str(context)
    return identifier.rstrip("/").rsplit("/", 1)[-1].rsplit("#", 1)[-1] or identifier


def term_graphs(dataset) -> dict[str, list[str]]:
    """Term -> the named graphs it appears in, anywhere in the dataset.

    Membership is a property of the term, not of the triple that names it.
    """
    where: dict[str, list[str]] = {}
    for subject, _p, obj, context in dataset.quads((None, None, None, None)):
        name = graph_name(context)
        for term in (subject, obj):
            seen = where.setdefault(str(term), [])
            if name not in seen:
                seen.append(name)
    return where


def classify_quads(quads, graphs: dict[str, list[str]]) -> dict:
    """Classify (subject, predicate, object, graph name) tuples so the renderer can draw them.

    `rdf:type` becomes its subject's `types`, a literal object its subject's `attributes`, and
    `prov:` terms stay but are marked for the overlay; `hidden` counts each.
    """
    nodes: dict[str, dict] = {}
    links: list[dict] = []
    types: Counter = Counter()
    predicates: Counter = Counter()
    hidden = {"type_edges": 0, "literal_edges": 0, "provenance_edges": 0}

    quads = list(quads)
    # blank labels change per regeneration: key a blank node by the edge reaching it
    blank_ids: dict[str, str] = {}
    for subject, predicate, obj, _name in quads:
        if isinstance(obj, BNode) and not isinstance(subject, BNode):
            blank_ids.setdefault(
                str(obj), f"{subject}#{str(predicate).rsplit('/', 1)[-1].rsplit('#', 1)[-1]}"
            )

    for subject, predicate, obj, name in quads:
        ends = []
        linked = predicate != RDF.type and not isinstance(obj, Literal)
        for term in (subject, obj) if linked else (subject,):
            term_id = blank_ids.get(str(term), str(term)) if isinstance(term, BNode) else str(term)
            if term_id not in nodes:
                nodes[term_id] = {
                    "id": term_id,
                    "label": str(term).rsplit("/", 1)[-1].rsplit("#", 1)[-1],
                    "value": str(term),
                    "types": [],
                    "attributes": {},
                    "degree": 0,
                    "kind": (
                        "blank"
                        if isinstance(term, BNode)
                        else "provenance"
                        if term in PROV
                        else "resource"
                    ),
                    "graphs": graphs.get(str(term), []),
                }
            ends.append(nodes[term_id])
        source = ends[0]
        if predicate == RDF.type:
            short = str(obj).rsplit("/", 1)[-1].rsplit("#", 1)[-1]
            source["types"].append(short)
            types[short] += 1
            hidden["type_edges"] += 1
            continue
        label = str(predicate).rsplit("/", 1)[-1].rsplit("#", 1)[-1]
        if isinstance(obj, Literal):
            source["attributes"].setdefault(label, []).append(str(obj))
            hidden["literal_edges"] += 1
            continue
        target = ends[1]
        provenance = any(term in PROV for term in (subject, predicate, obj))
        kind = "provenance" if provenance else "relation"
        links.append(
            {
                "source": source["id"],
                "target": target["id"],
                "label": label,
                "value": str(predicate),
                "kind": kind,
                "graph": name,
            }
        )
        source["degree"] += 1
        target["degree"] += 1
        predicates[label] += 1
        if kind == "provenance":
            hidden["provenance_edges"] += 1
    return {
        "nodes": list(nodes.values()),
        "links": links,
        "types": dict(types),
        "predicates": dict(predicates),
        "hidden": hidden,
    }


def run_notes(path: Path) -> list[str]:
    """Every tag used by the run's notes."""
    state = roots.GENERATIONS / STATE_REL / path.relative_to(roots.GENERATIONS)
    notes = json_file(state / "notes.json").get("notes", [])
    return list(dict.fromkeys(tag for note in notes for tag in note.get("tags") or []))


def run_info(path: Path) -> dict:
    info = RunInfo(path)
    health = info.health or {}
    try:
        period_ns = resolve_run(path)[2].header.nominal_period_ns
    except (RecordError, DecodeError, OSError):
        period_ns = 0
    run = rec_run(path)
    record = run.store.load(run.id) or Dataset()
    record.default_union = True
    started = record.value(record.value(run.node, TIME.hasBeginning), TIME.inXSDDateTimeStamp)
    labels = annotations(path)
    notes = json_file(
        roots.GENERATIONS / STATE_REL / path.relative_to(roots.GENERATIONS) / "notes.json"
    ).get("notes", [])
    return {
        **labels,
        "path": str(path.relative_to(roots.GENERATIONS)),
        "id": info.run_id,
        "started": str(started) if started is not None else None,
        "complete": health.get("complete") or info.verdict is Verdict.PASSED,
        "state": info.state,
        "verdict": info.verdict,
        "live": info.is_live(),
        "written_frames": health.get("written_frames"),
        "duration_s": health.get("written_frames", 0) * period_ns / 1e9,
        "dropped_frames": health.get("dropped_frames"),
        "tags": list(dict.fromkeys((*labels["tags"], *run_notes(path)))),
        "notes_text": "\n".join(note.get("text", "") for note in notes),
    }


def generation_platform(generation_dir: Path) -> dict:
    """What this generation is deployed on, as its model states it; empty while the generation
    has not yet written its model."""
    try:
        return platform_facts(generation_model(generation_dir / "generated").graph)
    except ValueError:
        return {}


def generation_cameras(generation_dir: Path) -> list[dict]:
    """The cameras this generation can record, as its model's agents carry them."""
    try:
        cameras = agent_cameras(generation_model(generation_dir / "generated"))
    except ValueError:
        return []
    return [
        {
            "id": camera.id,
            "width": camera.width,
            "height": camera.height,
            "topic": camera.topic,
            "message": camera.message,
        }
        for camera in cameras
    ]


def run_files(run_dir: Path) -> list[dict]:
    """Every file this run wrote, named relative to the run.

    The run's own tree only, not the generation files its manifest points to.
    """
    return [
        {"name": str(path.relative_to(run_dir)), "path": str(path), "size": path.stat().st_size}
        for path in run_dir.rglob("*")
        if path.is_file()
    ]


def video_file(run_dir: Path, camera: str) -> Path:
    path = (run_dir / "logs" / f"{camera}.mp4").resolve()
    if path.parent != (run_dir / "logs").resolve() or not path.is_file():
        raise ValueError(f"no recording for camera: {camera}")
    return path


def run_ended(run_dir: Path) -> bool:
    """Whether rec records the run's end, which the runner states only once it is archived."""
    state = RunInfo(run_dir).state
    trace(f"run_ended {run_dir.name}: state={state}")
    return state in ENDED


def run_recorded(run_dir: Path | None) -> bool | None:
    """Whether the ended run's provenance records a frame log it generated; None until it ended."""
    if run_dir is None or not run_ended(run_dir):
        return None
    log = (run_dir / "logs" / FRAME_LOG_PACKED).resolve()
    return log in [output.resolve() for output in run_outputs(run_dir)]
