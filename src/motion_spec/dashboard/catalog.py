# SPDX-License-Identifier: MPL-2.0

"""What a generation is, read off the bundle: its facts, its sources, its runs.

Everything here comes from what the generation itself carries -- its contract, its model
graph, the files it vendored. The IR is the generator's, and is never read here.
"""

from __future__ import annotations

import json
import re
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

from google.protobuf.message import DecodeError
from rdflib import RDF, BNode, Literal

from motion_spec.dashboard import roots
from motion_spec.dashboard.graph import LIVE_GRAPH, MODEL_GRAPH, RUNTIME_GRAPH, deployed_devices
from motion_spec.dashboard.roots import LAYOUT_REL, directory_size, json_file, stamp_iso, trace
from motion_spec.dashboard.runs import GenerationInfo, RunInfo
from motion_spec.dashboard.sources import aligned_rows, authored_lines
from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.archive import ArchiveError
from motion_spec.introspection.replay import read_health, resolve_archive

# How REC says a run is over; anything else (QUEUED, RUNNING) means it still has work to do.
RUN_ENDED = {"COMPLETED", "FAILED", "INTERRUPTED", "CANCELLED"}


def _last_run(path: Path) -> dict | None:
    """How the newest run ended, or that it is still going -- the sidebar's one-glance answer."""
    runs = GenerationInfo(path).runs
    if not runs:
        return None
    newest = runs[0]
    return {"id": newest.run_id, "status": newest.status, "live": newest.is_live()}


def generation_info(path: Path) -> dict:
    layout = json_file(path / "generated/contract/frame_layout.json")
    source = next((item for item in (path / "generated/source").glob("*.robmot")), None)
    return {
        "path": str(path.relative_to(roots.GENERATIONS)),
        "name": GenerationInfo(path, roots.GENERATIONS).model,
        "created": stamp_iso(GenerationInfo(path).timestamp),
        "variant": (path.parent.name if path.parent.parent != roots.GENERATIONS else None),
        "built_at": datetime.fromtimestamp(GenerationInfo(path).built_at, timezone.utc).isoformat(),
        "source": source.name if source else None,
        "backend": layout.get("platform", {}).get("backend"),
        "platform": layout.get("platform", {}).get("name"),
        "simulated": layout.get("platform", {}).get("simulated"),
        "schema_hash": layout.get("schema_hash"),
        "size_bytes": directory_size(path),
        "pools": layout.get("pools", {}),
        "runs": len(GenerationInfo(path).runs),
        "has_fsm": any((path / "generated/model").glob("*_fsm.svg")),
        "cameras": generation_cameras(path),
        "last_run": _last_run(path),
    }


def build_toolchain(generation_dir: Path) -> dict:
    """What this generation was actually built against, read out of its own CMake cache.

    Per generation rather than per machine: an old bundle keeps the versions it was built
    with, and a build that never ran reports nothing rather than today's install.
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
    # The wrapper's exported config names the MuJoCo it links; the version is in that path.
    for exported in sorted(config_dir.glob("*.cmake")):
        mujoco = re.search(r"mujoco-(\d+\.\d+\.\d+)", exported.read_text())
        if mujoco:
            toolchain["mujoco"] = mujoco.group(1)
            break
    return toolchain


def drift_summary(generation_dir: Path) -> dict:
    """Every source this generation archived, and whether the working tree still matches it.

    Provenance records where each file was archived, not where it came from, so the working
    copy is found the way the model imported it -- by name, nearest the model first.
    """
    model_dir = generation_model_dir(generation_dir)
    files = []
    for archived in sorted((generation_dir / "generated/source").glob("*")):
        if not archived.is_file():
            continue
        twin = workspace_twin(archived.name, model_dir)
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

    The generation keeps what it was built from; the working tree has whatever it has been
    edited into. Comparing the two is the only honest answer to "would generating again give
    me this?". The sides are aligned by `aligned_rows`, so the page only draws rows.
    """
    source_dir = generation_dir / "generated/source"
    if name:
        archived = source_dir / Path(name).name  # a name, never a path out of the archive
        archived = archived if archived.is_file() else None
    else:
        archived = next(source_dir.glob("*.robmot"), None)
    if archived is None:
        raise ValueError("this generation archived no such source")
    twin = workspace_twin(archived.name, generation_model_dir(generation_dir))
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
        # Where it actually is, not a root the page has to remember and join a name onto.
        "workspace_path": str(roots.WORKSPACE / twin),
        "archived": str(archived),
        "name": archived.name,
        **aligned_rows(
            archived.read_text().splitlines(), (roots.WORKSPACE / twin).read_text().splitlines()
        ),
    }


def generation_model_dir(generation_dir: Path) -> str | None:
    """The working directory this generation's model was authored in, if it is still there.

    The contract records the absolute config path the run was generated from; its parent is the
    model's own directory, which is where most of a generation's sources came from.
    """
    config = json_file(generation_dir / LAYOUT_REL).get("platform", {}).get("config")
    if not config:
        return None
    folder = Path(config).parent
    try:
        return str(folder.relative_to(roots.WORKSPACE)) if folder.is_dir() else None
    except ValueError:
        return None  # authored outside the sources root


def workspace_twin(name: str, model_dir: str | None) -> str | None:
    """The working-tree file a vendored source came from, or None if it is not there any more.

    Nearest first, the way the model imported it: the file beside the model, then the folder
    above it, as `../shared.ktree` does. A handful of `is_file()` checks -- searching the tree
    by name would cost a walk per page and could only ever guess between two matches anyway.
    """
    if not model_dir:
        return None
    folder = Path(model_dir)
    while True:
        candidate = folder / name if str(folder) != "." else Path(name)
        if (roots.WORKSPACE / candidate).is_file():
            return str(candidate)
        if folder.parent == folder or str(folder) == ".":
            return None
        folder = folder.parent


def generation_details(path: Path) -> dict:
    """Add authored motion metadata without slowing the generation sidebar."""
    details = generation_info(path)
    source_files = sorted((path / "generated/source").glob("*"))
    robmot = next((source for source in source_files if source.suffix == ".robmot"), None)
    fsm = next((source for source in source_files if source.suffix == ".fsm"), None)
    description = re.search(r'description:\s*"([^"]+)"', fsm.read_text()) if fsm else None
    # A generation with no run yet has no log to read, so the count comes off the source.
    constraints = authored_lines(robmot.read_text()) if robmot else {}
    # A simulation is a platform with a name; hardware is the devices it deploys, which the
    # model graph names one by one. Reading them costs a graph parse, so it stays off the list.
    details["platform"] = details["platform"] or " · ".join(deployed_devices(path)) or None
    details["authored_constraints"] = len(constraints)
    details["motions"] = len({motion for motion, _name in constraints})
    details["folder"] = str(path)
    details["spec_name"] = Path(details["source"] or path.name).stem
    details["description"] = description.group(1) if description else None
    # A generation vendors a copy of what it was built from; the tree lists the working file
    # that copy came from. Name it here so the page can open the file that is still authored,
    # not the snapshot -- and say nothing where the working tree no longer has one.
    # Only the model needs the way back: it is what `gen` is pointed at again. The rest are
    # its imports, kept here as the record of what this generation was built from.
    model_dir = generation_model_dir(path)
    details["source_files"] = [
        {
            "name": source.name,
            "path": str(source),
            "model": source == robmot,
            "workspace": workspace_twin(source.name, model_dir) if source == robmot else None,
        }
        for source in source_files
        if source.is_file()
    ]
    details["toolchain"] = build_toolchain(path)
    details["generated_files"] = [
        {"name": str(source.relative_to(path / "generated")), "path": str(source)}
        for source in sorted((path / "generated").rglob("*"))
        if source.is_file() and "source" not in source.relative_to(path / "generated").parts
    ]
    return details


# Generated output runs to megabytes -- a 13 MB ir.json handed to an editor locks the browser
# -- so a large file arrives cut off and says so, rather than arriving whole and stopping the
# page, or arriving cut and pretending to be the file.
GENERATED_MAX_BYTES = 2_000_000

# Tab, newline and carriage return are the only control bytes text has any business carrying.
_TEXT_CONTROL = {9, 10, 13}


def _is_binary(head: bytes) -> bool:
    """Whether this looks like packed bytes rather than something to read.

    The usual test -- a NUL in the first block -- passes a serialized frame log, whose wire
    format is field tags and lengths in the low bytes with hardly a NUL among them. Counting
    every control byte catches it, and leaves real source (which has none) far below the line.
    """
    if not head:
        return False
    control = sum(1 for byte in head if byte < 32 and byte not in _TEXT_CONTROL)
    return b"\0" in head or control > len(head) * 0.02


def read_generated(path: Path) -> dict:
    """One generated artifact as text, for reading in the dashboard's editor."""
    size = path.stat().st_size
    with path.open("rb") as handle:
        head = handle.read(GENERATED_MAX_BYTES)
    if _is_binary(head[:8192]):
        return {"absolute": str(path), "size": size, "binary": True, "text": ""}
    return {
        "absolute": str(path),
        "size": size,
        "binary": False,
        # replace, not strict: the cut can land mid-character, and one glyph is not a failure
        "text": head.decode("utf-8", "replace"),
        "truncated": size > GENERATED_MAX_BYTES,
    }


def rdf_name(term: object) -> str:
    """Return the compact RDF name used in the dashboard."""
    return str(term).rsplit("/", 1)[-1].rsplit("#", 1)[-1]


GRAPH_NAMES = {str(MODEL_GRAPH): "model", str(RUNTIME_GRAPH): "runtime", str(LIVE_GRAPH): "live"}
PROV_NS = "http://www.w3.org/ns/prov#"


def _is_prov(term) -> bool:
    return str(term).startswith(PROV_NS)


def _term_kind(term) -> str:
    if isinstance(term, BNode):
        return "blank"
    return "provenance" if _is_prov(term) else "resource"


def graph_name(context) -> str:
    """A quad's named graph, named as compactly as it can be.

    `urn:model`, `urn:runtime` and `urn:live` are the dashboard's own three. A JSON-LD file that
    declares a graph of its own lands in that graph instead -- the whole FSM is one -- and
    calling those "model" too would hide the split from every reader downstream.
    """
    identifier = str(getattr(context, "identifier", context))
    return GRAPH_NAMES.get(identifier) or rdf_name(identifier.rstrip("/")) or identifier


def term_graphs(dataset) -> dict[str, list[str]]:
    """Term -> the named graphs it appears in, anywhere in the dataset.

    Membership is a property of the term, not of the triple that happened to name it: a design
    IRI in `model` and `runtime` was modelled *and* ran, and that stays true in a result that
    only carries its design triples.
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

    Three edge classes never reach the picture as edges, because as edges they are hubs that a
    force layout cannot separate: `rdf:type` becomes its subject's `types`, a literal object
    becomes its subject's `attributes`, and `prov:` terms stay but are marked for the overlay.
    `hidden` counts each one, so a reader can account for every triple that is not a link.
    """
    nodes: dict[str, dict] = {}
    links: list[dict] = []
    types: Counter = Counter()
    predicates: Counter = Counter()
    hidden = {"type_edges": 0, "literal_edges": 0, "provenance_edges": 0}

    quads = list(quads)
    # A blank node's label changes every time the live overlay regenerates; its place in the
    # graph does not. Reached from an IRI, it borrows that edge as its stable payload id.
    blank_ids: dict[str, str] = {}
    for subject, predicate, obj, _name in quads:
        if isinstance(obj, BNode) and not isinstance(subject, BNode):
            blank_ids.setdefault(str(obj), f"{subject}#{rdf_name(predicate)}")

    def node(term) -> dict:
        term_id = blank_ids.get(str(term), str(term)) if isinstance(term, BNode) else str(term)
        entry = nodes.get(term_id)
        if entry is None:
            entry = nodes[term_id] = {
                "id": term_id,
                "label": rdf_name(term),
                "value": str(term),
                "types": [],
                "attributes": {},
                "degree": 0,
                "kind": _term_kind(term),
                "graphs": list(graphs.get(str(term), [])),
            }
        return entry

    for subject, predicate, obj, name in quads:
        source = node(subject)
        if predicate == RDF.type:
            short = rdf_name(obj)
            source["types"].append(short)
            types[short] += 1
            hidden["type_edges"] += 1
            continue
        if isinstance(obj, Literal):
            source["attributes"].setdefault(rdf_name(predicate), []).append(str(obj))
            hidden["literal_edges"] += 1
            continue
        target = node(obj)
        kind = "provenance" if any(map(_is_prov, (subject, predicate, obj))) else "relation"
        links.append(
            {
                "source": source["id"],
                "target": target["id"],
                "label": rdf_name(predicate),
                "value": str(predicate),
                "kind": kind,
                "graph": name,
            }
        )
        source["degree"] += 1
        target["degree"] += 1
        predicates[rdf_name(predicate)] += 1
        if kind == "provenance":
            hidden["provenance_edges"] += 1
    return {
        "nodes": list(nodes.values()),
        "links": links,
        "types": dict(types),
        "predicates": dict(predicates),
        "hidden": hidden,
    }


def provenance_graph(service) -> dict:
    """Every term and triple of one dataset -- model, runtime and live -- classified."""
    service.sync()
    quads = (
        (subject, predicate, obj, graph_name(context))
        for subject, predicate, obj, context in service.dataset.quads((None, None, None, None))
    )
    return {
        **classify_quads(quads, term_graphs(service.dataset)),
        "runtime_source": service.runtime_source,
    }


def _run_notes(path: Path) -> list[str]:
    """Read notes.json here rather than importing queries, which imports this module: every tag
    used by the run's notes."""
    stored = json_file(path / "notes.json")
    notes = stored.get("notes") if isinstance(stored, dict) else None
    notes = [note for note in notes if isinstance(note, dict)] if isinstance(notes, list) else []
    return list(dict.fromkeys(str(tag) for note in notes for tag in (note.get("tags") or [])))


def run_info(path: Path) -> dict:
    log = frame_log_pb.log_path(path / "logs/frame_log.pb")
    health = read_health(log) or {}
    run_id = RunInfo(path).run_id
    # A run that has only just started has no readable log yet; it still belongs in the list.
    try:
        period_ns = resolve_archive(path)[3].header.nominal_period_ns
    except (ArchiveError, DecodeError, OSError):
        period_ns = 0
    match = re.fullmatch(r"run-(\d{8}T\d{6}\d{6}Z)", run_id)
    tags = _run_notes(path)
    return {
        "path": str(path.relative_to(roots.GENERATIONS)),
        "id": run_id,
        "started": stamp_iso(match.group(1)) if match else None,
        "complete": health.get("complete") or RunInfo(path).status == "COMPLETED",
        "status": RunInfo(path).status,
        "written_frames": health.get("written_frames"),
        "duration_s": health.get("written_frames", 0) * period_ns / 1e9,
        "dropped_frames": health.get("dropped_frames"),
        "tags": tags,
    }


def generation_cameras(generation_dir: Path) -> list[dict]:
    """The cameras this generation can record, as its contract names them."""
    return [
        {
            "id": camera["id"],
            "width": camera.get("width"),
            "height": camera.get("height"),
            # Where a viewer reads it, or nothing: the page shows no live pane without one.
            "topic": camera.get("topic"),
            "message": camera.get("message"),
        }
        for camera in json_file(generation_dir / LAYOUT_REL).get("cameras") or []
        if camera.get("id")
    ]


def is_simulated(generation_dir: Path) -> bool:
    """Whether this generation targets a simulator, which is what makes a display a choice."""
    return bool(json_file(generation_dir / LAYOUT_REL).get("platform", {}).get("simulated"))


def run_files(run_dir: Path) -> list[dict]:
    """Every file this run wrote, named relative to the run.

    The run's own tree and nothing beyond it: the generation-owned files its manifest points
    back to are the generation page's to list.
    """
    return [
        {"name": str(path.relative_to(run_dir)), "path": str(path), "size": path.stat().st_size}
        for path in sorted(run_dir.rglob("*"))
        if path.is_file()
    ]


def run_videos(run_dir: Path) -> list[str]:
    """The cameras this run recorded, named by their video beside the log."""
    return sorted(path.stem for path in (run_dir / "logs").glob("*.mp4"))


def video_file(run_dir: Path, camera: str) -> Path:
    path = (run_dir / "logs" / f"{camera}.mp4").resolve()
    if path.parent != (run_dir / "logs").resolve() or not path.is_file():
        raise ValueError(f"no recording for camera: {camera}")
    return path


def run_ended(run_dir: Path) -> bool:
    """Whether this run is written and marked: archived, and REC says how it ended."""
    if not (run_dir / "manifest.json").exists():
        # No manifest, no archive -- and no reason to pay the REC parse on every live poll.
        trace(f"run_ended {run_dir.name}: archived=False")
        return False
    status = RunInfo(run_dir).status
    trace(f"run_ended {run_dir.name}: archived=True status={status}")
    return status in RUN_ENDED


def run_recorded(run_dir: Path | None) -> bool | None:
    """Whether the run wrote a frame log, per its manifest. None until the manifest exists."""
    manifest = run_dir / "manifest.json" if run_dir else None
    if manifest is None or not manifest.is_file():
        return None
    try:
        return json.loads(manifest.read_text()).get("recorded", True)
    except (OSError, ValueError):
        return None
