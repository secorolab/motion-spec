# SPDX-License-Identifier: MPL-2.0

"""What a generation is, read off the bundle: its facts, its sources, its runs.

Everything here comes from what the generation itself carries -- its contract, its model
graph, the files it vendored. The IR is the generator's, and is never read here.
"""

from __future__ import annotations

import difflib
import json
import re
from datetime import datetime, timezone
from pathlib import Path

from google.protobuf.message import DecodeError
from rdflib import Dataset

from motion_spec.dashboard import roots
from motion_spec.dashboard.graph import deployed_devices
from motion_spec.dashboard.roots import LAYOUT_REL, directory_size, json_file, stamp_iso, trace
from motion_spec.dashboard.runs import GenerationInfo, RunInfo
from motion_spec.dashboard.sources import authored_lines
from motion_spec.introspection.archive import ArchiveError
from motion_spec.introspection.replay import read_health, resolve_archive

# How REC says a run is over; anything else (QUEUED, RUNNING) means it still has work to do.
RUN_ENDED = {"COMPLETED", "FAILED", "INTERRUPTED", "CANCELLED"}


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
    me this?". The sides are aligned here, where difflib is, so the page only draws rows.
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
    was = archived.read_text().splitlines()
    now = (roots.WORKSPACE / twin).read_text().splitlines()
    rows, changed = [], False
    for kind, left_from, left_to, right_from, right_to in difflib.SequenceMatcher(
        None, was, now, autojunk=False
    ).get_opcodes():
        left = list(range(left_from, left_to))
        right = list(range(right_from, right_to))
        changed = changed or kind != "equal"
        # A replaced block pairs line for line, and the shorter side runs out into blanks
        # rather than shifting everything below it out of step with the other column.
        for index in range(max(len(left), len(right))):
            here = left[index] if index < len(left) else None
            there = right[index] if index < len(right) else None
            rows.append(
                {
                    "kind": kind,
                    "left": None if here is None else {"n": here + 1, "text": was[here]},
                    "right": None if there is None else {"n": there + 1, "text": now[there]},
                }
            )
    return {
        "workspace": twin,
        # Where it actually is, not a root the page has to remember and join a name onto.
        "workspace_path": str(roots.WORKSPACE / twin),
        "archived": str(archived),
        "name": archived.name,
        "same": not changed,
        "rows": rows,
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
    generated = path / "generated"
    details["rdf_graphs"] = sorted(
        str(source.relative_to(generated)) for source in generated.rglob("*.ld.json")
    )
    return details


def rdf_name(term: object) -> str:
    """Return the compact RDF name used in the dashboard."""
    return str(term).rsplit("/", 1)[-1].rsplit("#", 1)[-1]


def provenance_graph(path: Path, selected: list[str]) -> dict:
    """Return every RDF term and triple for the browser WebGL renderer."""
    graph = Dataset()
    generated = (path / "generated").resolve()
    for name in selected:
        source = (generated / name).resolve()
        if generated not in source.parents or source.suffix != ".json" or not source.is_file():
            raise ValueError("unknown RDF graph")
        graph.parse(source, format="json-ld")
    triples = [
        (subject, predicate, obj)
        for subject, predicate, obj, _context in graph.quads((None, None, None, None))
    ]
    terms = sorted(
        {term for subject, _predicate, obj in triples for term in (subject, obj)}, key=str
    )
    node_ids = {term: str(index) for index, term in enumerate(terms)}
    return {
        "nodes": [
            {"id": node_ids[term], "label": rdf_name(term), "value": str(term)} for term in terms
        ],
        "links": [
            {
                "source": node_ids[subject],
                "target": node_ids[obj],
                "label": rdf_name(predicate),
                "value": str(predicate),
            }
            for subject, predicate, obj in triples
        ],
    }


def run_info(path: Path) -> dict:
    log = path / "logs/frame_log.pb"
    health = read_health(log) or {}
    run_id = RunInfo(path).run_id
    # A run that has only just started has no readable log yet; it still belongs in the list.
    try:
        period_ns = resolve_archive(path)[3].header.nominal_period_ns
    except (ArchiveError, DecodeError, OSError):
        period_ns = 0
    match = re.fullmatch(r"run-(\d{8}T\d{6}\d{6}Z)", run_id)
    return {
        "path": str(path.relative_to(roots.GENERATIONS)),
        "id": run_id,
        "started": stamp_iso(match.group(1)) if match else None,
        "complete": health.get("complete") or RunInfo(path).status == "COMPLETED",
        "status": RunInfo(path).status,
        "written_frames": health.get("written_frames"),
        "duration_s": health.get("written_frames", 0) * period_ns / 1e9,
        "dropped_frames": health.get("dropped_frames"),
    }


def generation_cameras(generation_dir: Path) -> list[dict]:
    """The cameras this generation can record, as its contract names them."""
    return [
        {"id": camera["id"], "width": camera.get("width"), "height": camera.get("height")}
        for camera in json_file(generation_dir / LAYOUT_REL).get("cameras") or []
        if camera.get("id")
    ]


def is_simulated(generation_dir: Path) -> bool:
    """Whether this generation targets a simulator, which is what makes a display a choice."""
    return bool(json_file(generation_dir / LAYOUT_REL).get("platform", {}).get("simulated"))


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
