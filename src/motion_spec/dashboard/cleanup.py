# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Preview protected bundles and restore only trash belonging to the current root."""

from __future__ import annotations

import subprocess
from pathlib import Path

from motion_spec.dashboard import roots
from motion_spec.dashboard.jobs import run_status
from motion_spec.dashboard.metadata import annotations, baseline, generation_of
from motion_spec.dashboard.roots import directory_size, relative_path
from motion_spec.dashboard.runs import GenerationInfo, RunInfo


def protection(path: Path) -> str | None:
    """Include descendant pins and pending processes, not just recent log writes."""
    generation = generation_of(path)
    runs = [run.dir for run in GenerationInfo(generation).runs] if path == generation else [path]
    if run_status(generation).get("running") or any(RunInfo(run).is_live() for run in runs):
        return "active run"
    if annotations(path)["pinned"] or any(annotations(run)["pinned"] for run in runs):
        return "pinned generation or run"
    reference = baseline(path)
    if reference and (roots.GENERATIONS / reference) in [path, *runs]:
        return "model baseline"
    return None


def preview(paths: list[str]) -> dict:
    """Resolve and deduplicate the selection before counting or deleting anything."""
    if not isinstance(paths, list) or not paths or any(not isinstance(p, str) for p in paths):
        raise ValueError("select generation or run paths")
    targets = list(dict.fromkeys(relative_path(roots.GENERATIONS, value) for value in paths))
    for path in targets:
        try:
            generation_of(path)
        except ValueError as error:
            raise ValueError("only generation bundles and run archives can be deleted") from error
    targets = [path for path in targets if not any(other in path.parents for other in targets)]
    items = []
    for path in targets:
        generation = generation_of(path)
        items.append(
            {
                "path": str(path.relative_to(roots.GENERATIONS)),
                "bytes": directory_size(path),
                "runs": len(GenerationInfo(path).runs) if path == generation else 1,
                "blocked": protection(path),
            }
        )
    return {
        "items": items,
        "bytes": sum(item["bytes"] for item in items),
        "runs": sum(item["runs"] for item in items),
    }


def trash_entries() -> list[dict]:
    """Filter the desktop trash by original location before exposing any entry."""
    result = subprocess.run(["gio", "trash", "--list"], capture_output=True, text=True, check=False)
    if result.returncode:
        raise ValueError(f"could not list Trash: {result.stderr.strip()}")
    entries = []
    for line in result.stdout.splitlines():
        uri, separator, original = line.partition("\t")
        if not separator:
            continue
        path = Path(original).resolve()
        if roots.GENERATIONS.resolve() in path.parents:
            entries.append(
                {
                    "uri": uri,
                    "path": str(path.relative_to(roots.GENERATIONS.resolve())),
                    "exists": path.exists(),
                }
            )
    return entries


def restore(uri: str) -> dict:
    """Revalidate the trash URI and refuse collisions instead of overwriting files."""
    entry = next((entry for entry in trash_entries() if entry["uri"] == uri), None)
    if entry is None or entry["exists"]:
        raise ValueError("unknown Trash entry or original path already exists")
    result = subprocess.run(
        ["gio", "trash", "--restore", "--", uri], capture_output=True, text=True, check=False
    )
    if result.returncode:
        raise ValueError(f"could not restore: {result.stderr.strip()}")
    return entry
