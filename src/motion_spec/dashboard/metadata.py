# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""User annotations stay beside bundles without changing generated provenance."""

from __future__ import annotations

import json
import threading
from pathlib import Path

from motion_spec.dashboard import roots
from motion_spec.dashboard.roots import LAYOUT_REL, json_file

LOCK = threading.RLock()


def generation_of(path: Path) -> Path:
    """Reject directories that are neither generations nor their direct runs."""
    generation = path.parent.parent if path.parent.name == "runs" else path
    if not path.is_dir() or not (generation / LAYOUT_REL).is_file():
        raise ValueError("annotations belong to a generation or run")
    return generation


def model_folder(path: Path) -> Path:
    """Use the catalog's first path segment as the model identity."""
    generation = generation_of(path)
    return roots.GENERATIONS / generation.relative_to(roots.GENERATIONS).parts[0]


def write_document(path: Path, payload: dict) -> None:
    """Replace one annotation document atomically; callers serialize updates with LOCK."""
    temporary = path.with_suffix(path.suffix + ".writing")
    temporary.write_text(json.dumps(payload, indent=2) + "\n")
    temporary.replace(path)


def annotations(path: Path) -> dict:
    """Return user labels independently of generated facts and run-note tags."""
    stored = json_file(path / "dashboard.json")
    return {
        "label": stored.get("label", ""),
        "pinned": stored.get("pinned", False),
        "protected": stored.get("protected", False),
        "tags": stored.get("tags", []),
    }


def save_annotations(path: Path, changes: dict) -> dict:
    """Patch only supported user fields; preserve unrelated concurrent edits."""
    generation_of(path)
    if not isinstance(changes, dict) or changes.keys() - {"label", "pinned", "tags"}:
        raise ValueError("unknown annotation fields")
    if "label" in changes and (
        not isinstance(changes["label"], str) or len(changes["label"]) > 200
    ):
        raise ValueError("label must be text of at most 200 characters")
    if "pinned" in changes and type(changes["pinned"]) is not bool:
        raise ValueError("pinned must be boolean")
    if "tags" in changes and (
        not isinstance(changes["tags"], list)
        or len(changes["tags"]) > 20
        or any(not isinstance(tag, str) or len(tag) > 100 for tag in changes["tags"])
    ):
        raise ValueError("tags must be at most 20 strings of at most 100 characters")
    with LOCK:
        result = {**annotations(path), **changes}
        result["label"] = result["label"].strip()
        result["tags"] = list(dict.fromkeys(tag.strip() for tag in result["tags"] if tag.strip()))
        write_document(path / "dashboard.json", result)
        return result


def set_protected(path: Path, enabled: bool, confirm: str = "") -> dict:
    """Mark a bundle undeletable, and take that mark back only when its name is typed.

    Two layers rather than one: deleting refuses a protected bundle outright, and clearing the
    protection is a separate act that names what it clears, so no single click does both. Set
    independently of `pinned`, which a click toggles and which orders the catalog as well.
    """
    generation_of(path)
    if type(enabled) is not bool:
        raise ValueError("enabled must be boolean")
    if not enabled and (not isinstance(confirm, str) or confirm.strip() != path.name):
        raise ValueError(f"type '{path.name}' to remove its protection")
    with LOCK:
        result = {**annotations(path), "protected": enabled}
        write_document(path / "dashboard.json", result)
        return result


def baseline(path: Path) -> str | None:
    """One reference run per model, shared across its generations."""
    return json_file(model_folder(path) / "dashboard-model.json").get("baseline")


def set_baseline(path: Path, enabled: bool) -> dict:
    """A baseline must name an existing run and remains protected until cleared."""
    generation_of(path)
    if path.parent.name != "runs" or type(enabled) is not bool:
        raise ValueError("baseline must name a run and enabled must be boolean")
    relative = str(path.relative_to(roots.GENERATIONS))
    with LOCK:
        current = baseline(path)
        value = relative if enabled else (None if current == relative else current)
        write_document(model_folder(path) / "dashboard-model.json", {"baseline": value})
    return {"baseline": value}
