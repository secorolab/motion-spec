# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The publishing convention every IR record follows: which fields are construction-only,
how a dataclass tree serializes, and how a list of records dedupes by id.
"""

from __future__ import annotations

import json
from dataclasses import fields, is_dataclass

# Marks a field as a construction input rather than part of the published IR: ir_gen reads
# it off the live object, ir.json leaves it out. New fields default to internal until a
# template or a downstream consumer needs them.
INTERNAL = {"ir_internal": True}


def _published(o):
    """asdict() minus every field marked INTERNAL, at any nesting depth."""
    if is_dataclass(o) and not isinstance(o, type):
        return {
            f.name: _published(getattr(o, f.name))
            for f in fields(o)
            if not f.metadata.get("ir_internal")
        }
    if isinstance(o, (list, tuple)):
        return [_published(v) for v in o]
    if isinstance(o, dict):
        return {k: _published(v) for k, v in o.items()}
    return o


class DataclassJSONEncoder(json.JSONEncoder):
    """Encode IR dataclasses without importing the RDF parser."""

    def default(self, o):
        if is_dataclass(o) and not isinstance(o, type):
            return _published(o)
        return super().default(o)


def unique_by_id(items: list) -> list:
    """Records by id, each once. An equal record met again is one fact stated twice; a different
    record under the same id would merge two things, and is refused. A record or row with no id
    names nothing, so it always stays.

    Raises:
        ValueError: one id names two different records.
    """
    result = []
    first_by_id: dict = {}
    for item in items:
        key = item.get("id") if isinstance(item, dict) else item.id
        if key is None:
            result.append(item)
            continue
        first = first_by_id.setdefault(key, item)
        if first is item:
            result.append(item)
        elif first != item:
            raise ValueError(f"id '{key}' names two different records:\n  {first!r}\n  {item!r}")
    return result
