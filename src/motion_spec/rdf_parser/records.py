# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Shared IR record helpers: dict-or-dataclass field access, dedupe, naming and units."""

from __future__ import annotations

import re
from dataclasses import asdict
from enum import Enum
from functools import wraps


def _ros_camel_to_snake(name: str) -> str:
    """rosidl message-name -> header stem (Trinary->trinary, TrinaryStamped->trinary_stamped)."""
    s = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", name)
    s = re.sub(r"([a-z\d])([A-Z])", r"\1_\2", s)
    return s.lower()
def _ros_type_parts(ros_type: str) -> tuple[str, str, str]:
    """`pkg/msg/CamelType` -> (pkg, "pkg/msg/camel_type.hpp", "pkg::msg::CamelType"), derived from
    the rosidl naming rule rather than hardcoded.
    """
    parts = ros_type.split("/")
    pkg, msg_name = parts[0], parts[-1]
    sub = parts[1] if len(parts) == 3 else "msg"
    include = f"{pkg}/{sub}/{_ros_camel_to_snake(msg_name)}.hpp"
    cpp_type = f"{pkg}::{sub}::{msg_name}"
    return pkg, include, cpp_type
def memoize(func):
    """Decorator caching a Parser method's result per instance, keyed by its arguments."""

    @wraps(func)
    def decorator(self, *args, **kwargs):
        # Scope by func identity: position(uri) and quantity(uri) share a (uri,) cache key.
        """Return the cached result, computing and storing it on first call."""
        key = (func.__qualname__,) + args + tuple(kwargs.items())
        if key not in self.cache:
            self.cache[key] = func(self, *args, **kwargs)
        return self.cache[key]

    return decorator
def escape(s):
    """Return a graph-safe identifier for a local name."""
    s = re.sub(r"[^0-9A-Za-z_]", "_", str(s))
    if s and s[0].isdigit():
        s = f"_{s}"
    return s
def _dedupe_by_id(items):
    """Deduplicate items by id, keeping the first occurrence."""
    result = []
    seen = set()
    for item in items:
        key = getattr(item, "id", None)
        if key is None:
            result.append(item)
            continue
        if key in seen:
            continue
        seen.add(key)
        result.append(item)
    return result
def _kebab(text: str) -> str:
    """Kebab-case an id fragment for use as an IRI path segment."""
    return text.replace("_", "-").lower()
def _prune(row: dict) -> dict:
    """Drop the keys an introspection row leaves unset."""
    return {k: v for k, v in row.items() if v is not None and v != []}
def _dedupe_dicts(entries):
    """Deduplicate dict rows by id, keeping the first occurrence."""
    result = []
    seen = set()
    for entry in entries:
        value = entry.get("id")
        if value in seen:
            continue
        seen.add(value)
        result.append(entry)
    return result
def _add_group_type_flags(groups: list) -> None:
    """Set is_pose/is_twist/is_wrench on pose-axis error groups from their superobject type."""
    for g in groups:
        so_type = _field(g, "superobject_type", "Pose")
        _set_field(g, "is_pose", so_type == "Pose")
        _set_field(g, "is_twist", so_type in ("VelocityTwist", "AccelerationTwist"))
        _set_field(g, "is_wrench", so_type == "Wrench")
def _field(obj, key, default=None):
    """Read a field from either a dict or a dataclass instance, so derivations run on the native IR
    without a dict round-trip. str-Enum values are normalized to their string value.
    """
    if obj is None:
        return default
    val = obj.get(key, default) if isinstance(obj, dict) else getattr(obj, key, default)
    return val.value if isinstance(val, Enum) else val
def _set_field(obj, key, value) -> None:
    """Set a field on either a dict or a dataclass instance (the field must exist on the
    dataclass for it to serialize)."""
    if isinstance(obj, dict):
        obj[key] = value
    else:
        setattr(obj, key, value)
def _as_dict(obj) -> dict:
    """Plain-dict view of a dict or dataclass (for building rows from all fields)."""
    return obj if isinstance(obj, dict) else asdict(obj)
def expand_vector_fields(
    item,
    field: str,
    component_names: tuple[str, ...] = ("x", "y", "z"),
    default: list[float] | None = None,
) -> None:
    """Expand a vector field into <field>_<component> parts. An omitted value falls back to
    ``default``; a value whose arity does not match component_names raises rather than being padded.
    """
    values = _field(item, field)
    if values is None:
        # Env placement shorthand: omitted position/orientation means zero/identity.
        values = list(default) if default is not None else [0.0] * len(component_names)
    if not isinstance(values, list) or len(values) != len(component_names):
        item_id = _field(item, "id", "<unknown>")
        raise ValueError(
            f"Scene item '{item_id}' has invalid '{field}'; expected {len(component_names)} values."
        )
    for name, value in zip(component_names, values):
        _set_field(item, f"{field}_{name}", value)
def require_field(obj_id: str, field: str, value) -> None:
    """Raise if a required procedural scene-object field is missing."""
    if value is None:
        raise ValueError(
            f"Procedural scene object '{obj_id}' is missing required field "
            f"'{field}'. Add it to the .robmot model — silent defaults are no "
            f"longer applied."
        )
def _pose_component(component_id: str, data_by_id: dict) -> dict:
    """Structured pose component: either a literal ``value`` or a ``ref`` id that the
    backend template renders via access-expr. Backend-agnostic — no C++/KDL here."""
    component = data_by_id.get(component_id)
    reference_value = _field(component, "reference_value")
    if reference_value:
        return {"value": None, "ref": reference_value}
    value = _field(component, "value")
    if value is not None:
        return {"value": str(value), "ref": None}
    return {"value": None, "ref": component_id}
def _signal_id(value):
    """Id string of a signal value (a str or an object with an id)."""
    if isinstance(value, str):
        return value
    return _field(value, "id")
def _cpp_identifier(name: str) -> str:
    """Sanitize a MuJoCo joint name into a C++ identifier fragment."""
    return re.sub(r"[^0-9A-Za-z_]", "_", name)
def _sole(ids: set) -> str | None:
    """The one id in a set, or None when several instances write the same value."""
    return next(iter(ids)) if len(ids) == 1 else None
def _index_by_id(items: list) -> dict:
    """Index IR items (dicts or dataclasses) by their id (skips id-less entries)."""
    return {iid: item for item in items if (iid := _field(item, "id"))}
