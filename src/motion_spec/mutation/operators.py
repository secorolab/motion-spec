# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

"""Mutation sites in a .robmot model, found in its text.

The DSL's declarations have fixed syntactic forms, so the sites are found by matching those forms
rather than by parsing: a mutant only has to be text the compiler reads back, and a patch that
touches one number keeps the rest of the model byte-identical to the reference.
"""

import re
from dataclasses import dataclass

# A spec constant with a literal numeric value; `pi rad` and the like carry no number to scale.
CONSTANT = re.compile(
    r"(length|angle|torque|angular-acceleration|linear-acceleration)"
    r"\s+([A-Za-z0-9-]+)\s*=\s*(-?[0-9.]+)"
)
DIRECTION = re.compile(r"direction\s+([A-Za-z0-9-]+)\s*\{[^}]*\}\s*=\s*\(([^)]+)\)")
GAIN = re.compile(r"(Kp|Ki|Kd):\s*(-?[0-9.]+)")
CONTROLLER = re.compile(r"pid\s+([A-Za-z0-9-]+)\s*\{")
DEBOUNCE = re.compile(r"satisfied for\s+([0-9.]+)\s*s")
MONITOR = re.compile(r"([A-Za-z0-9-]+):\s*monitor")


@dataclass
class MutationSite:
    """One authored term, and the single edit that mutates it."""

    file: str
    span: tuple[int, int]
    kind: str
    name: str
    original: str
    mutated: str
    operator: str
    # The element a perfect diagnosis should point at, when the site knows it. Evaluation only.
    element_uri: str | None = None


def discover_sites(text: str, file: str = "") -> list[MutationSite]:
    """Every mutation this text admits, one site per (term, operator) pair."""
    sites = []
    for match in CONSTANT.finditer(text):
        name, literal = match.group(2), match.group(3)
        value = _value(literal)
        if value is None or value == 0.0:
            continue  # scaling zero is not a mutation
        span = match.span(3)
        for operator, factor in (("scale_constant_x2", 2.0), ("scale_constant_half", 0.5)):
            sites.append(
                MutationSite(
                    file,
                    span,
                    "constant",
                    name,
                    literal,
                    _number(value * factor, literal),
                    operator,
                )
            )
    for match in DIRECTION.finditer(text):
        flipped = _flip(match.group(2))
        if flipped is None:
            continue
        sites.append(
            MutationSite(
                file,
                match.span(2),
                "direction",
                match.group(1),
                match.group(2),
                flipped,
                "flip_direction_component",
            )
        )
    for match in GAIN.finditer(text):
        gain, literal = match.group(1), match.group(2)
        value = _value(literal)
        if value is None:
            continue
        controller = _enclosing(CONTROLLER, text, match.start())
        if controller is None:
            continue
        if gain == "Kp" and value != 0.0:
            sites.append(
                MutationSite(
                    file,
                    match.span(2),
                    "gain",
                    controller,
                    literal,
                    _number(value * 4.0, literal),
                    "gain_x4",
                )
            )
        elif gain == "Kd" and value != 0.0:
            sites.append(
                MutationSite(file, match.span(2), "gain", controller, literal, "0", "gain_zero")
            )
    for match in DEBOUNCE.finditer(text):
        literal = match.group(1)
        value = _value(literal)
        monitor = _enclosing(MONITOR, text, match.start())
        if value is None or value == 0.0 or monitor is None:
            continue
        sites.append(
            MutationSite(
                file,
                match.span(1),
                "debounce",
                monitor,
                literal,
                _number(value * 4.0, literal),
                "debounce_x4",
            )
        )
    return sites


def natural_sites(faults: list[dict], text: str, file: str = "") -> list[MutationSite]:
    """Hand-recorded faults as sites: each `find` located in the text, `replace` put in its place.

    A natural fault is not enumerated from a form; it is one authoring mistake that actually
    happened, recorded verbatim. It names the element it damaged, because no operator tag says so.
    """
    sites = []
    for fault in faults:
        name, find = fault["name"], fault["find"]
        occurrences = text.count(find)
        if occurrences != 1:
            raise ValueError(f"{name}: `find` occurs {occurrences} times in the model, not once")
        start = text.index(find)
        sites.append(
            MutationSite(
                file,
                (start, start + len(find)),
                "natural",
                name,
                find,
                fault["replace"],
                "natural",
                fault["element_uri"],
            )
        )
    return sites


def apply(text: str, site: MutationSite) -> str:
    """The model text with this one site mutated."""
    start, end = site.span
    if text[start:end] != site.original:
        raise ValueError(f"{site.operator} at {site.span}: text is not {site.original!r}")
    return text[:start] + site.mutated + text[end:]


def _enclosing(pattern: re.Pattern, text: str, position: int) -> str | None:
    """The name the last `pattern` before `position` captured -- the block this site sits in."""
    found = None
    for match in pattern.finditer(text, 0, position):
        found = match.group(1)
    return found


def _value(literal: str) -> float | None:
    try:
        return float(literal)
    except ValueError:
        return None


def _number(value: float, original: str) -> str:
    """Format a scaled value the way the original was written, so the grammar reads it back."""
    text = f"{value:.10g}"
    if "." in original and "." not in text and "e" not in text:
        text += ".0"
    return text


def _flip(components: str) -> str | None:
    """The component list with the sign of its first nonzero component flipped."""
    parts = [part.strip() for part in components.split(",")]
    for index, part in enumerate(parts):
        value = _value(part)
        if value is None:
            return None
        if value != 0.0:
            parts[index] = part[1:].strip() if part.startswith("-") else f"-{part}"
            return ", ".join(parts)
    return None
