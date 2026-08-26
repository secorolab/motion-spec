# SPDX-License-Identifier: MPL-2.0

"""Two generations side by side: what the model says, what happened, what the values did.

Three axes, always together and always in that order. A duration that moved with no model
change is a flaky run or a real nondeterminism, and telling those two apart is the whole
value -- a behaviour diff with its model diff missing cannot be read at all.

Nothing here computes a statistic of its own. Coordination comes from the runtime-graph
views, signals from the frame-log reports, and this module only aligns and subtracts. No
delta is ranked or coloured: a longer state may be the fix, and the dashboard cannot know.
"""

from __future__ import annotations

from pathlib import Path

import rdflib
from rdflib.compare import graph_diff

from motion_spec.dashboard.analysis import run_reports
from motion_spec.dashboard.catalog import rdf_name
from motion_spec.dashboard.graph import MODEL_GRAPH, load_model_graph, model_manifest
from motion_spec.dashboard.queries import DECLARED_ACTIVITIES, gates
from motion_spec.dashboard.queries import compare as compare_runs
from motion_spec.introspection.archive import ArchiveError
from motion_spec.introspection.replay import resolve_archive

# What 018 restricts its statistics to, said in the payload so a reader knows the table is
# not every recorded quantity.
WATCHED = "signals a constraint or monitor judges"


# -- the model axis ------------------------------------------------------------------------
#
# Over the graph, never over the .robmot text: a reordered file or a renamed generation must
# not read as a change. rdflib canonicalises both sides before subtracting, so a blank node
# that moved but says the same thing does not either.


def _design_graph(generation_dir: Path) -> rdflib.Graph:
    """A generation's design graph with its generation provenance taken out.

    That provenance is the whole of what a regenerate moves -- the activity IRIs, the
    generatedAtTime stamps, the file:// location of every output -- and it is a closed island
    no design triple points into, so dropping it costs the diff nothing else.
    """
    # Local: one namespace constant, with the DSL's parser generator behind the import.
    from motion_spec_dsl.gens import DSLPROV

    manifest = model_manifest(generation_dir)
    if manifest is None:
        return rdflib.Graph()
    dataset = rdflib.Dataset(default_union=True)
    model = load_model_graph(manifest, dataset, dataset.graph(MODEL_GRAPH))
    design = rdflib.Graph()
    for triple in model:
        if not any(str(term).startswith(DSLPROV) for term in (triple[0], triple[2])):
            design.add(triple)
    return design


def _literal(term):
    """A literal as something JSON can carry; anything else as the term it is."""
    if isinstance(term, rdflib.Literal):
        value = term.toPython()
        return value if isinstance(value, (bool, int, float, str)) else str(term)
    return str(term)


def _by_subject(graph: rdflib.Graph) -> dict:
    index: dict = {}
    for subject, predicate, obj in graph:
        index.setdefault(subject, {}).setdefault(predicate, []).append(obj)
    return index


def _term(subject, changed: rdflib.Graph) -> dict:
    """A term that appeared or vanished whole, read off the diff that holds all its triples.

    Canonicalisation gives a blank node a hash for a label, which names nothing a reader can
    look up, so an anonymous term answers with what it is instead of with that hash.
    """
    types = sorted(rdf_name(kind) for kind in changed.objects(subject, rdflib.RDF.type))
    if isinstance(subject, rdflib.BNode):
        return {"iri": "", "name": " ".join(types) or "anonymous", "types": types}
    return {"iri": str(subject), "name": rdf_name(subject), "types": types}


def _coordination_elements(graph: rdflib.Graph) -> set[str]:
    """What the design says can be occupied at all -- the two sides' shared identity."""
    return {str(row[0]) for row in graph.query(DECLARED_ACTIVITIES)}


def model_diff(left_dir: Path, right_dir: Path) -> dict:
    """What changed in the design graph, classified by what the reader can act on.

    A value that moved is the common case and the most legible row: the term, the property,
    the number before and the number after. A term that appeared or vanished is next. What
    is left is structure -- an operand repointed, a controller set changed -- and it is
    reported as what went and what came rather than pretended to be a value.
    """
    left, right = _design_graph(left_dir), _design_graph(right_dir)
    _both, only_left, only_right = graph_diff(left, right)
    gone, came = _by_subject(only_left), _by_subject(only_right)
    left_terms, right_terms = set(left.subjects()), set(right.subjects())
    values, structure = [], []
    for subject in sorted(gone.keys() | came.keys(), key=str):
        if subject not in right_terms or subject not in left_terms:
            continue  # the whole term went or came; it is reported as that, not as an edit
        before, after = gone.get(subject, {}), came.get(subject, {})
        for predicate in sorted(before.keys() | after.keys(), key=str):
            was, now = before.get(predicate, []), after.get(predicate, [])
            row = {
                "iri": str(subject),
                "name": rdf_name(subject),
                "predicate": str(predicate),
                "property": rdf_name(predicate),
            }
            if len(was) == len(now) == 1 and all(
                isinstance(term, rdflib.Literal) for term in (*was, *now)
            ):
                values.append({**row, "left": _literal(was[0]), "right": _literal(now[0])})
            else:
                structure.append(
                    {
                        **row,
                        "removed": [_literal(term) for term in was],
                        "added": [_literal(term) for term in now],
                    }
                )
    return {
        "same_model": _coordination_elements(left) == _coordination_elements(right),
        "values": sorted(values, key=lambda row: (row["name"], row["property"])),
        "removed": sorted(
            (_term(subject, only_left) for subject in gone if subject not in right_terms),
            key=lambda row: row["name"],
        ),
        "added": sorted(
            (_term(subject, only_right) for subject in came if subject not in left_terms),
            key=lambda row: row["name"],
        ),
        "structure": sorted(structure, key=lambda row: (row["name"], row["property"])),
    }


# -- the coordination axis -----------------------------------------------------------------


def _header(run_dir: Path):
    """The run's own contract header, or nothing when it recorded no frame log.

    A run can be started with logging off; it still occupied states and still has a runtime
    graph, so the coordination axis answers and the signal axis says it cannot.
    """
    try:
        return resolve_archive(Path(run_dir))[3].header
    except (ArchiveError, OSError):
        return None


def _reached_end(spans: list[dict], header) -> bool | None:
    """Whether the run ever occupied the element its contract names as the end."""
    if header is None or not 0 <= header.end_state < len(header.fsm_states):
        return None
    return any(span["element"] == header.fsm_states[header.end_state].iri for span in spans)


def _platform(header) -> str:
    return f"{header.platform_name or 'unnamed'}{' (simulated)' if header.simulated else ''}"


def _get(side: dict | None, key: str):
    return None if side is None else side[key]


def _delta(before, after):
    return None if before is None or after is None else round(after - before, 3)


def _gate_pair(left: dict | None, right: dict | None) -> dict:
    named = left or right
    return {
        "monitor": named["monitor"],
        "name": named["monitor_name"],
        "event": named["event_name"],
        "left_waited_s": _get(left, "waited_s"),
        "right_waited_s": _get(right, "waited_s"),
        "delta_s": _delta(_get(left, "waited_s"), _get(right, "waited_s")),
        "left_rearms": _get(left, "rearm_count"),
        "right_rearms": _get(right, "rearm_count"),
        "left_fired": None if left is None else left["fired_step"] is not None,
        "right_fired": None if right is None else right["fired_step"] is not None,
    }


def _gate_rows(left_run: Path, right_run: Path) -> list[dict]:
    """Each gate's arming on both sides, aligned by monitor, a re-arming its own row."""
    pending: dict[str, list[dict]] = {}
    for row in gates(right_run)["gates"]:
        pending.setdefault(row["monitor"], []).append(row)
    rows = []
    for row in gates(left_run)["gates"]:
        queue = pending.get(row["monitor"]) or []
        rows.append(_gate_pair(row, queue.pop(0) if queue else None))
    rows += [_gate_pair(None, row) for queue in pending.values() for row in queue]
    return rows


def coordination_diff(left_run: Path, right_run: Path) -> dict:
    """What happened, aligned: occupancy durations, control-flow order, gate waits and re-arms.

    Order first. A state visited that was not visited before means the two runs took
    different paths, and durations along different paths are not comparable at all -- so the
    payload says the order differs before it says anything about a duration.
    """
    aligned = compare_runs(left_run, right_run)
    spans = {side: aligned[side]["spans"] for side in ("left", "right")}
    order = {side: [span["element"] for span in rows] for side, rows in spans.items()}
    return {
        "order_differs": order["left"] != order["right"],
        "left_order": [span["name"] for span in spans["left"]],
        "right_order": [span["name"] for span in spans["right"]],
        "left_source": aligned["left"]["runtime_source"],
        "right_source": aligned["right"]["runtime_source"],
        # Whether each run ever occupied what its own contract names as the end.
        "left_reached_end": _reached_end(spans["left"], _header(left_run)),
        "right_reached_end": _reached_end(spans["right"], _header(right_run)),
        "activities": aligned["activities"],
        "gates": _gate_rows(left_run, right_run),
    }


# -- the signal axis -----------------------------------------------------------------------
#
# Aligned on an occupancy, not on a state: a state entered twice is two rows, because a
# re-entry appearing or disappearing is the change worth seeing.


def _occupancies(reports: dict) -> dict[int, tuple[str, int]]:
    """Span index -> (activity, which entry of it), so the two sides align occupancy by
    occupancy rather than by summing everything one state ever did."""
    seen: dict[str, int] = {}
    keys = {}
    for span in reports["states"]:
        seen[span["state_id"]] = nth = seen.get(span["state_id"], -1) + 1
        keys[span["state_index"]] = (span["state_id"], nth)
    return keys


def _paired(left: dict, right: dict, kind: str, identity) -> dict:
    """Both sides' rows of one report kind, keyed by occupancy and by what the row is about."""
    paired: dict = {}
    for side, reports in (("left", left), ("right", right)):
        keys = _occupancies(reports)
        for row in reports[kind]:
            key = keys.get(row["state_index"])
            if key is not None:
                paired.setdefault((key, identity(row)), {})[side] = row
    return paired


def _sides(pair: dict, *fields) -> dict:
    """One row's numbers on both sides, with the deltas of whatever both sides recorded.

    Sign and magnitude, never a verdict: which direction is desirable is the reader's to say.
    """
    left, right = pair.get("left"), pair.get("right")
    out = {}
    for field in fields:
        before, after = _get(left, field), _get(right, field)
        out[f"left_{field}"] = before
        out[f"right_{field}"] = after
        out[f"delta_{field}"] = None if before is None or after is None else after - before
    return out


def _named(pair: dict, field: str):
    """What a row is of, which either side answers the same way."""
    return (pair.get("left") or pair["right"])[field]


def _during(paired: dict, key) -> list:
    """One occupancy's rows of a report kind, ordered by what each row is about."""
    rows = [(what, pair) for (occupancy, what), pair in paired.items() if occupancy == key]
    return sorted(rows, key=lambda row: str(row[0]))


def _saturation(left: dict, right: dict) -> list[dict]:
    """Torque clipping per joint, aligned as 018 reports it -- per run, not per occupancy.

    A clipped joint carries the states it was clipped in rather than a span, so this row sits
    outside the per-activity tables instead of being re-derived into one.
    """
    paired: dict = {}
    for side, reports in (("left", left), ("right", right)):
        for row in reports["saturation"]:
            paired.setdefault((row["solver"], row["joint"]), {})[side] = row
    return [
        {
            "solver": solver,
            "joint": joint,
            "left_states": _get(pair.get("left"), "states") or [],
            "right_states": _get(pair.get("right"), "states") or [],
            **_sides(pair, "ticks", "requested_nm", "limit_nm"),
        }
        for (solver, joint), pair in sorted(paired.items())
    ]


def signal_diff(left_run: Path, right_run: Path) -> dict:
    """Per occupancy, what the values a gate judges did on each side, and by how much it moved.

    Restricted to the signals 018 already restricts itself to; the payload says so, because
    a table that quietly left something out reads as a table of everything.
    """
    left, right = run_reports(left_run), run_reports(right_run)
    watched = _paired(left, right, "signals", lambda row: row["signal"])
    contact = _paired(left, right, "contact", lambda _row: "")
    modes = _paired(left, right, "oscillation", lambda row: row["solver"])
    order = list(dict.fromkeys([*_occupancies(left).values(), *_occupancies(right).values()]))
    activities = []
    for key in order:
        entry = {
            "name": key[0],
            "entry": key[1],
            "signals": [
                {
                    "signal": signal,
                    "gate": _named(pair, "gate"),
                    "band": _named(pair, "band"),
                    **_sides(pair, "peak", "pk_pk", "rms"),
                }
                for signal, pair in _during(watched, key)
            ],
            "contact": [
                {
                    "gate": _named(pair, "gate"),
                    "band": _named(pair, "band"),
                    **_sides(pair, "peak_n", "rebound", "ripple", "settle_s"),
                }
                for _one, pair in _during(contact, key)
            ],
            "modes": [
                {"solver": solver, **_sides(pair, "mode_hz", "mean_amplitude", "agreeing")}
                for solver, pair in _during(modes, key)
            ],
        }
        if any(entry[kind] for kind in ("signals", "contact", "modes")):
            activities.append(entry)
    return {
        "restricted_to": WATCHED,
        "activities": activities,
        "saturation": _saturation(left, right),
    }


# -- what the three axes are worth -----------------------------------------------------------


def compare_generations(
    left: Path, right: Path, left_run: Path | None = None, right_run: Path | None = None
) -> dict:
    """Two generations along all three axes, and a plain statement of what does not line up.

    Runs are optional: two generations with no run between them still have a model diff, and
    saying "these two models differ by one gain" is worth more than refusing. Where a
    comparison is unsound it is said at the top and what aligns is still shown -- refusing is
    unhelpful, and pretending is worse.
    """
    model = model_diff(left, right)
    notes = []
    if not model["same_model"]:
        notes.append("the two sides declare different coordination elements; only what aligns")
    comparable = {
        "same_model": model["same_model"],
        "same_schema_hash": None,
        "same_platform": None,
        "both_completed": None,
        "notes": notes,
    }
    payload = {
        "comparable": comparable,
        "model": model,
        "coordination": {"order_differs": False, "activities": [], "gates": []},
        "signals": {"restricted_to": WATCHED, "activities": [], "saturation": []},
    }
    if left_run is None or right_run is None:
        notes.append("no run picked on both sides; the model diff is all there is to show")
        return payload
    coordination = coordination_diff(left_run, right_run)
    payload["coordination"] = coordination
    # Said before any duration: durations along a different path are not comparable at all.
    if coordination["order_differs"]:
        notes.insert(0, "the two runs took different paths; durations are not comparable")
    for side in ("left", "right"):
        if coordination[f"{side}_source"] != "archive":
            notes.append(
                f"the {side} run has no archived runtime graph; its waits and re-arms are "
                "unavailable, not zero"
            )
    comparable["both_completed"] = bool(
        coordination["left_reached_end"] and coordination["right_reached_end"]
    )
    for side, run in (("left", left_run), ("right", right_run)):
        if coordination[f"{side}_reached_end"] is False:
            notes.append(f"{Path(run).name} did not reach its end state")
    before, after = _header(left_run), _header(right_run)
    if before is None or after is None:
        notes.append("a side recorded no frame log; nothing can be said about its signals")
        return payload
    comparable["same_schema_hash"] = before.schema_hash == after.schema_hash
    if not comparable["same_schema_hash"]:
        notes.append("the two contracts differ; a slot index may not mean the same thing")
    comparable["same_platform"] = _platform(before) == _platform(after)
    if not comparable["same_platform"]:
        notes.append(
            f"{_platform(before)} against {_platform(after)}; "
            "signal figures across platforms are not like for like"
        )
    payload["signals"] = signal_diff(left_run, right_run)
    return payload
