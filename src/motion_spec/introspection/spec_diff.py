# SPDX-License-Identifier: MPL-2.0
"""Diff two generations' specification graphs and classify every change by layer.

The layer split is mechanical, not judged: a changed subject is *binding/realization* when
it is a constraint-handler resource (controllers, gains, monitors wiring, solvers,
execution context) or an embodiment resource (scene, agent, kinematic chain, dynamics);
it is *task* when it is a constraint, a motion-section resource (while/until/when/shared),
or coordination (FSM, events). Geometry splits by where it is authored: under the model's
motion/shared sections it is task, under scene namespaces it is binding. Everything else
(provenance, units) is metadata and reported separately.
"""

from __future__ import annotations

import json
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

from rdflib import RDF, Graph

# Type namespaces that decide a layer on their own.
_BINDING_TYPE_NS = (
    "/task/constraint-handler#",
    "/task/solver-specification#",
    "/metamodels/algorithm#",
    "/execution-context#",
    "/kinematic-chain/",
    "/newtonian-rigid-body-dynamics/",
    "/environment#",
    "/agent#",
)
_TASK_TYPE_NS = ("/task/constraint#", "/behaviour/fsm#", "/behaviour/event-loop#")
# Subject-path markers, used where the type alone cannot split (geometry, coordinates).
_TASK_SUBJECT_MARKS = ("/while/", "/until/", "/when/", "/shared/", "/fsm/")
_BINDING_SUBJECT_MARKS = (
    "/handler-",
    "/models/scenes/",
    "/models/agents/",
    "/models/environments/",
)
_METADATA_NS = ("http://www.w3.org/ns/prov#", "http://qudt.org/")


@dataclass
class SubjectDelta:
    """One changed subject: which side each triple moved to, and its layer."""

    subject: str
    layer: str
    types: list[str] = field(default_factory=list)
    added: list[tuple[str, str]] = field(default_factory=list)
    removed: list[tuple[str, str]] = field(default_factory=list)


def load_model_graph(generation: Path) -> Graph:
    """Every JSON-LD model graph of a generation, as one rdflib graph."""
    model_dir = generation / "generated" / "model"
    if not model_dir.is_dir():
        raise FileNotFoundError(f"{generation} holds no generated/model directory")
    graph = Graph()
    for path in sorted(model_dir.glob("*.ld.json")):
        graph.parse(path, format="json-ld")
    return graph


def _classify(subject: str, types: list[str]) -> str:
    for t in types:
        if any(mark in t for mark in _BINDING_TYPE_NS):
            return "binding"
    for t in types:
        if any(mark in t for mark in _TASK_TYPE_NS):
            return "task"
    # A quantity node's types say only what it measures; where it is authored says whose it
    # is, so the subject path decides before the metadata fallback.
    if any(mark in subject for mark in _BINDING_SUBJECT_MARKS):
        return "binding"
    if any(mark in subject for mark in _TASK_SUBJECT_MARKS):
        return "task"
    if types and all(any(t.startswith(ns) for ns in _METADATA_NS) for t in types):
        return "metadata"
    return "other"


def diff_graphs(a: Graph, b: Graph) -> list[SubjectDelta]:
    """Changed subjects between two graphs, each with its mechanical layer."""
    added = set(b) - set(a)
    removed = set(a) - set(b)
    deltas: dict[str, SubjectDelta] = {}

    def delta_for(subject) -> SubjectDelta:
        key = str(subject)
        if key not in deltas:
            types = sorted({str(o) for g in (a, b) for o in g.objects(subject, RDF.type)})
            deltas[key] = SubjectDelta(key, _classify(key, types), types)
        return deltas[key]

    for s, p, o in added:
        delta_for(s).added.append((str(p), str(o)))
    for s, p, o in removed:
        delta_for(s).removed.append((str(p), str(o)))
    return sorted(deltas.values(), key=lambda d: (d.layer, d.subject))


def summarize(deltas: list[SubjectDelta]) -> dict:
    """Per-layer counts: changed subjects and moved triples."""
    layers: dict[str, dict] = defaultdict(lambda: {"subjects": 0, "added": 0, "removed": 0})
    for d in deltas:
        row = layers[d.layer]
        row["subjects"] += 1
        row["added"] += len(d.added)
        row["removed"] += len(d.removed)
    return dict(layers)


def as_json(deltas: list[SubjectDelta]) -> str:
    return json.dumps(
        {
            "summary": summarize(deltas),
            "subjects": [
                {
                    "subject": d.subject,
                    "layer": d.layer,
                    "types": d.types,
                    "added": d.added,
                    "removed": d.removed,
                }
                for d in deltas
            ],
        },
        indent=1,
    )
