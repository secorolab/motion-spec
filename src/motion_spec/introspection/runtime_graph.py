# SPDX-License-Identifier: MPL-2.0
"""Recover runtime RDF from archive-local run data."""

from __future__ import annotations

import json
from pathlib import Path

import rdflib

from motion_spec.introspection.archive import load_manifest, sha256_file

PROV = rdflib.Namespace("http://www.w3.org/ns/prov#")
BDD = rdflib.Namespace("https://secorolab.github.io/metamodels/acceptance-criteria/bdd#")
AGN = rdflib.Namespace("https://secorolab.github.io/metamodels/agent#")
OBS = rdflib.Namespace("https://secorolab.github.io/metamodels/observation#")
RT = rdflib.Namespace("https://secorolab.github.io/metamodels/runtime#")
MSRUN = rdflib.Namespace("https://secorolab.github.io/motion-spec/runtime/")
RUNTIME_RDF_CONTRACT_VERSION = 1


def _node(identifier: str) -> rdflib.URIRef:
    safe = identifier.replace(":", "/")
    return MSRUN[safe]


def project_runtime(run_dir: Path | str, frames: list[dict], *, frame_count: int | None = None) -> rdflib.Graph:
    run_dir, manifest = load_manifest(run_dir)
    schema = json.loads((run_dir / manifest["files"]["schema"]).read_text())
    g = rdflib.Graph()
    for prefix, ns in {
        "prov": PROV,
        "bdd": BDD,
        "agn": AGN,
        "obs": OBS,
        "rt": RT,
        "msrun": MSRUN,
    }.items():
        g.bind(prefix, ns)

    run = _node(f"run:{manifest['run_id']}")
    activity = _node(schema.get("runtime_provenance", {}).get("activity_id", "activity:controller_execution"))
    producer = _node(schema.get("runtime_provenance", {}).get("producer_agent_id", "agent:controller_process"))
    runtime = _node(schema.get("runtime_provenance", {}).get("runtime_agent_id", "agent:runtime"))
    frame_log = _node("entity:frame_log")
    schema_entity = _node("entity:schema_json")
    layout_entity = _node("entity:frame_layout_json")
    model_entity = _node("entity:model_jsonld")
    provenance_entity = _node("entity:provenance_jsonld")

    g.add((run, rdflib.RDF.type, PROV.Entity))
    g.add((run, rdflib.RDF.type, RT.Runtime))
    g.add((activity, rdflib.RDF.type, PROV.Activity))
    g.add((activity, rdflib.RDF.type, BDD.SimulatedExecution))
    g.add((activity, PROV.wasAssociatedWith, producer))
    g.add((activity, PROV.used, schema_entity))
    g.add((activity, PROV.used, layout_entity))
    g.add((activity, PROV.used, model_entity))
    g.add((activity, PROV.used, provenance_entity))
    g.add((producer, rdflib.RDF.type, PROV.SoftwareAgent))
    g.add((producer, rdflib.RDF.type, OBS.ObservationProvider))
    g.add((producer, PROV.actedOnBehalfOf, runtime))
    g.add((runtime, rdflib.RDF.type, PROV.SoftwareAgent))
    if str(schema.get("runtime_provenance", {}).get("runtime_agent_id", "")).endswith(":mujoco"):
        g.add((runtime, rdflib.RDF.type, RT.MuJoCoRuntime))
    g.add((frame_log, rdflib.RDF.type, PROV.Entity))
    g.add((frame_log, PROV.wasGeneratedBy, activity))
    g.add(
        (
            frame_log,
            PROV.atLocation,
            rdflib.URIRef((run_dir / manifest["files"]["frame_log"]).resolve().as_uri()),
        )
    )
    for entity, key in (
        (schema_entity, "schema"),
        (layout_entity, "frame_layout"),
        (model_entity, "model"),
        (provenance_entity, "provenance"),
    ):
        rel = manifest.get("files", {}).get(key)
        if rel and (run_dir / rel).exists():
            g.add((entity, rdflib.RDF.type, PROV.Entity))
            g.add((entity, PROV.atLocation, rdflib.URIRef((run_dir / rel).resolve().as_uri())))
    g.add((run, MSRUN.contractVersion, rdflib.Literal(RUNTIME_RDF_CONTRACT_VERSION)))
    g.add((run, MSRUN.runId, rdflib.Literal(manifest["run_id"])))
    g.add((run, MSRUN.frameCount, rdflib.Literal(len(frames) if frame_count is None else frame_count)))
    g.add((run, PROV.wasGeneratedBy, activity))

    states = schema.get("fsm", {}).get("states", [])
    for frame in frames:
        frame_node = _node(f"frame:{manifest['run_id']}:{frame['step']}")
        g.add((frame_node, rdflib.RDF.type, MSRUN.Frame))
        g.add((frame_node, MSRUN.step, rdflib.Literal(frame["step"])))
        g.add((frame_node, MSRUN.t, rdflib.Literal(frame["t"])))
        state_idx = frame.get("fsm_state", -1)
        if 0 <= state_idx < len(states):
            g.add((frame_node, MSRUN.activeState, rdflib.URIRef(states[state_idx]["uri"])))
        g.add((run, MSRUN.frame, frame_node))
    return g


def write_runtime_ttl(run_dir: Path | str, frames: list[dict], *, frame_count: int | None = None) -> Path:
    run_dir = Path(run_dir)
    graph = project_runtime(run_dir, frames, frame_count=frame_count)
    out = run_dir / "runtime.ttl"
    graph.serialize(out, format="turtle")
    manifest_path = run_dir / "manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        manifest.setdefault("artifacts", {})["runtime.ttl"] = {
            "role": "runtime",
            "sha256": sha256_file(out),
        }
        manifest.setdefault("files", {})["runtime_ttl"] = "runtime.ttl"
        manifest_path.write_text(json.dumps(manifest, indent=4) + "\n")
    return out
