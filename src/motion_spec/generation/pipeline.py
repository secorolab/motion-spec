# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""High-level generation and build pipeline for motion models."""

from __future__ import annotations

import json
import os
import shutil
from datetime import UTC, datetime
from pathlib import Path

import rdflib

from motion_spec_dsl.rdf_parser.vocab import APP

from motion_spec.runs.provenance import file_path
from motion_spec.setup import build_jobs
from motion_spec.utils import generation_log, tee

PROV = rdflib.Namespace("http://www.w3.org/ns/prov#")


def _placed(graph: rdflib.Graph, generated: Path, sources: dict[Path, Path]) -> rdflib.Graph:
    """A tool's provenance with each file at its place in GENERATED: a source at its kept copy."""
    for subject, location in list(graph.subject_objects(PROV.atLocation)):
        path = file_path(location)
        if path is None:
            continue
        target = sources.get(path.resolve(), path.resolve())
        if target.is_relative_to(generated):
            graph.set((subject, PROV.atLocation, rdflib.URIRef(os.path.relpath(target, generated))))
    return graph


def _scoped(graph: rdflib.Graph, generated: Path) -> rdflib.Graph:
    """A placed tool graph under this generation's names: files by place, packages by release."""
    from motion_spec.runs.provenance import generation_scope, package_agent_uri

    scope = generation_scope(generated.parent)
    names = {
        node: scope[f"entity/generated/{location}"]
        for node, location in graph.subject_objects(PROV.atLocation)
        if file_path(location) is None
    }
    for activity in graph.subjects(rdflib.RDF.type, PROV.Activity):
        names[activity] = scope[f"activity/{str(activity).partition('/activity/')[2]}"]
    for agent in graph.subjects(rdflib.RDF.type, PROV.SoftwareAgent):
        names[agent] = package_agent_uri(
            str(graph.value(agent, rdflib.SDO.name)),
            graph.value(agent, rdflib.SDO.softwareVersion),
            graph.value(agent, rdflib.SDO.identifier),
        )
    scoped = rdflib.Graph()
    for subject, predicate, value in graph:
        scoped.add((names.get(subject, subject), predicate, names.get(value, value)))
    return scoped


def new_id(name: str) -> str:
    """Return a time-ordered identifier unique to one generation or run."""
    return f"{name}-{datetime.now(UTC).strftime('%Y%m%dT%H%M%S%fZ')}"


def create_generation_dir(
    model: Path, output_dir: Path | None = None, name: str | None = None
) -> Path:
    """Create <base>/<name>/<timestamp> for MODEL, where base is `-o` or ./generation.

    NAME defaults to the model's own stem; one model generated several ways names each tree
    for what tells them apart.
    """
    base = output_dir or Path.cwd() / "generation"
    generation = base / (name or model.stem) / datetime.now(UTC).strftime("%Y%m%dT%H%M%S%fZ")
    generation.mkdir(parents=True, exist_ok=False)
    generation = generation.resolve()
    _copy_config(generation)
    return generation


def _copy_config(generation: Path) -> None:
    """Keep the settings this generation was made under; a later one may be made under others.

    From the generation first: `-o` aside, it sits in the workspace whose config applied, which
    the working directory need not be anywhere near.
    """
    from motion_spec.config import CONFIG_FILE, find_config

    path = find_config(generation) or find_config()
    if path is not None:
        shutil.copy2(path, generation / CONFIG_FILE)


def generate_model(
    model: Path, generation: Path, *, stage: str = "code", env: dict[str, str] | None = None
) -> Path:
    """Generate MODEL through IR or C++ code and return its generated-artifact directory.

    ENV is the environment the caller resolved; codegen looks for `stst` along its PATH, so a
    workspace tool is found rather than whatever this process happens to have first.
    """
    from coord_dsl.generators.fsm import gen_json
    from coord_dsl.rdf.fsm import get_fsm_graph
    from coord_dsl.registration import gen_cpp, gen_fsm_dot_file
    from scene_dsl.gens import scenex_kdl_gen
    from scene_dsl.rdf.scenex import create_scenex_model_graph
    from textx import metamodel_for_file
    from textx.exceptions import TextXError

    from motion_spec_dsl.gens import IMPORT_BASE, generate
    from motion_spec_dsl.rdf_parser.check import validate_dataset
    from motion_spec.classes.base import DataclassJSONEncoder
    from motion_spec.runs.provenance import (
        GENERATION_DOCUMENT,
        GRAPH_COORD_DSL,
        GRAPH_DSL,
        GRAPH_MOTION_SPEC,
        build_derivation_document,
        record_code_generation,
        record_ir_generation,
        record_tool_generation,
        write_generation_document,
    )
    from motion_spec.rdf_parser.ir import generate_ir
    from motion_spec.rdf_parser.model import Model

    generated = generation / "generated"
    model_dir = generated / "model"
    try:
        authored = metamodel_for_file(str(model)).model_from_file(str(model))
        dataset, dsl_provenance = generate(authored, model_dir)
    except TextXError as problem:
        raise RuntimeError(f"the DSL rejected {model.name}: {problem}") from problem
    manifest = model_dir / f"{model.stem}-app.ld.json"
    # The authored sources the DSL read, kept with the generation before anything records them.
    source_dir = generated / "source"
    source_dir.mkdir()
    sources: dict[Path, Path] = {}
    for location in dsl_provenance.objects(predicate=PROV.atLocation):
        path = file_path(location)
        if path is None or path.is_relative_to(generated) or not path.is_file():
            continue
        kept = source_dir / path.name
        if kept.exists() and kept.read_bytes() != path.read_bytes():
            raise RuntimeError(f"generated source-name collision: {path.name}")
        shutil.copy2(path, kept)
        sources[path.resolve()] = kept
    provenance = {
        GRAPH_DSL: _scoped(_placed(dsl_provenance, generated, sources), generated),
        GRAPH_COORD_DSL: rdflib.Graph(),
        GRAPH_MOTION_SPEC: rdflib.Graph(),
    }
    # One file, one node: every later record names a file by the node the DSL's provenance gave it.
    node_at = {
        location: node for node, location in provenance[GRAPH_DSL].subject_objects(PROV.atLocation)
    }
    # The scene and the FSM are the tools' own graphs, each named by the file it was read from
    # and kept beside the model, since the IR names their nodes.
    scenes, fsms = [], []
    tool_graphs = {"scene_dsl": ([], []), "coord_dsl": ([], [])}
    started = datetime.now(UTC)
    for imported in authored.imports:
        for document in imported._tx_loaded_models:
            source = Path(document._tx_filename).resolve()
            if source.suffix == ".scenex":
                graph = create_scenex_model_graph(document)
                scenes.append(document)
                package = "scene_dsl"
            elif source.suffix == ".fsm":
                graph, fsm_ref = get_fsm_graph(document)
                fsms.append((document, graph, fsm_ref))
                package = "coord_dsl"
            else:
                continue
            named = dataset.graph(rdflib.URIRef(source.as_uri()))
            named += graph
            written = model_dir / f"{source.name}.ld.json"
            graph.serialize(destination=written, format="json-ld")
            tool_graphs[package][0].append(
                node_at[rdflib.URIRef(os.path.relpath(sources[source], generated))]
            )
            tool_graphs[package][1].append(written)
    for package, (read, written) in tool_graphs.items():
        if written:
            record_tool_generation(
                provenance[GRAPH_MOTION_SPEC],
                generated,
                "scene_graph_generation" if package == "scene_dsl" else "fsm_graph_generation",
                package,
                read,
                written,
                started=started,
                ended=datetime.now(UTC),
            )
    if len(fsms) > 1:
        raise RuntimeError(f"{model.name} imports {len(fsms)} FSMs; the program runs one")
    fsm = None
    if fsms:
        fsm_document, fsm_graph, fsm_ref = fsms[0]
        fsm = gen_json(fsm_graph, fsm_ref)
        fsm["namespace_uri"] = fsm_document.fsm.ns.uri
    conforms, report = validate_dataset(dataset)
    if not conforms:
        raise RuntimeError(f"generated RDF failed SHACL validation:\n{report}")
    from motion_spec.generation.codegen import resolve_model_assets

    loaded = Model(
        graph=dataset,
        app_path=manifest.resolve(),
        namespaces=tuple(namespace.uri for namespace in authored.namespaces),
    )
    ir_path = model_dir / "ir.json"
    started = datetime.now(UTC)
    ir = json.loads(json.dumps(generate_ir(loaded, fsm), cls=DataclassJSONEncoder))
    resolve_model_assets(ir, model.resolve().parent)
    ir_path.write_text(json.dumps(ir, indent=4))
    # The derivation graph extends the model's own graphs, so it is written beside them.
    derived_path = model_dir / f"{model.stem}-derived.ld.json"
    authored = [
        document
        for document in dataset.graphs()
        if document.identifier != loaded.derived.identifier
    ]
    derived_path.write_text(
        json.dumps(build_derivation_document(ir, loaded.derived, authored, generated), indent=4)
        + "\n"
    )
    record_ir_generation(
        provenance[GRAPH_MOTION_SPEC],
        generated,
        node_at[rdflib.URIRef(os.path.relpath(manifest, generated))],
        [
            node_at[rdflib.URIRef(os.path.relpath(path, generated))]
            for path in (
                *(
                    model_dir / str(location).removeprefix(IMPORT_BASE)
                    for location in dataset.objects(predicate=APP["import"])
                ),
                *(sources[Path(document._tx_filename).resolve()] for document in scenes),
                *(sources[Path(document._tx_filename).resolve()] for document, _, _ in fsms),
            )
        ],
        ir,
        ir_path,
        derived_path,
        started=started,
        ended=datetime.now(UTC),
    )
    # coord-dsl's provenance record swaps in rdf-utils' own resolver, so it runs after validation.
    if fsm is not None and shutil.which("dot") is not None:
        gen_fsm_dot_file(
            None, fsm_document, model_dir / f"{fsm['name']}.svg", True, False, format="svg"
        )
    if stage == "code":
        from motion_spec.generation.codegen import generate_code
        from motion_spec.setup import find_stst

        controller_dir = generated / "controller"
        controller_dir.mkdir()
        started = datetime.now(UTC)
        stst = find_stst((env or {}).get("PATH")) or "stst"
        written = generate_code(ir_path, controller_dir, generated / "contract", stst, fsm)
        record_code_generation(
            provenance[GRAPH_MOTION_SPEC],
            generated,
            ir_path,
            written,
            started=started,
            ended=datetime.now(UTC),
        )
        if fsm is not None:
            gen_cpp(None, fsm_document, controller_dir / f"{fsm['name']}.hpp", True, False)
        started = datetime.now(UTC)
        for document in scenes:
            scenex_kdl_gen(None, document, controller_dir / "headers", True, False)
        record_tool_generation(
            provenance[GRAPH_MOTION_SPEC],
            generated,
            "scene_kdl_generation",
            "scene_dsl",
            tool_graphs["scene_dsl"][0],
            [
                controller_dir / "headers" / f"{Path(document._tx_filename).stem}.kdl.hpp"
                for document in scenes
            ],
            started=started,
            ended=datetime.now(UTC),
        )
    # coord-dsl keeps its own record beside each file it writes; it joins the generation's here.
    for directory in (model_dir, generated / "controller"):
        record = directory / "provenance.ld.json"
        if record.is_file():
            # Named by place, a file coord-dsl read is the node the DSL's record gave it.
            provenance[GRAPH_COORD_DSL] += _scoped(
                _placed(rdflib.Graph().parse(record, format="json-ld"), generated, sources),
                generated,
            )
            record.unlink()
    write_generation_document(generated / GENERATION_DOCUMENT, provenance)
    return generated


def build_generation(
    generation: Path,
    *,
    prefixes: tuple[Path, ...] = (),
    jobs: int | None = None,
    env: dict[str, str] | None = None,
) -> Path:
    """Configure and build GENERATION, returning the controller executable.

    ENV is what both cmake invocations run under; None inherits this process's.
    """
    controller = generation / "generated" / "controller"
    if not (controller / "CMakeLists.txt").is_file():
        raise RuntimeError(f"generated CMake project not found: {controller}")
    build = generation / "build"
    configure = [
        "cmake",
        "-S",
        str(controller),
        "-B",
        str(build),
        # CMake defaults to no build type, which compiles the control loop unoptimized.
        # From ENV when there is one: a build type set by the sourced file is part of the
        # environment the caller asked to build under.
        f"-DCMAKE_BUILD_TYPE={(env or os.environ).get('MOTION_SPEC_BUILD_TYPE', 'RelWithDebInfo')}",
    ]
    if prefixes:
        configure.append(
            f"-DCMAKE_PREFIX_PATH={';'.join(str(path.resolve()) for path in prefixes)}"
        )
    from motion_spec.runs.provenance import record_build

    log = generation_log(generation)
    started = datetime.now(UTC)
    tee(configure, log=log, env=env)
    tee(["cmake", "--build", str(build), "--parallel", str(build_jobs(jobs))], log=log, env=env)
    executable = build / "main"
    if not executable.is_file():
        raise RuntimeError(f"controller executable not found after build: {executable}")
    record_build(
        generation / "generated", controller, executable, started=started, ended=datetime.now(UTC)
    )
    return executable
