# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""High-level generation and build pipeline for motion models."""

from __future__ import annotations

import json
import os
import shutil
import urllib.parse
from datetime import UTC, datetime
from pathlib import Path

import rdflib

from motion_spec_dsl.rdf_parser.vocab import APP

from motion_spec.setup import build_jobs
from motion_spec.utils import generation_log, tee, tool_environment

PROV = rdflib.Namespace("http://www.w3.org/ns/prov#")


def _file_path(value: rdflib.term.Identifier) -> Path | None:
    parsed = urllib.parse.urlparse(str(value))
    return Path(urllib.parse.unquote(parsed.path)) if parsed.scheme == "file" else None


# Every spelling of prov:atLocation a compacted JSON-LD document can carry: the alias the
# metamodel context defines, the CURIE, and the expanded IRI.
_LOCATION_KEYS = ("atLocation", "prov:atLocation", "http://www.w3.org/ns/prov#atLocation")


def _json_objects(node):
    """Every node object in a parsed JSON-LD document. `@context` is term definitions, not data,
    so it is not descended into."""
    if isinstance(node, dict):
        yield node
        for key, value in node.items():
            if key != "@context":
                yield from _json_objects(value)
    elif isinstance(node, list):
        for item in node:
            yield from _json_objects(item)


def _relocated(value, document: Path, locations: dict[Path, Path]):
    """The archive-relative path for one location value, or it unchanged."""
    if isinstance(value, list):
        return [_relocated(item, document, locations) for item in value]
    if isinstance(value, dict):
        return (
            {**value, "@id": _relocated(value["@id"], document, locations)}
            if "@id" in value
            else value
        )
    source = _file_path(rdflib.URIRef(value)) if isinstance(value, str) else None
    if source is None:
        return value
    target = locations.get(source.resolve())
    if target is None and source.exists() and source.is_relative_to(document.parent):
        target = source
    if target is None:
        return value
    return urllib.parse.quote(os.path.relpath(target, document.parent), safe="/..")


def _rewrite_locations(document: Path, locations: dict[Path, Path]) -> None:
    """Point every prov:atLocation at where the artifact now lives.

    Edits the parsed JSON in place rather than round-tripping through rdflib: a serialized
    Dataset carries only triples, and rewriting through one dropped the document's @context and
    its schema_version key on the way out.
    """
    data = json.loads(document.read_text())
    for node in _json_objects(data):
        for key in _LOCATION_KEYS:
            if key in node:
                node[key] = _relocated(node[key], document, locations)
    document.write_text(json.dumps(data, indent=2) + "\n")


def _fold_provenance(document: Path, graph_id, source: Path) -> None:
    """Move one tool's provenance document into its named graph of the generation document."""
    from motion_spec.introspection.provenance import append_generation_graph

    data = json.loads(source.read_text())
    append_generation_graph(document, graph_id, data.get("@graph", []))
    source.unlink()


def _organize_generation(model_dir: Path, controller_dir: Path | None = None) -> None:
    from motion_spec.introspection.provenance import (
        GENERATION_DOCUMENT,
        GRAPH_COORD_DSL,
        GRAPH_DSL,
    )

    generated = model_dir.parent
    document = generated / GENERATION_DOCUMENT
    source_dir = generated / "source"
    source_dir.mkdir(exist_ok=True)

    locations: dict[Path, Path] = {}
    coord_provenance = model_dir / "provenance.ld.json"
    if coord_provenance.is_file():
        locations[coord_provenance.resolve()] = document
    dsl_provenance = model_dir / "provenance" / "dsl.ld.json"
    dataset = rdflib.Dataset().parse(dsl_provenance, format="json-ld")
    for _, _, location, _ in dataset.quads((None, PROV.atLocation, None, None)):
        source = _file_path(location)
        if source is None:
            continue
        if source.resolve() in locations:
            continue
        if source.is_relative_to(model_dir):
            controller_artifact = (
                controller_dir / source.name
                if controller_dir
                and (source.name == "fsm_ir.json" or source.name.endswith("_fsm.hpp"))
                else None
            )
            locations[source.resolve()] = (
                controller_artifact
                if controller_artifact is not None and controller_artifact.is_file()
                else source
            )
            continue
        if not source.is_file() or source.is_relative_to(model_dir):
            continue
        target = source_dir / source.name
        if target.exists() and target.read_bytes() != source.read_bytes():
            raise RuntimeError(f"generated source-name collision: {source.name}")
        shutil.copy2(source, target)
        locations[source.resolve()] = target

    locations[dsl_provenance.resolve()] = document

    app_manifest = next(model_dir.glob("*-app.ld.json"))
    app = rdflib.Dataset().parse(app_manifest, format="json-ld")
    for subject, predicate, path, context in list(app.quads((None, APP.path, None, None))):
        graph = app.graph(context)
        graph.remove((subject, predicate, path))
        graph.add((subject, predicate, rdflib.Literal(".")))
    app.serialize(app_manifest, format="json-ld", indent=2)

    if controller_dir:
        contract_dir = generated / "contract"
        contract_dir.mkdir()
        for artifact in ("frame_layout.json", "frame_log.proto", "frame_log_header.pb"):
            source = controller_dir / artifact
            target = contract_dir / artifact
            source.replace(target)
            locations[source.resolve()] = target
        shutil.rmtree(controller_dir / ".stst")

    _fold_provenance(document, GRAPH_DSL, dsl_provenance)
    (model_dir / "provenance").rmdir()
    if coord_provenance.is_file():
        _fold_provenance(document, GRAPH_COORD_DSL, coord_provenance)
    _rewrite_locations(document, locations)


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
    from motion_spec_dsl.rdf_parser.check import validate_manifest
    from motion_spec.classes.base import DataclassJSONEncoder
    from motion_spec.introspection.provenance import (
        build_derivation_document,
        record_code_generation,
        record_ir_generation,
    )
    from motion_spec.rdf_parser.ir import generate_ir

    generated = generation / "generated"
    model_dir = generated / "model"
    # Into the generation it is about, beside the run consoles: what the DSL said about this
    # model belongs with this model's artifacts, not in the workspace's own record.
    dsl_returncode = tee(
        ["textx", "generate", str(model.resolve()), "--target", "jsonld", "-o", str(model_dir)],
        log=generation_log(generation),
        cwd=model.parent,
        env=tool_environment(env),
        check=False,
    )
    if dsl_returncode:
        # The DSL already reported the offending line on stderr; don't bury it under an argv dump.
        raise RuntimeError(f"the DSL rejected {model.name}, see the error above")
    manifest = model_dir / f"{model.stem}-app.ld.json"
    conforms, report = validate_manifest(manifest)
    if not conforms:
        raise RuntimeError(f"generated RDF failed SHACL validation:\n{report}")
    from motion_spec.generation.codegen import resolve_model_assets

    ir_path = model_dir / "ir.json"
    started = datetime.now(UTC)
    ir = json.loads(json.dumps(generate_ir(manifest), cls=DataclassJSONEncoder))
    resolve_model_assets(ir, model.resolve().parent)
    ir_path.write_text(json.dumps(ir, indent=4, sort_keys=True))
    # The derivation graph extends the model's own graphs, so it is written beside them.
    derived_path = model_dir / f"{model.stem}-derived.ld.json"
    derived_path.write_text(json.dumps(build_derivation_document(ir), indent=4) + "\n")
    record_ir_generation(
        generated,
        manifest,
        ir,
        {
            "entity:motion_spec_ir": ir_path,
            f"entity:generated_{derived_path.name}": derived_path,
        },
        started=started,
        ended=datetime.now(UTC),
    )
    if stage == "code":
        from motion_spec.generation.codegen import generate_code
        from motion_spec.setup import find_stst

        controller_dir = generated / "controller"
        controller_dir.mkdir()
        for artifact in (*model_dir.glob("*_fsm.hpp"), model_dir / "fsm_ir.json"):
            if artifact.is_file():
                artifact.replace(controller_dir / artifact.name)
        started = datetime.now(UTC)
        stst = find_stst((env or {}).get("PATH")) or "stst"
        written = generate_code(ir_path, controller_dir, stst)
        # The solver chain is the scene's, so it is emitted from the scene graph (plan 013).
        from motion_spec.generation.scene_kdl import write_scene_kdl_header
        from motion_spec.rdf_parser.model import load_model
        from motion_spec.rdf_parser.resources import scene_graph

        written.append(
            write_scene_kdl_header(
                scene_graph(load_model(manifest)),
                controller_dir / "headers",
                model.name,
                base_dir=model.parent,
            )
        )
        record_code_generation(
            generated, ir_path, written, started=started, ended=datetime.now(UTC)
        )
        _organize_generation(model_dir, controller_dir)
    else:
        _organize_generation(model_dir)
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
        "-DMOTION_SPEC_ENABLE_INTROSPECTION=ON",
        # CMake defaults to no build type, which compiles the control loop unoptimized.
        # From ENV when there is one: a build type set by the sourced file is part of the
        # environment the caller asked to build under.
        f"-DCMAKE_BUILD_TYPE={(env or os.environ).get('MOTION_SPEC_BUILD_TYPE', 'RelWithDebInfo')}",
    ]
    if prefixes:
        configure.append(
            f"-DCMAKE_PREFIX_PATH={';'.join(str(path.resolve()) for path in prefixes)}"
        )
    from motion_spec.introspection.provenance import record_build

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
