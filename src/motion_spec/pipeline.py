# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: OpenAI

"""High-level generation and build pipeline for motion models."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import urllib.parse
from datetime import UTC, datetime
from pathlib import Path

import rdflib

from motion_spec.namespace import APP

PROV = rdflib.Namespace("http://www.w3.org/ns/prov#")


def _file_path(value: rdflib.term.Identifier) -> Path | None:
    parsed = urllib.parse.urlparse(str(value))
    return Path(urllib.parse.unquote(parsed.path)) if parsed.scheme == "file" else None


def _rewrite_locations(document: Path, locations: dict[Path, Path]) -> None:
    dataset = rdflib.Dataset().parse(document, format="json-ld")
    for subject, predicate, location, context in list(
        dataset.quads((None, PROV.atLocation, None, None))
    ):
        source = _file_path(location)
        target = locations.get(source.resolve()) if source else None
        if (
            target is None
            and source is not None
            and source.exists()
            and source.is_relative_to(document.parent.parent)
        ):
            target = source
        if target is None:
            continue
        graph = dataset.graph(context)
        graph.remove((subject, predicate, location))
        graph.add(
            (
                subject,
                predicate,
                rdflib.URIRef(
                    urllib.parse.quote(os.path.relpath(target, document.parent), safe="/..")
                ),
            )
        )
    dataset.serialize(document, format="json-ld", indent=2)


def _organize_generation(model_dir: Path, controller_dir: Path | None = None) -> None:
    generated = model_dir.parent
    provenance_dir = generated / "provenance"
    source_dir = generated / "source"
    for directory in (provenance_dir, source_dir):
        directory.mkdir(exist_ok=True)

    locations: dict[Path, Path] = {}
    coord_provenance = model_dir / "provenance.ld.json"
    target_coord_provenance = provenance_dir / "coord-dsl.ld.json"
    if coord_provenance.is_file():
        locations[coord_provenance.resolve()] = target_coord_provenance
        coord_provenance.replace(target_coord_provenance)
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
                if controller_dir and (source.name == "fsm_ir.json" or source.name.endswith("_fsm.hpp"))
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

    target_dsl_provenance = provenance_dir / "dsl.ld.json"
    locations[dsl_provenance.resolve()] = target_dsl_provenance
    dsl_provenance.replace(target_dsl_provenance)
    (model_dir / "provenance").rmdir()

    app_manifest = next(model_dir.glob("*-app.ld.json"))
    app = rdflib.Dataset().parse(app_manifest, format="json-ld")
    for subject, predicate, imported, context in list(
        app.quads((None, APP["import"], None, None))
    ):
        if str(imported).endswith("provenance/dsl.ld.json"):
            graph = app.graph(context)
            graph.remove((subject, predicate, imported))
            graph.add((subject, predicate, rdflib.URIRef("../provenance/dsl.ld.json")))
    for subject, predicate, path, context in list(app.quads((None, APP.path, None, None))):
        graph = app.graph(context)
        graph.remove((subject, predicate, path))
        graph.add((subject, predicate, rdflib.Literal(".")))
    app.serialize(app_manifest, format="json-ld", indent=2)

    _rewrite_locations(target_dsl_provenance, locations)
    if target_coord_provenance.is_file():
        _rewrite_locations(target_coord_provenance, locations)
    if controller_dir:
        contract_dir = generated / "contract"
        contract_dir.mkdir()
        for artifact in ("schema.json", "frame_layout.json", "frame_log.proto"):
            source = controller_dir / artifact
            target = contract_dir / artifact
            source.replace(target)
            locations[source.resolve()] = target
        motion_spec_provenance = provenance_dir / "motion-spec.ld.json"
        codegen_provenance = controller_dir / "provenance.ld.json"
        codegen_provenance.replace(motion_spec_provenance)
        locations[codegen_provenance.resolve()] = motion_spec_provenance
        _rewrite_locations(motion_spec_provenance, locations)
        shutil.rmtree(controller_dir / ".stst")

def new_id(name: str) -> str:
    """Return a time-ordered identifier unique to one generation or run."""
    return f"{name}-{datetime.now(UTC).strftime('%Y%m%dT%H%M%S%fZ')}"


def create_generation_dir(model: Path, output_dir: Path | None = None) -> Path:
    """Create a new, uniquely identified generation directory for MODEL."""
    generation = output_dir or Path.cwd() / "generation" / new_id(model.stem)
    generation.mkdir(parents=True, exist_ok=False)
    return generation.resolve()


def generate_model(model: Path, generation: Path, *, stage: str = "code") -> Path:
    """Generate MODEL through IR or C++ code and return its generated-artifact directory."""
    from motion_spec.check import validate_manifest
    from motion_spec.entities import DataclassJSONEncoder
    from motion_spec.ir_gen import generate_ir

    generated = generation / "generated"
    model_dir = generated / "model"
    dsl = subprocess.run(
        ["textx", "generate", str(model.resolve()), "--target", "jsonld", "-o", str(model_dir)],
        cwd=model.parent,
    )
    if dsl.returncode:
        # The DSL already reported the offending line on stderr; don't bury it under an argv dump.
        raise RuntimeError(f"the DSL rejected {model.name}, see the error above")
    manifest = model_dir / f"{model.stem}-app.ld.json"
    conforms, report = validate_manifest(manifest)
    if not conforms:
        raise RuntimeError(f"generated RDF failed SHACL validation:\n{report}")
    ir_path = model_dir / "ir.json"
    ir_path.write_text(json.dumps(generate_ir(manifest), cls=DataclassJSONEncoder, indent=4))
    if stage == "code":
        from motion_spec.codegen import generate_code
        from motion_spec.setup import find_stst

        controller_dir = generated / "controller"
        controller_dir.mkdir()
        for artifact in (*model_dir.glob("*_fsm.hpp"), model_dir / "fsm_ir.json"):
            if artifact.is_file():
                artifact.replace(controller_dir / artifact.name)
        generate_code(ir_path, controller_dir, find_stst() or "stst")
        _organize_generation(model_dir, controller_dir)
    else:
        _organize_generation(model_dir)
    return generated


def build_generation(
    generation: Path, *, prefixes: tuple[Path, ...] = (), jobs: int | None = None
) -> Path:
    """Configure and build GENERATION, returning the controller executable."""
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
    ]
    if prefixes:
        configure.append(f"-DCMAKE_PREFIX_PATH={';'.join(str(path.resolve()) for path in prefixes)}")
    subprocess.run(configure, check=True)
    command = ["cmake", "--build", str(build), "--parallel"]
    if jobs is not None:
        command.append(str(jobs))
    subprocess.run(command, check=True)
    executable = build / "main"
    if not executable.is_file():
        raise RuntimeError(f"controller executable not found after build: {executable}")
    return executable
