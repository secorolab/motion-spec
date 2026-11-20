# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

"""The on-disk generated tree a run is recorded and verified against."""

from __future__ import annotations

import json
import shutil
from datetime import UTC, datetime
from pathlib import Path

import pytest
from rdf_utils import DatasetFormat
from rdf_utils.models.prov import (
    add_agent,
    add_directory_location,
    capture_file_prov,
    create_bundle,
    get_py_pkg_info,
    load_transformation_prov,
)
from rdflib import Dataset
from rdflib.namespace import PROV
from support import FRAME, MODEL_GRAPH, PROVENANCE, SCHEMA

from motion_spec.generation.artifacts import (
    build_frame_layout,
    build_frame_log_header_record,
    field_names_and_format,
)
from motion_spec.runs.provenance import GENERATION_DOCUMENT, add_package
from motion_spec.telemetry import frame_log_pb


@pytest.fixture
def source_tree(tmp_path: Path) -> Path:
    """A generated/ tree as generation lays it out, plus the log a run of it would write.

    TMP_PATH is the generation: a run of it goes in TMP_PATH/runs/<run-id>.
    """
    path = tmp_path / "generated"
    (path / "contract").mkdir(parents=True)
    (path / "contract" / "frame_layout.json").write_text(
        json.dumps(build_frame_layout(SCHEMA), indent=4)
    )
    shutil.copyfile(frame_log_pb.PROTO, path / "contract" / "frame_log.proto")
    (path / GENERATION_DOCUMENT).parent.mkdir()
    (path / GENERATION_DOCUMENT).write_text(json.dumps(PROVENANCE, indent=4))
    (path / "model").mkdir()
    model_document = {**PROVENANCE, "@graph": [*PROVENANCE["@graph"], MODEL_GRAPH]}
    (path / "model" / "model-app.ld.json").write_text(json.dumps(model_document, indent=4))
    (path / "model" / "ir.json").write_text(
        json.dumps({"configuration": {"platform": {"simulated": True}}})
    )
    # A run runs what the generation's build generated.
    (tmp_path / "main").write_text("binary\n")
    (path / "controller" / "headers").mkdir(parents=True)
    (path / "controller" / "headers" / "runtime.hpp").write_text("// generated runtime\n")
    (path / "controller" / "main.cpp").write_text("// generated main\n")
    # motion-spec's bundle: the IR derived from the DSL's document, the controller sources from
    # the IR and the build's executable from them, which the runner and the verifier read back.
    dataset = Dataset()
    bundle = dataset.graph(create_bundle(dataset, get_py_pkg_info("motion_spec").purl))
    roots = {tmp_path.resolve(): add_directory_location(bundle, tmp_path)}
    now = datetime.now(UTC)
    motion_spec = add_package(bundle, "motion_spec")
    dsl = get_py_pkg_info("motion_spec_dsl").purl
    add_agent(bundle, dsl, (PROV.SoftwareAgent,), "motion_spec_dsl")
    document = capture_file_prov(bundle, path / "model" / "model-app.ld.json", roots=roots)
    bundle.add((document, PROV.wasAttributedTo, dsl))
    ir = capture_file_prov(bundle, path / "model" / "ir.json", roots=roots)
    load_transformation_prov(bundle, [document], [ir], motion_spec, started=now, ended=now)
    sources = [
        capture_file_prov(bundle, path / "controller" / name, roots=roots)
        for name in ("headers/runtime.hpp", "main.cpp")
    ]
    load_transformation_prov(bundle, [ir], sources, motion_spec, started=now, ended=now)
    executable = capture_file_prov(bundle, tmp_path / "main", roots=roots)
    load_transformation_prov(bundle, sources, [executable], motion_spec, started=now, ended=now)
    (path / GENERATION_DOCUMENT).parent.mkdir(parents=True, exist_ok=True)
    (path / GENERATION_DOCUMENT).write_text(dataset.serialize(format=DatasetFormat.JSON_LD))
    flat = {name: 0 for name in field_names_and_format(SCHEMA["pools"])[1]} | FRAME
    # The writer closes its log with its own account of the frames it was handed.
    health = frame_log_pb.record_class_of(frame_log_pb.descriptor_set())()
    health.health.attempted_frames = health.health.accepted_frames = 1
    health.health.written_frames = 1
    health.health.complete = True
    with (path / "frame_log.pb").open("wb") as log:
        frame_log_pb.write_delimited(log, build_frame_log_header_record(SCHEMA))
        frame_log_pb.write_delimited(log, frame_log_pb.frame_record(flat, SCHEMA))
        frame_log_pb.write_delimited(log, health.SerializeToString())
    return path
