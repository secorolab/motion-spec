# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest
import rdflib
from rdf_utils.models.vocab import URI_GEOM_PRED_X, URI_GEOM_TYPE_VECTOR_XYZ
from rdflib.namespace import PROV
from support import _source_tree

from motion_spec.introspection import runner
from motion_spec.introspection.archive import verify_manifest
from motion_spec.introspection.provenance import (
    _slug,
    prov_uri,
    rec_run_lifecycle,
    run_entity_uri,
)
from motion_spec.introspection.ros_video import real_camera_recordings
from motion_spec.introspection.runner import run_cataloged

REC = rdflib.Namespace("https://secorolab.github.io/metamodels/rec#")
PROV_EXT = rdflib.Namespace("https://secorolab.github.io/metamodels/prov#")
AGN = rdflib.Namespace("https://secorolab.github.io/metamodels/agent#")
QUDT = rdflib.Namespace("http://qudt.org/schema/qudt/")


def test_real_camera_recordings_selects_declared_ros_topics() -> None:
    schema = {
        "platform": {"simulated": False},
        "cameras": [
            {"id": "perception", "topic": "/perception/color", "rate_hz": 30.0},
            {"id": "rk", "topic": "/rk/color", "rate_hz": 15.0},
        ],
    }

    recordings = real_camera_recordings(schema, ["rk", "missing", "perception"])

    assert [(item.id, item.topic, item.rate_hz) for item in recordings] == [
        ("rk", "/rk/color", 15.0),
        ("perception", "/perception/color", 30.0),
    ]
    assert real_camera_recordings({**schema, "platform": {"simulated": True}}, ["rk"]) == []


def _log_copy_executable(path: Path) -> Path:
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import os\n"
        "import shutil\n"
        "import sys\n"
        "shutil.copyfile(sys.argv[1], os.environ['MOTION_SPEC_FRAME_LOG'])\n"
    )
    path.chmod(path.stat().st_mode | 0o111)
    return path


def _noisy_executable(path: Path, exit_code: int) -> Path:
    """Writes to both streams, copies a frame log when given one, then exits `exit_code`."""
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import os\n"
        "import shutil\n"
        "import sys\n"
        "print('hello from the run', flush=True)\n"
        "print('frame log:', repr(os.environ['MOTION_SPEC_FRAME_LOG']), flush=True)\n"
        "print('boom', file=sys.stderr, flush=True)\n"
        "if len(sys.argv) > 1:\n"
        "    shutil.copyfile(sys.argv[1], os.environ['MOTION_SPEC_FRAME_LOG'])\n"
        f"sys.exit({exit_code})\n"
    )
    path.chmod(path.stat().st_mode | 0o111)
    return path


def test_runner_catalogs_run_from_start_and_archives_outputs(tmp_path: Path) -> None:
    source = _source_tree(tmp_path / "source")
    executable = _log_copy_executable(tmp_path / "log-copy")
    run_dir = tmp_path / "run-001"

    result = run_cataloged(
        run_dir,
        source_dir=source,
        executable=executable,
        executable_args=[str(source / "frame_log.pb"), "--headless"],
        run_id="run-001",
    )

    assert result == 0
    manifest = verify_manifest(run_dir)
    assert manifest["run_id"] == "run-001"
    # Archiving packs the log, so the manifest names it as it now is on disk.
    assert manifest["files"]["frame_log"] == "logs/frame_log.pb.zst"
    assert manifest["files"]["log_producer_executable"] == "controller/executable/log-copy"
    assert "runtime_ttl" not in manifest["files"]

    rec_graph = rdflib.Graph().parse(run_dir / "rec.ld.json", format="json-ld")
    lifecycle = rec_run_lifecycle(rec_graph)
    assert lifecycle["status"] == "COMPLETED"
    assert lifecycle["started_time"]
    assert lifecycle["completed_time"]
    labels = {str(value) for value in rec_graph.objects(None, rdflib.RDFS.label)}
    assert {"log_producer_executable", "frame_log"} <= labels


def test_the_run_is_an_execution_of_the_executable_and_its_arguments(tmp_path: Path) -> None:
    """What the run used: the program, and the command line it was given. Not the IR, not the
    generation's own documents -- those are what the archive keeps, not what the run read."""
    source = _source_tree(tmp_path / "source")
    executable = _log_copy_executable(tmp_path / "log-copy")
    run_dir = tmp_path / "run-006"

    run_cataloged(
        run_dir,
        source_dir=source,
        executable=executable,
        executable_args=[str(source / "frame_log.pb"), "--headless"],
        run_id="run-006",
    )

    rec_graph = rdflib.Graph().parse(run_dir / "rec.ld.json", format="json-ld")
    run = rdflib.URIRef(prov_uri("run:run-006"))
    assert (run, rdflib.RDF.type, PROV_EXT.Execution) in rec_graph
    assert (run, PROV.wasAssociatedWith, rdflib.URIRef(prov_uri("agent:motion_spec"))) in rec_graph
    # The controller process acts for the runtime the model named, and every modelled robot
    # the generation declared is an agent of the run.
    controller = rdflib.URIRef(prov_uri("agent:controller_process"))
    runtime = rdflib.URIRef(prov_uri("agent:runtime_mujoco"))
    assert (controller, PROV.actedOnBehalfOf, runtime) in rec_graph
    assert (rdflib.URIRef(prov_uri("agent:modelled:arm1")), rdflib.RDF.type, AGN.ModelledAgent) in (
        rec_graph
    )

    arguments = rdflib.URIRef(run_entity_uri("run-006", "arguments"))
    assert (run, PROV.used, arguments) in rec_graph
    assert str(rec_graph.value(arguments, rdflib.RDFS.label)).endswith("frame_log.pb --headless")
    used_labels = {
        str(rec_graph.value(entity, rdflib.RDFS.label))
        for entity in rec_graph.objects(run, PROV.used)
    }
    assert "log_producer_executable" in used_labels
    assert not {"ir", "provenance", "model"} & used_labels


def test_interrupted_runner_records_a_terminal_state(tmp_path: Path, monkeypatch) -> None:
    source = _source_tree(tmp_path / "source")
    executable = _log_copy_executable(tmp_path / "log-copy")
    run_dir = tmp_path / "run-002"

    def interrupt(_executable, _args, *, cwd, frame_log, run_id, rec_path, **_recording):
        frame_log.parent.mkdir(parents=True)
        shutil.copyfile(source / "frame_log.pb", frame_log)
        runner._finish_rec_run(rec_path, run_id, "INTERRUPTED")
        return 130

    monkeypatch.setattr(runner, "_run_executable", interrupt)

    result = run_cataloged(run_dir, source_dir=source, executable=executable, run_id="run-002")

    assert result == 130
    rec_graph = rdflib.Graph().parse(run_dir / "rec.ld.json", format="json-ld")
    assert rec_run_lifecycle(rec_graph)["status"] == "INTERRUPTED"


def test_a_draw_is_a_generalization_of_the_quantity_it_sampled(tmp_path: Path) -> None:
    """Sampling is an activity of its own: it used the quantity and generated this run's draw."""
    source = _source_tree(tmp_path / "source")
    executable = _log_copy_executable(tmp_path / "log-copy")
    run_dir = tmp_path / "run-007"
    quantity = "https://example.test/spec/start-pose"

    def sampling(*args, **kwargs):
        (run_dir / "logs").mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / "frame_log.pb", run_dir / "logs" / "frame_log.pb")
        (run_dir / "logs" / "sampling.json").write_text(
            json.dumps(
                {
                    "seed": 7,
                    "drawn_at": "2026-09-19T10:00:00.000000Z",
                    "draws": {quantity: {"distribution": "d", "values": [0.1, 0.2, 0.3]}},
                }
            )
        )
        return 0

    monkeypatch_run = pytest.MonkeyPatch()
    monkeypatch_run.setattr(runner, "_run_executable", sampling)
    try:
        run_cataloged(run_dir, source_dir=source, executable=executable, run_id="run-007")
    finally:
        monkeypatch_run.undo()

    rec_graph = rdflib.Graph().parse(run_dir / "rec.ld.json", format="json-ld")
    draw = rdflib.URIRef(run_entity_uri("run-007", f"draw/{quantity}"))
    activity = rdflib.URIRef(f"{prov_uri('run:run-007')}/sampling/{_slug(quantity)}")

    assert (activity, rdflib.RDF.type, PROV_EXT.Generalization) in rec_graph
    assert (activity, PROV.used, rdflib.URIRef(quantity)) in rec_graph
    assert (rdflib.URIRef(quantity), rdflib.RDF.type, PROV.Entity) in rec_graph
    assert (draw, PROV.wasGeneratedBy, activity) in rec_graph
    assert (draw, PROV.specializationOf, rdflib.URIRef(quantity)) in rec_graph
    # A three-vector is a coordinate, not a bare number.
    assert (draw, rdflib.RDF.type, URI_GEOM_TYPE_VECTOR_XYZ) in rec_graph
    assert rec_graph.value(draw, URI_GEOM_PRED_X).toPython() == 0.1
    # The seed stays what it always was: a metric of the run.
    seeds = [
        rec_graph.value(metric, QUDT.value)
        for metric in rec_graph.subjects(rdflib.RDF.type, REC.Metric)
        if str(rec_graph.value(metric, rdflib.RDFS.label)) == "sampling/seed"
    ]
    assert seeds and seeds[0].toPython() == 7


def test_console_is_captured_and_mirrored(tmp_path: Path, capfd) -> None:
    source = _source_tree(tmp_path / "source")
    executable = _noisy_executable(tmp_path / "noisy", 0)
    run_dir = tmp_path / "run-003"

    result = run_cataloged(
        run_dir,
        source_dir=source,
        executable=executable,
        executable_args=[str(source / "frame_log.pb")],
        run_id="run-003",
    )

    assert result == 0
    console = (run_dir / "logs" / "console.log").read_text()
    assert "hello from the run" in console
    assert "boom" in console
    assert "hello from the run" in capfd.readouterr().out
    manifest = verify_manifest(run_dir)
    assert manifest["files"]["console"] == "logs/console.log"


def test_run_without_a_log_still_catalogs_itself(tmp_path: Path) -> None:
    # --no-log: the runtime is told to record nothing (an empty path), and what the run leaves
    # -- console, rec, a manifest that says so -- must still stand on its own.
    source = _source_tree(tmp_path / "source")
    executable = _noisy_executable(tmp_path / "noisy-quiet", 0)
    run_dir = tmp_path / "run-005"

    result = run_cataloged(
        run_dir,
        source_dir=source,
        executable=executable,
        run_id="run-005",
        record_log=False,
    )

    assert result == 0
    console = (run_dir / "logs" / "console.log").read_text()
    assert "frame log: ''" in console
    assert not (run_dir / "logs" / "frame_log.pb").exists()
    assert (run_dir / "rec.ld.json").exists()
    manifest = verify_manifest(run_dir)
    assert manifest["recorded"] is False
    assert "frame_log" not in manifest["files"]
    assert manifest["files"]["console"] == "logs/console.log"


def test_crashed_run_leaves_its_error_output(tmp_path: Path) -> None:
    # No frame log, no manifest -- console.log is the only evidence of why the run died.
    source = _source_tree(tmp_path / "source")
    executable = _noisy_executable(tmp_path / "noisy-fail", 3)
    run_dir = tmp_path / "run-004"

    result = run_cataloged(run_dir, source_dir=source, executable=executable, run_id="run-004")

    assert result == 3
    assert not (run_dir / "manifest.json").exists()
    assert "boom" in (run_dir / "logs" / "console.log").read_text()
