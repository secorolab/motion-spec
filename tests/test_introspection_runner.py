# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

import base64
import io
import shutil
import threading
import time
from pathlib import Path

import rdflib
from frame_log_fixture import occurrence
from support import _schema, _source_tree

from motion_spec.generation.artifacts import build_frame_log_header_record
from motion_spec.introspection import frame_log_pb, runner
from motion_spec.introspection.archive import verify_manifest
from motion_spec.introspection.provenance import prov_uri, rec_run_lifecycle
from motion_spec.introspection.ros_video import real_camera_recordings
from motion_spec.introspection.runner import run_cataloged
from motion_spec.introspection.runtime_graph import RuntimeGraphWriter

REC = rdflib.Namespace("https://secorolab.github.io/metamodels/rec#")


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


def _records(source: Path) -> list[str]:
    """What a stub run is handed to reproduce: the two records a real run writes."""
    return [str(source / "frame_log.pb"), str(source / "occurrences.pb")]


def _log_copy_executable(path: Path) -> Path:
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import os\n"
        "import shutil\n"
        "import sys\n"
        "shutil.copyfile(sys.argv[1], os.environ['MOTION_SPEC_FRAME_LOG'])\n"
        "shutil.copyfile(sys.argv[2], os.environ['MOTION_SPEC_OCCURRENCE_LOG'])\n"
    )
    path.chmod(path.stat().st_mode | 0o111)
    return path


def _noisy_executable(path: Path, exit_code: int) -> Path:
    """Writes to both streams, copies the run's records when given them, then exits."""
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import os\n"
        "import shutil\n"
        "import sys\n"
        "print('hello from the run', flush=True)\n"
        "print('frame log:', repr(os.environ['MOTION_SPEC_FRAME_LOG']), flush=True)\n"
        "print('boom', file=sys.stderr, flush=True)\n"
        "if len(sys.argv) > 2:\n"
        "    shutil.copyfile(sys.argv[1], os.environ['MOTION_SPEC_FRAME_LOG'])\n"
        "    shutil.copyfile(sys.argv[2], os.environ['MOTION_SPEC_OCCURRENCE_LOG'])\n"
        f"sys.exit({exit_code})\n"
    )
    path.chmod(path.stat().st_mode | 0o111)
    return path


def _slow_stream_executable(path: Path, payload: str) -> Path:
    """Writes a complete occurrence stream, then holds the run open for three seconds."""
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import base64\n"
        "import os\n"
        "import sys\n"
        "import time\n"
        "with open(os.environ['MOTION_SPEC_OCCURRENCE_LOG'], 'wb') as fh:\n"
        "    fh.write(base64.b64decode(sys.argv[1]))\n"
        "    fh.flush()\n"
        "time.sleep(3)\n"
    )
    path.chmod(path.stat().st_mode | 0o111)
    return path


def _stream_payload() -> str:
    """A header record and one state change, base64 so a stub can write it with no imports."""
    schema = _schema()
    buffer = io.BytesIO()
    frame_log_pb.write_delimited(buffer, build_frame_log_header_record(schema))
    frame_log_pb.write_delimited(
        buffer,
        frame_log_pb.occurrence_record(
            occurrence("STATE_CHANGE", 0, state_since_wall_ns=100), schema
        ),
    )
    return base64.b64encode(buffer.getvalue()).decode()


def test_runner_catalogs_run_from_start_and_archives_outputs(tmp_path: Path) -> None:
    source = _source_tree(tmp_path / "source")
    executable = _log_copy_executable(tmp_path / "log-copy")
    run_dir = tmp_path / "run-001"

    result = run_cataloged(
        run_dir,
        source_dir=source,
        executable=executable,
        executable_args=_records(source),
        run_id="run-001",
    )

    assert result == 0
    manifest = verify_manifest(run_dir)
    assert manifest["run_id"] == "run-001"
    # Archiving packs the log, so the manifest names it as it now is on disk. The occurrence
    # stream is never packed: it is small, and the graph is projected from it as it grows.
    assert manifest["files"]["frame_log"] == "logs/frame_log.pb.zst"
    assert manifest["files"]["occurrences"] == "logs/occurrences.pb"
    assert manifest["files"]["log_producer_executable"] == "controller/executable/log-copy"
    # The graph was projected while the run ran, and the journal beside it holds every triple.
    assert manifest["files"]["runtime_ttl"] == "runtime/runtime.ttl"
    nt_lines = (run_dir / "runtime" / "runtime.nt").read_text().splitlines()
    assert len(nt_lines) >= len(rdflib.Graph().parse(run_dir / "runtime" / "runtime.ttl"))

    # REC writes a PROV graph: lifecycle is an rdf:type on the run, roles are rdfs:label.
    rec_graph = rdflib.Graph().parse(run_dir / "rec.ld.json", format="json-ld")
    lifecycle = rec_run_lifecycle(rec_graph)
    assert lifecycle["status"] == "COMPLETED"
    assert lifecycle["started_time"]
    assert lifecycle["completed_time"]
    labels = {str(value) for value in rec_graph.objects(None, rdflib.RDFS.label)}
    assert {"log_producer_executable", "frame_log", "runtime_ttl"} <= labels
    # rec and the runtime graph describe one run node, not two.
    assert (rdflib.URIRef(prov_uri("run:run-001")), rdflib.RDF.type, REC.CompletedRun) in rec_graph
    assert (
        rdflib.URIRef(prov_uri("activity:run_cataloging")),
        rdflib.RDF.type,
        rdflib.URIRef("http://www.w3.org/ns/prov#Activity"),
    ) in rec_graph


def test_interrupted_run_keeps_the_graph_it_had_projected(tmp_path: Path, monkeypatch) -> None:
    """A run killed partway leaves the journal complete to its last occurrence, and a graph."""
    source = _source_tree(tmp_path / "source")
    executable = _log_copy_executable(tmp_path / "log-copy")
    run_dir = tmp_path / "run-002"

    def interrupt(
        _executable, _args, *, cwd, frame_log, occurrence_log, run_id, rec_path, graph_writer, **_r
    ):
        frame_log.parent.mkdir(parents=True)
        shutil.copyfile(source / "frame_log.pb", frame_log)
        shutil.copyfile(source / "occurrences.pb", occurrence_log)
        graph_writer.start()
        try:
            runner._finish_rec_run(rec_path, run_id, "INTERRUPTED")
        finally:
            graph_writer.stop()
        return 130

    monkeypatch.setattr(runner, "_run_executable", interrupt)

    result = run_cataloged(run_dir, source_dir=source, executable=executable, run_id="run-002")

    assert result == 130
    graph = rdflib.Graph().parse(run_dir / "runtime" / "runtime.ttl", format="turtle")
    journal = (run_dir / "runtime" / "runtime.nt").read_text().splitlines()
    # The journal ends at the last occurrence the run got to; the Turtle is one view of it.
    assert journal and len(journal) >= len(graph)
    rec_graph = rdflib.Graph().parse(run_dir / "rec.ld.json", format="json-ld")
    assert rec_run_lifecycle(rec_graph)["status"] == "INTERRUPTED"


def test_the_graph_names_the_run_before_the_executable_exits(tmp_path: Path) -> None:
    """runtime.ttl is on disk and names the run while the run is still going."""
    source = _source_tree(tmp_path / "source")
    executable = _slow_stream_executable(tmp_path / "slow", _stream_payload())
    run_dir = tmp_path / "run-006"
    outcome: list = []

    thread = threading.Thread(
        target=lambda: outcome.append(
            run_cataloged(
                run_dir,
                source_dir=source,
                executable=executable,
                executable_args=[_stream_payload()],
                run_id="run-006",
            )
        )
    )
    thread.start()
    runtime_ttl = run_dir / "runtime" / "runtime.ttl"
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not runtime_ttl.exists():
        time.sleep(0.05)
    assert runtime_ttl.exists()
    assert "run-006" in runtime_ttl.read_text()
    thread.join(timeout=30)
    assert outcome == [0]


def test_the_writer_leaves_nothing_when_the_run_recorded_nothing(tmp_path: Path) -> None:
    """No stream is no graph: the writer states nothing rather than an empty record."""
    writer = RuntimeGraphWriter(
        tmp_path, run_id="run-007", occurrence_log=tmp_path / "logs" / "occurrences.pb"
    )
    writer.start()
    assert writer.stop() is None
    assert not (tmp_path / "runtime").exists()


def test_console_is_captured_and_mirrored(tmp_path: Path, capfd) -> None:
    source = _source_tree(tmp_path / "source")
    executable = _noisy_executable(tmp_path / "noisy", 0)
    run_dir = tmp_path / "run-003"

    result = run_cataloged(
        run_dir,
        source_dir=source,
        executable=executable,
        executable_args=_records(source),
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
        run_dir, source_dir=source, executable=executable, run_id="run-005", record_log=False
    )

    assert result == 0
    console = (run_dir / "logs" / "console.log").read_text()
    assert "frame log: ''" in console
    assert not (run_dir / "logs" / "frame_log.pb").exists()
    # Nothing was recorded, so there is nothing to project and nothing promises a graph.
    assert not (run_dir / "runtime").exists()
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
