# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""How a run's lifecycle is read back, and what verification asks of the rec document."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
import rdflib
from support import _source_tree, _start_run

from motion_spec.introspection.archive import (
    ArchiveError,
    create_archive_manifest,
    verify_manifest,
)
from motion_spec.introspection.provenance import (
    ensure_local_rec_importable,
    prov_uri,
    rec_run_lifecycle,
    rec_run_lifecycle_from_file,
)

ensure_local_rec_importable()

RUN_ID = "run-test"
RUN = rdflib.URIRef(prov_uri(f"run:{RUN_ID}"))
OSLC_AUTO = rdflib.Namespace("http://open-services.net/ns/auto#")
PROV_EXT = rdflib.Namespace("https://secorolab.github.io/metamodels/prov#")


def _archive(tmp_path: Path) -> Path:
    """One recorded run with its manifest and its lifecycle document."""
    source = _source_tree(tmp_path / "source")
    executable = tmp_path / "main"
    executable.write_text("binary\n")
    run_dir = tmp_path / RUN_ID
    _start_run(run_dir, source, executable, RUN_ID)
    create_archive_manifest(
        run_dir, source_dir=source, run_id=RUN_ID, log_producer_executable=executable
    )
    return run_dir


def _lifecycle(state: str, verdict: str) -> rdflib.Graph:
    graph = rdflib.Graph()
    graph.add((RUN, rdflib.RDF.type, PROV_EXT.Execution))
    graph.set((RUN, OSLC_AUTO.state, OSLC_AUTO[state]))
    graph.set((RUN, OSLC_AUTO.verdict, OSLC_AUTO[verdict]))
    return graph


@pytest.mark.parametrize(
    ("state", "verdict", "status"),
    [
        ("queued", "unavailable", "QUEUED"),
        ("inProgress", "unavailable", "RUNNING"),
        ("complete", "passed", "COMPLETED"),
        ("complete", "failed", "FAILED"),
        ("complete", "error", "INTERRUPTED"),
        ("canceled", "unavailable", "CANCELLED"),
    ],
)
def test_the_oslc_pair_is_what_says_how_a_run_ended(state, verdict, status) -> None:
    """REC records a lifecycle as an OSLC Automation (state, verdict); this is the whole map."""
    assert rec_run_lifecycle(_lifecycle(state, verdict))["status"] == status


def test_a_document_written_before_the_oslc_terms_has_no_status_and_does_not_raise(
    tmp_path: Path,
) -> None:
    """An archive from an older REC states its lifecycle as an rdf:type nothing reads now.

    It is still a document, and a run list that walks over one must report it as unknown
    rather than stop.
    """
    path = tmp_path / "rec.ld.json"
    path.write_text(
        json.dumps(
            {
                "@context": {"rec": "https://secorolab.github.io/metamodels/rec#"},
                "@id": str(RUN),
                "@type": "rec:CompletedRun",
                "rec:run-id": RUN_ID,
            }
        )
    )
    lifecycle = rec_run_lifecycle_from_file(path)

    assert lifecycle["status"] is None
    assert lifecycle["started_time"] is None and lifecycle["completed_time"] is None
    # And a document that is not there at all reads the same way.
    assert rec_run_lifecycle_from_file(tmp_path / "absent.ld.json")["status"] is None


def test_verification_writes_nothing(tmp_path: Path) -> None:
    run_dir = _archive(tmp_path)
    before = {path.name for path in run_dir.rglob("*")}

    assert verify_manifest(run_dir)["run_id"] == RUN_ID

    assert {path.name for path in run_dir.rglob("*")} == before


def test_a_rec_document_naming_another_run_is_reported_not_patched(tmp_path: Path) -> None:
    """The rec document has to describe this run; another IRI for it is a failed archive."""
    run_dir = _archive(tmp_path)
    rec_path = run_dir / "rec.ld.json"
    other = f"https://secorolab.github.io/rec/run/{RUN_ID}"
    rec_path.write_text(rec_path.read_text().replace(str(RUN), other))

    with pytest.raises(ArchiveError):
        verify_manifest(run_dir)
