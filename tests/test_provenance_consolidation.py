# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The run's documents consolidate into one dataset at the end of the run.

Consolidation is a report about what was recorded, so a dataset that will not join is printed
and the run stands; what the manifest promises is only what was written, and verification holds
the promise to account.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
import rdflib
from support import _source_tree

from motion_spec.introspection.archive import (
    ArchiveError,
    consolidate_provenance,
    create_archive_manifest,
    verify_manifest,
)
from motion_spec.introspection.provenance import ensure_local_rec_importable, prov_uri
from motion_spec.introspection.replay import decode_frames
from motion_spec.introspection.runtime_graph import write_runtime_ttl

ensure_local_rec_importable()

RUN_ID = "run-test"
RUN = rdflib.URIRef(prov_uri(f"run:{RUN_ID}"))
MS_PROV = rdflib.Namespace("https://secorolab.github.io/metamodels/motion-spec/prov#")
REC = rdflib.Namespace("https://secorolab.github.io/metamodels/rec#")


def _archive(tmp_path: Path) -> Path:
    """One recorded run with its manifest, lifecycle document and runtime graph."""
    source = _source_tree(tmp_path / "source")
    run_dir = tmp_path / RUN_ID
    create_archive_manifest(run_dir, source_dir=source, run_id=RUN_ID)
    write_runtime_ttl(run_dir, decode_frames(run_dir / "logs" / "frame_log.pb"))
    return run_dir


def test_consolidation_writes_the_dataset_and_the_manifest_entry(tmp_path: Path) -> None:
    from rec.consolidate import INFERRED_GRAPH, REC_GRAPH, RUNTIME_GRAPH

    run_dir = _archive(tmp_path)
    path = consolidate_provenance(run_dir)

    assert path == run_dir / "provenance.trig"
    manifest = json.loads((run_dir / "manifest.json").read_text())
    assert manifest["files"]["provenance_trig"] == "provenance.trig"
    dataset = rdflib.Dataset()
    dataset.parse(path, format="trig")
    assert (RUN, rdflib.RDF.type, MS_PROV.TaskExecution) in dataset.graph(RUNTIME_GRAPH)
    assert (RUN, REC["run-id"], None) in dataset.graph(REC_GRAPH)
    assert (RUN, rdflib.RDF.type, rdflib.PROV.Activity) in dataset.graph(INFERRED_GRAPH)
    assert verify_manifest(run_dir)["files"]["provenance_trig"] == "provenance.trig"


def test_verification_holds_the_manifest_to_its_promise(tmp_path: Path) -> None:
    run_dir = _archive(tmp_path)
    consolidate_provenance(run_dir)
    (run_dir / "provenance.trig").unlink()
    with pytest.raises(ArchiveError, match="provenance.trig"):
        verify_manifest(run_dir)


def test_an_old_vocabulary_archive_reports_the_migration_and_leaves_the_run_alone(
    tmp_path: Path, capsys
) -> None:
    run_dir = _archive(tmp_path)
    (run_dir / "runtime" / "runtime.ttl").write_text(
        "@prefix prov: <http://www.w3.org/ns/prov#> .\n"
        f"<https://secorolab.github.io/motion-spec/runtime/run/{RUN_ID}> a prov:Entity .\n"
    )
    assert consolidate_provenance(run_dir) is None
    assert "--recover-runtime-ttl" in capsys.readouterr().err
    assert "provenance_trig" not in json.loads((run_dir / "manifest.json").read_text())["files"]
    assert not (run_dir / "provenance.trig").exists()


def test_a_run_id_match_without_the_shared_iri_is_reported_not_patched(
    tmp_path: Path, capsys
) -> None:
    run_dir = _archive(tmp_path)
    rec_path = run_dir / "rec.ld.json"
    other = f"https://secorolab.github.io/rec/run/{RUN_ID}"
    rec_path.write_text(rec_path.read_text().replace(str(RUN), other))

    assert consolidate_provenance(run_dir) is None
    message = capsys.readouterr().err
    assert str(RUN) in message and other in message
    assert not (run_dir / "provenance.trig").exists()
