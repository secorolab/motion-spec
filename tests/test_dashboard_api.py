# SPDX-License-Identifier: MPL-2.0
"""The dashboard's HTTP surface, over a real server on a real socket: what it refuses and what
it never deletes."""

from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest
from rec import State, Verdict, jsonld

from motion_spec.dashboard import roots, server
from motion_spec.generation.artifacts import build_frame_layout
from motion_spec.runs.provenance import rec_document

from support import DASHBOARD_SCHEMA

RUN = "demo/20260821T000000Z/runs/run-1"


@pytest.fixture
def host(tmp_path, monkeypatch):
    """A server on an ephemeral port, rooted at a tree holding one finished run at RUN."""
    generation = tmp_path / "demo" / "20260821T000000Z"
    layout = generation / "generated" / "contract" / "frame_layout.json"
    layout.parent.mkdir(parents=True)
    layout.write_text(json.dumps(build_frame_layout(DASHBOARD_SCHEMA)))
    run = generation / "runs" / "run-1"
    (run / "logs").mkdir(parents=True)
    (run / "manifest.json").write_text(json.dumps({"run_id": "run-1", "files": {}}))
    doc = jsonld.document(
        "https://example.test/run/r1", {"state": State.COMPLETE, "verdict": Verdict.PASSED}
    )
    rec_document(run).write_text(json.dumps(doc))
    monkeypatch.setattr(roots, "GENERATIONS", tmp_path)
    monkeypatch.setattr(roots, "WORKSPACE", tmp_path)
    monkeypatch.setattr(server, "LIFECYCLE", None)
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), server.DashboardHandler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{httpd.server_address[1]}"
    httpd.shutdown()


@pytest.mark.parametrize("endpoint", ["/api/source", "/api/model/lint"])
def test_a_path_outside_the_root_is_refused(host, endpoint):
    with pytest.raises(urllib.error.HTTPError) as raised:
        urllib.request.urlopen(f"{host}{endpoint}?path=../../etc/passwd")
    assert json.loads(raised.value.read())["error"] == "unknown path"


@pytest.mark.parametrize(
    "headers",
    [{"Content-Type": "text/plain"}, {"Content-Type": "application/json", "Origin": "https://evil.example"}],
    ids=["not-json", "cross-origin"],
)
def test_only_a_same_origin_json_post_is_accepted(host, headers):
    request = urllib.request.Request(
        f"{host}/api/queries",
        data=json.dumps({"path": RUN, "queries": []}).encode(),
        headers=headers,
    )
    with pytest.raises(urllib.error.HTTPError) as raised:
        urllib.request.urlopen(request)
    assert raised.value.code == 403
    assert json.loads(raised.value.read())["error"] == "cross-origin request"


def test_deleting_trashes_a_run_and_only_generations_and_runs(host, tmp_path, monkeypatch):
    trashed = []
    monkeypatch.setattr(server, "trash", trashed.append)
    (tmp_path / "elsewhere").mkdir()
    with pytest.raises(urllib.error.HTTPError) as raised:
        urllib.request.urlopen(
            urllib.request.Request(
                f"{host}/api/delete",
                data=json.dumps({"paths": ["elsewhere"]}).encode(),
                headers={"Content-Type": "application/json"},
            )
        )
    assert "only generation bundles and run archives" in json.loads(raised.value.read())["error"]

    request = urllib.request.Request(
        f"{host}/api/delete",
        data=json.dumps({"paths": [RUN]}).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(request) as response:
        assert json.load(response)["deleted"] == 1
    assert trashed == [tmp_path / RUN]  # moved, never removed
