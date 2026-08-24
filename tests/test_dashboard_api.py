# SPDX-License-Identifier: MPL-2.0
"""The dashboard's HTTP surface, over a real server on a real socket.

The functions behind these endpoints are tested elsewhere; what is tested here is the contract
a browser sees: which paths answer, what they answer with, and what they refuse.
"""

from __future__ import annotations

import json
import threading
import urllib.error
import urllib.parse
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

from motion_spec.dashboard import roots, server, sources

from motion_spec.generation.artifacts import build_frame_layout

from test_dashboard_runs import _archived_run, _contract_schema


@pytest.fixture
def dashboard(tmp_path, monkeypatch):
    """A server on an ephemeral port, rooted at a tree holding one archived run."""
    run = _archived_run(tmp_path, vendored=True)
    # the catalog knows a generation by its contract, and a run's length by its health file
    layout = run.parent.parent / "generated" / "contract" / "frame_layout.json"
    layout.parent.mkdir(parents=True)
    layout.write_text(json.dumps(build_frame_layout(_contract_schema())))
    (run / "logs" / "frame_log.pb.health.json").write_text(json.dumps({"written_frames": 1}))
    monkeypatch.setattr(roots, "GENERATIONS", tmp_path)
    monkeypatch.setattr(roots, "WORKSPACE", tmp_path)
    monkeypatch.setattr(server, "LIFECYCLE", None)
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), server.DashboardHandler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    host = f"http://127.0.0.1:{httpd.server_address[1]}"

    def get(path):
        with urllib.request.urlopen(f"{host}{path}") as response:
            return json.load(response)

    def post(path, body, content_type="application/json", origin=None):
        headers = {"Content-Type": content_type}
        if origin:
            headers["Origin"] = origin
        request = urllib.request.Request(
            f"{host}{path}", data=json.dumps(body).encode(), headers=headers
        )
        with urllib.request.urlopen(request) as response:
            return json.load(response)

    yield type(
        "Dashboard",
        (),
        {
            "get": staticmethod(get),
            "post": staticmethod(post),
            "host": host,
            "run": run,
            "root": tmp_path,
        },
    )
    httpd.shutdown()


def _relative(dashboard, path):
    return urllib.parse.quote(str(path.relative_to(dashboard.root)))


def test_the_index_and_its_assets_are_served(dashboard):
    with urllib.request.urlopen(f"{dashboard.host}/") as response:
        assert response.status == 200
        assert b"<title>" in response.read()


def test_generations_and_runs_are_listed(dashboard):
    generations = dashboard.get("/api/generations")
    assert [g["name"] for g in generations] == ["demo"]
    assert generations[0]["runs"] == 1
    assert generations[0]["built_at"].startswith("20")


def test_a_run_replays_with_its_constraints_and_events(dashboard):
    replay = dashboard.get(f"/api/replay?path={_relative(dashboard, dashboard.run)}")
    assert replay["frames"] == 1
    assert [c["name"] for c in replay["constraints"]] == ["hold-height", "settled"]
    assert replay["events"][0]["kind"] == "state"


def test_a_plot_samples_only_the_window_it_is_given(dashboard):
    path = _relative(dashboard, dashboard.run)
    whole = dashboard.get(f"/api/plot?path={path}&signal=timing.period_ms")
    windowed = dashboard.get(f"/api/plot?path={path}&signal=timing.period_ms&window=0&window=0")
    assert whole["first_frame"] == 0
    assert windowed["first_frame"] == 0
    assert len(windowed["signals"]["timing.period_ms"]) == 1


def test_an_unknown_signal_says_which_one(dashboard):
    path = _relative(dashboard, dashboard.run)
    with pytest.raises(urllib.error.HTTPError) as raised:
        dashboard.get(f"/api/plot?path={path}&signal=no_such_signal")
    assert "unknown signal: no_such_signal" in json.loads(raised.value.read())["error"]


def test_queries_are_kept_with_the_run(dashboard):
    path = str(dashboard.run.relative_to(dashboard.root))
    assert dashboard.get(f"/api/queries?path={urllib.parse.quote(path)}") == []
    assert dashboard.post("/api/queries", {"path": path, "queries": ["ASK { ?s ?p ?o }"]}) == {
        "saved": 1
    }
    assert dashboard.get(f"/api/queries?path={urllib.parse.quote(path)}") == ["ASK { ?s ?p ?o }"]
    assert json.loads((dashboard.run / "queries.json").read_text())["queries"] == [
        "ASK { ?s ?p ?o }"
    ]


def test_queries_belong_to_a_run_not_a_generation(dashboard):
    generation = str(dashboard.run.parent.parent.relative_to(dashboard.root))
    with pytest.raises(urllib.error.HTTPError) as raised:
        dashboard.post("/api/queries", {"path": generation, "queries": ["x"]})
    assert "queries belong to a run" in json.loads(raised.value.read())["error"]


def test_a_source_is_served_and_only_an_authored_one(dashboard):
    source = dashboard.run / "source" / "demo.robmot"
    served = dashboard.get(f"/api/source?path={_relative(dashboard, source)}")
    assert served["text"] == source.read_text()
    assert served["editors"]
    (dashboard.root / "notes.md").write_text("not a model")
    with pytest.raises(urllib.error.HTTPError) as raised:
        dashboard.get("/api/source?path=notes.md")
    assert "not an authored model file" in json.loads(raised.value.read())["error"]


def test_a_generation_lints_without_a_run(dashboard):
    """The lint reads the design graph, so it answers for a generation on its own -- and a
    generation whose graph declares nothing has nothing to say rather than failing."""
    generation = _relative(dashboard, dashboard.run.parent.parent)
    assert dashboard.get(f"/api/model/lint?path={generation}") == {"items": []}


def test_a_path_outside_the_root_is_refused(dashboard):
    for endpoint in ("/api/source", "/api/model/lint"):
        with pytest.raises(urllib.error.HTTPError) as raised:
            dashboard.get(f"{endpoint}?path=../../etc/passwd")
        assert json.loads(raised.value.read())["error"] == "unknown path"


def test_only_a_same_origin_json_post_is_accepted(dashboard):
    path = str(dashboard.run.relative_to(dashboard.root))
    for kwargs in ({"content_type": "text/plain"}, {"origin": "https://evil.example"}):
        with pytest.raises(urllib.error.HTTPError) as raised:
            dashboard.post("/api/queries", {"path": path, "queries": []}, **kwargs)
        assert raised.value.code == 403
        assert json.loads(raised.value.read())["error"] == "cross-origin request"


def test_deleting_a_run_moves_it_to_the_trash(dashboard, monkeypatch):
    trashed = []
    monkeypatch.setattr(server, "trash", lambda path: trashed.append(path))
    path = str(dashboard.run.relative_to(dashboard.root))
    assert dashboard.post("/api/delete", {"paths": [path]})["deleted"] == 1
    assert trashed == [dashboard.run]  # moved, never removed


def test_only_generations_and_runs_can_be_deleted(dashboard, monkeypatch):
    monkeypatch.setattr(server, "trash", lambda path: pytest.fail(f"trashed {path}"))
    (dashboard.root / "elsewhere").mkdir()
    with pytest.raises(urllib.error.HTTPError) as raised:
        dashboard.post("/api/delete", {"paths": ["elsewhere"]})
    assert "only generation bundles and run archives" in json.loads(raised.value.read())["error"]


def test_a_missing_jupyter_says_what_to_install(dashboard, monkeypatch):
    monkeypatch.setattr(sources.shutil, "which", lambda _name: None)
    with pytest.raises(urllib.error.HTTPError) as raised:
        dashboard.get("/api/jupyter")
    assert "pip install 'motion-spec[replay]'" in json.loads(raised.value.read())["error"]


def test_an_unknown_endpoint_is_a_404(dashboard):
    for call in (
        lambda: dashboard.get("/api/nonsense"),
        lambda: dashboard.post("/api/nonsense", {}),
    ):
        with pytest.raises(urllib.error.HTTPError) as raised:
            call()
        assert raised.value.code == 404
