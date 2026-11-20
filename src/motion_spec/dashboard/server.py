# SPDX-License-Identifier: MPL-2.0

"""Serve recorded motion-spec generations and runs to the dashboard frontend."""

from __future__ import annotations

import atexit
import contextlib
import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import ClassVar
from urllib.parse import parse_qs, urlparse

from motion_spec.dashboard import roots, ros_camera
from motion_spec.dashboard.catalog import (
    drift_summary,
    generation_details,
    generation_summary,
    read_generated,
    run_files,
    run_summary,
    source_drift,
    video_file,
)
from motion_spec.dashboard.cleanup import preview, restore, trash_entries
from motion_spec.dashboard.jobs import (
    console_log_for,
    console_slice,
    generate_console,
    generate_status,
    generation_status,
    health_report,
    pick_environment,
    run_status,
    start_generate,
    start_run,
    stop_run,
    use_environment,
)
from motion_spec.dashboard.live import live_state, run_control
from motion_spec.dashboard.metadata import (
    LOCK,
    annotations,
    model_folder,
    save_annotations,
    set_baseline,
    set_protected,
)
from motion_spec.dashboard.notebook import jupyter_server, run_notebook, stop_jupyter
from motion_spec.dashboard.queries import (
    model_lint,
    run_notes,
    run_query,
    save_notes,
    save_queries,
    saved_queries,
)
from motion_spec.dashboard.replay import plot_series, replay_metadata, run_verdict
from motion_spec.dashboard.roots import (
    FRONTEND,
    HEADER_REL,
    current_roots,
    directory_size,
    expected_path,
    json_file,
    pick_root,
    relative_path,
    set_root,
    storage_totals,
)
from motion_spec.dashboard.runs import GenerationCatalog, GenerationInfo
from motion_spec.dashboard.sources import (
    authored_sources,
    git_checkout,
    git_diff,
    open_source,
    open_terminal,
    read_source,
    save_source,
    source_path,
)
from motion_spec.devices import probe_devices
from motion_spec.runs.lifecycle_events import socket_path
from motion_spec.utils import trash

LIFECYCLE = None
PORT = 0


def generation_runs(value: str, query: dict[str, list[str]]) -> list[dict]:
    """The runs of generation VALUE, or with `model` those of every generation of its model."""
    generation = relative_path(roots.GENERATIONS, value)
    if not query.get("model"):
        return [run_summary(run.dir) for run in GenerationInfo(generation).runs]
    model = generation.relative_to(roots.GENERATIONS).parts[0]
    return [
        run_summary(run.dir) | {"generation_label": annotations(g.dir)["label"]}
        for g in GenerationCatalog(roots.GENERATIONS).generations()
        if g.model == model
        for run in g.runs
    ]


def target_status(value: str, query: dict[str, list[str]]) -> dict:
    """The status of VALUE: a generation's when it holds a frame log header, else a run's."""
    target = relative_path(roots.GENERATIONS, value)
    if (target / HEADER_REL).exists():
        return generation_status(target)
    return run_status(target)


def environment_health(value: str, query: dict[str, list[str]]) -> dict:
    """The health report, or with `env` the report after switching to that environment."""
    if "env" in query:
        return use_environment(query["env"][0])
    return health_report(bool(query.get("refresh")))


def drift(value: str, query: dict[str, list[str]]) -> dict:
    """How generation VALUE's sources drifted: a summary, or one `file`'s or all files' drift."""
    generation = relative_path(roots.GENERATIONS, value)
    if query.get("summary"):
        return drift_summary(generation)
    return source_drift(generation, query.get("file", [""])[0] or None)


def console_text(value: str, query: dict[str, list[str]]) -> dict:
    """The console from `offset` on: of a generate `job`, or of run VALUE."""
    offset = int(query.get("offset", ["0"])[0])
    job = query.get("job", [""])[0]
    if job:
        return generate_console(job, offset)
    # a run directory is named before it exists
    return console_slice(console_log_for(expected_path(roots.GENERATIONS, value)), offset)


def plot_window(value: str, query: dict[str, list[str]]) -> dict:
    """Run VALUE's `signal` series, cut to a two-bound `window` when one is given."""
    bounds = query.get("window", [])
    return plot_series(
        relative_path(roots.GENERATIONS, value),
        query.get("signal", []),
        (int(bounds[0]), int(bounds[1])) if len(bounds) == 2 else None,
    )


GET_JSON = {
    "/api/roots": lambda value, query: current_roots(),
    "/api/trash": lambda value, query: trash_entries(),
    "/api/pick-env": lambda value, query: pick_environment(),
    "/api/storage": lambda value, query: storage_totals(),
    "/api/sources": lambda value, query: authored_sources(),
    "/api/annotations": lambda value, query: annotations(relative_path(roots.GENERATIONS, value)),
    "/api/generation": lambda value, query: generation_details(
        relative_path(roots.GENERATIONS, value)
    ),
    "/api/generated": lambda value, query: read_generated(relative_path(roots.GENERATIONS, value)),
    "/api/model/lint": lambda value, query: model_lint(relative_path(roots.GENERATIONS, value)),
    "/api/run/files": lambda value, query: run_files(relative_path(roots.GENERATIONS, value)),
    "/api/run/verdict": lambda value, query: run_verdict(relative_path(roots.GENERATIONS, value)),
    "/api/devices": lambda value, query: probe_devices(relative_path(roots.GENERATIONS, value)),
    "/api/queries": lambda value, query: saved_queries(expected_path(roots.GENERATIONS, value)),
    "/api/notes": lambda value, query: run_notes(expected_path(roots.GENERATIONS, value)),
    "/api/replay": lambda value, query: replay_metadata(expected_path(roots.GENERATIONS, value)),
    "/api/source": lambda value, query: read_source(value),
    "/api/source-diff": lambda value, query: git_diff(value),
    "/api/baseline": lambda value, query: {
        "baseline": json_file(
            model_folder(relative_path(roots.GENERATIONS, value)) / "dashboard-model.json"
        ).get("baseline")
    },
    "/api/pick-root": lambda value, query: pick_root(query.get("kind", [""])[0]),
    "/api/generations": lambda value, query: [
        generation_summary(generation.dir)
        for generation in GenerationCatalog(roots.GENERATIONS).generations()
    ],
    "/api/source-drift": drift,
    "/api/runs": generation_runs,
    "/api/run": target_status,
    "/api/health": environment_health,
    "/api/generate": lambda value, query: generate_status(query.get("job", [""])[0]),
    "/api/console": console_text,
    "/api/plot": plot_window,
    "/api/jupyter": lambda value, query: jupyter_server(PORT),
}


def restored(body: dict) -> dict:
    """Restore the trashed entry `uri`; the cached sizes are stale after."""
    result = restore(body["uri"])
    directory_size.cache_clear()
    storage_totals.cache_clear()
    return result


def started_run(body: dict) -> dict:
    """Start a run of the generation at `path` with its `options`."""
    target = relative_path(roots.GENERATIONS, body["path"])
    with LOCK:
        return start_run(target, body.get("options") or {})


def started_generate(body: dict) -> dict:
    """Start generating the .robmot source at `path`."""
    if Path(body["path"]).suffix != ".robmot":
        raise ValueError("only a .robmot generates")
    return start_generate(source_path(body["path"]))


def deleted(body: dict) -> dict:
    """Trash the selected `paths`, unless any is protected, and the folders that leaves empty."""
    with LOCK:
        report = preview(body["paths"])
        blocked = [item for item in report["items"] if item["blocked"]]
        if blocked:
            raise ValueError(
                "protected selection: "
                + "; ".join(f"{item['path']} ({item['blocked']})" for item in blocked)
            )
        targets = [relative_path(roots.GENERATIONS, item["path"]) for item in report["items"]]
        for target in targets:
            trash(target)
    folders = 0
    for target in targets:
        folder = target.parent
        while roots.GENERATIONS in folder.parents and folder.is_dir() and not any(folder.iterdir()):
            trash(folder)
            folders += 1
            folder = folder.parent
    directory_size.cache_clear()
    storage_totals.cache_clear()
    return {"deleted": len(targets), "folders": folders}


POST_JSON = {
    "/api/annotations": lambda body: save_annotations(
        relative_path(roots.GENERATIONS, body["path"]), body["changes"]
    ),
    "/api/baseline": lambda body: set_baseline(
        relative_path(roots.GENERATIONS, body["path"]), body["enabled"]
    ),
    "/api/protect": lambda body: set_protected(
        relative_path(roots.GENERATIONS, body["path"]), body["enabled"], body.get("confirm", "")
    ),
    "/api/delete-preview": lambda body: preview(body["paths"]),
    "/api/restore": restored,
    "/api/roots": lambda body: set_root(body["kind"], body["path"]),
    "/api/source": lambda body: save_source(body["path"], body["text"]),
    "/api/source-checkout": lambda body: git_checkout(body["path"]),
    "/api/open": lambda body: open_source(body["path"], body.get("editor")),
    "/api/terminal": lambda body: open_terminal(body["path"]),
    "/api/run": started_run,
    "/api/run/stop": lambda body: stop_run(relative_path(roots.GENERATIONS, body["path"])),
    "/api/generate": started_generate,
    "/api/live": lambda body: live_state(
        expected_path(roots.GENERATIONS, body["path"]), body.get("signals") or ()
    ),
    "/api/control": lambda body: run_control(
        relative_path(roots.GENERATIONS, body["path"]), body.get("options") or {}
    ),
    "/api/queries": lambda body: save_queries(
        expected_path(roots.GENERATIONS, body["path"]), body["queries"]
    ),
    "/api/notes": lambda body: save_notes(
        expected_path(roots.GENERATIONS, body["path"]), body.get("notes", [])
    ),
    "/api/sparql": lambda body: run_query(
        relative_path(roots.GENERATIONS, body["path"]),
        body["query"],
        offset=int(body.get("offset") or 0),
    ),
    "/api/notebook": lambda body: run_notebook(
        relative_path(roots.GENERATIONS, body["path"]), PORT
    ),
    "/api/delete": deleted,
}


def byte_range(header: str, size: int) -> tuple[int, int, bool]:
    """The first and last of SIZE bytes a Range HEADER asks for, and whether it is one."""
    start, end = 0, size - 1
    asked = re.fullmatch(r"bytes=(\d*)-(\d*)", header)
    if asked and (asked.group(1) or asked.group(2)):
        if asked.group(1):
            start = min(int(asked.group(1)), size - 1)
            end = int(asked.group(2)) if asked.group(2) else end
        else:
            start = max(0, size - int(asked.group(2)))
        end = min(end, size - 1)
    return start, end, asked is not None


class LifecycleListener:
    """Bridge REC lifecycle datagrams into one dashboard revision stream."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._revision = 0
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        socket_path().unlink(missing_ok=True)
        self._socket.bind(str(socket_path()))
        threading.Thread(target=self.listen, daemon=True).start()

    def wait(self, revision: int, timeout: float) -> int:
        """Wait for a new REC transition or a stream keepalive timeout."""
        with self._condition:
            self._condition.wait_for(lambda: self._revision != revision, timeout=timeout)
            return self._revision

    def listen(self) -> None:
        while True:
            self._socket.recv(4096)
            with self._condition:
                self._revision += 1
                self._condition.notify_all()


class DashboardHandler(SimpleHTTPRequestHandler):
    """Expose only data endpoints; all dashboard layout lives in static browser assets."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=FRONTEND, **kwargs)

    def end_headers(self) -> None:
        """Always serve the local dashboard shell and scripts fresh."""
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def send_json(self, data: object, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(data, separators=(",", ":")).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if not parsed.path.startswith("/api/"):
            return super().do_GET()
        # GETs have side effects: refuse cross-site browser requests
        site = self.headers.get("Sec-Fetch-Site")
        if not (site is None or site in ("same-origin", "none")):
            return self.send_json({"error": "cross-site request"}, HTTPStatus.FORBIDDEN)
        if parsed.path == "/api/events":
            return self.stream_events()
        query = parse_qs(parsed.query)
        value = query.get("path", [""])[0]
        try:
            if parsed.path in self.GET_STREAMS:
                return self.GET_STREAMS[parsed.path](self, value, query)
            if parsed.path not in GET_JSON:
                return self.send_json({"error": "unknown endpoint"}, HTTPStatus.NOT_FOUND)
            self.send_json(GET_JSON[parsed.path](value, query))
        except ValueError as exc:
            self.send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        # an uncaught error drops the connection unanswered
        except Exception as exc:  # noqa: BLE001
            self.send_json(self.failed(exc), HTTPStatus.INTERNAL_SERVER_ERROR)

    def stream_events(self) -> None:
        """Send a lifecycle event on every REC transition, and a keepalive between them."""
        assert LIFECYCLE is not None
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        revision = LIFECYCLE.wait(-1, 0)
        try:
            while True:
                changed = LIFECYCLE.wait(revision, 30)
                if changed != revision:
                    revision = changed
                    self.wfile.write(b"event: lifecycle\ndata: changed\n\n")
                else:
                    self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return

    def send_video(self, value: str, query: dict[str, list[str]]) -> None:
        """Send run VALUE's `camera` video, or the byte range the request asks of it."""
        video = video_file(relative_path(roots.GENERATIONS, value), query.get("camera", [""])[0])
        size = video.stat().st_size
        start, end, partial = byte_range(self.headers.get("Range", ""), size)
        self.send_response(HTTPStatus.PARTIAL_CONTENT if partial else HTTPStatus.OK)
        self.send_header("Content-Type", "video/mp4")
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Length", str(end - start + 1))
        if partial:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.end_headers()
        with video.open("rb") as fh:
            fh.seek(start)
            remaining = end - start + 1
            while remaining > 0 and (chunk := fh.read(min(1 << 16, remaining))):
                self.wfile.write(chunk)
                remaining -= len(chunk)

    def send_ros_camera(self, value: str, query: dict[str, list[str]]) -> None:
        """Stream the ROS camera `topic` as MJPEG while its source is alive."""
        source = ros_camera.source_for(query.get("topic", [""])[0])
        if not source.alive():
            return self.send_json(
                {"error": source.error or "no ROS env"}, HTTPStatus.SERVICE_UNAVAILABLE
            )
        self.send_response(HTTPStatus.OK)
        self.send_header(
            "Content-Type", f"multipart/x-mixed-replace; boundary={ros_camera.BOUNDARY}"
        )
        self.end_headers()
        try:
            for part in ros_camera.mjpeg_stream(source, source.alive):
                self.wfile.write(part)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return

    GET_STREAMS: ClassVar = {"/api/video": send_video, "/api/ros-camera": send_ros_camera}

    def do_POST(self) -> None:
        """Update dashboard roots, open a source, or delete selected runs and generations."""
        # against the request's own Host, so both 127.0.0.1 and localhost pass
        origin = self.headers.get("Origin")
        if self.headers.get_content_type() != "application/json" or (
            origin is not None
            and urlparse(origin).hostname != urlparse(f"//{self.headers.get('Host', '')}").hostname
        ):
            return self.send_json({"error": "cross-origin request"}, HTTPStatus.FORBIDDEN)
        try:
            length = int(self.headers["Content-Length"])
            body = json.loads(self.rfile.read(length))
            if self.path not in POST_JSON:
                return self.send_json({"error": "unknown endpoint"}, HTTPStatus.NOT_FOUND)
            self.send_json(POST_JSON[self.path](body))
        except ValueError as exc:
            self.send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except Exception as exc:  # noqa: BLE001 -- any failure answers as an API error
            self.send_json(self.failed(exc), HTTPStatus.INTERNAL_SERVER_ERROR)

    @staticmethod
    def failed(exc: Exception) -> dict:
        """A fault, not a bad request: the page is told, and the terminal gets the traceback."""
        traceback.print_exc()
        return {"error": f"dashboard error: {type(exc).__name__}: {exc}"}


def serve(port: int = 8080, logs: Path | None = None, sources: Path | None = None) -> None:
    """Serve the dashboard for one pair of roots until interrupted."""
    global LIFECYCLE, PORT
    PORT = port
    if logs is not None:
        roots.GENERATIONS = Path(logs).expanduser().resolve()
        roots.WORKSPACE = roots.GENERATIONS.parent
    if sources is not None:
        roots.WORKSPACE = Path(sources).expanduser().resolve()
    LIFECYCLE = LifecycleListener()
    own = roots.pidfile(port)
    own.write_text(f"{os.getpid()}\n")
    atexit.register(own.unlink, missing_ok=True)
    atexit.register(stop_jupyter)
    for name in (signal.SIGTERM, signal.SIGINT):
        signal.signal(name, lambda *_: sys.exit(0))
    server = ThreadingHTTPServer(("127.0.0.1", port), DashboardHandler)
    print(f"motion-spec dashboard: http://127.0.0.1:{port}")
    print(f"  runs from {roots.GENERATIONS}\n  sources from {roots.WORKSPACE}")
    server.serve_forever()


def serve_detached(
    port: int, logs: Path, sources: Path | None, script: Path | None, log_file: Path | None
) -> tuple[int, Path]:
    """Start `motion-spec dashboard` on PORT in a session of its own, under SCRIPT's environment
    or none, appending its output to LOG_FILE, else dashboard.log in LOGS.

    Returns:
        The dashboard's pid and the file it logs to.
    """
    destination = (log_file or logs / "dashboard.log").expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    argv = [sys.argv[0], "dashboard", "--port", str(port), "--logs", str(logs)]
    if sources is not None:
        argv += ["--sources", str(sources)]
    argv += ["--env", str(script)] if script else ["--no-env"]
    with destination.open("ab") as sink:
        child = subprocess.Popen(argv, stdout=sink, stderr=sink, start_new_session=True)
    return child.pid, destination


def check_port(port: int, wait: float = 0) -> None:
    """Return once nothing answers on the loopback PORT, waiting up to WAIT seconds for it.

    Raises:
        RuntimeError: something still answers there.
    """
    deadline = time.monotonic() + wait
    while True:
        with socket.socket() as probe:
            probe.settimeout(0.2)
            if probe.connect_ex(("127.0.0.1", port)) != 0:
                return
        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"port {port} is already serving. Stop it, or pass --port for a second dashboard."
            )
        time.sleep(0.1)


def stop_dashboards(port: int | None) -> list[int]:
    """Stop the dashboards serving, on PORT or all, and the labs they started: the pids stopped.

    Each records its pid on start, and its lab's; a lab whose dashboard had to be killed
    outright is stopped from that record too, or the next dashboard starts a second one.

    Raises:
        RuntimeError: none was serving.
    """
    records = [
        path
        for pattern in (roots.pidfile(port or "*"), roots.pidfile(port or "*", "lab"))
        for path in pattern.parent.glob(pattern.name)
    ]
    stopped = []
    for path in records:
        try:
            pid = int(path.read_text().strip())
        except (OSError, ValueError):
            pass
        else:
            if stop(pid):
                stopped.append(pid)
        path.unlink(missing_ok=True)
    if not stopped:
        where = f" on port {port}" if port else ""
        raise RuntimeError(f"no dashboard is serving{where}.")
    return stopped


def stop(pid: int) -> bool:
    """SIGTERM, then SIGKILL after 5 s; False when PID was already gone."""
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        return False
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return True
        time.sleep(0.2)
    with contextlib.suppress(ProcessLookupError):
        os.kill(pid, signal.SIGKILL)
    return True
