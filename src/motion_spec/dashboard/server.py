# SPDX-License-Identifier: MPL-2.0

"""Serve recorded motion-spec generations and runs to the dashboard frontend."""

from __future__ import annotations

import argparse
import atexit
import json
import re
import signal
import socket
import sys
import threading
import traceback
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from motion_spec.dashboard import roots, ros_camera
from motion_spec.dashboard.analysis import run_reports
from motion_spec.dashboard.catalog import (
    drift_summary,
    generation_details,
    generation_info,
    is_simulated,
    provenance_graph,
    read_generated,
    run_info,
    source_drift,
    video_file,
)
from motion_spec.dashboard.jobs import (
    console_log_for,
    console_slice,
    generate_console,
    generate_status,
    health_report,
    run_status,
    start_generate,
    start_run,
    stop_run,
)
from motion_spec.dashboard.live import live_state, run_control
from motion_spec.dashboard.notebook import jupyter_server, run_notebook, stop_jupyter
from motion_spec.dashboard.queries import (
    activity_constraints,
    compare,
    gates,
    generation_graph,
    graph_sources,
    model_lint,
    run_query,
    save_queries,
    saved_queries,
    timeline,
)
from motion_spec.dashboard.replay import plot_data, replay_data
from motion_spec.dashboard.roots import (
    FRONTEND,
    GENERATION_DIR_ENV,
    LAYOUT_REL,
    current_roots,
    directory_size,
    expected_path,
    pick_root,
    relative_path,
    set_root,
    storage_info,
    trash,
)
from motion_spec.dashboard.runs import GenerationCatalog
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
from motion_spec.introspection import frame_log_pb
from motion_spec.introspection.lifecycle_events import socket_path

LIFECYCLE = None

# Off the loopback interface, the dashboard is someone else's browser on the network: it may
# watch and replay, and drive a simulation, but never delete, touch sources, or reach a real
# robot. Local access (127.0.0.1) is unrestricted regardless of LAN_MODE.
LAN_MODE = False

# Naming the restriction and the way past it: the reader is most often the person who started
# the server, on the machine that started it, having reached it by its network address.
LAN_REFUSED = (
    "This dashboard is shared on the network (started with --lan), and deleting, editing "
    "sources and driving real hardware stay on the machine it runs on. "
    "Open it at http://127.0.0.1:{port}/ there to do this."
)


def lan_refusal(port: int) -> str:
    return LAN_REFUSED.format(port=port)


LAN_GET_ALLOWED = frozenset(
    {
        "/api/events",
        "/api/generations",
        "/api/generation",
        "/api/generation-graph",
        "/api/graph-sources",
        "/api/generated",
        "/api/model/lint",
        "/api/source-drift",
        "/api/storage",
        "/api/runs",
        "/api/run",
        "/api/run/timeline",
        "/api/run/gates",
        "/api/run/constraints",
        "/api/run/compare",
        "/api/console",
        "/api/queries",
        "/api/video",
        "/api/ros-camera",
        "/api/replay",
        "/api/plot",
        "/api/reports",
        "/api/roots",
        "/api/sources",
        "/api/source",
        "/api/source-diff",
        "/api/generate",
    }
)

LAN_POST_ALLOWED = frozenset(
    {
        "/api/run",
        "/api/run/stop",
        "/api/live",
        "/api/control",
        "/api/queries",
        "/api/sparql",
        "/api/generate",
    }
)


class LifecycleListener:
    """Bridge REC lifecycle datagrams into one dashboard revision stream."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._revision = 0
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        socket_path().unlink(missing_ok=True)
        self._socket.bind(str(socket_path()))
        threading.Thread(target=self._listen, daemon=True).start()

    def wait(self, revision: int, timeout: float) -> int:
        """Wait for a new REC transition or a stream keepalive timeout."""
        with self._condition:
            self._condition.wait_for(lambda: self._revision != revision, timeout=timeout)
            return self._revision

    def _listen(self) -> None:
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

    def send_video(self, path: Path) -> None:
        """Serve one recording, honouring Range so the player can seek."""
        size = path.stat().st_size
        start, end = 0, size - 1
        asked = re.fullmatch(r"bytes=(\d*)-(\d*)", self.headers.get("Range", "") or "")
        if asked and (asked.group(1) or asked.group(2)):
            if asked.group(1):
                start = min(int(asked.group(1)), size - 1)
                end = int(asked.group(2)) if asked.group(2) else end
            else:
                start = max(0, size - int(asked.group(2)))
            end = min(end, size - 1)
        partial = asked is not None
        self.send_response(HTTPStatus.PARTIAL_CONTENT if partial else HTTPStatus.OK)
        self.send_header("Content-Type", "video/mp4")
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Length", str(end - start + 1))
        if partial:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.end_headers()
        with path.open("rb") as fh:
            fh.seek(start)
            remaining = end - start + 1
            while remaining > 0 and (chunk := fh.read(min(1 << 16, remaining))):
                self.wfile.write(chunk)
                remaining -= len(chunk)

    def stream_ros_camera(self, topic: str) -> None:
        """Stream a ROS image topic as MJPEG: the live view a real platform records nothing of."""
        source = ros_camera.source_for(topic)
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

    def send_json(self, data: object, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(data, separators=(",", ":")).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def stream_events(self) -> None:
        """Keep one browser stream open for REC lifecycle changes."""
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

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if not parsed.path.startswith("/api/"):
            return super().do_GET()
        if not self._from_this_page():
            return self.send_json({"error": "cross-site request"}, HTTPStatus.FORBIDDEN)
        if LAN_MODE and not self._is_local_client() and parsed.path not in LAN_GET_ALLOWED:
            return self.send_json({"error": self._lan_refusal()}, HTTPStatus.FORBIDDEN)
        if parsed.path == "/api/events":
            return self.stream_events()
        try:
            query = parse_qs(parsed.query)
            value = query.get("path", [""])[0]
            if parsed.path == "/api/roots":
                payload = current_roots()
                if LAN_MODE and not self._is_local_client():
                    payload = {**payload, "restricted": True}
                return self.send_json(payload)
            if parsed.path == "/api/pick-root":
                return self.send_json(pick_root(query.get("kind", [""])[0]))
            if parsed.path == "/api/generations":
                return self.send_json(
                    [
                        generation_info(generation.dir)
                        for generation in GenerationCatalog([roots.GENERATIONS]).generations()
                    ]
                )
            if parsed.path == "/api/storage":
                return self.send_json(storage_info())
            if parsed.path == "/api/generation":
                return self.send_json(generation_details(relative_path(roots.GENERATIONS, value)))
            if parsed.path == "/api/source-drift":
                generation = relative_path(roots.GENERATIONS, value)
                file = query.get("file", [""])[0]
                if query.get("summary"):
                    return self.send_json(drift_summary(generation))
                return self.send_json(source_drift(generation, file or None))
            if parsed.path == "/api/generation-graph":
                # The whole model graph, imports followed -- not the picked files, which cut
                # the graph at file boundaries the model does not have.
                return self.send_json(
                    provenance_graph(generation_graph(relative_path(roots.GENERATIONS, value)))
                )
            if parsed.path == "/api/graph-sources":
                # The explore panel binds as the run page opens, which is before a just-named
                # run has a directory: no sources yet is an answer, not a bad request.
                run = expected_path(roots.GENERATIONS, value)
                return self.send_json(graph_sources(run) if run.exists() else [])
            if parsed.path == "/api/generated":
                return self.send_json(read_generated(relative_path(roots.GENERATIONS, value)))
            if parsed.path == "/api/model/lint":
                return self.send_json(model_lint(relative_path(roots.GENERATIONS, value)))
            if parsed.path == "/api/runs":
                generation = relative_path(roots.GENERATIONS, value)
                runs = sorted(path for path in generation.glob("runs/*") if path.is_dir())
                return self.send_json([run_info(path) for path in reversed(runs)])
            if parsed.path == "/api/run":
                return self.send_json(run_status(relative_path(roots.GENERATIONS, value)))
            if parsed.path == "/api/run/timeline":
                return self.send_json(
                    timeline(
                        relative_path(roots.GENERATIONS, value), query.get("iri", [""])[0] or None
                    )
                )
            if parsed.path == "/api/run/gates":
                return self.send_json(gates(relative_path(roots.GENERATIONS, value)))
            if parsed.path == "/api/run/constraints":
                return self.send_json(
                    activity_constraints(
                        relative_path(roots.GENERATIONS, value), query.get("occ", [""])[0]
                    )
                )
            if parsed.path == "/api/run/compare":
                return self.send_json(
                    compare(
                        relative_path(roots.GENERATIONS, query.get("left", [""])[0]),
                        relative_path(roots.GENERATIONS, query.get("right", [""])[0]),
                    )
                )
            if parsed.path == "/api/devices":
                return self.send_json(probe_devices(relative_path(roots.GENERATIONS, value)))
            if parsed.path == "/api/health":
                return self.send_json(health_report(bool(query.get("refresh", [""])[0])))
            if parsed.path == "/api/generate":
                return self.send_json(generate_status(query.get("job", [""])[0]))
            if parsed.path == "/api/console":
                offset = int(query.get("offset", ["0"])[0])
                job = query.get("job", [""])[0]
                if job:
                    return self.send_json(generate_console(job, offset))
                # expected_path, not relative_path: a run directory is named before it exists
                target = expected_path(roots.GENERATIONS, value)
                return self.send_json(console_slice(console_log_for(target), offset))
            if parsed.path == "/api/queries":
                return self.send_json(saved_queries(expected_path(roots.GENERATIONS, value)))
            if parsed.path == "/api/video":
                run = relative_path(roots.GENERATIONS, value)
                return self.send_video(video_file(run, query.get("camera", [""])[0]))
            if parsed.path == "/api/ros-camera":
                return self.stream_ros_camera(query.get("topic", [""])[0])
            if parsed.path == "/api/replay":
                return self.send_json(replay_data(expected_path(roots.GENERATIONS, value)))
            if parsed.path == "/api/reports":
                # All three reports off one sweep: three passes over a 160 MB log is the cost
                # that would matter, so the route asks for them together or not at all.
                return self.send_json(run_reports(relative_path(roots.GENERATIONS, value)))
            if parsed.path == "/api/plot":
                bounds = query.get("window", [])
                return self.send_json(
                    plot_data(
                        relative_path(roots.GENERATIONS, value),
                        query.get("signal", []),
                        (int(bounds[0]), int(bounds[1])) if len(bounds) == 2 else None,
                    )
                )
            if parsed.path == "/api/sources":
                return self.send_json(sorted(authored_sources()))
            if parsed.path == "/api/jupyter":
                return self.send_json(jupyter_server())
            if parsed.path == "/api/source":
                return self.send_json(read_source(value))
            if parsed.path == "/api/source-diff":
                return self.send_json(git_diff(value))
            self.send_json({"error": "unknown endpoint"}, HTTPStatus.NOT_FOUND)
        except ValueError as exc:
            self.send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        # Any failure answers as an API error: a handler that dies silently closes the
        # connection instead, and the client sees nothing to report.
        except Exception as exc:  # noqa: BLE001
            self.send_json(self._failed(exc), HTTPStatus.INTERNAL_SERVER_ERROR)

    def do_POST(self) -> None:
        """Update dashboard roots, open a source, or delete selected runs and generations."""
        if not self._same_origin():
            return self.send_json({"error": "cross-origin request"}, HTTPStatus.FORBIDDEN)
        if LAN_MODE and not self._is_local_client() and self.path not in LAN_POST_ALLOWED:
            return self.send_json({"error": self._lan_refusal()}, HTTPStatus.FORBIDDEN)
        try:
            length = int(self.headers["Content-Length"])
            body = json.loads(self.rfile.read(length))
            if self.path == "/api/roots":
                return self.send_json(set_root(body["kind"], body["path"]))
            if self.path == "/api/source":
                return self.send_json(save_source(body["path"], body["text"]))
            if self.path == "/api/source-checkout":
                return self.send_json(git_checkout(body["path"]))
            if self.path == "/api/open":
                return self.send_json(open_source(body["path"], body.get("editor")))
            if self.path == "/api/terminal":
                return self.send_json(open_terminal(body["path"]))
            if self.path == "/api/run":
                target = relative_path(roots.GENERATIONS, body["path"])
                remote = LAN_MODE and not self._is_local_client()
                if remote and not is_simulated(target):
                    raise ValueError("only simulated generations can be started from the network")
                options = body.get("options") or {}
                if remote:
                    # The GUI would open on this machine's display, not the LAN viewer's --
                    # nothing there to watch it, and nothing here to stop it. Headless only.
                    options = {**options, "headless": True}
                return self.send_json(start_run(target, options))
            if self.path == "/api/run/stop":
                return self.send_json(stop_run(relative_path(roots.GENERATIONS, body["path"])))
            if self.path == "/api/generate":
                # source_path takes any authored file; only a .robmot is a whole run to make
                if not body["path"].endswith(".robmot"):
                    raise ValueError("only a .robmot generates")
                return self.send_json(start_generate(source_path(body["path"])))
            if self.path == "/api/live":
                # A run is polled from the moment it is named, before the runtime has written
                # its directory: live_state answers that with `started: False`.
                return self.send_json(
                    live_state(
                        expected_path(roots.GENERATIONS, body["path"]), body.get("signals") or ()
                    )
                )
            if self.path == "/api/control":
                return self.send_json(
                    run_control(
                        relative_path(roots.GENERATIONS, body["path"]), body.get("options") or {}
                    )
                )
            if self.path == "/api/queries":
                return self.send_json(
                    save_queries(expected_path(roots.GENERATIONS, body["path"]), body["queries"])
                )
            if self.path == "/api/sparql":
                return self.send_json(
                    run_query(
                        relative_path(roots.GENERATIONS, body["path"]),
                        body["query"],
                        offset=int(body.get("offset") or 0),
                    )
                )
            if self.path == "/api/notebook":
                return self.send_json(run_notebook(relative_path(roots.GENERATIONS, body["path"])))
            if self.path != "/api/delete":
                return self.send_json({"error": "unknown endpoint"}, HTTPStatus.NOT_FOUND)
            selected = body["paths"]
            targets = [relative_path(roots.GENERATIONS, value) for value in selected]
            if not targets or any(not self._deletable(path) for path in targets):
                raise ValueError("only generation bundles and run archives can be deleted")
            targets = [
                target
                for target in targets
                if not any(target in other.parents for other in targets)
            ]
            for target in sorted(targets, key=lambda item: len(item.parts), reverse=True):
                trash(target)
            # a model folder emptied of its generations is no longer a model folder
            folders = 0
            for folder in {target.parent for target in targets}:
                if (
                    roots.GENERATIONS in folder.parents
                    and folder.is_dir()
                    and not any(folder.iterdir())
                ):
                    trash(folder)
                    folders += 1
            directory_size.cache_clear()
            storage_info.cache_clear()
            self.send_json({"deleted": len(targets), "folders": folders})
        except ValueError as exc:
            self.send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except Exception as exc:  # noqa: BLE001 -- any failure answers as an API error
            self.send_json(self._failed(exc), HTTPStatus.INTERNAL_SERVER_ERROR)

    def _same_origin(self) -> bool:
        """Only the dashboard's own page may POST: these endpoints delete and spawn.

        Compared against the request's own Host, not a hardcoded loopback name, so this holds
        whether the page was loaded from 127.0.0.1 or from this machine's LAN address.
        """
        if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
            return False
        origin = self.headers.get("Origin")
        if origin is None:
            return True
        return urlparse(origin).hostname == self.headers.get("Host", "").split(":")[0]

    def _is_local_client(self) -> bool:
        """Whether this request came from this machine, not the network LAN_MODE opened up.

        By address, not by host: reaching this machine at its own LAN address is a network
        request, and the browser doing it is often the one sitting in front of the server.
        """
        return self.client_address[0] in ("127.0.0.1", "::1")

    def _lan_refusal(self) -> str:
        """Why this was refused and where it can be done instead, at this server's own port."""
        return lan_refusal(int(self.server.server_address[1]))

    @staticmethod
    def _failed(exc: Exception) -> dict:
        """A fault, not a bad request: the page is told, and the terminal gets the traceback.

        Anything that is not a ValueError here is the dashboard's own mistake. Answering those
        with 400 and one line of text hides them as the reader's fault and loses the stack.
        """
        traceback.print_exc()
        return {"error": f"dashboard error: {type(exc).__name__}: {exc}"}

    def _from_this_page(self) -> bool:
        """Whether a browser says this request came from the dashboard itself.

        Reading is not harmless here: a GET opens a folder chooser, connects to the robot,
        starts a notebook server. Any page in any tab can ask for one of those as an image or a
        no-cors fetch, and never see the answer -- the side effect is the attack. Browsers
        stamp where a request came from, so that stamp is the gate. Nothing stamps it when the
        client is not a browser (curl, the CLI, a test), and those are left alone.
        """
        site = self.headers.get("Sec-Fetch-Site")
        return site is None or site in ("same-origin", "none")

    @staticmethod
    def _deletable(path: Path) -> bool:
        return (path / LAYOUT_REL).exists() or (
            path.parent.name == "runs"
            and frame_log_pb.log_path(path / "logs/frame_log.pb").exists()
        )


def _lan_ip() -> str | None:
    """This machine's LAN address, found the way you'd find your own outbound route."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("8.8.8.8", 80))  # UDP connect: routes locally, sends nothing
            return probe.getsockname()[0]
    except OSError:
        return None


def serve(
    port: int = 8080, logs: Path | None = None, sources: Path | None = None, lan: bool = False
) -> None:
    """Serve the dashboard for one pair of roots until interrupted."""
    global LIFECYCLE, LAN_MODE
    LAN_MODE = lan
    if logs is not None:
        roots.GENERATIONS = Path(logs).expanduser().resolve()
        roots.WORKSPACE = roots.GENERATIONS.parent
    if sources is not None:
        roots.WORKSPACE = Path(sources).expanduser().resolve()
    LIFECYCLE = LifecycleListener()
    atexit.register(stop_jupyter)
    for name in (signal.SIGTERM, signal.SIGINT):
        signal.signal(name, lambda *_: sys.exit(0))
    host = "0.0.0.0" if lan else "127.0.0.1"
    server = ThreadingHTTPServer((host, port), DashboardHandler)
    print(f"motion-spec dashboard: http://127.0.0.1:{port}")
    if lan:
        lan_host = _lan_ip() or "<this machine's LAN IP>"
        print(
            f"  also on the network at http://{lan_host}:{port}"
            " (replay + simulated runs only, no delete)"
        )
    print(f"  runs from {roots.GENERATIONS}\n  sources from {roots.WORKSPACE}")
    server.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--logs", type=Path, help=f"generation root (default ${GENERATION_DIR_ENV})"
    )
    parser.add_argument(
        "--sources", type=Path, help="model sources root (default: the logs root's parent)"
    )
    parser.add_argument(
        "--lan",
        action="store_true",
        help="reachable from the network: replay and simulated runs only, no delete or source access",
    )
    args = parser.parse_args()
    serve(args.port, args.logs, args.sources, args.lan)


if __name__ == "__main__":
    main()
