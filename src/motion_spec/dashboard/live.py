# SPDX-License-Identifier: MPL-2.0

"""A run as it happens: the block the runtime publishes, and the loop it drives."""

from __future__ import annotations

import os
import threading
import time
from collections import deque
from pathlib import Path

from motion_spec.dashboard.catalog import run_ended
from motion_spec.dashboard.frames import (
    CORE_SIGNALS,
    ShmFrameReader,
    SignalFields,
    read,
)
from motion_spec.dashboard.jobs import RUNNING, mark_stopped
from motion_spec.dashboard.replay import header_record, log_events
from motion_spec.dashboard.roots import trace
from motion_spec.dashboard.runs import GenerationInfo
from motion_spec.dashboard.tail import FrameLogTail
from motion_spec.runs.control import SPEED_MAX, SPEED_MIN, acknowledged, control_run, run_channel
from motion_spec.runs.provenance import rec_run, run_generation
from motion_spec.runs.replay import resolve_run
from motion_spec.runs.verify import RecordError
from motion_spec.telemetry import frame_log_pb
from motion_spec.telemetry.frame_log_pb import shm_name_for, shm_path

LIVE_SESSIONS: dict[str, dict] = {}

LIVE_IDLE_S = 3.0

LIVE_PLOT_POINTS = 300

LIVE_SAMPLE_HZ = 200

LIVE_RING = 4000

LIVE_SESSION_S = 5.0


class ShmSampler(threading.Thread):
    """Copy the runtime's latest-frame block into a ring the live polls serve from.

    The block holds one frame, so a state shorter than a sample period can be missed; the frame
    log keeps every tick.
    """

    def __init__(self, reader: ShmFrameReader, fields: SignalFields, states: list, fired: list):
        super().__init__(daemon=True)
        self.reader, self.fields = reader, fields
        self.states, self.fired = states, fired
        self.lock = threading.Lock()
        self.ring = deque(maxlen=LIVE_RING)
        self.taken = 0  # poll cursors count in these
        self.events: list = []
        self.frame, self.t, self.motion = 0, 0.0, None
        self.reading = False
        self.advanced = 0.0
        self.touched = time.monotonic()
        self.stopped = threading.Event()
        self.seen = None
        self._state = self._event = None

    def run(self) -> None:
        period = 1.0 / LIVE_SAMPLE_HZ
        checks = 0
        while not self.stopped.wait(period):
            if time.monotonic() - self.touched > LIVE_SESSION_S:
                break
            raw = self.reader.latest_raw()
            if raw is None:
                # stat the block about once a second
                checks += 1
                gone = self.reading and checks % LIVE_SAMPLE_HZ == 0
                if gone and not shm_path(self.reader.name).exists():
                    break
                continue
            self.absorb(raw)
        self.reader.close()

    def absorb(self, raw: bytes) -> None:
        """One sample, dropped unless the run has stepped since the last one."""
        self.reading = True
        core = {key: read(self.fields.offsets[key], raw) for key in CORE_SIGNALS}
        step = core["step"]
        if step == self.seen:
            return
        # an armed run's standing frame is not an advance
        first = self.seen is None
        self.seen = step
        if first:
            return
        with self.lock:
            self.ring.append((self.taken, step, core["t"], core["active_motion"], raw))
            self.taken += 1
            self.frame, self.t, self.motion = step, core["t"], core["active_motion"]
            self.advanced = time.monotonic()
            self.events += transition_rows(
                step, core, self._event, self._state, self.fired, self.states
            )
            self._event, self._state = core["last_event"], core["fsm_state"]

    def increment(self, cursor: int | None, names: tuple) -> tuple[int, dict]:
        """The plot of NAMES since a poll's CURSOR, thinned to LIVE_PLOT_POINTS, and its new cursor.

        A first poll's cursor is None: it starts from now.
        """
        if cursor is None:
            cursor = self.taken
        with self.lock:
            ring, taken = list(self.ring), self.taken
        new = ring[max(0, cursor - (taken - len(ring))) :]
        stride = max(1, -(-len(new) // LIVE_PLOT_POINTS))
        sampled = new[::stride]
        rows = [
            self.fields.extract(raw, motion, names) for _index, _step, _t, motion, raw in sampled
        ]
        plot = {
            "frames": [step for _index, step, _t, _motion, _raw in sampled],
            "series": {name: [row[at] for row in rows] for at, name in enumerate(names)},
        }
        return taken, plot


def transition_rows(frame: int, core, event_was, state_was, fired: list, states: list) -> list:
    """The event and state rows FRAME adds where its `last_event` or `fsm_state` changed."""
    rows = []
    event, state = core["last_event"], core["fsm_state"]
    if event != event_was and 0 <= event < len(fired):
        rows.append({"frame": frame, "kind": "event", "label": fired[event]})
    if state != state_was:
        label = states[state] if 0 <= state < len(states) else str(state)
        rows.append({"frame": frame, "kind": "state", "label": label})
    return rows


NOT_STARTED_STATE = {
    "started": False,
    "writing": False,
    "archived": False,
    "frames": 0,
    "duration": 0.0,
    "events": [],
    "control": None,
    "active_motion": None,
}


def live_sampler(run_dir: Path, contract) -> ShmSampler | None:
    """This run's frame-block sampler, started, or None when the generation names no layout."""
    try:
        generation = run_generation(run_dir)
        if generation is None:
            raise ValueError("the run's document names no generation")
        layout = GenerationInfo(generation).layout
        fields = SignalFields(layout, contract)
    except (OSError, ValueError, KeyError) as error:
        trace(f"live sampler {run_dir.name}: no frame layout ({error})")
        return None
    name = os.environ.get("MOTION_SPEC_SHM_NAME") or shm_name_for(layout.schema_hash, run_dir.name)
    sampler = ShmSampler(
        ShmFrameReader(name, layout),
        fields,
        [state.id for state in contract.header.fsm_states],
        [event.id for event in contract.header.fsm_events],
    )
    sampler.start()
    return sampler


def close_sessions() -> None:
    """Stop following every run followed so far: its sampler, log tail and control channel."""
    for stale in list(LIVE_SESSIONS.values()):
        if stale.get("sampler") is not None:
            stale["sampler"].stopped.set()
        if stale.get("tail") is not None:
            stale["tail"].close()
        if stale.get("channel") is not None:
            stale["channel"].close()
    LIVE_SESSIONS.clear()


def open_session(run_dir: Path) -> dict | None:
    """Follow this run from now on, closing the one followed before; None while it is not started."""
    log_less = False
    try:
        _, log, contract = resolve_run(run_dir)
    except RecordError:
        # with logs off, the control block comes from the generation's contract
        started = RUNNING.get(str(run_dir))
        if not started or started.get("recorded") is not False:
            return None
        log = header_record(run_dir)
        if log is None:
            return None
        contract, log_less = frame_log_pb.read_contract(log), True
    close_sessions()
    session = LIVE_SESSIONS[str(run_dir)] = {
        "log": str(log),
        "log_less": log_less,
        "contract": contract,
        "sampler": live_sampler(run_dir, contract),
        "cursor": None,
        "tail": None,
        "events": [],
        "frame": 0,
        "size": -1,
        "state": None,
        "event": None,
        "motion": None,
    }
    return session


def control_status(
    session: dict, run_dir: Path, log: Path, live: ShmSampler | None
) -> tuple[dict, bool]:
    """The run's control block as the live page shows it, and whether the run is still writing."""
    moving = live is not None and time.monotonic() - live.advanced < LIVE_IDLE_S
    # a ping writes and waits: only an idle run gets one
    channel = session.get("channel")
    if channel is None:
        channel = session["channel"] = run_channel(run_dir)
    now = time.monotonic()
    cached = session.get("control_alive")
    if moving:
        alive = True
        session["control_alive"] = (True, now)
    elif cached is not None and now - cached[1] < 2.0:
        alive = cached[0]
    elif channel.open() is not None:
        channel.set_pause(bool(channel.paused))
        alive = acknowledged(channel)
        session["control_alive"] = (alive, now)
    else:
        alive = False
        session["control_alive"] = (False, now)
    control = {
        "available": channel.open() is not None,
        "alive": alive,
        "paused": channel.paused,
        "speed": channel.speed,
        "seq": channel.seq,
        "applied": channel.applied,
        "speed_range": [SPEED_MIN, SPEED_MAX],
    }
    stat = log.stat()
    grew = stat.st_size > session["size"] >= 0
    session["size"] = stat.st_size
    writing = alive or (moving if live else grew)
    if not control["available"] and not live and not session["log_less"]:
        writing = writing or time.time() - stat.st_mtime < LIVE_IDLE_S
    return control, writing


def live_state(run_dir: Path, signals=()) -> dict:
    """How far a run being written has got, and the states and events it has passed.

    `signals` are served from the shared-memory sampler's ring as the increment since this
    page's last poll; a build that publishes no block falls back to following the log. A run
    named but not yet writing answers `started: False`.
    """
    session = LIVE_SESSIONS.get(str(run_dir)) or open_session(run_dir)
    if session is None:
        return NOT_STARTED_STATE
    log, contract = Path(session["log"]), session["contract"]
    sampler = session["sampler"]
    if sampler is not None and not sampler.is_alive():
        sampler = session["sampler"] = live_sampler(run_dir, contract)
        session["cursor"] = None
    if sampler is not None:
        sampler.touched = time.monotonic()
    live = sampler if sampler is not None and sampler.reading else None
    period = (contract.header.nominal_period_ns or 1_000_000) / 1e9
    names = tuple(signals)
    plot = None
    if names and live:
        session["cursor"], plot = live.increment(session["cursor"], names)
    if live:
        frame, t, motion = live.frame, live.t, live.motion
    else:
        if session["tail"] is None:
            session["tail"] = FrameLogTail(log)
        states = [state.id for state in contract.header.fsm_states]
        fired = [event.id for event in contract.header.fsm_events]
        for logged in session["tail"].poll(max(1, round(0.05 / period))):
            index = logged["step"]
            session["frame"] = index
            session["t"] = logged["t"]
            session["motion"] = logged["active_motion"]
            session["events"] += transition_rows(
                index, logged, session["event"], session["state"], fired, states
            )
            session["event"], session["state"] = logged["last_event"], logged["fsm_state"]
        frame, t, motion = session["frame"], session.get("t"), session["motion"]
    # the log keeps every tick; the sampler misses edges between samples
    try:
        events = list(log_events(log, contract)["events"])
    except OSError:
        events = []
    if not events:
        events = list(live.events) if live else session["events"]
    motions = contract.header.motions
    control, writing = control_status(session, run_dir, log, live)
    return {
        "started": writing if session["log_less"] else True,
        "writing": writing,
        "archived": run_ended(run_dir),
        "frames": frame + 1,
        "duration": t or (frame + 1) * period,
        "events": events,
        "control": control,
        "active_motion": (
            motions[motion].id if motion is not None and 0 <= motion < len(motions) else None
        ),
        **({"plot": plot} if plot is not None else {}),
    }


def run_control(run_dir: Path, options: dict) -> dict:
    """What `motion-spec control` and `motion-spec cancel` do, for the live page.

    `alive` is the loop's own ack, not a guess about who started it; only a simulated run
    creates the block.
    """
    action = options.get("action") or "status"
    if action == "cancel":
        mark_stopped(run_dir)
        rec_run(run_dir).cancel()
        action = "status"
    value = options.get("ticks") if action == "step" else options.get("speed")
    return control_run(run_dir, action, None if value is None else float(value))
