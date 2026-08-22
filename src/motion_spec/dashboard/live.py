# SPDX-License-Identifier: MPL-2.0

"""A run as it happens: the block the runtime publishes, and the loop it drives."""

from __future__ import annotations

import os
import threading
import time
from collections import deque
from pathlib import Path

from motion_spec.dashboard.catalog import run_ended
from motion_spec.dashboard.control import SPEED_MAX, SPEED_MIN, ControlChannel
from motion_spec.dashboard.frames import FrameLayout, ShmFrameReader, SignalFields, shm_path
from motion_spec.dashboard.roots import LAYOUT_REL, json_file, trace
from motion_spec.dashboard.tail import FrameLogTail
from motion_spec.introspection.replay import resolve_archive

_LIVE: dict[str, dict] = {}


# A log nobody has appended to for this long is one nobody is writing any more.
LIVE_IDLE_S = 3.0


# The most points one poll returns per signal; a poll's worth of ticks is decimated onto this.
LIVE_PLOT_POINTS = 300


# How often the sampler copies the runtime's latest-frame block.
LIVE_SAMPLE_HZ = 200


# ~20 s of run at that rate: how far back a poll can still reach.
LIVE_RING = 4000


# A session nobody has polled for this long has no page behind it any more.
LIVE_SESSION_S = 5.0


class ShmSampler(threading.Thread):
    """Copy the runtime's latest-frame block into a ring the live polls serve from.

    The block holds one frame, so a state that lasts under ~5 ms can pass between two samples;
    the frame log keeps every tick and the archive stays what a reader goes back to. This feeds
    the live view only, where re-reading the log once per poll is what made the page lag.
    """

    def __init__(self, reader: ShmFrameReader, fields: SignalFields, states: list, fired: list):
        super().__init__(daemon=True)
        self.reader, self.fields = reader, fields
        self.states, self.fired = states, fired
        self.lock = threading.Lock()
        self.ring = deque(maxlen=LIVE_RING)
        self.taken = 0  # samples ever appended; a poll cursor counts in these
        self.events: list = []
        self.frame, self.t, self.motion = 0, 0.0, None
        self.reading = False  # the block has published at least one frame
        self.advanced = 0.0  # monotonic clock at the last step advance
        self.touched = time.monotonic()
        self.stopped = threading.Event()
        self.seen = None  # step of the last frame read, advance or not
        self._state = self._event = None

    def touch(self) -> None:
        self.touched = time.monotonic()

    def moving(self) -> bool:
        """Whether the run stepped recently -- what `writing` means with no log to watch."""
        return time.monotonic() - self.advanced < LIVE_IDLE_S

    def since(self, cursor: int) -> tuple:
        """The samples appended since `cursor`, and the cursor that follows them."""
        with self.lock:
            ring, taken = list(self.ring), self.taken
        return ring[max(0, cursor - (taken - len(ring))) :], taken

    def close(self) -> None:
        self.stopped.set()

    def run(self) -> None:
        period = 1.0 / LIVE_SAMPLE_HZ
        checks = 0
        while not self.stopped.wait(period):
            if time.monotonic() - self.touched > LIVE_SESSION_S:
                break
            raw = self.reader.latest_raw()
            if raw is None:
                # The block appears when the runtime starts and goes when the run unlinks it;
                # a stat every sample would be its own load, so ask about once a second.
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
        core = self.fields.core(raw)
        step = core["step"]
        if step == self.seen:
            return
        # The block's standing frame is not an advance: a run armed at the play button may have
        # published one already, and armed means no events yet.
        first, self.seen = self.seen is None, step
        if first:
            return
        with self.lock:
            self.ring.append((self.taken, step, core["t"], core["active_motion"], raw))
            self.taken += 1
            self.frame, self.t, self.motion = step, core["t"], core["active_motion"]
            self.advanced = time.monotonic()
            if core["last_event"] != self._event:
                self._event = core["last_event"]
                if 0 <= self._event < len(self.fired):
                    self.events.append(
                        {"frame": step, "kind": "event", "label": self.fired[self._event]}
                    )
            if core["fsm_state"] != self._state:
                self._state = core["fsm_state"]
                label = (
                    self.states[self._state]
                    if 0 <= self._state < len(self.states)
                    else str(self._state)
                )
                self.events.append({"frame": step, "kind": "state", "label": label})


def _live_sampler(run_dir: Path, contract) -> ShmSampler | None:
    """This run's frame-block sampler, started, or None when the generation names no layout."""
    try:
        layout = FrameLayout.for_generation(run_dir.parent.parent)
        fields = SignalFields(layout, contract)
    except (OSError, ValueError, KeyError) as error:
        trace(f"live sampler {run_dir.name}: no frame layout ({error})")
        return None
    sampler = ShmSampler(
        ShmFrameReader(os.environ.get("MOTION_SPEC_SHM_NAME") or layout.shm_name, layout, contract),
        fields,
        [state.id for state in contract.header.fsm_states],
        [event.id for event in contract.header.fsm_events],
    )
    sampler.start()
    return sampler


def _follow_log(session: dict, log: Path, contract, period: float) -> None:
    """Where the run is, read off the log itself -- for a build that publishes no frame block.

    The archive is protobuf, so this parses; it is the fallback, never the live path.
    """
    if session["tail"] is None:
        session["tail"] = FrameLogTail(log)
    states = [state.id for state in contract.header.fsm_states]
    fired = [event.id for event in contract.header.fsm_events]
    for frame in session["tail"].poll(max(1, round(0.05 / period))):
        index = frame["step"]
        session["frame"] = index
        session["t"] = frame["t"]
        session["motion"] = frame["active_motion"]
        if frame["last_event"] != session["event"]:
            session["event"] = frame["last_event"]
            if 0 <= session["event"] < len(fired):
                session["events"].append(
                    {"frame": index, "kind": "event", "label": fired[session["event"]]}
                )
        if frame["fsm_state"] != session["state"]:
            session["state"] = frame["fsm_state"]
            label = (
                states[session["state"]]
                if 0 <= session["state"] < len(states)
                else str(session["state"])
            )
            session["events"].append({"frame": index, "kind": "state", "label": label})


def _close_live(session: dict) -> None:
    """A session that ends drops its sampler and whatever log handle it fell back to."""
    if session.get("sampler") is not None:
        session["sampler"].close()
    if session.get("tail") is not None:
        session["tail"].close()
    if session.get("channel") is not None:
        session["channel"].close()


def live_state(run_dir: Path, signals=()) -> dict:
    """How far a run being written has got, and the states and events it has passed.

    Live is the runtime's shared-memory frame, sampled at LIVE_SAMPLE_HZ into a ring: `signals`
    is served out of that ring as the increment since this page's last poll, values and events
    and states alike. Only a build that publishes no block falls back to following the log.
    """
    session = _LIVE.get(str(run_dir))
    if session is None:
        # Resolving parses the full header contract -- once per session, never per poll.
        _, log, _manifest, contract = resolve_archive(run_dir)
        for stale in list(_LIVE.values()):
            _close_live(stale)
        _LIVE.clear()
        session = _LIVE[str(run_dir)] = {
            "log": str(log),
            "contract": contract,
            "sampler": _live_sampler(run_dir, contract),
            "cursor": None,
            "tail": None,
            "events": [],
            "frame": 0,
            "size": -1,
            "state": None,
            "event": None,
            "motion": None,
        }
    log, contract = Path(session["log"]), session["contract"]
    sampler = session["sampler"]
    if sampler is not None and not sampler.is_alive():
        # A page whose tab was backgrounded stops polling; the sampler gives up and this asks
        # again. What it missed meanwhile is the archive's, not the live view's.
        sampler = session["sampler"] = _live_sampler(run_dir, contract)
        session["cursor"] = None
    if sampler is not None:
        sampler.touch()
    live = sampler is not None and sampler.reading
    period = (contract.header.nominal_period_ns or 1_000_000) / 1e9
    if not live:
        _follow_log(session, log, contract, period)
    names = tuple(signals)
    plot = None
    if names and live:
        # Only a cursor being created skips to the ring's end: history is the page's /api/plot
        # backfill. The cursor counts samples, not names, so a signal set that changes mid-run
        # -- a motion entering opens new charts -- keeps every sample since the last poll.
        if session["cursor"] is None:
            session["cursor"] = sampler.taken
        new, session["cursor"] = sampler.since(session["cursor"])
        stride = max(1, -(-len(new) // LIVE_PLOT_POINTS))
        sampled = new[::stride]
        rows = [
            sampler.fields.extract(raw, motion, names) for _index, _step, _t, motion, raw in sampled
        ]
        plot = {
            "frames": [step for _index, step, _t, _motion, _raw in sampled],
            "series": {name: [row[at] for row in rows] for at, name in enumerate(names)},
        }
    frame = sampler.frame if live else session["frame"]
    t = sampler.t if live else session.get("t")
    motion = sampler.motion if live else session["motion"]
    events = list(sampler.events) if live else session["events"]
    motions = contract.header.motions
    # A loop that answers is live even while paused. Only a run with no control block to ask
    # -- real hardware, or a build older than it -- has to be judged by its log growing.
    control = _control_status(session, run_dir, sampler.moving() if live else False)
    stat = log.stat()
    grew = stat.st_size > session["size"] >= 0
    session["size"] = stat.st_size
    writing = control["alive"] or (sampler.moving() if live else grew)
    if not control["available"] and not live:
        writing = writing or time.time() - stat.st_mtime < LIVE_IDLE_S
    return {
        "writing": writing,
        # Finished means archived and marked: the run row can only say how it ended once REC
        # has recorded that.
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


def _control_status(session: dict, run_dir: Path, moving: bool) -> dict:
    """The control block read, not pinged: a status poll must never write and wait.

    A moving sampler is proof of life for free; only an idle run gets the publish-and-ack
    ping, at most once every couple of seconds, so an armed run's polls stay cheap and a
    dead runtime is still found out.
    """
    channel = session.get("channel")
    if channel is None:
        generation_dir = run_dir.parents[1]
        channel = session["channel"] = ControlChannel(
            json_file(generation_dir / LAYOUT_REL).get("schema_hash")
        )
    now = time.monotonic()
    if moving:
        alive = True
        session["control_alive"] = (True, now)
    else:
        cached = session.get("control_alive")
        if cached is not None and now - cached[1] < 2.0:
            alive = cached[0]
        else:
            if channel.available:
                channel.set_pause(bool(channel.paused))
                alive = acknowledged(channel)
            else:
                alive = False
            session["control_alive"] = (alive, now)
    return {
        "available": channel.available,
        "alive": alive,
        "paused": channel.paused,
        "speed": channel.speed,
        "seq": channel.seq,
        "applied": channel.applied,
        "speed_range": [SPEED_MIN, SPEED_MAX],
    }


def run_control(path: Path, options: dict) -> dict:
    """Pause, step, cancel, or slow a simulation through the block its loop polls.

    Takes a run or its generation. `alive` is the loop's own ack, not a guess about who
    started it; only a simulated run creates the block.
    """
    generation_dir = path if (path / LAYOUT_REL).exists() else path.parent.parent
    channel = ControlChannel(json_file(generation_dir / LAYOUT_REL).get("schema_hash"))
    action = options.get("action")
    if action == "pause":
        channel.set_pause(True)
    elif action == "resume":
        channel.set_pause(False)
    elif action == "step":
        channel.set_pause(True)
        channel.request_steps(int(options.get("ticks") or 1))
    elif action == "speed":
        channel.set_speed(float(options.get("speed")))
    elif action == "cancel":
        channel.request_stop()
    elif action is None:
        # re-publish what the block says, to ask the loop for an ack
        channel.set_pause(bool(channel.paused))
    else:
        raise ValueError(f"unknown control action: {action}")
    return {
        "available": channel.available,
        "alive": acknowledged(channel),
        "paused": channel.paused,
        "speed": channel.speed,
        "seq": channel.seq,
        "applied": channel.applied,
        "speed_range": [SPEED_MIN, SPEED_MAX],
    }


def acknowledged(channel: ControlChannel, timeout_s: float = 0.4) -> bool:
    """Whether the loop reads the block: it copies each seq back once applied."""
    if not channel.available:
        return False
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if (channel.applied or 0) >= (channel.seq or 0):
            return True
        time.sleep(0.01)
    return False
