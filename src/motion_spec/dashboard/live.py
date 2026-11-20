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
from motion_spec.dashboard.replay import log_events
from motion_spec.dashboard.roots import trace
from motion_spec.dashboard.runs import GenerationInfo
from motion_spec.dashboard.tail import FrameLogTail
from motion_spec.runs.control import SPEED_MAX, SPEED_MIN, acknowledged, control_run, run_channel
from motion_spec.runs.provenance import rec_run, run_generation
from motion_spec.runs.replay import resolve_run
from motion_spec.runs.verify import RecordError
from motion_spec.telemetry import frame_log_pb
from motion_spec.telemetry.frame_log_pb import shm_name_for, shm_path

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
        core = {key: read(self.fields.offsets[key], raw) for key in CORE_SIGNALS}
        step = core["step"]
        if step == self.seen:
            return
        # The block's standing frame is not an advance: a run armed at the play button may have
        # published one already, and armed means no events yet.
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


_NOT_STARTED = {
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


def live_state(run_dir: Path, signals=()) -> dict:
    """How far a run being written has got, and the states and events it has passed.

    Live is the runtime's shared-memory frame, sampled at LIVE_SAMPLE_HZ into a ring: `signals`
    is served out of that ring as the increment since this page's last poll, values and events
    and states alike. Only a build that publishes no block falls back to following the log.

    A run is named, and its directory made, before the runtime writes a frame log; the page polls
    from the moment it is named. That poll is a well-formed question about a run that has not
    started, so it is answered -- `started: False` -- rather than rejected.
    """
    session = _LIVE.get(str(run_dir))
    if session is None:
        log_less = False
        # Resolving parses the full header contract -- once per session, never per poll.
        try:
            _, log, contract = resolve_run(run_dir)
        except RecordError:
            # No frame log yet: named but not writing. A run started with logs off never has
            # one, so its control block is reached through the generation's own contract
            # record instead; the page then has a loop to drive and the run has a play button.
            started = RUNNING.get(str(run_dir))
            if not started or started.get("recorded") is not False:
                return _NOT_STARTED
            generation = run_generation(run_dir)
            log = generation / "generated/contract/frame_log_header.pb" if generation else None
            if log is None or not log.is_file():
                return _NOT_STARTED
            contract, log_less = frame_log_pb.read_contract(log), True
        # A session that ends drops its sampler and whatever log handle it fell back to.
        for stale in list(_LIVE.values()):
            if stale.get("sampler") is not None:
                stale["sampler"].stopped.set()
            if stale.get("tail") is not None:
                stale["tail"].close()
            if stale.get("channel") is not None:
                stale["channel"].close()
        _LIVE.clear()
        session = _LIVE[str(run_dir)] = {
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
    log, contract = Path(session["log"]), session["contract"]
    sampler = session["sampler"]
    if sampler is not None and not sampler.is_alive():
        # A page whose tab was backgrounded stops polling; the sampler gives up and this asks
        # again. What it missed meanwhile is the archive's, not the live view's.
        sampler = session["sampler"] = live_sampler(run_dir, contract)
        session["cursor"] = None
    if sampler is not None:
        sampler.touched = time.monotonic()
    live = sampler is not None and sampler.reading
    period = (contract.header.nominal_period_ns or 1_000_000) / 1e9
    if not live:
        # Where the run is, read off the log itself -- for a build that publishes no frame
        # block. The archive is protobuf, so this parses; it is the fallback, never the live path.
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
    names = tuple(signals)
    plot = None
    if names and live:
        # Only a new cursor skips to the ring's end; /api/plot is where history comes from.
        # The cursor counts samples, not names, so a signal set that changes mid-run keeps
        # every sample taken since the last poll.
        if session["cursor"] is None:
            session["cursor"] = sampler.taken
        with sampler.lock:
            ring, taken = list(sampler.ring), sampler.taken
        new = ring[max(0, session["cursor"] - (taken - len(ring))) :]
        session["cursor"] = taken
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
    # Markers come off the log, which keeps every tick, while the sampled block misses any
    # edge falling between two samples. The scan is incremental, so a poll pays for the frames
    # written since the last one. A run with no log yet falls back to what the block said.
    try:
        events = list(log_events(log, contract)["events"])
    except OSError:
        events = []
    if not events:
        events = list(sampler.events) if live else session["events"]
    motions = contract.header.motions
    # A loop that answers is live even while paused. Only a run with no control block to ask
    # -- real hardware, or a build older than it -- has to be judged by its log growing.
    # Whether the run stepped recently is what `writing` means with no log to watch.
    moving = live and time.monotonic() - sampler.advanced < LIVE_IDLE_S
    # The control block is read, not pinged: a status poll must never write and wait. A moving
    # sampler is proof of life for free; only an idle run gets the publish-and-ack ping, at most
    # once every couple of seconds, so an armed run's polls stay cheap and a dead runtime is
    # still found out.
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
    elif channel.available:
        channel.set_pause(bool(channel.paused))
        alive = acknowledged(channel)
        session["control_alive"] = (alive, now)
    else:
        alive = False
        session["control_alive"] = (False, now)
    control = {
        "available": channel.available,
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
    # A log-less run's "log" is the generation's contract record; its mtime says nothing.
    if not control["available"] and not live and not session["log_less"]:
        writing = writing or time.time() - stat.st_mtime < LIVE_IDLE_S
    return {
        # With no log to appear, a log-less run has started once its loop answers.
        "started": writing if session["log_less"] else True,
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


def run_control(run_dir: Path, options: dict) -> dict:
    """What `motion-spec control` and `motion-spec cancel` do, for the live page.

    `alive` is the loop's own ack, not a guess about who started it; only a simulated run
    creates the block.
    """
    action = options.get("action") or "status"
    if action == "cancel":
        # Said before asking, so the generation page reports the end as a stop, not a failure.
        mark_stopped(run_dir)
        rec_run(run_dir).cancel()
        action = "status"
    value = options.get("ticks") if action == "step" else options.get("speed")
    return control_run(run_dir, action, None if value is None else float(value))
