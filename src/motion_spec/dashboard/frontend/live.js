// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/** A run being written: the poll that follows it, the cards that grow with it, the controls. */

import { bindToggle, iconMarkup } from "./components.js";
import { $, $$, api, nextPlotKey, post, readStored, snack, state, writeStored } from "./core.js";
import {
  constraintKey,
  loadReplay,
  movePlayhead,
  renderMarkers,
  setTransportMode,
  settle,
  showRunWithoutLog,
  showSpeed,
  updateReadout,
} from "./run.js";

// A long run would grow a live series without bound; the finished-run reload replaces it anyway.
export const LIVE_POINT_CAP = 20000;

// Enough to cover the second or two an /api/plot read takes, not a second copy of the run.
const LIVE_BUFFER_CAP = 5000;

// Each poll is drawn in this many steps: growth the eye follows, at a fifth of the redraws.
const LIVE_DRIP_SLICES = 5;

const LIVE_DRIP_MS = 50;

// Drive the sim this run belongs to; alive is the loop's own ack.
export async function simControl(options) {
  const answer = await post("/api/control", { path: state.runPath, options }).catch(() => null);
  state.live = answer?.alive ? answer : null;
  if (state.live) showSpeed(state.live.speed);
  setTransportMode();
  return answer;
}

// The plot drip belongs to the poll that fed it: both timers stop together.
export function stopLiveWatch() {
  clearInterval(state.liveWatch);
  clearInterval(state.liveDrip);
  state.dripping = null;
}

// Follow the log as it is written; when it stops, load the finished run.
export async function followLiveRun(runPath) {
  stopLiveWatch();
  let settling = false; // writing stopped, archive not yet written
  let handed = false; // the finished run is being loaded; nothing may hand over twice
  const poll = async () => {
    if (state.runPath !== runPath) return stopLiveWatch();
    const wasFollowing = state.following;
    // Plots off asks for no series: the run's frames, events and states still come back.
    const signals = livePlotsOn() ? liveSignals() : [];
    const live = await post("/api/live", { path: runPath, signals }).catch(() => null);
    // Again after the await: a cancel can take the reader off this page mid-request, and
    // everything below reaches for elements the generation page does not have.
    if (state.runPath !== runPath) return stopLiveWatch();
    // `started: false` means named but nothing written; null means the request failed.
    if ((!live || live.started === false) && state.replay.pending) return holdForStart(runPath);
    // Armed: the loop answers, paused before its first frame, so no log growth will ever lift
    // the holding overlay. What the run waits for is the play button.
    const armed = Boolean(live?.control?.alive && live.control.paused && !live.events.length);
    if ((live?.writing || armed) && state.replay.pending) {
      state.replay.pending = false;
      settle(false);
    }
    state.following = Boolean(live?.writing);
    state.live = state.following && live.control?.alive ? live.control : null;
    if (!state.following) {
      // A poll still in flight when the handover began would settle a page that has moved on.
      if (handed) return;
      // The run stops writing before it is archived and verified, so the page waits for that
      // rather than showing half a run.
      if ((wasFollowing || settling) && live && !live.archived) {
        settling = true;
        return settle(true);
      }
      stopLiveWatch();
      settle(false);
      setTransportMode();
      // read the finished run back, for its health and full frame count
      if (!wasFollowing && !settling) return null;
      settling = false;
      if (!state.announced.has(runPath)) snack("run finished");
      state.announced.add(runPath);
      handed = true;
      carryOpenCards(runPath);
      // A run that kept no log has no archive to load: what it left is its console.
      return loadReplay(runPath).catch(() => showWhenOver(runPath));
    }
    if (state.live) showSpeed(state.live.speed);
    showLiveFrames(live, armed);
    appendLivePoints(live.plot);
    trackActiveMotion(live.active_motion);
  };
  await poll();
  // One timer for events and plot increments both. The server answers out of its shm ring in
  // microseconds, so the beat is set by what the eye wants, not by what a read costs.
  state.liveWatch = setInterval(poll, 250);
}

// The runner verifies and reports after the run itself is over; the console view waits for it.
function showWhenOver(runPath) {
  const hop = setInterval(async () => {
    if (state.runPath !== runPath) return clearInterval(hop);
    const status = await api(`/api/run?path=${encodeURIComponent(runPath)}`).catch(() => null);
    if (status && !status.busy) {
      clearInterval(hop);
      await showRunWithoutLog(state.replay.generation, runPath, status);
    }
  }, 300);
}

// Named before anything is on disk: hold until the log begins or the runner gives up.
async function holdForStart(runPath) {
  const status = await api(`/api/run?path=${encodeURIComponent(runPath)}`).catch(() => null);
  if (status && !status.busy) {
    stopLiveWatch();
    settle(false);
    await showRunWithoutLog(state.replay.generation, runPath, status);
  }
}

function showLiveFrames(live, armed) {
  state.replay.frames = live.frames;
  state.replay.duration = live.duration;
  state.replay.events = live.events;
  $(".timeline").max = Math.max(0, live.frames - 1);
  renderMarkers();
  if (!state.live?.paused) state.frame = live.frames - 1;
  state.frame = Math.min(state.frame, live.frames - 1);
  $(".timeline").value = state.frame;
  updateReadout();
  if (armed) $("#readout").textContent = "armed — press play";
  movePlayhead();
  setTransportMode();
}

// Carry the open cards across the reload; the archive page reopens exactly these.
function carryOpenCards(runPath) {
  state.reopenRows = $$("#constraints .constraint")
    .filter((row) => row.dataset.plotted)
    .map(constraintKey);
  state.reopenMotion = state.activeMotion;
  state.autoPlot = runPath;
}

// What the registered charts draw, plus what the motion that just entered will draw. A gated
// signal is non-null only while its motion runs, so waiting for its card would be too late.
function liveSignals() {
  const registered = new Set([...state.livePlots.values()].flat());
  state.pendingSignals.forEach((signal) => {
    if (registered.has(signal)) state.pendingSignals.delete(signal);
  });
  return [...registered, ...state.pendingSignals];
}

// Extend each live chart with the increment this poll carried; its history came from /api/plot.
function appendLivePoints(plot) {
  if (!plot?.frames?.length) return;
  const arrived = new Map(
    Object.entries(plot.series ?? {}).map(([signal, values]) => [
      signal,
      plot.frames
        .map((frame, point) => [frame, values[point]])
        .filter(([, value]) => value != null),
    ]),
  );
  // Keep every increment, whether or not a chart carries it yet: a card registers only after
  // its history render, and by then what landed meanwhile is gone from the server.
  arrived.forEach((points, signal) => {
    if (!points.length) return;
    let kept = state.liveBuffer.get(signal);
    if (!kept) state.liveBuffer.set(signal, (kept = []));
    for (const point of points) kept.push(point);
    if (kept.length > LIVE_BUFFER_CAP) kept.splice(0, kept.length - LIVE_BUFFER_CAP);
  });
  const batch = new Map();
  state.livePlots.forEach((signals, chart) => {
    // The drip consumes what it is handed, so each chart draws from its own copy.
    const added = signals.map((signal) => [...(arrived.get(signal) ?? [])]);
    // A chart whose signals were all gated off this poll has nothing to redraw.
    if (added.some((points) => points.length)) batch.set(chart, added);
  });
  dripLivePoints(batch);
}

// Drawing a whole poll in one go reads as a jump, so it is spread over the poll period on one
// shared timer -- no per-chart timers, no echarts animation.
function dripLivePoints(batch) {
  clearInterval(state.liveDrip);
  // Whatever the last batch had left goes in now: the charts never fall behind the run.
  if (state.dripping) drawLivePoints(state.dripping, 1);
  state.dripping = batch.size ? batch : null;
  if (!state.dripping) return;
  let slices = LIVE_DRIP_SLICES;
  state.liveDrip = setInterval(() => {
    drawLivePoints(batch, slices);
    if (--slices > 0) return;
    clearInterval(state.liveDrip);
    state.dripping = null;
  }, LIVE_DRIP_MS);
}

function drawLivePoints(batch, slices) {
  batch.forEach((added, adapter) => {
    if (adapter.isDisposed?.()) return; // its card closed while this batch was dripping
    const take = added.map((points) => points.splice(0, Math.ceil(points.length / slices)));
    if (!take.some((points) => points.length)) return;
    adapter.push(take);
  });
}

// A live card, drawn by uPlot. Each signal keeps its own (x, y) table, because a gated signal
// misses frames its siblings have, and uPlot.join aligns them per redraw.
export function addLivePlot(signals, title, detail, { row = null, data = null } = {}) {
  const card = document.createElement("section");
  card.className = "plot-card";
  card.dataset.live = "true";
  card.innerHTML =
    '<header><div><strong></strong><small></small></div><div class="plot-actions">' +
    `<button class="remove-plot" title="Remove plot" aria-label="Remove plot">${iconMarkup("x")}</button>` +
    '</div></header><div class="plot-signals"></div><div class="plot-chart"></div>';
  card.querySelector("strong").textContent = title;
  card.querySelector("small").textContent = detail;
  const palette = ["#e07a5f", "#79c6a5", "#9da9c7", "#f0c36a"];
  card.querySelector(".plot-signals").replaceChildren(
    ...signals.map((signal, index) => {
      const chip = document.createElement("span");
      chip.className = "plot-signal";
      chip.style.setProperty("--series", palette[index % palette.length]);
      chip.textContent = signal;
      return chip;
    }),
  );
  $("#plots").append(card);
  if (row) card.dataset.row = row.dataset.plotKey ??= nextPlotKey();
  const holder = card.querySelector(".plot-chart");
  const axis = {
    stroke: "#73777d",
    grid: { stroke: "#383d45", width: 1 },
    ticks: { stroke: "#383d45" },
  };
  const tables = signals.map(() => [[], []]);
  signals.forEach((signal, index) => {
    const [xs, ys] = tables[index];
    (data?.signals?.[signal] ?? []).forEach((value, point) => {
      if (value == null) return;
      xs.push((data.first_frame ?? 0) + point * (data.sample_step ?? 1));
      ys.push(value);
    });
    // What landed while this card was opening sits in the buffer, nowhere else.
    const last = xs.at(-1) ?? -Infinity;
    (state.liveBuffer.get(signal) ?? []).forEach(([frame, value]) => {
      if (frame > last) {
        xs.push(frame);
        ys.push(value);
      }
    });
  });
  const u = new uPlot(
    {
      width: Math.max(holder.clientWidth, 320),
      height: 240,
      legend: { show: false },
      cursor: { show: false },
      scales: { x: { time: false } },
      series: [
        {},
        ...signals.map((_signal, index) => ({
          stroke: palette[index % palette.length],
          width: 1.5,
          spanGaps: false,
          points: { show: false },
        })),
      ],
      axes: [axis, axis],
    },
    uPlot.join(tables),
    holder,
  );
  const adapter = {
    uplot: u,
    push(added) {
      added.forEach((points, index) => {
        const [xs, ys] = tables[index];
        for (const [frame, value] of points) {
          xs.push(frame);
          ys.push(value);
        }
        if (xs.length > LIVE_POINT_CAP) {
          xs.splice(0, xs.length - LIVE_POINT_CAP);
          ys.splice(0, ys.length - LIVE_POINT_CAP);
        }
      });
      u.setData(uPlot.join(tables));
    },
    resize() {
      u.setSize({ width: Math.max(holder.clientWidth, 320), height: 240 });
    },
    isDisposed: () => !card.isConnected,
  };
  const observer = new ResizeObserver(() => card.isConnected && adapter.resize());
  observer.observe(holder);
  card.querySelector(".remove-plot").onclick = () => {
    observer.disconnect();
    state.livePlots.delete(adapter);
    u.destroy();
    card.remove();
    if (row && !$(`#plots [data-row="${card.dataset.row}"]`)) delete row.dataset.plotted;
  };
  state.livePlots.set(adapter, signals);
}

// Where the run is now: light that motion's block and open its constraints as it enters.
// `live` separates an arriving run from a replayed one, whose history is all on disk already;
// `plots` is the reader's choice, a different toggle on each of the two pages.
export function trackActiveMotion(handler, { plots = livePlotsOn(), live = true } = {}) {
  const motion = state.motionOf?.[handler] ?? handler;
  if (!motion || motion === state.activeMotion) return;
  // A motion this page watched enter starts where the live ring already is, so it needs no
  // history read. Only the motion the page opened onto was running before anyone was looking.
  const watched = live && state.activeMotion !== null;
  state.activeMotion = motion;
  let entered = null;
  $$("#constraints .constraint-motion").forEach((heading) => {
    const here = heading.textContent === motion;
    heading.classList.toggle("active-motion", here);
    if (here) entered = heading;
  });
  // The panel reads along: the active motion's block scrolls into view as the run enters it.
  const panel = $("#constraints");
  if (panel && entered) {
    panel.scrollTo({ top: entered.offsetTop - panel.offsetTop, behavior: "smooth" });
  }
  // Where the run is stays visible with plots off; only the opening of charts is the choice.
  if (!plots) return;
  askForMotionSignals(motion);
  // A row belongs to this entry when its data flows now, which for a when-guard is the motion
  // that owns its monitor rather than the one it was authored in.
  const owns = (row) => (row.dataset.monitorOwner ?? row.dataset.motion) === motion;
  // Everything auto-opened elsewhere closes on the transition, so the page carries one
  // motion's charts. Rows a person clicked are not in autoRows and stay.
  for (const row of state.autoRows ?? []) {
    if (row.isConnected && row.dataset.plotted && !owns(row)) row.click();
  }
  const rows = $$("#constraints .constraint").filter(
    (row) => owns(row) && row.dataset.plottable && !row.dataset.plotted,
  );
  state.autoRows = new Set(rows);
  // Handed back so a caller that has to wait for the unfold can; the live poll does not.
  return rows.length ? openRowsTogether(rows, watched) : Promise.resolve();
}

// Ask from the next poll on, before a card exists: the rows below register a second later,
// and by then a short motion is over.
function askForMotionSignals(motion) {
  state.replay.constraints
    .filter((constraint) => (constraint.motion ?? "shared") === motion)
    .forEach((constraint) =>
      [
        ...constraint.tracking,
        ...constraint.error,
        ...constraint.control,
        ...(constraint.members ?? []).map((member) => member.error),
      ].forEach((signal) => state.pendingSignals.add(signal)),
    );
  $$("#constraints .constraint")
    .filter(
      (row) =>
        (row.dataset.monitorOwner ?? row.dataset.motion) === motion &&
        row.dataset.kind === "monitored",
    )
    .forEach((row) =>
      (row.plotSignals ?? []).forEach((signal) => state.pendingSignals.add(signal)),
    );
}

// Every card of one motion off one history read: a dozen rows clicked apart is a dozen
// full-log parses, and each one blocks the poll behind it.
async function openRowsTogether(rows, watched) {
  // A motion that just entered has no history to read: the ring increments are all of it.
  let data = { signals: {}, first_frame: 0, sample_step: 1 };
  const signals = [...new Set(rows.flatMap((row) => row.plotSignals ?? []))];
  if (!watched && signals.length) {
    const query = new URLSearchParams({ path: state.runPath });
    signals.forEach((signal) => query.append("signal", signal));
    data = await api(`/api/plot?${query}`).catch(() => data);
  }
  // Each echarts init costs a frame's worth of work, so a dozen at once is a visible hitch.
  for (const row of rows) {
    if (!row.isConnected) continue; // the page moved on mid-unfold
    row.openPlots(data);
    await new Promise((resolve) => requestAnimationFrame(() => resolve()));
  }
}

// Only the charts are the reader's choice: events, states and the console stream either way.
const LIVE_PLOTS_KEY = "motion-spec.live-plots";

let livePlots = readStored(LIVE_PLOTS_KEY) !== "off";

export function livePlotsOn() {
  return livePlots;
}

export function bindLivePlots() {
  bindToggle($("#live-plots"), "live plots", livePlotsOn, () => {
    livePlots = !livePlots;
    writeStored(LIVE_PLOTS_KEY, livePlots ? "on" : "off");
  });
}
