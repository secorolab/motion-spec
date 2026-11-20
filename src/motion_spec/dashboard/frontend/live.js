// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/** A run being written: the poll that follows it, the cards that grow with it, the controls. */

import { bindToggle, iconMarkup } from "./components.js";
import { $, $$, api, nextPlotKey, post, readStored, snack, state, writeStored } from "./core.js";
import { AXIS_COLOUR, GRID_COLOUR, SERIES_COLOURS } from "./plots.js";
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

// bounds a long run's live series; the finished-run reload replaces it
const LIVE_POINT_CAP = 20000;

const LIVE_BUFFER_CAP = 5000;

const LIVE_DRIP_SLICES = 5;

const LIVE_DRIP_MS = 50;

export async function simControl(options) {
  const answer = await post("/api/control", { path: state.runPath, options }).catch(() => null);
  state.live = answer?.alive ? answer : null;
  if (state.live) showSpeed(state.live.speed);
  setTransportMode();
  return answer;
}

function stopLiveWatch() {
  clearInterval(state.liveWatch);
  clearInterval(state.liveDrip);
  state.dripping = null;
}

export async function followLiveRun(runPath) {
  stopLiveWatch();
  let settling = false;
  let handed = false; // guards against a second handover
  const poll = async () => {
    if (state.runPath !== runPath) return stopLiveWatch();
    const wasFollowing = state.following;
    const signals = livePlotsOn() ? liveSignals() : [];
    const live = await post("/api/live", { path: runPath, signals }).catch(() => null);
    // again after the await: a cancel can leave this page mid-request
    if (state.runPath !== runPath) return stopLiveWatch();
    if ((!live || live.started === false) && state.replay.pending) return holdForStart(runPath);
    // armed: paused before its first frame, waiting for play
    const armed = Boolean(live?.control?.alive && live.control.paused && !live.events.length);
    if ((live?.writing || armed) && state.replay.pending) {
      state.replay.pending = false;
      settle(false);
    }
    state.following = Boolean(live?.writing);
    state.live = state.following && live.control?.alive ? live.control : null;
    if (!state.following) {
      if (handed) return;
      // wait for archive and verification rather than show half a run
      if ((wasFollowing || settling) && live && !live.archived) {
        settling = true;
        return settle(true);
      }
      stopLiveWatch();
      settle(false);
      setTransportMode();
      if (!wasFollowing && !settling) return null;
      settling = false;
      if (!state.announced.has(runPath)) snack("run finished");
      state.announced.add(runPath);
      handed = true;
      carryOpenCards(runPath);
      return loadReplay(runPath).catch(() => showWhenOver(runPath));
    }
    if (state.live) showSpeed(state.live.speed);
    showLiveFrames(live, armed);
    appendLivePoints(live.plot);
    trackActiveMotion(live.active_motion);
  };
  await poll();
  state.liveWatch = setInterval(poll, 250);
}

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

function carryOpenCards(runPath) {
  state.reopenRows = $$("#constraints .constraint")
    .filter((row) => row.dataset.plotted)
    .map(constraintKey);
  state.reopenMotion = state.activeMotion;
  state.autoPlot = runPath;
}

// a gated signal is non-null only while its motion runs, so ask ahead
function liveSignals() {
  const registered = new Set([...state.livePlots.values()].flat());
  state.pendingSignals.forEach((signal) => {
    if (registered.has(signal)) state.pendingSignals.delete(signal);
  });
  return [...registered, ...state.pendingSignals];
}

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
  // buffer every increment: a card registers only after its history render
  arrived.forEach((points, signal) => {
    if (!points.length) return;
    const kept = state.liveBuffer.get(signal) ?? [];
    for (const point of points) kept.push(point);
    if (kept.length > LIVE_BUFFER_CAP) kept.splice(0, kept.length - LIVE_BUFFER_CAP);
    state.liveBuffer.set(signal, kept);
  });
  const batch = new Map();
  state.livePlots.forEach((signals, chart) => {
    const added = signals.map((signal) => [...(arrived.get(signal) ?? [])]);
    if (added.some((points) => points.length)) batch.set(chart, added);
  });
  dripLivePoints(batch);
}

function dripLivePoints(batch) {
  clearInterval(state.liveDrip);
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
    if (adapter.isDisposed()) return;
    const take = added.map((points) => points.splice(0, Math.ceil(points.length / slices)));
    if (!take.some((points) => points.length)) return;
    adapter.push(take);
  });
}

// one (x, y) table per signal: gated signals miss frames, uPlot.join aligns them
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
  card.querySelector(".plot-signals").replaceChildren(
    ...signals.map((signal, index) => {
      const chip = document.createElement("span");
      chip.className = "plot-signal";
      chip.style.setProperty("--series", SERIES_COLOURS[index % SERIES_COLOURS.length]);
      chip.textContent = signal;
      return chip;
    }),
  );
  $("#plots").append(card);
  if (row) card.dataset.row = row.dataset.plotKey ??= nextPlotKey();
  const holder = card.querySelector(".plot-chart");
  const axis = {
    stroke: AXIS_COLOUR,
    grid: { stroke: GRID_COLOUR, width: 1 },
    ticks: { stroke: GRID_COLOUR },
  };
  const tables = signals.map(() => [[], []]);
  signals.forEach((signal, index) => {
    const [xs, ys] = tables[index];
    (data?.signals?.[signal] ?? []).forEach((value, point) => {
      if (value == null) return;
      xs.push((data.first_frame ?? 0) + point * (data.sample_step ?? 1));
      ys.push(value);
    });
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
          stroke: SERIES_COLOURS[index % SERIES_COLOURS.length],
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

export function trackActiveMotion(handler, { plots = livePlotsOn(), live = true } = {}) {
  const motion = state.motionOf?.[handler] ?? handler;
  if (!motion || motion === state.activeMotion) return;
  const watched = live && state.activeMotion !== null;
  state.activeMotion = motion;
  const headings = $$("#constraints .constraint-motion");
  headings.forEach((heading) =>
    heading.classList.toggle("active-motion", heading.textContent === motion),
  );
  const entered = headings.findLast((heading) => heading.textContent === motion);
  const panel = $("#constraints");
  if (panel && entered) {
    panel.scrollTo({ top: entered.offsetTop - panel.offsetTop, behavior: "smooth" });
  }
  if (!plots) return;
  askForMotionSignals(motion);
  const owns = (row) => (row.dataset.monitorOwner ?? row.dataset.motion) === motion;
  for (const row of state.autoRows ?? []) {
    if (row.isConnected && row.dataset.plotted && !owns(row)) row.click();
  }
  const rows = $$("#constraints .constraint").filter(
    (row) => owns(row) && row.dataset.plottable && !row.dataset.plotted,
  );
  state.autoRows = new Set(rows);
  return rows.length ? openRowsTogether(rows, watched) : Promise.resolve();
}

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

// one history read for all rows: each read is a full-log parse
async function openRowsTogether(rows, watched) {
  let data = { signals: {}, first_frame: 0, sample_step: 1 };
  const signals = [...new Set(rows.flatMap((row) => row.plotSignals ?? []))];
  if (!watched && signals.length) {
    const query = new URLSearchParams({ path: state.runPath });
    signals.forEach((signal) => query.append("signal", signal));
    data = await api(`/api/plot?${query}`).catch(() => data);
  }
  for (const row of rows) {
    if (!row.isConnected) continue;
    row.openPlots(data);
    await new Promise((resolve) => requestAnimationFrame(() => resolve()));
  }
}

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
