// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * A run being written: the poll that follows it, the cards that grow with it, and the
 * controls that drive the loop behind it.
 */

import { $, $$, api, nextPlotKey, post, snack, state } from "./core.js";
import { addPlot } from "./plots.js";
import { loadReplay, movePlayhead, renderMarkers, setTransportMode, settle, showRunWithoutLog, showSpeed, updateReadout } from "./run.js";

// A long run would grow a live series without bound; the finished-run reload replaces it anyway.
export const LIVE_POINT_CAP = 20000;

// What a live chart renders: enough for the recent story at constant redraw cost.
export const LIVE_WINDOW = 4000;

// What a chart that is still loading its history can be handed when it registers: enough for
// the second or two an /api/plot read takes, not a second copy of the run.
export const LIVE_BUFFER_CAP = 5000;

// A 500 ms poll drawn in five steps: growth the eye follows, at a fifth of the redraws a
// 100 ms poll would cost.
export const LIVE_DRIP_SLICES = 5;

export const LIVE_DRIP_MS = 50;

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
  let settling = false;   // writing stopped, archive not yet written
  let handed = false;     // the finished run is being loaded; nothing may hand over twice
  const poll = async () => {
    if (state.runPath !== runPath) return stopLiveWatch();
    const wasFollowing = state.following;
    // Plots off asks for no series: the run's frames, events and states still come back.
    const signals = livePlotsOn() ? liveSignals() : [];
    const live = await post("/api/live", { path: runPath, signals }).catch(() => null);
    // Checked again after the await, not only before it: a cancel can take the reader back to
    // the generation page while this request is in flight, and everything below reaches for
    // elements that page does not have.
    if (state.runPath !== runPath) return stopLiveWatch();
    // `started: false` is the server saying the run is named but has written nothing yet; a null
    // is the request itself having failed. Both mean there is no run to follow.
    if ((!live || live.started === false) && state.replay.pending) {
      // named before anything is on disk: hold until the log begins or the runner gives up
      const status = state.replay.generation
        ? await api(`/api/run?path=${encodeURIComponent(state.replay.generation)}`)
            .catch(() => null)
        : null;
      if (status && !status.busy) {
        stopLiveWatch();
        settle(false);
        await showRunWithoutLog(state.replay.generation, runPath, status);
      }
      return;
    }
    // Armed: the loop answers and is paused before its first frame, so no log growth will ever
    // lift the holding overlay. Drop it -- what the run waits for is the play button.
    // No frame has been read yet exactly while the tail has produced no state line at all.
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
      // The run stops writing before it is archived and verified; until that lands the page
      // has nothing final to show, so it waits rather than showing half a run.
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
      if (!state.announced) snack("run finished");
      state.announced = true;
      handed = true;
      // Carry the open cards across the reload: the archive page reopens exactly these, or
      // falls back to the last motion that ran when the page held none.
      state.reopenRows = $$("#constraints .constraint")
        .filter((row) => row.dataset.plotted)
        .map((row) => `${row.dataset.motion}/${row.querySelector("strong")?.textContent}`);
      state.reopenMotion = state.activeMotion;
      state.autoPlot = runPath;
      return loadReplay(runPath).catch(() => {});
    }
    if (state.live) showSpeed(state.live.speed);
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
    appendLivePoints(live.plot);
    trackActiveMotion(live.active_motion);
  };
  await poll();
  // One timer for events and plot increments both; a 1 kHz run reads stale at a slower beat.
  // The server answers out of its shm ring in microseconds, so the beat is what the eye wants.
  state.liveWatch = setInterval(poll, 250);
}

// What the poll must ask for: what the registered charts draw, plus what the motion that just
// entered will draw once its cards have loaded. A gated signal is only non-null while its
// motion runs, so asking from the card's first render on would ask after the motion is over.
export function liveSignals() {
  const registered = new Set([...state.livePlots.values()].flat());
  state.pendingSignals.forEach((signal) => {
    if (registered.has(signal)) state.pendingSignals.delete(signal);
  });
  return [...registered, ...state.pendingSignals];
}

// Extend each live chart with the increment this poll carried; its history came from /api/plot.
export function appendLivePoints(plot) {
  if (!plot?.frames?.length) return;
  const arrived = new Map(Object.entries(plot.series ?? {}).map(([signal, values]) => [
    signal,
    plot.frames.map((frame, point) => [frame, values[point]]).filter(([, value]) => value != null),
  ]));
  // Keep every increment, whether or not a chart carries it yet: a card registers only after
  // its history render, and by then what landed meanwhile is gone from the server.
  arrived.forEach((points, signal) => {
    if (!points.length) return;
    const kept = [...(state.liveBuffer.get(signal) ?? []), ...points];
    state.liveBuffer.set(signal, kept.slice(Math.max(0, kept.length - LIVE_BUFFER_CAP)));
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

// One poll carries half a second of run, and drawing it in one go reads as a jump. Spread each
// batch over the poll period on one shared timer -- no per-chart timers, no echarts animation.
export function dripLivePoints(batch) {
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

// One slice of a batch: an even share of what each card has left, appended and dropped.
// Live cards are uPlot adapters -- a canvas chart built for streaming, where a full setData
// costs a millisecond; nothing here touches the replay library.
export function drawLivePoints(batch, slices) {
  batch.forEach((added, adapter) => {
    if (adapter.isDisposed?.()) return;   // its card closed while this batch was dripping
    const take = added.map((points) => points.splice(0, Math.ceil(points.length / slices)));
    if (!take.some((points) => points.length)) return;
    adapter.push(take);
  });
}

// A live card: the same chrome as a replay card, drawn by uPlot. Each signal keeps its own
// (x, y) table -- a gated signal misses frames its siblings have -- and uPlot.join aligns
// them per redraw. On the finished-run reload these cards are rebuilt as echarts replay cards.
export function addLivePlot(signals, title, detail, { row = null, data = null } = {}) {
  const card = document.createElement("section");
  card.className = "plot-card";
  card.dataset.live = "true";
  card.innerHTML = '<header><div><strong></strong><small></small></div><div class="plot-actions"><button class="remove-plot" title="Remove plot">×</button></div></header><div class="plot-signals"></div><div class="plot-chart"></div>';
  card.querySelector("strong").textContent = title;
  card.querySelector("small").textContent = detail;
  const palette = ["#e07a5f", "#79c6a5", "#9da9c7", "#f0c36a"];
  card.querySelector(".plot-signals").replaceChildren(...signals.map((signal, index) => {
    const chip = document.createElement("span");
    chip.className = "plot-signal";
    chip.style.setProperty("--series", palette[index % palette.length]);
    chip.textContent = signal;
    return chip;
  }));
  $("#plots").append(card);
  if (row) card.dataset.row = `${row.dataset.plotKey ??= nextPlotKey()}`;
  const holder = card.querySelector(".plot-chart");
  const axis = { stroke: "#73777d", grid: { stroke: "#383d45", width: 1 }, ticks: { stroke: "#383d45" } };
  const u = new uPlot({
    width: Math.max(holder.clientWidth, 320), height: 240,
    legend: { show: false }, cursor: { show: false },
    scales: { x: { time: false } },
    series: [{}, ...signals.map((_signal, index) => ({
      stroke: palette[index % palette.length], width: 1.5, spanGaps: false, points: { show: false },
    }))],
    axes: [axis, axis],
  }, uPlot.join(signals.map(() => [[], []])), holder);
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
      if (frame > last) { xs.push(frame); ys.push(value); }
    });
  });
  const adapter = {
    uplot: u,
    push(added) {
      added.forEach((points, index) => {
        const [xs, ys] = tables[index];
        for (const [frame, value] of points) { xs.push(frame); ys.push(value); }
        if (xs.length > LIVE_POINT_CAP) {
          xs.splice(0, xs.length - LIVE_POINT_CAP);
          ys.splice(0, ys.length - LIVE_POINT_CAP);
        }
      });
      u.setData(uPlot.join(tables));
    },
    resize() { u.setSize({ width: Math.max(holder.clientWidth, 320), height: 240 }); },
    isDisposed: () => !card.isConnected,
  };
  u.setData(uPlot.join(tables));
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

// Where the run is now: highlight that motion's block and put every one of its constraints
// on screen -- the constraint itself and its controller/monitor machinery -- as it enters.
// `live` is what separates a run arriving from a run being replayed: a replayed motion's
// history is on disk however many motions came before it, so it is never already-watched.
// `plots` is the reader's choice, which is a different toggle on each of the two pages.
export function trackActiveMotion(handler, { plots = livePlotsOn(), live = true } = {}) {
  const motion = state.motionOf?.[handler] ?? handler;
  if (!motion || motion === state.activeMotion) return;
  // A motion this page watched enter has nothing behind it: it starts where the live ring
  // already is. Only the motion the page opened onto was running before anyone was looking.
  const watched = live && state.activeMotion !== null;
  state.activeMotion = motion;
  $$("#constraints .constraint-motion").forEach((heading) =>
    heading.classList.toggle("active-motion", heading.textContent === motion));
  // The panel reads along: the active motion's block scrolls into view as the run enters it.
  const panel = $("#constraints");
  const heading = $$("#constraints .constraint-motion").find((h) => h.textContent === motion);
  if (panel && heading) {
    panel.scrollTo({ top: heading.offsetTop - panel.offsetTop, behavior: "smooth" });
  }
  // Where the run is stays visible with plots off; only the opening of charts is the choice.
  if (!plots) return;
  // Ask for this motion's signals from the next poll on, before a single card exists: the
  // rows below open charts that only register a second later, and by then a short motion is
  // over. The same fields the row click plots, plus what its members plot.
  // A row belongs to this entry when its data flows now: authored here with its monitor's
  // owner elsewhere means the guard already ran during the predecessor -- skip it; a guard
  // authored elsewhere but owned here is exactly what runs now.
  const owns = (row) => (row.dataset.monitorOwner ?? row.dataset.motion) === motion;
  state.replay.constraints
    .filter((constraint) => (constraint.motion ?? "shared") === motion)
    .forEach((constraint) => [
      ...constraint.tracking, ...constraint.error, ...constraint.control,
      ...(constraint.members ?? []).map((member) => member.error),
    ].forEach((signal) => state.pendingSignals.add(signal)));
  $$("#constraints .constraint")
    .filter((row) => owns(row) && row.dataset.kind === "monitored")
    .forEach((row) => (row.plotSignals ?? []).forEach((signal) => state.pendingSignals.add(signal)));
  // Cards follow the run: everything auto-opened that does not belong to THIS entry closes
  // on the transition, so the page carries one motion's charts, not the whole run's. Rows a
  // person clicked are not in autoRows and stay.
  for (const row of state.autoRows ?? []) {
    if (row.isConnected && row.dataset.plotted && !owns(row)) row.click();
  }
  const rows = $$("#constraints .constraint")
    .filter((row) => owns(row) && row.dataset.plottable && !row.dataset.plotted);
  state.autoRows = new Set(rows);
  // Handed back so a caller that has to wait for the unfold can; the live poll does not.
  return rows.length ? openRowsTogether(rows, watched) : Promise.resolve();
}

// Every card of one motion off ONE history read: a dozen rows clicked apart is a dozen
// full-log parses, and each one blocks the poll behind it.
export async function openRowsTogether(rows, watched) {
  // Nothing to read for a motion that just entered -- the poll's ring increments are its whole
  // history, and the buffered gap addPlot draws already covers what landed while cards opened.
  let data = { signals: {}, first_frame: 0, sample_step: 1 };
  const signals = [...new Set(rows.flatMap((row) => row.plotSignals ?? []))];
  if (!watched && signals.length) {
    const query = new URLSearchParams({ path: state.runPath });
    signals.forEach((signal) => query.append("signal", signal));
    data = await api(`/api/plot?${query}`).catch(() => data);
  }
  // A motion can bring a dozen cards; each echarts init costs a frame's worth of work, so
  // opening them all at once is a visible hitch. One card per animation frame reads as the
  // panel unfolding instead.
  for (const row of rows) {
    if (!row.isConnected) continue;   // the page moved on mid-unfold
    row.openPlots(data);
    await new Promise((resolve) => requestAnimationFrame(() => resolve()));
  }
}

// Plotting as the run writes is the reader's choice, not the run's: events, states and the
// console stream either way. Kept per browser, like the editor and the video panel.
export const LIVE_PLOTS_KEY = "motion-spec.live-plots";

export let livePlots = true;

try { livePlots = localStorage.getItem(LIVE_PLOTS_KEY) !== "off"; } catch { /* private */ }

export function livePlotsOn() {
  return livePlots;
}

export function bindLivePlots() {
  const button = $("#live-plots");
  const label = () => {
    button.setAttribute("aria-pressed", String(livePlots));
    button.textContent = `live plots: ${livePlots ? "on" : "off"}`;
  };
  button.onclick = () => {
    livePlots = !livePlots;
    try { localStorage.setItem(LIVE_PLOTS_KEY, livePlots ? "on" : "off"); } catch { /* private */ }
    label();
  };
  label();
}
