// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * One run's page: its constraints, its transport, its recording and its console --
 * whether the run is finished or still being written.
 */

import { COMPARE_MARKUP, loadCompare } from "./compare.js";
import { appendConsole, consoleExcerpt, showEmpty } from "./components.js";
import { $, $$, api, snack, state } from "./core.js";
import { EXPLORE_MARKUP, bindExplore } from "./explore.js";
import { highlightGeneration, selectGeneration, stopRun } from "./generations.js";
import { addLivePlot, bindLivePlots, followLiveRun, livePlotsOn, simControl } from "./live.js";
import { openNotebook } from "./notebook.js";
import { addPlot, cursorOption } from "./plots.js";
import { showReports } from "./reports.js";
import { setView } from "./routing.js";
import { loadViews } from "./views.js";

// A run that was just started: its page, open before its log exists. The server names the
// run as it starts it and /api/replay answers from the generation's serialized contract, so
// the page is normally complete before the runtime is even up. Generations from before that
// contract file existed fall back to a holding card until the log begins.
export async function openPendingRun(runPath) {
  // A run this page started plots itself -- live as motions enter, or all at once when a
  // headless run outpaces the page and only its archive is left to show.
  state.autoPlot = runPath;
  if (await loadReplay(runPath).then(() => true).catch(() => false)) return;
  state.anchor = runPath;
  stopPlayback();
  setView("run", runPath);
  state.runPath = runPath;
  state.generationPath = runPath.split("/runs/")[0];
  state.live = state.following = null;
  highlightGeneration();
  // A run that never reaches its first frame is waited on here, so this is where it is ended.
  const waiting = showEmpty({
    spinner: true,
    title: "Run starting…",
    detail: "Waiting for the first frames of the log.",
  });
  waiting.insertAdjacentHTML("beforeend", '<button id="stop-pending">stop run</button>');
  $("#stop-pending").onclick = async (event) => {
    event.target.disabled = true;
    await stopRun(state.generationPath).catch((error) => snack(error.message));
  };
  const hop = setInterval(async () => {
    if (state.runPath !== runPath) return clearInterval(hop);   // the user went elsewhere
    if (await loadReplay(runPath).then(() => true).catch(() => false)) return clearInterval(hop);
    const status = await api(`/api/run?path=${encodeURIComponent(state.generationPath)}`)
      .catch(() => null);
    if (status && !status.busy) {
      // over without ever writing a log: the runner's own words are all there is to show
      clearInterval(hop);
      await showRunWithoutLog(state.generationPath, runPath, status);
    }
  }, 300);
}

export async function loadReplay(path) {
  state.anchor = path;
  stopPlayback();
  setView("run", path);
  state.replay = await api(`/api/replay?path=${encodeURIComponent(path)}`);
  state.charts.forEach((chart) => chart.dispose());
  state.charts = [];
  state.livePlots.clear();
  state.pendingSignals.clear();
  state.liveBuffer.clear();
  state.activeMotion = null;
  $("#content").innerHTML = replayShell(path);
  state.runPath = path;
  state.generationPath = state.replay.generation;
  state.live = null;
  highlightGeneration();
  state.frame = 0;
  const timeline = $(".timeline");
  timeline.max = Math.max(0, state.replay.frames - 1);
  timeline.disabled = false;

  // gate id -> the name the source spells it: a row carries the name, not the gate it ran under
  state.motionOf = state.replay.motion_names ?? {};
  populateConstraints();
  // The archive of a run this page started: reopen the cards that were open when it ended --
  // or, when none were (a run faster than the page), the last motion that ran. Never the
  // whole run's card set: eighty charts in one page is what made live pages crawl.
  if (state.autoPlot === path && !state.replay.pending && livePlotsOn()) {
    state.autoPlot = null;
    const keys = state.reopenRows ?? [];
    let lastMotion = state.reopenMotion;
    state.reopenRows = state.reopenMotion = null;
    if (!keys.length && !lastMotion) {
      lastMotion = state.replay.constraints
        .filter((constraint) => constraint.window)
        .sort((a, b) => b.window[1] - a.window[1])[0]?.motion;
    }
    // Each row is a full-log /api/plot read: click them apart so several never land in one
    // frame. Detached on purpose -- the page is usable while the cards fill in.
    (async () => {
      const rows = $$("#constraints .constraint").filter((row) => {
        if (!row.dataset.plottable || row.dataset.plotted || row.dataset.idle) return false;
        const key = `${row.dataset.motion}/${row.querySelector("strong")?.textContent}`;
        return keys.length ? keys.includes(key) : row.dataset.motion === lastMotion;
      });
      for (const row of rows) {
        if (state.runPath !== path) return;   // the reader moved on
        row.click();
        await new Promise((resolve) => setTimeout(resolve, 50));
      }
    })();
  }
  $("#plot").onclick = () => addPlot([]);
  $("#notebook").onclick = () => openNotebook(state.runPath).catch((error) => snack(error.message));
  bindLivePlots();
  bindPanels();
  // A side panel that cannot load is not the page failing to load.
  bindExplore(path).catch((error) => snack(error.message));
  bindTransport();
  setTransportMode();
  showVideos(path, state.replay.videos ?? []);
  // Nothing recorded: a real platform has a camera, but only ROS to reach it through.
  if (!state.replay.videos?.length) showRosCamera(state.replay.generation);
  followLiveRun(path);
  followConsole(path);
  // Built from the generation's contract before the runtime wrote anything: hold the page
  // until the log begins.
  if (state.replay.pending) settle(true, "waiting for the run to start…");
  $("#back").onclick = () => selectGeneration(state.replay.generation);
  updateReadout();
}

// The panel floats over the page, so the page ends above it rather than behind it.
export function reserveVideoSpace() {
  const transport = $(".transport");
  if (transport) {
    document.documentElement.style.setProperty("--transport-h", `${transport.offsetHeight}px`);
  }
  const panel = $(".videos");
  const covered = panel && !panel.hidden && !panel.classList.contains("minimized");
  const style = document.documentElement.style;
  style.setProperty("--video-h", covered ? `${panel.offsetHeight + 16}px` : "0px");
  style.setProperty("--video-w", covered ? `${panel.offsetWidth + 16}px` : "0px");
}

// Waiting on the run: nothing on the page is worth clicking yet.
export function settle(waiting, message = "archiving the run…") {
  $(".settling").hidden = !waiting;
  $(".settling span").textContent = message;
  $(".replay").classList.toggle("busy", waiting);
  $(".transport").classList.toggle("busy", waiting);
}

// What the run recorded: one camera large, the rest alongside to swap in.
export function showVideos(runPath, cameras) {
  const panel = $(".videos");
  panel.hidden = !cameras.length;
  if (!cameras.length) return reserveVideoSpace();
  const url = (camera) =>
    `/api/video?path=${encodeURIComponent(runPath)}&camera=${encodeURIComponent(camera)}`;
  const main = panel.querySelector("video");
  main.oncontextmenu = (event) => event.preventDefault();
  // The other half of syncVideo's one-seek-at-a-time: a drag's newest target lands here.
  main.onseeked = () => {
    const queued = main._queuedSeek;
    main._queuedSeek = undefined;
    if (queued !== undefined) main.currentTime = queued;
  };
  bindVideoExpand(panel);
  const minimize = panel.querySelector(".video-min");
  const setMinimized = (small) => {
    panel.classList.toggle("minimized", small);
    minimize.textContent = small ? "\u25a1" : "\u2013";
    minimize.title = small ? "Show the recording" : "Minimize";
    try { localStorage.setItem("motion-spec.video-minimized", String(small)); } catch { /* private */ }
    reserveVideoSpace();
  };
  let small = false;
  try { small = localStorage.getItem("motion-spec.video-minimized") === "true"; } catch { /* private */ }
  minimize.onclick = () => setMinimized(!panel.classList.contains("minimized"));
  setMinimized(small);
  const show = (camera) => {
    main.src = url(camera);
    // A video that is never played paints nothing: put it where the cursor is once it knows
    // how long it is.
    main.onloadedmetadata = () => {
      // Fit the box to the recording rather than the recording to the box: a 3D view with the
      // panels open is portrait, and a landscape box would letterbox it.
      const portrait = main.videoHeight > main.videoWidth;
      main.style.width = portrait ? "auto" : "100%";
      main.style.height = portrait ? "var(--video-h, 34vh)" : "auto";
      reserveVideoSpace();
      syncVideo(true);
    };
    panel.querySelector(".video-name").textContent = camera;
    panel.querySelectorAll(".video-pick").forEach((pick) =>
      pick.setAttribute("aria-pressed", pick.dataset.camera === camera));
  };
  // Nothing to swap between with one camera: the strip would be the same view again.
  const strip = panel.querySelector(".video-strip");
  strip.hidden = cameras.length < 2;
  panel.classList.toggle("one-camera", cameras.length < 2);
  strip.replaceChildren(...cameras.map((camera) => {
    const pick = document.createElement("button");
    pick.className = "video-pick";
    pick.dataset.camera = camera;
    pick.innerHTML = `<video muted playsinline preload="metadata"></video><span>${camera}</span>`;
    const thumb = pick.querySelector("video");
    thumb.src = url(camera);
    // Show something of the run rather than the black frame it opens on.
    thumb.onloadedmetadata = () => { thumb.currentTime = Math.min(1, thumb.duration / 2); };
    pick.onclick = () => show(camera);
    return pick;
  }));
  show(cameras[0]);
  reserveVideoSpace();
}

// The expand toggle is shared by the recording pane and the ROS pane: one class, one size
// custom property the video rules read, remembered like the minimize state.
function bindVideoExpand(panel) {
  const expand = panel.querySelector(".video-max");
  const setExpanded = (large) => {
    panel.classList.toggle("expanded", large);
    expand.textContent = large ? "⤡" : "⤢";
    expand.title = large ? "Shrink" : "Expand";
    try { localStorage.setItem("motion-spec.video-expanded", String(large)); } catch { /* private */ }
    reserveVideoSpace();
  };
  let large = false;
  try { large = localStorage.getItem("motion-spec.video-expanded") === "true"; } catch { /* private */ }
  expand.onclick = () => setExpanded(!panel.classList.contains("expanded"));
  setExpanded(large);
}

// The same pane, pointed at a live ROS topic: hardware records nothing to replay afterwards,
// so the picture is whatever the camera is publishing now -- not a track the timeline drives.
async function showRosCamera(generationPath) {
  if (!generationPath) return;
  const generation = state.generation?.path === generationPath
    ? state.generation
    : await api(`/api/generation?path=${encodeURIComponent(generationPath)}`).catch(() => null);
  if (generation?.simulated !== false || state.replay?.generation !== generationPath) return;
  // The model names its cameras, and a published camera is <id>/color -- the external driver
  // for the same sensor is expected on the same name. Anything else is typed in.
  const declared = generation.cameras?.[0]?.id;
  const defaultTopic = declared ? `/${declared}/color` : "";
  const panel = $(".videos");
  panel.hidden = false;
  panel.classList.add("one-camera", "ros-camera");
  panel.querySelector(".video-strip").hidden = true;
  // The minimize button belongs to showVideos' recording panel; this one is a single live view.
  panel.querySelector(".video-min").hidden = true;
  bindVideoExpand(panel);
  const main = panel.querySelector(".video-main");
  main.innerHTML = `<img class="ros-frame" alt=""><div class="ros-topic"><input type="text" spellcheck="false" title="ROS image topic"><span class="ros-status"></span></div>`;
  const frame = main.querySelector(".ros-frame");
  const topic = main.querySelector("input");
  const status = main.querySelector(".ros-status");
  topic.value = state.rosTopic ?? defaultTopic;
  topic.placeholder = "ROS image topic";
  const subscribe = () => {
    state.rosTopic = topic.value.trim() || defaultTopic;
    topic.value = state.rosTopic;
    status.textContent = "";
    if (!state.rosTopic) return;
    // A new query each time, so the browser reconnects rather than showing the stalled stream.
    frame.src = `/api/ros-camera?topic=${encodeURIComponent(state.rosTopic)}&t=${Date.now()}`;
  };
  frame.onerror = () => {
    frame.removeAttribute("src");
    status.textContent = "source unavailable (no ROS env)";
    reserveVideoSpace();
  };
  frame.onload = () => reserveVideoSpace();
  topic.onchange = subscribe;
  subscribe();
  reserveVideoSpace();
}

// Follow the run's terminal words the way the plots follow its frames.
export function followConsole(runPath) {
  clearInterval(state.consoleWatch);
  let offset = 0;
  const pre = $("#console-text");
  const poll = async () => {
    if (state.runPath !== runPath || !pre.isConnected) return clearInterval(state.consoleWatch);
    const slice = await api(`/api/console?path=${encodeURIComponent(runPath)}&offset=${offset}`)
      .catch(() => null);
    if (!slice || !slice.text) return;
    offset = slice.offset;
    appendConsole(pre, slice.text);
  };
  poll();
  state.consoleWatch = setInterval(poll, 1000);
}

// A run over with no log to open. Told not to write one is not the same as never having run:
// the run's manifest says which, and a log-less run still has its console to show.
export async function showRunWithoutLog(generationPath, runPath, status) {
  if (status?.recorded !== false) return showRunNeverStarted(generationPath, status?.exit_code);
  const empty = showEmpty({
    eyebrow: "NO LOG",
    title: "This run recorded no frame log (logs off).",
    detail: "No replay and no plots. The console is what the run left behind.",
  });
  empty.append(await consoleExcerpt(runPath, 500));
}

// A run that ended before it wrote anything: what the runner printed is all there is to show.
export async function showRunNeverStarted(generationPath, exitCode) {
  const empty = showEmpty({
    eyebrow: "ERROR",
    title: "The run never started.",
    detail: `exited ${exitCode ?? "?"}`,
  });
  empty.append(await consoleExcerpt(generationPath));
}

export function populateConstraints() {
  const groups = new Map();
  state.replay.constraints.forEach((constraint) => {
    const motion = constraint.motion ?? "shared";
    groups.set(motion, [...(groups.get(motion) ?? []), constraint]);
  });
  const container = $("#constraints");
  container.replaceChildren(...[...groups].flatMap(([motion, constraints]) => {
    const heading = document.createElement("div");
    heading.className = "constraint-motion";
    heading.textContent = motion;
    return [heading, ...constraints.map((constraint) => {
    const row = document.createElement("div");
    const line = document.createElement("span");
    const name = document.createElement("strong");
    const expression = document.createElement("span");
    row.className = "constraint";
    row.dataset.kind = constraint.kind;
    row.dataset.motion = motion;   // which block a live run's active motion lights up
    // A when-guard's monitor is owned by the predecessor motion: its data flows in THAT
    // motion's window, so plots and auto-opening follow the owner, not the authored block.
    const monitorPrefix = constraint.monitors?.[0]?.split(".")[0];
    const owner = state.replay.monitor_owners?.[monitorPrefix];
    if (owner && owner !== motion) row.dataset.monitorOwner = owner;
    if (!constraint.window) row.dataset.idle = "true";
    line.textContent = constraint.line ? `L${constraint.line}` : "";
    name.textContent = constraint.name;
    expression.textContent = constraint.expression;
    row.append(line, name, expression);
    if (constraint.members?.length) {
      const members = document.createElement("button");
      members.className = "plot-members";
      members.textContent = `plot ${constraint.members.length} members`;
      members.title = "One plot per watched constraint: they are not the same quantity";
      members.onclick = (event) => {
        event.stopPropagation();
        row.dataset.plotted = "true";
        constraint.members.forEach((member) => addPlot(
          [member.error],
          `${constraint.motion ?? "shared"} / ${member.id}`,
          `watched by ${constraint.name}`,
          {
            row,
            // the member's error is judged against its own band, around satisfied
            constraint: {
              ...constraint,
              tolerance: member.tolerance ?? constraint.tolerance,
              setpoints: [{ label: "satisfied", value: 0 }],
            },
          },
        ));
      };
      row.append(members);
    }
    if (constraint.tracking.length || constraint.control.length || constraint.monitors.length) {
      row.dataset.plottable = "true";
      row.title = constraint.window
        ? `Plot this ${constraint.kind} constraint`
        : "This motion never ran in this recording";
      // measured against its setpoint where there is one; otherwise the error against zero
      const tracked = constraint.tracking.length ? constraint.tracking : constraint.error;
      const machinery = constraint.kind === "monitored"
        ? constraint.monitors
        : [...constraint.error, ...constraint.control];
      // What one history read has to cover for this row's cards to draw themselves.
      row.plotSignals = [...tracked, ...machinery];
      // `data` is a prefetched /api/plot answer shared with the other rows of the same motion;
      // without one each card fetches its own, which is what a lone human click wants.
      row.openPlots = (data = null) => {
        row.dataset.plotted = "true";
        const title = `${constraint.motion ?? "shared"} / ${constraint.name}`;
        const between = constraint.between.length ? ` · ${constraint.between.join(" vs ")}` : "";
        const where = constraint.line ? `L${constraint.line}: ` : "";
        const detail = `${where}${constraint.expression ?? constraint.name}${between}`;
        // A when-guard's monitor runs during the predecessor motion; say so on the card.
        const owner = row.dataset.monitorOwner;
        const monitorDetail = owner ? `${detail} · evaluated during ${owner}` : detail;
        if (state.following && livePlotsOn()) {
          // Live: ONE uPlot card per constraint -- its tracked signal and its controller
          // machinery share the frame axis; the monitor keeps its own card, value and
          // satisfied live on a scale of their own. The archive reload rebuilds the full
          // replay card set.
          const combined = [
            ...new Set([...tracked, ...(constraint.kind === "monitored" ? [] : machinery)]),
          ];
          if (combined.length) addLivePlot(combined, title, detail, { row, data });
          if (constraint.kind === "monitored" && constraint.monitors.length) {
            addLivePlot(constraint.monitors, `${title} · monitor`, monitorDetail, { row, data });
          }
          return;
        }
        const bands = constraint.tracking.length
          ? constraint
          : { ...constraint, setpoints: [{ label: "satisfied", value: 0 }] };
        if (tracked.length) addPlot(tracked, `${title} · constraint`, detail, { row, constraint: bands, data });
        if (constraint.kind === "monitored") {
          if (constraint.monitors.length) {
            addPlot(constraint.monitors, `${title} · monitor`, monitorDetail, { row, constraint, data });
          }
          return;
        }
        if (machinery.length) addPlot(machinery, `${title} · controller`, detail, { row, constraint, gains: constraint.gains, data });
      };
      row.onclick = () => {
        if (row.dataset.plotted) {
          $("#plots").querySelectorAll(`[data-row="${row.dataset.plotKey}"] .remove-plot`)
            .forEach((button) => button.click());
          return;
        }
        row.openPlots();
      };
    } else {
      row.dataset.unavailable = "true";
      row.title = constraint.evaluator
        ? "No recorded signal"
        : "Nothing evaluates this constraint in this generation";
    }
    return row;
    })];
  }));
  $("#constraint-search").oninput = (event) => filter(".constraint", event.target.value);
}

export function replayShell(path) {
  // The run page's markup, with what the reply fills left blank: the numbers, the timeline's
  // range and the constraint list.
  return `<div class="replay"><div class="replay-heading"><button id="back" title="Back to generation">←</button><h1>${path.split("/").pop()}</h1><div class="replay-tabs"><button data-panel="plots" class="active">Plots</button><button data-panel="reports">Reports</button><button data-panel="views">Views</button><button data-panel="compare">Compare</button><button data-panel="explore">Explore</button><button data-panel="console">Console</button></div><span class="eyebrow">RUN</span></div><section id="panel-plots"><div class="constraint-panel"><div class="eyebrow">SOURCE CONSTRAINTS</div><input id="constraint-search" type="search" placeholder="Search .robmot constraints"><div id="constraints" class="constraints"></div></div><div class="chart-controls"><button id="plot">Add empty plot</button><button id="notebook">Open in Jupyter</button><button id="live-plots" title="Plot signals as the run writes them">live plots: on</button></div><div id="plots" class="plots"></div></section><section id="panel-reports" hidden></section><section id="panel-views" hidden></section><section id="panel-compare" hidden>${COMPARE_MARKUP}</section><section id="panel-explore" hidden>${EXPLORE_MARKUP}</section><section id="panel-console" hidden><pre id="console-text" class="console"></pre></section></div><div class="settling" hidden><div class="spinner"></div><span>archiving the run…</span></div><div class="videos" hidden><button class="video-max" title="Expand"></button><button class="video-min" title="Minimize"></button><div class="video-main"><video preload="auto" playsinline disablepictureinpicture controlslist="nodownload noplaybackrate noremoteplayback"></video><span class="video-name"></span></div><div class="video-strip"></div></div><div class="transport"><div class="transport-controls"><button id="step-back" title="Previous frame">‹</button><button id="play">Play</button><button id="step-forward" title="Next frame">›</button><details class="picker speed-menu"><summary>1×</summary><div class="picker-panel"><button data-value="0.25">0.25×</button><button data-value="0.5">0.5×</button><button data-value="1" aria-pressed="true">1×</button><button data-value="2">2×</button><button data-value="5">5×</button></div></details><span id="readout" class="path"></span><span class="marker-legend"><i class="lg lg-state"></i>state <i class="lg lg-event"></i>event <i class="lg lg-satisfied"></i>satisfied <i class="lg lg-unsatisfied"></i>lost <i class="lg lg-monitor"></i>monitor</span><button id="cancel-run" title="End the run" hidden>cancel</button></div><div class="markers"></div><input class="timeline" type="range" min="0" max="0" value="0" disabled></div>`;
}

export function bindPanels() {
  const show = (panel, remember = true) => {
    document.querySelectorAll(".replay-tabs button").forEach((button) =>
      button.classList.toggle("active", button.dataset.panel === panel));
    $("#panel-plots").hidden = panel !== "plots";
    $("#panel-reports").hidden = panel !== "reports";
    $("#panel-views").hidden = panel !== "views";
    $("#panel-compare").hidden = panel !== "compare";
    $("#panel-explore").hidden = panel !== "explore";
    $("#panel-console").hidden = panel !== "console";
    // The sweep is paid for when the tab is opened, and once per run.
    if (panel === "reports" && state.runPath) showReports(state.runPath);
    // A panel that cannot load is not the page failing to load.
    if (panel === "views") loadViews().catch((error) => snack(error.message));
    if (panel === "compare") loadCompare().catch((error) => snack(error.message));
    state.charts.forEach((chart) => chart.resize());
    if (!remember) return;
    const view = new URLSearchParams(location.hash.slice(1));
    view.set("panel", panel);
    history.replaceState(null, "", `#${view}`);
  };
  document.querySelectorAll(".replay-tabs button").forEach((button) => {
    button.onclick = () => show(button.dataset.panel);
  });
  // a reload lands back where it left off, like the run it reopens
  show(new URLSearchParams(location.hash.slice(1)).get("panel") ?? "plots", false);
}

// One transport for a recording or the live run writing it.
export function bindTransport() {
  $(".timeline").oninput = (event) => seek(Number(event.target.value));
  $("#step-back").onclick = () => seek(state.frame - 1);
  $("#step-forward").onclick = () =>
    state.live ? simControl({ action: "step" }) : seek(state.frame + 1);
  $("#play").onclick = () =>
    state.live ? simControl({ action: state.live.paused ? "resume" : "pause" }) : togglePlayback();
  $("#cancel-run").onclick = () => simControl({ action: "cancel" });
  $$(".speed-menu .picker-panel button").forEach((option) => {
    option.onclick = () => {
      $(".speed-menu").open = false;
      setSpeed(Number(option.dataset.value));
    };
  });
  showSpeed(state.speed);
  renderMarkers();
}

// Replay speed is the page's; a live run's is the loop's.
export function setSpeed(speed) {
  state.speed = speed;
  showSpeed(speed);
  const video = playingVideo();
  if (video) video.playbackRate = speed;
  if (state.live) return simControl({ action: "speed", speed });
  if (state.timer) {   // restart the animation on the new rate
    stopPlayback();
    togglePlayback();
  }
}

export function showSpeed(speed) {
  $$(".speed-menu .picker-panel button").forEach((option) =>
    option.setAttribute("aria-pressed", Number(option.dataset.value) === speed));
  const summary = $(".speed-menu summary");
  if (summary) summary.textContent = `${speed}×`;
}

// A growing log is followed; only a loop that answers can be paused.
export function setTransportMode() {
  const following = Boolean(state.following);
  const driving = Boolean(state.live);
  $("#play").textContent = driving ? (state.live.paused ? "Play" : "Pause") : "Play";
  $("#play").title = driving ? "Pause the simulation" : "Play the recording";
  $("#step-back").disabled = driving;
  $("#step-forward").disabled = driving && !state.live.paused;
  $("#step-forward").title = driving ? "Advance one tick" : "Next frame";
  $(".timeline").disabled = driving || !state.replay.frames;
  $("#cancel-run").hidden = !driving;
  $(".transport").classList.toggle("transport-live", following);
}

// A constraint that chatters would bury the state markers it happened under.
const SATISFIED_MARKER_CAP = 12;

export function renderMarkers() {
  // A satisfied bit can rise and fall a hundred times under one state: keep the first few per
  // slot, and draw them behind the state and event markers rather than over them.
  const seen = new Map();
  const kept = state.replay.events.filter((event) => {
    if (event.kind !== "satisfied" && event.kind !== "unsatisfied") return true;
    const count = (seen.get(event.label) ?? 0) + 1;
    seen.set(event.label, count);
    return count <= SATISFIED_MARKER_CAP;
  });
  const weight = (event) => (event.kind === "state" || event.kind === "event" ? 1 : 0);
  $(".markers").replaceChildren(...kept.sort((a, b) => weight(a) - weight(b)).map((event) => {
    const marker = document.createElement("button");
    marker.className = `marker marker-${event.kind}`;
    marker.style.left = trackLeft(event.frame);
    marker.title = `${event.label} @ frame ${event.frame}`;
    marker.onclick = () => seek(event.frame);
    return marker;
  }));
  const playhead = document.createElement("div");
  playhead.className = "playhead";
  $(".markers").append(playhead);
  movePlayhead();
}

export function trackFraction(frame) {
  return frame / Math.max(1, state.replay.frames - 1);
}

export function trackLeft(frame) {
  // the thumb travels inset by half its width, so markers must follow the same geometry
  return `calc(var(--thumb) / 2 + ${trackFraction(frame)} * (100% - var(--thumb)))`;
}

export function movePlayhead() {
  $(".transport").style.setProperty("--f", trackFraction(state.frame));
}

export function updateCursor() {
  // A cursor pinned to the live edge draws nothing worth a setOption on every chart; it comes
  // back by itself when following ends and the finished run is on screen.
  if (state.following) return;
  state.charts.forEach((chart) => chart.setOption(cursorOption(state.frame, chart.cursorIndex ?? 0)));
}

export function filter(selector, value) {
  const query = value.toLowerCase();
  document.querySelectorAll(selector).forEach((item) => {
    item.hidden = !item.textContent.toLowerCase().includes(query);
  });
}

export function seek(frame) {
  state.frame = Math.max(0, Math.min(state.replay.frames - 1, frame));
  $(".timeline").value = state.frame;
  updateReadout();
  updateCursor();
  movePlayhead();
  syncVideo();
}

// The recording of the run, if this run has one on screen.
export function playingVideo() {
  const panel = $(".videos");
  const video = panel && !panel.hidden ? panel.querySelector("video") : null;
  return video && video.duration && isFinite(video.duration) ? video : null;
}

// Frames and video are two clocks over the same run: map by position, not by seconds, and
// leave a deadband so the video playing itself does not fight the cursor it is driving.
export function syncVideo(force = false) {
  const video = playingVideo();
  if (!video) return;
  const target = trackFraction(state.frame) * video.duration;
  if (!force && Math.abs(video.currentTime - target) <= 0.25) return;
  // Scrubbing fires seeks faster than the decoder lands them, and each new one aborts the
  // last mid-decode: the picture stutters. Land one seek at a time; a drag only queues its
  // newest target, applied by the seeked handler when the decoder is free.
  if (video.seeking) video._queuedSeek = target;
  else video.currentTime = target;
}

export function togglePlayback() {
  if (state.timer) return stopPlayback();
  const fps = state.speed * state.replay.frames / Math.max(state.replay.duration, 1e-9);
  let cursor = state.frame >= state.replay.frames - 1 ? 0 : state.frame;
  let last = performance.now();
  const video = playingVideo();
  if (video) {
    video.playbackRate = state.speed;
    if (cursor === 0) video.currentTime = 0;
    video.play().catch(() => {});
  }
  const tick = (now) => {
    // With a recording the video is the clock, so the cursor follows what is on screen.
    if (video && !video.paused) cursor = (video.currentTime / video.duration) * (state.replay.frames - 1);
    else cursor += (now - last) * fps / 1000;
    last = now;
    if (cursor >= state.replay.frames - 1) {
      seek(state.replay.frames - 1);
      return stopPlayback();
    }
    seek(Math.round(cursor));
    state.timer = requestAnimationFrame(tick);
  };
  state.timer = requestAnimationFrame(tick);
  $("#play").textContent = "Pause";
}

export function stopPlayback() {
  playingVideo()?.pause();
  if (!state.timer) return;
  cancelAnimationFrame(state.timer);
  state.timer = null;
  if ($("#play")) $("#play").textContent = "Play";
}

export function updateReadout() {
  const duration = state.replay.duration;
  const time = duration * state.frame / Math.max(1, state.replay.frames - 1);
  const frames = `${state.frame.toLocaleString()} / ${state.replay.frames.toLocaleString()}`;
  $("#readout").textContent =
    `${time.toFixed(3)} / ${duration.toFixed(3)} s · frame ${frames}${state.following ? " · live" : ""}`;
}
