// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * One run's page: its constraints, its transport, its recording and its console --
 * whether the run is finished or still being written.
 */

import { appendConsole, consoleExcerpt, showEmpty } from "./components.js";
import { annotationEditor } from "./annotations.js";
import { bindInspection, restoreInspection } from "./inspection.js";
import { $, $$, api, askConfirm, copyText, formatBytes, post, seconds, snack, stampText, state } from "./core.js";
import { EXPLORE_MARKUP, bindExplore } from "./explore.js";
import { highlightGeneration, loadGenerations, readRunOptions, selectGeneration, stopRun } from "./generations.js";
import { addLivePlot, bindLivePlots, followLiveRun, livePlotsOn, simControl, trackActiveMotion } from "./live.js";
import { openNotebook } from "./notebook.js";
import { addPlot, cursorOption, progressiveOn, seriesUpTo, setProgressive } from "./plots.js";
import { showReports } from "./reports.js";
import { setView } from "./routing.js";
import { showGenerated } from "./sources.js";

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
  state.notes = [];
  const timeline = $(".timeline");
  timeline.max = Math.max(0, state.replay.frames - 1);
  timeline.disabled = false;

  // gate id -> the name the source spells it: a row carries the name, not the gate it ran under
  state.motionOf = state.replay.motion_names ?? {};
  state.motionByIri = state.replay.motion_iri_names ?? {};
  // slot id -> the motion and constraint it serves: an edge in the log names the slot.
  state.slotOwner = Object.fromEntries(Object.values(state.replay.signal_index ?? {})
    .filter((where) => where.slot).map((where) => [where.slot, where]));
  populateConstraints();
  // Edges draw for the constraints whose charts are open, so the strip follows the cards.
  state.plottedWatch?.disconnect();
  state.plottedWatch = new MutationObserver(() => renderMarkers());
  state.plottedWatch.observe($("#constraints"), { attributes: true, subtree: true, attributeFilter: ["data-plotted"] });
  // The archive of a run this page started: reopen the cards that were open when it ended --
  // or, when none were (a run faster than the page), the last motion that ran. Never the
  // whole run's card set: eighty charts in one page is what made live pages crawl.
  const reopened = state.autoPlot === path && !state.replay.pending && livePlotsOn();
  if (reopened) {
    state.autoPlot = null;
    const keys = state.reopenRows ?? [];
    const lastMotion = state.reopenMotion;
    state.reopenRows = state.reopenMotion = null;
    openPlotsFor(path, keys, lastMotion);
  }
  $("#plot").onclick = () => addPlot([]);
  state.motionSpans = motionSpans();
  state.opening = false;
  bindAutoPlot();
  // The reopened set is what this page had open when the run ended; following would close it
  // on the spot to show frame 0's motion instead.
  if (!reopened) followMotion();
  $("#notebook").onclick = () => openNotebook(state.runPath).catch((error) => snack(error.message));
  bindLivePlots();
  bindProgressive();
  bindPanels();
  // A side panel that cannot load is not the page failing to load.
  bindExplore(path).catch((error) => snack(error.message));
  bindTransport();
  // Detached on purpose: the strip is already drawn from the frame log, and the graph these
  // bars come from must never be something the page waits on. A run that has not started has
  // no spans to draw yet either; settle() re-reads them the moment the hold lifts.
  if (!state.replay.pending) loadTimelineOverlay(path).catch(() => {});
  setTransportMode();
  showVideos(path, state.replay.videos ?? []);
  // Nothing recorded: a real platform has a camera, but only ROS to reach it through.
  if (!state.replay.videos?.length) showRosCamera(state.replay.generation);
  followLiveRun(path);
  bindConsoleSearch();
  followConsole(path);
  // Built from the generation's contract before the runtime wrote anything: hold the page
  // until the log begins.
  if (state.replay.pending) settle(true, "waiting for the run to start…");
  $("#back").onclick = () => selectGeneration(state.replay.generation);
  bindRunAgain(path);
  updateReadout();
  annotationEditor(path, (data) => {
    state.generation = null;
    if (state.runPath === path) {
      $(".replay-heading h1").textContent = data.label || path.split("/").pop();
      $("#panel-reports").dataset.run = "";
    }
    return loadGenerations(true);
  }).then((editor) => {
    if (state.runPath !== path) return;
    $(".replay-heading")?.after(editor);
    $(".replay-heading h1").textContent = editor.elements.label.value || path.split("/").pop();
    $(".replay-heading h1").title = path;
  }).catch((error) => snack(error.message));
  api(`/api/notes?path=${encodeURIComponent(path)}`).then((data) => {
    if (state.runPath === path) { state.notes = data.notes; renderMarkers(); }
  }).catch((error) => snack(error.message));
  bindInspection(path);
  if (!state.replay.pending && !state.following) restoreInspection(path);
}

// The same start the generation page makes, from the run it is being compared against. The
// choices are the run bar's remembered ones; the generation page is where they are changed.
function bindRunAgain(runPath) {
  const button = $("#run-again");
  const generation = state.replay.generation;
  button.hidden = Boolean(state.replay.pending);
  button.onclick = async () => {
    button.disabled = true;
    try {
      const started = await post("/api/run", { path: generation, options: readRunOptions(generation) });
      await openPendingRun(started.run);
    } catch (error) {
      button.disabled = false;
      snack(error.message);
    }
  };
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
  // The live poll settles the page after an await, by which time the reader may have left the
  // run page entirely -- cancel a run and go back and there is no overlay left to settle.
  const overlay = $(".settling");
  if (!overlay) return;
  const waited = !overlay.hidden;
  overlay.hidden = !waiting;
  overlay.querySelector("span").textContent = message;
  $(".replay")?.classList.toggle("busy", waiting);
  $(".transport")?.classList.toggle("busy", waiting);
  // What the bars were read from changes when the wait ends -- a run that had no graph now has
  // one, and a projection is replaced by the archive. Re-read there, never on a timer.
  if (waited && !waiting && state.runPath) loadTimelineOverlay(state.runPath, true).catch(() => {});
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
  bindVideoMinimize(panel);
  const show = (camera) => {
    state.camera = camera;
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

// Out of the way of the page under it, whether the pane is showing a recording or a live
// hardware camera -- both float over the same run page and cover the same panels.
function bindVideoMinimize(panel) {
  const minimize = panel.querySelector(".video-min");
  const setMinimized = (small) => {
    panel.classList.toggle("minimized", small);
    minimize.textContent = small ? "□" : "–";
    minimize.title = small ? "Show the camera" : "Minimize";
    try { localStorage.setItem("motion-spec.video-minimized", String(small)); } catch { /* private */ }
    reserveVideoSpace();
  };
  let small = false;
  try { small = localStorage.getItem("motion-spec.video-minimized") === "true"; } catch { /* private */ }
  minimize.onclick = () => setMinimized(!panel.classList.contains("minimized"));
  setMinimized(small);
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

// The same pane, pointed at a live ROS topic: this run has no recording to replay, so the
// picture is whatever the camera is publishing now -- not a track the timeline drives. That is
// every hardware run, and a simulated one that was not started with --record.
async function showRosCamera(generationPath) {
  if (!generationPath) return;
  // The cached generation only answers here if it carries the cameras: the side panel stores a
  // lighter record for the list, and reading `cameras` off that one silently finds none.
  const cached = state.generation?.path === generationPath && state.generation.cameras
    ? state.generation
    : null;
  const generation = cached
    ?? await api(`/api/generation?path=${encodeURIComponent(generationPath)}`).catch(() => null);
  // Nothing is re-read off `state` after the await: a run starting rewrites `state.replay` in
  // place, and checking it here is a race the pane used to lose silently.
  if (!generation) return;
  // Where the camera is published is the model's to state -- a subscription naming it -- and
  // never this page's to guess from its name. A camera no channel carries has no live view.
  // Every camera the model gives a channel, in the order the scene declares them. The picker
  // offers these and nothing else: a topic the model does not state is not this page's to invent.
  const carried = generation.cameras?.filter((entry) => entry.topic) ?? [];
  if (!carried.length) return;
  const declaredTopic = carried[0].topic;
  const panel = $(".videos");
  panel.hidden = false;
  panel.classList.add("one-camera", "ros-camera");
  panel.querySelector(".video-strip").hidden = true;
  bindVideoExpand(panel);
  bindVideoMinimize(panel);
  const main = panel.querySelector(".video-main");
  const options = carried
    .map((entry) => `<option value="${entry.topic}">${entry.id} — ${entry.topic}</option>`)
    .join("");
  main.innerHTML = `<img class="ros-frame" alt=""><span class="video-name"></span><div class="ros-topic"><select title="ROS image topic">${options}</select><span class="ros-status"></span></div>`;
  const frame = main.querySelector(".ros-frame");
  const topic = main.querySelector("select");
  const status = main.querySelector(".ros-status");
  const name = main.querySelector(".video-name");
  // The topic picked last is remembered for the session, unless this run does not carry it.
  const remembered = carried.some((entry) => entry.topic === state.rosTopic);
  topic.value = remembered ? state.rosTopic : declaredTopic;
  const subscribe = () => {
    state.rosTopic = topic.value;
    status.textContent = "";
    name.textContent = carried.find((entry) => entry.topic === state.rosTopic)?.id ?? "";
    // A new query each time, so the browser reconnects rather than showing the stalled stream.
    frame.src = `/api/ros-camera?topic=${encodeURIComponent(state.rosTopic)}&t=${Date.now()}`;
  };
  frame.onerror = () => {
    frame.removeAttribute("src");
    // An armed run publishes nothing until it is played, so this is a wait, not a dead end:
    // retry until frames start, rather than leaving the pane blank until the topic is reselected.
    status.textContent = "waiting for frames…";
    reserveVideoSpace();
    clearTimeout(state.rosRetry);
    state.rosRetry = setTimeout(() => {
      if (frame.isConnected) subscribe();
    }, 2000);
  };
  frame.onload = () => {
    status.textContent = "";
    reserveVideoSpace();
  };
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
    // What just arrived is part of the log the search bar is searching.
    findInConsole();
  };
  poll();
  state.consoleWatch = setInterval(poll, 1000);
}

// The matches the console's search bar is standing on, and which one ↑/↓ last stepped to.
let consoleHits = [];
let consoleHit = 0;

// Find in the log the way a terminal does. The browser paints the matches from its highlight
// registry rather than from elements wrapped around them: the log is appended to while it is
// read, and rewriting its nodes would fight both that and the ANSI spans already in it.
function findInConsole(step = 0) {
  const pre = $("#console-text");
  const input = $("#console-search");
  if (!pre || !input || !CSS.highlights) return;
  const needle = input.value.toLowerCase();
  consoleHits = [];
  if (needle) {
    const walk = document.createTreeWalker(pre, NodeFilter.SHOW_TEXT);
    for (let node = walk.nextNode(); node; node = walk.nextNode()) {
      const text = node.data.toLowerCase();
      // A word broken across an ANSI colour change is two text nodes and is not matched.
      for (let at = text.indexOf(needle); at !== -1; at = text.indexOf(needle, at + needle.length)) {
        const range = new Range();
        range.setStart(node, at);
        range.setEnd(node, at + needle.length);
        consoleHits.push(range);
      }
    }
  }
  consoleHit = consoleHits.length
    ? (consoleHit + step + consoleHits.length * 2) % consoleHits.length
    : 0;
  CSS.highlights.set("console-match", new Highlight(...consoleHits));
  CSS.highlights.set("console-match-current", new Highlight(...consoleHits.slice(consoleHit, consoleHit + 1)));
  $("#console-matches").textContent = needle
    ? (consoleHits.length ? `${consoleHit + 1} / ${consoleHits.length}` : "no matches")
    : "";
  $("#console-prev").disabled = $("#console-next").disabled = consoleHits.length < 2;
  // Only a step is asked to move the view: re-searching as the run writes must not yank it.
  if (step && consoleHits.length) {
    const hit = consoleHits[consoleHit].getBoundingClientRect();
    pre.scrollTop += hit.top - pre.getBoundingClientRect().top - pre.clientHeight / 2;
  }
}

export function bindConsoleSearch() {
  const input = $("#console-search");
  // Nothing to paint matches with: the field would take words and show nothing for them.
  if (!CSS.highlights) return input.closest(".console-bar").remove();
  input.oninput = () => { consoleHit = 0; findInConsole(); };
  input.onkeydown = (event) => {
    if (event.key === "Escape") { input.value = ""; consoleHit = 0; return findInConsole(); }
    if (event.key !== "Enter") return;
    findInConsole(event.shiftKey ? -1 : 1);
  };
  $("#console-prev").onclick = () => findInConsole(-1);
  $("#console-next").onclick = () => findInConsole(1);
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
  return `<div class="replay"><div class="replay-heading"><button id="back" title="Back to generation">←</button><h1>${path.split("/").pop()}</h1><button id="run-again" title="Run this generation again with the run bar's last choices">↻ run again</button><div class="replay-tabs"><button data-panel="plots" class="active">Plots</button><button data-panel="reports">Reports</button><button data-panel="explore">Explore</button><button data-panel="console">Console</button><button data-panel="files">Files</button><button data-panel="notes">Notes</button></div><span class="eyebrow">RUN</span></div><section id="panel-plots"><div class="constraint-panel"><div class="eyebrow">SOURCE CONSTRAINTS</div><input id="constraint-search" type="search" placeholder="Search .robmot constraints"><div id="constraints" class="constraints"></div></div><div class="chart-controls"><button id="auto-plot" title="Open each motion's plots as the cursor enters it, the way a live run does">auto plot: off</button><button id="plot">Add empty plot</button><button id="notebook">Open in Jupyter</button><button id="live-plots" title="Plot signals as the run writes them">live plots: on</button><button id="progressive-plots" title="Draw a replayed run the way a live one arrives: nothing past the cursor">progressive: off</button></div><div id="plots" class="plots"></div></section><section id="panel-reports" hidden></section><section id="panel-explore" hidden>${EXPLORE_MARKUP}</section><section id="panel-console" hidden><div class="console-bar"><input id="console-search" type="search" spellcheck="false" placeholder="Search the console"><span id="console-matches"></span><button id="console-prev" title="Previous match (Shift+Enter)" disabled>↑</button><button id="console-next" title="Next match (Enter)" disabled>↓</button></div><pre id="console-text" class="console"></pre></section><section id="panel-files" hidden><div class="generated-files run-files"></div></section><section id="panel-notes" hidden><div class="notes-compose"><textarea class="note-text" rows="3" placeholder="What happened, what to try next — Ctrl+Enter adds"></textarea><div class="notes-compose-bar"><input class="note-tags" placeholder="tags, comma separated" spellcheck="false"><button class="note-add">add note</button><span class="notes-state"></span></div></div><div class="notes-bar" hidden><input type="checkbox" class="pick notes-pick-all" title="Select every note"><span class="notes-picked"></span><button class="note-action notes-delete" disabled>delete selected</button></div><div class="notes-list"></div></section></div><div class="settling" hidden><div class="spinner"></div><span>archiving the run…</span></div><div class="videos" hidden><button class="video-max" title="Expand"></button><button class="video-min" title="Minimize"></button><div class="video-main"><video preload="auto" playsinline disablepictureinpicture controlslist="nodownload noplaybackrate noremoteplayback"></video><span class="video-name"></span></div><div class="video-strip"></div></div><div class="transport"><div class="transport-controls"><button id="step-back" title="Previous frame">‹</button><button id="play">Play</button><button id="step-forward" title="Next frame">›</button><details class="picker speed-menu"><summary>1×</summary><div class="picker-panel"><button data-value="0.25">0.25×</button><button data-value="0.5">0.5×</button><button data-value="1" aria-pressed="true">1×</button><button data-value="2">2×</button><button data-value="5">5×</button></div></details><span id="readout" class="path"></span><span class="marker-legend"><span class="lg-item"><i class="lg lg-state"></i>state</span><span class="lg-item"><i class="lg lg-state lg-transition"></i>on event</span><span class="lg-item lg-event-item" hidden><i class="lg lg-event"></i>event</span><span class="lg-item lg-edges" hidden><i class="lg lg-satisfied"></i>satisfied</span><span class="lg-item lg-edges" hidden><i class="lg lg-unsatisfied"></i>lost</span><span class="lg-item"><i class="lg lg-monitor"></i>monitor</span><span class="lg-item lg-wait-item" hidden><i class="lg lg-wait"></i>wait</span><span class="lg-item lg-slow-item" hidden><i class="lg lg-wait lg-slow"></i>slow</span></span><button id="cancel-run" title="End the run" hidden>cancel</button></div><div class="markers"></div><input class="timeline" type="range" min="0" max="0" value="0" disabled></div>`;
}

export function bindPanels() {
  const show = (panel, remember = true) => {
    document.querySelectorAll(".replay-tabs button").forEach((button) =>
      button.classList.toggle("active", button.dataset.panel === panel));
    $("#panel-plots").hidden = panel !== "plots";
    $("#panel-reports").hidden = panel !== "reports";
    $("#panel-explore").hidden = panel !== "explore";
    $("#panel-console").hidden = panel !== "console";
    $("#panel-files").hidden = panel !== "files";
    $("#panel-notes").hidden = panel !== "notes";
    // A panel that cannot load is not the page failing to load.
    if (panel === "reports") showReports(state.runPath).catch((error) => snack(error.message));
    if (panel === "files") showRunFiles(state.runPath).catch((error) => snack(error.message));
    if (panel === "notes") showNotes(state.runPath).catch((error) => snack(error.message));
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
  // a hash naming a panel this page no longer offers falls back to plots
  const stored = new URLSearchParams(location.hash.slice(1)).get("panel");
  show(["plots", "reports", "explore", "console", "files", "notes"].includes(stored) ? stored : "plots", false);
}

// What the run wrote, read on each opening of the tab: a live run is still adding to it.
// Folders fold the way the generation page's do; the run's top-level files stand alone.
export async function showRunFiles(runPath) {
  const files = await api(`/api/run/files?path=${encodeURIComponent(runPath)}`);
  const list = $(".run-files");
  if (!list) return;
  const entry = (file) => {
    const row = document.createElement("div");
    row.className = "source-file";
    row.title = `${file.path} — click to read`;
    row.innerHTML = '<span></span><span class="file-size"></span><button class="copy-path" title="Copy full path">⧉</button>';
    row.firstChild.textContent = file.name;
    row.querySelector(".file-size").textContent = formatBytes(file.size);
    row.onclick = () => showGenerated(`${runPath}/${file.full}`).catch((error) => snack(error.message));
    const copy = row.lastChild;
    copy.onclick = async (event) => {
      event.stopPropagation();
      await copyText(file.path);
      copy.textContent = "✓";
      setTimeout(() => { copy.textContent = "⧉"; }, 900);
    };
    return row;
  };
  const folders = new Map();
  const loose = [];
  files.forEach((file) => {
    const [folder, ...rest] = file.name.split("/");
    if (!rest.length) return loose.push({ ...file, full: file.name });
    folders.set(folder, [...(folders.get(folder) ?? []), { ...file, full: file.name, name: rest.join("/") }]);
  });
  list.replaceChildren(...loose.map(entry), ...[...folders].map(([folder, inside]) => {
    const group = document.createElement("details");
    group.className = "generated-folder";
    group.open = true;
    const summary = document.createElement("summary");
    summary.textContent = `${folder} · ${inside.length}`;
    group.append(summary, ...inside.map(entry));
    return group;
  }));
}

// Every note kept beside the run, newest first, with a place to add the next one. The whole
// list is what is saved: notes are few and small, and one write is one thing to get right.
export async function showNotes(runPath, container = null) {
  const panel = container ?? $("#panel-notes");
  if (!panel) return;
  const list = panel.querySelector(".notes-list");
  const text = panel.querySelector(".note-text");
  const tags = panel.querySelector(".note-tags");
  const status = panel.querySelector(".notes-state");
  const splitTags = (value) => value.split(",").map((tag) => tag.trim()).filter(Boolean);
  let notes = (await api(`/api/notes?path=${encodeURIComponent(runPath)}`)).notes;
  let frameInput = null;
  let endInput = null;
  if (!container) {
    panel.querySelector(".note-position")?.remove();
    const position = document.createElement("div");
    position.className = "note-position";
    position.innerHTML = '<button type="button">Attach current frame</button><label>Frame<input type="number" min="0" step="1" placeholder="General note"></label><label>End frame (optional)<input type="number" min="0" step="1"></label>';
    [frameInput, endInput] = position.querySelectorAll("input");
    frameInput.max = endInput.max = Math.max(0, state.replay.frames - 1);
    position.querySelector("button").onclick = () => { frameInput.value = state.frame; };
    panel.querySelector(".notes-compose").append(position);
  }
  // Picked by id, so a selection survives a redraw and is dropped only with the note it names.
  const picked = new Set();
  const bar = panel.querySelector(".notes-bar");
  const pickAll = bar.querySelector(".notes-pick-all");
  const syncBar = () => {
    bar.hidden = !notes.length;
    bar.querySelector(".notes-picked").textContent = picked.size ? `${picked.size} selected` : "";
    bar.querySelector(".notes-delete").disabled = !picked.size;
    pickAll.checked = picked.size > 0 && picked.size === notes.length;
    pickAll.indeterminate = picked.size > 0 && picked.size < notes.length;
  };
  pickAll.onchange = () => {
    picked.clear();
    if (pickAll.checked) notes.forEach((note) => picked.add(note.id));
    render();
  };
  bar.querySelector(".notes-delete").onclick = async () => {
    const count = picked.size;
    if (!(await askConfirm({ message: `Delete ${count} note${count === 1 ? "" : "s"}?`, confirmLabel: "Delete" }))) return;
    save(notes.filter((note) => !picked.has(note.id)));
  };
  const save = async (next) => {
    status.textContent = "saving…";
    try {
      notes = (await post("/api/notes", { path: runPath, notes: next })).notes;
      state.cache.generations = null;
      if (!container && state.runPath === runPath) { state.notes = notes; renderMarkers(); }
      status.textContent = "";
      render();
    } catch (error) {
      status.textContent = `not saved: ${error.message}`;
    }
  };
  const card = (note) => {
    const box = document.createElement("article");
    box.className = "note";
    const head = document.createElement("div");
    head.className = "note-head";
    const pick = Object.assign(document.createElement("input"), { type: "checkbox", className: "pick", checked: picked.has(note.id), title: "Select" });
    pick.onchange = () => { pick.checked ? picked.add(note.id) : picked.delete(note.id); syncBar(); };
    head.append(pick, Object.assign(document.createElement("span"), { className: "note-when", textContent: stampText(note.created) }));
    head.append(...note.tags.map((tag) => Object.assign(document.createElement("span"), { className: "run-tag", textContent: tag })));
    if (!container && note.frame !== null && note.frame !== undefined) {
      const jump = document.createElement("button");
      jump.className = "note-action";
      jump.textContent = `Frame ${note.frame}${note.end_frame == null ? "" : `–${note.end_frame}`}`;
      jump.onclick = () => jumpToFinding(note.frame, null, null, note.end_frame);
      head.append(jump);
    }
    const edit = Object.assign(document.createElement("button"), { className: "note-action", textContent: "edit" });
    const remove = Object.assign(document.createElement("button"), { className: "note-action", textContent: "delete" });
    head.append(edit, remove);
    const body = Object.assign(document.createElement("p"), { className: "note-body", textContent: note.text });
    box.append(head, body);
    remove.onclick = async () => {
      if (!(await askConfirm({ message: "Delete this note?", confirmLabel: "Delete" }))) return;
      save(notes.filter((other) => other.id !== note.id));
    };
    edit.onclick = () => {
      const editor = Object.assign(document.createElement("textarea"), { className: "note-text", rows: 3, value: note.text });
      const editTags = Object.assign(document.createElement("input"), { className: "note-tags", value: note.tags.join(", "), spellcheck: false });
      const keep = Object.assign(document.createElement("button"), { className: "note-action", textContent: "save" });
      const drop = Object.assign(document.createElement("button"), { className: "note-action", textContent: "cancel" });
      const bar = Object.assign(document.createElement("div"), { className: "notes-compose-bar" });
      bar.append(editTags, keep, drop);
      body.replaceWith(editor, bar);
      editor.focus();
      keep.onclick = () => save(notes.map((other) => (other.id === note.id
        ? { ...other, text: editor.value, tags: splitTags(editTags.value) }
        : other)));
      drop.onclick = render;
    };
    return box;
  };
  const render = () => {
    const ordered = [...notes].sort((a, b) => (a.created < b.created ? 1 : -1));
    picked.forEach((id) => { if (!notes.some((note) => note.id === id)) picked.delete(id); });
    list.replaceChildren(...ordered.map(card));
    syncBar();
    if (!ordered.length) list.append(Object.assign(document.createElement("p"), { className: "path notes-empty", textContent: "No notes yet." }));
  };
  const add = () => {
    if (!text.value.trim()) return;
    const frame = frameInput?.value ? Number(frameInput.value) : null;
    const end = endInput?.value ? Number(endInput.value) : null;
    if (frameInput && (!frameInput.checkValidity() || !endInput.checkValidity()
      || end !== null && (frame === null || end < frame))) {
      status.textContent = "Choose a valid frame or range within this recording.";
      return;
    }
    save([...notes, { text: text.value, tags: splitTags(tags.value), frame, end_frame: end }]).then(() => {
      if (!status.textContent) { text.value = ""; tags.value = ""; }
    });
  };
  panel.querySelector(".note-add").onclick = add;
  text.onkeydown = (event) => { if ((event.ctrlKey || event.metaKey) && event.key === "Enter") add(); };
  render();
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
  // Starting another run while this one drives the loop would fight it for the platform.
  const again = $("#run-again");
  if (again) again.hidden = driving;
  $(".transport").classList.toggle("transport-live", following);
  // "live plots" governs a run being written; on a recording it is a switch for nothing.
  // "auto plot" is its counterpart there, and the live poll follows motions on its own.
  const live = following || Boolean(state.replay.pending);
  $("#live-plots").hidden = !live;
  $("#auto-plot").hidden = live;
  $("#progressive-plots").hidden = live;
}

// Marks of one kind closer than this many pixels are drawn as one.
const CLUSTER_PX = 4;

export function renderMarkers() {
  // An edge belongs to the constraint whose chart is open: with no chart it is texture, with
  // one it says when the line on that chart was inside its band. The log names the slot; the
  // header's signal index says which constraint the slot served.
  const plotted = new Set($$("#constraints .constraint[data-plotted]")
    .map((row) => `${row.dataset.motion}/${row.querySelector("strong").textContent}`));
  const edge = (event) => event.kind === "satisfied" || event.kind === "unsatisfied";
  // The event that moved the FSM is the transition: one mark, outlined, naming the event in
  // its tooltip. Only an event that moved nothing stands on its own.
  const causes = new Map();
  state.replay.events.forEach((event) => { if (event.kind === "state") causes.set(event.frame, []); });
  const kept = state.replay.events.filter((event) => {
    if (event.kind === "event") {
      const entry = causes.get(event.frame) ?? causes.get(event.frame + 1);
      if (entry) entry.push(event.label);
      return !entry;
    }
    if (!edge(event)) return true;
    const where = state.slotOwner[event.label];
    return Boolean(where) && plotted.has(`${where.motion}/${where.constraint}`);
  });
  // A pixel is what can be told apart: marks of one kind closer than a few of them are one mark
  // with several names, not several marks on top of each other.
  const width = $(".markers").clientWidth || 1000;
  const px = (frame) => trackFraction(frame) * width;
  const clusters = [];
  const last = {};
  kept.sort((a, b) => a.frame - b.frame || a.kind.localeCompare(b.kind)).forEach((event) => {
    const label = edge(event) ? state.slotOwner[event.label].constraint : event.label;
    const open = last[event.kind];
    if (open && px(event.frame) - px(open.frame) < CLUSTER_PX) {
      open.labels.push(label);
      open.until = event.frame;
      return;
    }
    last[event.kind] = { kind: event.kind, frame: event.frame, until: event.frame, labels: [label] };
    clusters.push(last[event.kind]);
  });
  // The legend names what is drawn, nothing more.
  $$(".lg-edges").forEach((item) => { item.hidden = !clusters.some(edge); });
  $(".lg-event-item").hidden = !clusters.some((mark) => mark.kind === "event");
  const weight = (mark) => (mark.kind === "state" || mark.kind === "event" ? 1 : 0);
  $(".markers").replaceChildren(...clusters.sort((a, b) => weight(a) - weight(b)).map((mark) => {
    const marker = document.createElement("button");
    marker.className = `marker marker-${mark.kind}`;
    marker.style.left = trackLeft(mark.frame);
    const on = mark.kind === "state" ? causes.get(mark.frame) ?? [] : [];
    marker.classList.toggle("marker-transition", on.length > 0);
    marker.title = (mark.labels.length === 1
      ? `${mark.labels[0]} @ frame ${mark.frame}`
      : `${mark.labels.length} × ${mark.kind} @ frames ${mark.frame}–${mark.until}: ${mark.labels.join(", ")}`)
      + (on.length ? ` · on ${on.join(", ")}` : "");
    marker.onclick = () => { seek(mark.frame); revealMotion(motionAt(mark.frame)); };
    return marker;
  }));
  const playhead = document.createElement("div");
  playhead.className = "playhead";
  $(".markers").append(playhead);
  for (const note of state.notes ?? []) {
    if (note.frame === null || note.frame === undefined) continue;
    const marker = document.createElement("button");
    marker.className = "marker marker-note";
    marker.style.left = trackLeft(note.frame);
    marker.textContent = "◆";
    marker.title = `Note at frame ${note.frame}: ${note.text}`;
    marker.onclick = () => jumpToFinding(note.frame, null, null, note.end_frame);
    $(".markers").append(marker);
  }
  // replaceChildren just took the bars with it, and the live poll comes back through here four
  // times a second, so they are put back from what was already fetched.
  paintOverlay();
  movePlayhead();
}

// How far past its declared dwell a gate has to hold before the wait is worth remarking on.
const SLOW_GATE = 1.2;

// What a payload was read from, said plainly: an archived graph is the whole record, a
// projection is strided and cannot be asked how long anything waited.
function sourceNote(data) {
  if (data.runtime_source === "archive") return "from the archived runtime graph";
  if (data.runtime_source === "projected") {
    return "projected from a run still going — waits and re-arms not yet known";
  }
  return "no runtime graph recorded yet";
}

// One gate, in the words a reader would use: when it became satisfiable, when it fired, and
// what it did in between. Never a verdict -- a long wait may be exactly what was wanted.
function gateStory(gate, period) {
  if (gate.fired_step === null) return "never fired";
  const at = (step) => (period ? `${(step * period).toFixed(2)} s` : `step ${step}`);
  const parts = [`satisfiable at ${at(gate.first_held_step)}`, `fired at ${at(gate.fired_step)}`];
  if (gate.declared_dwell_s !== null) parts.push(`declared dwell ${gate.declared_dwell_s.toFixed(2)} s`);
  if (gate.waited_s !== null && gate.declared_dwell_s && gate.waited_s > gate.declared_dwell_s * SLOW_GATE) {
    parts.push(`waited ${(gate.waited_s - gate.declared_dwell_s).toFixed(2)} s beyond its dwell`);
  }
  if (gate.rearm_count) parts.push(`condition broke ${gate.rearm_count}× before firing`);
  return parts.join(" · ");
}

// The two trackLeft expressions subtract, leaving the fraction of the thumb's travel a span
// covers -- so a bar keeps the axis' geometry without measuring anything.
function spanWidth(begin, end) {
  return `calc(${trackFraction(end) - trackFraction(begin)} * (100% - var(--thumb)))`;
}

// Where a span that never closed ends: the log's current edge, which a live run keeps moving.
const lastFrame = () => Math.max(0, state.replay.frames - 1);

function place(node, begin, end) {
  node.style.left = trackLeft(begin);
  node.style.width = spanWidth(begin, end);
}

// Motion spans as background stripes inside the strip, not above it: the occupancy is the
// context its markers happened in, and drawn in-band it costs the transport no height -- which
// #content's padding and .replay's height are both hardcoded against.
function renderSpans(data) {
  $(".markers").prepend(...data.spans.map((span, index) => {
    const bar = document.createElement("button");
    bar.className = "span span-motion";
    bar.dataset.element = span.element;
    bar.dataset.band = index % 2;   // adjacent motions told apart without inventing a palette
    place(bar, span.begin_step, span.end_step ?? lastFrame());
    // The authored name, joined by the motion's IRI; the graph's handler name is the fallback.
    // Named in the tooltip only: a strip of labels that fit here and not there read as noise.
    const name = state.motionByIri[span.element] ?? span.name;
    bar.title = [
      name,
      `entered ${seconds(span.entered_s)}`,
      span.duration_s === null ? "open" : `for ${seconds(span.duration_s)}`,
      span.event ? `on ${span.event}` : "",
    ].filter(Boolean).join(" · ");
    bar.onclick = () => { seek(span.begin_step); revealMotion(name); };
    return bar;
  }));
}

// A click on the strip lands the table on the motion it points at: the block lights the way a
// live run lights it, and the list scrolls so the block is what is read. Opens no plots.
function revealMotion(name) {
  if (!name) return;
  trackActiveMotion(name, { plots: false, live: false });
  const panel = $("#constraints");
  const heading = $$("#constraints .constraint-motion").find((h) => h.textContent === name);
  if (!panel || !heading) return;
  // offsetTop of a stuck heading is where it is stuck, not where it lives; the distance from
  // the panel's own scroll box says where it lives.
  const top = heading.getBoundingClientRect().top - panel.getBoundingClientRect().top + panel.scrollTop;
  panel.scrollTo({ top, behavior: "smooth" });
}

// The wait only: .marker-monitor already draws the instant the gate fired, and a second dot on
// the same step would read as a second firing.
function renderGateWaits(data) {
  const armed = data.gates.filter(
    (gate) => gate.first_held_step !== null && gate.fired_step !== null,
  );
  // The same line gateStory draws: past its dwell by the margin is worth a colour of its own.
  const isSlow = (gate) =>
    gate.waited_s !== null && gate.declared_dwell_s && gate.waited_s > gate.declared_dwell_s * SLOW_GATE;
  $(".lg-wait-item").hidden = !armed.length;
  $(".lg-slow-item").hidden = !armed.some(isSlow);
  $(".markers").prepend(...armed.map((gate) => {
    const wait = document.createElement("div");
    wait.className = "gate-wait";
    wait.classList.toggle("gate-slow", Boolean(isSlow(gate)));
    place(wait, gate.first_held_step, gate.fired_step);
    wait.title = [
      gate.monitor_name,
      gateStory(gate, data.period_s),
      data.runtime_source === "archive" ? "" : sourceNote(data),
    ].filter(Boolean).join(" · ");
    return wait;
  }));
}

// Bars are an enhancement, never a precondition: the frame log has already drawn the markers by
// the time these land, and a graph query that fails leaves the strip exactly as it was.
export async function loadTimelineOverlay(path, force = false) {
  if (!force && state.spanOverlay?.path === path) return paintOverlay();
  const query = `path=${encodeURIComponent(path)}`;
  const [timeline, gates] = await Promise.all([
    api(`/api/run/timeline?${query}`),
    api(`/api/run/gates?${query}`),
  ]);
  state.spanOverlay = { path, timeline, gates };
  paintOverlay();
}

// The live poll rebuilds the strip four times a second and the axis stretches as the log grows,
// so the bars are repainted from what was fetched rather than re-asked of the graph.
function paintOverlay() {
  const overlay = state.spanOverlay;
  if (!overlay || overlay.path !== state.runPath || !$(".markers")) return;
  // Idempotent whoever calls it: renderMarkers has already cleared the strip, but a settle or a
  // second load has not, and prepending onto what is there would stack a second set of bars.
  $$(".span-motion, .gate-wait").forEach((node) => node.remove());
  $(".markers").title = sourceNote(overlay.timeline);
  renderSpans(overlay.timeline);
  renderGateWaits(overlay.gates);
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

// Turning it off has to put back what it cut, so both directions go through updateCursor.
function bindProgressive() {
  const button = $("#progressive-plots");
  const label = () => {
    button.setAttribute("aria-pressed", String(progressiveOn()));
    button.textContent = `progressive: ${progressiveOn() ? "on" : "off"}`;
  };
  button.onclick = () => {
    setProgressive(!progressiveOn());
    state.charts.forEach((chart) => {
      chart.drawnUpTo = null;
      if (chart.fullSeries) chart.setOption(seriesUpTo(chart, state.frame));
    });
    label();
  };
  label();
}

// When each motion ran, from the spans its own constraints were evaluated over. The recording
// answers this itself; turning a state's name (S_PICK_ABOVE) into a motion's (pick-above) would
// be a spelling rule invented here and true only until someone renames a state.
function motionSpans() {
  const spans = new Map();
  (state.replay?.constraints ?? []).forEach(({ motion, window }) => {
    if (!motion || !window) return;
    const span = spans.get(motion);
    spans.set(motion, span
      ? [Math.min(span[0], window[0]), Math.max(span[1], window[1])]
      : [...window]);
  });
  return [...spans].sort((left, right) => left[1][0] - right[1][0]);
}

// The motion the run is in at this frame -- or the last one it was in, since the frames before
// a motion's constraints start evaluating, and the gaps between motions, still belong to it.
// Spans can overlap where a guard outlives its motion; the one entered most recently wins.
function motionAt(frame) {
  const spans = state.motionSpans ?? [];
  let current = null;
  for (const [motion, [from, to]] of spans) {
    if (from <= frame && frame <= to) current = motion;
  }
  if (current) return current;
  const entered = spans.filter(([, [from]]) => from <= frame);
  return (entered.length ? entered.at(-1) : spans[0])?.[0] ?? null;
}

// Cards follow the cursor the way they follow a live run: the motion being replayed has its
// charts open and they close behind it. One motion's charts, never the whole run's.
export const AUTO_PLOT_KEY = "motion-spec.auto-plot";

let followMotions = false;

try { followMotions = localStorage.getItem(AUTO_PLOT_KEY) === "on"; } catch { /* private */ }

export function followMotion() {
  if (state.restoringInspection) return;
  // A live run is already followed by its poll; this is the same behaviour for a recording.
  if (!followMotions || state.following || state.opening) return;
  const motion = motionAt(state.frame);
  if (!motion || motion === state.activeMotion) return;
  // Scrubbing crosses motions faster than a card can load; one unfold at a time, then a
  // re-check, so a drag lands on the motion it stopped at instead of every one it passed.
  state.opening = true;
  Promise.resolve(trackActiveMotion(motion, { plots: true, live: false }))
    .catch(() => {})
    .finally(() => { state.opening = false; followMotion(); });
}

function bindAutoPlot() {
  const button = $("#auto-plot");
  const label = () => {
    button.setAttribute("aria-pressed", String(followMotions));
    button.textContent = `auto plot: ${followMotions ? "on" : "off"}`;
  };
  button.onclick = () => {
    followMotions = !followMotions;
    try { localStorage.setItem(AUTO_PLOT_KEY, followMotions ? "on" : "off"); } catch { /* private */ }
    label();
    followMotion();
  };
  label();
}

// The cards worth opening on their own: the ones a named set asks for, else every plottable
// constraint of the last motion that ran. Never the whole run's card set -- eighty charts in
// one page is what made live pages crawl.
function openPlotsFor(path, keys, motion) {
  const wanted = motion ?? state.replay.constraints
    .filter((constraint) => constraint.window)
    .sort((left, right) => right.window[1] - left.window[1])[0]?.motion;
  // Each row is a full-log /api/plot read: click them apart so several never land in one
  // frame. Detached on purpose -- the page is usable while the cards fill in.
  (async () => {
    const rows = $$("#constraints .constraint").filter((row) => {
      if (!row.dataset.plottable || row.dataset.plotted || row.dataset.idle) return false;
      const key = `${row.dataset.motion}/${row.querySelector("strong")?.textContent}`;
      return keys.length ? keys.includes(key) : row.dataset.motion === wanted;
    });
    if (!rows.length) return snack("Nothing recorded to plot in this run.");
    for (const row of rows) {
      if (state.runPath !== path) return;   // the reader moved on
      row.click();
      await new Promise((resolve) => setTimeout(resolve, 50));
    }
  })();
}

export function updateCursor() {
  // A cursor pinned to the live edge draws nothing worth a setOption on every chart; it comes
  // back by itself when following ends and the finished run is on screen.
  if (state.following) return;
  state.charts.forEach((chart) => {
    // Only when the cut actually moves: at 5x most frames land between two samples, and
    // redrawing every series to draw the identical line is the whole cost of playing.
    if (progressiveOn() && chart.fullSeries) {
      const cut = seriesUpTo(chart, state.frame);
      const drawn = cut.series[0]?.data.length ?? 0;
      if (drawn !== chart.drawnUpTo) {
        chart.drawnUpTo = drawn;
        chart.setOption(cut);
      }
    }
    chart.setOption(cursorOption(state.frame, chart.cursorIndex ?? 0));
  });
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
  followMotion();
  movePlayhead();
  syncVideo();
}

/** Reports and annotations use the same seek path as the transport and video. */
export function jumpToFinding(frame, motion = null, constraint = null, endFrame = null) {
  $(".replay-tabs [data-panel=plots]").click();
  seek(frame);
  const row = [...$$("#constraints .constraint")].find((row) =>
    (!motion || row.dataset.motion === motion) && constraint && row.querySelector("strong")?.textContent === constraint);
  if (row && !row.dataset.plotted) row.click();
  if (endFrame != null) state.charts.forEach((chart) => chart.dispatchAction?.({
    type: "dataZoom", startValue: frame, endValue: endFrame,
  }));
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
