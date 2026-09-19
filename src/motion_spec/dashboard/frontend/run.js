// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/** One run's page: constraints, transport, recording and console, finished or still growing. */

import {
  appendConsole,
  bindToggle,
  consoleExcerpt,
  copyPathButton,
  fileFolder,
  fileRow,
  iconMarkup,
  setPickAll,
  showEmpty,
} from "./components.js";
import { NOTES_MARKUP, annotationEditor } from "./annotations.js";
import { bindInspection, restoreInspection } from "./inspection.js";
import {
  $,
  $$,
  api,
  askConfirm,
  formatBytes,
  post,
  readStored,
  snack,
  snackError,
  stampText,
  state,
  writeStored,
} from "./core.js";
import { EXPLORE_MARKUP, bindExplore } from "./explore.js";
import {
  highlightGeneration,
  loadGenerations,
  readRunOptions,
  selectGeneration,
  stopRun,
} from "./generations.js";
import {
  addLivePlot,
  bindLivePlots,
  followLiveRun,
  livePlotsOn,
  simControl,
  trackActiveMotion,
} from "./live.js";
import { openNotebook } from "./notebook.js";
import { addPlot, cursorOption, progressiveOn, seriesUpTo, setProgressive } from "./plots.js";
import { showReports } from "./reports.js";
import { hashView, setView, writeHash } from "./routing.js";
import { enhanceSelect } from "./selects.js";
import { showGenerated } from "./sources.js";

const PANELS = ["plots", "reports", "explore", "console", "files", "notes"];

// The page of a run just named, open before its log exists. /api/replay answers from the
// generation's contract, so a generation built before that file existed holds here instead.
export async function openPendingRun(runPath) {
  state.autoPlot = runPath;
  if (await opened(runPath)) return;
  state.anchor = runPath;
  stopPlayback();
  setView("run", runPath);
  state.runPath = runPath;
  state.generationPath = runPath.split("/runs/")[0];
  state.live = state.following = null;
  highlightGeneration();
  showRunStarting();
  const hop = setInterval(async () => {
    if (state.runPath !== runPath) return clearInterval(hop); // the user went elsewhere
    if (await opened(runPath)) return clearInterval(hop);
    const status = await api(`/api/run?path=${encodeURIComponent(state.generationPath)}`).catch(
      () => null,
    );
    if (status && !status.busy) {
      // over without ever writing a log: the runner's own words are all there is to show
      clearInterval(hop);
      await showRunWithoutLog(state.generationPath, runPath, status);
    }
  }, 300);
}

const opened = (runPath) =>
  loadReplay(runPath)
    .then(() => true)
    .catch(() => false);

// A run that never reaches its first frame is waited on here, so this is where it is ended.
function showRunStarting() {
  const waiting = showEmpty({
    spinner: true,
    title: "Run starting…",
    detail: "Waiting for the first frames of the log.",
  });
  waiting.insertAdjacentHTML("beforeend", '<button id="stop-pending">stop run</button>');
  $("#stop-pending").onclick = async (event) => {
    event.target.disabled = true;
    await stopRun(state.generationPath).catch(snackError);
  };
}

// A run over with no log. Told not to write one is not the same as never having run, and the
// manifest says which.
export async function showRunWithoutLog(generationPath, runPath, status) {
  if (status?.recorded !== false) return showRunNeverStarted(generationPath, status?.exit_code);
  const empty = showEmpty({
    eyebrow: "NO LOG",
    title: "This run recorded no frame log (logs off).",
    detail: "No replay and no plots. The console is what the run left behind.",
  });
  empty.append(await consoleExcerpt(runPath, 500));
}

async function showRunNeverStarted(generationPath, exitCode) {
  const empty = showEmpty({
    eyebrow: "ERROR",
    title: "The run never started.",
    detail: `exited ${exitCode ?? "?"}`,
  });
  empty.append(await consoleExcerpt(generationPath));
}

export async function loadReplay(path) {
  state.anchor = path;
  stopPlayback();
  setView("run", path);
  state.replay = await api(`/api/replay?path=${encodeURIComponent(path)}`);
  dropOpenCharts();
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
  indexReplay();
  populateConstraints();
  watchPlottedRows();
  // The archive of a run this page started reopens the cards that were open when it ended.
  const reopened = state.autoPlot === path && !state.replay.pending && livePlotsOn();
  if (reopened) reopenPlots(path);
  $("#plot").onclick = () => addPlot([]);
  state.motionSpans = motionSpans();
  state.opening = false;
  bindAutoPlot();
  // Following would close that set on the spot to show frame 0's motion instead.
  if (!reopened) followMotion();
  $("#notebook").onclick = () => openNotebook(state.runPath).catch(snackError);
  bindLivePlots();
  bindProgressive();
  bindPanels();
  // A side panel that cannot load is not the page failing to load.
  bindExplore(path).catch(snackError);
  bindTransport();
  setTransportMode();
  showVideos(path, state.replay.videos ?? []);
  // Nothing recorded: a real platform has a camera, but only ROS to reach it through.
  if (!state.replay.videos?.length) showRosCamera(state.replay.generation);
  followLiveRun(path);
  bindConsoleSearch();
  followConsole(path);
  if (state.replay.pending) settle(true, "waiting for the run to start…");
  $("#back").onclick = () => selectGeneration(state.replay.generation);
  bindRunAgain(path);
  updateReadout();
  mountAnnotations(path);
  loadNotes(path);
  bindInspection(path);
  if (!state.replay.pending && !state.following) restoreInspection(path);
}

function dropOpenCharts() {
  state.charts.forEach((chart) => chart.dispose());
  state.charts = [];
  state.livePlots.clear();
  state.pendingSignals.clear();
  state.liveBuffer.clear();
  state.activeMotion = null;
}

function indexReplay() {
  // gate id -> the name the source spells it: a row carries the name, not the gate it ran under
  state.motionOf = state.replay.motion_names ?? {};
  // slot id -> the motion and constraint it serves: an edge in the log names the slot.
  state.slotOwner = Object.fromEntries(
    Object.values(state.replay.signal_index ?? {})
      .filter((where) => where.slot)
      .map((where) => [where.slot, where]),
  );
}

// Edges draw for the constraints whose charts are open, so the strip follows the cards.
function watchPlottedRows() {
  state.plottedWatch?.disconnect();
  state.plottedWatch = new MutationObserver(() => renderMarkers());
  state.plottedWatch.observe($("#constraints"), {
    attributes: true,
    subtree: true,
    attributeFilter: ["data-plotted"],
  });
}

function reopenPlots(path) {
  state.autoPlot = null;
  const keys = state.reopenRows ?? [];
  const lastMotion = state.reopenMotion;
  state.reopenRows = state.reopenMotion = null;
  openPlotsFor(path, keys, lastMotion);
}

function mountAnnotations(path) {
  annotationEditor(path, (data) => {
    state.generation = null;
    if (state.runPath === path) {
      $(".replay-heading h1").textContent = data.label || path.split("/").pop();
      $("#panel-reports").dataset.run = "";
    }
    return loadGenerations(true);
  })
    .then((editor) => {
      if (state.runPath !== path) return;
      $(".replay-heading")?.after(editor);
      $(".replay-heading h1").textContent = editor.elements.label.value || path.split("/").pop();
      $(".replay-heading h1").title = path;
    })
    .catch(snackError);
}

function loadNotes(path) {
  api(`/api/notes?path=${encodeURIComponent(path)}`)
    .then((data) => {
      if (state.runPath === path) {
        state.notes = data.notes;
        renderMarkers();
      }
    })
    .catch(snackError);
}

// The same start the generation page makes, with the run bar's remembered choices.
function bindRunAgain(runPath) {
  const button = $("#run-again");
  const generation = state.replay.generation;
  button.hidden = Boolean(state.replay.pending);
  button.onclick = async () => {
    button.disabled = true;
    try {
      const started = await post("/api/run", {
        path: generation,
        options: readRunOptions(generation),
      });
      await openPendingRun(started.run);
    } catch (error) {
      button.disabled = false;
      snackError(error);
    }
  };
}

const REPLAY_TABS =
  '<div class="replay-tabs"><button data-panel="plots" class="active">Plots</button>' +
  '<button data-panel="reports">Reports</button><button data-panel="explore">Explore</button>' +
  '<button data-panel="console">Console</button><button data-panel="files">Files</button>' +
  '<button data-panel="notes">Notes</button></div>';

const PLOTS_PANEL =
  '<section id="panel-plots"><div class="constraint-panel">' +
  '<div class="eyebrow">SOURCE CONSTRAINTS</div>' +
  '<input id="constraint-search" type="search" placeholder="Search .robmot constraints">' +
  '<div id="constraints" class="constraints"></div></div><div class="chart-controls">' +
  '<button id="auto-plot" title="Open each motion\'s plots as the cursor enters it, the way a live run does">auto plot: off</button>' +
  '<button id="plot">Add empty plot</button><button id="notebook">Open in Jupyter</button>' +
  '<button id="live-plots" title="Plot signals as the run writes them">live plots: on</button>' +
  '<button id="progressive-plots" title="Draw a replayed run the way a live one arrives: nothing past the cursor">progressive: off</button>' +
  '</div><div id="plots" class="plots"></div></section>';

const CONSOLE_PANEL =
  '<section id="panel-console" hidden><div class="console-bar">' +
  '<input id="console-search" type="search" spellcheck="false" placeholder="Search the console">' +
  '<span id="console-matches"></span>' +
  `<button id="console-prev" title="Previous match (Shift+Enter)" aria-label="Previous match" disabled>${iconMarkup("arrow-up")}</button>` +
  `<button id="console-next" title="Next match (Enter)" aria-label="Next match" disabled>${iconMarkup("arrow-down")}</button></div>` +
  '<pre id="console-text" class="console"></pre></section>';

const NOTES_PANEL = `<section id="panel-notes" hidden>${NOTES_MARKUP}</section>`;

const VIDEO_PANEL =
  '<div class="videos" hidden><button class="video-max" title="Expand"></button>' +
  '<button class="video-min" title="Minimize"></button><div class="video-main">' +
  '<video preload="auto" playsinline disablepictureinpicture controlslist="nodownload noplaybackrate noremoteplayback"></video>' +
  '<span class="video-name"></span></div><div class="video-strip"></div></div>';

const MARKER_LEGEND =
  '<span class="marker-legend">' +
  '<span class="lg-item"><i class="lg lg-state"></i>state</span>' +
  '<span class="lg-item"><i class="lg lg-state lg-transition"></i>on event</span>' +
  '<span class="lg-item lg-event-item" hidden><i class="lg lg-event"></i>event</span>' +
  '<span class="lg-item lg-edges" hidden><i class="lg lg-satisfied"></i>satisfied</span>' +
  '<span class="lg-item lg-edges" hidden><i class="lg lg-unsatisfied"></i>lost</span>' +
  '<span class="lg-item"><i class="lg lg-monitor"></i>monitor</span></span>';

const TRANSPORT =
  '<div class="transport"><div class="transport-controls">' +
  `<button id="step-back" title="Previous frame" aria-label="Previous frame">${iconMarkup("chevron-left")}</button>` +
  '<button id="play">Play</button>' +
  `<button id="step-forward" title="Next frame" aria-label="Next frame">${iconMarkup("chevron-right")}</button>` +
  '<details class="picker speed-menu"><summary>1×</summary><div class="picker-panel">' +
  '<button data-value="0.25">0.25×</button><button data-value="0.5">0.5×</button>' +
  '<button data-value="1" aria-pressed="true">1×</button><button data-value="2">2×</button>' +
  '<button data-value="5">5×</button></div></details>' +
  '<span id="readout" class="path"></span>' +
  MARKER_LEGEND +
  '<button id="cancel-run" title="End the run" hidden>cancel</button></div>' +
  '<div class="markers"></div>' +
  '<input class="timeline" type="range" min="0" max="0" value="0" disabled></div>';

function replayShell(path) {
  return (
    '<div class="replay"><div class="replay-heading">' +
    `<button id="back" title="Back to generation" aria-label="Back to generation">${iconMarkup("arrow-left")}</button>` +
    `<h1>${path.split("/").pop()}</h1>` +
    '<button id="run-again" title="Run this generation again with the run bar\'s last choices">' +
    `${iconMarkup("refresh-cw")} run again</button>` +
    REPLAY_TABS +
    '<span class="eyebrow">RUN</span></div>' +
    PLOTS_PANEL +
    '<section id="panel-reports" hidden></section>' +
    `<section id="panel-explore" hidden>${EXPLORE_MARKUP}</section>` +
    CONSOLE_PANEL +
    '<section id="panel-files" hidden><div class="generated-files run-files"></div></section>' +
    NOTES_PANEL +
    "</div>" +
    '<div class="settling" hidden><div class="spinner"></div><span>archiving the run…</span></div>' +
    VIDEO_PANEL +
    TRANSPORT
  );
}

function bindPanels() {
  const show = (panel, remember = true) => {
    document
      .querySelectorAll(".replay-tabs button")
      .forEach((button) => button.classList.toggle("active", button.dataset.panel === panel));
    PANELS.forEach((name) => {
      $(`#panel-${name}`).hidden = name !== panel;
    });
    // A panel that cannot load is not the page failing to load.
    if (panel === "reports") showReports(state.runPath).catch(snackError);
    if (panel === "files") showRunFiles(state.runPath).catch(snackError);
    if (panel === "notes") showNotes(state.runPath).catch(snackError);
    state.charts.forEach((chart) => chart.resize());
    if (!remember) return;
    writeHash({ set: { panel }, replace: true });
  };
  document.querySelectorAll(".replay-tabs button").forEach((button) => {
    button.onclick = () => show(button.dataset.panel);
  });
  // A hash naming a panel this page no longer offers falls back to plots.
  const stored = hashView().get("panel");
  show(PANELS.includes(stored) ? stored : "plots", false);
}

// Re-read on each opening of the tab: a live run is still adding to it.
async function showRunFiles(runPath) {
  const files = await api(`/api/run/files?path=${encodeURIComponent(runPath)}`);
  const list = $(".run-files");
  if (!list) return;
  const entry = (file) => {
    const row = fileRow(file.name, `${file.path} — click to read`, () =>
      showGenerated(`${runPath}/${file.full}`).catch(snackError),
    );
    const size = document.createElement("span");
    size.className = "file-size";
    size.textContent = formatBytes(file.size);
    row.append(size, copyPathButton(file.path, { stopClick: true }));
    return row;
  };
  const folders = new Map();
  const loose = [];
  files.forEach((file) => {
    const [folder, ...rest] = file.name.split("/");
    if (!rest.length) return loose.push({ ...file, full: file.name });
    const entry = { ...file, full: file.name, name: rest.join("/") };
    const inside = folders.get(folder);
    if (inside) inside.push(entry);
    else folders.set(folder, [entry]);
  });
  list.replaceChildren(
    ...loose.map(entry),
    ...[...folders].map(([folder, inside]) => fileFolder(folder, inside.map(entry), true)),
  );
}

// Every note kept beside the run, newest first. The whole list is saved on any change: notes
// are few and small, and one write is one thing to get right.
export async function showNotes(runPath, container = null) {
  const panel = container ?? $("#panel-notes");
  if (!panel) return;
  const list = panel.querySelector(".notes-list");
  const text = panel.querySelector(".note-text");
  const tags = panel.querySelector(".note-tags");
  const status = panel.querySelector(".notes-state");
  const splitTags = (value) =>
    value
      .split(",")
      .map((tag) => tag.trim())
      .filter(Boolean);
  let notes = (await api(`/api/notes?path=${encodeURIComponent(runPath)}`)).notes;
  // Only the run page can attach a note to a frame; the generation page has no cursor.
  const [frameInput, endInput] = container ? [null, null] : addFrameFields(panel);
  // Picked by id, so a selection survives a redraw and is dropped only with the note it names.
  const picked = new Set();
  const bar = panel.querySelector(".notes-bar");
  const pickAll = bar.querySelector(".notes-pick-all");
  const syncBar = () => {
    bar.hidden = !notes.length;
    bar.querySelector(".notes-picked").textContent = picked.size ? `${picked.size} selected` : "";
    bar.querySelector(".notes-delete").disabled = !picked.size;
    setPickAll(pickAll, picked.size, notes.length);
  };
  pickAll.onchange = () => {
    picked.clear();
    if (pickAll.checked) notes.forEach((note) => picked.add(note.id));
    render();
  };
  bar.querySelector(".notes-delete").onclick = async () => {
    const count = picked.size;
    if (
      !(await askConfirm({
        message: `Delete ${count} note${count === 1 ? "" : "s"}?`,
        confirmLabel: "Delete",
      }))
    )
      return;
    save(notes.filter((note) => !picked.has(note.id)));
  };
  const save = async (next) => {
    status.textContent = "saving…";
    try {
      notes = (await post("/api/notes", { path: runPath, notes: next })).notes;
      state.cache.generations = null;
      if (!container && state.runPath === runPath) {
        state.notes = notes;
        renderMarkers();
      }
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
    const pick = Object.assign(document.createElement("input"), {
      type: "checkbox",
      className: "pick",
      checked: picked.has(note.id),
      title: "Select",
    });
    pick.onchange = () => {
      pick.checked ? picked.add(note.id) : picked.delete(note.id);
      syncBar();
    };
    head.append(
      pick,
      Object.assign(document.createElement("span"), {
        className: "note-when",
        textContent: stampText(note.created),
      }),
    );
    head.append(
      ...note.tags.map((tag) =>
        Object.assign(document.createElement("span"), { className: "run-tag", textContent: tag }),
      ),
    );
    if (!container && note.frame !== null && note.frame !== undefined) {
      const jump = document.createElement("button");
      jump.className = "note-action";
      jump.textContent = `Frame ${note.frame}${note.end_frame == null ? "" : `–${note.end_frame}`}`;
      jump.onclick = () => jumpToFinding(note.frame, null, null, note.end_frame);
      head.append(jump);
    }
    const edit = Object.assign(document.createElement("button"), {
      className: "note-action",
      textContent: "edit",
    });
    const remove = Object.assign(document.createElement("button"), {
      className: "note-action",
      textContent: "delete",
    });
    head.append(edit, remove);
    const body = Object.assign(document.createElement("p"), {
      className: "note-body",
      textContent: note.text,
    });
    box.append(head, body);
    remove.onclick = async () => {
      if (!(await askConfirm({ message: "Delete this note?", confirmLabel: "Delete" }))) return;
      save(notes.filter((other) => other.id !== note.id));
    };
    edit.onclick = () => {
      const editor = Object.assign(document.createElement("textarea"), {
        className: "note-text",
        rows: 3,
        value: note.text,
      });
      const editTags = Object.assign(document.createElement("input"), {
        className: "note-tags",
        value: note.tags.join(", "),
        spellcheck: false,
      });
      const keep = Object.assign(document.createElement("button"), {
        className: "note-action",
        textContent: "save",
      });
      const drop = Object.assign(document.createElement("button"), {
        className: "note-action",
        textContent: "cancel",
      });
      const editBar = Object.assign(document.createElement("div"), {
        className: "notes-compose-bar",
      });
      editBar.append(editTags, keep, drop);
      body.replaceWith(editor, editBar);
      editor.focus();
      keep.onclick = () =>
        save(
          notes.map((other) =>
            other.id === note.id
              ? { ...other, text: editor.value, tags: splitTags(editTags.value) }
              : other,
          ),
        );
      drop.onclick = render;
    };
    return box;
  };
  const render = () => {
    const ordered = [...notes].sort((a, b) => (a.created < b.created ? 1 : -1));
    const live = new Set(notes.map((note) => note.id));
    picked.forEach((id) => {
      if (!live.has(id)) picked.delete(id);
    });
    list.replaceChildren(...ordered.map(card));
    syncBar();
    if (!ordered.length)
      list.append(
        Object.assign(document.createElement("p"), {
          className: "path notes-empty",
          textContent: "No notes yet.",
        }),
      );
  };
  const add = () => {
    if (!text.value.trim()) return;
    const frame = frameInput?.value ? Number(frameInput.value) : null;
    const end = endInput?.value ? Number(endInput.value) : null;
    if (
      frameInput &&
      (!frameInput.checkValidity() ||
        !endInput.checkValidity() ||
        (end !== null && (frame === null || end < frame)))
    ) {
      status.textContent = "Choose a valid frame or range within this recording.";
      return;
    }
    save([...notes, { text: text.value, tags: splitTags(tags.value), frame, end_frame: end }]).then(
      () => {
        if (!status.textContent) {
          text.value = "";
          tags.value = "";
        }
      },
    );
  };
  panel.querySelector(".note-add").onclick = add;
  text.onkeydown = (event) => {
    if ((event.ctrlKey || event.metaKey) && event.key === "Enter") add();
  };
  render();
}

function addFrameFields(panel) {
  panel.querySelector(".note-position")?.remove();
  const position = document.createElement("div");
  position.className = "note-position";
  position.innerHTML =
    '<button type="button">Attach current frame</button><label>Frame<input type="number" min="0" step="1" placeholder="General note"></label><label>End frame (optional)<input type="number" min="0" step="1"></label>';
  const [frameInput, endInput] = position.querySelectorAll("input");
  frameInput.max = endInput.max = Math.max(0, state.replay.frames - 1);
  position.querySelector("button").onclick = () => {
    frameInput.value = state.frame;
  };
  panel.querySelector(".notes-compose").append(position);
  return [frameInput, endInput];
}

export const constraintKey = (row) =>
  `${row.dataset.motion}/${row.querySelector("strong")?.textContent}`;

function populateConstraints() {
  const groups = new Map();
  state.replay.constraints.forEach((constraint) => {
    const motion = constraint.motion ?? "shared";
    const group = groups.get(motion);
    if (group) group.push(constraint);
    else groups.set(motion, [constraint]);
  });
  $("#constraints").replaceChildren(
    ...[...groups].flatMap(([motion, constraints]) => {
      const heading = document.createElement("div");
      heading.className = "constraint-motion";
      heading.textContent = motion;
      return [heading, ...constraints.map((constraint) => constraintRow(constraint, motion))];
    }),
  );
  $("#constraint-search").oninput = (event) => filterRows(".constraint", event.target.value);
}

function constraintRow(constraint, motion) {
  const row = document.createElement("div");
  const line = document.createElement("span");
  const name = document.createElement("strong");
  const expression = document.createElement("span");
  row.className = "constraint";
  row.dataset.kind = constraint.kind;
  row.dataset.motion = motion; // which block a live run's active motion lights up
  // A when-guard's monitor runs in the predecessor motion's window, so its plots follow the
  // owner rather than the block it was authored in.
  const monitorPrefix = constraint.monitors?.[0]?.split(".")[0];
  const owner = state.replay.monitor_owners?.[monitorPrefix];
  if (owner && owner !== motion) row.dataset.monitorOwner = owner;
  if (!constraint.window) row.dataset.idle = "true";
  line.textContent = constraint.line ? `L${constraint.line}` : "";
  name.textContent = constraint.name;
  expression.textContent = constraint.expression;
  row.append(line, name, expression);
  if (constraint.members?.length) row.append(memberButton(row, constraint));
  if (constraint.tracking.length || constraint.control.length || constraint.monitors.length) {
    bindRowPlots(row, constraint, motion);
  } else {
    row.dataset.unavailable = "true";
    row.title = constraint.evaluator
      ? "No recorded signal"
      : "Nothing evaluates this constraint in this generation";
  }
  return row;
}

function memberButton(row, constraint) {
  const members = document.createElement("button");
  members.className = "plot-members";
  members.textContent = `plot ${constraint.members.length} members`;
  members.title = "One plot per watched constraint: they are not the same quantity";
  members.onclick = (event) => {
    event.stopPropagation();
    row.dataset.plotted = "true";
    constraint.members.forEach((member) =>
      addPlot(
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
      ),
    );
  };
  return members;
}

function bindRowPlots(row, constraint, motion) {
  row.dataset.plottable = "true";
  row.title = constraint.window
    ? `Plot this ${constraint.kind} constraint`
    : "This motion never ran in this recording";
  // measured against its setpoint where there is one; otherwise the error against zero
  const tracked = constraint.tracking.length ? constraint.tracking : constraint.error;
  const machinery =
    constraint.kind === "monitored"
      ? constraint.monitors
      : [...constraint.error, ...constraint.control];
  row.plotSignals = [...tracked, ...machinery];
  const title = `${motion} / ${constraint.name}`;
  const between = constraint.between.length ? ` · ${constraint.between.join(" vs ")}` : "";
  const where = constraint.line ? `L${constraint.line}: ` : "";
  const detail = `${where}${constraint.expression ?? constraint.name}${between}`;
  // A when-guard's monitor runs during the predecessor motion; say so on the card.
  const monitorDetail = row.dataset.monitorOwner
    ? `${detail} · evaluated during ${row.dataset.monitorOwner}`
    : detail;
  // Live: one card per constraint, because its tracked signal and controller machinery share
  // the frame axis. The monitor keeps its own, its value and satisfied being another scale.
  const openLive = (data) => {
    const combined = [
      ...new Set([...tracked, ...(constraint.kind === "monitored" ? [] : machinery)]),
    ];
    if (combined.length) addLivePlot(combined, title, detail, { row, data });
    if (constraint.kind === "monitored" && constraint.monitors.length) {
      addLivePlot(constraint.monitors, `${title} · monitor`, monitorDetail, { row, data });
    }
  };
  const openReplay = (data) => {
    const bands = constraint.tracking.length
      ? constraint
      : { ...constraint, setpoints: [{ label: "satisfied", value: 0 }] };
    if (tracked.length)
      addPlot(tracked, `${title} · constraint`, detail, { row, constraint: bands, data });
    if (constraint.kind === "monitored") {
      if (constraint.monitors.length) {
        addPlot(constraint.monitors, `${title} · monitor`, monitorDetail, {
          row,
          constraint,
          data,
        });
      }
      return;
    }
    if (machinery.length)
      addPlot(machinery, `${title} · controller`, detail, {
        row,
        constraint,
        gains: constraint.gains,
        data,
      });
  };
  // `data` is a prefetched /api/plot answer shared across one motion's rows; without one the
  // card fetches its own, which is what a lone click wants.
  row.openPlots = (data = null) => {
    row.dataset.plotted = "true";
    if (state.following && livePlotsOn()) return openLive(data);
    openReplay(data);
  };
  row.onclick = () => {
    if (row.dataset.plotted) {
      $("#plots")
        .querySelectorAll(`[data-row="${row.dataset.plotKey}"] .remove-plot`)
        .forEach((button) => button.click());
      return;
    }
    row.openPlots();
  };
}

function filterRows(selector, value) {
  const query = value.toLowerCase();
  document.querySelectorAll(selector).forEach((item) => {
    item.hidden = !item.textContent.toLowerCase().includes(query);
  });
}

// Cards follow the cursor the way they follow a live run: one motion's charts open and close
// behind it, never the whole run's -- eighty charts in one page is what made live pages crawl.
const AUTO_PLOT_KEY = "motion-spec.auto-plot";

let followMotions = readStored(AUTO_PLOT_KEY) === "on";

function bindAutoPlot() {
  bindToggle(
    $("#auto-plot"),
    "auto plot",
    () => followMotions,
    () => {
      followMotions = !followMotions;
      writeStored(AUTO_PLOT_KEY, followMotions ? "on" : "off");
      followMotion();
    },
  );
}

function followMotion() {
  if (state.restoringInspection) return;
  // A live run is already followed by its poll; this is the same behaviour for a recording.
  if (!followMotions || state.following || state.opening) return;
  const motion = motionAt(state.frame);
  if (!motion || motion === state.activeMotion) return;
  // Scrubbing crosses motions faster than a card can load: one unfold at a time, then a
  // re-check, so a drag lands on the motion it stopped at rather than every one it passed.
  state.opening = true;
  Promise.resolve(trackActiveMotion(motion, { plots: true, live: false }))
    .catch(() => {})
    .finally(() => {
      state.opening = false;
      followMotion();
    });
}

// When each motion ran, from the spans its own constraints were evaluated over. Deriving it
// from state names (S_PICK_ABOVE to pick-above) would be a spelling rule invented here.
function motionSpans() {
  const spans = new Map();
  (state.replay?.constraints ?? []).forEach(({ motion, window }) => {
    if (!motion || !window) return;
    const span = spans.get(motion);
    spans.set(
      motion,
      span ? [Math.min(span[0], window[0]), Math.max(span[1], window[1])] : [...window],
    );
  });
  return [...spans].sort((left, right) => left[1][0] - right[1][0]);
}

// The motion at this frame, or the last one entered: the gaps between motions belong to the
// motion before them. Spans overlap where a guard outlives its motion, and the latest wins.
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

// Light a motion's block and scroll the constraint list to it. Opens no plots.
function revealMotion(name) {
  if (!name) return;
  trackActiveMotion(name, { plots: false, live: false });
  const panel = $("#constraints");
  const heading = $$("#constraints .constraint-motion").find((h) => h.textContent === name);
  if (!panel || !heading) return;
  // offsetTop of a stuck heading is where it is stuck, not where it lives.
  const top =
    heading.getBoundingClientRect().top - panel.getBoundingClientRect().top + panel.scrollTop;
  panel.scrollTo({ top, behavior: "smooth" });
}

// The cards a named set asks for, else every plottable constraint of the last motion that ran.
function openPlotsFor(path, keys, motion) {
  const wanted =
    motion ??
    state.replay.constraints
      .filter((constraint) => constraint.window)
      .sort((left, right) => right.window[1] - left.window[1])[0]?.motion;
  const asked = new Set(keys);
  // Each row is a full-log /api/plot read: click them apart so several never land in one
  // frame. Detached on purpose -- the page is usable while the cards fill in.
  (async () => {
    const rows = $$("#constraints .constraint").filter((row) => {
      if (!row.dataset.plottable || row.dataset.plotted || row.dataset.idle) return false;
      return asked.size ? asked.has(constraintKey(row)) : row.dataset.motion === wanted;
    });
    if (!rows.length) return snack("Nothing recorded to plot in this run.");
    for (const row of rows) {
      if (state.runPath !== path) return; // the reader moved on
      row.click();
      await new Promise((resolve) => setTimeout(resolve, 50));
    }
  })();
}

// One transport for a recording or the live run writing it.
function bindTransport() {
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
function setSpeed(speed) {
  state.speed = speed;
  showSpeed(speed);
  const video = playingVideo();
  if (video) video.playbackRate = speed;
  if (state.live) return simControl({ action: "speed", speed });
  if (state.timer) {
    // restart the animation on the new rate
    stopPlayback();
    togglePlayback();
  }
}

export function showSpeed(speed) {
  $$(".speed-menu .picker-panel button").forEach((option) =>
    option.setAttribute("aria-pressed", Number(option.dataset.value) === speed),
  );
  const summary = $(".speed-menu summary");
  if (summary) summary.textContent = `${speed}×`;
}

// A growing log is followed; only a loop that answers can be paused.
export function setTransportMode() {
  const following = Boolean(state.following);
  const driving = Boolean(state.live);
  const play = $("#play");
  const stepForward = $("#step-forward");
  play.textContent = driving ? (state.live.paused ? "Play" : "Pause") : "Play";
  play.title = driving ? "Pause the simulation" : "Play the recording";
  $("#step-back").disabled = driving;
  stepForward.disabled = driving && !state.live.paused;
  stepForward.title = driving ? "Advance one tick" : "Next frame";
  $(".timeline").disabled = driving || !state.replay.frames;
  $("#cancel-run").hidden = !driving;
  // Starting another run while this one drives the loop would fight it for the platform.
  const again = $("#run-again");
  if (again) again.hidden = driving;
  $(".transport").classList.toggle("transport-live", following);
  // "live plots" only means something on a run being written; "auto plot" is its counterpart
  // on a recording, since the live poll follows motions on its own.
  const live = following || Boolean(state.replay.pending);
  $("#live-plots").hidden = !live;
  $("#auto-plot").hidden = live;
  $("#progressive-plots").hidden = live;
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

export function updateReadout() {
  const duration = state.replay.duration;
  const time = (duration * state.frame) / Math.max(1, state.replay.frames - 1);
  const frames = `${state.frame.toLocaleString()} / ${state.replay.frames.toLocaleString()}`;
  $("#readout").textContent =
    `${time.toFixed(3)} / ${duration.toFixed(3)} s · frame ${frames}${state.following ? " · live" : ""}`;
}

function togglePlayback() {
  if (state.timer) return stopPlayback();
  const fps = (state.speed * state.replay.frames) / Math.max(state.replay.duration, 1e-9);
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
    if (video && !video.paused)
      cursor = (video.currentTime / video.duration) * (state.replay.frames - 1);
    else cursor += ((now - last) * fps) / 1000;
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

/** Reports and annotations use the same seek path as the transport and video. */
export function jumpToFinding(frame, motion = null, constraint = null, endFrame = null) {
  $(".replay-tabs [data-panel=plots]").click();
  seek(frame);
  const row = $$("#constraints .constraint").find(
    (candidate) =>
      (!motion || candidate.dataset.motion === motion) &&
      constraint &&
      candidate.querySelector("strong")?.textContent === constraint,
  );
  if (row && !row.dataset.plotted) row.click();
  if (endFrame != null)
    state.charts.forEach((chart) =>
      chart.dispatchAction?.({
        type: "dataZoom",
        startValue: frame,
        endValue: endFrame,
      }),
    );
}

// Turning it off has to put back what it cut, so both directions go through updateCursor.
function bindProgressive() {
  bindToggle($("#progressive-plots"), "progressive", progressiveOn, () => {
    setProgressive(!progressiveOn());
    state.charts.forEach((chart) => {
      chart.drawnUpTo = null;
      if (chart.fullSeries) chart.setOption(seriesUpTo(chart, state.frame));
    });
  });
}

export function updateCursor() {
  // A cursor pinned to the live edge is not worth a setOption on every chart.
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

// Waiting on the run: nothing on the page is worth clicking yet.
export function settle(waiting, message = "archiving the run…") {
  // The live poll settles after an await, by which time the reader may have left the page.
  const overlay = $(".settling");
  if (!overlay) return;
  overlay.hidden = !waiting;
  overlay.querySelector("span").textContent = message;
  $(".replay")?.classList.toggle("busy", waiting);
  $(".transport")?.classList.toggle("busy", waiting);
}

// Marks of one kind closer than this many pixels are drawn as one.
const CLUSTER_PX = 4;

export function renderMarkers() {
  const clusters = markerClusters();
  // The legend names what is drawn, nothing more.
  const anyEdge = clusters.some(isEdge);
  $$(".lg-edges").forEach((item) => {
    item.hidden = !anyEdge;
  });
  $(".lg-event-item").hidden = !clusters.some((mark) => mark.kind === "event");
  const weight = (mark) => (mark.kind === "state" || mark.kind === "event" ? 1 : 0);
  const playhead = document.createElement("div");
  playhead.className = "playhead";
  $(".markers").replaceChildren(
    ...clusters.sort((a, b) => weight(a) - weight(b)).map(markerButton),
    playhead,
    ...noteMarkers(),
  );
  movePlayhead();
}

const isEdge = (event) => event.kind === "satisfied" || event.kind === "unsatisfied";

// The event that moved the FSM is drawn on the state mark; only one that moved nothing stands
// on its own.
function transitionCauses() {
  const causes = new Map();
  state.replay.events.forEach((event) => {
    if (event.kind === "state") causes.set(event.frame, []);
  });
  return causes;
}

// Marks of one kind too close to tell apart become one mark with several names.
function markerClusters() {
  // An edge is only drawn for a constraint whose chart is open; with no chart it is texture.
  const plotted = new Set($$("#constraints .constraint[data-plotted]").map(constraintKey));
  const causes = transitionCauses();
  const kept = state.replay.events.filter((event) => {
    if (event.kind === "event") {
      const entry = causes.get(event.frame) ?? causes.get(event.frame + 1);
      if (entry) entry.push(event.label);
      return !entry;
    }
    if (!isEdge(event)) return true;
    const where = state.slotOwner[event.label];
    return Boolean(where) && plotted.has(`${where.motion}/${where.constraint}`);
  });
  const width = $(".markers").clientWidth || 1000;
  const px = (frame) => trackFraction(frame) * width;
  const clusters = [];
  const last = {};
  kept
    .sort((a, b) => a.frame - b.frame || a.kind.localeCompare(b.kind))
    .forEach((event) => {
      const label = isEdge(event) ? state.slotOwner[event.label].constraint : event.label;
      const open = last[event.kind];
      if (open && px(event.frame) - px(open.frame) < CLUSTER_PX) {
        open.labels.push(label);
        open.until = event.frame;
        return;
      }
      last[event.kind] = {
        kind: event.kind,
        frame: event.frame,
        until: event.frame,
        labels: [label],
        causes,
      };
      clusters.push(last[event.kind]);
    });
  return clusters;
}

function markerButton(mark) {
  const marker = document.createElement("button");
  marker.className = `marker marker-${mark.kind}`;
  marker.style.left = trackLeft(mark.frame);
  const on = mark.kind === "state" ? (mark.causes.get(mark.frame) ?? []) : [];
  marker.classList.toggle("marker-transition", on.length > 0);
  marker.title =
    (mark.labels.length === 1
      ? `${mark.labels[0]} @ frame ${mark.frame}`
      : `${mark.labels.length} × ${mark.kind} @ frames ${mark.frame}–${mark.until}: ${mark.labels.join(", ")}`) +
    (on.length ? ` · on ${on.join(", ")}` : "");
  marker.onclick = () => {
    seek(mark.frame);
    revealMotion(motionAt(mark.frame));
  };
  return marker;
}

function noteMarkers() {
  return (state.notes ?? [])
    .filter((note) => note.frame !== null && note.frame !== undefined)
    .map((note) => {
      const marker = document.createElement("button");
      marker.className = "marker marker-note";
      marker.style.left = trackLeft(note.frame);
      marker.textContent = "◆";
      marker.title = `Note at frame ${note.frame}: ${note.text}`;
      marker.onclick = () => jumpToFinding(note.frame, null, null, note.end_frame);
      return marker;
    });
}

export function trackFraction(frame) {
  return frame / Math.max(1, state.replay.frames - 1);
}

function trackLeft(frame) {
  // the thumb travels inset by half its width, so markers must follow the same geometry
  return `calc(var(--thumb) / 2 + ${trackFraction(frame)} * (100% - var(--thumb)))`;
}

export function movePlayhead() {
  $(".transport").style.setProperty("--f", trackFraction(state.frame));
}

// The panel floats over the page, so the page ends above it rather than behind it.
export function reserveVideoSpace() {
  const transport = $(".transport");
  const panel = $(".videos");
  const covered = panel && !panel.hidden && !panel.classList.contains("minimized");
  // Every measurement first: writing a custom property between two reads forces a reflow.
  const transportHeight = transport?.offsetHeight ?? 0;
  const videoHeight = covered ? panel.offsetHeight + 16 : 0;
  const videoWidth = covered ? panel.offsetWidth + 16 : 0;
  const style = document.documentElement.style;
  if (transport) style.setProperty("--transport-h", `${transportHeight}px`);
  style.setProperty("--video-h", `${videoHeight}px`);
  style.setProperty("--video-w", `${videoWidth}px`);
}

// Expand and minimize, both remembered per browser; expanded is one custom property the
// video rules read.
const VIDEO_TOGGLES = [
  {
    control: ".video-max",
    className: "expanded",
    key: "motion-spec.video-expanded",
    on: { glyph: "⤡", title: "Shrink" },
    off: { glyph: "⤢", title: "Expand" },
  },
  {
    control: ".video-min",
    className: "minimized",
    key: "motion-spec.video-minimized",
    on: { glyph: "□", title: "Show the camera" },
    off: { glyph: "–", title: "Minimize" },
  },
];

function bindVideoToggles(panel) {
  VIDEO_TOGGLES.forEach(({ control, className, key, on, off }) => {
    const button = panel.querySelector(control);
    const apply = (active) => {
      panel.classList.toggle(className, active);
      button.textContent = active ? on.glyph : off.glyph;
      button.title = active ? on.title : off.title;
      writeStored(key, String(active));
      reserveVideoSpace();
    };
    button.onclick = () => apply(!panel.classList.contains(className));
    apply(readStored(key) === "true");
  });
}

// What the run recorded: one camera large, the rest alongside to swap in.
function showVideos(runPath, cameras) {
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
  bindVideoToggles(panel);
  const show = (camera) => {
    state.camera = camera;
    main.src = url(camera);
    // A video that is never played paints nothing until it is seeked.
    main.onloadedmetadata = () => {
      // Fit the box to the recording: a 3D view with the panels open is portrait, and a
      // landscape box would letterbox it.
      const portrait = main.videoHeight > main.videoWidth;
      main.style.width = portrait ? "auto" : "100%";
      main.style.height = portrait ? "var(--video-h, 34vh)" : "auto";
      reserveVideoSpace();
      syncVideo(true);
    };
    panel.querySelector(".video-name").textContent = camera;
    panel
      .querySelectorAll(".video-pick")
      .forEach((pick) => pick.setAttribute("aria-pressed", pick.dataset.camera === camera));
  };
  // Nothing to swap between with one camera: the strip would be the same view again.
  const strip = panel.querySelector(".video-strip");
  strip.hidden = cameras.length < 2;
  panel.classList.toggle("one-camera", cameras.length < 2);
  strip.replaceChildren(
    ...cameras.map((camera) => {
      const pick = document.createElement("button");
      pick.className = "video-pick";
      pick.dataset.camera = camera;
      pick.innerHTML = `<video muted playsinline preload="metadata"></video><span>${camera}</span>`;
      const thumb = pick.querySelector("video");
      thumb.src = url(camera);
      // Show something of the run rather than the black frame it opens on.
      thumb.onloadedmetadata = () => {
        thumb.currentTime = Math.min(1, thumb.duration / 2);
      };
      pick.onclick = () => show(camera);
      return pick;
    }),
  );
  show(cameras[0]);
  reserveVideoSpace();
}

// The same pane pointed at a live ROS topic, for a run with no recording: the picture is
// whatever the camera publishes now, not a track the timeline drives.
async function showRosCamera(generationPath) {
  if (!generationPath) return;
  // Only a cached generation carrying `cameras` answers here: the sidebar keeps a lighter
  // record, and reading `cameras` off that one silently finds none.
  const cached =
    state.generation?.path === generationPath && state.generation.cameras ? state.generation : null;
  const generation =
    cached ??
    (await api(`/api/generation?path=${encodeURIComponent(generationPath)}`).catch(() => null));
  // Nothing is re-read off `state` after the await: a run starting rewrites `state.replay` in
  // place, and checking it here is a race the pane used to lose silently.
  if (!generation) return;
  // Only the cameras the model gives a channel, in the order the scene declares them: a topic
  // the model does not state is not this page's to invent.
  const carried = generation.cameras?.filter((entry) => entry.topic) ?? [];
  if (!carried.length) return;
  const declaredTopic = carried[0].topic;
  const panel = $(".videos");
  panel.hidden = false;
  panel.classList.add("one-camera", "ros-camera");
  panel.querySelector(".video-strip").hidden = true;
  bindVideoToggles(panel);
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
  enhanceSelect(topic);
  const subscribe = () => {
    state.rosTopic = topic.value;
    status.textContent = "";
    name.textContent = carried.find((entry) => entry.topic === state.rosTopic)?.id ?? "";
    // A new query each time, so the browser reconnects rather than showing the stalled stream.
    frame.src = `/api/ros-camera?topic=${encodeURIComponent(state.rosTopic)}&t=${Date.now()}`;
  };
  frame.onerror = () => {
    frame.removeAttribute("src");
    // An armed run publishes nothing until it is played, so this is a wait, not a dead end.
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

// The recording of the run, if this run has one on screen.
function playingVideo() {
  const panel = $(".videos");
  const video = panel && !panel.hidden ? panel.querySelector("video") : null;
  return video && video.duration && isFinite(video.duration) ? video : null;
}

// Frames and video are two clocks over the same run: map by position, not by seconds, and
// leave a deadband so the video playing itself does not fight the cursor it is driving.
function syncVideo(force = false) {
  const video = playingVideo();
  if (!video) return;
  const target = trackFraction(state.frame) * video.duration;
  if (!force && Math.abs(video.currentTime - target) <= 0.25) return;
  // Scrubbing fires seeks faster than the decoder lands them, and each new one aborts the last
  // mid-decode. One at a time: a drag queues only its newest target.
  if (video.seeking) video._queuedSeek = target;
  else video.currentTime = target;
}

function followConsole(runPath) {
  clearInterval(state.consoleWatch);
  let offset = 0;
  const pre = $("#console-text");
  const poll = async () => {
    if (state.runPath !== runPath || !pre.isConnected) return clearInterval(state.consoleWatch);
    const slice = await api(
      `/api/console?path=${encodeURIComponent(runPath)}&offset=${offset}`,
    ).catch(() => null);
    if (!slice || !slice.text) return;
    offset = slice.offset;
    appendConsole(pre, slice.text);
    // What just arrived is part of the log the search bar is searching.
    findInConsole();
  };
  poll();
  state.consoleWatch = setInterval(poll, 1000);
}

let consoleHits = [];
let consoleHit = 0;

// Matches are painted through the CSS highlight registry, not by wrapping elements around
// them: the log is appended to while it is read, and it already carries ANSI spans.
function findInConsole(step = 0) {
  const pre = $("#console-text");
  const input = $("#console-search");
  if (!pre || !input || !CSS.highlights) return;
  const needle = input.value.toLowerCase();
  consoleHits = needle ? matchRanges(pre, needle) : [];
  consoleHit = consoleHits.length
    ? (consoleHit + step + consoleHits.length * 2) % consoleHits.length
    : 0;
  CSS.highlights.set("console-match", new Highlight(...consoleHits));
  CSS.highlights.set(
    "console-match-current",
    new Highlight(...consoleHits.slice(consoleHit, consoleHit + 1)),
  );
  $("#console-matches").textContent = needle
    ? consoleHits.length
      ? `${consoleHit + 1} / ${consoleHits.length}`
      : "no matches"
    : "";
  $("#console-prev").disabled = $("#console-next").disabled = consoleHits.length < 2;
  // Only a step is asked to move the view: re-searching as the run writes must not yank it.
  if (step && consoleHits.length) {
    const hit = consoleHits[consoleHit].getBoundingClientRect();
    pre.scrollTop += hit.top - pre.getBoundingClientRect().top - pre.clientHeight / 2;
  }
}

function matchRanges(pre, needle) {
  const found = [];
  const walk = document.createTreeWalker(pre, NodeFilter.SHOW_TEXT);
  for (let node = walk.nextNode(); node; node = walk.nextNode()) {
    const text = node.data.toLowerCase();
    // A word broken across an ANSI colour change is two text nodes and is not matched.
    for (let at = text.indexOf(needle); at !== -1; at = text.indexOf(needle, at + needle.length)) {
      const range = new Range();
      range.setStart(node, at);
      range.setEnd(node, at + needle.length);
      found.push(range);
    }
  }
  return found;
}

function bindConsoleSearch() {
  const input = $("#console-search");
  // Nothing to paint matches with: the field would take words and show nothing for them.
  if (!CSS.highlights) return input.closest(".console-bar").remove();
  input.oninput = () => {
    consoleHit = 0;
    findInConsole();
  };
  input.onkeydown = (event) => {
    if (event.key === "Escape") {
      input.value = "";
      consoleHit = 0;
      return findInConsole();
    }
    if (event.key !== "Enter") return;
    findInConsole(event.shiftKey ? -1 : 1);
  };
  $("#console-prev").onclick = () => findInConsole(-1);
  $("#console-next").onclick = () => findInConsole(1);
}
