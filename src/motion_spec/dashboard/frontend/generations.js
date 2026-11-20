// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/** The generations list, and the page one generation opens: facts, runs, devices, drift. */

import {
  consoleExcerpt,
  deviceRows,
  fact,
  fileFolder,
  fileRow,
  icon,
  iconMarkup,
  listItem,
  setPickAll,
} from "./components.js";
import { annotationEditor, NOTES_MARKUP } from "./annotations.js";
import { saveInspection } from "./inspection.js";
import {
  $,
  api,
  copyText,
  formatBytes,
  post,
  readStored,
  showError,
  snack,
  snackError,
  stampText,
  state,
  writeStored,
} from "./core.js";
import { showExplore } from "./explore.js";
import { hashView, setTab, setView, writeHash } from "./routing.js";
import { loadReplay, openPendingRun, showNotes, stopPlayback } from "./run.js";
import { openSource, showGenerated, showSource } from "./sources.js";

const RUNS_PER_PAGE = 10;

// one per model: its generations share cameras and choices
const runOptionsKey = (generationPath) =>
  `motion-spec.run-options.${generationPath.split("/")[0]}`;

export function readRunOptions(generationPath) {
  try {
    return JSON.parse(readStored(runOptionsKey(generationPath))) ?? {};
  } catch {
    return {};
  }
}

function saveRunOptions(generationPath, options) {
  writeStored(runOptionsKey(generationPath), JSON.stringify(options));
}

const SORTS = {
  size: (left, right) => right.size_bytes - left.size_bytes,
  "last-run": (left, right) => (right.last_run?.id ?? "").localeCompare(left.last_run?.id ?? ""),
  newest: (left, right) =>
    (right.created ?? right.built_at).localeCompare(left.created ?? left.built_at),
};

export async function loadGenerations(refresh = false) {
  const request = ++state.listRequest;
  if (refresh || !state.cache.generations) {
    const [generations, storage, roots] = await Promise.all([
      api("/api/generations"),
      api("/api/storage"),
      api("/api/roots"),
    ]);
    state.cache.generations = { generations, storage, roots };
  }
  // health and notebook still browse generations
  if (request !== state.listRequest || state.tab === "sources") return;
  $("#list-title").textContent = "GENERATIONS";
  const { generations, storage, roots } = state.cache.generations;
  state.roots = roots;
  bindBrowserTools();
  $("#storage").textContent = formatBytes(storage.generations_bytes);
  $("#generation-root").hidden = false;
  $("#generation-root").value = roots.logs;
  const sort = SORTS[$("#generation-sort").value] ?? SORTS.newest;
  const ordered = [...generations].sort(
    (left, right) => Number(right.pinned) - Number(left.pinned) || sort(left, right),
  );
  const groups = Map.groupBy(ordered, (generation) =>
    generation.pinned ? "★ Pinned" : generation.name,
  );
  $("#browser").replaceChildren(
    ...[...groups].map(([model, entries]) => generationGroup(model, entries, roots.logs)),
  );
  filterGenerations($("#search").value);
  highlightGeneration();
}

function generationGroup(model, entries, logsRoot) {
  const group = document.createElement("details");
  const openKey = `motion-spec.group.${logsRoot}.${model}`;
  group.className = "generation-group";
  group.open = readStored(openKey) !== "closed";
  group.ontoggle = () => {
    if (!$("#search").value) writeStored(openKey, group.open ? "open" : "closed");
  };
  const summary = document.createElement("summary");
  const label = document.createElement("span");
  label.className = "group-name";
  label.textContent = model;
  const all = document.createElement("button");
  all.className = "select-group";
  all.title = `Select every generation of ${model}`;
  all.textContent = "select all";
  all.onclick = (event) => {
    event.preventDefault();
    event.stopPropagation();
    const paths = entries
      .filter(
        (generation) =>
          !group.querySelector(`[data-path="${CSS.escape(generation.path)}"]`)?.hidden,
      )
      .map((generation) => generation.path);
    const adding = paths.some((path) => !state.selected.has(path));
    paths.forEach((path) => {
      if (state.selected.has(path) !== adding) toggleSelection(path, "sidebar");
    });
  };
  summary.append(icon("chevron-right", "fold-marker"), label, all);
  group.append(summary, ...entries.map(generationItem));
  return group;
}

function generationItem(generation) {
  const folder = generation.path.split("/").pop();
  const lastRun = lifecycleWord(generation.last_run);
  const item = listItem(
    generation.label || folder,
    [
      generation.protected ? "Protected" : null,
      generation.pinned ? generation.name : null,
      generation.label ? folder : null,
      ...generation.tags,
      ...generation.note_tags,
      generation.variant,
      stampText(generation.created ?? generation.built_at),
      `${generation.runs} runs`,
      formatBytes(generation.size_bytes),
    ]
      .filter(Boolean)
      .join(" · "),
    (event) => {
      if (event.shiftKey) return pickRange(generation.path, $("#browser"), "sidebar");
      return event.metaKey || event.ctrlKey
        ? toggleSelection(generation.path, "sidebar")
        : selectGeneration(generation.path);
    },
    generation.path,
    generation.last_run ? (generation.last_run.live ? "live" : (lastRun ?? "unknown")) : null,
  );
  item.dataset.search = [
    generation.name,
    generation.path,
    generation.label,
    ...generation.tags,
    ...generation.note_tags,
    item.textContent,
  ]
    .join(" ")
    .toLowerCase();
  item.dataset.pinned = String(generation.pinned);
  item.dataset.runtime = generation.simulated ? "simulation" : "hardware";
  item.dataset.runStatus = generation.last_run?.live ? "running" : (lastRun ?? "");
  return item;
}

// a complete run shows its verdict, any other its OSLC state
function lifecycleWord(run) {
  if (!run?.state) return null;
  return run.state === "complete" ? run.verdict : run.state;
}

export function filterGenerations(value) {
  const query = value.toLowerCase();
  const wanted = $("#generation-filter")?.value ?? "all";
  const shown = (item) =>
    wanted === "all" ||
    item.dataset.runStatus === wanted ||
    item.dataset.runtime === wanted ||
    (wanted === "pinned" && item.dataset.pinned === "true");
  document.querySelectorAll(".generation-group").forEach((group) => {
    const items = [...group.querySelectorAll(".item")];
    items.forEach((item) => {
      item.hidden = !item.dataset.search.includes(query) || !shown(item);
    });
    const visible = items.some((item) => !item.hidden);
    group.hidden = !visible;
    if (query && visible) group.open = true;
  });
}

/** Browser controls survive refreshes. */
function bindBrowserTools() {
  if ($("#generation-tools")) return;
  const tools = document.createElement("div");
  tools.id = "generation-tools";
  tools.className = "browser-tools";
  tools.innerHTML = `
    <select id="generation-filter" aria-label="Filter generations">
      <option value="all">All generations</option>
      <option value="pinned">Pinned</option>
      <option value="running">Running</option>
      <option value="failed">Failed</option>
      <option value="hardware">Hardware</option>
      <option value="simulation">Simulation</option>
    </select>
    <select id="generation-sort" aria-label="Sort generations">
      <option value="newest">Newest</option>
      <option value="last-run">Last run</option>
      <option value="size">Largest</option>
    </select>
  `;
  $("#search").parentElement.after(tools);
  for (const kind of ["filter", "sort"]) {
    const key = `motion-spec.generation-${kind}`;
    const select = tools.querySelector(`#generation-${kind}`);
    select.value = readStored(key) ?? (kind === "filter" ? "all" : "newest");
    select.onchange = () => {
      writeStored(key, select.value);
      loadGenerations().catch(showError);
    };
  }
}

export function highlightGeneration() {
  document.querySelectorAll("#browser .item").forEach((item) => {
    item.classList.toggle("selected", item.dataset.path === state.generationPath);
  });
  const selected = document.querySelector("#browser .item.selected");
  if (!selected) return;
  const list = $("#browser").getBoundingClientRect();
  const item = selected.getBoundingClientRect();
  if (item.top < list.top || item.bottom > list.bottom)
    selected.scrollIntoView({ block: "nearest" });
}

function pickRange(path, container, source) {
  // shift extends from the last item picked, over what is actually on screen
  const paths = [...container.querySelectorAll("[data-path]")]
    .filter((item) => item.offsetParent !== null)
    .map((item) => item.dataset.path);
  const from = paths.indexOf(state.anchor ?? path);
  const to = paths.indexOf(path);
  if (from < 0 || to < 0) return toggleSelection(path, source);
  const [start, end] = from < to ? [from, to] : [to, from];
  paths.slice(start, end + 1).forEach((item) => {
    if (!state.selected.has(item)) toggleSelection(item, source, false);
  });
}

function toggleSelection(path, source = "sidebar", anchor = true) {
  if (anchor) state.anchor = path;
  const picked = !state.selected.has(path);
  picked ? state.selected.add(path) : state.selected.delete(path);
  document.querySelectorAll(`[data-path="${CSS.escape(path)}"]`).forEach((item) => {
    item.classList.toggle("picked", picked);
    const box = item.querySelector(":scope > input[type=checkbox]");
    if (box) box.checked = picked;
  });
  const actions = $("#selection-actions");
  actions.dataset.side = source;
  actions.hidden = !state.selected.size;
  $("#selection-count").textContent = `${state.selected.size} selected`;
}

export async function selectGeneration(path) {
  saveInspection();
  state.anchor = path;
  clearInterval(state.liveWatch);
  clearInterval(state.consoleWatch);
  state.runPath = null;
  state.live = state.following = null;
  state.livePlots.clear();
  state.pendingSignals.clear();
  state.liveBuffer.clear();
  state.activeMotion = null;
  setView("generation", path);
  state.generationPath = path;
  if (state.generation?.path === path) {
    $("#content").replaceChildren(state.generation.node);
    highlightGeneration();
    $(".run-bar")?.refreshRun?.();
    return api(`/api/runs?path=${encodeURIComponent(path)}`).then(state.generation.setRuns);
  }
  const [generation, runs] = await Promise.all([
    api(`/api/generation?path=${encodeURIComponent(path)}`),
    api(`/api/runs?path=${encodeURIComponent(path)}`),
  ]);
  if (state.generationPath !== path || hashView().get("generation") !== path) return;
  highlightGeneration();

  const shown = $("#content .generation");
  const page = shown ?? $("#generation-template").content.cloneNode(true);
  page.querySelector("h1").textContent = generation.spec_name ?? generation.name;
  page.querySelector(".path").textContent = generation.folder;
  page.querySelector(".copy-generation-path").onclick = () => copyText(generation.folder);
  page.querySelector(".generation-description").textContent = generation.description ?? "";
  page.querySelector(".facts").innerHTML = factsMarkup(generation);
  page.querySelector(".source-files").replaceChildren(...sourceFileRows(generation, path));
  page.querySelector(".generated-files").replaceChildren(...generatedFileFolders(generation, path));
  bindGenerationConsole(page, path);
  const setRuns = bindRunList(page, generation, runs);
  bindRunAgain(page, path, generation.cameras ?? [], generation.simulated);
  bindDevices(page, path, generation.simulated);
  if (!shown) $("#content").replaceChildren(page);
  const mounted = $("#content .generation");
  const metadata = await annotationEditor(path, () => loadGenerations(true));
  if (!mounted.isConnected) return;
  mountAnnotations(mounted, metadata, path);
  state.generation = { path, node: $("#content .generation"), setRuns };
  $("#content").querySelector(".explore-link").onclick = () => showExplore(path).catch(snackError);
}

// read on open: a log is thousands of lines
function bindGenerationConsole(page, path) {
  const panel = page.querySelector(".generation-console");
  const pre = panel.querySelector("pre");
  panel.ontoggle = () => {
    if (!panel.open || panel.dataset.loaded !== undefined) return;
    panel.dataset.loaded = "";
    consoleExcerpt(path, 500)
      .then((loaded) => {
        pre.replaceChildren(...loaded.childNodes);
        if (!pre.textContent) pre.textContent = "no console log for this generation";
      })
      .catch(() => {
        pre.textContent = "could not read the console log";
      });
  };
}

function factsMarkup(generation) {
  return [
    fact("Runtime", generation.simulated ? "Simulated" : "Hardware", "fact-key"),
    fact("Platform", generation.platform, "fact-key fact-wide", generation.toolchain?.mujoco),
    fact("Backend", generation.backend, "", generation.toolchain?.wrapper),
    fact("Motions", generation.motions),
    fact("Authored constraints", generation.authored_constraints),
    fact("Runs", generation.runs),
    fact("Generated", stampText(generation.created)),
    fact("Folder size", formatBytes(generation.size_bytes)),
    fact("Schema", generation.schema_hash, "fact-mono"),
  ].join("");
}

function sourceFileRows(generation, path) {
  return generation.source_files.map((source) => {
    const row = fileRow(
      source.name,
      source.model
        ? `${source.path} — the model this was generated from`
        : `${source.path} — click to copy the path`,
      source.model ? (event) => modelMenu(row, source, path, event) : () => copyText(source.path),
    );
    if (source.model) row.classList.add("is-model");
    return row;
  });
}

function generatedFileFolders(generation, path) {
  const folders = Map.groupBy(generation.generated_files, (source) => source.name.split("/")[0]);
  return [...folders].map(([folder, files]) =>
    fileFolder(
      folder,
      files.map((source) =>
        fileRow(
          source.name.slice(folder.length + 1) || folder,
          `${source.path} — click to read, shift-click to copy the path`,
          (event) =>
            event.shiftKey
              ? copyText(source.path)
              : showGenerated(`${path}/generated/${source.name}`).catch(snackError),
        ),
      ),
    ),
  );
}

function mountAnnotations(mounted, metadata, path) {
  mounted.querySelector(".annotation-editor")?.remove();
  mounted.querySelector(".generation-notes")?.remove();
  mounted.querySelector(".generation-description").after(metadata);
  const notes = document.createElement("details");
  notes.className = "generation-notes fold";
  notes.innerHTML =
    `<summary>${iconMarkup("chevron-right", "fold-marker")}Generation notes</summary>` +
    `<div class="generation-notes-body">${NOTES_MARKUP}</div>`;
  metadata.after(notes);
  notes.ontoggle = () => {
    if (notes.open) showNotes(path, notes.querySelector(".generation-notes-body")).catch(showError);
  };
}

function modelMenu(row, source, generationPath, event) {
  event.stopPropagation();
  document.querySelector(".row-menu")?.remove();
  const menu = document.createElement("div");
  menu.className = "row-menu picker-panel";
  const missing = !source.workspace;
  const actions = [
    ["open in sources", () => showSource(source.workspace, source.path), missing],
    ["diff against source", () => showDrift(generationPath, source.name), false],
    ["copy archived path", () => copyText(source.path), false],
  ];
  menu.append(
    ...actions.map(([name, run, disabled]) => {
      const action = document.createElement("button");
      action.textContent = name;
      action.disabled = disabled;
      if (disabled) action.title = "this model is no longer in the sources tree";
      action.onclick = (click) => {
        click.stopPropagation();
        menu.remove();
        run();
      };
      return action;
    }),
  );
  row.append(menu);
  setTimeout(() => document.addEventListener("click", () => menu.remove(), { once: true }));
}

function bindRunList(page, generation, runs) {
  const list = page.querySelector(".runs");
  const pickAll = page.querySelector(".run-pick-all");
  const pager = page.querySelector(".run-pagination");
  const from = page.querySelector(".run-from");
  const to = page.querySelector(".run-to");
  const search = page.querySelector(".run-search");
  const clear = page.querySelector(".run-filter-clear");
  const count = page.querySelector(".run-count");
  const empty = document.createElement("div");
  empty.className = "runs-empty";
  empty.textContent = "No run started in that window.";
  let all = runs;
  let rows = runs;
  let runPage = 0;

  const syncPickAll = () =>
    setPickAll(pickAll, rows.filter((run) => state.selected.has(run.path)).length, rows.length);
  const countRuns = () => {
    count.textContent =
      rows.length === all.length
        ? all.length
          ? `${all.length} recorded`
          : "none yet"
        : `${rows.length} of ${all.length}`;
  };
  const renderRuns = () => {
    countRuns();
    const shown = rows.slice(runPage * RUNS_PER_PAGE, (runPage + 1) * RUNS_PER_PAGE);
    list.replaceChildren(
      ...shown.map((run, index) =>
        runRow(run, generation, runPage * RUNS_PER_PAGE + index + 1, syncPickAll),
      ),
    );
    syncPickAll();
  };
  const drawPager = () => {
    const total = Math.max(1, Math.ceil(rows.length / RUNS_PER_PAGE));
    if (total < 2) return pager.replaceChildren();
    pager.innerHTML =
      `<button aria-label="Previous page">${iconMarkup("chevron-left")}</button>` +
      `<span>${runPage + 1} / ${total}</span>` +
      `<button aria-label="Next page">${iconMarkup("chevron-right")}</button>`;
    const [back, next] = pager.querySelectorAll("button");
    back.onclick = () => {
      runPage = Math.max(0, runPage - 1);
      draw();
    };
    next.onclick = () => {
      runPage = Math.min(total - 1, runPage + 1);
      draw();
    };
  };
  const draw = () => {
    renderRuns();
    empty.remove();
    if (!rows.length && all.length) list.after(empty);
    drawPager();
  };
  const applyFilter = () => {
    const after = from.value ? new Date(from.value).getTime() : -Infinity;
    const before = to.value ? new Date(to.value).getTime() : Infinity;
    const needle = search.value.trim().toLowerCase();
    rows = all
      .filter((run) => {
        const started = run.started ? new Date(run.started).getTime() : NaN;
        const inWindow = Number.isNaN(started)
          ? !from.value && !to.value
          : started >= after && started <= before;
        if (!inWindow || !needle) return inWindow;
        return [run.id, run.label, run.notes_text, ...(run.tags ?? [])]
          .join("\n")
          .toLowerCase()
          .includes(needle);
      })
      .sort((left, right) => Number(right.pinned) - Number(left.pinned));
    clear.hidden = !from.value && !to.value && !search.value;
    runPage = 0;
    draw();
  };
  pickAll.onchange = () => {
    rows.forEach((run) => {
      if (state.selected.has(run.path) !== pickAll.checked) toggleSelection(run.path, "main");
    });
    syncPickAll();
  };
  from.onchange = to.oninput = from.oninput = to.onchange = applyFilter;
  search.oninput = applyFilter;
  clear.onclick = () => {
    from.value = to.value = search.value = "";
    applyFilter();
  };
  draw();
  return (fresh) => {
    all = fresh;
    applyFilter();
  };
}

function runRow(run, generation, number, syncPickAll) {
  const row = document.createElement("div");
  row.className = "run";
  row.dataset.path = run.path;
  row.classList.toggle("picked", state.selected.has(run.path));
  const status = lifecycleWord(run) ?? (run.complete ? "passed" : "incomplete");
  const short = run.id.replace(/^run-\d{8}T/, "").replace(/Z$/, "");
  row.innerHTML = `<input type="checkbox" class="pick" title="Select; shift-click to select a range"><span>${number}</span><strong>${short}</strong><span>${stampText(run.started)}</span><span>${run.duration_s.toFixed(2)} s</span><span>${(run.written_frames ?? 0).toLocaleString()}</span><span class="badge badge-${status}">${status}</span>`;
  row.firstChild.checked = state.selected.has(run.path);
  row.querySelector("strong").textContent =
    `${run.pinned ? "★ " : ""}${run.protected ? "Protected · " : ""}${generation.baseline === run.path ? "Baseline · " : ""}${run.label || short}`;
  row.firstChild.onclick = (event) => {
    event.stopPropagation();
    event.shiftKey
      ? pickRange(run.path, row.parentElement, "main")
      : toggleSelection(run.path, "main");
    syncPickAll();
  };
  if (run.tags?.length) {
    const note = document.createElement("div");
    note.className = "run-note";
    note.append(
      ...run.tags.map((tag) =>
        Object.assign(document.createElement("span"), { className: "run-tag", textContent: tag }),
      ),
    );
    row.append(note);
  }
  if (run.live) row.querySelector(".badge").append(" ", stopButton(run.path));
  row.title = `${run.id} — open replay; Ctrl/Cmd-click to select`;
  row.onclick = (event) => {
    if (event.shiftKey) return pickRange(run.path, row.parentElement, "main");
    return event.metaKey || event.ctrlKey
      ? toggleSelection(run.path, "main")
      : loadReplay(run.path);
  };
  return row;
}

function stopButton(runPath) {
  const stop = document.createElement("button");
  stop.className = "run-stop";
  stop.textContent = "stop";
  stop.onclick = (event) => {
    event.stopPropagation();
    stop.disabled = true;
    stopRun(runPath).catch((error) => {
      stop.disabled = false;
      snackError(error);
    });
  };
  return stop;
}

// signals the process, so it also reaches a run still connecting or hung
export function stopRun(path) {
  return post("/api/run/stop", { path });
}

export async function stopRuns(paths) {
  const results = await Promise.allSettled(paths.map(stopRun));
  const stopped = results.filter((result) => result.status === "fulfilled").length;
  const skipped = results.length - stopped;
  const count = `stopped ${stopped} run${stopped === 1 ? "" : "s"}`;
  snack(
    stopped
      ? `${count}${skipped ? `, ${skipped} not running here` : ""}`
      : "none of these runs is running here",
  );
  return stopped;
}

export function bindRunAgain(page, path, cameras, simulated) {
  const bar = page.querySelector(".run-bar");
  bar.querySelectorAll(".run-choice").forEach((choice) => {
    choice.querySelectorAll("button").forEach((option) => {
      option.onclick = () => {
        choice
          .querySelectorAll("button")
          .forEach((other) => other.setAttribute("aria-pressed", other === option));
        saveRunOptions(path, options());
      };
    });
  });
  const camera = mountCameraMenu(bar, cameras, simulated, () => saveRunOptions(path, options()));
  const options = () => ({
    ...Object.fromEntries(
      [...bar.querySelectorAll(".run-choice[data-option]")].map((choice) => [
        choice.dataset.option,
        choice.querySelector('button[aria-pressed="true"]')?.dataset.value === "true",
      ]),
    ),
    cameras: camera.chosen(),
  });
  restoreChoices(bar, path, camera.label);
  watchRun(page, bar, path, options);
}

// simulators render any camera; hardware records only those with a ROS image topic
function mountCameraMenu(bar, cameras, simulated, save) {
  const record = bar.querySelector(".run-record");
  const offered = simulated
    ? [{ id: "default", title: "standard camera view" }, ...cameras]
    : cameras.filter((camera) => camera.topic);
  const menu = document.createElement("details");
  menu.className = "picker camera-menu";
  menu.innerHTML = '<summary></summary><div class="picker-panel"></div>';
  const summary = menu.querySelector("summary");
  const chosen = () =>
    [...menu.querySelectorAll('.run-camera[aria-pressed="true"]')]
      .filter((chip) => !chip.hidden)
      .map((chip) => chip.dataset.camera);
  const label = () => {
    const available = [...menu.querySelectorAll(".run-camera")].filter((chip) => !chip.hidden);
    const picked = chosen().length;
    summary.textContent = picked ? `${picked} camera${picked === 1 ? "" : "s"}` : "none";
    record.hidden = !available.length;
    if (!available.length) menu.open = false;
  };
  menu.querySelector(".picker-panel").append(
    ...offered.map((camera) => {
      const chip = document.createElement("button");
      chip.className = "run-camera";
      chip.dataset.camera = camera.id;
      chip.setAttribute("aria-pressed", "false");
      chip.textContent = camera.id;
      chip.title = [
        camera.topic,
        camera.width ? `${camera.width}×${camera.height}` : (camera.title ?? camera.id),
      ]
        .filter(Boolean)
        .join(" · ");
      chip.onclick = () => {
        chip.setAttribute("aria-pressed", chip.getAttribute("aria-pressed") !== "true");
        label();
        save();
      };
      return chip;
    }),
  );
  record.querySelector(".camera-menu")?.remove();
  record.append(menu);
  label();
  return { chosen, label };
}

function restoreChoices(bar, path, label) {
  const remembered = readRunOptions(path);
  bar.querySelectorAll(".run-choice[data-option]").forEach((choice) => {
    const value = remembered[choice.dataset.option];
    if (typeof value !== "boolean") return;
    choice
      .querySelectorAll("button")
      .forEach((option) =>
        option.setAttribute("aria-pressed", option.dataset.value === String(value)),
      );
  });
  if (Array.isArray(remembered.cameras)) {
    bar
      .querySelectorAll(".run-camera")
      .forEach((chip) =>
        chip.setAttribute("aria-pressed", remembered.cameras.includes(chip.dataset.camera)),
      );
    label();
  }
}

function watchRun(page, bar, path, options) {
  const status = bar.querySelector(".run-state");
  const start = bar.querySelector(".run-start");
  const live = page.querySelector(".run-live");
  const failed = page.querySelector(".run-console");
  // written in place, so this may hold the last generation's failure
  failed.hidden = true;
  failed.replaceChildren();
  live.replaceChildren();
  const watched = new Set(); // runs seen here, so each ending is reported once
  const check = async () => {
    const answer = await api(`/api/run?path=${encodeURIComponent(path)}`).catch(() => null);
    if (!answer) return;
    const running = answer.runs.filter((run) => run.running);
    running.forEach((run) => watched.add(run.path));
    live.replaceChildren(...running.map(liveRow));
    if (running.length > 1) live.append(stopAllRow(running.map((run) => run.path)));
    for (const run of answer.runs) {
      if (run.running || !watched.delete(run.path)) continue;
      status.textContent = run.stopped
        ? `${run.id} stopped`
        : run.exit_code
          ? `${run.id} exited ${run.exit_code}`
          : `${run.id} finished`;
      // a stopped run exits non-zero too
      if (run.exit_code && !run.stopped) showPostMortem(failed, path);
      // the run's own page may have announced it already
      if (!state.announced.has(run.path)) {
        snack(run.exit_code ? `run failed (${run.exit_code})` : "run finished");
      }
      state.announced.add(run.path);
      const setRuns = state.generation?.setRuns;
      if (setRuns) {
        api(`/api/runs?path=${encodeURIComponent(path)}`)
          .then(setRuns)
          .catch(() => {});
      }
    }
    if (!running.length) clearInterval(state.runWatch);
  };
  const watch = () => {
    clearInterval(state.runWatch);
    state.runWatch = setInterval(check, 2000);
    check();
  };
  start.onclick = async () => {
    start.disabled = true;
    status.textContent = "starting…";
    try {
      const started = await post("/api/run", { path, options: options() });
      status.textContent = started.recording?.length
        ? `recording ${started.recording.join(", ")}`
        : "";
      watched.add(started.path);
      watch();
      await openPendingRun(started.run);
    } catch (error) {
      status.textContent = error.message;
    }
    start.disabled = false;
  };
  bar.refreshRun = check;
  watch();
}

function stopAllRow(paths) {
  const row = document.createElement("div");
  row.className = "live-run";
  const stop = document.createElement("button");
  stop.className = "run-stop";
  stop.textContent = `stop all ${paths.length}`;
  stop.onclick = () => {
    stop.disabled = true;
    stopRuns(paths).finally(() => (stop.disabled = false));
  };
  row.append(stop);
  return row;
}

function liveRow(run) {
  const row = document.createElement("div");
  row.className = "live-run";
  row.innerHTML = `<strong></strong><span>running · pid ${run.pid}</span>`;
  row.querySelector("strong").textContent = run.id;
  row.querySelector("strong").onclick = () => openPendingRun(run.path);
  row.append(stopButton(run.path));
  return row;
}

// swallowed: the status line already said how the run ended
function showPostMortem(failed, path) {
  consoleExcerpt(path)
    .then((pre) => {
      failed.replaceChildren(...pre.childNodes);
      failed.hidden = !failed.textContent;
    })
    .catch(() => {
      failed.hidden = true;
    });
}

// probe only on request: each probe is a connection attempt
function bindDevices(page, path, simulated) {
  const panel = page.querySelector(".devices");
  const rows = panel.querySelector(".device-rows");
  const status = panel.querySelector(".devices-state");
  rows.replaceChildren();
  status.textContent = "";
  panel.hidden = simulated;
  if (simulated) return;
  panel.querySelector(".devices-test").onclick = async () => {
    status.textContent = "testing…";
    rows.replaceChildren();
    try {
      const report = await api(`/api/devices?path=${encodeURIComponent(path)}`);
      status.textContent = report.config ? "" : "this generation archived no robot.toml";
      rows.replaceChildren(...report.devices.flatMap(deviceRows));
      panel.scrollIntoView({ behavior: "smooth", block: "nearest" });
    } catch (error) {
      status.textContent = error.message;
    }
  };
}

export async function showGitDiff(source, push = true) {
  state.viewing = null;
  if (push) {
    writeHash({
      drop: ["run", "generation", "source", "diff", "file", "panel"],
      set: { gitdiff: source, tab: "sources" },
      replace: hashView().get("gitdiff") === source,
    });
  }
  const diff = await api(`/api/source-diff?path=${encodeURIComponent(source)}`).catch((error) => ({
    error: error.message,
  }));
  $("#content").replaceChildren(
    driftPage(diff, {
      left: diff.archived ? `committed · ${diff.archived}` : "committed",
      right: diff.workspace_path ? `working tree · ${diff.workspace_path}` : "working tree",
      unchanged: "Unchanged — the working tree matches the last commit.",
      back: () => openSource(source),
    }),
  );
}

export async function showDrift(generationPath, file = null, push = true) {
  state.anchor = generationPath;
  stopPlayback();
  state.runPath = null;
  state.generationPath = generationPath;
  if (push) {
    const view = hashView();
    writeHash({
      drop: ["run", "generation", "source", "panel"],
      set: { diff: generationPath, file: file || null, tab: "logs" },
      replace: view.get("diff") === generationPath && view.get("file") === file,
    });
  }
  const query = new URLSearchParams({ path: generationPath });
  if (file) query.set("file", file);
  const pending = api(`/api/source-drift?${query}`).catch((error) => ({ error: error.message }));
  const summary = await api(
    `/api/source-drift?path=${encodeURIComponent(generationPath)}&summary=1`,
  ).catch(() => ({ files: [] }));
  renderDriftList(generationPath, summary.files ?? [], file);
  const drift = await pending;
  $("#content").replaceChildren(
    driftPage(drift, {
      left: drift.archived ? `archived · ${drift.archived}` : "archived",
      right: drift.workspace_path
        ? `authored now · ${drift.workspace_path}`
        : "not in the sources tree",
      missing:
        "This model is no longer in the sources tree, so there is nothing to compare it with.",
      unchanged: "Unchanged — generating again would start from the same model.",
      back: async () => {
        await loadGenerations(true).catch(() => {});
        selectGeneration(generationPath);
      },
    }),
  );
}

function driftPage(drift, { left, right, missing, unchanged, back }) {
  const page = document.createElement("article");
  page.className = "diff-page";
  page.innerHTML =
    `<div class="page-heading"><button id="back" title="Back" aria-label="Back">${iconMarkup("arrow-left")}</button>` +
    '<h1></h1><span class="eyebrow">DIFF</span></div><p class="diff-state"></p>' +
    '<div class="diff-columns"><div class="diff-head"></div><div class="diff-head"></div></div>' +
    '<div class="diff-body"></div>';
  page.querySelector("h1").textContent = drift.name ?? "model";
  const [leftHead, rightHead] = page.querySelectorAll(".diff-head");
  leftHead.textContent = left;
  rightHead.textContent = right;
  page.querySelector(".diff-state").textContent = drift.error
    ? drift.error
    : !drift.workspace
      ? (missing ?? "There is nothing to compare this with.")
      : drift.same
        ? unchanged
        : `${drift.rows.filter((row) => row.kind !== "equal").length} lines differ.`;
  page.querySelector(".diff-body").replaceChildren(...(drift.rows ?? []).map(diffRow));
  page.querySelector("#back").onclick = back;
  return page;
}

function diffRow(row) {
  const line = document.createElement("div");
  line.className = "diff-row";
  line.dataset.kind = row.kind;
  for (const [side, cell] of [
    ["left", row.left],
    ["right", row.right],
  ]) {
    const number = document.createElement("span");
    number.className = "diff-n";
    number.textContent = cell ? cell.n : "";
    const text = document.createElement("span");
    text.className = `diff-text diff-${side}`;
    text.textContent = cell ? cell.text : "";
    if (!cell) text.dataset.blank = "true";
    line.append(number, text);
  }
  return line;
}

function renderDriftList(generationPath, files, selected) {
  $("#list-title").textContent = "RUN SOURCES";
  $("#storage").textContent = "";
  const chosen = selected ?? files.find((file) => file.model)?.name;
  $("#browser").replaceChildren(
    ...files.map((file) => {
      const item = document.createElement("button");
      item.className = "item drift-item";
      item.dataset.status = file.status;
      item.classList.toggle("selected", file.name === chosen);
      const name = document.createElement("span");
      name.className = "item-name";
      name.textContent = file.name;
      const mark = document.createElement("small");
      mark.textContent =
        { missing: "not in sources", changed: "changed" }[file.status] ?? "unchanged";
      item.append(name, mark);
      item.onclick = () => showDrift(generationPath, file.name);
      return item;
    }),
  );
}

export function showGeneration(path, run = false) {
  setTab("logs");
  loadGenerations(true)
    .then(() => selectGeneration(path))
    .then(() => {
      if (run) $(".run-bar .run-start")?.click();
    })
    .catch(showError);
}
