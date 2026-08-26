// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * The generations tab: the list on the left, and the page one generation opens --
 * its facts, its runs, its devices, and what its sources have become since.
 */

import { consoleExcerpt, deviceRows, fact, listItem } from "./components.js";
import { $, api, copyText, formatBytes, post, showError, snack, stampText, state } from "./core.js";
import { showExplore } from "./explore.js";
import { setTab, setView } from "./routing.js";
import { filter, loadReplay, openPendingRun, showSpeed, stopPlayback } from "./run.js";
import { openSource, showSource } from "./sources.js";

export async function loadGenerations(refresh = false) {
  const request = ++state.listRequest;
  if (refresh || !state.cache.generations) {
    const [generations, storage, roots] = await Promise.all([api("/api/generations"), api("/api/storage"), api("/api/roots")]);
    state.cache.generations = { generations, storage, roots };
  }
  // Only the sources tab takes the sidebar away; health and notebook still browse generations,
  // so asking for "logs" here drops the list on the floor and leaves the sidebar blank.
  if (request !== state.listRequest || state.tab === "sources") return;
  $("#list-title").textContent = "GENERATIONS";
  const { generations, storage, roots } = state.cache.generations;
  state.roots = roots;
  $("#storage").textContent = formatBytes(storage.generations_bytes);
  $("#generation-root").hidden = false;
  $("#generation-root").value = roots.logs;
  const groups = new Map();
  generations.forEach((generation) => groups.set(generation.name, [...(groups.get(generation.name) ?? []), generation]));
  $("#browser").replaceChildren(...[...groups].map(([model, entries]) => {
    const group = document.createElement("details");
    group.className = "generation-group";
    group.open = true;
    const summary = document.createElement("summary");
    const label = document.createElement("span");
    label.className = "group-name";
    label.textContent = model;
    summary.append(label);
    const all = document.createElement("button");
    all.className = "select-group";
    all.title = `Select every generation of ${model}`;
    all.textContent = "select all";
    all.onclick = (event) => {
      event.preventDefault();
      event.stopPropagation();
      const paths = entries.map((generation) => generation.path);
      const adding = paths.some((path) => !state.selected.has(path));
      paths.forEach((path) => {
        if (state.selected.has(path) !== adding) toggleSelection(path, "sidebar");
      });
    };
    summary.append(all);
    group.append(summary, ...entries.map((generation) => listItem(
      // The folder as it is on disk; when it was made is the line under it, not this one.
      generation.path.split("/").pop(),
      [generation.variant, stampText(generation.built_at),
       `${generation.runs} runs`, formatBytes(generation.size_bytes)].filter(Boolean).join(" · "),
      (event) => {
        if (event.shiftKey) return pickRange(generation.path, $("#browser"), "sidebar");
        return event.metaKey || event.ctrlKey
          ? toggleSelection(generation.path, "sidebar")
          : selectGeneration(generation.path);
      },
      generation.path,
    )));
    return group;
  }));
  filterGenerations($("#search").value);
  highlightGeneration();
}

export function filterGenerations(value) {
  const query = value.toLowerCase();
  document.querySelectorAll(".generation-group").forEach((group) => {
    let visible = false;
    group.querySelectorAll(".item").forEach((item) => {
      item.hidden = !item.textContent.toLowerCase().includes(query);
      visible ||= !item.hidden;
    });
    group.hidden = !visible;
    if (query && visible) group.open = true;
  });
}

// End a run the way a process ends. The transport's cancel asks the loop to stop, which only a
// loop already ticking can hear; this reaches a run that is still connecting, or hung.
export function stopRun(path) {
  return post("/api/run/stop", { path });
}

export function bindRunAgain(page, path, cameras, simulated) {
  const bar = page.querySelector(".run-bar");
  const state_ = bar.querySelector(".run-state");
  const start = bar.querySelector(".run-start");
  const halt = bar.querySelector(".run-stop");
  const failed = page.querySelector(".run-console");
  const devices = page.querySelector(".devices");
  // Each option is a choice between two named states, not a flag to guess the meaning of.
  bar.querySelectorAll(".run-choice").forEach((choice) => {
    choice.querySelectorAll("button").forEach((option) => {
      option.onclick = () => choice.querySelectorAll("button").forEach((other) =>
        other.setAttribute("aria-pressed", other === option));
    });
  });
  // Hardware takes neither: the CLI rejects a headless real run, and recording one comes later.
  const headless = bar.querySelector('.run-choice[data-option="headless"]');
  headless.hidden = !simulated;
  if (state.restricted) {
    // A GUI window opens on this machine's display, not the LAN viewer's -- nothing here
    // could watch it or close it, so the server forces headless regardless; show that
    // instead of a GUI choice that would quietly do something else.
    const gui = headless.querySelector('button[data-value="false"]');
    gui.disabled = true;
    gui.title = "GUI is disabled for network access: it would open on this machine's display, not yours";
    gui.setAttribute("aria-pressed", "false");
    headless.querySelector('button[data-value="true"]').setAttribute("aria-pressed", "true");
  }
  // Only a run with no window to pace it has a speed to choose; a GUI run is realtime already.
  const speed = bar.querySelector(".run-speed");
  const showSpeed = () => {
    speed.hidden = !simulated
      || headless.querySelector('button[aria-pressed="true"]')?.dataset.value !== "true";
  };
  headless.querySelectorAll("button").forEach((option) => option.addEventListener("click", showSpeed));
  showSpeed();
  let chosen = () => [];
  if (simulated) {
    // The cameras the model declares, plus the standard view, which needs no declaring.
    // A list that opens: a scene can declare more cameras than a bar has room for.
    const record = bar.querySelector(".run-record");
    const offered = [{ id: "default", title: "standard camera view" }, ...cameras];
    const menu = document.createElement("details");
    menu.className = "picker camera-menu";
    menu.innerHTML = '<summary></summary><div class="picker-panel"></div>';
    const summary = menu.querySelector("summary");
    chosen = () => [...menu.querySelectorAll('.run-camera[aria-pressed="true"]')]
      .filter((chip) => !chip.hidden).map((chip) => chip.dataset.camera);
    const label = () => {
      const available = [...menu.querySelectorAll(".run-camera")].filter((chip) => !chip.hidden);
      const picked = chosen().length;
      summary.textContent = picked ? `${picked} camera${picked === 1 ? "" : "s"}` : "none";
      // Nothing to record from is not an empty menu to open: the whole control goes away.
      record.hidden = !available.length;
      if (!available.length) menu.open = false;
    };
    menu.querySelector(".picker-panel").append(...offered.map((camera) => {
      const chip = document.createElement("button");
      chip.className = "run-camera";
      chip.dataset.camera = camera.id;
      chip.setAttribute("aria-pressed", "false");
      chip.textContent = camera.id;
      chip.title = camera.width ? `${camera.width}×${camera.height}` : camera.title ?? camera.id;
      chip.onclick = () => {
        chip.setAttribute("aria-pressed", chip.getAttribute("aria-pressed") !== "true");
        label();
      };
      return chip;
    }));
    record.append(menu);
    label();
  }

  // Every named choice, hardware included: only the display, the speed and the cameras are a
  // simulator's alone, and the server drops those for a real run. Whether to log is not.
  const options = () => ({
    ...Object.fromEntries(
      [...bar.querySelectorAll(".run-choice[data-option]")].map((choice) => [
        choice.dataset.option,
        choice.querySelector('button[aria-pressed="true"]')?.dataset.value === "true",
      ]),
    ),
    ...(simulated ? { cameras: chosen() } : {}),
  });
  // While it runs the page cannot say more than the runner does; watch until it stops, then
  // put the run it made in the list.
  let sawRunning = false;
  const check = async () => {
    const status = await api(`/api/run?path=${encodeURIComponent(path)}`).catch(() => null);
    if (!status) return;
    if (status.running) {
      sawRunning = true;
      start.disabled = true;
      failed.hidden = true;
      halt.hidden = false;
      state_.textContent = status.pid ? `running · pid ${status.pid}` : "running";
      return;
    }
    clearInterval(state.runWatch);
    start.disabled = false;
    halt.hidden = true;
    state_.textContent = sawRunning
      ? (status.exit_code ? `exited ${status.exit_code}` : "run finished")
      : "";
    // A run that failed says why here rather than sending the reader to a file.
    if (sawRunning && status.exit_code) {
      consoleExcerpt(path).then((pre) => {
        failed.replaceChildren(...pre.childNodes);
        failed.hidden = !failed.textContent;
      });
    }
    api(`/api/runs?path=${encodeURIComponent(path)}`).then(state.generation?.setRuns);
    // Only an ending this page watched happen is news, and the run's own page may have said
    // it already, seconds earlier.
    if (sawRunning && !state.announced) {
      snack(status.exit_code ? `run failed (${status.exit_code})` : "run finished");
    }
    if (sawRunning) state.announced = true;
    sawRunning = false;
  };
  halt.onclick = async () => {
    halt.disabled = true;
    state_.textContent = "stopping…";
    try {
      await stopRun(path);
    } catch (error) {
      state_.textContent = error.message;
    }
    halt.disabled = false;
    check();
  };
  page.querySelector(".run-bar").refreshRun = check;
  const watch = () => {
    clearInterval(state.runWatch);
    // the run directory exists as it starts, so look once now
    check();
    state.runWatch = setInterval(check, 2000);
  };
  start.onclick = async () => {
    start.disabled = true;
    state_.textContent = "starting…";
    state.announced = false;   // this run has not reported its ending yet
    try {
      const started = await post("/api/run", { path, options: options() });
      state_.textContent = started.recording?.length
        ? `running · pid ${started.pid} · recording ${started.recording.join(", ")}`
        : `running · pid ${started.pid}`;
      watch();
      // The server named the run before anything is on disk: open its page now and let it
      // wait for the log, so nothing of the run happens off-screen.
      await openPendingRun(started.run);
    } catch (error) {
      start.disabled = false;
      state_.textContent = error.message;
      // A run refused over the wire already probed it: show that where devices are reported.
      if (error.devices) devices?.showReport(error);
    }
  };
  api(`/api/run?path=${encodeURIComponent(path)}`).then((status) => {
    if (!status.running) return;
    sawRunning = true;
    start.disabled = true;
    state_.textContent = `running · pid ${status.pid}`;
    watch();
  }).catch(() => {});
}

// A hardware generation names every endpoint it drives, so the wire can be tested here rather
// than failing into the run console. Only when asked: a probe is a connection attempt.
export function bindDevices(page, path) {
  const panel = page.querySelector(".devices");
  const rows = panel.querySelector(".device-rows");
  const state_ = panel.querySelector(".devices-state");
  panel.hidden = false;
  // A probe from anywhere lands here: the refused run's is the same report this button asks for.
  panel.showReport = (report) => {
    state_.textContent = report.config ? "" : "this generation archived no robot.toml";
    rows.replaceChildren(...report.devices.flatMap(deviceRows));
    panel.scrollIntoView({ behavior: "smooth", block: "nearest" });
  };
  panel.querySelector(".devices-test").onclick = async () => {
    state_.textContent = "testing…";
    rows.replaceChildren();
    try {
      panel.showReport(await api(`/api/devices?path=${encodeURIComponent(path)}`));
    } catch (error) {
      state_.textContent = error.message;
    }
  };
}

export async function selectGeneration(path) {
  state.anchor = path;
  clearInterval(state.liveWatch);   // following a live run belongs to the run page that left
  clearInterval(state.consoleWatch);
  state.runPath = null;
  state.live = state.following = null;
  state.livePlots.clear();
  state.pendingSignals.clear();
  state.liveBuffer.clear();
  state.activeMotion = null;
  setView("generation", path);
  state.generationPath = path;
  // Coming back to a generation already built: put it back and refresh what can have changed,
  // rather than tearing the page down and laying it out again around the same facts.
  if (state.generation?.path === path) {
    $("#content").replaceChildren(state.generation.node);
    highlightGeneration();
    // This page was put aside mid-run; what it says about that run is only what was true then.
    $(".run-bar")?.refreshRun?.();
    return api(`/api/runs?path=${encodeURIComponent(path)}`).then(state.generation.setRuns);
  }
  let [generation, runs] = await Promise.all([
    api(`/api/generation?path=${encodeURIComponent(path)}`),
    api(`/api/runs?path=${encodeURIComponent(path)}`),
  ]);
  highlightGeneration();

  const page = $("#generation-template").content.cloneNode(true);
  page.querySelector("h1").textContent = generation.spec_name ?? generation.name;
  page.querySelector(".path").textContent = generation.folder;
  page.querySelector(".copy-generation-path").onclick = () => copyText(generation.folder);
  page.querySelector(".generation-description").textContent = generation.description ?? "";
  page.querySelector(".facts").innerHTML = [
    // What a run does first: where it runs and on what. Then what it is made of, then its
    // bookkeeping -- read left to right, the page answers "what will run" before "how big".
    fact("Runtime", generation.simulated ? "Simulated" : "Hardware", "fact-key"),
    // The version belongs to the thing it versions, not to a box of its own.
    // Wider than the rest: a simulation is one name, hardware is every device it deploys.
    fact("Platform", generation.platform, "fact-key fact-wide", generation.toolchain?.mujoco),
    fact("Backend", generation.backend, "", generation.toolchain?.wrapper),
    fact("Motions", generation.motions),
    fact("Authored constraints", generation.authored_constraints),
    fact("Runs", generation.runs),
    fact("Generated", stampText(generation.created)),
    fact("Folder size", formatBytes(generation.size_bytes)),
    fact("Schema", generation.schema_hash, "fact-mono"),
  ].join("");
  page.querySelector(".source-files").replaceChildren(...generation.source_files.map((source) => {
    const entry = document.createElement("div");
    // The .robmot this generation was made from is marked here rather than named twice.
    entry.className = source.model ? "source-file is-model" : "source-file";
    // The model is the file `gen` is pointed at again: reading it, generating from it and
    // seeing what it has become since are all things to do with it, so clicking it asks
    // which. Its imports are archived copies -- their path is all there is to take.
    entry.title = source.model
      ? `${source.path} — the model this was generated from`
      : `${source.path} — click to copy the path`;
    entry.innerHTML = `<span>${source.name}</span>`;
    entry.onclick = source.model
      ? (event) => modelMenu(entry, source, path, event)
      : () => copyText(source.path);
    return entry;
  }));
  const generatedGroups = new Map();
  generation.generated_files.forEach((source) => {
    const [folder, ...rest] = source.name.split("/");
    generatedGroups.set(folder, [...(generatedGroups.get(folder) ?? []), { ...source, name: rest.join("/") || folder }]);
  });
  page.querySelector(".generated-files").replaceChildren(...[...generatedGroups].map(([folder, files]) => {
    const group = document.createElement("details");
    group.className = "generated-folder";
    const summary = document.createElement("summary");
    summary.textContent = `${folder} · ${files.length}`;
    group.append(summary, ...files.map((source) => {
      const entry = document.createElement("div");
      entry.className = "source-file";
      // Generated output, not a source: there is nothing in the tree to open it in.
      entry.title = `${source.path} — click to copy the path`;
      entry.innerHTML = `<span>${source.name}</span>`;
      entry.onclick = () => copyText(source.path);
      return entry;
    }));
    return group;
  }));
  const runList = page.querySelector(".runs");
  let runPage = 0;
  let rows = runs;
  const countRuns = () => {
    const label = page.querySelector(".run-count");
    if (!label) return;
    const all = runs.length;
    label.textContent = rows.length === all
      ? (all ? `${all} recorded` : "none yet")
      : `${rows.length} of ${all}`;
  };
  const renderRuns = () => (countRuns(), runList.replaceChildren(...rows.slice(runPage * 10, runPage * 10 + 10).map((run, index) => {
    const row = document.createElement("div");
    row.className = "run";
    row.dataset.path = run.path;
    row.classList.toggle("picked", state.selected.has(run.path));
    const status = run.status ?? (run.complete ? "COMPLETED" : "INCOMPLETE");
    // Every id begins `run-<date>T`; what distinguishes one row from the next is the time.
    const short = run.id.replace(/^run-\d{8}T/, "").replace(/Z$/, "");
    row.innerHTML = `<span>${runPage * 10 + index + 1}</span><strong>${short}</strong><span>${stampText(run.started)}</span><span>${run.duration_s.toFixed(2)} s</span><span>${(run.written_frames ?? 0).toLocaleString()}</span><span class="badge badge-${status.toLowerCase()}">${status}</span>`;
    row.title = `${run.id} — open replay; Ctrl/Cmd-click to select`;
    row.onclick = (event) => {
      if (event.shiftKey) return pickRange(run.path, row.parentElement, "main");
      return event.metaKey || event.ctrlKey
        ? toggleSelection(run.path, "main")
        : loadReplay(run.path);
    };
    return row;
  })));
  const pager = page.querySelector(".run-pagination");
  // Pages follow whatever the filter left, so they are counted per draw, not once.
  const drawPager = () => {
    const total = Math.max(1, Math.ceil(rows.length / 10));
    if (total < 2) return pager.replaceChildren();
    pager.innerHTML = `<button>‹</button><span>${runPage + 1} / ${total}</span><button>›</button>`;
    const [back, next] = pager.querySelectorAll("button");
    back.onclick = () => { runPage = Math.max(0, runPage - 1); draw(); };
    next.onclick = () => { runPage = Math.min(total - 1, runPage + 1); draw(); };
  };
  const empty = document.createElement("div");
  empty.className = "runs-empty";
  empty.textContent = "No run started in that window.";
  const draw = () => {
    renderRuns();
    empty.remove();
    if (!rows.length && runs.length) runList.after(empty);
    drawPager();
  };
  // The list is what someone reaches for when they know roughly when a run happened.
  const from = page.querySelector(".run-from");
  const to = page.querySelector(".run-to");
  const clear = page.querySelector(".run-filter-clear");
  const applyFilter = () => {
    const after = from.value ? new Date(from.value).getTime() : -Infinity;
    const before = to.value ? new Date(to.value).getTime() : Infinity;
    rows = runs.filter((run) => {
      const started = run.started ? new Date(run.started).getTime() : NaN;
      return Number.isNaN(started) ? !from.value && !to.value : started >= after && started <= before;
    });
    clear.hidden = !from.value && !to.value;
    runPage = 0;
    draw();
  };
  from.onchange = to.oninput = from.oninput = to.onchange = applyFilter;
  clear.onclick = () => { from.value = to.value = ""; applyFilter(); };
  draw();
  bindRunAgain(page, path, generation.cameras ?? [], generation.simulated);
  if (!generation.simulated) bindDevices(page, path);
  $("#content").replaceChildren(page);
  state.generation = {
    path,
    folder: generation.folder,
    // The run page asks this to decide between the recording and the live ROS camera.
    simulated: generation.simulated,
    node: $("#content .generation"),
    setRuns: (fresh) => {
      runs = fresh;
      applyFilter();
    },
  };
  // The file picker is gone: a query is the lens now, and file boundaries are not something
  // the model graph has. The entry point stays, pointed at Explore with the whole graph.
  $("#content").querySelector(".explore-link").onclick = () =>
    showExplore(path).catch((error) => snack(error.message));
}

export function highlightGeneration() {
  document.querySelectorAll("#browser .item").forEach((item) => {
    item.classList.toggle("selected", item.dataset.path === state.generationPath);
  });
  // only bring it into view if it is not already there; scrolling a visible item is disorienting
  const selected = document.querySelector("#browser .item.selected");
  if (!selected) return;
  const list = $("#browser").getBoundingClientRect();
  const item = selected.getBoundingClientRect();
  if (item.top < list.top || item.bottom > list.bottom) selected.scrollIntoView({ block: "nearest" });
}

export function pickRange(path, container, source) {
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

export function toggleSelection(path, source = "sidebar", anchor = true) {
  if (anchor) state.anchor = path;
  state.selected.has(path) ? state.selected.delete(path) : state.selected.add(path);
  document.querySelectorAll("[data-path]").forEach((item) => {
    if (item.dataset.path === path) item.classList.toggle("picked", state.selected.has(path));
  });
  const button = $("#delete-selected");
  $("#selection-actions").dataset.side = source;
  $("#selection-actions").hidden = !state.selected.size;
  $("#selection-count").textContent = `${state.selected.size} selected`;
  button.textContent = "Delete";
}

// A file named on a generation page, read on the Sources tab where files are read. The path
// is resolved against whichever root holds it, since a generation's copies are not under the
// sources root; anything the viewer cannot open falls back to copying its path.
// What there is to do with the model a generation was built from. Anchored under the row it
// belongs to, and closed by the next click anywhere -- the page's other menus behave so.
export function modelMenu(row, source, generationPath, event) {
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
  menu.append(...actions.map(([name, run, disabled]) => {
    const action = document.createElement("button");
    action.textContent = name;
    action.disabled = disabled;
    if (disabled) action.title = "this model is no longer in the sources tree";
    action.onclick = (click) => { click.stopPropagation(); menu.remove(); run(); };
    return action;
  }));
  row.append(menu);
  setTimeout(() => document.addEventListener("click", function away() {
    menu.remove();
    document.removeEventListener("click", away);
  }, { once: true }));
}

// The same page for the same question asked of git: what this file has become since it was
// last committed. Reuses the drift page whole -- the reply carries the same rows.
export async function showGitDiff(source, push = true) {
  state.viewing = null;
  if (push) {
    const view = new URLSearchParams(location.hash.slice(1));
    const unchanged = view.get("gitdiff") === source;
    view.delete("run");
    view.delete("generation");
    view.delete("source");
    view.delete("diff");
    view.delete("file");
    view.delete("panel");
    view.set("gitdiff", source);
    view.set("tab", "sources");
    history[unchanged ? "replaceState" : "pushState"](null, "", `#${view}`);
  }
  const diff = await api(`/api/source-diff?path=${encodeURIComponent(source)}`)
    .catch((error) => ({ error: error.message }));
  $("#content").replaceChildren(driftPage(diff, {
    left: diff.archived ? `committed · ${diff.archived}` : "committed",
    right: diff.workspace_path ? `working tree · ${diff.workspace_path}` : "working tree",
    unchanged: "Unchanged — the working tree matches the last commit.",
    back: () => openSource(source),
  }));
}

// What the model has become since this generation was made: a page of its own, the archived
// copy beside the file as it is authored now.
export async function showDrift(generationPath, file = null, push = true) {
  state.anchor = generationPath;
  stopPlayback();
  state.runPath = null;
  state.generationPath = generationPath;
  state.diffPath = generationPath;
  if (push) {
    const view = new URLSearchParams(location.hash.slice(1));
    const unchanged = view.get("diff") === generationPath && view.get("file") === file;
    view.delete("run");
    view.delete("generation");
    view.delete("source");
    view.delete("panel");
    view.set("diff", generationPath);
    if (file) view.set("file", file); else view.delete("file");
    view.set("tab", "logs");
    history[unchanged ? "replaceState" : "pushState"](null, "", `#${view}`);
  }
  // The run's own sources take over the list while its differences are what is being read.
  const summary = await api(
    `/api/source-drift?path=${encodeURIComponent(generationPath)}&summary=1`,
  ).catch(() => ({ files: [] }));
  renderDriftList(generationPath, summary.files ?? [], file);
  const query = new URLSearchParams({ path: generationPath });
  if (file) query.set("file", file);
  const drift = await api(`/api/source-drift?${query}`).catch((error) => ({ error: error.message }));
  $("#content").replaceChildren(driftPage(drift, {
    left: drift.archived ? `archived · ${drift.archived}` : "archived",
    right: drift.workspace_path ? `authored now · ${drift.workspace_path}` : "not in the sources tree",
    missing: "This model is no longer in the sources tree, so there is nothing to compare it with.",
    unchanged: "Unchanged — generating again would start from the same model.",
    back: async () => {
      state.diffPath = null;
      await loadGenerations(true).catch(() => {});
      selectGeneration(generationPath);
    },
  }));
}

// One page for both diffs: the reply says what the two sides are, this says what to call them
// and where the back button goes.
export function driftPage(drift, { left, right, missing, unchanged, back }) {
  const page = document.createElement("article");
  page.className = "diff-page";
  page.innerHTML = '<div class="page-heading"><button id="back" title="Back">←</button>'
    + '<h1></h1><span class="eyebrow">DIFF</span></div><p class="diff-state"></p>'
    + '<div class="diff-columns"><div class="diff-head"></div><div class="diff-head"></div></div>'
    + '<div class="diff-body"></div>';
  page.querySelector("h1").textContent = drift.name ?? "model";
  const [leftHead, rightHead] = page.querySelectorAll(".diff-head");
  leftHead.textContent = left;
  rightHead.textContent = right;
  page.querySelector(".diff-state").textContent = drift.error
    ? drift.error
    : !drift.workspace
      ? missing ?? "There is nothing to compare this with."
      : drift.same
        ? unchanged
        : `${drift.rows.filter((row) => row.kind !== "equal").length} lines differ.`;
  page.querySelector(".diff-body").replaceChildren(...(drift.rows ?? []).map((row) => {
    const line = document.createElement("div");
    line.className = "diff-row";
    line.dataset.kind = row.kind;
    for (const [side, cell] of [["left", row.left], ["right", row.right]]) {
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
  }));
  page.querySelector("#back").onclick = back;
  return page;
}

// The generation's own sources, in the sidebar, each saying whether the working tree still
// matches what this run was built from.
export function renderDriftList(generationPath, files, selected) {
  $("#list-title").textContent = "RUN SOURCES";
  $("#storage").textContent = "";
  const chosen = selected ?? files.find((file) => file.model)?.name;
  $("#browser").replaceChildren(...files.map((file) => {
    const item = document.createElement("button");
    item.className = "item drift-item";
    item.dataset.status = file.status;
    item.classList.toggle("selected", file.name === chosen);
    const name = document.createElement("span");
    name.className = "item-name";
    name.textContent = file.name;
    const mark = document.createElement("small");
    mark.textContent = file.status === "missing"
      ? "not in sources"
      : file.status === "changed" ? "changed" : "unchanged";
    item.append(name, mark);
    item.onclick = () => showDrift(generationPath, file.name);
    return item;
  }));
}

// Leave the source page for a generation just made: the tab, the sidebar list and the URL all
// move with it, and the list has to be re-read because the new generation is not in it yet.
export function showGeneration(path, run = false) {
  setTab("logs");
  loadGenerations(true)
    .then(() => selectGeneration(path))
    // Running is that page's own act: it tests the devices, names the run and follows it.
    .then(() => { if (run) $(".run-bar .run-start")?.click(); })
    .catch(showError);
}
