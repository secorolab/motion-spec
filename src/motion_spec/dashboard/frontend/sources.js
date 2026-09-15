// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/** The sources tab: the model tree, one file open in the editor, and generating from it. */

import { appendConsole, copyPathButton, listItem } from "./components.js";
import {
  $,
  $$,
  api,
  askConfirm,
  copyText,
  formatBytes,
  post,
  readStored,
  snack,
  snackError,
  state,
  writeStored,
} from "./core.js";
import { mountEditor, revealLine } from "./editor.js";
import { showGeneration, showGitDiff } from "./generations.js";
import { hashView, setTab, setView, writeHash } from "./routing.js";
import { loadReplay } from "./run.js";

// What the server sends at most; only ever used to say so.
const GENERATED_SHOWN = 2_000_000;

export async function loadSources(refresh = false) {
  const request = ++state.listRequest;
  if (refresh || !state.cache.sources) {
    const [sources, roots] = await Promise.all([api("/api/sources"), api("/api/roots")]);
    state.cache.sources = { sources, roots };
  }
  if (request !== state.listRequest || state.tab !== "sources") return;
  $("#list-title").textContent = "SOURCES";
  $("#storage").textContent = "";
  $("#generation-root").hidden = false;
  const { sources, roots } = state.cache.sources;
  state.roots = roots;
  $("#generation-root").value = roots.sources;
  $("#browser").replaceChildren(...renderSourceNode(sourceTree(sources), "", 0, true));
  filterSources($("#search").value);
}

function sourceTree(sources) {
  const root = { folders: new Map(), files: [] };
  sources.forEach((source) => {
    const parts = source.split("/");
    const file = parts.pop();
    let node = root;
    parts.forEach((part) => {
      node.folders.set(part, node.folders.get(part) ?? { folders: new Map(), files: [] });
      node = node.folders.get(part);
    });
    node.files.push({ file, source });
  });
  return root;
}

function renderSourceNode(node, name, depth, isRoot = false) {
  if (!isRoot) {
    const branch = document.createElement("details");
    branch.className = "source-node";
    branch.open = true;
    branch.style.setProperty("--depth", depth);
    const summary = document.createElement("summary");
    summary.textContent = name;
    branch.append(summary);
    branch.append(...renderSourceNode(node, "", depth + 1, true));
    return [branch];
  }
  const children = [];
  [...node.folders]
    .sort(([left], [right]) => left.localeCompare(right))
    .forEach(([folder, child]) => {
      children.push(...renderSourceNode(child, folder, depth));
    });
  node.files
    .sort((left, right) => left.file.localeCompare(right.file))
    .forEach(({ file, source }) => {
      const leaf = document.createElement("div");
      leaf.className = "source-leaf";
      leaf.style.setProperty("--depth", depth);
      const item = listItem(
        file,
        null,
        () => {
          openSource(source);
        },
        source,
      );
      scrollLongName(item);
      leaf.append(
        item,
        copyPathButton(() => `${state.roots.sources}/${source}`),
      );
      // Built once: the filter runs over every leaf on every keystroke.
      leaf.dataset.search = leaf.textContent.toLowerCase();
      children.push(leaf);
    });
  return children;
}

function scrollLongName(item) {
  item.onmouseenter = () => {
    const style = getComputedStyle(item);
    const available =
      item.clientWidth - parseFloat(style.paddingLeft) - parseFloat(style.paddingRight);
    const distance = Math.max(0, item.querySelector(".item-name").scrollWidth - available);
    item.style.setProperty("--scroll-distance", `${distance}px`);
    item.classList.toggle("is-scrolling", distance > 0);
  };
  item.onmouseleave = () => item.classList.remove("is-scrolling");
}

export function filterSources(value) {
  const query = value.toLowerCase();
  document.querySelectorAll(".source-leaf").forEach((item) => {
    item.hidden = !item.dataset.search.includes(query);
  });
  // Deepest first, and each branch asks only its own children: asking every descendant instead
  // walks the whole tree once per branch.
  [...document.querySelectorAll(".source-node")].reverse().forEach((node) => {
    const visible = [...node.children].some(
      (child) => child.matches(".source-leaf, .source-node") && !child.hidden,
    );
    node.hidden = !visible;
    if (query && visible) node.open = true;
  });
}

// Always the working-tree file, never a generation's archived copy of it.
export async function showSource(workspace, absolute, line = null) {
  if (!workspace) {
    await copyText(absolute);
    return snack("not in the sources tree any more — path copied");
  }
  setTab("sources");
  await loadSources().catch(() => {});
  try {
    await openSource(workspace, null, true, line);
  } catch (error) {
    await copyText(absolute);
    snack(`${error.message} — path copied`);
  }
}

// Generated output in the same editor the sources tab mounts, minus everything that writes:
// the generator owns these files. Back goes to the generation, or to the run that wrote it.
export async function showGenerated(relative, push = true) {
  const run = relative.match(/^(.*\/runs\/[^/]+)\/(.+)$/);
  const [generation, name] = run
    ? [run[1].split("/runs/")[0], run[2]]
    : relative.split("/generated/");
  state.viewing = relative;
  state.generationPath = generation;
  if (push) setView("generated", relative);
  const read = await api(`/api/generated?path=${encodeURIComponent(relative)}`);
  if (state.viewing !== relative) return;
  $("#content").innerHTML =
    '<div class="viewer"><div class="viewer-top">' +
    `<div class="page-heading"><button id="back" title="Back to ${run ? "run" : "generation"}">←</button>` +
    `<h1></h1><span class="eyebrow">${run ? "RUN" : "GENERATED"}</span></div>` +
    '<div class="viewer-heading"><p class="path"></p></div>' +
    '<p class="syntax-state generated-note" hidden></p></div><div id="source-text"></div></div>';
  $(".viewer h1").textContent = name.split("/").pop();
  $(".viewer .path").textContent = read.absolute;
  $("#back").onclick = () => (run ? loadReplay(run[1]) : showGeneration(generation));
  const note = $(".generated-note");
  if (read.binary || read.truncated) {
    note.hidden = false;
    note.textContent = read.binary
      ? `${formatBytes(read.size)} of packed binary — nothing to read as text.`
      : `Showing the first ${formatBytes(GENERATED_SHOWN)} of ${formatBytes(read.size)}.`;
  }
  if (read.binary) {
    $("#source-text").textContent = "";
    return;
  }
  await mountEditor($("#source-text"), relative, read.text, true);
}

// Two groups of actions: what to do with the file, and what to make from it.
const VIEWER_MARKUP =
  '<div class="viewer"><div class="viewer-top"><div class="page-heading"><h1></h1><span class="eyebrow">SOURCE</span></div><div class="viewer-heading"><p class="path"></p><div class="open-with"><span class="bar file-bar"><button id="save-source" hidden>save</button><button id="diff-source" hidden>diff</button><button id="checkout-source" hidden>checkout</button><span class="split"><button id="open-editor"></button><details class="picker picker-down" id="editor-choice"><summary title="Choose the editor"><svg viewBox="0 0 12 12" aria-hidden="true"><path d="M2.5 4.5 6 8l3.5-3.5"/></svg></summary><div class="picker-panel"></div></details></span></span><span class="bar build-bar"><button id="gen-source">generate</button><button id="gen-run-source" class="is-primary">generate &amp; run</button></span></div></div><p class="syntax-state" hidden></p></div><div id="source-text"></div></div>';

export async function openSource(source, absolute = null, push = true, line = null) {
  state.viewing = source;
  if (push) {
    writeHash({
      drop: ["run", "generation", "panel"],
      set: { source, tab: "sources" },
      replace: hashView().get("source") === source,
    });
  }
  const {
    text,
    git_head: gitHead,
    editors,
    terminal,
    absolute: where,
  } = await api(`/api/source?path=${encodeURIComponent(source)}`);
  if (state.viewing !== source) return;
  $("#content").innerHTML = VIEWER_MARKUP;
  $(".viewer h1").textContent = source.split("/").pop();
  // A generation's vendored copy is not under the sources root; show where it really is.
  $(".viewer .path").textContent = where ?? absolute ?? source;
  revealInTree(source);
  await mountEditor($("#source-text"), source, text, state.restricted, gitHead, line);
  // With no commit to compare against there is no diff to show and nothing to go back to.
  const dirty = gitHead != null && gitHead !== text;
  $("#diff-source").hidden = !dirty;
  $("#diff-source").onclick = () => showGitDiff(source).catch(snackError);
  if (state.restricted) {
    // Opening this machine's editor or terminal is not for a LAN viewer. `.file-bar` sets its
    // own `display: flex` above the browser's `[hidden]`, so only an inline style hides it.
    $(".viewer .file-bar").style.display = "none";
  } else {
    bindCheckout(source, absolute, dirty);
    bindEditorChoice(source, editors, terminal);
  }
  // Only a .robmot is a whole generation to make; the other authored files are parts of one.
  if (source.endsWith(".robmot")) {
    bindGenerate(source);
    showLint(source).catch(() => {});
  }
}

// A file opened from a generation is otherwise unfindable in the tree.
function revealInTree(source) {
  $$("#browser .item").forEach((item) =>
    item.classList.toggle("selected", item.dataset.path === source),
  );
  const listed = $(`#browser .item[data-path="${CSS.escape(source)}"]`);
  if (!listed) return;
  // Every collapsed folder above it, not just the nearest, or the entry stays hidden.
  for (let node = listed.closest("details"); node; node = node.parentElement?.closest("details")) {
    node.open = true;
  }
  // Scrolling an entry the reader can already see moves the list under their eyes for nothing.
  const browser = $("#browser");
  const list = browser.getBoundingClientRect();
  const entry = listed.getBoundingClientRect();
  if (entry.top < list.top || entry.bottom > list.bottom) {
    browser.scrollTop += entry.top - list.top - browser.clientHeight / 2 + entry.height / 2;
  }
}

function bindCheckout(source, absolute, dirty) {
  const checkout = $("#checkout-source");
  checkout.hidden = !dirty;
  checkout.title = "Discard uncommitted changes, restoring this file from git HEAD";
  checkout.onclick = async () => {
    const ok = await askConfirm({
      message: `Discard uncommitted changes to ${source.split("/").pop()} and restore it from git HEAD? This cannot be undone.`,
      confirmLabel: "Discard changes",
    });
    if (!ok) return;
    try {
      await post("/api/source-checkout", { path: source });
      snack("Restored from git HEAD");
      await openSource(source, absolute, false); // re-read: the file on disk has changed
    } catch (error) {
      snackError(error);
    }
  };
}

function bindEditorChoice(source, editors, terminal) {
  const options = [...editors];
  if (terminal) options.push("terminal");
  const menu = $("#editor-choice");
  const opener = $("#open-editor");
  const label = (name) => (name === "terminal" ? `terminal (${terminal})` : name);
  let chosen = readStored("motion-spec.editor");
  if (!options.includes(chosen)) chosen = options[0] ?? "";
  const name = () => {
    opener.textContent = chosen ? `open in ${label(chosen)}` : "no editor found";
  };
  name();
  opener.disabled = !chosen;
  menu.hidden = options.length < 2;
  menu.querySelector(".picker-panel").replaceChildren(
    ...options.map((option) => {
      const pick = document.createElement("button");
      pick.textContent = label(option);
      pick.setAttribute("aria-pressed", String(option === chosen));
      pick.onclick = () => {
        chosen = option;
        writeStored("motion-spec.editor", option);
        name();
        menu
          .querySelectorAll(".picker-panel button")
          .forEach((other) => other.setAttribute("aria-pressed", String(other === pick)));
        menu.open = false;
      };
      return pick;
    }),
  );
  opener.onclick = async () => {
    try {
      if (chosen === "terminal") {
        const { opened } = await post("/api/terminal", { path: source });
        snack(`${terminal} at ${opened}`);
      } else {
        await post("/api/open", { path: source, editor: chosen });
        snack(`Opened in ${chosen}`);
      }
    } catch (error) {
      snackError(error);
    }
  };
}

// The design graph is made by generating, so the findings come from this model's newest
// generation and there is nothing to say until one exists. A finding is a warning at most.
export async function showLint(source) {
  const box = document.createElement("section");
  box.className = "lint";
  box.hidden = true;
  $(".viewer-top").after(box);
  const file = source.split("/").pop();
  const generations = await api("/api/generations").catch(() => []);
  const newest = generations
    .filter((generation) => generation.source === file)
    .sort((left, right) => String(right.created).localeCompare(String(left.created)))[0];
  if (!newest || state.viewing !== source || !box.isConnected) return;
  const { items } = await api(`/api/model/lint?path=${encodeURIComponent(newest.path)}`).catch(
    () => ({ items: null }),
  );
  if (!items || state.viewing !== source || !box.isConnected) return;
  const head = document.createElement("p");
  head.className = "lint-head";
  head.textContent = `${items.length || "no"} lint finding${items.length === 1 ? "" : "s"} · ${newest.path}`;
  box.append(head, ...items.map(lintRow));
  box.hidden = false;
}

function lintRow(item) {
  const row = document.createElement("button");
  row.className = "lint-item";
  row.type = "button";
  row.disabled = !item.source_line;
  const cell = (className, text) => {
    const span = document.createElement("span");
    span.className = className;
    span.textContent = text;
    return span;
  };
  row.append(
    cell("lint-sev", item.severity),
    cell("lint-name", item.name),
    cell("lint-line", item.source_line ? `:${item.source_line}` : ""),
    cell("lint-why", `${item.rule} — ${item.why}`),
  );
  row.title = item.iri;
  row.onclick = () => revealLine(item.source_line);
  return row;
}

// Generate, build and optionally run a model from its own page, showing the terminal doing it.
export function bindGenerate(source) {
  const generate = $("#gen-source");
  const start = $("#gen-run-source");
  const status = document.createElement("span");
  status.className = "run-state";
  const open = document.createElement("button");
  open.textContent = "open generation";
  open.hidden = true;
  $(".viewer .build-bar").append(open);
  $(".viewer .open-with").append(status);
  const pre = document.createElement("pre");
  pre.className = "console";
  pre.hidden = true;
  $(".viewer-top").after(pre);

  const stop = () => {
    clearInterval(state.generateWatch);
    clearInterval(state.generateConsole);
  };
  const busy = (working) => {
    generate.disabled = start.disabled = working;
  };
  const gone = () => state.viewing !== source || !pre.isConnected;
  const launch = async (run) => {
    stop();
    busy(true);
    open.hidden = true;
    status.textContent = "starting…";
    pre.textContent = "";
    pre.hidden = false;
    let offset = 0;
    try {
      const started = await post("/api/generate", { path: source });
      status.textContent = `running · pid ${started.pid}`;
      const tail = async () => {
        if (gone()) return stop();
        const slice = await api(
          `/api/console?job=${encodeURIComponent(started.job)}&offset=${offset}`,
        ).catch(() => null);
        if (!slice || !slice.text) return;
        offset = slice.offset;
        appendConsole(pre, slice.text);
      };
      const check = async () => {
        if (gone()) return stop();
        const job = await api(`/api/generate?job=${encodeURIComponent(started.job)}`).catch(
          () => null,
        );
        if (!job) return;
        status.textContent = job.busy ? `running · pid ${job.pid}` : `exited ${job.exit_code}`;
        if (job.busy) return;
        clearInterval(state.generateWatch);
        busy(false);
        await tail(); // the closing lines land after the process is already gone
        clearInterval(state.generateConsole);
        if (!job.generation) return;
        // A failure stays put, where its console is, and offers the way in instead.
        if (job.exit_code) {
          open.hidden = false;
          open.onclick = () => showGeneration(job.generation);
          return;
        }
        showGeneration(job.generation, run);
      };
      await tail();
      state.generateConsole = setInterval(tail, 1000);
      state.generateWatch = setInterval(check, 2000);
    } catch (error) {
      stop();
      busy(false);
      status.textContent = error.message;
    }
  };
  generate.onclick = () => launch(false);
  start.onclick = () => launch(true);
}
