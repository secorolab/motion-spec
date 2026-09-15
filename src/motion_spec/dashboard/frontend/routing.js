// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/** What a hash means and what changing it shows: the one place that decides both. */

import { showEmpty } from "./components.js";
import { saveInspection } from "./inspection.js";
import { $, RESTRICTED_TABS, showError, state } from "./core.js";
import { showExplore } from "./explore.js";
import { loadGenerations, selectGeneration, showDrift, showGitDiff } from "./generations.js";
import { loadHealth } from "./health.js";
import { loadNotebook } from "./notebook.js";
import { loadReplay, stopPlayback } from "./run.js";
import { loadSources, openSource, showGenerated } from "./sources.js";

// A page tab fills the content pane itself, so it cannot also be showing a view: selecting one
// drops every view parameter, or the hash names two things and a reload picks the wrong one.
const PAGE_TABS = { health: loadHealth, notebook: loadNotebook };
const VIEW_PARAMS = [
  "run",
  "generation",
  "explore",
  "source",
  "generated",
  "diff",
  "gitdiff",
  "file",
  "panel",
];

// A hash can still name more than one view; this is the order in which they win.
const VIEW_ORDER = ["explore", "run", "diff", "gitdiff", "source", "generated", "generation"];

const OPEN_VIEW = {
  explore: (view) => showExplore(view.get("explore"), false),
  run: (view) => loadReplay(view.get("run")),
  diff: (view) => showDrift(view.get("diff"), view.get("file"), false),
  gitdiff: (view) => showGitDiff(view.get("gitdiff"), false),
  // The viewer re-renders itself; it must not push the entry it is restoring back on.
  source: (view) => openSource(view.get("source"), null, false),
  generated: (view) => showGenerated(view.get("generated"), false),
  generation: (view) => selectGeneration(view.get("generation")),
};

export const hashView = () => new URLSearchParams(location.hash.slice(1));

export function writeHash({ drop = [], set = {}, replace = false }) {
  const view = hashView();
  drop.forEach((param) => view.delete(param));
  Object.entries(set).forEach(([param, value]) => {
    if (value === null) view.delete(param);
    else view.set(param, value);
  });
  history[replace ? "replaceState" : "pushState"](null, "", `#${view}`);
}

export function setView(kind, path) {
  saveInspection();
  const view = hashView();
  const sameTarget = view.get(kind) === path;
  const unchanged =
    sameTarget &&
    !view.has(kind === "run" ? "generation" : "run") &&
    !view.has("source") &&
    !view.has("diff");
  writeHash({
    drop: [
      "run",
      "generation",
      "explore",
      "source",
      "generated",
      "diff",
      ...(sameTarget ? [] : ["frame"]),
    ],
    set: { [kind]: path, tab: state.tab, ...(kind === "run" ? {} : { panel: null }) },
    replace: unchanged,
  });
}

export function setTab(tab, push = true) {
  saveInspection();
  state.tab = tab;
  const tools = $("#generation-tools");
  if (tools) tools.hidden = tab === "sources";
  document.querySelectorAll("aside button[data-tab]").forEach((button) => {
    button.classList.toggle("active", button.dataset.tab === tab);
  });
  writeHash({
    drop: tab in PAGE_TABS ? VIEW_PARAMS : [],
    set: { tab },
    replace: !push,
  });
}

// Only the sources tab swaps the sidebar list. Asking "is this the logs tab" instead would
// empty the sidebar the moment health or notebook opened.
export function sidebarLoader() {
  return state.tab === "sources" ? loadSources : loadGenerations;
}

export function openTab(tab) {
  if (state.restricted && RESTRICTED_TABS.includes(tab)) return;
  setTab(tab);
  const page = PAGE_TABS[tab] ?? (tab === "logs" ? loadGenerations : loadSources);
  return page().catch(showError);
}

export function goHome() {
  saveInspection();
  stopPlayback();
  state.generationPath = null;
  state.runPath = null;
  setTab("logs");
  history.replaceState(null, "", `${location.pathname}#tab=logs`);
  showEmpty({
    eyebrow: "GENERATIONS",
    title: "Choose a generation.",
    detail: "Its build metadata and recorded runs will appear here.",
  });
  return loadGenerations().catch(showError);
}

export function loadLocation() {
  const view = hashView();
  const requested = view.get("tab") ?? "logs";
  setTab(state.restricted && RESTRICTED_TABS.includes(requested) ? "logs" : requested, false);
  const loadSidebar = sidebarLoader();
  // Uncover the empty state even when nothing loaded, or a failed restore leaves a blank pane.
  const settle = (loading) =>
    loading.catch(showError).finally(() => document.documentElement.classList.remove("restoring"));
  // setTab has just dropped the view parameters on a page tab, so hold no generation either.
  const page = PAGE_TABS[state.tab];
  state.generationPath = page ? null : generationOf(view);
  if (page) return settle(loadSidebar().then(() => page()));
  const named = VIEW_ORDER.find((param) => view.has(param));
  if (!named) return settle(loadSidebar());
  const open = () => OPEN_VIEW[named](view);
  // The drift page fills the sidebar itself, with the generation's own sources.
  return settle(named === "diff" ? open() : loadSidebar().then(open));
}

function generationOf(view) {
  return (
    view.get("generation") ??
    view.get("explore") ??
    view.get("diff") ??
    view.get("run")?.split("/runs/")[0] ??
    view.get("generated")?.split(/\/(generated|runs)\//)[0] ??
    null
  );
}
