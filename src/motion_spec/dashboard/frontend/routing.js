// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * Where the reader is, in the URL and in the tabs: one place decides what a hash means
 * and what changing it shows.
 */

import { showEmpty } from "./components.js";
import { $, RESTRICTED_TABS, showError, state } from "./core.js";
import { showExplore } from "./explore.js";
import { loadGenerations, selectGeneration, showDrift, showGitDiff } from "./generations.js";
import { loadHealth } from "./health.js";
import { loadNotebook } from "./notebook.js";
import { loadReplay, stopPlayback } from "./run.js";
import { loadSources, openSource, showGenerated } from "./sources.js";

// A tab that is a page of its own, not a list to pick from: it fills the content pane itself,
// so it cannot also be showing a generation. Selecting one drops the view the hash still named,
// and restoring one wins over that view -- otherwise the hash names two and a reload picks the
// other. Every other tab only chooses which list the sidebar shows.
const PAGE_TABS = { health: loadHealth, notebook: loadNotebook };
const VIEW_PARAMS = ["run", "generation", "explore", "source", "diff", "gitdiff", "file", "panel"];

export function goHome() {
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

export function setView(kind, path) {
  const view = new URLSearchParams(location.hash.slice(1));
  const unchanged = view.get(kind) === path
    && !view.has(kind === "run" ? "generation" : "run")
    && !view.has("source") && !view.has("diff");
  view.delete("run");
  view.delete("generation");
  view.delete("explore");
  view.delete("source");
  view.delete("generated");
  view.delete("diff");
  view.set(kind, path);
  view.set("tab", state.tab);
  if (kind !== "run") view.delete("panel");
  history[unchanged ? "replaceState" : "pushState"](null, "", `#${view}`);
}

export function setTab(tab, push = true) {
  state.tab = tab;
  document.querySelectorAll("aside button[data-tab]").forEach((button) => {
    button.classList.toggle("active", button.dataset.tab === tab);
  });
  const view = new URLSearchParams(location.hash.slice(1));
  view.set("tab", tab);
  if (tab in PAGE_TABS) VIEW_PARAMS.forEach((param) => view.delete(param));
  history[push ? "pushState" : "replaceState"](null, "", `#${view}`);
}

// Which list the sidebar shows. Only the sources tab swaps it; a page tab leaves it alone,
// so asking by "is this the logs tab" empties it the moment health or notebook is open.
export function sidebarLoader() {
  return state.tab === "sources" ? loadSources : loadGenerations;
}

export function openTab(tab) {
  if (state.restricted && RESTRICTED_TABS.includes(tab)) return;
  setTab(tab);
  const page = PAGE_TABS[tab] ?? (tab === "logs" ? loadGenerations : loadSources);
  return page().catch(showError);
}

export function loadLocation() {
  const view = new URLSearchParams(location.hash.slice(1));
  const requested = view.get("tab") ?? "logs";
  // A restricted viewer gets the ordinary landing tab instead, not a dead end from a
  // restored or shared #tab=notebook link.
  setTab(state.restricted && RESTRICTED_TABS.includes(requested) ? "logs" : requested, false);
  const loadSidebar = sidebarLoader();
  // The empty state is hidden until something replaces it, so uncover it even when nothing
  // could be loaded -- a failed restore must not leave the pane blank with no way back.
  const settle = (loading) => loading
    .catch(showError)
    .finally(() => document.documentElement.classList.remove("restoring"));
  // On a page tab setTab has just dropped the view the hash also named, so hold no generation
  // either: the state and the URL have to agree on which single view this is.
  const page = PAGE_TABS[state.tab];
  state.generationPath = page ? null
    : view.get("generation") ?? view.get("explore") ?? view.get("diff")
      ?? view.get("run")?.split("/runs/")[0]
      ?? view.get("generated")?.split("/generated/")[0] ?? null;
  if (page) return settle(loadSidebar().then(() => page()));
  if (view.has("explore")) {
    settle(loadSidebar().then(() => showExplore(view.get("explore"), false)));
  }
  else if (view.has("run")) {
    settle(loadSidebar().then(() => loadReplay(view.get("run"))));
  }
  else if (view.has("diff")) {
    settle(showDrift(view.get("diff"), view.get("file"), false));
  }
  else if (view.has("gitdiff")) {
    settle(loadSidebar().then(() => showGitDiff(view.get("gitdiff"), false)));
  }
  else if (view.has("source")) {
    // The viewer re-renders itself; it must not push the entry it is restoring back on.
    settle(loadSidebar().then(() => openSource(view.get("source"), null, false)));
  }
  else if (view.has("generated")) {
    settle(loadSidebar().then(() => showGenerated(view.get("generated"), false)));
  }
  else if (view.has("generation")) {
    settle(loadSidebar().then(() => selectGeneration(view.get("generation"))));
  }
  else settle(loadSidebar());
}
