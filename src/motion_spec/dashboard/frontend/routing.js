// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * Where the reader is, in the URL and in the tabs: one place decides what a hash means
 * and what changing it shows.
 */

import { showEmpty } from "./components.js";
import { $, showError, state } from "./core.js";
import { loadGenerations, selectGeneration, showDrift } from "./generations.js";
import { loadReplay, stopPlayback } from "./run.js";
import { loadSources, openSource } from "./sources.js";

export function goHome() {
  stopPlayback();
  state.generationPath = null;
  state.runPath = null;
  setTab("logs");
  history.replaceState(null, "", `${location.pathname}#tab=logs`);
  showEmpty({
    eyebrow: "REPLAY / 01",
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
  view.delete("source");
  view.delete("diff");
  view.set(kind, path);
  view.set("tab", state.tab);
  if (kind !== "run") view.delete("panel");
  history[unchanged ? "replaceState" : "pushState"](null, "", `#${view}`);
}

export function setTab(tab, push = true) {
  state.tab = tab;
  document.querySelectorAll("nav button[data-tab]").forEach((button) => {
    button.classList.toggle("active", button.dataset.tab === tab);
  });
  const view = new URLSearchParams(location.hash.slice(1));
  view.set("tab", tab);
  history[push ? "pushState" : "replaceState"](null, "", `#${view}`);
}

export function loadLocation() {
  const view = new URLSearchParams(location.hash.slice(1));
  state.generationPath = view.get("generation") ?? view.get("diff")
    ?? view.get("run")?.split("/runs/")[0] ?? null;
  setTab(view.get("tab") ?? "logs", false);
  const loadSidebar = state.tab === "logs" ? loadGenerations : loadSources;
  const rendered = () => document.documentElement.classList.remove("restoring");
  if (view.has("run")) {
    loadSidebar().then(() => loadReplay(view.get("run"))).then(rendered).catch(showError);
  }
  else if (view.has("diff")) {
    showDrift(view.get("diff"), view.get("file"), false).then(rendered).catch(showError);
  }
  else if (view.has("source")) {
    // The viewer re-renders itself; it must not push the entry it is restoring back on.
    loadSidebar().then(() => openSource(view.get("source"), null, false)).then(rendered).catch(showError);
  }
  else if (view.has("generation")) {
    loadSidebar().then(() => selectGeneration(view.get("generation"))).then(rendered).catch(showError);
  }
  else loadSidebar().then(rendered).catch(showError);
}
