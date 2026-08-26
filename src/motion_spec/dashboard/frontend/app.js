// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * The dashboard: what is wired to what, once, when the page loads.
 */

import { $, RESTRICTED_TABS, api, askConfirm, post, showError, snack, state } from "./core.js";
import { filterGenerations, loadGenerations, selectGeneration } from "./generations.js";
import { goHome, loadLocation, openTab, sidebarLoader } from "./routing.js";
import { reserveVideoSpace } from "./run.js";
import { filterSources, loadSources } from "./sources.js";

document.querySelectorAll("aside button[data-tab]").forEach((button) => {
  button.onclick = () => openTab(button.dataset.tab);
});

export const lifecycleEvents = new EventSource("/api/events");

lifecycleEvents.addEventListener("lifecycle", () => {
  state.cache.generations = null;
  if (state.tab !== "sources") loadGenerations().catch(showError);
});

$("#refresh").onclick = () => sidebarLoader()(true).catch(showError);

$("#delete-selected").onclick = async () => {
  if (!state.selected.size) return;
  const ok = await askConfirm({
    message: `Delete ${state.selected.size} selected generation${state.selected.size === 1 ? "" : "s"} or run${state.selected.size === 1 ? "" : "s"}? This cannot be undone.`,
    confirmLabel: "Delete",
  });
  if (!ok) return;
  let data;
  try {
    data = await post("/api/delete", { paths: [...state.selected] });
  } catch (error) {
    return showError(error);
  }
  const parts = [`${data.deleted} item${data.deleted === 1 ? "" : "s"}`];
  if (data.folders) parts.push(`${data.folders} empty folder${data.folders === 1 ? "" : "s"}`);
  snack(`Moved ${parts.join(" and ")} to Trash`);
  // Only the page being looked at has to move, and then only as far as its parent.
  const trashed = [...state.selected];
  const inTrash = (path) => path && trashed.some((gone) => path === gone || path.startsWith(`${gone}/`));
  const generation = state.generationPath;
  state.selected.clear();
  $("#selection-actions").hidden = true;
  state.cache = {};
  if (inTrash(generation)) {
    state.generation = null;
    return goHome();
  }
  await loadGenerations(true).catch(showError);
  if (!generation) return;
  if (inTrash(state.runPath)) return selectGeneration(generation).catch(showError);
  api(`/api/runs?path=${encodeURIComponent(generation)}`).then(state.generation?.setRuns);
};

$("#clear-selection").onclick = () => {
  state.selected.clear();
  document.querySelectorAll(".picked").forEach((item) => item.classList.remove("picked"));
  $("#selection-actions").hidden = true;
};

document.onkeydown = (event) => {
  if (event.key === "Escape" && state.selected.size) $("#clear-selection").click();
};

// Escape closes the editor's find box, wherever the key was pressed: its field and the text
// are different key-handling scopes inside the editor library, and this is neither -- it is
// the page, closing a panel it can see, with the panel's own button. First refusal of the key:
// with a find box open, escape means this and not clearing a selection.
document.addEventListener("keydown", (event) => {
  if (event.key !== "Escape") return;
  const close = $(".cm-panel.cm-search button[name=close]");
  if (!close) return;
  event.preventDefault();
  event.stopPropagation();
  close.click();
}, true);

$("#search").oninput = (event) => {
  if (state.tab === "sources") filterSources(event.target.value);
  else filterGenerations(event.target.value);
};

export async function updateRoot(path) {
  const kind = state.tab === "sources" ? "sources" : "logs";
  state.roots = await post("/api/roots", { kind, path });
  state.cache = {};
  (kind === "logs" ? loadGenerations : loadSources)(true).catch(showError);
}

$("#generation-root").onchange = (event) => updateRoot(event.target.value).catch(showError);

$("#pick-root").onclick = async () => {
  const kind = state.tab === "sources" ? "sources" : "logs";
  state.roots = await api(`/api/pick-root?kind=${kind}`);
  state.cache = {};
  (kind === "logs" ? loadGenerations : loadSources)(true).catch(showError);
};

$("#sidebar-toggle").onclick = () => {
  document.body.classList.toggle("sidebar-collapsed");
  const collapsed = document.body.classList.contains("sidebar-collapsed");
  localStorage.setItem("motion-spec.sidebar-collapsed", collapsed);
  state.autoCollapsed = false;   // a deliberate choice outlives the width that suggested it
  showSidebarState();
  reserveVideoSpace();
};

export function showSidebarState() {
  const collapsed = document.body.classList.contains("sidebar-collapsed");
  $("#sidebar-toggle").textContent = collapsed ? "☰" : "×";
  $("#sidebar-toggle").title = collapsed ? "Expand navigation" : "Collapse navigation";
}

showSidebarState();

// A narrow window has no room for both: the sidebar folds away and comes back with the width,
// without overwriting what the reader chose at a width where both fit.
export const NARROW = 1000;

export function fitSidebar() {
  const narrow = window.innerWidth < NARROW;
  if (narrow && !document.body.classList.contains("sidebar-collapsed")) {
    document.body.classList.add("sidebar-collapsed");
    state.autoCollapsed = true;
  } else if (!narrow && state.autoCollapsed) {
    document.body.classList.remove("sidebar-collapsed");
    state.autoCollapsed = false;
  }
  showSidebarState();
  reserveVideoSpace();
}

window.addEventListener("resize", fitSidebar);

fitSidebar();

$("#home").onclick = goHome;

window.onpopstate = loadLocation;

// Known before the first tab renders, so a restored #tab=notebook never gets a chance to
// try and fail: the server drops those requests for a LAN viewer, this just hides the door.
state.roots = await api("/api/roots").catch(() => ({}));
state.restricted = Boolean(state.roots.restricted);
if (state.restricted) {
  RESTRICTED_TABS.forEach((tab) => {
    document.querySelectorAll(`[data-tab="${tab}"]`).forEach((button) => { button.style.display = "none"; });
  });
}

loadLocation();

export const asideWidth = (value) => {
  const width = Math.min(720, Math.max(200, value));
  document.documentElement.style.setProperty("--aside", `${width}px`);
  localStorage.setItem("motion-spec.aside", width);
};

asideWidth(Number(localStorage.getItem("motion-spec.aside")) || 290);

$("#aside-resize").onpointerdown = (event) => {
  event.preventDefault();
  document.body.classList.add("resizing");
  const move = (moved) => asideWidth(moved.clientX);
  const up = () => {
    document.body.classList.remove("resizing");
    removeEventListener("pointermove", move);
    removeEventListener("pointerup", up);
  };
  addEventListener("pointermove", move);
  addEventListener("pointerup", up);
};

$("#aside-resize").ondblclick = () => asideWidth(290);
