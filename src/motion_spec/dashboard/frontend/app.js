// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/** What is wired to what, once, when the page loads. */

import {
  $,
  RESTRICTED_TABS,
  api,
  askConfirm,
  formatBytes,
  post,
  readStored,
  showError,
  snack,
  state,
  writeStored,
} from "./core.js";
import { filterGenerations, loadGenerations, selectGeneration } from "./generations.js";
import { goHome, loadLocation, openTab, sidebarLoader } from "./routing.js";
import { reserveVideoSpace } from "./run.js";
import { installSelectTheme } from "./selects.js";
import { filterSources, loadSources } from "./sources.js";
import { browseTrash, forgetTrash, loadTrash } from "./trash.js";

// Below this width the sidebar folds away by itself, and unfolds again above it.
const NARROW = 1000;

const ASIDE_DEFAULT = 290;

installSelectTheme();

document.querySelectorAll("aside button[data-tab]").forEach((button) => {
  button.onclick = () => openTab(button.dataset.tab);
});

const lifecycleEvents = new EventSource("/api/events");

lifecycleEvents.addEventListener("lifecycle", () => {
  state.cache.generations = null;
  if (state.tab !== "sources") loadGenerations().catch(showError);
});

$("#refresh").onclick = () => sidebarLoader()(true).catch(showError);

$("#search").oninput = (event) => {
  if (state.tab === "sources") filterSources(event.target.value);
  else filterGenerations(event.target.value);
};

$("#delete-selected").onclick = () => deleteSelected();

$("#clear-selection").onclick = () => {
  state.selected.clear();
  document.querySelectorAll(".picked").forEach((item) => item.classList.remove("picked"));
  document.querySelectorAll(".pick").forEach((box) => {
    box.checked = box.indeterminate = false;
  });
  $("#selection-actions").hidden = true;
};

document.onkeydown = (event) => {
  if (event.key === "Escape" && state.selected.size) $("#clear-selection").click();
};

// Escape closes the editor's find box wherever it was pressed, and takes the key before the
// selection handler above: with a find box open, escape means close it.
document.addEventListener(
  "keydown",
  (event) => {
    if (event.key !== "Escape") return;
    const close = $(".cm-panel.cm-search button[name=close]");
    if (!close) return;
    event.preventDefault();
    event.stopPropagation();
    close.click();
  },
  true,
);

async function deleteSelected() {
  if (!state.selected.size) return;
  let preview;
  try {
    preview = await post("/api/delete-preview", { paths: [...state.selected] });
  } catch (error) {
    return showError(error);
  }
  const eligible = preview.items.filter((item) => !item.blocked);
  const blocked = preview.items.filter((item) => item.blocked);
  if (!eligible.length)
    return showError(
      new Error(
        `Nothing can be moved to Trash. ${blocked.map((item) => `${item.path}: ${item.blocked}`).join("; ")}`,
      ),
    );
  const ok = await askConfirm({
    message: `Move ${eligible.length} items (${eligible.reduce((n, item) => n + item.runs, 0)} runs, ${formatBytes(eligible.reduce((n, item) => n + item.bytes, 0))}) to Trash? Restore them from the Trash browser. Disk space is only freed when Trash is emptied.\n\n${eligible.map((item) => item.path).join("\n")}${blocked.length ? "\n\nProtected; excluded:\n" + blocked.map((item) => `${item.path}: ${item.blocked}`).join("\n") : ""}`,
    confirmLabel: "Move to Trash",
  });
  if (!ok) return;
  let data;
  try {
    data = await post("/api/delete", { paths: eligible.map((item) => item.path) });
  } catch (error) {
    return showError(error);
  }
  const parts = [`${data.deleted} item${data.deleted === 1 ? "" : "s"}`];
  if (data.folders) parts.push(`${data.folders} empty folder${data.folders === 1 ? "" : "s"}`);
  snack(`Moved ${parts.join(" and ")} to Trash`);
  await afterDelete(eligible.map((item) => item.path));
}

// Only the page being looked at moves, and then only as far as its parent.
async function afterDelete(trashed) {
  const inTrash = (path) =>
    path && trashed.some((gone) => path === gone || path.startsWith(`${gone}/`));
  const generation = state.generationPath;
  state.selected.clear();
  $("#selection-actions").hidden = true;
  state.cache = {};
  forgetTrash();
  if (inTrash(generation)) {
    state.generation = null;
    return goHome();
  }
  await loadGenerations(true).catch(showError);
  if (!generation) return;
  if (inTrash(state.runPath)) return selectGeneration(generation).catch(showError);
  const setRuns = state.generation?.setRuns;
  if (setRuns) {
    api(`/api/runs?path=${encodeURIComponent(generation)}`)
      .then(setRuns)
      .catch(showError);
  }
}

function useRoots(kind, roots) {
  state.roots = roots;
  state.cache = {};
  forgetTrash();
  (kind === "logs" ? loadGenerations : loadSources)(true).catch(showError);
}

const rootKind = () => (state.tab === "sources" ? "sources" : "logs");

$("#generation-root").onchange = async (event) => {
  const kind = rootKind();
  useRoots(kind, await post("/api/roots", { kind, path: event.target.value }));
};

$("#pick-root").onclick = async () => {
  const kind = rootKind();
  useRoots(kind, await api(`/api/pick-root?kind=${kind}`));
};

$("#browse-trash").onclick = browseTrash;

$("#home").onclick = goHome;

const sidebarToggle = $("#sidebar-toggle");

sidebarToggle.onclick = () => {
  document.body.classList.toggle("sidebar-collapsed");
  writeStored(
    "motion-spec.sidebar-collapsed",
    String(document.body.classList.contains("sidebar-collapsed")),
  );
  state.autoCollapsed = false; // a deliberate choice outlives the width that suggested it
  showSidebarState();
  reserveVideoSpace();
};

function showSidebarState() {
  const collapsed = document.body.classList.contains("sidebar-collapsed");
  const label = collapsed ? "Expand navigation" : "Collapse navigation";
  // Collapsed, the CSS draws the brand mark on this button, so it must carry no glyph.
  sidebarToggle.textContent = collapsed ? "" : "×";
  sidebarToggle.title = label;
  sidebarToggle.setAttribute("aria-label", label);
}

function fitSidebar() {
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

const asideWidth = (value) => {
  const width = Math.min(720, Math.max(200, value));
  document.documentElement.style.setProperty("--aside", `${width}px`);
  writeStored("motion-spec.aside", String(width));
};

const asideResize = $("#aside-resize");

asideResize.onpointerdown = (event) => {
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

asideResize.ondblclick = () => asideWidth(ASIDE_DEFAULT);

showSidebarState();

window.addEventListener("resize", fitSidebar);

fitSidebar();

window.onpopstate = loadLocation;

// Read before the first tab renders, so a restored #tab=notebook never gets to try and fail.
state.roots = await api("/api/roots").catch(() => ({}));
state.restricted = Boolean(state.roots.restricted);
$("#brand-version").textContent = state.roots.version ?? "";
$("#browse-trash").hidden = state.restricted;
if (state.restricted) {
  RESTRICTED_TABS.forEach((tab) => {
    document.querySelectorAll(`[data-tab="${tab}"]`).forEach((button) => {
      button.style.display = "none";
    });
  });
} else {
  loadTrash().catch(() => {});
}

loadLocation();

asideWidth(Number(readStored("motion-spec.aside")) || ASIDE_DEFAULT);
