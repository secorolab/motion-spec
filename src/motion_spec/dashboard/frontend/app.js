// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * The dashboard: what is wired to what, once, when the page loads.
 */

import { $, RESTRICTED_TABS, api, askConfirm, copyText, formatBytes, post, showError, snack, stampText, state } from "./core.js";
import { filterGenerations, loadGenerations, selectGeneration } from "./generations.js";
import { goHome, loadLocation, openTab, sidebarLoader } from "./routing.js";
import { reserveVideoSpace } from "./run.js";
import { installSelectTheme } from "./selects.js";
import { filterSources, loadSources } from "./sources.js";

let trashEntries;

const loadTrash = () => trashEntries ??= api("/api/trash");

installSelectTheme();

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
  let preview;
  try { preview = await post("/api/delete-preview", { paths: [...state.selected] }); }
  catch (error) { return showError(error); }
  const eligible = preview.items.filter((item) => !item.blocked);
  const blocked = preview.items.filter((item) => item.blocked);
  if (!eligible.length) return showError(new Error(`Nothing can be moved to Trash. ${blocked.map((item) => `${item.path}: ${item.blocked}`).join("; ")}`));
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
  // Only the page being looked at has to move, and then only as far as its parent.
  const trashed = eligible.map((item) => item.path);
  const inTrash = (path) => path && trashed.some((gone) => path === gone || path.startsWith(`${gone}/`));
  const generation = state.generationPath;
  state.selected.clear();
  $("#selection-actions").hidden = true;
  state.cache = {};
  trashEntries = null;
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
  document.querySelectorAll(".pick").forEach((box) => { box.checked = box.indeterminate = false; });
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
  trashEntries = null;
  (kind === "logs" ? loadGenerations : loadSources)(true).catch(showError);
}

$("#generation-root").onchange = (event) => updateRoot(event.target.value).catch(showError);

$("#pick-root").onclick = async () => {
  const kind = state.tab === "sources" ? "sources" : "logs";
  state.roots = await api(`/api/pick-root?kind=${kind}`);
  state.cache = {};
  trashEntries = null;
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
  // Collapsed, the rail carries the mark instead of a glyph -- the CSS draws it, so the button
  // keeps no text of its own. The label still says what pressing it does, which is the part a
  // logo cannot say for itself.
  $("#sidebar-toggle").textContent = collapsed ? "" : "×";
  $("#sidebar-toggle").title = collapsed ? "Expand navigation" : "Collapse navigation";
  $("#sidebar-toggle").setAttribute(
    "aria-label", collapsed ? "Expand navigation" : "Collapse navigation"
  );
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

$("#browse-trash").onclick = async () => {
  let entries;
  try { entries = await loadTrash(); }
  catch (error) { return showError(error); }
  const dialog = document.createElement("dialog");
  dialog.className = "trash-browser";
  dialog.innerHTML = '<header><div><span class="eyebrow">RECOVERABLE FILES</span><h2>Trash</h2></div><button class="close trash-action">Close</button></header><p>Only items from this generation root are shown. Existing paths will never be overwritten.</p><div class="browser-tools"><input type="search" placeholder="Filter trashed paths" aria-label="Filter trashed paths"></div><div class="run-header trash-columns"><span>Path</span><button class="trash-date-sort" aria-sort="descending">Deleted ↓</button><span>Actions</span></div><div class="trash-entries"></div><p role="status"></p>';
  document.body.append(dialog);
  dialog.querySelector(".close").onclick = () => dialog.close();
  dialog.onclose = () => dialog.remove();
  try {
    const list = dialog.querySelector(".trash-entries");
    const dateSort = dialog.querySelector(".trash-date-sort");
    let newestFirst = true;
    let shown = 100;
    const groupLabel = (value) => {
      if (!value) return "Unknown time";
      const date = new Date(value);
      const today = new Date();
      const yesterday = new Date(today);
      yesterday.setDate(today.getDate() - 1);
      if (date.toDateString() === today.toDateString()) return "Today";
      if (date.toDateString() === yesterday.toDateString()) return "Yesterday";
      return date.toLocaleDateString(undefined, { weekday: "long", day: "numeric", month: "long", year: "numeric" });
    };
    const draw = () => {
      const needle = dialog.querySelector("input").value.toLowerCase();
      const visible = entries.filter((entry) => entry.path.toLowerCase().includes(needle)).sort((left, right) => {
        const order = (new Date(right.deleted_at ?? 0)) - (new Date(left.deleted_at ?? 0));
        return newestFirst ? order : -order;
      });
      const page = visible.slice(0, shown);
      const groups = new Map();
      page.forEach((entry) => {
        const label = groupLabel(entry.deleted_at);
        groups.set(label, [...(groups.get(label) ?? []), entry]);
      });
      const rows = [...groups].flatMap(([group, groupEntries]) => {
        const heading = document.createElement("span");
        heading.className = "eyebrow";
        heading.textContent = group;
        return [heading, ...groupEntries.map((entry) => {
        const row = document.createElement("div");
        row.className = "item";
        const details = document.createElement("div");
        const label = document.createElement("strong");
        label.textContent = entry.path;
        const time = document.createElement("small");
        time.textContent = stampText(entry.deleted_at);
        details.append(label);
        const actions = document.createElement("div");
        actions.className = "trash-actions";
        const copy = document.createElement("button");
        copy.textContent = "Copy";
        copy.onclick = () => copyText(`${state.roots.logs}/${entry.path}`);
        const button = document.createElement("button");
        button.textContent = entry.exists ? "Path exists" : "Restore";
        button.disabled = entry.exists;
        button.onclick = async () => {
          button.disabled = true;
          try {
            await post("/api/restore", { uri: entry.uri });
            entries.splice(entries.indexOf(entry), 1);
            draw();
            state.cache = {};
            state.generation = null;
            await loadGenerations(true);
            dialog.querySelector('[role="status"]').textContent = `Restored ${entry.path}`;
          } catch (error) {
            button.disabled = false;
            dialog.querySelector('[role="status"]').textContent = error.message;
          }
        };
        actions.append(copy, button);
        row.append(details, time, actions);
        return row;
        })];
      });
      if (page.length < visible.length) {
        const pager = document.createElement("div");
        pager.className = "table-pager";
        const count = document.createElement("span");
        count.textContent = `${page.length} of ${visible.length}`;
        const more = document.createElement("button");
        more.textContent = "Show more";
        more.onclick = () => {
          shown += 100;
          draw();
        };
        pager.append(count, more);
        rows.push(pager);
      }
      list.replaceChildren(...rows);
      if (!list.children.length) list.textContent = "No matching items in Trash.";
    };
    dialog.querySelector("input").oninput = () => {
      shown = 100;
      draw();
    };
    dateSort.onclick = () => {
      newestFirst = !newestFirst;
      dateSort.textContent = `Deleted ${newestFirst ? "↓" : "↑"}`;
      dateSort.setAttribute("aria-sort", newestFirst ? "descending" : "ascending");
      draw();
    };
    draw();
  } catch (error) { dialog.querySelector('[role="status"]').textContent = error.message; }
  dialog.showModal();
};

window.onpopstate = loadLocation;

// Known before the first tab renders, so a restored #tab=notebook never gets a chance to
// try and fail: the server drops those requests for a LAN viewer, this just hides the door.
state.roots = await api("/api/roots").catch(() => ({}));
state.restricted = Boolean(state.roots.restricted);
$("#browse-trash").hidden = state.restricted;
if (!state.restricted) loadTrash().catch(() => {});
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
