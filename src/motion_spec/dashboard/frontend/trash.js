// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/** What was moved to the desktop trash, and putting it back, in one dialog over the page. */

import { api, copyText, post, showError, stampText, state } from "./core.js";
import { loadGenerations } from "./generations.js";

const PAGE_SIZE = 100;

const DIALOG_MARKUP =
  '<header><div><span class="eyebrow">RECOVERABLE FILES</span><h2>Trash</h2></div><button class="close trash-action">Close</button></header><p>Only items from this generation root are shown. Existing paths will never be overwritten.</p><div class="browser-tools"><input type="search" placeholder="Filter trashed paths" aria-label="Filter trashed paths"></div><div class="run-header trash-columns"><span>Path</span><button class="trash-date-sort" aria-sort="descending">Deleted ↓</button><span>Actions</span></div><div class="trash-entries"></div><p role="status"></p>';

let entries;

export const loadTrash = () => (entries ??= api("/api/trash"));

export function forgetTrash() {
  entries = null;
}

export async function browseTrash() {
  let trashed;
  try {
    trashed = await loadTrash();
  } catch (error) {
    return showError(error);
  }
  const dialog = document.createElement("dialog");
  dialog.className = "trash-browser";
  dialog.innerHTML = DIALOG_MARKUP;
  document.body.append(dialog);
  dialog.querySelector(".close").onclick = () => dialog.close();
  dialog.onclose = () => dialog.remove();
  try {
    fillTrash(dialog, trashed);
  } catch (error) {
    dialog.querySelector('[role="status"]').textContent = error.message;
  }
  dialog.showModal();
}

function fillTrash(dialog, trashed) {
  const list = dialog.querySelector(".trash-entries");
  const dateSort = dialog.querySelector(".trash-date-sort");
  const status = dialog.querySelector('[role="status"]');
  const search = dialog.querySelector("input");
  // Parsed once: sorting on Date objects builds two of them per comparison, per keystroke.
  const when = new Map(trashed.map((entry) => [entry, Date.parse(entry.deleted_at ?? "") || 0]));
  let newestFirst = true;
  let shown = PAGE_SIZE;
  const draw = () => {
    const needle = search.value.toLowerCase();
    const visible = trashed
      .filter((entry) => entry.path.toLowerCase().includes(needle))
      .sort((left, right) => (newestFirst ? 1 : -1) * (when.get(right) - when.get(left)));
    const page = visible.slice(0, shown);
    const rows = [...byDay(page)].flatMap(([day, dayEntries]) => {
      const heading = document.createElement("span");
      heading.className = "eyebrow";
      heading.textContent = day;
      return [heading, ...dayEntries.map((entry) => trashRow(entry, trashed, status, draw))];
    });
    if (page.length < visible.length)
      rows.push(
        moreButton(page.length, visible.length, () => {
          shown += PAGE_SIZE;
          draw();
        }),
      );
    list.replaceChildren(...rows);
    if (!list.children.length) list.textContent = "No matching items in Trash.";
  };
  search.oninput = () => {
    shown = PAGE_SIZE;
    draw();
  };
  dateSort.onclick = () => {
    newestFirst = !newestFirst;
    dateSort.textContent = `Deleted ${newestFirst ? "↓" : "↑"}`;
    dateSort.setAttribute("aria-sort", newestFirst ? "descending" : "ascending");
    draw();
  };
  draw();
}

function byDay(page) {
  const groups = new Map();
  page.forEach((entry) => {
    const label = dayLabel(entry.deleted_at);
    const day = groups.get(label);
    if (day) day.push(entry);
    else groups.set(label, [entry]);
  });
  return groups;
}

function dayLabel(value) {
  if (!value) return "Unknown time";
  const date = new Date(value);
  const today = new Date();
  const yesterday = new Date(today);
  yesterday.setDate(today.getDate() - 1);
  if (date.toDateString() === today.toDateString()) return "Today";
  if (date.toDateString() === yesterday.toDateString()) return "Yesterday";
  return date.toLocaleDateString(undefined, {
    weekday: "long",
    day: "numeric",
    month: "long",
    year: "numeric",
  });
}

function trashRow(entry, trashed, status, redraw) {
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
  const restore = document.createElement("button");
  restore.textContent = entry.exists ? "Path exists" : "Restore";
  restore.disabled = entry.exists;
  restore.onclick = async () => {
    restore.disabled = true;
    try {
      await post("/api/restore", { uri: entry.uri });
      trashed.splice(trashed.indexOf(entry), 1);
      redraw();
      state.cache = {};
      state.generation = null;
      await loadGenerations(true);
      status.textContent = `Restored ${entry.path}`;
    } catch (error) {
      restore.disabled = false;
      status.textContent = error.message;
    }
  };
  actions.append(copy, restore);
  row.append(details, time, actions);
  return row;
}

function moreButton(count, total, showMore) {
  const pager = document.createElement("div");
  pager.className = "table-pager";
  const label = document.createElement("span");
  label.textContent = `${count} of ${total}`;
  const more = document.createElement("button");
  more.textContent = "Show more";
  more.onclick = showMore;
  pager.append(label, more);
  return pager;
}
