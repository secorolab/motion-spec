// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

import { api, askConfirm, post, snack, state } from "./core.js";

export const splitTags = (value) =>
  value
    .split(",")
    .map((tag) => tag.trim())
    .filter(Boolean);

/** Edit a run's or generation's metadata without renaming the archive on disk. */
export async function annotationEditor(path, changed = () => {}) {
  let data = await api(`/api/annotations?path=${encodeURIComponent(path)}`);
  const box = document.createElement("form");
  box.className = "annotation-editor";
  box.innerHTML = `
    <button type="button" class="pin"></button>
    <button type="button" class="lock"></button>
    <label>Label<input name="label" maxlength="200" placeholder="A name to remember"></label>
    <label>Tags<input name="tags" placeholder="tags, comma separated"></label>
    <button class="save">Save</button>
    <span role="status"></span>
  `;
  const label = box.elements.label;
  const tags = box.elements.tags;
  const pin = box.querySelector(".pin");
  const lock = box.querySelector(".lock");
  const status = box.querySelector('[role="status"]');
  const draw = () => {
    label.value = data.label;
    tags.value = data.tags.join(", ");
    pin.textContent = data.pinned ? "★ Pinned" : "☆ Pin";
    pin.setAttribute("aria-pressed", String(data.pinned));
    lock.textContent = data.protected ? "Protected" : "Protect";
    lock.setAttribute("aria-pressed", String(data.protected));
    lock.title = data.protected
      ? "Deleting is refused; removing this asks for the folder name"
      : "Refuse deletion of this bundle";
  };
  const update = async (endpoint, body, saved) => {
    try {
      data = await post(endpoint, { path, ...body });
      state.cache.generations = null;
      status.textContent = saved(data);
      draw();
      await changed(data);
    } catch (error) {
      status.textContent = error.message;
    }
  };
  const save = (changes) => update("/api/annotations", { changes }, () => "Saved");
  pin.onclick = () => save({ pinned: !data.pinned });
  lock.onclick = async () => {
    const name = path.split("/").pop();
    const confirm = data.protected ? name : "";
    if (
      data.protected &&
      !(await askConfirm({
        message: `Remove the protection on ${name}? Type its name to confirm; it can then be deleted.`,
        confirmLabel: "Remove protection",
        require: name,
      }))
    )
      return;
    update("/api/protect", { enabled: !data.protected, confirm }, (saved) =>
      saved.protected ? "Protected from deletion" : "Protection removed",
    );
  };
  box.onsubmit = (event) => {
    event.preventDefault();
    save({ label: label.value, tags: splitTags(tags.value) });
  };
  draw();
  if (path.includes("/runs/")) {
    const button = document.createElement("button");
    button.type = "button";
    const current = await api(`/api/baseline?path=${encodeURIComponent(path)}`);
    let selected = current.baseline === path;
    const title = () => {
      button.textContent = selected ? "Clear model baseline" : "Use as model baseline";
    };
    title();
    button.onclick = async () => {
      try {
        await post("/api/baseline", { path, enabled: !selected });
        selected = !selected;
        state.cache.generations = null;
        title();
        snack(selected ? "Model baseline saved and protected" : "Model baseline cleared");
        await changed(data);
      } catch (error) {
        status.textContent = error.message;
      }
    };
    box.append(button);
  }
  return box;
}

// Shared with the run page's Notes panel, which wraps it in a section of its own.
export const NOTES_MARKUP = `
  <div class="notes-compose">
    <textarea
      class="note-text"
      rows="3"
      placeholder="What happened, what to try next — Ctrl+Enter adds"
    ></textarea>
    <div class="notes-compose-bar">
      <input class="note-tags" placeholder="tags, comma separated" spellcheck="false">
      <button class="note-add">add note</button>
      <span class="notes-state" role="status"></span>
    </div>
  </div>
  <div class="notes-bar" hidden>
    <input type="checkbox" class="pick notes-pick-all" title="Select every note">
    <span class="notes-picked"></span>
    <button class="note-action notes-delete" disabled>delete selected</button>
  </div>
  <div class="notes-list"></div>
`;
