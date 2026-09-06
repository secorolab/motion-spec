// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

import { api, post, snack, state } from "./core.js";

/** Edit user metadata without renaming the archive or its generated sources. */
export async function annotationEditor(path, changed = () => {}) {
  let data = await api(`/api/annotations?path=${encodeURIComponent(path)}`);
  const box = document.createElement("form");
  box.className = "annotation-editor";
  box.innerHTML = '<button type="button" class="pin"></button><label>Label<input name="label" maxlength="200" placeholder="A name to remember"></label><label>Tags<input name="tags" placeholder="tags, comma separated"></label><button class="save">Save</button><span role="status"></span>';
  const label = box.elements.label;
  const tags = box.elements.tags;
  const pin = box.querySelector(".pin");
  const status = box.querySelector('[role="status"]');
  const draw = () => {
    label.value = data.label;
    tags.value = data.tags.join(", ");
    pin.textContent = data.pinned ? "★ Pinned" : "☆ Pin";
    pin.setAttribute("aria-pressed", String(data.pinned));
  };
  const save = async (changes) => {
    try {
      data = await post("/api/annotations", { path, changes });
      state.cache.generations = null;
      status.textContent = "Saved";
      draw();
      await changed(data);
    } catch (error) { status.textContent = error.message; }
  };
  pin.onclick = () => save({ pinned: !data.pinned });
  box.onsubmit = (event) => {
    event.preventDefault();
    save({ label: label.value, tags: tags.value.split(",").map((tag) => tag.trim()).filter(Boolean) });
  };
  draw();
  if (path.includes("/runs/")) {
    const button = document.createElement("button");
    button.type = "button";
    const current = await api(`/api/baseline?path=${encodeURIComponent(path)}`);
    let selected = current.baseline === path;
    const title = () => { button.textContent = selected ? "Clear model baseline" : "Use as model baseline"; };
    title();
    button.onclick = async () => {
      try {
        await post("/api/baseline", { path, enabled: !selected });
        selected = !selected;
        state.cache.generations = null;
        title();
        snack(selected ? "Model baseline saved and protected" : "Model baseline cleared");
        await changed(data);
      } catch (error) { status.textContent = error.message; }
    };
    box.append(button);
  }
  if (state.restricted) box.querySelectorAll("input, button").forEach((node) => { node.disabled = true; });
  return box;
}

export const NOTES_MARKUP = '<div class="notes-compose"><textarea class="note-text" rows="3" placeholder="What happened, what to try next — Ctrl+Enter adds"></textarea><div class="notes-compose-bar"><input class="note-tags" placeholder="tags, comma separated"><button class="note-add">add note</button><span class="notes-state" role="status"></span></div></div><div class="notes-bar" hidden><input type="checkbox" class="pick notes-pick-all" title="Select every note"><span class="notes-picked"></span><button class="note-action notes-delete" disabled>delete selected</button></div><div class="notes-list"></div>';
