// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/** The small pieces every page is built from: list rows, facts, file rows, consoles, devices. */

import { $, api, copyText, state } from "./core.js";

function part(tag, text = "", className = "") {
  const node = document.createElement(tag);
  node.textContent = text;
  if (className) node.className = className;
  return node;
}

export function listItem(name, detail, click, path, mark = null) {
  const button = document.createElement("button");
  button.className = "item";
  button.dataset.path = path;
  button.classList.toggle("picked", state.selected.has(path));
  const label = document.createElement("span");
  label.className = "item-name";
  label.textContent = name;
  const viewport = document.createElement("span");
  viewport.className = "item-name-viewport";
  if (mark) {
    const dot = document.createElement("i");
    dot.className = `run-mark run-mark-${mark.toLowerCase()}`;
    dot.title = `last run: ${mark.toLowerCase()}`;
    viewport.append(dot);
  }
  viewport.append(label);
  button.append(viewport);
  if (detail) {
    const small = document.createElement("small");
    small.textContent = detail;
    button.append(small);
  }
  button.onclick = click;
  return button;
}

export function fact(label, value, kind = "", version = null) {
  const cell = document.createElement("div");
  cell.className = `fact ${kind}`.trim();
  const name = document.createElement("label");
  const text = document.createElement("span");
  name.textContent = label;
  text.textContent = value ?? "—";
  text.title = [value, version].filter(Boolean).join(" ");
  if (version) {
    const tag = document.createElement("i");
    tag.className = "fact-version";
    tag.textContent = version;
    text.append(" ", tag);
  }
  cell.append(name, text);
  return cell.outerHTML;
}

export function fileRow(name, title, click) {
  const row = document.createElement("div");
  row.className = "source-file";
  row.title = title;
  row.append(part("span", name));
  row.onclick = click;
  return row;
}

export function fileFolder(name, rows, open = false) {
  const group = document.createElement("details");
  group.className = "generated-folder";
  group.open = open;
  group.append(part("summary", `${name} · ${rows.length}`), ...rows);
  return group;
}

const COPY_GLYPH = "⧉";

// `stopClick` keeps the copy from reaching a row that is itself clickable.
export function copyPathButton(path, { stopClick = false } = {}) {
  const button = document.createElement("button");
  button.className = "copy-path";
  button.textContent = COPY_GLYPH;
  button.title = "Copy full path";
  button.onclick = async (event) => {
    if (stopClick) event.stopPropagation();
    await copyText(typeof path === "function" ? path() : path);
    button.textContent = "✓";
    setTimeout(() => {
      button.textContent = COPY_GLYPH;
    }, 900);
  };
  return button;
}

export function setPickAll(box, picked, total) {
  box.checked = picked > 0 && picked === total;
  box.indeterminate = picked > 0 && picked < total;
}

export function bindToggle(button, name, isOn, toggle) {
  const draw = () => {
    button.setAttribute("aria-pressed", String(isOn()));
    button.textContent = `${name}: ${isOn() ? "on" : "off"}`;
  };
  button.onclick = () => {
    toggle();
    draw();
  };
  draw();
}

// One row per thing actually knocked on, showing host, port and path only: credentials from
// the config must not reach the page.
export function deviceRows(device) {
  const targets =
    device.kind === "network"
      ? device.ports.map((probe) => ({ ...probe, where: `${device.host}:${probe.port}` }))
      : [{ ok: device.ok, detail: device.detail, where: device.device }];
  if (!targets.length) targets.push({ ok: null, detail: "no port to test", where: device.host });
  return targets.map((target) => {
    const row = document.createElement("div");
    row.className = "device";
    row.dataset.ok = String(target.ok);
    row.innerHTML =
      '<span class="device-dot"></span><strong></strong>' +
      '<span class="device-where"></span><span class="device-detail"></span>';
    row.querySelector("strong").textContent = device.name;
    row.querySelector(".device-where").textContent = target.where;
    row.querySelector(".device-detail").textContent = target.ok ? "" : (target.detail ?? "");
    return row;
  });
}

// Stay on the newest line, unless the reader scrolled up to read an older one.
export function appendConsole(pre, text) {
  const follow = pre.scrollHeight - pre.scrollTop - pre.clientHeight < 40;
  // Every piece goes in through textContent, so log bytes can never become markup.
  text = (pre._ansiTail ?? "") + text;
  // An escape split across two polls would render as junk once; hold the tail back instead.
  const cut = text.match(/\x1b(\[[0-9;]*)?$/);
  pre._ansiTail = cut ? cut[0] : "";
  if (cut) text = text.slice(0, -cut[0].length);
  for (const piece of text.split(/(\x1b\[[0-9;]*[A-Za-z])/)) {
    if (!piece) continue;
    if (piece.startsWith("\x1b")) {
      if (piece.endsWith("m")) pre._ansiClasses = sgrClasses(pre._ansiClasses ?? [], piece);
      continue; // cursor-movement escapes draw nothing on a page
    }
    if (!pre._ansiClasses?.length) {
      pre.append(document.createTextNode(piece));
    } else {
      const span = document.createElement("span");
      span.className = pre._ansiClasses.join(" ");
      span.textContent = piece;
      pre.append(span);
    }
  }
  if (follow) pre.scrollTop = pre.scrollHeight;
}

function sgrClasses(classes, escape) {
  for (const code of escape
    .slice(2, -1)
    .split(";")
    .map((n) => Number(n || 0))) {
    if (code === 0) classes = [];
    else if (code === 1) classes = [...classes.filter((c) => c !== "ansi-b"), "ansi-b"];
    else if (code === 22) classes = classes.filter((c) => c !== "ansi-b");
    else if (code === 39) classes = classes.filter((c) => c === "ansi-b");
    else if ((code >= 30 && code <= 37) || (code >= 90 && code <= 97)) {
      classes = [
        ...classes.filter((c) => !c.startsWith("ansi-3") && !c.startsWith("ansi-9")),
        `ansi-${code}`,
      ];
    }
  }
  return classes;
}

export async function consoleExcerpt(path, lines = 50) {
  const slice = await api(`/api/console?path=${encodeURIComponent(path)}`).catch(() => null);
  const pre = document.createElement("pre");
  pre.className = "console";
  appendConsole(pre, (slice?.text ?? "").trimEnd().split("\n").slice(-lines).join("\n"));
  return pre;
}

export function emptyState({ eyebrow = "", title = "", detail = "", spinner = false } = {}) {
  const empty = document.createElement("div");
  empty.className = "empty";
  if (spinner) empty.append(part("div", "", "spinner"));
  if (eyebrow) empty.append(part("span", eyebrow));
  empty.append(part("h1", title), part("p", detail));
  return empty;
}

export function showEmpty(options) {
  const empty = emptyState(options);
  $("#content").replaceChildren(empty);
  return empty;
}

// One pair of listeners for every menu on the page. Binding per menu leaked a pair per open.
document.addEventListener("click", (event) => {
  document.querySelectorAll("details.picker[open]").forEach((menu) => {
    if (!menu.contains(event.target)) menu.open = false;
  });
});

document.addEventListener("keydown", (event) => {
  if (event.key !== "Escape") return;
  document.querySelectorAll("details.picker[open]").forEach((menu) => {
    menu.open = false;
  });
});
