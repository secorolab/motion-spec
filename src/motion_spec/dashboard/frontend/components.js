// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * The pieces every page is built from: a row in the list, a fact in a header, a
 * terminal that keeps its colours, a device and what it answered.
 */

import { $, api, state } from "./core.js";

export function listItem(name, detail, click, path) {
  const button = document.createElement("button");
  button.className = "item";
  button.dataset.path = path;
  button.classList.toggle("picked", state.selected.has(path));
  const label = document.createElement("span");
  label.className = "item-name";
  label.textContent = name;
  const viewport = document.createElement("span");
  viewport.className = "item-name-viewport";
  viewport.append(label);
  button.append(viewport);
  if (detail) {
    const small = document.createElement("small");
    small.textContent = detail;
    button.append(small);
  }
  button.onclick = (event) => click(event);
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
  // A version is about the thing, not the thing: it rides along in its own hand.
  if (version) {
    const tag = document.createElement("i");
    tag.className = "fact-version";
    tag.textContent = version;
    text.append(" ", tag);
  }
  cell.append(name, text);
  return cell.outerHTML;
}

// One row per thing actually knocked on: a network device has a row per port, a serial one its
// device node. Host, port and path only -- what the config holds to log in with stays there.
export function deviceRows(device) {
  const targets = device.kind === "network"
    ? device.ports.map((probe) => ({ ...probe, where: `${device.host}:${probe.port}` }))
    : [{ ok: device.ok, detail: device.detail, where: device.device }];
  if (!targets.length) targets.push({ ok: null, detail: "no port to test", where: device.host });
  return targets.map((target) => {
    const row = document.createElement("div");
    row.className = "device";
    row.dataset.ok = String(target.ok);
    row.innerHTML = '<span class="device-dot"></span><strong></strong>'
      + '<span class="device-where"></span><span class="device-detail"></span>';
    row.querySelector("strong").textContent = device.name;
    row.querySelector(".device-where").textContent = target.where;
    row.querySelector(".device-detail").textContent = target.ok ? "" : target.detail ?? "";
    return row;
  });
}

// Stay on the newest line, unless the reader scrolled up to read an older one.
export function appendConsole(pre, text) {
  const follow = pre.scrollHeight - pre.scrollTop - pre.clientHeight < 40;
  // The log is terminal output: render its SGR color escapes as spans (built with
  // textContent, so log content can never become markup) and draw nothing for the rest.
  text = (pre._ansiTail ?? "") + text;
  // An escape split across two polls would render as junk once; hold the tail back instead.
  const cut = text.match(/\x1b(\[[0-9;]*)?$/);
  pre._ansiTail = cut ? cut[0] : "";
  if (cut) text = text.slice(0, -cut[0].length);
  for (const part of text.split(/(\x1b\[[0-9;]*[A-Za-z])/)) {
    if (!part) continue;
    if (part.startsWith("\x1b")) {
      if (part.endsWith("m")) pre._ansiClasses = sgrClasses(pre._ansiClasses ?? [], part);
      continue;   // cursor-movement escapes draw nothing on a page
    }
    if (!pre._ansiClasses?.length) {
      pre.append(document.createTextNode(part));
    } else {
      const span = document.createElement("span");
      span.className = pre._ansiClasses.join(" ");
      span.textContent = part;
      pre.append(span);
    }
  }
  if (follow) pre.scrollTop = pre.scrollHeight;
}

// One SGR escape applied to the classes in force: reset clears, bold and the 16 foreground
// colors map to .ansi-* rules; anything else changes nothing.
export function sgrClasses(classes, escape) {
  for (const code of escape.slice(2, -1).split(";").map((n) => Number(n || 0))) {
    if (code === 0) classes = [];
    else if (code === 1) classes = [...classes.filter((c) => c !== "ansi-b"), "ansi-b"];
    else if (code === 22) classes = classes.filter((c) => c !== "ansi-b");
    else if (code === 39) classes = classes.filter((c) => c === "ansi-b");
    else if ((code >= 30 && code <= 37) || (code >= 90 && code <= 97)) {
      classes = [...classes.filter((c) => !c.startsWith("ansi-3") && !c.startsWith("ansi-9")), `ansi-${code}`];
    }
  }
  return classes;
}

// The runner's last words, for a page that would otherwise name a file to go read in a terminal.
export async function consoleExcerpt(path, lines = 50) {
  const slice = await api(`/api/console?path=${encodeURIComponent(path)}`).catch(() => null);
  const pre = document.createElement("pre");
  pre.className = "console";
  appendConsole(pre, (slice?.text ?? "").trimEnd().split("\n").slice(-lines).join("\n"));
  return pre;
}

// The page with nothing on it yet: what is missing, why, and whatever there is to do about it.
// Six pages said this in six hand-written strings; they say it here instead.
export function emptyState({ eyebrow = "", title = "", detail = "", spinner = false } = {}) {
  const empty = document.createElement("div");
  empty.className = "empty";
  if (spinner) empty.append(part("div", "", "spinner"));
  if (eyebrow) empty.append(part("span", eyebrow));
  empty.append(part("h1", title), part("p", detail));
  return empty;
}

// Put one on the page, and hand it back for whatever else belongs in it.
export function showEmpty(options) {
  const empty = emptyState(options);
  $("#content").replaceChildren(empty);
  return empty;
}

function part(tag, text = "", className = "") {
  const node = document.createElement(tag);
  node.textContent = text;
  if (className) node.className = className;
  return node;
}

// Every menu on the page closes the same way: a click anywhere else, or escape. One pair of
// listeners for all of them, rather than one pair per menu ever built -- a generation page
// opened twenty times used to leave twenty behind, each still holding a menu long gone.
document.addEventListener("click", (event) => {
  document.querySelectorAll("details.picker[open]").forEach((menu) => {
    if (!menu.contains(event.target)) menu.open = false;
  });
});

document.addEventListener("keydown", (event) => {
  if (event.key !== "Escape") return;
  document.querySelectorAll("details.picker[open]").forEach((menu) => { menu.open = false; });
});
