// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/** Shared page state, the fetch wrappers, and the small feedback every view uses. */

import { showEmpty } from "./components.js";

export const state = {
  tab: "logs",
  roots: {},
  cache: {},
  listRequest: 0,
  restricted: false,
  anchor: null,
  selected: new Set(),
  generationPath: null,
  runPath: null,
  replay: null,
  frame: 0,
  timer: null,
  speed: 1,
  spanOverlay: null,
  live: null,
  consoleWatch: null,
  activeMotion: null,
  autoPlot: null,
  rosTopic: null,
  charts: [],
  livePlots: new Map(),
  pendingSignals: new Set(),
  liveBuffer: new Map(),
  queries: [],
  query: -1,
};

// The server answers these with 403 for a LAN viewer; hiding them here too keeps the reader
// out of a dead end. The sources tab is reachable, and hides its write controls in sources.js.
export const RESTRICTED_TABS = ["notebook", "health"];

let plotKeys = 0;

export const nextPlotKey = () => String(++plotKeys);

export const $ = (selector) => document.querySelector(selector);

export const $$ = (selector) => [...document.querySelectorAll(selector)];

let snackTimer;

export function snack(message) {
  const snackbar = $("#snack");
  snackbar.textContent = message;
  snackbar.classList.add("visible");
  clearTimeout(snackTimer);
  snackTimer = setTimeout(() => snackbar.classList.remove("visible"), 1400);
}

export const snackError = (error) => snack(error.message);

export async function copyText(value) {
  await navigator.clipboard.writeText(value);
  snack("Copied path");
}

// A refusal is not a failure, so it must not say "could not load": nothing broke.
export function showError(error) {
  const refused = error.status === 403;
  showEmpty({
    eyebrow: refused ? "NOT ALLOWED" : "ERROR",
    title: refused ? "This is not allowed here." : "Could not load data.",
    detail: error.message,
  });
}

async function request(path, options) {
  const response = await fetch(path, options);
  const data = await response.json();
  if (!response.ok) throw Object.assign(Error(data.error), data, { status: response.status });
  return data;
}

export const api = (path) => request(path);

export const post = (path, body) =>
  request(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });

// A private window refuses storage; a preference nobody could save must not fail the page.
export function readStored(key, fallback = null) {
  try {
    return localStorage.getItem(key) ?? fallback;
  } catch {
    return fallback;
  }
}

export function writeStored(key, value) {
  try {
    localStorage.setItem(key, value);
  } catch {
    // storage is unavailable
  }
}

export const seconds = (value) =>
  value === null || value === undefined ? "—" : `${value.toFixed(2)} s`;

export function stampText(iso) {
  if (!iso) return "unknown time";
  return new Date(iso).toLocaleString(undefined, {
    day: "2-digit",
    month: "short",
    year: "numeric",
    hour: "2-digit",
    minute: "2-digit",
    hour12: false,
  });
}

export function formatBytes(bytes) {
  return bytes >= 1024 ** 3
    ? `${(bytes / 1024 ** 3).toFixed(1)} GB`
    : `${(bytes / 1024 / 1024).toFixed(1)} MB`;
}

export function askConfirm({ message, confirmLabel = "Confirm", require = "" }) {
  return new Promise((resolve) => {
    const dialog = document.createElement("dialog");
    dialog.className = "ask";
    dialog.innerHTML =
      '<p></p><input class="ask-typed" hidden autocomplete="off" spellcheck="false">' +
      '<div class="ask-actions"><button value="no">Cancel</button>' +
      '<button value="yes" class="ask-yes"></button></div>';
    dialog.querySelector("p").textContent = message;
    const yes = dialog.querySelector(".ask-yes");
    const typed = dialog.querySelector(".ask-typed");
    yes.textContent = confirmLabel;
    const settled = () => !require || typed.value.trim() === require;
    if (require) {
      typed.hidden = false;
      typed.placeholder = require;
      yes.disabled = true;
      typed.oninput = () => {
        yes.disabled = !settled();
      };
    }
    dialog.querySelectorAll("button").forEach((button) => {
      button.onclick = () => dialog.close(button.value);
    });
    dialog.onclose = () => {
      dialog.remove();
      resolve(dialog.returnValue === "yes");
    };
    dialog.onkeydown = (event) => {
      if (event.key === "Enter") {
        event.preventDefault();
        if (settled()) dialog.close("yes");
      }
    };
    document.body.append(dialog);
    dialog.showModal();
    (require ? typed : yes).focus();
  });
}
