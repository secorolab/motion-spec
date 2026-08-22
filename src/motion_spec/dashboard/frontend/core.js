// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * Everything every view needs: the page's shared state, the server, and the small
 * feedback the whole dashboard speaks in.
 */

import { showEmpty } from "./components.js";

export let plotKeys = 0;

export const nextPlotKey = () => String(++plotKeys);

export const state = { replay: null, queries: [], query: -1, anchor: null, runPath: null, generationPath: null, tab: "logs", frame: 0, charts: [], selected: new Set(), timer: null, roots: {}, cache: {}, listRequest: 0, live: null, speed: 1, consoleWatch: null, livePlots: new Map(), pendingSignals: new Set(), liveBuffer: new Map(), activeMotion: null, autoPlot: null };

export const $ = (selector) => document.querySelector(selector);

export const $$ = (selector) => [...document.querySelectorAll(selector)];

export let snackTimer;

export function snack(message) {
  $("#snack").textContent = message;
  $("#snack").classList.add("visible");
  clearTimeout(snackTimer);
  snackTimer = setTimeout(() => $("#snack").classList.remove("visible"), 1400);
}

export async function copyText(value) {
  await navigator.clipboard.writeText(value);
  snack("Copied path");
}

export async function api(path) {
  const response = await fetch(path);
  const data = await response.json();
  if (!response.ok) throw Error(data.error);
  return data;
}

export async function post(path, body) {
  const response = await fetch(path, {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
  });
  const data = await response.json();
  // A refusal can carry what it found out; the message alone would throw that away.
  if (!response.ok) throw Object.assign(Error(data.error), data);
  return data;
}

export function stampText(iso) {
  if (!iso) return "unknown time";
  return new Date(iso).toLocaleString(undefined, {
    day: "2-digit", month: "short", year: "numeric",
    hour: "2-digit", minute: "2-digit", hour12: false,
  });
}

export function formatBytes(bytes) {
  return bytes >= 1024 ** 3
    ? `${(bytes / 1024 ** 3).toFixed(1)} GB`
    : `${(bytes / 1024 / 1024).toFixed(1)} MB`;
}

export function askConfirm({ message, confirmLabel = "Confirm" }) {
  return new Promise((resolve) => {
    const dialog = document.createElement("dialog");
    dialog.className = "ask";
    dialog.innerHTML = '<p></p><div class="ask-actions"><button value="no">Cancel</button>'
      + '<button value="yes" class="ask-yes"></button></div>';
    dialog.querySelector("p").textContent = message;
    dialog.querySelector(".ask-yes").textContent = confirmLabel;
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
        dialog.close("yes");
      }
    };
    document.body.append(dialog);
    dialog.showModal();
    dialog.querySelector(".ask-yes").focus();
  });
}

export function showError(error) {
  showEmpty({ eyebrow: "ERROR", title: "Could not load data.", detail: error.message });
}
