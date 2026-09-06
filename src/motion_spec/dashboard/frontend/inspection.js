// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

import { $, snack, state } from "./core.js";
import { addPlot } from "./plots.js";
import { seek } from "./run.js";

const key = (kind, path) => `motion-spec.${kind}.${state.roots.logs}.${path}`;
const read = (kind, path) => {
  try { return JSON.parse(localStorage.getItem(key(kind, path))); }
  catch { return null; }
};

function snapshot() {
  return {
    frame: state.frame,
    camera: state.camera,
    panel: $(".replay-tabs .active")?.dataset.panel,
    plots: [...document.querySelectorAll("#plots .plot-card")].map((card) => card.inspection?.()).filter(Boolean),
  };
}

/** Save only an actual archived replay, never a pending or streaming page. */
export function saveInspection() {
  if (!state.runPath || !$(".replay") || state.replay?.pending || state.following || state.restoringInspection) return;
  try { localStorage.setItem(key("inspection", state.runPath), JSON.stringify(snapshot())); }
  catch { snack("Browser storage is full; inspection view was not saved"); }
}

function applyView(view, preset = false) {
  if (!view || !Array.isArray(view.plots)) return;
  state.restoringInspection = true;
  try {
    document.querySelectorAll("#plots .remove-plot").forEach((button) => button.click());
    const missing = new Set();
    for (const plot of view.plots) {
      const signals = plot.signals.filter((signal) => {
        const exists = state.replay.signals.includes(signal);
        if (!exists) missing.add(signal);
        return exists;
      });
      if (plot.signals.length && !signals.length) continue;
      addPlot(signals, plot.title, plot.detail, {
        constraint: preset ? null : plot.constraint,
        gains: preset ? null : plot.gains,
        zoom: plot.zoom,
      });
    }
    if (!preset) {
      if (Number.isInteger(view.frame)) seek(view.frame);
      const camera = [...document.querySelectorAll(".video-pick")].find((pick) => pick.dataset.camera === view.camera);
      camera?.click();
      const panel = [...document.querySelectorAll(".replay-tabs button")].find((button) => button.dataset.panel === view.panel);
      panel?.click();
    }
    if (missing.size) snack(`Unavailable signals skipped: ${[...missing].join(", ")}`);
  } finally { state.restoringInspection = false; }
}

/** An explicit moment in a shared URL takes precedence over the local cursor. */
export function restoreInspection(path) {
  const view = read("inspection", path);
  const url = new URLSearchParams(location.hash.slice(1));
  const frame = url.get("frame");
  applyView(view);
  if (frame !== null && /^\d+$/.test(frame)) seek(Number(frame));
  if (url.has("panel")) {
    [...document.querySelectorAll(".replay-tabs button")].find((button) => button.dataset.panel === url.get("panel"))?.click();
  }
}

/** Named presets carry signal choices across runs; time windows remain run-specific. */
export function bindInspection(path) {
  const controls = document.createElement("div");
  controls.className = "inspection-controls";
  controls.innerHTML = '<button class="copy-moment">Copy link to this moment</button><input class="preset-name" aria-label="Plot preset name" placeholder="Preset name" maxlength="100"><button class="save-preset">Save plots</button><select class="presets" aria-label="Plot preset"></select><button class="load-preset">Load</button><button class="delete-preset">Delete preset</button>';
  $(".chart-controls").after(controls);
  const model = state.generationPath.split("/")[0];
  let presets = read("plot-presets", model) ?? {};
  const picker = controls.querySelector(".presets");
  const draw = () => {
    picker.replaceChildren(new Option("Choose plot preset", ""), ...Object.keys(presets).sort().map((name) => new Option(name, name)));
  };
  const store = () => {
    localStorage.setItem(key("plot-presets", model), JSON.stringify(presets));
    draw();
  };
  controls.querySelector(".copy-moment").onclick = async () => {
    const url = new URL(location.href);
    const params = new URLSearchParams(url.hash.slice(1));
    params.set("frame", state.frame);
    params.set("panel", "plots");
    url.hash = params;
    try { await navigator.clipboard.writeText(url.href); snack("Copied moment link"); }
    catch (error) { snack(error.message); }
  };
  controls.querySelector(".save-preset").onclick = () => {
    const name = controls.querySelector(".preset-name").value.trim();
    if (!name) return snack("Name the plot preset first");
    presets = { ...presets, [name]: snapshot() };
    try { store(); picker.value = name; snack("Plot preset saved"); } catch (error) { snack(error.message); }
  };
  controls.querySelector(".load-preset").onclick = () => {
    if (picker.value) applyView(presets[picker.value], true);
  };
  controls.querySelector(".delete-preset").onclick = () => {
    if (!picker.value) return;
    delete presets[picker.value];
    try { store(); } catch (error) { snack(error.message); }
  };
  draw();
}

window.addEventListener("pagehide", saveInspection);
