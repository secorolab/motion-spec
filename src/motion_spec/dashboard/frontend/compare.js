// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

/**
 * Did this change alter behaviour? Two generations, three axes, in one order.
 *
 * Model first, then coordination, then signals -- a duration that moved with no model change
 * is a flaky run or a real nondeterminism, and that is the distinction the page exists for.
 * Nothing here is coloured or ranked: which direction is desirable is the reader's to say.
 */

import { $, api, state } from "./core.js";
import { section, table } from "./views.js";

export const COMPARE_MARKUP =
  '<div class="views"><div class="compare-bar">'
  + '<span class="compare-left"></span>'
  + '<select class="compare-gen"></select><select class="compare-run"></select>'
  + '</div><div class="compare-slot"></div></div>';

const dash = "—";

// Enough digits to see a gain move, few enough to scan a column of them.
function num(value) {
  if (value === null || value === undefined) return dash;
  if (typeof value !== "number") return String(value);
  if (value === 0) return "0";
  const size = Math.abs(value);
  return size >= 1e4 || size < 1e-3 ? value.toExponential(2) : String(Number(value.toPrecision(4)));
}

const signed = (value) =>
  value === null || value === undefined ? dash : `${value >= 0 ? "+" : "−"}${num(Math.abs(value))}`;

const seconds = (value) => (value === null || value === undefined ? dash : `${value.toFixed(2)} s`);

// The tail of an IRI is what a table cell can show of one; the row's title carries the term.
const short = (value) =>
  typeof value === "string" && value.includes("/") ? value.split(/[/#]/).pop() : num(value);

const list = (values) => (values && values.length ? values.map(short).join(", ") : dash);

// A statement about the comparison itself, not about either side: shown before any number,
// because a reader who does not know two runs took different paths will read the durations.
function notes(lines) {
  const body = document.createElement("ul");
  body.className = "compare-notes";
  lines.forEach((line) => {
    const item = document.createElement("li");
    item.textContent = line;
    body.append(item);
  });
  return section("Not fully comparable", "read the deltas below with these in mind", body);
}

function renderModel(model) {
  const parts = document.createElement("div");
  parts.className = "views";
  if (model.values.length) {
    parts.append(section("Values", "a gain, a band, a limit", table(
      ["term", "property", "left", "right"],
      model.values.map((row) => ({
        cells: [row.name, row.property, num(row.left), num(row.right)],
        title: row.iri,
      })),
    )));
  }
  ["added", "removed"].forEach((kind) => {
    if (!model[kind].length) return;
    parts.append(section(`Terms ${kind}`, "", table(
      ["term", "kind"],
      model[kind].map((row) => ({ cells: [row.name, row.types.join(", ") || dash], title: row.iri })),
    )));
  });
  if (model.structure.length) {
    parts.append(section("Structure", "an operand, a controller set, a solver's limits", table(
      ["term", "property", "was", "now"],
      model.structure.map((row) => ({
        cells: [row.name, row.property, list(row.removed), list(row.added)],
        title: row.iri,
      })),
    )));
  }
  if (!parts.children.length) {
    const same = document.createElement("p");
    same.className = "view-note";
    same.textContent = "the two design graphs are identical";
    parts.append(same);
  }
  return section("Model", "over the graph, not the text", parts);
}

function renderCoordination(coordination) {
  const parts = document.createElement("div");
  parts.className = "views";
  parts.append(section("Activities", "a re-entry is its own row, never summed", table(
    ["activity", "left", "right", "delta"],
    coordination.activities.map((row) => ({
      cells: [row.name, seconds(row.left_s), seconds(row.right_s), signed(row.delta_s)],
      tint: row.left_s === null || row.right_s === null,
    })),
  )));
  if (coordination.gates.length) {
    parts.append(section("Gates", "waits and re-arms, never a verdict", table(
      ["gate", "left wait", "right wait", "delta", "re-arms"],
      coordination.gates.map((row) => ({
        cells: [
          row.name,
          seconds(row.left_waited_s),
          seconds(row.right_waited_s),
          signed(row.delta_s),
          `${row.left_rearms ?? dash} → ${row.right_rearms ?? dash}`,
        ],
        tint: row.left_fired !== row.right_fired,
      })),
    )));
  }
  const note = coordination.order_differs
    ? "the control-flow order differs — durations along different paths are not comparable"
    : "same control-flow order";
  return section("Coordination", note, parts);
}

function renderSignals(signals) {
  const parts = document.createElement("div");
  parts.className = "views";
  signals.activities.forEach((activity) => {
    const rows = [
      ...activity.signals.map((row) => ({
        cells: [
          row.signal,
          `peak ${num(row.left_peak)} → ${num(row.right_peak)}`,
          signed(row.delta_peak),
          `pk-pk ${signed(row.delta_pk_pk)} · rms ${signed(row.delta_rms)}`,
          row.gate ?? dash,
        ],
      })),
      ...activity.contact.map((row) => ({
        cells: [
          "contact",
          `peak ${num(row.left_peak_n)} N → ${num(row.right_peak_n)} N`,
          signed(row.delta_peak_n),
          `ripple ${num(row.left_ripple)} → ${num(row.right_ripple)}`,
          row.gate ?? dash,
        ],
      })),
      ...activity.modes.map((row) => ({
        cells: [
          `coherent mode · ${row.solver}`,
          `${num(row.left_mode_hz)} Hz → ${num(row.right_mode_hz)} Hz`,
          signed(row.delta_mode_hz),
          `amplitude ${num(row.left_mean_amplitude)} → ${num(row.right_mean_amplitude)}`,
          dash,
        ],
        // A mode that appeared or vanished is the finding; a mode that moved a band is not.
        tint: (row.left_mode_hz === null) !== (row.right_mode_hz === null),
      })),
    ];
    const entry = activity.entry ? ` (entry ${activity.entry + 1})` : "";
    parts.append(section(activity.name + entry, "", table(
      ["what", "left → right", "delta", "and", "gate"],
      rows,
    )));
  });
  if (signals.saturation.length) {
    parts.append(section("Saturation", "per run, as the report gives it", table(
      ["joint", "left ticks", "right ticks", "delta", "states"],
      signals.saturation.map((row) => ({
        cells: [
          `${row.solver} joint ${row.joint}`,
          num(row.left_ticks),
          num(row.right_ticks),
          signed(row.delta_ticks),
          [...new Set([...row.left_states, ...row.right_states])].join(", ") || dash,
        ],
      })),
    )));
  }
  return section("Signals", `restricted to ${signals.restricted_to}`, parts);
}

function render(slot, payload) {
  const parts = [];
  if (payload.comparable.notes.length) parts.push(notes(payload.comparable.notes));
  parts.push(renderModel(payload.model));
  if (payload.coordination.activities.length) parts.push(renderCoordination(payload.coordination));
  if (payload.signals.activities.length || payload.signals.saturation.length) {
    parts.push(renderSignals(payload.signals));
  }
  slot.replaceChildren(...parts);
}

// The whole comparison in the URL, so it can be pasted to a colleague: the left side is the
// run already open, the right side is what was picked here.
function remember(generation, run) {
  const view = new URLSearchParams(location.hash.slice(1));
  view.set("panel", "compare");
  for (const [key, value] of [["cmp_gen", generation], ["cmp_run", run]]) {
    if (value) view.set(key, value);
    else view.delete(key);
  }
  history.replaceState(null, "", `#${view}`);
}

async function load(shell, generation, run) {
  const slot = shell.querySelector(".compare-slot");
  if (!generation) return slot.replaceChildren();
  slot.textContent = "comparing…";
  // Runs are a pair or neither: a model diff against a generation with no run picked is
  // still worth showing, and the route says so in its notes.
  const query = new URLSearchParams({ left: state.generationPath, right: generation });
  if (run) {
    query.set("left_run", state.runPath);
    query.set("right_run", run);
  }
  render(slot, await api(`/api/compare?${query}`));
}

async function fillRuns(shell, generation, chosen) {
  const picker = shell.querySelector(".compare-run");
  picker.replaceChildren(new Option("model only", ""));
  if (!generation) return;
  const runs = await api(`/api/runs?path=${encodeURIComponent(generation)}`);
  runs.forEach((run) => picker.append(new Option(run.id, run.path)));
  if (chosen) picker.value = chosen;
}

export async function loadCompare() {
  const shell = $("#panel-compare");
  const view = new URLSearchParams(location.hash.slice(1));
  const wanted = `${state.runPath}|${view.get("cmp_gen") ?? ""}|${view.get("cmp_run") ?? ""}`;
  if (shell.dataset.loaded === wanted) return;
  shell.dataset.loaded = wanted;
  shell.querySelector(".compare-left").textContent = `${state.runPath.split("/").pop()} against`;
  const generations = await api("/api/generations");
  const genPicker = shell.querySelector(".compare-gen");
  genPicker.replaceChildren(new Option("compare with…", ""));
  generations.forEach((generation) =>
    genPicker.append(new Option(`${generation.name} · ${generation.path.split("/").pop()}`, generation.path)));
  genPicker.value = view.get("cmp_gen") ?? "";
  await fillRuns(shell, genPicker.value, view.get("cmp_run"));
  const runPicker = shell.querySelector(".compare-run");
  const show = async () => {
    remember(genPicker.value, runPicker.value);
    shell.dataset.loaded = `${state.runPath}|${genPicker.value}|${runPicker.value}`;
    await load(shell, genPicker.value, runPicker.value);
  };
  genPicker.onchange = async () => {
    await fillRuns(shell, genPicker.value, null);
    await show();
  };
  runPicker.onchange = show;
  if (genPicker.value) await load(shell, genPicker.value, runPicker.value);
}
