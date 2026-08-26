// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

/**
 * What happened when, and what it waited on: three tables read straight off the run's graph.
 *
 * Nothing here opens the frame log. Durations come from the tick rate the run recorded, so a
 * run whose graph carries none shows steps and no seconds rather than a made-up number.
 */

import { $, api, state } from "./core.js";
import { seek } from "./run.js";

// How far past its declared dwell a gate has to hold before the wait is worth remarking on.
const SLOW_GATE = 1.2;

const seconds = (value) => (value === null || value === undefined ? "—" : `${value.toFixed(2)} s`);

const signed = (value) =>
  value === null || value === undefined ? "—" : `${value >= 0 ? "+" : "−"}${Math.abs(value).toFixed(2)} s`;

export function table(headers, rows) {
  const node = document.createElement("table");
  const head = document.createElement("tr");
  headers.forEach((name) => {
    const cell = document.createElement("th");
    cell.textContent = name;
    head.append(cell);
  });
  node.append(head);
  rows.forEach(({ cells, tint, seekTo, title }) => {
    const line = document.createElement("tr");
    line.classList.toggle("tinted", Boolean(tint));
    if (title) line.title = title;
    cells.forEach((text) => {
      const cell = document.createElement("td");
      cell.textContent = text;
      line.append(cell);
    });
    // Clicking a row moves the playhead of the run already open beside these tables.
    if (seekTo !== null && seekTo !== undefined) {
      line.classList.add("seekable");
      line.onclick = () => seek(seekTo);
    }
    node.append(line);
  });
  return node;
}

export function section(title, note, body) {
  const wrap = document.createElement("div");
  wrap.className = "view";
  const heading = document.createElement("div");
  heading.className = "view-head";
  const name = document.createElement("h2");
  name.textContent = title;
  heading.append(name);
  if (note) {
    const detail = document.createElement("span");
    detail.className = "view-note";
    detail.textContent = note;
    heading.append(detail);
  }
  wrap.append(heading, body);
  return wrap;
}

// What a payload was read from, said plainly: an archived graph is the whole record, a
// projection is strided and cannot be asked how long anything waited.
function sourceNote(data) {
  if (data.runtime_source === "archive") return "from the archived runtime graph";
  if (data.runtime_source === "projected") {
    return "projected from a run still going — waits and re-arms not yet known";
  }
  return "no runtime graph recorded yet";
}

function renderTimeline(data, iri) {
  const longest = Math.max(...data.spans.map((span) => span.duration_s ?? 0), 0);
  // What held throughout an occupancy is a cross join, so the route answers it only for a
  // timeline narrowed to one element -- and the column only exists where it is filled.
  const headers = ["activity", "entered", "duration", "entry event"];
  const rows = data.spans.map((span) => ({
    cells: [
      span.name,
      seconds(span.entered_s),
      span.duration_s === null ? "open" : seconds(span.duration_s),
      span.event ?? "—",
      ...(iri ? [span.satisfied.join(", ") || "—"] : []),
    ],
    tint: longest > 0 && span.duration_s === longest,
    seekTo: span.begin_step,
  }));
  const note = [sourceNote(data), iri ? "one element only" : ""].filter(Boolean).join(" · ");
  return section("Timeline", note, table(iri ? [...headers, "satisfied throughout"] : headers, rows));
}

// One gate, in the words a reader would use: when it became satisfiable, when it fired, and
// what it did in between. Never a verdict -- a long wait may be exactly what was wanted.
function gateStory(gate, period) {
  if (gate.fired_step === null) return "never fired";
  const at = (step) => (period ? `${(step * period).toFixed(2)} s` : `step ${step}`);
  const parts = [`satisfiable at ${at(gate.first_held_step)}`, `fired at ${at(gate.fired_step)}`];
  if (gate.declared_dwell_s !== null) parts.push(`declared dwell ${gate.declared_dwell_s.toFixed(2)} s`);
  if (gate.waited_s !== null && gate.declared_dwell_s && gate.waited_s > gate.declared_dwell_s * SLOW_GATE) {
    parts.push(`waited ${(gate.waited_s - gate.declared_dwell_s).toFixed(2)} s beyond its dwell`);
  }
  if (gate.rearm_count) parts.push(`condition broke ${gate.rearm_count}× before firing`);
  return parts.join(" · ");
}

function renderGates(data) {
  const rows = data.gates.map((gate) => ({
    cells: [
      gate.monitor_name,
      gate.event_name ?? "—",
      gateStory(gate, data.period_s),
      gate.declared_dwell_s === null ? "—" : `${gate.declared_dwell_s.toFixed(2)} s`,
      gate.rearm_count === null ? "—" : String(gate.rearm_count),
    ],
    seekTo: gate.fired_step,
    title: gate.members.length ? `watches: ${gate.members.join(", ")}` : "",
  }));
  return section("Gates", sourceNote(data), table(
    ["gate", "event", "what happened", "declared dwell", "re-arms"],
    rows,
  ));
}

function renderCompare(data, rightPath) {
  const rows = data.activities.map((row) => ({
    cells: [row.name, seconds(row.left_s), seconds(row.right_s), signed(row.delta_s)],
  }));
  const note = [
    `against ${rightPath.split("/").pop()}`,
    data.same_model ? "" : "different models — aligned where they align",
  ].filter(Boolean).join(" · ");
  return section("Compare", note, table(["activity", "this run", "other run", "delta"], rows));
}

async function renderComparePicker(shell) {
  const picker = shell.querySelector(".compare-pick");
  const runs = await api(`/api/runs?path=${encodeURIComponent(state.generationPath)}`);
  picker.replaceChildren(new Option("compare with…", ""));
  runs.filter((run) => run.path !== state.runPath)
    .forEach((run) => picker.append(new Option(run.id, run.path)));
  picker.onchange = async () => {
    const other = picker.value;
    shell.querySelector(".compare-slot").replaceChildren();
    if (!other) return;
    const data = await api(
      `/api/run/compare?left=${encodeURIComponent(state.runPath)}&right=${encodeURIComponent(other)}`,
    );
    shell.querySelector(".compare-slot").replaceChildren(renderCompare(data, other));
  };
}

// Rendered on first open of the panel and not before: two graph queries a reader may never ask.
export async function loadViews() {
  const shell = $("#panel-views");
  if (shell.dataset.loaded === state.runPath) return;
  shell.dataset.loaded = state.runPath;
  shell.innerHTML = '<div class="views"><div class="timeline-slot"></div>'
    + '<div class="gates-slot"></div>'
    + '<div class="compare-bar"><select class="compare-pick"></select></div>'
    + '<div class="compare-slot"></div></div>';
  // 016's Occurrences action lands here with the design IRI it wants the timeline narrowed to.
  const iri = new URLSearchParams(location.hash.slice(1)).get("iri");
  const path = encodeURIComponent(state.runPath);
  const query = `/api/run/timeline?path=${path}${iri ? `&iri=${encodeURIComponent(iri)}` : ""}`;
  const [timeline, gates] = await Promise.all([
    api(query),
    api(`/api/run/gates?path=${path}`),
  ]);
  shell.querySelector(".timeline-slot").replaceChildren(renderTimeline(timeline, iri));
  shell.querySelector(".gates-slot").replaceChildren(renderGates(gates));
  await renderComparePicker(shell);
}
