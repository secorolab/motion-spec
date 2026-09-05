// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

/**
 * A run measured against itself and against another: this run's timeline beside a second run's,
 * then what its signals did -- coherent oscillation, contact, torque saturation.
 *
 * The three signal reports are one request -- the server sweeps the log once -- and it is made
 * when the tab is first opened rather than with the page, because a finished log is ~160 MB.
 * Compare reads graphs instead, and opens no log at all.
 */

import { $, api, seconds, state } from "./core.js";

const num = (value, digits = 3) => (value ?? null) === null ? "—" : value.toFixed(digits);
const exp = (value) => ((value ?? null) === null ? "—" : value.toExponential(2));

const signed = (value) =>
  value === null || value === undefined ? "—" : `${value >= 0 ? "+" : "−"}${Math.abs(value).toFixed(2)} s`;

const REPORTS = [
  {
    key: "oscillation",
    title: "oscillation",
    columns: ["state", "chain", "mode", "joints agreeing", "mean amplitude"],
    cells: (row) => [
      row.state_id,
      row.solver,
      row.mode_hz === null ? "scattered" : `${row.mode_hz.toFixed(2)} Hz`,
      `${row.agreeing} of ${row.joints}`,
      exp(row.mean_amplitude),
    ],
    // Coherence is the finding: one state where several joints agree says more than the
    // largest amplitude anywhere, which a motion's own sweep would win every time.
    summary: (rows) => {
      const found = rows.filter((row) => row.mode_hz !== null);
      if (!found.length) return `no coherent mode in ${rows.length} states`;
      const worst = found.reduce((a, b) => (b.mean_amplitude > a.mean_amplitude ? b : a));
      return `${worst.state_id} — coherent ${worst.mode_hz.toFixed(2)} Hz across `
        + `${worst.agreeing} of ${worst.joints} joints (mean ${exp(worst.mean_amplitude)})`;
    },
  },
  {
    key: "contact",
    title: "contact",
    columns: ["state", "peak", "axis", "rebound", "settle", "ripple", "band"],
    cells: (row) => [
      row.state_id,
      `${num(row.peak_n, 1)} N`,
      row.axis,
      `${num(row.rebound)} m/s`,
      row.settle_s === null ? "—" : `${num(row.settle_s, 2)} s`,
      num(row.ripple, 4),
      row.band === null ? "—" : `${num(row.band)} (${row.gate})`,
    ],
    summary: (rows) => {
      const worst = rows.reduce((a, b) => (b.peak_n > a.peak_n ? b : a));
      const band = worst.band === null ? "" : ` vs band ${num(worst.band)}`;
      return `${worst.state_id} — peak ${num(worst.peak_n, 1)} N, rebound ${num(worst.rebound)} m/s`
        + `, ripple ${num(worst.ripple, 4)}${band}`;
    },
  },
  {
    key: "saturation",
    title: "saturation",
    columns: ["chain", "joint", "ticks clipped", "applied", "requested", "states"],
    cells: (row) => [
      row.solver,
      `joint_${row.joint}`,
      row.ticks,
      `${num(row.limit_nm, 2)} Nm${row.limit_id ? ` (${row.limit_id})` : ""}`,
      `${num(row.requested_nm, 2)} Nm`,
      row.states.join(", "),
    ],
    summary: (rows) => {
      const worst = rows.reduce((a, b) => (b.ticks > a.ticks ? b : a));
      return `joint_${worst.joint} tau — ${worst.ticks} ticks clipped at `
        + `${num(worst.limit_nm, 1)} Nm (${worst.states.join(", ")})`;
    },
  },
];

function table(headers, rows) {
  const node = document.createElement("table");
  const head = document.createElement("tr");
  headers.forEach((name) => {
    const cell = document.createElement("th");
    cell.textContent = name;
    head.append(cell);
  });
  node.append(head);
  rows.forEach((cells) => {
    const line = document.createElement("tr");
    cells.forEach((text) => {
      const cell = document.createElement("td");
      cell.textContent = text;
      line.append(cell);
    });
    node.append(line);
  });
  return node;
}

// Every report on this page folds the same way: a title, what it found in one line, and the
// table behind it. Compare included -- it is a report the reader asked for by picking a run.
function section(title, note, body) {
  const box = document.createElement("details");
  box.className = "report";
  const head = document.createElement("summary");
  const name = document.createElement("b");
  name.textContent = title;
  const detail = document.createElement("span");
  detail.textContent = note;
  head.append(name, detail);
  box.append(head);
  if (body) box.append(body);
  return box;
}

function signalReport(report, rows) {
  return section(
    report.title,
    rows.length ? report.summary(rows) : "nothing recorded",
    rows.length ? table(report.columns, rows.map(report.cells)) : null,
  );
}

function renderCompare(data, rightPath) {
  const rows = data.activities.map((row) =>
    [row.name, seconds(row.left_s), seconds(row.right_s), signed(row.delta_s)]);
  const note = [
    `against ${rightPath.split("/").pop()}`,
    data.same_model ? "" : "different models — aligned where they align",
  ].filter(Boolean).join(" · ");
  const box = section("compare", note, table(["activity", "this run", "other run", "delta"], rows));
  box.open = true;   // picking a run is the ask; folding the answer away would undo it
  return box;
}

// Reached, held, fired, left: the four questions asked of every motion after a run.
function renderVerdict(data) {
  const at = (frame) => (frame === null || frame === undefined ? "—" : seconds(frame * data.period_s));
  const rows = [];
  data.motions.forEach((motion) => {
    motion.constraints.forEach((row) => {
      const held = row.kind === "goal" && row.active ? `${Math.round((100 * row.satisfied) / row.active)} %` : "—";
      const reached = row.kind === "goal" ? (row.first_satisfied === null ? "never" : at(row.first_satisfied - motion.window[0])) : "—";
      const fired = row.kind === "monitor" ? (row.fired === null ? "never" : at(row.fired - motion.window[0])) : "—";
      rows.push([motion.motion, row.id, row.kind, reached, held, row.kind === "goal" ? String(row.losses) : "—", fired]);
    });
    motion.exits.forEach((exit) => rows.push([
      motion.motion, "→ " + exit.to_state, "exit", at(exit.frame - motion.window[0]), "—", "—",
      [...exit.events, ...exit.monitors].join(", ") || "—",
    ]));
  });
  const goals = data.motions.flatMap((motion) => motion.constraints.filter((row) => row.kind === "goal"));
  const unreached = goals.filter((row) => row.first_satisfied === null).length;
  const lost = goals.filter((row) => row.losses > 0).length;
  const note = !goals.length ? "no goal constraints recorded"
    : `${goals.length - unreached} of ${goals.length} goals reached` + (lost ? `, ${lost} lost after reaching` : "");
  const box = section("verdict", note, rows.length
    ? table(["motion", "constraint", "kind", "reached / left at", "held", "lost", "fired / via"], rows)
    : null);
  box.open = true;
  return box;
}

async function renderComparePicker(panel) {
  const picker = panel.querySelector(".compare-pick");
  const runs = await api(`/api/runs?path=${encodeURIComponent(state.generationPath)}`);
  picker.replaceChildren(new Option("compare with…", ""));
  runs.filter((run) => run.path !== state.runPath)
    .forEach((run) => picker.append(new Option(run.id, run.path)));
  picker.onchange = async () => {
    const other = picker.value;
    const slot = panel.querySelector(".compare-slot");
    slot.replaceChildren();
    if (!other) return;
    const data = await api(
      `/api/run/compare?left=${encodeURIComponent(state.runPath)}&right=${encodeURIComponent(other)}`,
    );
    slot.replaceChildren(renderCompare(data, other));
  };
}

const holding = (text) =>
  Object.assign(document.createElement("p"), { className: "path", textContent: text });

export async function showReports(runPath) {
  const panel = $("#panel-reports");
  if (!panel || panel.dataset.run === runPath) return;
  panel.dataset.run = runPath;
  panel.innerHTML = '<div class="compare-bar"><select class="compare-pick"></select></div>'
    + '<div class="compare-slot"></div>';
  // The picker is one cheap listing, so it is usable while the log sweep below it runs.
  await renderComparePicker(panel);
  // The verdict comes off the marker scan the page already ran, so it lands before the sweep.
  try {
    panel.append(renderVerdict(await api(`/api/run/verdict?path=${encodeURIComponent(runPath)}`)));
  } catch (error) {
    panel.append(holding(`verdict unavailable: ${error.message}`));
  }
  panel.append(holding("sweeping the frame log…"));
  let data;
  try {
    data = await api(`/api/reports?path=${encodeURIComponent(runPath)}`);
  } catch (error) {
    panel.dataset.run = "";   // so reopening the tab tries the sweep again
    panel.lastElementChild.replaceWith(holding(error.message));
    return;
  }
  // A period the header never carried is a guess, and durations built on it say so.
  const clock = holding(`control period ${(data.period_s * 1e3).toFixed(3)} ms`
    + (data.period_exact ? "" : " (assumed — this archive records no period)"));
  panel.lastElementChild.replaceWith(clock);
  panel.append(...REPORTS.map((report) => signalReport(report, data[report.key])));
}
