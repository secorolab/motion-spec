// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

/**
 * What a signal did over a run: coherent oscillation, contact, torque saturation.
 *
 * One request answers all three -- the server sweeps the log once -- and it is made when the
 * tab is first opened rather than with the page, because a finished log is ~160 MB.
 */

import { $, api } from "./core.js";

const num = (value, digits = 3) => (value ?? null) === null ? "—" : value.toFixed(digits);
const exp = (value) => ((value ?? null) === null ? "—" : value.toExponential(2));

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

function section(report, rows) {
  const box = document.createElement("details");
  box.className = "report";
  const head = document.createElement("summary");
  head.innerHTML = `<b>${report.title}</b><span>${rows.length ? report.summary(rows) : "nothing recorded"}</span>`;
  box.append(head);
  if (!rows.length) return box;
  const table = document.createElement("table");
  const columns = document.createElement("tr");
  report.columns.forEach((name) => {
    const cell = document.createElement("th");
    cell.textContent = name;
    columns.append(cell);
  });
  table.append(columns);
  rows.forEach((row) => {
    const line = document.createElement("tr");
    report.cells(row).forEach((value) => {
      const cell = document.createElement("td");
      cell.textContent = value;
      line.append(cell);
    });
    table.append(line);
  });
  box.append(table);
  return box;
}

export async function showReports(runPath) {
  const panel = $("#panel-reports");
  if (!panel || panel.dataset.run === runPath) return;
  panel.dataset.run = runPath;
  panel.replaceChildren(Object.assign(document.createElement("p"), {
    className: "path",
    textContent: "sweeping the frame log…",
  }));
  let data;
  try {
    data = await api(`/api/reports?path=${encodeURIComponent(runPath)}`);
  } catch (error) {
    panel.dataset.run = "";
    panel.replaceChildren(Object.assign(document.createElement("p"), {
      className: "path",
      textContent: error.message,
    }));
    return;
  }
  const clock = document.createElement("p");
  clock.className = "path";
  // A period the header never carried is a guess, and durations built on it say so.
  clock.textContent = `control period ${(data.period_s * 1e3).toFixed(3)} ms`
    + (data.period_exact ? "" : " (assumed — this archive records no period)");
  panel.replaceChildren(clock, ...REPORTS.map((report) => section(report, data[report.key])));
}
