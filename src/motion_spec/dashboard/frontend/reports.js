// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

/** A run's verdict: per motion, what was reached, held, fired and left. */

import { $, api, seconds } from "./core.js";
import { jumpToFinding } from "./run.js";

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
      if (text instanceof Node) cell.append(text);
      else cell.textContent = text;
      line.append(cell);
    });
    node.append(line);
  });
  return node;
}

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

function jumpLink(label, frame, motion = null, constraint = null) {
  const button = document.createElement("button");
  button.className = "report-jump";
  button.textContent = label;
  button.title = `Open replay at frame ${frame}`;
  button.onclick = () => jumpToFinding(frame, motion, constraint);
  return button;
}

// Reached, held, fired, left: the four questions asked of every motion after a run.
function renderVerdict(data) {
  const at = (frame) =>
    frame === null || frame === undefined ? "—" : seconds(frame * data.period_s);
  const rows = [];
  data.motions.forEach((motion) => {
    motion.constraints.forEach((row) => {
      const held =
        row.kind === "goal" && row.active
          ? `${Math.round((100 * row.satisfied) / row.active)} %`
          : "—";
      const reached =
        row.kind === "goal"
          ? jumpLink(
              row.first_satisfied === null
                ? "never — inspect motion"
                : at(row.first_satisfied - motion.window[0]),
              row.first_satisfied ?? motion.window[0],
              motion.motion,
              row.id,
            )
          : "—";
      const fired =
        row.kind === "monitor"
          ? jumpLink(
              row.fired === null ? "never — inspect motion" : at(row.fired - motion.window[0]),
              row.fired ?? motion.window[0],
              motion.motion,
              row.id,
            )
          : "—";
      rows.push([
        motion.motion,
        row.id,
        row.kind,
        reached,
        held,
        row.kind === "goal" ? String(row.losses) : "—",
        fired,
      ]);
    });
    motion.exits.forEach((exit) =>
      rows.push([
        motion.motion,
        "→ " + exit.to_state,
        "exit",
        jumpLink(at(exit.frame - motion.window[0]), exit.frame),
        "—",
        "—",
        [...exit.events, ...exit.monitors].join(", ") || "—",
      ]),
    );
  });
  const goals = data.motions.flatMap((motion) =>
    motion.constraints.filter((row) => row.kind === "goal"),
  );
  const unreached = goals.filter((row) => row.first_satisfied === null).length;
  const lost = goals.filter((row) => row.losses > 0).length;
  const note = !goals.length
    ? "no goal constraints recorded"
    : `${goals.length - unreached} of ${goals.length} goals reached` +
      (lost ? `, ${lost} lost after reaching` : "");
  const box = section(
    "verdict",
    note,
    rows.length
      ? table(
          ["motion", "constraint", "kind", "reached / left at", "held", "lost", "fired / via"],
          rows,
        )
      : null,
  );
  box.open = true;
  return box;
}

const holding = (text) =>
  Object.assign(document.createElement("p"), { className: "path", textContent: text });

export async function showReports(runPath) {
  const panel = $("#panel-reports");
  if (!panel || panel.dataset.run === runPath) return;
  panel.dataset.run = runPath;
  panel.replaceChildren();
  try {
    panel.append(renderVerdict(await api(`/api/run/verdict?path=${encodeURIComponent(runPath)}`)));
  } catch (error) {
    panel.dataset.run = "";
    panel.append(holding(`verdict unavailable: ${error.message}`));
  }
}
