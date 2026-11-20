// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

/** A run's verdict: per motion, what was reached, held, fired and left. */

import { table } from "./components.js";
import { $, api, seconds } from "./core.js";
import { jumpToFinding } from "./run.js";

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

function renderVerdict(data) {
  const rows = data.motions.flatMap((motion) => {
    const at = (frame) => seconds((frame - motion.window[0]) * data.period_s);
    const jump = (row, frame) =>
      jumpLink(
        frame === null ? "never — inspect motion" : at(frame),
        frame ?? motion.window[0],
        motion.motion,
        row.id,
      );
    return [
      ...motion.constraints.map((row) => [
        motion.motion,
        row.id,
        row.kind,
        row.kind === "goal" ? jump(row, row.first_satisfied) : "—",
        row.kind === "goal" && row.active
          ? `${Math.round((100 * row.satisfied) / row.active)} %`
          : "—",
        row.kind === "goal" ? String(row.losses) : "—",
        row.kind === "monitor" ? jump(row, row.fired) : "—",
      ]),
      ...motion.exits.map((exit) => [
        motion.motion,
        "→ " + exit.to_state,
        "exit",
        jumpLink(at(exit.frame), exit.frame),
        "—",
        "—",
        [...exit.events, ...exit.monitors].join(", ") || "—",
      ]),
    ];
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

export async function showReports(runPath) {
  const panel = $("#panel-reports");
  if (!panel || panel.dataset.run === runPath) return;
  panel.dataset.run = runPath;
  panel.replaceChildren();
  try {
    panel.append(renderVerdict(await api(`/api/run/verdict?path=${encodeURIComponent(runPath)}`)));
  } catch (error) {
    panel.dataset.run = "";
    panel.append(
      Object.assign(document.createElement("p"), {
        className: "path",
        textContent: `verdict unavailable: ${error.message}`,
      }),
    );
  }
}
