// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * Whether this installation can build and run anything at all.
 */

import { $, api, showError, stampText, state } from "./core.js";

// Why a build or a run cannot work, before it is tried: the CLI's own checks, on the page.
// The checks cost seconds, so the server runs them on a thread and this follows it.
export async function loadHealth(refresh = false) {
  clearTimeout(state.healthWatch);
  const report = await api(`/api/health${refresh ? "?refresh=1" : ""}`);
  renderHealth(report);
  if (report.running) state.healthWatch = setTimeout(() => loadHealth().catch(showError), 2000);
}

export function renderHealth(report) {
  const page = document.createElement("article");
  page.className = "health";
  page.innerHTML = '<div class="page-heading"><h1>Installation health</h1>'
    + '<span class="eyebrow">HEALTH</span></div>'
    + '<div class="health-bar"><button class="health-recheck">re-check</button>'
    + '<span class="health-state"></span></div><div class="health-groups"></div>';
  page.querySelector(".health-state").textContent = report.running
    ? "checking…"
    : report.stamp ? `checked ${stampText(new Date(report.stamp * 1000).toISOString())}` : "";
  const recheck = page.querySelector(".health-recheck");
  recheck.disabled = report.running;
  recheck.onclick = () => loadHealth(true).catch(showError);
  const groups = new Map();
  (report.checks ?? []).forEach((check) => {
    groups.set(check.profile, [...(groups.get(check.profile) ?? []), check]);
  });
  page.querySelector(".health-groups").replaceChildren(...[...groups].map(([profile, checks]) => {
    const group = document.createElement("div");
    group.className = "health-group";
    const heading = document.createElement("div");
    heading.className = "health-profile";
    heading.textContent = profile;
    group.append(heading, ...checks.map((check) => {
      const row = document.createElement("div");
      row.className = "health-check";
      row.dataset.ok = String(check.ok);
      row.innerHTML = '<span class="health-mark"></span><strong></strong>'
        + '<span class="health-what"></span><code class="health-remedy"></code>';
      row.querySelector(".health-mark").textContent = check.ok ? "✓" : "✗";
      row.querySelector("strong").textContent = check.dependency;
      row.querySelector(".health-what").textContent = check.what;
      // A failing check without the command that fixes it is a complaint, not a report.
      row.querySelector(".health-remedy").textContent = check.ok ? "" : check.detail;
      return row;
    }));
    return group;
  }));
  $("#content").replaceChildren(page);
}
