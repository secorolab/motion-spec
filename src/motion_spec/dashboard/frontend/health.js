// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * Whether this installation can build and run anything at all -- and what it is made of.
 */

import { $, api, showError, stampText, state } from "./core.js";

// Why a build or a run cannot work, before it is tried: the CLI's own checks, on the page.
// The checks cost seconds, so the server runs them on a thread and this follows it.
export async function loadHealth(refresh = false) {
  clearTimeout(state.healthWatch);
  const [report, storage] = await Promise.all([
    api(`/api/health${refresh ? "?refresh=1" : ""}`),
    api("/api/storage").catch(() => null),
  ]);
  renderHealth(report, storage);
  if (report.running) state.healthWatch = setTimeout(() => loadHealth().catch(showError), 2000);
}

export function renderHealth(report, storage) {
  const page = document.createElement("article");
  page.className = "health";
  page.innerHTML = '<div class="page-heading"><h1>Installation health</h1>'
    + '<span class="eyebrow">HEALTH</span></div>'
    + '<div class="health-bar"><span class="health-verdict"></span>'
    + '<span class="health-state"></span><span class="health-spacer"></span>'
    + '<button class="health-recheck">re-check</button></div>'
    + '<div class="health-groups"></div>';

  const checks = report.checks ?? [];
  // An optional dependency left out of the build is not a failing check, so it neither reddens
  // the verdict nor counts against its profile -- it is only listed, so its absence is visible.
  const failing = checks.filter((check) => !check.ok && !check.optional);
  const verdict = page.querySelector(".health-verdict");
  if (report.running && !checks.length) {
    verdict.textContent = "checking…";
  } else {
    verdict.textContent = failing.length
      ? `${failing.length} of ${checks.length} checks failing`
      : `all ${checks.length} checks pass`;
    verdict.dataset.ok = String(!failing.length);
  }
  page.querySelector(".health-state").textContent = report.running
    ? "checking…"
    : report.stamp ? `checked ${stampText(new Date(report.stamp * 1000).toISOString())}` : "";
  const recheck = page.querySelector(".health-recheck");
  recheck.disabled = report.running;
  recheck.onclick = () => loadHealth(true).catch(showError);

  // One list language for everything: name on the left, value on the right, one row each.
  const list = (title, count) => {
    const section = document.createElement("section");
    section.className = "health-list";
    const heading = document.createElement("header");
    heading.className = "health-profile";
    heading.innerHTML = '<span></span><span class="health-count"></span>';
    heading.querySelector("span").textContent = title;
    if (count) {
      const cell = heading.querySelector(".health-count");
      cell.textContent = count.text;
      cell.dataset.ok = String(count.ok);
    }
    section.append(heading);
    return section;
  };
  const row = (name, value, { mark, dim, title } = {}) => {
    const line = document.createElement("div");
    line.className = "health-row";
    line.innerHTML = '<span class="health-mark"></span>'
      + '<span class="health-key"></span><span class="health-what"></span>'
      + '<span class="health-value"></span>';
    if (mark) {
      line.querySelector(".health-mark").textContent = { ok: "✓", fail: "✗", absent: "·" }[mark];
      line.dataset.mark = mark;
    }
    line.querySelector(".health-key").textContent = name;
    line.querySelector(".health-what").textContent = dim ?? "";
    const cell = line.querySelector(".health-value");
    // LRM marks pin the bidi order: the cell is direction:rtl only to ellipsize on the left.
    cell.textContent = value == null ? "—" : `‎${value}‎`;
    if (title ?? value) cell.title = title ?? value;
    return line;
  };

  // What the installation is, next to whether it works: enough to cite in a bug report.
  const env = report.environment ?? {};
  const gb = (bytes) => (bytes == null ? null : `${(bytes / 1e9).toFixed(2)} GB`);
  const environment = list("environment");
  environment.append(
    row("motion-spec", env.motion_spec),
    row("python", env.python, { title: env.executable }),
    row("ROS distro", env.ros_distro ?? "not sourced"),
    row("generations root", env.generations),
    row("generations on disk", gb(storage?.generations_bytes)),
    row("of that, frame logs", gb(storage?.logs_bytes)),
  );

  const groups = new Map();
  checks.forEach((check) => {
    groups.set(check.profile, [...(groups.get(check.profile) ?? []), check]);
  });
  page.querySelector(".health-groups").replaceChildren(environment, ...[...groups].map(([profile, rows]) => {
    const required = rows.filter((check) => !check.optional);
    const passed = required.filter((check) => check.ok).length;
    const group = list(profile, {
      text: `${passed} of ${required.length}`, ok: passed === required.length,
    });
    rows.forEach((check) => {
      // The key column already names the dependency: drop the module/cmake boilerplate tail.
      const tidy = check.path?.replace(/\/__init__\.py$/, "").replace(/\/cmake\/.*$/, "");
      const mark = check.ok ? "ok" : check.optional ? "absent" : "fail";
      group.append(row(check.dependency, check.ok ? tidy : check.optional ? "not built" : null,
        { mark, dim: check.what, title: check.path }));
      // Failing: one indented line with the why, the one command that fixes it, and where it lives.
      if (check.ok || check.optional) return;
      const fix = document.createElement("div");
      fix.className = "health-fix";
      fix.innerHTML = '<span class="health-why"></span><code></code><a target="_blank" rel="noopener"></a>';
      fix.querySelector(".health-why").textContent = check.why || "";
      fix.querySelector("code").textContent = check.detail || "";
      const anchor = fix.querySelector("a");
      if (check.source) {
        anchor.href = check.source;
        anchor.textContent = check.source.replace(/^https?:\/\//, "");
      } else anchor.remove();
      if (!check.detail) fix.querySelector("code").remove();
      group.append(fix);
    });
    return group;
  }));
  $("#content").replaceChildren(page);
}
