// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * JupyterLab, in the page, on the run being read.
 */

import { showEmpty } from "./components.js";
import { $, api, post } from "./core.js";
import { setTab } from "./routing.js";
import { stopPlayback } from "./run.js";

export async function loadNotebook(url = null) {
  stopPlayback();
  $("#list-title").textContent = "NOTEBOOK";
  $("#browser").replaceChildren();
  $("#content").innerHTML = '<div class="notebook"><iframe title="JupyterLab"></iframe></div>';
  try {
    const lab = url ? { url } : await api("/api/jupyter");
    $(".notebook iframe").src = lab.url;
  } catch (error) {
    // a missing JupyterLab is a thing to install, not a page that failed to load
    showEmpty({
      eyebrow: "NOTEBOOK",
      title: "JupyterLab is not available.",
      detail: error.message,
    });
  }
}

export async function openNotebook(runPath) {
  setTab("notebook");
  const { url } = await post("/api/notebook", { path: runPath });
  await loadNotebook(url);
}
