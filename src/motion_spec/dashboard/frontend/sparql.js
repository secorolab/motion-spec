// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * Asking a run's graph a question, and keeping the questions worth asking again.
 */

import { $, api, post, state } from "./core.js";

export const CANNED = {
  "controller gains": "SELECT ?controller ?kp ?ki ?kd WHERE {\n"
    + "  ?controller cstr-hdl:proportional-gain ?kp ;\n"
    + "              cstr-hdl:integral-gain ?ki ;\n"
    + "              cstr-hdl:derivative-gain ?kd .\n} LIMIT 50",
  "what this run observed": "SELECT ?property (COUNT(*) AS ?observations) WHERE {\n"
    + "  GRAPH <urn:runtime> { ?o sosa:observedProperty ?property }\n"
    + "} GROUP BY ?property ORDER BY DESC(?observations) LIMIT 50",
  "latest values": "SELECT ?property ?value WHERE {\n"
    + "  GRAPH <urn:live> { ?o sosa:observedProperty ?property ; sosa:hasSimpleResult ?value }\n"
    + "} LIMIT 50",
  "graph sizes": "SELECT ?graph (COUNT(*) AS ?triples) WHERE {\n"
    + "  GRAPH ?graph { ?s ?p ?o }\n} GROUP BY ?graph",
};

export function queryLabel(query) {
  const line = query.split("\n").map((text) => text.trim())
    .find((text) => text && !text.startsWith("#")) ?? "query";
  return line.length > 30 ? `${line.slice(0, 29)}…` : line;
}

export function renderRail() {
  const rail = $(".query-rail");
  rail.replaceChildren(...state.queries.map((entry, index) => {
    const tab = document.createElement("button");
    tab.className = "query-tab";
    tab.classList.toggle("active", index === state.query);
    tab.classList.toggle("draft", !entry.data);
    tab.title = entry.query;
    tab.textContent = queryLabel(entry.query);
    tab.onclick = () => selectQuery(index);
    const close = document.createElement("span");
    close.className = "query-close";
    close.textContent = "×";
    close.onclick = (event) => {
      event.stopPropagation();
      state.queries.splice(index, 1);
      state.query = Math.min(state.query, state.queries.length - 1);
      saveQueries();
      state.queries.length ? selectQuery(state.query) : renderRail();
    };
    tab.append(close);
    return tab;
  }), $("#new-query"));
}

export function selectQuery(index) {
  state.query = index;
  const entry = state.queries[index];
  $("#query").value = entry.query;
  renderRail();
  if (entry.data) {
    $("#query-status").textContent = entry.status;
    renderAnswer(entry.data);
  } else {
    runQuery();
  }
}

export function saveQueries() {
  // beside the run, so they outlive this browser and travel with the archive
  post("/api/queries", { path: state.runPath, queries: state.queries.map((entry) => entry.query) })
    .catch((error) => { $("#query-status").textContent = `not saved: ${error.message}`; });
}

export async function runQuery() {
  const query = $("#query").value;
  $("#query-status").textContent = "Running…";
  let entry = state.queries[state.query];
  if (entry && entry.query !== query && !entry.data) {
    entry.query = query;
    saveQueries();
  } else if (!entry || entry.query !== query) {
    entry = { query };
    state.queries.push(entry);
    state.query = state.queries.length - 1;
    saveQueries();
  }
  renderRail();
  try {
    const data = await post("/api/sparql", { path: state.runPath, query });
    entry.data = data;
    entry.status = `${data.count} row${data.count === 1 ? "" : "s"}`
      + `${data.truncated ? " (first 500)" : ""} · ${data.elapsed_ms} ms`
      // an empty model graph is a run detached from its generation, not a query that found nothing
      + `${data.model_triples ? "" : " · model graph unavailable"}`;
    $("#query-status").textContent = entry.status;
    renderAnswer(data);
  } catch (error) {
    entry.data = null;
    entry.status = error.message;
    $("#query-status").textContent = error.message;
    $("#answer").replaceChildren();
  }
}

export async function bindSparql() {
  state.queries = [];
  state.query = -1;
  $(".sparql-canned").replaceChildren(...Object.entries(CANNED).map(([label, query]) => {
    const button = document.createElement("button");
    button.textContent = label;
    button.onclick = () => { $("#query").value = query; runQuery(); };
    return button;
  }));
  $("#new-query").onclick = () => {
    state.queries.push({ query: "SELECT ?s ?p ?o WHERE {\n  ?s ?p ?o\n} LIMIT 20" });
    state.query = state.queries.length - 1;
    $("#query").value = state.queries[state.query].query;
    $("#answer").replaceChildren();
    $("#query-status").textContent = "not run yet";
    saveQueries();
    renderRail();
    $("#query").focus();
  };
  $("#ask").onclick = runQuery;
  $("#query").onkeydown = (event) => {
    if ((event.ctrlKey || event.metaKey) && event.key === "Enter") runQuery();
  };
  $("#query").value = CANNED["controller gains"];
  renderRail();
  const stored = await api(`/api/queries?path=${encodeURIComponent(state.runPath)}`);
  if (state.runPath && stored.length) {
    state.queries = stored.map((query) => ({ query }));
    renderRail();
  }
}

export function renderAnswer(data) {
  const table = document.createElement("table");
  const head = document.createElement("tr");
  data.headers.forEach((name) => {
    const cell = document.createElement("th");
    cell.textContent = name;
    head.append(cell);
  });
  table.append(head);
  data.rows.forEach((row) => {
    const line = document.createElement("tr");
    row.forEach((term) => {
      const cell = document.createElement("td");
      cell.textContent = term ?? "";
      line.append(cell);
    });
    table.append(line);
  });
  $("#answer").replaceChildren(table);
}
