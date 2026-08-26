// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

/**
 * Explore: one query, two projections. The table and the picture are the same answer, so
 * there is no "show me this file" mode that is not a query someone can edit and rerun.
 */

import { $, $$, api, post, snack, state } from "./core.js";
import { addPlot } from "./plots.js";
import { setView } from "./routing.js";
import { showSource } from "./sources.js";

// Every one of these is run against a maintained generation before it ships: a canned query
// that answers nothing teaches the wrong vocabulary. Prefixes are spelled out rather than
// leaned on, so each query says which metamodel its terms come from -- and so the suite can
// read this dictionary and run every entry in it.
export const CANNED = {
  "everything in this graph": `CONSTRUCT { ?s ?p ?o }
WHERE { ?s ?p ?o }`,

  "state timeline": `PREFIX ms-prov: <https://secorolab.github.io/metamodels/motion-spec/prov#>
PREFIX prov: <http://www.w3.org/ns/prov#>
PREFIX time: <http://www.w3.org/2006/time#>
PREFIX fsm: <https://secorolab.github.io/metamodels/behaviour/fsm#>
SELECT ?state ?from ?to WHERE {
  ?occ a ms-prov:MotionExecution ;
       prov:used ?state ;
       time:hasBeginning ?begin ;
       time:hasEnd ?end .
  ?state a fsm:State .
  ?begin time:inTimePosition/time:numericPosition ?from .
  ?end time:inTimePosition/time:numericPosition ?to .
} ORDER BY ?from`,

  "what caused a transition": `PREFIX prov: <http://www.w3.org/ns/prov#>
PREFIX time: <http://www.w3.org/2006/time#>
PREFIX fsm: <https://secorolab.github.io/metamodels/behaviour/fsm#>
SELECT ?transition ?step ?cause WHERE {
  ?occ a prov:Activity ;
       prov:used ?transition ;
       time:hasTime ?instant ;
       prov:wasInformedBy ?prior .
  ?transition a fsm:Transition .
  ?instant time:inTimePosition/time:numericPosition ?step .
  ?prior prov:used ?cause .
} ORDER BY ?step`,

  "satisfied during a state": `PREFIX ms-prov: <https://secorolab.github.io/metamodels/motion-spec/prov#>
PREFIX prov: <http://www.w3.org/ns/prov#>
PREFIX time: <http://www.w3.org/2006/time#>
PREFIX fsm: <https://secorolab.github.io/metamodels/behaviour/fsm#>
PREFIX cstr: <https://comp-rob2b.github.io/metamodels/task/constraint#>
SELECT ?state ?constraint ?from ?to WHERE {
  { SELECT ?state ?entered ?left WHERE {
      ?stateOcc a ms-prov:MotionExecution ;
                prov:used ?state ;
                time:hasBeginning ?stateBegin ;
                time:hasEnd ?stateEnd .
      ?state a fsm:State .
      ?stateBegin time:inTimePosition/time:numericPosition ?entered .
      ?stateEnd time:inTimePosition/time:numericPosition ?left . } }
  { SELECT DISTINCT ?constraint ?from ?to WHERE {
      ?occ a ms-prov:ConstraintMaintenance ;
           prov:used ?constraint ;
           time:hasBeginning ?begin ;
           time:hasEnd ?end .
      ?constraint a cstr:Constraint .
      ?begin time:inTimePosition/time:numericPosition ?from .
      ?end time:inTimePosition/time:numericPosition ?to . } }
  FILTER(?from >= ?entered && ?from <= ?left)
} ORDER BY ?entered ?from`,

  "modelled, never ran": `PREFIX prov: <http://www.w3.org/ns/prov#>
PREFIX cstr: <https://comp-rob2b.github.io/metamodels/task/constraint#>
SELECT ?constraint WHERE {
  GRAPH <urn:model> { ?constraint a cstr:Constraint }
  FILTER NOT EXISTS { GRAPH <urn:runtime> { ?occ prov:used ?constraint } }
} ORDER BY ?constraint`,

  "recorded results": `PREFIX ms-prov: <https://secorolab.github.io/metamodels/motion-spec/prov#>
PREFIX prov: <http://www.w3.org/ns/prov#>
PREFIX time: <http://www.w3.org/2006/time#>
PREFIX sosa: <http://www.w3.org/ns/sosa/>
SELECT ?element ?value ?step WHERE {
  ?occ a ms-prov:ConstraintMaintenance ;
       prov:used ?element ;
       sosa:hasSimpleResult ?value ;
       time:hasBeginning ?begin .
  ?begin time:inTimePosition/time:numericPosition ?step .
} ORDER BY ?step`,

  "controller gains": `PREFIX cstr-hdl: <https://comp-rob2b.github.io/metamodels/task/constraint-handler#>
SELECT ?controller ?kp ?ki ?kd WHERE {
  ?controller cstr-hdl:proportional-gain ?kp ;
              cstr-hdl:integral-gain ?ki ;
              cstr-hdl:derivative-gain ?kd .
} ORDER BY ?controller`,

  "graph sizes": `SELECT ?graph (COUNT(*) AS ?triples) WHERE {
  GRAPH ?graph { ?s ?p ?o }
} GROUP BY ?graph`,
};

export const EXPLORE_MARKUP = `<div class="explore">
  <div class="query-rail"><button id="new-query" class="new-query">+ query</button></div>
  <div class="explore-body">
    <div class="explore-canned"></div>
    <textarea id="query" spellcheck="false"></textarea>
    <div class="explore-run">
      <button id="ask">Run query</button>
      <span class="explore-views"><button data-view="table" class="active">Table</button><button data-view="graph">Graph</button></span>
      <span id="query-status" class="path"></span>
    </div>
    <div id="answer"></div>
    <div class="explore-graph" hidden>
      <div class="graph-bar">
        <div class="graph-search-panel"><input class="graph-search" type="search" placeholder="Search nodes"><div class="graph-matches"></div></div>
        <span class="graph-depth"><em>depth</em><button data-depth="1" class="active">1</button><button data-depth="2">2</button><button data-depth="3">3</button></span>
        <button class="graph-back" hidden>back</button>
        <button class="graph-clear" hidden>clear focus</button>
        <button class="graph-export">export as query</button>
        <button class="graph-halt" hidden>stop</button>
        <button class="graph-fullscreen" title="Fullscreen graph">⛶</button>
      </div>
      <div class="graph-crumbs"></div>
      <div class="graph-main">
        <div class="graph-canvas"></div>
        <div class="graph-side">
          <div class="graph-details">Select a node for its RDF details.</div>
          <div class="graph-legend"></div>
        </div>
      </div>
      <div class="graph-status"></div>
    </div>
  </div>
</div>`;

// One Explore at a time; a second page replaces it, renderer and layout included.
const explore = {
  path: null,
  view: "table",
  data: null,
  payload: null,
  focus: [],
  expanded: new Set(),
  pins: new Set(),
  depth: 1,
  hidden: new Set(),
  only: null,
  filter: "",
  renderer: null,
  layout: null,
  settling: null,
  drawing: 0,
  generation: null,
};

export function queryLabel(query) {
  const line = query.split("\n").map((text) => text.trim())
    .find((text) => text && !text.startsWith("#") && !text.startsWith("PREFIX")) ?? "query";
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
    showAnswer(entry.data);
  } else {
    runQuery();
  }
}

export function saveQueries() {
  // beside the run, so they outlive this browser and travel with the archive; a generation
  // has nowhere to keep them, and that is not an error worth showing
  if (!state.runPath) return;
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
    const data = await post("/api/sparql", { path: explore.path, query });
    entry.data = data;
    entry.status = statusLine(data);
    $("#query-status").textContent = entry.status;
    showAnswer(data);
  } catch (error) {
    entry.data = null;
    entry.status = error.message;
    $("#query-status").textContent = error.message;
    $("#answer").replaceChildren();
  }
}

function statusLine(data) {
  const hidden = data.graph?.hidden ?? {};
  const folded = Object.entries(hidden).filter(([, count]) => count)
    .map(([name, count]) => `${count} ${name.replace("_edges", "")}`).join(" · ");
  return `${data.count} row${data.count === 1 ? "" : "s"}`
    + `${data.truncated ? " (table shows 500)" : ""} · ${data.elapsed_ms} ms`
    // an empty model graph is a run detached from its generation, not a query that found nothing
    + `${data.model_triples ? "" : " · model graph unavailable"}`
    + `${data.runtime_source ? ` · runtime: ${data.runtime_source}` : " · no runtime graph"}`
    + `${folded ? ` · folded ${folded}` : ""}`;
}

// A fresh answer drops the exploration built on the last one, except the pins -- that is what
// pinning is for.
function showAnswer(data) {
  explore.data = data;
  explore.payload = mergePins(data.graph);
  explore.focus = [];
  explore.expanded = new Set();
  explore.hidden = new Set();
  explore.only = null;
  renderTable(data);
  showView(explore.payload ? explore.view : "table");
}

function mergePins(payload) {
  const kept = explore.payload;
  if (!kept || !explore.pins.size) return payload && { ...payload };
  const merged = payload ? { ...payload, nodes: [...payload.nodes], links: [...payload.links] }
    : { nodes: [], links: [], types: {}, predicates: {}, hidden: {} };
  const have = new Set(merged.nodes.map((node) => node.id));
  kept.nodes.filter((node) => explore.pins.has(node.id) && !have.has(node.id))
    .forEach((node) => merged.nodes.push(node));
  const ids = new Set(merged.nodes.map((node) => node.id));
  const edges = new Set(merged.links.map(edgeKey));
  kept.links.filter((link) => ids.has(link.source) && ids.has(link.target)
    && !edges.has(edgeKey(link))).forEach((link) => merged.links.push(link));
  return merged;
}

const edgeKey = (link) => `${link.source} ${link.value} ${link.target}`;

export function renderTable(data) {
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

function showView(name) {
  explore.view = name;
  $$(".explore-views button").forEach((button) =>
    button.classList.toggle("active", button.dataset.view === name));
  $("#answer").hidden = name !== "table";
  $(".explore-graph").hidden = name !== "graph";
  // Drawing nothing is not a graph; say so where the picture would be.
  $$(".explore-views button").forEach((button) => {
    button.disabled = button.dataset.view === "graph" && !explore.payload;
  });
  if (name === "graph") drawGraph().catch((error) => snack(error.message));
}

/* ---------------------------------------------------------------- the picture */

// Colour by primary type, deterministically and without a palette to run out of. Vividness is
// the second dimension: a term the run also used is saturated, one only modelled is muted.
function colourFor(node) {
  const type = node.types[0] ?? "";
  let hash = 0;
  for (const character of type) hash = (hash * 31 + character.charCodeAt(0)) % 360;
  const ran = node.graphs.includes("runtime") && node.graphs.includes("model");
  return type ? `hsl(${hash}, ${ran ? 68 : 26}%, ${ran ? 64 : 52}%)` : (ran ? "#e07a5f" : "#6c7079");
}

const sizeFor = (node) => Math.min(12, 3 + Math.log1p(node.degree) * 1.7);

function visible(payload) {
  const wanted = (node) => {
    const types = node.types.length ? node.types : [""];
    if (explore.only) return types.includes(explore.only);
    return !types.every((type) => explore.hidden.has(type));
  };
  const nodes = payload.nodes.filter(wanted);
  const ids = new Set(nodes.map((node) => node.id));
  return { nodes, links: payload.links.filter((l) => ids.has(l.source) && ids.has(l.target)) };
}

// Focus isolates: the ego network is built and handed to the renderer, so what is outside it
// is not drawn at all rather than drawn in a dimmer colour on top of the selection.
function egoNetwork(payload, roots, depth) {
  const near = new Map();
  payload.links.forEach((link) => {
    near.set(link.source, (near.get(link.source) ?? new Set()).add(link.target));
    near.set(link.target, (near.get(link.target) ?? new Set()).add(link.source));
  });
  let front = new Set(roots);
  const reached = new Set(front);
  for (let hop = 0; hop < depth; hop += 1) {
    const next = new Set();
    front.forEach((id) => (near.get(id) ?? []).forEach((other) => {
      if (!reached.has(other)) { reached.add(other); next.add(other); }
    }));
    front = next;
  }
  return {
    nodes: payload.nodes.filter((node) => reached.has(node.id)),
    links: payload.links.filter((l) => reached.has(l.source) && reached.has(l.target)),
  };
}

// What the drawing is centred on: the node in focus, and everything expanded since. Without
// the expanded ones an Expand inside a focused view merges into a subgraph nobody is drawing.
// No focus means no centre at all -- the whole result is the picture.
function drawnRoots() {
  if (!explore.focus.length) return null;
  return [explore.focus[explore.focus.length - 1], ...explore.expanded];
}

async function drawGraph() {
  const shell = $(".explore-graph");
  const target = shell.querySelector(".graph-canvas");
  const status = shell.querySelector(".graph-status");
  if (!explore.payload) return;
  const request = ++explore.drawing;
  const roots = drawnRoots();
  const drawn = visible(roots ? egoNetwork(explore.payload, roots, explore.depth) : explore.payload);
  // Neither the legend nor the trail needs the renderer, and the renderer is fetched from a
  // CDN this machine may not reach; drawing them first is what the reader keeps if it fails.
  renderLegend();
  renderCrumbs();
  if (!drawn.nodes.length) {
    target.replaceChildren();
    status.textContent = "Nothing left to draw — every type is hidden.";
    return;
  }
  status.textContent = "Loading renderer…";
  const [{ default: Sigma }, { default: Graphology }, { default: ForceAtlas2Layout }] =
    await Promise.all([
      import("https://esm.sh/sigma@3.0.2?bundle"),
      import("https://esm.sh/graphology@0.25.4?bundle"),
      import("https://esm.sh/graphology-layout-forceatlas2@0.10.1/worker?bundle"),
    ]);
  if (request !== explore.drawing) return;   // a second click got here first
  stopGraph();
  target.replaceChildren();
  const graph = new Graphology.MultiDirectedGraph();
  drawn.nodes.forEach((node, index) => graph.addNode(node.id, {
    ...node,
    x: Math.cos(index * 2.399) * (1 + index / drawn.nodes.length),
    y: Math.sin(index * 2.399) * (1 + index / drawn.nodes.length),
    size: sizeFor(node),
    color: colourFor(node),
    forceLabel: explore.pins.has(node.id) || (roots ?? []).includes(node.id),
  }));
  drawn.links.forEach((edge, index) => graph.addEdgeWithKey(String(index), edge.source, edge.target, {
    ...edge,
    size: edge.kind === "provenance" ? 0.3 : 0.6,
    color: edge.kind === "provenance" ? "#4b4e53" : "#73777d",
    type: "arrow",
  }));
  const renderer = new Sigma(graph, target, {
    renderEdgeLabels: false,
    renderLabels: true,
    labelRenderedSizeThreshold: 6,
    labelColor: { color: "#f1eee7" },
  });
  renderer.on("clickNode", ({ node }) => showDetails(node));
  explore.renderer = renderer;
  const layout = new ForceAtlas2Layout(graph, {
    settings: { barnesHutOptimize: drawn.nodes.length > 200, gravity: 1, scalingRatio: 8 },
  });
  explore.layout = layout;
  layout.start();
  settle(graph, layout, drawn, status, shell);
}

// Run the layout to convergence rather than for a fixed budget: poll how far the whole graph
// moved between polls and stop once it has stopped moving. The ceiling is a backstop for a
// graph that never settles, not the normal exit.
function settle(graph, layout, drawn, status, shell) {
  const halt = shell.querySelector(".graph-halt");
  const positions = () => graph.mapNodes((_id, attributes) => [attributes.x, attributes.y]);
  let before = positions();
  let still = 0;
  const started = Date.now();
  const done = (why) => {
    clearInterval(explore.settling);
    explore.settling = null;
    halt.hidden = true;
    layout.stop();
    status.textContent = `${drawn.nodes.length} nodes · ${drawn.links.length} relationships · ${why}`;
  };
  halt.hidden = false;
  halt.onclick = () => done("stopped");
  explore.settling = setInterval(() => {
    const now = positions();
    const extent = Math.max(1e-6, ...now.flat().map(Math.abs));
    const moved = now.reduce((total, [x, y], index) =>
      total + Math.abs(x - before[index][0]) + Math.abs(y - before[index][1]), 0);
    before = now;
    still = moved / (now.length * extent) < 2e-3 ? still + 1 : 0;
    const seconds = (Date.now() - started) / 1000;
    status.textContent = `Arranging… ${drawn.nodes.length} nodes (${seconds.toFixed(0)} s)`;
    if (still >= 5) return done("settled");
    if (seconds > 30) return done("stopped at the 30 s ceiling");
  }, 300);
}

function stopGraph() {
  clearInterval(explore.settling);
  explore.settling = null;
  explore.renderer?.kill();
  explore.layout?.kill();
  explore.renderer = explore.layout = null;
}

function renderCrumbs() {
  const crumbs = $(".graph-crumbs");
  const shell = $(".explore-graph");
  shell.querySelector(".graph-back").hidden = !explore.focus.length;
  shell.querySelector(".graph-clear").hidden = !explore.focus.length;
  crumbs.replaceChildren(...explore.focus.map((id, index) => {
    const step = document.createElement("button");
    step.className = "graph-crumb";
    step.textContent = shortName(id);
    step.onclick = () => {
      explore.focus = explore.focus.slice(0, index + 1);
      drawGraph().catch((error) => snack(error.message));
    };
    return step;
  }));
}

const shortName = (iri) => iri.split(/[/#]/).filter(Boolean).pop() ?? iri;

// 151 types is not a legend, it is a wall. The ones worth naming, an "others" bucket for the
// tail, and a filter for whatever is not in either.
function renderLegend() {
  const legend = $(".graph-legend");
  const counts = new Map();
  explore.payload.nodes.forEach((node) => {
    const type = node.types[0] ?? "(untyped)";
    counts.set(type, (counts.get(type) ?? 0) + 1);
  });
  const ranked = [...counts].sort((left, right) => right[1] - left[1]);
  const shown = ranked.filter(([type]) =>
    !explore.filter || type.toLowerCase().includes(explore.filter));
  const top = shown.slice(0, 20);
  const rest = shown.slice(20);
  legend.replaceChildren();
  const filter = document.createElement("input");
  filter.type = "search";
  filter.className = "legend-filter";
  filter.placeholder = `Filter ${counts.size} types`;
  filter.value = explore.filter;
  filter.oninput = () => {
    explore.filter = filter.value.toLowerCase().trim();
    renderLegend();
  };
  legend.append(filter);
  top.forEach(([type, count]) => legend.append(legendRow(type, count)));
  if (rest.length) {
    const others = document.createElement("button");
    others.className = "legend-row legend-others";
    others.textContent = `${rest.length} more types · ${rest.reduce((n, [, c]) => n + c, 0)} nodes`;
    others.onclick = () => {
      rest.forEach(([type]) => explore.hidden.has(type)
        ? explore.hidden.delete(type) : explore.hidden.add(type));
      drawGraph().catch((error) => snack(error.message));
    };
    legend.append(others);
  }
  const predicates = [...Object.entries(explore.payload.predicates ?? {})]
    .sort((left, right) => right[1] - left[1]).slice(0, 12);
  if (predicates.length) {
    const heading = document.createElement("div");
    heading.className = "legend-heading";
    heading.textContent = "relationships";
    legend.append(heading, ...predicates.map(([name, count]) => {
      const row = document.createElement("div");
      row.className = "legend-row legend-predicate";
      row.innerHTML = '<span class="legend-name"></span><span class="legend-count"></span>';
      row.querySelector(".legend-name").textContent = name;
      row.querySelector(".legend-count").textContent = count;
      return row;
    }));
  }
}

function legendRow(type, count) {
  const row = document.createElement("button");
  row.className = "legend-row";
  row.classList.toggle("is-off", explore.hidden.has(type));
  row.classList.toggle("is-only", explore.only === type);
  row.title = "click to hide · shift-click to isolate";
  row.innerHTML = '<i class="legend-swatch"></i><span class="legend-name"></span><span class="legend-count"></span>';
  row.querySelector(".legend-swatch").style.background = colourFor({ types: [type], graphs: ["model", "runtime"], degree: 1 });
  row.querySelector(".legend-name").textContent = type;
  row.querySelector(".legend-count").textContent = count;
  row.onclick = (event) => {
    if (event.shiftKey) explore.only = explore.only === type ? null : type;
    else explore.hidden.has(type) ? explore.hidden.delete(type) : explore.hidden.add(type);
    drawGraph().catch((error) => snack(error.message));
  };
  return row;
}

/* ---------------------------------------------------------------- one node */

function showDetails(id) {
  const node = explore.payload.nodes.find((entry) => entry.id === id);
  const details = $(".graph-details");
  if (!node) return;
  details.replaceChildren();
  const heading = document.createElement("div");
  heading.className = "detail-head";
  heading.innerHTML = '<strong></strong><span class="path"></span>';
  heading.querySelector("strong").textContent = node.label;
  heading.querySelector(".path").textContent = node.value;
  details.append(heading);
  if (node.types.length) {
    const types = document.createElement("div");
    types.className = "detail-types";
    node.types.forEach((type) => {
      const tag = document.createElement("i");
      tag.textContent = type;
      types.append(tag);
    });
    details.append(types);
  }
  const rows = Object.entries(node.attributes ?? {});
  if (rows.length) {
    const table = document.createElement("table");
    table.className = "detail-attributes";
    rows.forEach(([name, values]) => {
      const line = document.createElement("tr");
      line.innerHTML = "<th></th><td></td>";
      line.querySelector("th").textContent = name;
      line.querySelector("td").textContent = values.join(" · ");
      table.append(line);
    });
    details.append(table);
  }
  const actions = document.createElement("div");
  actions.className = "detail-actions";
  nodeActions(node).forEach(([label, run]) => {
    const button = document.createElement("button");
    button.textContent = label;
    button.onclick = run;
    actions.append(button);
  });
  details.append(actions);
}

// An action appears only when it resolves: an offer that cannot be honoured is worse than
// nothing, so each of these is built only after its target has been found.
const sameName = (left, right) =>
  String(left ?? "").toLowerCase().replaceAll("_", "-")
  === String(right ?? "").toLowerCase().replaceAll("_", "-");

// A constraint IRI reads `<model>/<motion>/<phase>/<constraint>`; anything else names no motion.
function authoredIn(iri) {
  const parts = iri.split("/");
  return ["while", "until", "when"].includes(parts.at(-2)) ? parts.at(-3) : null;
}

function nodeActions(node) {
  const actions = [];
  // Only a run has a recording to plot, a constraint list to point at, and a page whose tabs
  // these actions cross into; a generation's model graph has none of that.
  const replay = explore.path === state.runPath ? state.replay : null;
  const row = (replay?.constraints ?? []).find((constraint) =>
    constraint.evaluator === node.value
    || (constraint.members ?? []).some((member) => member.iri === node.value)
    // The graph spells a constraint as its IRI's last segment and the log header as its id --
    // `hold-position` against `hold_position` -- so the comparison folds the separator. The
    // motion comes from the IRI too, because `hold-elbow` is authored in five of them.
    || (sameName(constraint.name, node.label)
        && (!authoredIn(node.value) || sameName(constraint.motion, authoredIn(node.value)))));
  if (row?.line) actions.push(["open in source", () => openInSource(row.line)]);
  if ((replay?.signals ?? []).includes(node.label)) {
    actions.push(["plot signal", () => {
      openPanel("plots");
      addPlot([node.label], node.label, node.value);
    }]);
  }
  if (row) actions.push(["show constraint", () => showConstraint(row)]);
  if (node.graphs.includes("model") && node.graphs.includes("runtime")
      && document.querySelector('.replay-tabs button[data-panel="views"]')) {
    actions.push(["occurrences", () => openPanel("views", { iri: node.value })]);
  }
  actions.push(["focus", () => {
    explore.focus.push(node.id);
    drawGraph().catch((error) => snack(error.message));
  }]);
  // DESCRIBE names an IRI; a blank node has none to name, so it is not offered one.
  if (node.kind !== "blank") {
    actions.push(["expand", () => expand(node).catch((error) => snack(error.message))]);
  }
  actions.push([explore.pins.has(node.id) ? "unpin" : "pin", () => {
    explore.pins.has(node.id) ? explore.pins.delete(node.id) : explore.pins.add(node.id);
    showDetails(node.id);
    drawGraph().catch((error) => snack(error.message));
  }]);
  return actions;
}

// Expand merges; it never replaces. Exploring is meant to accumulate, and a click that threw
// away what was already found would make every step cost the last one.
async function expand(node) {
  const data = await post("/api/sparql", {
    path: explore.path,
    query: `DESCRIBE <${node.value}>`,
  });
  if (!data.graph) return snack("nothing more to draw for this node");
  const nodes = new Map(explore.payload.nodes.map((entry) => [entry.id, entry]));
  data.graph.nodes.forEach((entry) => nodes.has(entry.id) || nodes.set(entry.id, entry));
  const edges = new Set(explore.payload.links.map(edgeKey));
  const links = [...explore.payload.links,
    ...data.graph.links.filter((link) => !edges.has(edgeKey(link)))];
  explore.payload = { ...explore.payload, nodes: [...nodes.values()], links };
  explore.expanded.add(node.id);
  await drawGraph();
  showDetails(node.id);
}

// Where exploring ends: the node set as SPARQL someone else can run, saved like any other tab.
// Both ends are named, not just the subject: `VALUES ?s { .. } ?s ?p ?o` also drags in every
// second-hop object, so rerunning it would draw a bigger graph than the one being exported.
// Literals stay, because they are folded into their subject's attributes rather than drawn.
function exportQuery() {
  const roots = explore.focus.length
    ? [explore.focus[explore.focus.length - 1]]
    : null;
  const drawn = visible(roots ? egoNetwork(explore.payload, roots, explore.depth) : explore.payload);
  const named = drawn.nodes.filter((node) => node.kind !== "blank");
  const iris = named.map((node) => `    <${node.value}>`).join("\n");
  if (!iris) return snack("nothing drawn to export");
  const inside = named.map((node) => `<${node.value}>`).join(", ");
  const query = "CONSTRUCT { ?s ?p ?o }\nWHERE {\n  VALUES ?s {\n"
    + `${iris}\n  }\n  ?s ?p ?o\n  FILTER(!isIRI(?o) || ?o IN (${inside}))\n}`;
  state.queries.push({ query });
  state.query = state.queries.length - 1;
  $("#query").value = query;
  saveQueries();
  renderRail();
  runQuery();
}

/* ---------------------------------------------------------------- going elsewhere */

// The run page's own tabs are the way between panels; clicking one keeps whatever 017 or the
// plots panel reads out of the hash, and an absent tab means the action was never offered.
function openPanel(panel, params = {}) {
  const view = new URLSearchParams(location.hash.slice(1));
  Object.entries(params).forEach(([key, value]) =>
    value == null ? view.delete(key) : view.set(key, value));
  history.replaceState(null, "", `#${view}`);
  document.querySelector(`.replay-tabs button[data-panel="${panel}"]`)?.click();
}

async function openInSource(line) {
  explore.generation ??= await api(
    `/api/generation?path=${encodeURIComponent(state.generationPath)}`);
  const model = explore.generation.source_files.find((source) => source.model);
  if (!model) return snack("this generation archived no model source");
  await showSource(model.workspace, model.path, line);
}

function showConstraint(row) {
  openPanel("plots");
  const target = $$("#constraints .constraint").find((element) =>
    element.dataset.motion === row.motion
    && element.querySelector("strong")?.textContent === row.name);
  if (!target) return snack(`${row.name} is not in this run's constraint list`);
  target.scrollIntoView({ block: "center" });
  target.click();
}

/* ---------------------------------------------------------------- wiring */

// A generation's model graph on its own, reached from the generation page. A run explores the
// same way from its own tab, with its recording joined in.
export async function showExplore(path, push = true) {
  if (push) setView("explore", path);
  state.runPath = null;
  state.generationPath = path;
  $("#content").innerHTML = '<div class="explore-page"><div class="page-heading">'
    + '<h1></h1><span class="eyebrow">EXPLORE</span></div>'
    + `<p class="path"></p>${EXPLORE_MARKUP}</div>`;
  $(".explore-page h1").textContent = path.split("/").pop();
  $(".explore-page > .path").textContent = path;
  await bindExplore(path, true);
}

export async function bindExplore(path, run = false) {
  stopGraph();
  explore.path = path;
  explore.view = "table";
  explore.data = explore.payload = explore.generation = null;
  explore.focus = [];
  explore.expanded = new Set();
  explore.pins = new Set();
  explore.hidden = new Set();
  explore.only = null;
  explore.filter = "";
  state.queries = [];
  state.query = -1;
  $(".explore-canned").replaceChildren(...Object.entries(CANNED).map(([label, query]) => {
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
  $$(".explore-views button").forEach((button) => {
    button.onclick = () => showView(button.dataset.view);
  });
  const shell = $(".explore-graph");
  $$(".graph-depth button").forEach((button) => {
    button.onclick = () => {
      explore.depth = Number(button.dataset.depth);
      $$(".graph-depth button").forEach((other) => other.classList.toggle("active", other === button));
      drawGraph().catch((error) => snack(error.message));
    };
  });
  shell.querySelector(".graph-back").onclick = () => {
    explore.focus.pop();
    drawGraph().catch((error) => snack(error.message));
  };
  shell.querySelector(".graph-clear").onclick = () => {
    explore.focus = [];
    explore.expanded = new Set();
    drawGraph().catch((error) => snack(error.message));
  };
  shell.querySelector(".graph-export").onclick = exportQuery;
  shell.querySelector(".graph-fullscreen").onclick = () =>
    document.fullscreenElement ? document.exitFullscreen() : shell.requestFullscreen();
  // A search that only highlights is the complaint this page exists to fix: picking a match
  // focuses it, so the answer to "where is it" is a drawing of it and its neighbours.
  shell.querySelector(".graph-search").oninput = (event) => {
    const wanted = event.target.value.toLowerCase().trim();
    const matches = shell.querySelector(".graph-matches");
    if (!wanted || !explore.payload) return matches.replaceChildren();
    // A term whose own name matches is what was searched for; one that merely lives under a
    // matching IRI is a relative of it, and goes below.
    matches.replaceChildren(...explore.payload.nodes
      .filter((node) => `${node.label} ${node.value}`.toLowerCase().includes(wanted))
      .sort((left, right) => Number(right.label.toLowerCase().includes(wanted))
        - Number(left.label.toLowerCase().includes(wanted)))
      .slice(0, 12)
      .map((node) => {
        const button = document.createElement("button");
        button.dataset.kind = node.types[0] ?? "term";
        button.textContent = node.label;
        button.onclick = () => {
          explore.focus.push(node.id);
          showDetails(node.id);
          drawGraph().catch((error) => snack(error.message));
        };
        return button;
      }));
  };
  $("#query").value = CANNED["everything in this graph"];
  renderRail();
  const stored = state.runPath === path
    ? await api(`/api/queries?path=${encodeURIComponent(path)}`)
    : [];
  if (stored.length) {
    state.queries = stored.map((query) => ({ query }));
    renderRail();
  }
  if (run) await runQuery();
}
