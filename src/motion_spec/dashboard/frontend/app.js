// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

const MAGNIFIER = "M8.2,2 A6,6 0 1,0 8.2,14.2 A6,6 0 1,0 8.2,2 M12.7,12.7 L17.4,17.4";
const MAGNIFIER_IN = `path://${MAGNIFIER} M5.4,8.1 L11,8.1 M8.2,5.3 L8.2,10.9`;
const MAGNIFIER_OUT = `path://${MAGNIFIER} M5.4,8.1 L11,8.1`;
const RESET_ARROW = "path://M15.4,9.6 A6.2,6.2 0 1,1 9.2,3.4 M9.2,0.7 L9.2,6.1 M6.5,3.4 L11.9,3.4";
let plotKeys = 0;
const state = { replay: null, queries: [], query: -1, anchor: null, runPath: null, generationPath: null, tab: "logs", frame: 0, charts: [], selected: new Set(), timer: null, roots: {}, cache: {}, listRequest: 0 };
const $ = (selector) => document.querySelector(selector);
let snackTimer;

function snack(message) {
  $("#snack").textContent = message;
  $("#snack").classList.add("visible");
  clearTimeout(snackTimer);
  snackTimer = setTimeout(() => $("#snack").classList.remove("visible"), 1400);
}

async function copyText(value) {
  await navigator.clipboard.writeText(value);
  snack("Copied path");
}

async function api(path) {
  const response = await fetch(path);
  const data = await response.json();
  if (!response.ok) throw Error(data.error);
  return data;
}

async function post(path, body) {
  const response = await fetch(path, {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
  });
  const data = await response.json();
  if (!response.ok) throw Error(data.error);
  return data;
}

function listItem(name, detail, click, path) {
  const button = document.createElement("button");
  button.className = "item";
  button.dataset.path = path;
  button.classList.toggle("picked", state.selected.has(path));
  const label = document.createElement("span");
  label.className = "item-name";
  label.textContent = name;
  const viewport = document.createElement("span");
  viewport.className = "item-name-viewport";
  viewport.append(label);
  button.append(viewport);
  if (detail) {
    const small = document.createElement("small");
    small.textContent = detail;
    button.append(small);
  }
  button.onclick = (event) => click(event);
  return button;
}

function fact(label, value) {
  return `<div class="fact"><label>${label}</label>${value ?? "—"}</div>`;
}

function stampText(iso) {
  if (!iso) return "unknown time";
  return new Date(iso).toLocaleString(undefined, {
    day: "2-digit", month: "short", year: "numeric",
    hour: "2-digit", minute: "2-digit", hour12: false,
  });
}

function formatBytes(bytes) {
  return bytes >= 1024 ** 3
    ? `${(bytes / 1024 ** 3).toFixed(1)} GB`
    : `${(bytes / 1024 / 1024).toFixed(1)} MB`;
}

async function loadGenerations(refresh = false) {
  const request = ++state.listRequest;
  if (refresh || !state.cache.generations) {
    const [generations, storage, roots] = await Promise.all([api("/api/generations"), api("/api/storage"), api("/api/roots")]);
    state.cache.generations = { generations, storage, roots };
  }
  if (request !== state.listRequest || state.tab !== "logs") return;
  $("#list-title").textContent = "GENERATIONS";
  const { generations, storage, roots } = state.cache.generations;
  state.roots = roots;
  $("#storage").textContent = formatBytes(storage.generations_bytes);
  $("#generation-root").hidden = false;
  $("#generation-root").value = roots.logs;
  const groups = new Map();
  generations.forEach((generation) => groups.set(generation.name, [...(groups.get(generation.name) ?? []), generation]));
  $("#browser").replaceChildren(...[...groups].map(([model, entries]) => {
    const group = document.createElement("details");
    group.className = "generation-group";
    group.open = true;
    const summary = document.createElement("summary");
    const label = document.createElement("span");
    label.className = "group-name";
    label.textContent = model;
    summary.append(label);
    const all = document.createElement("button");
    all.className = "select-group";
    all.title = `Select every generation of ${model}`;
    all.textContent = "select all";
    all.onclick = (event) => {
      event.preventDefault();
      event.stopPropagation();
      const paths = entries.map((generation) => generation.path);
      const adding = paths.some((path) => !state.selected.has(path));
      paths.forEach((path) => {
        if (state.selected.has(path) !== adding) toggleSelection(path, "sidebar");
      });
    };
    summary.append(all);
    group.append(summary, ...entries.map((generation) => listItem(
      generation.created,
      [generation.variant, stampText(generation.built_at),
       `${generation.runs} runs`, formatBytes(generation.size_bytes)].filter(Boolean).join(" · "),
      (event) => {
        if (event.shiftKey) return pickRange(generation.path, $("#browser"), "sidebar");
        return event.metaKey || event.ctrlKey
          ? toggleSelection(generation.path, "sidebar")
          : selectGeneration(generation.path);
      },
      generation.path,
    )));
    return group;
  }));
  filterGenerations($("#search").value);
  highlightGeneration();
}

function filterGenerations(value) {
  const query = value.toLowerCase();
  document.querySelectorAll(".generation-group").forEach((group) => {
    let visible = false;
    group.querySelectorAll(".item").forEach((item) => {
      item.hidden = !item.textContent.toLowerCase().includes(query);
      visible ||= !item.hidden;
    });
    group.hidden = !visible;
    if (query && visible) group.open = true;
  });
}

async function selectGeneration(path) {
  state.anchor = path;
  setView("generation", path);
  state.generationPath = path;
  const generation = await api(`/api/generation?path=${encodeURIComponent(path)}`);
  const runs = await api(`/api/runs?path=${encodeURIComponent(path)}`);
  $("#status").textContent = generation.path;
  highlightGeneration();

  const page = $("#generation-template").content.cloneNode(true);
  page.querySelector("h1").textContent = generation.spec_name ?? generation.name;
  page.querySelector(".path").textContent = generation.folder;
  page.querySelector(".copy-generation-path").onclick = () => copyText(generation.folder);
  page.querySelector(".generation-description").textContent = generation.description ?? "";
  page.querySelector(".facts").innerHTML = [
    fact("Source", generation.source),
    fact("Generated", generation.created),
    fact("Backend", generation.backend),
    fact("Platform", generation.platform),
    fact("Runtime", generation.simulated ? "Simulated" : "Hardware"),
    fact("Motions", generation.motions),
    fact("Authored constraints", generation.authored_constraints),
    fact("Schema", generation.schema_hash),
    fact("Runs", generation.runs),
    fact("Folder size", formatBytes(generation.size_bytes)),
  ].join("");
  page.querySelector(".source-files").replaceChildren(...generation.source_files.map((source) => {
    const entry = document.createElement("div");
    entry.className = "source-file";
    entry.title = source.path;
    entry.innerHTML = `<span>${source.name}</span><button title="Copy full path">⧉</button>`;
    entry.querySelector("button").onclick = () => copyText(source.path);
    return entry;
  }));
  const generatedGroups = new Map();
  generation.generated_files.forEach((source) => {
    const [folder, ...rest] = source.name.split("/");
    generatedGroups.set(folder, [...(generatedGroups.get(folder) ?? []), { ...source, name: rest.join("/") || folder }]);
  });
  page.querySelector(".generated-files").replaceChildren(...[...generatedGroups].map(([folder, files]) => {
    const group = document.createElement("details");
    group.className = "generated-folder";
    const summary = document.createElement("summary");
    summary.textContent = `${folder} · ${files.length}`;
    group.append(summary, ...files.map((source) => {
      const entry = document.createElement("div");
      entry.className = "source-file";
      entry.title = source.path;
      entry.innerHTML = `<span>${source.name}</span><button title="Copy full path">⧉</button>`;
      entry.querySelector("button").onclick = () => copyText(source.path);
      return entry;
    }));
    return group;
  }));
  const runList = page.querySelector(".runs");
  const pages = Math.ceil(runs.length / 10);
  let runPage = 0;
  const renderRuns = () => runList.replaceChildren(...runs.slice(runPage * 10, runPage * 10 + 10).map((run, index) => {
    const row = document.createElement("div");
    row.className = "run";
    row.dataset.path = run.path;
    row.classList.toggle("picked", state.selected.has(run.path));
    row.innerHTML = `<span>${runPage * 10 + index + 1}</span><strong>${run.id}</strong><span>${run.started}</span><span>${run.duration_s.toFixed(2)} s</span><span>${(run.written_frames ?? 0).toLocaleString()}</span><span class="badge">${run.status ?? (run.complete ? "COMPLETED" : "incomplete")}</span>`;
    row.title = "Open replay; Ctrl/Cmd-click to select";
    row.onclick = (event) => {
      if (event.shiftKey) return pickRange(run.path, row.parentElement, "main");
      return event.metaKey || event.ctrlKey
        ? toggleSelection(run.path, "main")
        : loadReplay(run.path);
    };
    return row;
  }));
  renderRuns();
  const pager = page.querySelector(".run-pagination");
  if (pages > 1) pager.innerHTML = `<button>‹</button><span>1 / ${pages}</span><button>›</button>`;
  pager.querySelectorAll("button")[0]?.addEventListener("click", () => { runPage = Math.max(0, runPage - 1); renderRuns(); pager.querySelector("span").textContent = `${runPage + 1} / ${pages}`; });
  pager.querySelectorAll("button")[1]?.addEventListener("click", () => { runPage = Math.min(pages - 1, runPage + 1); renderRuns(); pager.querySelector("span").textContent = `${runPage + 1} / ${pages}`; });
  $("#content").replaceChildren(page);
  const graphPage = $("#content");
  const options = graphPage.querySelector(".graph-options");
  const graphStatus = graphPage.querySelector(".graph-status");
  const graphSearch = graphPage.querySelector(".graph-search");
  const graphMatches = graphPage.querySelector(".graph-matches");
  const shell = graphPage.querySelector(".graph-shell");
  const target = graphPage.querySelector(".generation-graph");
  const details = graphPage.querySelector(".graph-details");
  let graphRequest = 0;
  options.replaceChildren(...generation.rdf_graphs.map((name) => {
    const label = document.createElement("label");
    label.innerHTML = `<input type="checkbox" value="${name}"> ${name}`;
    return label;
  }));
  const selectedFiles = () => [...options.querySelectorAll("input:checked")].map((input) => input.value);
  const renderGraph = async () => {
    const selected = selectedFiles();
    const request = ++graphRequest;
    if (!selected.length) {
      shell.hidden = true;
      graphSearch.disabled = true;
      graphMatches.replaceChildren();
      graphStatus.textContent = "Select an RDF file to render its complete graph.";
      return;
    }
    const query = new URLSearchParams({ path });
    selected.forEach((name) => query.append("graph", name));
    graphStatus.textContent = "Rendering RDF graph…";
    try {
      const data = await api(`/api/generation-graph?${query}`);
      if (request !== graphRequest) return;
      const [{ default: Sigma }, { default: Graphology }, { default: ForceAtlas2Layout }] = await Promise.all([
        import("https://esm.sh/sigma@3.0.2?bundle"),
        import("https://esm.sh/graphology@0.25.4?bundle"),
        import("https://esm.sh/graphology-layout-forceatlas2@0.10.1/worker?bundle"),
      ]);
      if (request !== graphRequest) return;
      state.graphRenderer?.kill();
      state.graphLayout?.kill();
      shell.hidden = false;
      target.replaceChildren();
      if (!data.nodes.length) {
        graphSearch.disabled = true;
        graphStatus.textContent = "This JSON-LD file contains no RDF triples.";
        target.innerHTML = '<div class="graph-empty">No RDF resources or relationships to render.</div>';
        return;
      }
      const graph = new Graphology.MultiDirectedGraph();
      data.nodes.forEach((node, index) => graph.addNode(node.id, { ...node, x: Math.cos(index * 2.399), y: Math.sin(index * 2.399), size: 3, color: "#607fd4" }));
      data.links.forEach((edge, index) => graph.addEdgeWithKey(String(index), edge.source, edge.target, { ...edge, size: .4, color: "#73777d", type: "arrow" }));
      let selectedNode = null;
      let selectedNeighbors = new Set();
      let selectedEdges = new Set();
      let hasSelection = false;
      const renderer = new Sigma(graph, target, {
        renderEdgeLabels: false,
        renderLabels: true,
        labelRenderedSizeThreshold: 0,
        labelColor: { color: "#f1eee7" },
        defaultDrawNodeHover: (context, data, settings) => {
          if (!data.label) return;
          const font = `${settings.labelWeight} ${settings.labelSize}px ${settings.labelFont}`;
          context.font = font;
          const x = data.x + data.size + 6;
          const y = data.y - settings.labelSize / 2 - 5;
          const width = context.measureText(data.label).width + 12;
          context.fillStyle = "#202327";
          context.fillRect(x - 6, y, width, settings.labelSize + 10);
          context.fillStyle = "#f1eee7";
          context.fillText(data.label, x, y + settings.labelSize + 1);
        },
        nodeReducer: (node, attributes) => !hasSelection ? attributes : { ...attributes, color: node === selectedNode || selectedNeighbors.has(node) ? "#e07a5f" : "#4b4e53", forceLabel: node === selectedNode },
        edgeReducer: (edge, attributes) => !hasSelection ? attributes : { ...attributes, color: selectedEdges.has(edge) ? "#e07a5f" : "#30343a", size: selectedEdges.has(edge) ? 1.5 : .2 },
      });
      const focus = (id) => {
        selectedNode = id;
        selectedNeighbors = new Set(graph.neighbors(id));
        selectedEdges = new Set(graph.edges(id));
        hasSelection = true;
        const node = graph.getNodeAttributes(id);
        details.textContent = `${node.label}\n${node.value}`;
        renderer.refresh();
      };
      const focusEdge = (id) => {
        const edge = graph.getEdgeAttributes(id);
        selectedNode = null;
        selectedNeighbors = new Set([edge.source, edge.target]);
        selectedEdges = new Set([id]);
        hasSelection = true;
        details.textContent = `${edge.label}\n${edge.value}`;
        renderer.refresh();
      };
      renderer.on("clickNode", ({ node }) => focus(node));
      renderer.on("downNode", ({ node }) => focus(node));
      state.graphRenderer = renderer;
      const layout = new ForceAtlas2Layout(graph, { settings: { barnesHutOptimize: true, gravity: 1, scalingRatio: 8 } });
      state.graphLayout = layout;
      layout.start();
      graphStatus.textContent = "Arranging RDF graph…";
      setTimeout(() => {
        if (state.graphLayout !== layout) return;
        layout.stop();
        graphStatus.textContent = `${data.nodes.length} resources · ${data.links.length} relationships`;
      }, Math.min(5000, 500 + data.nodes.length * 10));
      graphSearch.disabled = false;
      graphSearch.value = "";
      graphMatches.replaceChildren();
      graphSearch.oninput = () => {
        const query = graphSearch.value.toLowerCase().trim();
        if (!query) { graphMatches.replaceChildren(); selectedNode = null; selectedNeighbors = new Set(); selectedEdges = new Set(); hasSelection = false; renderer.refresh(); return; }
        const matches = [
          ...data.nodes.filter((node) => `${node.label} ${node.value}`.toLowerCase().includes(query)).map((node) => ({ kind: "node", label: node.label, select: () => focus(node.id) })),
          ...data.links.map((edge, index) => ({ ...edge, id: String(index) })).filter((edge) => `${edge.label} ${edge.value}`.toLowerCase().includes(query)).map((edge) => ({
            kind: "edge",
            label: `${data.nodes.find((node) => node.id === edge.source)?.label ?? edge.source} → ${edge.label} → ${data.nodes.find((node) => node.id === edge.target)?.label ?? edge.target}`,
            select: () => focusEdge(edge.id),
          })),
        ];
        graphMatches.replaceChildren(...matches.slice(0, 12).map((match) => {
          const button = document.createElement("button");
          button.dataset.kind = match.kind;
          button.textContent = match.label;
          button.onclick = match.select;
          return button;
        }));
      };
      graphPage.querySelector(".graph-fullscreen").onclick = () =>
        document.fullscreenElement ? document.exitFullscreen() : shell.requestFullscreen();
    } catch (error) { if (request === graphRequest) { shell.hidden = true; graphStatus.textContent = error.message; } }
  };
  options.addEventListener("change", renderGraph);
}

async function loadReplay(path) {
  state.anchor = path;
  stopPlayback();
  setView("run", path);
  state.replay = await api(`/api/replay?path=${encodeURIComponent(path)}`);
  state.runPath = path;
  state.generationPath = state.replay.generation;
  highlightGeneration();
  state.frame = 0;
  state.charts.forEach((chart) => chart.dispose());
  state.charts = [];
  $("#status").textContent = "";
  $("#content").innerHTML = `<div class="replay"><div class="replay-heading"><div><div class="eyebrow">REPLAY</div><h1>${path.split("/").pop()}</h1><p class="path">${state.replay.frames.toLocaleString()} frames · ${state.replay.duration.toFixed(3)} s</p></div><button id="back" title="Back to generation">← Back</button></div><div class="replay-tabs"><button data-panel="plots" class="active">Plots</button><button data-panel="sparql">SPARQL</button></div><section id="panel-plots"><div class="constraint-panel"><div class="eyebrow">SOURCE CONSTRAINTS</div><input id="constraint-search" type="search" placeholder="Search .robmot constraints"><div id="constraints" class="constraints"></div></div><div class="chart-controls"><button id="plot">Add empty plot</button><button id="notebook">Open in Jupyter</button></div><div id="plots" class="plots"></div></section><section id="panel-sparql" hidden><div class="sparql"><div class="query-rail"><button id="new-query" class="new-query">+ query</button></div><div class="sparql-body"><div class="sparql-canned"></div><textarea id="query" spellcheck="false"></textarea><div class="sparql-run"><button id="ask">Run query</button><span id="query-status" class="path"></span></div><div id="answer"></div></div></div></section></div><div class="transport"><div class="transport-controls"><button id="step-back" title="Previous frame">‹</button><button id="play">Play</button><button id="step-forward" title="Next frame">›</button><span id="readout" class="path"></span></div><div class="markers"></div><input class="timeline" type="range" min="0" max="${state.replay.frames - 1}" value="0"></div>`;

  populateConstraints();
  $("#plot").onclick = () => addPlot([]);
  $("#notebook").onclick = () => openNotebook(state.runPath).catch((error) => snack(error.message));
  bindPanels();
  bindSparql().catch(showError);
  bindTransport();
  $("#back").onclick = () => selectGeneration(state.replay.generation);
  updateReadout();
}

function populateConstraints() {
  const groups = new Map();
  state.replay.constraints.forEach((constraint) => {
    const motion = constraint.motion ?? "shared";
    groups.set(motion, [...(groups.get(motion) ?? []), constraint]);
  });
  const container = $("#constraints");
  container.replaceChildren(...[...groups].flatMap(([motion, constraints]) => {
    const heading = document.createElement("div");
    heading.className = "constraint-motion";
    heading.textContent = motion;
    return [heading, ...constraints.map((constraint) => {
    const row = document.createElement("div");
    const line = document.createElement("span");
    const name = document.createElement("strong");
    const expression = document.createElement("span");
    row.className = "constraint";
    row.dataset.kind = constraint.kind;
    if (!constraint.window) row.dataset.idle = "true";
    line.textContent = `L${constraint.line}`;
    name.textContent = constraint.name;
    expression.textContent = constraint.expression;
    row.append(line, name, expression);
    if (constraint.tracking.length || constraint.control.length || constraint.monitors.length) {
      row.dataset.plottable = "true";
      row.title = constraint.window
        ? `Plot this ${constraint.kind} constraint`
        : "This motion never ran in this recording";
      row.onclick = () => {
        if (row.dataset.plotted) {
          $("#plots").querySelectorAll(`[data-row="${row.dataset.plotKey}"] .remove-plot`)
            .forEach((button) => button.click());
          return;
        }
        row.dataset.plotted = "true";
        const title = `${constraint.motion ?? "shared"} / ${constraint.name}`;
        const between = constraint.between.length ? ` · ${constraint.between.join(" vs ")}` : "";
        const detail = `L${constraint.line}: ${constraint.expression}${between}`;
        // measured against its setpoint where there is one; otherwise the error against zero
        const tracked = constraint.tracking.length ? constraint.tracking : constraint.error;
        const bands = constraint.tracking.length
          ? constraint
          : { ...constraint, setpoints: [{ label: "satisfied", value: 0 }] };
        if (tracked.length) addPlot(tracked, `${title} · constraint`, detail, { row, constraint: bands });
        if (constraint.kind === "monitored") {
          if (constraint.monitors.length) {
            addPlot(constraint.monitors, `${title} · monitor`, detail, { row, constraint });
          }
          return;
        }
        const machinery = [...constraint.error, ...constraint.control];
        if (machinery.length) addPlot(machinery, `${title} · controller`, detail, { row, constraint, gains: constraint.gains });
      };
    } else {
      row.dataset.unavailable = "true";
      row.title = constraint.evaluator
        ? "No recorded signal"
        : "Nothing evaluates this constraint in this generation";
    }
    return row;
    })];
  }));
  $("#constraint-search").oninput = (event) => filter(".constraint", event.target.value);
}

const CANNED = {
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

function bindPanels() {
  const show = (panel, remember = true) => {
    document.querySelectorAll(".replay-tabs button").forEach((button) =>
      button.classList.toggle("active", button.dataset.panel === panel));
    $("#panel-plots").hidden = panel !== "plots";
    $("#panel-sparql").hidden = panel !== "sparql";
    state.charts.forEach((chart) => chart.resize());
    if (!remember) return;
    const view = new URLSearchParams(location.hash.slice(1));
    view.set("panel", panel);
    history.replaceState(null, "", `#${view}`);
  };
  document.querySelectorAll(".replay-tabs button").forEach((button) => {
    button.onclick = () => show(button.dataset.panel);
  });
  // a reload lands back where it left off, like the run it reopens
  show(new URLSearchParams(location.hash.slice(1)).get("panel") ?? "plots", false);
}

function queryLabel(query) {
  const line = query.split("\n").map((text) => text.trim())
    .find((text) => text && !text.startsWith("#")) ?? "query";
  return line.length > 30 ? `${line.slice(0, 29)}…` : line;
}

function renderRail() {
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

function selectQuery(index) {
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

function saveQueries() {
  // beside the run, so they outlive this browser and travel with the archive
  post("/api/queries", { path: state.runPath, queries: state.queries.map((entry) => entry.query) })
    .catch((error) => { $("#query-status").textContent = `not saved: ${error.message}`; });
}

async function runQuery() {
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

async function bindSparql() {
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

function renderAnswer(data) {
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

function bindTransport() {
  $(".timeline").oninput = (event) => seek(Number(event.target.value));
  $("#step-back").onclick = () => seek(state.frame - 1);
  $("#step-forward").onclick = () => seek(state.frame + 1);
  $("#play").onclick = togglePlayback;
  renderMarkers();
}

function renderMarkers() {
  $(".markers").replaceChildren(...state.replay.events.map((event) => {
    const marker = document.createElement("button");
    marker.className = `marker marker-${event.kind}`;
    marker.style.left = trackLeft(event.frame);
    marker.title = `${event.label} @ frame ${event.frame}`;
    marker.onclick = () => seek(event.frame);
    return marker;
  }));
  const playhead = document.createElement("div");
  playhead.className = "playhead";
  $(".markers").append(playhead);
  movePlayhead();
}

function trackFraction(frame) {
  return frame / Math.max(1, state.replay.frames - 1);
}

function trackLeft(frame) {
  // the thumb travels inset by half its width, so markers must follow the same geometry
  return `calc(var(--thumb) / 2 + ${trackFraction(frame)} * (100% - var(--thumb)))`;
}

function movePlayhead() {
  $(".transport").style.setProperty("--f", trackFraction(state.frame));
}

function exportChart(chart, name, format) {
  if (format === "png" || format === "jpg") {
    const link = document.createElement("a");
    link.download = `${name}.${format}`;
    link.href = chart.getDataURL({
      type: format === "png" ? "png" : "jpeg", pixelRatio: 2, backgroundColor: "#16181b",
    });
    return link.click();
  }
  const svg = vectorSvg(chart);
  if (format === "svg") {
    const link = document.createElement("a");
    link.download = `${name}.svg`;
    link.href = URL.createObjectURL(new Blob([svg], { type: "image/svg+xml" }));
    link.click();
    return setTimeout(() => URL.revokeObjectURL(link.href), 1000);
  }
  printVector(svg, chart.getWidth(), chart.getHeight());
}

function vectorSvg(chart) {
  // the on-screen chart is canvas; re-render the same option through the SVG renderer
  const holder = document.createElement("div");
  holder.style.cssText =
    `position:fixed;left:-10000px;top:0;width:${chart.getWidth()}px;height:${chart.getHeight()}px`;
  document.body.append(holder);
  const vector = echarts.init(holder, null, { renderer: "svg" });
  vector.setOption({ ...chart.getOption(), animation: false, toolbox: { show: false } });
  const svg = vector.renderToSVGString();
  vector.dispose();
  holder.remove();
  return svg;
}

function printVector(svg, width, height) {
  const frame = document.createElement("iframe");
  frame.style.cssText = "position:fixed;right:0;bottom:0;width:0;height:0;border:0";
  document.body.append(frame);
  frame.contentDocument.write(
    `<style>@page{size:${width}px ${height}px;margin:0}body{margin:0}</style>${svg}`,
  );
  frame.contentDocument.close();
  frame.contentWindow.focus();
  frame.contentWindow.print();
  setTimeout(() => frame.remove(), 60000);
}

function setpointBands(constraint) {
  if (!constraint?.setpoints?.length) return {};
  const tolerance = constraint.tolerance ?? 0;
  return {
    markLine: {
      symbol: "none", animation: false, silent: true,
      label: {
        color: "#f0c36a", position: "insideEndTop",
        formatter: ({ value }) => `setpoint ${value}`,
      },
      lineStyle: { color: "#f0c36a", type: "dashed", width: 1 },
      data: constraint.setpoints.map((point) => ({ yAxis: point.value })),
    },
    markArea: {
      silent: true,
      itemStyle: { color: "rgba(240, 195, 106, .1)" },
      data: constraint.setpoints.map((point) => [
        { yAxis: point.value - tolerance }, { yAxis: point.value + tolerance },
      ]),
    },
  };
}

function cursorOption(frame, index = 0) {
  return { series: [...Array.from({ length: index }, () => ({})), { markLine: {
    symbol: "none", animation: false, silent: true,
    label: { show: false },
    lineStyle: { color: "#f1eee7", width: 1, opacity: .7 },
    data: [{ xAxis: frame }],
  } }] };
}

function updateCursor() {
  state.charts.forEach((chart) => chart.setOption(cursorOption(state.frame, chart.cursorIndex ?? 0)));
}

function filter(selector, value) {
  const query = value.toLowerCase();
  document.querySelectorAll(selector).forEach((item) => {
    item.hidden = !item.textContent.toLowerCase().includes(query);
  });
}

function highlightGeneration() {
  document.querySelectorAll("#browser .item").forEach((item) => {
    item.classList.toggle("selected", item.dataset.path === state.generationPath);
  });
  // only bring it into view if it is not already there; scrolling a visible item is disorienting
  const selected = document.querySelector("#browser .item.selected");
  if (!selected) return;
  const list = $("#browser").getBoundingClientRect();
  const item = selected.getBoundingClientRect();
  if (item.top < list.top || item.bottom > list.bottom) selected.scrollIntoView({ block: "nearest" });
}

function pickRange(path, container, source) {
  // shift extends from the last item picked, over what is actually on screen
  const paths = [...container.querySelectorAll("[data-path]")]
    .filter((item) => item.offsetParent !== null)
    .map((item) => item.dataset.path);
  const from = paths.indexOf(state.anchor ?? path);
  const to = paths.indexOf(path);
  if (from < 0 || to < 0) return toggleSelection(path, source);
  const [start, end] = from < to ? [from, to] : [to, from];
  paths.slice(start, end + 1).forEach((item) => {
    if (!state.selected.has(item)) toggleSelection(item, source, false);
  });
}

function toggleSelection(path, source = "sidebar", anchor = true) {
  if (anchor) state.anchor = path;
  state.selected.has(path) ? state.selected.delete(path) : state.selected.add(path);
  document.querySelectorAll("[data-path]").forEach((item) => {
    if (item.dataset.path === path) item.classList.toggle("picked", state.selected.has(path));
  });
  const button = $("#delete-selected");
  $("#selection-actions").dataset.side = source;
  $("#selection-actions").hidden = !state.selected.size;
  $("#selection-count").textContent = `${state.selected.size} selected`;
  button.textContent = "Delete";
}

function seek(frame) {
  state.frame = Math.max(0, Math.min(state.replay.frames - 1, frame));
  $(".timeline").value = state.frame;
  updateReadout();
  updateCursor();
  movePlayhead();
}

function togglePlayback() {
  if (state.timer) return stopPlayback();
  const fps = state.replay.frames / Math.max(state.replay.duration, 1e-9);
  let cursor = state.frame >= state.replay.frames - 1 ? 0 : state.frame;
  let last = performance.now();
  const tick = (now) => {
    cursor += (now - last) * fps / 1000;
    last = now;
    if (cursor >= state.replay.frames - 1) {
      seek(state.replay.frames - 1);
      return stopPlayback();
    }
    seek(Math.round(cursor));
    state.timer = requestAnimationFrame(tick);
  };
  state.timer = requestAnimationFrame(tick);
  $("#play").textContent = "Pause";
}

function stopPlayback() {
  if (!state.timer) return;
  cancelAnimationFrame(state.timer);
  state.timer = null;
  if ($("#play")) $("#play").textContent = "Play";
}

function updateReadout() {
  const entered = [...state.replay.events].reverse()
    .find((item) => item.kind === "state" && item.frame <= state.frame);
  const time = state.replay.duration * state.frame / Math.max(1, state.replay.frames - 1);
  $("#readout").textContent = `${time.toFixed(3)} s · frame ${state.frame.toLocaleString()} / ${state.replay.frames.toLocaleString()} · ${entered?.label ?? "—"}`;
}

function addPlot(signals = [], title = signals.join(" · ") || "New plot", detail = "", options = {}) {
  const { row = null, constraint = null, gains = null } = options;
  const card = document.createElement("section");
  card.className = "plot-card";
  card.innerHTML = '<header><div><strong></strong><small></small></div><div class="plot-actions"><details class="export-plot"><summary title="Export plot"><svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.3" stroke-linecap="square"><path d="M9.5 2.5h4v4"/><path d="M13.5 2.5 8 8"/><path d="M12 9v4.5H2.5V4h4.5"/></svg></summary><div class="export-menu"><button value="png">PNG</button><button value="jpg">JPG</button><button value="svg">SVG</button><button value="pdf">PDF</button></div></details><button class="expand-plot" title="Fullscreen plot">⛶</button><button class="remove-plot" title="Remove plot">×</button></div></header><div class="plot-tools"><select></select><button class="add-signal">Add signal</button></div><div class="plot-signals"></div><div class="plot-chart"></div><div class="plot-facts"></div>';
  card.querySelector("strong").textContent = title;
  card.querySelector("small").textContent = detail;
  card.querySelector(".plot-facts").replaceChildren(...Object.entries(gains ?? {}).map(([key, value]) => {
    const fact = document.createElement("span");
    fact.innerHTML = "<b></b><i></i>";
    fact.querySelector("b").textContent = key;
    fact.querySelector("i").textContent = value;
    return fact;
  }));
  const select = card.querySelector(".plot-tools select");
  state.replay.signals.forEach((signal) => select.add(new Option(signal, signal)));
  $("#plots").append(card);
  const chart = echarts.init(card.querySelector(".plot-chart"));
  state.charts.push(chart);
  // the plot grid reflows as cards come and go; the canvas only follows if told
  const observer = new ResizeObserver(() => chart.resize());
  observer.observe(card.querySelector(".plot-chart"));
  if (row) card.dataset.row = `${row.dataset.plotKey ??= String(++plotKeys)}`;
  const discard = () => {
    observer.disconnect();
    chart.dispose();
    state.charts = state.charts.filter((item) => item !== chart);
    card.remove();
  };
  card.querySelector(".expand-plot").onclick = () =>
    document.fullscreenElement ? document.exitFullscreen() : card.requestFullscreen();
  function zoom(factor) {
    const [{ start = 0, end = 100 } = {}] = chart.getOption().dataZoom ?? [];
    const middle = (start + end) / 2;
    const half = Math.min(50, (end - start) * factor / 2);
    chart.dispatchAction({
      type: "dataZoom",
      start: Math.max(0, middle - half),
      end: Math.min(100, middle + half),
    });
  }
  const exporter = card.querySelector(".export-plot");
  exporter.ontoggle = () => {
    if (exporter.open) exporter.querySelector(".export-menu button").focus();
  };
  exporter.onkeydown = (event) => {
    if (event.key !== "Escape") return;
    exporter.open = false;
    exporter.querySelector("summary").focus();
  };
  exporter.addEventListener("focusout", (event) => {
    if (!exporter.contains(event.relatedTarget)) exporter.open = false;
  });
  exporter.querySelectorAll(".export-menu button").forEach((option) => {
    option.onclick = () => {
      exporter.open = false;
      exportChart(chart, title.replace(/[^\w.-]+/g, "_"), option.value);
    };
  });
  card.querySelector(".remove-plot").onclick = () => {
    discard();
    if (row && !$(`#plots [data-row="${card.dataset.row}"]`)) delete row.dataset.plotted;
  };
  card.querySelector(".add-signal").onclick = () => {
    if (signals.includes(select.value)) return;
    discard();
    addPlot([...signals, select.value], title, detail, { row, constraint, gains });
  };
  if (!signals.length) {
    chart.setOption({ graphic: { type: "text", left: "center", top: "middle", style: { text: "Choose a signal above", fill: "#73777d" } } });
    return;
  }
  const palette = ["#e07a5f", "#79c6a5", "#9da9c7", "#f0c36a"];
  card.querySelector(".plot-signals").replaceChildren(...signals.map((signal, index) => {
    const chip = document.createElement("button");
    chip.className = "plot-signal";
    chip.style.setProperty("--series", palette[index % palette.length]);
    chip.textContent = signal;
    chip.title = "Remove this signal";
    chip.onclick = () => {
      discard();
      addPlot(signals.filter((item) => item !== signal), title, detail, { row, constraint, gains });
    };
    return chip;
  }));
  chart.showLoading("default", { text: "Loading recorded data…" });
  const query = new URLSearchParams({ path: state.runPath });
  signals.forEach((signal) => query.append("signal", signal));
  // sample inside the motion's window, or a short motion falls between two samples
  (constraint?.window ?? []).forEach((bound) => query.append("window", bound));
  api(`/api/plot?${query}`).then((data) => {
    chart.hideLoading();
    chart.setOption({
    animation: false,
    color: ["#e07a5f", "#79c6a5", "#9da9c7", "#f0c36a"],
    grid: { left: 62, right: 22, top: 34, bottom: 52 },
    dataZoom: [{ type: "inside", filterMode: "none" }],
    toolbox: {
      right: 8, top: 2, itemSize: 15, itemGap: 12,
      iconStyle: { color: "none", borderColor: "#73777d", borderWidth: 1.1 },
      emphasis: { iconStyle: { borderColor: "#e07a5f" } },
      feature: {
        myZoomIn: {
          show: true, title: "Zoom in", icon: MAGNIFIER_IN,
          onclick: () => zoom(0.5),
        },
        myZoomOut: {
          show: true, title: "Zoom out", icon: MAGNIFIER_OUT,
          onclick: () => zoom(2),
        },
        myZoomReset: {
          show: true, title: "Reset zoom", icon: RESET_ARROW,
          onclick: () => chart.dispatchAction({ type: "dataZoom", start: 0, end: 100 }),
        },
      },
    },
    tooltip: { trigger: "axis", backgroundColor: "#202327", borderColor: "#383d45", textStyle: { color: "#f1eee7" } },
    legend: { show: false },
    xAxis: {
      type: "value", scale: true, name: "frame", nameLocation: "middle", nameGap: 26,
      nameTextStyle: { color: "#73777d" },
      min: constraint?.window?.[0], max: constraint?.window?.[1],
      axisLabel: { color: "#73777d" }, axisLine: { lineStyle: { color: "#73777d" } },
      splitLine: { lineStyle: { color: "#383d45" } },
    },
    yAxis: {
      type: "value", axisLabel: { color: "#73777d" }, axisLine: { lineStyle: { color: "#73777d" } },
      splitLine: { lineStyle: { color: "#383d45" } },
    },
    series: [
      ...signals.map((signal, index) => ({
        name: signal,
        type: "line",
        showSymbol: false,
        data: data.signals[signal]
          .map((value, point) => value == null ? null : [(data.first_frame ?? 0) + point * data.sample_step, value])
          .filter(Boolean),
        lineStyle: { width: 1.5 },
        ...(index === 0 ? setpointBands(constraint) : {}),
      })),
      { name: "__cursor", type: "line", data: [], silent: true },
    ],
    });
    chart.cursorIndex = signals.length;
    chart.setOption(cursorOption(state.frame, chart.cursorIndex));
  }).catch((error) => chart.showLoading("default", { text: error.message }));
}

async function loadSources(refresh = false) {
  const request = ++state.listRequest;
  if (refresh || !state.cache.sources) {
    const [sources, roots] = await Promise.all([api("/api/sources"), api("/api/roots")]);
    state.cache.sources = { sources, roots };
  }
  if (request !== state.listRequest || state.tab !== "sources") return;
  $("#list-title").textContent = "SOURCES";
  $("#storage").textContent = "";
  $("#generation-root").hidden = false;
  const { sources, roots } = state.cache.sources;
  state.roots = roots;
  $("#generation-root").value = roots.sources;
  const root = { folders: new Map(), files: [] };
  sources.forEach((source) => {
    const parts = source.split("/");
    const file = parts.pop();
    let node = root;
    parts.forEach((part) => {
      node.folders.set(part, node.folders.get(part) ?? { folders: new Map(), files: [] });
      node = node.folders.get(part);
    });
    node.files.push({ file, source });
  });
  $("#browser").replaceChildren(...renderSourceNode(root, "", 0, true));
  filterSources($("#search").value);
}

function renderSourceNode(node, name, depth, isRoot = false) {
  const children = [];
  if (!isRoot) {
    const branch = document.createElement("details");
    branch.className = "source-node";
    branch.open = true;
    branch.style.setProperty("--depth", depth);
    const summary = document.createElement("summary");
    summary.textContent = name;
    branch.append(summary);
    branch.append(...renderSourceNode(node, "", depth + 1, true));
    return [branch];
  }
  [...node.folders].sort(([left], [right]) => left.localeCompare(right)).forEach(([folder, child]) => {
    children.push(...renderSourceNode(child, folder, depth));
  });
  node.files.sort((left, right) => left.file.localeCompare(right.file)).forEach(({ file, source }) => {
    const leaf = document.createElement("div");
    leaf.className = "source-leaf";
    leaf.style.setProperty("--depth", depth);
    const item = listItem(file, null, () => { $("#status").textContent = `${state.roots.sources}/${source}`; openSource(source); }, source);
    item.onmouseenter = () => {
      const style = getComputedStyle(item);
      const available = item.clientWidth - parseFloat(style.paddingLeft) - parseFloat(style.paddingRight);
      const distance = Math.max(0, item.querySelector(".item-name").scrollWidth - available);
      item.style.setProperty("--scroll-distance", `${distance}px`);
      item.classList.toggle("is-scrolling", distance > 0);
    };
    item.onmouseleave = () => item.classList.remove("is-scrolling");
    const copy = document.createElement("button");
    copy.className = "copy-path";
    copy.textContent = "⧉";
    copy.title = "Copy full path";
    copy.onclick = async () => {
      await copyText(`${state.roots.sources}/${source}`);
      copy.textContent = "✓";
      setTimeout(() => { copy.textContent = "⧉"; }, 900);
    };
    leaf.append(item, copy);
    children.push(leaf);
  });
  return children;
}

async function openSource(source) {
  state.viewing = source;
  const { text, editors, terminal } = await api(`/api/source?path=${encodeURIComponent(source)}`);
  if (state.viewing !== source) return;
  $("#content").innerHTML = `<div class="viewer"><div class="viewer-heading"><div><div class="eyebrow">SOURCE</div><h1></h1><p class="path"></p></div><div class="open-with"><select id="editor-choice"></select><button id="open-editor">Open</button></div></div><pre id="source-text"></pre></div>`;
  $(".viewer h1").textContent = source.split("/").pop();
  $(".viewer .path").textContent = `${state.roots.sources}/${source}`;
  $("#source-text").replaceChildren(...text.replace(/\n$/, "").split("\n").map((line) => {
    const row = document.createElement("span");
    row.className = "code-line";
    row.textContent = line;
    return row;
  }));
  const options = [...editors];
  if (terminal) options.push("terminal");
  const choice = $("#editor-choice");
  choice.replaceChildren(...options.map((name) => new Option(name === "terminal" ? `terminal (${terminal}) · folder` : name, name)));
  const remembered = localStorage.getItem("motion-spec.editor");
  if (options.includes(remembered)) choice.value = remembered;
  choice.onchange = () => localStorage.setItem("motion-spec.editor", choice.value);
  $("#open-editor").onclick = async () => {
    try {
      if (choice.value === "terminal") {
        const { opened } = await post("/api/terminal", { path: source });
        snack(`${terminal} at ${opened}`);
      } else {
        await post("/api/open", { path: source, editor: choice.value });
        snack(`Opened in ${choice.value}`);
      }
    } catch (error) { snack(error.message); }
  };
}

function filterSources(value) {
  const query = value.toLowerCase();
  document.querySelectorAll(".source-leaf").forEach((item) => {
    item.hidden = !item.textContent.toLowerCase().includes(query);
  });
  [...document.querySelectorAll(".source-node")].reverse().forEach((node) => {
    const visible = [...node.querySelectorAll(".source-leaf")].some((item) => !item.hidden);
    node.hidden = !visible;
    if (query && visible) node.open = true;
  });
}

document.querySelectorAll("nav button[data-tab]").forEach((button) => {
  button.onclick = () => {
    setTab(button.dataset.tab);
    if (state.tab === "notebook") return loadNotebook().catch(showError);
    (state.tab === "logs" ? loadGenerations : loadSources)().catch(showError);
  };
});

async function loadNotebook(url = null) {
  stopPlayback();
  $("#list-title").textContent = "NOTEBOOK";
  $("#browser").replaceChildren();
  $("#content").innerHTML = '<div class="notebook"><iframe title="JupyterLab"></iframe></div>';
  $("#status").textContent = "Starting JupyterLab…";
  try {
    const lab = url ? { url } : await api("/api/jupyter");
    $(".notebook iframe").src = lab.url;
    $("#status").textContent = lab.url.split("?")[0];
  } catch (error) {
    // a missing JupyterLab is a thing to install, not a page that failed to load
    $("#status").textContent = "";
    $("#content").innerHTML = '<div class="empty"><span>NOTEBOOK</span><h1></h1><p></p></div>';
    $(".empty h1").textContent = "JupyterLab is not available.";
    $(".empty p").textContent = error.message;
  }
}

async function openNotebook(runPath) {
  setTab("notebook");
  const { url } = await post("/api/notebook", { path: runPath });
  await loadNotebook(url);
}
const lifecycleEvents = new EventSource("/api/events");
lifecycleEvents.addEventListener("lifecycle", () => {
  state.cache.generations = null;
  if (state.tab === "logs") loadGenerations().catch(showError);
});
$("#refresh").onclick = () => {
  const load = state.tab === "logs" ? loadGenerations : loadSources;
  load(true).catch(showError);
};
$("#delete-selected").onclick = async () => {
  if (!state.selected.size) return;
  const ok = await askConfirm({
    message: `Delete ${state.selected.size} selected generation${state.selected.size === 1 ? "" : "s"} or run${state.selected.size === 1 ? "" : "s"}? This cannot be undone.`,
    confirmLabel: "Delete",
  });
  if (!ok) return;
  const response = await fetch("/api/delete", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ paths: [...state.selected] }),
  });
  const data = await response.json();
  if (!response.ok) return showError(Error(data.error));
  const parts = [`${data.deleted} item${data.deleted === 1 ? "" : "s"}`];
  if (data.folders) parts.push(`${data.folders} empty folder${data.folders === 1 ? "" : "s"}`);
  snack(`Moved ${parts.join(" and ")} to Trash`);
  // what was open may be what was just trashed
  const gone = [...state.selected].some((path) =>
    [state.generationPath, state.runPath].some((open) => open === path || open?.startsWith(`${path}/`)));
  state.selected.clear();
  $("#selection-actions").hidden = true;
  state.cache = {};
  if (gone) return goHome();
  loadGenerations(true).catch(showError);
};
$("#clear-selection").onclick = () => {
  state.selected.clear();
  document.querySelectorAll(".picked").forEach((item) => item.classList.remove("picked"));
  $("#selection-actions").hidden = true;
};
document.onkeydown = (event) => {
  if (event.key === "Escape" && state.selected.size) $("#clear-selection").click();
};
$("#search").oninput = (event) => {
  if (state.tab === "sources") filterSources(event.target.value);
  else filterGenerations(event.target.value);
};
async function updateRoot(path) {
  const kind = state.tab === "logs" ? "logs" : "sources";
  state.roots = await post("/api/roots", { kind, path });
  state.cache = {};
  (kind === "logs" ? loadGenerations : loadSources)(true).catch(showError);
}

$("#generation-root").onchange = (event) => updateRoot(event.target.value).catch(showError);
$("#pick-root").onclick = async () => {
  const kind = state.tab === "logs" ? "logs" : "sources";
  state.roots = await api(`/api/pick-root?kind=${kind}`);
  state.cache = {};
  (kind === "logs" ? loadGenerations : loadSources)(true).catch(showError);
};
$("#sidebar-toggle").onclick = () => {
  document.body.classList.toggle("sidebar-collapsed");
  const collapsed = document.body.classList.contains("sidebar-collapsed");
  localStorage.setItem("motion-spec.sidebar-collapsed", collapsed);
  $("#sidebar-toggle").textContent = collapsed ? "☰" : "×";
  $("#sidebar-toggle").title = collapsed ? "Expand navigation" : "Collapse navigation";
};
if (document.body.classList.contains("sidebar-collapsed")) {
  $("#sidebar-toggle").textContent = "☰";
  $("#sidebar-toggle").title = "Expand navigation";
}
function goHome() {
  stopPlayback();
  state.generationPath = null;
  state.runPath = null;
  setTab("logs");
  history.replaceState(null, "", `${location.pathname}#tab=logs`);
  $("#status").textContent = "Choose a generation";
  $("#content").innerHTML = '<div class="empty"><span>REPLAY / 01</span><h1>Choose a generation.</h1><p>Its build metadata and recorded runs will appear here.</p></div>';
  return loadGenerations().catch(showError);
}

$("#home").onclick = goHome;

function setView(kind, path) {
  const view = new URLSearchParams(location.hash.slice(1));
  const unchanged = view.get(kind) === path && !view.has(kind === "run" ? "generation" : "run");
  view.delete("run");
  view.delete("generation");
  view.set(kind, path);
  view.set("tab", state.tab);
  if (kind !== "run") view.delete("panel");
  history[unchanged ? "replaceState" : "pushState"](null, "", `#${view}`);
}

function setTab(tab, push = true) {
  state.tab = tab;
  document.querySelectorAll("nav button[data-tab]").forEach((button) => {
    button.classList.toggle("active", button.dataset.tab === tab);
  });
  const view = new URLSearchParams(location.hash.slice(1));
  view.set("tab", tab);
  history[push ? "pushState" : "replaceState"](null, "", `#${view}`);
}

function askConfirm({ message, confirmLabel = "Confirm" }) {
  return new Promise((resolve) => {
    const dialog = document.createElement("dialog");
    dialog.className = "ask";
    dialog.innerHTML = '<p></p><div class="ask-actions"><button value="no">Cancel</button>'
      + '<button value="yes" class="ask-yes"></button></div>';
    dialog.querySelector("p").textContent = message;
    dialog.querySelector(".ask-yes").textContent = confirmLabel;
    dialog.querySelectorAll("button").forEach((button) => {
      button.onclick = () => dialog.close(button.value);
    });
    dialog.onclose = () => {
      dialog.remove();
      resolve(dialog.returnValue === "yes");
    };
    dialog.onkeydown = (event) => {
      if (event.key === "Enter") {
        event.preventDefault();
        dialog.close("yes");
      }
    };
    document.body.append(dialog);
    dialog.showModal();
    dialog.querySelector(".ask-yes").focus();
  });
}

function showError(error) {
  $("#content").innerHTML = `<div class="empty"><span>ERROR</span><h1>Could not load data.</h1><p>${error.message}</p></div>`;
}

function loadLocation() {
  const view = new URLSearchParams(location.hash.slice(1));
  state.generationPath = view.get("generation") ?? view.get("run")?.split("/runs/")[0] ?? null;
  setTab(view.get("tab") ?? "logs", false);
  const loadSidebar = state.tab === "logs" ? loadGenerations : loadSources;
  if (view.has("run")) loadSidebar().then(() => loadReplay(view.get("run"))).catch(showError);
  else if (view.has("generation")) loadSidebar().then(() => selectGeneration(view.get("generation"))).catch(showError);
  else loadSidebar().catch(showError);
}

window.onpopstate = loadLocation;
loadLocation();

const asideWidth = (value) => {
  const width = Math.min(720, Math.max(200, value));
  document.documentElement.style.setProperty("--aside", `${width}px`);
  localStorage.setItem("motion-spec.aside", width);
};

asideWidth(Number(localStorage.getItem("motion-spec.aside")) || 290);
$("#aside-resize").onpointerdown = (event) => {
  event.preventDefault();
  document.body.classList.add("resizing");
  const move = (moved) => asideWidth(moved.clientX);
  const up = () => {
    document.body.classList.remove("resizing");
    removeEventListener("pointermove", move);
    removeEventListener("pointerup", up);
  };
  addEventListener("pointermove", move);
  addEventListener("pointerup", up);
};
$("#aside-resize").ondblclick = () => asideWidth(290);
