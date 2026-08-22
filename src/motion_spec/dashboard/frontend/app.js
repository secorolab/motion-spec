// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

const MAGNIFIER = "M8.2,2 A6,6 0 1,0 8.2,14.2 A6,6 0 1,0 8.2,2 M12.7,12.7 L17.4,17.4";
const MAGNIFIER_IN = `path://${MAGNIFIER} M5.4,8.1 L11,8.1 M8.2,5.3 L8.2,10.9`;
const MAGNIFIER_OUT = `path://${MAGNIFIER} M5.4,8.1 L11,8.1`;
const RESET_ARROW = "path://M15.4,9.6 A6.2,6.2 0 1,1 9.2,3.4 M9.2,0.7 L9.2,6.1 M6.5,3.4 L11.9,3.4";
let plotKeys = 0;
const state = { replay: null, queries: [], query: -1, anchor: null, runPath: null, generationPath: null, tab: "logs", frame: 0, charts: [], selected: new Set(), timer: null, roots: {}, cache: {}, listRequest: 0, live: null, speed: 1, consoleWatch: null, livePlots: new Map(), pendingSignals: new Set(), liveBuffer: new Map(), activeMotion: null, autoPlot: null };
// A long run would grow a live series without bound; the finished-run reload replaces it anyway.
const LIVE_POINT_CAP = 20000;
// What a live chart renders: enough for the recent story at constant redraw cost.
const LIVE_WINDOW = 4000;
// What a chart that is still loading its history can be handed when it registers: enough for
// the second or two an /api/plot read takes, not a second copy of the run.
const LIVE_BUFFER_CAP = 5000;
// A 500 ms poll drawn in five steps: growth the eye follows, at a fifth of the redraws a
// 100 ms poll would cost.
const LIVE_DRIP_SLICES = 5;
const LIVE_DRIP_MS = 50;
const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];
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
      stampText(generation.created),
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

function bindRunAgain(page, path, cameras, simulated) {
  const bar = page.querySelector(".run-bar");
  const state_ = bar.querySelector(".run-state");
  const start = bar.querySelector(".run-start");
  const failed = page.querySelector(".run-console");
  // Each option is a choice between two named states, not a flag to guess the meaning of.
  bar.querySelectorAll(".run-choice").forEach((choice) => {
    choice.querySelectorAll("button").forEach((option) => {
      option.onclick = () => choice.querySelectorAll("button").forEach((other) =>
        other.setAttribute("aria-pressed", other === option));
    });
  });
  // Hardware takes neither: the CLI rejects a headless real run, and recording one comes later.
  const headless = bar.querySelector('.run-choice[data-option="headless"]');
  headless.hidden = !simulated;
  // Only a run with no window to pace it has a speed to choose; a GUI run is realtime already.
  const speed = bar.querySelector(".run-speed");
  const showSpeed = () => {
    speed.hidden = !simulated
      || headless.querySelector('button[aria-pressed="true"]')?.dataset.value !== "true";
  };
  headless.querySelectorAll("button").forEach((option) => option.addEventListener("click", showSpeed));
  showSpeed();
  let chosen = () => [];
  if (simulated) {
    // The cameras the model declares, plus the standard view, which needs no declaring.
    // A list that opens: a scene can declare more cameras than a bar has room for.
    const record = bar.querySelector(".run-record");
    const offered = [{ id: "default", title: "standard camera view" }, ...cameras];
    const menu = document.createElement("details");
    menu.className = "picker camera-menu";
    menu.innerHTML = '<summary></summary><div class="picker-panel"></div>';
    const summary = menu.querySelector("summary");
    chosen = () => [...menu.querySelectorAll('.run-camera[aria-pressed="true"]')]
      .filter((chip) => !chip.hidden).map((chip) => chip.dataset.camera);
    const label = () => {
      const available = [...menu.querySelectorAll(".run-camera")].filter((chip) => !chip.hidden);
      const picked = chosen().length;
      summary.textContent = picked ? `${picked} camera${picked === 1 ? "" : "s"}` : "none";
      // Nothing to record from is not an empty menu to open: the whole control goes away.
      record.hidden = !available.length;
      if (!available.length) menu.open = false;
    };
    menu.querySelector(".picker-panel").append(...offered.map((camera) => {
      const chip = document.createElement("button");
      chip.className = "run-camera";
      chip.dataset.camera = camera.id;
      chip.setAttribute("aria-pressed", "false");
      chip.textContent = camera.id;
      chip.title = camera.width ? `${camera.width}×${camera.height}` : camera.title ?? camera.id;
      chip.onclick = () => {
        chip.setAttribute("aria-pressed", chip.getAttribute("aria-pressed") !== "true");
        label();
      };
      return chip;
    }));
    record.append(menu);
    menu.onkeydown = (event) => {
      if (event.key === "Escape") menu.open = false;
    };
    // Same as the other menus on this page: clicking away closes it.
    document.addEventListener("click", (event) => {
      if (menu.open && !menu.contains(event.target)) menu.open = false;
    });
    label();
  }

  // Every named choice, hardware included: only the display, the speed and the cameras are a
  // simulator's alone, and the server drops those for a real run. Whether to log is not.
  const options = () => ({
    ...Object.fromEntries(
      [...bar.querySelectorAll(".run-choice[data-option]")].map((choice) => [
        choice.dataset.option,
        choice.querySelector('button[aria-pressed="true"]')?.dataset.value === "true",
      ]),
    ),
    ...(simulated ? { cameras: chosen() } : {}),
  });
  // While it runs the page cannot say more than the runner does; watch until it stops, then
  // put the run it made in the list.
  let sawRunning = false;
  const check = async () => {
    const status = await api(`/api/run?path=${encodeURIComponent(path)}`).catch(() => null);
    if (!status) return;
    if (status.running) {
      sawRunning = true;
      start.disabled = true;
      failed.hidden = true;
      state_.textContent = status.pid ? `running · pid ${status.pid}` : "running";
      return;
    }
    clearInterval(state.runWatch);
    start.disabled = false;
    state_.textContent = sawRunning
      ? (status.exit_code ? `exited ${status.exit_code}` : "run finished")
      : "";
    // A run that failed says why here rather than sending the reader to a file.
    if (sawRunning && status.exit_code) {
      consoleExcerpt(path).then((pre) => {
        failed.replaceChildren(...pre.childNodes);
        failed.hidden = !failed.textContent;
      });
    }
    api(`/api/runs?path=${encodeURIComponent(path)}`).then(state.generation?.setRuns);
    // Only an ending this page watched happen is news, and the run's own page may have said
    // it already, seconds earlier.
    if (sawRunning && !state.announced) {
      snack(status.exit_code ? `run failed (${status.exit_code})` : "run finished");
    }
    if (sawRunning) state.announced = true;
    sawRunning = false;
  };
  page.querySelector(".run-bar").refreshRun = check;
  const watch = () => {
    clearInterval(state.runWatch);
    // the run directory exists as it starts, so look once now
    check();
    state.runWatch = setInterval(check, 2000);
  };
  start.onclick = async () => {
    start.disabled = true;
    state_.textContent = "starting…";
    state.announced = false;   // this run has not reported its ending yet
    try {
      const started = await post("/api/run", { path, options: options() });
      state_.textContent = started.recording?.length
        ? `running · pid ${started.pid} · recording ${started.recording.join(", ")}`
        : `running · pid ${started.pid}`;
      watch();
      // The server named the run before anything is on disk: open its page now and let it
      // wait for the log, so nothing of the run happens off-screen.
      await openPendingRun(started.run);
    } catch (error) {
      start.disabled = false;
      state_.textContent = error.message;
    }
  };
  api(`/api/run?path=${encodeURIComponent(path)}`).then((status) => {
    if (!status.running) return;
    sawRunning = true;
    start.disabled = true;
    state_.textContent = `running · pid ${status.pid}`;
    watch();
  }).catch(() => {});
}

// A hardware generation names every endpoint it drives, so the wire can be tested here rather
// than failing into the run console. Only when asked: a probe is a connection attempt.
function bindDevices(page, path) {
  const panel = page.querySelector(".devices");
  const rows = panel.querySelector(".device-rows");
  const state_ = panel.querySelector(".devices-state");
  panel.hidden = false;
  panel.querySelector(".devices-test").onclick = async () => {
    state_.textContent = "testing…";
    rows.replaceChildren();
    try {
      const report = await api(`/api/devices?path=${encodeURIComponent(path)}`);
      state_.textContent = report.config ? "" : "this generation archived no robot.toml";
      rows.replaceChildren(...report.devices.flatMap(deviceRows));
    } catch (error) {
      state_.textContent = error.message;
    }
  };
}

// One row per thing actually knocked on: a network device has a row per port, a serial one its
// device node. Host, port and path only -- what the config holds to log in with stays there.
function deviceRows(device) {
  const targets = device.kind === "network"
    ? device.ports.map((probe) => ({ ...probe, where: `${device.host}:${probe.port}` }))
    : [{ ok: device.ok, detail: device.detail, where: device.device }];
  if (!targets.length) targets.push({ ok: null, detail: "no port to test", where: device.host });
  return targets.map((target) => {
    const row = document.createElement("div");
    row.className = "device";
    row.dataset.ok = String(target.ok);
    row.innerHTML = '<span class="device-dot"></span><strong></strong>'
      + '<span class="device-where"></span><span class="device-detail"></span>';
    row.querySelector("strong").textContent = device.name;
    row.querySelector(".device-where").textContent = target.where;
    row.querySelector(".device-detail").textContent = target.ok ? "" : target.detail ?? "";
    return row;
  });
}

async function selectGeneration(path) {
  state.anchor = path;
  clearInterval(state.liveWatch);   // following a live run belongs to the run page that left
  clearInterval(state.consoleWatch);
  state.runPath = null;
  state.live = state.following = null;
  state.livePlots.clear();
  state.pendingSignals.clear();
  state.liveBuffer.clear();
  state.activeMotion = null;
  setView("generation", path);
  state.generationPath = path;
  // Coming back to a generation already built: put it back and refresh what can have changed,
  // rather than tearing the page down and laying it out again around the same facts.
  if (state.generation?.path === path) {
    $("#content").replaceChildren(state.generation.node);
    $("#status").textContent = state.generation.folder;
    highlightGeneration();
    // This page was put aside mid-run; what it says about that run is only what was true then.
    $(".run-bar")?.refreshRun?.();
    return api(`/api/runs?path=${encodeURIComponent(path)}`).then(state.generation.setRuns);
  }
  const [generation, runs] = await Promise.all([
    api(`/api/generation?path=${encodeURIComponent(path)}`),
    api(`/api/runs?path=${encodeURIComponent(path)}`),
  ]);
  $("#status").textContent = generation.path;
  highlightGeneration();

  const page = $("#generation-template").content.cloneNode(true);
  page.querySelector("h1").textContent = generation.spec_name ?? generation.name;
  page.querySelector(".path").textContent = generation.folder;
  page.querySelector(".copy-generation-path").onclick = () => copyText(generation.folder);
  page.querySelector(".generation-description").textContent = generation.description ?? "";
  page.querySelector(".facts").innerHTML = [
    fact("Source", generation.source),
    fact("Generated", stampText(generation.created)),
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
  let rows = runs;
  const renderRuns = () => runList.replaceChildren(...rows.slice(runPage * 10, runPage * 10 + 10).map((run, index) => {
    const row = document.createElement("div");
    row.className = "run";
    row.dataset.path = run.path;
    row.classList.toggle("picked", state.selected.has(run.path));
    const status = run.status ?? (run.complete ? "COMPLETED" : "INCOMPLETE");
    row.innerHTML = `<span>${runPage * 10 + index + 1}</span><strong>${run.id}</strong><span>${stampText(run.started)}</span><span>${run.duration_s.toFixed(2)} s</span><span>${(run.written_frames ?? 0).toLocaleString()}</span><span class="badge badge-${status.toLowerCase()}">${status}</span>`;
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
  bindRunAgain(page, path, generation.cameras ?? [], generation.simulated);
  if (!generation.simulated) bindDevices(page, path);
  $("#content").replaceChildren(page);
  state.generation = {
    path,
    folder: generation.folder,
    node: $("#content .generation"),
    setRuns: (fresh) => {
      rows = fresh;
      runPage = Math.min(runPage, Math.max(0, Math.ceil(fresh.length / 10) - 1));
      renderRuns();
    },
  };
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

// A run that was just started: its page, open before its log exists. The server names the
// run as it starts it and /api/replay answers from the generation's serialized contract, so
// the page is normally complete before the runtime is even up. Generations from before that
// contract file existed fall back to a holding card until the log begins.
async function openPendingRun(runPath) {
  // A run this page started plots itself -- live as motions enter, or all at once when a
  // headless run outpaces the page and only its archive is left to show.
  state.autoPlot = runPath;
  if (await loadReplay(runPath).then(() => true).catch(() => false)) return;
  state.anchor = runPath;
  stopPlayback();
  setView("run", runPath);
  state.runPath = runPath;
  state.generationPath = runPath.split("/runs/")[0];
  state.live = state.following = null;
  highlightGeneration();
  $("#status").textContent = "";
  $("#content").innerHTML =
    '<div class="empty"><div class="spinner"></div><h1>Run starting…</h1>'
    + "<p>Waiting for the first frames of the log.</p></div>";
  const hop = setInterval(async () => {
    if (state.runPath !== runPath) return clearInterval(hop);   // the user went elsewhere
    if (await loadReplay(runPath).then(() => true).catch(() => false)) return clearInterval(hop);
    const status = await api(`/api/run?path=${encodeURIComponent(state.generationPath)}`)
      .catch(() => null);
    if (status && !status.busy) {
      // over without ever writing a log: the runner's own words are all there is to show
      clearInterval(hop);
      await showRunWithoutLog(state.generationPath, runPath, status);
    }
  }, 300);
}

async function loadReplay(path) {
  state.anchor = path;
  stopPlayback();
  setView("run", path);
  state.replay = await api(`/api/replay?path=${encodeURIComponent(path)}`);
  state.charts.forEach((chart) => chart.dispose());
  state.charts = [];
  state.livePlots.clear();
  state.pendingSignals.clear();
  state.liveBuffer.clear();
  state.activeMotion = null;
  $("#status").textContent = "";
  $("#content").innerHTML = replayShell(path);
  state.runPath = path;
  state.generationPath = state.replay.generation;
  state.live = null;
  highlightGeneration();
  state.frame = 0;
  const timeline = $(".timeline");
  timeline.max = Math.max(0, state.replay.frames - 1);
  timeline.disabled = false;

  // gate id -> the name the source spells it: a row carries the name, not the gate it ran under
  state.motionOf = state.replay.motion_names ?? {};
  populateConstraints();
  // The archive of a run this page started: reopen the cards that were open when it ended --
  // or, when none were (a run faster than the page), the last motion that ran. Never the
  // whole run's card set: eighty charts in one page is what made live pages crawl.
  if (state.autoPlot === path && !state.replay.pending && livePlotsOn()) {
    state.autoPlot = null;
    const keys = state.reopenRows ?? [];
    let lastMotion = state.reopenMotion;
    state.reopenRows = state.reopenMotion = null;
    if (!keys.length && !lastMotion) {
      lastMotion = state.replay.constraints
        .filter((constraint) => constraint.window)
        .sort((a, b) => b.window[1] - a.window[1])[0]?.motion;
    }
    // Each row is a full-log /api/plot read: click them apart so several never land in one
    // frame. Detached on purpose -- the page is usable while the cards fill in.
    (async () => {
      const rows = $$("#constraints .constraint").filter((row) => {
        if (!row.dataset.plottable || row.dataset.plotted || row.dataset.idle) return false;
        const key = `${row.dataset.motion}/${row.querySelector("strong")?.textContent}`;
        return keys.length ? keys.includes(key) : row.dataset.motion === lastMotion;
      });
      for (const row of rows) {
        if (state.runPath !== path) return;   // the reader moved on
        row.click();
        await new Promise((resolve) => setTimeout(resolve, 50));
      }
    })();
  }
  $("#plot").onclick = () => addPlot([]);
  $("#notebook").onclick = () => openNotebook(state.runPath).catch((error) => snack(error.message));
  bindLivePlots();
  bindPanels();
  // A side panel that cannot load is not the page failing to load.
  bindSparql().catch((error) => snack(error.message));
  bindTransport();
  setTransportMode();
  showVideos(path, state.replay.videos ?? []);
  followLiveRun(path);
  followConsole(path);
  // Built from the generation's contract before the runtime wrote anything: hold the page
  // until the log begins.
  if (state.replay.pending) settle(true, "waiting for the run to start…");
  $("#back").onclick = () => selectGeneration(state.replay.generation);
  updateReadout();
}

// The panel floats over the page, so the page ends above it rather than behind it.
function reserveVideoSpace() {
  const transport = $(".transport");
  if (transport) {
    document.documentElement.style.setProperty("--transport-h", `${transport.offsetHeight}px`);
  }
  const panel = $(".videos");
  const covered = panel && !panel.hidden && !panel.classList.contains("minimized");
  const style = document.documentElement.style;
  style.setProperty("--video-h", covered ? `${panel.offsetHeight + 16}px` : "0px");
  style.setProperty("--video-w", covered ? `${panel.offsetWidth + 16}px` : "0px");
}

// Waiting on the run: nothing on the page is worth clicking yet.
function settle(waiting, message = "archiving the run…") {
  $(".settling").hidden = !waiting;
  $(".settling span").textContent = message;
  $(".replay").classList.toggle("busy", waiting);
  $(".transport").classList.toggle("busy", waiting);
}

// What the run recorded: one camera large, the rest alongside to swap in.
function showVideos(runPath, cameras) {
  const panel = $(".videos");
  panel.hidden = !cameras.length;
  if (!cameras.length) return reserveVideoSpace();
  const url = (camera) =>
    `/api/video?path=${encodeURIComponent(runPath)}&camera=${encodeURIComponent(camera)}`;
  const main = panel.querySelector("video");
  main.oncontextmenu = (event) => event.preventDefault();
  // The other half of syncVideo's one-seek-at-a-time: a drag's newest target lands here.
  main.onseeked = () => {
    const queued = main._queuedSeek;
    main._queuedSeek = undefined;
    if (queued !== undefined) main.currentTime = queued;
  };
  const minimize = panel.querySelector(".video-min");
  const setMinimized = (small) => {
    panel.classList.toggle("minimized", small);
    minimize.textContent = small ? "\u25a1" : "\u2013";
    minimize.title = small ? "Show the recording" : "Minimize";
    try { localStorage.setItem("motion-spec.video-minimized", String(small)); } catch { /* private */ }
    reserveVideoSpace();
  };
  let small = false;
  try { small = localStorage.getItem("motion-spec.video-minimized") === "true"; } catch { /* private */ }
  minimize.onclick = () => setMinimized(!panel.classList.contains("minimized"));
  setMinimized(small);
  const show = (camera) => {
    main.src = url(camera);
    // A video that is never played paints nothing: put it where the cursor is once it knows
    // how long it is.
    main.onloadedmetadata = () => {
      // Fit the box to the recording rather than the recording to the box: a 3D view with the
      // panels open is portrait, and a landscape box would letterbox it.
      const portrait = main.videoHeight > main.videoWidth;
      main.style.width = portrait ? "auto" : "100%";
      main.style.height = portrait ? "34vh" : "auto";
      reserveVideoSpace();
      syncVideo(true);
    };
    panel.querySelector(".video-name").textContent = camera;
    panel.querySelectorAll(".video-pick").forEach((pick) =>
      pick.setAttribute("aria-pressed", pick.dataset.camera === camera));
  };
  // Nothing to swap between with one camera: the strip would be the same view again.
  const strip = panel.querySelector(".video-strip");
  strip.hidden = cameras.length < 2;
  panel.classList.toggle("one-camera", cameras.length < 2);
  strip.replaceChildren(...cameras.map((camera) => {
    const pick = document.createElement("button");
    pick.className = "video-pick";
    pick.dataset.camera = camera;
    pick.innerHTML = `<video muted playsinline preload="metadata"></video><span>${camera}</span>`;
    const thumb = pick.querySelector("video");
    thumb.src = url(camera);
    // Show something of the run rather than the black frame it opens on.
    thumb.onloadedmetadata = () => { thumb.currentTime = Math.min(1, thumb.duration / 2); };
    pick.onclick = () => show(camera);
    return pick;
  }));
  show(cameras[0]);
  reserveVideoSpace();
}

// Drive the sim this run belongs to; alive is the loop's own ack.
async function simControl(options) {
  const answer = await post("/api/control", { path: state.runPath, options }).catch(() => null);
  state.live = answer?.alive ? answer : null;
  if (state.live) showSpeed(state.live.speed);
  setTransportMode();
  return answer;
}

// The plot drip belongs to the poll that fed it: both timers stop together.
function stopLiveWatch() {
  clearInterval(state.liveWatch);
  clearInterval(state.liveDrip);
  state.dripping = null;
}

// Follow the log as it is written; when it stops, load the finished run.
async function followLiveRun(runPath) {
  stopLiveWatch();
  let settling = false;   // writing stopped, archive not yet written
  let handed = false;     // the finished run is being loaded; nothing may hand over twice
  const poll = async () => {
    if (state.runPath !== runPath) return stopLiveWatch();
    const wasFollowing = state.following;
    // Plots off asks for no series: the run's frames, events and states still come back.
    const signals = livePlotsOn() ? liveSignals() : [];
    const live = await post("/api/live", { path: runPath, signals }).catch(() => null);
    if (!live && state.replay.pending) {
      // named before anything is on disk: hold until the log begins or the runner gives up
      const status = state.replay.generation
        ? await api(`/api/run?path=${encodeURIComponent(state.replay.generation)}`)
            .catch(() => null)
        : null;
      if (status && !status.busy) {
        stopLiveWatch();
        settle(false);
        await showRunWithoutLog(state.replay.generation, runPath, status);
      }
      return;
    }
    // Armed: the loop answers and is paused before its first frame, so no log growth will ever
    // lift the holding overlay. Drop it -- what the run waits for is the play button.
    // No frame has been read yet exactly while the tail has produced no state line at all.
    const armed = Boolean(live?.control?.alive && live.control.paused && !live.events.length);
    if ((live?.writing || armed) && state.replay.pending) {
      state.replay.pending = false;
      settle(false);
    }
    state.following = Boolean(live?.writing);
    state.live = state.following && live.control?.alive ? live.control : null;
    if (!state.following) {
      // A poll still in flight when the handover began would settle a page that has moved on.
      if (handed) return;
      // The run stops writing before it is archived and verified; until that lands the page
      // has nothing final to show, so it waits rather than showing half a run.
      if ((wasFollowing || settling) && live && !live.archived) {
        settling = true;
        return settle(true);
      }
      stopLiveWatch();
      settle(false);
      setTransportMode();
      // read the finished run back, for its health and full frame count
      if (!wasFollowing && !settling) return null;
      settling = false;
      if (!state.announced) snack("run finished");
      state.announced = true;
      handed = true;
      // Carry the open cards across the reload: the archive page reopens exactly these, or
      // falls back to the last motion that ran when the page held none.
      state.reopenRows = $$("#constraints .constraint")
        .filter((row) => row.dataset.plotted)
        .map((row) => `${row.dataset.motion}/${row.querySelector("strong")?.textContent}`);
      state.reopenMotion = state.activeMotion;
      state.autoPlot = runPath;
      return loadReplay(runPath).catch(() => {});
    }
    if (state.live) showSpeed(state.live.speed);
    state.replay.frames = live.frames;
    state.replay.duration = live.duration;
    state.replay.events = live.events;
    $(".timeline").max = Math.max(0, live.frames - 1);
    renderMarkers();
    if (!state.live?.paused) state.frame = live.frames - 1;
    state.frame = Math.min(state.frame, live.frames - 1);
    $(".timeline").value = state.frame;
    updateReadout();
    if (armed) $("#readout").textContent = "armed — press play";
    movePlayhead();
    setTransportMode();
    appendLivePoints(live.plot);
    trackActiveMotion(live.active_motion);
  };
  await poll();
  // One timer for events and plot increments both; a 1 kHz run reads stale at a slower beat.
  // The server answers out of its shm ring in microseconds, so the beat is what the eye wants.
  state.liveWatch = setInterval(poll, 250);
}

// What the poll must ask for: what the registered charts draw, plus what the motion that just
// entered will draw once its cards have loaded. A gated signal is only non-null while its
// motion runs, so asking from the card's first render on would ask after the motion is over.
function liveSignals() {
  const registered = new Set([...state.livePlots.values()].flat());
  state.pendingSignals.forEach((signal) => {
    if (registered.has(signal)) state.pendingSignals.delete(signal);
  });
  return [...registered, ...state.pendingSignals];
}

// Extend each live chart with the increment this poll carried; its history came from /api/plot.
function appendLivePoints(plot) {
  if (!plot?.frames?.length) return;
  const arrived = new Map(Object.entries(plot.series ?? {}).map(([signal, values]) => [
    signal,
    plot.frames.map((frame, point) => [frame, values[point]]).filter(([, value]) => value != null),
  ]));
  // Keep every increment, whether or not a chart carries it yet: a card registers only after
  // its history render, and by then what landed meanwhile is gone from the server.
  arrived.forEach((points, signal) => {
    if (!points.length) return;
    const kept = [...(state.liveBuffer.get(signal) ?? []), ...points];
    state.liveBuffer.set(signal, kept.slice(Math.max(0, kept.length - LIVE_BUFFER_CAP)));
  });
  const batch = new Map();
  state.livePlots.forEach((signals, chart) => {
    // The drip consumes what it is handed, so each chart draws from its own copy.
    const added = signals.map((signal) => [...(arrived.get(signal) ?? [])]);
    // A chart whose signals were all gated off this poll has nothing to redraw.
    if (added.some((points) => points.length)) batch.set(chart, added);
  });
  dripLivePoints(batch);
}

// One poll carries half a second of run, and drawing it in one go reads as a jump. Spread each
// batch over the poll period on one shared timer -- no per-chart timers, no echarts animation.
function dripLivePoints(batch) {
  clearInterval(state.liveDrip);
  // Whatever the last batch had left goes in now: the charts never fall behind the run.
  if (state.dripping) drawLivePoints(state.dripping, 1);
  state.dripping = batch.size ? batch : null;
  if (!state.dripping) return;
  let slices = LIVE_DRIP_SLICES;
  state.liveDrip = setInterval(() => {
    drawLivePoints(batch, slices);
    if (--slices > 0) return;
    clearInterval(state.liveDrip);
    state.dripping = null;
  }, LIVE_DRIP_MS);
}

// One slice of a batch: an even share of what each card has left, appended and dropped.
// Live cards are uPlot adapters -- a canvas chart built for streaming, where a full setData
// costs a millisecond; nothing here touches the replay library.
function drawLivePoints(batch, slices) {
  batch.forEach((added, adapter) => {
    if (adapter.isDisposed?.()) return;   // its card closed while this batch was dripping
    const take = added.map((points) => points.splice(0, Math.ceil(points.length / slices)));
    if (!take.some((points) => points.length)) return;
    adapter.push(take);
  });
}

// A live card: the same chrome as a replay card, drawn by uPlot. Each signal keeps its own
// (x, y) table -- a gated signal misses frames its siblings have -- and uPlot.join aligns
// them per redraw. On the finished-run reload these cards are rebuilt as echarts replay cards.
function addLivePlot(signals, title, detail, { row = null, data = null } = {}) {
  const card = document.createElement("section");
  card.className = "plot-card";
  card.dataset.live = "true";
  card.innerHTML = '<header><div><strong></strong><small></small></div><div class="plot-actions"><button class="remove-plot" title="Remove plot">×</button></div></header><div class="plot-signals"></div><div class="plot-chart"></div>';
  card.querySelector("strong").textContent = title;
  card.querySelector("small").textContent = detail;
  const palette = ["#e07a5f", "#79c6a5", "#9da9c7", "#f0c36a"];
  card.querySelector(".plot-signals").replaceChildren(...signals.map((signal, index) => {
    const chip = document.createElement("span");
    chip.className = "plot-signal";
    chip.style.setProperty("--series", palette[index % palette.length]);
    chip.textContent = signal;
    return chip;
  }));
  $("#plots").append(card);
  if (row) card.dataset.row = `${row.dataset.plotKey ??= String(++plotKeys)}`;
  const holder = card.querySelector(".plot-chart");
  const axis = { stroke: "#73777d", grid: { stroke: "#383d45", width: 1 }, ticks: { stroke: "#383d45" } };
  const u = new uPlot({
    width: Math.max(holder.clientWidth, 320), height: 240,
    legend: { show: false }, cursor: { show: false },
    scales: { x: { time: false } },
    series: [{}, ...signals.map((_signal, index) => ({
      stroke: palette[index % palette.length], width: 1.5, spanGaps: false, points: { show: false },
    }))],
    axes: [axis, axis],
  }, uPlot.join(signals.map(() => [[], []])), holder);
  const tables = signals.map(() => [[], []]);
  signals.forEach((signal, index) => {
    const [xs, ys] = tables[index];
    (data?.signals?.[signal] ?? []).forEach((value, point) => {
      if (value == null) return;
      xs.push((data.first_frame ?? 0) + point * (data.sample_step ?? 1));
      ys.push(value);
    });
    // What landed while this card was opening sits in the buffer, nowhere else.
    const last = xs.at(-1) ?? -Infinity;
    (state.liveBuffer.get(signal) ?? []).forEach(([frame, value]) => {
      if (frame > last) { xs.push(frame); ys.push(value); }
    });
  });
  const adapter = {
    uplot: u,
    push(added) {
      added.forEach((points, index) => {
        const [xs, ys] = tables[index];
        for (const [frame, value] of points) { xs.push(frame); ys.push(value); }
        if (xs.length > LIVE_POINT_CAP) {
          xs.splice(0, xs.length - LIVE_POINT_CAP);
          ys.splice(0, ys.length - LIVE_POINT_CAP);
        }
      });
      u.setData(uPlot.join(tables));
    },
    resize() { u.setSize({ width: Math.max(holder.clientWidth, 320), height: 240 }); },
    isDisposed: () => !card.isConnected,
  };
  u.setData(uPlot.join(tables));
  const observer = new ResizeObserver(() => card.isConnected && adapter.resize());
  observer.observe(holder);
  card.querySelector(".remove-plot").onclick = () => {
    observer.disconnect();
    state.livePlots.delete(adapter);
    u.destroy();
    card.remove();
    if (row && !$(`#plots [data-row="${card.dataset.row}"]`)) delete row.dataset.plotted;
  };
  state.livePlots.set(adapter, signals);
}

// Where the run is now: highlight that motion's block and put every one of its constraints
// on screen -- the constraint itself and its controller/monitor machinery -- as it enters.
function trackActiveMotion(handler) {
  const motion = state.motionOf?.[handler] ?? handler;
  if (!motion || motion === state.activeMotion) return;
  // A motion this page watched enter has nothing behind it: it starts where the live ring
  // already is. Only the motion the page opened onto was running before anyone was looking.
  const watched = state.activeMotion !== null;
  state.activeMotion = motion;
  $$("#constraints .constraint-motion").forEach((heading) =>
    heading.classList.toggle("active-motion", heading.textContent === motion));
  // The panel reads along: the active motion's block scrolls into view as the run enters it.
  const panel = $("#constraints");
  const heading = $$("#constraints .constraint-motion").find((h) => h.textContent === motion);
  if (panel && heading) {
    panel.scrollTo({ top: heading.offsetTop - panel.offsetTop, behavior: "smooth" });
  }
  // Where the run is stays visible with plots off; only the opening of charts is the choice.
  if (!livePlotsOn()) return;
  // Ask for this motion's signals from the next poll on, before a single card exists: the
  // rows below open charts that only register a second later, and by then a short motion is
  // over. The same fields the row click plots, plus what its members plot.
  // A row belongs to this entry when its data flows now: authored here with its monitor's
  // owner elsewhere means the guard already ran during the predecessor -- skip it; a guard
  // authored elsewhere but owned here is exactly what runs now.
  const owns = (row) => (row.dataset.monitorOwner ?? row.dataset.motion) === motion;
  state.replay.constraints
    .filter((constraint) => (constraint.motion ?? "shared") === motion)
    .forEach((constraint) => [
      ...constraint.tracking, ...constraint.error, ...constraint.control,
      ...(constraint.members ?? []).map((member) => member.error),
    ].forEach((signal) => state.pendingSignals.add(signal)));
  $$("#constraints .constraint")
    .filter((row) => owns(row) && row.dataset.kind === "monitored")
    .forEach((row) => (row.plotSignals ?? []).forEach((signal) => state.pendingSignals.add(signal)));
  // Cards follow the run: everything auto-opened that does not belong to THIS entry closes
  // on the transition, so the page carries one motion's charts, not the whole run's. Rows a
  // person clicked are not in autoRows and stay.
  for (const row of state.autoRows ?? []) {
    if (row.isConnected && row.dataset.plotted && !owns(row)) row.click();
  }
  const rows = $$("#constraints .constraint")
    .filter((row) => owns(row) && row.dataset.plottable && !row.dataset.plotted);
  state.autoRows = new Set(rows);
  if (rows.length) openRowsTogether(rows, watched);
}

// Every card of one motion off ONE history read: a dozen rows clicked apart is a dozen
// full-log parses, and each one blocks the poll behind it.
async function openRowsTogether(rows, watched) {
  // Nothing to read for a motion that just entered -- the poll's ring increments are its whole
  // history, and the buffered gap addPlot draws already covers what landed while cards opened.
  let data = { signals: {}, first_frame: 0, sample_step: 1 };
  const signals = [...new Set(rows.flatMap((row) => row.plotSignals ?? []))];
  if (!watched && signals.length) {
    const query = new URLSearchParams({ path: state.runPath });
    signals.forEach((signal) => query.append("signal", signal));
    data = await api(`/api/plot?${query}`).catch(() => data);
  }
  // A motion can bring a dozen cards; each echarts init costs a frame's worth of work, so
  // opening them all at once is a visible hitch. One card per animation frame reads as the
  // panel unfolding instead.
  for (const row of rows) {
    if (!row.isConnected) continue;   // the page moved on mid-unfold
    row.openPlots(data);
    await new Promise((resolve) => requestAnimationFrame(() => resolve()));
  }
}

// Stay on the newest line, unless the reader scrolled up to read an older one.
function appendConsole(pre, text) {
  const follow = pre.scrollHeight - pre.scrollTop - pre.clientHeight < 40;
  // The log is terminal output: render its SGR color escapes as spans (built with
  // textContent, so log content can never become markup) and draw nothing for the rest.
  text = (pre._ansiTail ?? "") + text;
  // An escape split across two polls would render as junk once; hold the tail back instead.
  const cut = text.match(/\x1b(\[[0-9;]*)?$/);
  pre._ansiTail = cut ? cut[0] : "";
  if (cut) text = text.slice(0, -cut[0].length);
  for (const part of text.split(/(\x1b\[[0-9;]*[A-Za-z])/)) {
    if (!part) continue;
    if (part.startsWith("\x1b")) {
      if (part.endsWith("m")) pre._ansiClasses = sgrClasses(pre._ansiClasses ?? [], part);
      continue;   // cursor-movement escapes draw nothing on a page
    }
    if (!pre._ansiClasses?.length) {
      pre.append(document.createTextNode(part));
    } else {
      const span = document.createElement("span");
      span.className = pre._ansiClasses.join(" ");
      span.textContent = part;
      pre.append(span);
    }
  }
  if (follow) pre.scrollTop = pre.scrollHeight;
}

// One SGR escape applied to the classes in force: reset clears, bold and the 16 foreground
// colors map to .ansi-* rules; anything else changes nothing.
function sgrClasses(classes, escape) {
  for (const code of escape.slice(2, -1).split(";").map((n) => Number(n || 0))) {
    if (code === 0) classes = [];
    else if (code === 1) classes = [...classes.filter((c) => c !== "ansi-b"), "ansi-b"];
    else if (code === 22) classes = classes.filter((c) => c !== "ansi-b");
    else if (code === 39) classes = classes.filter((c) => c === "ansi-b");
    else if ((code >= 30 && code <= 37) || (code >= 90 && code <= 97)) {
      classes = [...classes.filter((c) => !c.startsWith("ansi-3") && !c.startsWith("ansi-9")), `ansi-${code}`];
    }
  }
  return classes;
}

// Follow the run's terminal words the way the plots follow its frames.
function followConsole(runPath) {
  clearInterval(state.consoleWatch);
  let offset = 0;
  const pre = $("#console-text");
  const poll = async () => {
    if (state.runPath !== runPath || !pre.isConnected) return clearInterval(state.consoleWatch);
    const slice = await api(`/api/console?path=${encodeURIComponent(runPath)}&offset=${offset}`)
      .catch(() => null);
    if (!slice || !slice.text) return;
    offset = slice.offset;
    appendConsole(pre, slice.text);
  };
  poll();
  state.consoleWatch = setInterval(poll, 1000);
}

// The runner's last words, for a page that would otherwise name a file to go read in a terminal.
async function consoleExcerpt(path, lines = 50) {
  const slice = await api(`/api/console?path=${encodeURIComponent(path)}`).catch(() => null);
  const pre = document.createElement("pre");
  pre.className = "console";
  appendConsole(pre, (slice?.text ?? "").trimEnd().split("\n").slice(-lines).join("\n"));
  return pre;
}

// A run over with no log to open. Told not to write one is not the same as never having run:
// the run's manifest says which, and a log-less run still has its console to show.
async function showRunWithoutLog(generationPath, runPath, status) {
  if (status?.recorded !== false) return showRunNeverStarted(generationPath, status?.exit_code);
  $("#content").innerHTML = '<div class="empty"><span>NO LOG</span>'
    + "<h1>This run recorded no frame log (logs off).</h1>"
    + "<p>No replay and no plots. The console is what the run left behind.</p></div>";
  $(".empty").append(await consoleExcerpt(runPath, 500));
}

// A run that ended before it wrote anything: what the runner printed is all there is to show.
async function showRunNeverStarted(generationPath, exitCode) {
  $("#content").innerHTML = '<div class="empty"><span>ERROR</span>'
    + "<h1>The run never started.</h1><p></p></div>";
  $(".empty p").textContent = `exited ${exitCode ?? "?"}`;
  $(".empty").append(await consoleExcerpt(generationPath));
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
    row.dataset.motion = motion;   // which block a live run's active motion lights up
    // A when-guard's monitor is owned by the predecessor motion: its data flows in THAT
    // motion's window, so plots and auto-opening follow the owner, not the authored block.
    const monitorPrefix = constraint.monitors?.[0]?.split(".")[0];
    const owner = state.replay.monitor_owners?.[monitorPrefix];
    if (owner && owner !== motion) row.dataset.monitorOwner = owner;
    if (!constraint.window) row.dataset.idle = "true";
    line.textContent = constraint.line ? `L${constraint.line}` : "";
    name.textContent = constraint.name;
    expression.textContent = constraint.expression;
    row.append(line, name, expression);
    if (constraint.members?.length) {
      const members = document.createElement("button");
      members.className = "plot-members";
      members.textContent = `plot ${constraint.members.length} members`;
      members.title = "One plot per watched constraint: they are not the same quantity";
      members.onclick = (event) => {
        event.stopPropagation();
        row.dataset.plotted = "true";
        constraint.members.forEach((member) => addPlot(
          [member.error],
          `${constraint.motion ?? "shared"} / ${member.id}`,
          `watched by ${constraint.name}`,
          {
            row,
            // the member's error is judged against its own band, around satisfied
            constraint: {
              ...constraint,
              tolerance: member.tolerance ?? constraint.tolerance,
              setpoints: [{ label: "satisfied", value: 0 }],
            },
          },
        ));
      };
      row.append(members);
    }
    if (constraint.tracking.length || constraint.control.length || constraint.monitors.length) {
      row.dataset.plottable = "true";
      row.title = constraint.window
        ? `Plot this ${constraint.kind} constraint`
        : "This motion never ran in this recording";
      // measured against its setpoint where there is one; otherwise the error against zero
      const tracked = constraint.tracking.length ? constraint.tracking : constraint.error;
      const machinery = constraint.kind === "monitored"
        ? constraint.monitors
        : [...constraint.error, ...constraint.control];
      // What one history read has to cover for this row's cards to draw themselves.
      row.plotSignals = [...tracked, ...machinery];
      // `data` is a prefetched /api/plot answer shared with the other rows of the same motion;
      // without one each card fetches its own, which is what a lone human click wants.
      row.openPlots = (data = null) => {
        row.dataset.plotted = "true";
        const title = `${constraint.motion ?? "shared"} / ${constraint.name}`;
        const between = constraint.between.length ? ` · ${constraint.between.join(" vs ")}` : "";
        const where = constraint.line ? `L${constraint.line}: ` : "";
        const detail = `${where}${constraint.expression ?? constraint.name}${between}`;
        // A when-guard's monitor runs during the predecessor motion; say so on the card.
        const owner = row.dataset.monitorOwner;
        const monitorDetail = owner ? `${detail} · evaluated during ${owner}` : detail;
        if (state.following && livePlotsOn()) {
          // Live: ONE uPlot card per constraint -- its tracked signal and its controller
          // machinery share the frame axis; the monitor keeps its own card, value and
          // satisfied live on a scale of their own. The archive reload rebuilds the full
          // replay card set.
          const combined = [
            ...new Set([...tracked, ...(constraint.kind === "monitored" ? [] : machinery)]),
          ];
          if (combined.length) addLivePlot(combined, title, detail, { row, data });
          if (constraint.kind === "monitored" && constraint.monitors.length) {
            addLivePlot(constraint.monitors, `${title} · monitor`, monitorDetail, { row, data });
          }
          return;
        }
        const bands = constraint.tracking.length
          ? constraint
          : { ...constraint, setpoints: [{ label: "satisfied", value: 0 }] };
        if (tracked.length) addPlot(tracked, `${title} · constraint`, detail, { row, constraint: bands, data });
        if (constraint.kind === "monitored") {
          if (constraint.monitors.length) {
            addPlot(constraint.monitors, `${title} · monitor`, monitorDetail, { row, constraint, data });
          }
          return;
        }
        if (machinery.length) addPlot(machinery, `${title} · controller`, detail, { row, constraint, gains: constraint.gains, data });
      };
      row.onclick = () => {
        if (row.dataset.plotted) {
          $("#plots").querySelectorAll(`[data-row="${row.dataset.plotKey}"] .remove-plot`)
            .forEach((button) => button.click());
          return;
        }
        row.openPlots();
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

function replayShell(path) {
  // The run page's markup, with what the reply fills left blank: the numbers, the timeline's
  // range and the constraint list.
  return `<div class="replay"><div class="replay-heading"><button id="back" title="Back to generation">←</button><h1>${path.split("/").pop()}</h1><div class="replay-tabs"><button data-panel="plots" class="active">Plots</button><button data-panel="sparql">SPARQL</button><button data-panel="console">Console</button></div><span class="eyebrow">RUN</span></div><section id="panel-plots"><div class="constraint-panel"><div class="eyebrow">SOURCE CONSTRAINTS</div><input id="constraint-search" type="search" placeholder="Search .robmot constraints"><div id="constraints" class="constraints"></div></div><div class="chart-controls"><button id="plot">Add empty plot</button><button id="notebook">Open in Jupyter</button><button id="live-plots" title="Plot signals as the run writes them">live plots: on</button></div><div id="plots" class="plots"></div></section><section id="panel-sparql" hidden><div class="sparql"><div class="query-rail"><button id="new-query" class="new-query">+ query</button></div><div class="sparql-body"><div class="sparql-canned"></div><textarea id="query" spellcheck="false"></textarea><div class="sparql-run"><button id="ask">Run query</button><span id="query-status" class="path"></span></div><div id="answer"></div></div></div></section><section id="panel-console" hidden><pre id="console-text" class="console"></pre></section></div><div class="settling" hidden><div class="spinner"></div><span>archiving the run…</span></div><div class="videos" hidden><button class="video-min" title="Minimize"></button><div class="video-main"><video preload="auto" playsinline disablepictureinpicture controlslist="nodownload noplaybackrate noremoteplayback"></video><span class="video-name"></span></div><div class="video-strip"></div></div><div class="transport"><div class="transport-controls"><button id="step-back" title="Previous frame">‹</button><button id="play">Play</button><button id="step-forward" title="Next frame">›</button><details class="picker speed-menu"><summary>1×</summary><div class="picker-panel"><button data-value="0.25">0.25×</button><button data-value="0.5">0.5×</button><button data-value="1" aria-pressed="true">1×</button><button data-value="2">2×</button><button data-value="5">5×</button></div></details><span id="readout" class="path"></span><button id="cancel-run" title="End the run" hidden>cancel</button></div><div class="markers"></div><input class="timeline" type="range" min="0" max="0" value="0" disabled></div>`;
}

// Plotting as the run writes is the reader's choice, not the run's: events, states and the
// console stream either way. Kept per browser, like the editor and the video panel.
const LIVE_PLOTS_KEY = "motion-spec.live-plots";
let livePlots = true;
try { livePlots = localStorage.getItem(LIVE_PLOTS_KEY) !== "off"; } catch { /* private */ }

function livePlotsOn() {
  return livePlots;
}

function bindLivePlots() {
  const button = $("#live-plots");
  const label = () => {
    button.setAttribute("aria-pressed", String(livePlots));
    button.textContent = `live plots: ${livePlots ? "on" : "off"}`;
  };
  button.onclick = () => {
    livePlots = !livePlots;
    try { localStorage.setItem(LIVE_PLOTS_KEY, livePlots ? "on" : "off"); } catch { /* private */ }
    label();
  };
  label();
}

function bindPanels() {
  const show = (panel, remember = true) => {
    document.querySelectorAll(".replay-tabs button").forEach((button) =>
      button.classList.toggle("active", button.dataset.panel === panel));
    $("#panel-plots").hidden = panel !== "plots";
    $("#panel-sparql").hidden = panel !== "sparql";
    $("#panel-console").hidden = panel !== "console";
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

// One transport for a recording or the live run writing it.
function bindTransport() {
  $(".timeline").oninput = (event) => seek(Number(event.target.value));
  $("#step-back").onclick = () => seek(state.frame - 1);
  $("#step-forward").onclick = () =>
    state.live ? simControl({ action: "step" }) : seek(state.frame + 1);
  $("#play").onclick = () =>
    state.live ? simControl({ action: state.live.paused ? "resume" : "pause" }) : togglePlayback();
  $("#cancel-run").onclick = () => simControl({ action: "cancel" });
  $$(".speed-menu .picker-panel button").forEach((option) => {
    option.onclick = () => {
      $(".speed-menu").open = false;
      setSpeed(Number(option.dataset.value));
    };
  });
  document.addEventListener("click", (event) => {
    const speed = $(".speed-menu");
    if (speed?.open && !speed.contains(event.target)) speed.open = false;
  });
  showSpeed(state.speed);
  renderMarkers();
}

// Replay speed is the page's; a live run's is the loop's.
function setSpeed(speed) {
  state.speed = speed;
  showSpeed(speed);
  const video = playingVideo();
  if (video) video.playbackRate = speed;
  if (state.live) return simControl({ action: "speed", speed });
  if (state.timer) {   // restart the animation on the new rate
    stopPlayback();
    togglePlayback();
  }
}

function showSpeed(speed) {
  $$(".speed-menu .picker-panel button").forEach((option) =>
    option.setAttribute("aria-pressed", Number(option.dataset.value) === speed));
  const summary = $(".speed-menu summary");
  if (summary) summary.textContent = `${speed}×`;
}

// A growing log is followed; only a loop that answers can be paused.
function setTransportMode() {
  const following = Boolean(state.following);
  const driving = Boolean(state.live);
  $("#play").textContent = driving ? (state.live.paused ? "Play" : "Pause") : "Play";
  $("#play").title = driving ? "Pause the simulation" : "Play the recording";
  $("#step-back").disabled = driving;
  $("#step-forward").disabled = driving && !state.live.paused;
  $("#step-forward").title = driving ? "Advance one tick" : "Next frame";
  $(".timeline").disabled = driving || !state.replay.frames;
  $("#cancel-run").hidden = !driving;
  $(".transport").classList.toggle("transport-live", following);
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

function nameApart(entries) {
  // Three axes of one constraint all read "error": tell them apart by what is left of the
  // slot's name once the part they share is gone.
  const repeated = new Set(
    entries.map((entry) => entry.label)
      .filter((label, index, all) => all.indexOf(label) !== index),
  );
  if (!repeated.size) return;
  const slots = entries.filter((entry) => entry.slot).map((entry) => entry.slot);
  let shared = 0;
  while (slots.length > 1 && slots.every((slot) => slot.startsWith(slots[0].slice(0, shared + 1)))) {
    shared += 1;
  }
  entries.forEach((entry) => {
    if (!repeated.has(entry.label) || !entry.slot) return;
    entry.label = `${entry.label} · ${entry.slot.slice(shared).replace(/^_/, "") || entry.slot}`;
  });
}

function signalPlace(signal) {
  // Where a signal belongs and what to call it there: the constraint it serves if the header
  // says, otherwise the slot it is a component of, so a pose is one entry and not seven.
  const where = state.replay.signal_index?.[signal];
  if (where) {
    const motion = state.motionOf?.[where.motion] ?? where.motion;
    return [`${motion} · ${where.constraint}`, where.role];
  }
  if (signal.startsWith("timing.")) return ["timing", signal.slice("timing.".length)];
  const [slot, ...part] = signal.split(".");
  if (part.length) return [`${spatialKind(part[0])} · ${slot}`, part.join(".")];
  return ["quantities", signal];
}

function spatialKind(part) {
  if (part === "position" || part === "orientation") return "pose";
  if (part === "linear" || part === "angular") return "twist";
  if (part === "force" || part === "torque") return "wrench";
  return "slot";
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
      itemStyle: { color: "rgba(240, 195, 106, .12)", borderColor: "rgba(240, 195, 106, .35)", borderWidth: 1, borderType: "dashed" },
      label: {
        show: tolerance > 0, position: "insideEndBottom", color: "#f0c36a", fontSize: 10,
        formatter: `±${tolerance}`,
      },
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
  // A cursor pinned to the live edge draws nothing worth a setOption on every chart; it comes
  // back by itself when following ends and the finished run is on screen.
  if (state.following) return;
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
  syncVideo();
}

// The recording of the run, if this run has one on screen.
function playingVideo() {
  const panel = $(".videos");
  const video = panel && !panel.hidden ? panel.querySelector("video") : null;
  return video && video.duration && isFinite(video.duration) ? video : null;
}

// Frames and video are two clocks over the same run: map by position, not by seconds, and
// leave a deadband so the video playing itself does not fight the cursor it is driving.
function syncVideo(force = false) {
  const video = playingVideo();
  if (!video) return;
  const target = trackFraction(state.frame) * video.duration;
  if (!force && Math.abs(video.currentTime - target) <= 0.25) return;
  // Scrubbing fires seeks faster than the decoder lands them, and each new one aborts the
  // last mid-decode: the picture stutters. Land one seek at a time; a drag only queues its
  // newest target, applied by the seeked handler when the decoder is free.
  if (video.seeking) video._queuedSeek = target;
  else video.currentTime = target;
}

function togglePlayback() {
  if (state.timer) return stopPlayback();
  const fps = state.speed * state.replay.frames / Math.max(state.replay.duration, 1e-9);
  let cursor = state.frame >= state.replay.frames - 1 ? 0 : state.frame;
  let last = performance.now();
  const video = playingVideo();
  if (video) {
    video.playbackRate = state.speed;
    if (cursor === 0) video.currentTime = 0;
    video.play().catch(() => {});
  }
  const tick = (now) => {
    // With a recording the video is the clock, so the cursor follows what is on screen.
    if (video && !video.paused) cursor = (video.currentTime / video.duration) * (state.replay.frames - 1);
    else cursor += (now - last) * fps / 1000;
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
  playingVideo()?.pause();
  if (!state.timer) return;
  cancelAnimationFrame(state.timer);
  state.timer = null;
  if ($("#play")) $("#play").textContent = "Play";
}

function updateReadout() {
  const duration = state.replay.duration;
  const time = duration * state.frame / Math.max(1, state.replay.frames - 1);
  const frames = `${state.frame.toLocaleString()} / ${state.replay.frames.toLocaleString()}`;
  $("#readout").textContent =
    `${time.toFixed(3)} / ${duration.toFixed(3)} s · frame ${frames}${state.following ? " · live" : ""}`;
}

function addPlot(signals = [], title = signals.join(" · ") || "New plot", detail = "", options = {}) {
  const { row = null, constraint = null, gains = null, data: prefetched = null } = options;
  const card = document.createElement("section");
  card.className = "plot-card";
  card.innerHTML = '<header><div><strong></strong><small></small></div><div class="plot-actions"><details class="export-plot"><summary title="Export plot"><svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.3" stroke-linecap="square"><path d="M9.5 2.5h4v4"/><path d="M13.5 2.5 8 8"/><path d="M12 9v4.5H2.5V4h4.5"/></svg></summary><div class="export-menu"><button value="png">PNG</button><button value="jpg">JPG</button><button value="svg">SVG</button><button value="pdf">PDF</button></div></details><button class="expand-plot" title="Fullscreen plot">⛶</button><button class="remove-plot" title="Remove plot">×</button></div></header><details class="signal-menu"><summary>+ add signal</summary><div class="signal-panel"><input class="signal-filter" type="search" placeholder="Filter signals"><div class="signal-list"></div></div></details><div class="plot-signals"></div><div class="plot-chart"></div><div class="plot-facts"></div>';
  card.querySelector("strong").textContent = title;
  card.querySelector("small").textContent = detail;
  card.querySelector(".plot-facts").replaceChildren(...Object.entries(gains ?? {}).map(([key, value]) => {
    const fact = document.createElement("span");
    fact.innerHTML = "<b></b><i></i>";
    fact.querySelector("b").textContent = key;
    fact.querySelector("i").textContent = value;
    return fact;
  }));
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
    state.livePlots.delete(chart);
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
  const menu = card.querySelector(".signal-menu");
  const filter = card.querySelector(".signal-filter");
  const list = card.querySelector(".signal-list");
  const fillSignals = () => {
    const query = filter.value.toLowerCase().trim();
    const groups = new Map();
    state.replay.signals.forEach((signal) => {
      const [group, label] = signalPlace(signal);
      if (query && !`${signal} ${label} ${group}`.toLowerCase().includes(query)) return;
      groups.set(group, [...(groups.get(group) ?? []), {
        signal, label, slot: state.replay.signal_index?.[signal]?.slot,
      }]);
    });
    groups.forEach(nameApart);
    list.replaceChildren(...[...groups].flatMap(([group, entries]) => {
      const heading = document.createElement("div");
      heading.className = "signal-group";
      heading.textContent = group;
      return [heading, ...entries.map(({ signal, label }) => {
        const option = document.createElement("button");
        option.className = "signal-option";
        option.textContent = label;
        option.title = signal;
        option.disabled = signals.includes(signal);
        option.onclick = () => {
          menu.open = false;
          discard();
          addPlot([...signals, signal], title, detail, { row, constraint, gains });
        };
        return option;
      })];
    }));
  };
  filter.oninput = fillSignals;
  menu.ontoggle = () => {
    if (!menu.open) return;
    filter.value = "";
    fillSignals();
    filter.focus();
  };
  menu.onkeydown = (event) => {
    if (event.key === "Escape") menu.open = false;
  };
  fillSignals();
  card.querySelector(".remove-plot").onclick = () => {
    discard();
    if (row && !$(`#plots [data-row="${card.dataset.row}"]`)) delete row.dataset.plotted;
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
  (prefetched ? Promise.resolve(prefetched) : api(`/api/plot?${query}`)).then((data) => {
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
        // A shared prefetch carries the whole motion's signals; a card asks only for its own.
        data: (data.signals?.[signal] ?? [])
          .map((value, point) => value == null ? null : [(data.first_frame ?? 0) + point * data.sample_step, value])
          .filter(Boolean),
        lineStyle: { width: 1.5 },
        ...(index === 0 ? setpointBands(constraint) : {}),
      })),
      { name: "__cursor", type: "line", data: [], silent: true },
    ],
    });
    chart.cursorIndex = signals.length;
    // Same as updateCursor: while the run is being followed the cursor sits on the live edge.
    if (!state.following) chart.setOption(cursorOption(state.frame, chart.cursorIndex));
    // Live cards are addLivePlot's uPlot adapters; a replay card never registers for streaming.
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
  // Only a .robmot is a whole generation to make; the other authored files are parts of one.
  if (source.endsWith(".robmot")) bindGenerate(source);
}

// Generate, build and run a model from its own page, showing the terminal that does it.
function bindGenerate(source) {
  // Two ways in: make the generation and stop, or make it and watch it run.
  const generate = document.createElement("button");
  generate.className = "run-start";
  generate.textContent = "generate";
  const start = document.createElement("button");
  start.className = "run-start";
  start.textContent = "▶ generate & run";
  const state_ = document.createElement("span");
  state_.className = "run-state";
  const open = document.createElement("button");
  open.className = "run-start";
  open.textContent = "open generation";
  open.hidden = true;
  $(".viewer .open-with").append(generate, start, open, state_);
  const pre = document.createElement("pre");
  pre.className = "console";
  pre.hidden = true;
  $(".viewer-heading").after(pre);

  const stop = () => {
    clearInterval(state.generateWatch);
    clearInterval(state.generateConsole);
  };
  const busy = (working) => { generate.disabled = start.disabled = working; };
  const gone = () => state.viewing !== source || !pre.isConnected;
  const launch = async (run) => {
    stop();
    busy(true);
    open.hidden = true;
    state_.textContent = "starting…";
    pre.textContent = "";
    pre.hidden = false;
    let offset = 0;
    try {
      const started = await post("/api/generate", { path: source, run });
      state_.textContent = `running · pid ${started.pid}`;
      const tail = async () => {
        if (gone()) return stop();
        const slice = await api(`/api/console?job=${encodeURIComponent(started.job)}&offset=${offset}`)
          .catch(() => null);
        if (!slice || !slice.text) return;
        offset = slice.offset;
        appendConsole(pre, slice.text);
      };
      const check = async () => {
        if (gone()) return stop();
        const status = await api(`/api/generate?job=${encodeURIComponent(started.job)}`)
          .catch(() => null);
        if (!status) return;
        state_.textContent = status.busy
          ? `running · pid ${status.pid}`
          : `exited ${status.exit_code}`;
        if (status.generation) {
          open.hidden = false;
          open.onclick = () => {
            setTab("logs");
            // the generation is new: the sidebar has not listed it yet
            loadGenerations(true).then(() => selectGeneration(status.generation)).catch(showError);
          };
        }
        if (status.busy) return;
        clearInterval(state.generateWatch);
        busy(false);
        await tail();   // the closing lines land after the process is already gone
        clearInterval(state.generateConsole);
      };
      await tail();
      state.generateConsole = setInterval(tail, 1000);
      state.generateWatch = setInterval(check, 2000);
    } catch (error) {
      stop();
      busy(false);
      state_.textContent = error.message;
    }
  };
  generate.onclick = () => launch(false);
  start.onclick = () => launch(true);
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
$("#health").onclick = () => loadHealth().catch(showError);

// Why a build or a run cannot work, before it is tried: the CLI's own checks, on the page.
// The checks cost seconds, so the server runs them on a thread and this follows it.
async function loadHealth(refresh = false) {
  clearTimeout(state.healthWatch);
  const report = await api(`/api/health${refresh ? "?refresh=1" : ""}`);
  renderHealth(report);
  if (report.running) state.healthWatch = setTimeout(() => loadHealth().catch(showError), 2000);
}

function renderHealth(report) {
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
  $("#status").textContent = "installation health";
}
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
  // Only the page being looked at has to move, and then only as far as its parent.
  const trashed = [...state.selected];
  const inTrash = (path) => path && trashed.some((gone) => path === gone || path.startsWith(`${gone}/`));
  const generation = state.generationPath;
  state.selected.clear();
  $("#selection-actions").hidden = true;
  state.cache = {};
  if (inTrash(generation)) {
    state.generation = null;
    return goHome();
  }
  await loadGenerations(true).catch(showError);
  if (!generation) return;
  if (inTrash(state.runPath)) return selectGeneration(generation).catch(showError);
  api(`/api/runs?path=${encodeURIComponent(generation)}`).then(state.generation?.setRuns);
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
  state.autoCollapsed = false;   // a deliberate choice outlives the width that suggested it
  showSidebarState();
  reserveVideoSpace();
};
function showSidebarState() {
  const collapsed = document.body.classList.contains("sidebar-collapsed");
  $("#sidebar-toggle").textContent = collapsed ? "☰" : "×";
  $("#sidebar-toggle").title = collapsed ? "Expand navigation" : "Collapse navigation";
}
showSidebarState();

// A narrow window has no room for both: the sidebar folds away and comes back with the width,
// without overwriting what the reader chose at a width where both fit.
const NARROW = 1000;
function fitSidebar() {
  const narrow = window.innerWidth < NARROW;
  if (narrow && !document.body.classList.contains("sidebar-collapsed")) {
    document.body.classList.add("sidebar-collapsed");
    state.autoCollapsed = true;
  } else if (!narrow && state.autoCollapsed) {
    document.body.classList.remove("sidebar-collapsed");
    state.autoCollapsed = false;
  }
  showSidebarState();
  reserveVideoSpace();
}
window.addEventListener("resize", fitSidebar);
fitSidebar();
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
  const rendered = () => document.documentElement.classList.remove("restoring");
  if (view.has("run")) {
    loadSidebar().then(() => loadReplay(view.get("run"))).then(rendered).catch(showError);
  }
  else if (view.has("generation")) {
    loadSidebar().then(() => selectGeneration(view.get("generation"))).then(rendered).catch(showError);
  }
  else loadSidebar().then(rendered).catch(showError);
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
