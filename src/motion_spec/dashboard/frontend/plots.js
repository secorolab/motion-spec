// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/** A chart of a finished run: what it plots, what it is called, and what it exports to. */

import { $, api, nextPlotKey, readStored, state, writeStored } from "./core.js";

const SERIES_COLOURS = ["#e07a5f", "#79c6a5", "#9da9c7", "#f0c36a"];

const AXIS_COLOUR = "#73777d";
const GRID_COLOUR = "#383d45";
const TEXT_COLOUR = "#f1eee7";
const BAND_COLOUR = "#f0c36a";

const MAGNIFIER = "M8.2,2 A6,6 0 1,0 8.2,14.2 A6,6 0 1,0 8.2,2 M12.7,12.7 L17.4,17.4";

const MAGNIFIER_IN = `path://${MAGNIFIER} M5.4,8.1 L11,8.1 M8.2,5.3 L8.2,10.9`;

const MAGNIFIER_OUT = `path://${MAGNIFIER} M5.4,8.1 L11,8.1`;

const RESET_ARROW = "path://M15.4,9.6 A6.2,6.2 0 1,1 9.2,3.4 M9.2,0.7 L9.2,6.1 M6.5,3.4 L11.9,3.4";

const PLOT_CARD_MARKUP =
  '<header><div><strong></strong><small></small></div><div class="plot-actions"><details class="export-plot"><summary title="Export plot"><svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.3" stroke-linecap="square"><path d="M9.5 2.5h4v4"/><path d="M13.5 2.5 8 8"/><path d="M12 9v4.5H2.5V4h4.5"/></svg></summary><div class="export-menu"><button value="png">PNG</button><button value="jpg">JPG</button><button value="svg">SVG</button><button value="pdf">PDF</button></div></details><button class="expand-plot" title="Fullscreen plot">⛶</button><button class="remove-plot" title="Remove plot">×</button></div></header><details class="signal-menu"><summary>+ add signal</summary><div class="signal-panel"><input class="signal-filter" type="search" placeholder="Filter signals"><div class="signal-list"></div></div></details><div class="plot-signals"></div><div class="plot-chart"></div><div class="plot-facts"></div>';

export function addPlot(
  signals = [],
  title = signals.join(" · ") || "New plot",
  detail = "",
  options = {},
) {
  const { row = null, constraint = null, gains = null, data: prefetched = null } = options;
  const card = document.createElement("section");
  card.className = "plot-card";
  card.innerHTML = PLOT_CARD_MARKUP;
  card.querySelector("strong").textContent = title;
  card.querySelector("small").textContent = detail;
  card.querySelector(".plot-facts").replaceChildren(...gainFacts(gains));
  $("#plots").append(card);
  const chart = echarts.init(card.querySelector(".plot-chart"));
  card.inspection = () => ({
    signals,
    title,
    detail,
    constraint,
    gains,
    zoom: chart.getOption().dataZoom?.map(({ start, end }) => ({ start, end })),
  });
  state.charts.push(chart);
  // the plot grid reflows as cards come and go; the canvas only follows if told
  const observer = new ResizeObserver(() => chart.resize());
  observer.observe(card.querySelector(".plot-chart"));
  if (row) card.dataset.row = row.dataset.plotKey ??= nextPlotKey();
  const discard = () => {
    observer.disconnect();
    chart.dispose();
    state.charts = state.charts.filter((item) => item !== chart);
    state.livePlots.delete(chart);
    card.remove();
  };
  const replaceSignals = (next) => {
    const sibling = card.nextSibling;
    const zoom = card.inspection().zoom;
    discard();
    addPlot(next, title, detail, { row, constraint, gains, zoom });
    const replacement = $("#plots").lastElementChild;
    if (sibling) $("#plots").insertBefore(replacement, sibling);
  };
  card.querySelector(".expand-plot").onclick = () =>
    document.fullscreenElement ? document.exitFullscreen() : card.requestFullscreen();
  bindExport(card, chart, title);
  bindSignalMenu(card, signals, replaceSignals);
  card.querySelector(".remove-plot").onclick = () => {
    discard();
    if (row && !$(`#plots [data-row="${card.dataset.row}"]`)) delete row.dataset.plotted;
  };
  if (!signals.length) {
    chart.setOption({
      graphic: {
        type: "text",
        left: "center",
        top: "middle",
        style: { text: "Choose a signal above", fill: AXIS_COLOUR },
      },
    });
    return;
  }
  card.querySelector(".plot-signals").replaceChildren(
    ...signals.map((signal, index) => {
      const chip = document.createElement("button");
      chip.className = "plot-signal";
      chip.style.setProperty("--series", SERIES_COLOURS[index % SERIES_COLOURS.length]);
      chip.textContent = signal;
      chip.title = "Remove this signal";
      chip.onclick = () => replaceSignals(signals.filter((item) => item !== signal));
      return chip;
    }),
  );
  chart.showLoading("default", { text: "Loading recorded data…" });
  drawSignals(card, chart, signals, { constraint, prefetched, zoom: options.zoom });
}

function gainFacts(gains) {
  return Object.entries(gains ?? {}).map(([key, value]) => {
    const cell = document.createElement("span");
    cell.innerHTML = "<b></b><i></i>";
    cell.querySelector("b").textContent = key;
    cell.querySelector("i").textContent = value;
    return cell;
  });
}

function drawSignals(card, chart, signals, { constraint, prefetched, zoom }) {
  const query = new URLSearchParams({ path: state.runPath });
  signals.forEach((signal) => query.append("signal", signal));
  // sample inside the motion's window, or a short motion falls between two samples
  (constraint?.window ?? []).forEach((bound) => query.append("window", bound));
  (prefetched ? Promise.resolve(prefetched) : api(`/api/plot?${query}`))
    .then((data) => {
      if (chart.isDisposed()) return;
      chart.hideLoading();
      const seriesValues = signals.map((signal) => data.signals?.[signal] ?? []);
      const step = data.sample_step || 1;
      const firstFrame = data.first_frame ?? 0;
      // Kept whole so progressive drawing has something to cut from, and something to restore to.
      const drawn = seriesValues.map((values) => points(values, firstFrame, step));
      const nearest = trackNearestSeries(card, chart, seriesValues, firstFrame, step);
      chart.setOption(chartOption(chart, signals, drawn, constraint, nearest));
      chart.cursorIndex = signals.length;
      if (zoom?.length) chart.setOption({ dataZoom: zoom });
      chart.fullSeries = drawn;
      if (progressive) {
        // The axis stays the run's full width whatever is drawn into it, so the line grows across
        // a picture that does not move under it.
        chart.setOption({
          xAxis: {
            min: constraint?.window?.[0] ?? firstFrame,
            max: constraint?.window?.[1] ?? Math.max(0, (state.replay?.frames ?? 1) - 1),
          },
        });
        chart.setOption(seriesUpTo(chart, state.frame));
      }
      // Same as updateCursor: while the run is being followed the cursor sits on the live edge.
      if (!state.following) chart.setOption(cursorOption(state.frame, chart.cursorIndex));
    })
    .catch((error) => chart.showLoading("default", { text: error.message }));
}

const points = (values, firstFrame, step) =>
  values
    .map((value, point) => (value == null ? null : [firstFrame + point * step, value]))
    .filter(Boolean);

// Which line the pointer is nearest, so one tooltip row can stand out. Measured in pixels for
// a scale-fair distance, and held until another line is clearly closer: constraint lines cross
// and sit pixel-adjacent, and the choice would flicker on every mouse-move there.
const HYSTERESIS_PX = 6;

function trackNearestSeries(card, chart, seriesValues, firstFrame, step) {
  let nearestIndex = -1;
  const plotDom = card.querySelector(".plot-chart");
  plotDom.addEventListener("mousemove", (event) => {
    const [x] = chart.convertFromPixel({ xAxisIndex: 0, yAxisIndex: 0 }, [
      event.offsetX,
      event.offsetY,
    ]);
    const point = Math.round((x - firstFrame) / step);
    const distances = seriesValues.map((values) => {
      const value = values[point];
      if (value == null) return Infinity;
      const pixelY = chart.convertToPixel({ xAxisIndex: 0, yAxisIndex: 0 }, [x, value])[1];
      return Math.abs(pixelY - event.offsetY);
    });
    let best = distances.indexOf(Math.min(...distances));
    if (distances[best] === Infinity) best = -1;
    if (
      nearestIndex >= 0 &&
      best !== nearestIndex &&
      distances[nearestIndex] - distances[best] < HYSTERESIS_PX
    )
      best = nearestIndex;
    nearestIndex = best;
  });
  plotDom.addEventListener("mouseleave", () => {
    nearestIndex = -1;
  });
  return () => nearestIndex;
}

function zoomBy(chart, factor) {
  const [{ start = 0, end = 100 } = {}] = chart.getOption().dataZoom ?? [];
  const middle = (start + end) / 2;
  const half = Math.min(50, ((end - start) * factor) / 2);
  chart.dispatchAction({
    type: "dataZoom",
    start: Math.max(0, middle - half),
    end: Math.min(100, middle + half),
  });
}

function chartOption(chart, signals, drawn, constraint, nearest) {
  return {
    animation: false,
    color: [...SERIES_COLOURS],
    grid: { left: 62, right: 22, top: 34, bottom: 52 },
    dataZoom: [{ type: "inside", filterMode: "none" }],
    toolbox: {
      right: 8,
      top: 2,
      itemSize: 15,
      itemGap: 12,
      iconStyle: { color: "none", borderColor: AXIS_COLOUR, borderWidth: 1.1 },
      emphasis: { iconStyle: { borderColor: "#e07a5f" } },
      feature: {
        myZoomIn: {
          show: true,
          title: "Zoom in",
          icon: MAGNIFIER_IN,
          onclick: () => zoomBy(chart, 0.5),
        },
        myZoomOut: {
          show: true,
          title: "Zoom out",
          icon: MAGNIFIER_OUT,
          onclick: () => zoomBy(chart, 2),
        },
        myZoomReset: {
          show: true,
          title: "Reset zoom",
          icon: RESET_ARROW,
          onclick: () => chart.dispatchAction({ type: "dataZoom", start: 0, end: 100 }),
        },
      },
    },
    tooltip: {
      trigger: "axis",
      backgroundColor: "#202327",
      borderColor: GRID_COLOUR,
      textStyle: { color: TEXT_COLOUR },
      formatter: (params) => tooltipRows(params, nearest()),
    },
    legend: { show: false },
    xAxis: {
      type: "value",
      scale: true,
      name: "frame",
      nameLocation: "middle",
      nameGap: 26,
      nameTextStyle: { color: AXIS_COLOUR },
      min: constraint?.window?.[0],
      max: constraint?.window?.[1],
      axisLabel: { color: AXIS_COLOUR },
      axisLine: { lineStyle: { color: AXIS_COLOUR } },
      splitLine: { lineStyle: { color: GRID_COLOUR } },
    },
    yAxis: {
      type: "value",
      axisLabel: { color: AXIS_COLOUR },
      axisLine: { lineStyle: { color: AXIS_COLOUR } },
      splitLine: { lineStyle: { color: GRID_COLOUR } },
    },
    series: [
      ...signals.map((signal, index) => ({
        name: signal,
        type: "line",
        showSymbol: false,
        data: drawn[index],
        lineStyle: { width: 1.5 },
        ...(index === 0 ? setpointBands(constraint) : {}),
      })),
      { name: "__cursor", type: "line", data: [], silent: true },
    ],
  };
}

function tooltipRows(params, nearestIndex) {
  const frame = params[0]?.axisValue ?? "";
  const rows = params
    .filter((entry) => entry.seriesName !== "__cursor")
    .map((entry) => {
      const active = entry.seriesIndex === nearestIndex;
      const value = Array.isArray(entry.value) ? entry.value[1] : entry.value;
      const style = active ? "color:#f1eee7;font-weight:700" : "color:#9aa0a6";
      return (
        `<div style="display:flex;align-items:center;gap:6px;${style}">` +
        `<span style="width:8px;height:8px;border-radius:50%;background:${entry.color};flex:none"></span>` +
        `<span style="overflow:hidden;text-overflow:ellipsis;white-space:nowrap">${entry.seriesName}</span>` +
        `<span style="margin-left:auto;padding-left:10px">${value ?? "—"}</span></div>`
      );
    })
    .join("");
  return `<div style="color:#73777d;margin-bottom:4px">frame ${frame}</div>${rows}`;
}

function setpointBands(constraint) {
  if (!constraint?.setpoints?.length) return {};
  const tolerance = constraint.tolerance ?? 0;
  return {
    markLine: {
      symbol: "none",
      animation: false,
      silent: true,
      label: {
        color: BAND_COLOUR,
        position: "insideEndTop",
        formatter: ({ value }) => `setpoint ${value}`,
      },
      lineStyle: { color: BAND_COLOUR, type: "dashed", width: 1 },
      data: constraint.setpoints.map((point) => ({ yAxis: point.value })),
    },
    markArea: {
      silent: true,
      itemStyle: {
        color: "rgba(240, 195, 106, .12)",
        borderColor: "rgba(240, 195, 106, .35)",
        borderWidth: 1,
        borderType: "dashed",
      },
      label: {
        show: tolerance > 0,
        position: "insideEndBottom",
        color: BAND_COLOUR,
        fontSize: 10,
        formatter: `±${tolerance}`,
      },
      data: constraint.setpoints.map((point) => [
        { yAxis: point.value - tolerance },
        { yAxis: point.value + tolerance },
      ]),
    },
  };
}

export function cursorOption(frame, index = 0) {
  return {
    series: [
      ...Array.from({ length: index }, () => ({})),
      {
        markLine: {
          symbol: "none",
          animation: false,
          silent: true,
          label: { show: false },
          lineStyle: { color: TEXT_COLOUR, width: 1, opacity: 0.7 },
          data: [{ xAxis: frame }],
        },
      },
    ],
  };
}

function bindSignalMenu(card, signals, replaceSignals) {
  const menu = card.querySelector(".signal-menu");
  const filter = card.querySelector(".signal-filter");
  const list = card.querySelector(".signal-list");
  const chosen = new Set(signals);
  const fillSignals = () => {
    const query = filter.value.toLowerCase().trim();
    const groups = new Map();
    state.replay.signals.forEach((signal) => {
      const [group, label] = signalPlace(signal);
      if (query && !`${signal} ${label} ${group}`.toLowerCase().includes(query)) return;
      const entry = { signal, label, slot: state.replay.signal_index?.[signal]?.slot };
      const inside = groups.get(group);
      if (inside) inside.push(entry);
      else groups.set(group, [entry]);
    });
    groups.forEach(nameApart);
    list.replaceChildren(
      ...[...groups].flatMap(([group, entries]) => {
        const heading = document.createElement("div");
        heading.className = "signal-group";
        heading.textContent = group;
        return [
          heading,
          ...entries.map(({ signal, label }) => {
            const option = document.createElement("button");
            option.className = "signal-option";
            option.textContent = label;
            option.title = signal;
            option.disabled = chosen.has(signal);
            option.onclick = () => {
              menu.open = false;
              replaceSignals([...signals, signal]);
            };
            return option;
          }),
        ];
      }),
    );
  };
  filter.oninput = fillSignals;
  // Filled on opening, never on creation: a run has hundreds of signals and a page has dozens
  // of cards, and the list nobody opened is never read.
  menu.ontoggle = () => {
    if (!menu.open) return;
    filter.value = "";
    fillSignals();
    filter.focus();
  };
  menu.onkeydown = (event) => {
    if (event.key === "Escape") menu.open = false;
  };
}

// Where a signal belongs: the constraint it serves if the header says, else the slot it is a
// component of, so a pose is one entry and not seven.
function signalPlace(signal) {
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

// Three axes of one constraint all read "error"; tell them apart by whatever is left of the
// slot name once the shared prefix is gone.
function nameApart(entries) {
  const repeated = new Set(
    entries.map((entry) => entry.label).filter((label, index, all) => all.indexOf(label) !== index),
  );
  if (!repeated.size) return;
  const slots = entries.filter((entry) => entry.slot).map((entry) => entry.slot);
  let shared = 0;
  while (
    slots.length > 1 &&
    slots.every((slot) => slot.startsWith(slots[0].slice(0, shared + 1)))
  ) {
    shared += 1;
  }
  entries.forEach((entry) => {
    if (!repeated.has(entry.label) || !entry.slot) return;
    entry.label = `${entry.label} · ${entry.slot.slice(shared).replace(/^_/, "") || entry.slot}`;
  });
}

function bindExport(card, chart, title) {
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
}

function exportChart(chart, name, format) {
  if (format === "png" || format === "jpg") {
    const link = document.createElement("a");
    link.download = `${name}.${format}`;
    link.href = chart.getDataURL({
      type: format === "png" ? "png" : "jpeg",
      pixelRatio: 2,
      backgroundColor: "#16181b",
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
  holder.style.cssText = `position:fixed;left:-10000px;top:0;width:${chart.getWidth()}px;height:${chart.getHeight()}px`;
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

// Draw a recording the way a run arrives: nothing past the cursor, into an axis already scaled
// to the whole run. A growing axis would rescale under the reader on every frame.
const PROGRESSIVE_KEY = "motion-spec.progressive-replay";

let progressive = readStored(PROGRESSIVE_KEY) === "on";

export function progressiveOn() {
  return progressive;
}

export function setProgressive(on) {
  progressive = on;
  writeStored(PROGRESSIVE_KEY, on ? "on" : "off");
}

// The points are in frame order, so the cut is a binary search, not a scan.
function pointsUpTo(series, frame) {
  let low = 0;
  let high = series.length;
  while (low < high) {
    const middle = (low + high) >> 1;
    if (series[middle][0] <= frame) low = middle + 1;
    else high = middle;
  }
  return low;
}

export function seriesUpTo(chart, frame) {
  const full = chart.fullSeries;
  if (!full) return null;
  return {
    series: full.map((series) => ({
      data: progressive ? series.slice(0, pointsUpTo(series, frame)) : series,
    })),
  };
}
