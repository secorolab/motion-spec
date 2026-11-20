// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/** CodeMirror, imported once, themed to the page, and told which lines changed since HEAD. */

import { $, post, snack } from "./core.js";

let codemirror = null;

let mounted = null;

export function revealLine(number) {
  if (!mounted || !(number >= 1) || number > mounted.state.doc.lines) return;
  const line = mounted.state.doc.line(number);
  mounted.dispatch({ selection: { anchor: line.from }, scrollIntoView: true });
  mounted.focus();
}

// each the exact file `codemirror` imports, or a second copy ignores its decorations
const CM_VIEW = "./vendor/esm/cm-view.mjs";

const CM_SEARCH = "./vendor/esm/cm-search.mjs";

const CM_LANGUAGE = "./vendor/esm/cm-language.mjs";

const CM_TAGS = "./vendor/esm/lezer-highlight.mjs";

// legacy modes: a lang-* pack's own @codemirror/language copy renders no highlighting
const LANGUAGE_PACKS = [
  {
    suffixes: /\.(?:h|hh|hpp|hxx|c|cc|cpp|cxx|inc)$/i,
    url: "./vendor/esm/cm-clike.mjs",
    parser: (pack) => pack.cpp,
  },
  {
    suffixes: /\.json$/i,
    url: "./vendor/esm/cm-javascript.mjs",
    parser: (pack) => pack.json,
  },
];

const packs = new Map();

async function languageFor(name, StreamLanguage) {
  const wanted = LANGUAGE_PACKS.find(({ suffixes }) => suffixes.test(name));
  if (!wanted) return structuralMode(StreamLanguage);
  if (!packs.has(wanted.url)) {
    packs.set(
      wanted.url,
      import(wanted.url).then(wanted.parser).catch(() => null),
    );
  }
  const parser = await packs.get(wanted.url);
  return parser ? StreamLanguage.define(parser) : structuralMode(StreamLanguage);
}

async function editorModule() {
  codemirror ??= Promise.all([
    import("./vendor/esm/codemirror.mjs"),
    import(CM_VIEW),
    import(CM_SEARCH),
    import(CM_LANGUAGE),
    import(CM_TAGS),
  ]).then(([setup, view, find, language, highlight]) => ({
    ...setup,
    ...view,
    ...find,
    ...language,
    ...highlight,
  }));
  return codemirror;
}

const TOKYO_NIGHT = {
  comment: "#565f89",
  red: "#f7768e",
  orange: "#ff9e64",
  yellow: "#e0af68",
  green: "#9ece6a",
  teal: "#73daca",
  cyan: "#2ac3de",
  sky: "#89ddff",
  blue: "#7aa2f7",
  purple: "#bb9af7",
  text: "#c0caf5",
  dim: "#a9b1d6",
};

function tokyoNightStyle(HighlightStyle, tags) {
  return HighlightStyle.define([
    { tag: tags.comment, color: TOKYO_NIGHT.comment, fontStyle: "italic" },
    { tag: tags.keyword, color: TOKYO_NIGHT.purple },
    { tag: tags.string, color: TOKYO_NIGHT.green },
    { tag: tags.number, color: TOKYO_NIGHT.orange },
    { tag: tags.typeName, color: TOKYO_NIGHT.cyan },
    { tag: tags.propertyName, color: TOKYO_NIGHT.blue },
    { tag: tags.variableName, color: TOKYO_NIGHT.text },
    { tag: tags.operator, color: TOKYO_NIGHT.sky },
    { tag: tags.punctuation, color: TOKYO_NIGHT.dim },
    { tag: tags.bool, color: TOKYO_NIGHT.orange },
    { tag: tags.atom, color: TOKYO_NIGHT.orange },
    { tag: tags.invalid, color: TOKYO_NIGHT.red },
  ]);
}

function structuralMode(StreamLanguage) {
  return StreamLanguage.define({
    // kind: the last word declares, so the next is its name; depth: inside `(ns=..)`
    startState: () => ({ kind: false, depth: 0 }),
    token(stream, state) {
      if (stream.match(/^\/\/.*/)) return "comment";
      if (stream.match(/^\/\*/)) {
        stream.match(/^[\s\S]*?(\*\/|$)/);
        return "comment";
      }
      if (stream.match(/^"(?:[^"\\]|\\.)*"?/)) return "string";
      if (stream.match(/^'(?:[^'\\]|\\.)*'?/)) return "string";
      if (stream.match(/^<[^<>\n]*>?/)) return "typeName";
      if (stream.match(/^-?\d+(?:\.\d+)?(?:e-?\d+)?/)) {
        stream.match(/^\s*[a-zA-Z]+(?:\^-?\d)?(?:\/[a-zA-Z]+(?:\^-?\d)?)*\b/);
        return "number";
      }
      if (stream.match(/^[A-Za-z_][\w-]*(?:[ \t]+[A-Za-z_][\w-]*)*(?=[ \t]*:)/)) {
        state.kind = false;
        return "propertyName";
      }
      // `KIND... (ns=..) NAME`: the bracket divides the kind from the name
      if (
        stream.match(
          /^[A-Za-z_][\w-]*(?:[ \t]+[A-Za-z_][\w-]*)*(?=[ \t]*\([^)]*\)[ \t]*[A-Za-z_][\w-]*)/,
        )
      ) {
        state.kind = true;
        return "keyword";
      }
      const word = stream.match(/^[A-Za-z_][\w-]*/);
      if (word) {
        if (state.depth > 0) return null;
        const kind = state.kind;
        state.kind = false;
        if (kind) return "variableName";
        // declares when a name then `=` or `{` follows; a bare `{` opens an unnamed block
        if (stream.match(/^\s*\{/, false)) return "keyword";
        if (stream.match(/^\s+(?:\([^)]*\)\s*)?[A-Za-z_][\w-]*\s*[={]/, false)) {
          state.kind = true;
          return "keyword";
        }
        return null;
      }
      const mark = stream.match(/^[{}[\](),]/);
      if (mark) {
        if (mark[0] === "(") state.depth += 1;
        if (mark[0] === ")") state.depth = Math.max(0, state.depth - 1);
        return "punctuation";
      }
      if (stream.match(/^[=.]/)) return "operator";
      stream.next();
      return null;
    },
    languageData: { commentTokens: { line: "//", block: { open: "/*", close: "*/" } } },
  });
}

function myersInsertions(a, b) {
  const n = a.length;
  const m = b.length;
  const max = n + m;
  let v = new Map([[1, 0]]);
  const trace = [];
  let dEnd = -1;
  outer: for (let d = 0; d <= max; d += 1) {
    trace.push(v);
    const next = new Map(v);
    for (let k = -d; k <= d; k += 2) {
      let x;
      if (k === -d || (k !== d && (v.get(k - 1) ?? -1) < (v.get(k + 1) ?? -1))) {
        x = v.get(k + 1) ?? 0;
      } else {
        x = (v.get(k - 1) ?? 0) + 1;
      }
      let y = x - k;
      while (x < n && y < m && a[x] === b[y]) {
        x += 1;
        y += 1;
      }
      next.set(k, x);
      if (x >= n && y >= m) {
        dEnd = d;
        break outer;
      }
    }
    v = next;
  }
  const added = new Set();
  let x = n;
  let y = m;
  for (let d = dEnd; d > 0; d -= 1) {
    const vPrev = trace[d];
    const k = x - y;
    const prevK =
      k === -d || (k !== d && (vPrev.get(k - 1) ?? -1) < (vPrev.get(k + 1) ?? -1)) ? k + 1 : k - 1;
    const prevX = vPrev.get(prevK) ?? 0;
    const prevY = prevX - prevK;
    while (x > prevX && y > prevY) {
      x -= 1;
      y -= 1;
    }
    if (x === prevX) {
      y -= 1;
      added.add(y);
    } else {
      x -= 1;
    }
  }
  return added;
}

export async function mountEditor(
  holder,
  source,
  text,
  readOnly = false,
  gitHead = null,
  line = null,
) {
  let cm;
  mounted = null;
  try {
    cm = await editorModule();
  } catch (error) {
    holder.textContent = text;
    holder.classList.add("plain-source");
    return snack(`code editor unavailable (${error.message}) — showing plain text`);
  }
  const {
    EditorView,
    basicSetup,
    Decoration,
    ViewPlugin,
    search,
    StreamLanguage,
    HighlightStyle,
    syntaxHighlighting,
    tags,
    forceParsing,
  } = cm;
  const save = $("#save-source");
  const dirty = (is) => {
    if (!save) return;
    save.hidden = !is;
    save.textContent = "save";
  };
  const write = async (target) => {
    if (!save || save.hidden) return;
    const written = target.state.doc.toString();
    try {
      save.textContent = "saving…";
      const { check } = await post("/api/source", { path: source, text: written });
      save.hidden = false;
      save.textContent = "saved";
      setTimeout(() => dirty(false), 1200);
      showCheck(target, check);
    } catch (error) {
      snack(error.message);
      dirty(true);
    }
  };
  const badLine = Decoration.line({ class: "cm-badLine" });
  const problem = { line: null };
  const badMarks = (doc) =>
    problem.line && problem.line <= doc.lines
      ? Decoration.set([badLine.range(doc.line(problem.line).from)])
      : Decoration.none;
  const trackProblem = ViewPlugin.fromClass(
    class {
      constructor(target) {
        this.decorations = badMarks(target.state.doc);
      }

      update(target) {
        if (target.docChanged) problem.line = null;
        this.decorations = badMarks(target.state.doc);
      }
    },
    { decorations: (plugin) => plugin.decorations },
  );
  const showCheck = (target, check) => {
    problem.line = check && !check.ok && check.line ? check.line : null;
    target.dispatch({}); // empty dispatch redraws the mark
    const banner = $(".syntax-state");
    if (!banner) return;
    banner.hidden = !check || check.ok;
    if (!check || check.ok) return;
    banner.textContent = check.line ? `line ${check.line}: ${check.message}` : check.message;
    banner.onclick = () => revealLine(check.line);
  };
  holder.style.setProperty("--head", `${$(".viewer-top")?.offsetHeight ?? 60}px`);
  // baseline is git HEAD, else the open-time text
  const was = (gitHead ?? text).split("\n");
  const changedLine = Decoration.line({ class: "cm-changedLine" });
  const marked = (doc) => {
    const now = doc.toString().split("\n");
    let head = 0;
    while (head < now.length && head < was.length && now[head] === was[head]) head += 1;
    let tail = 0;
    while (
      tail < now.length - head &&
      tail < was.length - head &&
      now[now.length - 1 - tail] === was[was.length - 1 - tail]
    ) {
      tail += 1;
    }
    const before = was.slice(head, was.length - tail);
    const after = now.slice(head, now.length - tail);
    if (!after.length) return [];
    return [...myersInsertions(before, after)].map((index) => head + index + 1);
  };
  const changes = (doc) =>
    Decoration.set(
      marked(doc)
        .sort((left, right) => left - right)
        .map((n) => changedLine.range(doc.line(n).from)),
    );
  const trackChanges = ViewPlugin.fromClass(
    class {
      constructor(target) {
        this.decorations = changes(target.state.doc);
      }

      update(target) {
        if (target.docChanged) this.decorations = changes(target.state.doc);
      }
    },
    { decorations: (plugin) => plugin.decorations },
  );
  const view = new EditorView({
    parent: holder,
    doc: text,
    extensions: [
      basicSetup,
      // by default Ctrl-F opens off the bottom of a page-scrolled editor
      search({ top: true }),
      await languageFor(source, StreamLanguage),
      // after basicSetup, so this highlighter is asked before the stock one
      syntaxHighlighting(tokyoNightStyle(HighlightStyle, tags)),
      trackChanges,
      trackProblem,
      EditorView.lineWrapping,
      EditorView.editable.of(!readOnly),
      EditorView.updateListener.of((update) => {
        if (update.docChanged) dirty(true);
      }),
      EditorView.theme(
        {
          "&": { color: "var(--text)", backgroundColor: "transparent", fontSize: "12.5px" },
          ".cm-content": { fontFamily: "var(--code)", padding: "0" },
          ".cm-gutters": {
            backgroundColor: "transparent",
            color: "var(--dim)",
            border: "0",
            fontFamily: "var(--code)",
          },
          // translucent: the selection is drawn behind the lines
          ".cm-activeLine": { backgroundColor: "rgba(255, 255, 255, .06)" },
          ".cm-changedLine": { boxShadow: "inset 2px 0 0 var(--accent)" },
          ".cm-badLine": { backgroundColor: "rgba(224, 122, 95, .18)" },
          ".cm-activeLineGutter": { backgroundColor: "transparent", color: "var(--muted)" },
          "&.cm-focused": { outline: "none" },
          ".cm-selectionBackground, &.cm-focused .cm-selectionBackground, ::selection": {
            backgroundColor: "#38414d",
          },
          ".cm-cursor": { borderLeftColor: "var(--accent)" },
          ".cm-searchMatch": { backgroundColor: "#3a3320" },
          ".cm-searchMatch.cm-searchMatch-selected": { backgroundColor: "#5a4a1e" },
          ".cm-panels": {
            position: "sticky",
            top: "var(--head, 60px)",
            zIndex: "2",
            height: "0",
            overflow: "visible",
            backgroundColor: "transparent",
            border: "0",
            color: "var(--muted)",
          },
          // a grid, not flex: a shrink-to-fit flex box never breaks at the panel's <br>
          ".cm-panel.cm-search": {
            position: "absolute",
            top: "8px",
            right: "0",
            maxWidth: "100%",
            display: "grid",
            gridTemplateColumns: "repeat(7, auto)",
            justifyItems: "start",
            alignItems: "center",
            gap: "6px",
            padding: "8px 30px 8px 10px",
            backgroundColor: "var(--side)",
            border: "1px solid var(--line)",
            boxShadow: "0 8px 22px rgba(0, 0, 0, .5)",
            fontFamily: "var(--mono)",
            fontSize: "10px",
          },
          ".cm-panel.cm-search label": {
            display: "inline-flex",
            alignItems: "center",
            gap: "5px",
            margin: "0",
            color: "var(--dim)",
            fontSize: "10px",
            letterSpacing: ".06em",
          },
          ".cm-panel.cm-search input[type=checkbox]": {
            appearance: "none",
            WebkitAppearance: "none",
            width: "12px",
            height: "12px",
            margin: "0",
            padding: "0",
            border: "1px solid var(--line)",
            borderRadius: "0",
            backgroundColor: "var(--bg)",
            cursor: "pointer",
          },
          ".cm-panel.cm-search input[type=checkbox]:hover": { borderColor: "var(--muted)" },
          ".cm-panel.cm-search input[type=checkbox]:focus-visible, .cm-textfield:focus-visible": {
            outline: "1px solid var(--accent)",
            outlineOffset: "1px",
          },
          ".cm-button:focus-visible, .cm-panel.cm-search [name=close]:focus-visible": {
            outline: "1px solid var(--accent)",
            outlineOffset: "1px",
          },
          ".cm-panel.cm-search input[type=checkbox]:checked": {
            backgroundColor: "var(--accent)",
            borderColor: "var(--accent)",
            boxShadow: "inset 0 0 0 2px var(--side)",
          },
          ".cm-panel.cm-search label:has(input:checked)": { color: "var(--text)" },
          ".cm-panel.cm-search br": { display: "none" },
          ".cm-textfield": {
            padding: "4px 8px",
            border: "1px solid var(--line)",
            borderRadius: "0",
            backgroundColor: "var(--bg)",
            color: "var(--text)",
            fontFamily: "var(--mono)",
            fontSize: "11px",
          },
          ".cm-textfield:focus": { outline: "none", borderColor: "var(--accent)" },
          ".cm-button": {
            padding: "4px 10px",
            border: "1px solid var(--line)",
            borderRadius: "0",
            backgroundColor: "transparent",
            backgroundImage: "none",
            color: "var(--muted)",
            fontFamily: "var(--mono)",
            fontSize: "10px",
            letterSpacing: ".06em",
            cursor: "pointer",
          },
          ".cm-button:hover": { borderColor: "var(--accent)", color: "var(--accent)" },
          ".cm-button:active": { backgroundImage: "none", backgroundColor: "var(--surface)" },
          ".cm-panel.cm-search [name=close]": {
            top: "6px",
            right: "6px",
            display: "grid",
            placeItems: "center",
            width: "18px",
            height: "18px",
            padding: "0",
            border: "1px solid var(--line)",
            backgroundColor: "var(--bg)",
            color: "var(--muted)",
            fontSize: "14px",
            lineHeight: "1",
            cursor: "pointer",
          },
          ".cm-panel.cm-search [name=close]:hover": {
            borderColor: "var(--accent)",
            color: "var(--accent)",
          },
        },
        { dark: true },
      ),
    ],
  });
  mounted = view;
  dirty(false);
  if (save) save.onclick = () => write(view);
  holder.addEventListener("keydown", (event) => {
    if ((event.metaKey || event.ctrlKey) && event.key === "s") {
      event.preventDefault();
      write(view);
    }
  });
  // parse all now, or scrolling flashes plain text; the budget is a backstop
  forceParsing(view, view.state.doc.length, 150);
  if (line && line <= view.state.doc.lines) {
    const at = view.state.doc.line(line).from;
    view.dispatch({
      selection: { anchor: at },
      effects: EditorView.scrollIntoView(at, { y: "center" }),
    });
  }
  view.focus();
}
