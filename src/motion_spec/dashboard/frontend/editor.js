// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

/**
 * The editor itself: CodeMirror, fetched once, themed to the page, and told which lines
 * have changed since git HEAD.
 */

import { $, api, post, snack } from "./core.js";

// The file, in an editor rather than a rendering of one: arrows move a cursor through the
// text, the line under it is marked by the editor itself, and small changes can be made and
// saved here instead of round-tripping through a terminal. CodeMirror does all of that; the
// module is fetched once and kept.
export let codemirror = null;

// The view the page is currently showing. Anything that can point at a line of the open file
// jumps to it through `revealLine`, the same dispatch the syntax banner already makes.
let mounted = null;

export function revealLine(number) {
  if (!mounted || !(number >= 1) || number > mounted.state.doc.lines) return;
  const line = mounted.state.doc.line(number);
  mounted.dispatch({ selection: { anchor: line.from }, scrollIntoView: true });
  mounted.focus();
}

// Vendored, not fetched: the dashboard runs beside a robot, on machines that are often off
// the internet, and a page that pulls its editor from a CDN at load half-loads there.
// `scripts/vendor_frontend.py` mirrors these and pins the versions; the names are stable, so
// a version bump moves files underneath without touching a line here.
//
// The file `codemirror` itself imports its view from. Asking for the same one gets the same
// module instance; a different spelling of the same package (@codemirror/view@6.26.3, say)
// is a second copy of the library, whose decorations the first copy quietly ignores.
export const CM_VIEW = "./vendor/esm/cm-view.mjs";

// Same rule for search: `basicSetup` already carries it, and this is where its panel is
// configured -- so it has to be the copy of the package that setup itself imported.
export const CM_SEARCH = "./vendor/esm/cm-search.mjs";

// And again for the language support the highlighting is hung off: `basicSetup` already
// carries the highlighter, so the mode has to come from the copy of the package it uses.
export const CM_LANGUAGE = "./vendor/esm/cm-language.mjs";

// The tag vocabulary a highlight style is written against, which the language package takes
// but does not re-export.
export const CM_TAGS = "./vendor/esm/lezer-highlight.mjs";

// The DSLs are what this editor is for, so their mode is the default. Generated output is
// not a DSL -- it is the C++ and JSON the generator wrote -- and reading it in a mode that
// knows neither leaves keywords, types and preprocessor lines all the same colour.
// Legacy modes, not the lang-* packs: a lang-* pack builds its LanguageSupport out of its own
// copy of @codemirror/language, which the copy basicSetup uses ignores -- the file then renders
// with no highlighting at all. A legacy mode is a plain parser object with no CodeMirror
// dependency, so it goes through the same StreamLanguage `structuralMode` already uses.
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

// A pack this machine cannot fetch is not a reason to refuse the file: it reads in the
// structural mode, the same as everything else without a pack of its own.
export async function languageFor(name, StreamLanguage) {
  const wanted = LANGUAGE_PACKS.find(({ suffixes }) => suffixes.test(name));
  if (!wanted) return structuralMode(StreamLanguage);
  if (!packs.has(wanted.url)) {
    packs.set(wanted.url, import(wanted.url).then(wanted.parser).catch(() => null));
  }
  const parser = await packs.get(wanted.url);
  return parser ? StreamLanguage.define(parser) : structuralMode(StreamLanguage);
}

export async function editorModule() {
  codemirror ??= Promise.all([
    import("./vendor/esm/codemirror.mjs"),
    import(CM_VIEW),
    import(CM_SEARCH),
    import(CM_LANGUAGE),
    import(CM_TAGS),
  ]).then(([setup, view, find, language, highlight]) => (
    { ...setup, ...view, ...find, ...language, ...highlight }
  ));
  return codemirror;
}

// Tokyo Night (night). The page's own palette is warm and deliberately narrow -- one accent
// for everything that stands out -- which is why the stock highlighter clashed with it; code
// wants more colours than the surrounding page does, so this is a palette of its own.
export const TOKYO_NIGHT = {
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

export function tokyoNightStyle(HighlightStyle, tags) {
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

// Highlighting by shape rather than by vocabulary: one mode for every DSL here, because what
// they share is punctuation, not keywords. A word is only a keyword if it opens a block or
// labels a field -- which the text says structurally, so nothing has to keep a list of terms
// per language and go stale when a grammar gains one.
export function structuralMode(StreamLanguage) {
  return StreamLanguage.define({
    // `kind` is whether the word just read was one that declares, so the next word is the
    // name it declares; `depth` keeps the parameters inside `(ns=app)` from being mistaken
    // for that name, since they sit between the two.
    startState: () => ({ kind: false, depth: 0 }),
    token(stream, state) {
      if (stream.match(/^\/\/.*/)) return "comment";
      if (stream.match(/^\/\*/)) {
        stream.match(/^[\s\S]*?(\*\/|$)/);
        return "comment";
      }
      // Both quotes: .scenex spells its asset paths with single ones, .bdd its namespaces.
      if (stream.match(/^"(?:[^"\\]|\\.)*"?/)) return "string";
      if (stream.match(/^'(?:[^'\\]|\\.)*'?/)) return "string";
      // A reference to something named elsewhere: <kinova.joint_3>, the DSLs' one sigil.
      if (stream.match(/^<[^<>\n]*>?/)) return "typeName";
      // A number carries its unit with it -- 0.05 m/s, -45.0 deg, 100.0 Hz -- and the unit is
      // part of the value, not a word beside it.
      if (stream.match(/^-?\d+(?:\.\d+)?(?:e-?\d+)?/)) {
        stream.match(/^\s*[a-zA-Z]+(?:\^-?\d)?(?:\/[a-zA-Z]+(?:\^-?\d)?)*\b/);
        return "number";
      }
      // A label may be several words -- `time extractor:`, `entity mapper:` -- so the whole
      // run up to the colon is one name, not a word coloured beside an uncoloured one.
      if (stream.match(/^[A-Za-z_][\w-]*(?:[ \t]+[A-Za-z_][\w-]*)*(?=[ \t]*:)/)) {
        state.kind = false;
        return "propertyName";
      }
      // `KIND... (ns=..) NAME` is how every one of these languages declares a thing, and the
      // bracket is what divides it: whatever many words came before is the kind, whatever
      // follows is the name. Nothing else in these grammars puts a name after a bracket, so
      // it needs no trailing `{` to be sure -- `obs policy (ns=..) trin` declares as much as
      // `observation (ns=..) pose {` does.
      if (stream.match(
        /^[A-Za-z_][\w-]*(?:[ \t]+[A-Za-z_][\w-]*)*(?=[ \t]*\([^)]*\)[ \t]*[A-Za-z_][\w-]*)/,
      )) {
        state.kind = true;
        return "keyword";
      }
      const word = stream.match(/^[A-Za-z_][\w-]*/);
      if (word) {
        // Inside `(...)` this is a parameter, not the name being declared: leave the flag
        // standing so it still belongs to the name that follows the closing bracket.
        if (state.depth > 0) return null;
        const kind = state.kind;
        state.kind = false;
        // The word after a declaring word is what is being declared -- the name in
        // `linear-velocity descend-vel =` or in `pid ctrl-home-position {`.
        if (kind) return "variableName";
        // A word declares when a name and then `=` or `{` follow it, which is what separates
        // `linear-velocity descend-vel =` from `equal to <...>`: same shape of two words, but
        // only one of them is a declaration. A word sitting straight against `{` opens a
        // block with no name of its own -- `context {`, `while {`.
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

// Which lines of `b` are additions or changes against `a`, by Myers' O(ND) shortest-edit-script
// algorithm -- the same family git diff itself uses. Cost tracks the number of *differences*,
// not the file size, so a small edit stays cheap even in a large file no matter where in it the
// edit sits; a same-size table indexed purely by line counts (what this replaced) can't offer
// that; it has to give up past some size and mark everything changed instead of guessing.
function myersInsertions(a, b) {
  const n = a.length;
  const m = b.length;
  const max = n + m;
  let v = new Map([[1, 0]]);
  const trace = [];
  let dEnd = -1;
  outer:
  for (let d = 0; d <= max; d += 1) {
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
      while (x < n && y < m && a[x] === b[y]) { x += 1; y += 1; }
      next.set(k, x);
      if (x >= n && y >= m) { dEnd = d; break outer; }
    }
    v = next;
  }
  const added = new Set();
  let x = n;
  let y = m;
  for (let d = dEnd; d > 0; d -= 1) {
    const vPrev = trace[d];
    const k = x - y;
    const prevK = k === -d || (k !== d && (vPrev.get(k - 1) ?? -1) < (vPrev.get(k + 1) ?? -1))
      ? k + 1 : k - 1;
    const prevX = vPrev.get(prevK) ?? 0;
    const prevY = prevX - prevK;
    while (x > prevX && y > prevY) { x -= 1; y -= 1; }
    if (x === prevX) { y -= 1; added.add(y); } else { x -= 1; }
  }
  return added;
}

export async function mountEditor(holder, source, text, readOnly = false, gitHead = null, line = null) {
  let cm;
  mounted = null;   // whatever was open is gone; nothing may jump into it
  try {
    cm = await editorModule();
  } catch (error) {
    // No editor to be had: the file still has to be readable.
    holder.textContent = text;
    holder.classList.add("plain-source");
    return snack(`code editor unavailable (${error.message}) — showing plain text`);
  }
  const {
    EditorView, basicSetup, Decoration, ViewPlugin, search,
    StreamLanguage, HighlightStyle, syntaxHighlighting, tags, forceParsing,
  } = cm;
  // Absent in a viewer that cannot save -- generated output is read, never written back.
  const save = $("#save-source");
  // A button for something there is nothing to do is clutter: it arrives with the first edit.
  const dirty = (is) => {
    if (!save) return;
    save.hidden = !is;
    save.textContent = "save";
  };
  const write = async (target) => {
    if (!save || save.hidden) return true;
    const written = target.state.doc.toString();
    try {
      save.textContent = "saving…";
      const { check } = await post("/api/source", { path: source, text: written });
      // The comparison stays against git, not against what was just written: an uncommitted
      // save is still a change from HEAD's point of view.
      save.hidden = false;
      save.textContent = "saved";
      setTimeout(() => dirty(false), 1200);
      // The file is written either way; a model that no longer parses is worth saying out
      // loud, and worth pointing at, rather than left to be discovered by a failed generate.
      showCheck(target, check);
    } catch (error) {
      snack(error.message);
      dirty(true);
    }
    return true;
  };
  // Where the parse gave up, marked on the line it names and said in the bar above the file.
  // The verdict belongs to the text that was checked: the next edit clears it rather than
  // leaving a stale mark pointing at a line that has since moved or been fixed.
  const badLine = Decoration.line({ class: "cm-badLine" });
  const problem = { line: null };
  const badMarks = (doc) => (problem.line && problem.line <= doc.lines
    ? Decoration.set([badLine.range(doc.line(problem.line).from)])
    : Decoration.none);
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
    target.dispatch({});   // nothing to change; it asks the mark to be redrawn
    const banner = $(".syntax-state");
    if (!banner) return;
    banner.hidden = !check || check.ok;
    if (!check || check.ok) return;
    banner.textContent = check.line ? `line ${check.line}: ${check.message}` : check.message;
    banner.onclick = () => {
      if (!check.line || check.line > target.state.doc.lines) return;
      const line = target.state.doc.line(check.line);
      target.dispatch({ selection: { anchor: line.from }, scrollIntoView: true });
      target.focus();
    };
  };
  // Where the find box parks: just under the file header, which is as tall as its own contents.
  holder.style.setProperty("--head", `${$(".viewer-top")?.offsetHeight ?? 60}px`);
  // Marks are measured against git HEAD, not against what the file looked like when this
  // session opened it -- so a file already dirty in the working tree shows that on open, and
  // a fresh commit is the only thing that ever clears them. No commit yet (new file, no repo,
  // no git installed) falls back to the open-time text, which marks nothing.
  const saved = { lines: (gitHead ?? text).split("\n") };
  const changedLine = Decoration.line({ class: "cm-changedLine" });
  const marked = (doc) => {
    const now = [];
    for (let n = 1; n <= doc.lines; n += 1) now.push(doc.line(n).text);
    const was = saved.lines;
    // The matching ends are not part of any change; trimming them first keeps what Myers has
    // to search small in the common case of one edit in an otherwise-untouched file.
    let head = 0;
    while (head < now.length && head < was.length && now[head] === was[head]) head += 1;
    let tail = 0;
    while (
      tail < now.length - head
      && tail < was.length - head
      && now[now.length - 1 - tail] === was[was.length - 1 - tail]
    ) {
      tail += 1;
    }
    const before = was.slice(head, was.length - tail);
    const after = now.slice(head, now.length - tail);
    if (!after.length) return new Set();
    return new Set([...myersInsertions(before, after)].map((index) => head + index + 1));
  };
  const changes = (doc) => Decoration.set(
    [...marked(doc)].sort((left, right) => left - right).map((n) => changedLine.range(doc.line(n).from)),
  );
  const trackChanges = ViewPlugin.fromClass(
    class {
      constructor(target) {
        this.decorations = changes(target.state.doc);
      }

      // The baseline is fixed to git HEAD for the life of this editor, so only the text
      // itself moving is worth recomputing over -- not the updates that merely scrolled.
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
        // Ctrl-F opens at the foot of the file by default, which on a page-scrolled editor is
        // nowhere in particular.
        search({ top: true }),
        await languageFor(source, StreamLanguage),
        // After basicSetup, whose own highlighter is registered as a fallback: this one is
        // asked first, and the stock palette only answers for tags this does not name.
        syntaxHighlighting(tokyoNightStyle(HighlightStyle, tags)),
        trackChanges,
        trackProblem,
        EditorView.lineWrapping,
        EditorView.editable.of(!readOnly),
        EditorView.updateListener.of((update) => {
          if (update.docChanged) dirty(true);
        }),
        EditorView.theme({
          "&": { color: "var(--text)", backgroundColor: "transparent", fontSize: "12.5px" },
          ".cm-content": { fontFamily: "var(--code)", padding: "0" },
          ".cm-gutters": {
            backgroundColor: "transparent", color: "var(--dim)", border: "0",
            fontFamily: "var(--code)",
          },
          // Translucent on purpose: the selection is drawn in a layer behind the lines, so an
          // opaque active line would hide every selection made inside it.
          ".cm-activeLine": { backgroundColor: "rgba(255, 255, 255, .06)" },
          // Unsaved lines carry a mark down their edge, the way an editor's gutter does.
          ".cm-changedLine": { boxShadow: "inset 2px 0 0 var(--accent)" },
          // Where the parser stopped: the one line to look at, tinted rather than edge-marked
          // so it does not read as another uncommitted change.
          ".cm-badLine": { backgroundColor: "rgba(224, 122, 95, .18)" },
          ".cm-activeLineGutter": { backgroundColor: "transparent", color: "var(--muted)" },
          "&.cm-focused": { outline: "none" },
          ".cm-selectionBackground, &.cm-focused .cm-selectionBackground, ::selection": {
            backgroundColor: "#38414d",
          },
          ".cm-cursor": { borderLeftColor: "var(--accent)" },
          ".cm-searchMatch": { backgroundColor: "#3a3320" },
          ".cm-searchMatch.cm-searchMatch-selected": { backgroundColor: "#5a4a1e" },
          // Search arrives as bare browser widgets in a bar across the top. It belongs in the
          // corner instead, out of the way of the text it is searching: a box of no height,
          // stuck to the top so scrolling the results keeps it in sight.
          ".cm-panels": {
            position: "sticky", top: "var(--head, 60px)", zIndex: "2",
            height: "0", overflow: "visible",
            backgroundColor: "transparent", border: "0", color: "var(--muted)",
          },
          // Two rows: find, its three buttons and its three options above; replace and its two
          // buttons below. Seven columns because the first row has seven controls -- the panel
          // is built by the search extension, and this is the shape it hands over. A grid, not
          // a wrapping flex row: the box is sized to its contents, and a shrink-to-fit flex
          // container measures as if nothing wrapped, so the panel's own <br> never breaks.
          ".cm-panel.cm-search": {
            position: "absolute", top: "8px", right: "0", maxWidth: "100%",
            display: "grid", gridTemplateColumns: "repeat(7, auto)",
            justifyItems: "start", alignItems: "center", gap: "6px",
            padding: "8px 30px 8px 10px",
            backgroundColor: "var(--side)", border: "1px solid var(--line)",
            boxShadow: "0 8px 22px rgba(0, 0, 0, .5)",
            fontFamily: "var(--mono)", fontSize: "10px",
          },
          ".cm-panel.cm-search label": {
            display: "inline-flex", alignItems: "center", gap: "5px",
            margin: "0", color: "var(--dim)", fontSize: "10px", letterSpacing: ".06em",
          },
          // A native checkbox is a white box whatever the page around it is; this is the same
          // box the rest of the bar's controls are drawn as, filled when it is on.
          ".cm-panel.cm-search input[type=checkbox]": {
            appearance: "none", WebkitAppearance: "none",
            width: "12px", height: "12px", margin: "0", padding: "0",
            border: "1px solid var(--line)", borderRadius: "0",
            backgroundColor: "var(--bg)", cursor: "pointer",
          },
          ".cm-panel.cm-search input[type=checkbox]:hover": { borderColor: "var(--muted)" },
          // Drawn from scratch, so the ring the browser would have drawn is drawn too.
          ".cm-panel.cm-search input[type=checkbox]:focus-visible, .cm-textfield:focus-visible": {
            outline: "1px solid var(--accent)", outlineOffset: "1px",
          },
          ".cm-button:focus-visible, .cm-panel.cm-search [name=close]:focus-visible": {
            outline: "1px solid var(--accent)", outlineOffset: "1px",
          },
          ".cm-panel.cm-search input[type=checkbox]:checked": {
            backgroundColor: "var(--accent)", borderColor: "var(--accent)",
            boxShadow: "inset 0 0 0 2px var(--side)",
          },
          ".cm-panel.cm-search label:has(input:checked)": { color: "var(--text)" },
          // The break is the grid's business now; a hidden element is not a grid item at all,
          // so the replace field starts the second row by itself.
          ".cm-panel.cm-search br": { display: "none" },
          ".cm-textfield": {
            padding: "4px 8px", border: "1px solid var(--line)", borderRadius: "0",
            backgroundColor: "var(--bg)", color: "var(--text)",
            fontFamily: "var(--mono)", fontSize: "11px",
          },
          ".cm-textfield:focus": { outline: "none", borderColor: "var(--accent)" },
          ".cm-button": {
            padding: "4px 10px", border: "1px solid var(--line)", borderRadius: "0",
            backgroundColor: "transparent", backgroundImage: "none", color: "var(--muted)",
            fontFamily: "var(--mono)", fontSize: "10px", letterSpacing: ".06em", cursor: "pointer",
          },
          ".cm-button:hover": { borderColor: "var(--accent)", color: "var(--accent)" },
          ".cm-button:active": { backgroundImage: "none", backgroundColor: "var(--surface)" },
          // Closing is a control of its own, not a stray character in the corner.
          ".cm-panel.cm-search [name=close]": {
            top: "6px", right: "6px",
            display: "grid", placeItems: "center", width: "18px", height: "18px", padding: "0",
            border: "1px solid var(--line)", backgroundColor: "var(--bg)",
            color: "var(--muted)", fontSize: "14px", lineHeight: "1", cursor: "pointer",
          },
          ".cm-panel.cm-search [name=close]:hover": {
            borderColor: "var(--accent)", color: "var(--accent)",
          },
        }, { dark: true }),
    ],
  });
  mounted = view;
  dirty(false);
  if (save) save.onclick = () => write(view);
  // The binding people expect from an editor, without importing a second package for it.
  holder.addEventListener("keydown", (event) => {
    if ((event.metaKey || event.ctrlKey) && event.key === "s") {
      event.preventDefault();
      write(view);
    }
  });
  // Tokenising stops at whatever the viewport needed, so scrolling into the rest of the file
  // arrives before its colours do -- a line of plain text for a frame, then the highlight.
  // These files run to hundreds of lines, not hundreds of thousands, and the whole of one
  // tokenises in single-digit milliseconds: parse it all now and there is nothing left to
  // catch up on later. The budget is what keeps that promise honest on a file big enough to
  // break it, which then simply falls back to highlighting as it goes.
  forceParsing(view, view.state.doc.length, 150);
  // Opened at a line someone was pointed at: put the cursor there and scroll it into the middle.
  if (line && line <= view.state.doc.lines) {
    const at = view.state.doc.line(line).from;
    view.dispatch({ selection: { anchor: at }, effects: EditorView.scrollIntoView(at, { y: "center" }) });
  }
  view.focus();
}
