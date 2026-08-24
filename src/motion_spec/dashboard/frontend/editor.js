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

// The URL `codemirror` itself imports its view from. Asking for the same string gets the same
// module instance; a different spelling of the same package (@codemirror/view@6.26.3, say)
// is a second copy of the library, whose decorations the first copy quietly ignores.
export const CM_VIEW = "https://esm.sh/@codemirror/view@^6.0.0?target=es2022";

// Same rule for search: `basicSetup` already carries it, and this is where its panel is
// configured -- so it has to be the copy of the package that setup itself imported.
export const CM_SEARCH = "https://esm.sh/@codemirror/search@^6.0.0?target=es2022";

export async function editorModule() {
  codemirror ??= Promise.all([
    import("https://esm.sh/codemirror@6.0.1"),
    import(CM_VIEW),
    import(CM_SEARCH),
  ]).then(([setup, view, find]) => ({ ...setup, ...view, ...find }));
  return codemirror;
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

export async function mountEditor(holder, source, text, readOnly = false, gitHead = null) {
  let cm;
  try {
    cm = await editorModule();
  } catch (error) {
    // No editor to be had: the file still has to be readable.
    holder.textContent = text;
    holder.classList.add("plain-source");
    return snack(`code editor unavailable (${error.message}) — showing plain text`);
  }
  const { EditorView, basicSetup, Decoration, ViewPlugin, search } = cm;
  const save = $("#save-source");
  // A button for something there is nothing to do is clutter: it arrives with the first edit.
  const dirty = (is) => {
    save.hidden = !is;
    save.textContent = "save";
  };
  const write = async (target) => {
    if (save.hidden) return true;
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
  dirty(false);
  save.onclick = () => write(view);
  // The binding people expect from an editor, without importing a second package for it.
  holder.addEventListener("keydown", (event) => {
    if ((event.metaKey || event.ctrlKey) && event.key === "s") {
      event.preventDefault();
      write(view);
    }
  });
  view.focus();
}
