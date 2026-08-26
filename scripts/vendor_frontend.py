#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0

"""Mirror the dashboard's ES modules from esm.sh into the package.

The dashboard runs beside a robot, on machines that are often off the internet, and a page
that fetches its editor from a CDN at load is a page that half-loads there. echarts and uPlot
were already vendored as plain scripts; these are ES modules, which import each other, so
copying one file is not enough -- the whole graph comes down and every specifier in it is
rewritten to point at the copy next to it.

    python scripts/vendor_frontend.py [--check]

`--check` re-fetches and reports what would change without writing, so CI can say when a
pinned version has drifted. Run it whenever an import URL in the frontend changes.
"""

from __future__ import annotations

import argparse
import re
import sys
import urllib.request
from pathlib import Path
from urllib.parse import urljoin, urlsplit

ROOT = Path(__file__).resolve().parent.parent
VENDOR = ROOT / "src/motion_spec/dashboard/frontend/vendor/esm"

# Every module the frontend imports by URL. Keep in step with the import sites; the versions
# are pinned here rather than in the pages, so one edit re-pins every copy.
ENTRIES = {
    "codemirror": "https://esm.sh/codemirror@6.0.1",
    "cm-view": "https://esm.sh/@codemirror/view@^6.0.0?target=es2022",
    "cm-search": "https://esm.sh/@codemirror/search@^6.0.0?target=es2022",
    "cm-language": "https://esm.sh/@codemirror/language@^6.0.0?target=es2022",
    "lezer-highlight": "https://esm.sh/@lezer/highlight@^1.0.0?target=es2022",
    "cm-clike": "https://esm.sh/@codemirror/legacy-modes@^6.0.0/mode/clike?target=es2022",
    "cm-javascript": (
        "https://esm.sh/@codemirror/legacy-modes@^6.0.0/mode/javascript?target=es2022"
    ),
    "sigma": "https://esm.sh/sigma@3.0.2?bundle",
    "graphology": "https://esm.sh/graphology@0.25.4?bundle",
    "forceatlas2": "https://esm.sh/graphology-layout-forceatlas2@0.10.1/worker?bundle",
}

# `from "x"`, `import("x")`, `export * from "x"` -- every specifier esm.sh writes. The keyword
# must stand on its own (`Array.from(...)` and `.import` appear all over minified bodies), and
# what it names must look like a module path, or a minified string argument reads as an import.
SPECIFIER = re.compile(
    r"""(?<![\w$.])(?:from|import)\s*\(?\s*["'](/[^"']*|\.{1,2}/[^"']*|https?://[^"']*)["']"""
)


def key(url: str) -> str:
    """One module, one identity. `@codemirror/view@^6.0.0` reached as an entry carries the
    `?target=` esm.sh was asked for and reached as a dependency carries none; they are the
    same file, and treating them as two writes one copy and links to the other."""
    return urlsplit(url)._replace(query="").geturl()


def local_path(url: str, name: str | None = None) -> Path:
    """Where one esm.sh URL lands under the vendor directory.

    An entry gets the short name the frontend imports it by, so a version bump moves files
    underneath without touching a single import string -- and so no page has to spell a range
    like `@^6.0.0` into a URL path. Everything below keeps esm.sh's own layout, which is what
    its internal references are written against.

    The query is dropped: it selects a build, and the path esm.sh redirects to already names
    the build it selected. An extensionless path gets `.mjs` so the server serves it as a
    module rather than as bytes of unknown kind.
    """
    if name:
        return VENDOR / f"{name}.mjs"
    path = urlsplit(url).path.lstrip("/")
    return VENDOR / (path if Path(path).suffix in (".mjs", ".js", ".css") else f"{path}.mjs")


def fetch(url: str) -> str:
    with urllib.request.urlopen(url, timeout=60) as response:  # noqa: S310 - pinned esm.sh
        return response.read().decode()


def mirror(entries: dict[str, str], seen: dict[str, tuple[Path, str]]) -> None:
    """Fetch every module and everything they import, rewriting each specifier to its copy.

    Every entry is queued before any graph is walked: `codemirror` imports the same view and
    language modules the frontend asks for by name, and whichever is reached first decides
    where the file lands. Entries first means a named file, every time.
    """
    # Where each module lands, decided before anything is fetched for the entries and on first
    # sight for the rest, so a link written into one file always names the file that exists.
    chosen = {key(url): local_path(url, name) for name, url in entries.items()}
    pending = list(entries.values())
    while pending:
        url = pending.pop()
        if key(url) in seen:
            continue
        text = fetch(url)
        here = chosen.setdefault(key(url), local_path(url))
        for specifier in set(SPECIFIER.findall(text)):
            if specifier.startswith((".", "data:")):
                continue   # already relative, or inline
            target = urljoin(url, specifier)
            if urlsplit(target).netloc != "esm.sh":
                print(f"  ! leaves esm.sh, left alone: {specifier}", file=sys.stderr)
                continue
            pending.append(target)
            there = chosen.setdefault(key(target), local_path(target))
            relative = Path(*[".."] * (len(here.relative_to(VENDOR).parts) - 1))
            replacement = (relative / there.relative_to(VENDOR)).as_posix()
            text = text.replace(f'"{specifier}"', f'"./{replacement}"')
            text = text.replace(f"'{specifier}'", f"'./{replacement}'")
        seen[key(url)] = (here, text)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="report drift, write nothing")
    args = parser.parse_args()

    seen: dict[str, tuple[Path, str]] = {}
    for name, url in ENTRIES.items():
        print(f"{name}: {url}")
    mirror(ENTRIES, seen)

    changed = []
    for path, text in seen.values():
        if not path.exists() or path.read_text() != text:
            changed.append(path.relative_to(VENDOR))
            if not args.check:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text)
    total = sum(len(text.encode()) for _path, text in seen.values())
    print(f"\n{len(seen)} modules, {total / 1e6:.1f} MB")
    if not changed:
        print("up to date")
        return 0
    print(f"{'would change' if args.check else 'written'}: {len(changed)}")
    for path in sorted(changed)[:20]:
        print(f"  {path}")
    return 1 if args.check else 0


if __name__ == "__main__":
    raise SystemExit(main())
