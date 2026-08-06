# SPDX-License-Identifier: MPL-2.0
"""The layer boundary: `rdf_parser` states the model, only the view knows the target language."""

from __future__ import annotations

import ast
from pathlib import Path

from motion_spec.rdf_parser import ir

# Target-language syntax. A string literal in the lowering holding any of these is C++ that the
# templates can neither see nor change -- the projection it belongs to would be a second view.
FORBIDDEN = ("KDL::", "std::", "->", "shared.", "#include", "nullptr")

# Prose, not emitted text: docstrings say "a -> b" about the model and are not the view's business.
_HAS_DOCSTRING = (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)


def _docstring_ids(tree: ast.AST) -> set[int]:
    return {
        id(node.body[0].value)
        for node in ast.walk(tree)
        if isinstance(node, _HAS_DOCSTRING)
        and node.body
        and isinstance(node.body[0], ast.Expr)
        and isinstance(node.body[0].value, ast.Constant)
        and isinstance(node.body[0].value.value, str)
    }


def test_no_target_language_syntax_in_the_rdf_parser() -> None:
    violations = []
    for path in sorted(Path(ir.__file__).parent.glob("*.py")):
        tree = ast.parse(path.read_text())
        docstrings = _docstring_ids(tree)
        for node in ast.walk(tree):
            if not isinstance(node, ast.Constant) or not isinstance(node.value, str):
                continue
            if id(node) in docstrings:
                continue
            for token in FORBIDDEN:
                if token in node.value:
                    violations.append(f"{path.name}:{node.lineno}: {token!r} in {node.value!r}")
    assert not violations, "target syntax in layer B (belongs in templates/):\n" + "\n".join(
        violations
    )
