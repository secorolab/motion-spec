# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

from pathlib import Path

from pygments.lexers.special import TextLexer
from sphinx.highlighting import lexers

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "motion-spec"
copyright = "2025, Sven Schneider, Vamsi Kalagaturu"
author = "Sven Schneider, Vamsi Kalagaturu"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["myst_parser", "sphinx.ext.graphviz", "sphinx.ext.mathjax", "sphinx.ext.todo"]
myst_enable_extensions = ["colon_fence", "deflist"]
myst_heading_anchors = 3
lexers["robmot"] = TextLexer()

templates_path = ["_templates"]
exclude_patterns = []

numfig = True
math_numfig = True
numfig_format = {
    "figure": "Figure %s",
    "table": "Table %s",
    "code-block": "Listing %s",
    "section": "Section %s"
}

todo_include_todos = True

# -- Options for LaTeX output ------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-latex-output

latex_elements = {
    "preamble": r"\newcommand{\vect}{\boldsymbol}"
} 

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "furo"
html_title = "motion-spec"
html_static_path = ["_static"]

# -- MathJax -----------------------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/math.html

mathjax_path = "https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js"

# -- DSL path diagrams --------------------------------------------------------
# The dsl/concepts.md "Paths" figures are plots of the actual documented equations
# (see dsl/_plots/*.py), not hand-drawn approximations. Regenerate them into
# _static/dsl-paths/ before every build so a `{figure}` reference always finds
# current output -- this runs the scripts directly rather than through
# matplotlib's own Sphinx plot directive, whose caption/cross-reference wiring
# doesn't work through MyST's RST compatibility layer.


def _generate_dsl_path_diagrams(app):
    import runpy
    import sys

    plots_dir = Path(__file__).parent / "dsl" / "_plots"
    output_dir = Path(__file__).parent / "_static" / "dsl-paths"
    output_dir.mkdir(parents=True, exist_ok=True)

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    sys.path.insert(0, str(plots_dir))
    try:
        for name in ("path_kinds", "path_frame", "speed_profile"):
            plt.close("all")
            runpy.run_path(str(plots_dir / f"{name}.py"), run_name="__main__")
            plt.savefig(output_dir / f"{name}.svg")
            plt.close("all")

        # One small standalone figure per path kind, reusing the same panel-drawing
        # functions as the combined overview above -- placed next to each kind's own
        # excerpt and equation rather than making the reader scroll back to the grid.
        for kind in ("lerp", "circle", "arc", "helix", "figure8"):
            plt.close("all")
            runpy.run_path(
                str(plots_dir / "path_kind_single.py"),
                init_globals={"PANEL_NAME": kind},
                run_name="__main__",
            )
            plt.savefig(output_dir / f"path_kind_{kind}.svg")
            plt.close("all")
    finally:
        sys.path.remove(str(plots_dir))


def setup(app):
    app.connect("builder-inited", _generate_dsl_path_diagrams)
