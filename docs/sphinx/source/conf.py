# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

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
