# SPDX-License-Identifier: MPL-2.0
"""Shared look for the path diagrams rendered by Sphinx's matplotlib plot directive.

Every diagram in this package plots the actual closed-form equations documented in
concepts.md (not a hand-drawn approximation), so a figure can never drift from the math
it illustrates -- regenerating it is a doc build, not a redraw.
"""

import matplotlib.pyplot as plt
import numpy as np

INK = "#1f2933"
ACCENT = "#2f6fed"
GUIDE = "#9aa5b1"
MEASURED = "#e2543a"
CAPTION = "#52606d"

plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.size": 12,
        "axes.edgecolor": INK,
        "text.color": INK,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
    }
)


def rot2d(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def clean_axes(ax, title=None, note=None, pad=0.35):
    """Equal-aspect, spine-free axes with an optional title and caption note."""
    ax.set_aspect("equal")
    ax.axis("off")
    if title:
        ax.set_title(title, fontweight="bold", color=INK, pad=10)
    if note:
        x0, x1 = ax.get_xlim()
        y0, _ = ax.get_ylim()
        ax.text(
            (x0 + x1) / 2,
            y0 - pad,
            note,
            ha="center",
            va="top",
            fontsize=10,
            color=CAPTION,
        )


def arrow(ax, p_from, p_to, color=INK, lw=2, style="-|>"):
    ax.annotate(
        "",
        xy=p_to,
        xytext=p_from,
        arrowprops=dict(arrowstyle=style, color=color, lw=lw, shrinkA=0, shrinkB=0),
    )


def dot(ax, p, color=INK, ms=5):
    ax.plot(*p, marker="o", color=color, ms=ms, mec="none")


def out_of_page_marker(ax, p, color=ACCENT):
    """The circled-dot symbol for a vector pointing out of the drawing plane."""
    ax.plot(*p, marker="o", mfc="none", mec=color, ms=11, mew=1.5)
    ax.plot(*p, marker=".", color=color, ms=5)
