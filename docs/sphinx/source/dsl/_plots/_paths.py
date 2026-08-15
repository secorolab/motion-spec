"""Panel-drawing functions for the five path kinds, shared by the combined overview
figure (path_kinds.py) and the per-kind standalone figures (path_kind_*.py) so both are
generated from the exact same plotting code -- never redrawn twice."""

import numpy as np

from _style import ACCENT, GUIDE, INK, arrow, clean_axes, dot, out_of_page_marker, rot2d

S = np.linspace(0, 1, 200)


def draw_lerp(ax):
    # p(s) = (1-s) p0 + s p1  (eq:path-lerp, position term)
    p0, p1 = np.array([0.0, 0.0]), np.array([2.6, 1.9])
    pts = np.outer(1 - S, p0) + np.outer(S, p1)
    assert np.allclose(pts[0], p0) and np.allclose(pts[-1], p1)

    arrow(ax, p0, p1)
    dot(ax, p0)
    dot(ax, p1)
    sm = 0.4
    pm = (1 - sm) * p0 + sm * p1
    dot(ax, pm, color=ACCENT, ms=6)
    ax.text(pm[0] + 0.08, pm[1] + 0.08, "$p(s)$", color=ACCENT)
    ax.text(p0[0], p0[1] - 0.28, "$p_0$", ha="center")
    ax.text(p1[0], p1[1] + 0.22, "$p_1$", ha="center")
    ax.set_xlim(-0.6, 3.2)
    ax.set_ylim(-0.7, 2.5)
    clean_axes(ax, "lerp", r"straight line, $p_0 \to p_1$")


def draw_circle(ax):
    # p(s) = c + Rot(n, 2*pi*s)(p0 - c)  (eq:path-circle)
    c = np.array([0.0, 0.0])
    r = 1.0
    p0 = c + np.array([0.0, r])
    circle_pts = np.array([c + rot2d(2 * np.pi * s) @ (p0 - c) for s in S])
    assert np.allclose(circle_pts[0], p0) and np.allclose(circle_pts[-1], p0, atol=1e-6)

    ax.plot(circle_pts[:, 0], circle_pts[:, 1], color=INK, lw=2)
    ax.plot([c[0], p0[0]], [c[1], p0[1]], "--", color=GUIDE)
    dot(ax, p0)
    dot(ax, c, ms=3)
    out_of_page_marker(ax, c + np.array([0.35, 0.0]))
    ax.text(c[0] + 0.55, c[1] - 0.05, "$n$", color=ACCENT)
    ax.text(c[0] - 0.18, c[1] - 0.05, "$c$", ha="right")
    ax.text(p0[0], p0[1] + 0.18, "$p_0$", ha="center")
    arc_s = np.linspace(0.0, 0.16, 20)
    arc_pts = np.array([c + rot2d(2 * np.pi * s) @ (p0 - c) for s in arc_s])
    ax.plot(arc_pts[:, 0], arc_pts[:, 1], color=ACCENT, lw=2)
    arrow(ax, arc_pts[-2], arc_pts[-1], color=ACCENT)
    ax.text(arc_pts[-1, 0] + 0.12, arc_pts[-1, 1] + 0.05, r"$2\pi s$", color=ACCENT)
    ax.set_xlim(-1.4, 1.4)
    ax.set_ylim(-1.3, 1.4)
    clean_axes(ax, "circle", r"center $c$, radius $|p_0-c|$, normal $n$")


def draw_arc(ax):
    # derived quantities from eq:path-arc-derived, then eq:path-arc
    p0, p1 = np.array([-1.4, 0.0]), np.array([1.4, 0.0])
    a = 0.9  # sagitta
    q = p1 - p0
    length = np.linalg.norm(q)
    m = length / 2
    qhat = q / length
    b = np.array([-qhat[1], qhat[0]])  # in-plane stand-in for n x qhat
    radius = (a**2 + m**2) / (2 * a)
    d = (a**2 - m**2) / (2 * a)
    c = (p0 + p1) / 2 + d * b
    theta = 2 * np.arccos(-d / radius)
    arc_pts = np.array([c + rot2d(-theta * s) @ (p0 - c) for s in S])
    assert np.allclose(arc_pts[0], p0) and np.allclose(arc_pts[-1], p1, atol=1e-6)

    ax.plot(arc_pts[:, 0], arc_pts[:, 1], color=INK, lw=2)
    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], "--", color=GUIDE)
    dot(ax, p0)
    dot(ax, p1)
    mid = (p0 + p1) / 2
    apex = arc_pts[len(arc_pts) // 2]
    arrow(ax, mid, apex, color=ACCENT)
    dot(ax, mid, color=GUIDE, ms=3)
    ax.text(apex[0] + 0.1, (mid[1] + apex[1]) / 2, "$a$", color=ACCENT)
    ax.text(mid[0], mid[1] - 0.22, "$m$", ha="center", fontsize=10, color="#52606d")
    ax.text(p0[0], p0[1] - 0.24, "$p_0$", ha="center")
    ax.text(p1[0], p1[1] - 0.24, "$p_1$", ha="center")
    ax.set_xlim(-2.0, 2.0)
    ax.set_ylim(-0.6, 1.3)
    clean_axes(ax, "arc", r"chord $\ell$, sagitta $a$")


def draw_helix(ax):
    # p(s) = c + Rot(u, 2*pi*N*s)(p0 - c) + u*h*N*s  (eq:path-helix), side projection
    N = 2.5
    radius = 0.55
    x = radius * np.cos(2 * np.pi * N * S)
    z = S  # rise normalized to 1 over the full path; h is the per-revolution scale below
    assert np.isclose(x[0], radius) and np.isclose(z[0], 0.0)

    ax.plot(x, z, color=INK, lw=2)
    ax.plot([0, 0], [-0.05, 1.12], "--", color=GUIDE)
    arrow(ax, (0, 1.06), (0, 1.18))
    ax.text(0.07, 1.19, "$u$", va="bottom")
    dot(ax, (x[0], z[0]))
    ax.text(x[0] + 0.1, z[0] - 0.05, "$p_0$", va="top")
    s1, s2 = 1 / N, 2 / N  # one full revolution apart
    xb = radius + 0.35
    arrow(ax, (xb, s1), (xb, s2), color=ACCENT, style="<|-|>")
    ax.text(xb + 0.12, (s1 + s2) / 2, "$h$", color=ACCENT, va="center")
    ax.set_xlim(-1.1, 1.5)
    ax.set_ylim(-0.2, 1.3)
    clean_axes(ax, "helix", r"axis $u$, pitch $h$, $N$ turns")


def draw_figure8(ax):
    # Gerono form of eq:path-figure-eight
    radius = 1.0
    tau = 2 * np.pi * S + np.pi / 2
    u, v = np.array([1.0, 0.0]), np.array([0.0, 1.0])
    anchor = np.array([0.0, 0.0])
    fig8 = anchor + radius * np.outer(np.cos(tau), u) + radius * np.outer(
        np.sin(tau) * np.cos(tau), v
    )
    assert np.allclose(fig8[0], anchor, atol=1e-6) and np.allclose(fig8[-1], anchor, atol=1e-6)

    ax.plot(fig8[:, 0], fig8[:, 1], color=INK, lw=2)
    dot(ax, anchor, color=ACCENT, ms=6)
    ax.text(anchor[0] - 0.12, anchor[1] + 0.16, "$a$", color=ACCENT, ha="right")
    arrow(ax, (-1.25, 0), (1.25, 0), color=GUIDE)
    arrow(ax, (0, -0.7), (0, 0.7), color=GUIDE)
    ax.text(1.3, 0.05, "$u$")
    ax.text(0.08, 0.75, "$v$")
    # r is the radius to the lobe tip on the u-axis, at tau=0 in the Gerono form.
    arrow(ax, anchor, (radius, 0.0), color=ACCENT)
    ax.text(radius / 2, 0.1, "$r$", color=ACCENT, ha="center")
    ax.set_xlim(-1.5, 1.6)
    ax.set_ylim(-0.9, 0.9)
    clean_axes(ax, "figure8", r"anchor $a$, plane $(u,v)$, radius $r$")


PANELS = {
    "lerp": draw_lerp,
    "circle": draw_circle,
    "arc": draw_arc,
    "helix": draw_helix,
    "figure8": draw_figure8,
}
