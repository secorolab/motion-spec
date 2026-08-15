import numpy as np
import matplotlib.pyplot as plt

from _style import ACCENT, GUIDE, INK, MEASURED, arrow, clean_axes, dot, out_of_page_marker

# A generic smooth path -- the projection/tangent/frame construction applies to any path,
# so this is just an illustrative cubic Bezier, not one of the five named kinds.
P0, P1, P2, P3 = (
    np.array([0.0, 0.0]),
    np.array([2.0, 3.4]),
    np.array([5.4, -2.2]),
    np.array([8.2, 2.6]),
)


def p(s):
    return (
        (1 - s) ** 3 * P0
        + 3 * (1 - s) ** 2 * s * P1
        + 3 * (1 - s) * s**2 * P2
        + s**3 * P3
    )


# eq:path-projection -- s* minimizes the squared distance to the measured point, searched
# here over the whole path (a real controller narrows this to a small window around the
# previous cycle's s*, but the objective is the same).
s_grid = np.linspace(0.0, 1.0, 4000)
curve = np.array([p(s) for s in s_grid])
p_m = np.array([4.5, -1.8])  # well clear of the curve, so the projection is visible
s_star = s_grid[np.argmin(np.sum((curve - p_m) ** 2, axis=1))]
p_star = p(s_star)

# eq:path-tangent -- central difference, clamped at the endpoints.
delta = 1e-4
s_minus, s_plus = np.clip([s_star - delta, s_star + delta], 0.0, 1.0)
raw_tangent = p(s_plus) - p(s_minus)
t = raw_tangent / np.linalg.norm(raw_tangent)

# eq:path-frame -- n1 parallel-transported from a previous cycle. With no real "previous
# cycle" for a static figure, fall back to the documented initialization rule: the world
# axis least aligned with the tangent.
world_axes = [np.array([1.0, 0.0]), np.array([0.0, 1.0])]
n1_prev = min(world_axes, key=lambda axis: abs(np.dot(axis, t)))
n1_tilde = n1_prev - t * np.dot(t, n1_prev)
n1 = n1_tilde / np.linalg.norm(n1_tilde)
# n2 = t x n1; both t and n1 are in-plane here, so n2 always points out of the page.
assert abs(np.dot(t, n1)) < 1e-9

e_p = p_star - p_m

fig, ax = plt.subplots(figsize=(6.4, 4.6))
ax.plot(curve[:, 0], curve[:, 1], color=INK, lw=2.4)
dot(ax, P0)
dot(ax, P3)
ax.text(P0[0] - 0.15, P0[1] - 0.1, "$s=0$", ha="right", color="#52606d")
ax.text(P3[0] + 0.15, P3[1], "$s=1$", ha="left", color="#52606d")

# window highlight around s*, purely illustrative of "a small window, not the whole path"
window = [s for s in s_grid if abs(s - s_star) < 0.08]
window_pts = np.array([p(s) for s in window])
ax.plot(window_pts[:, 0], window_pts[:, 1], color=ACCENT, lw=7, alpha=0.15, solid_capstyle="round")

dot(ax, p_star)
ax.text(p_star[0], p_star[1] + 0.35, "$p(s^*)$", ha="center")

scale = 1.3
arrow(ax, p_star, p_star + scale * t, color=ACCENT)
ax.text(*(p_star + scale * t + np.array([0.15, 0.0])), "$t$", color=ACCENT)

arrow(ax, p_star, p_star + scale * n1, color=ACCENT)
ax.text(*(p_star + scale * n1 + np.array([0.1, 0.1])), "$n_1$", color=ACCENT)

out_of_page_marker(ax, p_star - scale * 0.55 * t)
ax.text(*(p_star - scale * 0.75 * t + np.array([0.0, 0.3])), "$n_2$", color=ACCENT, ha="center")

dot(ax, p_m, color=MEASURED)
ax.text(p_m[0] + 0.2, p_m[1] - 0.15, "$p_m$", color=MEASURED)
ax.plot([p_m[0], p_star[0]], [p_m[1], p_star[1]], "--", color=MEASURED, lw=1.5)
mid_err = (p_m + p_star) / 2
ax.text(mid_err[0] + 0.2, mid_err[1], "$e_p$", color=MEASURED)

ax.text(p_star[0], p_star[1] - 0.75, "window $W_k$", color=ACCENT, ha="center")

ax.set_xlim(-1.0, 9.4)
ax.set_ylim(-2.6, 4.4)
clean_axes(ax)
fig.tight_layout()
