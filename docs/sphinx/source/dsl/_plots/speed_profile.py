import numpy as np
import matplotlib.pyplot as plt

from _style import ACCENT, CAPTION, GUIDE, INK

v_c = 1.0
a_max = 1.6
# Low enough relative to a_max that the jerk-limited shoulder is a large fraction of each
# ramp -- otherwise the S-curve's rounding is too short, at this plot scale, to read as
# anything but a sharp corner.
j_max = 2.8
dt = 0.01
T = 6.0
steps = int(T / dt)
path_length = 3.6


def v_target_for(remaining):
    # eq:path-braking-speed
    v_b = np.sqrt(max(2 * a_max * remaining, 0.0))
    return np.sign(v_c) * min(abs(v_c), v_b)


# Each profile derives L_rem from its own actual integrated position every step -- exactly
# what the runtime does -- so the two curves brake against their own real remaining
# distance rather than a shared schedule that could go unphysical for one of them.

# eq:path-trapezoidal-profile
v_trap = np.zeros(steps + 1)
x_trap = 0.0
for k in range(steps):
    v_target = v_target_for(path_length - x_trap)
    v_trap[k + 1] = v_trap[k] + np.clip(v_target - v_trap[k], -a_max * dt, a_max * dt)
    x_trap += v_trap[k + 1] * dt

# eq:path-s-curve-profile
v_s = np.zeros(steps + 1)
a_s = np.zeros(steps + 1)
x_s = 0.0
for k in range(steps):
    v_target = v_target_for(path_length - x_s)
    a_star = np.clip((v_target - v_s[k]) / dt, -a_max, a_max)
    a_s[k + 1] = a_s[k] + np.clip(a_star - a_s[k], -j_max * dt, j_max * dt)
    v_s[k + 1] = np.clip(v_s[k] + a_s[k + 1] * dt, 0.0, abs(v_c))
    x_s += v_s[k + 1] * dt

assert v_trap[0] == 0.0 and v_s[0] == 0.0
assert v_trap.max() <= v_c + 1e-9 and v_s.max() <= v_c + 1e-9

t = np.arange(steps + 1) * dt
reached_cruise = np.argmax(v_trap >= v_c - 1e-6)
brake_index = reached_cruise + np.argmax(v_trap[reached_cruise:] < v_c - 1e-6)

fig, ax = plt.subplots(figsize=(6.4, 4.0))
ax.axhline(v_c, ls="--", color=GUIDE, lw=1)
ax.text(T, v_c, " $v_c$", va="center", color=CAPTION)
ax.axvline(t[brake_index], ls="--", color=GUIDE, lw=1)
ax.text(
    t[brake_index] + 0.08,
    v_c * 1.08,
    r"braking starts ($L_{\mathrm{rem}} \to v_b$)",
    color=CAPTION,
    fontsize=10,
)

ax.plot(t, v_trap, color=INK, lw=2.2, label=r"trapezoidal ($a_{\max}$ only)")
ax.plot(t, v_s, color=ACCENT, lw=2.2, label=r"S-curve ($a_{\max}$ and $j_{\max}$)")
ax.set_xlabel("$t$")
ax.set_ylabel("$v$")
ax.set_xlim(0, T)
ax.set_ylim(-0.05, v_c * 1.25)
ax.spines[["top", "right"]].set_visible(False)
ax.legend(frameon=False, loc="lower center", bbox_to_anchor=(0.5, -0.32), ncol=1)
fig.tight_layout()
