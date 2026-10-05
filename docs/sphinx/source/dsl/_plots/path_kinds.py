import matplotlib.pyplot as plt

from _paths import PANELS

fig, axes = plt.subplots(3, 2, figsize=(7.4, 10.6))
axes[2, 1].axis("off")

positions = [(0, 0), (0, 1), (1, 0), (1, 1), (2, 0)]
for (row, col), draw in zip(positions, PANELS.values()):
    draw(axes[row, col])

fig.tight_layout(h_pad=3.2, w_pad=2.0)
