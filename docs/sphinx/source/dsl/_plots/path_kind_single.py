"""One path kind's panel, standalone -- draws whichever kind the generator injects as
PANEL_NAME, reusing the exact same function the combined overview figure uses."""

import matplotlib.pyplot as plt

from _paths import PANELS

fig, ax = plt.subplots(figsize=(3.6, 4.0))
PANELS[PANEL_NAME](ax)
fig.tight_layout()
