import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

df = pd.read_csv("build/feature_status.csv")
df = df.sort_values("timestamp")

# Group features by timestamp
frames = list(df.groupby("timestamp"))

fig, ax = plt.subplots()
scatter = ax.scatter([], [], s=15)

ax.set_xlim(df["px"].min() - 10, df["px"].max() + 10)
ax.set_ylim(df["py"].min() - 10, df["py"].max() + 10)
ax.invert_yaxis()  # Camera coordinates have y increasing downward

ax.set_xlabel("X Position (pixels)")
ax.set_ylabel("Y Position (pixels)")

def update(frame):
    timestamp, group = frames[frame]

    positions = group[["px", "py"]].to_numpy()
    scatter.set_offsets(positions)

    ax.set_title(f"Time: {(timestamp - frames[0][0]) / 1e6:.2f} s")

    if frame < len(frames) - 1:
        next_timestamp = frames[frame + 1][0]
        dt = (next_timestamp - timestamp) / 1000  # microseconds to milliseconds
        ani.event_source.interval = max(1, dt)

    return scatter,

ani = FuncAnimation(
    fig,
    update,
    frames=len(frames),
    interval=50,
    blit=False,
    repeat=False
)

plt.show()