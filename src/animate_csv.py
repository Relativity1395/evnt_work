import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

df = pd.read_csv("build/feature_status.csv")
df = df.sort_values("timestamp")

# Group features by timestamp
frames = list(df.groupby("timestamp"))

# Assign consistent colors to feature IDs
feature_ids = df["id"].unique()
cmap = plt.get_cmap("turbo")
colors = {
    fid: cmap(i / max(1, len(feature_ids) - 1))
    for i, fid in enumerate(feature_ids)
}

fig, ax = plt.subplots()
scatter = ax.scatter([], [], s=15)
labels = []

ax.set_xlim(df["px"].min() - 10, df["px"].max() + 10)
ax.set_ylim(df["py"].min() - 10, df["py"].max() + 10)
ax.invert_yaxis()  # Camera coordinates have y increasing downward

ax.set_xlabel("X Position (pixels)")
ax.set_ylabel("Y Position (pixels)")

def update(frame):
    global labels

    timestamp, group = frames[frame]

    positions = group[["px", "py"]].to_numpy()
    scatter.set_offsets(positions)

    # Color each feature according to its ID
    scatter.set_color([colors[fid] for fid in group["id"]])

    # Remove previous ID labels
    for label in labels:
        label.remove()
    labels = []

    # Display IDs next to each feature
    for _, feature in group.iterrows():
        label = ax.text(
            feature["px"] + 3,
            feature["py"] - 3,
            str(int(feature["id"])),
            fontsize=7,
            color=colors[feature["id"]]
        )
        labels.append(label)

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