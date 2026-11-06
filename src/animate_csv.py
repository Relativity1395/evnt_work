import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv("/home/drone-laptop/feature_status_test_4.csv")
df = df.sort_values("timestamp")

PLAYBACK_SPEED = 0.5  # 0.5 = half speed, 1.0 = real time, 2.0 = double speed
ARROW_SCALE = 100000  # Flow (px/us) displayed over 100 ms

# Group features by timestamp
frames = list(df.groupby("timestamp", sort=True))

# Assign consistent colors to feature IDs
feature_ids = df["id"].unique()
cmap = plt.get_cmap("turbo")
colors = {
    fid: cmap(i / max(1, len(feature_ids) - 1))
    for i, fid in enumerate(feature_ids)
}

fig, ax = plt.subplots()
scatter = ax.scatter([], [], s=15)
quiver = None
labels = []

ax.set_xlim(df["px"].min() - 10, df["px"].max() + 10)
ax.set_ylim(df["py"].min() - 10, df["py"].max() + 10)
ax.invert_yaxis()
ax.set_aspect("equal")

ax.set_xlabel("X Position (pixels)")
ax.set_ylabel("Y Position (pixels)")

def update(frame):
    global labels, quiver

    timestamp, group = frames[frame]

    positions = group[["px", "py"]].to_numpy()
    scatter.set_offsets(positions)

    # Color features by ID
    frame_colors = [colors[fid] for fid in group["id"]]
    scatter.set_color(frame_colors)

    # Calculate optical flow vectors
    ux = group["ux"].to_numpy() * ARROW_SCALE
    uy = group["uy"].to_numpy() * ARROW_SCALE

    # Remove previous arrows
    if quiver is not None:
        quiver.remove()

    # Draw new arrows for current features
    quiver = ax.quiver(
        positions[:, 0],
        positions[:, 1],
        ux,
        uy,
        color=frame_colors,
        angles="xy",
        scale_units="xy",
        scale=1,
        width=0.005
    )

    # Remove previous ID labels
    for label in labels:
        label.remove()
    labels = []

    # Display feature IDs
    for _, feature in group.iterrows():
        label = ax.text(
            feature["px"] + 3,
            feature["py"] - 3,
            str(int(feature["id"])),
            fontsize=7,
            color=colors[feature["id"]]
        )
        labels.append(label)

    # Display elapsed camera time
    ax.set_title(
        f"Time: {(timestamp - frames[0][0]) / 1e6:.2f} s"
    )


# Custom timer for timestamp-based playback
current_frame = 0
timer = fig.canvas.new_timer(interval=100)

def advance_frame():
    global current_frame

    if current_frame >= len(frames):
        timer.stop()
        return

    update(current_frame)
    fig.canvas.draw_idle()

    # Calculate delay until next logged timestamp
    if current_frame < len(frames) - 1:
        current_time = frames[current_frame][0]
        next_time = frames[current_frame + 1][0]

        dt_ms = (next_time - current_time) / 1000.0 / PLAYBACK_SPEED
        timer.interval = max(1, int(dt_ms))
    else:
        timer.stop()

    current_frame += 1


timer.add_callback(advance_frame)

# Draw the first frame immediately
advance_frame()

# Start timer for subsequent frames
if current_frame < len(frames):
    timer.start()

plt.show()