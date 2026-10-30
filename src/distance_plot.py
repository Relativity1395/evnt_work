import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv("build/feature_status.csv")
df = df.sort_values("timestamp")

plt.figure(figsize=(12, 7))

for id_val, group in df.groupby("id"):
    group = group.sort_values("timestamp")

    t = (group["timestamp"] - group["timestamp"].iloc[0]) / 1e6

    dx = group["px"].diff().fillna(0)
    dy = group["py"].diff().fillna(0)

    total_movement = np.sqrt(dx**2 + dy**2).cumsum()

    plt.plot(t, total_movement, label=f"ID {id_val}")

plt.xlabel("Time (s)")
plt.ylabel("Total Movement (pixels)")
plt.title("Cumulative Pixel Movement of All Features")
plt.grid(True)
plt.show()