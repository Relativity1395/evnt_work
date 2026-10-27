import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv("build/feature_status.csv")

f = df[df.id == 0].sort_values("timestamp")
conv = df[df.converged == True]
df.groupby("id").timestamp.agg(["min", "max", "count"])
df["umag"] = np.hypot(df.ux, df.uy)
df.groupby("id").umag.mean()
t = np.array((f.timestamp - f.timestamp.iloc[0]) / 1e6)

plt.scatter(t, f.ux * 1e6, s=5, label="ux")
plt.scatter(t, f.uy * 1e6, s=5, label="uy")
plt.show()
plt.legend()


# plt.plot(t, np.array(f.cost))
# # plt.plot(t, np.array(f.uy))
# plt.xlabel("time(s)")
# plt.ylabel("costS")
# plt.show()

# df[df.converged].cost.hist(bins=100, alpha=0.5, label="converged")
# df[~df.converged].cost.hist(bins=100, alpha=0.5, label="not converged")
# plt.axvline(80000, color='r'); plt.legend(); plt.show()

# good = df[df.events >= 500]
# df["cost_per_event"] = good.cost / good.events.replace(0, np.nan)
# plt.hist(df.cost_per_event.dropna(), bins=100)
# plt.xlabel("cost / event"); plt.show()

# df["umag"] = np.hypot(df.ux, df.uy)
# df["umag_px_s"] = df.umag * 1e6          # convert px/us → px/s

# fid = 7
# f = df[df.id == fid].sort_values("timestamp")
# plt.scatter((f.timestamp - f.timestamp.iloc[0]) / 1e6, np.hypot(f.ux, f.uy) * 1e6, s=5)
# plt.xlabel("time (s)"); plt.ylabel("speed (px/s)"); plt.show()