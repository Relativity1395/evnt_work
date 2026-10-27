import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv("../build/feature_status.csv")

f = df[df.id == 0].sort_values("timestamp")
conv = df[df.converged == True]
df.groupby("id").timestamp.agg(["min", "max", "count"])
df["umag"] = np.hypot(df.ux, df.uy)
df.groupby("id").umag.mean()
t = np.array((f.timestamp - f.timestamp.iloc[0]) / 1e6)
plt.plot(t, np.array(f.ux))
plt.plot(t, np.array(f.uy))
plt.show()