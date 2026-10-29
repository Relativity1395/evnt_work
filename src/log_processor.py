import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv("feature_status.csv")

# conv = df[df.converged == True]
# df.groupby("id").timestamp.agg(["min", "max", "count"])
# # df["umag"] = np.hypot(df.ux, df.uy)
# # df.groupby("id").umag.mean()


# plt.scatter(t, f.ux)
# # plt.scatter(t, f.uy * 1e6, s=5, label="uy")
# plt.show()
# plt.legend()

df = df.sort_values("timestamp")

# 2. Group by 'id' and loop through each unique group
displacement = {}
for id_val, group in df.groupby("id"):

    # 'group' is a filtered DataFrame containing only rows for the current id_val

    
    # Calculate time elapsed since the first timestamp in this group
    # Note: If timestamp is in nanoseconds, dividing by 1e6 converts it to milliseconds
    t = (group["timestamp"] - group["timestamp"].iloc[0]).to_numpy() / 1e6

    # Calculate the net price change (last price minus first price)
    px_change = group["px"].iloc[-1] - group["px"].iloc[0]

    if px_change > 10:
        displacement[px_change] = id_val

    
    # Plot the result
    # (Note: Standard scatter plots usually require both an X and Y array of matching lengths.
    # If you just want to plot the single net change against the ID, you can do this:)
    # plt.scatter(id_val, px_change)

   


    # If your intention was actually to plot 't' against 'px' over time for this ID, use:
    # plt.plot(t, group['px'])

    plt.title(f"ID: {id_val}")
max_displacement_id = displacement[max(displacement.keys())]
print()


f = df[df.id == max_displacement_id].sort_values("timestamp")
t = f.timestamp - df.timestamp.min()

print
plt.plot(np.array(f.timestamp - df.timestamp.min()) / 1e6, np.array(f.px))
plt.show()




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
