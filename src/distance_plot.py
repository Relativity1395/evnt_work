import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# df = pd.read_csv("build/feature_status.csv")
df = pd.read_csv("build/feature_status.csv")
df = df.sort_values("timestamp")

# plt.figure(figsize=(12, 7))

'''
We need the following plots to tune epsilon:
1. scatter plot of feature movements in px and py (px -> x axis, py -> y axis)
2. scatter plot of feature moveemnts in px, t and py, t
3. scatter plots of optical flow in ux, t and uy, t
4. scatter plot of cost per event
5. window duration of each feature ID
6. cost vs time scatter plot
7. Plot the number of iterations per time window
8. 
'''


   

dx = df["px"].diff().fillna(0)
dy = df["py"].diff().fillna(0)
total_movement = np.sqrt(dx**2 + dy**2).cumsum()

fig, ax = plt.subplots(3, 2, figsize=(10, 3))
'''
Low event count per window size
'''

for id_val, group in df.groupby("id"):
    net_displacement = 0
    for i in range(len(group["px"])):
        net_displacement += group["px"].iloc[i]
    del_t = group["timestamp"].iloc[-1] - group["timestamp"].iloc[0]
    if net_displacement > 100000 and del_t > 1e6:
        ax[0][0].scatter(np.array(id_val), np.array(del_t))
        ax[0][0].set_ylabel("Net displacement")
        ax[0][0].set_xlabel("ID")
        ax[0][1].scatter(np.array(group["events"]), np.array(group["window_size"]))
        ax[0][1].set_xlabel("number of events")
        ax[0][1].set_ylabel("Window Size")
        ax[1][0].scatter(np.array(group["iters"]), np.array(group["cost"]))
        ax[1][0].set_ylabel("Cost")
        ax[1][0].set_xlabel("Iterations per winow")

        ax[1][1].scatter(np.array(group["timestamp"]), np.array(group["ux"]))
        ax[1][1].set_xlabel("time")
        ax[1][1].set_ylabel("flow in the x direction")
        ax[1][1].set_ylim(0, 0.0001)

plt.show()