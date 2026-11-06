import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# df = pd.read_csv("build/feature_status.csv")
df = pd.read_csv("/home/drone-laptop/feature_status_test_4.csv")
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
   
t = (df["timestamp"] - df["timestamp"].iloc[0]) / 1e6
ax[0][0].scatter(np.array(t), np.array(df["px"]), c = df["id"], cmap='tab20')
ax[0][0].set_xlabel("Time (s)")
ax[0][0].set_ylabel("Pixel in x (s)")
ax[0][0].set_ylim(0, 640)

ax[0][1].scatter(np.array(t), np.array(df["py"]), c = df["id"], cmap='tab20')
ax[0][1].set_xlabel("Time (s)")
ax[0][1].set_ylabel("Pixel position in y (s)")
ax[0][1].set_ylim(0, 480)

ax[1][0].scatter(np.array(df["events"]), np.array(df["cost"]))
ax[1][0].set_xlabel("Num Events")
ax[1][0].set_ylabel("Cost")

ax[1][1].scatter(
    np.array(df["px"]),
    np.array(df["py"]),
    c=t,
    cmap='coolwarm'  # Reversed: goes from Warm (Red) to Cold (Blue)

)  
ax[1][1].set_xlabel("Pixel poistion X")
ax[1][1].set_xlim(0, 640)
ax[1][1].set_ylabel("Pixel position y")
ax[1][1].set_ylim(0, 480)

# ax[2][0].scatter(np.array(df["id"]), np.array(total_movement), c = df["id"], cmap='tab20')
ax[2][1].scatter(np.array(df["events"]), np.array(df["window_size"]), c = df["id"], cmap='tab20')

ax[2][1].set_xlabel("Time(s)")
ax[2][1].set_ylabel("Window size")

# plt.xlabel("Time (s)")
plt.show()
plt.ylabel("Total Movement (pixels)")
plt.title("Cumulative Pixel Movement of All Features")
plt.grid(True)


'''
Low event count per window size
'''