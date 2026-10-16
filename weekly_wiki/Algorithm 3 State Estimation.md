# Algorithm 3 State Estimation
## Goal: Estimate the full 6 Degrees of Freedom pose and kinematic state of the IMU and Event Based Camera over time by combining high-rate event feature tracks with inertial measurements using an Extended Kalman Filter (EKF)
<img width="741" height="516" alt="image" src="https://github.com/user-attachments/assets/533751fe-8434-47e1-b49f-c9a3e75baef8" />

## Inputs:
- Sensor state : $s_i$
- Features : {f}
- IMU values : <img width="16" height="16" alt="image" src="https://github.com/user-attachments/assets/2964bbdd-09d1-48fe-80ac-4aed49623642" />  for t ∈ [ $T_i$, $T_i$ + $dt_i$]
__________________________________________________________________________________________________________________________
### Step 1: Propagate the Sensor State Mean and Covariance
#### Goal: Predict the current sensor state and full state covariance forward in time across the temporal window $[T_i, T_i + dt_i]$ using inertial measurements
#### Inputs: 
- Previous full filter state $S$ (specifically the current sensor state $s_i$ at time $T_i$) and its state covariance matrix
- IMU Measurements of linear acceleration ($a_k$), angular velocities ($\omega_k$), and time stamps ($\tau_k \in [T_i, T_i + dt_i]$)
#### Analytical Process:
1. If multiple IMU packets exist in the window, iterate sequentially using 5th-order Runge-Kutta numerical integration
   <img width="752" height="217" alt="image" src="https://github.com/user-attachments/assets/d5240cb5-cd8a-4aa4-bfda-7fae43a7393d" />
2. If the event window is faster than IMU sampling (zero new IMU packets), reuse the most recent IMU sample
3. Propagate the error-state covariance matrix, where $\Phi_k$ is the discrete-time state transition matrix derived from the IMU kinematic error model.

   <img width="492" height="86" alt="image" src="https://github.com/user-attachments/assets/62767305-7f49-43e4-bdb2-52eef98a8a7f" />

#### Output:
- Propagated state vector and covariance matrix at time $T_{i+1} = T_i + dt_i$
__________________________________________________________________________________________________________________________
### Step 2: Augment a new camera state
#### Goal: Append the newly predicted camera pose at time $T_{i+1}$ to the sliding window of historical camera poses in the state vector and expand the covariance matrix accordingly
#### Inputs: 
- Propagated full state vector $S$ and propagated state covariance matrix
- Jacobian matrix that maps the current IMU/sensor state to the camera pose
#### Analytical Process:
1. Extract the current camera orientation $\bar{q}(T_{i+1})$ and position $p(T_{i+1})$ from the propagated sensor state $s(T_{i+1})$ and append them to the full state vector $S$

- <img width="655" height="36" alt="image" src="https://github.com/user-attachments/assets/103dde33-78de-4bce-a6ca-edb997c76bbe" />

2. Compute the state-to-camera Jacobian
3. Update and expand the dimensions of the state covariance matrix using the Jacobian

#### Output:
- Augmented state vector $S$ (now containing the new pose) and expanded state covariance matrix

__________________________________________________________________________________________________________________________
### Step 3 (INSIDE LOOP): Per Feature Loop - Process Marginalized Features
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
### Step 3a: Remove Inconsistent Observations
#### Goal: Reject outlier 2D feature observations within the completed feature track caused by moving objects or tracking drift before using the track for state correction
#### Inputs:
- The 2D image coordinate history (observations) of the discarded feature track $f_j$ across the camera poses that observed it
- The corresponding historical camera poses $(\bar{q}, p)$ from the state vector $S$
#### Analytical Process:
1. Run a Reprojection RANSAC loop over the 2D observations in track $f_j$
2. Estimate a 3D point from minimal subsets of observations, project the 3D point back into all observing camera frames using the perspective projection model equation
- <img width="350" height="42" alt="image" src="https://github.com/user-attachments/assets/fdc3125e-b23f-44be-b7e2-369c6eda067d" />

3. Find the biggest group of measurements that all agree on the exact same 3D location, and ignore the rest as mistakes.

#### Output: A filtered set of inlier 2D observations and their corresponding camera pose indices for feature $f_j$
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
### Step 3b: Triangulate the Feature using Gauss-Newton (GN) Optimization
#### Goal: Estimate the 3D world coordinates $\hat{F}_j \in \mathbb{R}^3$ of the landmark corresponding to feature track $f_j$
#### Inputs:
- The inlier 2D image observations for feature $f_j$ (from Step 3a)
- The known historical camera poses $(\bar{q}, p)$ from $S$ that observed $f_j$
#### Analytical Process:
1. Treat the historical camera poses in $S$ as fixed/known
2. Formulate the nonlinear least-squares reprojection cost using the perspective projection model equations 
- <img width="447" height="127" alt="image" src="https://github.com/user-attachments/assets/c3c34511-e0b7-4275-8b57-994fb6ad895c" />
3. Solve for the 3D landmark position $\hat{F}_j$ via Gauss-Newton optimization to minimize the difference between observed 2D coordinates and projected 3D estimates

#### Output: Estimated 3D landmark position $\hat{F}_j$

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
### Step 3a: Compute the Uncorrelated Residuals $r_0^{(j)}$
#### Goal: Compute the measurement reprojection residual for feature $f_j$ and project it onto the null space of the feature Jacobian so the error depends only on the camera poses, eliminating the 3D feature position from the EKF state
#### Inputs:
- Inlier 2D observations of feature $f_j$
- Estimated 3D feature position $\hat{F}_j$ (from Step 3b)
- Observing camera poses from state vector $S$
#### Analytical Process:
1. Compute the predicted 2D projection in each observing camera pose using
- <img width="591" height="47" alt="image" src="https://github.com/user-attachments/assets/52d11348-5dc5-40aa-83de-107c4f5ae8b6" />
2. Calculate the stacked per-pose residual vector $r^{(j)}$ for feature $j$ as the difference between the actual 2D observations and the predicted projections
3. Linearize the measurement model around the current state estimate $S$ and feature estimate $\hat{F}_j$ to compute the state Jacobian $H_S^{(j)}$ and the feature Jacobian $H_F^{(j)}$
4. Project the residual $r^{(j)}$ and state Jacobian $H_S^{(j)}$ onto $A^T$ (Equation 13) to eliminate the feature error $\tilde{F}_j$ using equation
- <img width="732" height="143" alt="image" src="https://github.com/user-attachments/assets/53dc6a86-fec9-44b2-b20f-09816e2c8116" />


#### Output: The uncorrelated residual vector $r_0^{(j)}$, the projected state measurement Jacobian $H_0^{(j)} = A^T H_S^{(j)}$, and the projected noise covariance for feature $f_j$

__________________________________________________________________________________________________________________________
### Step 4: Stack All of the $r_0^{(j)}$
#### Goal: Aggregate the uncorrelated residuals and measurement Jacobians from all marginalized features in the current update cycle into a single linear system
#### Inputs: 
- All individual uncorrelated residual vectors $\{r_0^{(j)}\}$ and their corresponding Jacobians $\{H_0^{(j)}\}$ generated in Step 3c
#### Analytical Process:
1. Vertically concatenate all $r_0^{(j)}$ vectors to form a single combined residual vector $r_0$
2. Vertically concatenate all $H_0^{(j)}$ matrices to form the combined measurement Jacobian matrix $H_0$

#### Output:
- Stacked residual vector $r_0$ and stacked measurement Jacobian $H_0$

__________________________________________________________________________________________________________________________
### Step 5: Perform QR Decomposition to Get the Final Residual
#### Goal: Compress the high-dimensional stacked residual $r_0$ and Jacobian $H_0$ to reduce the computational complexity of the Kalman gain matrix inversion
#### Inputs: Stacked residual vector $r_0$ and stacked measurement Jacobian $H_0$
#### Analytical Process:
1. Perform QR decomposition on the stacked Jacobian matrix $H_0$ using equation below where $Q_1$ and $Q_2$ have orthonormal columns and $T_H$ is an upper triangular matrix
- <img width="307" height="80" alt="image" src="https://github.com/user-attachments/assets/53d87865-ab1b-4fa7-881c-5691a4b4dc6b" />

2. Project the stacked residual $r_0$ onto $Q_1^T$ and discard the zero-Jacobian subspace corresponding to $Q_2$ using equation
- <img width="186" height="60" alt="image" src="https://github.com/user-attachments/assets/9ad2374f-264e-4853-9164-771732a69867" />

3. Transform the measurement noise covariance matrix using $Q_1^T$
#### Output: Dimension-reduced final residual vector $r_n$, upper-triangular measurement Jacobian $T_H$, and reduced measurement noise covariance

__________________________________________________________________________________________________________________________
### Step 6: Update the state and state covariance
#### Goal: Combine the compressed visual measurement residual $r_n$ with the predicted state to correct drift in the current IMU state and historical camera pose
#### Inputs:
- Augmented state vector $S$ and state covariance matrix (from Step 2)
- Reduced final residual $r_n$, upper-triangular Jacobian $T_H$, and reduced measurement noise covariance (from Step 5)
#### Analytical Process:
1. Compute the standard EKF Kalman gain $K$ using the state covariance matrix, the compressed Jacobian $T_H$, and the compressed measurement noise covariance
2. Compute the state correction vector using equation
- <img width="487" height="42" alt="image" src="https://github.com/user-attachments/assets/5655799a-4d95-47e6-854e-d5a00852de2f" />
3. Apply $\Delta S$ to update the state vector $S$ (using quaternion multiplication for the orientation states $\bar{q}$ and vector addition for $b_g, v, b_a, p$)
4. Update the state covariance matrix using the standard EKF covariance update equation, and prune any historical camera poses from $S$ and the covariance matrix that no longer observe any active features
#### Output: Updated full filter state $S$ (yielding the updated sensor state $s_{i+1}$) and updated state covariance matrix for the next iteration of Algorithm 1


