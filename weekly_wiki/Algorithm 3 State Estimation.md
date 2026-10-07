# Algorithm 3 State Estimation
## Goal: Estimate the full 6-DoF pose and kinematic state of the IMU and Event Based Camera over time by fusing high-rate event feature tracks with inertial measurements using an Extended Kalman Filter (EKF)
## Inputs:
- Sensor state : $s_i$
- Features : {f}
- IMU values : <img width="16" height="16" alt="image" src="https://github.com/user-attachments/assets/2964bbdd-09d1-48fe-80ac-4aed49623642" />  for t ∈ [ $T_i$, $T_i$ + $dt_i$]
__________________________________________________________________________________________________________________________
### Step 1: IMU State and Covariance Propagation
#### Goal: Integrate IMU measurements across temporal window $[T_i, T_{i+1}]$
#### Analytical Process:
1. If multiple IMU packets exist in the window, iterate sequentially using 5th-order Runge-Kutta numerical integration
   <img width="752" height="217" alt="image" src="https://github.com/user-attachments/assets/d5240cb5-cd8a-4aa4-bfda-7fae43a7393d" />
2. If the event window is faster than IMU sampling (zero new IMU packets), reuse the most recent IMU sample
3. Propagate the error-state covariance matrix, where $\Phi_k$ is the discrete-time state transition matrix derived from the IMU kinematic error model.

   <img width="492" height="86" alt="image" src="https://github.com/user-attachments/assets/62767305-7f49-43e4-bdb2-52eef98a8a7f" />

__________________________________________________________________________________________________________________________
### Step 2: Camera State Augmentation
#### Goal: Clone the current sensor pose into the state history
#### Analytical Process:
1. Augment the state with a new camera pose at the current time

<img width="655" height="36" alt="image" src="https://github.com/user-attachments/assets/103dde33-78de-4bce-a6ca-edb997c76bbe" />

2. Update the covariance using the Jacobian that maps the IMU state to the camera state

__________________________________________________________________________________________________________________________
### Step 3 (INSIDE LOOP): Outlier Rejection via RANSAC
#### Goal: Find the largest set of inliers that project to the same point in space, based on reprojection error to remove moving objects and other erroneous measurements from the track
#### Analytical Process:
1. For a feature track pending marginalization with $M$ observations across camera poses, evaluate subsets of 2D measurements against their multi-view epipolar / reprojection constraints
2. Compute the reprojection error for each observation relative to the estimated 3D position
3. Discard observations that exceed the inlier threshold. If the remaining inlier count is insufficient for triangulation ($< 2$ views), reject and drop the feature entirely

__________________________________________________________________________________________________________________________
### Step 4 (INSIDE LOOP): Triangulate the feature using GN Optimization
#### Goal: Estimate 3D position of the feature <img width="20" height="30" alt="image" src="https://github.com/user-attachments/assets/78441719-ded9-4028-a684-34e353d63b6b" />  with its past observations and known/estimated camera poses.

#### Analytical Process:
1. 

__________________________________________________________________________________________________________________________
### Step 5 (INSIDE LOOP): Compute the uncorrelated residuals <img width="30" height="25" alt="image" src="https://github.com/user-attachments/assets/6b38abb9-bee9-41c9-9bda-ab9fdb8f8705" />
#### Goal: Take the difference between the observed and estimated feature positions to find the residual.
#### Analytical Process:
1. Calculate the difference between the observed and estimated feature positions
2. Left multiply residual, <img width="25" height="25" alt="image" src="https://github.com/user-attachments/assets/bb51b9cf-6275-47b8-80ac-8dfc60f11f5f" /> by the left null space, A, of feature Jacobian $H_F$ to eliminate the feature position up to a first order approximation.
<img width="891" height="172" alt="image" src="https://github.com/user-attachments/assets/e8efb2e1-ded6-4edf-ac80-ac0064ba6c70" />


__________________________________________________________________________________________________________________________
### Step 6: Stack all of the remaining <img width="30" height="25" alt="image" src="https://github.com/user-attachments/assets/6b38abb9-bee9-41c9-9bda-ab9fdb8f8705" />
#### Goal: Preform elimination procedure for all features and stack all uncorrelated residuals <img width="25" height="25" alt="image" src="https://github.com/user-attachments/assets/d11f0e62-c6b3-4345-aae0-1f0ed2db0482" /> to obtain the final residual <img width="20" height="20" alt="image" src="https://github.com/user-attachments/assets/ab012802-6b6c-4c42-bf09-5888c79ea92b" />

#### Analytical Process:
1. Collect the projected residuals $r_0^{(j)}$ and projected state Jacobians $H_0^{(j)}$ from every marginalized feature $j$
2. Vertically stack the residuals 1 - $M$ in a vector $r_0$
3. Vertically stack the state measurement matricies 1 - $M$ in vector $H_0$

__________________________________________________________________________________________________________________________
### Step 7: Preform QR decomposition to get the final residual
#### Goal: Compress the stacked measurement system by eliminating redundant rows, reducing matrix dimensions to match the state dimension
#### Analytical Process:

__________________________________________________________________________________________________________________________
### Step 8: Update the state and state covariance
#### Goal: Compute the Kalman gain, correct the nominal IMU state and camera poses using the compressed visual residuals, and update the error-state covariance matrix
#### Analytical Process:

