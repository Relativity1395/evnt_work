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
### Step 3: Outlier Rejection via RANSAC
#### Goal: 
#### Analytical Process:

__________________________________________________________________________________________________________________________
### Step 4: Triangulate the feature using GN Optimization
#### Goal: 
#### Analytical Process:

__________________________________________________________________________________________________________________________
### Step 5: Compute the uncorrelated residuals <img width="30" height="25" alt="image" src="https://github.com/user-attachments/assets/6b38abb9-bee9-41c9-9bda-ab9fdb8f8705" />
#### Goal: 
#### Analytical Process:

__________________________________________________________________________________________________________________________
### Step 6: Stack all of the <img width="30" height="25" alt="image" src="https://github.com/user-attachments/assets/6b38abb9-bee9-41c9-9bda-ab9fdb8f8705" />
#### Goal: 
#### Analytical Process:

__________________________________________________________________________________________________________________________
### Step 7: Preform QR decomposition to get the final residual
#### Goal: 
#### Analytical Process:

__________________________________________________________________________________________________________________________
### Step 8: Update the state and state covariance
#### Goal: 
#### Analytical Process:

