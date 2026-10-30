# Multi-State Constraint Kalman Filter (MSCFK) Algorithm Breakdown
## Goal: Implement a EKF to complete pose estimation with features and Intertial Measurment Unit (IMU) for Event-Based Visual-Inertial Odometry (EVIO). To do this we use past camera poses in the state vector and uses visual features purely to create geometric constraints between those poses.

<img width="656" height="412" alt="image" src="https://github.com/user-attachments/assets/5f1e5323-a7ea-4827-a319-79bfc33602b7" />

## Variables:
- IMU-affixed Frame : {I}
- global frame of reference : {G}
