# IMU 3D Pose Propagation Guide (Continuous-Time Differential Equations via RK5)

This document serves as an engineering reference for updating (propagating) the 3D IMU pose and state vector in a visual-inertial Extended Kalman Filter (EKF), following the Multi-State Constraint Kalman Filter (MSCKF) and Event-based Visual Inertial Odometry (EVIO) formulations.

## Table of Contents

1. [State Vector and Conventions](#1-state-vector-and-conventions)
2. [Step 1: Acquire and Compensate Raw IMU Measurements](#2-step-1-acquire-and-compensate-raw-imu-measurements)
3. [Step 2: Continuous-Time State Differential Equations](#3-step-2-continuous-time-state-differential-equations)
4. [Step 3: 5th-Order Runge-Kutta (RK5) Numerical Integration](#4-step-3-5th-order-runge-kutta-rk5-numerical-integration)
5. [Step 4: Post-Integration Quaternion Normalization](#5-step-4-post-integration-quaternion-normalization)
6. [Step 5: Companion EKF Covariance Propagation](#6-step-5-companion-ekf-covariance-propagation)
7. [Implementation Notes and Common Pitfalls](#7-implementation-notes-and-common-pitfalls)
8. [References](#8-references)

## 1. State Vector and Conventions

The evolving IMU state is represented by a $16 \times 1$ state vector:

$$
X_{IMU} = \begin{bmatrix} \mathstrut_G^I\overline{q}^T & b_g^T & {}^G v_I^T & b_a^T & {}^G p_I^T \end{bmatrix}^T
$$

| Symbol | Size | Frame | Description |
| --- | --- | --- | --- |
| $\mathstrut_G^I\overline{q}$ | $4 \times 1$ | $\lbrace G \rbrace \to \lbrace I \rbrace$ | Unit quaternion rotating vectors from Global frame $\lbrace G \rbrace$ to IMU frame $\lbrace I \rbrace$ |
| $b_g$ | $3 \times 1$ | $\lbrace I \rbrace$ | Gyroscope measurement bias (modeled as a random walk) |
| ${}^G v_I$ | $3 \times 1$ | $\lbrace G \rbrace$ | Linear velocity of the IMU expressed in $\lbrace G \rbrace$ |
| $b_a$ | $3 \times 1$ | $\lbrace I \rbrace$ | Accelerometer measurement bias (modeled as a random walk) |
| ${}^G p_I$ | $3 \times 1$ | $\lbrace G \rbrace$ | 3D position of the IMU expressed in $\lbrace G \rbrace$ |

> **Critical Convention Check (JPL Quaternion Order):**
> The MSCKF formulation uses **JPL quaternion conventions** with the vector part first and scalar part last: $\overline{q} = [q_x, q_y, q_z, q_w]^T$.
>
> * $C_{\hat{q}} = C(\mathstrut_G^I\hat{\overline{q}})$ is the $3 \times 3$ rotation matrix mapping a vector from $\lbrace G \rbrace$ to $\lbrace I \rbrace$.
> * $C_{\hat{q}}^T$ maps a vector from $\lbrace I \rbrace$ back to $\lbrace G \rbrace$.
> * If your codebase uses Hamilton (scalar-first) quaternions or stores $\mathstrut_I^G q$ (body-to-global), verify the signs in $\Omega(\omega)$ and transpose $C_{\hat{q}}$ accordingly.

## 2. Step 1: Acquire and Compensate Raw IMU Measurements

At discrete time $t_k$ with sampling interval $T = t_{k+1} - t_k$ (or sub-interval $\Delta t$), the IMU outputs raw angular velocity $\omega_m \in \mathbb{R}^3$ and raw linear acceleration $a_m \in \mathbb{R}^3$.

Subtract the current estimated biases $\hat{b}_g$ and $\hat{b}_a$ (and Earth's rotation $\omega_G$ if operating in an Earth-Centered, Earth-Fixed [ECEF] global frame).

### Bias-Corrected Linear Acceleration

$$
\hat{a} = a_m - \hat{b}_a
$$

### Bias-Corrected Angular Velocity (Full ECEF Model)

For large-scale outdoor navigation where Earth's rotation is non-negligible:

$$
\hat{\omega} = \omega_m - \hat{b}_g - C_{\hat{q}}\omega_G
$$

### Bias-Corrected Angular Velocity (Simplified Local Model)

For indoor or small-scale navigation where $\omega_G \approx 0$:

$$
\hat{\omega} = \omega_m - \hat{b}_g
$$

> **Note on High-Rate Visual Sensors (e.g., Event Cameras):**
> If visual updates arrive at a higher frequency than IMU samples, hold the most recently received IMU reading $(\omega_m, a_m)$ to propagate across the shorter interval $\Delta t$.

## 3. Step 2: Continuous-Time State Differential Equations

Applying the expectation operator to the continuous-time kinematic equations yields the state derivative function used inside the integrator:

$$
\dot{\hat{X}}_{IMU} = f(\hat{X}_{IMU}, \omega_m, a_m)
$$

### Option A: Full ECEF Model (Accounting for Earth's Rotation)

Use this model when $\lbrace G \rbrace$ is an ECEF frame and Coriolis/centrifugal effects are non-negligible:

$$
\begin{aligned}
\mathstrut_G^I\dot{\hat{\overline{q}}} &= \frac{1}{2}\Omega(\hat{\omega}) \mathstrut_G^I\hat{\overline{q}} \cr
\dot{\hat{b}}_g &= 0_{3 \times 1} \cr
{}^G\dot{\hat{v}}_I &= C_{\hat{q}}^T \hat{a} - 2\lfloor\omega_G \times\rfloor {}^G\hat{v}_I - \lfloor\omega_G \times\rfloor^2 {}^G\hat{p}_I + {}^G g \cr
\dot{\hat{b}}_a &= 0_{3 \times 1} \cr
{}^G\dot{\hat{p}}_I &= {}^G\hat{v}_I
\end{aligned}
$$

### Option B: Simplified Local-Frame Model (Neglecting Earth's Rotation)

Use this model for indoor robotics, drones, or short-range visual-inertial odometry where $\omega_G \approx 0$:

$$
\begin{aligned}
\mathstrut_G^I\dot{\hat{\overline{q}}} &= \frac{1}{2}\Omega(\omega_m - \hat{b}_g) \mathstrut_G^I\hat{\overline{q}} \cr
\dot{\hat{b}}_g &= 0_{3 \times 1} \cr
{}^G\dot{\hat{v}}_I &= C_{\hat{q}}^T (a_m - \hat{b}_a) + {}^G g \cr
\dot{\hat{b}}_a &= 0_{3 \times 1} \cr
{}^G\dot{\hat{p}}_I &= {}^G\hat{v}_I
\end{aligned}
$$

### Supporting Matrix Definitions

For any 3D angular rate vector $\omega = [\omega_x, \omega_y, \omega_z]^T$, the $4 \times 4$ quaternion kinematic matrix $\Omega(\omega)$ and the $3 \times 3$ skew-symmetric cross-product matrix $\lfloor\omega \times\rfloor$ are defined as:

$$
\Omega(\omega) = \begin{bmatrix}
-\lfloor\omega \times\rfloor & \omega \cr
-\omega^T & 0
\end{bmatrix}, \qquad
\lfloor\omega \times\rfloor = \begin{bmatrix}
0 & -\omega_z & \omega_y \cr
\omega_z & 0 & -\omega_x \cr
-\omega_y & \omega_x & 0
\end{bmatrix}
$$

Here, ${}^G g \in \mathbb{R}^3$ is the known gravitational acceleration vector expressed in the global frame $\lbrace G \rbrace$.

## 4. Step 3: 5th-Order Runge-Kutta (RK5) Numerical Integration

To propagate $\hat{X}_{IMU}$ across the fixed IMU sampling interval $T$ (from $t_k$ to $t_k + T$), use **Butcher's explicit 6-stage, 5th-Order Runge-Kutta (RK5)** method.

### 4.1. Define the Active State and Derivative Function

During a single propagation interval $[t_k, t_k + T]$, the raw IMU measurements $(\omega_m, a_m)$ are held constant (Zero-Order Hold), and the bias derivatives are zero ($\dot{\hat{b}}_g = 0$, $\dot{\hat{b}}_a = 0$). Because the biases do not evolve during propagation, you only need to integrate the 10-element active pose-and-velocity state $y$:

$$
y = \begin{bmatrix}
\mathstrut_G^I\hat{\overline{q}} \cr
{}^G\hat{v}_I \cr
{}^G\hat{p}_I
\end{bmatrix}
$$

Given any state $y$, the derivative function $f(y) = \dot{y}$ returns the 10-element slope vector $[\dot{\overline{q}}^T, \dot{v}^T, \dot{p}^T]^T$.

#### Full ECEF Model

$$
f(y) = \begin{bmatrix}
\frac{1}{2}\Omega\left(\omega_m - \hat{b}_g - C(\hat{\overline{q}})\omega_G\right)\hat{\overline{q}} \cr
C(\hat{\overline{q}})^T (a_m - \hat{b}_a) - 2\lfloor\omega_G \times\rfloor {}^G\hat{v}_I - \lfloor\omega_G \times\rfloor^2 {}^G\hat{p}_I + {}^G g \cr
{}^G\hat{v}_I
\end{bmatrix}
$$

#### Simplified Local Model

When Earth's rotation is negligible ($\omega_G = 0$):

$$
f(y) = \begin{bmatrix}
\frac{1}{2}\Omega(\omega_m - \hat{b}_g)\hat{\overline{q}} \cr
C(\hat{\overline{q}})^T (a_m - \hat{b}_a) + {}^G g \cr
{}^G\hat{v}_I
\end{bmatrix}
$$

Because the IMU measurements are held constant across $T$, $f(y)$ has no explicit dependence on time $t$—it depends solely on the intermediate state $y$ passed into it.

### 4.2. Evaluate the Six RK5 Stage Equations

Starting from the current active state $y_k$ at time $t_k$ with step size $T$, evaluate the six 10-element derivative vectors ($k_1$ through $k_6$) sequentially:

$$
\begin{aligned}
k_1 &= f\left(y_k\right) \cr
k_2 &= f\left(y_k + \frac{1}{4} T k_1\right) \cr
k_3 &= f\left(y_k + \frac{1}{8} T k_1 + \frac{1}{8} T k_2\right) \cr
k_4 &= f\left(y_k - \frac{1}{2} T k_2 + T k_3\right) \cr
k_5 &= f\left(y_k + \frac{3}{16} T k_1 + \frac{9}{16} T k_4\right) \cr
k_6 &= f\left(y_k - \frac{3}{7} T k_1 + \frac{2}{7} T k_2 + \frac{12}{7} T k_3 - \frac{12}{7} T k_4 + \frac{8}{7} T k_5\right)
\end{aligned}
$$

Each stage $k_i$ represents the rate of change $[\dot{\overline{q}}_i^T, \dot{v}_i^T, \dot{p}_i^T]^T$ sampled at a different fractional point across $[t_k, t_k + T]$ ($c = 0, \frac{1}{4}, \frac{1}{4}, \frac{1}{2}, \frac{3}{4}, 1$).

### 4.3. Final Weighted State Update

Combine the six stage slopes to advance the active state from $t_k$ to $t_{k+1} = t_k + T$:

$$
y_{k+1} = y_k + \frac{T}{90}\left(7k_1 + 32k_3 + 12k_4 + 32k_5 + 7k_6\right)
$$

Note that $k_2$ has a weight of $0$ in this final sum; it acts solely as an intermediate predictor used to evaluate $k_3$, $k_4$, and $k_6$.

The estimated sensor biases remain unchanged across the propagation step:

$$
\hat{b}_{g, k+1} = \hat{b}_{g, k}, \qquad \hat{b}_{a, k+1} = \hat{b}_{a, k}
$$

> **Note on Alternative RK5 Tableaus:**
> If your codebase delegates integration to an external numerical library (e.g., Boost Odeint or SciPy), it may employ an embedded 6-stage 5th-order Butcher tableau such as **Cash-Karp** or **Runge-Kutta-Fehlberg (RKF45)**, which swap the fractional weights to support adaptive step-size error estimation. For fixed-rate IMU intervals $T$, Butcher's RK5 formulation above is direct and avoids unnecessary error-estimation overhead.

## 5. Step 4: Post-Integration Quaternion Normalization

Linear combinations of quaternions during RK5 integration do not strictly preserve unit norm ($\Vert\overline{q}\Vert_2 = 1$). Handle normalization at two points:

### 1. Intermediate Stages Inside the Derivative Function

Because the arguments passed into $f(y)$ for stages $k_2$ through $k_6$ linearly add scaled quaternion derivatives to the base quaternion, normalize the temporary intermediate quaternion before converting it to the rotation matrix $C(\hat{\overline{q}}_{\text{temp}})$ so that the rotation matrix remains strictly orthogonal ($C \in SO(3)$):

$$
\hat{\overline{q}}_{\text{temp}} \leftarrow \frac{\hat{\overline{q}}_{\text{temp}}}{\Vert\hat{\overline{q}}_{\text{temp}}\Vert_2}
$$

### 2. Post-Update Normalization

Immediately after computing $y_{k+1}$, re-normalize the updated orientation quaternion to preserve the unit-quaternion constraint:

$$
\mathstrut_G^I\hat{\overline{q}}(t_{k+1}) \leftarrow \frac{\mathstrut_G^I\hat{\overline{q}}(t_{k+1})}{\Vert\mathstrut_G^I\hat{\overline{q}}(t_{k+1})\Vert_2}
$$

## 6. Step 5: Companion EKF Covariance Propagation

Updating the IMU state mean must be paired with propagating the EKF error-state covariance matrix. The $15 \times 1$ IMU error-state is defined as:

$$
\tilde{X}_{IMU} = \begin{bmatrix} \delta\theta_I^T & \tilde{b}_g^T & {}^G\tilde{v}_I^T & \tilde{b}_a^T & {}^G\tilde{p}_I^T \end{bmatrix}^T
$$

where attitude error uses the multiplicative error quaternion $\overline{q} = \delta\overline{q} \otimes \hat{\overline{q}}$ with $\delta\overline{q} \simeq [\frac{1}{2}\delta\theta^T, 1]^T$.

### 1. Partition the Covariance Matrix

Split the full EKF covariance $P_{k \vert k}$ into IMU ($15 \times 15$), Camera ($6N \times 6N$), and cross-correlation ($15 \times 6N$) blocks:

$$
P_{k \vert k} = \begin{bmatrix}
P_{II_{k \vert k}} & P_{IC_{k \vert k}} \cr
P_{IC_{k \vert k}}^T & P_{CC_{k \vert k}}
\end{bmatrix}
$$

### 2. Linearized Continuous-Time Error Dynamics

$$
\dot{\tilde{X}}_{IMU} = F\tilde{X}_{IMU} + G n_{IMU}
$$

where $n_{IMU} = [n_g^T, n_{wg}^T, n_a^T, n_{wa}^T]^T$ has calibrated continuous-time noise covariance $Q_{IMU}$, and the system matrices $F$ and $G$ are:

$$
F = \begin{bmatrix}
-\lfloor\hat{\omega}\times\rfloor & -I_3 & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} \cr
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} \cr
-C_{\hat{q}}^T\lfloor\hat{a}\times\rfloor & 0_{3\times 3} & -2\lfloor\omega_G\times\rfloor & -C_{\hat{q}}^T & -\lfloor\omega_G\times\rfloor^2 \cr
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} \cr
0_{3\times 3} & 0_{3\times 3} & I_3 & 0_{3\times 3} & 0_{3\times 3}
\end{bmatrix}
$$

$$
G = \begin{bmatrix}
-I_3 & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} \cr
0_{3\times 3} & I_3 & 0_{3\times 3} & 0_{3\times 3} \cr
0_{3\times 3} & 0_{3\times 3} & -C_{\hat{q}}^T & 0_{3\times 3} \cr
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & I_3 \cr
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3}
\end{bmatrix}
$$

Note: In the simplified local model, set $\omega_G = 0_{3 \times 1}$ in $F$.

### 3. Integrate IMU Covariance and State Transition Matrix

Numerically integrate over $\tau \in [0, T]$:

$$
\begin{aligned}
\dot{P}_{II} &= F P_{II} + P_{II} F^T + G Q_{IMU} G^T, && P_{II}(t_k) = P_{II_{k \vert k}} \cr
\dot{\Phi}(t_k + \tau, t_k) &= F \Phi(t_k + \tau, t_k), && \Phi(t_k, t_k) = I_{15}
\end{aligned}
$$

### 4. Reassemble the Propagated Covariance Matrix

$$
P_{k+1 \vert k} = \begin{bmatrix}
P_{II_{k+1 \vert k}} & \Phi(t_k + T, t_k) P_{IC_{k \vert k}} \cr
P_{IC_{k \vert k}}^T \Phi(t_k + T, t_k)^T & P_{CC_{k \vert k}}
\end{bmatrix}
$$

## 7. Implementation Notes and Common Pitfalls

* **Quaternion Library Mismatch (Eigen / ROS / SciPy):** Standard libraries like Eigen (`Eigen::Quaterniond`) and ROS typically follow **Hamilton** conventions, whereas MSCKF uses **JPL** conventions. Always unit-test your quaternion-to-rotation-matrix conversion and $\Omega(\omega)$ matrix against a known pure rotation (e.g., constant $\omega_z = 1\text{ rad/s}$ for $1\text{ s}$) before integrating into the filter.
* **Gravity Vector Sign:** Because an accelerometer at rest measures specific force $+g$ pointing upward (opposite to gravity), ensure the sign of ${}^G g$ in ${}^G\dot{\hat{v}}_I = C(\hat{\overline{q}})^T \hat{a} + {}^G g$ cancels out the static accelerometer reading (i.e., when stationary and level, $C(\hat{\overline{q}})^T a_m + {}^G g = 0$).
* **Floating-Point Covariance Symmetry:** After numerical integration of the covariance matrix, enforce symmetry via $P_{II} \leftarrow \frac{1}{2}(P_{II} + P_{II}^T)$ to prevent numerical drift from breaking positive semi-definiteness.
* **Zero-Bias Integration Efficiency:** Because $\dot{\hat{b}}_g = 0$ and $\dot{\hat{b}}_a = 0$, integrating only the 10-element active state $y = [\hat{\overline{q}}^T, \hat{v}^T, \hat{p}^T]^T$ (as shown in Section 4) avoids redundant zero arithmetic across all six RK5 stages.

## 8. References

1. A. I. Mourikis and S. I. Roumeliotis, *"A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation,"* IEEE International Conference on Robotics and Automation (ICRA), 2007. (Section III-A & III-B).
2. A. Z. Zhu, N. Atanasov, and K. Daniilidis, *"Event-based Visual Inertial Odometry,"* IEEE Conference on Computer Vision and Pattern Recognition (CVPR), 2017. (Section 6).