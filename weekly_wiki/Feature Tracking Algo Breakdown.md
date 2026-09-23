# **Feature tracking algorithm breakdown**

## **Overview of the Algorithm and Block Diagram**

<img width="821" height="680" alt="image" src="https://github.com/user-attachments/assets/94c29962-10ef-4a0b-8dcc-8e31cf258582" />


<img width="1024" height="559" alt="image" src="https://github.com/user-attachments/assets/a87d6f36-7234-471f-a00b-24621b733a54" />


## The sequence of feature tracking algorithm is as follows:

#### **Step 1:** Spatiotemporal Neighborhood Extraction ($W_i$)
#### **Step 2:** EM 1 — Optical Flow Optimization ($u$)
#### **Step 3:** EM 2 — Template Alignment & Drift Correction ($\sigma, b$)
#### **Step 4:** Feature Position Update & Adaptive Window Sizing

## Step 1: Spatiotemporal Neighborhood Extraction ($W_i$)
### Goal: 
#### Extract only the events in the current time slice $[T_i, T_i + dt_i]$ that belong to the local spatial neighborhood of feature $f(T_i)$. This restricts association searches to a local window and enforces the assumption that optical flow is locally uniform.

### Variables: 
- $T_i$: Start time of current processing window
- $dt_i$: Window duration
- $f(T_i) \in \mathbb{R}^2$: Feature center coordinates in the image plane at $T_i$
- $e_k = (x_k, t_k)$: Raw event with coordinates $x_k \in \mathbb{R}^2$ and timestamp $t_k \in \mathbb{R}$
- $\bar{t}_k = t_k - T_i$: Elapsed time of an event from window start
- $u \in \mathbb{R}^2$: Optical flow vector (velocity in pixels/second)
- $\xi$: Spatial search radius in pixels (e.g., $15$ px for a $31 \times 31$ patch)

### Outputs:
#### $W_i$: Resulting set of $n_i$ events inside the spatiotemporal bounds

### Analytical Process:
#### 1. Query the event stream for events $e_k$ satisfying $t_k \in [T_i, T_i + dt_i]$
#### 2. For each candidate event, apply a linear flow displacement along the previous flow vector: <img width="96" height="27" alt="image" src="https://github.com/user-attachments/assets/2c68bc05-f966-4f88-b5a0-40ccf196feec" />
#### 3. Retain events whose displaced positions fall within radius $\xi$ of the feature position:
$$
\|(x_k - \bar{t}_k u_{i-1}) - f(T_i)\| \le \xi
$$
#### 4. Store passing events in buffer $W_i$ for EM 1

## Step 2: EM 1 — Optical Flow Estimation ($u$)
### Goal:
#### Estimate the 2D velocity vector $u = [u_x, u_y]^T$ of the feature within $[T_i, T_i + dt_i]$. Because event-to-landmark association is unknown, an Expectation-Maximization (EM) loop iteratively resolves data associations (E-Step) and updates velocity via closed-form weighted least squares (M-Step)

### Variables:

- <img width="70" height="25" alt="image" src="https://github.com/user-attachments/assets/c000dd84-aca8-4ec1-8f98-1c505c5619ee" /> : Prior template points, formed by forward-propagating the previous window's events to the current timestamp: <img width="300" height="25" alt="image" src="https://github.com/user-attachments/assets/b691713d-8d36-49fb-943e-a297db568c0a" />
- $r_{kj} \in [0, 1]$: Soft association probability that event $k$ originated from template point $j$.
- $\Sigma$: Measurement noise covariance matrix (set to $2I$).
- $\phi(z; \mu, \Sigma)$: 2D Gaussian probability density function centered at $\mu$ with covariance $\Sigma$.
- $\epsilon_1$: Convergence tolerance threshold for the flow cost.

### Analytical Process
#### 1. Expectation (E-Step): Displace current events backward to $T_i$ via $x_k - \bar{t}_k u$ and compute posterior probability weights against template points:
####  <img width="578" height="96" alt="image" src="https://github.com/user-attachments/assets/db10868f-b9f3-438c-97ed-daa46389157f" />

#### 2. Maximization (M-Step): Fix weights $r_{kj}$ and compute the optimal flow vector $u$ minimizing $\sum_k \sum_j r_{kj} \Vert{}(x_k - \bar{t}_k u) - \tilde{l}_j^{i-1}\Vert{}^2$:
#### <img width="576" height="87" alt="image" src="https://github.com/user-attachments/assets/0f9ec6d1-38f7-4edd-aa88-26919983ca88" />

#### 3. Termination: Repeat until the change in cost function is below $\epsilon_1$. Once converged, set $x'_k = x_k - \bar{t}_k u$. 


## Step 3: EM 2 — Template Alignment & Drift Correction ($\sigma, b$)
### Goal: 
#### Eliminate cumulative tracking drift by aligning current flow-corrected events with the canonical reference template <img width="25" height="40" alt="image" src="https://github.com/user-attachments/assets/79aeb4db-353a-4608-ae28-e089ef750b78" /> captured at feature birth ($T_{i*}$). Known rotation from the filter eliminates angular degrees of freedom, leaving only a scale $\sigma$ and 2D translation offset $b$ to estimate.   

### Variables:
- $T_{i*}$: Timestamp when the feature was first instantiated
- <img width="25" height="30" alt="image" src="https://github.com/user-attachments/assets/3ec7be93-f94b-426d-8b93-195593306473" /> : Anchor template point coordinates recorded at $T_{i*}$
- $^{i*}R_i \in SO(3)$: Relative 3D rotation from the current frame to the initial anchor frame (provided by filter state)
- $^{i*}R_i \in SO(3)$: Relative 3D rotation from the current frame to the initial anchor frame (provided by filter state)
- $y_k^i$: Rotated, feature-centered coordinates of the current propagated events: <img width="511" height="67" alt="image" src="https://github.com/user-attachments/assets/2751c4b6-42ac-4ee3-b7b5-c90a8a27bc22" />
- $\sigma \in \mathbb{R}$: Estimated scale variation
- $b \in \mathbb{R}^2$: Translational drift correction vector
- $\bar{y}, \bar{l}$: Centroids of $\{y_k\}$ and $\{\tilde{l}_j^{i*}\}$ respectively- 


### Analytical Process: 
#### 1. Pose Rotation & Centering: Forward-propagate events to $T_{i+1}$, project them into the coordinate frame of $T_{i*}$ via $^{i*}R_i$, and subtract the projected feature center estimate
#### 2. Expectation (E-Step): Compute correspondence probabilities between warped points $(\sigma y_k - b)$ and reference points $\tilde{l}_j^{i*}$: <img width="516" height="90" alt="image" src="https://github.com/user-attachments/assets/bf9f999b-78ed-41ee-b61e-153415d20640" />
#### 3. Maximization (M-Step): Solve for optimal scale $\sigma$ and translation $b$ using scaled Iterative Closest Point (ICP): <img width="417" height="96" alt="image" src="https://github.com/user-attachments/assets/e5477398-d408-4ddf-9998-50f6a12770a5" /> <img width="412" height="232" alt="image" src="https://github.com/user-attachments/assets/3649e5e4-0e05-48f3-a697-01384cdff7fd" />
#### 4. State Adjustment: Iterate until cost change $< \epsilon_2$. Update the tracked feature center by combining estimated flow displacement with translation correction: <img width="225" height="35" alt="image" src="https://github.com/user-attachments/assets/f85f2273-1ee5-416b-b7e5-2fa1bc074cae" />

## Step 4: Feature Position Update & Adaptive Window Sizing
### Goal: 
#### Calculate the temporal window duration $dt_{i+1}$ for the next tracking cycle. This ensures features travel roughly $k = 3$ pixels per window regardless of sensor velocity, preventing motion blur from violating the constant optical flow assumption
### Variables:
- $k$: Target pixel travel distance (fixed at $3$ pixels)
- $\Vert{}u_m\Vert{}_2$: Euclidean magnitude of optical flow for feature $m$
- $\mathcal{F}$: The full set of currently tracked features
- $dt(f_m) = \frac{k}{\Vert{}u_m\Vert{}_2}$: Window lifetime for feature $m$
- $dt_{i+1}$: Window duration adopted for the subsequent tracking iteration

###Analytical Process:
#### 1. For each successfully tracked feature $m \in \mathcal{F}$, compute its travel time for $k$ pixels: $dt(f_m) = \frac{3}{\Vert{}u_m\Vert{}_2}$
#### 2. Take the median over all features to reject outliers caused by degeneracies or aperture issues <img width="418" height="97" alt="image" src="https://github.com/user-attachments/assets/fe453db5-f126-4b77-bfcd-b6ea2b60a543" />
#### 3. Pass $dt_{i+1}$ along with updated feature positions $\{f\}$ to the MSCKF state estimator and the next tracker call






