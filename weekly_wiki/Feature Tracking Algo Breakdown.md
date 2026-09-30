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
#### 1. Expectation (E-Step): Discard invalid event-to-template pairs to accelerate correspondence matching and avoid tracking corruption. Also find the weighted probability that event originated from template point
a. Displace current events backward to $T_i$: <img width="105" height="25" alt="image" src="https://github.com/user-attachments/assets/4e1aa22b-aaba-452f-a1e2-5cb142a2f4a2" />

b. Outlier Rejection: Compute Mahalanobis distance between displaced event <img width="105" height="25" alt="image" src="https://github.com/user-attachments/assets/4e1aa22b-aaba-452f-a1e2-5cb142a2f4a2" /> and landmark projection <img width="25" height="25" alt="image" src="https://github.com/user-attachments/assets/f69c982c-ff11-42d3-9f83-c6ed8a752a13" />. 
- Mahalanobis distance: <img width="917" height="115" alt="image" src="https://github.com/user-attachments/assets/0b6ebddc-7b0c-41e1-bd0a-980e954d1525" />
We only want pairs (k, j) where $d_{k,j}$ <= 4 pixels
Set $r_{kj}$ = 0 for all pairs where $d_{kj}$ > threashold



c. Displace current events backward to $T_i$ via $x_k - \bar{t}_k u$ and compute posterior probability weights against template points:
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
- $y_k^i$: Rotated, feature-centered coordinates of the current propagated events: <img width="511" height="67" alt="image" src="https://github.com/user-attachments/assets/2751c4b6-42ac-4ee3-b7b5-c90a8a27bc22" />
- $\sigma \in \mathbb{R}$: Estimated scale variation
- $b \in \mathbb{R}^2$: Translational drift correction vector
- $\bar{y}, \bar{l}$: Centroids of $\{y_k\}$ and $\{\tilde{l}_j^{i*}\}$ respectively- 


### Analytical Process: 
#### 1. Extract rotation prior from EKF State
##### Goal: Remove orientation differences between the current camera frame and the keyframe template
##### Variables used
- $^{i*}R_i$ : 3D rotation matrix transforming coordinates from the current camera pose at time $T_i$ to the initial template frame at time $T_{i^*}$
- $s_i$: Sensor state vector
##### Steps:
1. Retrieve $\bar{q}(T_{i^*})$ (Initial Feature Pose) and $\bar{q}(T_i)$ (Current Camera Pose) from sensor state vector for time interval from $s_i$
2. Compute the relative rotation $^{i^*}R_i$ relative to the camera pose where the feature template was first initialized
##### Output: 
- Relative rotation matrix $^{i^*}R_i$
_____________________________________________________________________________________________________
#### 2. Rotate and Center Current Feature Points
##### Goal: Project forward-propagated event points into the template coordinate frame and center them around the expected feature center
##### Variables used:
-  <img width="24" height="30" alt="image" src="https://github.com/user-attachments/assets/47e39bd9-0d9c-4593-8680-690307a43a45" /> : Forward-propagated event points at the end of the current window $T_{i+1}$(defined in Equation 7)
-  <img width="352" height="32" alt="image" src="https://github.com/user-attachments/assets/75c425e3-a31a-43e9-806c-b82b1f91b7c1" />
- $f(T_i)$: Feature position at time $T_i$
- $u_i$: Converged optical flow vector from the preceding EM step
- $dt_i$: Window duration ($T_{i+1} - T_i$)
- $\pi(\cdot)$: Standard perspective projection function $\pi([X, Y, Z]^T) = [X/Z, Y/Z]^T$
- $y_k^i$: Rotated and centered 2D point
##### Steps
1. Estimate the uncorrected feature position at time $T_{i+1}$ using linear propagation: $f(T_i) + u_i dt_i$
2. Append depth/homogeneous coordinates to each propagated point <img width="24" height="30" alt="image" src="https://github.com/user-attachments/assets/47e39bd9-0d9c-4593-8680-690307a43a45" /> and the estimated center $f(T_i) + u_i dt_i$
3. Rotate both by $^{i^*}R_i$ and project back into normalized image coordinates via $\pi(\cdot)$:
- <img width="491" height="71" alt="image" src="https://github.com/user-attachments/assets/7547ef69-0595-4a30-a202-34842468aef7" />
##### Output:
- Set of centered, rotation-compensated points $\{y_k^i\}_{k=1}^{n_i}$
_____________________________________________________________________________________________________
#### 3. Outlier Rejection via Distance Gating
##### Goal: Discard invalid event-to-template pairs to accelerate correspondence matching and avoid tracking corruption
##### Variables:
- <img width="24" height="30" alt="image" src="https://github.com/user-attachments/assets/47e39bd9-0d9c-4593-8680-690307a43a45" /> : Points in the reference template recorded at onset time $T_{i^*}$
- $\Sigma$: Measurement covariance matrix (set to $2I$)
- $d_{\text{Mahalanobis}}$: Threshold distance (set to ?????? pixels)
##### Steps:
1. Query the pre-built $k$-d tree of template points <img width="24" height="30" alt="image" src="https://github.com/user-attachments/assets/47e39bd9-0d9c-4593-8680-690307a43a45" /> for each centered point <img width="78" height="27" alt="image" src="https://github.com/user-attachments/assets/c0d2ccbd-756a-4eb5-84eb-e3f7983e73d3" />

2. Filter out point pairs whose Mahalanobis distance exceeds ?????? pixels similar to EM 1
   <img width="517" height="72" alt="image" src="https://github.com/user-attachments/assets/7c4cbb4d-99d2-4b6d-90ae-98c3370b2cee" />

##### Output: 
- Pruned set of candidate pairs $(y_k^i, \tilde{l}_j^{i^*})$
_____________________________________________________________________________________________________
#### 4. Expectation Step (E-Step)
##### Goal: Compute probabilistic data association weights between the current points and template points
##### Variables: 
- <img width="75" height="25" alt="image" src="https://github.com/user-attachments/assets/e2f75f75-24d1-4d35-b7a4-b3d8d6b306d1" /> : Gaussian probability density function evaluated at $y_k$ with mean $\tilde{l}_j^{i^*}$ and covariance $\Sigma$
- $r_{kj}$: Posterior probability that point $y_k$ corresponds to template landmark $j$
##### Steps: 
1. Evaluate the Gaussian likelihood of each active pair
2. Normalize over all candidate template landmarks $j'$ associated with event $k$:
- <img width="230" height="40" alt="image" src="https://github.com/user-attachments/assets/f550237d-f670-4e18-8432-719bc1ee30a7" />
##### Output:
- Normalized association weight matrix $r_{kj}$ (where $\sum_j r_{kj} = 1$
_____________________________________________________________________________________________________
#### 5. Maximization Step (M-Step)
##### Goal: Find the optimal scale $\sigma$ and translation offset $b$ in closed form given the current association probabilities
##### Variables:
- $\bar{y}$: Centroid of current transformed points
- $\bar{l}$: Centroid of reference template points
- $\sigma$: Scale parameter
- $b$: 2D translation offset
##### Steps:
1. Compute the empirical centroids:
- <img width="302" height="67" alt="image" src="https://github.com/user-attachments/assets/799dc81b-30ef-4ebc-8a6e-ec917f9f000a" />
2. Solve for the scale factor $\sigma$ via scaled ICP:
- <img width="402" height="95" alt="image" src="https://github.com/user-attachments/assets/04ba9913-d5fc-4fcb-8b87-fbee53311011" />
3. Solve for the 2D alignment translation $b$:
  - <img width="397" height="72" alt="image" src="https://github.com/user-attachments/assets/db5eb32a-b075-4bab-8c96-2b74fa5b97f5" />
##### Output:
- Updated parameters $(\sigma, b)$
_____________________________________________________________________________________________________
#### 6. Cost Evaluation & Convergence Check
##### Goal: Evaluate the weighted residual error to determine if the EM loop has converged
##### Variables: 
- $\text{cost}$: Total residual alignment mismatch
- $\epsilon_2$: Convergence threshold
##### Steps: 
1. Compute the updated objective function value:
- <img width="253" height="62" alt="image" src="https://github.com/user-attachments/assets/c0430737-d7ee-4543-97f9-52b1c7997646" />
3. If $\text{cost} > \epsilon_2$, repeat from Step 4 (E-Step)
4. If $\text{cost} \le \epsilon_2$, exit the loop
#### Output:
- Final converged translation offset $b$ and scale $\sigma$
_____________________________________________________________________________________________________
#### 7. Apply Feature Drift Correction
##### Goal: Adjust the feature's 2D position by integrating the optical flow and subtracting the template translation error
##### Variables: 
- $f$: 2D feature coordinates
- $u$: Optical flow velocity
- $dt_i$: Window interval
- $b$: Converged translation offset from template alignment
##### Steps:
1. Propagate the feature coordinate along flow vector $u$ over window duration $dt_i$
2. Subtract the estimated alignment offset $b$ to cancel accumulated drift:
- <img width="137" height="28" alt="image" src="https://github.com/user-attachments/assets/3d17c2c6-e551-47c7-836d-743ec4384e2d" />
##### Output:
- Corrected 2D feature position $f(T_{i+1})$ ready for track server storage and odometry state updates

## Step 4: Feature Position Update & Adaptive Window Sizing
### Goal: 
#### Complete 2-point RANSAC to remove features whose tracking has failed. Then calculate the temporal window duration $dt_{i+1}$ for the next tracking cycle. This ensures features travel roughly $k = 3$ pixels per window regardless of sensor velocity, preventing motion blur from violating the constant optical flow assumption
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






