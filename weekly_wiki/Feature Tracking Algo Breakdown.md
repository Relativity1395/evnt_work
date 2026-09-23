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

### Inputs: 
#### $f(T_i) \in \mathbb{R}^2$: Current estimated feature location on the image plane at time $T_i$.
#### $\mathcal{E}$: Incoming stream of raw events, where each event $k$ is $(x_k, t_k)$ with pixel coordinates $x_k \in \mathbb{R}^2$ and timestamp $t_k \in \mathbb{R}$. 
#### $dt_i \in \mathbb{R}$: Current temporal window length
#### $\xi \in \mathbb{R}$: Spatial neighborhood radius in pixels (typically a $31 \times 31$ window, giving $\xi = 15$). 
#### $u_{i-1} \in \mathbb{R}^2$: Flow estimate from the previous time step (used as prior). 

### Outputs:
#### $W_i = \{(x_k, t_k)\}_{k=1}^{n_i}$: Subset of $n_i$ events belonging to the feature's spatiotemporal bounding volume.

### Theoretical Breakdown
#### 1. $W_i = \{(x_k, t_k)\}_{k=1}^{n_i}$: Subset of $n_i$ events belonging to the feature's spatiotemporal bounding volume.
#### 2. Compute relative time offset: $\bar{t}_k = t_k - T_i$
#### 3. Apply spatial boundary check around feature position $f(T_i)$: 

$$
\|(x_k - \bar{t}_k u_{i-1}) - f(T_i)\| \le \xi
$$

#### 4. Store passing events in buffer $W_i$ for EM 1

## Step 2: EM 1 — Optical Flow Optimization ($u$)
### Goal:
#### Determine the 2D velocity vector $u = [u_x, u_y]^T$ that aligns events in $W_i$ with the forward-propagated landmark positions from the previous step ($\tilde{l}^{i-1}$), canceling motion blur within the window

### Inputs:
#### $W_i = \{(x_k, t_k)\}_{k=1}^{n_i}$: Extracted events
#### <img width="70" height="25" alt="image" src="https://github.com/user-attachments/assets/c000dd84-aca8-4ec1-8f98-1c505c5619ee" /> : Prior template points, formed by forward-propagating the previous window's events to the current timestamp: <img width="300" height="25" alt="image" src="https://github.com/user-attachments/assets/b691713d-8d36-49fb-943e-a297db568c0a" />
#### $\Sigma = 2I_{2 \times 2}$: Measurement covariance matrix

### Outputs:
#### $u \in \mathbb{R}^2$: Refined local optical flow vector for the window
#### $x'_k = x_k - \bar{t}_k u$: Motion-compensated event locations aligned at time $T_i$

### Theoretical Breakdown
#### 1. Initialize: Set $u \leftarrow u_{i-1}$. Build a static KD-Tree over the decimated previous template points $\{\tilde{l}_j^{i-1}\}$
#### 2. Loop until $\Delta \text{Cost} < \epsilon_1$ or max iterations reached:
- Expectation (E-Step) — Soft Correspondence: Propagate each event backwards to $T_i$ using current $u$: $x_{\text{back}, k} = <img width="83" height="31" alt="image" src="https://github.com/user-attachments/assets/b89edd60-1f7e-4973-b271-ecec45ae2972" />
$. Query the KD-Tree for template points within the 4-pixel Mahalanobis threshold. Compute the soft association probability $r_{kj}$ via Gaussian weighting:

## Step 3: EM 2 — Template Alignment & Drift Correction ($\sigma, b$)

## Step 4: Feature Position Update & Adaptive Window Sizing



