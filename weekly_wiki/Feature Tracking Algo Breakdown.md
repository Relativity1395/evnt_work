# **Feature tracking algorithm breakdown**

## **Overview of the Algorithm and Block Diagram**

<img width="821" height="680" alt="image" src="https://github.com/user-attachments/assets/94c29962-10ef-4a0b-8dcc-8e31cf258582" />


<img width="1024" height="559" alt="image" src="https://github.com/user-attachments/assets/a87d6f36-7234-471f-a00b-24621b733a54" />


## The sequence of feature tracking algorithm is as follows:

1. Spatiotemporal Neighborhood Extraction ($W_i$)
2. EM 1 — Optical Flow Optimization ($u$)
3. EM 2 — Template Alignment & Drift Correction ($\sigma, b$)
4. Feature Position Update & Adaptive Window Sizing

## Step 1: Spatiotemporal Neighborhood Extraction ($W_i$)
### Goal: Extract only the events in the current time slice $[T_i, T_i + dt_i]$ that belong to the local spatial neighborhood of feature $f(T_i)$. This restricts association searches to a local window and enforces the assumption that optical flow is locally uniform.

### Inputs: 
#### $f(T_i) \in \mathbb{R}^2$: Current estimated feature location on the image plane at time $T_i$.
#### $\mathcal{E}$: Incoming stream of raw events, where each event $k$ is $(x_k, t_k)$ with pixel coordinates $x_k \in \mathbb{R}^2$ and timestamp $t_k \in \mathbb{R}$. 
#### $dt_i \in \mathbb{R}$: Current temporal window length
#### $\xi \in \mathbb{R}$: Spatial neighborhood radius in pixels (typically a $31 \times 31$ window, giving $\xi = 15$). 
#### $u_{i-1} \in \mathbb{R}^2$: Flow estimate from the previous time step (used as prior). 

### Outputs:
#### $W_i = \{(x_k, t_k)\}_{k=1}^{n_i}$: Subset of $n_i$ events belonging to the feature's spatiotemporal bounding volume.

## Step 2: EM 1 — Optical Flow Optimization ($u$)

## Step 3: EM 2 — Template Alignment & Drift Correction ($\sigma, b$)

## Step 4: Feature Position Update & Adaptive Window Sizing



