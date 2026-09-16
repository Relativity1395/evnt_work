# **Feature tracking algorithm breakdown**

## **Overview of the Algorithm and Block Diagram**

<img width="821" height="680" alt="image" src="https://github.com/user-attachments/assets/94c29962-10ef-4a0b-8dcc-8e31cf258582" />


<img width="1024" height="559" alt="image" src="https://github.com/user-attachments/assets/a87d6f36-7234-471f-a00b-24621b733a54" />

<img width="1024" height="559" alt="image" src="https://github.com/user-attachments/assets/8a208269-8d7e-4f65-acc6-9bbf782e487c" />

## Recap of inputs

### Sensor state, si
<img width="591" height="88" alt="image" src="https://github.com/user-attachments/assets/cca2465e-a2bf-4986-a4ff-631cc067763e" />

we only require q, v, and p for Algo 2

q is the rotation

v is the velocity

p is the poistion

### Time Window Boundaries, $T_i$

<img width="458" height="108" alt="image" src="https://github.com/user-attachments/assets/c141c9c3-46d1-4f58-ad8a-ec86a7deaa27" />

$T_i$ and $T_{i+1}$ are the start and end points of our Time Window

### Window Size dt

<img width="405" height="103" alt="image" src="https://github.com/user-attachments/assets/773f7fa0-023f-4741-b541-eac6edc8c03b" />

To solve for the time window size we take a feature and use this formula to estimate how long it takes to move k pixels over the magnitude of the optical flow, u.

### Events generated during time window t ∈ [Ti, Ti + dti]

This is the new events captured by the event camera during the time window t ∈ [Ti, Ti + dti]. Events include x - position, y - position, t - time, and p - polarity

<img width="401" height="95" alt="image" src="https://github.com/user-attachments/assets/b7ed50b2-6a72-48db-a2ea-86cb9256fad9" />




#### The sequence of the algorithm is as follows:

1. eFAST initial feature generation
2. Initial optical flow estimate and landmark generation
3. Time window calculation
4. Probability fitting previous landmark with current optical flow and feature
5. New optical flow estimate of window
6. Use new optical flow for step 2 and continue from there until threshold is reached

### **Step one: eFAST Initial Feature Generation**
Think of this as a black box: If a feature is discovered with eFAST, we use this as our initial estimate of the feature before detection, we take the coordinates that return from this and use it for tracking

### **Step two: Initial Optical Flow estimate and Landmark Generation**
When i=0, we do not have an optical flow estimate so we use u0 = 0. We also use this for l00 for the landmark.

If i does not equal zero, we use the optical flow calculated from step 5 from window i-1, and we generate the landmark with the optical flow from step 2 of i-1, or step 5 of i-2. The optical flow ui is always stated to be the optical flow generated from step 5 of i-1.

### **Step 3: Time Window Calculation:**
When i=0, we define the window to a sufficient amount of events such that we can do a "good" initial estimate in step 5. say, 50k events

Otherwise, we use the median formula here to calculate the window size

### **Find the Probability that Previous Landmark correlates with current feature**
Now we need to find the probability that the previous landmark correlates with the current flow/feature fi and ui.

<img width="958" height="126" alt="image" src="https://github.com/user-attachments/assets/0670dfc1-6f85-413b-b8e0-b4f53bf5f180" />

### Updating Optical Flow
Once we have everything we need: landmarks, current flow, probability and time window, we can update the optical flow using this equation

