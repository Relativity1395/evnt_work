**Feature tracking algorithm breakdown**

**Overview of the Algorithm and Block Diagram**

The sequence of the algorithm is as follows:

1. eFAST initial feature generation
2. Initial optical flow estimate and landmark generation
3. Time window calculation
4. Probability fitting previous landmark with current optical flow and feature
5. New optical flow estimate of window
6. Use new optical flow for step 2 and continue from there until threshold is reached

Step one: eFAST Initial Feature Generation
Think of this as a black box: If a feature is discovered with eFAST, we use this as our initial estimate of the feature before detection, we take the coordinates that return from this and use it for tracking

Step two: Initial Optical Flow estimate and Landmark Generation
When i=0, we do not have an optical flow estimate so we use u0 = 0. We also use this for l00 for the landmark.

If i does not equal zero, we use the optical flow calculated from step 5 from window i-1, and we generate the landmark with the optical flow from step 2 of i-1, or step 5 of i-2. The optical flow ui is always stated to be the optical flow generated from step 5 of i-1.

Step 3: Time Window Calculation:
When i=0, we define the window to a sufficient amount of events such that we can do a "good" initial estimate in step 5. say, 50k events

Otherwise, we use the median formula here to calculate the window size

Find the Probability that Previous Landmark correlates with current feature
Now we need to find the probability that the previous landmark correlates with the current flow/feature fi and ui.

alt text

Updating Optical Flow
Once we have everything we need: landmarks, current flow, probability and time window, we can update the optical flow using this equation
