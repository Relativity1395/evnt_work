# EVIO §5.1 — Optical Flow (EM1)

## The Problem

Before anything, we have raw event streams firing on the screen. Assuming the lens is calibrated correctly, you should see a clear image of the environment — or nothing, if nothing is moving and no light source is changing. But you'll notice you can still see yourself: minute vibrations and breathing are enough for the camera to pick up. We need an algorithm that tracks the movement of each pixel.

## Definitions

Before the optical flow algorithm, some prerequisite terms:

**Pixel position (x)** — a vector `<x, y>` on the image plane marking where an event fired.

**Timestamp (t)** — the time an event fired.

**Optical flow (u)** — the velocity of a feature on the image plane in x and y. It is recovered from the local gradient of the time surface, not the gradient itself.

**Feature (f)** — an intersection of event edges (a corner), where the surrounding pixel information is compressed into a single important point.

**Landmark (L)** — an abstract 3D point representing a physical object seen by the camera (exists in ℝ³).

**Landmark projection (l)** — the 2D image of a landmark L.

## Optical Flow and Events

The velocity of a pixel (or a collection of pixels) in x and y is known as optical flow. We use it to track objects moving in the frame, the ego-motion of the camera when objects are still, or both. This is done using the properties an event gives us.

An event contains:

1. **Position** — `(x, y)` coordinates.
2. **Timestamp** — when the event fired.
3. **Polarity** — whether brightness increased or decreased.

With these properties we can extract an event's motion. But first, some organization: instead of tracking the velocity of every event, we track a group of them, organized into what's known as a *corner* or *feature*.

## Features, Corners, and Edges

We don't want to track every event — not at tens of millions per second. Instead we group them into features (corners). A corner is an intersection of edges, and an edge is a stream of events firing in close proximity in space and time. Picture a line of pixels firing in a straight line within a small window: that's an edge.

**Edge** — consecutive events firing in close proximity in space and time, sharing the same polarity.

**Corner** — an intersection of edges.

A corner can also be called a feature. We use the corners from the event frame to form the first features we track for optical flow.

We use corners instead of edges because an edge only gives us velocity perpendicular to itself. A corner gives us two dimensions of velocity, solving what's known as the *aperture problem* and providing more information at a given time for marginally more processing.

## Detecting Corners (SAE)

To compute optical flow, we first generate corners from the event frame using a **Surface of Active Events (SAE)** — a record of *when* each pixel last fired.

The SAE is a matrix holding a timestamp at each `(column x, row y)` position. There is one SAE per polarity.

To detect a corner, we look at the events around the most recently fired event along two concentric circles — 3 and 4 events out from the center. If those events form an arc that fired close together in space and time, a corner exists and the center event is marked.

That marked event is a basic feature — the starting point for optical flow.

Up to this point, corner detection is handled for you by a library called **eFAST**. It's already in the repo — nothing more needs to be done to use it. From here on, we implement the algorithm from the paper: generating optical flow.

## Generating Optical Flow

We track the velocity of these features by fitting an optical flow to their motion until an optimal flow is reached.

If a feature were tracked continuously, it would trace a smooth path — at every position and time there is a distinct velocity along it. But here we don't have continuous data: we have discrete positions and timestamps, and even then we don't want to compute a new flow at every timestamp.

So two questions: in what increments do we advance the feature's position, and how do we update that position correctly?

This is the hardest concept to grasp, because it's a chicken-and-egg problem. We can't update the feature without knowing its velocity — but we can't find the velocity until we know where the feature moved and how fast.

## Expectation-Maximization (EM)

Expectation-Maximization (EM) is the algorithm we use to generate optical flow, and it resolves the chicken-and-egg problem. At its core, this is an optimization.

**Problem:** track a feature across discrete optical-flow steps, over variable-length windows, until an optimized flow is reached.

Once we have that flow, a separate algorithm (not covered here) corrects drift against the feature's initial frame.

To calculate flow, we use:

1. The time window that tracks the feature — [T<sub>i</sub>, T<sub>i+1</sub>]
2. The landmark projection on the frame — *l*
3. The flow from the previous window — u<sub>i−1</sub>

`i` is the current window number, counting from the initial corner detection.

- There are n<sub>i</sub> events in window *i*.
- *j* indexes the references from the previous window — there are n<sub>i−1</sub> of them (the events of window *i−1*).

**Why these values?**

Because events are asynchronous, we have no fixed notion of how long to track a feature. So we size each window to hold just enough events that the optical flow stays roughly *constant* across it — which lets us solve for a single flow value per window.

**The window.** We open a window [T<sub>i</sub>, T<sub>i+1</sub>] to track the current feature so we can compute a roughly correct position for it in the next window:

> ### **f(t) = f(T<sub>i</sub>) + (t − T<sub>i</sub>) · u**

where *u* is the average flow over the window.

**The landmark.** We use the landmark projection as the reference the events are matched against — where we expect the feature to line up. The landmark is *l<sub>j</sub>*, but we have no way to know the *current* landmark directly; we only have past values. So we approximate it as the previous window's events, propagated forward to now:

> ### **l̃<sub>j</sub><sup>i−1</sup> = { x + (T<sub>i</sub> − t) · u<sub>i−1</sub>  |  (x, t) ∈ W<sub>i−1</sub> }**

Because the flow iterates toward the true flow at every step, as *i* grows l̃<sub>j</sub><sup>i−1</sup> approaches the true *l<sub>j</sub>*.

By window `i = 2` we have the reference, the window, and the previous flow — all produced back in `i = 1`. Now what?

## Solving for Flow (the Optimization)

Now that we have these values, what do we do with them? Because this is an optimization, we look for the flow *u* that minimizes the mismatch between the events and their landmark references:

> ### **min<sub>u</sub>  Σ<sub>k</sub> Σ<sub>j</sub>  r<sub>kj</sub> · ‖ (x<sub>k</sub> − t̄<sub>k</sub> · u) − l<sub>j</sub> ‖²**

where t̄<sub>k</sub> = t<sub>k</sub> − T<sub>i</sub> is how long after the window start event *k* fired.

Breaking down the inner term:

- x<sub>k</sub> − t̄<sub>k</sub> · u — event *k* back-propagated to the window start T<sub>i</sub> using the flow *u*. (This is an *event*, not the feature.)
- l<sub>j</sub> — the landmark projection (the reference).
- (x<sub>k</sub> − t̄<sub>k</sub> · u) − l<sub>j</sub> — how far the back-propagated event lands from its landmark reference. If *u* is correct, this shrinks toward zero.

We multiply each squared residual by an association weight r<sub>kj</sub> (a pdf weight, defined later), sum over all event–landmark pairs, and find the *u* that makes that total smallest.

That total is the **cost**. We compare the cost — not the flow — to a preset threshold. Once the cost drops below it, the flow is good enough and we stop iterating.

## The E-step and M-step

Since we can't know the true *l<sub>j</sub>*, we substitute the approximation l̃<sub>j</sub><sup>i−1</sup> in the cost above. Then we iterate two steps.

**E-step — compute the weights.** Given the current flow, we calculate each association weight r<sub>kj</sub> as a normalized Gaussian:

> ### **r<sub>kj</sub> = φ( x<sub>k</sub> − t̄<sub>k</sub>u ; l̃<sub>j</sub><sup>i−1</sup>, Σ ) / Σ<sub>j′</sub> φ( x<sub>k</sub> − t̄<sub>k</sub>u ; l̃<sub>j′</sub><sup>i−1</sup>, Σ )**

φ(v; μ, Σ) is a Gaussian evaluated at *v* with mean μ and covariance Σ (here Σ = 2*I*). The denominator sums over every reference *j′*, so one event's weights sum to 1.

**M-step — update the flow.** With the weights fixed, we solve for the flow that minimizes the cost. It has a closed form:

> ### **u = ( Σ<sub>k</sub> Σ<sub>j</sub> r<sub>kj</sub> (x<sub>k</sub> − l̃<sub>j</sub><sup>i−1</sup>) t̄<sub>k</sub> ) / ( Σ<sub>k</sub> Σ<sub>j</sub> r<sub>kj</sub> t̄<sub>k</sub><sup>2</sup> )**

We alternate E-step and M-step until the cost drops below the threshold. Then we compute the three values — window, reference, and flow — again for *i* = 3, and repeat.

## Initial Conditions

Now that the algorithm is laid out, we need to handle what happens when the camera turns on.

### Time Window Initialization

At t = 0 we have no flow estimate, so we can't size the window by motion. Instead we use an arbitrary window filled by event count — the paper uses 50,000 events.

### Optical Flow Initialization

We seed the flow at zero: u<sub>0</sub> = 0.

How do we build the landmarks and r<sub>kj</sub> with no previous window? There is no window i−1 to propagate from, so we use the current window's own events as the reference, back-propagated by the seed flow. We compute r<sub>kj</sub> from that zero-flow seed, then solve this initialization M-step (here the references and the events are the same set):

u = ( Σ<sub>k</sub> Σ<sub>j</sub> r<sub>kj</sub> (x<sub>k</sub> − x<sub>j</sub>)(t̄<sub>k</sub> − t̄<sub>j</sub>) ) / ( Σ<sub>k</sub> Σ<sub>j</sub> r<sub>kj</sub> (t̄<sub>k</sub> − t̄<sub>j</sub>)<sup>2</sup> )

where both k and j index the current window's events, and r<sub>kj</sub> is computed as in the E-step.

You might expect this to take many iterations to climb from zero to a good estimate — it does. Because the reference set is the current events (which move as u changes), it must be rebuilt every iteration, which is slow and bad for k-d trees. But this runs only once per feature, at birth, so it is marginalized out over a long track.
