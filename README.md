# Algorithmic Robot Planning 236901

This repo contains implementations for 3 homework assignments and a final project completed for an algorithmic robot planning course given at the CS faculty Technion. Each folder contains a supplementary PDF report that provides extra documentation and context for the code implementations. These reports include relevant details like theoretical proofs, experimental analyses, problem formulations, and algorithm suggestions.

## Homework 1

The AMP-HW1 folder contains code implementing concepts including:

- Minkowski Sums
  - We implemented Minkowski Sums and proved properties like distributivity over set union.

- Exact Motion Planning
  - Implemented an exact algorithm to plan motion for a diamond-shaped robot among polygonal obstacles.
  - Uses Minkowski sums to transform the robot and obstacles into a configuration space representation.
  - Constructs a visibility graph of the free C-space and searches it to find an optimal collision-free path.

## Homework 2

The AMP-HW2 folder contains analyses of:

- High-Dimensional Point Distributions
  - Studied the distribution of random points in high-dimensional balls.
  - As dimensionality increases, most of the measure concentrates close to the boundary.
  - This suggests collision checking can be made more efficient by reducing the connection radius.

- Motion Planning for Tethered Robots
  - Planning paths subject to tether length constraints using homotopy classes.
  - Encoding tether configurations compactly as h-signatures. 
  - Constructing a graph augmented with h-invariants and using Dijkstra's to find shortest feasible paths.

## Homework 3

The AMP-HW3 folder implements:

- Forward Kinematics and Validation for 3-Link Planar Robot
  - Calculates end effector positions from joint configurations using trigonometry.
  - Checks for self-collisions using polygon intersection tests.

- Sampling-Based Motion Planners
  - Implemented RRT and RRT* algorithms.
  - Explored effects of tuning parameters like goal bias and step size.
  - RRT* significantly outperformed RRT in solution cost while only doubling computation time.

- RRT for Inspection Planning
  - Adapted RRT to maximize coverage of inspecting a set of points.
  - Implemented a k-RRTI variant making connections based on coverage gain. 


## Final Project

The final project involved task inspection planning - given one robot performing a known trajectory, our goal was to plan motion for a second robot to maximize sensing coverage of the first robot's end effector over time. This project required adapting the concepts from previous assignments to a problem without a predefined goal state. We developed incremental sampling-based approaches biased toward improving inspection coverage.

- Implemented a naive baseline sampling-based planner
  - Initializes inspection plan and repeatedly samples random configurations
  - Only adds samples that improve coverage without replacing vertices
  - Very fast and achieved high coverage despite simplicity

- Attempted to improve the baseline planner with:
  - Multiple sampling modes (don't replace, always replace, probabilistically replace vertices) 
  - Biased sampling toward promising areas
  - More complex planning did not improve performance

- Adapted RRT to use coverage gain as heuristic
  - Connects new vertices based on coverage improvement
  - Produced higher quality solutions but slower runtimes

- Implemented RRT variant handling the time dimension
  - Connects vertices based on timestamp and configuration
  - Used time as a factor when connecting vertices

- Simple sampling method was remarkably effective
  - Flexibility to "jump" between configurations mitigated need for optimal search
  - RRT approaches improved quality but slower runtime

In summary, a basic sampling planner performed very well on this task inspection problem. The ability to freely move between configurations reduced the benefits of more complex optimal planning.

## Authors  

- Andrew Elashkin
- Yonatan Sommer
