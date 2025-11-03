# Path Planner and Path Validator Implementation


## 1 Introduction

This project builds a practical motion planning pipeline in ROS 2 with two main parts: a Path
Planner that proposes a route, and a Path Validator that checks if that route is actually safe
to follow. The planner uses the A* algorithm on a grid map to generate collision-free paths. The
validator then inspects that path against safety margins, start/goal requirements, and obstacle
geometry. If the validator rejects a path, the system can automatically try to improve it and
replan.

## 2 Path Planner: What We Implemented

We developed a ROS 2 node (PathPlannerPublisher) that creates paths through a known 2D
environment.

### Environment and Grid

Environment data (start, goal, obstacles, bounds, robot radius) are read from a YAML file.
Obstacles are axis-aligned rectangles. We discretize the world into an occupancy grid at a
user-defined resolution.

### A* Search (4/8-connected)

We run A* on that grid to find a short, feasible route. The planner supports both 4-connected
and 8-connected neighbors. A Euclidean distance heuristic keeps the search efficient.

### Inflation and Replanning

To keep a safe distance from obstacles, we inflate them by a chosen radius (robot radius plus an
extra clearance, unless a custom value is set). If the validator later marks the path as unsafe,
the planner can automatically increase the inflation radius slightly and replan. This creates
wider margins around obstacles when needed.

### Post-processing (Smoothing & Interpolation)

Raw grid paths can be jagged. We therefore attempt straight-line shortcuts where possible and
then resample the path at a fixed spacing. The result is smoother and easier for a controller to
track.

### ROS Interfaces and Visualization

The planner publishes the path on navmsgs/Path and draws obstacles in RViz using marker
arrays. This makes debugging and tuning more transparent.


## 3 Path Validator: What We Implemented

We also built a ROS 2 node (PathValidatorPublisher) that checks if a proposed path is safe
and sensible.

### Core Idea: Check on an Inflated Grid

Like the planner, the validator builds an occupancy grid from the same YAML. It then inflates
obstacles by a radius that reflects the robot size and a small margin (optionally reduced by a
“deflate” parameter for auditing). Validating on the inflated grid makes the check conservative
and practical for real robots.

### What the Validator Checks

- Basic sanity: The path must have at least two poses and use the expected frameid.
- Start/goal adherence: The first and last poses must lie within a tolerance of the
    configured start and goal. We add a small “slack” so minor discretization errors do not
    cause false rejections.
- Continuity: The distance between consecutive poses must not exceed a maximum step
    size, so there are no gaps that a controller cannot bridge.
- Collision along segments: Every line segment between consecutive poses must stay in
    free space on the inflated grid.

### Algorithms for the Validator

For collision checking along each segment, the validator uses a grid ray-casting method based
on the Bresenham line traversal. In simple terms, for each pair of consecutive waypoints, it
walks through the exact set of grid cells that the straight line between them passes. If any of
those cells are occupied on the inflated map, that segment is flagged as colliding. This approach
is fast, deterministic, and well-suited to discrete occupancy grids.

### Outputs and Debugging

The validator publishes a Boolean message pathvalid. It also publishes markers that show the
path and, if there is a problem, a red line to highlight the first collision point and the offending
segment. Old debug markers are cleared when the path becomes valid again.

## 4 Problems We Faced and How We Addressed Them

During development and testing, we met several common but important issues.

1. Frame ID mismatches Sometimes the path and validator used different frameid values,
which caused immediate failure. We added an explicit check and a clear error message. The fix
in practice is simple: make sure both nodes use the same frame (e.g., map).
2. Choosing the right inflation If inflation is too small, the robot might pass too close to
obstacles; if it is too large, A* may not find any path. We exposed inflation as a parameter and
added the auto-replan feature. On invalidation, the planner increases inflation in small steps
and tries again.


3. Start/goal tolerances Strict tolerances at start and goal sometimes rejected otherwise
safe paths, especially with grid rounding. We introduced a small slack that recognizes these
tiny differences while still enforcing the intent.
4. Discontinuities after smoothing/interpolation Post-processing can create a step
larger than the validator’s allowed segment length. We handled this by checking the max
step and reporting where it fails, so spacing can be tuned.
5. Replanning loops without improvement In some worlds, simply increasing inflation
still does not produce a valid path. We added limits to how far inflation can grow and made
the warnings explicit. A smarter future approach would adjust multiple parameters or switch
strategies (e.g., try a different cost map or a different planner).
6. Stale debug markers Old collision markers sometimes remained visible. We now clear
them when a path is valid to keep the RViz view clean and trustworthy.

## 5 Results on RViz

The output of the path planning is given below:
<img width="886" height="730" alt="1" src="https://github.com/user-attachments/assets/6131d92f-aff8-4fed-96eb-2020bcb381aa" />



## 6 Conclusion

Overall, the system works as intended: the A* planner quickly proposes a path on a grid map,
and the validator uses Bresenham line traversal on an inflated occupancy grid to confirm that
every segment is safe and that the path respects start/goal rules and continuity. The automatic
replanning helps recover from invalid paths by increasing obstacle margins.


