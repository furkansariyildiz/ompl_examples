## Context

The package is currently a minimal ROS/catkin package with no OMPL example code. The first example should teach the OMPL planning pipeline without introducing ROS message passing, map handling, visualization, or runtime parameter loading.

## Goals / Non-Goals

**Goals:**
- Provide a small C++ executable that can be built by catkin.
- Demonstrate the basic OMPL flow: state space, bounds, validity checker, planner setup, solving, path simplification, and path printing.
- Keep all world data hardcoded so the example is easy to read and modify.

**Non-Goals:**
- Do not publish ROS topics or RViz markers in this initial example.
- Do not load maps, obstacles, start states, or goal states from files or ROS parameters.
- Do not compare multiple planners yet.

## Decisions

- Use `ompl::base::RealVectorStateSpace` with dimension 2 for the state model.
  - Rationale: the example only needs Cartesian `x` and `y` coordinates, and this is the most direct way to express a 2D continuous planning problem in OMPL.
  - Alternative considered: `SE2StateSpace`, but orientation would add a concept that is not needed for the first example.

- Use hardcoded circular obstacles in the validity checker.
  - Rationale: circles keep the collision logic short and readable while still showing how a planner queries state validity.
  - Alternative considered: rectangular obstacles, but they introduce more boundary cases for no additional learning value in the first pass.

- Keep reusable 2D helper declarations in `include/ompl_examples/planning_2d.h` and their implementation in `src/planning_2d.cpp`.
  - Rationale: separating the obstacle model, validity checker, and path printing from `main()` keeps the example readable and gives later examples a natural place to reuse small helpers.
  - Alternative considered: keeping everything in `src/plan_2d.cpp`, which is acceptable for a one-file demo but becomes noisy as the package grows.

- Use `ompl::geometric::RRTConnect` as the initial planner.
  - Rationale: it is a common, fast bidirectional planner that usually finds a path quickly in simple spaces.
  - Alternative considered: `RRTstar`, but asymptotic optimality is not the focus of the introductory example.

- Print the solved path to standard output.
  - Rationale: console output is enough to verify behavior and inspect the generated path without adding visualization dependencies.
  - Alternative considered: RViz visualization, which is useful later but would make the initial example harder to understand.

## Risks / Trade-offs

- OMPL may not be installed on the local system -> declare the dependency clearly and let the build fail with an actionable CMake/package error if the dependency is missing.
- Hardcoded input limits reuse -> keep the source readable so later changes can move world data to ROS parameters or YAML.
- Planner output is nondeterministic -> tests should validate build behavior rather than require exact path coordinates.
