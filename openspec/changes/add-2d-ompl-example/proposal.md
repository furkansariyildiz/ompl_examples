## Why

This package needs a minimal OMPL example that makes the core planning flow easy to understand before adding heavier ROS integrations. A small 2D example provides a clear starting point for learning state spaces, validity checking, planner setup, and path output.

## What Changes

- Add a C++ executable that solves a hardcoded 2D path planning problem with OMPL.
- Model the planning state as a 2D real vector with bounded `x` and `y` coordinates.
- Add simple hardcoded obstacles and a validity checker that rejects states inside those obstacles.
- Run an OMPL planner from a fixed start state to a fixed goal state.
- Print the resulting path to the console when planning succeeds.

## Capabilities

### New Capabilities
- `minimal-2d-planning-example`: Covers the behavior of a small executable that demonstrates the OMPL 2D planning workflow.

### Modified Capabilities

None.

## Impact

- Adds a new C++ source file under `src/`.
- Updates the catkin build configuration to compile and link against OMPL.
- Updates package dependencies to declare the OMPL dependency.
