# OMPL Examples

This package contains small OMPL examples built with catkin.

## 2D Planning Example

The `plan_2d` executable demonstrates a minimal 2D path planning problem:

- A bounded `RealVectorStateSpace` with `x` and `y` coordinates in `[0, 10]`
- Hardcoded circular obstacles
- A state validity checker that rejects states inside obstacles
- An `RRTConnect` planner
- Console output for the solved path

Reusable obstacle and path-printing helpers live in `include/ompl_examples/planning_2d.h` and `src/planning_2d.cpp`.

## Dependencies

Install OMPL for ROS Noetic before building:

```bash
sudo apt-get update
sudo apt-get install ros-noetic-ompl
```

## Build

From the catkin workspace root:

```bash
catkin build ompl_examples
```

## Run

After sourcing the workspace:

```bash
source devel/setup.bash
rosrun ompl_examples plan_2d
```
