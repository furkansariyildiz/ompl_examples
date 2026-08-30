## 1. Build Configuration

- [x] 1.1 Add OMPL as a package dependency.
- [x] 1.2 Update CMake configuration to find OMPL and build the `plan_2d` executable.

## 2. Core Implementation

- [x] 2.1 Add a C++ source file for the minimal 2D planning example.
- [x] 2.2 Implement bounded 2D state space setup, hardcoded circular obstacles, state validity checking, planner execution, and console path output.
- [x] 2.3 Extract reusable 2D helper declarations and definitions into header/source files.

## 3. Verification

- [x] 3.1 Build the package with catkin.
- [x] 3.2 Run the executable when the local OMPL dependency is available, or document the missing dependency if it is not available.
