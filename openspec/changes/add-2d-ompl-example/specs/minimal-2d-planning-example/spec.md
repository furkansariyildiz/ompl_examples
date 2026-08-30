## ADDED Requirements

### Requirement: Buildable 2D OMPL Example
The package SHALL provide a catkin-built C++ executable named `plan_2d` that links against OMPL.

#### Scenario: Example target is available
- **WHEN** the package is configured with catkin in an environment where OMPL is installed
- **THEN** the build system exposes a `plan_2d` executable target

### Requirement: Two-Dimensional Planning Problem
The executable SHALL configure a two-dimensional bounded planning space with hardcoded start and goal states.

#### Scenario: Start and goal are configured
- **WHEN** the executable starts
- **THEN** it configures a 2D state space with finite bounds and fixed start and goal coordinates

### Requirement: Obstacle Validity Checking
The executable SHALL reject states that fall inside hardcoded obstacles and accept states that are outside all obstacles.

#### Scenario: Planner queries state validity
- **WHEN** the planner evaluates sampled states
- **THEN** states inside any obstacle are treated as invalid

### Requirement: Path Planning Output
The executable SHALL attempt to solve the planning problem and print either the solved path or a failure message.

#### Scenario: Planning succeeds
- **WHEN** OMPL finds a path from the start state to the goal state
- **THEN** the executable prints path coordinates to standard output

#### Scenario: Planning fails
- **WHEN** OMPL cannot find a path within the configured solve timeout
- **THEN** the executable prints a clear failure message to standard output
