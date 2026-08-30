#include "ompl_examples/planning_2d.h"

#include <ompl/base/PlannerStatus.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main(int argc, char **argv)
{
  const std::string output_csv_path = argc > 1 ? argv[1] : "";

  auto space = std::make_shared<ob::RealVectorStateSpace>(2);

  // Set the bounds of the space to be in [0, 10] in both dimensions
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0.0);
  bounds.setHigh(10.0);
  space->setBounds(bounds);

  // Setting up the SimpleSetup object
  og::SimpleSetup simple_setup(space);

  // Define the obstacles
  // X, Y, Radius
  const std::vector<ompl_examples::CircleObstacle> obstacles = {
      {5.0, 5.0, 1.25},
      {3.0, 6.5, 1.0},
      {7.0, 3.0, 1.0},
  };

  // Set the state validity checker
  // The lambda function captures the obstacles vector and checks if a state is valid
  // A state is valid if it is not inside any of the defined obstacles 
  simple_setup.setStateValidityChecker(
      [&obstacles](const ob::State *state) { return ompl_examples::isStateValid(state, obstacles); });
  simple_setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

  // Define the start state
  ob::ScopedState<> start(space);
  start[0] = 1.0;
  start[1] = 1.0;

  // Define the goal state
  ob::ScopedState<> goal(space);
  goal[0] = 9.0;
  goal[1] = 9.0;

  // Set the start and goal states in the SimpleSetup object
  // Set the planner to RRTConnect
  simple_setup.setStartAndGoalStates(start, goal);
  simple_setup.setPlanner(std::make_shared<og::RRTConnect>(simple_setup.getSpaceInformation()));

  std::cout << "Planning from (" << start[0] << ", " << start[1] << ") to (" << goal[0] << ", "
            << goal[1] << ")...\n";

  // Attempt to solve the planning problem within a time limit of 1 second
  const ob::PlannerStatus solved = simple_setup.solve(1.0);

  if (!solved)
  {
    std::cout << "No path found within the solve timeout.\n";
    return 1;
  }

  // Simplify the solution path to make it more efficient and easier to follow
  simple_setup.simplifySolution();

  // Retrieve the solution path and interpolate it to have 40 states for smoother visualization
  og::PathGeometric path = simple_setup.getSolutionPath();
  path.interpolate(40);
  ompl_examples::printPath(path);

  if (!output_csv_path.empty())
  {
    if (!ompl_examples::writePathCsv(path, output_csv_path))
    {
      std::cout << "Failed to write path CSV: " << output_csv_path << "\n";
      return 1;
    }

    std::cout << "Wrote path CSV: " << output_csv_path << "\n";
  }

  return 0;
}
