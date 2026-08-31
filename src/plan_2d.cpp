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

bool hasYamlExtension(const std::string &file_path)
{
  return file_path.size() >= 5 &&
         (file_path.substr(file_path.size() - 5) == ".yaml" ||
          file_path.substr(file_path.size() - 4) == ".yml");
}

int main(int argc, char **argv)
{
  std::string config_path;
  std::string output_csv_path;

  if (argc > 2)
  {
    config_path = argv[1];
    output_csv_path = argv[2];
  }
  else if (argc > 1)
  {
    const std::string first_argument = argv[1];
    if (hasYamlExtension(first_argument))
    {
      config_path = first_argument;
    }
    else
    {
      output_csv_path = first_argument;
    }
  }

  ompl_examples::PlanningWorld world = ompl_examples::makeDefaultWorld();
  if (!config_path.empty())
  {
    std::string error_message;
    if (!ompl_examples::loadWorldConfig(config_path, world, error_message))
    {
      std::cout << "Failed to load world config: " << config_path << "\n";
      std::cout << error_message << "\n";
      return 1;
    }
  }

  auto space = std::make_shared<ob::RealVectorStateSpace>(2);

  // Set the bounds of the space to be in [world.bound_low, world.bound_high] in both dimensions
  ob::RealVectorBounds bounds(2);
  bounds.setLow(world.bound_low);
  bounds.setHigh(world.bound_high);
  space->setBounds(bounds);

  // Setting up the SimpleSetup object
  og::SimpleSetup simple_setup(space);

  // Define the obstacles
  // X, Y, Radius
  const std::vector<ompl_examples::CircleObstacle> obstacles = world.obstacles;
  const ompl_examples::RobotModel robot = world.robot;
  const double collision_padding = ompl_examples::getCollisionPadding(robot);

  // Set the state validity checker
  // The lambda function captures the obstacles vector and checks if a state is valid
  // A state is valid if it is not inside any inflated obstacle collision radius
  simple_setup.setStateValidityChecker(
      [&obstacles, &robot](const ob::State *state) { return ompl_examples::isStateValid(state, obstacles, robot); });
  simple_setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

  // Define the start state
  ob::ScopedState<> start(space);
  start[0] = world.start.x;
  start[1] = world.start.y;

  // Define the goal state
  ob::ScopedState<> goal(space);
  goal[0] = world.goal.x;
  goal[1] = world.goal.y;

  // Set the start and goal states in the SimpleSetup object
  // Set the planner to RRTConnect
  simple_setup.setStartAndGoalStates(start, goal);
  simple_setup.setPlanner(std::make_shared<og::RRTConnect>(simple_setup.getSpaceInformation()));

  std::cout << "Planning from (" << start[0] << ", " << start[1] << ") to (" << goal[0] << ", "
            << goal[1] << ")...\n";
  std::cout << "Robot radius: " << robot.radius << ", safety margin: " << robot.safety_margin
            << ", obstacle padding: " << collision_padding << "\n";

  // Attempt to solve the planning problem within a time limit of 1 second
  const ob::PlannerStatus solved = simple_setup.solve(1.0);

  if (!solved)
  {
    std::cout << "No path found within the solve timeout.\n";
    return 1;
  }

  og::PathGeometric path = simple_setup.getSolutionPath();
  if (!ompl_examples::isPathValid(path, obstacles, robot))
  {
    std::cout << "The planner returned a path that intersects an obstacle.\n";
    return 1;
  }

  // Simplify the solution path to make it more efficient and easier to follow
  simple_setup.simplifySolution();
  const og::PathGeometric simplified_path = simple_setup.getSolutionPath();
  if (ompl_examples::isPathValid(simplified_path, obstacles, robot))
  {
    path = simplified_path;
  }
  else
  {
    std::cout << "The simplified path intersects an obstacle; using the original solution path.\n";
  }

  // Retrieve the solution path and interpolate it to have 1000 states for smoother visualization
  path.interpolate(1000);
  if (!ompl_examples::isPathValid(path, obstacles, robot))
  {
    std::cout << "The interpolated path intersects an obstacle.\n";
    return 1;
  }

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
