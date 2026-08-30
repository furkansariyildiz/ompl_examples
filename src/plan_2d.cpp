#include "ompl_examples/planning_2d.h"

#include <ompl/base/PlannerStatus.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <iostream>
#include <memory>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main()
{
  auto space = std::make_shared<ob::RealVectorStateSpace>(2);

  ob::RealVectorBounds bounds(2);
  bounds.setLow(0.0);
  bounds.setHigh(10.0);
  space->setBounds(bounds);

  og::SimpleSetup simple_setup(space);

  const std::vector<ompl_examples::CircleObstacle> obstacles = {
      {5.0, 5.0, 1.25},
      {3.0, 6.5, 1.0},
      {7.0, 3.0, 1.0},
  };

  simple_setup.setStateValidityChecker(
      [&obstacles](const ob::State *state) { return ompl_examples::isStateValid(state, obstacles); });
  simple_setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

  ob::ScopedState<> start(space);
  start[0] = 1.0;
  start[1] = 1.0;

  ob::ScopedState<> goal(space);
  goal[0] = 9.0;
  goal[1] = 9.0;

  simple_setup.setStartAndGoalStates(start, goal);
  simple_setup.setPlanner(std::make_shared<og::RRTConnect>(simple_setup.getSpaceInformation()));

  std::cout << "Planning from (" << start[0] << ", " << start[1] << ") to (" << goal[0] << ", "
            << goal[1] << ")...\n";

  const ob::PlannerStatus solved = simple_setup.solve(1.0);

  if (!solved)
  {
    std::cout << "No path found within the solve timeout.\n";
    return 1;
  }

  simple_setup.simplifySolution();

  og::PathGeometric path = simple_setup.getSolutionPath();
  path.interpolate(40);
  ompl_examples::printPath(path);

  return 0;
}
