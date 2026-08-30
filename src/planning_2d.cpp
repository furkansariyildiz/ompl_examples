#include "ompl_examples/planning_2d.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <iomanip>
#include <iostream>

namespace ompl_examples
{

bool CircleObstacle::contains(double x, double y) const
{
  const double dx = x - center_x;
  const double dy = y - center_y;
  return (dx * dx + dy * dy) <= (radius * radius);
}

bool isStateValid(const ompl::base::State *state, const std::vector<CircleObstacle> &obstacles)
{
  const auto *real_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
  const double x = real_state->values[0];
  const double y = real_state->values[1];

  for (const auto &obstacle : obstacles)
  {
    if (obstacle.contains(x, y))
    {
      return false;
    }
  }

  return true;
}

void printPath(const ompl::geometric::PathGeometric &path)
{
  std::cout << "Solved path with " << path.getStateCount() << " states:\n";
  std::cout << std::fixed << std::setprecision(3);
  path.printAsMatrix(std::cout);
}

}  // namespace ompl_examples
