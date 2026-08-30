#ifndef OMPL_EXAMPLES_PLANNING_2D_H
#define OMPL_EXAMPLES_PLANNING_2D_H

#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>

#include <string>
#include <vector>

namespace ompl_examples
{

struct CircleObstacle
{
  double center_x;
  double center_y;
  double radius;

  bool contains(double x, double y) const;
};

struct Point2D
{
  double x;
  double y;
};

struct PlanningWorld
{
  double bound_low;
  double bound_high;
  Point2D start;
  Point2D goal;
  std::vector<CircleObstacle> obstacles;
};

PlanningWorld makeDefaultWorld();

bool loadWorldConfig(const std::string &file_path, PlanningWorld &world, std::string &error_message);

bool isStateValid(const ompl::base::State *state, const std::vector<CircleObstacle> &obstacles);

bool isPathValid(const ompl::geometric::PathGeometric &path, const std::vector<CircleObstacle> &obstacles);

void printPath(const ompl::geometric::PathGeometric &path);

bool writePathCsv(const ompl::geometric::PathGeometric &path, const std::string &file_path);

}  // namespace ompl_examples

#endif  // OMPL_EXAMPLES_PLANNING_2D_H
