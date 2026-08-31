#include "ompl_examples/planning_2d.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

namespace ompl_examples
{

namespace
{

Point2D getPointFromState(const ompl::base::State *state)
{
  const auto *real_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
  return {real_state->values[0], real_state->values[1]};
}

double squaredDistanceToSegment(const Point2D &point, const Point2D &segment_start, const Point2D &segment_end)
{
  const double segment_x = segment_end.x - segment_start.x;
  const double segment_y = segment_end.y - segment_start.y;
  const double segment_length_squared = segment_x * segment_x + segment_y * segment_y;

  if (segment_length_squared <= std::numeric_limits<double>::epsilon())
  {
    const double dx = point.x - segment_start.x;
    const double dy = point.y - segment_start.y;
    return dx * dx + dy * dy;
  }

  const double projected =
      ((point.x - segment_start.x) * segment_x + (point.y - segment_start.y) * segment_y) /
      segment_length_squared;
  const double clamped = std::max(0.0, std::min(1.0, projected));
  const Point2D closest = {segment_start.x + clamped * segment_x, segment_start.y + clamped * segment_y};

  const double dx = point.x - closest.x;
  const double dy = point.y - closest.y;
  return dx * dx + dy * dy;
}

}  // namespace

bool CircleObstacle::contains(double x, double y, double padding) const
{
  // Check if the point (x, y) is inside the circle defined by center (center_x, center_y) and radius
  const double dx = x - center_x;
  const double dy = y - center_y;
  const double collision_radius = radius + padding;
  return (dx * dx + dy * dy) <= (collision_radius * collision_radius);
}

PlanningWorld makeDefaultWorld()
{
  PlanningWorld world;
  world.bound_low = 0.0;
  world.bound_high = 10.0;
  world.start = {1.0, 1.0};
  world.goal = {9.0, 9.0};
  world.robot = {0.0, 0.0};
  world.obstacles = {
      {5.0, 5.0, 1.25},
      {3.0, 6.5, 1.0},
      {7.0, 3.0, 1.0},
  };
  return world;
}

double getCollisionPadding(const RobotModel &robot)
{
  return robot.radius + robot.safety_margin;
}

double getCollisionRadius(const CircleObstacle &obstacle, const RobotModel &robot)
{
  return obstacle.radius + getCollisionPadding(robot);
}

bool loadWorldConfig(const std::string &file_path, PlanningWorld &world, std::string &error_message)
{
  try
  {
    const YAML::Node root = YAML::LoadFile(file_path);

    world.bound_low = root["bounds"]["low"].as<double>();
    world.bound_high = root["bounds"]["high"].as<double>();
    world.start = {root["start"]["x"].as<double>(), root["start"]["y"].as<double>()};
    world.goal = {root["goal"]["x"].as<double>(), root["goal"]["y"].as<double>()};

    const YAML::Node robot = root["robot"];
    if (robot)
    {
      if (robot["radius"])
      {
        world.robot.radius = robot["radius"].as<double>();
      }
      if (robot["safety_margin"])
      {
        world.robot.safety_margin = robot["safety_margin"].as<double>();
      }
    }

    if (world.robot.radius < 0.0)
    {
      error_message = "The robot radius must be non-negative.";
      return false;
    }
    if (world.robot.safety_margin < 0.0)
    {
      error_message = "The robot safety margin must be non-negative.";
      return false;
    }

    world.obstacles.clear();
    const YAML::Node obstacles = root["obstacles"];
    if (!obstacles || !obstacles.IsSequence())
    {
      error_message = "The 'obstacles' field must be a YAML sequence.";
      return false;
    }

    for (const auto &obstacle : obstacles)
    {
      const double radius = obstacle["radius"].as<double>();
      if (radius < 0.0)
      {
        error_message = "Obstacle radii must be non-negative.";
        return false;
      }

      world.obstacles.push_back({
          obstacle["center"]["x"].as<double>(),
          obstacle["center"]["y"].as<double>(),
          radius,
      });
    }
  }
  catch (const std::exception &error)
  {
    error_message = error.what();
    return false;
  }

  return true;
}

bool isStateValid(const ompl::base::State *state, const std::vector<CircleObstacle> &obstacles,
                  const RobotModel &robot)
{
  // Cast the state to a RealVectorStateSpace::StateType to access its coordinates
  const Point2D point = getPointFromState(state);
  const double collision_padding = getCollisionPadding(robot);

  // Check if the point (x, y) is inside any of the defined obstacles
  for (const auto &obstacle : obstacles)
  {
    if (obstacle.contains(point.x, point.y, collision_padding))
    {
      return false;
    }
  }

  return true;
}

bool isPathValid(const ompl::geometric::PathGeometric &path, const std::vector<CircleObstacle> &obstacles,
                 const RobotModel &robot)
{
  if (path.getStateCount() == 0)
  {
    return false;
  }

  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    if (!isStateValid(path.getState(i), obstacles, robot))
    {
      return false;
    }
  }

  for (std::size_t i = 1; i < path.getStateCount(); ++i)
  {
    const Point2D segment_start = getPointFromState(path.getState(i - 1));
    const Point2D segment_end = getPointFromState(path.getState(i));

    for (const auto &obstacle : obstacles)
    {
      const Point2D obstacle_center = {obstacle.center_x, obstacle.center_y};
      const double collision_radius = getCollisionRadius(obstacle, robot);
      const double radius_squared = collision_radius * collision_radius;

      if (squaredDistanceToSegment(obstacle_center, segment_start, segment_end) <= radius_squared)
      {
        return false;
      }
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

bool writePathCsv(const ompl::geometric::PathGeometric &path, const std::string &file_path)
{
  std::ofstream output(file_path);
  if (!output)
  {
    return false;
  }

  output << "x,y\n";
  output << std::fixed << std::setprecision(6);

  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
    output << state->values[0] << "," << state->values[1] << "\n";
  }

  return true;
}

}  // namespace ompl_examples
