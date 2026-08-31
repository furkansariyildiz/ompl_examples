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

/**
 * @brief Creates a default planning world with predefined bounds, start and goal points, and obstacles.
 * @return A PlanningWorld object representing the default world.
 */
PlanningWorld makeDefaultWorld();

/**
 * @brief Loads a planning world configuration from a YAML file.
 * @param file_path The path to the YAML configuration file.
 * @param world A reference to a PlanningWorld object where the loaded configuration will be stored.
 * @param error_message A reference to a string where any error messages will be stored if loading fails.
 * @return True if the configuration was loaded successfully, false otherwise.
 */
bool loadWorldConfig(const std::string &file_path, PlanningWorld &world, std::string &error_message);

/**
 * @brief Checks if a given state is valid (i.e., not inside any obstacles).
 * @param state A pointer to the state to be checked.
 * @param obstacles A vector of CircleObstacle objects representing the obstacles in the environment.
 * @return True if the state is valid, false otherwise.
 */
bool isStateValid(const ompl::base::State *state, const std::vector<CircleObstacle> &obstacles);

/**
 * @brief Checks if a given path is valid (i.e., does not intersect any obstacles).
 * @param path A reference to the PathGeometric object representing the path to be checked.
 * @param obstacles A vector of CircleObstacle objects representing the obstacles in the environment.
 * @return True if the path is valid, false otherwise.
 */
bool isPathValid(const ompl::geometric::PathGeometric &path, const std::vector<CircleObstacle> &obstacles);

/**
 * @brief Prints the states of a given path to the standard output.
 * @param path A reference to the PathGeometric object representing the path to be printed.
 */
void printPath(const ompl::geometric::PathGeometric &path);

/**
 * @brief Writes the states of a given path to a CSV file.
 * @param path A reference to the PathGeometric object representing the path to be written.
 * @param file_path The path to the output CSV file.
 * @return True if the path was written successfully, false otherwise.
 */
bool writePathCsv(const ompl::geometric::PathGeometric &path, const std::string &file_path);

}  // namespace ompl_examples

#endif  // OMPL_EXAMPLES_PLANNING_2D_H
