#include "ompl_examples/planning_2d.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{

nav_msgs::Path toPathMessage(const og::PathGeometric &path, const std::string &frame_id)
{
  nav_msgs::Path path_message;
  path_message.header.stamp = ros::Time::now();
  path_message.header.frame_id = frame_id;
  path_message.poses.reserve(path.getStateCount());

  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    const auto *state = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();

    geometry_msgs::PoseStamped pose;
    pose.header = path_message.header;
    pose.pose.position.x = state->values[0];
    pose.pose.position.y = state->values[1];
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    path_message.poses.push_back(pose);
  }

  return path_message;
}

std::unique_ptr<og::PathGeometric> solvePath(const ompl_examples::PlanningWorld &world, double solve_timeout,
                                             int interpolate_count)
{
  auto space = std::make_shared<ob::RealVectorStateSpace>(2);

  ob::RealVectorBounds bounds(2);
  bounds.setLow(world.bound_low);
  bounds.setHigh(world.bound_high);
  space->setBounds(bounds);

  og::SimpleSetup simple_setup(space);

  const std::vector<ompl_examples::CircleObstacle> obstacles = world.obstacles;
  const ompl_examples::RobotModel robot = world.robot;

  simple_setup.setStateValidityChecker(
      [&obstacles, &robot](const ob::State *state) { return ompl_examples::isStateValid(state, obstacles, robot); });
  simple_setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

  ob::ScopedState<> start(space);
  start[0] = world.start.x;
  start[1] = world.start.y;

  ob::ScopedState<> goal(space);
  goal[0] = world.goal.x;
  goal[1] = world.goal.y;

  simple_setup.setStartAndGoalStates(start, goal);
  simple_setup.setPlanner(std::make_shared<og::RRTConnect>(simple_setup.getSpaceInformation()));

  const ob::PlannerStatus solved = simple_setup.solve(solve_timeout);
  if (!solved)
  {
    ROS_ERROR_STREAM("No path found within " << solve_timeout << " seconds.");
    return nullptr;
  }

  std::unique_ptr<og::PathGeometric> path(new og::PathGeometric(simple_setup.getSolutionPath()));
  if (!ompl_examples::isPathValid(*path, obstacles, robot))
  {
    ROS_ERROR("The planner returned a path that intersects a collision boundary.");
    return nullptr;
  }

  simple_setup.simplifySolution();
  const og::PathGeometric simplified_path = simple_setup.getSolutionPath();
  if (ompl_examples::isPathValid(simplified_path, obstacles, robot))
  {
    path.reset(new og::PathGeometric(simplified_path));
  }
  else
  {
    ROS_WARN("The simplified path intersects a collision boundary; using the original solution path.");
  }

  if (interpolate_count > 0)
  {
    path->interpolate(static_cast<unsigned int>(interpolate_count));
  }

  if (!ompl_examples::isPathValid(*path, obstacles, robot))
  {
    ROS_ERROR("The interpolated path intersects a collision boundary.");
    return nullptr;
  }

  return path;
}

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_2d_node");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  std::string world_config;
  std::string path_topic;
  std::string frame_id;
  std::string output_csv;
  double solve_timeout;
  int interpolate_count;

  private_node_handle.param<std::string>("world_config", world_config, "");
  private_node_handle.param<std::string>("path_topic", path_topic, "/ompl_examples/path");
  private_node_handle.param<std::string>("frame_id", frame_id, "map");
  private_node_handle.param<std::string>("output_csv", output_csv, "");
  private_node_handle.param<double>("solve_timeout", solve_timeout, 1.0);
  private_node_handle.param<int>("interpolate_count", interpolate_count, 1000);

  ompl_examples::PlanningWorld world = ompl_examples::makeDefaultWorld();
  if (!world_config.empty())
  {
    std::string error_message;
    if (!ompl_examples::loadWorldConfig(world_config, world, error_message))
    {
      ROS_ERROR_STREAM("Failed to load world config: " << world_config << ": " << error_message);
      return 1;
    }
  }

  ROS_INFO_STREAM("Planning from (" << world.start.x << ", " << world.start.y << ") to (" << world.goal.x << ", "
                                   << world.goal.y << ")");
  ROS_INFO_STREAM("Robot radius: " << world.robot.radius << ", safety margin: " << world.robot.safety_margin
                                   << ", obstacle padding: " << ompl_examples::getCollisionPadding(world.robot));

  std::unique_ptr<og::PathGeometric> path = solvePath(world, solve_timeout, interpolate_count);
  if (!path)
  {
    return 1;
  }

  if (!output_csv.empty() && !ompl_examples::writePathCsv(*path, output_csv))
  {
    ROS_ERROR_STREAM("Failed to write path CSV: " << output_csv);
    return 1;
  }

  if (!output_csv.empty())
  {
    ROS_INFO_STREAM("Wrote path CSV: " << output_csv);
  }

  ros::Publisher path_publisher = node_handle.advertise<nav_msgs::Path>(path_topic, 1, true);
  const nav_msgs::Path path_message = toPathMessage(*path, frame_id);
  path_publisher.publish(path_message);

  ROS_INFO_STREAM("Published " << path_message.poses.size() << " poses on " << path_topic);
  ROS_INFO("Keeping node alive so late subscribers can receive the latched path.");

  ros::spin();
  return 0;
}
