#!/usr/bin/env python3

from pathlib import Path

import rospy
from nav_msgs.msg import Path as RosPath
from plot_2d_path import (
    get_collision_padding,
    get_collision_radius,
    read_world_config,
    validate_path_against_world,
)


class LivePathPlotter:
    def __init__(self):
        self.world_config = rospy.get_param("~world_config", "")
        self.path_topic = rospy.get_param("~path_topic", "/ompl_examples/path")
        self.output_path = rospy.get_param("~output", "")
        self.once = rospy.get_param("~once", False)

        if self.output_path:
            import matplotlib

            matplotlib.use("Agg")

        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle

        self.plt = plt
        self.world = read_world_config(Path(self.world_config) if self.world_config else None)

        world_min = self.world["bounds"]["low"]
        world_max = self.world["bounds"]["high"]
        start = (self.world["start"]["x"], self.world["start"]["y"])
        goal = (self.world["goal"]["x"], self.world["goal"]["y"])

        self.figure, self.axes = plt.subplots(figsize=(7, 7))
        self.axes.set_title("Live 2D OMPL Path")
        self.axes.set_xlabel("x")
        self.axes.set_ylabel("y")
        self.axes.set_xlim(world_min, world_max)
        self.axes.set_ylim(world_min, world_max)
        self.axes.set_aspect("equal", adjustable="box")
        self.axes.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)

        collision_padding = get_collision_padding(self.world)
        for obstacle_index, obstacle_config in enumerate(self.world["obstacles"]):
            center = obstacle_config["center"]
            obstacle = Circle(
                (center["x"], center["y"]),
                obstacle_config["radius"],
                facecolor="tab:red",
                edgecolor="darkred",
                alpha=0.35,
                linewidth=1.5,
                label="Obstacle" if obstacle_index == 0 else None,
            )
            self.axes.add_patch(obstacle)

            if collision_padding > 0.0:
                collision_boundary = Circle(
                    (center["x"], center["y"]),
                    get_collision_radius(obstacle_config, self.world),
                    facecolor="none",
                    edgecolor="tab:orange",
                    linestyle="--",
                    linewidth=1.5,
                    label="Collision boundary" if obstacle_index == 0 else None,
                )
                self.axes.add_patch(collision_boundary)

        (self.path_line,) = self.axes.plot([], [], color="tab:blue", linewidth=2.0, label="Path")
        self.axes.scatter(*start, color="tab:green", s=80, label="Start", zorder=3)
        self.axes.scatter(*goal, color="tab:purple", s=80, label="Goal", zorder=3)
        self.axes.legend(loc="upper left")
        self.figure.tight_layout()

        self.subscriber = rospy.Subscriber(self.path_topic, RosPath, self.path_callback, queue_size=1)
        rospy.loginfo("Listening for nav_msgs/Path on %s", self.path_topic)

    def path_callback(self, message):
        path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
        if not path_points:
            rospy.logwarn("Received an empty path message.")
            return

        try:
            validate_path_against_world(path_points, self.world)
        except ValueError as error:
            rospy.logerr(str(error))
            return

        path_x = [point[0] for point in path_points]
        path_y = [point[1] for point in path_points]
        self.path_line.set_data(path_x, path_y)
        self.axes.relim()
        self.axes.autoscale_view(False, False, False)
        self.figure.canvas.draw_idle()

        if self.output_path:
            self.figure.savefig(self.output_path, dpi=160)
            rospy.loginfo("Wrote live plot image: %s", self.output_path)

        if self.once:
            rospy.signal_shutdown("Received one path message.")

    def spin(self):
        if self.output_path:
            rospy.spin()
            return

        self.plt.ion()
        self.plt.show(block=False)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.plt.pause(0.01)
            rate.sleep()


def main():
    rospy.init_node("live_plot_2d_path")
    plotter = LivePathPlotter()
    plotter.spin()


if __name__ == "__main__":
    main()
