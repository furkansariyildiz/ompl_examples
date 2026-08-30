#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import yaml


DEFAULT_WORLD = {
    "bounds": {
        "low": 0.0,
        "high": 10.0,
    },
    "start": {
        "x": 1.0,
        "y": 1.0,
    },
    "goal": {
        "x": 9.0,
        "y": 9.0,
    },
    "obstacles": [
        {"center": {"x": 5.0, "y": 5.0}, "radius": 1.25},
        {"center": {"x": 3.0, "y": 6.5}, "radius": 1.0},
        {"center": {"x": 7.0, "y": 3.0}, "radius": 1.0},
    ],
}


def read_path_csv(file_path):
    path_points = []

    with open(file_path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            path_points.append((float(row["x"]), float(row["y"])))

    return path_points


def read_world_config(file_path):
    if not file_path:
        return DEFAULT_WORLD

    with open(file_path, "r") as yaml_file:
        world = yaml.safe_load(yaml_file)

    required_fields = ("bounds", "start", "goal", "obstacles")
    for field in required_fields:
        if field not in world:
            raise ValueError(f"World config is missing required field: {field}")

    return world


def squared_distance_to_segment(point, segment_start, segment_end):
    segment_x = segment_end[0] - segment_start[0]
    segment_y = segment_end[1] - segment_start[1]
    segment_length_squared = segment_x * segment_x + segment_y * segment_y

    if segment_length_squared == 0.0:
        dx = point[0] - segment_start[0]
        dy = point[1] - segment_start[1]
        return dx * dx + dy * dy

    projected = (
        (point[0] - segment_start[0]) * segment_x
        + (point[1] - segment_start[1]) * segment_y
    ) / segment_length_squared
    clamped = max(0.0, min(1.0, projected))
    closest = (
        segment_start[0] + clamped * segment_x,
        segment_start[1] + clamped * segment_y,
    )

    dx = point[0] - closest[0]
    dy = point[1] - closest[1]
    return dx * dx + dy * dy


def find_path_intersections(path_points, world):
    intersections = []

    for segment_index in range(1, len(path_points)):
        segment_start = path_points[segment_index - 1]
        segment_end = path_points[segment_index]

        for obstacle_index, obstacle_config in enumerate(world["obstacles"]):
            center = obstacle_config["center"]
            obstacle_center = (center["x"], center["y"])
            radius = obstacle_config["radius"]
            distance_squared = squared_distance_to_segment(
                obstacle_center, segment_start, segment_end
            )

            if distance_squared <= radius * radius:
                intersections.append(
                    {
                        "segment_index": segment_index,
                        "obstacle_index": obstacle_index,
                        "distance": distance_squared**0.5,
                        "radius": radius,
                    }
                )

    return intersections


def validate_path_against_world(path_points, world):
    intersections = find_path_intersections(path_points, world)
    if not intersections:
        return

    first = intersections[0]
    raise ValueError(
        "Path CSV intersects the configured obstacles. "
        f"First hit: segment {first['segment_index']} intersects obstacle "
        f"{first['obstacle_index']} "
        f"(distance {first['distance']:.6f} <= radius {first['radius']:.6f}). "
        "Regenerate the CSV with the same YAML config before plotting."
    )


def plot_scene(path_points, world, output_path=None):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    world_min = world["bounds"]["low"]
    world_max = world["bounds"]["high"]
    start = (world["start"]["x"], world["start"]["y"])
    goal = (world["goal"]["x"], world["goal"]["y"])

    figure, axes = plt.subplots(figsize=(7, 7))

    axes.set_title("2D OMPL Path")
    axes.set_xlabel("x")
    axes.set_ylabel("y")
    axes.set_xlim(world_min, world_max)
    axes.set_ylim(world_min, world_max)
    axes.set_aspect("equal", adjustable="box")
    axes.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)

    for obstacle_config in world["obstacles"]:
        center = obstacle_config["center"]
        obstacle = Circle(
            (center["x"], center["y"]),
            obstacle_config["radius"],
            facecolor="tab:red",
            edgecolor="darkred",
            alpha=0.35,
            linewidth=1.5,
        )
        axes.add_patch(obstacle)

    if path_points:
        path_x = [point[0] for point in path_points]
        path_y = [point[1] for point in path_points]
        axes.plot(path_x, path_y, color="tab:blue", linewidth=2.0, label="Path")

    axes.scatter(*start, color="tab:green", s=80, label="Start", zorder=3)
    axes.scatter(*goal, color="tab:purple", s=80, label="Goal", zorder=3)
    axes.legend(loc="upper left")

    figure.tight_layout()

    if output_path:
        figure.savefig(output_path, dpi=160)
        print(f"Wrote plot image: {output_path}")
    else:
        plt.show()


def parse_args():
    parser = argparse.ArgumentParser(description="Plot a 2D OMPL path CSV with matplotlib.")
    parser.add_argument("path_csv", help="Path CSV generated by plan_2d.")
    parser.add_argument(
        "--config",
        "-c",
        help="Optional YAML world config used by plan_2d.",
    )
    parser.add_argument(
        "--output",
        "-o",
        help="Optional image output path. If omitted, an interactive plot window is shown.",
    )
    parser.add_argument(
        "--allow-invalid",
        action="store_true",
        help="Plot even if the CSV path intersects the configured obstacles.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    path_csv = Path(args.path_csv)
    config_path = Path(args.config) if args.config else None

    if not path_csv.exists():
        raise FileNotFoundError(f"Path CSV does not exist: {path_csv}")

    if config_path and not config_path.exists():
        raise FileNotFoundError(f"World config does not exist: {config_path}")

    output_path = Path(args.output) if args.output else None
    if output_path:
        import matplotlib

        matplotlib.use("Agg")

    path_points = read_path_csv(path_csv)
    world = read_world_config(config_path)
    if not args.allow_invalid:
        validate_path_against_world(path_points, world)

    plot_scene(path_points, world, output_path)


if __name__ == "__main__":
    main()
