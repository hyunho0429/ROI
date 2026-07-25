#!/usr/bin/env python3
"""Generate a Stanley-compatible CSV path with RRT*.

Example:
    rosrun path_planning morai_rrt_star_path_generator.py \
      --start-x 0 --start-y 0 --goal-x 30 --goal-y 10 \
      --x-min -10 --x-max 40 --y-min -10 --y-max 20 \
      --obstacle 12,2,2.5 --output /tmp/rrt_path.csv
"""

import argparse
import csv
import math
import os

from path_planning.rrt_star import CircularObstacle, RRTStarPlanner, smooth_path


def _parse_obstacle(value):
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            "obstacle must be formatted as x,y,radius_m"
        )
    try:
        x_m, y_m, radius_m = (float(part) for part in parts)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "obstacle values must be numeric"
        ) from error
    if radius_m <= 0.0:
        raise argparse.ArgumentTypeError("obstacle radius must be positive")
    return CircularObstacle(x_m, y_m, radius_m)


def _parse_obstacles(value):
    if value is None or not value.strip() or value.strip().lower() in {"none", "null"}:
        return []
    return [_parse_obstacle(item) for item in value.split(";") if item.strip()]


def _heading_deg(first, second):
    return math.degrees(math.atan2(second[1] - first[1], second[0] - first[0]))


def _write_stanley_csv(path, output_file, target_speed_kmh):
    output_path = os.path.abspath(os.path.expanduser(output_file))
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["sequence", "x", "y", "z", "heading_deg", "target_speed_kmh"],
        )
        writer.writeheader()
        for index, point in enumerate(path):
            if len(path) == 1:
                heading = 0.0
            elif index < len(path) - 1:
                heading = _heading_deg(point, path[index + 1])
            else:
                heading = _heading_deg(path[index - 1], point)
            writer.writerow(
                {
                    "sequence": index,
                    "x": "{:.6f}".format(point[0]),
                    "y": "{:.6f}".format(point[1]),
                    "z": "0.000000",
                    "heading_deg": "{:.6f}".format(heading),
                    "target_speed_kmh": "{:.3f}".format(target_speed_kmh),
                }
            )
    return output_path


def argument_parser():
    parser = argparse.ArgumentParser(
        description="Generate an ENU CSV path using RRT* for the MORAI Stanley runner."
    )
    parser.add_argument("--start-x", type=float, required=True)
    parser.add_argument("--start-y", type=float, required=True)
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--x-min", type=float, required=True)
    parser.add_argument("--x-max", type=float, required=True)
    parser.add_argument("--y-min", type=float, required=True)
    parser.add_argument("--y-max", type=float, required=True)
    parser.add_argument(
        "--obstacle",
        action="append",
        type=_parse_obstacle,
        default=[],
        help="circular obstacle as x,y,radius_m; repeat for multiple obstacles",
    )
    parser.add_argument(
        "--obstacles",
        default="",
        help="semicolon-separated circular obstacles, e.g. '12,2,2.5;18,-1,1.8'",
    )
    parser.add_argument("--step-size-m", type=float, default=1.0)
    parser.add_argument("--max-iterations", type=int, default=1800)
    parser.add_argument("--goal-sample-rate", type=float, default=0.15)
    parser.add_argument("--search-radius-m", type=float, default=8.0)
    parser.add_argument("--goal-tolerance-m", type=float, default=2.0)
    parser.add_argument("--collision-margin-m", type=float, default=0.7)
    parser.add_argument("--random-seed", type=int, default=20)
    parser.add_argument("--smooth-iterations", type=int, default=60)
    parser.add_argument("--target-speed-kmh", type=float, default=20.0)
    parser.add_argument(
        "--output",
        default="/tmp/morai_rrt_star_path.csv",
        help="output CSV path consumed by morai_stanley_udp.launch path:=...",
    )
    return parser


def _validate(args):
    if args.x_min >= args.x_max or args.y_min >= args.y_max:
        raise ValueError("bounds must satisfy min < max")
    if args.step_size_m <= 0.0:
        raise ValueError("step-size-m must be positive")
    if args.max_iterations <= 0:
        raise ValueError("max-iterations must be positive")
    if not 0.0 <= args.goal_sample_rate <= 1.0:
        raise ValueError("goal-sample-rate must be in [0, 1]")
    if args.search_radius_m <= 0.0:
        raise ValueError("search-radius-m must be positive")
    if args.goal_tolerance_m <= 0.0:
        raise ValueError("goal-tolerance-m must be positive")
    if args.collision_margin_m < 0.0:
        raise ValueError("collision-margin-m cannot be negative")
    if args.target_speed_kmh < 0.0:
        raise ValueError("target-speed-kmh cannot be negative")


def main():
    args = argument_parser().parse_args()
    _validate(args)
    obstacles = list(args.obstacle) + _parse_obstacles(args.obstacles)
    planner = RRTStarPlanner(
        (args.start_x, args.start_y),
        (args.goal_x, args.goal_y),
        obstacles=obstacles,
        x_bounds=(args.x_min, args.x_max),
        y_bounds=(args.y_min, args.y_max),
        step_size_m=args.step_size_m,
        max_iterations=args.max_iterations,
        goal_sample_rate=args.goal_sample_rate,
        search_radius_m=args.search_radius_m,
        goal_tolerance_m=args.goal_tolerance_m,
        collision_margin_m=args.collision_margin_m,
        random_seed=args.random_seed,
    )
    raw_path = planner.plan()
    path = smooth_path(raw_path, iterations=args.smooth_iterations)
    output_path = _write_stanley_csv(path, args.output, args.target_speed_kmh)
    print(
        "RRT* path generated: {} points, {} obstacles -> {}".format(
            len(path), len(obstacles), output_path
        )
    )


if __name__ == "__main__":
    main()
