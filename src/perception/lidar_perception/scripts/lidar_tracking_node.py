#!/usr/bin/env python3
"""Run the standalone Kalman-plus-Hungarian tracking RViz demo."""

from lidar_perception.lidar_perception_ros import run_kalman_hungarian


if __name__ == "__main__":
    run_kalman_hungarian()
