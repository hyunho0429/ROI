#!/usr/bin/env python3
"""Strict LiDAR variant that removes every unsupported single-ring return."""

from morai_lidar_pointcloud_udp_ground import main


if __name__ == "__main__":
    main("strict")
