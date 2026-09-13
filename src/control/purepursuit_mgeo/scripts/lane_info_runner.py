#!/usr/bin/env python3
"""Launch the camera team's existing live_lane_info_publisher_v2.py unchanged.

This wrapper belongs to purepursuit_mgeo.  It only locates and executes the
existing camera_perception/lane/live_lane_info_publisher_v2.py so the unified
stack publishes /perception/camera/lane_info without modifying camera-team
source files.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

import rospkg


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=1101)
    ap.add_argument("--checkpoint", default="")
    ap.add_argument("--cam-set", default="")
    ap.add_argument("--device", default="auto")
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--topic", default="/perception/camera/lane_info")
    args, _ = ap.parse_known_args()

    pkg_root = Path(rospkg.RosPack().get_path("camera_perception"))
    target = pkg_root / "lane" / "live_lane_info_publisher_v2.py"
    if not target.is_file():
        raise SystemExit(f"lane-info publisher not found: {target}")

    target_args = [
        "--ip", args.ip,
        "--port", str(args.port),
        "--every", str(max(1, args.every)),
        "--topic", args.topic,
    ]
    if args.checkpoint:
        target_args += ["--checkpoint", args.checkpoint]
    if args.cam_set:
        target_args += ["--cam-set", args.cam_set]
    if args.device and args.device.lower() != "auto":
        target_args += ["--device", args.device]

    os.chdir(str(target.parent))
    os.execv(sys.executable, [sys.executable, str(target)] + target_args)


if __name__ == "__main__":
    main()
