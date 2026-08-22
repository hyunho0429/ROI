#!/usr/bin/env python3
"""Execute preserved feature-camera programs as roslaunch-managed processes."""

import argparse
import os
from pathlib import Path
import sys


def default_stack_root():
    # source layout: <root>/src/perception/camera_perception/scripts/this_file.py
    return str(Path(__file__).resolve().parents[4])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("lane", "yolo"), required=True)
    parser.add_argument("--camera-stack-root", default=default_stack_root())
    parser.add_argument("--cam-ip", required=True)
    parser.add_argument("--cam-port", type=int, required=True)
    parser.add_argument("--base-model", default="yolov8n.pt")
    parser.add_argument("--custom-model", default="null.pt")
    parser.add_argument("--confidence", type=float, default=0.4)
    parser.add_argument(
        "--car-detected-topic", default="/perception/camera/car_detected"
    )
    args, _ = parser.parse_known_args()

    stack_root = Path(args.camera_stack_root).expanduser().resolve()
    sensor_dir = stack_root / "Sensor"
    if args.mode == "lane":
        target = sensor_dir / "LaneCandidates.py"
        target_args = ["--cam-ip", args.cam_ip, "--cam-port", str(args.cam_port)]
    else:
        target = sensor_dir / "YoloCamera_v2.py"
        target_args = [
            "--cam-ip", args.cam_ip,
            "--cam-port", str(args.cam_port),
            "--base-model", args.base_model,
            "--custom-model", args.custom_model,
            "--confidence", str(args.confidence),
            "--car-detected-topic", args.car_detected_topic,
        ]

    if not target.is_file():
        parser.error(f"camera feature script not found: {target}")

    os.chdir(str(stack_root))
    os.execv(sys.executable, [sys.executable, str(target)] + target_args)


if __name__ == "__main__":
    main()
