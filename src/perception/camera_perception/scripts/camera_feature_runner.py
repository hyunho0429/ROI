#!/usr/bin/env python3
"""Execute preserved feature-camera programs as roslaunch-managed processes."""

import argparse
import os
from pathlib import Path
import sys


def default_stack_root():
    # source layout: <root>/src/perception/camera_perception/scripts/this_file.py
    return str(Path(__file__).resolve().parents[4])


def parse_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode", choices=("lane", "lane_oracle", "yolo"), required=True
    )
    parser.add_argument("--camera-stack-root", default=default_stack_root())
    parser.add_argument("--cam-ip", required=True)
    parser.add_argument("--cam-port", type=int, required=True)
    parser.add_argument("--base-model", default="yolov8n.pt")
    parser.add_argument("--custom-model", default="null.pt")
    parser.add_argument("--confidence", type=float, default=0.4)
    parser.add_argument("--inference-size", type=int, default=416)
    parser.add_argument("--display-fps", type=float, default=0.0)
    parser.add_argument("--cpu-threads", type=int, default=0)
    parser.add_argument(
        "--car-detected-topic", default="/perception/camera/car_detected"
    )
    parser.add_argument(
        "--person-detected-topic", default="/perception/camera/person_detected"
    )
    parser.add_argument("--mgeo-path", default=None)
    parser.add_argument("--cam-set-path", default=None)
    parser.add_argument("--sensor-id", type=int, default=1)
    parser.add_argument("--ego-status-topic", default="/Ego_topic")
    parser.add_argument(
        "--dashed-lane-topic",
        default="/perception/camera/dashed_lane_detected",
    )
    parser.add_argument("--lane-oracle-overlay", type=parse_bool, default=False)
    args, _ = parser.parse_known_args()

    stack_root = Path(args.camera_stack_root).expanduser().resolve()
    sensor_dir = stack_root / "Sensor"
    if args.mode == "lane":
        target = sensor_dir / "LaneCandidates.py"
        target_args = ["--cam-ip", args.cam_ip, "--cam-port", str(args.cam_port)]
    elif args.mode == "lane_oracle":
        target = sensor_dir / "LiveLaneOracle.py"
        mgeo_path = (
            Path(args.mgeo_path).expanduser().resolve()
            if args.mgeo_path
            else stack_root
            / "src"
            / "path_planning"
            / "mgeo"
            / "R_KR_PR_K-city_2025"
        )
        cam_set_path = (
            Path(args.cam_set_path).expanduser().resolve()
            if args.cam_set_path
            else sensor_dir / "cam_set.json"
        )
        target_args = [
            "--mgeo", str(mgeo_path),
            "--cam-set", str(cam_set_path),
            "--sensor-id", str(args.sensor_id),
            "--cam-ip", args.cam_ip,
            "--cam-port", str(args.cam_port),
            "--ros-status-topic", args.ego_status_topic,
            "--dashed-lane-topic", args.dashed_lane_topic,
            "--no-udp",
        ]
        if not args.lane_oracle_overlay:
            target_args.append("--no-overlay")
    else:
        target = sensor_dir / "YoloCamera_v2.py"
        target_args = [
            "--cam-ip", args.cam_ip,
            "--cam-port", str(args.cam_port),
            "--base-model", args.base_model,
            "--custom-model", args.custom_model,
            "--confidence", str(args.confidence),
            "--inference-size", str(args.inference_size),
            "--display-fps", str(args.display_fps),
            "--cpu-threads", str(args.cpu_threads),
            "--car-detected-topic", args.car_detected_topic,
            "--person-detected-topic", args.person_detected_topic,
        ]

    if not target.is_file():
        parser.error(f"camera feature script not found: {target}")

    os.chdir(str(stack_root))
    os.execv(sys.executable, [sys.executable, str(target)] + target_args)


if __name__ == "__main__":
    main()
