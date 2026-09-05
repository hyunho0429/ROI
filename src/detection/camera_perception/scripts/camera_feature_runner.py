#!/usr/bin/env python3
"""Run the integrated object or lane detector under roslaunch."""

import argparse
import os
from pathlib import Path
import sys


def default_stack_root():
    # source layout: <root>/src/detection/camera_perception/scripts/this_file.py
    return str(Path(__file__).resolve().parents[4])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("lane", "yolo"), required=True)
    parser.add_argument("--camera-stack-root", default=default_stack_root())
    parser.add_argument("--cam-ip", required=True)
    parser.add_argument("--cam-port", type=int, required=True)
    parser.add_argument("--base-model", default="yolov8n.pt")
    parser.add_argument("--custom-model", default="best0902.pt")
    parser.add_argument("--confidence", type=float, default=0.4)
    parser.add_argument("--inference-size", type=int, default=416)
    parser.add_argument("--display-fps", type=float, default=0.0)
    parser.add_argument("--cpu-threads", type=int, default=0)
    parser.add_argument("--traffic-light-topic", default="/detection/traffic_light")
    parser.add_argument("--obstacle-topic", default="/detection/obstacle")
    parser.add_argument("--lane-checkpoint", default="")
    parser.add_argument("--lane-cam-set", default="")
    parser.add_argument("--lane-device", default="")
    parser.add_argument("--lane-every", type=int, default=1)
    parser.add_argument("--lane-scale", type=float, default=1.0)
    parser.add_argument("--lane-bev", action="store_true")
    parser.add_argument("--lane-ros-publish", action="store_true")
    parser.add_argument(
        "--dashed-lane-topic",
        default="/perception/camera/dashed_lane_detected",
    )
    parser.add_argument(
        "--stopline-detected-topic",
        default="/perception/camera/stopline_detected",
    )
    parser.add_argument(
        "--stopline-distance-topic",
        default="/perception/camera/stopline_distance_m",
    )
    parser.add_argument(
        "--stopline-stop-topic",
        default="/perception/stopline/stop_required",
    )
    parser.add_argument("--stopline-stop-distance-m", type=float, default=1.0)
    parser.add_argument("--stopline-clear-confirmation-s", type=float, default=0.5)
    parser.add_argument(
        "--car-detected-topic", default="/perception/camera/car_detected"
    )
    parser.add_argument(
        "--person-detected-topic", default="/perception/camera/person_detected"
    )
    args, _ = parser.parse_known_args()

    try:
        import rospkg
        package_root = Path(rospkg.RosPack().get_path("camera_perception"))
    except ImportError:
        package_root = Path(__file__).resolve().parents[1]
    except rospkg.ResourceNotFound:
        package_root = Path(__file__).resolve().parents[1]
    if args.mode == "lane":
        target = package_root / "lane" / "live_overlay.py"
        target_args = [
            "--ip", args.cam_ip,
            "--port", str(args.cam_port),
            "--every", str(args.lane_every),
            "--scale", str(args.lane_scale),
        ]
        if args.lane_checkpoint:
            target_args.extend(("--checkpoint", args.lane_checkpoint))
        if args.lane_cam_set:
            target_args.extend(("--cam-set", args.lane_cam_set))
        if args.lane_device and args.lane_device.lower() != "auto":
            target_args.extend(("--device", args.lane_device))
        if args.lane_bev:
            target_args.append("--bev")
        if args.lane_ros_publish:
            target_args.extend(
                (
                    "--ros-publish",
                    "--dashed-lane-topic", args.dashed_lane_topic,
                    "--stopline-detected-topic", args.stopline_detected_topic,
                    "--stopline-distance-topic", args.stopline_distance_topic,
                    "--stopline-stop-topic", args.stopline_stop_topic,
                    "--stopline-stop-distance-m",
                    str(args.stopline_stop_distance_m),
                    "--stopline-clear-confirmation-s",
                    str(args.stopline_clear_confirmation_s),
                )
            )
    else:
        target = package_root / "scripts" / "camera_object_detection_node.py"
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
            "--traffic-light-topic", args.traffic_light_topic,
            "--obstacle-topic", args.obstacle_topic,
        ]

    if not target.is_file():
        parser.error(f"camera feature script not found: {target}")

    os.chdir(str(package_root))
    os.execv(sys.executable, [sys.executable, str(target)] + target_args)


if __name__ == "__main__":
    main()
