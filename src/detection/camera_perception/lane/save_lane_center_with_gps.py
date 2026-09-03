#!/usr/bin/env python3
"""
카메라 중심 waypoint + GPS/EKF pose 동시 저장기 (ROS1)

입력
----
1) /perception/camera/lane_info   std_msgs/String(JSON)
   - live_lane_info_publisher.py 출력
   - centerline_points: [[x_base, y_base], ...]

2) /localization/odometry         nav_msgs/Odometry
   - 차량의 map 좌표 pose
   - 다른 GPS/EKF odometry 토픽을 쓰면 --odom-topic 으로 변경

출력 CSV
--------
각 center waypoint마다 한 행씩 저장한다.

timestamp
lane_valid
confidence
output_status
waypoint_index

center_x_base_m
center_y_base_m

ego_x_map_m
ego_y_map_m
ego_yaw_rad

center_x_map_m
center_y_map_m

즉:
- base_link 중심점 원본
- 같은 시각의 차량 map pose
- 중심점을 map 좌표로 변환한 값

을 모두 저장한다.

좌표 변환
---------
base_link:
    +x = 전방
    +y = 좌측

map:
    odometry quaternion의 yaw 사용

X_map = ego_x + cos(yaw)*x_base - sin(yaw)*y_base
Y_map = ego_y + sin(yaw)*x_base + cos(yaw)*y_base

실행 예
-------
python3 save_lane_center_with_gps.py

토픽 변경:
python3 save_lane_center_with_gps.py \
    --lane-topic /perception/camera/lane_info \
    --odom-topic /localization/odometry

파일명 변경:
python3 save_lane_center_with_gps.py --output lane_gps_compare.csv
"""

import argparse
import csv
import json
import math
import os
import threading
import time

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    """geometry_msgs/Quaternion -> yaw [rad]"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def base_to_map(x_base, y_base, ego_x, ego_y, yaw):
    """base_link XY -> map XY"""
    c = math.cos(yaw)
    s = math.sin(yaw)

    x_map = ego_x + c * x_base - s * y_base
    y_map = ego_y + s * x_base + c * y_base

    return x_map, y_map


class LaneGpsLogger:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()

        self.latest_odom = None
        self.latest_odom_time = None

        self.rows_written = 0
        self.frames_written = 0

        out_path = os.path.abspath(args.output)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)

        self.fp = open(out_path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.fp)

        self.writer.writerow([
            "lane_timestamp",
            "ros_receive_time",
            "odom_timestamp",
            "odom_age_sec",

            "lane_valid",
            "confidence",
            "output_status",
            "lane_state",

            "waypoint_index",

            "center_x_base_m",
            "center_y_base_m",

            "ego_x_map_m",
            "ego_y_map_m",
            "ego_yaw_rad",

            "center_x_map_m",
            "center_y_map_m",

            "left_lane_type",
            "right_lane_type",

            "stopline_detected",
            "stopline_distance_m",
        ])
        self.fp.flush()

        rospy.Subscriber(
            args.odom_topic,
            Odometry,
            self.odom_cb,
            queue_size=20,
        )

        rospy.Subscriber(
            args.lane_topic,
            String,
            self.lane_cb,
            queue_size=10,
        )

        rospy.loginfo("[lane_gps_logger] lane topic : %s", args.lane_topic)
        rospy.loginfo("[lane_gps_logger] odom topic : %s", args.odom_topic)
        rospy.loginfo("[lane_gps_logger] output     : %s", out_path)
        rospy.loginfo(
            "[lane_gps_logger] max odom age: %.3f sec",
            args.max_odom_age
        )

    def odom_cb(self, msg):
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0:
            stamp = rospy.Time.now().to_sec()

        with self.lock:
            self.latest_odom = msg
            self.latest_odom_time = stamp

    def lane_cb(self, msg):
        receive_time = rospy.Time.now().to_sec()

        try:
            data = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn_throttle(
                2.0,
                "[lane_gps_logger] lane JSON parse fail: %s",
                str(e)
            )
            return

        # 잘못된 카메라 출력은 비교 데이터에 넣지 않는다.
        if not data.get("lane_valid", False):
            return

        center_points = data.get("centerline_points") or []
        if not center_points:
            return

        lane_timestamp = data.get("timestamp", receive_time)

        with self.lock:
            odom = self.latest_odom
            odom_time = self.latest_odom_time

        if odom is None or odom_time is None:
            rospy.logwarn_throttle(
                2.0,
                "[lane_gps_logger] 아직 odometry를 받지 못했습니다."
            )
            return

        # lane timestamp가 wall time 기반이고 odom은 ROS time일 수 있으므로,
        # 우선 receive_time 기준 age도 함께 안전하게 본다.
        odom_age = abs(receive_time - odom_time)

        if odom_age > self.args.max_odom_age:
            rospy.logwarn_throttle(
                2.0,
                "[lane_gps_logger] odom too old: %.3f sec",
                odom_age
            )
            return

        p = odom.pose.pose.position
        q = odom.pose.pose.orientation

        ego_x = float(p.x)
        ego_y = float(p.y)
        yaw = quat_to_yaw(q)

        confidence = data.get("confidence")
        output_status = data.get("output_status")
        lane_state = data.get("lane_state")

        left_lane = data.get("left_lane") or {}
        right_lane = data.get("right_lane") or {}

        left_type = left_lane.get("type")
        right_type = right_lane.get("type")

        stopline_detected = data.get("stopline_detected", False)
        stopline_dist = data.get("stopline_distance_m")

        wrote = 0

        for i, pt in enumerate(center_points):
            if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                continue

            try:
                xb = float(pt[0])
                yb = float(pt[1])
            except (TypeError, ValueError):
                continue

            xm, ym = base_to_map(
                xb, yb,
                ego_x, ego_y,
                yaw
            )

            self.writer.writerow([
                lane_timestamp,
                receive_time,
                odom_time,
                odom_age,

                True,
                confidence,
                output_status,
                lane_state,

                i,

                xb,
                yb,

                ego_x,
                ego_y,
                yaw,

                xm,
                ym,

                left_type,
                right_type,

                stopline_detected,
                stopline_dist,
            ])

            wrote += 1

        if wrote:
            self.fp.flush()
            self.frames_written += 1
            self.rows_written += wrote

            rospy.loginfo_throttle(
                1.0,
                "[lane_gps_logger] frames=%d rows=%d latest_center_points=%d",
                self.frames_written,
                self.rows_written,
                wrote
            )

    def close(self):
        try:
            self.fp.flush()
            self.fp.close()
        except Exception:
            pass


def build_parser():
    ap = argparse.ArgumentParser(
        description="카메라 중심 waypoint와 GPS/EKF pose를 같은 CSV에 저장"
    )

    ap.add_argument(
        "--lane-topic",
        default="/perception/camera/lane_info",
    )

    ap.add_argument(
        "--odom-topic",
        default="/localization/odometry",
        help="GPS/EKF 기반 차량 pose를 주는 nav_msgs/Odometry 토픽",
    )

    ap.add_argument(
        "--output",
        default="lane_gps_compare.csv",
    )

    ap.add_argument(
        "--max-odom-age",
        type=float,
        default=0.20,
        help="lane 수신 시 허용하는 최신 odometry 최대 age [sec]",
    )

    return ap


def main():
    args = build_parser().parse_args()

    rospy.init_node(
        "lane_center_gps_logger",
        anonymous=False
    )

    logger = LaneGpsLogger(args)

    rospy.on_shutdown(logger.close)

    try:
        rospy.spin()
    finally:
        logger.close()


if __name__ == "__main__":
    main()
