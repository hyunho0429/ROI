#!/usr/bin/env python3
"""
카메라 좌/우 차선 + 중심 waypoint + GPS/EKF pose 동시 저장기 (ROS1)

입력
----
1) /perception/camera/lane_info   std_msgs/String(JSON)
   - live_lane_info_publisher_v2.py 출력
   - left_boundary_points
   - centerline_points
   - right_boundary_points
   - left_lane.type / right_lane.type

2) /localization/odometry         nav_msgs/Odometry
   - 차량의 map 좌표 pose

출력 CSV
--------
한 파일에 좌/중앙/우측 점을 모두 저장한다.

point_role:
    LEFT_BOUNDARY
    CENTERLINE
    RIGHT_BOUNDARY

각 행:
- base_link 좌표
- map 좌표
- 차선 종류
- dashed 여부
- lane 상태/신뢰도
- GPS/EKF ego pose
- 정지선 정보

MATLAB에서 point_role과 lane_type을 기준으로
좌/우 차선을 실선/점선으로 복구할 수 있다.
"""

import argparse
import csv
import json
import math
import os
import threading

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def base_to_map(x_base, y_base, ego_x, ego_y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)

    x_map = ego_x + c * x_base - s * y_base
    y_map = ego_y + s * x_base + c * y_base

    return x_map, y_map


class LaneMapLogger:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()

        self.latest_odom = None
        self.latest_odom_time = None

        self.frames_written = 0
        self.rows_written = 0

        out_path = os.path.abspath(args.output)
        out_dir = os.path.dirname(out_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

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
            "center_source",

            "point_role",
            "point_index",

            "point_x_base_m",
            "point_y_base_m",

            "ego_x_map_m",
            "ego_y_map_m",
            "ego_yaw_rad",

            "point_x_map_m",
            "point_y_map_m",

            "lane_type",
            "lane_dashed",

            "left_lane_type",
            "left_lane_dashed",
            "right_lane_type",
            "right_lane_dashed",

            "lane_width_m",

            "stopline_detected",
            "stopline_distance_m",

            "reasons",
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

        rospy.loginfo("[lane_map_logger] lane topic : %s", args.lane_topic)
        rospy.loginfo("[lane_map_logger] odom topic : %s", args.odom_topic)
        rospy.loginfo("[lane_map_logger] output     : %s", out_path)
        rospy.loginfo("[lane_map_logger] save invalid frames: %s",
                      args.save_invalid)

    def odom_cb(self, msg):
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0:
            stamp = rospy.Time.now().to_sec()

        with self.lock:
            self.latest_odom = msg
            self.latest_odom_time = stamp

    def _write_points(
        self,
        points,
        point_role,
        lane_type,
        lane_dashed,
        common,
    ):
        wrote = 0

        for i, pt in enumerate(points):
            if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                continue

            try:
                xb = float(pt[0])
                yb = float(pt[1])
            except (TypeError, ValueError):
                continue

            xm, ym = base_to_map(
                xb,
                yb,
                common["ego_x"],
                common["ego_y"],
                common["yaw"],
            )

            self.writer.writerow([
                common["lane_timestamp"],
                common["receive_time"],
                common["odom_time"],
                common["odom_age"],

                common["lane_valid"],
                common["confidence"],
                common["output_status"],
                common["lane_state"],
                common["center_source"],

                point_role,
                i,

                xb,
                yb,

                common["ego_x"],
                common["ego_y"],
                common["yaw"],

                xm,
                ym,

                lane_type,
                lane_dashed,

                common["left_type"],
                common["left_dashed"],
                common["right_type"],
                common["right_dashed"],

                common["lane_width"],

                common["stopline_detected"],
                common["stopline_dist"],

                common["reasons"],
            ])

            wrote += 1

        return wrote

    def lane_cb(self, msg):
        receive_time = rospy.Time.now().to_sec()

        try:
            data = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn_throttle(
                2.0,
                "[lane_map_logger] lane JSON parse fail: %s",
                str(e),
            )
            return

        lane_valid = bool(data.get("lane_valid", False))

        if not lane_valid and not self.args.save_invalid:
            return

        left_points = data.get("left_boundary_points") or []
        center_points = data.get("centerline_points") or []
        right_points = data.get("right_boundary_points") or []

        # invalid 프레임에서는 기존 publisher가 points를 []로 내보낼 수 있다.
        # 그래도 상태 자체를 분석하고 싶으면 아래에서 STATE_ONLY 한 행을 저장한다.
        any_points = bool(left_points or center_points or right_points)

        lane_timestamp = data.get("timestamp", receive_time)

        with self.lock:
            odom = self.latest_odom
            odom_time = self.latest_odom_time

        if odom is None or odom_time is None:
            rospy.logwarn_throttle(
                2.0,
                "[lane_map_logger] 아직 odometry를 받지 못했습니다.",
            )
            return

        odom_age = abs(receive_time - odom_time)

        if odom_age > self.args.max_odom_age:
            rospy.logwarn_throttle(
                2.0,
                "[lane_map_logger] odom too old: %.3f sec",
                odom_age,
            )
            return

        p = odom.pose.pose.position
        q = odom.pose.pose.orientation

        ego_x = float(p.x)
        ego_y = float(p.y)
        yaw = quat_to_yaw(q)

        left_lane = data.get("left_lane") or {}
        right_lane = data.get("right_lane") or {}

        left_type = left_lane.get("type")
        left_dashed = left_lane.get("dashed")

        right_type = right_lane.get("type")
        right_dashed = right_lane.get("dashed")

        reasons = data.get("reasons") or []
        if isinstance(reasons, list):
            reasons = "|".join(str(x) for x in reasons)
        else:
            reasons = str(reasons)

        common = {
            "lane_timestamp": lane_timestamp,
            "receive_time": receive_time,
            "odom_time": odom_time,
            "odom_age": odom_age,

            "lane_valid": lane_valid,
            "confidence": data.get("confidence"),
            "output_status": data.get("output_status"),
            "lane_state": data.get("lane_state"),
            "center_source": data.get("center_source"),

            "ego_x": ego_x,
            "ego_y": ego_y,
            "yaw": yaw,

            "left_type": left_type,
            "left_dashed": left_dashed,
            "right_type": right_type,
            "right_dashed": right_dashed,

            "lane_width": data.get("lane_width_m"),

            "stopline_detected": data.get("stopline_detected", False),
            "stopline_dist": data.get("stopline_distance_m"),

            "reasons": reasons,
        }

        wrote = 0

        wrote += self._write_points(
            left_points,
            "LEFT_BOUNDARY",
            left_type,
            left_dashed,
            common,
        )

        # 중심선은 물리적인 차선 종류가 아니므로 CENTERLINE로 기록
        wrote += self._write_points(
            center_points,
            "CENTERLINE",
            "CENTERLINE",
            False,
            common,
        )

        wrote += self._write_points(
            right_points,
            "RIGHT_BOUNDARY",
            right_type,
            right_dashed,
            common,
        )

        # invalid 등으로 점이 하나도 없어도 상태 분석용 한 행 저장 가능
        if not any_points and self.args.save_invalid:
            self.writer.writerow([
                lane_timestamp,
                receive_time,
                odom_time,
                odom_age,

                lane_valid,
                data.get("confidence"),
                data.get("output_status"),
                data.get("lane_state"),
                data.get("center_source"),

                "STATE_ONLY",
                -1,

                "",
                "",

                ego_x,
                ego_y,
                yaw,

                "",
                "",

                "",
                "",

                left_type,
                left_dashed,
                right_type,
                right_dashed,

                data.get("lane_width_m"),

                data.get("stopline_detected", False),
                data.get("stopline_distance_m"),

                reasons,
            ])
            wrote += 1

        if wrote:
            self.fp.flush()
            self.frames_written += 1
            self.rows_written += wrote

            rospy.loginfo_throttle(
                1.0,
                "[lane_map_logger] frames=%d rows=%d "
                "L=%d C=%d R=%d valid=%s",
                self.frames_written,
                self.rows_written,
                len(left_points),
                len(center_points),
                len(right_points),
                lane_valid,
            )

    def close(self):
        try:
            self.fp.flush()
            self.fp.close()
        except Exception:
            pass


def build_parser():
    ap = argparse.ArgumentParser(
        description="좌/우 차선+중심선+GPS/EKF를 map 좌표 CSV로 저장"
    )

    ap.add_argument(
        "--lane-topic",
        default="/perception/camera/lane_info",
    )

    ap.add_argument(
        "--odom-topic",
        default="/localization/odometry",
    )

    ap.add_argument(
        "--output",
        default="lane_map_compare.csv",
    )

    ap.add_argument(
        "--max-odom-age",
        type=float,
        default=0.20,
    )

    ap.add_argument(
        "--save-invalid",
        action="store_true",
        help="lane_valid=false 상태도 STATE_ONLY 행으로 저장",
    )

    return ap


def main():
    args = build_parser().parse_args()

    rospy.init_node(
        "lane_map_logger",
        anonymous=False,
    )

    logger = LaneMapLogger(args)
    rospy.on_shutdown(logger.close)

    try:
        rospy.spin()
    finally:
        logger.close()


if __name__ == "__main__":
    main()
