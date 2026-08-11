#!/usr/bin/env python3
"""MGeo 경로 점을 nav_msgs/Odometry pose로 재생하는 Pure Pursuit 테스트 노드."""

from __future__ import annotations

import math
import os

import rospy
from nav_msgs.msg import Odometry

from purepursuit_mgeo.path import PathPoint, load_mgeo_path


def yaw_to_quaternion(yaw_rad: float):
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def path_yaw(points, index: int, previous_yaw: float) -> float:
    """현재 점에서 다음 유효한 경로 점을 향하는 yaw를 계산한다."""

    current = points[index]
    for offset in range(1, len(points)):
        candidate = points[(index + offset) % len(points)]
        dx = candidate.x - current.x
        dy = candidate.y - current.y
        if math.hypot(dx, dy) > 1e-6:
            return math.atan2(dy, dx)
    return previous_yaw


class MgeoPathPoseReplay:
    def __init__(self) -> None:
        rospy.init_node("replay_mgeo_path_pose", anonymous=False)

        default_path = os.path.join(
            os.environ.get("HOME", "/home"),
            "morai_ws",
            "data",
            "routes",
            "2026_molit_comp_global_path.txt",
        )
        path_file = rospy.get_param("~path_file", default_path)
        self.points = load_mgeo_path(path_file)
        self.pose_topic = rospy.get_param("~pose_topic", "/localization/odometry")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))
        self.speed_mps = float(rospy.get_param("~speed_mps", 2.0))
        self.loop = bool(rospy.get_param("~loop", False))
        self.goal_hold_sec = float(rospy.get_param("~goal_hold_sec", 1.0))

        self.publisher = rospy.Publisher(self.pose_topic, Odometry, queue_size=10)
        rospy.loginfo(
            "MGeo pose replay: path=%s points=%d topic=%s speed=%.2f loop=%s",
            path_file,
            len(self.points),
            self.pose_topic,
            self.speed_mps,
            self.loop,
        )

    def publish_pose(self, point: PathPoint, yaw: float) -> None:
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.map_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = point.x
        message.pose.pose.position.y = point.y
        message.pose.pose.position.z = point.z
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = self.speed_mps * math.cos(yaw)
        message.twist.twist.linear.y = self.speed_mps * math.sin(yaw)
        self.publisher.publish(message)

    def run(self) -> None:
        rate = rospy.Rate(max(self.rate_hz, 1.0))
        index = 0
        yaw = 0.0

        while not rospy.is_shutdown():
            point = self.points[index]
            yaw = path_yaw(self.points, index, yaw)
            self.publish_pose(point, yaw)

            if index >= len(self.points) - 1:
                if self.loop:
                    index = 0
                else:
                    end_time = rospy.Time.now() + rospy.Duration(max(self.goal_hold_sec, 0.0))
                    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                        self.publish_pose(point, yaw)
                        rate.sleep()
                    rospy.loginfo("MGeo 경로 pose 재생을 완료했다.")
                    return
            else:
                index += 1
            rate.sleep()


if __name__ == "__main__":
    try:
        MgeoPathPoseReplay().run()
    except rospy.ROSInterruptException:
        pass
