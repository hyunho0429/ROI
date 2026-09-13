#!/usr/bin/env python3
"""Generic gap-acceptance gate for roundabout / yield-line insertion.

The global path still defines HOW the car enters the roundabout.  This node only
answers WHEN it may enter.  A mission/region detector asserts ``merge_request``
near the yield line; while requested, map-frame LiDAR tracks are transformed to
the ego frame and tested against a configurable conflict point/zone.

Safety behavior
---------------
* stale odometry/obstacles while requested => STOP;
* current occupancy near the conflict point => STOP;
* moving object predicted to pass the conflict point inside the required time
  gap => STOP;
* gap must remain clear for ``clear_confirm_s`` before release;
* after release, GO is latched until merge_request becomes false, preventing a
  dangerous stop/restart oscillation after the vehicle has committed to entry.

No /ctrl_cmd is published here. Output ``~stop_required`` is OR'ed in the
existing Pure Pursuit controller.  An optional scenario-specific intersection
stop override is performed only in Pure Pursuit when merge_request AND
merge_allowed are both true.
"""

from __future__ import annotations

import json
import math
from typing import List, Optional

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from lidar_perception.msg import LidarObstacleArray


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class RoundaboutMergeGate:
    IDLE = "IDLE"
    WAIT = "WAIT_GAP"
    CONFIRM = "CONFIRM_CLEAR"
    COMMITTED = "COMMITTED_GO"
    SENSOR_STOP = "SENSOR_STOP"

    def __init__(self) -> None:
        rospy.init_node("roundabout_merge_gate", anonymous=False)

        self.request_topic = rospy.get_param("~request_topic", "/planning/merge_request")
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odometry")
        self.obstacle_topic = rospy.get_param(
            "~obstacle_topic", "/perception/lidar/tracked_obstacles_map"
        )
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 0.50))
        self.obstacle_timeout_s = float(rospy.get_param("~obstacle_timeout_s", 0.60))

        # Conflict point is expressed in current base_link/FLU coordinates.
        # The mission detector should assert request close to the yield line.
        self.conflict_x_m = float(rospy.get_param("~conflict_x_m", 7.0))
        self.conflict_y_m = float(rospy.get_param("~conflict_y_m", 0.0))
        self.conflict_radius_m = float(rospy.get_param("~conflict_radius_m", 3.0))

        # Roundabout circulating traffic normally approaches from the ego-left.
        # Keep the zone configurable so this node can be reused for other merges.
        self.search_x_min_m = float(rospy.get_param("~search_x_min_m", -12.0))
        self.search_x_max_m = float(rospy.get_param("~search_x_max_m", 25.0))
        self.search_y_min_m = float(rospy.get_param("~search_y_min_m", 0.0))
        self.search_y_max_m = float(rospy.get_param("~search_y_max_m", 22.0))
        self.minimum_moving_speed_mps = float(
            rospy.get_param("~minimum_moving_speed_mps", 0.25)
        )
        self.prediction_horizon_s = float(
            rospy.get_param("~prediction_horizon_s", 8.0)
        )
        self.required_arrival_gap_s = float(
            rospy.get_param("~required_arrival_gap_s", 4.0)
        )
        self.clear_confirm_s = float(rospy.get_param("~clear_confirm_s", 0.8))
        self.extra_obstacle_radius_m = float(
            rospy.get_param("~extra_obstacle_radius_m", 0.5)
        )

        self.requested = False
        self.latest_odom: Optional[Odometry] = None
        self.latest_odom_at: Optional[rospy.Time] = None
        self.latest_obstacles: Optional[LidarObstacleArray] = None
        self.latest_obstacles_at: Optional[rospy.Time] = None
        self.clear_since: Optional[rospy.Time] = None
        self.committed = False
        self.state = self.IDLE

        self.stop_pub = rospy.Publisher("~stop_required", Bool, queue_size=1)
        self.allowed_pub = rospy.Publisher("~allowed", Bool, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)

        rospy.Subscriber(self.request_topic, Bool, self._request_cb, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=5)
        rospy.Subscriber(self.obstacle_topic, LidarObstacleArray, self._obstacle_cb, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self._timer_cb
        )
        rospy.logwarn(
            "Roundabout merge gate ready: request=%s conflict=(%.1f,%.1f) radius=%.1fm required_gap=%.1fs",
            self.request_topic,
            self.conflict_x_m,
            self.conflict_y_m,
            self.conflict_radius_m,
            self.required_arrival_gap_s,
        )

    def _request_cb(self, msg: Bool) -> None:
        new_value = bool(msg.data)
        if self.requested and not new_value:
            self.committed = False
            self.clear_since = None
            self.state = self.IDLE
        self.requested = new_value

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.latest_odom_at = rospy.Time.now()

    def _obstacle_cb(self, msg: LidarObstacleArray) -> None:
        self.latest_obstacles = msg
        self.latest_obstacles_at = rospy.Time.now()

    def _fresh(self, now: rospy.Time) -> bool:
        if self.latest_odom_at is None or self.latest_obstacles_at is None:
            return False
        return (
            (now - self.latest_odom_at).to_sec() <= self.odom_timeout_s
            and (now - self.latest_obstacles_at).to_sec() <= self.obstacle_timeout_s
        )

    def _blocking_objects(self) -> List[dict]:
        if self.latest_odom is None or self.latest_obstacles is None:
            return []
        pose = self.latest_odom.pose.pose
        yaw = _yaw_from_quaternion(pose.orientation)
        c = math.cos(yaw)
        s = math.sin(yaw)
        blockers: List[dict] = []

        for obs in self.latest_obstacles.obstacles:
            dx = float(obs.center_x_map) - float(pose.position.x)
            dy = float(obs.center_y_map) - float(pose.position.y)
            # map -> base_link: R(-yaw)
            ox = c * dx + s * dy
            oy = -s * dx + c * dy
            if not (
                self.search_x_min_m <= ox <= self.search_x_max_m
                and self.search_y_min_m <= oy <= self.search_y_max_m
            ):
                continue

            vx = c * float(obs.velocity_x_map) + s * float(obs.velocity_y_map)
            vy = -s * float(obs.velocity_x_map) + c * float(obs.velocity_y_map)
            speed = math.hypot(vx, vy)
            rx = ox - self.conflict_x_m
            ry = oy - self.conflict_y_m
            obstacle_radius = 0.5 * math.hypot(
                max(0.1, float(obs.length)), max(0.1, float(obs.width))
            )
            allowed_radius = self.conflict_radius_m + obstacle_radius + self.extra_obstacle_radius_m
            current_dist = math.hypot(rx, ry)

            reason = None
            t_ca = None
            d_ca = None
            if current_dist <= allowed_radius:
                reason = "occupying_conflict_zone"
            elif speed >= self.minimum_moving_speed_mps:
                vv = vx * vx + vy * vy
                t = -(rx * vx + ry * vy) / max(vv, 1.0e-9)
                t = max(0.0, min(self.prediction_horizon_s, t))
                cx = rx + vx * t
                cy = ry + vy * t
                closest = math.hypot(cx, cy)
                t_ca = t
                d_ca = closest
                if closest <= allowed_radius and t <= self.required_arrival_gap_s:
                    reason = "predicted_conflict"

            if reason is not None:
                blockers.append(
                    {
                        "id": int(obs.id),
                        "reason": reason,
                        "x": round(ox, 2),
                        "y": round(oy, 2),
                        "speed_mps": round(speed, 2),
                        "current_dist_m": round(current_dist, 2),
                        "t_closest_s": None if t_ca is None else round(t_ca, 2),
                        "closest_dist_m": None if d_ca is None else round(d_ca, 2),
                    }
                )
        return blockers

    def _timer_cb(self, _event) -> None:
        now = rospy.Time.now()
        stop = False
        allowed = False
        blockers: List[dict] = []
        freshness = self._fresh(now)

        if not self.requested:
            self.state = self.IDLE
            self.committed = False
            self.clear_since = None
        elif self.committed:
            # Never retract GO while the vehicle is already entering. Mission
            # logic must drop merge_request after the insertion segment.
            self.state = self.COMMITTED
            allowed = True
            stop = False
        elif not freshness:
            self.state = self.SENSOR_STOP
            self.clear_since = None
            stop = True
        else:
            blockers = self._blocking_objects()
            if blockers:
                self.state = self.WAIT
                self.clear_since = None
                stop = True
            else:
                if self.clear_since is None:
                    self.clear_since = now
                clear_age = (now - self.clear_since).to_sec()
                if clear_age < self.clear_confirm_s:
                    self.state = self.CONFIRM
                    stop = True
                else:
                    self.committed = True
                    self.state = self.COMMITTED
                    allowed = True
                    stop = False
                    rospy.logwarn("ROUNDABOUT MERGE COMMITTED: clear gap confirmed")

        self.stop_pub.publish(Bool(data=stop))
        self.allowed_pub.publish(Bool(data=allowed))
        payload = {
            "state": self.state,
            "requested": self.requested,
            "stop_required": stop,
            "allowed": allowed,
            "sensor_fresh": freshness,
            "clear_age_s": (
                None if self.clear_since is None else round((now - self.clear_since).to_sec(), 3)
            ),
            "blockers": blockers,
            "conflict_point_base_link": [self.conflict_x_m, self.conflict_y_m],
            "required_arrival_gap_s": self.required_arrival_gap_s,
        }
        self.status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )
        rospy.loginfo_throttle(
            1.0,
            "MergeGate state=%s request=%s stop=%s allowed=%s blockers=%d",
            self.state,
            self.requested,
            stop,
            allowed,
            len(blockers),
        )


def main() -> None:
    try:
        RoundaboutMergeGate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
