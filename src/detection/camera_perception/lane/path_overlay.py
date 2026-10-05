"""Draw planner paths (nav_msgs/Path, map frame) on the lane camera window.

The camera model projects *ego* points: origin at the rear-axle centre, x
forward, y left. /localization/odometry reports the same origin, so a map
point is moved into the ego pose at the frame's capture time and placed on
the road plane (z = ROAD_Z_EGO) before projection.

ROS is imported lazily by PathOverlay so the geometry helpers stay testable
without a ROS installation.
"""

import json
import math
import threading
import time
from collections import deque

import cv2
import numpy as np

PATH_COLOR = (255, 0, 255)          # magenta: planner output
REF_COLOR = (170, 170, 170)         # grey: reference (base/global) path
STALE_COLOR = (90, 90, 90)
STATE_COLORS = {                    # committed manoeuvres stand out
    "LANE_CHANGE": (0, 165, 255),   # orange
    "REJOIN": (255, 255, 0),        # cyan
}
MIN_X_M = 2.5       # the camera sits at x=1.9 m; nearer points leave the image
MAX_X_M = 80.0
STALE_S = 1.0
NODE_MIN_PX = 8.0   # screen spacing between drawn path points
HUD_Y = 66          # below lane_viz (0-46) and live_overlay.draw_values (46-66)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def map_to_ego(points_xy, pose):
    """Map-frame (N, 2) points -> ego frame of ``pose`` = (x, y, yaw)."""
    pts = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    x0, y0, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = pts[:, 0] - x0, pts[:, 1] - y0
    return np.column_stack([c * dx + s * dy, -s * dx + c * dy])


class PoseHistory:
    """Wall-clock stamped odometry poses, looked up by camera receive time."""

    def __init__(self, maxlen=400):
        self._samples = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def add(self, stamp, pose):
        with self._lock:
            self._samples.append((float(stamp), tuple(pose)))

    def at(self, stamp=None, tolerance=0.25):
        """Pose nearest to ``stamp``; the latest pose when stamp is None."""
        with self._lock:
            samples = list(self._samples)
        if not samples:
            return None
        if stamp is None:
            return samples[-1][1]
        t, pose = min(samples, key=lambda sample: abs(sample[0] - stamp))
        return pose if abs(t - stamp) <= tolerance else None


class PathOverlay:
    def __init__(self, rospy, path_topic, odom_topic,
                 ref_path_topic="", state_topic=""):
        from nav_msgs.msg import Odometry, Path
        from std_msgs.msg import String

        self.path_topic = path_topic
        self.poses = PoseHistory()
        self._lock = threading.Lock()
        self._paths = {}            # topic -> (receive wall time, (N, 2) array)
        self._state = None
        rospy.Subscriber(odom_topic, Odometry, self._odom_cb, queue_size=10)
        rospy.Subscriber(path_topic, Path, self._path_cb, path_topic,
                         queue_size=1)
        self.ref_path_topic = ref_path_topic
        if ref_path_topic:
            rospy.Subscriber(ref_path_topic, Path, self._path_cb,
                             ref_path_topic, queue_size=1)
        if state_topic:
            rospy.Subscriber(state_topic, String, self._state_cb, queue_size=1)

    def _odom_cb(self, msg):
        pose = msg.pose.pose
        self.poses.add(time.time(), (pose.position.x, pose.position.y,
                                     yaw_from_quaternion(pose.orientation)))

    def _path_cb(self, msg, topic):
        pts = np.array([(p.pose.position.x, p.pose.position.y)
                        for p in msg.poses], dtype=float).reshape(-1, 2)
        with self._lock:
            self._paths[topic] = (time.time(), pts)

    def _state_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._state = data if isinstance(data, dict) else None

    def draw(self, vis, cam, road_z, frame_stamp=None):
        """Draw onto ``vis`` (the cropped frame that ``cam`` projects into)."""
        now = time.time()
        with self._lock:
            paths = dict(self._paths)
            state = dict(self._state) if self._state else {}
        pose = self.poses.at(frame_stamp)
        hud = []
        if pose is None:
            hud.append("path: no odometry at frame time")
        else:
            if self.ref_path_topic in paths:
                _, ref = paths[self.ref_path_topic]
                self._draw_path(vis, cam, road_z, ref, pose, REF_COLOR, 1, False)
            if self.path_topic in paths:
                received, pts = paths[self.path_topic]
                age = now - received
                color = (STALE_COLOR if age > STALE_S else
                         STATE_COLORS.get(state.get("state"), PATH_COLOR))
                shown = self._draw_path(vis, cam, road_z, pts, pose, color, 2, True)
                hud.append(f"path {len(pts)}pts ({shown} in view) age {age:.1f}s"
                           + (" STALE" if age > STALE_S else ""))
            else:
                hud.append(f"path: waiting for {self.path_topic}")
        if state:
            hud.append(f"{state.get('state')} {state.get('reason')} "
                       f"v*={state.get('target_speed_mps')} "
                       f"stop={state.get('stop')} n={state.get('lane_changes_done')}")
        cv2.rectangle(vis, (0, HUD_Y), (vis.shape[1], HUD_Y + 20), (0, 0, 0), -1)
        cv2.putText(vis, "  |  ".join(hud), (8, HUD_Y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

    @staticmethod
    def _draw_path(vis, cam, road_z, pts, pose, color, thickness, nodes):
        if len(pts) == 0:
            return 0
        ego = map_to_ego(pts, pose)
        idx = np.flatnonzero((ego[:, 0] >= MIN_X_M) & (ego[:, 0] <= MAX_X_M))
        ego = ego[idx]
        if len(ego) == 0:
            return 0
        uv, valid = cam.project(np.column_stack([ego, np.full(len(ego), road_z)]))
        uv, ego, idx = uv[valid], ego[valid], idx[valid]
        h, w = vis.shape[:2]
        inb = ((uv[:, 0] > -2000) & (uv[:, 0] < w + 2000)
               & (uv[:, 1] > -2000) & (uv[:, 1] < h + 2000))
        uv, ego, idx = uv[inb].astype(np.int32), ego[inb], idx[inb]
        if len(uv) >= 2:
            cv2.polylines(vis, [uv], False, color, thickness, cv2.LINE_AA)
        if nodes:
            last = None
            for (u, v), x in zip(uv, ego[:, 0]):
                # Far points crowd together on screen; skip those that would
                # overlap the previous dot and shrink the rest with distance.
                if last is not None and math.hypot(u - last[0], v - last[1]) < NODE_MIN_PX:
                    continue
                cv2.circle(vis, (int(u), int(v)), max(2, int(6 - x / 12.0)),
                           color, -1, cv2.LINE_AA)
                last = (u, v)
            if len(idx) and idx[-1] == len(pts) - 1:
                # The path's last point is visible: Pure Pursuit stops here.
                u, v = uv[-1]
                cv2.drawMarker(vis, (int(u), int(v)), (0, 0, 255),
                               cv2.MARKER_TILTED_CROSS, 18, 2)
        return len(uv)
