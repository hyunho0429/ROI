#!/usr/bin/env python3
"""
실시간 차선 검출 결과를 주행팀용으로 안정화하여 ROS1 토픽으로 출력한다.

기존 lane_detection.py는 수정하지 않는다.

역할
----
1) LaneDetector 결과 수신
2) 좌/우 차선 상태 및 유효성 검사
3) 프레임 간 이상치 제거
4) EMA smoothing
5) 차량 기준(base_link) 좌/우 경계점 생성
6) 차량 기준 중심선 waypoint 생성
7) 곡률 / 횡오차 / 방위오차 계산
8) 좌/우 차선 종류 및 정지선 거리 출력
9) 신뢰도와 상태를 함께 ROS topic으로 publish

출력 토픽
---------
/perception/camera/lane_info   std_msgs/String(JSON)

좌표계
-----
base_link / FLU
+x : 차량 전방
+y : 차량 좌측
-y : 차량 우측
단위: meter

예:
centerline_points = [[5.0, 0.02], [6.0, 0.05], ...]
-> 차량 기준 5m 앞에서 중심선이 좌측 0.02m에 위치
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np

try:
    import rospy
    from std_msgs.msg import String
except ImportError as e:
    raise SystemExit(
        "ROS1 rospy/std_msgs를 찾지 못했습니다. "
        "ROS 환경을 source한 뒤 실행하세요."
    ) from e

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lane_detection import (
    LaneDetector,
    default_checkpoint,
    EVAL_X_NEAR,
    EVAL_X_FAR,
    LANE_WIDTH_M,
)
from morai_camera import DEFAULT_IP, DEFAULT_PORT, CameraStream


# ==========================================================================
# 주행팀 출력 / 안정화 파라미터
# ==========================================================================

OUTPUT_TOPIC = "/perception/camera/lane_info"
FRAME_ID = "base_link"

# Waypoint 생성 범위
WAYPOINT_X_MIN_M = 5.0
WAYPOINT_X_MAX_M = 25.0
WAYPOINT_STEP_M = 1.0

# 중심 waypoint 후처리
WAYPOINT_MAX_DY_M = 0.25
WAYPOINT_OUTLIER_M = 0.20
MIN_CENTER_WAYPOINTS = 3

# 차로 폭 유효범위
LANE_WIDTH_MIN_M = 2.7
LANE_WIDTH_MAX_M = 4.2
LANE_WIDTH_CHANGE_MAX_M = 0.8

# 차선이 최소 이 정도 길이는 관측되어야 정상으로 본다.
MIN_USABLE_SPAN_M = 5.0

# 프레임 간 최대 변화율
MAX_LATERAL_RATE_MPS = 2.0
MAX_HEADING_RATE_RADPS = 1.2
MAX_CURVATURE_RATE_1PMPS = 0.08

# 순간 미검출/이상치 시 이전 안정값 유지 시간
HOLD_SEC = 0.30

# EMA smoothing
EMA_ALPHA = 0.25

# 직선 판정
STRAIGHT_KAPPA_THRESH = 1.0 / 1000.0

# 최소 신뢰도
MIN_CONFIDENCE_FOR_VALID = 0.45


def _finite(v):
    return v is not None and math.isfinite(float(v))


def _lane_span(lane):
    if lane is None:
        return 0.0
    return max(0.0, float(lane.x_range[1] - lane.x_range[0]))


def _lane_y(lane, x, extrap=1.0):
    """
    lane이 실제로 관측된 x 범위 근처에서만 값을 사용한다.
    과도한 외삽 방지.
    """
    if lane is None:
        return None
    lo, hi = lane.x_range
    if x < lo - extrap or x > hi + extrap:
        return None
    return float(np.polyval(lane.coef, x))


def _lane_meta(lane):
    if lane is None:
        return {
            "detected": False,
            "type": None,
            "dashed": None,
            "track_id": None,
            "age": 0,
            "coef": None,
            "x_range_m": None,
            "n_points": 0,
        }

    return {
        "detected": True,
        "type": lane.name,
        "dashed": bool(lane.is_dashed),
        "track_id": lane.track_id,
        "age": int(lane.age),
        "coef": [round(float(v), 8) for v in lane.coef],
        "x_range_m": [
            round(float(lane.x_range[0]), 3),
            round(float(lane.x_range[1]), 3),
        ],
        "n_points": int(lane.n_points),
    }


def center_coef_from_result(res):
    """
    중심선 2차식 coef 생성.

    BOTH:
        좌/우 차선의 평균

    LEFT_ONLY:
        왼쪽 차선을 차로폭 절반만큼 우측으로 이동

    RIGHT_ONLY:
        오른쪽 차선을 차로폭 절반만큼 좌측으로 이동
    """
    l, r = res.ego_left, res.ego_right

    if l is not None and r is not None:
        return (
            (np.asarray(l.coef, dtype=float) +
             np.asarray(r.coef, dtype=float)) / 2.0,
            "BOTH",
        )

    if l is not None:
        c = np.asarray(l.coef, dtype=float).copy()
        c[-1] -= LANE_WIDTH_M / 2.0
        return c, "LEFT_ONLY"

    if r is not None:
        c = np.asarray(r.coef, dtype=float).copy()
        c[-1] += LANE_WIDTH_M / 2.0
        return c, "RIGHT_ONLY"

    return None, "NONE"


def signed_curvature_from_coef(coef, x=10.0):
    """
    y=f(x)의 signed curvature [1/m]

    + : 좌회전 방향
    - : 우회전 방향
    """
    if coef is None:
        return None

    coef = np.asarray(coef, dtype=float)
    d1 = np.polyder(coef, 1)
    d2 = np.polyder(coef, 2)

    yp = float(np.polyval(d1, x))
    ypp = float(np.polyval(d2, x))

    denom = (1.0 + yp * yp) ** 1.5
    if denom <= 1e-12:
        return None

    return ypp / denom


def curvature_output(kappa):
    if not _finite(kappa):
        return None, "UNKNOWN"

    k = float(kappa)

    if abs(k) < STRAIGHT_KAPPA_THRESH:
        return None, "STRAIGHT"

    radius = 1.0 / abs(k)
    direction = "LEFT" if k > 0.0 else "RIGHT"
    return radius, direction


class EMA:
    def __init__(self, alpha=EMA_ALPHA):
        self.alpha = float(alpha)
        self.value = None

    def update(self, value):
        if not _finite(value):
            return self.value

        value = float(value)
        if self.value is None:
            self.value = value
        else:
            self.value = (
                self.alpha * value +
                (1.0 - self.alpha) * self.value
            )
        return self.value



def filter_centerline_points(points):
    """centerline waypoint의 국소 이상치를 제거/보간한다."""
    if len(points) < 3:
        return points, []

    pts = [[float(x), float(y)] for x, y in points]
    reasons = []

    # 시작/끝 단독 튐 제거
    while len(pts) >= 3 and abs(pts[0][1] - pts[1][1]) > WAYPOINT_MAX_DY_M:
        reasons.append(
            f"DROP_FIRST_CENTER_OUTLIER_x={pts[0][0]:.1f}_dy="
            f"{abs(pts[0][1] - pts[1][1]):.3f}"
        )
        pts.pop(0)

    while len(pts) >= 3 and abs(pts[-1][1] - pts[-2][1]) > WAYPOINT_MAX_DY_M:
        reasons.append(
            f"DROP_LAST_CENTER_OUTLIER_x={pts[-1][0]:.1f}_dy="
            f"{abs(pts[-1][1] - pts[-2][1]):.3f}"
        )
        pts.pop()

    if len(pts) < 3:
        return [[round(x, 3), round(y, 3)] for x, y in pts], reasons

    # 내부 단독 튐은 앞/뒤 점으로 선형보간
    filtered = [pts[0]]
    for i in range(1, len(pts) - 1):
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        x2, y2 = pts[i + 1]

        if abs(x2 - x0) < 1e-9:
            y_expected = (y0 + y2) / 2.0
        else:
            ratio = (x1 - x0) / (x2 - x0)
            y_expected = y0 + ratio * (y2 - y0)

        err = abs(y1 - y_expected)

        if err > WAYPOINT_OUTLIER_M:
            reasons.append(
                f"INTERPOLATE_CENTER_OUTLIER_x={x1:.1f}_err={err:.3f}"
            )
            filtered.append([x1, y_expected])
        else:
            filtered.append([x1, y1])

    filtered.append(pts[-1])

    # 보간 후에도 연속 점프가 크면 해당 점 제거
    final = [filtered[0]]
    for p in filtered[1:]:
        dy = abs(p[1] - final[-1][1])
        if dy > WAYPOINT_MAX_DY_M:
            reasons.append(
                f"DROP_CENTER_JUMP_x={p[0]:.1f}_dy={dy:.3f}"
            )
            continue
        final.append(p)

    return [[round(x, 3), round(y, 3)] for x, y in final], reasons


class LaneOutputStabilizer:
    def __init__(self):
        self.lat_filter = EMA()
        self.heading_filter = EMA()
        self.kappa_filter = EMA()

        self.prev_raw = None
        self.last_good = None
        self.last_good_t = None

    # ----------------------------------------------------------------------
    # 기하학적 유효성 검사
    # ----------------------------------------------------------------------
    def _geometry_check(self, res):
        reasons = []
        score = 1.0

        l, r = res.ego_left, res.ego_right

        if l is None and r is None:
            return False, 0.0, ["NO_EGO_LANES"], None

        spans = [_lane_span(x) for x in (l, r) if x is not None]
        if spans and min(spans) < MIN_USABLE_SPAN_M:
            reasons.append("SHORT_LANE_SPAN")
            score *= 0.55

        lane_width = None

        if l is not None and r is not None:
            widths = []

            for x in (EVAL_X_NEAR, EVAL_X_FAR):
                ly = _lane_y(l, x, extrap=3.0)
                ry = _lane_y(r, x, extrap=3.0)

                if _finite(ly) and _finite(ry):
                    widths.append(float(ly - ry))

            if not widths:
                reasons.append("NO_COMMON_EVAL_RANGE")
                return False, 0.0, reasons, None

            lane_width = float(np.mean(widths))

            if any(w <= 0.0 for w in widths):
                reasons.append("LANES_CROSSED")
                return False, 0.0, reasons, lane_width

            if not (LANE_WIDTH_MIN_M <= lane_width <= LANE_WIDTH_MAX_M):
                reasons.append("BAD_LANE_WIDTH")
                score *= 0.20

            if (
                len(widths) >= 2 and
                abs(widths[-1] - widths[0]) > LANE_WIDTH_CHANGE_MAX_M
            ):
                reasons.append("WIDTH_CHANGES_TOO_FAST")
                score *= 0.40

        else:
            reasons.append("SINGLE_SIDE_ESTIMATE")
            score *= 0.65

        return score >= 0.35, score, reasons, lane_width

    # ----------------------------------------------------------------------
    # 프레임 간 순간 점프 검사
    # ----------------------------------------------------------------------
    def _temporal_check(self, lat, heading, kappa, now):
        reasons = []
        score = 1.0

        if self.prev_raw is None:
            self.prev_raw = {
                "t": now,
                "lat": lat,
                "heading": heading,
                "kappa": kappa,
            }
            return True, score, reasons

        dt = max(now - self.prev_raw["t"], 1e-3)

        def rate_bad(cur, prev, limit):
            if not (_finite(cur) and _finite(prev)):
                return False
            return abs(float(cur) - float(prev)) / dt > limit

        if rate_bad(lat, self.prev_raw["lat"], MAX_LATERAL_RATE_MPS):
            reasons.append("LATERAL_JUMP")
            score *= 0.15

        if rate_bad(heading, self.prev_raw["heading"], MAX_HEADING_RATE_RADPS):
            reasons.append("HEADING_JUMP")
            score *= 0.25

        if rate_bad(kappa, self.prev_raw["kappa"], MAX_CURVATURE_RATE_1PMPS):
            reasons.append("CURVATURE_JUMP")
            score *= 0.35

        self.prev_raw = {
            "t": now,
            "lat": lat,
            "heading": heading,
            "kappa": kappa,
        }

        return score >= 0.35, score, reasons

    def _age_score(self, res):
        ages = [
            lane.age
            for lane in (res.ego_left, res.ego_right)
            if lane is not None and lane.age is not None
        ]

        if not ages:
            return 0.0

        age = min(ages)
        return min(1.0, 0.55 + 0.15 * max(0, age - 1))

    # ----------------------------------------------------------------------
    # 실제 주행용 좌/우 경계 + 중심 waypoint 생성
    # ----------------------------------------------------------------------
    def _build_waypoints(self, res, lane_state):
        left = res.ego_left
        right = res.ego_right

        left_points = []
        right_points = []
        center_points = []

        xs = np.arange(
            WAYPOINT_X_MIN_M,
            WAYPOINT_X_MAX_M + 1e-6,
            WAYPOINT_STEP_M
        )

        for x in xs:
            ly = _lane_y(left, x, extrap=1.0)
            ry = _lane_y(right, x, extrap=1.0)

            # 실제 검출된 좌측 경계
            if _finite(ly):
                left_points.append([
                    round(float(x), 3),
                    round(float(ly), 3),
                ])

            # 실제 검출된 우측 경계
            if _finite(ry):
                right_points.append([
                    round(float(x), 3),
                    round(float(ry), 3),
                ])

            # 중심선
            cy = None

            if _finite(ly) and _finite(ry):
                cy = (float(ly) + float(ry)) / 2.0

            elif _finite(ly):
                cy = float(ly) - LANE_WIDTH_M / 2.0

            elif _finite(ry):
                cy = float(ry) + LANE_WIDTH_M / 2.0

            if cy is not None:
                center_points.append([
                    round(float(x), 3),
                    round(float(cy), 3),
                ])

        center_points, center_filter_reasons = filter_centerline_points(center_points)
        return left_points, center_points, right_points, center_filter_reasons

    def update(self, res, now=None):
        now = time.time() if now is None else float(now)

        center_coef, lane_state = center_coef_from_result(res)

        lat = res.lateral_error()
        heading = res.heading_error()
        kappa = signed_curvature_from_coef(center_coef, x=10.0)

        geom_ok, geom_score, geom_reasons, lane_width = \
            self._geometry_check(res)

        temp_ok, temp_score, temp_reasons = \
            self._temporal_check(lat, heading, kappa, now)

        age_score = self._age_score(res)

        confidence = geom_score * temp_score
        confidence *= (0.7 + 0.3 * age_score)
        confidence = max(0.0, min(1.0, confidence))

        reasons = geom_reasons + temp_reasons

        fresh_valid = (
            lane_state != "NONE"
            and geom_ok
            and temp_ok
            and confidence >= MIN_CONFIDENCE_FOR_VALID
            and _finite(lat)
            and _finite(heading)
        )

        # ==================================================================
        # 정상 fresh 값
        # ==================================================================
        if fresh_valid:
            lat_s = self.lat_filter.update(lat)
            heading_s = self.heading_filter.update(heading)
            kappa_s = self.kappa_filter.update(kappa)

            radius, direction = curvature_output(kappa_s)

            left_points, center_points, right_points, center_filter_reasons = \
                self._build_waypoints(res, lane_state)

            reasons.extend(center_filter_reasons)

            # 중심 waypoint가 너무 적으면 제어용으로 인정하지 않는다.
            if len(center_points) < MIN_CENTER_WAYPOINTS:
                fresh_valid = False
                reasons.append("TOO_FEW_CENTER_WAYPOINTS")
            else:
                out = {
                    "timestamp": now,

                    # 좌표계
                    "frame_id": FRAME_ID,
                    "coordinate_convention": {
                        "x": "forward_m",
                        "y": "left_m",
                    },

                    # 신뢰도/상태
                    "lane_valid": True,
                    "output_status": "FRESH",
                    "lane_state": lane_state,
                    "center_source": lane_state,
                    "confidence": round(confidence, 3),
                    "reasons": reasons,

                    # 좌측 차선
                    "left_lane": _lane_meta(res.ego_left),

                    # 우측 차선
                    "right_lane": _lane_meta(res.ego_right),

                    # 차로 폭
                    "lane_width_m": (
                        None if lane_width is None
                        else round(float(lane_width), 3)
                    ),

                    # 실제 주행용 좌표
                    "left_boundary_points": left_points,
                    "centerline_points": center_points,
                    "right_boundary_points": right_points,

                    # 제어용 파생값
                    "lateral_error_m": round(float(lat_s), 3),
                    "heading_error_rad": round(float(heading_s), 4),

                    "curvature_1pm": (
                        None if kappa_s is None
                        else round(float(kappa_s), 6)
                    ),
                    "curvature_radius_m": (
                        None if radius is None
                        else round(float(radius), 2)
                    ),
                    "curve_direction": direction,

                    # 정지선
                    "stopline_detected": res.stopline_dist is not None,
                    "stopline_distance_m": (
                        None if res.stopline_dist is None
                        else round(float(res.stopline_dist), 3)
                    ),

                    # 디버깅/성능
                    "n_lanes": int(len(res.lanes)),
                    "infer_ms": round(float(res.infer_ms), 1),
                    "post_ms": round(float(res.post_ms), 1),
                }

                self.last_good = dict(out)
                self.last_good_t = now
                return out

        # ==================================================================
        # 순간 미검출 / 순간 이상치: 이전 정상값 잠시 유지
        # ==================================================================
        if (
            self.last_good is not None
            and self.last_good_t is not None
            and now - self.last_good_t <= HOLD_SEC
        ):
            out = dict(self.last_good)

            out.update({
                "timestamp": now,
                "lane_valid": True,
                "output_status": "HELD",
                "raw_lane_state": lane_state,
                "confidence": round(
                    max(0.0, float(self.last_good["confidence"]) * 0.75),
                    3,
                ),
                "reasons": reasons + ["USING_LAST_GOOD"],

                # 정지선은 현재 프레임 결과를 사용
                "stopline_detected": res.stopline_dist is not None,
                "stopline_distance_m": (
                    None if res.stopline_dist is None
                    else round(float(res.stopline_dist), 3)
                ),

                "n_lanes": int(len(res.lanes)),
                "infer_ms": round(float(res.infer_ms), 1),
                "post_ms": round(float(res.post_ms), 1),
            })

            return out

        # ==================================================================
        # 충분히 오래 차선이 없거나 계속 비정상: INVALID
        # ==================================================================
        return {
            "timestamp": now,

            "frame_id": FRAME_ID,
            "coordinate_convention": {
                "x": "forward_m",
                "y": "left_m",
            },

            "lane_valid": False,
            "output_status": "INVALID",
            "lane_state": lane_state,
            "center_source": lane_state,
            "confidence": round(confidence, 3),
            "reasons": reasons or ["NO_RELIABLE_LANE"],

            "left_lane": _lane_meta(res.ego_left),
            "right_lane": _lane_meta(res.ego_right),

            "lane_width_m": (
                None if lane_width is None
                else round(float(lane_width), 3)
            ),

            # invalid일 때는 주행팀이 절대 따라가지 않도록 비운다.
            "left_boundary_points": [],
            "centerline_points": [],
            "right_boundary_points": [],

            "lateral_error_m": None,
            "heading_error_rad": None,
            "curvature_1pm": None,
            "curvature_radius_m": None,
            "curve_direction": "UNKNOWN",

            "stopline_detected": res.stopline_dist is not None,
            "stopline_distance_m": (
                None if res.stopline_dist is None
                else round(float(res.stopline_dist), 3)
            ),

            "n_lanes": int(len(res.lanes)),
            "infer_ms": round(float(res.infer_ms), 1),
            "post_ms": round(float(res.post_ms), 1),
        }


def build_arg_parser():
    ap = argparse.ArgumentParser(
        description="실시간 카메라 차선 -> 안정화 -> local waypoint -> ROS"
    )

    ap.add_argument("--checkpoint", default=default_checkpoint())
    ap.add_argument("--cam-set", default=None)
    ap.add_argument("--bonnet", default=None)
    ap.add_argument("--no-bonnet", action="store_true")
    ap.add_argument("--no-track", action="store_true")

    ap.add_argument("--ip", default=DEFAULT_IP)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)

    ap.add_argument("--device", default=None, help="cuda / cpu")
    ap.add_argument("--every", type=int, default=1)

    ap.add_argument(
        "--topic",
        default=OUTPUT_TOPIC,
        help="ROS 출력 토픽"
    )

    return ap


def main(argv=None):
    args = build_arg_parser().parse_args(argv)

    rospy.init_node("camera_lane_info_publisher", anonymous=False)

    pub = rospy.Publisher(
        args.topic,
        String,
        queue_size=1
    )

    pipe = LaneDetector(
        args.checkpoint,
        cam_set=args.cam_set,
        bonnet_mask=False if args.no_bonnet else args.bonnet,
        device=args.device,
        track=not args.no_track,
    )

    stabilizer = LaneOutputStabilizer()

    rospy.loginfo(
        "[camera_lane_info] epoch=%s backbone=%s device=%s",
        pipe.ckpt_info.get("epoch"),
        pipe.ckpt_info.get("backbone"),
        pipe.device,
    )

    rospy.loginfo(
        "[camera_lane_info] output topic=%s frame=%s",
        args.topic,
        FRAME_ID,
    )

    cam = CameraStream(args.ip, args.port).start()

    rospy.loginfo(
        "[camera_lane_info] camera %s:%s waiting...",
        args.ip,
        args.port,
    )

    if not cam.wait_first(timeout=15.0):
        cam.stop()
        raise SystemExit(
            "카메라 프레임이 안 옵니다. "
            "시뮬레이터와 IP/포트를 확인하세요."
        )

    rospy.loginfo("[camera_lane_info] camera stream started")

    last_seq = -1
    n_since = 0

    try:
        while not rospy.is_shutdown():
            frame, seq = cam.latest()

            if frame is None or seq == last_seq:
                time.sleep(0.001)
                continue

            last_seq = seq
            n_since += 1

            if n_since < max(1, args.every):
                continue

            n_since = 0

            res = pipe.run(frame)
            out = stabilizer.update(res)

            msg = String()
            msg.data = json.dumps(
                out,
                ensure_ascii=False,
                separators=(",", ":"),
            )

            pub.publish(msg)

    except KeyboardInterrupt:
        pass

    finally:
        cam.stop()
        rospy.loginfo("[camera_lane_info] stopped")


if __name__ == "__main__":
    main()
