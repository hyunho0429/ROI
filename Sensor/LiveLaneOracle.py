#!/usr/bin/env python3
"""HD맵으로 실시간 차선/정지선 정보를 만들어 UDP 로 내보낸다 (임시 오라클).

`GenerateLabels.py`가 녹화본(meta.jsonl)에 대해 오프라인으로 하는 일을,
카메라·자차상태 UDP 를 직접 받아 **실시간으로** 똑같이 한다:

    카메라 JPEG UDP + 자차상태 UDP  (RecordDrive.py 의 워커·동기화 보정 그대로 재사용)
        --[GenerateLabels.render_frame_labels + assign_lane_ids]--> 좌/우/정지선 벡터
        --[BuildPolyTargets.fit_slot]--> 좌/우 각각 3차 다항식(픽셀 좌표)
        --[live_lane_oracle_protocol.pack_lane_oracle]--> UDP 로 송신

**HD맵은 실차에 없으므로 대회 주행에는 쓸 수 없다.** 나중에 학습된
PolyRegression 모델이 완성되면 이 스크립트를 끄고, 모델이 같은 와이어
포맷(픽셀 좌표 3차 다항식)을 직접 채우도록 갈아끼운다 — 그래서 출력 스키마를
`BuildPolyTargets.py`의 학습 타깃과 동일하게 맞췄다.

`src/detection/mgeo_lane_stub`(자차 좌표/미터 기준)와는 다른 도구다. 저건
카메라 없이 자차 상태만으로 돌아가는 더 가벼운 스텁이고, 이건 카메라 영상까지
받아 픽셀 좌표로 계산한다 — 실제 모델이 낼 형식에 더 가깝다.

사용법::

    python LiveLaneOracle.py --dest-ip 127.0.0.1 --dest-port 4022

기본으로 실시간 카메라 오버레이 창이 뜬다 (--no-overlay 로 끈다).
"""

import argparse
import math
import socket
import threading
import time

import cv2
import numpy as np

import BuildPolyTargets
import RecordDrive
from GenerateLabels import (
    assign_lane_ids, build_link_kdtrees, load_boundaries, load_camera,
    load_link_elevation, render_frame_labels, CLASS_COLORS, CLASS_STOPLINE,
)
from live_lane_oracle_protocol import CATEGORY_WHITE_DASHED, pack_lane_oracle

DEFAULT_DEST_IP = "127.0.0.1"
DEFAULT_DEST_PORT = 4022      # lane_stub_udp.py(4021)와 겹치지 않는 포트


# GenerateLabels.CLASS_* -> live_lane_oracle_protocol.CATEGORY_* (BuildPolyTargets
# 와 같은 3-way; 카테고리 없는 클래스(유도선 등)는 fit_slot 이 이미 -1 로 둔다)
def _class_color(cls):
    return CLASS_COLORS.get(cls, (200, 200, 200))


def build_arg_parser():
    ap = argparse.ArgumentParser(
        description="HD맵으로 실시간 픽셀 좌표 차선/정지선 오라클을 UDP 로 내보낸다")
    # GenerateLabels.py 와 같은 이유로 기본값을 안 둔다 — 저장소마다(car, ROI)
    # mgeo/cam_set.json 이 있는 경로 깊이가 달라서, 여기서 상대경로로 추측하면
    # 다른 저장소로 옮겼을 때 조용히 엉뚱한 경로를 가리키게 된다.
    ap.add_argument("--mgeo", required=True, help="MGeo 폴더 (lane_boundary_set.json 등)")
    ap.add_argument("--cam-set", required=True, help="cam_set.json 경로")
    ap.add_argument("--sensor-id", type=int, default=1, help="카메라 SensorUniqueID (1=전방)")
    ap.add_argument("--cam-ip", default=RecordDrive.DEFAULT_CAM_IP)
    ap.add_argument("--cam-port", type=int, default=RecordDrive.DEFAULT_CAM_PORT)
    ap.add_argument("--status-ip", default="0.0.0.0")
    ap.add_argument("--status-port", type=int, default=RecordDrive.DEFAULT_STATUS_PORT)
    ap.add_argument(
        "--ros-status-topic",
        default=None,
        help=("UDP를 다시 bind하지 않고 morai_msgs/EgoVehicleStatus ROS 토픽을 "
              "RecordDrive 보간 이력으로 사용한다"),
    )
    ap.add_argument(
        "--dashed-lane-topic",
        default="/perception/camera/dashed_lane_detected",
        help="좌측 ego 경계가 점선일 때 true를 발행할 std_msgs/Bool 토픽",
    )
    ap.add_argument("--dest-ip", default=DEFAULT_DEST_IP)
    ap.add_argument("--dest-port", type=int, default=DEFAULT_DEST_PORT)
    ap.add_argument("--no-udp", action="store_true",
                    help="LaneOracle 바이너리 UDP는 보내지 않고 ROS 점선 토픽만 발행한다")
    ap.add_argument("--no-overlay", action="store_true",
                    help="실시간 카메라 오버레이 창을 띄우지 않는다")
    return ap


def _draw_overlay(frame, cam, left_vec, right_vec, stop_vec):
    """계산에 쓴 것과 완전히 같은 points_uv 를 그대로 그린다 — 근사 없음."""
    out = frame.copy()
    for vec, thickness in ((left_vec, 2), (right_vec, 2)):
        if vec is None:
            continue
        pts = np.asarray(vec["points_uv"], dtype=np.int32)
        if len(pts) >= 2:
            cv2.polylines(out, [pts], False, _class_color(vec["cls"]), thickness)
    if stop_vec is not None:
        pts = np.asarray(stop_vec["points_uv"], dtype=np.int32)
        if len(pts) >= 2:
            cv2.polylines(out, [pts], False, _class_color(CLASS_STOPLINE), 3)
    label = []
    if left_vec is not None:
        label.append(f"L={left_vec['cls_name']}")
    if right_vec is not None:
        label.append(f"R={right_vec['cls_name']}")
    if stop_vec is not None:
        label.append(f"stop={stop_vec['ego_fwd_m']:.1f}m")
    cv2.rectangle(out, (0, 0), (out.shape[1], 24), (0, 0, 0), -1)
    cv2.putText(out, "  ".join(label) or "no lane", (8, 17),
               cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return out


def _ros_status_snapshot(message):
    """Convert the shared ROS Ego status into RecordDrive's interpolation row."""
    stamp = message.header.stamp.to_sec()
    if stamp <= 0.0:
        return None, None
    velocity = message.velocity
    snapshot = {
        "pos_x": float(message.position.x),
        "pos_y": float(message.position.y),
        "pos_z": float(message.position.z),
        "yaw": RecordDrive._wrap180(float(message.heading)),
        # Competition EgoVehicleStatus ROS compatibility output exposes heading
        # but not roll/pitch.  Zero keeps the original projection convention
        # deterministic; road elevation is still taken from MGeo links.
        "roll": 0.0,
        "pitch": 0.0,
        "status_sec": int(message.header.stamp.secs),
        "status_nsec": int(message.header.stamp.nsecs),
        "signed_vel": math.sqrt(
            float(velocity.x) ** 2
            + float(velocity.y) ** 2
            + float(velocity.z) ** 2
        ),
        "ang_vel_z": 0.0,
        "steer": float(message.wheel_angle),
        "accel": float(message.accel),
        "brake": float(message.brake),
        "gear": 0,
        "link_id": "",
    }
    return snapshot, stamp


def _left_fit_is_dashed(left_fit):
    """True only for a valid MGeo white-dashed ego-left boundary."""
    return bool(
        left_fit is not None
        and left_fit["conf"]
        and left_fit["broken"]
        and left_fit["category"] == CATEGORY_WHITE_DASHED
    )


def run(args):
    rospy = None
    dashed_publisher = None
    status_subscriber = None
    if args.ros_status_topic:
        import rospy as rospy_module
        from morai_msgs.msg import EgoVehicleStatus
        from std_msgs.msg import Bool

        rospy = rospy_module
        rospy.init_node("live_lane_oracle", anonymous=False)
        dashed_publisher = rospy.Publisher(
            args.dashed_lane_topic, Bool, queue_size=1
        )

        def status_callback(message):
            snapshot, timestamp = _ros_status_snapshot(message)
            if snapshot is None:
                rospy.logwarn_throttle(
                    2.0,
                    "LiveLaneOracle is waiting for packet-time Ego status stamps",
                )
                return
            RecordDrive.push_status_snapshot(snapshot, timestamp)

        status_subscriber = rospy.Subscriber(
            args.ros_status_topic,
            EgoVehicleStatus,
            status_callback,
            queue_size=100,
        )
        rospy.logwarn(
            "HD-map lane oracle: status=%s dashed_output=%s",
            args.ros_status_topic,
            args.dashed_lane_topic,
        )

    print("[oracle] 지도 로딩 중...")
    boundaries = load_boundaries(args.mgeo)
    links = load_link_elevation(args.mgeo)
    link_trees = build_link_kdtrees(links)
    fallback_z = float(np.median([b["points"][:, 2].mean() for b in boundaries]))
    cam = load_camera(args.cam_set, args.sensor_id, "horizontal")
    print(f"[oracle] 경계 {len(boundaries)}개, 카메라 {cam.native_width}x{cam.native_height} "
          f"fx={cam.fx:.1f}")

    # 카메라·상태 UDP 수신과 동기화 보정(카메라 파이프라인 지연 0.090s 포함)은
    # RecordDrive.py 것을 그대로 쓴다 — 오늘 실측으로 확정한 그 로직을 다시
    # 만들면 미묘하게 달라질 위험이 있다.
    threading.Thread(target=RecordDrive.camera_worker,
                     args=(args.cam_ip, args.cam_port), daemon=True).start()
    if args.ros_status_topic:
        print(f"[oracle] 카메라 {args.cam_ip}:{args.cam_port}  상태 ROS {args.ros_status_topic}")
    else:
        threading.Thread(target=RecordDrive.status_worker,
                         args=(args.status_ip, args.status_port), daemon=True).start()
        print(f"[oracle] 카메라 {args.cam_ip}:{args.cam_port}  "
              f"상태 {args.status_ip}:{args.status_port}")

    send_sock = None if args.no_udp else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.dest_ip, args.dest_port)
    if send_sock is not None:
        print(f"[oracle] 차로 오라클 송신: {dest[0]}:{dest[1]}")
    if not args.no_overlay:
        print("[oracle] 카메라 오버레이 표시 중 (ESC 로 종료)")

    last_key = None
    n_sent = n_left = n_right = n_stop = 0
    last_log = time.time()
    try:
        while rospy is None or not rospy.is_shutdown():
            with RecordDrive._frame_lock:
                frame = None if RecordDrive._latest_frame is None else RecordDrive._latest_frame.copy()
                key = RecordDrive._latest_frame_key
            if frame is None or key == last_key:
                time.sleep(0.005)
                continue
            last_key = key

            if frame.shape[1::-1] != (cam.width, cam.height):
                cam = cam.scaled(frame.shape[1], frame.shape[0])

            t_cam = key[0] + key[1] * 1e-9
            row, _pose_dt = RecordDrive.interp_status(t_cam - RecordDrive.CAMERA_PIPELINE_LATENCY)
            if row is None:
                time.sleep(0.005)
                continue

            vectors = []
            render_frame_labels(cam, boundaries, link_trees, row, fallback_z,
                                crosswalks=(), bonnet=None, frame=frame, vectors=vectors)
            assign_lane_ids(vectors)

            left_vec = next((v for v in vectors if v.get("lane_index") == -1), None)
            right_vec = next((v for v in vectors if v.get("lane_index") == 1), None)
            stops = sorted((v for v in vectors if str(v.get("lane_id", "")).startswith("stopline_")),
                          key=lambda v: v["ego_fwd_m"])
            stop_vec = stops[0] if stops else None

            left_fit = BuildPolyTargets.fit_slot(left_vec, cam.width, cam.height) if left_vec else None
            right_fit = BuildPolyTargets.fit_slot(right_vec, cam.width, cam.height) if right_vec else None
            left_pack = None if left_fit is None else {
                **left_fit,
                "lower_v": left_fit["lower_y"],
                "upper_v": left_fit["upper_y"],
                "points": left_vec["points_uv"],
            }
            right_pack = None if right_fit is None else {
                **right_fit,
                "lower_v": right_fit["lower_y"],
                "upper_v": right_fit["upper_y"],
                "points": right_vec["points_uv"],
            }

            n_left += left_pack is not None
            n_right += right_pack is not None
            n_stop += stop_vec is not None

            sec = int(t_cam)
            nsec = int(round((t_cam - sec) * 1e9))
            if send_sock is not None:
                out = pack_lane_oracle(
                    sec, nsec, left=left_pack, right=right_pack,
                    stopline_valid=stop_vec is not None,
                    stopline_distance_m=(
                        stop_vec["ego_fwd_m"] if stop_vec is not None else 0.0
                    ),
                )
                send_sock.sendto(out, dest)
            if dashed_publisher is not None:
                left_dashed = _left_fit_is_dashed(left_fit)
                dashed_publisher.publish(Bool(data=left_dashed))
                if left_dashed:
                    rospy.loginfo_throttle(
                        1.0,
                        "HD-map oracle detected the LEFT ego-lane boundary as dashed",
                    )
            n_sent += 1

            now = time.time()
            if now - last_log >= 2.0:
                print(f"[oracle] 송신 {n_sent}  L {n_left/n_sent*100:.0f}%  "
                      f"R {n_right/n_sent*100:.0f}%  stop {n_stop/n_sent*100:.0f}%")
                last_log = now

            if not args.no_overlay:
                shown = _draw_overlay(frame, cam, left_vec, right_vec, stop_vec)
                cv2.imshow("LiveLaneOracle (ESC to quit)", shown)
                if (cv2.waitKey(1) & 0xFF) == 27:
                    print("\n[oracle] ESC 로 종료.")
                    break
    except KeyboardInterrupt:
        print("\n[oracle] 종료.")
    finally:
        if dashed_publisher is not None:
            try:
                dashed_publisher.publish(Bool(data=False))
            except rospy.ROSException:
                pass
        if send_sock is not None:
            send_sock.close()
        cv2.destroyAllWindows()


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    run(args)


if __name__ == "__main__":
    main()
