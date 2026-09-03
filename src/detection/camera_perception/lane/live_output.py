#!/usr/bin/env python3
"""[3/3] 시뮬레이터 카메라를 실시간으로 받아 **출력값만** 낸다 (그림 없음).

===========================================================================
실행 (그대로 복사해서 붙여넣기)
===========================================================================
cd ~/morai_ws/src/ROI/src/detection/camera_perception/lane
C:/Users/user/anaconda3/envs/vision_env/python.exe live_output.py --udp 127.0.0.1:7600

  화면으로만 보기    --udp 를 지운다 (표준출력에 JSON 한 줄씩)
  전송만 하기        --quiet 추가
  점열까지 보내기    --points 추가
  GPU 로            --device cuda
===========================================================================

**오버레이도 저장도 하지 않는다.** 실주행에서 제어에 넘길 값만 뽑는 경로다.
그림이 필요하면 live_overlay.py, 저장이 필요하면 offline_test.py 를 쓴다.

후처리는 lane_pipeline.LanePipeline.run() 하나만 쓴다. 세 스크립트가 같은
코드를 부르므로 **집에서 본 결과와 실주행 결과가 갈릴 일이 없다.**

--------------------------------------------------------------------------
출력 규격은 아직 미정이다
--------------------------------------------------------------------------
지금은 LaneResult.as_dict() 를 통째로 보낸다. 제어팀과 정하고 나면
lane_pipeline.LaneResult.as_dict() 한 곳만 줄이면 세 스크립트에 다 반영된다.

들어 있는 것:
    left / right        자차 좌우 차선 경계, 자차 좌표계 점열 [[x, y], ...]
                        (x 전방, y 좌측, 미터). **경계 두 줄을 그대로 주는 것은
                        장애물 회피 때문이다** - 중심선만 주면 "어디까지 비켜도
                        되는지"를 표현할 수 없다.
    left_type/right_type  white_solid / white_dashed / yellow (차선변경 가부 판단)
    lateral_error       차로 중심 기준 횡오차 (m). 음수 = 왼쪽으로 치우침
    heading_error       차로 방향 대비 방위 오차 (rad)
    stopline_dist       자차 앞 정지선까지 (m), 없으면 null
"""

import argparse
import json
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# **lane_viz 를 import 하지 않는다.** 실주행에서 제어로 값만 보내는 경로라
# 시각화 코드가 들어올 이유가 없다.
from lane_detection import LaneDetector, default_checkpoint
from morai_camera import DEFAULT_IP, DEFAULT_PORT, CameraStream


def build_arg_parser():
    ap = argparse.ArgumentParser(description="시뮬레이터 실시간 차선 출력값")
    ap.add_argument("--checkpoint", default=default_checkpoint())
    ap.add_argument("--cam-set", default=None)
    ap.add_argument("--bonnet", default=None,
                    help="보닛 마스크 png. 기본은 코드에 박힌 폴리곤")
    ap.add_argument("--no-bonnet", action="store_true", help="보닛 제거를 끈다")
    ap.add_argument("--no-track", action="store_true", help="프레임 간 추적을 끈다")
    ap.add_argument("--ip", default=DEFAULT_IP, help="카메라 수신 IP")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="카메라 수신 포트")
    ap.add_argument("--device", default=None, help="cuda / cpu")
    ap.add_argument("--udp", default=None, metavar="HOST:PORT",
                    help="이 주소로 JSON 을 UDP 전송한다 (예: 127.0.0.1:7600)")
    ap.add_argument("--quiet", action="store_true",
                    help="표준출력으로 JSON 을 찍지 않는다 (UDP 전송만)")
    ap.add_argument("--points", action="store_true",
                    help="left/right 점열까지 보낸다 (기본은 빼고 요약값만 - "
                         "점열이 프레임당 수십 개라 UDP 패킷이 커진다)")
    ap.add_argument("--stats-sec", type=float, default=2.0,
                    help="통계를 몇 초마다 stderr 로 찍을지 (0=안 찍음)")
    return ap


def main(argv=None):
    args = build_arg_parser().parse_args(argv)

    sender = None
    if args.udp:
        host, _, port = args.udp.partition(":")
        if not port:
            raise SystemExit("--udp 는 HOST:PORT 형식입니다 (예: 127.0.0.1:7600)")
        sender = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (host, int(port)))

    pipe = LaneDetector(args.checkpoint, cam_set=args.cam_set,
                        bonnet_mask=False if args.no_bonnet else args.bonnet,
                        device=args.device, track=not args.no_track)
    print(f"[out] epoch {pipe.ckpt_info['epoch']} ({pipe.ckpt_info['backbone']}) "
          f"device={pipe.device} 보닛 {pipe.bonnet_source}", file=sys.stderr)

    cam = CameraStream(args.ip, args.port).start()
    print(f"[out] {args.ip}:{args.port} 대기 중...", file=sys.stderr)
    if not cam.wait_first(timeout=15.0):
        raise SystemExit("카메라 프레임이 안 옵니다. 시뮬레이터와 IP/포트를 확인하세요.")
    print("[out] 수신 시작. Ctrl+C 로 종료합니다.", file=sys.stderr)

    last_seq = -1
    n = n_left = n_right = n_stop = 0
    t_stat = time.time()
    ms_sum = 0.0

    try:
        while True:
            frame, seq = cam.latest()
            if frame is None or seq == last_seq:
                time.sleep(0.002)
                continue
            last_seq = seq

            res = pipe.run(frame)
            payload = res.as_dict(points=args.points)
            payload["seq"] = seq
            payload["t"] = round(time.time(), 3)

            line = json.dumps(payload, ensure_ascii=False)
            if sender is not None:
                sender[0].sendto(line.encode("utf-8"), sender[1])
            if not args.quiet:
                print(line, flush=True)

            n += 1
            n_left += res.ego_left is not None
            n_right += res.ego_right is not None
            n_stop += res.stopline_dist is not None
            ms_sum += res.infer_ms + res.post_ms

            if args.stats_sec and time.time() - t_stat >= args.stats_sec:
                dt = time.time() - t_stat
                print(f"[out] {n/dt:5.1f} FPS  평균 {ms_sum/max(n,1):5.0f}ms  "
                      f"좌 {n_left}/{n} 우 {n_right}/{n} 정지선 {n_stop}/{n}",
                      file=sys.stderr)
                t_stat = time.time()
                n = n_left = n_right = n_stop = 0
                ms_sum = 0.0
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        if sender is not None:
            sender[0].close()
        print("[out] 종료", file=sys.stderr)


if __name__ == "__main__":
    main()
