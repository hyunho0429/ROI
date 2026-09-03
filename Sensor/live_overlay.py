#!/usr/bin/env python3
"""[2/3] 시뮬레이터 카메라를 실시간으로 받아 **화면에 오버레이**한다.

===========================================================================
실행 (그대로 복사해서 붙여넣기)
===========================================================================
cd C:/MSC/AutoMobility/car/src/perception/camera_perception/post_processing
C:/Users/user/anaconda3/envs/vision_env/python.exe live_overlay.py --bev

  CPU 라 끊기면      --every 2      (2프레임마다 추론)
  창이 크면          --scale 0.7
  카메라 주소 변경   --ip 192.168.0.200 --port 1101
  GPU 로            --device cuda
===========================================================================

**저장 기능은 일부러 넣지 않았다.** 저장이 필요하면 offline_test.py 를 쓴다 —
디스크 쓰기가 끼면 실시간 루프가 프레임을 놓치고, 그러면 "화면에서 본 지연"이
실제 지연과 달라져 판단이 흐려진다.

후처리는 lane_pipeline.LanePipeline.run() 하나만 쓴다. offline_test.py,
live_output.py 와 **완전히 같은 코드**다.

화면 상단 HUD 에는 **live_output.py 가 제어로 보내는 값과 같은 값**을 같이
찍는다 (횡오차 / 방위오차 / 정지선 / 자차 좌우 차선 종류). 같은
LaneResult 에서 뽑으므로 "눈으로 본 값"과 "제어가 받은 값"이 갈리지 않는다.

    키:  q/ESC 종료   m 마스크 토글   l 차선 토글   b 조감도 토글
         v 값 HUD 토글   p 일시정지
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lane_detection import LaneDetector, default_checkpoint
from lane_viz import draw, draw_bev
from morai_camera import DEFAULT_IP, DEFAULT_PORT, CameraStream


def draw_values(vis, res, y=46):
    """제어로 나가는 값을 HUD 두 번째 띠에 찍는다.

    lane_viz.draw() 가 위 46px 를 검은 띠로 쓰므로 그 아래에 이어 붙인다.
    값은 LaneResult 에서 바로 뽑는다 - live_output.py 가 as_dict() 로 보내는
    것과 **같은 값**이라 화면과 UDP 가 어긋나지 않는다.
    """
    le, he = res.lateral_error(), res.heading_error()
    sd, l, r = res.stopline_dist, res.ego_left, res.ego_right
    txt = "  ".join((
        "lat " + ("--" if le is None else f"{le:+.2f}m"),
        "head " + ("--" if he is None else f"{np.degrees(he):+.1f}deg"),
        "stop " + ("--" if sd is None else f"{sd:.1f}m"),
        "L " + (l.name[:6] if l else "--"),
        "R " + (r.name[:6] if r else "--"),
    ))
    cv2.rectangle(vis, (0, y), (vis.shape[1], y + 20), (0, 0, 0), -1)
    # 차로 중심을 못 잡은 프레임은 회색으로 - 값이 없다는 걸 한눈에 본다.
    col = (0, 255, 0) if (le is not None and he is not None) else (150, 150, 150)
    cv2.putText(vis, txt, (8, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)


def build_arg_parser():
    ap = argparse.ArgumentParser(description="시뮬레이터 실시간 차선 오버레이")
    ap.add_argument("--checkpoint", default=default_checkpoint())
    ap.add_argument("--cam-set", default=None)
    ap.add_argument("--bonnet", default=None,
                    help="보닛 마스크 png. 기본은 코드에 박힌 폴리곤이라 보통 "
                         "줄 필요가 없다")
    ap.add_argument("--no-bonnet", action="store_true", help="보닛 제거를 끈다")
    ap.add_argument("--no-track", action="store_true", help="프레임 간 추적을 끈다")
    ap.add_argument("--ip", default=DEFAULT_IP)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--device", default=None, help="cuda / cpu")
    ap.add_argument("--bev", action="store_true", help="조감도 창도 띄운다")
    ap.add_argument("--no-values", action="store_true",
                    help="출력값 HUD 를 끄고 시작한다 (실행 중 v 로도 토글)")
    ap.add_argument("--scale", type=float, default=1.0, help="표시 배율")
    ap.add_argument("--every", type=int, default=1,
                    help="N 프레임마다 추론 (CPU 처럼 느린 환경에서 화면을 부드럽게)")
    return ap


def main(argv=None):
    args = build_arg_parser().parse_args(argv)

    pipe = LaneDetector(args.checkpoint, cam_set=args.cam_set,
                        bonnet_mask=False if args.no_bonnet else args.bonnet,
                        device=args.device, track=not args.no_track)
    print(f"[live] epoch {pipe.ckpt_info['epoch']} ({pipe.ckpt_info['backbone']}) "
          f"device={pipe.device} 보닛 {pipe.bonnet_source} "
          f"추적 {'끔' if args.no_track else '켬'}")

    cam = CameraStream(args.ip, args.port).start()
    print(f"[live] {args.ip}:{args.port} 대기 중...")
    if not cam.wait_first(timeout=15.0):
        raise SystemExit("카메라 프레임이 안 옵니다. 시뮬레이터와 IP/포트를 확인하세요.")
    print("[live] 수신 시작. q 또는 ESC 로 종료합니다.")

    show_mask = show_lanes = True
    show_values = not args.no_values
    show_bev = args.bev
    bev_open = False
    paused = False
    last_seq, res, frame = -1, None, None
    t_prev, fps = time.time(), 0.0
    n_since = 0

    try:
        while True:
            if not paused:
                f, seq = cam.latest()
                if f is not None and seq != last_seq:
                    last_seq = seq
                    n_since += 1
                    if n_since >= args.every:
                        n_since = 0
                        frame = f
                        res = pipe.run(frame)
                        now = time.time()
                        dt = now - t_prev
                        t_prev = now
                        fps = 0.9 * fps + 0.1 * (1.0 / dt) if dt > 0 and fps else \
                            (1.0 / dt if dt > 0 else 0.0)

            if res is not None and frame is not None:
                vis = draw(res, frame, pipe, show_mask=show_mask,
                           show_lanes=show_lanes)
                if show_values:
                    draw_values(vis, res)
                cv2.putText(vis, f"{fps:.1f} FPS" + ("  [PAUSED]" if paused else ""),
                            (vis.shape[1] - 190, 18), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 255, 255), 1)
                if args.scale != 1.0:
                    vis = cv2.resize(vis, None, fx=args.scale, fy=args.scale)
                cv2.imshow("lane overlay", vis)
                if show_bev:
                    bev = draw_bev(res)
                    if bev is not None:
                        cv2.imshow("bev", bev)
                        bev_open = True
                elif bev_open:
                    # 창이 있는지 cv2.getWindowProperty 로 묻지 않는다 - 만든 적이
                    # 없으면 예외를 던진다 (--bev 없이 켰을 때 첫 프레임에서 죽음).
                    cv2.destroyWindow("bev")
                    bev_open = False

            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            elif k == ord("m"):
                show_mask = not show_mask
            elif k == ord("l"):
                show_lanes = not show_lanes
            elif k == ord("b"):
                show_bev = not show_bev
            elif k == ord("v"):
                show_values = not show_values
            elif k == ord("p"):
                paused = not paused
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        cv2.destroyAllWindows()
        print("[live] 종료")


if __name__ == "__main__":
    main()
