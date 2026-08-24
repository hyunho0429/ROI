import ctypes

# X11 멀티스레드 충돌 방지 설정
try:
  X11 = ctypes.CDLL("libX11.so.6")
  X11.XInitThreads()
except Exception:
  pass

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

import threading
import time
import cv2
import numpy as np
from lib.define.Camera import Camera
from lib.network.UDP import Receiver

# 공통 IP 설정
IP = "192.168.0.185"

# 카메라별 포트 및 설정 정의 (순서대로 1, 2, 3, 4분할에 매칭)
CAMERA_CONFIGS = [
    {"name": "Cam 1", "port": 1101},
    {"name": "Cam 2", "port": 1111},
    {"name": "Cam 3", "port": 1121},
    {"name": "Cam 4", "port": 1131},
]

# 4개 카메라의 최신 프레임을 안전하게 공유하기 위한 딕셔너리와 락(Lock)
latest_frames = {}
frame_lock = threading.Lock()


def camera_thread_worker(cam_name, port):
  """각 카메라별 독립적인 UDP 수신 및 처리 스레드 함수"""
  cam_data = Receiver(IP, port, Camera())
  is_boundingbox = True

  # 초기 빈 이미지 생성 (데이터 수신 전 화면 깨짐 방지용 기본 검은 화면)
  default_image = np.zeros((480, 640, 3), dtype=np.uint8)
  cv2.putText(
      default_image,
      f"{cam_name} Connecting...",
      (160, 240),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.8,
      (255, 255, 255),
      2,
  )

  with frame_lock:
    latest_frames[cam_name] = default_image

  while True:
    try:
      data = cam_data.get_data()
      if data is None:
        time.sleep(0.01)
        continue

      # 데이터 유효성 검사
      if not hasattr(data, "image") or not data.image.data:
        continue

      image_np = np.frombuffer(data.image.data, dtype=np.uint8)
      if image_np.size == 0:
        continue

      image = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
      if image is None or image.size == 0:
        continue

      # Bounding Box 처리
      if is_boundingbox and time.time() - data.bbox_timestamp < 1:
        data.utils.setResolution(w=640, h=480, fov=90)
        bbox_info = data.bbox
        _3d_bb_img = image.copy()

        for bbox_info in bbox_info.object_data:
          if (
              bbox_info.Group == 0
              and bbox_info.Class == 0
              and bbox_info.SubClass == 0
          ):
            break

          color = (bbox_info.Group, bbox_info.Class, bbox_info.SubClass)

          # 2D BBox 그리기
          point_2 = (
              int(bbox_info._2D_BBOX_XMAX),
              int(bbox_info._2D_BBOX_YMAX),
          )
          point_1 = (
              int(bbox_info._2D_BBOX_XMIN),
              int(bbox_info._2D_BBOX_YMIN),
          )
          data.utils.draw_2d_bbox(_3d_bb_img, point_1, point_2, color)

          # 3D BBox 그리기
          point_list = data.utils.projectPoints(bbox_info)
          data.utils.draw_3d_bbox(_3d_bb_img, point_list, color)

        display_img = _3d_bb_img
      else:
        display_img = image

      # 크기 통일 (640x480으로 리사이즈하여 4분할 격자 정렬 맞춤)
      display_img = cv2.resize(display_img, (640, 480))

      # 카메라 이름 텍스트 오버레이
      cv2.putText(
          display_img,
          cam_name,
          (30, 40),
          cv2.FONT_HERSHEY_SIMPLEX,
          1.0,
          (0, 255, 0),
          2,
      )

      # 공유 딕셔너리에 최신 프레임 업데이트
      with frame_lock:
        latest_frames[cam_name] = display_img

    except Exception:
      continue


def main():
  # 멀티 스레딩 시작
  for config in CAMERA_CONFIGS:
    t = threading.Thread(
        target=camera_thread_worker,
        args=(config["name"], config["port"]),
        daemon=True,
    )
    t.start()

  # 잠시 대기하여 초기 프레임 수집 유도
  time.sleep(1)

  try:
    while True:
      with frame_lock:
        # 4개의 카메라 프레임 가져오기 (없으면 기본 검은 화면)
        img1 = latest_frames.get(
            "Cam 1", np.zeros((480, 640, 3), dtype=np.uint8)
        )
        img2 = latest_frames.get(
            "Cam 2", np.zeros((480, 640, 3), dtype=np.uint8)
        )
        img3 = latest_frames.get(
            "Cam 3", np.zeros((480, 640, 3), dtype=np.uint8)
        )
        img4 = latest_frames.get(
            "Cam 4", np.zeros((480, 640, 3), dtype=np.uint8)
        )

      # 2x2 그리드로 이미지 병합 (위쪽: Cam1 + Cam2, 아래쪽: Cam3 + Cam4)
      top_row = np.hstack((img1, img2))
      bottom_row = np.hstack((img3, img4))
      grid_img = np.vstack((top_row, bottom_row))

      # 전체 해상도가 너무 클 경우 보기 좋게 전체 축소 (예: 50%)
      grid_img = cv2.resize(grid_img, (1280, 960))

      # 단 하나의 창으로 4분할 화면 출력
      cv2.imshow("MORAI 4-Channel Multi-Camera View", grid_img)

      if cv2.waitKey(1) & 0xFF == ord("q"):
        break

  except KeyboardInterrupt:
    print("프로그램을 종료합니다.")
  finally:
    cv2.destroyAllWindows()


if __name__ == "__main__":
  main()
