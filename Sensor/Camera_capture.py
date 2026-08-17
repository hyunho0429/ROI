import ctypes

# X11 멀티스레드 충돌 방지 설정
try:
    X11 = ctypes.CDLL("libX11.so.6")
    X11.XInitThreads()
except Exception:
    pass

import sys
import os
from pathlib import Path
from datetime import datetime

sys.path.append(str(Path(__file__).resolve().parents[1]))

import threading
import time
import cv2
import numpy as np
from lib.define.Camera import Camera
from lib.network.UDP import Receiver

# 공통 IP 설정
IP = "192.168.0.200"

CAMERA_CONFIGS = [
    {"name": "Cam 1", "port": 1101},
    {"name": "Cam 2", "port": 1111},
    {"name": "Cam 3", "port": 1121},
    {"name": "Cam 4", "port": 1131},
]

latest_frames = {}
frame_lock = threading.Lock()

# 캡처된 이미지가 저장될 폴더 생성
SAVE_DIR = Path(__file__).resolve().parent / "dataset" / "images"
os.makedirs(SAVE_DIR, exist_ok=True)


def camera_thread_worker(cam_name, port):
    cam_data = Receiver(IP, port, Camera())
    
    while True:
        try:
            data = cam_data.get_data()
            if data is None:
                time.sleep(0.01)
                continue

            if not hasattr(data, "image") or not data.image.data:
                continue

            image_np = np.frombuffer(data.image.data, dtype=np.uint8)
            if image_np.size == 0:
                continue

            image = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
            if image is None or image.size == 0:
                continue

            # Bounding Box 없이 순수 이미지로 리사이즈
            display_img = cv2.resize(image, (640, 480))

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

    time.sleep(1) # 초기 수집 대기
    
    print("="*50)
    print("🤖 자동 캡처 모드가 시작되었습니다.")
    print(f"📁 저장 경로: {SAVE_DIR}")
    print("🚀 프로그램을 실행하자마자 1초마다 자동 촬영 중입니다.")
    print("종료하려면 터미널에서 Ctrl+C 를 누르세요.")
    print("="*50)

    try:
        last_capture_time = time.time()
        capture_interval = 1.0 # 2초마다 저장

        while True:
            with frame_lock:
                img1 = latest_frames.get("Cam 1")
                img2 = latest_frames.get("Cam 2")
                img3 = latest_frames.get("Cam 3")
                img4 = latest_frames.get("Cam 4")

            # 모든 카메라 프레임이 수신되었는지 확인
            if img1 is not None and img2 is not None and img3 is not None and img4 is not None:
                # 1초 주기로 저장
                if time.time() - last_capture_time >= capture_interval:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:19]
                    
                    cv2.imwrite(str(SAVE_DIR / f"cam1_{timestamp}.jpg"), img1)
                    cv2.imwrite(str(SAVE_DIR / f"cam2_{timestamp}.jpg"), img2)
                    cv2.imwrite(str(SAVE_DIR / f"cam3_{timestamp}.jpg"), img3)
                    cv2.imwrite(str(SAVE_DIR / f"cam4_{timestamp}.jpg"), img4)
                    
                    print(f"[{timestamp}] 4채널 저장 완료")
                    last_capture_time = time.time()

            time.sleep(0.05) # CPU 점유율 방지

    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")


if __name__ == "__main__":
    main()
