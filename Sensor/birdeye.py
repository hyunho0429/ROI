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

import time
import cv2
import numpy as np
from lib.define.Camera import Camera
from lib.network.UDP import Receiver

# IP 및 4번 카메라 설정
IP = "192.168.0.200"
PORT = 1131  # Cam 4 포트


# --- 1. 버드아이뷰 변환 함수 ---
def bird_eye_view(image):
    """
    정면 카메라 이미지를 위에서 내려다보는 버드아이뷰 영상으로 변환
    """
    height, width = image.shape[:2]
    
    src_points = np.float32([
        [width * 0.25, height * 0.60],  # 멀리 있는 좌측 차선 너머까지
        [width * 0.75, height * 0.60],  # 멀리 있는 우측 차선 너머까지
        [width * 0.95, height * 0.98],  # 차 바로 앞 우측 끝
        [width * 0.05, height * 0.98]   # 차 바로 앞 좌측 끝
    ])

    dst_points = np.float32([
        [width * 0.20, 0],               # 좌측 상단
        [width * 0.80, 0],               # 우측 상단
        [width * 0.80, height],          # 우측 하단
        [width * 0.20, height]           # 좌측 하단
    ])

    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    warped_image = cv2.warpPerspective(
    image, 
    matrix, 
    (width, height), 
    flags=cv2.INTER_CUBIC # 픽셀을 부드럽고 보정해서 펼쳐줌
    )
    return warped_image


# --- 2. 원본과 버드아이뷰 나란히 출력하는 함수 ---
def process_cam4_bev(raw_image):
    # bird_eye_view 함수로 변환 수행
    bev_img = bird_eye_view(raw_image)

    # 원본과 버드아이뷰 영상을 나란히(Side-by-Side) 보기
    combined = np.hstack((raw_image, bev_img))

    cv2.imshow("Original vs Bird's Eye View", combined)
    cv2.waitKey(1)


# --- 3. UDP 메인 수신 루프 ---
def main():
    cam_data = Receiver(IP, PORT, Camera())
    print(f"[Cam 4] UDP 이미지 수신 시작 (Port: {PORT})...")

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

            raw_image = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
            if raw_image is None or raw_image.size == 0:
                continue

            # 버드아이뷰 처리 및 화면 출력 실행
            process_cam4_bev(raw_image)

        except Exception as e:
            print(f"[Cam 4 Error] {e}")
            time.sleep(0.01)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
