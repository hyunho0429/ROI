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
from ultralytics import YOLO  # YOLO 모델 라이브러리
from lib.define.Camera import Camera
from lib.network.UDP import Receiver

# 공통 IP 설정
IP = "192.168.0.200"

# Cam 4 전용 설정 (Port: 1131)
CAM_NAME = "Cam 4"
PORT = 1131

# 💡 1. 투트랙 모델 로드
# (1) 기본 사물 탐지 모델 (사람, 차량, 버스, 정지표지판, 동물 등)
BASE_MODEL_PATH = "yolov8n.pt" 

# (2) 커스텀 모델 (신호등 R/G/Y, 모라이 장애물 등)
CUSTOM_MODEL_PATH = "null.pt" 

print(f"[{CAM_NAME}] YOLOv8 모델 로딩 중...")
base_model = YOLO(BASE_MODEL_PATH)
custom_model = YOLO(CUSTOM_MODEL_PATH)
print("기본 모델 및 커스텀 모델 로드 완료!")

def main():
    """Cam 4 전용 UDP 수신 및 투트랙 검출 메인 함수"""
    cam_data = Receiver(IP, PORT, Camera())
    
    print(f"[{CAM_NAME}] MORAI UDP 카메라 연결 시도 중... (Port: {PORT})")

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

            img_h, img_w, _ = image.shape

            # 💡 2. [트랙 1] 기본 사물 탐지 (사람, 차, 표지판 등)
            base_results = base_model.predict(source=image, conf=0.4, verbose=False)
            # 기본 탐지 결과를 원본 이미지에 1차로 시각화
            annotated_frame = base_results[0].plot()

            # 💡 3. [트랙 2] 커스텀 탐지 (신호등, 모라이 장애물)
            custom_results = custom_model.predict(source=image, conf=0.4, verbose=False)
            # 커스텀 탐지 결과를 기존 1차 시각화 프레임 위에 중첩해서 시각화
            annotated_frame = custom_results[0].plot(img=annotated_frame)

            # 💡 4. 터미널 로그 출력 및 좌표 필터링 (소프트 ROI)
            for box in custom_results[0].boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = custom_model.names[cls_id]
                
                # Y축 중심 좌표 계산 (box.xywh -> [x_center, y_center, width, height])
                y_center = float(box.xywh[0][1])

                # [소프트 ROI 로직 예시]
                # 신호등 라벨인데 화면 하단(60% 초과 지점)에서 탐지된 경우, 바닥 불빛/오탐지로 보고 무시
                if ("Red" in label or "Green" in label or "Yellow" in label) and y_center > (img_h * 0.6):
                    continue

                print(f"[{CAM_NAME}] 감지: {label} ({conf*100:.1f}%)")

            # 💡 5. 통합 시각화 화면 출력
            cv2.imshow(f"MORAI {CAM_NAME} Traffic & Object Monitor", annotated_frame)

            # 'q' 키를 누르면 모니터링 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"[{CAM_NAME}] Error: {e}")
            time.sleep(0.01)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
