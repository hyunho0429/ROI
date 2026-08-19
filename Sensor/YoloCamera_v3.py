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
from ultralytics import YOLO
from lib.define.Camera import Camera
from lib.network.UDP import Receiver

IP = "192.168.0.200"
CAM_NAME = "Cam 4"
PORT = 1131

# 💡 1. 모델 로드
BASE_MODEL_PATH = "yolov8n.pt" 
CUSTOM_MODEL_PATH = "null.pt" 

print(f"[{CAM_NAME}] 모델 로딩 중...")
base_model = YOLO(BASE_MODEL_PATH)
custom_model = YOLO(CUSTOM_MODEL_PATH)
print("모델 로드 완료!")

# COCO 기본 클래스 중 관심 대상 (사람:0, 자전거:1, 승용차:2, 오토바이:3, 버스:5, 트럭:7, 정지표지판:11 등)
# ⚠️ Class 9 (traffic light)는 기본 모델에서 탐지/시각화하지 않도록 제외
BASE_TARGET_CLASSES = [0, 1, 2, 3, 5, 7, 11]

def main():
    cam_data = Receiver(IP, PORT, Camera())
    print(f"[{CAM_NAME}] MORAI UDP 수신 시작 (Port: {PORT})")

    while True:
        try:
            data = cam_data.get_data()
            if data is None or not hasattr(data, "image") or not data.image.data:
                time.sleep(0.01)
                continue

            image_np = np.frombuffer(data.image.data, dtype=np.uint8)
            if image_np.size == 0:
                continue

            image = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
            if image is None or image.size == 0:
                continue

            img_h, img_w, _ = image.shape

            # 💡 2. [트랙 1] 기본 사물 탐지 (신호등 제외, 차량/사람/표지판만 검출)
            base_results = base_model.predict(
                source=image, 
                classes=BASE_TARGET_CLASSES,  # traffic light(9) 제외
                conf=0.4, 
                verbose=False
            )
            # 기본 객체들만 1차 시각화
            annotated_frame = base_results[0].plot()

            # 💡 3. [트랙 2] 커스텀 탐지 (신호등 Red/Green/Yellow, 모라이 장애물)
            custom_results = custom_model.predict(source=image, conf=0.4, verbose=False)

            # 💡 4. 커스텀 결과 필터링 (종횡비 + Y좌표 필터)
            for box in custom_results[0].boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = custom_model.names[cls_id]
                
                xc, yc, w, h = box.xywh[0].cpu().numpy()
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())

                aspect_ratio = w / float(h) if h > 0 else 0.0
                rel_y = yc / img_h

                # 신호등 색상 라벨인지 확인 (Red, Green, Yellow 등)
                is_traffic_light = any(color in label for color in ["Red", "Green", "Yellow", "red", "green", "yellow"])

                if is_traffic_light:
                    # 1) 화면 하단 65% 초과 영역 제거 (바닥 반사 / 근거리 보행자 신호등)
                    if rel_y > 0.65:
                        continue

                    # 2) 종횡비(가로/세로 비율) 필터: 세로형 보행자 신호(AR ~0.5) 제거
                    #    가로형 차량 신호등(3~4구)은 통상 1.1 이상
                    if aspect_ratio < 1.1:
                        continue

                    # 3) 원근법으로 겹치는 중간 영역(40% ~ 65%)에서는 더 엄격하게 검증
                    if rel_y >= 0.40 and aspect_ratio < 1.3:
                        continue

                # --- ✅ 검증 통과한 신호등 / 커스텀 장애물만 시각화 ---
                print(f"[{CAM_NAME}] 최종 유효 인식: {label} ({conf*100:.1f}%) | 종횡비: {aspect_ratio:.2f}")

                # 라벨에 따른 박스 색상 지정
                if "Red" in label or "red" in label:
                    color = (0, 0, 255)      # 빨강
                elif "Yellow" in label or "yellow" in label:
                    color = (0, 255, 255)    # 노랑
                elif "Green" in label or "green" in label:
                    color = (0, 255, 0)      # 초록
                else:
                    color = (255, 0, 255)    # 기타 커스텀 장애물 (보라)

                # 박스 및 텍스트 렌더링
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    annotated_frame, 
                    f"{label} {conf:.2f} (AR:{aspect_ratio:.1f})", 
                    (x1, max(18, y1 - 8)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, 
                    color, 
                    2
                )

            # 💡 5. 최종 모니터링 화면 출력
            cv2.imshow(f"MORAI {CAM_NAME} Monitor", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"[{CAM_NAME}] Error: {e}")
            time.sleep(0.01)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
