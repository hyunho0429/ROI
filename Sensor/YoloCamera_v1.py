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

# 💡 1. 콜랩에서 학습하여 우분투로 가져온 .pt 파일 경로 지정
# (파일이 같은 폴더에 있다면 'filename.pt'만 적으시면 됩니다. 이부분 수정하시면 됩니다.)
MODEL_PATH = "traffic_light_front_cam4.pt"  # 예: "/home/user/workspace/best.pt"

print(f"[{CAM_NAME}] YOLOv8 신호등 모델 로딩 중...")
model = YOLO(MODEL_PATH)
print("모델 로드 완료!")

def main():
    """Cam 4 전용 UDP 수신 및 신호등 검출 메인 함수"""
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

            # 💡 2. YOLOv8 신호등 실시간 추론 (conf=0.4 threshold 적용)
            results = model.predict(source=image, conf=0.4, verbose=False)

            # 💡 3. 검출 결과를 원본 이미지에 보라색/녹색 상자로 그리기
            annotated_frame = results[0].plot()

            # 💡 4. 터미널 로그로 인식 상태 출력
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = model.names[cls_id]
                print(f"[{CAM_NAME}] 신호 감지: {label} ({conf*100:.1f}%)")

            # 💡 5. 별도 OpenCV 팝업창으로 Cam 4 화면 실시간 모니터링
            cv2.imshow(f"MORAI {CAM_NAME} Traffic Light Monitor", annotated_frame)

            # 'q' 키를 누르면 모니터링 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"[{CAM_NAME}] Error: {e}")
            time.sleep(0.01)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
