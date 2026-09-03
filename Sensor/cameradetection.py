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

# 💡 ROS 및 메시지 임포트
import rospy
from std_msgs.msg import Header
try:
    from common.msg import ObjectInfoArray, ObjectInfo
except ImportError:
    print("[경고] common.msg를 찾을 수 없습니다. catkin_make와 source devel/setup.bash를 확인하세요.")

IP = "0.0.0.0"
CAM_NAME = "Cam 4"
PORT = 1131

# 💡 1. 모델 로드
BASE_MODEL_PATH = "yolov8n.pt" 
CUSTOM_MODEL_PATH = "best0902.pt"  # 학습 완료된 가중치

print(f"[{CAM_NAME}] YOLO 모델 로딩 중...")
base_model = YOLO(BASE_MODEL_PATH)
custom_model = YOLO(CUSTOM_MODEL_PATH)
print("모델 로드 완료!")

# COCO 기본 클래스 중 관심 대상 (traffic light 9 제외)
BASE_TARGET_CLASSES = [0, 1, 2, 3, 5, 7, 11]
TRAFFIC_KEYWORDS = ["red", "green", "yellow", "left", "amber", "traffic"]

def parse_traffic_signal(label):
    """신호등 라벨에 맞춰 명세서 표준 class_name, UI 텍스트, 박스 색상 반환"""
    lbl = label.lower()
    
    if "red" in lbl and "left" in lbl:
        return "Red_Left", "RED + LEFT (Stop & Left)", (0, 165, 255)
    elif "green" in lbl and "left" in lbl:
        return "Green_Left", "GREEN + LEFT (All Go)", (0, 255, 128)
    elif "red" in lbl and "yellow" in lbl:
        return "Red_Yellow", "RED + YELLOW (Ready)", (0, 128, 255)
    elif "left" in lbl:
        return "Left", "LEFT ONLY (Go Left)", (255, 255, 0)
    elif "red" in lbl:
        return "Red", "RED (Stop)", (0, 0, 255)
    elif "yellow" in lbl or "amber" in lbl:
        return "Yellow", "YELLOW (Decel)", (0, 255, 255)
    elif "green" in lbl:
        return "Green", "GREEN (Go)", (0, 255, 0)
    
    return None, None, None

def main():
    # 💡 1. ROS 노드 및 토픽 분리 퍼블리셔 초기화
    rospy.init_node("yolo_camera_4_node", anonymous=True)
    traffic_pub = rospy.Publisher("/detection/traffic_light", ObjectInfoArray, queue_size=1)
    obstacle_pub = rospy.Publisher("/detection/obstacle", ObjectInfoArray, queue_size=1)

    cam_data = Receiver(IP, PORT, Camera())
    print(f"[{CAM_NAME}] MORAI UDP 수신 및 토픽 분리 발행 시작 (Port: {PORT})")

    seq = 0

    while not rospy.is_shutdown():
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

            # 💡 2. [트랙 1] 기본 사물 탐지 (차량/사람/표지판)
            base_results = base_model.predict(
                source=image, 
                classes=BASE_TARGET_CLASSES,
                conf=0.5, 
                verbose=False
            )
            annotated_frame = base_results[0].plot()

            # 💡 3. [트랙 2] 커스텀 탐지
            custom_results = custom_model.predict(source=image, conf=0.5, verbose=False)

	    # 💡 4. 신호등용 / 장애물용 메시지 분리 생성
            traffic_msg = ObjectInfoArray()
            traffic_msg.header = Header(seq=seq, stamp=rospy.Time.now(), frame_id="camera_link")
            traffic_msg.objects = []

            obstacle_msg = ObjectInfoArray()
            obstacle_msg.header = Header(seq=seq, stamp=rospy.Time.now(), frame_id="camera_link")
            obstacle_msg.objects = []

            # ==========================================
            # 💡 [추가된 부분] 기본 모델(차량, 사람 등) 결과를 장애물 토픽에 추가
            # ==========================================
            for box in base_results[0].boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = base_model.names[cls_id]  # 예: 'person', 'car', 'bus' 등
                
                xc, yc, w, h = box.xywh[0].cpu().numpy()
                
                obj_info = ObjectInfo()
                
                # --- 💡 교통수단 클래스명 통합 ('Car'로 덮어쓰기) ---
                if label in ["car", "bus", "truck", "motorcycle", "bicycle"]:
                    obj_info.class_name = "Car"
                else:
                # 사람(person)이나 정지표지판(stop sign) 등은 그대로 첫 글자만 대문자로 변환
                    obj_info.class_name = label.capitalize() 
                # ------------------------------------------------

                # 첫 글자를 대문자로 변환해서 깔끔하게 전송 (예: "person" -> "Person")
                obj_info.conf = float(conf)
                obj_info.x_center = float(xc)
                obj_info.y_center = float(yc)
                obj_info.width = float(w)
                obj_info.height = float(h)
                
                # 차량, 사람 등은 모두 '장애물(obstacle)' 토픽에 담습니다
                obstacle_msg.objects.append(obj_info)
                
            # 💡 5. 커스텀 결과 필터링 및 분류 패킹
            for box in custom_results[0].boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = custom_model.names[cls_id]
                
                xc, yc, w, h = box.xywh[0].cpu().numpy()
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())

                aspect_ratio = w / float(h) if h > 0 else 0.0
                rel_y = yc / img_h
                lbl_low = label.lower()

                is_traffic = any(kw in lbl_low for kw in TRAFFIC_KEYWORDS)

                # ==========================================
                # 🚦 [A] 신호등 객체 처리
                # ==========================================
                if is_traffic:
                    # 기하학적 필터링 (보행자 신호등 및 바닥 노이즈 차단)
                    if rel_y > 0.65 or aspect_ratio < 1.1:
                        continue
                    if rel_y >= 0.40 and aspect_ratio < 1.3:
                        continue

                    class_name, display_text, box_color = parse_traffic_signal(label)
                    if class_name is None:
                        continue

                    obj_info = ObjectInfo()
                    obj_info.class_name = class_name
                    obj_info.conf = float(conf)
                    obj_info.x_center = float(xc)
                    obj_info.y_center = float(yc)
                    obj_info.width = float(w)
                    obj_info.height = float(h)
                    
                    # 신호등 토픽 배열에만 추가
                    traffic_msg.objects.append(obj_info)

                    # UI 시각화
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), box_color, 2)
                    cv2.putText(
                        annotated_frame, 
                        f"{display_text} {conf:.2f}", 
                        (x1, max(18, y1 - 8)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, 
                        box_color, 
                        2
                    )

                # ==========================================
                # 🚧 [B] 장애물 객체 처리 (라바콘, 드럼통 등)
                # ==========================================
                else:
		     # --- 💡 [수정됨] 무시할 객체 리스트 ---
                    # 라바콘, 바리케이드 + 커스텀 모델이 중복으로 잡는 교통수단 싹 다 제외
                    ignore_keywords = [
                        "cone", "drum", "barrier", 
                        "bike", "bicycle", "truck", "bus", "car", "motorcycle", "vehicle"
                    ]
                    
                    if any(kw in lbl_low for kw in ignore_keywords):
                        continue
                    # ------------------------------------
                
		     # --- 💡 [추가] 장애물 노이즈 필터링 ---
                    # 1. ROI 필터: 화면 하위 20% (rel_y > 0.80) 영역 무시
                    # 차량 범퍼 바로 앞 바닥의 횡단보도, 정지선, 차선 오인식 차단
                    if rel_y > 0.80:
                        continue
                    
                    # 2. 종횡비(가로/세로) 필터: 가로가 비정상적으로 긴 객체 무시
                    # 콘이나 드럼통은 세로가 더 길거나(AR < 1.0) 정사각형(AR ≒ 1.0)에 가깝습니다.
                    # 가로가 세로보다 1.5배 이상 길다면 바닥에 그려진 선일 확률이 99%입니다.
                    if aspect_ratio > 1.5:
                        continue
                    # ------------------------------------
                    obj_info = ObjectInfo()
                    obj_info.class_name = label.capitalize()
                    obj_info.conf = float(conf)
                    obj_info.x_center = float(xc)
                    obj_info.y_center = float(yc)
                    obj_info.width = float(w)
                    obj_info.height = float(h)
                    
                    # 장애물 토픽 배열에만 추가
                    obstacle_msg.objects.append(obj_info)

                    # UI 시각화 (보라색)
                    box_color = (255, 0, 255)
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), box_color, 2)
                    cv2.putText(
                        annotated_frame, 
                        f"{label.upper()} {conf:.2f}", 
                        (x1, max(18, y1 - 8)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, 
                        box_color, 
                        2
                    )

            # 💡 6. 각각 독립된 토픽으로 발행 (Publish)
            traffic_pub.publish(traffic_msg)
            obstacle_pub.publish(obstacle_msg)
            seq += 1

            if traffic_msg.objects:
                print(f"[{CAM_NAME}] 🚦 신호등 발행: {[obj.class_name for obj in traffic_msg.objects]}")
            if obstacle_msg.objects:
                print(f"[{CAM_NAME}] 🚧 장애물 발행: {[obj.class_name for obj in obstacle_msg.objects]}")

            # 💡 7. 모니터링 화면 출력
            cv2.imshow(f"MORAI {CAM_NAME} Monitor", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"[{CAM_NAME}] Error: {e}")
            time.sleep(0.01)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
