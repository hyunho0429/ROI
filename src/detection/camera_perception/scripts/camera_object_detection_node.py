import ctypes
import argparse
import os
import threading
import torch
import torch.nn as nn
import ultralytics.nn.tasks as tasks

class ChannelAttention(nn.Module):
    def __init__(self, channels, reduction=16):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(channels, max(1, channels // reduction), bias=False),
            nn.ReLU(inplace=True),
            nn.Linear(max(1, channels // reduction), channels, bias=False)
        )
        self.sigmoid = nn.Sigmoid()
    def forward(self, x):
        b, c, _, _ = x.size()
        avg_out = self.fc(x.mean((2, 3)).view(b, c)).view(b, c, 1, 1)
        max_out = self.fc(x.amax((2, 3)).view(b, c)).view(b, c, 1, 1)
        return x * self.sigmoid(avg_out + max_out)

class SpatialAttention(nn.Module):
    def __init__(self, kernel_size=7):
        super().__init__()
        self.conv = nn.Conv2d(2, 1, kernel_size, padding=kernel_size // 2, bias=False)
        self.sigmoid = nn.Sigmoid()
    def forward(self, x):
        avg_out = torch.mean(x, dim=1, keepdim=True)
        max_out, _ = torch.max(x, dim=1, keepdim=True)
        mask = self.sigmoid(self.conv(torch.cat([avg_out, max_out], dim=1)))
        return x * mask

class CBAM(nn.Module):
    def __init__(self, c1, kernel_size=7):
        super().__init__()
        self.ca = ChannelAttention(c1)
        self.sa = SpatialAttention(kernel_size)
    def forward(self, x):
        return self.sa(self.ca(x))

# YOLO 내부 파서에 CBAM 강제 등록
tasks.CBAM = CBAM

# X11 멀티스레드 충돌 방지 설정
try:
    X11 = ctypes.CDLL("libX11.so.6")
    X11.XInitThreads()
except Exception:
    pass

import sys
from pathlib import Path

PACKAGE_SOURCE = Path(__file__).resolve().parents[1] / "src"
if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))

import time
import cv2
import numpy as np
from camera_perception.camera_udp import LatestCameraReceiver
from camera_perception.highway_vehicle import (
    HIGHWAY_VEHICLE_CLASSES,
    highway_vehicle_detected,
)

# 공통 IP 설정 (기존 기본값은 유지하고 환경변수/CLI로 덮어쓸 수 있다.)
IP = os.environ.get("MORAI_YOLO_CAM_IP", "0.0.0.0")

# Cam 4 전용 설정 (Port: 1131)
CAM_NAME = "Cam 4"
PORT = int(os.environ.get("MORAI_YOLO_CAM_PORT", "1131"))

# 💡 1. 투트랙 모델 로드
# (1) 기본 사물 탐지 모델 (사람, 차량, 버스, 정지표지판, 동물 등)
BASE_MODEL_PATH = os.environ.get("MORAI_YOLO_BASE_MODEL", "yolov8n.pt")

# (2) 커스텀 모델 (신호등 R/G/Y, 모라이 장애물 등)
CUSTOM_MODEL_PATH = os.environ.get("MORAI_YOLO_CUSTOM_MODEL", "best0902.pt")
CAR_DETECTED_TOPIC = os.environ.get(
    "MORAI_YOLO_CAR_TOPIC", "/perception/camera/car_detected"
)
PERSON_DETECTED_TOPIC = os.environ.get(
    "MORAI_YOLO_PERSON_TOPIC", "/perception/camera/person_detected"
)
INFERENCE_SIZE = int(os.environ.get("MORAI_YOLO_INFERENCE_SIZE", "416"))
# 0 means that the MORAI source rate controls the display.  Adding a 33 ms GUI
# wait to a receiver that already waits for a 30 Hz frame would halve the rate.
DISPLAY_FPS = float(os.environ.get("MORAI_YOLO_DISPLAY_FPS", "0.0"))
CPU_THREADS = int(os.environ.get("MORAI_YOLO_CPU_THREADS", "0"))

# person, unified car, stop sign.  The competition dataset labels every
# relevant vehicle (including bus/train) as ``car``, so raw COCO bus/truck
# classes must not independently activate the situation gates.
BASE_TARGET_CLASSES = [0, 2, 5, 7, 11]
TRAFFIC_KEYWORDS = (
    "red", "green", "yellow", "left", "right", "arrow", "amber", "traffic"
)


def _parse_traffic_signal(label):
    normalized = label.lower()
    # GREEN has the highest priority, including ambiguous mixed-label names.
    if "green" in normalized and "left" in normalized:
        return "Green_Left", "GREEN + LEFT", (0, 255, 128)
    if "green" in normalized and "right" in normalized:
        return "Green_Right", "GREEN + RIGHT", (0, 255, 128)
    if "green" in normalized and "arrow" in normalized:
        return "Green_Arrow", "GREEN + ARROW", (0, 255, 128)
    if "green" in normalized:
        return "Green", "GREEN", (0, 255, 0)
    if "red" in normalized and "left" in normalized:
        return "Red_Left", "RED + LEFT", (0, 165, 255)
    if "red" in normalized and "right" in normalized:
        return "Red_Right", "RED + RIGHT", (0, 165, 255)
    if "red" in normalized and "arrow" in normalized:
        return "Red_Arrow", "RED + ARROW", (0, 165, 255)
    if "red" in normalized and "yellow" in normalized:
        return "Red_Yellow", "RED + YELLOW", (0, 128, 255)
    if "left" in normalized:
        return "Left", "LEFT", (255, 255, 0)
    if "right" in normalized:
        return "Right", "RIGHT", (255, 255, 0)
    if "arrow" in normalized:
        return "Arrow", "ARROW", (255, 255, 0)
    if "red" in normalized:
        return "Red", "RED", (0, 0, 255)
    if "yellow" in normalized or "amber" in normalized:
        return "Yellow", "YELLOW", (0, 255, 255)
    return None, None, None

def _resolve_model_path(model_path):
    """Resolve bundled feature-camera weights before trying Ultralytics cache."""
    if os.path.isabs(model_path):
        return model_path
    package_path = Path(__file__).resolve().parents[1]
    bundled_path = package_path / "models" / model_path
    return str(bundled_path) if bundled_path.exists() else model_path


def main(ip=IP, port=PORT, base_model_path=BASE_MODEL_PATH,
         custom_model_path=CUSTOM_MODEL_PATH, confidence=0.4,
         car_detected_topic=CAR_DETECTED_TOPIC,
         person_detected_topic=PERSON_DETECTED_TOPIC,
         traffic_light_topic="/detection/traffic_light",
         obstacle_topic="/detection/obstacle",
         inference_size=INFERENCE_SIZE, display_fps=DISPLAY_FPS,
         cpu_threads=CPU_THREADS, show_raw_preview=False):
    """Cam 4 UDP receive, asynchronous YOLO inference, and live display.

    Camera receive/display must not wait for model inference.  The inference
    worker always replaces its pending input with the newest frame, so a slow
    CPU lowers detection FPS without building seconds of stale video.
    """
    # argparse/help와 ROS launch 구조 검증은 모델 설정 파일 접근 없이 가능하게 한다.
    import rospy
    from std_msgs.msg import Bool, Header
    from common.msg import ObjectInfo, ObjectInfoArray
    from ultralytics import YOLO
    from collections import Counter
    
    show_raw_preview = bool(show_raw_preview)

    # PyTorch otherwise tends to occupy every vCPU in a small VirtualBox VM,
    # starving the UDP/decode/GUI thread as soon as the first inference starts.
    import torch
    selected_cpu_threads = None
    if not torch.cuda.is_available():
        available = max(1, os.cpu_count() or 1)
        selected_cpu_threads = (
            max(1, int(cpu_threads))
            if int(cpu_threads) > 0
            else max(1, min(2, available - 1))
        )
        torch.set_num_threads(selected_cpu_threads)
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            # It can only be set before inter-op work begins. Inference still
            # respects set_num_threads when another library initialized it.
            pass
        print(
            f"[{CAM_NAME}] CPU inference threads={selected_cpu_threads} "
            f"(available={available})"
        )

    rospy.init_node("yolo_camera", anonymous=False)
    car_detected_publisher = rospy.Publisher(
        car_detected_topic, Bool, queue_size=1
    )
    person_detected_publisher = rospy.Publisher(
        person_detected_topic, Bool, queue_size=1
    )
    traffic_light_publisher = rospy.Publisher(
        traffic_light_topic, ObjectInfoArray, queue_size=1
    )
    obstacle_publisher = rospy.Publisher(
        obstacle_topic, ObjectInfoArray, queue_size=1
    )
    detection_state = {"car": False, "person": False}
    detection_state_lock = threading.Lock()

    def publish_detection_state(_event=None):
        with detection_state_lock:
            car = detection_state["car"]
            person = detection_state["person"]
        car_detected_publisher.publish(Bool(data=car))
        person_detected_publisher.publish(Bool(data=person))

    detection_heartbeat_timer = rospy.Timer(
        rospy.Duration(0.1),
        publish_detection_state,
    )

    def object_message(box, model, class_name=None):
        cls_id = int(box.cls[0])
        xc, yc, width, height = box.xywh[0].detach().cpu().tolist()
        message = ObjectInfo()
        message.class_name = class_name or str(model.names[cls_id]).capitalize()
        message.conf = float(box.conf[0])
        message.x_center = float(xc)
        message.y_center = float(yc)
        message.width = float(width)
        message.height = float(height)
        return message

    def object_array(sequence, objects):
        message = ObjectInfoArray()
        message.header = Header(
            seq=int(sequence), stamp=rospy.Time.now(), frame_id="camera_link"
        )
        message.objects = list(objects)
        return message

    print(f"[{CAM_NAME}] YOLOv8 모델 로딩 중...")
    base_model = YOLO(_resolve_model_path(base_model_path))

    resolved_custom_path = _resolve_model_path(custom_model_path)
    custom_model = None
    if os.path.isfile(resolved_custom_path):
        custom_model = YOLO(resolved_custom_path)
        print(f"[{CAM_NAME}] 커스텀 모델 로드 완료: {resolved_custom_path}")
    else:
        print(f"[{CAM_NAME}] 경고: 커스텀 모델을 찾지 못해 기본 YOLO만 실행합니다: "
              f"{resolved_custom_path}")

    cam_data = LatestCameraReceiver(ip, port)
    last_frame_sequence = 0

    pending_condition = threading.Condition()
    pending_frame = {"sequence": 0, "image": None, "received_at": 0.0}
    result_lock = threading.Lock()
    latest_result = {
        "revision": 0,
        "sequence": 0,
        "source_image": None,
        "detections": (),
        "stage": "WAITING",
        "inference_ms": 0.0,
        "latency_ms": 0.0,
        "completed_at": 0.0,
        "fps": 0.0,
    }
    stop_worker = threading.Event()
    
    
    def collect_detections(result, model, color, image_height, is_custom=False):
        detections = []
        boxes = result.boxes if result.boxes is not None else ()
        for box in boxes:
            cls_id = int(box.cls[0])
            score = float(box.conf[0])
            label = str(model.names[cls_id])
            coords = box.xyxy[0].detach().cpu().tolist()
            if len(coords) != 4:
                continue
            x1, y1, x2, y2 = coords

            y_center = (y1 + y2) * 0.5
            if is_custom and any(
                name in label for name in ("Red", "Green", "Yellow")
            ) and y_center > image_height * 0.6:
                continue

            detections.append((x1, y1, x2, y2, label, score, color))
        return detections

    def inference_worker():
        last_inferred_sequence = 0
    
    def collect_custom_detections(result, model, image_height):
        """Apply the feature-camera traffic-light and obstacle filters."""
        detections = []
        traffic_objects = []
        obstacle_objects = []
        boxes = result.boxes if result.boxes is not None else ()
        for box in boxes:
            cls_id = int(box.cls[0])
            score = float(box.conf[0])
            label = str(model.names[cls_id])
            normalized = label.lower()
            xc, yc, width, height = box.xywh[0].detach().cpu().tolist()
            aspect_ratio = width / float(height) if height > 0.0 else 0.0
            relative_y = yc / float(image_height)
            x1, y1, x2, y2 = box.xyxy[0].detach().cpu().tolist()

            if any(keyword in normalized for keyword in TRAFFIC_KEYWORDS):
                if relative_y > 0.65 or aspect_ratio < 1.1:
                    continue
                if relative_y >= 0.40 and aspect_ratio < 1.3:
                    continue
                class_name, display_text, color = _parse_traffic_signal(label)
                if class_name is None:
                    continue
                traffic_objects.append(object_message(box, model, class_name))
                detections.append(
                    (x1, y1, x2, y2, display_text, score, color)
                )
                continue

            ignored = (
                "cone", "drum", "barrier", "bike", "bicycle", "truck",
                "bus", "car", "motorcycle", "vehicle",
            )
            if any(keyword in normalized for keyword in ignored):
                continue
            if relative_y > 0.80 or aspect_ratio > 1.5:
                continue
            obstacle_objects.append(object_message(box, model))
            detections.append(
                (x1, y1, x2, y2, label, score, (255, 0, 255))
            )
        return detections, traffic_objects, obstacle_objects

    def inference_worker():
        last_inferred_sequence = 0
        smoothed_fps = 0.0
        last_base_completed_at = 0.0
        while not stop_worker.is_set() and not rospy.is_shutdown():
            with pending_condition:
                pending_condition.wait_for(
                    lambda: stop_worker.is_set()
                    or pending_frame["sequence"] > last_inferred_sequence,
                    timeout=0.1,
                )
                if stop_worker.is_set():
                    return
                sequence = pending_frame["sequence"]
                image = pending_frame["image"]
                received_at = pending_frame["received_at"]

            if image is None or sequence <= last_inferred_sequence:
                continue
            last_inferred_sequence = sequence
            started_at = time.monotonic()

            try:
                # ==========================================
                # 1. 추론 전 ROI 크롭 (상단: 신호등, 하단: 장애물)
                # ==========================================
                h, w = image.shape[:2]

                # (1) 신호등용 상단 크롭 (y: 0 ~ h*0.5)
                img_tf = image[0:int(h * 0.5), :]
                # 필요에 따라 .track() 또는 .predict() 사용
                # results_tf = base_model.track(img_tf, persist=True, tracker="bytetrack.yaml")

                # (2) 장애물용 하단 크롭 (y: h*0.3 ~ h)
                cut_y = int(h * 0.3)
                img_obs = image[cut_y:h, :]
                # results_obs = custom_model.track(img_obs, persist=True, tracker="bytetrack.yaml")
            
                # ⚠️ [참고] 하단 크롭 결과 BBox 좌표를 원본으로 복원할 때는 
                # 반드시 y 좌표(y1, y2, yc 등)에 `cut_y`를 더해주는 산수를 거쳐야 합니다!
                # 예시: 원본_y = 크롭된_y + cut_y
                # ==========================================            
                base_results = base_model.track(
                    source=image,
                    classes=BASE_TARGET_CLASSES,
                    imgsz=inference_size,
                    conf=confidence,
                    persist=True,
                    tracker="bytetrack.yaml",
                    verbose=False,
                )
                base_boxes = (
                    base_results[0].boxes
                    if base_results[0].boxes is not None
                    else ()
                )
                detected_labels = {
                    str(base_model.names[int(box.cls[0])]).strip().lower()
                    for box in base_boxes
                }
                base_detections = collect_detections(
                    base_results[0], base_model, (0, 255, 0), image.shape[0]
                )
                base_objects = []
                for box in base_boxes:
                    label = str(base_model.names[int(box.cls[0])]).lower()
                    class_name = "Car" if label == "car" else label.capitalize()
                    base_objects.append(object_message(box, base_model, class_name))

                # Publish and display the COCO road-vehicle/person result immediately.
                # When null.pt exists, waiting for its second inference here
                # nearly doubles the age of the frame shown in the YOLO window.
                base_completed_at = time.monotonic()
                base_elapsed = max(base_completed_at - started_at, 1e-6)
                base_interval = (
                    base_completed_at - last_base_completed_at
                    if last_base_completed_at > 0.0
                    else base_elapsed
                )
                instant_fps = 1.0 / max(base_interval, 1e-6)
                smoothed_fps = (
                    instant_fps
                    if smoothed_fps <= 0.0
                    else 0.8 * smoothed_fps + 0.2 * instant_fps
                )
                last_base_completed_at = base_completed_at

                with result_lock:
                    latest_result.update(
                        revision=latest_result["revision"] + 1,
                        sequence=sequence,
                        # The receiver and GUI never mutate this decoded image.
                        # Avoid one full-frame copy on the latency-critical path.
                        source_image=image,
                        detections=tuple(base_detections),
                        stage="BASE",
                        inference_ms=base_elapsed * 1000.0,
                        latency_ms=max(
                            base_completed_at - received_at, 0.0
                        ) * 1000.0,
                        completed_at=base_completed_at,
                        fps=smoothed_fps,
                    )

                # One shared unified-car state feeds both the highway and
                # intersection situation gates.
                car_detected = highway_vehicle_detected(detected_labels)
                person_detected = "person" in detected_labels
                with detection_state_lock:
                    detection_state["car"] = car_detected
                    detection_state["person"] = person_detected
                publish_detection_state()
                obstacle_publisher.publish(object_array(sequence, base_objects))

                if car_detected:
                    rospy.loginfo_throttle(
                        1.0,
                        "YOLO unified Car detected (%s); camera condition is true",
                        ",".join(
                            sorted(detected_labels.intersection(HIGHWAY_VEHICLE_CLASSES))
                        ),
                    )
                if person_detected:
                    rospy.logwarn_throttle(
                        1.0,
                        "YOLO person detected; pedestrian fusion camera condition is true",
                    )

	   	
                
                # ==========================================
                # 신호등 전용 딥 크롭(Deep Crop) 및 시계열 필터 로직
                # ==========================================
                global track_history
                if 'track_history' not in globals():
                    track_history = {}

                # 💡 [핵심 수정] GUI(화면 출력)를 위한 빈 리스트를 미리 선언합니다!
                custom_detections = [] 

                if custom_model is not None:
                    h, w = image.shape[:2]
                    
                    # 🔥 1. 화면 과감하게 자르기 (ROI)
                    # 좌우 20%씩 날리고, 하단 50% 날림
                    c_x1 = int(w * 0.2)
                    c_x2 = int(w * 0.6)
                    c_y1 = int(w * 0.05)
                    c_y2 = int(h * 0.5)
                    
                    cropped_image = image[c_y1:c_y2, c_x1:c_x2]
                    
                    # 잘라낸 이미지로만 YOLO 트래킹 추론
                    custom_results = custom_model.track(
                        source=cropped_image,
                        imgsz=inference_size,
                        conf=confidence, 
                        persist=True,
                        tracker="bytetrack.yaml",
                        verbose=False,
                    )
                    
                    tracked_traffic_objects = []
                    
                    for result in custom_results:
                        boxes = result.boxes
                        if boxes.id is not None:
                            xyxys = boxes.xyxy.cpu().numpy()
                            confs = boxes.conf.cpu().numpy()
                            classes = boxes.cls.int().cpu().tolist()
                            track_ids = boxes.id.int().cpu().tolist()
            
                            for xyxy, conf, cls, track_id in zip(xyxys, confs, classes, track_ids):
                                class_name_temp = custom_model.names[cls]
                                is_traffic_light = any(kw in class_name_temp.lower() for kw in ['red', 'yellow', 'green', 'left'])
                                
                                # 🔥 신호등이 아니면 무시
                                if not is_traffic_light:
                                    continue
                                
                                # =========================================================
                                # 💡 [추가] 비정상적으로 거대한 박스(오탐지) 쳐내기!
                                # =========================================================
                                box_w = xyxy[2] - xyxy[0] # 박스의 가로 길이
                                box_h = xyxy[3] - xyxy[1] # 박스의 세로 길이
                                
                                # 원본 이미지 너비(w)와 높이(h) 기준 최대 비율 설정
                                # 신호등은 화면 전체 너비의 25%, 높이의 15%를 넘을 수 없다고 가정합니다.
                                max_allowed_w = w * 0.15 
                                max_allowed_h = h * 0.10 
                                
                                if box_w > max_allowed_w or box_h > max_allowed_h:
                                    # 박스가 허용치보다 크면 헛것(구름, 구조물 등)으로 간주하고 버림!
                                    continue
                                # =========================================================

                                # ================= 시계열 다수결(Majority Vote) =================
                                if track_id not in track_history:
                                    track_history[track_id] = []
                    
                                track_history[track_id].append(cls)
                
                                WINDOW_SIZE = 5
                                if len(track_history[track_id]) > WINDOW_SIZE:
                                    track_history[track_id].pop(0)
                                    
                                stable_cls = Counter(track_history[track_id]).most_common(1)[0][0]
                                class_name = custom_model.names[stable_cls]
                                # ================================================================
                                
                                # 🔥 2. 잘라냈던 좌표를 원본 화면 좌표로 복구
                                x1 = xyxy[0] + c_x1
                                y1 = xyxy[1] + c_y1
                                x2 = xyxy[2] + c_x1
                                y2 = xyxy[3] + c_y1
                                
                                # 💡 ROS ObjectInfo 메시지 형식 조립
                                from common.msg import ObjectInfo 
                                msg = ObjectInfo()
                                msg.class_name = class_name
                                msg.conf = float(conf)
                                msg.x_center = float((x1 + x2) / 2.0)
                                msg.y_center = float((y1 + y2) / 2.0)
                                msg.width = float(x2 - x1)
                                msg.height = float(y2 - y1)
                                
                                tracked_traffic_objects.append(msg)
                                
                                # 모니터링 GUI 창에 띄울 박스와 색상 설정
                                color = (0, 255, 255)
                                if 'red' in class_name.lower(): color = (0, 0, 255)
                                elif 'green' in class_name.lower(): color = (0, 255, 0)
                                
                                custom_detections.append((x1, y1, x2, y2, class_name, float(conf), color))
                    
                    
                    # 💡 메모리 누수 방지: 현재 화면에 잡힌 ID만 추려내서, 
                    # 과거 수첩(track_history)에 남아있는 안 보이는 ID들을 싹 지워줍니다.
                    current_ids = [track_id for result in custom_results if result.boxes.id is not None for track_id in result.boxes.id.int().cpu().tolist()]
                    # 수첩에 적힌 ID 중 현재 안 보이는 것은 삭제!
                    track_history = {tid: hist for tid, hist in track_history.items() if tid in current_ids}
                    
                    if custom_detections:
                        # 인식된 신호등 이름들만 뽑아서 쉼표로 연결 (예: RED, YELLOW)
                        labels = ", ".join(sorted({d[4] for d in custom_detections}))
                        rospy.loginfo_throttle(
                            1.0, "%s custom detections: %s", CAM_NAME, labels
                        )
                    # =========================================================
                    # 3. 최종 퍼블리시
                    traffic_light_publisher.publish(
                        object_array(sequence, tracked_traffic_objects)
                    )
                    
                
                    # Preserve the custom detector, but apply it as a second
                    # revision of the exact same frame. The base result has
                    # already reached the display and ROS topics above.
                    custom_completed_at = time.monotonic()
                    with result_lock:
                        latest_result.update(
                            revision=latest_result["revision"] + 1,
                            sequence=sequence,
                            source_image=image,
                            detections=tuple(
                                base_detections + custom_detections
                            ),
                            stage="BASE+CUSTOM",
                            inference_ms=max(
                                custom_completed_at - started_at, 0.0
                            ) * 1000.0,
                            latency_ms=max(
                                custom_completed_at - received_at, 0.0
                            ) * 1000.0,
                            completed_at=custom_completed_at,
                            fps=smoothed_fps,
                        )
                else:
                    traffic_light_publisher.publish(object_array(sequence, ()))

            except Exception as error:
                rospy.logerr_throttle(1.0, "YOLO inference error: %s", error)

    worker = threading.Thread(
        target=inference_worker,
        name="morai-yolo-inference",
        daemon=True,
    )
    worker.start()
    last_display_at = 0.0
    smoothed_live_fps = 0.0
    last_live_image = None
    last_live_frame_at = None
    last_detection_display_revision = 0
    live_window = f"MORAI {CAM_NAME} Live Preview"
    detection_window = f"MORAI {CAM_NAME} YOLO Detection (Frame Matched)"
    
    print(f"[{CAM_NAME}] MORAI UDP 카메라 연결 시도 중... ({ip}:{port})")

    while not rospy.is_shutdown():
        try:
            frame = cam_data.wait_for_latest(last_frame_sequence, timeout=0.1)
            if frame is None:
                # Even with no UDP frame, pump GUI events so the window does
                # not become frozen/unresponsive. Show an explicit watchdog
                # warning instead of silently leaving the last image onscreen.
                now = time.monotonic()
                stale_for = (
                    now - last_live_frame_at
                    if last_live_frame_at is not None
                    else float("inf")
                )
                if stale_for > 0.5:
                    health = cam_data.health_snapshot()
                    rospy.logwarn_throttle(
                        1.0,
                        "No complete camera frame for %.2fs; receiver=%s",
                        stale_for,
                        health,
                    )
                    if show_raw_preview:
                        waiting = (
                            last_live_image.copy()
                            if last_live_image is not None
                            else np.zeros((480, 640, 3), dtype=np.uint8)
                        )
                        cv2.rectangle(
                            waiting,
                            (0, 0),
                            (waiting.shape[1], 38),
                            (0, 0, 180),
                            -1,
                        )
                        cv2.putText(
                            waiting,
                            "NO NEW CAMERA FRAME - check MORAI UDP",
                            (8, 26),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.62,
                            (255, 255, 255),
                            2,
                        )
                        cv2.imshow(live_window, waiting)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue
            last_frame_sequence = frame.sequence

            image_np = np.frombuffer(frame.jpeg_data, dtype=np.uint8)
            if image_np.size == 0:
                continue

            image = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
            if image is None or image.size == 0:
                continue
            if show_raw_preview:
                last_live_image = image
            last_live_frame_at = time.monotonic()

            # Replace the pending inference job instead of queueing this frame.
            with pending_condition:
                pending_frame["sequence"] = frame.sequence
                pending_frame["image"] = image
                pending_frame["received_at"] = last_live_frame_at
                pending_condition.notify()

            now = time.monotonic()
            if (
                display_fps > 0.0
                and last_display_at > 0.0
                and now - last_display_at < 1.0 / display_fps
            ):
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue
            if last_display_at > 0.0:
                instant_live_fps = 1.0 / max(now - last_display_at, 1e-6)
                smoothed_live_fps = (
                    instant_live_fps
                    if smoothed_live_fps <= 0.0
                    else 0.9 * smoothed_live_fps + 0.1 * instant_live_fps
                )
            last_display_at = now

            # The live preview intentionally has no boxes. A box is only valid
            # for the exact source frame used by its YOLO inference.
            with result_lock:
                shown_result = dict(latest_result)
            result_age_ms = (
                (time.monotonic() - shown_result["completed_at"]) * 1000.0
                if shown_result["completed_at"] > 0.0
                else 0.0
            )
            if show_raw_preview:
                display_frame = image.copy()
                status = (
                    f"LIVE {smoothed_live_fps:.1f} FPS | "
                    f"YOLO {shown_result['fps']:.1f} FPS | "
                    f"infer {shown_result['inference_ms']:.0f} ms | "
                    f"latency {shown_result['latency_ms']:.0f} ms | "
                    f"age {result_age_ms:.0f} ms"
                )
                cv2.rectangle(
                    display_frame,
                    (0, 0),
                    (display_frame.shape[1], 30),
                    (0, 0, 0),
                    -1,
                )
                cv2.putText(
                    display_frame,
                    status,
                    (8, 21),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                )
                cv2.imshow(live_window, display_frame)

            # A BASE revision is displayed as soon as the primary detector
            # finishes. If configured, BASE+CUSTOM follows on the same exact
            # frame without delaying car/person output behind the second model.
            result_revision = int(shown_result["revision"])
            result_sequence = int(shown_result["sequence"])
            matched_source = shown_result["source_image"]
            if (
                matched_source is not None
                and result_revision > last_detection_display_revision
            ):
                matched_frame = matched_source.copy()
                for x1, y1, x2, y2, label, score, color in shown_result["detections"]:
                    p1 = (max(0, int(x1)), max(0, int(y1)))
                    p2 = (
                        min(matched_frame.shape[1] - 1, int(x2)),
                        min(matched_frame.shape[0] - 1, int(y2)),
                    )
                    cv2.rectangle(matched_frame, p1, p2, color, 2)
                    cv2.putText(
                        matched_frame,
                        f"{label} {score:.2f}",
                        (p1[0], max(18, p1[1] - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        2,
                    )
                detection_status = (
                    f"{shown_result['stage']} FRAME {result_sequence} | "
                    f"YOLO {shown_result['fps']:.1f} FPS | "
                    f"infer {shown_result['inference_ms']:.0f} ms | "
                    f"latency {shown_result['latency_ms']:.0f} ms"
                )
                cv2.rectangle(
                    matched_frame,
                    (0, 0),
                    (matched_frame.shape[1], 30),
                    (0, 0, 0),
                    -1,
                )
                cv2.putText(
                    matched_frame,
                    detection_status,
                    (8, 21),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                )
                cv2.imshow(detection_window, matched_frame)
                last_detection_display_revision = result_revision

            # 'q' 키를 누르면 모니터링 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"[{CAM_NAME}] Error: {e}")
            time.sleep(0.01)

    stop_worker.set()
    with pending_condition:
        pending_condition.notify_all()
    worker.join(timeout=1.0)
    try:
        with detection_state_lock:
            detection_state["car"] = False
            detection_state["person"] = False
        car_detected_publisher.publish(Bool(data=False))
        person_detected_publisher.publish(Bool(data=False))
    except rospy.ROSException:
        pass
    detection_heartbeat_timer.shutdown()
    cam_data.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MORAI UDP YOLO 객체 탐지")
    parser.add_argument("--cam-ip", default=IP)
    parser.add_argument("--cam-port", type=int, default=PORT)
    parser.add_argument("--base-model", default=BASE_MODEL_PATH)
    parser.add_argument("--custom-model", default=CUSTOM_MODEL_PATH)
    parser.add_argument("--confidence", type=float, default=0.4)
    parser.add_argument("--car-detected-topic", default=CAR_DETECTED_TOPIC)
    parser.add_argument("--person-detected-topic", default=PERSON_DETECTED_TOPIC)
    parser.add_argument(
        "--traffic-light-topic", default="/detection/traffic_light"
    )
    parser.add_argument("--obstacle-topic", default="/detection/obstacle")
    parser.add_argument("--inference-size", type=int, default=INFERENCE_SIZE)
    parser.add_argument(
        "--display-fps",
        type=float,
        default=DISPLAY_FPS,
        help="maximum live display FPS; 0 follows the MORAI source rate",
    )
    parser.add_argument(
        "--cpu-threads",
        type=int,
        default=CPU_THREADS,
        help="PyTorch CPU threads; 0 reserves at least one vCPU for camera/GUI",
    )
    parser.add_argument(
        "--show-raw-preview",
        type=int,
        choices=(0, 1),
        default=0,
        help="show the unprocessed camera window (0 disables it)",
    )
    args = parser.parse_args()
    main(args.cam_ip, args.cam_port, args.base_model, args.custom_model,
         args.confidence, args.car_detected_topic, args.person_detected_topic,
         args.traffic_light_topic, args.obstacle_topic,
         args.inference_size, args.display_fps, args.cpu_threads,
         bool(args.show_raw_preview))
