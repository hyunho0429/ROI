import ctypes
import argparse
import os
import threading
import torch
import torch.nn as nn
import ultralytics.nn.tasks as tasks
from ultralytics.nn.modules.block import C2f

# ==========================================
# 💡 [커스텀 어텐션 모듈] EMA 및 C2f_EMA 등록
# ==========================================
class EMA(nn.Module):
    def __init__(self, channels, factor=8):
        super(EMA, self).__init__()
        self.groups = factor
        assert channels // self.groups > 0
        self.softmax = nn.Softmax(-1)
        self.agp = nn.AdaptiveAvgPool2d((1, 1))
        self.pool_h = nn.AdaptiveAvgPool2d((None, 1))
        self.pool_w = nn.AdaptiveAvgPool2d((1, None))
        self.gn = nn.GroupNorm(channels // self.groups, channels // self.groups)
        self.conv1x1 = nn.Conv2d(channels // self.groups, channels // self.groups, kernel_size=1, stride=1, padding=0)
        self.conv3x3 = nn.Conv2d(channels // self.groups, channels // self.groups, kernel_size=3, stride=1, padding=1)

    def forward(self, x):
        b, c, h, w = x.size()
        group_x = x.reshape(b * self.groups, -1, h, w)
        x_h = self.pool_h(group_x)
        x_w = self.pool_w(group_x).permute(0, 1, 3, 2)
        hw = self.conv1x1(torch.cat([x_h, x_w], dim=2))
        x_h, x_w = torch.split(hw, [h, w], dim=2)
        x1 = self.gn(group_x * x_h.sigmoid() * x_w.permute(0, 1, 3, 2).sigmoid())
        x2 = self.conv3x3(group_x)
        x11 = self.softmax(self.agp(x1).reshape(b * self.groups, -1, 1).permute(0, 2, 1))
        x12 = x2.reshape(b * self.groups, c // self.groups, -1)
        x21 = self.softmax(self.agp(x2).reshape(b * self.groups, -1, 1).permute(0, 2, 1))
        x22 = x1.reshape(b * self.groups, c // self.groups, -1)
        weights = (torch.matmul(x11, x12) + torch.matmul(x21, x22)).reshape(b * self.groups, 1, h, w)
        return (group_x * weights.sigmoid()).reshape(b, c, h, w)

class C2f_EMA(C2f):
    def __init__(self, c1, c2, n=1, shortcut=False, g=1, e=0.5):
        super().__init__(c1, c2, n, shortcut, g, e)
        self.m = nn.ModuleList(EMA(self.c) for _ in range(n))

tasks.C2f_EMA = C2f_EMA

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

IP = os.environ.get("MORAI_YOLO_CAM_IP", "0.0.0.0")
CAM_NAME = "Cam 4"
PORT = int(os.environ.get("MORAI_YOLO_CAM_PORT", "1131"))

BASE_MODEL_PATH = os.environ.get("MORAI_YOLO_BASE_MODEL", "yolov8n.pt")
CUSTOM_MODEL_PATH = os.environ.get("MORAI_YOLO_CUSTOM_MODEL", "best0902.pt")
CAR_DETECTED_TOPIC = os.environ.get("MORAI_YOLO_CAR_TOPIC", "/perception/camera/car_detected")
PERSON_DETECTED_TOPIC = os.environ.get("MORAI_YOLO_PERSON_TOPIC", "/perception/camera/person_detected")
INFERENCE_SIZE = int(os.environ.get("MORAI_YOLO_INFERENCE_SIZE", "416"))
DISPLAY_FPS = float(os.environ.get("MORAI_YOLO_DISPLAY_FPS", "0.0"))
CPU_THREADS = int(os.environ.get("MORAI_YOLO_CPU_THREADS", "0"))

BASE_TARGET_CLASSES = [0, 2, 11]
TRAFFIC_KEYWORDS = ("red", "green", "yellow", "left", "right", "arrow", "amber", "traffic")

def _parse_traffic_signal(label):
    normalized = label.lower()
    if "green" in normalized and "left" in normalized: return "Green_Left", "GREEN + LEFT", (0, 255, 128)
    if "green" in normalized and "right" in normalized: return "Green_Right", "GREEN + RIGHT", (0, 255, 128)
    if "green" in normalized and "arrow" in normalized: return "Green_Arrow", "GREEN + ARROW", (0, 255, 128)
    if "green" in normalized: return "Green", "GREEN", (0, 255, 0)
    if "red" in normalized and "left" in normalized: return "Red_Left", "RED + LEFT", (0, 165, 255)
    if "red" in normalized and "right" in normalized: return "Red_Right", "RED + RIGHT", (0, 165, 255)
    if "red" in normalized and "arrow" in normalized: return "Red_Arrow", "RED + ARROW", (0, 165, 255)
    if "red" in normalized and "yellow" in normalized: return "Red_Yellow", "RED + YELLOW", (0, 128, 255)
    if "left" in normalized: return "Left", "LEFT", (255, 255, 0)
    if "right" in normalized: return "Right", "RIGHT", (255, 255, 0)
    if "arrow" in normalized: return "Arrow", "ARROW", (255, 255, 0)
    if "red" in normalized: return "Red", "RED", (0, 0, 255)
    if "yellow" in normalized or "amber" in normalized: return "Yellow", "YELLOW", (0, 255, 255)
    return None, None, None

def _resolve_model_path(model_path):
    if os.path.isabs(model_path): return model_path
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
         cpu_threads=CPU_THREADS, show_raw_preview=False,
         window_size=5):
         
    import rospy
    from std_msgs.msg import Bool, Header
    from common.msg import ObjectInfo, ObjectInfoArray
    from ultralytics import YOLO
    from collections import Counter
    
    show_raw_preview = bool(show_raw_preview)
    selected_cpu_threads = None
    if not torch.cuda.is_available():
        available = max(1, os.cpu_count() or 1)
        selected_cpu_threads = max(1, int(cpu_threads)) if int(cpu_threads) > 0 else max(1, min(2, available - 1))
        torch.set_num_threads(selected_cpu_threads)
        try: torch.set_num_interop_threads(1)
        except RuntimeError: pass
        print(f"[{CAM_NAME}] CPU inference threads={selected_cpu_threads} (available={available})")

    rospy.init_node("yolo_camera", anonymous=False)
    car_detected_publisher = rospy.Publisher(car_detected_topic, Bool, queue_size=1)
    person_detected_publisher = rospy.Publisher(person_detected_topic, Bool, queue_size=1)
    traffic_light_publisher = rospy.Publisher(traffic_light_topic, ObjectInfoArray, queue_size=1)
    obstacle_publisher = rospy.Publisher(obstacle_topic, ObjectInfoArray, queue_size=1)
    
    detection_state = {"car": False, "person": False}
    detection_state_lock = threading.Lock()

    def publish_detection_state(_event=None):
        with detection_state_lock:
            car, person = detection_state["car"], detection_state["person"]
        car_detected_publisher.publish(Bool(data=car))
        person_detected_publisher.publish(Bool(data=person))

    detection_heartbeat_timer = rospy.Timer(rospy.Duration(0.1), publish_detection_state)

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
        message.header = Header(seq=int(sequence), stamp=rospy.Time.now(), frame_id="camera_link")
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
        print(f"[{CAM_NAME}] 경고: 커스텀 모델을 찾지 못해 기본 YOLO만 실행합니다: {resolved_custom_path}")

    cam_data = LatestCameraReceiver(ip, port)
    pending_condition = threading.Condition()
    pending_frame = {"sequence": 0, "image": None, "received_at": 0.0}
    result_lock = threading.Lock()
    latest_result = {
        "revision": 0, "sequence": 0, "source_image": None,
        "detections": (), "stage": "WAITING", "inference_ms": 0.0,
        "latency_ms": 0.0, "completed_at": 0.0, "fps": 0.0,
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
            if len(coords) != 4: continue
            x1, y1, x2, y2 = coords
            y_center = (y1 + y2) * 0.5
            if is_custom and any(name in label for name in ("Red", "Green", "Yellow")) and y_center > image_height * 0.6:
                continue
            detections.append((x1, y1, x2, y2, label, score, color))
        return detections

    def inference_worker():
        last_inferred_sequence = 0
        smoothed_fps = 0.0
        last_base_completed_at = 0.0
        
        global track_history
        if 'track_history' not in globals(): track_history = {}
        
        while not stop_worker.is_set() and not rospy.is_shutdown():
            with pending_condition:
                pending_condition.wait_for(
                    lambda: stop_worker.is_set() or pending_frame["sequence"] > last_inferred_sequence, timeout=0.1
                )
                if stop_worker.is_set(): return
                sequence = pending_frame["sequence"]
                image = pending_frame["image"]
                received_at = pending_frame["received_at"]

            if image is None or sequence <= last_inferred_sequence: continue
            last_inferred_sequence = sequence
            started_at = time.monotonic()

            try:
                base_results = base_model.track(
                    source=image, classes=BASE_TARGET_CLASSES, imgsz=inference_size,
                    conf=confidence, persist=True, tracker="bytetrack.yaml", verbose=False
                )
                base_boxes = base_results[0].boxes if base_results[0].boxes is not None else ()
                detected_labels = {str(base_model.names[int(box.cls[0])]).strip().lower() for box in base_boxes}
                base_detections = collect_detections(base_results[0], base_model, (0, 255, 0), image.shape[0])
                
                base_objects = []
                for box in base_boxes:
                    label = str(base_model.names[int(box.cls[0])]).lower()
                    class_name = "Car" if label == "car" else label.capitalize()
                    base_objects.append(object_message(box, base_model, class_name))

                base_completed_at = time.monotonic()
                base_elapsed = max(base_completed_at - started_at, 1e-6)
                base_interval = base_completed_at - last_base_completed_at if last_base_completed_at > 0.0 else base_elapsed
                smoothed_fps = (1.0 / max(base_interval, 1e-6)) if smoothed_fps <= 0.0 else 0.8 * smoothed_fps + 0.2 * (1.0 / max(base_interval, 1e-6))
                last_base_completed_at = base_completed_at

                with result_lock:
                    latest_result.update(
                        revision=latest_result["revision"] + 1, sequence=sequence,
                        source_image=image, detections=tuple(base_detections),
                        stage="BASE", inference_ms=base_elapsed * 1000.0,
                        latency_ms=max(base_completed_at - received_at, 0.0) * 1000.0,
                        completed_at=base_completed_at, fps=smoothed_fps,
                    )

                car_detected = highway_vehicle_detected(detected_labels)
                person_detected = "person" in detected_labels
                with detection_state_lock:
                    detection_state["car"], detection_state["person"] = car_detected, person_detected
                publish_detection_state()
                obstacle_publisher.publish(object_array(sequence, base_objects))

                if car_detected: rospy.loginfo_throttle(1.0, "YOLO unified Car detected")
                if person_detected: rospy.logwarn_throttle(1.0, "YOLO person detected")

                custom_detections = [] 
                tracked_traffic_objects = []

                if custom_model is not None:
                    h, w = image.shape[:2]
                    
                    c_x1 = int(w * 0.3)
                    c_x2 = int(w * 0.6)
                    c_y1 = int(h * 0.05)
                    c_y2 = int(h * 0.6)
                    
                    cropped_image = image[c_y1:c_y2, c_x1:c_x2]
                    
                    custom_results = custom_model.track(
                        source=cropped_image, imgsz=inference_size, conf=confidence, 
                        persist=True, tracker="bytetrack.yaml", verbose=False
                    )
                    
                    for result in custom_results:
                        boxes = result.boxes
                        if boxes.id is not None:
                            xyxys, confs, classes, track_ids = boxes.xyxy.cpu().numpy(), boxes.conf.cpu().numpy(), boxes.cls.int().cpu().tolist(), boxes.id.int().cpu().tolist()
            
                            for xyxy, conf, cls, track_id in zip(xyxys, confs, classes, track_ids):
                                class_name_temp = custom_model.names[cls]
                                if not any(kw in class_name_temp.lower() for kw in ['red', 'yellow', 'green', 'left']):
                                    continue
                                
                                box_w, box_h = xyxy[2] - xyxy[0], xyxy[3] - xyxy[1]
                                
                                if box_w <= 0 or box_h <= 0: continue
    
                                aspect_ratio = box_w / box_h  # (가로 / 높이) 비율 계산
    
                                # 가로가 세로의 2배 ~ 5배 사이인 가로형 박스만 신호등으로 인정합니다.
                                # (세로로 길쭉한 박스나 정사각형은 이 조건에 걸려 자동으로 버려집니다.)
                                if not (2.0 <= aspect_ratio <= 5.0):
                                    continue
                                
                                if box_w > w * 0.15 or box_h > h * 0.10: 
                                    continue

                                if track_id not in track_history: track_history[track_id] = []
                                track_history[track_id].append(cls)
                
                                if len(track_history[track_id]) > window_size: track_history[track_id].pop(0)
                                stable_cls = Counter(track_history[track_id]).most_common(1)[0][0]
                                class_name = custom_model.names[stable_cls]
                                
                                x1, y1, x2, y2 = xyxy[0] + c_x1, xyxy[1] + c_y1, xyxy[2] + c_x1, xyxy[3] + c_y1
                                
                                from common.msg import ObjectInfo 
                                msg = ObjectInfo()
                                msg.class_name, msg.conf = class_name, float(conf)
                                msg.x_center, msg.y_center = float((x1 + x2) / 2.0), float((y1 + y2) / 2.0)
                                msg.width, msg.height = float(x2 - x1), float(y2 - y1)
                                tracked_traffic_objects.append(msg)
                                
                                color = (0, 0, 255) if 'red' in class_name.lower() else ((0, 255, 0) if 'green' in class_name.lower() else (0, 255, 255))
                                custom_detections.append((x1, y1, x2, y2, class_name, float(conf), color))
                    
                    current_ids = [track_id for result in custom_results if result.boxes.id is not None for track_id in result.boxes.id.int().cpu().tolist()]
                    track_history = {tid: hist for tid, hist in track_history.items() if tid in current_ids}
                    
                    if custom_detections:
                        labels = ", ".join(sorted({d[4] for d in custom_detections}))
                        rospy.loginfo_throttle(1.0, "%s custom detections: %s", CAM_NAME, labels)

                    traffic_light_publisher.publish(object_array(sequence, tracked_traffic_objects))
                    
                    custom_completed_at = time.monotonic()
                    with result_lock:
                        latest_result.update(
                            revision=latest_result["revision"] + 1, sequence=sequence, source_image=image,
                            detections=tuple(base_detections + custom_detections), stage="BASE+CUSTOM",
                            inference_ms=max(custom_completed_at - started_at, 0.0) * 1000.0,
                            latency_ms=max(custom_completed_at - received_at, 0.0) * 1000.0,
                            completed_at=custom_completed_at, fps=smoothed_fps,
                        )
                else:
                    traffic_light_publisher.publish(object_array(sequence, ()))

            except Exception as error:
                rospy.logerr_throttle(1.0, "YOLO inference error: %s", error)

    worker = threading.Thread(target=inference_worker, name="morai-yolo-inference", daemon=True)
    worker.start()
    
    last_display_at, smoothed_live_fps = 0.0, 0.0
    last_live_image, last_live_frame_at = None, None
    last_detection_display_revision = 0
    live_window, detection_window = f"MORAI {CAM_NAME} Live Preview", f"MORAI {CAM_NAME} YOLO Detection (Frame Matched)"
    
    print(f"[{CAM_NAME}] MORAI UDP 카메라 연결 시도 중... ({ip}:{port})")
    last_frame_sequence = 0

    while not rospy.is_shutdown():
        try:
            frame = cam_data.wait_for_latest(last_frame_sequence, timeout=0.1)
            if frame is None:
                now = time.monotonic()
                stale_for = now - last_live_frame_at if last_live_frame_at is not None else float("inf")
                if stale_for > 0.5:
                    if show_raw_preview:
                        waiting = last_live_image.copy() if last_live_image is not None else np.zeros((480, 640, 3), dtype=np.uint8)
                        cv2.rectangle(waiting, (0, 0), (waiting.shape[1], 38), (0, 0, 180), -1)
                        cv2.putText(waiting, "NO NEW CAMERA FRAME - check MORAI UDP", (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
                        cv2.imshow(live_window, waiting)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
                continue
                
            last_frame_sequence = frame.sequence
            image_np = np.frombuffer(frame.jpeg_data, dtype=np.uint8)
            if image_np.size == 0: continue
            image = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
            if image is None or image.size == 0: continue
            
            if show_raw_preview: last_live_image = image
            last_live_frame_at = time.monotonic()

            with pending_condition:
                pending_frame["sequence"], pending_frame["image"], pending_frame["received_at"] = frame.sequence, image, last_live_frame_at
                pending_condition.notify()

            now = time.monotonic()
            if display_fps > 0.0 and last_display_at > 0.0 and now - last_display_at < 1.0 / display_fps:
                if cv2.waitKey(1) & 0xFF == ord('q'): break
                continue
                
            if last_display_at > 0.0:
                instant_live_fps = 1.0 / max(now - last_display_at, 1e-6)
                smoothed_live_fps = instant_live_fps if smoothed_live_fps <= 0.0 else 0.9 * smoothed_live_fps + 0.1 * instant_live_fps
            last_display_at = now

            with result_lock: shown_result = dict(latest_result)
            result_age_ms = (time.monotonic() - shown_result["completed_at"]) * 1000.0 if shown_result["completed_at"] > 0.0 else 0.0
            
            if show_raw_preview:
                display_frame = image.copy()
                status = f"LIVE {smoothed_live_fps:.1f} FPS | YOLO {shown_result['fps']:.1f} FPS | infer {shown_result['inference_ms']:.0f} ms | latency {shown_result['latency_ms']:.0f} ms | age {result_age_ms:.0f} ms"
                cv2.rectangle(display_frame, (0, 0), (display_frame.shape[1], 30), (0, 0, 0), -1)
                cv2.putText(display_frame, status, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.imshow(live_window, display_frame)

            result_revision, result_sequence = int(shown_result["revision"]), int(shown_result["sequence"])
            matched_source = shown_result["source_image"]
            if matched_source is not None and result_revision > last_detection_display_revision:
                matched_frame = matched_source.copy()
                for x1, y1, x2, y2, label, score, color in shown_result["detections"]:
                    p1, p2 = (max(0, int(x1)), max(0, int(y1))), (min(matched_frame.shape[1] - 1, int(x2)), min(matched_frame.shape[0] - 1, int(y2)))
                    cv2.rectangle(matched_frame, p1, p2, color, 2)
                    cv2.putText(matched_frame, f"{label} {score:.2f}", (p1[0], max(18, p1[1] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                detection_status = f"{shown_result['stage']} FRAME {result_sequence} | YOLO {shown_result['fps']:.1f} FPS | infer {shown_result['inference_ms']:.0f} ms | latency {shown_result['latency_ms']:.0f} ms"
                cv2.rectangle(matched_frame, (0, 0), (matched_frame.shape[1], 30), (0, 0, 0), -1)
                cv2.putText(matched_frame, detection_status, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.imshow(detection_window, matched_frame)
                last_detection_display_revision = result_revision

            if cv2.waitKey(1) & 0xFF == ord('q'): break

        except Exception as e:
            print(f"[{CAM_NAME}] Error: {e}")
            time.sleep(0.01)

    stop_worker.set()
    with pending_condition: pending_condition.notify_all()
    worker.join(timeout=1.0)
    try:
        with detection_state_lock: detection_state["car"], detection_state["person"] = False, False
        car_detected_publisher.publish(Bool(data=False))
        person_detected_publisher.publish(Bool(data=False))
    except rospy.ROSException: pass
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
    parser.add_argument("--traffic-light-topic", default="/detection/traffic_light")
    parser.add_argument("--obstacle-topic", default="/detection/obstacle")
    parser.add_argument("--inference-size", type=int, default=INFERENCE_SIZE)
    parser.add_argument("--window-size", type=int, default=5, help="다수결 필터에 사용할 프레임 수")
    parser.add_argument("--display-fps", type=float, default=DISPLAY_FPS)
    parser.add_argument("--cpu-threads", type=int, default=CPU_THREADS)
    parser.add_argument("--show-raw-preview", type=int, choices=(0, 1), default=0)
    
    args = parser.parse_args()
    main(args.cam_ip, args.cam_port, args.base_model, args.custom_model,
         args.confidence, args.car_detected_topic, args.person_detected_topic,
         args.traffic_light_topic, args.obstacle_topic,
         args.inference_size, args.display_fps, args.cpu_threads,
         bool(args.show_raw_preview), args.window_size)
