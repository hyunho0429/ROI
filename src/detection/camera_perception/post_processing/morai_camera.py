"""MORAI 시뮬레이터 카메라 UDP 수신 — live_overlay.py / live_output.py 공용.

패키지 공용 `LatestCameraReceiver`가 UDP 조각을 조립하고 최신 완성 프레임만
보관한다. 이 래퍼는 JPEG 디코딩 결과와 촬영 시각을 차선 후처리에 전달한다.
단독 `camera_perception.launch`와 통합 launch가 같은 검증된 수신 경로를 쓴다.
"""

import os
import threading
import time

import cv2
import numpy as np

from camera_perception.camera_udp import LatestCameraReceiver

# **이 IP 는 시뮬레이터 주소가 아니라 이쪽에서 bind 하는 로컬 주소다.**
# Receiver 가 socket.bind((ip, port)) 를 하기 때문에 시뮬레이터 PC 의 IP 를
# 넣으면 "Cannot assign requested address" 로 죽는다. 어느 인터페이스로
# 들어오든 받도록 0.0.0.0 으로 둔다.
DEFAULT_IP = os.environ.get("MORAI_CAM_IP", "0.0.0.0")
DEFAULT_PORT = int(os.environ.get("MORAI_CAM_PORT", "1101"))


class CameraStream:
    """최신 프레임 하나만 유지하는 UDP 카메라 수신기.

        cam = CameraStream().start()
        frame, seq = cam.latest()          # 아직 없으면 (None, -1)
    """

    def __init__(self, ip=DEFAULT_IP, port=DEFAULT_PORT):
        self.ip, self.port = ip, port
        self._frame = None
        self._seq = -1
        self._stamp = 0.0               # 패킷에 실린 촬영 시각 (초)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self.decode_errors = 0

    def start(self):
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()

    def latest(self, with_stamp=False):
        """최신 프레임. `with_stamp=True` 면 (frame, seq, stamp) 를 준다.

        `stamp` 는 시뮬레이터가 패킷에 넣은 **촬영 시각**(초, `time.time()` 과
        같은 epoch)이다. 받은 시각이 아니다.

        이 둘을 구분해야 하는 이유는 실측 때문이다 - 촬영에서 수신까지
        중앙값 128ms, p90 145ms 가 걸린다. IMU 같은 다른 센서를 이 프레임에
        맞출 때 "지금 시각" 을 쓰면 128ms 만큼 미래의 값을 쓰게 된다.
        요레이트 5도/s 인 완만한 커브에서도 0.64도 차이이고, 지면 투영에서
        자세 1도는 40m 에서 거리 45% 오차다 (s04 주석 실측).

        기본값을 False 로 둔 것은 기존 호출부(`frame, seq = cam.latest()`)를
        깨지 않기 위해서다.
        """
        with self._lock:
            if self._frame is None:
                return (None, -1, 0.0) if with_stamp else (None, -1)
            if with_stamp:
                return self._frame.copy(), self._seq, self._stamp
            return self._frame.copy(), self._seq

    def wait_first(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.latest()[0] is not None:
                return True
            time.sleep(0.05)
        return False

    def _worker(self):
        receiver = LatestCameraReceiver(self.ip, self.port)
        sequence = 0
        try:
            while not self._stop.is_set():
                frame = receiver.wait_for_latest(sequence, timeout=0.1)
                if frame is None:
                    continue
                sequence = frame.sequence
                buf = np.frombuffer(frame.jpeg_data, dtype=np.uint8)
                image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if image is None or image.size == 0:
                    self.decode_errors += 1
                    continue
                with self._lock:
                    self._stamp = float(frame.sec) + float(frame.nsec) * 1e-9
                    self._frame = image
                    self._seq = sequence
        except (AttributeError, ValueError, OSError, cv2.error) as ex:
            print(f"[camera] 복구 가능한 오류: {ex}")
        finally:
            receiver.close()
