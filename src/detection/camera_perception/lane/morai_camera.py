"""MORAI 시뮬레이터 카메라 UDP 수신 — live_overlay.py / live_output.py 공용.

`RecordDrive.camera_worker` 와 같은 방식이다. 별도 스레드가 최신 프레임 하나만
들고 있고, 본 루프는 필요할 때 그걸 집어간다 (프레임 큐를 쌓지 않는다 —
실시간 처리에서는 늦은 프레임을 미루는 것보다 버리는 게 맞다).

**같은 프레임을 다시 디코드하지 않는다.** `Receiver.get_data()` 는 폴링마다
같은 객체를 돌려주는데(초당 150회 이상), 키를 안 보고 매번 imdecode 하면
같은 1280x720 JPEG 을 계속 다시 푸느라 CPU 를 다 쓴다 (RecordDrive 에서
녹화가 0.6 FPS 까지 떨어졌던 원인).
"""

import os
import threading
import time

import cv2
import numpy as np

from camera_perception.camera_udp import LatestCameraReceiver

# **이 IP 는 시뮬레이터 주소가 아니라 이쪽에서 bind 하는 로컬 주소다.**
# Receiver 가 socket.bind((ip, port)) 를 하기 때문에, 시뮬레이터 PC 의 IP
# (예: 192.168.0.161) 를 넣으면 "Cannot assign requested address" 로 죽는다.
# 시뮬레이터가 어느 인터페이스로 쏘든 받도록 0.0.0.0 으로 둔다. 보내는 쪽
# 주소는 시뮬레이터의 센서 UDP destination 설정에서 이 PC 를 가리켜야 한다.
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

    def latest(self):
        with self._lock:
            if self._frame is None:
                return None, -1
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
                    self._frame = image
                    self._seq = sequence
        except (AttributeError, ValueError, OSError, cv2.error) as ex:
            print(f"[camera] 복구 가능한 오류: {ex}")
        finally:
            receiver.close()
