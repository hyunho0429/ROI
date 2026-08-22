"""Low-latency MORAI camera UDP receiver.

The generic UDP receiver copies every camera packet through an unbounded
multiprocessing queue. A camera frame is split across multiple packets, so a
slow consumer can end up processing a large backlog of old packets. This
receiver assembles frames in a dedicated thread and retains only the newest
complete JPEG frame.
"""

from dataclasses import dataclass
import ctypes
import socket
import threading

from lib.define.Camera import Camera


@dataclass(frozen=True)
class CameraFrame:
    """Immutable snapshot of one complete MORAI JPEG frame."""

    sequence: int
    sec: int
    nsec: int
    index: int
    jpeg_data: bytes


class LatestCameraReceiver:
    """Receive MORAI camera UDP data without accumulating stale frames."""

    def __init__(self, ip, port, receive_buffer_bytes=4 * 1024 * 1024):
        self._latest_frame = None
        self._sequence = 0
        self._closed = False
        self._condition = threading.Condition()

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(
            socket.SOL_SOCKET, socket.SO_RCVBUF, int(receive_buffer_bytes)
        )
        try:
            self.socket.bind((ip, int(port)))
        except Exception:
            self._closed = True
            self.socket.close()
            raise

        self._packet = Camera()
        self._packet_size = ctypes.sizeof(self._packet)

        self._thread = threading.Thread(
            target=self._receive_loop,
            name=f"morai-camera-{port}",
            daemon=True,
        )
        self._thread.start()

    def _receive_loop(self):
        while not self._closed:
            try:
                raw_data, _ = self.socket.recvfrom(self._packet_size)
            except OSError:
                if self._closed:
                    return
                continue

            if not raw_data:
                continue

            ctypes.memmove(
                ctypes.addressof(self._packet), raw_data, len(raw_data)
            )

            try:
                self._packet.parsing()
            except (AttributeError, UnicodeDecodeError, ValueError):
                continue

            # Only a MOR packet ending in EI completes a JPEG frame. BOX and
            # intermediate MOR packets must never wake the inference loop.
            if bytes(self._packet.header) != b"MOR":
                continue
            if bytes(self._packet.image.tail) != b"EI":
                continue

            jpeg_data = bytes(self._packet.image.data)
            if not jpeg_data:
                continue

            with self._condition:
                self._sequence += 1
                self._latest_frame = CameraFrame(
                    sequence=self._sequence,
                    sec=int(self._packet.image.sec),
                    nsec=int(self._packet.image.nsec),
                    index=int(self._packet.image.index),
                    jpeg_data=jpeg_data,
                )
                self._condition.notify_all()

    def wait_for_latest(self, after_sequence=0, timeout=0.1):
        """Return a newer frame, skipping any intermediate stale frames."""

        with self._condition:
            self._condition.wait_for(
                lambda: self._closed
                or (
                    self._latest_frame is not None
                    and self._latest_frame.sequence > after_sequence
                ),
                timeout=timeout,
            )
            if self._closed or self._latest_frame is None:
                return None
            if self._latest_frame.sequence <= after_sequence:
                return None
            return self._latest_frame

    def close(self):
        condition = getattr(self, "_condition", None)
        if condition is None:
            return
        with condition:
            if self._closed:
                return
            self._closed = True
            condition.notify_all()
        try:
            self.socket.close()
        except (AttributeError, OSError):
            pass

    def __del__(self):
        self.close()
