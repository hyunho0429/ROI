"""정지선을 지나 검출이 사라져도 완전 정지까지 제동을 유지한다."""

import math


class StoplineBrakeLatch:
    IDLE = "IDLE"
    BRAKING = "BRAKING"
    WAIT_CLEAR = "WAIT_CLEAR"

    def __init__(
        self,
        stopped_speed_mps=0.15,
        stop_hold_sec=1.0,
        rearm_clear_sec=1.0,
    ):
        self.stopped_speed_mps = float(stopped_speed_mps)
        self.stop_hold_sec = float(stop_hold_sec)
        self.rearm_clear_sec = float(rearm_clear_sec)
        if self.stopped_speed_mps < 0.0:
            raise ValueError("stopped_speed_mps must be non-negative")
        if self.stop_hold_sec < 0.0 or self.rearm_clear_sec < 0.0:
            raise ValueError("stop-line timing values must be non-negative")
        self.state = self.IDLE
        self.stopped_since = None
        self.clear_since = None

    def update(self, raw_stop_required, speed_mps, timestamp_sec):
        speed = max(0.0, float(speed_mps))
        now = float(timestamp_sec)
        if not math.isfinite(speed) or not math.isfinite(now):
            raise ValueError("stop-line latch inputs must be finite")

        if self.state == self.IDLE:
            if raw_stop_required:
                self.state = self.BRAKING
                self.stopped_since = None

        elif self.state == self.BRAKING:
            # 카메라에서 정지선이 사라져도 차량이 실제로 설 때까지 해제하지 않는다.
            if speed <= self.stopped_speed_mps:
                if self.stopped_since is None or now < self.stopped_since:
                    self.stopped_since = now
                if now - self.stopped_since >= self.stop_hold_sec:
                    self.state = self.WAIT_CLEAR
                    self.clear_since = None
            else:
                self.stopped_since = None

        elif self.state == self.WAIT_CLEAR:
            # 동일 정지선을 계속 보고 있을 때 즉시 재정지하지 않도록 재무장한다.
            if raw_stop_required:
                self.clear_since = None
            elif self.clear_since is None or now < self.clear_since:
                self.clear_since = now
            elif now - self.clear_since >= self.rearm_clear_sec:
                self.state = self.IDLE
                self.clear_since = None

        return self.state == self.BRAKING
