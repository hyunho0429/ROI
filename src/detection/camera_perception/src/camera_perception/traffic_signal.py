"""YOLO 신호등 클래스명을 주행/정지 조건으로 변환한다."""

import math


YELLOW_SIGNAL_KEYWORDS = ("yellow", "amber")
TURN_SIGNAL_KEYWORDS = ("left", "right", "arrow", "좌회전", "우회전")


def traffic_signal_requires_stop(class_names):
    """단독 RED 또는 Yellow 계열 클래스가 있으면 True를 반환한다.

    RED는 같은 프레임에 좌·우회전 계열 신호가 없어야 정지 조건이다.
    Yellow/Amber 계열은 조합 클래스도 정지 조건으로 처리한다.
    """

    normalized_names = [str(name).strip().lower() for name in class_names]
    yellow_detected = any(
        keyword in name
        for name in normalized_names
        for keyword in YELLOW_SIGNAL_KEYWORDS
    )
    if yellow_detected:
        return True

    red_detected = "red" in normalized_names
    turn_signal_detected = any(
        keyword in name
        for name in normalized_names
        for keyword in TURN_SIGNAL_KEYWORDS
    )
    return red_detected and not turn_signal_detected


class TrafficSignalStopLatch:
    """단독 RED/Yellow는 즉시 정지하고 연속 정상 검출 뒤에만 해제한다."""

    def __init__(self, clear_confirmation_s=0.5):
        self.clear_confirmation_s = float(clear_confirmation_s)
        if self.clear_confirmation_s < 0.0:
            raise ValueError("clear_confirmation_s must be non-negative")
        self.stop_required = False
        self.clear_since = None

    def update(self, class_names, timestamp_sec):
        now = float(timestamp_sec)
        if not math.isfinite(now):
            raise ValueError("timestamp_sec must be finite")
        if traffic_signal_requires_stop(class_names):
            self.stop_required = True
            self.clear_since = None
        elif self.stop_required:
            if self.clear_since is None or now < self.clear_since:
                self.clear_since = now
            if now - self.clear_since >= self.clear_confirmation_s:
                self.stop_required = False
                self.clear_since = None
        return self.stop_required
