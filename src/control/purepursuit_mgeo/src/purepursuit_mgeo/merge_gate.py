"""Fail-safe driving gate driven by the merge-gap perception status."""


VALID_SIDES = ("left", "right", "either")


class MergeDecisionGate:
    """Convert merge perception messages into a stop/continue decision.

    This class deliberately does not generate a lane-change trajectory.  When
    enabled, it only permits the existing path follower to continue if the
    configured adjacent side is confirmed available.
    """

    def __init__(self, target_side, timeout_s):
        self.target_side = str(target_side).strip().lower()
        self.timeout_s = float(timeout_s)
        if self.target_side not in VALID_SIDES:
            raise ValueError("merge target side must be left, right, or either")
        if self.timeout_s <= 0.0:
            raise ValueError("merge status timeout must be positive")
        self.received_at_s = None
        self.valid = False
        self.left_available = False
        self.right_available = False

    def update(self, valid, left_available, right_available, received_at_s):
        self.valid = bool(valid)
        self.left_available = bool(left_available)
        self.right_available = bool(right_available)
        self.received_at_s = float(received_at_s)

    def stop_required(self, now_s):
        if self.received_at_s is None:
            return True, "waiting_for_merge_status"
        age_s = max(0.0, float(now_s) - self.received_at_s)
        if age_s > self.timeout_s:
            return True, "merge_status_stale"
        if not self.valid:
            return True, "merge_status_invalid"
        if self.target_side == "left":
            available = self.left_available
        elif self.target_side == "right":
            available = self.right_available
        else:
            available = self.left_available or self.right_available
        return (False, "merge_available") if available else (True, "merge_blocked")
