"""Pure state logic for the highway-environment gate."""


class HighwayEnvironmentLatch:
    """Optionally keep the highway state active after its first detection."""

    def __init__(self, latch_once=True):
        self.latch_once = bool(latch_once)
        self.latched = False

    def update(self, conditions_met):
        conditions_met = bool(conditions_met)
        if conditions_met:
            self.latched = True
        return self.latched if self.latch_once else conditions_met
