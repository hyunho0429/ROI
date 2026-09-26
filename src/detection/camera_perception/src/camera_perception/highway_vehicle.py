"""Shared camera vehicle-class selection for driving-situation gates."""


# Road vehicles that can establish the camera-side highway condition.  These
# are the same COCO classes requested from the base detector.
HIGHWAY_VEHICLE_CLASSES = frozenset(("car", "bus", "truck"))


def highway_vehicle_detected(labels):
    """Return true when a car, bus or truck is present."""
    normalized = {str(label).strip().lower() for label in labels}
    return bool(normalized.intersection(HIGHWAY_VEHICLE_CLASSES))
