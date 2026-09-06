"""Shared COCO vehicle-class selection for the highway camera gate."""


HIGHWAY_VEHICLE_CLASSES = frozenset(
    ("car", "bus", "truck", "motorcycle", "bicycle")
)


def highway_vehicle_detected(labels):
    """Return true when a raw YOLO label normalizes to the unified Car class."""
    normalized = {str(label).strip().lower() for label in labels}
    return bool(normalized.intersection(HIGHWAY_VEHICLE_CLASSES))
