"""Shared COCO vehicle-class selection for the highway camera gate."""


HIGHWAY_VEHICLE_CLASSES = frozenset(("car", "bus", "truck"))


def highway_vehicle_detected(labels):
    """Return true when any configured road-vehicle class is present."""
    normalized = {str(label).strip().lower() for label in labels}
    return bool(normalized.intersection(HIGHWAY_VEHICLE_CLASSES))
