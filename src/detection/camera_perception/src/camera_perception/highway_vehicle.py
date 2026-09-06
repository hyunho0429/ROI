"""Shared camera vehicle-class selection for driving-situation gates."""


# The competition model was trained with every relevant vehicle type folded
# into one ``car`` class.  Both the highway and intersection gates therefore
# consume the same single semantic class instead of assigning different
# meanings to bus/train/truck labels.
HIGHWAY_VEHICLE_CLASSES = frozenset(("car",))


def highway_vehicle_detected(labels):
    """Return true only when the unified competition ``car`` class is present."""
    normalized = {str(label).strip().lower() for label in labels}
    return bool(normalized.intersection(HIGHWAY_VEHICLE_CLASSES))
