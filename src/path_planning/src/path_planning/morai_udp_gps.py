"""Small NMEA0183 RMC/GGA parser for MORAI GPS UDP datagrams."""

import math
from dataclasses import dataclass


KNOT_TO_MPS = 0.5144444444444445


class GpsPacketError(ValueError):
    """Raised when a GPS datagram contains no valid RMC/GGA sentence."""


@dataclass(frozen=True)
class GPSMessage:
    """Semantic mirror of beta_drive morai_msgs/GPSMessage (without Header)."""

    latitude: float
    longitude: float
    altitude: float
    eastOffset: float
    northOffset: float
    status: int


@dataclass(frozen=True)
class GpsMeasurement:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float = None
    speed_mps: float = None
    course_deg: float = None
    fix_valid: bool = True

    def to_beta_drive_message(self, east_offset_m, north_offset_m):
        """Add map constants absent from NMEA and expose GPSMessage fields."""
        if self.altitude_m is None:
            raise ValueError("GPSMessage conversion needs a merged GGA altitude")
        values = (
            self.latitude_deg,
            self.longitude_deg,
            self.altitude_m,
            float(east_offset_m),
            float(north_offset_m),
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("GPSMessage conversion contains a non-finite value")
        return GPSMessage(
            latitude=self.latitude_deg,
            longitude=self.longitude_deg,
            altitude=self.altitude_m,
            eastOffset=float(east_offset_m),
            northOffset=float(north_offset_m),
            status=1 if self.fix_valid else 0,
        )


def _verify_checksum(sentence):
    if not sentence.startswith("$"):
        return False
    if "*" not in sentence:
        return True
    body, checksum_text = sentence[1:].split("*", 1)
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    try:
        return checksum == int(checksum_text[:2], 16)
    except ValueError:
        return False


def _coordinate(value, hemisphere, latitude):
    if not value:
        raise ValueError("empty NMEA coordinate")
    degree_digits = 2 if latitude else 3
    degrees = float(value[:degree_digits])
    minutes = float(value[degree_digits:])
    coordinate = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        coordinate = -coordinate
    elif hemisphere not in ("N", "E"):
        raise ValueError("invalid NMEA hemisphere")
    return coordinate


def _parse_sentence(sentence):
    if not _verify_checksum(sentence):
        return None
    payload = sentence[1:].split("*", 1)[0]
    fields = payload.split(",")
    message_type = fields[0][-3:]
    try:
        if message_type == "GGA" and len(fields) >= 10:
            fix_valid = int(fields[6] or "0") > 0
            latitude = _coordinate(fields[2], fields[3], True)
            longitude = _coordinate(fields[4], fields[5], False)
            # MORAI Denied Area blackout is documented to output zeroes.
            fix_valid = fix_valid and not (latitude == 0.0 and longitude == 0.0)
            return GpsMeasurement(
                latitude,
                longitude,
                altitude_m=float(fields[9]) if fields[9] else None,
                fix_valid=fix_valid,
            )
        if message_type == "RMC" and len(fields) >= 9:
            fix_valid = fields[2] == "A"
            latitude = _coordinate(fields[3], fields[4], True)
            longitude = _coordinate(fields[5], fields[6], False)
            fix_valid = fix_valid and not (latitude == 0.0 and longitude == 0.0)
            return GpsMeasurement(
                latitude,
                longitude,
                speed_mps=float(fields[7]) * KNOT_TO_MPS if fields[7] else None,
                course_deg=float(fields[8]) if fields[8] else None,
                fix_valid=fix_valid,
            )
    except (ValueError, IndexError):
        return None
    return None


def parse_nmea_datagram(packet):
    """Merge valid RMC/GGA sentences from one UDP datagram."""
    text = packet.decode("ascii", errors="ignore").replace("\x00", "")
    measurements = []
    for line in text.replace("\r", "\n").split("\n"):
        line = line.strip()
        if line:
            measurement = _parse_sentence(line)
            if measurement is not None:
                measurements.append(measurement)
    if not measurements:
        raise GpsPacketError("datagram contains no valid RMC/GGA sentence")

    valid = [measurement for measurement in measurements if measurement.fix_valid]
    if not valid:
        last = measurements[-1]
        return GpsMeasurement(last.latitude_deg, last.longitude_deg, fix_valid=False)
    latest = valid[-1]
    altitude = next((item.altitude_m for item in reversed(valid) if item.altitude_m is not None), None)
    speed = next((item.speed_mps for item in reversed(valid) if item.speed_mps is not None), None)
    course = next((item.course_deg for item in reversed(valid) if item.course_deg is not None), None)
    numeric = [latest.latitude_deg, latest.longitude_deg]
    numeric.extend(value for value in (altitude, speed, course) if value is not None)
    if not all(math.isfinite(value) for value in numeric):
        raise GpsPacketError("GPS datagram contains a non-finite value")
    return GpsMeasurement(
        latest.latitude_deg,
        latest.longitude_deg,
        altitude_m=altitude,
        speed_mps=speed,
        course_deg=course,
        fix_valid=True,
    )
