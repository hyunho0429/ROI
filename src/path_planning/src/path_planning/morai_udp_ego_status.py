"""Ego Vehicle Status entry point for the shared MORAI status layout."""

from path_planning.morai_udp_competition_status import (
    CompetitionStatusPacketError,
    CompetitionVehicleStatus,
    parse_competition_vehicle_status,
)


EgoVehicleStatus = CompetitionVehicleStatus
EgoVehicleStatusPacketError = CompetitionStatusPacketError


def parse_ego_vehicle_status(packet):
    """Parse the shared 181-byte or extended 229-byte status layout."""
    return parse_competition_vehicle_status(packet)
