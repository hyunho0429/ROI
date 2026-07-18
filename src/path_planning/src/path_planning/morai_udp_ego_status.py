"""Parser entry point for the documented 181-byte MORAI Ego Vehicle Status."""

from path_planning.morai_udp_competition_status import (
    BASE_PACKET_SIZE,
    CompetitionStatusPacketError,
    CompetitionVehicleStatus,
    parse_competition_vehicle_status,
)


EgoVehicleStatus = CompetitionVehicleStatus
EgoVehicleStatusPacketError = CompetitionStatusPacketError


def parse_ego_vehicle_status(packet):
    """Parse only the documented 181-byte Ego Vehicle Status layout."""
    if len(packet) != BASE_PACKET_SIZE:
        raise EgoVehicleStatusPacketError(
            "Ego Vehicle Status requires 181 bytes, received {} (header={!r})".format(
                len(packet), packet[:15]
            )
        )
    return parse_competition_vehicle_status(packet)
