#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MORAI GPS·IMU UDP 수신 노드.

기본 출력 토픽:
  - /gps : morai_msgs/GPSMessage
  - /Imu : sensor_msgs/Imu

UDP 포트는 MORAI PC에서 전송 대상으로 설정한 Ubuntu 포트와 일치해야 한다.
현재 팀 기준값은 GPS 3001, IMU 4001이다.
"""

from __future__ import annotations

from dataclasses import dataclass
import socket
import struct
import threading
import time
from typing import Callable, Optional

import rospy
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu

from morai_udp_bridge.protocol import (
    GPS_PACKET_SIZE,
    IMU_PACKET_SIZE,
    GpsMeasurement,
    ImuMeasurement,
    ProtocolError,
    normalize_quaternion,
    parse_gps_packet,
    parse_imu_packet,
)


@dataclass
class ReceiverStats:
    packets: int = 0
    parsed: int = 0
    rejected: int = 0
    last_packet_time: float = 0.0
    last_error: str = ""


class UdpReceiver(threading.Thread):
    """UDP 수신을 담당하며 파싱과 ROS publish를 분리한다."""

    def __init__(
        self,
        *,
        name: str,
        bind_ip: str,
        port: int,
        recv_size: int,
        parser: Callable[[bytes], object],
        callback: Callable[[object], None],
        stats: ReceiverStats,
    ) -> None:
        super().__init__(name=name, daemon=True)
        self.name = name
        self.bind_ip = bind_ip
        self.port = port
        self.recv_size = recv_size
        self.parser = parser
        self.callback = callback
        self.stats = stats
        self._stop_event = threading.Event()
        self._socket: Optional[socket.socket] = None

    def stop(self) -> None:
        self._stop_event.set()
        if self._socket is not None:
            self._socket.close()

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket = sock
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
            sock.bind((self.bind_ip, self.port))
            sock.settimeout(0.5)
            rospy.loginfo("[%s] UDP bind %s:%d, recv_size=%d", self.name, self.bind_ip, self.port, self.recv_size)

            while not rospy.is_shutdown() and not self._stop_event.is_set():
                try:
                    packet, sender = sock.recvfrom(self.recv_size)
                except socket.timeout:
                    continue
                except OSError as exc:
                    if not self._stop_event.is_set() and not rospy.is_shutdown():
                        rospy.logerr_throttle(5.0, "[%s] socket 수신 오류: %s", self.name, exc)
                    break

                self.stats.packets += 1
                self.stats.last_packet_time = time.monotonic()
                try:
                    parsed = self.parser(packet)
                except (ProtocolError, UnicodeError, struct.error) as exc:
                    self.stats.rejected += 1
                    self.stats.last_error = str(exc)
                    rospy.logwarn_throttle(
                        5.0,
                        "[%s] 패킷 폐기 sender=%s:%d len=%d error=%s",
                        self.name,
                        sender[0],
                        sender[1],
                        len(packet),
                        exc,
                    )
                    continue

                self.stats.parsed += 1
                self.callback(parsed)
        finally:
            try:
                sock.close()
            except OSError:
                pass
            self._socket = None

class MoraiUdpBridge:
    def __init__(self) -> None:
        rospy.init_node("morai_udp_bridge", anonymous=False)

        bind_ip = rospy.get_param("~bind_ip", "0.0.0.0")
        gps_port = int(rospy.get_param("~gps_port", 3001))
        imu_port = int(rospy.get_param("~imu_port", 4001))
        gps_topic = rospy.get_param("~gps_topic", "/gps")
        imu_topic = rospy.get_param("~imu_topic", "/Imu")
        gps_frame = rospy.get_param("~gps_frame_id", "gps")
        imu_frame = rospy.get_param("~imu_frame_id", "imu_link")
        east_offset = float(rospy.get_param("~east_offset", 0.0))
        north_offset = float(rospy.get_param("~north_offset", 0.0))
        validate_checksum = bool(rospy.get_param("~gps_validate_checksum", True))
        use_packet_time = bool(rospy.get_param("~imu_use_packet_time", True))
        normalize_orientation = bool(rospy.get_param("~imu_normalize_orientation", True))
        allow_legacy_107 = bool(rospy.get_param("~imu_allow_legacy_107", True))

        self.gps_frame = gps_frame
        self.imu_frame = imu_frame
        self.east_offset = east_offset
        self.north_offset = north_offset
        self.use_packet_time = use_packet_time
        self.normalize_orientation = normalize_orientation
        self.allow_legacy_107 = allow_legacy_107
        self.last_gga_altitude: Optional[float] = None
        self.last_gps_status = 0
        self.last_imu_layout = ""

        self.gps_pub = rospy.Publisher(gps_topic, GPSMessage, queue_size=20)
        self.imu_pub = rospy.Publisher(imu_topic, Imu, queue_size=50)

        self.gps_stats = ReceiverStats()
        self.imu_stats = ReceiverStats()

        self.gps_receiver = UdpReceiver(
            name="GPS",
            bind_ip=bind_ip,
            port=gps_port,
            recv_size=GPS_PACKET_SIZE,
            parser=lambda packet: parse_gps_packet(
                packet,
                expected_packet_size=GPS_PACKET_SIZE,
                validate_checksum=validate_checksum,
                allow_short_packet=True,
            ),
            callback=self.publish_gps,
            stats=self.gps_stats,
        )
        self.imu_receiver = UdpReceiver(
            name="IMU",
            bind_ip=bind_ip,
            port=imu_port,
            recv_size=IMU_PACKET_SIZE,
            parser=lambda packet: parse_imu_packet(
                packet,
                expected_packet_size=IMU_PACKET_SIZE,
                allow_legacy_107=allow_legacy_107,
            ),
            callback=self.publish_imu,
            stats=self.imu_stats,
        )

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "MORAI UDP bridge 시작: bind=%s GPS=%d->%s IMU=%d->%s",
            bind_ip,
            gps_port,
            gps_topic,
            imu_port,
            imu_topic,
        )

    def publish_gps(self, measurement: object) -> None:
        gps = measurement
        if not isinstance(gps, GpsMeasurement):
            return

        if gps.altitude is not None:
            self.last_gga_altitude = gps.altitude
        altitude = self.last_gga_altitude if self.last_gga_altitude is not None else 0.0
        self.last_gps_status = gps.status

        message = GPSMessage()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.gps_frame
        message.latitude = gps.latitude
        message.longitude = gps.longitude
        message.altitude = altitude
        message.eastOffset = self.east_offset
        message.northOffset = self.north_offset
        message.status = gps.status
        self.gps_pub.publish(message)

        rospy.loginfo_throttle(
            2.0,
            "GPS %s lat=%.8f lon=%.8f alt=%.3f status=%d packet=%d",
            gps.sentence_type,
            gps.latitude,
            gps.longitude,
            altitude,
            gps.status,
            gps.packet_length,
        )

    def publish_imu(self, measurement: object) -> None:
        imu = measurement
        if not isinstance(imu, ImuMeasurement):
            return

        message = Imu()
        if self.use_packet_time and imu.sec > 0:
            message.header.stamp = rospy.Time(imu.sec, imu.nsec)
        else:
            message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.imu_frame

        quaternion = imu.quaternion
        if self.normalize_orientation:
            quaternion = normalize_quaternion(quaternion)
        w, x, y, z = quaternion
        message.orientation.x = x
        message.orientation.y = y
        message.orientation.z = z
        message.orientation.w = w
        message.angular_velocity.x = imu.angular_velocity[0]
        message.angular_velocity.y = imu.angular_velocity[1]
        message.angular_velocity.z = imu.angular_velocity[2]
        message.linear_acceleration.x = imu.linear_acceleration[0]
        message.linear_acceleration.y = imu.linear_acceleration[1]
        message.linear_acceleration.z = imu.linear_acceleration[2]
        self.imu_pub.publish(message)

        if self.last_imu_layout != imu.layout:
            self.last_imu_layout = imu.layout
            rospy.logwarn(
                "IMU packet layout=%s length=%d data_length=%d",
                imu.layout,
                imu.packet_length,
                imu.data_length,
            )
        rospy.loginfo_throttle(
            2.0,
            "IMU q=(%.5f,%.5f,%.5f,%.5f) gyro=(%.4f,%.4f,%.4f) accel=(%.4f,%.4f,%.4f)",
            w,
            x,
            y,
            z,
            *imu.angular_velocity,
            *imu.linear_acceleration,
        )

    def shutdown(self) -> None:
        self.gps_receiver.stop()
        self.imu_receiver.stop()

    def spin(self) -> None:
        self.gps_receiver.start()
        self.imu_receiver.start()
        rospy.spin()


if __name__ == "__main__":
    try:
        MoraiUdpBridge().spin()
    except rospy.ROSInterruptException:
        pass
