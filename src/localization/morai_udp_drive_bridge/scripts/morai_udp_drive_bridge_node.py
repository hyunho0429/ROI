#!/usr/bin/env python3
"""MORAI EgoVehicleStatus 수신 및 EgoCtrlCmd 송신 ROS 브릿지."""

from __future__ import annotations

import math
import socket
import threading
import time
from typing import Optional

import rospy
from geometry_msgs.msg import Vector3
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus

from morai_udp_drive_bridge.protocol import (
    EGO_STATUS_PACKET_SIZE,
    ProtocolError,
    build_ego_ctrl_cmd,
    parse_ego_vehicle_status,
)


class MoraiUdpDriveBridge:
    def __init__(self) -> None:
        rospy.init_node("morai_udp_drive_bridge", anonymous=False)

        self.status_bind_ip = rospy.get_param("~status_bind_ip", "0.0.0.0")
        self.status_port = int(rospy.get_param("~status_port", 909))
        self.status_topic = rospy.get_param("~status_topic", "/Ego_topic")
        self.status_frame_id = rospy.get_param("~status_frame_id", "map")
        self.control_remote_ip = rospy.get_param("~control_remote_ip", "192.168.0.151")
        self.control_remote_port = int(rospy.get_param("~control_remote_port", 9093))
        self.control_bind_ip = rospy.get_param("~control_bind_ip", "0.0.0.0")
        self.control_bind_port = int(rospy.get_param("~control_bind_port", 0))
        self.command_topic = rospy.get_param("~command_topic", "/ctrl_cmd")
        self.send_rate_hz = float(rospy.get_param("~send_rate_hz", 20.0))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 0.5))
        self.max_wheel_angle_rad = float(
            rospy.get_param("~max_wheel_angle_rad", math.radians(40.0))
        )
        self.status_use_packet_time = bool(rospy.get_param("~status_use_packet_time", False))

        self.status_pub = rospy.Publisher(self.status_topic, EgoVehicleStatus, queue_size=20)
        rospy.Subscriber(self.command_topic, CtrlCmd, self.command_callback, queue_size=20)

        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if self.control_bind_port > 0:
            self.send_socket.bind((self.control_bind_ip, self.control_bind_port))

        self.last_command: Optional[CtrlCmd] = None
        self.last_command_time = 0.0
        self.send_timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.send_rate_hz, 1.0)), self.send_timer_callback
        )

        self.stop_event = threading.Event()
        self.status_thread = threading.Thread(
            target=self.status_receive_loop,
            name="morai-ego-status-receiver",
            daemon=True,
        )
        self.status_thread.start()
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "MORAI UDP drive bridge: status bind=%s:%d topic=%s, control remote=%s:%d",
            self.status_bind_ip,
            self.status_port,
            self.status_topic,
            self.control_remote_ip,
            self.control_remote_port,
        )

    def status_receive_loop(self) -> None:
        receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        try:
            receive_socket.bind((self.status_bind_ip, self.status_port))
        except OSError as exc:
            rospy.logfatal(
                "EgoVehicleStatus UDP bind 실패 %s:%d: %s",
                self.status_bind_ip,
                self.status_port,
                exc,
            )
            receive_socket.close()
            return
        receive_socket.settimeout(0.5)
        rospy.loginfo(
            "EgoVehicleStatus UDP bind %s:%d expected=%d bytes",
            self.status_bind_ip,
            self.status_port,
            EGO_STATUS_PACKET_SIZE,
        )

        try:
            while not rospy.is_shutdown() and not self.stop_event.is_set():
                try:
                    packet, sender = receive_socket.recvfrom(EGO_STATUS_PACKET_SIZE)
                except socket.timeout:
                    continue
                except OSError as exc:
                    if not self.stop_event.is_set():
                        rospy.logerr_throttle(5.0, "EgoVehicleStatus UDP 수신 오류: %s", exc)
                    break

                try:
                    measurement = parse_ego_vehicle_status(packet)
                except (ProtocolError, ValueError) as exc:
                    rospy.logwarn_throttle(
                        5.0,
                        "EgoVehicleStatus 패킷 폐기 sender=%s:%d len=%d error=%s",
                        sender[0],
                        sender[1],
                        len(packet),
                        exc,
                    )
                    continue

                self.publish_status(measurement)
        finally:
            receive_socket.close()

    def publish_status(self, measurement) -> None:
        message = EgoVehicleStatus()
        if self.status_use_packet_time and measurement.sec > 0:
            message.header.stamp = rospy.Time(measurement.sec, measurement.nsec)
        else:
            message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.status_frame_id
        message.unique_id = 0
        message.acceleration = Vector3(
            measurement.acceleration_x_mps2,
            measurement.acceleration_y_mps2,
            measurement.acceleration_z_mps2,
        )
        message.position = Vector3(
            measurement.pos_x_m,
            measurement.pos_y_m,
            measurement.pos_z_m,
        )
        message.velocity = Vector3(
            measurement.velocity_x_kmh / 3.6,
            measurement.velocity_y_kmh / 3.6,
            measurement.velocity_z_kmh / 3.6,
        )
        message.heading = measurement.heading_deg
        message.accel = measurement.accel_pedal
        message.brake = measurement.brake_pedal
        message.wheel_angle = measurement.steer_deg
        self.status_pub.publish(message)
        rospy.loginfo_throttle(
            2.0,
            "Ego UDP pos=(%.2f, %.2f, %.2f) vel=(%.2f, %.2f) heading=%.2f deg link=%s",
            measurement.pos_x_m,
            measurement.pos_y_m,
            measurement.pos_z_m,
            measurement.velocity_x_kmh / 3.6,
            measurement.velocity_y_kmh / 3.6,
            measurement.heading_deg,
            measurement.link_id,
        )

    def command_callback(self, message: CtrlCmd) -> None:
        self.last_command = message
        self.last_command_time = time.monotonic()

    def send_timer_callback(self, _event) -> None:
        message = self.last_command
        is_fresh = message is not None and time.monotonic() - self.last_command_time <= self.command_timeout_sec

        if not is_fresh:
            packet = build_ego_ctrl_cmd(cmd_type=2, velocity_kmh=0.0, brake=1.0)
        else:
            cmd_type = int(getattr(message, "longlCmdType", 2))
            steering_rad = float(getattr(message, "steering", 0.0))
            steer_normalized = steering_rad / max(self.max_wheel_angle_rad, 1e-6)
            packet = build_ego_ctrl_cmd(
                cmd_type=cmd_type,
                velocity_kmh=max(0.0, float(getattr(message, "velocity", 0.0)) * 3.6),
                acceleration_mps2=float(getattr(message, "acceleration", 0.0)),
                accel=float(getattr(message, "accel", 0.0)),
                brake=float(getattr(message, "brake", 0.0)),
                steer_normalized=steer_normalized,
                ctrl_mode=2,
                gear=4,
            )

        try:
            self.send_socket.sendto(packet, (self.control_remote_ip, self.control_remote_port))
        except OSError as exc:
            rospy.logerr_throttle(5.0, "EgoCtrlCmd UDP 송신 오류: %s", exc)

    def shutdown(self) -> None:
        self.stop_event.set()
        try:
            stop_packet = build_ego_ctrl_cmd(cmd_type=2, velocity_kmh=0.0, brake=1.0)
            self.send_socket.sendto(stop_packet, (self.control_remote_ip, self.control_remote_port))
            self.send_socket.close()
        except OSError:
            pass


if __name__ == "__main__":
    try:
        MoraiUdpDriveBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
