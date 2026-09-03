#!/usr/bin/env python3

import math
import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))
from lib.network.UDP import Receiver
from lib.define.GPS import GPS
from lib.define.IMU import IMU

MORAI_IP = '192.168.0.200'
GPS_PORT = 3001
IMU_PORT = 4001

class EKFEstimator:
    def __init__(self):
        # 상태 벡터 [x, y, vx, vy, yaw, yaw_rate]^T
        self.x = np.zeros((6, 1))
        self.P = np.eye(6) * 1.0
        self.Q = np.diag([0.01, 0.01, 0.05, 0.05, 0.01, 0.01])
        self.R = np.diag([0.5, 0.5])  # GPS 측정 노이즈
        
        # 센서 수신기 초기화
        self.sensor_gps = Receiver(MORAI_IP, GPS_PORT, GPS())
        self.sensor_imu = Receiver(MORAI_IP, IMU_PORT, IMU())
        
        self.is_initialized = False

    def initialize_pose(self, init_x, init_y, init_yaw):
        self.x[0, 0] = init_x
        self.x[1, 0] = init_y
        self.x[4, 0] = init_yaw
        self.is_initialized = True

    def spin_once(self, dt):
        """
        주기가 될 때마다 센서 데이터를 읽어와서 EKF 예측 및 업데이트를 혼자서 수행
        """
        if not self.is_initialized:
            return

        # 1. IMU 데이터 수신 및 Predict 수행
        imu_data = self.sensor_imu.get_data()
        if imu_data:
            accel_x = getattr(imu_data, 'lin_acc_x', 0.0)
            accel_y = getattr(imu_data, 'lin_acc_y', 0.0)
            yaw_rate = getattr(imu_data, 'ang_vel_z', 0.0)
        else:
            accel_x, accel_y, yaw_rate = 0.0, 0.0, 0.0

        # Predict 단계
        px, py, vx, vy, yaw = self.x[0,0], self.x[1,0], self.x[2,0], self.x[3,0], self.x[4,0]
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

        self.x[0, 0] = px + (vx * cos_yaw - vy * sin_yaw) * dt
        self.x[1, 0] = py + (vx * sin_yaw + vy * cos_yaw) * dt
        self.x[2, 0] = vx + accel_x * dt
        self.x[3, 0] = vy + accel_y * dt
        self.x[4, 0] = yaw + yaw_rate * dt
        self.x[5, 0] = yaw_rate
        self.x[4, 0] = math.atan2(math.sin(self.x[4, 0]), math.cos(self.x[4, 0]))

        F = np.eye(6)
        F[0, 2] = cos_yaw * dt
        F[0, 3] = -sin_yaw * dt
        F[0, 4] = (-vx * sin_yaw - vy * cos_yaw) * dt
        F[1, 2] = sin_yaw * dt
        F[1, 3] = cos_yaw * dt
        F[1, 4] = (vx * cos_yaw - vy * sin_yaw) * dt
        self.P = F @ self.P @ F.T + self.Q

        # 2. GPS 데이터 수신 및 Update 수행

        # 2. GPS 데이터 수신 및 Update 수행
        gps_data = self.sensor_gps.get_data()
        if gps_data:
            gps_data.parsing()
            if gps_data.gpgga.lat != 0.0:
                lat = gps_data.gpgga.lat
                lon = gps_data.gpgga.lon
                

                EAST_OFFSET = 302595.0
                NORTH_OFFSET = 4124145.0
                

                scale_lat = 111319.490793
                scale_lon = 111319.490793 * math.cos(math.radians(lat))
                

                if not hasattr(self, 'ref_lat') or self.ref_lat is None:
                    self.ref_lat = lat
                    self.ref_lon = lon

                # 첫 위치를 (0, 0)으로 맞추는 상대 미터 좌표계 변환 방식 (경로 파일과 맞추기 가장 안전함)
                gps_y = (lat - self.ref_lat) * scale_lat
                gps_x = (lon - self.ref_lon) * scale_lon

                H = np.zeros((2, 6))
                H[0, 0] = 1.0
                H[1, 1] = 1.0

                z = np.array([[gps_x], [gps_y]])
                z_pred = H @ self.x
                y = z - z_pred

                S = H @ self.P @ H.T + self.R
                K = self.P @ H.T @ np.linalg.inv(S)

                self.x = self.x + K @ y
                self.P = (np.eye(6) - K @ H) @ self.P
                self.x[4, 0] = math.atan2(math.sin(self.x[4, 0]), math.cos(self.x[4, 0]))
    def get_states(self):
        return {
            'x': float(self.x[0, 0]),
            'y': float(self.x[1, 0]),
            'vx': float(self.x[2, 0]),
            'vy': float(self.x[3, 0]),
            'yaw_deg': float(math.degrees(self.x[4, 0])),
            'yaw_rate': float(self.x[5, 0])
        }
