#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
Pure Pursuit Controller
MORAI K-City 2025

Input
-----
current_x
current_y
current_yaw(rad)

global_path (list)

Output
------
steering angle (rad)

Author
------
Refactored from pure1.py
"""

import math
import numpy as np


class PurePursuitController:

    def __init__(self,
                 wheel_base=1.04,
                 lookahead_distance=3.0):

        self.wheel_base = wheel_base
        self.lookahead_distance = lookahead_distance

        ####################################################
        # Vehicle State
        ####################################################

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        ####################################################
        # Path
        ####################################################

        self.path = []

        self.current_index = 0

        self.finish = False

    ########################################################

    def update_pose(self,
                    x,
                    y,
                    yaw):

        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw

    ########################################################

    def update_path(self, path):

        if len(path) == 0:
            return

    # 기존 경로와 동일하면 갱신하지 않음
        if len(self.path) == len(path):
            return

        self.path = path
 
        self.current_index = 0
        self.finish = False

    ########################################################

    def pure_pursuit(self):

        ####################################################
        # Path Empty
        ####################################################

        if len(self.path) == 0:

            return 0.0

        ####################################################
        # Finish
        ####################################################

        if self.finish:

            return 0.0

        ####################################################
        # Current Position
        ####################################################

        XC = self.current_x
        YC = self.current_y

        ####################################################
        # Current Waypoint
        ####################################################

        while True:

            if self.current_index >= len(self.path):

                self.finish = True

                return 0.0

            XR = self.path[self.current_index][0]
            YR = self.path[self.current_index][1]

            distance = math.sqrt(
                (XR-XC)**2 +
                (YR-YC)**2
            )

            if distance > self.lookahead_distance:

                break

            self.current_index += 1

        ####################################################
        # Target Line
        ####################################################

        Cp = np.array([XC, YC])

        Rp = np.array([XR, YR])

        A, B, C = self.line_equation(
            Cp,
            Rp
        )

        TP1, TP2 = self.cl_intersect(
            Cp,
            self.lookahead_distance,
            A,
            B,
            C
        )

        ####################################################
        # Choose Target Point
        ####################################################

        d1 = math.sqrt(
            (XR-TP1[0])**2 +
            (YR-TP1[1])**2
        )

        d2 = math.sqrt(
            (XR-TP2[0])**2 +
            (YR-TP2[1])**2
        )

        if d1 < d2:

            XT = TP1[0]
            YT = TP1[1]

        else:

            XT = TP2[0]
            YT = TP2[1]

        ####################################################
        # Heading Error
        ####################################################

        bearing = math.atan2(
            YT-YC,
            XT-XC
        )

        alpha = self.wrap_to_pi(
            bearing -
            self.current_yaw
        )

        ####################################################
        # Steering Angle
        ####################################################

        sin_alpha = math.sin(alpha)

        if abs(sin_alpha) < 1e-8:
            return 0.0

        ####################################################
        # Pure Pursuit Steering
        ####################################################

        delta = math.atan2(
            2.0 * self.wheel_base * sin_alpha,
            self.lookahead_distance
        )

        delta = np.clip(
            delta,
            -0.52,
            0.52
        )

        return delta

    ########################################################
    # Utility Functions
    ########################################################

    def line_equation(self, cp, tp):

        x1 = float(cp[0])
        y1 = float(cp[1])

        x2 = float(tp[0])
        y2 = float(tp[1])

        # Ax + By + C = 0

        A = y1 - y2
        B = x2 - x1
        C = x1 * y2 - x2 * y1

        return A, B, C

    ########################################################

    def cl_intersect(self,
                     center,
                     radius,
                     A,
                     B,
                     C):

        ax = float(center[0])
        ay = float(center[1])

        den = A*A + B*B

        if den < 1e-10:

            return center, center

        norm = math.sqrt(den)

        t = np.array([
            -B,
             A
        ]) / norm

        p1 = center + radius * t
        p2 = center - radius * t

        return p1, p2

    ########################################################

    def wrap_to_pi(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    ########################################################

    def is_finished(self):

        return self.finish

    ########################################################

    def reset(self):

        self.current_index = 0
        self.finish = False

    ########################################################

    def set_lookahead(self, ld):

        self.lookahead_distance = ld

    ########################################################

    def get_current_index(self):

        return self.current_index
