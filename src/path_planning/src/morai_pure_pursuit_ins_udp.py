#!/usr/bin/env python3
"""Run MORAI Pure Pursuit with 15-state INS error-state EKF."""

from path_planning.pure_pursuit_udp_runtime import main


if __name__ == "__main__":
    main("ins")
