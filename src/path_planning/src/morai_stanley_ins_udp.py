#!/usr/bin/env python3
"""Run MORAI Stanley control with 15-state INS error-state EKF."""

from path_planning.stanley_udp_runtime import main


if __name__ == "__main__":
    main("ins")
