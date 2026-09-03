#!/usr/bin/env python3
"""Compatibility command: run MORAI Stanley with speed-aided dead reckoning."""

from path_planning.stanley_udp_runtime import main


if __name__ == "__main__":
    main("dead-reckoning")
