#!/usr/bin/env python3
"""roslaunch wrapper for camera_perception/lane/live_lane_info_publisher_v2.py.

The sensor-team handover currently starts that script manually in a second
terminal.  This wrapper locates it inside the camera_perception package and
execs it, so the final avoidance launch can bring the geometry publisher up in
one command without modifying the sensor team's camera launch chain.
"""

import os
from pathlib import Path
import sys

import rospkg


def main() -> None:
    package_root = Path(rospkg.RosPack().get_path("camera_perception"))
    candidates = [
        package_root / "lane" / "live_lane_info_publisher_v2.py",
        package_root / "scripts" / "live_lane_info_publisher_v2.py",
        package_root / "live_lane_info_publisher_v2.py",
    ]
    target = next((p for p in candidates if p.is_file()), None)
    if target is None:
        raise SystemExit(
            "live_lane_info_publisher_v2.py not found in camera_perception. "
            "Copy the supplied file to <camera_perception>/lane/."
        )
    os.chdir(str(target.parent))
    os.execv(sys.executable, [sys.executable, str(target)] + sys.argv[1:])


if __name__ == "__main__":
    main()
