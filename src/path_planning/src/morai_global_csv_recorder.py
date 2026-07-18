#!/usr/bin/env python3
"""Record the MORAI ego vehicle's ENU path at uniform 3-D intervals."""

import datetime
import math
import os
import threading

import rospy
from morai_msgs.msg import EgoVehicleStatus

from path_planning.csv_path_writer import CsvPathWriter, resample_segment


def _as_bool(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class MoraiGlobalCsvRecorder:
    def __init__(self):
        self._topic = rospy.get_param("~ego_topic", "/Ego_topic")
        self._sample_distance = float(rospy.get_param("~sample_distance", 0.5))
        self._append = _as_bool(rospy.get_param("~append", False))
        self._output_file = os.path.abspath(
            os.path.expanduser(
                rospy.get_param(
                    "~output_file",
                    "~/morai_paths/morai_global_path.csv",
                )
            )
        )

        if self._sample_distance <= 0.0:
            raise ValueError("~sample_distance must be greater than zero")

        self._lock = threading.RLock()
        self._previous_sample = None
        self._distance_to_next = self._sample_distance
        self._sequence = 0
        self._closed = False

        self._writer = CsvPathWriter(self._output_file, append=self._append)
        rospy.on_shutdown(self.close)
        self._subscriber = rospy.Subscriber(
            self._topic,
            EgoVehicleStatus,
            self._status_callback,
            queue_size=1,
        )
        rospy.loginfo("MORAI global path CSV recorder started")
        rospy.loginfo("  ego topic: %s", self._topic)
        rospy.loginfo("  3-D sample distance: %.3f m", self._sample_distance)
        rospy.loginfo("  output file: %s", self._output_file)

    def _status_callback(self, message):
        message_time = message.header.stamp.to_sec()
        if message_time <= 0.0:
            message_time = rospy.Time.now().to_sec()

        sample = {
            "message_time_sec": message_time,
            "unique_id": message.unique_id,
            "enu": (
                float(message.position.x),
                float(message.position.y),
                float(message.position.z),
            ),
            "heading_deg": float(message.heading),
            "velocity": (
                float(message.velocity.x),
                float(message.velocity.y),
                float(message.velocity.z),
            ),
        }
        with self._lock:
            if self._closed:
                return
            if self._previous_sample is None:
                self._write_sample(sample)
                self._previous_sample = sample
                return

            fractions, self._distance_to_next = resample_segment(
                self._previous_sample["enu"],
                sample["enu"],
                self._distance_to_next,
                self._sample_distance,
            )
            for fraction in fractions:
                self._write_sample(self._interpolate(self._previous_sample, sample, fraction))
            self._previous_sample = sample

    @staticmethod
    def _interpolate(start, end, fraction):
        def lerp(first, second):
            return first + (second - first) * fraction

        heading_delta = (end["heading_deg"] - start["heading_deg"] + 180.0) % 360.0 - 180.0
        return {
            "message_time_sec": lerp(start["message_time_sec"], end["message_time_sec"]),
            "unique_id": end["unique_id"],
            "enu": tuple(lerp(a, b) for a, b in zip(start["enu"], end["enu"])),
            "heading_deg": (start["heading_deg"] + heading_delta * fraction) % 360.0,
            "velocity": tuple(lerp(a, b) for a, b in zip(start["velocity"], end["velocity"])),
        }

    def _write_sample(self, sample):
        enu = sample["enu"]
        velocity = sample["velocity"]
        now = rospy.Time.now().to_sec()
        self._sequence += 1

        row = {
            "sequence": self._sequence,
            "recorded_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "ros_time_sec": "{:.9f}".format(now),
            "message_time_sec": "{:.9f}".format(sample["message_time_sec"]),
            "unique_id": sample["unique_id"],
            "global_enu_x_m": "{:.9f}".format(enu[0]),
            "global_enu_y_m": "{:.9f}".format(enu[1]),
            "global_enu_z_m": "{:.9f}".format(enu[2]),
            "heading_deg": "{:.6f}".format(sample["heading_deg"]),
            "velocity_x_mps": "{:.6f}".format(velocity[0]),
            "velocity_y_mps": "{:.6f}".format(velocity[1]),
            "velocity_z_mps": "{:.6f}".format(velocity[2]),
            "speed_mps": "{:.6f}".format(math.sqrt(sum(value * value for value in velocity))),
        }

        self._writer.write(row)
        rospy.logdebug(
            "Saved point %d: ENU=(%.3f, %.3f, %.3f)",
            self._sequence,
            enu[0],
            enu[1],
            enu[2],
        )

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            if hasattr(self, "_writer"):
                self._writer.close()
        rospy.loginfo("CSV saved: %s", self._output_file)


def main():
    rospy.init_node("morai_global_csv_recorder", anonymous=False)
    recorder = MoraiGlobalCsvRecorder()
    try:
        rospy.spin()
    finally:
        recorder.close()


if __name__ == "__main__":
    main()
