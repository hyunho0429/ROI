#!/usr/bin/env python3
"""YOLO 신호등 객체를 단독 RED/Yellow 정지 Bool 토픽으로 변환한다."""

import time

import rospy
from common.msg import ObjectInfoArray
from std_msgs.msg import Bool

from camera_perception.traffic_signal import TrafficSignalStopLatch


class TrafficLightStopNode:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/detection/traffic_light")
        output_topic = rospy.get_param(
            "~output_topic", "/perception/traffic_light/stop_required"
        )
        clear_confirmation_s = float(
            rospy.get_param("~clear_confirmation_s", 0.5)
        )
        self.publisher = rospy.Publisher(output_topic, Bool, queue_size=1, latch=True)
        self.latch = TrafficSignalStopLatch(clear_confirmation_s)
        self.stop_required = False
        rospy.Subscriber(input_topic, ObjectInfoArray, self.callback, queue_size=1)
        rospy.on_shutdown(self.shutdown)
        self.publisher.publish(Bool(data=False))
        rospy.logwarn(
            "Traffic-light stop: input=%s output=%s classes=RED-only/Yellow/Amber "
            "clear_confirmation=%.2fs",
            input_topic,
            output_topic,
            clear_confirmation_s,
        )

    def callback(self, message):
        class_names = [item.class_name for item in message.objects]
        stop_required = self.latch.update(class_names, time.monotonic())
        if stop_required != self.stop_required:
            rospy.logwarn(
                "[TRAFFIC LIGHT] %s classes=%s",
                "STOP (RED-only/YELLOW)" if stop_required else "GO",
                ",".join(class_names) if class_names else "none",
            )
        self.stop_required = stop_required
        self.publisher.publish(Bool(data=stop_required))

    def shutdown(self):
        self.publisher.publish(Bool(data=False))


def main():
    rospy.init_node("traffic_light_stop")
    TrafficLightStopNode()
    rospy.spin()


if __name__ == "__main__":
    main()
