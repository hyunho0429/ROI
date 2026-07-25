#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import csv
import os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def publish_path():
    rospy.init_node('path_publisher_node')
    path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
    rate = rospy.Rate(10) # 1초에 10번씩 경로 쏴주기

    path_msg = Path()
    path_msg.header.frame_id = "map"

    # CSV 파일이 있는 경로 (이전 대화 내용 기반으로 작성됨, 필요시 수정)
    csv_file_path = os.path.expanduser("~/catkin_ws/src/ROI/path/path_1.csv")

    try:
        with open(csv_file_path, 'r') as f:
            rdr = csv.reader(f)
            next(rdr) # 첫 번째 줄(헤더) 건너뛰기
            for line in rdr:
                if line:
                    pose = PoseStamped()
                    pose.pose.position.x = float(line[0])
                    pose.pose.position.y = float(line[1])
                    path_msg.poses.append(pose)
        rospy.loginfo(f"성공! {len(path_msg.poses)}개의 웨이포인트를 퍼블리시합니다.")
    except Exception as e:
        rospy.logerr(f"경로 파일을 찾을 수 없습니다: {e}")
        return

    # 제어기가 받을 수 있도록 계속 퍼블리시
    while not rospy.is_shutdown():
        path_msg.header.stamp = rospy.Time.now()
        path_pub.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass
