#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import csv
import os
from nav_msgs.msg import Path # 경로 데이터를 받는 ROS 표준 메시지 타입

class PathToCSV:
    def __init__(self):
        rospy.init_node('path_to_csv_node', anonymous=True)
        
        # 다익스트라 경로 토픽
        self.path_topic = "/global_path" 
        
        # 저장할 경로 설정 (ROI 패키지 내 path 폴더)
        self.save_dir = os.path.expanduser("~/catkin_ws/src/ROI/path")
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        self.csv_file_path = os.path.join(self.save_dir, "path_2.csv")
        
        rospy.Subscriber(self.path_topic, Path, self.path_callback)
        
        self.is_saved = False
        rospy.loginfo(f"경로를 기다리는 중... 저장 위치: {self.csv_file_path}")

    def path_callback(self, msg):
        if self.is_saved:
            return
            
        # [수정 포인트] 데이터가 비어 있는 래치(Latch) 패킷은 무시하고 패스합니다.
        if not msg.poses:
            rospy.logwarn("빈 경로 데이터(Latch) 수신. 실제 RViz 클릭 입력을 대기합니다...")
            return
            
        rospy.loginfo(f"유효한 경로 데이터를 수신했습니다! (포인트 개수: {len(msg.poses)}개) CSV로 저장을 시작합니다.")
        
        try:
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['x', 'y'])
                
                for pose in msg.poses:
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    writer.writerow([x, y])
                    
            rospy.loginfo(f"성공! 경로가 {self.csv_file_path}에 저장되었습니다.")
            self.is_saved = True
            rospy.signal_shutdown("CSV 저장 완료")
            
        except Exception as e:
            rospy.logerr(f"파일 저장 중 에러 발생: {e}")

if __name__ == '__main__':
    try:
        PathToCSV()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
