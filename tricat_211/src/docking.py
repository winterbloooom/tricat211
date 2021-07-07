#!/usr/bin/env python

import rospy
import numpy as np
from darknet_ros.msg import BoundingBoxes
from darknet_ros.msg import BoundingBox
from dwa_path_planner import *
import cv2
import pyrealsense2 as rs2

###### Global variables ######
## dock condition
DOCK_DEPTH = 3 #[m]. 임시. dock 깊이
DOCK_WIDTH = 2 #[m]. 임시. dock 너비
## bounding box
CRITICAL_BOX_SIZE = 10000 # 임시. bounding box가 이 값 이상 크기를 가지는지 판단


###### Docking Class ######
class Docking():
    def __init__(self):
        ## Mark
        self.dist_to_mark = 0 #배에서 마크까지 거리
        ## Boat
        self.ship_speed = 0 # [m/s] from GPS???
        ## ROS
        self.r = rospy.Rate(5)
        self.publisher = rospy.Publisher('', , queue_size=10) # 뭘 publish 할건가? DWA에 연결만 할건가?
        ## OpenCV
        rospy.Subscriber('/bounding_box', BoundingBox, self.scan_OpenCV) #이거 맞나 확인!
        rospy.Subscriber('/bounding_boxes', BoundingBoxes, self.scan_OpenCV) #이거 맞나 확인!
        self.bounding_box_size = 0
        self.img_class = "" #무슨 모양을 판단할 건지
        self.img_probability = 0 #정확도
        ##DWA
        self.dwa_control = DWA_Calc()
        self.enterance_center = (0, 0) # 입구 진입지점

    def scan_OpenCV(self, msg):
        self.bounding_box_size = abs(msg.xmax - msg.xmin) * abs(msg.ymax - msg.ymin)
        self.center_x = abs(msg.xmax - msg.xmin)
        self.center_y = abs(msg.ymax - msg.ymin)
        self.img_class = msg.class #예약어로 들어가는 문제 해결
        self.img_probability = msg.probability
        #무슨 모양 판단할 건지, 정확도 필터링 필요 ++ boundingbox 크기 여기서 필터링 할까?

    def scan_realsense(self, msg):
        #카메라에서 받아올 것?


    def distance_detection(self):
        self.dist_to_mark = rs2.depth_frame.get_distance(self.center_x, self.center_y)  #특정 픽셀의 depth 도출. m단위로 반환 // 중간지점 맞나?
        # 진입지점 설정 여기서...?
        
    def process_docking(self):
        if self.bounding_box_size >= CRITICAL_BOX_SIZE or Goal.state == 1 :
            # 진입지점 x, y좌표 찍는 법???
            Goal.set_enterance_center(x, y)
            dwa_control.dwa_control() #맞나?
        elif Goal.state == 2:
            dwa_control.dwa_control() #맞나?
        elif Goal.state == 3:
            #정렬
            if (self.dist_to_mark - (dwa_control.boat_length/2)) > 0.5 :
                #앞으로 전진해서 주차
            else :
                #정지
    
    def run_all_process(self):
        self.distance_detection()
        self.process_docking()


def main():
    docking = Docking()

    while not rospy.is_shutdown():
        docking.run_all_process()
        rospy.sleep(1)

    rospy.spin()


if __name__ == '__main__':
    main()

        