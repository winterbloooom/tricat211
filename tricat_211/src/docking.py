#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import pyrealsense2 as rs2

from darknet_ros.msg import BoundingBoxes
from darknet_ros.msg import BoundingBox
from tricat_211.msg import HeadingAngle
from tricat_211.msg import FilteringObstacles
from tricat_211.msg import FilteringWalls
from tricat_211.msg import Docking

###### Docking Class ######
class Docking():
    def __init__(self):
        docking_config = rospy.get_param("DOCKING")
        ## ROS
        self.r = rospy.Rate(5)
        self.docking_pub = rospy.Publisher('/docking_direc', Docking, queue_size=10)
        ## obstacles
        self.min_distance = 0.0
        self.near_obstacle = None
        self.ob_position = ""
        rospy.Subscriber('/filtering_obstacles', FilteringObstacles, self.obstacle_callback)
        rospy.Subscriber('/filtering_walls', FilteringWalls, self.wall_callback)
        
        ## OpenCV, Darknet
        #rospy.Subscriber('/bounding_box', BoundingBox, functionName)
        rospy.Subscriber('/bounding_boxes', BoundingBoxes, self.scan_bounding_boxes)
        self.bounding_boxes_list = []
        self.target_mark = None
        self.target_box_size = docking_config['target_box_size']
        self.bounding_box_size = 0
        self.target_class = docking_config['target_class']
        self.img_class = None
        self.img_probability = 0
        self.target_probability = docking_config['target_probability']
        self.dist_to_mark = 0
        ## moving
        self.direction = "" #0 = stop, 1 = forward, 2=backward, 3=right, 4=left #지금은 일단 텍스트 처리함
        self.angle = 0.0

    def scan_bounding_boxes(self, msg):
        self.bounding_boxes_list = msg.bounding_boxes 
        if len(msg.bounding_boxes) == 0: #작동 안되면 counting 메시지로 처리
            self.angle = 0
            self.direction = "BACKWARD"
            self.boat_control()
        # 화면 안에 targetmark가 사라질 때 처리 추가하기
        if len(self.bounding_boxes_list) < 3 and self.target_mark is None:
            # DWA 끄고 도킹 시작하는 지점에 왔는데 3개가 다 안 보일 경우, 또는 target_mark가 설정되지 않았을 경우
            self.boat_control(0)
        elif self.target_mark is None:
            # 3개 다 보이고 target_mark가 설정되지 않았을 경우임. 지금 설정함
            for i in range(len(self.bounding_boxes_list)):
                if msg.Class == img_Class: #우리가 판단하려는 클래스와 같다면
                    self.target_mark = self.bounding_boxes_list[i] #target 마크를 설정함
            self.set_target_mark_info()
            # 다 돌았는데 없다면 어떻게 할 것인가??
        
    def set_target_mark_info(self):
        self.bounding_box_size = abs(self.target_mark.xmax - self.target_mark.xmin) * abs(self.target_mark.ymax - self.target_mark.ymin) #확인하기
        self.center_x = float((msg.xmax - msg.xmin) / 2)
        self.center_y = float((msg.ymax - msg.ymin) / 2)
        self.img_Class = self.target_mark.Class
        #self.img_probability = self.target_mark.probability
        self.dist_to_mark = rs2.depth_frame.get_distance(self.center_x, self.center_y)  #특정 픽셀의 depth 도출. m단위로 반환 // 박스 사이즈 쓸거면 얘는 안 쓸지도?

    def img_detection(self):
        if self.img_probability < self.target_probability or self.bounding_box_size < self.target_box_size:
            self.direction = "TO_FORWARD"
            self.boat_control() #전진해서 확률과 크기 더 높게
        else:
            mark_ROI_control() # ROI 설정해 왼쪽/오른쪽/직진 설정하는 부분!!
            
        
    def mark_ROI_control(self):
        # 판단하려는 마크를 ROI 안에 들어왔는지 판단하고 direction으로 설정
    

    def obstacle_callback(self, msg):
        radius = 0.0
        theta = 0.0
        #아무것도 없을 경우 처리하기!
        for i in range(len(msg.circles)):
            if msg.circles[i].radius < radius:
                radius =  msg.circles[i].radius
                thata = msg.circles[i].theta
        if self.min_distance > radius and radius > 0.0:
            self.min_distance = radius
            if 10 < theta < 80:
                #단위 맞나 확인!, 각도 조절!
                self.ob_position = "AT_RIGHT"
            elif 80 <= theta <= 110:
                self.ob_position = "AT_CENTER"
            elif 110 < theta < 170:
                self.ob_position = "AT_LEFT"
            else:
                self.ob_position = "AT_BACK"


    def wall_callback(self, msg):
        dist = 0.0
        direc = ""
        i = 0
        min_i = 0
        #장애물이 배 뒤쪽으로 있는 경우는 빠지나??
        #아무것도 리턴 안 할 경우 오류 처리
        #좌우중앙 범위 다시 설정
        while i < len(msg.distance):
            if msg.walls[i].distance < dist:
                if msg.walls[i].end_y > 0 and msg.walls[i].start_y > 0:
                    min_i = i
            i = i + 1
        if self.min_distance > msg.walls[i].distance > 0.0:
            if msg.walls[i].start_x >= 0 and msg.walls[i].end_x >= 0:
                self.ob_position = "AT_RIGHT"
            elif msg.walls[i].start_x >= 0 and msg.walls[i].end_x < 0:
                self.ob_position = "AT_CENTER"
            elif msg.walls[i].start_x < 0 and msg.walls[i].end_x >= 0:
                self.ob_position = "AT_CENTER"
            elif msg.walls[i].start_x < 0 and msg.walls[i].end_x < 0:
                self.ob_position = "AT_LEFT"
        

    def ob_avoid(self):
        # 정지 상태일 때 처리하기
        if 0 < self.min_distance < 3:
            if self.ob_position == "AT_RIGHT":
                self.direction = "TO_LEFT"
            elif self.ob_position == "AT_CENTER":
                # 직진 상태인데 전방에 장애물 인식하면? ROI대로 가자
                # dock의 mark 바로 아래를 인식할 경우?
            elif self.ob_position == "AT_LEFT":
                self.direction = "TO_RIGHT"
            self.angle = map(self.min_distance, 0, 3, 10, 60) #좌우에 따라 direction 대신 각도 +, - 설정하기 또는 min_distance에 부호 붙여넣기?
            #self.boat_control()
        else:
            #일정 반경 안에 들어오지 않는 이상 우선순위를 마크 쪽으로
            pass

    def boat_control(self):
        # direction 따라 후진, 전진, 좌회전, 우회전 움직이도록->각도로 바꾸기
        #한번 배 전진(움직)이고 정지한 뒤 heading을 마크 쪽으로 두기를 반복?
        
            
    
    def run_all_process(self):
        self.img_detection()
        self.ob_avoid()
        #####화면 안에 없으면 후진 기능 추가하기
    
    def DockingPublisher(self):
        docking_msg = Docking()
        docking_msg.angle = self.angle
        self.docking_pub.publish(docking_msg)

def map_func(x, input_min, input_max, output_min, output_max):
    return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min

def main():
    docking = Docking()

    while not rospy.is_shutdown():
        docking.run_all_process()
        docking.DockingPublisher()

        rospy.sleep(1)

    rospy.spin()


if __name__ == '__main__':
    main()

############ DUMP ################
    # def set_target_mark(self):
    #     # #which mark should we choose?
    #     # #filter marks
    #     # self.target_mark_position = [] #x, y coordinates
    #     # if self.distance_detection():
    #     #     process_docking()
        

    #     self.bounding_box_size = abs(msg.xmax - msg.xmin) * abs(msg.ymax - msg.ymin)
    #     self.center_x = abs(msg.xmax - msg.xmin)
    #     self.center_y = abs(msg.ymax - msg.ymin)
    #     self.img_class = msg.Class
    #     self.img_probability = msg.probability
    #     #무슨 모양 판단할 건지, 정확도 필터링 필요 ++ boundingbox 크기 여기서 필터링 할까?

    # def docking_process(self):
    #     ###### 이전 코드. 무시하세요 ######
    #     # if self.bounding_box_size >= CRITICAL_BOX_SIZE or Goal.state == 1 :
    #     #     # 진입지점 x, y좌표 찍는 법???
    #     #     # 마크까지 원호 그리고(heading 고려해서) 원호 그리며 나아가되 벽 만나면 피하는 걸로?
    #     #     Goal.set_enterance_center(x, y)
    #     #     dwa_control.dwa_control() #맞나?
    #     # elif Goal.state == 2:
    #     #     dwa_control.dwa_control() #맞나?
    #     # elif Goal.state == 3:
    #     #     #정렬
    #     #     if (self.dist_to_mark - (dwa_control.boat_length/2)) > 0.5 :
    #     #         #앞으로 전진해서 주차
    #     #     else :
    #     #         #정지
        
    #     # ROI 설정해 왼쪽/오른쪽/직진 설정하는 부분!!    