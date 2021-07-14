#!/usr/bin/env python

import rospy
import math
import time
import numpy as np
import cv2
import pyrealsense2 as rs2

from darknet_ros.msg import BoundingBoxes
from darknet_ros.msg import BoundingBox
from tricat_211.msg import HeadingAngle
from tricat_211.msg import FilteringObstacles
from tricat_211.msg import FilteringWalls
from tricat_211.msg import Docking

file_name = '/Users/woogkingzzang/PycharmProjects/Open CV/opencv_dnn_202005/video/yolo_01.mp4'
weight_file = "/Users/woogkingzzang/PycharmProjects/Open CV/opencv_dnn_202005/yolo/yolov3.weights"
cfg_file = "/Users/woogkingzzang/PycharmProjects/Open CV/opencv_dnn_202005/yolo/yolov3.cfg"
names_file = "/Users/woogkingzzang/PycharmProjects/Open CV/opencv_dnn_202005/yolo/coco.names"

###### Docking Class ######
class Docking():
    def __init__(self):
        docking_config = rospy.get_param("DOCKING")

        ## ROS
        self.r = rospy.Rate(5)
        self.direction = ""
                 #0 = stop, 1 = forward, 2=backward, 3=rotate # for now, expressed text
        self.angle = 0.0
        self.docking_pub = rospy.Publisher('/docking_direc', Docking, queue_size=10)
        
        ## obstacles
        self.circles_list = []
        self.walls_list = []
        self.angle_avoid_ob = 0.0
        self.ob_exist = 0 #0 : None, 1 : exist
        rospy.Subscriber('/filtering_obstacles', FilteringObstacles, self.circle_callback)
        rospy.Subscriber('/filtering_walls', FilteringWalls, self.wall_callback)
        
        ## camera
        #rospy.Subscriber('/bounding_box', BoundingBox, functionName)
        rospy.Subscriber('/bounding_boxes', BoundingBoxes, self.camera_callback)
                #[0]float64 probability. [1]int64 xmin. [2]int64 ymin. [3]int64 xmax. 
                #[4]int64 ymax. [5]int16 id. [6]string Class
        self.bounding_boxes_list = []
        self.target_mark = None
        self.target_class = docking_config['target_class']
        self.img_class = None
        self.min_confidence = docking_config['min_confidence']
        self.distance_to_target = 0
        self.direction_to_target = ""
        self.angle_to_target = 0.0
        

    def camera_callback(self, msg):
        # get message from camera scanning
        if len(msg.bounding_boxes) != 0: #if not working, use 'counting msg'
            self.bounding_boxes_list = msg.bounding_boxes
        else:
            self.angle_to_target = 0
            self.direction_to_target = "BACKWARD"
        

    def camera_decision(self):
        j = 0
        for i in range(len(self.bounding_boxes_list)):
            # 2-1) target mark is detected
            if msg.Class == self.target_class:
                # 2-1-1) set target info
                self.target_mark = self.bounding_boxes_list[i]
                self.distance_to_target =  rs2.depth_frame.get_distance(
                    float((self.target_mark[3]-self.target_mark[1])/2), 
                    float((self.target_mark[4]-self.target_mark[2])/2))
                # 2-1-2) using ROI, set direction and moving angle
                self.use_cv()
                # 2-1-3) end for loop
                break
            else:
                j = j + 1
        # 2-2) target mark doesn't detected even after loop
        if j >= len(self.bounding_boxes_list): # need to check!!!!!!!!!!!!!!!!!
            self.angle_to_target = 0
            self.direction_to_target = "BACKWARD"


    def use_cv(self):
        # Load Yolo
        net = cv2.dnn.readNet(weight_file, cfg_file)
        classes = []
        with open(names_file, "r") as f:
            classes = [line.strip() for line in f.readlines()]
        layer_names = net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        colors = np.random.uniform(0, 255, size=(len(classes), 3))
        # Open camera
        cap = cv2.VideoCapture(0)
        if not cap.isOpened:
            print('--(!)Error opening video capture')
            exit(0)
        while True:
            ret, frame = cap.read()
            # (video, start, end, BGR, thickness)
            cv2.rectangle(frame, (0,60), (425,660),(0,0,255),3)
            cv2.rectangle(frame, (425,60), (850,660),(0,0,255),3)
            cv2.rectangle(frame, (850,60), (1280,660),(0,0,255),3)
            if ret is True:    
                self.handle_frame(frame) 
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()
    
    def handle_frame(self, frame):
        start_time = time.time()
        img = cv2.resize(frame, None, fx=1.0, fy=1.0)
        height, width, channels = img.shape
        cv2.imshow("Original Image", img)
        # Detecting objects
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)
        # Showing informations on the screen
        class_ids = []
        confidences = []
        boxes = []
        w = 0
        h = 0
        x = 0
        y = 0
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > min_confidence:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    # Rectangle coordinates              
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, min_confidence, 0.4)
        font = cv2.FONT_HERSHEY_PLAIN
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = "{}: {:.2f}".format(classes[class_ids[i]], confidences[i]*100)
                print(i, label)
                color = colors[i]
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, label, (x, y - 5), font, 1, color, 1)
                boxx = center_x
                # temporary ROI => adjust while field test, use param
                #Left
                if boxx < 425:
                    servo = map_func(boxx,0, 425, 0, 80 ) # servo
                    # 전진 signal = 1
                    print('Trun Left') # 좌회전
                    print(servo)
                #Middle
                elif 425< boxx & boxx <850:
                    servo = map_func(boxx, 425, 850, 80, 100)
                    # forward signal = 1
                    print('GO Straight') # Go Straight
                    print(servo)
                #Right
                elif boxx> 850:
                    servo = map_func(boxx, 850, 1280, 100, 180 )
                    # forward signal = 1
                    print('Turn Right')
                    print(servo)
        end_time = time.time()
        process_time = end_time - start_time
        print("=== A frame took {:.5f} seconds".format(process_time))
        cv2.imshow("YOLO Video", img)


    def circle_callback(self, msg):
        # when obstacle is detected
        if len(msg.circles) != 0:
            self.circles_list = msg.circles
            self.circles_decision()
            

    def circles_decision(self):
        d = self.circles_list[0].d #distance from ob
        theta = self.circles_list[0].theta #angle from boat
        # 1) find nearest obstacle
        for i in range(len(self.circles_list)):
            if self.circles_list[i].d < d:
                d =  self.circles_list[i].d
                thata = self.circles_list[i].theta
                # check whether behind obstacle is being detected!!!!!!!!!!!!!!
        # 2) set move direction and angle
        if d < 5:
            # filter if other object is detected(ex. people). let's set this value with param!!!!!!!!!!!!!!!
            self.angle_avoid_ob =  -map_func(theta, -90, 90, -60, 60) # check if this angle is right. is (-) working?!!!!!!!!!!!!!
        # 3) move boat
        self.ob_exist = 1


    def wall_callback(self, msg):
        if len(msg.walls) != 0:
            self.walls_list = msg.walls
            

    def walls_decision(self):
        distance = self.walls_list[0].distance
            # 1) find nearest wall
            for i in range(len(self.walls_list)):
                if self.walls_list[i].distance < distance:
                    distance =  self.walls_list[i].distance
                    x = self.walls_lists[i].end_x - self.walls_list[i].start_x
                    y = self.walls_list[i].end_y - self.walls_list[i].start_y
            # 2) set move direction and angle
            self.angle_avoid_ob = -(math.atan2(y, x) * 180 / math.pi) #degree
            # 3) move boat
            self.ob_exist = 1


    def boat_control(self):
        if self.ob_exist == 1:
            self.angle = self.angle_avoid_ob
            self.direction = "ROTATE"
            self.ob_exist = 0
        else:
            self.angle = self.angle_to_target
            self.direction = self.direction_to_target
        

    def DockingPublisher(self):
        docking_msg = Docking()
        docking_msg.angle = self.angle
        self.docking_pub.publish(docking_msg)

def map_func(x, input_min, input_max, output_min, output_max):
    return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min
    # check whether it works well!!!!!!!!!!!!!!!

def main():
    docking = Docking()
    while not rospy.is_shutdown():
        docking.camera_decision()
        docking.boat_control()
        docking.DockingPublisher()
        # move boat once -> stop -> rotate heading to mark direction -> move boat again??????
        rospy.sleep(1)
    rospy.spin()


if __name__ == '__main__':
    main()

############ DUMP ################
#def camara_callback(self):
        # # 화면 안에 targetmark가 사라질 때 처리 추가하기
        # if len(self.bounding_boxes_list) < 3 and self.target_mark is None:
        #     # DWA 끄고 도킹 시작하는 지점에 왔는데 3개가 다 안 보일 경우, 또는 target_mark가 설정되지 않았을 경우
        #     self.boat_control(0)
        # elif self.target_mark is None:
        #     # 3개 다 보이고 target_mark가 설정되지 않았을 경우임. 지금 설정함
        #     for i in range(len(self.bounding_boxes_list)):
        #         if msg.Class == img_Class:
        #             self.target_mark = self.bounding_boxes_list[i]

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

    # def set_target_mark_info(self):
    #     # self.bounding_box_size = abs(self.target_mark.xmax - self.target_mark.xmin) * abs(self.target_mark.ymax - self.target_mark.ymin) #확인하기
    #     # self.center_x = float((msg.xmax - msg.xmin) / 2)
    #     # self.center_y = float((msg.ymax - msg.ymin) / 2)
    #     # self.img_Class = self.target_mark.Class
    #     #self.img_probability = self.target_mark.probability
    #     self.dist_to_mark = rs2.depth_frame.get_distance(self.center_x, self.center_y)  #특정 픽셀의 depth 도출. m단위로 반환 // 박스 사이즈 쓸거면 얘는 안 쓸지도?


    # def img_detection(self):
    # if self.img_probability < self.target_probability or self.bounding_box_size < self.target_box_size:
    #     self.direction = "TO_FORWARD"
    #     self.boat_control() #전진해서 확률과 크기 더 높게
    # else:
    #     mark_ROI_control() # ROI 설정해 왼쪽/오른쪽/직진 설정하는 부분!!

    # def obstacle_callback(self, msg):
        # if self.min_distance > radius and radius > 0.0:
        #             self.min_distance = radius
        #             if 10 < theta < 80:
        #                 #단위 맞나 확인!, 각도 조절!
        #                 self.ob_position = "AT_RIGHT"
        #             elif 80 <= theta <= 110:
        #                 self.ob_position = "AT_CENTER"
        #             elif 110 < theta < 170:
        #                 self.ob_position = "AT_LEFT"
        #             else:
        #                 self.ob_position = "AT_BACK"
    # def wall_callback(self, msg):
        # dist = 0.0
        # direc = ""
        # i = 0
        # min_i = 0
        # #장애물이 배 뒤쪽으로 있는 경우는 빠지나??
        # #아무것도 리턴 안 할 경우 오류 처리
        # #좌우중앙 범위 다시 설정
        # while i < len(msg.distance):
        #     if msg.walls[i].distance < dist:
        #         if msg.walls[i].end_y > 0 and msg.walls[i].start_y > 0:
        #             min_i = i
        #     i = i + 1
        # if self.min_distance > msg.walls[i].distance > 0.0:
        #     if msg.walls[i].start_x >= 0 and msg.walls[i].end_x >= 0:
        #         self.ob_position = "AT_RIGHT"
        #     elif msg.walls[i].start_x >= 0 and msg.walls[i].end_x < 0:
        #         self.ob_position = "AT_CENTER"
        #     elif msg.walls[i].start_x < 0 and msg.walls[i].end_x >= 0:
        #         self.ob_position = "AT_CENTER"
        #     elif msg.walls[i].start_x < 0 and msg.walls[i].end_x < 0:
        #         self.ob_position = "AT_LEFT"

    # def ob_avoid(self):
    #     # 정지 상태일 때 처리하기
    #     if 0 < self.min_distance < 3:
    #         if self.ob_position == "AT_RIGHT":
    #             self.direction = "TO_LEFT"
    #         elif self.ob_position == "AT_CENTER":
    #             # 직진 상태인데 전방에 장애물 인식하면? ROI대로 가자
    #             # dock의 mark 바로 아래를 인식할 경우?
    #         elif self.ob_position == "AT_LEFT":
    #             self.direction = "TO_RIGHT"
    #         self.angle = map(self.min_distance, 0, 3, 10, 60) #좌우에 따라 direction 대신 각도 +, - 설정하기 또는 min_distance에 부호 붙여넣기?
    #         #self.boat_control()
    #     else:
    #         #일정 반경 안에 들어오지 않는 이상 우선순위를 마크 쪽으로
    #         pass
