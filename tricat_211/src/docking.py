#!/usr/bin/env python

import rospy
import math
import time
import numpy as np
import cv2
import pyrealsense2 as rs2

from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from tricat_211.msg import HeadingAngle
from tricat_211.msg import FilteringWalls, FilteringObstacles
from tricat_211.msg import DockingMsg
from std_msgs.msg import Control, UInt16           # for `Output`

file_name = "/Users/woogkingzzang/PycharmProjects/Open CV/opencv_dnn_202005/video/yolo_01.mp4"
weight_file = "/Users/woogkingzzang/PycharmProjects/Open CV/opencv_dnn_202005/yolo/yolov3.weights"
cfg_file = "/Users/woogkingzzang/PycharmProjects/Open CV/opencv_dnn_202005/yolo/yolov3.cfg"
names_file = "/Users/woogkingzzang/PycharmProjects/Open CV/opencv_dnn_202005/yolo/coco.names"

###### Docking Class ######
class Docking():
    def __init__(self):
        docking_config = rospy.get_param("DOCKING")

        ## ROS
        self.direction = ""
        self.angle = 0.0
        self.docking_pub = rospy.Publisher('/docking_control', DockingMsg, queue_size=10)
        
        ## obstacles
        self.circles_list = []
        self.walls_list = []
        self.angle_avoid_ob = 0.0
        self.ob_exist = 0 #0 : None, 1 : exist
        rospy.Subscriber('/filtering_obstacles', FilteringObstacles, self.circle_callback)
        rospy.Subscriber('/filtering_walls', FilteringWalls, self.wall_callback)
        
        ## camera
        #rospy.Subscriber('/bounding_box', BoundingBox, functionName) #???????????????????????????
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
        
        ## control
        self.Servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)

    def control_publisher(self):
        output_msg = Control()
        output_msg.thruster = 1600
        output_msg.servo = round(self.angle)
        self.thruster_pub.publish(output_msg.thruster)
        self.Servo_pub.publish(output_msg.servo)
        
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
                    servo = map_func(boxx, 0, 425, 0, 80 ) # servo 0~80 edit need
                    # forward signal = 1
                    self.direction = "ROTATE"
                    self.angle = servo
                    # print('Trun Left')
                    # print(servo)
                #Middle
                elif 425< boxx & boxx <850:
                    servo = map_func(boxx, 425, 850, 80, 100)
                    # forward signal = 1
                    self.direction = "FORWARD"
                    self.angle = servo
                    # print('GO Straight') # Go Straight
                    # print(servo)
                #Right
                elif boxx> 850:
                    servo = map_func(boxx, 850, 1280, 100, 180 )
                    # forward signal = 1
                    self.direction = "ROTATE"
                    self.angle = servo
                    # print('Turn Right')
                    # print(servo)
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
            self.angle_avoid_ob = -map_func(theta, -90, 90, -60, 60) # check if this angle is right. is (-) working?!!!!!!!!!!!!!
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
        docking_msg = DockingMsg()
        docking_msg.angle = self.angle
        if self.direction == "FORWARD":
            docking_msg.direction = 1
        elif self.direction == "BACKWARD":
            docking_msg.direction = 2
        elif self.direction == "ROTATE":
            docking_msg.direction = 3
        else:
            docking_msg.direction = 0
        self.docking_pub.publish(docking_msg)

def map_func(x, input_min, input_max, output_min, output_max):
    return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min
    # check whether it works well!!!!!!!!!!!!!!!

def main():
    rospy.init_node('Dock', anonymous=False)
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