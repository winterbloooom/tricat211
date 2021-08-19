#!/usr/bin/env python

import rospy
import math
import pymap3d as pm
import numpy as np

from tricat_211.msg import Control, HeadingAngle  
from std_msgs.msg import UInt16
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu

def RAD2DEG(x):
    return x * 180. / math.pi

def DEG2RAD(x):
    return x / 180. * math.pi

class Goal:
    def __init__(self):
        self.boat_x = 0.0
        self.boat_y = 0.0

        self.yaw_rate = 0.0

        rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback)
        rospy.Subscriber("/bearing", HeadingAngle, self.heading_callback)
        rospy.Subscriber("/enu_position", Point, self.enu_callback)

        ## ENU & Waypoint List
        self.map_list = rospy.get_param("map_dd")
        self.lat_00, self.lon_00, self.alt_00 = self.map_list['map_00_lat'], self.map_list['map_00_lon'], self.map_list['map_00_alt']

        self.waypoints = rospy.get_param("waypoint_List/waypoints")
        self.way_list_gps = np.empty((0,3), float)
        for i in range(len(self.waypoints)):
            self.way_list_gps = np.append(self.way_list_gps, np.array([self.waypoints[i]]), axis=0)
        #self.way_list_gps = self.way_list_gps.astype(np.float64)
        self.goal_list = self.get_xy(self.way_list_gps) # ENU way list
        self.goal_x = self.goal_list[0][0]
        self.goal_y = self.goal_list[0][1]
        
        self.goal_range = rospy.get_param("goal_range")
        
        ## Direction Search
        self.angle = 0.0
        self.bearing = 0.0

        ## PID 
        #self.init_servo = 93
        self.servo_control = 93

        self.kp_servo = rospy.get_param("kp_servo")
        self.kd_servo = rospy.get_param("kd_servo")
        self.thruster_power = rospy.get_param("thruster_power")

        self.Servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)

    def get_xy(self, points):
        way_list = np.zeros_like(points)
        for i in range(len(points)):
            way_list[i][0], way_list[i][1] , way_list[i][2] = \
                pm.geodetic2enu(points[i][0], points[i][1], points[i][2], \
                 self.lat_00, self.lon_00, self.alt_00)
        way_list = np.delete(way_list, 2, axis=1) # axis z delete
        return way_list

    def yaw_rate_callback(self, data):
        self.yaw_rate = data.angular_velocity.z  # yaw_rate [rad/s]

    def heading_callback(self, data):
        self.bearing = data.bearing

    def enu_callback(self, data):
        self.boat_x = data.x  # East
        self.boat_y = data.y  # North

    def OPTIMAL_DIRECTION(self, b):
        dx = self.goal_x - self.boat_x
        dy = self.goal_y - self.boat_y
        self.angle = RAD2DEG(math.atan2(dx, dy))

        if dx >= 0 and dy >= 0: # Quadrant 1
            if b >= 0 : # right bearing
                self.t = self.angle - b
            elif b < 0: # left bearing
                if abs(b) < (180 - self.angle):
                    self.t = self.angle - b
                elif abs(b) >= (180 - self.angle):
                    self.t = -(360 - self.angle + b)

        elif dx < 0 and dy >= 0: # Quadrant 2
            if b >= 0 :
                if b < 180 + self.angle:
                    self.t = self.angle - b
                elif b >= 180 + self.angle:
                    self.t = 360 + self.angle - b
            elif b < 0:
                self.t = self.angle - b
                
        elif dx < 0 and dy < 0: # Quadrant 3
            if b >= 0 :
                if b < 180 + self.angle:
                    self.t = (self.angle - b)
                elif b >= 180 + self.angle:
                    self.t = 360 + (self.angle - b)
            elif b < 0:
                self.t = (self.angle - b)

        elif dx >= 0 and dy < 0: # Quadrant 4
            if b >= 0 :
                self.t = (self.angle - b)
            elif b < 0:
                if abs(b) < 180 - self.angle:
                    self.t = (self.angle - b)
                elif abs(b) >= 180 - self.angle:
                    self.t = self.angle - b - 360

        return self.t

    def target(self):
        
        return self.OPTIMAL_DIRECTION(self.bearing)

    def set_next_point(self):
        self.goal_list = np.delete(self.goal_list, 0, axis = 0)

    def arrival_check(self):
        self.dist_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)
        
        if self.dist_to_goal <= self.goal_range:
            return True
        else:
            return False

    def prt(self):
        if self.servo_control > 93+3: # left turn
            turn = "left"
        elif self.servo_control < 93-3: # right turn
            turn = "right"
        else:
            turn = "mid"
        print("distance : ", self.dist_to_goal)
        print("my xy : ",self.boat_x, self.boat_y)
        print("way xy : ",self.goal_x, self.goal_y)
        print("a, b, t : ", self.angle, self.bearing, self.OPTIMAL_DIRECTION(self.bearing))
        print("servo : " + turn, round(self.servo_control))
        print('-------------------------------------')
    
    def servo_pid_controller(self):
        # P ctrl
        error_angle = self.target() # deg

        # D ctrl
        yaw_rate = RAD2DEG(self.yaw_rate) # deg/s

        cp_servo = self.kp_servo * error_angle
        cd_servo = self.kd_servo * -yaw_rate

        servo_pd = -(cp_servo + cd_servo)
        self.servo_control = self.servo_control + servo_pd

        if self.servo_control > 93+24: #94+24
            self.servo_control = 93+24
        elif self.servo_control < 93-24:
            self.servo_control = 93-24
        else:
            pass

        return self.servo_control

    def control_publisher(self):
        output_msg = Control()
        output_msg.thruster = self.thruster_power  #  param thruster value
        output_msg.servo = round(self.servo_pid_controller())
        self.thruster_pub.publish(output_msg.thruster)
        self.Servo_pub.publish(output_msg.servo)


def main():
    rospy.init_node('Hopping_PD_controller', anonymous=False)

    goal = Goal()
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if goal.arrival_check():
            if len(goal.goal_list) == 0:
                goal.thruster_pub.publish(1500)
                goal.Servo_pub.publish(93)
                #rospy.on_shutdown()
                print("arrived final goal")
                break
            elif len(goal.goal_list) == 1:
                goal.goal_x = goal.goal_list[0][0]
                goal.goal_y = goal.goal_list[0][1]
                goal.set_next_point()
            else:
                goal.set_next_point()
                goal.goal_x = goal.goal_list[0][0]
                goal.goal_y = goal.goal_list[0][1]

        goal.control_publisher()
        goal.prt()
        
        rate.sleep()
        
    rospy.spin()


if __name__ == '__main__':
    main()