#!/usr/bin/env python

import rospy
import math
import pymap3d as pm
import numpy as np

from tricat_211.msg import Control, HeadingAngle  
from std_msgs.msg import UInt16
from geometry_msgs.msg import Point

class Goal:
    def __init__(self):
        self.boat_x = 0.0
        self.boat_y = 0.0

        rospy.Subscriber("/bearing", HeadingAngle, self.heading_callback)
        rospy.Subscriber("/enu_position", Point, self.enu_callback)

        self.map_list = rospy.get_param("map_dd")
        self.lat_00, self.lon_00, self.alt_00 = self.map_list['map_00_lat'], self.map_list['map_00_lon'], self.map_list['map_00_alt']

        self.waypoints = rospy.get_param("waypoint_List/waypoints")
        self.way_list_gps = np.empty((0,3), float)
        for i in range(len(self.waypoints)):
            self.way_list_gps = np.append(self.way_list_gps, np.array([self.waypoints[i]]), axis=0)
        
        self.way_list_gps = self.way_list_gps.astype(np.float64)

        self.goal_list = self.get_xy(self.way_list_gps)

        self.goal_range = rospy.get_param("waypoint_distance_tolerance")

        self.goal_x = self.goal_list[0][0]
        self.goal_y = self.goal_list[0][1]

        self.angle = 0.0
        self.target_angle = 0.0
        self.bearing = 0.0

        ## PID 
        self.init_servo = 94

        self.servo_control = 0

        self.kp_servo = rospy.get_param("kp_servo")

        self.rate = 20

        self.thruster_power = rospy.get_param("thruster_power")
    
        self.Servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)

    def get_xy(self, points):
        way_list = np.zeros_like(points)
        for i in range(len(points)):
            way_list[i][0], way_list[i][1] , way_list[i][2] = \
                pm.geodetic2enu(points[i][0], points[i][1], points[i][2], \
                 self.lat_00, self.lon_00, self.alt_00)
        way_list = np.delete(way_list, 2, axis=1)
        return way_list

    def heading_callback(self, data):
        self.bearing = data.bearing

    def enu_callback(self, data):
        self.boat_x = data.x  # East
        self.boat_y = data.y  # North
        #self.z = data.z

    def target(self):
        self.angle = math.atan2((self.goal_x - self.boat_x), (self.goal_y - self.boat_y))
        self.angle = self.angle * 180 / math.pi
        self.target_angle = self.angle - self.bearing
        return self.target_angle

    def set_next_point(self):
        self.goal_list = np.delete(self.goal_list, 0, axis = 0)

    def arrival_check(self):
        self.dist_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)
        
        if self.dist_to_goal <= self.goal_range:
            return True
        else:
            return False

    def prt(self):
        print("distance : ", self.dist_to_goal)
        print("my xy : ",self.boat_x, self.boat_y)
        print("way xy : ",self.goal_x, self.goal_y)
        print("bearing, target angle : ", self.bearing, self.target())
        print("servo : ", round(self.servo_pid_controller(),1))
        print('-------------------------------------')
    
    def servo_pid_controller(self):
        self.error_angle = self.target()

        self.cp_servo = self.kp_servo * -self.error_angle


        servo_pd = self.cp_servo
        self.servo_control = self.init_servo + servo_pd

        if self.servo_control > 118: #94+24
            self.servo_control = 118
        elif self.servo_control < 70:# 94-24
            self.servo_control = 70
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
    rospy.init_node('Hopping_P_controller', anonymous=False)

    goal = Goal()
    
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        if goal.arrival_check():
            if len(goal.goal_list) == 0:
                break  #arrived final goal
            elif len(goal.goal_list) == 1:
                goal.goal_x = goal.goal_list[0][0]
                goal.goal_y = goal.goal_list[0][1]
                goal.set_next_point()
            else:
                goal.set_next_point()
                goal.goal_x = goal.goal_list[0][0]
                goal.goal_y = goal.goal_list[0][1]
        #goal.target()
        #pid.servo_pid_controller()
        goal.control_publisher()
        goal.prt()
        
        rate.sleep()
        
    rospy.spin()


if __name__ == '__main__':
    main()