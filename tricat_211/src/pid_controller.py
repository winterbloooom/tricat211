#!/usr/bin/env python

import rospy
import math

from sensor_msgs.msg import Imu
from ublox_msgs.msg import NavPVT
from sensor_msgs.msg import NavSatFix
from tricat_211.msg import Control
from tricat_211.msg import DWA  # v, yaw_rate float64


class PID:
    def __init__(self):
        #self.angular_v_x = 0.0
        #self.angular_v_y = 0.0
        self.init_thruster = 1500
        self.init_servo = 150

        self.thruster_pwm = 0
        self.servo_control = 0

        self.cur_yaw_rate = 0.0
        self.cur_v = 0.0
    
        self.optimal_v = 0.0
        self.optimal_yaw_rate = 0.0

        self.kp_thruster = 50.0
        self.ki_thruster = 0.1
        self.kd_thruster = 0.01
        self.error_v = 0.0
        self.error_v_sum = 0.0

        self.kp_servo = 1.0
        self.ki_servo = 0.01
        self.kd_servo = 0.001
        self.error_yaw_rate = 0.0
        self.error_yaw_rate_sum = 0.0

        self.rate = 20

        rospy.Subscriber("/imu/data", Imu, self.imu_data_callback)
        rospy.Subscriber("/ublox_gps/navpvt", NavPVT, self.gps_data_callback)
        rospy.Subscriber("/best_U", DWA, self.DWA_data_callback)

        self.PID_controller_pub = rospy.Publisher("/control", Control, queue_size=10)

    def imu_data_callback(self, data):
        #self.angular_v_x = data.angular_velocity.x
        #self.angular_v_y = data.angular_velocity.y
        self.cur_yaw_rate = data.angular_velocity.z

    def gps_data_callback(self, data):
        self.cur_v = data.gSpeed

    def DWA_data_callback(self, data):
        self.optimal_v = data.v
        self.optimal_yaw_rate = data.yaw_rate

    def thruster_pid_controller(self):
        self.error_v = self.optimal_v - self.cur_v

        self.cp_thruster = self.kp_thruster * self.error_v

        self.ci_thruster = 0

        self.cd_thruster = 0

        self.thruster_pwm = self.init_thruster + self.cp_thruster# + self.ci_thruster + self.cd_thruster

        return self.thruster_pwm

    def servo_pid_contoller(self):
        self.error_yaw_rate = self.optimal_yaw_rate - self.cur_yaw_rate

        self.cp_servo = self.kp_servo * self.error_yaw_rate

        #self.error_yaw_rate_sum = self.error_yaw_rate_sum + self.error_yaw_rate * (1 / self.rate)
        #if motor is off :
        #    self.error_yaw_rate_sum = 0
        #self.ci_servo =  self.kd_servo * self.error_yaw_rate_sum
 
        prev_yaw_rate = self.cur_yaw_rate
        yaw_rate_derivative = (self.cur_yaw_rate - prev_yaw_rate) / (1 / self.rate)
        self.cd_servo = self.ki_servo * -yaw_rate_derivative

        #servo_pid = self.cp_servo+ self.ci_servo + self.cd_servo
        servo_pd = self.cp_servo + self.cd_servo
        self.servo_control = self.init_servo + servo_pd

        return self.servo_control

    def control_publisher(self):
        output_msg = Control()
        output_msg.thruster = self.thruster_pid_controller()
        output_msg.servo = self.servo_pid_controller()
        self.PID_controller_pub.publish(output_msg)


def main():
    rospy.init_node('PID_controller', anonymous=False)
    
    pid = PID()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        pid.control_publisher()
        rate.sleep()
        
    rospy.spin()

if __name__ == '__main__':
    main()
