#!/usr/bin/env python

import rospy
import math

from sensor_msgs.msg import Imu, NavSatFix
from ublox_msgs.msg import NavPVT
from tricat_211.msg import Control, DWA  # thruster, servo, v, yaw_rate float64
from std_msgs.msg import UInt16           # for `Output`

class PID:
    def __init__(self):
        PID_config = rospy.get_param("PID")

        #self.angular_v_x = 0.0
        #self.angular_v_y = 0.0
        self.init_thruster = 1500
        self.init_servo = 150

        self.thruster_pwm = 1500
        self.servo_control = 150

        self.cur_yaw_rate = 0.0
        self.cur_v = 0.0
    
        self.optimal_v = 0.0
        self.optimal_yaw_rate = 0.0

        self.kp_thruster = PID_config['kp_thruster']
        self.ki_thruster = PID_config['ki_thruster']
        self.kd_thruster = PID_config['kd_thruster']
        self.error_v = 0.0
        self.error_v_sum = 0.0
        self.prev_v = 0.0 # what is this

        self.kp_servo = PID_config['kp_servo']
        self.ki_servo = PID_config['ki_servo']
        self.kd_servo = PID_config['kd_servo']
        self.error_yaw_rate = 0.0
        self.error_yaw_rate_sum = 0.0
        self.prev_yaw_rate = 0.0 # what is this

        self.rate = PID_config['rate']

        rospy.Subscriber("/imu/data", Imu, self.imu_data_callback)
        rospy.Subscriber("/ublox_gps/navpvt", NavPVT, self.gps_data_callback)
        rospy.Subscriber("/best_U", DWA, self.DWA_data_callback)

        self.Servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)

    def imu_data_callback(self, data):
        #self.angular_v_x = data.angular_velocity.x
        #self.angular_v_y = data.angular_velocity.y
        self.cur_yaw_rate = data.angular_velocity.z

    def gps_data_callback(self, data):
        self.cur_v = data.gSpeed * 0.001 # mm/s -> m/s

    def DWA_data_callback(self, data):
        self.optimal_v = data.v
        self.optimal_yaw_rate = data.yaw_rate

    def thruster_pid_controller(self):
        self.error_v = self.optimal_v - self.cur_v

        self.cp_thruster = self.kp_thruster * self.error_v

        self.ci_thruster = 0

        v_derivative = (self.cur_v - self.prev_v) / 0.05 # division 1 / self.rate  # replaced with accel for drivative kick
        self.prev_v = self.cur_v
        self.cd_thruster = self.ki_thruster * -v_derivative

        self.thruster_pwm = self.thruster_pwm + self.cp_thruster + self.cd_thruster# + self.ci_thruster 

        return self.thruster_pwm

    def servo_pid_controller(self):
        self.error_yaw_rate = self.optimal_yaw_rate - self.cur_yaw_rate

        self.cp_servo = self.kp_servo * self.error_yaw_rate

        #self.error_yaw_rate_sum = self.error_yaw_rate_sum + self.error_yaw_rate * (1 / self.rate)
        #if motor is off :
        #    self.error_yaw_rate_sum = 0
        #self.ci_servo =  self.kd_servo * self.error_yaw_rate_sum
 
        yaw_rate_derivative = (self.cur_yaw_rate - self.prev_yaw_rate) / 0.05 # division 1 / self.rate
        self.prev_yaw_rate = self.cur_yaw_rate
        self.cd_servo = self.ki_servo * -yaw_rate_derivative

        #servo_pid = self.cp_servo+ self.ci_servo + self.cd_servo
        servo_pd = self.cp_servo + self.cd_servo
        self.servo_control = self.servo_control + servo_pd

        return self.servo_control

    def control_publisher(self):
        output_msg = Control()
        output_msg.thruster = round(self.thruster_pid_controller())
        output_msg.servo = round(self.servo_pid_controller())
        self.thruster_pub.publish(output_msg.thruster)
        self.Servo_pub.publish(output_msg.servo)
        


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
