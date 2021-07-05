#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Imu
from tricat_211.msg import HeadingAngle
import tf

class Local_Heading:
    def __init__(self):
        self.angular_v_x = 0.0
        self.angular_v_y = 0.0
        self.angular_v_z = 0.0

        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 0.0
        
        rospy.Subscriber("/imu/data", Imu, self.imu_data_callback)

        self.bearing_pub = rospy.Publisher("/bearing", HeadingAngle, queue_size=10)

    def imu_data_callback(self, data):
        self.angular_v_x = data.angular_velocity.x
        self.angular_v_y = data.angular_velocity.y
        self.angular_v_z = data.angular_velocity.z
        self.orientation_x = data.orientation.x
        self.orientation_y = data.orientation.y
        self.orientation_z = data.orientation.z
        self.orientation_w = data.orientation.w

    def anglePublisher(self):
        radian = HeadingAngle()
        
        quaternion = (self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w)
        
        euler = tf.transformations.euler_from_quaternion(quaternion)
        
        euler_x_deg = euler[0] * 180 / math.pi # roll
        euler_y_deg = euler[1] * 180 / math.pi # pitch
        euler_z_deg = euler[2] * 180 / math.pi # yaw

        euler_z_deg += 0
        radian.bearing = euler_z_deg % 360

        self.bearing_pub.publish(radian)
        
        #return radian.bearing

def main():
    rospy.init_node('IMU_Converter', anonymous=False)
    l = Local_Heading()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        l.anglePublisher()
        #rospy.loginfo(l.anglePublisher())
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
