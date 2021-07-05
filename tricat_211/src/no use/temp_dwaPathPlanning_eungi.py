#!/usr/bin/env python

import math
from enum import Enum
import numpy as np
import rospy

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import CircleObstacle
from obstacle_detector.msg import SegmentObstacle
from tricat_211.msg import LocalPosition
from tricat_211.msg import LocalHeading
from tricat_211.msg import FilteringObstacles
from tricat_211.msg import DWA


class Boat:
    def __init__(self):
        self.x = 0.0  # current x coordinate of boat
        self.y = 0.0  # current y coordinate of boat
        self.yaw = math.pi/8.0  # [rad] current rotation(yaw) of boat
        self.speed = 0.0  # [m/s] current speed of boat
        self.yaw_rate = 0.0  # [rad/s] 
        self.heading = 0.0  # heading angle(?)

        rospy.Subscriber("/local_position", LocalPosition, self.boat_xy_callback)
        rospy.Subscriber("/local_headingAngle", LocalHeading, self.heading_callback)

    def boat_xy_callback(self, msg):
        self.x = msg.x
        self.y = msg.y

    def heading_callback(self, msg):
        self.heading = msg.theta

    def boat_status(self):
        return np.array([self.x, self.y, self.yaw, self.speed, self.yaw_rate])


class Goal:
    def __init__(self):
        self.goal_list = np.array([[-1, -1],
                            [0, 2],
                            [4.0, 2.0],
                            [5.0, 4.0],
                            [5.0, 5.0],
                            [5.0, 6.0],
                            [5.0, 9.0],
                            [8.0, 9.0],
                            [7.0, 9.0],
                            [8.0, 10.0],
                            [9.0, 11.0],
                            [12.0, 13.0],
                            [12.0, 12.0],
                            [15.0, 15.0],
                            [13.0, 13.0]]
                            )

    def insert_goal_point(self):
        pass

    def remove_goal_point(self):
        pass
    def set_next_point(self):
        self.goal_list = np.delete(self.goal_list, 0, axis = 0)


class DWA_Calc:
    def __init__(self):
        dwa_config = rospy.get_param("DWA")

        self.max_speed = dwa_config['max_speed']  # [m/s]
        self.min_speed = dwa_config['min_speed']  # [m/s]
        self.max_yaw_rate = dwa_config['max_yaw_rate'] * math.pi / 180.0  # [rad/s]
        self.max_accel = dwa_config['max_accel']  # [m/ss]
        self.max_delta_yaw_rate = dwa_config['max_delta_yaw_rate'] * math.pi / 180.0  # [rad/ss]
        self.v_resolution = dwa_config['v_resolution']  # [m/s]
        self.yaw_rate_resolution = dwa_config['yaw_rate_resolution'] * math.pi / 180.0  # [rad/s]
        self.dt = dwa_config['dt']  # [s] Time tick for motion prediction
        self.predict_time = dwa_config['predict_time']  # [s]
        self.to_goal_cost_gain = dwa_config['to_goal_cost_gain']
        self.speed_cost_gain = dwa_config['speed_cost_gain']
        self.obstacle_cost_gain = dwa_config['obstacle_cost_gain']
        self.stuck_flag_cons = dwa_config['stuck_flag_cons']  # constant to prevent robot stucked

        self.boat_width = dwa_config['boat_width']  # [m] for collision check
        self.boat_length = dwa_config['boat_length']  # [m] for collision check
        self.goal_range = dwa_config['goal_range'] # [m]

        # self.obstacles_x = np.array([])
        # self.obstacles_y = np.array([])
        self.obstacles = np.array([])
        self.boat = Boat()
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal = [self.goal_x, self.goal_y]

        self.best_u = []

        self.dwa_pub = rospy.Publisher("/best_u", DWA, queue_size=10)
        #rospy.Publisher("/near_obstacle_list", DWA, queue_size=10)  # ????

    # def set_obstacle_list(self, ob_x_list, ob_y_list):
    #     self.obstacles = np.array([ob_x_list, ob_y_list])

    def arrival_check(self):
        dist_to_goal = math.hypot(self.boat.x - self.goal[0], self.boat.y - self.goal[1])
        if dist_to_goal <= self.goal_range:
            return True
        else:
            return False

    def DWAPublisher(self):
        self.dwa_pub.publish(self.best_u)

    def calc_dynamic_window(self):
        Vs = [self.min_speed, self.max_speed,
              -self.max_yaw_rate, self.max_yaw_rate]
        Vd = [self.boat.speed - self.max_accel * self.dt,
              self.boat.speed + self.max_accel * self.dt,
              self.boat.yaw_rate - self.max_delta_yaw_rate * self.dt,
              self.boat.yaw_rate + self.max_delta_yaw_rate * self.dt]
        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        return dw

    def calc_control_and_trajectory(self, dw):
        boat_init = np.array(self.boat.boat_status())
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        # best_trajectory = np.array([boat_init])

        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for yaw_rate in np.arange(dw[2], dw[3], self.yaw_rate_resolution):
                trajectory = self.predict_trajectory(boat_init, v, yaw_rate)

                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(trajectory)
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])
                ob_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(trajectory)

                final_cost = to_goal_cost + speed_cost + ob_cost

                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, yaw_rate]
                    if abs(best_u[0]) < self.stuck_flag_cons \
                            and abs(self.boat.speed) < self.stuck_flag_cons:
                        best_u[1] = -self.max_delta_yaw_rate

        return best_u

    def calc_obstacle_cost(self, trajectory):
        # dx = trajectory[:, 0] - self.obstacles_x
        # dy = trajectory[:, 1] - self.obstacles_y
        dx = trajectory[:, 0] - self.obstacles[:, 0]
        dy = trajectory[:, 1] - self.obstacles[:, 1]
        r = np.hypot(dx, dy)

        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])  #

        local_ob = self.obstacles[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob.dot(x) for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
       

        upper_check = local_ob[:, 0] <= self.boat_length / 2
        right_check = local_ob[:, 1] <= self.boat_width / 2
        bottom_check = local_ob[:, 0] >= -self.boat_length / 2
        left_check = local_ob[:, 1] >= -self.boat_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")

        min_r = np.min(r)
        return 1.0 / min_r

    def calc_to_goal_cost(self, trajectory):
        dx = self.goal[0] - trajectory[-1, 0]
        dy = self.goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost

    def dwa_control(self):
        dw = self.calc_dynamic_window()
        self.best_u = self.calc_control_and_trajectory(dw)

    def predict_trajectory(self, boat_init, v, yaw_rate):
        boat_pos = np.array(boat_init)
        trajectory = np.array(boat_pos)
        time = 0
        while time <= self.predict_time:
            boat_pos = self.motion(boat_pos, [v, yaw_rate])
            trajectory = np.vstack((trajectory, boat_pos))
            time += self.dt

        return trajectory

    def motion(self, boat_pos, u):
        boat_pos[2] += u[1] * self.dt
        boat_pos[0] += u[0] * math.cos(boat_pos[2]) * self.dt
        boat_pos[1] += u[0] * math.sin(boat_pos[2]) * self.dt
        boat_pos[3] = u[0]  
        boat_pos[4] = u[1]  

        return boat_pos


class Ob:
    def __init__(self):
        self.f_obstacle_list = []
        self.ob_list = np.array([])
        self.ob_x_list = np.array([])
        self.ob_y_list = np.array([])
        rospy.Subscriber('/filtering_obstacles', FilteringObstacles, self.filtering_obstacle_callback)

    def filtering_obstacle_callback(self, msg):
        self.f_obstacles = msg.circles
        # for i in range(len(self.f_obstacle_list)):
        #     np.append(self.ob_list, [self.f_obstacle_list[i].x, self.f_obstacle_list[i].y])

        for i in range(len(self.f_obstacle_list)):
            self.ob_x_list = np.append(self.ob_x_list, self.f_obstacle_list[i].x)
            self.ob_y_list = np.append(self.ob_y_list, self.f_obstacle_list[i].y)
        self.ob_list = np.column_stack([self.ob_x_list, self.ob_y_list])


def main():
    rospy.init_node('DWA', anonymous=False)
    rate = rospy.Rate(8)

    goal = Goal()
    obstacle = Ob()
    dwa_path = DWA_Calc()

    while not rospy.is_shutdown():
        if dwa_path.arrival_check():
            if len(goal.goal_list) == 0:
                break
            else:
                goal.set_next_point()
                # dwa_path.goal = goal.goal_list[0]
                dwa_path.goal_x = goal.goal_list[0][0]
                dwa_path.goal_y = goal.goal_list[0][1]

        # dwa_path.obstacles_x = obstacle.ob_x_list
        # dwa_path.obstacles_y = obstacle.ob_y_list
        dwa_path.obstacles = obstacle.ob_list
        dwa_path.dwa_control()
        dwa_path.DWAPublisher()
        
        rate.sleep(0.5)

    rospy.spin()

if __name__ == '__main__':
    main()