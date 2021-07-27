#!/usr/bin/env python

import math
from enum import Enum # delete check plz
import numpy as np
import rospy
import pymap3d as pm

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles, CircleObstacle, SegmentObstacle
from geometry_msgs.msg import Point
from tricat_211.msg import HeadingAngle, FilteringObstacles, FilteringWalls, DWA
from tricat_211.msg import FilteringWallsDWA, WallParticle

class Boat:
    def __init__(self):
        self.x = 0.0  # current x coordinate of boat
        self.y = 0.0  # current y coordinate of boat
        self.yaw = math.pi/8.0  # [rad] current rotation(yaw) of boat          initial yaw
        self.speed = 0.0  # [m/s] current speed of boat
        self.yaw_rate = 0.0  # [rad/s] 
        self.heading = 0.0  # heading angle(?)               # initial yaw = initial heading

        rospy.Subscriber("/enu_position", Point, self.boat_xy_callback)
        rospy.Subscriber("/bearing", HeadingAngle, self.heading_callback)

    def boat_xy_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    def heading_callback(self, msg):
        self.heading = msg.theta
        self.bearing = msg.bearing

    def boat_status(self):
        return np.array([self.x, self.y, self.yaw, self.speed, self.yaw_rate])


class Goal:
    def __init__(self):
        self.map_list = rospy.get_param("map_dd")

        self.origin = [self.map_list['map_00_lat'], self.map_list['map_00_lon'], self.map_list['map_00_alt']]
        # self.lat_00, self.lon_00, self.alt_00 = self.map_list['map_00_lat'], self.map_list['map_00_lon'], self.map_list['map_00_alt']
        
        self.waypoints = rospy.get_param("waypoint_List/waypoints")
        self.way_list_gps = np.empty((0,3), float)
        for i in range(len(self.waypoints)):
            self.way_list_gps = np.append(self.way_list_gps, np.array([self.waypoints[i]]), axis=0)
            
        self.way_list = get_xy(self.way_list_gps, self.origin)
        
    

    def set_next_point(self):
        self.way_list = np.delete(self.way_list, 0, axis = 0)


class DWA_Calc:
    def __init__(self):
        dwa_config = rospy.get_param("DWA")
        edge_points = rospy.get_param("edge_dd")

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
        self.safey_radius = dwa_config['safey_distance']

        self.boat_width = dwa_config['boat_width']  # [m] for collision check
        self.boat_length = dwa_config['boat_length']  # [m] for collision check
        self.goal_range = dwa_config['goal_range'] # [m]

        self.edge1 = [edge_points['map_00_lat'], edge_points['map_00_lon'], 20]
        self.edge2 = [edge_points['map_0y_lat'], edge_points['map_0y_lon'], 20]
        self.edge3 = [edge_points['map_x0_lat'], edge_points['map_x0_lon'], 20]
        self.edge4 = [edge_points['map_xy_lat'], edge_points['map_xy_lon'], 20]
        self.edge_points = [self.edge1, self.edge2, self.edge3, self.edge4]

        self.class_obstacle = Ob()
        self.obstacles = self.class_obstacle.ob_list
        # self.walls = self.class_obstacle.w_list
        self.boat = Boat()
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal = [self.goal_x, self.goal_y]

        self.best_u = []

        self.dwa_pub = rospy.Publisher("/best_U", DWA, queue_size=10)

    def obstacle_init(self):
        self.obstacles = self.class_obstacle.ob_list
        # self.walls = self.class_obstacle.w_list

    def arrival_check(self):
        dist_to_goal = math.hypot(self.boat.x - self.goal[0], self.boat.y - self.goal[1])
        if dist_to_goal <= self.goal_range:
            return True
        else:
            return False

    def DWAPublisher(self):
        dwa = DWA()
        dwa.v = self.best_u[0]
        dwa.yaw_rate = self.best_u[1]
        self.dwa_pub.publish(dwa)

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

        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for yaw_rate in np.arange(dw[2], dw[3], self.yaw_rate_resolution):
                trajectory = self.predict_trajectory(boat_init, v, yaw_rate)

                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(trajectory)
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])

                if len(self.obstacles) != 0 : #or len(self.walls)
                    ob_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(trajectory)
                    final_cost = to_goal_cost + speed_cost + ob_cost # 1>>>  2>  3>>
                else:
                    final_cost = to_goal_cost + speed_cost

                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, yaw_rate]
                    if abs(best_u[0]) < self.stuck_flag_cons \
                            and abs(self.boat.speed) < self.stuck_flag_cons:
                        best_u[1] = -self.max_delta_yaw_rate

        return best_u

    def calc_obstacle_cost(self, trajectory):
        dx = np.array([])
        dy = np.array([])
        min_r = 100000

        for i in range(len(trajectory)):
            for j in range(len(self.obstacles)):
                dx = trajectory[i, 0] - self.obstacles[j, 0]
                dy = trajectory[i, 1] - self.obstacles[j, 1]
                r = np.hypot(dx, dy)
                if min_r >= r:
                    min_r = r
        
        # for i in range(len(trajectory)):
        #     for j in range(len(self.walls)):
        #         dx = trajectory[i, 0] - self.walls[j, 0]
        #         dy = trajectory[i, 1] - self.walls[j, 1]
        #         r = np.hypot(dx, dy)
        #         if min_r >= r:
        #             min_r = r

        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        
        local_ob = self.obstacles[:, None] - trajectory[:, 0:2]

        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob.dot(x) for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])

        upper_check = local_ob[:, 0] <= 0.3  #set with param?????????????
        right_check = local_ob[:, 1] <= self.boat_width / 2
        bottom_check = local_ob[:, 0] >= -self.boat_length
        left_check = local_ob[:, 1] >= -self.boat_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")

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
            temp_boat_pos = boat_pos
            temp_boat_pos[2] += yaw_rate * self.dt # yaw_rate yaw
            temp_boat_pos[0] += v * math.cos(temp_boat_pos[2]) * self.dt # v x
            temp_boat_pos[1] += v * math.sin(temp_boat_pos[2]) * self.dt # v y
            temp_boat_pos[3] = v  # v v
            temp_boat_pos[4] = yaw_rate  # yaw_rate yaw_rate

            if self.check_line_out([boat_pos[0], boat_pos[1]]): #go out
                continue
            else:               
                trajectory = np.vstack((trajectory, temp_boat_pos))
                boat_pos = temp_boat_pos

            time += self.dt

        return trajectory

    def check_line_out(self, point):
        crossed = 0
        d_line_point = float("Inf")

        for i in range(4):
            j = (i+1)%4
            if(self.edge_points[i][1] > point[1]) != (self.edge_points[j][1] > point[1]):
                intersection = float((self.edge_points[j][0] - self.edge_points[i][0]) * (point[1]-self.edge_points[i][1]) \
                     / (self.edge_points[j][1] - self.edge_points[i][1]) + self.edge_points[i][0])
                if point[0] < intersection:
                    crossed += 1
                area = abs((self.edge_points[i][0] - point[0]) * (self.edge_points[j][1] - point[1]) \
                    - (self.edge_points[i][1] - point[1]) * (self.edge_points[j][0] - point[0]))
                p_to_p = math.sqrt((self.edge_points[i][0] - self.edge_point[j][0]) ** 2 + (self.edge_points[i][1] - self.edge_points[j][1]) ** 2 )
                d = area / p_to_p
                if d < d_line_point:
                    d_line_point = d

        if (crossed % 2) == 0 or d < self.safey_distance:
            return True #go out
        else :
            return False


class Ob:
    def __init__(self):
        self.ob_list = np.array([])
        self.ob_x_list = np.array([])
        self.ob_y_list = np.array([])

        # self.w_list = np.array([])
        # self.w_start_x_list = np.array([])
        # self.w_start_y_list = np.array([])
        # self.w_end_x_list = np.array([])
        # self.w_end_y_list = np.array([])
        self.w_list = np.array([])
        self.w_x_list = np.array([])
        self.w_y_list = np.array([])

        rospy.Subscriber('/filtering_obstacles', FilteringObstacles, self.filtering_obstacle_callback)
        # rospy.Subscriber('/filtering_walls', FilteringWalls, self.filtering_wall_callback)
        rospy.Subscriber('/filtering_walls', FilteringWallsDWA, self.filtering_wall_callback)

    def filtering_obstacle_callback(self, msg):
        f_obstacles = msg.circles
        for i in range(len(f_obstacles)):
            self.ob_x_list = np.append(self.ob_x_list, f_obstacles[i].x)
            self.ob_y_list = np.append(self.ob_y_list, f_obstacles[i].y)
        self.ob_list = np.column_stack([self.ob_x_list, self.ob_y_list])

    # def filtering_wall_callback(self, msg):
    #     f_walls = msg.walls
    #     for i in range(len(self.f_walls)):
    #         self.w_start_x_list = np.append(self.w_start_x_list, f_walls[i].start_x)
    #         self.w_start_y_list = np.append(self.w_start_y_list, f_walls[i].start_y)
    #         self.w_end_x_list = np.append(self.w_end_x_list, f_walls[i].end_x)
    #         self.w_end_y_list = np.append(self.w_end_x_list, f_walls[i].end_y)
    #     self.w_list = np.column_stack([self.w_start_x_list, self.w_start_y_list, self.w_end_x_list, self.w_end_y_list])

    def filtering_wall_callback(self, msg):
        f_walls = msg.particle
        for i in range(len(f_walls)):
            self.w_x_list = np.append(self.w_x_list, f_walls[i].x)
            self.w_y_list = np.append(self.w_y_list, f_walls[i].y)
        self.ob_list = np.column_stack([self.w_x_list, self.w_y_list])


def get_xy(points, origin):
    way_list = np.zeros_like(points)
    for i in range(len(points)):
        way_list[i][0], way_list[i][1] , way_list[i][2] = \
            pm.geodetic2enu(points[i][0], points[i][1], points[i][2], \
                origin[0], origin[1], origin[2])
    way_list = np.delete(way_list, 2, axis=1)
    return way_list


def shutdownhook():
    print("End the programe") #no meaning


def main():
    rospy.init_node('DWA', anonymous=False)
    rate = rospy.Rate(1)

    goal = Goal()
    dwa_path = DWA_Calc()

    dwa_path.goal_x = goal.way_list[0][0]
    dwa_path.goal_y = goal.way_list[0][1]
    
    while not rospy.is_shutdown():
        print(dwa_path.obstacles)        
        print("----")

        if dwa_path.arrival_check():
            if len(goal.way_list) == 0:
                rospy.on_shutdown(shutdownhook)
                break  #arrived final goal
            elif len(goal.way_list) == 1:
                dwa_path.goal_x = goal.way_list[0][0]
                dwa_path.goal_y = goal.way_list[0][1]
                goal.set_next_point()
            else:
                goal.set_next_point()
                dwa_path.goal_x = goal.way_list[0][0]
                dwa_path.goal_y = goal.way_list[0][1]
                

        dwa_path.obstacle_init()
        dwa_path.dwa_control()
        dwa_path.DWAPublisher()
        
        rate.sleep()
        # rospy.sleep(1)

    rospy.spin()

if __name__ == '__main__':
    main()