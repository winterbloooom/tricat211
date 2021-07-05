'''#!/usr/bin/env python

import math
from enum import Enum
import numpy as np
import rospy

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
    #std_msgs/Header header
    #obstacle_detector/SegmentObstacle[] segments
    #obstacle_detector/CircleObstacle[] circles
from obstacle_detector.msg import CircleObstacle
    #geometry_msgs/Point center
    #   float64 x y z
    #float64 radius
from obstacle_detector.msg import SegmentObstacle
    #geometry_msgs/Point first_point
    #   float64 x y z
    #geometry_msgs/Point last_point
    #   float64 x y z
from tricat_211.msg import LocalPosition
from tricat_211.msg import LocalHeading
from tricat_211.msg import NearObstacle
from tricat_211.msg import DWA


dwa_config = rospy.get_param("DWA")

class Config:
    def __init__(self):
        # robot parameter
        self.max_speed = dwa_config['max_speed']  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked

        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check
        
        self.goal_field = 3 # [m] for goal check

        self.ob = np.array([]) # obstacle list -> keep update


class ObstacleJudge:
    def __init__(self):
        self.decision_range = 5 # [m] for obstacle_judge check range

        self.obstacle_list = [] #get list from lidar
        self.ob_list = np.zeros([len(self.obstacle_list), 4])  # obstacle list - distance from boat(x, y, d), theta
                                                               # (obstacle number)*4

        rospy.Subscriber('/obstacles', Obstacles, self.obstacle_callback) # subtopic,msgtype, callback /// 이 메시지 안에 circles[]있음

    def obstacle_callback(self, Obstacles):
        self.obstacle_list = Obstacles.circles #.center.x/.center.y/.center.z/.radius
        self.segment_list = Obstacles.segments #.first_point.x/.first_point.y/.first_point.z/last_point

    def obstacle_judge(self):
        if len(self.obstacle_list) == 0:

            return

        for i in range(len(self.obstacle_list)):
            ob = self.obstacle_list
            ob_x = -ob[i].center.x
            ob_y = ob[i].center.y
            ob_d = math.sqrt(ob_x ** 2 + ob_y ** 2)
            ob_rad = ob[i].radius # dwa 사용하면 필요 없을까?
            ob_info = [ob_x, ob_y, ob_d, ob_rad]
            self.ob_list[i] = ob_info
            
        #distance decision , sorting
        for i in range(len(self.obstacle_list)) :

            if distance_decision(self.decision_range, self.ob_list[i, 2]):
                continue
            else:
                np.delete(self.ob_list, i)
            #sorting
        
        return self.ob_list



'''함수목록
레퍼런스 복사로 잘 작동하나 확인
 def swap(a, b):
     temp = a
     a = b
     b = temp'''



def boat_xy_callback(msg):
    boat_x = msg.x
    boat_y = msg.y

def heading_callback(msg):
    boat_heading = msg.theta

rospy.Subscriber("/local_position", LocalPosition, boat_xy_callback)
rospy.Subscriber("/local_headingAngle", LocalHeading, heading_callback)





def distance_decision(decision_range, distance):
    if 0 < distance < decision_range :
        return True
    else:
        return False


# DWA control을 계산해 u와 trajectory 리턴
def dwa_control(x, config, goal, ob):
    dw = calc_dynamic_window(x, config) #현재 위치 기반 dw
    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


#현 위치 기준으로 dw 계산해 리턴
def calc_dynamic_window(x, config):
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]
    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


#dw 기반으로 최종 input을 계산해 best_u[v, yaw]와 best_trajctory 리턴
def calc_control_and_trajectory(x, dw, config, goal, ob):
    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for yaw_rate in np.arange(dw[2], dw[3], config.yaw_rate_resolution):
            trajectory = predict_trajectory(x_init, v, yaw_rate, config)
            #cost 계산
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)
            #cost의 합
            final_cost = to_goal_cost + speed_cost + ob_cost 

            # 해당 v, yaw_rate에서의 최소 경로 탐색
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, yaw_rate]
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -config.max_delta_yaw_rate

    return best_u, best_trajectory


#장애물 cost 계산(inf:collision)
def calc_obstacle_cost(trajectory, ob, config):
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    #여기 뭔 말인지 해석 필요
    yaw = trajectory[:, 2]
    rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    rot = np.transpose(rot, [2, 0, 1])
    local_ob = ob[:, None] - trajectory[:, 0:2]
    local_ob = local_ob.reshape(-1, local_ob.shape[-1])
    local_ob = np.array([local_ob @ x for x in rot])
    local_ob = local_ob.reshape(-1, local_ob.shape[-1])
    upper_check = local_ob[:, 0] <= config.robot_length / 2
    right_check = local_ob[:, 1] <= config.robot_width / 2
    bottom_check = local_ob[:, 0] >= -config.robot_length / 2
    left_check = local_ob[:, 1] >= -config.robot_width / 2
    if (np.logical_and(np.logical_and(upper_check, right_check),
                       np.logical_and(bottom_check, left_check))).any():
        return float("Inf")

    min_r = np.min(r)
    return 1.0 / min_r


#서로 다른 각도에서 목표점 cost 계산
def calc_to_goal_cost(trajectory, goal):
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost


#현 위치 갱신(x리스트 리턴)
def motion(x, u, dt):
    x[2] += u[1] * dt  # x의 yaw += u의 각속도 * dt
    x[0] += u[0] * math.cos(x[2]) * dt  # x의 x좌표 += u의 선속 * cos(x의 yaw) * dt
    x[1] += u[0] * math.sin(x[2]) * dt  # x의 y좌표 += u의 선속 * sin(x의 yaw) * dt
    x[3] = u[0]  # x의 선속도 = u의 선속도
    x[4] = u[1]  # x의 각속도 = u의 각속도

    return x


#경로 추정해 trajectory 리턴
def predict_trajectory(x_init, v, yaw_rate, config):
    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, yaw_rate], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory

self.dwa_pub = rospy.Publisher("/best_trajectory", DWA, queue_size=10)
rospy.Publisher("/near_obstacle_list", DWA, queue_size=10)


def DWAPubisher():
    dwa = DWA()
    dwa.u = best_u[0]
    dwa.yaw = best_u[1]
    self.dwa_pub.publish(dwa)


def convert_lidar_ob(x, ob_list):
    boat_x = x[0]
    boat_y = x[1]
    converted_ob_list = np.array([])
    for i in range(len(ob_list)):
        if ob_list[i][0] > 0:   # 배로부터 장애물의 x거리 값
            obstacle_x = boat_x + ob_list[i][0]
        else:
            obstacle_x = boat_x - ob_list[i][0]

        if ob_list[i][1] > 0:   # 배로부터 장애물의 y거리 값
            obstacle_y = boat_y + ob_list[i][1]
        else:
            obstacle_y = boat_y - ob_list[i][1]

        np.append(converted_ob_list, [obstacle_x, obstacle_y])

    return converted_ob_list


def start_point():
    """시작지점의 위치를 받아오는 함수. 현 대입값은 임시"""
    start_x = boat_x
    start_y = boat_y
    start_yaw = math.pi / 8.0
    return np.array([start_x, start_y, start_yaw, 0.0, 0.0])


def goal_point():
    """목표지점의 위치를 받아오는 함수. 현 대입값은 임시"""
    goal_x = 10.0
    goal_y = 10.0
    return np.array([goal_x, goal_y])


def main():
    rospy.init_node('Near_Judge', anonymous = False)  #여기 확인!
    rate = rospy.Rate(8)

    detected_obstacles = ObstacleJudge()
    x = start_point() #x의 변수명을 바꾸는 것을 생각해보기
    goal = goal_point()
    trajectory = np.array(x)
    config = Config()

    while not rospy.is_shutdown():
        #일단 라이다에서부터 x거리, y거리만 사용. DWA에서 각도는 알아서 계산하니.
        ob_list = detected_obstacles.obstacle_judge()
        config.ob = convert_lidar_ob(x, ob_list) #obstacle의 클래스를 따로 만드는 것을 생각해보기. 해당 클래스 내에서 좌표 변환, 계산, 업데이트 기능 추가
        rate.sleep()
        
        #dwa, trajectory 갱신
        u, predicted_trajectory = dwa_control(x, config, goal, config.ob)
        x = motion(x, u, config.dt)
        trajectory = np.vstack((trajectory, x))

        """여기에 thruster를 조정하는 코드를 연결해야 함."""

        # 목표점에 도착했는지 판단
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.goal_field:
            break
    rospy.spin()


if __name__ == '__main__':
    main()'''