import pymap3d as pm
import numpy as np
import math

# def get_xy(lat, lon, alt):
#     e, n, u  = pm.geodetic2enu(lat, lon, alt, lat_00, lon_00, alt_00)

#     return e, n

def get_xy(points):
    way_list = np.zeros_like(points)
    for i in range(len(points)):
        way_list[i][0], way_list[i][1] , way_list[i][2] = pm.geodetic2enu(points[i][0], points[i][1], points[i][2], 37.448002351837694, 126.6533954619325, 20.0)
            #need start point GPS
    way_list = np.delete(way_list, 2, axis=1)
    return way_list

# start
lat_00 = 37.448002351837694
lon_00 = 126.6533954619325
alt_00 = 20.0

# waypoint
way_list = np.array([[37.44801141028958, 126.6534866483131, 20.0],
                                      [37.44807209873907, 126.65347927223846, 20.0],
                                      [37.448090731147886, 126.65353492807438, 20.0],
                                      [37.44799277786102, 126.65345513235782, 20.0]])

print(get_xy(way_list))







# class Goal:
#     def __init__(self):
#         #temp list // should sort the elements??
#         # gps dd waypoint
#         self.waypoint_array = np.array([(l,l,a),
#                 (l,l,a),
#                 (l,l,a),
#                 (l,l,a),
#                 (l,l,a)])

#         self.goal_list = get_xy(self.waypoint_array)
#         # self.goal_list = np.array([[-1, -1],
#         #                     [get_xy(1)[0], get_xy(1)[1]],
#         #                     [get_xy(2)[0], get_xy(2)[1]],
#         #                     [5.0, 4.0],
#         #                     [5.0, 5.0],
#         #                     [5.0, 6.0],
#         #                     [5.0, 9.0],
#         #                     [8.0, 9.0],
#         #                     [7.0, 9.0],
#         #                     [8.0, 10.0],
#         #                     [9.0, 11.0],
#         #                     [12.0, 13.0],
#         #                     [12.0, 12.0],
#         #                     [15.0, 15.0],
#         #                     [13.0, 13.0]]
#         #                     )
