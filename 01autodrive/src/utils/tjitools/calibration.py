from math import cos
from math import sin
from math import atan2
import numpy as np

class coordinate_transformation_2D():
    def __init__(self, obstacle1_car_xyz, obstacle1_radar_xyz, obstacle2_car_xyz, obstacle2_radar_xyz):
        self.angle_err = None
        self.x_err = None
        self.y_err = None
        self.R_matrix = None
        self.T_matrix = None
        self.R_T_matrix = None
        self.calculate_parameters(obstacle1_car_xyz, obstacle1_radar_xyz, obstacle2_car_xyz, obstacle2_radar_xyz)

    def calculate_parameters(self, obstacle1_car_xyz, obstacle1_radar_xyz, obstacle2_car_xyz, obstacle2_radar_xyz):

        self.angle_err = 0
        self.x_err = 0
        self.y_err = 0
        self.R_matrix = np.eye(3)
        self.T_matrix = np.eye(3)

        angle1 = atan2((obstacle2_car_xyz[1] - obstacle1_car_xyz[1]), (obstacle2_car_xyz[0] - obstacle1_car_xyz[0]))
        angle2 = atan2((obstacle2_radar_xyz[1] - obstacle1_radar_xyz[1]),
                       (obstacle2_radar_xyz[0] - obstacle1_radar_xyz[0]))
        self.angle_err = angle1 - angle2

        self.x_err = obstacle2_car_xyz[0] - (obstacle2_radar_xyz[0] * cos(self.angle_err) - obstacle2_radar_xyz[1] * sin(self.angle_err))
        self.y_err = obstacle2_car_xyz[1] - (obstacle2_radar_xyz[0] * sin(self.angle_err) + obstacle2_radar_xyz[1] * cos(self.angle_err))

        self.T_matrix[0][2] = self.x_err
        self.T_matrix[1][2] = self.y_err

        self.R_matrix[0][0] = cos(self.angle_err)
        self.R_matrix[0][1] = -sin(self.angle_err)
        self.R_matrix[1][0] = sin(self.angle_err)
        self.R_matrix[1][1] = cos(self.angle_err)
        self.R_T_matrix = np.dot(self.T_matrix, self.R_matrix)

    def XYZ_translate_single(self, obstacle_xyz):
        assert type(obstacle_xyz) is list or type(obstacle_xyz) is np.ndarray
        obstacle_xyz_numpy = np.ones(3)
        obstacle_xyz_numpy[0:2] = obstacle_xyz[0:2]
        car_xyz = np.dot(self.R_T_matrix, obstacle_xyz_numpy)
        car_xyz[2] = obstacle_xyz[2]
        if type(obstacle_xyz) is list:
            return car_xyz.tolist()
        else:
            return car_xyz

    def XYZ_translate_multi(self, obstacles_xyz):
        assert type(obstacles_xyz) is list or type(obstacles_xyz) is np.ndarray

        if type(obstacles_xyz) is np.ndarray:
            obstacles_xyz_numpy = np.ones_like(obstacles_xyz)
            obstacles_xyz_numpy[:, 0:2] = obstacles_xyz[:, 0:2]
        else:
            obstacles_xyz_numpy = np.ones_like(obstacles_xyz)
            obstacles_xyz_numpy[:, 0:2] = np.array(obstacles_xyz)[:, 0:2]

        assert obstacles_xyz_numpy.shape[0] > 0 and obstacles_xyz_numpy.shape[1] == 3
        obstacles_car_xyz = np.dot(self.R_T_matrix, obstacles_xyz_numpy.T).T


        if type(obstacles_xyz) is np.ndarray:
            obstacles_car_xyz[:, -1] = obstacles_xyz[:, -1]
            return obstacles_car_xyz
        else:
            obstacles_car_xyz[:, 2:3] = np.array(obstacles_xyz)[:, 2:3]
            return obstacles_car_xyz.tolist()
