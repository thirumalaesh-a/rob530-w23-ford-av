import rosbag
import rospy
import math
from math import sin, cos
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu, NavSatFix, TimeReference
from geometry_msgs.msg import PoseStamped
from enum import Enum
from utils import *

STATE_SIZE = 15

class states(Enum):
    x = 0
    y = 1
    z = 2
    roll = 3
    pitch = 4
    yaw = 5
    vel_x = 6
    vel_y = 7
    vel_z = 8
    vel_roll = 9
    vel_pitch = 10
    vel_yaw = 11
    acc_x = 12
    acc_y = 13
    acc_z = 14


##TO BE TESTED note: coming in, state should be in ENU frame 
def get_propagation_matrix(state, delta):

    pose = state[0:3]
    roll, pitch, yaw = state[3:6]

    sin_pitch = sin(pitch)
    cos_pitch = cos(pitch)
    inv_cos_pitch = 1 / cos_pitch
    tan_pitch = sin_pitch * inv_cos_pitch

    sin_roll = sin(roll)
    cos_roll = cos(roll)

    sin_yaw = sin(yaw)
    cos_yaw = cos(yaw)

    transfer_matrix = np.eye(STATE_SIZE)

    transfer_matrix[states.x.value, states.vel_x.value] = cos_yaw * cos_pitch * delta
    transfer_matrix[states.x.value, states.vel_y.value] = (cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) * delta
    transfer_matrix[states.x.value, states.vel_z.value] = (cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll) * delta
    transfer_matrix[states.x.value, states.acc_x.value] = 0.5 * transfer_matrix[states.x.value, states.vel_x.value] * delta
    transfer_matrix[states.x.value, states.acc_y.value] = 0.5 * transfer_matrix[states.x.value, states.vel_y.value] * delta
    transfer_matrix[states.x.value, states.acc_z.value] = 0.5 * transfer_matrix[states.x.value, states.vel_z.value] * delta


    transfer_matrix[states.y.value, states.vel_x.value] = sin_yaw * cos_pitch * delta
    transfer_matrix[states.y.value, states.vel_y.value] = (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) * delta
    transfer_matrix[states.y.value, states.vel_z.value] = (sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll) * delta
    transfer_matrix[states.y.value, states.acc_x.value] = 0.5 * transfer_matrix[states.y.value, states.vel_x.value] * delta
    transfer_matrix[states.y.value, states.acc_y.value] = 0.5 * transfer_matrix[states.y.value, states.vel_y.value] * delta
    transfer_matrix[states.y.value, states.acc_z.value] = 0.5 * transfer_matrix[states.y.value, states.vel_z.value] * delta

    transfer_matrix[states.z.value, states.vel_x.value] = - sin_pitch * delta
    transfer_matrix[states.z.value, states.vel_y.value] = (cos_pitch * sin_roll) * delta
    transfer_matrix[states.z.value, states.vel_z.value] = (cos_pitch * cos_roll) * delta
    transfer_matrix[states.z.value, states.acc_x.value] = 0.5 * transfer_matrix[states.z.value, states.vel_x.value] * delta
    transfer_matrix[states.z.value, states.acc_y.value] = 0.5 * transfer_matrix[states.z.value, states.vel_y.value] * delta
    transfer_matrix[states.z.value, states.acc_z.value] = 0.5 * transfer_matrix[states.z.value, states.vel_z.value] * delta

    transfer_matrix[states.roll.value, states.vel_roll.value] = delta
    transfer_matrix[states.roll.value, states.vel_pitch.value] = sin_roll * tan_pitch * delta
    transfer_matrix[states.roll.value, states.vel_yaw.value] = cos_roll * tan_pitch * delta

    transfer_matrix[states.pitch.value, states.vel_pitch.value] = cos_roll * delta
    transfer_matrix[states.pitch.value, states.vel_yaw.value] = -sin_roll * delta

    transfer_matrix[states.yaw.value, states.vel_pitch.value] = sin_roll * inv_cos_pitch * delta
    transfer_matrix[states.yaw.value, states.vel_yaw.value] = cos_roll * inv_cos_pitch * delta
    
    transfer_matrix[states.vel_x.value, states.acc_x.value] = delta
    transfer_matrix[states.vel_y.value, states.acc_y.value] = delta
    transfer_matrix[states.vel_z.value, states.acc_z.value] = delta

    val = np.matmul(transfer_matrix, state.reshape((15,1))).reshape((15,))
    val = val + (np.random.randn(15) * 1)

    return val


