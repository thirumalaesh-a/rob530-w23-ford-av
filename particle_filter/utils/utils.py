import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm, logm

from system.RobotState import *


PI = 3.14159265358979323846
def wrap2Pi(input):
    phases =  (( -input + np.pi) % (2.0 * np.pi ) - np.pi) * -1.0

    return phases

def wrap_all_angles(rot_vec):
    # @brief: clamps all values of a (3,) rotation vector to 0 - 360
    
    return_vec = np.zeros(3)
    return_vec[0] = wrap2Pi(rot_vec[0])
    return_vec[1] = wrap2Pi(rot_vec[1])
    return_vec[2] = wrap2Pi(rot_vec[2])

    return return_vec

def plot_error(results, gt):
    num_data_range = range(np.shape(results)[0])

    gt_x = gt[:,0]
    gt_y = gt[:,1]

    plot2 = plt.figure(2)
    plt.plot(num_data_range,results[:,0])
    plt.plot(num_data_range, 7.81*np.ones(np.shape(results)[0]))
    plt.title("Chi-square Statistics")
    plt.legend(["Chi-square Statistics", "p = 0.05 in 3 DOF"])
    plt.xlabel("Iterations")

    plot3,  (ax1, ax2, ax3) = plt.subplots(3,1)
    ax1.set_title("Deviation from Ground Truth with 3rd Sigma Contour")
    ax1.plot(num_data_range, results[:,1])
    ax1.set_ylabel("X")
    ax1.plot(num_data_range,results[:,4],'r')
    ax1.plot(num_data_range,-1*results[:,4],'r')
    
    ax1.legend(["Deviation from Ground Truth","3rd Sigma Contour"])
    ax2.plot(num_data_range,results[:,2])
    ax2.plot(num_data_range,results[:,5],'r')
    ax2.plot(num_data_range,-1*results[:,5],'r')
    ax2.set_ylabel("Y")

    ax3.plot(num_data_range,results[:,3])
    ax3.plot(num_data_range,results[:,6],'r')
    ax3.plot(num_data_range,-1*results[:,6],'r')
    ax3.set_ylabel("theta")
    ax3.set_xlabel("Iterations")
    
    plt.show()


def mat_from_quat(q):
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    mat = np.zeros((3,3))
    d = np.linalg.norm(q)
    assert d!= 0
    s = 2/d
    xs, ys, zs = x * s,  y * s,  z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs

    mat[0,:] = 1 - (yy+zz), xy - wz, xz + wy
    mat[1,:] = xy + wz, 1 - (xx+zz), yz - wx
    mat[2,:] = xz - wy, yz + wx, 1 - (xx+yy)

    return mat

def getRPY(mat):
    vec = getEulerYPR(mat)
    yaw, pitch, roll = vec[0], vec[1], vec[2]

    return yaw, pitch, roll

def getEulerYPR(mat, sol= 1):
    x = 0
    y = 1
    z = 2
    yaw = 0
    pitch = 1
    roll = 2 
    euler_out = np.empty(3) ##YAW, PITCH, ROLL
    euler_out2 = np.empty(3) ##YAW, PITCH, ROLL

    if mat[2,x] >= 1:
        euler_out[yaw] = 0
        euler_out2[yaw] = 0

        delta = np.arctan2(mat[2,y], mat[2, z])
        if mat[2,x] < 0:
            euler_out[pitch] =   PI / 2
            euler_out2[pitch] =  PI / 2
            euler_out[roll] = delta
            euler_out2[roll] = delta
        
        else: ##gimbal lock
            
            euler_out[pitch] = -PI / 2
            euler_out2[pitch] = -PI / 2
            euler_out[roll] = delta
            euler_out2[pitch] = delta

    else:
        euler_out[pitch] = np.arcsin(mat[2, x])
        euler_out2[pitch] = PI - euler_out[pitch]

        euler_out[roll] = np.arctan2(mat[2, y] / np.cos(euler_out[pitch]),  mat[2, z]  / np.cos(euler_out[pitch]))
        euler_out2[roll] = np.arctan2(mat[2, y] / np.cos(euler_out2[pitch]), mat[2, z] / np.cos(euler_out2[pitch]))

        euler_out[yaw] = np.arctan2(mat[1, x] / np.cos(euler_out[pitch]), mat[0, x] / np.cos(euler_out[pitch]))
        euler_out2[yaw] = np.arctan2(mat[1, x]  / np.cos(euler_out2[pitch]),    mat[0, x] / np.cos(euler_out2[pitch]))

    if sol == 1:
        return euler_out
    else:
        return euler_out2





def main():

    i = -7
    j = wrap2Pi(i)
    print(j)
    

if __name__ == '__main__':
    main()