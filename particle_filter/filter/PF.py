from scipy.spatial.transform import Rotation
from statistics import mean
from scipy.linalg import block_diag
from copy import deepcopy, copy
import rospy
import numpy as np
from system.RobotState import RobotState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from utils.utils import wrap2Pi, wrap_all_angles, mat_from_quat, getRPY

from scipy.stats import multivariate_normal
from numpy.random import default_rng
rng = default_rng()
from comm.path_publisher import *

from pyquaternion import Quaternion
import sys

class PF:
    def __init__(self, system, init):
        np.random.seed(20)
        self.gfun = system.gfun  # motion model
        self.hfun = system.hfun  # measurement model
        self.Q = system.Q # measurement noise covariance
        
        # PF parameters
        self.n = init.n
        self.Sigma = init.Sigma
        self.particles = init.particles
        self.particle_weight = init.particle_weight

        
        self.state_ = RobotState()
        self.mu = init.mu
        self.cov = init.Sigma
        self.output_publisher = rospy.Publisher('/filtered', PoseWithCovarianceStamped, queue_size=100000)
        self.check = 0
    
    def prediction(self, delta):

        if self.check!=0:
            assert delta!=0
        for i in range(len(self.particles[0])):
            self.particles[:,i] = self.gfun(self.particles[:,i], delta)
        print("predicting", delta, self.mu)


    def correction(self, z, time):
        
        for i in range(len(self.particles[0])):
            
            expected_measurement = self.hfun(self.particles[:, i]) #extract expected measurement from measurement model 

            received_measurement = Rotation.from_quat(z[3:]).as_euler('xyz') # transform measurement measurement
            roll, pitch, yaw = received_measurement[0], received_measurement[1], received_measurement[2]

            mean_for_the_dist = np.array([z[0],z[1],z[2], roll, pitch, yaw])
            
            dist = multivariate_normal(mean_for_the_dist, self.Q)
            self.particle_weight[i] *= dist.pdf(expected_measurement) #compute likelihood
        
        if self.check!=0:
            assert np.sum(self.particle_weight)!=0
        self.check=1

        self.particle_weight = self.particle_weight/ np.sum(self.particle_weight)
        print("summm", np.sum(self.particle_weight))

        assert np.sum(self.particle_weight)>=0.9 and np.sum(self.particle_weight) <=1.1
        print("correcting", self.mu)
        
        position = self.mu[0:3]
        orientation = Rotation.from_euler('xyz', self.mu[3:6])
        orientation = orientation.as_quat()

        pose_to_publish = PoseWithCovarianceStamped()
        pose_to_publish.pose.pose.position.x = position[0]
        pose_to_publish.pose.pose.position.y = position[1]
        pose_to_publish.pose.pose.position.z = position[2]
        
        pose_to_publish.pose.pose.orientation.x = orientation[0]
        pose_to_publish.pose.pose.orientation.y = orientation[1]
        pose_to_publish.pose.pose.orientation.z = orientation[2]
        pose_to_publish.pose.pose.orientation.w = orientation[3]

        pose_to_publish.header.stamp = time

        self.output_publisher.publish(pose_to_publish)

        neff = 1/np.sum(self.particle_weight**2) # effective sample size
        if neff < self.n - 50:
            print("resampling \n \n resampling")
            self.resample()   ##resampling

        with open('/home/thiruchl/ROB530/Localiztion/ess.txt', 'a') as f:
            f.write(str(neff)+'\n')

        self.mean_variance_3D()

    def resample(self):
        new_samples = np.zeros_like(self.particles)
        new_weight = np.zeros_like(self.particle_weight)
        W = np.cumsum(self.particle_weight)
        r = np.random.rand(1) / self.n
        count = 0
        for j in range(self.n):
            u = r + j/self.n
            while u > W[count]:
                count += 1
            new_samples[:,j] = self.particles[:,count]
            new_weight[j] = 1 / self.n
        self.particles = new_samples
        self.particle_weight = new_weight
    
    def mean_variance_3D(self):

        self.mu = np.mean(self.particles, axis= 1)
        ang = self.particles[3:6, :]
        ang = Rotation.from_euler('xyz', ang.T)
        ang1 = ang.as_euler('xyz')
        mean_ang = ang.mean()
        self.mu[3:6] = mean_ang.as_euler('xyz')
        new_part = np.concatenate((self.particles[0:3,:], ang1.T, self.particles[6:, :]))
        new_sub = new_part - self.mu.reshape((-1,1))
        self.cov = np.matmul(new_sub, new_sub.T) / self.n
        assert self.cov.shape[0] ==15 and self.cov.shape[1] == 15
    

    def getState(self):
        return deepcopy(self.state_)

    def setState(self, state):
        self.state_ = state