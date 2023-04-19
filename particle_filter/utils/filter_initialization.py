
import numpy as np
from functools import partial
from numpy.random import default_rng
rng = default_rng()

class myStruct:
    pass

init = myStruct()
    
def filter_initialization(sys, initialStateMean, initialStateCov, filter_name):

    if filter_name == 'PF':
        init.mu = initialStateMean
        init.Sigma = np.eye(15) * 1
        init.n = 1500
        init.particles = np.zeros((15, init.n)) # initialize particles
        init.particle_weight = np.zeros(init.n)
        L = np.linalg.cholesky(init.Sigma) 
        for i in range(init.n):
            init.particles[:,i] = L @ rng.standard_normal((15,1)).reshape(15) + init.mu
            init.particle_weight[i] = 1/init.n

        from filter.PF import PF
        filter = PF(sys, init)    
    
    return filter

    # if filter_name == 'PF':
    #     init.mu = initialStateMean
    #     init.Sigma = np.eye(3) * 1
    #     init.n = 500
    #     init.particles = np.zeros((3, init.n)) # initialize particles
    #     init.particle_weight = np.zeros(init.n)
    #     L = np.linalg.cholesky(init.Sigma) 
    #     for i in range(init.n):
    #         init.particles[:,i] = L @ rng.standard_normal((3,1)).reshape(3) + init.mu
    #         init.particle_weight[i] = 1/init.n

    #     from filter.PF import PF
    #     filter = PF(sys, init)    
    
    # return filter


"""
Get co-ordinate transformations into the rosbag - NED TO ENU
        X = Y
        Y = X
        Z = -Z 
        for both position and orientation
    
Implement quat -> (mat) -> euleryxz - refer to robot_localization package filename: matrix3x3.h, robot_localization_listener.cpp

Check our H functions (anything related to the update step)

If results aren't satisfactory, check the dynamics model using a unit test

"""