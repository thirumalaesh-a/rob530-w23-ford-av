
import numpy as np
from functools import partial
from action_model import get_propagation_matrix

from utils.utils import wrap2Pi

class myStruct:
    pass

def gfun(mu, delta): # replace with dynamics model (just mu)
    output = get_propagation_matrix(mu, delta)
    return output

def hfun(poses): # identity transform 
    # output = pose[:6] ##??check
    # output = np.array([pose[] ##??check
    output = np.array([poses[0], poses[1], poses[2], poses[3], poses[4], poses[5]])
    # output = np.array([poses[1], poses[0], -poses[2], poses[4], poses[3], -poses[5]])
    return output

    # # output = pose[:6] ##??check
    # # output = np.array([pose[] ##??check
    # output = np.array([poses[0], poses[1], poses[2]])
    # # output = np.array([poses[1], poses[0], -poses[2], poses[4], poses[3], -poses[5]])
    # return output


def system_initialization():
    sys = myStruct()
    sys.gfun = gfun
    sys.hfun = hfun
    sys.Q = np.eye(6)
    
    return sys

