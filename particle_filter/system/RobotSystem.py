import os
import sys
sys.path.append('.')
import yaml

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from  scipy.spatial.transform import Rotation as R

from system.RobotState import *
from comm.path_publisher import *
from utils.filter_initialization import filter_initialization
from utils.system_initialization import system_initialization
from utils.utils import *
import numpy as np

class RobotSystem:
    
    def __init__(self):

        # load params
        with open("config/settings.yaml", 'r') as stream:
            param = yaml.safe_load(stream)

        # load initial state and mean
        init_state_mean = np.array(param['initial_state_mean'])
        init_state_cov = np.diag(param['initial_state_variance'])**2

        # initialize dynamics and transform function
        self.system_ = system_initialization() 

        # initialize filter
        self.filter_name = 'PF'
        print("Initializing ", self.filter_name)
        self.filter_ = filter_initialization(self.system_, init_state_mean, init_state_cov, self.filter_name)

        # current state
        self.state_ = self.filter_.getState()

        # timing
        self.last_update_time = 0
        self.flag = False
        self.prediction_thresh = 0.001 # predicts if no measurement for this duration(in secs)

        # finished rosbag
        self.finished = False
        
        # timer set to nothing, then created later on
        timer = 0 

        # object for publishing the robot states
        self.pub = path_publisher()     # filter pose
        self.filtered_poses_path = 'filtered_poses.txt'
    

    def quit_func(self, message):
        # stop predicting
        self.timer.shutdown()

        # finished message
        print("Finished.")

        # clean exit
        os._exit(os.EX_OK)


    def filter_callback(self, data):

        if self.last_update_time == 0:
            delta = 0
            self.last_update_time = data.header.stamp
            # print("deltaaaaaaaaaaaa", delta)

        else:
            current_time = data.header.stamp
            delta_dur = current_time - self.last_update_time
            print(current_time, self.last_update_time, "time")
            # print("Secs", delta_dur.secs, delta_dur.nsecs)
            delta = delta_dur.to_sec()
            # print("deltaaaaaaaaaaaa", delta)

        self.predict(delta)
        # calculate delta time and structure data for correction
        
        data_send = np.array([data.pose.position.y, data.pose.position.x, -data.pose.position.z,data.pose.orientation.y,data.pose.orientation.x, -data.pose.orientation.z, data.pose.orientation.w])
        # data_send = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z,data.pose.orientation.x,data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

        # correct
        self.filter_.correction(data_send) 

        # get the final state from the filter    
        # self.state_ = self.filter_.getState() ##hmm
        # self.last_update_time = current_time

        # publish state to topic
        # self.pub.publish_pose(self.state_)

    
    def predict(self,delta):
        # assert delta!=0
        self.filter_.prediction(delta) 

    def sub_callback(self, msg):
        
            # translation
            tx = msg.pose.pose.position.x
            ty = msg.pose.pose.position.y
            tz = msg.pose.pose.position.z

            # rotation
            x_quat = msg.pose.pose.orientation.x
            y_quat = msg.pose.pose.orientation.y
            z_quat = msg.pose.pose.orientation.z
            w_quat = msg.pose.pose.orientation.w
            print(x_quat, y_quat, z_quat, w_quat)
            # convert quaternion to rotation matrix
            r = R.from_quat([x_quat, y_quat, z_quat, w_quat])
            r = r.as_euler('xyz')
            line = str(msg.header.stamp) + " "
            line = str(tx) + " "
            line = str(ty) + " "
            line = str(tz) + " "
            line = str(r[0]) + " "
            line = str(r[1]) + " "
            line = str(r[2]) + " "
            with open(self.filtered_poses_path, 'a') as fp:
                fp.write("%s\n" % line)
            
            # self.last_update_time = rospy.Time(secs=msg.header.stamp.secs, nsecs=msg.header.stamp.nsecs)



    def run_filter(self):

        # # initialize listener node
        rospy.init_node('listener', anonymous=True)
        
        # filtered_pose_subscriber = rospy.Subscriber('/filtered', PoseWithCovarianceStamped, self.sub_callback)
        
        # # subscribe to /pose_raw topic
        # rospy.Subscriber('/pose_raw', PoseStamped, self.filter_callback)

        # # start timer
        # # self.timer = rospy.Timer(rospy.Duration(secs=self.prediction_thresh), self.predict)
        # rospy.on_shutdown(self.quit_func)

        # # do not exit until node is stopped
        # rospy.spin()

        with open('/home/thiruchl/ROB530/Localiztion/ROB530_Localization-working_PF/poses.txt', 'r') as f_pose:
            poses = np.loadtxt(f_pose)
        length = len(poses)
        for i in range(length):
            state_send = np.array([poses[i,1], poses[i,2], poses[i,3], poses[i,4], poses[i,5], poses[i,6], poses[i,7]])
            # state_send = np.array([poses[i,2], poses[i,1], -poses[i,3], poses[i,5], poses[i,4], -poses[i,6], poses[i,7]])
            if self.last_update_time == 0:
                delta = 0
                self.last_update_time = rospy.Time(secs=poses[i,0]/1e9)
                current_time = rospy.Time(secs=poses[i,0]/1e9)
            else:
                current_time = rospy.Time(secs=poses[i,0]/1e9)
                delta_dur = current_time - self.last_update_time
                self.last_update_time = current_time
                print(current_time, self.last_update_time, "time")
                # print("Secs", delta_dur.secs, delta_dur.nsecs)
                delta = delta_dur.to_sec()
                # print("deltaaaaaaaaaaaa", delta)

            self.predict(delta)
            # calculate delta time and structure data for correction
        
            self.filter_.correction(state_send, current_time) 

            print("iteration::::"+str(i)+'/'+str(length))


    