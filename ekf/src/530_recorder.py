#!/usr/bin/env python3
import rospy
# from std_msgs.msg import String
# from geometry_msgs.msg import Quaternion
# from geometry_msgs.msg import Vector3 
# from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def listener():
    #Open a text file here?

    # spin() simply keeps python from exiting until this node is stopped
    with open('/home/thiruchl/catkin_ws/src/rob590-multi-agent-localization/src/DATA/Sample-Data/collected/530_stamped_traj_estimate.txt', 'w') as f_mypose, open('/home/thiruchl/catkin_ws/src/rob590-multi-agent-localization/src/DATA/Sample-Data/collected/530_stamped_groundtruth.txt', 'w') as f_truth:
    # with open('/home/thiru/rob590-multi-agent-localization/src/DATA/Sample-Data/collected/stamped_traj_estimate.txt', 'w') as f_mypose, open('/home/thiru/rob590-multi-agent-localization/src/DATA/Sample-Data/collected/stamped_groundtruth.txt', 'w') as f_truth:
        rospy.init_node('listener', anonymous=True)
        # rospy.Subscriber("pose_localized", PoseStamped, callback_posetruth, f_truth)
        rospy.Subscriber("pose_ground_truth", PoseStamped, callback_posetruth, f_truth)
        # rospy.Subscriber("v2pose_raw", PoseWithCovarianceStamped, callback_posetruth, f_truth) ##FOR POSE V POSE
        # rospy.Subscriber("odometry/filtered", Odometry, callback_mypose, f_mypose)
        rospy.Subscriber("odometry/filtered", Odometry, callback_mypose, f_mypose)
        rospy.spin()

def callback_posetruth(data, f_truth):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # f_truth.write("{} {} {} {} {} {} {} {} \n".format(data.header.stamp,data.pose.position.x,data.pose.position.y,data.pose.position.z, \
    #                                             data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w))

    f_truth.write("{} {} {} {} {} {} {} {} \n".format(data.header.stamp,data.pose.position.y,data.pose.position.x,-data.pose.position.z, \
                                                data.pose.orientation.y,data.pose.orientation.x,-data.pose.orientation.z,data.pose.orientation.w))

    #write to the text file here

def callback_mypose(data, f_mypose):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)                          #####LOOK BELOW CHANGE####
    # f_mypose.write("{} {} {} {} {} {} {} {} \n".format(data.header.stamp,data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z, \
    #                                             data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))
    
    f_mypose.write("{} {} {} {} {} {} {} {} \n".format(data.header.stamp,data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z, \
                                                data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))

    #write to the text file here

if __name__ == '__main__':
    listener()


###############################################FOR AVDATA TRUTH VS LOCALIZED############################################################
# def listener():
#     #Open a text file here?

#     # spin() simply keeps python from exiting until this node is stopped
#     with open('/home/thiru/DATA/Sample-Data/collected/mypose.txt', 'w') as f_mypose, open('/home/thiru/DATA/Sample-Data/collected/truth.txt', 'w') as f_truth:
#         rospy.init_node('listener', anonymous=True)
#         rospy.Subscriber("pose_ground_truth", PoseStamped, callback_posetruth, f_truth)
#         rospy.Subscriber("pose_localized", PoseStamped, callback_mypose, f_mypose)
#         rospy.spin()

# def callback_posetruth(data, f_truth):
#     # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#     f_truth.write("{} {} {} {} {} {} {} {} \n".format(data.header.stamp,data.pose.position.x,data.pose.position.y,data.pose.position.z, \
#                                                 data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w))
#     #write to the text file here

# def callback_mypose(data, f_mypose):
#     # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#     f_mypose.write("{} {} {} {} {} {} {} {} \n".format(data.header.stamp,data.pose.position.x,data.pose.position.y,data.pose.position.z, \
#                                                 data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w))
#     #write to the text file here

# if __name__ == '__main__':
#     listener()