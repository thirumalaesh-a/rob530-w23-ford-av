#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

def listener():

    with open('/home/thiruchl/ROB530/Localiztion/ROB530_Localization-working_PF/1stamped_traj_estimate.txt', 'w') as f_mypose, open('/home/thiruchl/ROB530/Localiztion/ROB530_Localization-working_PF/1stamped_groundtruth.txt', 'w') as f_truth:
        rospy.init_node('listener1', anonymous=True)

        rospy.Subscriber("pose_ground_truth", PoseStamped, callback_posetruth, f_truth)

        rospy.Subscriber("filtered/", PoseWithCovarianceStamped, callback_mypose, f_mypose)
        rospy.spin()

# def callback_posetruth(data, f_truth):
#         f_truth.write("{} {} {} {} {} {} {} {} \n".format(data.header.stamp,data.pose.position.x,data.pose.position.y,data.pose.position.z, \
#                                             data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w))

def callback_posetruth(data, f_truth):
        f_truth.write("{} {} {} {} {} {} {} {} \n".format(data.header.stamp,data.pose.position.y,data.pose.position.x,-data.pose.position.z, \
                                            data.pose.orientation.y,data.pose.orientation.x,-data.pose.orientation.z,data.pose.orientation.w))

def callback_mypose(data, f_mypose):

    f_mypose.write("{} {} {} {} {} {} {} {} \n".format(data.header.stamp,data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z, \
                                                data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))



if __name__ == '__main__':
    listener()

# #!/usr/bin/env python3
# import rospy
# from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

# def listener():

#     with open('/home/thiruchl/ROB530/Localiztion/ROB530_Localization-working_PF/poses.txt', 'w') as f_pose:
#         rospy.init_node('listener1', anonymous=True)

#         rospy.Subscriber("pose_raw/", PoseStamped, callback_pose, f_pose)
#         rospy.spin()

# def callback_pose(data, f_mypose):

#     f_mypose.write("{} {} {} {} {} {} {} {} \n".format(data.header.stamp,data.pose.position.x,data.pose.position.y,data.pose.position.z, \
#                                                 data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w))



# if __name__ == '__main__':
#     listener()

