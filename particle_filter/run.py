import rospy
from system.RobotSystem import *
from scipy.spatial.transform import Rotation as R

lines_gt = []
lines_fil = []
# filtered_poses_path = 'filtered_poses.txt'
ground_poses_path = 'ground_poses.txt'

# def sub_callback(msg):
    
#     # translation
#     tx = msg.pose.pose.position.x
#     ty = msg.pose.pose.position.y
#     tz = msg.pose.pose.position.z

#     # rotation
#     x_quat = msg.pose.pose.orientation.x
#     y_quat = msg.pose.pose.orientation.y
#     z_quat = msg.pose.pose.orientation.z
#     w_quat = msg.pose.pose.orientation.w

#     # convert quaternion to rotation matrix
#     r = R.from_quat([x_quat, y_quat, z_quat, w_quat])
#     r = r.as_matrix()

#     # format line to append
#     line = ""
#     for col in range(3):
#         line += str(r[0][col]) + " "
#     line += str(tx) + " "
#     for col in range(3):
#         line += str(r[1][col]) + " "
#     line += str(ty) + " "
#     for col in range(3):
#         line += str(r[2][col]) + " "
#     line += str(tz)
        
#     # append line to lines
#     lines_fil.append(line)

#     with open(filtered_poses_path, 'a') as fp:
#         fp.write("%s\n" % line)


def sub_callback_2(msg):
    
    # translation
    tx = msg.pose.position.x
    ty = msg.pose.position.y
    tz = msg.pose.position.z

    # rotation
    x_quat = msg.pose.orientation.x
    y_quat = msg.pose.orientation.y
    z_quat = msg.pose.orientation.z
    w_quat = msg.pose.orientation.w

    # convert quaternion to rotation matrix
    r = R.from_quat([x_quat, y_quat, z_quat, w_quat])
    # r = r.as_matrix()
    r1 = r.as_euler('xyz')

    # # format line to append
    # line = ""
    # for col in range(3):
    #     line += str(r[0][col]) + " "
    # line += str(tx) + " "
    # for col in range(3):
    #     line += str(r[1][col]) + " "
    # line += str(ty) + " "
    # for col in range(3):
    #     line += str(r[2][col]) + " "
    # line += str(tz)
        
    # # append line to lines
    # lines_gt.append(line)

    line = str(msg.header.stamp) + " "
    line = str(tx) + " "
    line = str(ty) + " "
    line = str(tz) + " "
    line = str(r1[0]) + " "
    line = str(r1[1]) + " "
    line = str(r1[2]) + " "


    with open(ground_poses_path, 'a') as fp:
        fp.write("%s\n" % line)

def main():
    robot_system = RobotSystem()

    # load params
    with open("config/settings.yaml", 'r') as stream:
        param = yaml.safe_load(stream)

    # subscribe to topic for particle filter generated poses
    pose_topic = param['pose_topic']
    # filtered_pose_subscriber = rospy.Subscriber('/filtered', PoseWithCovarianceStamped, sub_callback)
    ground_pose_subscriber = rospy.Subscriber('/pose_ground_truth', PoseStamped, sub_callback_2)

    try:
        robot_system.run_filter()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()


# TODO:
    # figure out how to properly set the state (see PF.py, RobotSystem.py, RobotState.py)
    # publish states to a ROS topic --- have a subscriber that subcribes and runs all of the time and writes to a csv
    # visualize particle filter pose using published poses to ROS topic
    # convert bag poses to poses.txt using script from first checkpoint
    # evaluate performance of localization - rpg_trajectory evaluation: https://github.com/uzh-rpg/rpg_trajectory_evaluation
    # improve
