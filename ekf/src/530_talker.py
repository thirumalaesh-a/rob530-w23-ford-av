#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

publisher = rospy.Publisher('/posewcov', PoseWithCovarianceStamped, queue_size=10000)
var = PoseWithCovarianceStamped()
def callback(data):
   global var
   rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
   var.header.stamp = data.header.stamp
   var.header.frame_id = data.header.frame_id
   var.header.frame_id = "pose_raw"
   #var.header = data.header
   var.pose.pose.position.x = data.pose.position.y *  1
   var.pose.pose.position.y = data.pose.position.x *  1
   var.pose.pose.position.z = data.pose.position.z *  -1

   var.pose.pose.orientation.x = data.pose.orientation.y  *  1 
   var.pose.pose.orientation.y = data.pose.orientation.x  *  1
   var.pose.pose.orientation.z = data.pose.orientation.z  *  -1
   var.pose.pose.orientation.w = data.pose.orientation.w 
   # var.pose.covariance = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
   var.pose.covariance = [0.2,0,0,0,0,0,0,0.2,0,0,0,0,0,0,0.3,0,0,0,0,0,0,0.1,0,0,0,0,0,0,0.1,0,0,0,0,0,0,0.1]
   # var.pose.covariance = [0.01,0,0,0,0.01,0,0,0,0.55]
   #var.covariance = 0
   publisher.publish(var)
 
def posestamped_listener():
   rospy.init_node('listener', anonymous=True)
   rospy.Subscriber("pose_raw", PoseStamped, callback)
   rospy.spin()
if __name__ == '__main__':
	posestamped_listener()
