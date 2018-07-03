#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

def callback(data):

	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	
def listener():
	#In ROS, nodes are uniquely named. If two nodes with the same
	#node are launched, the previous is kicked off. The
	#anonymous = True flag means that rospy will choose a unique
	#name for our 'listener' node so that mltiple listners can run
	#simultaneously
	
	rospy.init_node('coord_listener', anonymous=True)
	
	rospy.Subscriber("obj_coord", Float32MultiArray, callback)
	
	#spin() simply keeps python frm exiting until this node is stopped
	rospy.spin()

	
if __name__ == '__main__':
	listener()
