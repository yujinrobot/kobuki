#!/usr/bin/env python
#AUTHOR: Younghun Ju <yhju@yujinrobot.comm>, <yhju83@gmail.com>

import roslib; roslib.load_manifest('kobuki_node')
import rospy

from tf.transformations import euler_from_quaternion
from math import degrees

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D 

class Converter(object):
	def __init__(self):
		rospy.init_node("getOdom2D", anonymous=True)
		self.sub = rospy.Subscriber("odom", Odometry, self.OdomCallback)
		self.pub = rospy.Publisher("pose2d", Pose2D)
		
	def OdomCallback(self,data):
		px = data.pose.pose.position.x
		py = data.pose.pose.position.y
		quat = data.pose.pose.orientation
		q = [quat.x, quat.y, quat.z, quat.w]
		roll, pitch, yaw = euler_from_quaternion(q)
	
		vx = data.twist.twist.linear.x
		vy = data.twist.twist.linear.y
		yaw_rate = data.twist.twist.angular.z
		print "pose: x: {0:+2.5f}".format(px) + ", y: {0:+2.5f}".format(py)\
		+ ", th: {0:+.4f}".format(yaw) + " rad; "\
		+ "{0:+.2f}".format(degrees(yaw)) + " deg"
		print "rate: x: {0:+2.5f}".format(vx) + ", y: {0:+2.5f}".format(vy)\
		+ ", th: {0:+.2f}".format(yaw_rate) + " rad/s; "\
		+ "{0:+.2f}".format(degrees(yaw_rate)) + " deg/s"
		print '---'
		pose2d = Pose2D()
		pose2d.x = px
		pose2d.y = py
		pose2d.theta = yaw
		self.pub.publish(pose2d)

if __name__ == '__main__':	
	try:
		instance = Converter()
		print 
		print "It prints x, y, theta values from Odom message of mobile base."
		print
		rospy.spin()
	except rospy.ROSInterruptException: pass
