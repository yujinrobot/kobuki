#!/usr/bin/env python
#AUTHOR: Younghun Ju <yhju@yujinrobot.comm>, <yhju83@gmail.com>

import roslib; roslib.load_manifest('kobuki_node')
import rospy

from tf.transformations import euler_from_quaternion
from math import degrees

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64

class Converter(object):

	def __init__(self):
		rospy.init_node("getYaw", anonymous=True)
		self.sub_imu  = rospy.Subscriber("imu", Imu, self.ImuCallback)
		self.pub_angle = rospy.Publisher("angle", Float64, queue_size=10)
		self.pub_angle_rate = rospy.Publisher("angle_rate", Float64, queue_size=10)

	def ImuCallback(self,data):
		quat = data.orientation
		q = [quat.x, quat.y, quat.z, quat.w]
		roll, pitch, yaw = euler_from_quaternion(q)
		print "angle: " + "{0:+.4f}".format(yaw) + " rad; "\
		+ "{0:+.2f}".format(degrees(yaw)) + " deg"
		print "rate:  " + "{0:+.2f}".format(data.angular_velocity.z) + " rad/s; "\
		+ "{0:+.2f}".format(degrees(data.angular_velocity.z)) + " deg/s"
		print '---'
		angle = Float64()
		angle_rate = Float64()
		angle.data = yaw
		angle_rate.data = data.angular_velocity.z
		self.pub_angle.publish(angle)
		self.pub_angle_rate.publish(angle_rate)

if __name__ == '__main__':
	try:
		instance = Converter()
		print 
		print "It prints angle and angular_velocity from Imu message of single yaw gyro."
		print
		rospy.spin()
	except rospy.ROSInterruptException: pass
