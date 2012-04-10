#!/usr/bin/env python

# This is a script to convert the kobuki driver's output current data
# into a format that rxplot can eat.
#
# Can be useful if we're doing speed under payload tests to find
# problems.
#
# Note : doesn't work in its current state as we've removed all
# ycs components. Fix if you want to ;)


import roslib; roslib.load_manifest('kobuki_ros_node')
import rospy

from kobuki_comms.msg import Current
from standard_comms.msg import Float32ArrayStamped #standard_comms are deprecated!

def callback(data):
	pub = rospy.Publisher('out', Float32ArrayStamped)
	#rospy.loginfo("[" + rospy.get_name() + "] received")
	#print data.header_id
	#for i in range(len(data.current)):
	#	print "data " + repr(i) + ":" + repr(ord(data.current[i]))

	msg = Float32MultiArray()
	msg.header = data.header
	msg.data.append(25 * float(ord(data.current[0])))
	msg.data.append(25 * float(ord(data.current[1])))
	pub.publish(msg)

def relay():
	rospy.init_node('currents_interpreter')
	rospy.Subscriber('in', Current, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		relay()
	except rospy.ROSInterruptException: pass

