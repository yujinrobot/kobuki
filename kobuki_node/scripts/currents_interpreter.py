#!/usr/bin/env python
import roslib; roslib.load_manifest('iclebo_ros_node')
import rospy

from iclebo_comms.msg import iCleboCurrent
from standard_comms.msg import Float32ArrayStamped

def callback(data):
	pub = rospy.Publisher('out', Float32ArrayStamped)
	#rospy.loginfo("[" + rospy.get_name() + "] received")
	#print data.header_id
	#for i in range(len(data.current)):
	#	print "data " + repr(i) + ":" + repr(ord(data.current[i]))

	msg = Float32ArrayStamped()
	msg.header = data.header
	msg.data.append(25 * float(ord(data.current[0])))
	msg.data.append(25 * float(ord(data.current[1])))
	pub.publish(msg)

def relay():
	rospy.init_node('currents_interpreter')
	rospy.Subscriber('in', iCleboCurrent, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		relay()
	except rospy.ROSInterruptException: pass

