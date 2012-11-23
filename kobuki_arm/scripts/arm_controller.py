#!/usr/bin/env python
import roslib; roslib.load_manifest('kobuki_ros_node')
import rospy

from kobuki_msgs.msg import SensorData
#from standard_comms.msg import Float32ArrayStamped #standard_comms are deprecated!
from std_msgs.msg import Float64
from std_msgs.msg import UInt8

def callback(data):
	pub = rospy.Publisher('arm_control_out', Float64)
	#rospy.loginfo("[" + rospy.get_name() + "] received")
	#print data.header_id
	#for i in range(len(data.current)):
	#	print "data " + repr(i) + ":" + repr(ord(data.current[i]))

	button_pressed = data.data # Test
	#button_pressed = data.remote # Actual
	processed = False
	if button_pressed & 1 :
		print "button 1 pressed."
		processed = True
	if button_pressed & 2 :
		print "button 2 pressed."
		processed = True
	if button_pressed & 4 :
		print "button 3 pressed."
		processed = True
	if processed == False:
		print "none."

	msg = Float64()
	msg.data = 2*float(button_pressed)
	#msg = Float32ArrayStamped()
	#msg.header = data.header
	#msg.data.append(25 * float(ord(data.current[0])))
	#msg.data.append(25 * float(ord(data.current[1])))
	pub.publish(msg)

def relay():
	rospy.init_node('kobukibot_arm_control')
	rospy.Subscriber('button_data_in', UInt8, callback)  # For Test
	#rospy.Subscriber('button_data_in', SensorData, callback) # Actual Topic
	rospy.spin()

if __name__ == '__main__':
	try:
		relay()
	except rospy.ROSInterruptException: pass

