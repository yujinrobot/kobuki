#!/usr/bin/env python
import rospy

'''
Implementation of a ROS node, which does exactly nothing
'''
def idle_node():
    rospy.init_node('idle_node')
    rospy.loginfo("Idle node '" + rospy.get_name() + "' started.")
    rospy.spin()
    rospy.loginfo("Shutting down idle node '" + rospy.get_name() + "'.")

if __name__ == '__main__':
    idle_node()
