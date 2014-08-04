#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki/hydro-devel/kobuki_testsuite/LICENSE
#
##############################################################################
# Imports
##############################################################################

import threading
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String

##############################################################################
# Classes
##############################################################################

'''
    implements a rotating motion.
'''

class AngularAccelerateTest(threading.Thread):
    def __init__(self,cmd_vel_topic,log_topic, freq, accl):
        threading.Thread.__init__(self)
        self.pub_cmd = rospy.Publisher(cmd_vel_topic,Twist)
        self.pub_log = rospy.Publisher(log_topic,String)

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.twist = twist

        self.freq = freq
        self.accl = accl

        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        self._stop = False

        start = rospy.get_rostime()
        rospy.sleep(0.5)
        twist = self.twist
        freq = self.freq
        self.rate = rospy.Rate(freq)

        theta_dot_dot = self.accl
        msg = "Time : " + str(rospy.get_rostime().secs) + " Vel : " +str(twist.angular.z)
        while not rospy.is_shutdown() and not self._stop:
            twist.angular.z = twist.angular.z + ( 1.0 / freq ) * theta_dot_dot 
            msg = "Time : " + str(rospy.get_rostime().secs) + " Vel : " +str(twist.angular.z)
            self.log(msg)
            self.pub_cmd.publish(twist)
            self.rate.sleep()
        twist.angular.z = 0
        self.pub_cmd.publish(twist)

    def log(self,msg):
        rospy.loginfo(msg)
        t = String(msg)
        self.pub_log.publish(t)
