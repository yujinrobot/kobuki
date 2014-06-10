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

class RotateTest(threading.Thread):
    def __init__(self, cmd_vel_topic, log_topic, yaw_rate = 1.2):
        threading.Thread.__init__(self)
        self.pub_cmd = rospy.Publisher(cmd_vel_topic,Twist)
        self.pub_log = rospy.Publisher(log_topic,String)

        freq = 5
        self.rate = rospy.Rate(freq)
        self.yaw_rate = yaw_rate

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.twist = twist

        self.max_rotate_count = freq*int(3.14/ self.yaw_rate)

        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        self._stop = False

        start = rospy.get_rostime()
        rospy.sleep(0.5)
        twist = self.twist
        max_rotate_count = self.max_rotate_count
        rotate_count = max_rotate_count
        yaw_rate = self.yaw_rate

        while not rospy.is_shutdown() and not self._stop:
            if rotate_count == max_rotate_count:
                if twist.angular.z > 0:
                    mod = -1.0
                else:
                    mod = 1.0
                update = mod*yaw_rate/10.0

                while math.fabs(twist.angular.z) <= yaw_rate:
                    twist.angular.z = twist.angular.z + update
                    self.pub_cmd.publish(twist)
                    rospy.sleep(0.04)

                 # Make sure it is exact so the inequality in the while loop doesn't mess up next time around
                twist.angular.z = mod*yaw_rate
                rotate_count = 0
            else:
                rotate_count += 1
            now = rospy.get_rostime()
            msg = "Rotate: " + str(now.secs - start.secs)
            self.log(msg)
            self.pub_cmd.publish(twist)
            self.rate.sleep()

    def log(self,msg):
        rospy.loginfo(msg)

        t = String(msg)
        self.pub_log.publish(t)

