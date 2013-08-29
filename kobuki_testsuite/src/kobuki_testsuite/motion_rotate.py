#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki/master/kobuki_testsuite/LICENSE 
#
##############################################################################
# Imports
##############################################################################

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

class Rotate():
    def __init__(self,cmd_vel_topic):
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic,Twist)
        self._cmd_vel_frequency = 5
        self._rate = rospy.Rate(self._cmd_vel_frequency)
        self._yaw_rate = 1.2
        
        self._max_rotate_count = self._cmd_vel_frequency*int(3.14/ self._yaw_rate)
        self._stop = False
        self._running = False

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.twist = twist
        
        self._number_of_turns = -1 # continuous

    def init(self, yaw_rate, number_of_turns=-1):
        self._yaw_rate = yaw_rate
        self._max_rotate_count = self._cmd_vel_frequency*int(3.14/ self._yaw_rate)

    def shutdown(self):
        self.stop()
        while self._running:
            rospy.sleep(0.5)
        self.cmd_vel_publisher.unregister() #This one creates an error for some reason, probably because others already unregister.
        
    def stop(self):
        self._stop = True

    def execute(self):
        '''
          Drop this into threading.Thread or QThread for execution
        '''
        if self._running:
            rospy.logerr("Kobuki TestSuite: already executing a motion, ignoring the request")
            return
        self._stop = False
        self._running = True
        start = rospy.get_rostime()
        rospy.sleep(0.5)

        twist = self.twist
        max_rotate_count = self._max_rotate_count
        rotate_count = max_rotate_count

        while not self._stop and not rospy.is_shutdown():
            if rotate_count == max_rotate_count:
                if twist.angular.z > 0:
                    mod = -1.0
                else:
                    mod = 1.0
                update = mod*self._yaw_rate/10.0

                while math.fabs(twist.angular.z) <= self._yaw_rate:
                    twist.angular.z = twist.angular.z + update
                    #print "Command velocity: %s"%twist.angular.z
                    self.cmd_vel_publisher.publish(twist)
                    rospy.sleep(0.04)

                 # Make sure it is exact so the inequality in the while loop doesn't mess up next time around
                twist.angular.z = mod*self._yaw_rate
                rotate_count = 0
            else:
                rotate_count += 1
            now = rospy.get_rostime()
            msg = "Rotate: " + str(now.secs - start.secs)
            self.cmd_vel_publisher.publish(twist)
            self._rate.sleep()
        if not rospy.is_shutdown():
            cmd = Twist()
            cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd)
        self._running = False

