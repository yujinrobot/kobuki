#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Yujin Robot
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Yujin Robot nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Puts the robot into continual rotation - useful for aging/battery tests.

import roslib; roslib.load_manifest('kobuki_testsuite')
import rospy
import math
from geometry_msgs.msg import Twist

rospy.init_node("test_rotation")
pub = rospy.Publisher('/cmd_vel',Twist)
freq = 5
rate = rospy.Rate(freq)
twist = Twist()
yaw_rate = 1.2
twist.linear.x = 0
twist.linear.y = 0
twist.linear.z = 0
twist.angular.x = 0
twist.angular.y = 0
twist.angular.z = 0
max_rotate_count = freq*int(3.14/yaw_rate)
rotate_count = max_rotate_count
start = rospy.get_rostime()
rospy.sleep(0.5)
while not rospy.is_shutdown():
    if rotate_count == max_rotate_count:
        if twist.angular.z > 0:
            mod = -1.0
        else:
            mod = 1.0
        update = mod*yaw_rate/10.0
        while math.fabs(twist.angular.z) <= yaw_rate:
            twist.angular.z = twist.angular.z + update
            pub.publish(twist)
            rospy.sleep(0.04)
        # Make sure it is exact so the inequality in the while loop doesn't mess up next time around
        twist.angular.z = mod*yaw_rate
        rotate_count = 0
    else:
        rotate_count += 1
    now = rospy.get_rostime()
    rospy.loginfo("Rotate: %ds", now.secs - start.secs)
    pub.publish(twist)
    rate.sleep()
