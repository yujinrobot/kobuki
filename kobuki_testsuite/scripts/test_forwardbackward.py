#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Yujin Robot
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

#  

import rospy
from kobuki_testsuite import Square 
from geometry_msgs.msg import Twist

# Robot moves forward and backward to check robot moves straight or not.


def publish(pub, v, sec):
    cnt = 0
    m = sec * 2
    
    while cnt < m and not rospy.is_shutdown(): 
        pub.publish(v)
        cnt = cnt + 1
        rospy.sleep(0.5)

if __name__ == '__main__':

    rospy.init_node('test_square')

    cmdvel_topic ='/cmd_vel_mux/input/teleop'

    pub = rospy.Publisher(cmdvel_topic, Twist)

    vel = Twist() 

    while not rospy.is_shutdown():
        vel.linear.x = 0.05
        rospy.loginfo("Moving forward")
        publish(pub, vel, 15)
        rospy.loginfo("Done")
        rospy.sleep(2)

        vel.linear.x = -0.05
        rospy.loginfo("Moving backward")
        publish(pub, vel, 15)
        rospy.loginfo("Done")
        rospy.sleep(2)
