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

# Author: Younghun Ju <yhju@yujinrobot.com> <yhju83@gmail.com>

import roslib; roslib.load_manifest('kobuki_testsuite')
import rospy

import random

from geometry_msgs.msg import Twist
from kobuki_comms.msg import BumperEvent

twist = Twist()

def BumperEventCallback(data):
  #data.bumper
  twist.angular.z = 3.141592*random.uniform(-1,1)
  
if __name__ == '__main__':
  rospy.init_node('endless_bump')
  rate = rospy.Rate(10)

  rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, BumperEventCallback)
  pub = rospy.Publisher("cmd_vel", Twist)
  # subscribe bump
  # bump callback:
  #   turn robot randomly
  #   pub cmd_vel 
  twist.angular.z = 3.141592*random.uniform(-1,1)
  while not rospy.is_shutdown():
    pub.publish(twist)
    rate.sleep()
 
