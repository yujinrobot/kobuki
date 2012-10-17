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

from tf.transformations import euler_from_quaternion
from math import degrees, radians

import random

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_comms.msg import BumperEvent


def wrapToPi(x):
  import numpy as np
  return np.mod(x+np.pi,2*np.pi)-np.pi
#export to angles?  

class EndlessBump(object):
  def __init__(self):
    rospy.init_node('endless_bump')
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.BumperEventCallback)
    rospy.Subscriber("/odom", Odometry, self.OdometryCallback)
    self.pub = rospy.Publisher("cmd_vel", Twist)

    self.wz=0.0
    self.count=0
    self.state = 'translation'
    self.theta = 0.0
    self.theta_goal = 0.0
 
  def spin(self):
    while not rospy.is_shutdown():
      rate = rospy.Rate(50)
      self.pub.publish(self.transition())
      rate.sleep()

  def OdometryCallback(self, data):
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    self.theta = yaw
  
  def BumperEventCallback(self, data):
    self.state = 'step_back'
    self.count = 0
    
  def transition(self):
    twist = Twist()
    if self.state == 'translation':
      twist.linear.x = 0.3
  
    elif self.state == 'rotation':
      if abs(wrapToPi(self.theta_goal - self.theta)) < radians(1.0) :
        self.state = 'translation'
      else:
        twist.angular.z = self.wz
  
    elif self.state == 'step_back':
      if self.count > 10:
        rand = 3.141592*random.uniform(-1,1)
        self.theta_goal = wrapToPi( self.theta + rand ) 
        self.wz = 1.5 * rand/abs(rand)
        self.state = 'rotation'
      else:
        self.count += 1
        twist.linear.x = -0.1
        
    return twist

if __name__ == '__main__':
  try:
    instance = EndlessBump()
    instance.spin()
  except rospy.ROSInterruptException: pass
 
