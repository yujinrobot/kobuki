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


wz=0.0
count=0
state = 'translation'
theta = 0.0
theta_goal = 0.0

def wrapToPi(x):
  import numpy as np
  return np.mod(x+np.pi,2*np.pi)-np.pi
  
def OdometryCallback(data):
  global theta
  quat = data.pose.pose.orientation
  q = [quat.x, quat.y, quat.z, quat.w]
  roll, pitch, yaw = euler_from_quaternion(q)
  theta = yaw

def BumperEventCallback(data):
  #data.bumper
  global state
  global count
  state = 'step_back'
  count = 0
  
def transition():
  global state
  global count
  global theta
  global theta_goal
  global wz

  twist = Twist()
  if state == 'translation':
    twist.linear.x = 0.3

  elif state == 'rotation':
    if abs(wrapToPi(theta_goal - theta)) < radians(1.0) :
      state = 'translation'
    else:
      twist.angular.z = wz

  elif state == 'step_back':
    if count > 10:
      rand = 3.141592*random.uniform(-1,1)
      theta_goal = wrapToPi( theta + rand ) 
      wz = 1.5 * rand/abs(rand)
      state = 'rotation'
    else:
      count += 1
      twist.linear.x = -0.1
      
  return twist

if __name__ == '__main__':
  rospy.init_node('endless_bump')
  rate = rospy.Rate(50)

  rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, BumperEventCallback)
  rospy.Subscriber("/odom", Odometry, OdometryCallback)
  pub = rospy.Publisher("cmd_vel", Twist)
  # state
  # subscribe bump
  # 
  # bump callback:
  #   turn robot randomly; 30deg < x < 180deg or -180deg < x < -30deg
  #   pub cmd_vel 
  while not rospy.is_shutdown():
    twist = transition()
    pub.publish(twist)
    rate.sleep()
 
