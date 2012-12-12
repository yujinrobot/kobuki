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

import roslib; roslib.load_manifest('kobuki_auto_docking')
import rospy

import actionlib
import kobuki_auto_docking.msg 
from actionlib_msgs.msg import GoalStatus

def doneCb(status, goal):
  print ' * Done'
  if status == GoalStatus.PENDING   : print '    - PENDING   '
  if status == GoalStatus.ACTIVE    : print '    - ACTIVE    '
  if status == GoalStatus.PREEMPTED : print '    - PREEMPTED '
  if status == GoalStatus.SUCCEEDED : print '    - SUCCEEDED '
  if status == GoalStatus.ABORTED   : print '    - ABORTED   '
  if status == GoalStatus.REJECTED  : print '    - REJECTED  '
  if status == GoalStatus.PREEMPTING: print '    - PREEMPTING'
  if status == GoalStatus.RECALLING : print '    - RECALLING '
  if status == GoalStatus.RECALLED  : print '    - RECALLED  '
  if status == GoalStatus.LOST      : print '    - LOST      '
  print '    -', goal

def activeCb():
  print ' * Gone Active'

def feedbackCb(feedback):
  print ' * Feedback:', feedback

def dock_drive_client():
  client = actionlib.SimpleActionClient('dock_drive_action', kobuki_auto_docking.msg.AutoDockingAction)

  client.wait_for_server()

  goal = kobuki_auto_docking.msg.AutoDockingGoal(goal=5);

  client.send_goal(goal, doneCb, activeCb, feedbackCb)

  client.wait_for_result()

  return client.get_result()

if __name__ == '__main__':
  try:
    rospy.init_node('dock_drive_client_py')
    result = dock_drive_client()
    print ''
    print "Result: ", result
  except rospy.ROSInterruptException: 
    print "program interrupted before completion"
