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
#
# Continuously print angle and angular velocity from Imu messages


import roslib; roslib.load_manifest('kobuki_testsuite')
import rospy
import sys

from tf.transformations import euler_from_quaternion
from math import degrees

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


def ImuCallback(data):
  quat = data.orientation
  q = [quat.x, quat.y, quat.z, quat.w]
  roll, pitch, yaw = euler_from_quaternion(q)
  sys.stdout.write("\r\033[1mGyro Angle\033[0m: [" + "{0:+.4f}".format(yaw) + " rad]  ["\
                 + "{0: >+7.2f}".format(degrees(yaw)) + " deg]"\
                 + "            \033[1mRate\033[0m: [" + "{0:+.4f}".format(data.angular_velocity.z) + " rad/s]  ["\
                 + "{0: >+7.2f}".format(degrees(data.angular_velocity.z)) + " deg/s] ")
  # http://docs.python.org/library/string.html#formatexamples
  sys.stdout.flush()

rospy.init_node("test_gyro")
rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, ImuCallback)
rospy.spin()
print '' # for clean output
