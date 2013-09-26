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
# Move following a 90cm radius circle while publishing accumulated and
# average error on angular velocity, using gyroscope data as reference
# Useful for testing passive wheels configurations
# See https://github.com/yujinrobot/kobuki/issues/202 for more details


import roslib; roslib.load_manifest('kobuki_testsuite')
import rospy
import sys

from tf.transformations import euler_from_quaternion
from math import degrees

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion


class Test_slow_drive:
    # constructor
    def __init__(self):
        rospy.init_node("test_slow_drive")
        
        if (len(sys.argv) > 1):
            multip = int(sys.argv[1])
        else:
            multip = 1
        self.count = 0
        self.int_err = 0.0
        self.avg_err = 0.0
        self.target_v = 0.09*abs(multip)
        self.target_w = 0.1*multip
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist)
        self.int_err_pub = rospy.Publisher("/int_err", Float32)
        self.avg_err_pub = rospy.Publisher("/avg_err", Float32)

        rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, self.ImuCallback)
        rospy.Subscriber("/odom", Odometry, self.OdomCallback)
      
    def OdomCallback(self, data):
        self.odom = data.pose.pose.position.x
    
    def ImuCallback(self, data):
        self.count = self.count + 1
        if (self.count > 100):
            self.real_w = data.angular_velocity.z
            self.int_err += abs(self.target_w - self.real_w)
            self.avg_err =  self.int_err/(self.count - 100)
            
            self.int_err_pub.publish(self.int_err)
            self.avg_err_pub.publish(self.avg_err)

        self.twist.linear.x = self.target_v
        self.twist.angular.z = self.target_w
        self.cmd_vel_pub.publish(self.twist)
      

test_slow_drive_obj = Test_slow_drive()

while not rospy.is_shutdown():
    rospy.spin()
print ''
print test_slow_drive_obj.int_err
print test_slow_drive_obj.avg_err
print '' # for clean output
