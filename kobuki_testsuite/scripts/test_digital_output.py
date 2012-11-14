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

import roslib; roslib.load_manifest('kobuki_testsuite')
import rospy

from kobuki_msgs.msg import DigitalOutput

rospy.init_node("test_digital_output")
pub = rospy.Publisher('/mobile_base/commands/digital_output',DigitalOutput)
rate = rospy.Rate(1)
digital_output = DigitalOutput()
digital_output.values = [ False, False, False, False]
digital_output.mask = [ True, True, False, True ]
print ""
print "This program will start sending a variety of digital io signals to the robot."
print "It will set all output signals to false, then iteratively turn each one to True"
print "In doing so, it will cycle through a mask that will negate the setting for one"
print "of the outputs. The process then repeats itself masking the next output in the"
print "sequence instead."
print ""
while not rospy.is_shutdown():
    # incrementally convert a false, to true and then reset them all to false.
    try:
        index = digital_output.values.index(False)
        for i in range(0,4):
            if not digital_output.values[i]:
                digital_output.values[i] = True
                break
    except ValueError:
        digital_output.values = [ False, False, False, False]
        index = digital_output.mask.index(False)
        digital_output.mask[index] = True
        if index == 3:
            next_index = 0
        else:
            next_index = index + 1
        digital_output.mask[next_index] = False
    print digital_output
    pub.publish(digital_output)
    rate.sleep()
    
