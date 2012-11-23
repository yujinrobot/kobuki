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

from kobuki_msgs.msg import Led

colours = ["Black", "Green", "Orange", "Red"]

rospy.init_node("test_led_array")
pub = []
pub.append(rospy.Publisher('/mobile_base/commands/led1',Led))
pub.append(rospy.Publisher('/mobile_base/commands/led2',Led))
rate = rospy.Rate(1)
leds = []
leds.append(Led())
leds.append(Led())
leds[0].value = Led.GREEN
leds[1].value = Led.BLACK
rate.sleep()

while not rospy.is_shutdown():
    for led in leds:
        if led.value == Led.GREEN:
            led.value = Led.ORANGE
        elif led.value == Led.ORANGE:
            led.value = Led.RED
        elif led.value == Led.RED:
            led.value = Led.BLACK
        elif led.value == Led.BLACK:
            led.value = Led.GREEN
    print "[" + colours[leds[0].value] + "," + colours[leds[1].value] + "]" 
    pub[0].publish(leds[0])
    pub[1].publish(leds[1])
    rate.sleep()
    
