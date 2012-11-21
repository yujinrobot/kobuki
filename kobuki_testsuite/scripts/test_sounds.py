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

from kobuki_msgs.msg import Sound

sounds = [Sound.ON, Sound.OFF, Sound.RECHARGE, Sound.BUTTON, Sound.ERROR, Sound.CLEANINGSTART, Sound.CLEANINGEND]
texts = ["On", "Off", "Recharge", "Button", "Error", "CleaningStart", "CleaningEnd"]

rospy.init_node("test_sounds")
pub = rospy.Publisher('/mobile_base/commands/sound', Sound)
rate = rospy.Rate(0.5)

# Added below two line of code
# to wait until at least one subscriber is present or connected.
# Because rospy.Publisher will skip(or eat) first certain messages, if you publish just after rospy.init()
# Without this patch, first message - Sound.ON will never be published.
# I think this is a bug of rospy
# Younghun Ju
while not pub.get_num_connections():
  rate.sleep()

msg = Sound()
while not rospy.is_shutdown():
    for sound, text in zip(sounds, texts):
        msg.value = sound
        print text 
        pub.publish(msg)
        rate.sleep()
    break
    
