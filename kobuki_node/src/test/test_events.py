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

import roslib; roslib.load_manifest('kobuki_node')
import rospy

from kobuki_comms.msg import ButtonEvent
from kobuki_comms.msg import BumperEvent

def buttonEventCallback(data):
    if ( data.state == ButtonEvent.RELEASED ) :
        state = "released"
    else:
        state = "pressed"  
    if ( data.button == ButtonEvent.F0 ) :
        button = "F0"
    elif ( data.button == ButtonEvent.F1 ) :
        button = "F1"
    else:
        button = "F2"
    rospy.loginfo("Button %s was %s"%(state, button))

def bumperEventCallback(data):
    if ( data.state == BumperEvent.RELEASED ) :
        state = "released"
    else:
        state = "pressed"  
    if ( data.bumper == BumperEvent.LEFT ) :
        bumper = "Left"
    elif ( data.bumper == BumperEvent.CENTRE ) :
        bumper = "Centre"
    else:
        bumper = "Right"
    rospy.loginfo("%s bumper %s."%(bumper, state))
    
rospy.init_node("test_button_events")
rospy.Subscriber("/mobile_base/events/buttons",ButtonEvent,buttonEventCallback)
rospy.Subscriber("/mobile_base/events/bumpers",BumperEvent,bumperEventCallback)
print ""
print "Start pushing kobuki's buttons/bumper.....but be wary."
print ""
rospy.spin()
    