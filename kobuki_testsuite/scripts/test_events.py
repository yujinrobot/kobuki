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

from kobuki_msgs.msg import ButtonEvent
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent
from kobuki_msgs.msg import CliffEvent
from kobuki_msgs.msg import PowerSystemEvent
from kobuki_msgs.msg import DigitalInputEvent

def ButtonEventCallback(data):
    if ( data.state == ButtonEvent.RELEASED ) :
        state = "released"
    else:
        state = "pressed"  
    if ( data.button == ButtonEvent.Button0 ) :
        button = "B0"
    elif ( data.button == ButtonEvent.Button1 ) :
        button = "B1"
    else:
        button = "B2"
    rospy.loginfo("Button %s was %s."%(button, state))

def BumperEventCallback(data):
    if ( data.state == BumperEvent.RELEASED ) :
        state = "released"
    else:
        state = "pressed"  
    if ( data.bumper == BumperEvent.LEFT ) :
        bumper = "Left"
    elif ( data.bumper == BumperEvent.CENTER ) :
        bumper = "Center"
    else:
        bumper = "Right"
    rospy.loginfo("%s bumper is %s."%(bumper, state))
    
def WheelDropEventCallback(data):
    if ( data.state == WheelDropEvent.RAISED ) :
        state = "raised"
    else:
        state = "dropped"  
    if ( data.wheel == WheelDropEvent.LEFT ) :
        wheel = "Left"
    else:
        wheel = "Right"
    rospy.loginfo("%s wheel is %s."%(wheel, state))
    
def CliffEventCallback(data):
    if ( data.state == CliffEvent.FLOOR ) :
        state = "on the floor"
    else:
        state = "on the cliff"  
    if ( data.sensor == CliffEvent.LEFT ) :
        cliff = "Left"
    elif ( data.sensor == CliffEvent.CENTER ) :
        cliff = "Centre"
    else:
        cliff = "Right"
    rospy.loginfo("%s side of robot is %s."%(cliff, state))

def PowerEventCallback(data):
    if   ( data.event == PowerSystemEvent.UNPLUGGED ) :
        rospy.loginfo("Robot unplugged")
    elif ( data.event == PowerSystemEvent.PLUGGED_TO_ADAPTER ) :
        rospy.loginfo("Robot plugged to adapter")
    elif ( data.event == PowerSystemEvent.PLUGGED_TO_DOCKBASE ) :
        rospy.loginfo("Robot plugged to docking base")
    elif ( data.event == PowerSystemEvent.CHARGE_COMPLETED ) :
        rospy.loginfo("Robot charge completed")
    elif ( data.event == PowerSystemEvent.BATTERY_LOW ) :
        rospy.loginfo("Robot battery low")
    elif ( data.event == PowerSystemEvent.BATTERY_CRITICAL ) :
        rospy.loginfo("Robot battery critical")
    else:
        rospy.loginfo("WARN: Unexpected power system event: %d"%(data.event))

def InputEventCallback(data):
    val_str = ""
    for val in data.values:
        val_str = "%s, %s" % (val_str, str(val))
    rospy.loginfo("Digital input values: [" + val_str[2:] + "]")
    
rospy.init_node("test_events")
rospy.Subscriber("/mobile_base/events/button",ButtonEvent,ButtonEventCallback)
rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,BumperEventCallback)
rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent,WheelDropEventCallback)
rospy.Subscriber("/mobile_base/events/cliff",CliffEvent,CliffEventCallback)
rospy.Subscriber("/mobile_base/events/power_system",PowerSystemEvent,PowerEventCallback)
rospy.Subscriber("/mobile_base/events/digital_input",DigitalInputEvent,InputEventCallback)
print ""
print "Try kobuki's hardware components; the following events should be reported:"
print "  - buttons"
print "  - bumpers"
print "  - wheel drops"
print "  - cliffs"
print "  - plug/unplug adapter"
print "  - dock/undock on base"
print "  - charge completed"
print "  - battery low/critical"
print "  - digital input changes"
print ""
rospy.spin()
    
