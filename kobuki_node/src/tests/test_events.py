#!/usr/bin/env python

import roslib; roslib.load_manifest('kobuki_node')
import rospy

from kobuki_comms.msg import ButtonEvent

def callback(data):
    if ( data.event == ButtonEvent.Released ) :
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
    
rospy.init_node("test_button_events")
rospy.Subscriber("/kobuki/mobile_base/events/buttons",ButtonEvent,callback)
print ""
print "Start pushing kobuki's buttons.....but be wary."
print ""
rospy.spin()
    