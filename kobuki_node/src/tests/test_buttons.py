#!/usr/bin/env python

import roslib; roslib.load_manifest('kobuki_node')
import rospy

from kobuki_comms.msg import LedArray

colours = ["Black", "Green", "Orange", "Red"]

rospy.init_node("test_led_array")
pub = rospy.Publisher('/kobuki/driver/led_command',LedArray)
rate = rospy.Rate(1)
led_array = LedArray()
led_array.values = [LedArray.GREEN, LedArray.BLACK]
while not rospy.is_shutdown():
    new_led_array = LedArray()
    new_led_array.values = []
    for led in led_array.values:
        if led == LedArray.GREEN:
            new_led_array.values.append(LedArray.ORANGE)
        elif led == LedArray.ORANGE:
            new_led_array.values.append(LedArray.RED)
        elif led == LedArray.RED:
            new_led_array.values.append(LedArray.BLACK)
        elif led == LedArray.BLACK:
            new_led_array.values.append(LedArray.GREEN)
    led_array = new_led_array
    print "[" + colours[led_array.values[0]] + "," + colours[led_array.values[1]] + "]" 
    pub.publish(led_array)
    rate.sleep()
    