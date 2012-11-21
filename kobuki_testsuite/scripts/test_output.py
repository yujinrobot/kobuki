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

import roslib; roslib.load_manifest('kobuki_testsuite')
import rospy
import sys, select, termios, tty
import time
from datetime import datetime

from kobuki_msgs.msg import DigitalOutput, Led, Sound


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    #termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def printStatus(ext_values, dgt_values, leds):
    sys.stdout.write('\r ')
    for idx in range(0,4):
        if ext_values[idx]:
            sys.stdout.write('[ \033[1mOn\033[0m] ')
        else: 
            sys.stdout.write('[Off] ')
    sys.stdout.write('| [\033[1m'+textBindings[leds[0].value]+'\033[0m] ')
    sys.stdout.write('[\033[1m'+textBindings[leds[1].value]+'\033[0m] | ')

    for idx in range(0,4):
        if dgt_values[idx]:
            sys.stdout.write('[ \033[1mOn\033[0m] ')
        else: 
            sys.stdout.write('[Off] ')

    sys.stdout.write('\r')
    sys.stdout.flush()

keyBindings1 = {
    '1':0,
    '2':1,
    '3':2,
    '4':3,
}

keyBindings2 = {
    '5':0,
    '6':1,
}

keyBindings3 = {
    '7':0,
    '8':1,
    '9':2,
    '0':3,
}

keyBindings4 = {
    'a':Sound.ON,
    's':Sound.OFF,
    'd':Sound.RECHARGE,
    'f':Sound.BUTTON,
    'z':Sound.ERROR,
    'x':Sound.CLEANINGSTART,
    'c':Sound.CLEANINGEND,
}

colorBindings = {
    Led.GREEN:Led.ORANGE,
    Led.ORANGE:Led.RED,
    Led.RED:Led.BLACK,
    Led.BLACK:Led.GREEN,
}

textBindings = {
    Led.GREEN:' Green',
    Led.ORANGE:'Orange',
    Led.RED:'   Red',
    Led.BLACK:' Black',
}


settings = termios.tcgetattr(sys.stdin)

def clearing():
  sys.stdout.write('\n\r')
  sys.stdout.flush()
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  
#rospy initial setup
rospy.init_node("test_output")
rospy.on_shutdown(clearing)
rate = rospy.Rate(10)

pub_ext_pwr = rospy.Publisher('/mobile_base/commands/external_power',DigitalOutput)
pub_dgt_out = rospy.Publisher('/mobile_base/commands/digital_output',DigitalOutput)
pub = []
pub.append(rospy.Publisher('/mobile_base/commands/led1',Led))
pub.append(rospy.Publisher('/mobile_base/commands/led2',Led))
pub_sounds = rospy.Publisher('/mobile_base/commands/sound',Sound)

# initial values
external_power = DigitalOutput()
external_power.values = [ True, True, True, True ]
external_power.mask = [ True, True, True, True ]

digital_output = DigitalOutput()
digital_output.values = [ True, True, True, True ]
digital_output.mask = [ True, True, True, True ]

leds = []
leds.append(Led())
leds.append(Led())
leds[0].value = Led.GREEN
leds[1].value = Led.GREEN

# initialize outputs
while not pub_ext_pwr.get_num_connections():
  rate.sleep()
pub_ext_pwr.publish(external_power)

while not pub_dgt_out.get_num_connections():
  rate.sleep()
pub_dgt_out.publish(digital_output)

while not pub[0].get_num_connections():
  rate.sleep()
pub[0].publish(leds[0])

while not pub[1].get_num_connections():
  rate.sleep()
pub[1].publish(leds[1])

# statement
print ""
print "Control Every Output of Kobuki"
print "------------------------------"
print "1: Toggle the state of 3.3V"
print "2: Toggle the state of 5V"
print "3: Toggle the state of 12V5A(arm)"
print "4: Toggle the state of 12V1A(kinect)"
print ""
print "5: Control Led #1"
print "6: Control Led #2"
print ""
print "7~0: Toggle the state of DigitalOut_0~3"
print ""
print "Play Sounds"
print "-----------"
print "a: On   s: Off   d: Recharge   f: Button   z: Error   x: CleaningStart   c: CleaningEnd"
print ""
print "q: Quit"
print ""
print ""
print "  3.3V  5.0V 12V5A 12V1A |   Led #1   Led #2 |  DO_0  DO_1  DO_2  DO_3"
#print " [ On] [Off] [ On] [Off] | [Orange] [Orange] | [ On] [Off] [Off] [ On]"

while not rospy.is_shutdown():
    printStatus(external_power.values, digital_output.values, leds)
    key = getKey()
    if key == '': continue
    if key == 'q' or key == 'Q': 
       rospy.signal_shutdown('user reuest')

    elif key in keyBindings1.keys():
        external_power.values[keyBindings1[key]] ^= True
        printStatus(external_power.values, digital_output.values, leds)
        pub_ext_pwr.publish(external_power)

    elif key in keyBindings3.keys():
        digital_output.values[keyBindings3[key]] ^= True
        printStatus(external_power.values, digital_output.values, leds)
        pub_dgt_out.publish(digital_output)

    elif key in keyBindings2.keys():
        leds[keyBindings2[key]].value = colorBindings[ leds[keyBindings2[key]].value ]
        printStatus(external_power.values, digital_output.values, leds)
        pub[keyBindings2[key]].publish(leds[keyBindings2[key]])
    elif key in keyBindings4.keys():
        pub_sounds.publish(keyBindings4[key])
        #rate.sleep()

