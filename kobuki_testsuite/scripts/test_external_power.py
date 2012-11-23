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

from kobuki_msgs.msg import DigitalOutput

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    #termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

label = [
    '3.3V',
    '5V',
    '12V5A',
    '12V1A',
]

def printStatus(values):
    sys.stdout.write('\r ')
    for idx in range(0,4):
        if values[idx]:
#            sys.stdout.write(label[idx]+'[\033[1mOn\033[0m ] ')
            sys.stdout.write('[ \033[1mOn\033[0m] ')
        else: 
#            sys.stdout.write(label[idx]+'[Off] ')
            sys.stdout.write('[Off] ')
    sys.stdout.write('\r')
    sys.stdout.flush()


keyBindings = {
    '1':0,
    '2':1,
    '3':2,
    '4':3,
}

settings = termios.tcgetattr(sys.stdin)

def clearing():
    sys.stdout.write('\r\n')
    sys.stdout.flush()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

rospy.init_node("test_external_power")
rospy.on_shutdown(clearing)
pub = rospy.Publisher('/mobile_base/commands/external_power',DigitalOutput)
rate = rospy.Rate(10)
digital_output = DigitalOutput()
digital_output.values = [ True, True, True, True ]
digital_output.mask = [ True, True, True, True ]
print ""
#print "If you want to control the timing of publish, uses -p option."
#print ""
print "Control External Power"
print "----------------------"
print "1: Toggle the state of 3.3V"
print "2: Toggle the state of 5V"
print "3: Toggle the state of 12V5A(arm)"
print "4: Toggle the state of 12V1A(kinect)"
print ""
#print "p: publish power on/off status"
print "q: quit"
print ""
print "  3.3V  5.0V 12V5A 12V1A"
#print " [ On] [Off] [ On] [Off]"

# reset current state of external powers
while pub.get_num_connections():
    rate.sleep()
pub.publish(digital_output)

while not rospy.is_shutdown():
    key = getKey()
    printStatus(digital_output.values)
    if key == '': continue
    if key == 'q' or key == 'Q': 
       rospy.signal_shutdown('user request')
    elif key in keyBindings.keys():
        digital_output.values[keyBindings[key]] ^= True
        printStatus(digital_output.values)
        #print digital_output.values    
#    elif key == '\n' or key == 'p' or key == 'P':
        pub.publish(digital_output)
        #print ' - published [', datetime.fromtimestamp(time.time()).strftime("%Y-%m-%d %p %I:%M:%S"), ']'
        #rate.sleep()

