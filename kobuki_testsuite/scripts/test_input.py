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
import curses

import time
from datetime import datetime

from tf.transformations import euler_from_quaternion
from math import degrees

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from kobuki_comms.msg import SensorState
from kobuki_comms.msg import ButtonEvent
from kobuki_comms.msg import BumperEvent
from kobuki_comms.msg import WheelDropEvent
from kobuki_comms.msg import CliffEvent
from kobuki_comms.msg import PowerSystemEvent
from kobuki_comms.msg import DigitalInputEvent

def ImuCallback(data):
  quat = data.orientation
  q = [quat.x, quat.y, quat.z, quat.w]
  roll, pitch, yaw = euler_from_quaternion(q)
  imu_string=str("Gyro Angle: [" + "{0:+.4f}".format(yaw) + " rad]  ["\
                 + "{0: >+7.2f}".format(degrees(yaw)) + " deg]"\
                 + "            Rate: [" + "{0:+.4f}".format(data.angular_velocity.z) + " rad/s]  ["\
                 + "{0: >+7.2f}".format(degrees(data.angular_velocity.z)) + " deg/s] ")
  stdscr.addstr(5,3,imu_string)

def SensorStateCallback(data):
  stdscr.addstr(6,3,str("Analog input: [%4d, %4d, %4d, %4d]"     \
                                        %(data.analog_input[0], data.analog_input[1], \
                                          data.analog_input[2], data.analog_input[3])))

def DigitalInputEventCallback(data):
  stdscr.addstr(7,3,"Digital input: [%5s, %5s, %5s, %5s]"\
                     %(str(data.values[0]), str(data.values[1]), str(data.values[2]), str(data.values[3]))) 

button0 = { ButtonEvent.F0:0, ButtonEvent.F1:1, ButtonEvent.F2:2, } 
button1 = { ButtonEvent.RELEASED:'Released', ButtonEvent.PRESSED:'Pressed ', }
buttonS = [ 'Released',  'Released',  'Released', ] 

def ButtonEventCallback(data):
  buttonS[button0[data.button]]=button1[data.state]
  stdscr.addstr(8,3,str("Button: F0: %s F1: %s F2: %s"%(buttonS[0], buttonS[1], buttonS[2])))

bumper0 = { BumperEvent.LEFT:0, BumperEvent.CENTER:1, BumperEvent.RIGHT:2, } 
bumper1 = { BumperEvent.RELEASED:'Released', BumperEvent.PRESSED:'Pressed ', }
bumperS = [ 'Released',  'Released',  'Released', ] 

def BumperEventCallback(data):
  bumperS[bumper0[data.bumper]]=bumper1[data.state]
  stdscr.addstr(9,3,str("Bumper: Left: %s Center: %s Right: %s"%(bumperS[0], bumperS[1], bumperS[2])))

wheel0 = { WheelDropEvent.LEFT:0, WheelDropEvent.RIGHT:1, } 
wheel1 = { WheelDropEvent.RAISED:'Raised ', WheelDropEvent.DROPPED:'Dropped', }
wheelS = [ 'Raised ',  'Raised ', ] 

def WheelDropEventCallback(data):
  wheelS[wheel0[data.wheel]]=wheel1[data.state]
  stdscr.addstr(10,3,str("WheelDrop: Left: %s Right: %s"%(wheelS[0], wheelS[1])))

cliff0 = { CliffEvent.LEFT:0, CliffEvent.CENTER:1, CliffEvent.RIGHT:2, } 
cliff1 = { CliffEvent.FLOOR:'On the floor', CliffEvent.CLIFF:'On the cliff', }
cliffS = [ 'On the floor', 'On the floor', 'On the floor',]

def CliffEventCallback(data):
  cliffS[cliff0[data.sensor]]=cliff1[data.state]
  stdscr.addstr(11,3,str("Cliff: Left: %s Center: %s Right: %s"%(cliffS[0], cliffS[1], cliffS[2])))

power0 = { 
  PowerSystemEvent.UNPLUGGED:"Robot unplugged",
  PowerSystemEvent.PLUGGED_TO_ADAPTER:"Robot plugged to adapter",
  PowerSystemEvent.PLUGGED_TO_DOCKBASE:"Robot plugged to docking base",
  PowerSystemEvent.CHARGE_COMPLETED:"Robot charge completed",
  PowerSystemEvent.BATTERY_LOW:"Robot battery low",
  PowerSystemEvent.BATTERY_CRITICAL:"Robot battery critical", }


def PowerEventCallback(data):
  stdscr.addstr(12,3,str("Power: " + "{0: <80s}".format(power0[data.event])))

def clearing():
  curses.echo()
  stdscr.keypad(0)
  curses.endwin()  

rospy.init_node('test_input')
rospy.on_shutdown(clearing)
rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, ImuCallback )
rospy.Subscriber("/mobile_base/sensors/core", SensorState, SensorStateCallback )
rospy.Subscriber("/mobile_base/events/digital_input", DigitalInputEvent, DigitalInputEventCallback )
rospy.Subscriber("/mobile_base/events/button", ButtonEvent, ButtonEventCallback )
rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, BumperEventCallback )
rospy.Subscriber("/mobile_base/events/wheel_drop", WheelDropEvent, WheelDropEventCallback )
rospy.Subscriber("/mobile_base/events/cliff", CliffEvent, CliffEventCallback )
rospy.Subscriber("/mobile_base/events/power_system",PowerSystemEvent,PowerEventCallback)


stdscr = curses.initscr()
stdscr.addstr(2,1,"Test Every Input of Kobuki")
stdscr.addstr(3,1,"--------------------------")
stdscr.addstr(4,1,"q: Quit")
curses.noecho()
#curses.cbreak()
stdscr.keypad(1)
stdscr.nodelay(1)

idx=0
while not rospy.is_shutdown():
#  idx+=1
#  size = stdscr.getmaxyx()
  key = stdscr.getch()
#  stdscr.addstr(2,2,"%d"%idx)
#  stdscr.addstr(3,3,"%u x %u"%(size[1], size[0]))
#  stdscr.addstr(4,4,datetime.fromtimestamp(time.time()).strftime("%Y-%m-%d %p %I:%M:%S"))
  if key == 'q':
    rospy.signal_shutdown('user request')

  stdscr.refresh()

clearing()

