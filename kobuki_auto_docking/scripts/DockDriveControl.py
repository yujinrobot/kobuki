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

# Author: Younghun Ju <younghoon.hju@rnd.yujinrobot.com> <yhju83@gmail.com>

import roslib; roslib.load_manifest('kobuki_auto_docking')
import rospy
import sys, select, termios, tty, os
import time
from datetime import datetime
import types
import commands

from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import DigitalOutput, Led, Sound, SensorState, DockInfraRed, ExternalPower, MotorPower



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


settings = termios.tcgetattr(sys.stdin.fileno())

class Controller(object):

  def __init__(self):
    #rospy initial setup
    rospy.init_node("dock_drive_control")
    rospy.on_shutdown(self.clearing)
    rate = rospy.Rate(10)
    self.message = "Idle"
    self.publish_cmd_vel=False
    self.cmd_vel=Twist()
    self.sensors = SensorState()
    self.dock_ir = DockInfraRed()

    self.bat_name = self.getBatteryName()
    self.state = "N/A"
    self.percentage = 0.0

    self.pub = {
#      'enable':rospy.Publisher('/enable', String),
#      'disable':rospy.Publisher('/disable', String),
      'motor_power':rospy.Publisher('/mobile_base/commands/motor_power',MotorPower),
#      'do_dock':rospy.Publisher('/dock_drive/commands/do_dock', Empty),
#      'cancel_dock':rospy.Publisher('/dock_drive/commands/cancel_dock', Empty),
      'debug':rospy.Publisher('/dock_drive/debug/mode_shift', String),
      'external_power':rospy.Publisher('/mobile_base/commands/external_power',ExternalPower),
      'digital_output':rospy.Publisher('/mobile_base/commands/digital_output',DigitalOutput),
      'led1':rospy.Publisher('/mobile_base/commands/led1',Led),
      'led2':rospy.Publisher('/mobile_base/commands/led2',Led),
      'sound':rospy.Publisher('/mobile_base/commands/sound',Sound),
      'cmd_vel':rospy.Publisher('/mobile_base/commands/velocity',Twist),
    }
    self.sub = {
      'core':rospy.Subscriber('/mobile_base/sensors/core', SensorState, self.sensorsCallback),
      'dock_ir':rospy.Subscriber('/mobile_base/sensors/dock_ir', DockInfraRed, self.dockIRCallback),
    }
    self.keyBindings = {
      '1':(self.pub['debug'].publish, String('enable') ,'enable'),
      '2':(self.pub['debug'].publish, String('run') ,'run'),
      '3':(self.pub['debug'].publish, String('stop') ,'stop'),
      '4':(self.pub['debug'].publish, String('disable'),'disable'),
      '5':5,
      '6':6,
      '7':7,
      '8':'eight',
      '9':'nine',
      '0':'null',
#      'e':(self.pub['motor_power'].publish,MotorPower(MotorPower.ON),'enabled'),
#      'r':(self.pub['motor_power'].publish,MotorPower(MotorPower.OFF),'disabled'),
      'e':(self.toggleMotor,True,'enabled'),
      'r':(self.toggleMotor,False,'disabled'),
      ' ':(self.resetVel,'','resetted'),
      'a':(self.pub['sound'].publish,Sound(Sound.ON),'sound.on'),
      's':(self.pub['sound'].publish,Sound(Sound.OFF),'sound.off'),
      'd':(self.pub['sound'].publish,Sound(Sound.RECHARGE),'sound.recharge'),
      'f':(self.pub['sound'].publish,Sound(Sound.BUTTON),'sound.button'),
      'z':(self.pub['sound'].publish,Sound(Sound.ERROR),'sound.error'),
      'x':(self.pub['sound'].publish,Sound(Sound.CLEANINGSTART),'sound.cleaningstart'),
      'c':(self.pub['sound'].publish,Sound(Sound.CLEANINGEND),'sound.cleaningend'),
      'q':(rospy.signal_shutdown,'user reuest','quit'),
      'Q':(rospy.signal_shutdown,'user reuest','quit'),
    }
    rospy.Timer(rospy.Duration(0.1), self.keyopCallback)
    if len(self.bat_name) > 0:
      rospy.Timer(rospy.Duration(1.0), self.batteryCallback)
    rospy.Timer(rospy.Duration(1.0), self.stateCallback) # to check status of rostopics
    self.printFront()
    '''
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
    '''

  def spin(self):
    while not rospy.is_shutdown():
      self.printStatus() 
      self.processKeys()

  def getKey(self):
    fd = sys.stdin.fileno()
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~termios.ICANON & ~termios.ECHO
    new[6][termios.VMIN] = 0
    new[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, new)
    return sys.stdin.read(1)
    #return os.read(fd,1)  #equivalent
    #return key

  def processKeys(self):
    key = self.getKey()
    if key == '':
      #self.message = "non." #debug
      return
    if ord(key) == 27:
      akey = self.getKey()
      if len(akey) and ord(akey)==91:
        key = self.getKey()
        if self.publish_cmd_vel:
          if key == 'A': self.message = 'Up Arrow'    ; self.cmd_vel.linear.x += 0.05
          if key == 'B': self.message = 'Down Arrow'  ; self.cmd_vel.linear.x -= 0.05
          if key == 'C': self.message = 'Right Arrow' ; self.cmd_vel.angular.z -= 0.33
          if key == 'D': self.message = 'Left Arrow'  ; self.cmd_vel.angular.z += 0.33
        return
    if key in self.keyBindings.keys():
      if type(self.keyBindings[key]) is types.TupleType:
        f = self.keyBindings[key][0]
        v = self.keyBindings[key][1]
        m = self.keyBindings[key][2]
        if type(f) is types.InstanceType or type(f) is types.MethodType or type(f) is types.FunctionType : f(v)
        else:
          print type(f)
        self.message = str(m)
      else:
        self.message = str(self.keyBindings[key])
    else:
      self.message = "unknown key: " + str(len(key)) + ": " + str(ord(key))
    return
    '''
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
    '''

  def clearing(self):
    sys.stdout.write('\n\r')
    sys.stdout.flush()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

  def toggleMotor(self, on_off):
    if on_off:
      self.pub['motor_power'].publish(MotorPower(MotorPower.ON))
      self.publish_cmd_vel = True
    else:
      self.pub['motor_power'].publish(MotorPower(MotorPower.OFF))
      self.publish_cmd_vel = False
      self.cmd_vel = Twist()

  def keyopCallback(self, event):
    if not rospy.is_shutdown():
      if self.publish_cmd_vel:
        self.pub['cmd_vel'].publish(self.cmd_vel)

  def sensorsCallback(self, data):
    if not rospy.is_shutdown():
      self.sensors = data

  def dockIRCallback(self, data):
    if not rospy.is_shutdown():
      self.dock_ir = data

  def batteryCallback(self, event):
    rem = float(commands.getoutput("grep \"^remaining capacity\" /proc/acpi/battery/" + self.bat_name + "/state | awk '{ print $3 }'"))
    full = float(commands.getoutput("grep \"^last full capacity\" /proc/acpi/battery/" + self.bat_name + "/info | awk '{ print $4 }'"))
    self.state = commands.getoutput("grep \"^charging state\" /proc/acpi/battery/" + self.bat_name + "/state | awk '{ print $3 }'")
    self.percentage = float((rem/full) * 100.)

  def stateCallback(self, event):
    # do check
    num =  self.sub['core'].get_num_connections()
    if num  == 0:
      self.messages = 'core is disconnected({0:d})'.format(num)
    else:
      self.messages = 'core is connected({0:d})'.format(num)
    # do more handling and managing
    # get_num_connections are not reliable yet.

  def resetVel(self, x):
    self.cmd_vel = Twist()

  def printFront(self): 
    # statement
    print ""
    print " Dock Drive Controller"
    print "----------------------"
    print "1: do_dock"
    print "2: run"
    print "3: stop" 
    print "4: cancel_dock"
    print ""
    print "q: quit"
    print ""
    #print "  3.3V  5.0V 12V5A 12V1A |   Led #1   Led #2 |  DO_0  DO_1  DO_2  DO_3"
    #print " [ On] [Off] [ On] [Off] | [Orange] [Orange] | [ On] [Off] [Off] [ On]"

  def printStatus(self):
    sys.stdout.write('                                                                                                               \r')
    sys.stdout.write('[ \033[1m' + self.message + '\033[0m ]')

    if len(self.bat_name) > 0:
      sys.stdout.write('[Laptop: ' + "{0:2.2f}".format(self.percentage) + '% - ' + self.state + ']')

    src_str = ''
    if self.sensors.charger:
      src_str = '(adaptor)' if self.sensors.charger&16 else '(dock)'

    chg = self.sensors.charger&6
    if chg==0: chg_str = 'discharging'
    elif chg==2: chg_str = 'fully charged'
    else: chg_str = 'charging'
    sys.stdout.write('[Robot: ' + "{0:2.1f}".format(self.sensors.battery/10.) + 'V - ' + chg_str + src_str + ']')
    '''
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
    '''
    sys.stdout.write('\r')
    sys.stdout.flush()

  def getBatteryName(self):
    PATH = '/proc/acpi/battery'
    bat_name = ''
    if os.path.exists(PATH):
      bat = os.walk(PATH).next()
      if len(bat[1]) > 0:
        bat_name = bat[1][0]
    return bat_name


if __name__ == '__main__':
  try:
    instance = Controller()
    instance.spin()
  except rospy.ROSInterruptException: pass
