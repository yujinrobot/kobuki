#!/usr/bin/env python
#AUTHOR: Younghun Ju <yhju@yujinrobot.comm>, <yhju83@gmail.com>

import roslib; roslib.load_manifest('kobuki_node')
import rospy

from kobuki_msgs.msg import DockInfraRed
from kobuki_msgs.msg import SensorState

class Converter(object):

  def __init__(self):
    rospy.init_node("dock_ir_intepreter", anonymous=True)
    self.sub_dock_ir = rospy.Subscriber("/mobile_base/sensors/dock_ir", DockInfraRed, self.DockIRCallback)
    self.sub_core    = rospy.Subscriber("/mobile_base/sensors/core" , SensorState, self.SensorStateCallback)
    self.stack = []
    self.bumper = 0
    self.charger = 0


  def SensorStateCallback(self,data):
    self.bumper = data.bumper
    self.charger = data.charger

  def DockIRCallback(self,data):
    #v1 not filtering
    array = [ ord(x) for x in data.data]
    array.reverse()
    #v2 filtering
    self.stack.append(array)
    while len(self.stack) > 10:
      del self.stack[0]
    print 
    print "window: ", len(self.stack), "charger: ", self.charger, "[ON]" if self.charger else "[  ]", "bumper: ["\
      , "L" if self.bumper&4 else "~"\
      , "C" if self.bumper&2 else "~"\
      , "R" if self.bumper&1 else "~", "]"

    array = [0,0,0]
    for i in range(len(self.stack)):
      for j in range(3):
        array[j] |= self.stack[i][j]


    ostr = ""
    ostr_top = "[far ] "; ostr_bot = "[near] "; ostr_bin="[bin ] "; ostr_dec="[dec ] "
    for ir in array:
      #ver1: just digits: 
      #ostr += "{0:3d}".format(ord(array[i])) + " "
      #ver2: just binary number: 
      #ostr += "{0:#08b}".format(ord(array[i]))[2:] + " "
      #ver3: [far]/[near] = [R|C|L]/[R|C|L] for each channel
      #ostr_top = ""; ostr_bot = ""
      top = ir>>3; bot =ir
      ostr_top += " L" if top&2 else " ~" # far left is 16
      ostr_top += "|C" if top&1 else "|~" # far center is 8
      ostr_top += "|R" if top&4 else "|~" # far right is 32
      ostr_bot += " L" if bot&1 else " ~" # near left is 1
      ostr_bot += "|C" if bot&2 else "|~" # near center is 2
      ostr_bot += "|R" if bot&4 else "|~" # near right is 4
      ostr_top += " "; ostr_bot += " "
      ostr_bin += "{0:#08b}".format(ir)[2:] + " "
      #ostr_dec += "{0:3d}".format(ir) + " "
    ostr = ostr_top + "\n\r" + ostr_bot + "\n" + ostr_bin# + "\n" + ostr_dec
      #verN: average
      #verN: 
    print ostr

if __name__ == '__main__':
  try:
    instance = Converter()
    print 
    print "It converts dock_ir data to human friendly format."
    print
    rospy.spin()
  except rospy.ROSInterruptException: pass
