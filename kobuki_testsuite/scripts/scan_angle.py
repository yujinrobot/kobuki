#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki/master/kobuki_testsuite/LICENSE 
#
##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('kobuki_testsuite')
import rospy
from kobuki_testsuite import ScanToAngle

##############################################################################
# Main
##############################################################################

'''
  Hold the kinect up in front of a wall. This will calculate the relative
  heading angle it makes with the wall.
'''
if __name__ == '__main__':
    rospy.init_node('scan_to_angle')
    s = ScanToAngle('/scan', '/scan_angle')
    rospy.spin()
