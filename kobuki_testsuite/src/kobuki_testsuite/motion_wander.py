import roslib; roslib.load_manifest('kobuki_testsuite')
import rospy

from tf.transformations import euler_from_quaternion
from math import degrees, radians

import sys
import random

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_comms.msg import BumperEvent, CliffEvent


def wrapToPi(x):
    import numpy as np
    return np.mod(x+np.pi,2*np.pi)-np.pi

def sig(x):
    if x > 0: return +1
    if x < 0: return -1
    return 0

'''
  Implements a safe wandering random motion using bump and cliff sensors. 
'''
class SafeWandering(object):
    
    '''
      Initialise everything, then later start with the spin() method.
      
      @param topic names
      @type strings
    '''
    def __init__(self, cmd_vel_topic, odom_topic, bumper_topic, cliff_topic ):
        rospy.Subscriber(bumper_topic, BumperEvent, self.bumper_event_callback)
        rospy.Subscriber(cliff_topic, CliffEvent, self.cliff_event_callback)
        rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)
        self.pub = rospy.Publisher(cmd_vel_topic, Twist)
        self.rate = rospy.Rate(50)

        self.ok = True
        self.theta = 0.0
        self.theta_goal = 0.0

    def command(self, twist):
        self.pub.publish(twist)
        self.rate.sleep()
        if rospy.is_shutdown():
            sys.exit()

    def go(self):
        twist = Twist()
        twist.linear.x = 0.18
        while self.ok:
            self.command(twist)

    def stepback(self):
        twist = Twist()
        twist.linear.x = -0.1
        for i in range(0,35): 
            self.command(twist)
 
    def turn(self):
        twist = Twist()
        twist.angular.z = 1.8*sig(wrapToPi(self.theta_goal - self.theta))
        while not self.reached():
            self.command(twist)
        self.ok = True

    def reached(self):
        if abs(wrapToPi(self.theta_goal - self.theta)) < radians(5.0):
            return True
        else:
            return False

    def spin(self):
        while not rospy.is_shutdown():
            self.go()
            self.stepback()
            self.turn()

    ##########################################################################
    # Callbacks
    ##########################################################################

    def odometry_callback(self, data):
        quat = data.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        self.theta = yaw

      
    def bumper_event_callback(self, data):
        if data.state == BumperEvent.PRESSED:
            self.ok = False
            if   data.bumper == BumperEvent.LEFT:
                self.theta_goal = self.theta - 3.141592*random.uniform(0.2, 1.0)
            elif data.bumper == BumperEvent.RIGHT:
                self.theta_goal = self.theta + 3.141592*random.uniform(0.2, 1.0)
            else:
                self.theta_goal = wrapToPi(self.theta + 3.141592*random.uniform(-1.0, 1.0))

    def cliff_event_callback(self, data):
        if data.state == CliffEvent.CLIFF:
            self.ok = False
            if   data.sensor == CliffEvent.LEFT:
                self.theta_goal = self.theta - 3.141592*random.uniform(0.2, 1.0)
            elif data.sensor == CliffEvent.RIGHT:
                self.theta_goal = self.theta + 3.141592*random.uniform(0.2, 1.0)
            else:
                self.theta_goal = wrapToPi(self.theta + 3.141592*random.uniform(-1.0, 1.0))
 
