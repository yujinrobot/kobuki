#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki/hydro-devel/kobuki_testsuite/LICENSE
#
##############################################################################
# Python Imports
##############################################################################
import random
from math import degrees, radians

##############################################################################
# Ros Imports
##############################################################################
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent, CliffEvent

##############################################################################
# Local imports
##############################################################################
import utils

'''
  Implements a safe wandering random motion using bump and cliff sensors. 

      API:
        init(speed,distance) : (re)initialise parameters 
        stop()  - stop.
        execute() - pass this to a thread to run
        shutdown() - cleanup
      
      @param topic names
      @type strings

'''
class SafeWandering(object):
    '''
      Initialise everything, then starts with start()

      API:
        start() - start to wander. 
        stop()  - stop wandering.
        set_vels(lin_xvel,stepback_xvel,ang_zvel)
      
      @param topic names
      @type strings
    '''
    def __init__(self, cmd_vel_topic, odom_topic, bumper_topic, cliff_topic ):

        self.bumper_subscriber = rospy.Subscriber(bumper_topic, BumperEvent, self.bumper_event_callback)
        self.cliff_subscriber = rospy.Subscriber(cliff_topic, CliffEvent, self.cliff_event_callback)
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.rate = rospy.Rate(50)

        self.ok = True
        self.theta = 0.0
        self.theta_goal = 0.0
        self._stop = False
        self._running = False

        self._lin_xvel = 0.18
        self._stepback_xvel = -0.1
        self._ang_zvel = 1.8

    def init(self, speed, stepback_speed, angular_speed):
        self._lin_xvel = speed
        self._stepback_xvel = stepback_speed
        self._ang_zvel = angular_speed

        
    def shutdown(self):
        self.stop()
        while self._running:
            self.rate.sleep()
        #self.cmd_vel_publisher.unregister() This one creates an error for some reason, probably because climbing_frame already unregisters.
        self.odom_subscriber.unregister()
        self.bumper_subscriber.unregister()
        self.cliff_subscriber.unregister()

    def command(self, twist):
        '''
          Lowest level of command - i.e. this is where the fine grained looping happens
          so we check for aborts here. Don't waste time doing anything if this is the case.
        '''
        if self._stop or rospy.is_shutdown():
            return False
        self.cmd_vel_publisher.publish(twist)
        self.rate.sleep()
        return True

    def go(self):
        twist = Twist()
        while self.ok:
            twist.linear.x = self._lin_xvel
            if not self.command(twist):
                return False
        return True

    def stepback(self):
        twist = Twist()
        for i in range(0,35): 
            twist.linear.x = self._stepback_xvel
            if not self.command(twist):
                return False
        return True
 
    def turn(self):
        twist = Twist()
        while not self.reached():
            twist.angular.z = self._ang_zvel * utils.sign(utils.wrap_to_pi(self.theta_goal - self.theta))
            if not self.command(twist):
                return False
        self.ok = True
        return True

    def reached(self):
        if abs(utils.wrap_to_pi(self.theta_goal - self.theta)) < radians(5.0):
            return True
        else:
            return False

    def stop(self):
        self._stop = True

    def execute(self):
        if self._running:
            rospy.logerr("Kobuki TestSuite: already executing wandering, ignoring the request")
            return
        self._stop = False
        self._running = True
        while True:
            if not self.go() or not self.stepback() or not self.turn():
                break
        self._running = False
        if not rospy.is_shutdown():
            cmd = Twist()
            cmd.linear.x = 0.0
            self.cmd_vel_publisher.publish(cmd)

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
                self.theta_goal = utils.wrap_to_pi(self.theta + 3.141592*random.uniform(-1.0, 1.0))

    def cliff_event_callback(self, data):
        if data.state == CliffEvent.CLIFF:
            self.ok = False
            # print "Cliff event: %s,%s"%(str(data.sensor),str(data.state))
            if   data.sensor == CliffEvent.LEFT:
                self.theta_goal = self.theta - 3.141592*random.uniform(0.2, 1.0)
            elif data.sensor == CliffEvent.RIGHT:
                self.theta_goal = self.theta + 3.141592*random.uniform(0.2, 1.0)
            else:
                self.theta_goal = utils.wrap_to_pi(self.theta + 3.141592*random.uniform(-1.0, 1.0))
 
