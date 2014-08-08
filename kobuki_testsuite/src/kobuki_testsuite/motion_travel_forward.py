#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki/hydro-devel/kobuki_testsuite/LICENSE
#
##############################################################################
# Imports
##############################################################################

import math
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import CliffEvent
# Local imports
import utils

##############################################################################
# Classes
##############################################################################
    
'''
  Travels forward a set distance. 

      API:
        init(speed,distance) : (re)initialise parameters 
        stop()  - stop.
        execute() - pass this to a thread to run
        shutdown() - cleanup
'''
class TravelForward(object):
    '''
      Initialise everything
      
      @param topic names
      @type strings
    '''
    def __init__(self, cmd_vel_topic, odom_topic, cliff_sensor_topic):
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.cliff_sensor_subscriber = rospy.Subscriber(cliff_sensor_topic, CliffEvent , self.cliff_sensor_callback)
        self.speed = 0.7
        self.distance = 1.0
        self._current_pose = Pose()
        self._starting_pose = Pose()
        self._stop = False
        self._running = False
    
    def init(self, speed, distance):
        self.speed = speed
        self.distance = distance
        
    def shutdown(self):
        self.stop()
        while self._running:
            rospy.sleep(0.05)
        self.cmd_vel_publisher.unregister()
        self.odom_subscriber.unregister()
    
    def stop(self):
        self._stop = True

    def execute(self):
        '''
          Drop this into threading.Thread or QThread for execution
        '''
        if self._running:
            rospy.logerr("Kobuki TestSuite: already executing a motion, ignoring the request")
            return
        self._stop = False
        self._running = True
        rate = rospy.Rate(10)
        self._current_speed = 0.0
        current_distance_sq = 0.0
        distance_sq = self.distance*self.distance
        self._starting_pose = self._current_pose
        while not self._stop and not rospy.is_shutdown():
            if current_distance_sq > distance_sq:
                break
            else:
                current_distance_sq = (self._current_pose.position.x - self._starting_pose.position.x)*(self._current_pose.position.x - self._starting_pose.position.x) + \
                                   (self._current_pose.position.y - self._starting_pose.position.y)*(self._current_pose.position.y - self._starting_pose.position.y)
                #current_distance_sq += 0.01 # uncomment this and comment above for debugging
                print("Distance %s"%math.sqrt(current_distance_sq))
                if self.speed > 0:
                    if self._current_speed < self.speed:
                        self._current_speed += 0.01
                else:
                    if self._current_speed > self.speed:
                        self._current_speed -= 0.01
                cmd = Twist()
                cmd.linear.x = self._current_speed
                self.cmd_vel_publisher.publish(cmd)
            rate.sleep()
        if not rospy.is_shutdown():
            cmd = Twist()
            cmd.linear.x = 0.0
            self.cmd_vel_publisher.publish(cmd)
        self._running = False
        
    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    def cliff_sensor_callback(self, data):
        rospy.loginfo("Kobuki Testsuite: cliff event on sensor [%s]"%str(data.sensor))
        if data.state == CliffEvent.CLIFF:
            if not rospy.is_shutdown():
                cmd = Twist()
                cmd.linear.x = 0.0
                self.cmd_vel_publisher.publish(cmd)
            self.stop()
    
    def odometry_callback(self, data):
        self._current_pose = data.pose.pose
