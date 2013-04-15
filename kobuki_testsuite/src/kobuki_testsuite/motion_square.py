#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki/master/kobuki_testsuite/LICENSE 
#
##############################################################################
# Imports
##############################################################################

import rospy
import copy
import math
import PyKDL
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Imu

##############################################################################
# Classes
##############################################################################

class Pose2D(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.heading = None
        
    def configured(self):
        return self.x and self.y and self.heading
    
    def __str__(self):
        return ("[%s, %s, %s]"%(self.x, self.y, self.heading))

'''
    Implement a square motion
'''
class Square(object):
    STATE_FORWARD = "forward"
    STATE_STOP_FORWARD = "stop_forward"
    STATE_TURN = "turn"
    STATE_STOP_TURN = "stop_turn"
    
    def __init__(self,cmd_vel_topic, odom_topic, gyro_topic):
        self._rate = rospy.Rate(20)
        self._speed = 0.4
        self._side_distance = 1.0
        
        self._stop = False
        self._running = False
        self._turn_count = 0

        self._current_pose = Pose2D()
        self._starting_pose = None
        self._state = Square.STATE_FORWARD
        self.last_state = Square.STATE_FORWARD

        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic,Twist)
        self.gyro_subscriber  = rospy.Subscriber(gyro_topic, Imu, self.heading_callback)

    def init(self, speed, side_distance):
        # check for suitable values here
        self._speed = speed
        self._side_distance = side_distance

    def shutdown(self):
        self.stop()
        while self._running:
            rospy.sleep(0.5)
        self.cmd_vel_publisher.unregister()
        self.cmd_vel_publisher = None
        self.odom_subscriber.unregister()
        self.odom_subscriber = None
        self.gyro_subscriber.unregister()
        self.gyro_subscriber = None
        
    def stop(self):
        self._stop = True

    def execute(self):
        '''
          Drop this into threading.Thread or QThread for execution
        '''
        if self._running:
            rospy.logerr("Kobuki TestSuite: already executing a motion, ignoring the request")
            return
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while not self._current_pose.configured():
            if rospy.Time.now() > timeout:
                rospy.logerr("Kobuki TestSuite: no odometry data, aborting")
                return
            rospy.sleep(1.0)
            rospy.logwarn("Kobuki TestSuite: haven't received any odometry data yet")
        
        self._stop = False
        self._running = True
        start = rospy.get_rostime()
        rospy.sleep(0.5)

        #turning_arc_radius = side_distance/4.0
        rospy.loginfo("Kobuki Testsuite: executing a square motion pattern [%s]"%self._side_distance)
        self._starting_pose = copy.deepcopy(self._current_pose)
        
        while not self._stop and not rospy.is_shutdown():
            if self._state == Square.STATE_FORWARD:
                self._forward()
            elif self._state == Square.STATE_STOP_FORWARD:
                self._stop_forward()
            elif self._state == Square.STATE_TURN:
                self._turn()
            elif self._state == Square.STATE_STOP_TURN:
                self._stop_turn()
                if self._turn_count == 4:
                    break
            self._rate.sleep()
        if not rospy.is_shutdown():
            self._command(0.0, 0.0)
        self._running = False

    ##########################################################################
    # Utilities
    ##########################################################################
    
    def _has_reached_forward_goal(self):
        dx = self._current_pose.x - self._starting_pose.x
        dy = self._current_pose.y - self._starting_pose.y
        travelled_sq = dx*dx + dy*dy
        if travelled_sq > self._side_distance*self._side_distance:
            return True
        else:
            return False

    def _has_reached_turning_goal(self):
        #rospy.loginfo("Poses [%s]-[%s]"%(self._current_pose.heading,self._starting_pose.heading))
        if math.fabs(self._current_pose.heading - self._starting_pose.heading) > math.pi/2.0:
            return True
        else:
            return False
    
    def _command(self, linear, angular):
        '''
          Publishes a cmd_vel message for the kobuki.
        '''
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_publisher.publish(cmd)
        
    def _forward(self):
        if self._has_reached_forward_goal():
            rospy.loginfo("Kobuki TestSuite: commencing turn")
            self._state = Square.STATE_STOP_FORWARD
            #self._command(0.0,0.0)
        else:
            self._command(self._speed, 0.0)

    def _stop_forward(self):
        self._starting_pose =  copy.deepcopy(self._current_pose)
        self._state = Square.STATE_TURN
        alpha = 0.5
        self._command(alpha*self._speed, -alpha*self._speed/(self._side_distance/4.0))
        
    def _turn(self):
        if self._has_reached_turning_goal():
            self._state = Square.STATE_STOP_TURN
            #self._command(0.0,0.0)
        else:
            # set up a turning arc, 1/4 the length of the side
            # actually, this almost always creates incredibly fast turns, so crank it down by alpha
            alpha = 0.5
            self._command(alpha*self._speed, -alpha*self._speed/(self._side_distance/4.0))

    def _stop_turn(self):
        self._turn_count = self._turn_count + 1
        rospy.loginfo("Kobuki TestSuite: finished turn %s"%self._turn_count)
        self._starting_pose =  copy.deepcopy(self._current_pose)
        self._state = Square.STATE_FORWARD
        if self._turn_count == 4:
            self._command(0.0, 0.0)
        else:
            self._command(self._speed, 0.0)
        

    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    def odometry_callback(self, data):
        self._current_pose.x = data.pose.pose.position.x
        self._current_pose.y = data.pose.pose.position.y
        # if self._current_pose.configured():
        #     rospy.loginfo("TesSuite: %s"%self._current_pose)

    def heading_callback(self, data):
        quat = data.orientation
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        self._current_pose.heading = rot.GetRPY()[2]
        

if __name__ == '__main__':
    rospy.init_node('square')
    square = Square("/cmd_vel","/odom")
    rospy.spin()
