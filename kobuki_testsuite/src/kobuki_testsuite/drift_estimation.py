#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki/master/kobuki_testsuite/LICENSE 
#
##############################################################################
# Imports
##############################################################################

from math import *
import threading
import roslib; roslib.load_manifest('kobuki_qtestsuite')
import PyKDL
import rospy
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float32 as ScanAngle
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

##############################################################################
# Utilities
##############################################################################

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
        
##############################################################################
# Classes
##############################################################################

class ScanToAngle(object):
    def __init__(self, scan_topic, scan_angle_topic):
        self.min_angle = -0.3
        self.max_angle = 0.3
        self.scan_angle_publisher = rospy.Publisher(scan_angle_topic, ScanAngle)
        self.scan_subscriber = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)

    def init(self, min_angle=-0.3, max_angle=0.3):
        self.min_angle = min_angle
        self.max_angle = max_angle
        
    def shutdown(self):
        self.scan_angle_publisher.unregister()
        self.scan_subscriber.unregister()

    def scan_callback(self, msg):
        angle = msg.angle_min
        d_angle = msg.angle_increment
        sum_x = 0
        sum_y = 0
        sum_xx = 0
        sum_xy = 0
        num = 0
        for r in msg.ranges:
            if angle > self.min_angle and angle < self.max_angle and r < msg.range_max:
                x = sin(angle) * r
                y = cos(angle) * r
                sum_x += x
                sum_y += y
                sum_xx += x*x
                sum_xy += x*y
                num += 1
            angle += d_angle
        if num > 0:
            denominator = num*sum_xx-sum_x*sum_x
            if denominator != 0:
                angle=atan2((-sum_x*sum_y+num*sum_xy)/(denominator), 1)
                print("Scan Angle: %s"%str(angle))
                scan_angle = ScanAngle()
                scan_angle.data = angle
                self.scan_angle_publisher.publish(scan_angle)
        else:
            rospy.logerr("Please point me at a wall.")

class DriftEstimation(object):
    def __init__(self, odom_topic, scan_angle_topic, cmd_vel_topic):
        self.lock = threading.Lock()

        self.gyro_subscriber  = rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, self.imu_callback)
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.scan_angle_subscriber = rospy.Subscriber(scan_angle_topic, ScanAngle, self.scan_callback)
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist)

        self.rate = rospy.Rate(30)

        self.inital_wall_angle = rospy.get_param("inital_wall_angle", 0.1)
        self.gyro_angle = 0
        self.gyro_time = rospy.Time.now()
        self.scan_angle = 0
        self.scan_time = rospy.Time.now()
        self.odom_angle = 0
        self.odom_time = rospy.Time.now()

        self._stop = False
        self._running = False

    def init(self):
        pass
    
    def shutdown(self):
        self.stop()
        while self._running:
            self.rate.sleep()
        self.odom_subscriber.unregister()
        self.scan_angle_subscriber.unregister()
        self.cmd_vel_publisher.unregister()
    
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
        start = rospy.get_rostime()
        while not self._stop and not rospy.is_shutdown():
            self.align()
        if not rospy.is_shutdown():
            cmd = Twist()
            cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd)
        self._running = False
    
    def align(self):
        self.sync_timestamps()
        rospy.loginfo("Aligning base with wall")
        with self.lock:
            angle = self.scan_angle
        cmd = Twist()

        while angle < -self.inital_wall_angle or angle > self.inital_wall_angle:
            if self._stop or rospy.is_shutdown():
                break
            if angle > 0:
                cmd.angular.z = -0.3
            else:
                cmd.angular.z = 0.3
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.05)
            with self.lock:
                angle = self.scan_angle

    def sync_timestamps(self, start_time=None):
        if not start_time:
            start_time = rospy.Time.now() + rospy.Duration(0.5)
        while not self._stop and not rospy.is_shutdown():
            rospy.sleep(0.3)
            with self.lock:
                if self.gyro_time < start_time:
                    rospy.loginfo("Still waiting for imu")
                elif self.odom_time < start_time:
                    rospy.loginfo("Still waiting for odom")
                elif self.scan_time < start_time:
                    rospy.loginfo("Still waiting for scan")
                else:
                    return (self.imu_angle, self.odom_angle, self.scan_angle,
                            self.gyro_time, self.odom_time, self.scan_time)
        #exit(0)

    ##########################################################################
    # Ros Callbacks
    ##########################################################################
    
    def imu_callback(self, msg):
        with self.lock:
            angle = quat_to_angle(msg.orientation)
            self.imu_angle = angle
            self.gyro_time = msg.header.stamp

    def odom_callback(self, msg):
        with self.lock:
            angle = quat_to_angle(msg.pose.pose.orientation)
            self.odom_angle = angle
            self.odom_time = msg.header.stamp

    def scan_callback(self, msg):
        with self.lock:
            angle = msg.scan_angle
            self.scan_angle = angle
            self.scan_time = msg.header.stamp

def main():
    rospy.init_node('drift_estimation')
    robot = DriftEstimation()
    for speed in (0.3, 0.7, 1.0, 1.5):
        robot.align()

