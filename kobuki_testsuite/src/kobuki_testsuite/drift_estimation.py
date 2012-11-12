#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki/master/kobuki_testsuite/LICENSE 
#
##############################################################################
# Imports
##############################################################################

from math import *
import roslib; roslib.load_manifest('kobuki_qtestsuite')
import yaml
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32 as ScanAngle

##############################################################################
# Classes
##############################################################################

class ScanToAngle(object):
    def __init__(self, scan_topic, scan_angle_topic):
        self.min_angle = -0.3
        self.max_angle = 0.3
        self.scan_angle_publisher = rospy.Publisher(scan_angle_topic, ScanAngle)
        self.scan_subscriber = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)

    def init(self, min_angle, max_angle):
        self.min_angle = -0.3
        self.max_angle = 0.3
        
    def shutdown(self):
        self.stop()
        while self._running:
            self.rate.sleep()
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
    def __init__(self):
        pass

    def sync_timestamps(self, start_time=None):
        if not start_time:
            start_time = rospy.Time.now() + rospy.Duration(0.5)
        while not rospy.is_shutdown():
            rospy.sleep(0.3)
            with self.lock:
                if self.imu_time < start_time and self.has_gyro:
                    rospy.loginfo("Still waiting for imu")
                elif self.odom_time < start_time:
                    rospy.loginfo("Still waiting for odom")
                elif self.scan_time < start_time:
                    rospy.loginfo("Still waiting for scan")
                else:
                    return (self.imu_angle, self.odom_angle, self.scan_angle,
                            self.imu_time, self.odom_time, self.scan_time)
        exit(0)

    def align(self):
        self.sync_timestamps()
        rospy.loginfo("Aligning base with wall")
        with self.lock:
            angle = self.scan_angle
        cmd = Twist()

        while angle < -self.inital_wall_angle or angle > self.inital_wall_angle:
            if rospy.is_shutdown():
                exit(0)
            if angle > 0:
                cmd.angular.z = -0.3
            else:
                cmd.angular.z = 0.3
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.05)
            with self.lock:
                angle = self.scan_angle

def main():
    rospy.init_node('drift_estimation')
    robot = DriftEstimation()
    for speed in (0.3, 0.7, 1.0, 1.5):
        robot.align()

