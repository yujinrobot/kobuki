#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki/hydro-devel/kobuki_testsuite/LICENSE
#
##############################################################################
# Imports
##############################################################################

import math
import threading
import PyKDL
import rospy
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import ScanAngle

##############################################################################
# Utilities
##############################################################################

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]

def wrap_angle(angle):
    if angle <= math.pi and angle >= -math.pi:
        return angle
    elif angle < 0.0:
        return math.fmod(angle-math.pi,2.0*math.pi)+math.pi
    else:
        return math.fmod(angle+math.pi,2.0*math.pi)-math.pi;

##############################################################################
# Classes
##############################################################################

class ScanToAngle(object):
    def __init__(self, scan_topic, scan_angle_topic):
        self.min_angle = -0.3
        self.max_angle = 0.3
        self.lock = threading.Lock() # make sure we don't publish if the publisher is not there
        self._laser_scan_angle_publisher = rospy.Publisher(scan_angle_topic, ScanAngle, queue_size=10)
        self.scan_subscriber = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)

    def init(self, min_angle=-0.3, max_angle=0.3):
        self.min_angle = min_angle
        self.max_angle = max_angle

    def shutdown(self):
        print("Killing off scan angle publisher")
        if not rospy.is_shutdown():
            with self.lock:
                self.scan_subscriber.unregister()
                self.scan_subscriber = None
                self._laser_scan_angle_publisher.unregister()
                del self._laser_scan_angle_publisher
                self._laser_scan_angle_publisher = None

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
                x = math.sin(angle) * r
                y = math.cos(angle) * r
                sum_x += x
                sum_y += y
                sum_xx += x*x
                sum_xy += x*y
                num += 1
            angle += d_angle
        if num > 0:
            denominator = num*sum_xx-sum_x*sum_x
            if denominator != 0:
                angle = math.atan2((-sum_x*sum_y+num*sum_xy)/(denominator), 1)
                # print("Scan Angle: %s"%str(angle))
                relay = ScanAngle()
                relay.header = msg.header
                relay.scan_angle = angle
                with self.lock:
                    if self._laser_scan_angle_publisher:
                        self._laser_scan_angle_publisher.publish(relay)
        else:
            rospy.logerr("Please point me at a wall.")


class DriftEstimation(object):
    def __init__(self, laser_scan_angle_topic, gyro_scan_angle_topic, error_scan_angle_topic, cmd_vel_topic, gyro_topic):
        self.lock = threading.Lock()

        self._gyro_scan_angle_publisher = rospy.Publisher(gyro_scan_angle_topic, ScanAngle, queue_size=10)
        self._laser_scan_angle_subscriber = rospy.Subscriber(laser_scan_angle_topic, ScanAngle, self.scan_callback)
        self._error_scan_angle_publisher = rospy.Publisher(error_scan_angle_topic, ScanAngle, queue_size=10)
        self.gyro_subscriber  = rospy.Subscriber(gyro_topic, Imu, self.gyro_callback)
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        self.rate = rospy.Rate(30)

        self._gyro_angle = 0
        self._gyro_time = rospy.Time.now()
        self._scan_angle = 0
        self._scan_time = rospy.Time.now()

        ##############################
        # Parameters
        ##############################
        self._inital_wall_angle = 0.1
        self._max_angle = 0.4
        self._abs_yaw_rate = 0.3
        self._epsilon = 0.5  # radians value above which increasing difference in scan and gyro angle will be ignored

        self._centred_gyro_angle = None
        self._initial_gyro_offset = None

        self._stop = False
        self._running = False

    def init(self, yaw_rate):
        self._abs_yaw_rate = yaw_rate

    def shutdown(self):
        self.stop()
        while self._running:
            self.rate.sleep()
        if not rospy.is_shutdown():
            print("Shutting down drift estimation")
            self._gyro_scan_angle_publisher.unregister()
            self._gyro_scan_angle_publisher = None
            self._laser_scan_angle_subscriber.unregister()
            self._laser_scan_angle_subscriber = None
            self._error_scan_angle_publisher.unregister()
            self._error_scan_angle_publisher = None
            self.gyro_subscriber.unregister()
            self.gyro_subscriber = None
            self.cmd_vel_publisher.unregister()
            self.cmd_vel_publisher = None

    def stop(self):
        self._stop = True

    def execute(self):
        '''
          Drop this into threading.Thread or QThread for execution
        '''
        if self._running:
            rospy.logerr("Kobuki TestSuite: already executing, ignoring the request")
            return
        self._stop = False
        self._running = True
        if not self.align(): # centred_gyro_angle set in here
            rospy.loginfo("Kobuki Testsuite: could not align, please point me at a wall.")
            self._initial_gyro_offset = None
            self._running = False
            return
        with self.lock:
            if not self._initial_gyro_offset:
                self._initial_gyro_offset = self._gyro_angle - self._scan_angle
                print("Kobuki Testsuite: initial centre [%s]" % self._centred_gyro_angle)
                print("Kobuki Testsuite: initial offset [%s]" % self._initial_gyro_offset)
        last_gyro_time = rospy.get_rostime()
        last_scan_time = rospy.get_rostime()
        yaw_rate_cmd = self._abs_yaw_rate
        turn_count = 0
        gyro_timeout_count = 0
        while not self._stop and not rospy.is_shutdown():
            with self.lock:
                gyro_time = self._gyro_time
                scan_time = self._scan_time
                gyro_angle = self._gyro_angle
                scan_angle = self._scan_angle
            if gyro_time > last_gyro_time:
                last_gyro_time = gyro_time
                if wrap_angle(gyro_angle - self._centred_gyro_angle) > self._max_angle:
                    if yaw_rate_cmd > 0:
                        turn_count = turn_count + 1
                    yaw_rate_cmd = -self._abs_yaw_rate
                elif wrap_angle(gyro_angle - self._centred_gyro_angle) < -self._max_angle:
                    yaw_rate_cmd = self._abs_yaw_rate
                else:
                    yaw_rate_cmd = cmp(yaw_rate_cmd,0)*self._abs_yaw_rate
                cmd = Twist()
                cmd.angular.z = yaw_rate_cmd
                self.cmd_vel_publisher.publish(cmd)
                if scan_time > last_scan_time:
                    rospy.loginfo("Kobuki Testsuite: gyro, laser angle comparison [%s,%s]"%(gyro_angle - self._initial_gyro_offset, scan_angle))
                    last_scan_time = scan_time
            else:
                gyro_timeout_count = gyro_timeout_count + 1
                if gyro_timeout_count > 50:
                    rospy.logerr("Kobuki Testsuite: no gyro data for a long time.")
                    break
            if turn_count > 4:
                rospy.loginfo("Kobuki Testsuite: aligning.....")
                self.align()
                turn_count = 0
            rospy.sleep(0.05)
        if not rospy.is_shutdown():
            cmd = Twist()
            cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd)
        self._initial_gyro_offset = None
        self._running = False

    def align(self):
        with self.lock:
            angle = self._scan_angle
        no_data_count = 0
        count = 0
        cmd = Twist()
        while True: #self._inital_wall_angle:
            if angle == 0: # magic number, means we have no data yet
                no_data_count += 1
                if no_data_count == 40:
                    return False
            if self._stop or rospy.is_shutdown():
                return False
            elif count > 20:
                with self.lock:
                    self._centred_gyro_angle = self._gyro_angle
                cmd.angular.z = 0.0
                self.cmd_vel_publisher.publish(cmd)
                return True
            if math.fabs(angle) < 0.05:
                count = count + 1
            # The work
            if angle > 0:
                cmd.angular.z = -0.2
            else:
                cmd.angular.z = 0.2
            self.cmd_vel_publisher.publish(cmd)
            rospy.sleep(0.05)
            with self.lock:
                angle = self._scan_angle
        print("end of align")

    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    def gyro_callback(self, msg):
        with self.lock:
            angle = quat_to_angle(msg.orientation)
            self._gyro_angle = angle
            self._gyro_time = msg.header.stamp
            if self._initial_gyro_offset:
                gyro_scan_angle = ScanAngle()
                gyro_scan_angle.header = msg.header
                gyro_scan_angle.scan_angle = angle - self._initial_gyro_offset
                self._gyro_scan_angle_publisher.publish(gyro_scan_angle)
                if self._running:
                    error_scan_angle = ScanAngle()
                    error_scan_angle.header = msg.header
                    error_scan_angle.scan_angle = math.fabs(angle - self._initial_gyro_offset - self._scan_angle)
                    #if error_scan_angle.scan_angle < self._epsilon: # don't spam with errors greater than what we're looking for
                    self._error_scan_angle_publisher.publish(error_scan_angle)

    def scan_callback(self, msg):
        with self.lock:
            #rospy.loginfo("Kobuki Testsuite: scan angle [%s]"%msg.scan_angle)
            self._scan_angle = msg.scan_angle
            self._scan_time = msg.header.stamp


