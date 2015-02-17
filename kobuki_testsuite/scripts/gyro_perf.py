#!/usr/bin/env python
#AUTHOR: Younghun Ju <yhju@yujinrobot.com>, <yhju83@gmail.com>

import roslib; roslib.load_manifest('kobuki_testsuite')
import rospy

import time
from math import pi
from kobuki_msgs.msg import ButtonEvent
from kobuki_msgs.msg import Sound
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D

from tf.transformations import euler_from_quaternion
from math import degrees, radians
from numpy import mod

from kobuki_msgs.msg import ScanAngle

def wrap_to_pi(x):
    a = mod(mod(x,2*pi)+2*pi, 2*pi)
    if (a > pi):
      a -= 2*pi
    return a

class Tester(object):
  def __init__(self):
    rospy.init_node("gyro_perf")
    self.state='INIT'
    self.done = False
    self.reset_angle = True
    self.triggered = False
    self.scan_angle = None # Place holder until the first scan angle is received

    self.debug = rospy.get_param('~debug', False)

    self.command_vx = rospy.get_param('~command_vx', 0.1) # In [m/s], default is 0.1 m/s
    self.command_wz = radians(rospy.get_param('~command_wz', 5.0)) # In [deg/s], default is 5 deg/s
    # convert into [rad/s]
    if self.debug: print self.command_vx, self.command_wz

    self.init_imu = True
    self.init_angle = 0.0
    self.angle = 0.0
    self.angle_rate = 0.0
    self.diff_angle = 0.0
    self.accumulated_angle = 0.0

    self.init_time = rospy.Time.now().to_time()
    if self.debug: print 'init time: {0:2.4f}'.format(self.init_time)

    self.test_angle = rospy.get_param('~test_angle', 360.0) # in [deg], default is 360 deg
    self.max_sample = rospy.get_param('~max_sample', 500) # in count, default is 500 sample

    self.use_button = rospy.get_param('~use_button', False)

    self.sub_angle = rospy.Subscriber("angle_abs", ScanAngle, self.angleCallback) # Absolute angle from laser scanner
    self.sub_gyro = rospy.Subscriber("imu_data", Imu, self.imuCallback)   # /mobile_base/sensors/imu_data
    self.sub_button = rospy.Subscriber("button", ButtonEvent, self.buttonCallback)
    self.pub_velocity = rospy.Publisher("cmd_vel", Twist)                 # /mobile_base/commands/velocity
    self.pub_sound = rospy.Publisher("sound", Sound)

    if self.debug: print 'state:', self.state
    self.state='WAIT_CONNECTION'
    if self.debug: print 'state:', self.state
    rospy.Timer(rospy.Duration.from_sec(0.02), self.timerCallback) # 50 Hz

  def ending_condition(self):
    cond = abs(self.accumulated_angle) >= abs(radians(self.test_angle))
    return cond

  def timerCallback(self, event):
    if rospy.is_shutdown(): return
    if self.done: return

    self.elapsed_time = event.current_real.to_time()-self.init_time
    if event.last_real == None:
      event.last_real = rospy.Time.from_sec(0.0)
      event.last_expected = rospy.Time.from_sec(0.0)
      event.last_duration = 0.0

    if self.state == 'WAIT_CONNECTION':
      if self.sub_angle.get_num_connections() > 0 and\
        self.sub_gyro.get_num_connections() > 0 and\
        self.pub_velocity.get_num_connections() > 0:
          if self.debug: print 'connection ready.'

          self.state = 'ALIGNING'
          if self.debug: print 'state:', self.state
          return
      else:
        if self.debug: print 'not ready yet.'
        return

    if self.state == "ALIGNING":
      if self.scan_angle is None:
        return
      scan_angle = self.scan_angle
      if abs(scan_angle) > radians(1.0):
        cmd_vel = Twist()
        cmd_vel.angular.z = -0.33*scan_angle/abs(scan_angle)
        self.pub_velocity.publish(cmd_vel)
        self.init_time = rospy.Time.now().to_time()
        if self.debug: print 'state:', self.state, ':', scan_angle, '-->', cmd_vel.angular.z
        return
      else:
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0  # in [m/s]
        cmd_vel.angular.z = 0.0 # in [rad/s]
        self.pub_velocity.publish(cmd_vel)

        if self.elapsed_time > 5.0:
          if self.debug: print 'alining ready.'

          self.pub_sound.publish(Sound(value=Sound.RECHARGE))
          self.state = 'WAIT_TRIGGER'
          if self.debug: print 'state:', self.state
          return
        else:
          if self.debug: print 'state:', self.state
          return

    if self.state == 'WAIT_TRIGGER':
      if self.triggered or not self.use_button:
          self.pub_sound.publish(Sound(value=Sound.CLEANINGEND))
          self.state = 'CALC_ANGLE_PRERUN'
          self.reset_angle = True
          self.angle_count = 0
          if self.debug: print 'state:', self.state
          return
      else:
          return

    if self.state == 'CALC_ANGLE_PRERUN':
      if self.angle_count >= self.max_sample:
        self.angle_prerun = self.angle_avg
        if self.debug: print 'angle prerun: ', self.angle_prerun

        self.pub_sound.publish(Sound(value=Sound.RECHARGE))

        self.state = 'RUNNING'
        self.init_time = rospy.Time.now().to_time()
        if self.debug: print 'state:', self.state

        self.init_imu = True
        self.init_angle = 0.0
        self.angle = 0.0
        self.angle_rate = 0.0
        self.diff_angle = 0.0
        self.accumulated_angle = 0.0

        return
      else:
        if self.debug: print 'state:', self.state, ':', self.angle_count, self.scan_angle, self.angle_sum, self.angle_avg
        return

    if self.state == 'RUNNING':
      if self.ending_condition():
        self.state = 'STOP'
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0  # in [m/s]
        cmd_vel.angular.z = 0.0 # in [rad/s]
        self.pub_velocity.publish(cmd_vel)
        self.running_time = self.elapsed_time
        self.init_time = rospy.Time.now().to_time()
        if self.debug: print 'state:', self.state
        return
      else:
        cmd_vel = Twist()
        cmd_vel.linear.x = self.command_vx  # In [m/s]
        cmd_vel.angular.z = self.command_wz # In [rad/s]
        self.pub_velocity.publish(cmd_vel)
        if self.debug: print 'state:', self.state, ':', degrees(self.accumulated_angle), self.test_angle
        return

    if self.state == 'STOP':
      if self.elapsed_time > 5.0:
        self.reset_angle = True
        self.angle_count = 0
        self.state = 'CALC_ANGLE_POSTRUN'
        if self.debug: print 'state:', self.state
        return
      else:
        if self.debug: print 'state:', self.state
        return

    if self.state == 'CALC_ANGLE_POSTRUN':
      if self.angle_count >= self.max_sample:
        self.angle_postrun = self.angle_avg

        angle_gyro = self.accumulated_angle
        sign = angle_gyro/abs(angle_gyro)
        angle_laser = sign*radians(self.test_angle)+wrap_to_pi(self.angle_postrun - self.angle_prerun)

        rospy.loginfo('test_angle:    {0} deg'.format(self.test_angle))
        rospy.loginfo('test_command:  {0} m/s {1} deg/s'.format(self.command_vx, degrees(self.command_wz)))
        rospy.loginfo('running_time:  {0} s'.format(self.running_time))
        rospy.loginfo('')
        rospy.loginfo('angle_prerun:  {0} deg'.format(degrees(self.angle_prerun)))
        rospy.loginfo('angle_postrun: {0} deg'.format(degrees(self.angle_postrun)))
        rospy.loginfo('angle_gyro:    {0} deg'.format(degrees(angle_gyro)))
        rospy.loginfo('angle_laser:   {0} deg'.format(degrees(angle_laser)))
        rospy.loginfo('')
        rospy.loginfo('error:         {0} deg/rev'.format(angle_gyro/angle_laser*360.0-360.0))
        rospy.loginfo('Done')
        self.pub_sound.publish(Sound(value=Sound.CLEANINGSTART))

        self.state = 'DONE'
        self.done = True
        if self.debug: print 'state:', self.state
        rospy.signal_shutdown('jobs_done')
        time.sleep(5.0)
        return
      else:
        if self.debug: print 'state:', self.state, ':', self.angle_count, self.scan_angle, self.angle_sum, self.angle_avg
        return

  def imuCallback(self,data):
    if rospy.is_shutdown(): return
    quat = data.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)

    if self.init_imu:
      self.init_angle = yaw
      self.init_imu = False

    self.diff_angle = wrap_to_pi(yaw - self.init_angle - self.angle)
    self.accumulated_angle += self.diff_angle
    self.angle = wrap_to_pi(yaw - self.init_angle)
    self.angle_rate = data.angular_velocity.z

  def angleCallback(self,data):
    if rospy.is_shutdown(): return

    self.scan_angle = data.scan_angle

    if self.reset_angle:
      if self.debug: print 'Resetted'
      self.angle_count = 0
      self.angle_sum = 0
      self.scan_angle = 0
      self.reset_angle = False

    if self.angle_count >= self.max_sample:
      return

    self.angle_count += 1
    self.angle_sum += data.scan_angle
    self.angle_avg = self.angle_sum / self.angle_count

  def buttonCallback(self,data):
    if rospy.is_shutdown(): return
    if data.button == ButtonEvent.Button0 and data.state == ButtonEvent.RELEASED:
      #if self.debug: print 'button0 pressed'
      self.triggered=True
      return
    #if data.button == ButtonEvent.Button1 and data.state == ButtonEvent.RELEASED:
    #  if self.debug: print 'button1 pressed'; return
    #if data.button == ButtonEvent.Button2 and data.state == ButtonEvent.RELEASED:
    #  if self.debug: print 'button2 pressed'; return

if __name__ == '__main__':
  print
  print "It test gyro on kobuki and print gyro drift errors."
  print
  try:
    instance = Tester()
    rospy.spin()
  except rospy.ROSInterruptException: pass
