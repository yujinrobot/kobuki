/*
 * Copyright (c) 2013, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FAKE_KOBUKI_NODE_H_
#define _FAKE_KOBUKI_NODE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/DigitalOutput.h>
#include <kobuki_msgs/DigitalInputEvent.h>
#include <kobuki_msgs/ExternalPower.h>
#include <kobuki_msgs/DockInfraRed.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/MotorPower.h>
#include <kobuki_msgs/PowerSystemEvent.h>
#include <kobuki_msgs/RobotStateEvent.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/Sound.h>
#include <kobuki_msgs/VersionInfo.h>
#include <kobuki_msgs/WheelDropEvent.h>

#include "fake_kobuki.h"

namespace kobuki
{
  class FakeKobukiRos
  {
    public:
      FakeKobukiRos(std::string& node_name);
      ~FakeKobukiRos();

      bool init(ros::NodeHandle& nh);
      bool update();

    private:
      // private functions
      void advertiseTopics(ros::NodeHandle& nh);
      void subscribeTopics(ros::NodeHandle& nh);
      void publishVersionInfoOnce();

      // subscriber callbacks
      void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg);
      void subscribeMotorPowerCommand(const kobuki_msgs::MotorPowerConstPtr msg);

      void updateJoint(unsigned int index,double& w,ros::Duration step_time);
      void updateOdometry(double w_left,double w_right, ros::Duration step_time);
      void updateTF(geometry_msgs::TransformStamped& odom_tf);

      ///////////////////////////
      // Variables 
      //////////////////////////
      std::string name;
      ros::Time last_cmd_vel_time;
      ros::Time prev_update_time;

      // version_info, joint_states
      std::map<std::string,ros::Publisher> publisher;
      // button, bumper, cliff, wheel_drop, power_system, digital_input, robot_state
      std::map<std::string,ros::Publisher> event_publisher;
      // sensor_core, dock_ir, imu_data
      std::map<std::string,ros::Publisher> sensor_publisher;
      // no debug publisher
      tf::TransformBroadcaster        tf_broadcaster;

      // command subscribers
      std::map<std::string,ros::Subscriber> subscriber;

      FakeKobuki kobuki;
  };
}
#endif
