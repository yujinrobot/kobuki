/*
 * Copyright (c) 2012, Yujin Robot.
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
/**
 * @file /kobuki_node/include/kobuki_node/kobuki_node.hpp
 *
 * @brief Wraps the kobuki driver in a ROS node
 *
 * @date 10/04/2012
 **/

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef KOBUKI_NODE_HPP_
#define KOBUKI_NODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <ecl/sigslots.hpp>
#include <kobuki_comms/ButtonEvent.h>
#include <kobuki_comms/BumperEvent.h>
#include <kobuki_comms/CliffEvent.h>
#include <kobuki_comms/WheelDropEvent.h>
#include <kobuki_comms/PowerSystemEvent.h>
#include <kobuki_comms/DigitalInputEvent.h>
#include <kobuki_comms/DigitalOutput.h>
#include <kobuki_comms/Led.h>
#include <kobuki_comms/SensorState.h>
#include <kobuki_comms/DockInfraRed.h>
#include <kobuki_comms/Sound.h>
#include <kobuki_comms/VersionInfo.h>
#include <kobuki_driver/kobuki.hpp>
#include "diagnostics.hpp"
#include "odometry.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{
class KobukiNode
{
public:
  KobukiNode(std::string& node_name);
  ~KobukiNode();
  bool init(ros::NodeHandle& nh);
  bool spin();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  /*********************
   ** Variables
   **********************/
  std::string name; // name of the ROS node
  Kobuki kobuki;
  sensor_msgs::JointState joint_states;
  Odometry odometry;

  /*********************
   ** Ros Comms
   **********************/
  ros::Publisher version_info_publisher;
  ros::Publisher imu_data_publisher, sensor_state_publisher, joint_state_publisher, dock_ir_publisher;
  ros::Publisher button_event_publisher, input_event_publisher;
  ros::Publisher bumper_event_publisher, cliff_event_publisher, wheel_event_publisher, power_event_publisher;
  ros::Publisher raw_data_command_publisher;

  ros::Subscriber velocity_command_subscriber, digital_output_command_subscriber, external_power_command_subscriber;
  ros::Subscriber led1_command_subscriber, led2_command_subscriber, sound_command_subscriber;
  ros::Subscriber reset_odometry_subscriber;
  ros::Subscriber enable_subscriber, disable_subscriber; // may eventually disappear

  void advertiseTopics(ros::NodeHandle& nh);
  void subscribeTopics(ros::NodeHandle& nh);

  /*********************
  ** Ros Callbacks
  **********************/
  void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr);
  void subscribeLed1Command(const kobuki_comms::LedConstPtr);
  void subscribeLed2Command(const kobuki_comms::LedConstPtr);
  void subscribeDigitalOutputCommand(const kobuki_comms::DigitalOutputConstPtr);
  void subscribeExternalPowerCommand(const kobuki_comms::DigitalOutputConstPtr);
  void subscribeResetOdometry(const std_msgs::EmptyConstPtr);
  void subscribeSoundCommand(const kobuki_comms::SoundConstPtr);
  void enable(const std_msgs::StringConstPtr msg);
  void disable(const std_msgs::StringConstPtr msg);

  /*********************
   ** SigSlots
   **********************/
  ecl::Slot<const VersionInfo&> slot_version_info;
  ecl::Slot<> slot_stream_data;
  ecl::Slot<const ButtonEvent&> slot_button_event;
  ecl::Slot<const BumperEvent&> slot_bumper_event;
  ecl::Slot<const CliffEvent&>  slot_cliff_event;
  ecl::Slot<const WheelEvent&>  slot_wheel_event;
  ecl::Slot<const PowerEvent&>  slot_power_event;
  ecl::Slot<const InputEvent&>  slot_input_event;
  ecl::Slot<const std::string&> slot_debug, slot_info, slot_warn, slot_error;
  ecl::Slot<Command::Buffer&> slot_raw_data_command;

  /*********************
   ** Slot Callbacks
   **********************/
  void processStreamData();
  void publishWheelState();
  void publishInertia();
  void publishSensorState();
  void publishDockIRData();
  void publishVersionInfo(const VersionInfo &version_info);
  void publishButtonEvent(const ButtonEvent &event);
  void publishBumperEvent(const BumperEvent &event);
  void publishCliffEvent(const CliffEvent &event);
  void publishWheelEvent(const WheelEvent &event);
  void publishPowerEvent(const PowerEvent &event);
  void publishInputEvent(const InputEvent &event);

  // debugging
  void rosDebug(const std::string &msg) { ROS_DEBUG_STREAM("Kobuki : " << msg); }
  void rosInfo(const std::string &msg) { ROS_INFO_STREAM("Kobuki : " << msg); }
  void rosWarn(const std::string &msg) { ROS_WARN_STREAM("Kobuki : " << msg); }
  void rosError(const std::string &msg) { ROS_ERROR_STREAM("Kobuki : " << msg); }
  void publishRawDataCommand(Command::Buffer &buffer);

  /*********************
  ** Diagnostics
  **********************/
  diagnostic_updater::Updater updater;
  BatteryTask     battery_diagnostics;
  WatchdogTask   watchdog_diagnostics;
  CliffSensorTask   cliff_diagnostics;
  WallSensorTask   bumper_diagnostics;
  WheelDropTask     wheel_diagnostics;
  MotorCurrentTask  motor_diagnostics;
  GyroSensorTask     gyro_diagnostics;
  DigitalInputTask dinput_diagnostics;
  AnalogInputTask  ainput_diagnostics;
};

} // namespace kobuki

#endif /* KOBUKI_NODE_HPP_ */
