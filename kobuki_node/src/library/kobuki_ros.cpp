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
 * @file /kobuki_node/src/node/kobuki_node.cpp
 *
 * @brief Implementation for the ros kobuki node wrapper.
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <float.h>
#include <tf/tf.h>
#include <ecl/streams/string_stream.hpp>
#include <kobuki_msgs/VersionInfo.h>
#include "kobuki_node/kobuki_ros.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
 ** Implementation [KobukiRos]
 *****************************************************************************/

/**
 * @brief Default constructor.
 *
 * Make sure you call the init() method to fully define this node.
 */
KobukiRos::KobukiRos(std::string& node_name) :
    name(node_name), cmd_vel_timed_out_(false), serial_timed_out_(false),
    slot_version_info(&KobukiRos::publishVersionInfo, *this),
    slot_controller_info(&KobukiRos::publishControllerInfo, *this),
    slot_stream_data(&KobukiRos::processStreamData, *this),
    slot_button_event(&KobukiRos::publishButtonEvent, *this),
    slot_bumper_event(&KobukiRos::publishBumperEvent, *this),
    slot_cliff_event(&KobukiRos::publishCliffEvent, *this),
    slot_wheel_event(&KobukiRos::publishWheelEvent, *this),
    slot_power_event(&KobukiRos::publishPowerEvent, *this),
    slot_input_event(&KobukiRos::publishInputEvent, *this),
    slot_robot_event(&KobukiRos::publishRobotEvent, *this),
    slot_debug(&KobukiRos::rosDebug, *this),
    slot_info(&KobukiRos::rosInfo, *this),
    slot_warn(&KobukiRos::rosWarn, *this),
    slot_error(&KobukiRos::rosError, *this),
    slot_named(&KobukiRos::rosNamed, *this),
    slot_raw_data_command(&KobukiRos::publishRawDataCommand, *this),
    slot_raw_data_stream(&KobukiRos::publishRawDataStream, *this),
    slot_raw_control_command(&KobukiRos::publishRawControlCommand, *this)
{
  updater.setHardwareID("Kobuki");
  updater.add(battery_diagnostics);
  updater.add(watchdog_diagnostics);
  updater.add(bumper_diagnostics);
  updater.add(cliff_diagnostics);
  updater.add(wheel_diagnostics);
  updater.add(motor_diagnostics);
  updater.add(state_diagnostics);
  updater.add(gyro_diagnostics);
  updater.add(dinput_diagnostics);
  updater.add(ainput_diagnostics);
}

/**
 * This will wait some time while kobuki internally closes its threads and destructs
 * itself.
 */
KobukiRos::~KobukiRos()
{
  ROS_INFO_STREAM("Kobuki : waiting for kobuki thread to finish [" << name << "].");
}

bool KobukiRos::init(ros::NodeHandle& nh, ros::NodeHandle& nh_pub)
{
  /*********************
   ** Communications
   **********************/
  advertiseTopics(nh);
  subscribeTopics(nh);

  /*********************
   ** Slots
   **********************/
  slot_stream_data.connect(name + std::string("/stream_data"));
  slot_version_info.connect(name + std::string("/version_info"));
  slot_controller_info.connect(name + std::string("/controller_info"));
  slot_button_event.connect(name + std::string("/button_event"));
  slot_bumper_event.connect(name + std::string("/bumper_event"));
  slot_cliff_event.connect(name + std::string("/cliff_event"));
  slot_wheel_event.connect(name + std::string("/wheel_event"));
  slot_power_event.connect(name + std::string("/power_event"));
  slot_input_event.connect(name + std::string("/input_event"));
  slot_robot_event.connect(name + std::string("/robot_event"));
  slot_debug.connect(name + std::string("/ros_debug"));
  slot_info.connect(name + std::string("/ros_info"));
  slot_warn.connect(name + std::string("/ros_warn"));
  slot_error.connect(name + std::string("/ros_error"));
  slot_named.connect(name + std::string("/ros_named"));
  slot_raw_data_command.connect(name + std::string("/raw_data_command"));
  slot_raw_data_stream.connect(name + std::string("/raw_data_stream"));
  slot_raw_control_command.connect(name + std::string("/raw_control_command"));

  /*********************
   ** Driver Parameters
   **********************/
  Parameters parameters;

  nh.param("acceleration_limiter", parameters.enable_acceleration_limiter, false);
  nh.param("battery_capacity", parameters.battery_capacity, Battery::capacity);
  nh.param("battery_low", parameters.battery_low, Battery::low);
  nh.param("battery_dangerous", parameters.battery_dangerous, Battery::dangerous);

  parameters.sigslots_namespace = name; // name is automatically picked up by device_nodelet parent.
  if (!nh.getParam("device_port", parameters.device_port))
  {
    ROS_ERROR_STREAM("Kobuki : no device port given on the parameter server (e.g. /dev/ttyUSB0)[" << name << "].");
    return false;
  }

  /*********************
   ** Joint States
   **********************/
  std::string robot_description, wheel_left_joint_name, wheel_right_joint_name;

  nh.param("wheel_left_joint_name", wheel_left_joint_name, std::string("wheel_left_joint"));
  nh.param("wheel_right_joint_name", wheel_right_joint_name, std::string("wheel_right_joint"));

  // minimalistic check: are joint names present on robot description file?
  if (!nh_pub.getParam("robot_description", robot_description))
  {
    ROS_WARN("Kobuki : no robot description given on the parameter server");
  }
  else
  {
    if (robot_description.find(wheel_left_joint_name) == std::string::npos) {
      ROS_WARN("Kobuki : joint name %s not found on robot description", wheel_left_joint_name.c_str());
    }

    if (robot_description.find(wheel_right_joint_name) == std::string::npos) {
      ROS_WARN("Kobuki : joint name %s not found on robot description", wheel_right_joint_name.c_str());
    }
  }
  joint_states.name.push_back(wheel_left_joint_name);
  joint_states.name.push_back(wheel_right_joint_name);
  joint_states.position.resize(2,0.0);
  joint_states.velocity.resize(2,0.0);
  joint_states.effort.resize(2,0.0);

  /*********************
   ** Validation
   **********************/
  if (!parameters.validate())
  {
    ROS_ERROR_STREAM("Kobuki : parameter configuration failed [" << name << "].");
    ROS_ERROR_STREAM("Kobuki : " << parameters.error_msg << "[" << name << "]");
    return false;
  }
  else
  {
    if (parameters.simulation)
    {
      ROS_INFO("Kobuki : driver going into loopback (simulation) mode.");
    }
    else
    {
      ROS_INFO_STREAM("Kobuki : configured for connection on device_port "
                      << parameters.device_port << " [" << name << "].");
      ROS_INFO_STREAM("Kobuki : driver running in normal (non-simulation) mode" << " [" << name << "].");
    }
  }

  odometry.init(nh, name);

  /*********************
   ** Driver Init
   **********************/
  try
  {
    kobuki.init(parameters);
    ros::Duration(0.25).sleep(); // wait for some data to come in.
    if ( !kobuki.isAlive() ) {
      ROS_WARN_STREAM("Kobuki : no data stream, is kobuki turned on?");
      // don't need to return false here - simply turning kobuki on while spin()'ing should resurrect the situation.
    }
    kobuki.enable();
  }
  catch (const ecl::StandardException &e)
  {
    switch (e.flag())
    {
      case (ecl::OpenError):
      {
        ROS_ERROR_STREAM("Kobuki : could not open connection [" << parameters.device_port << "][" << name << "].");
        break;
      }
      default:
      {
        ROS_ERROR_STREAM("Kobuki : initialisation failed [" << name << "].");
        ROS_DEBUG_STREAM(e.what());
        break;
      }
    }
    return false;
  }
  // kobuki.printSigSlotConnections();
  return true;
}
/**
 * This is a worker function that runs in a background thread initiated by
 * the nodelet. It gathers diagnostics information from the kobuki driver,
 * and broadcasts the results to the rest of the ros ecosystem.
 *
 * Note that the actual driver data is collected via the slot callbacks in this class.
 *
 * @return Bool : true/false if successfully updated or not (kobuki driver shutdown).
 */
bool KobukiRos::update()
{
  if ( kobuki.isShutdown() )
  {
    ROS_ERROR_STREAM("Kobuki : Driver has been shutdown. Stopping update loop. [" << name << "].");
    return false;
  }

  if ( (kobuki.isEnabled() == true) && odometry.commandTimeout())
  {
    if ( !cmd_vel_timed_out_ )
    {
      kobuki.setBaseControl(0, 0);
      cmd_vel_timed_out_ = true;
      ROS_WARN("Kobuki : Incoming velocity commands not received for more than %.2f seconds -> zero'ing velocity commands", odometry.timeout().toSec());
    }
  }
  else
  {
    cmd_vel_timed_out_ = false;
  }

  bool is_alive = kobuki.isAlive();
  if ( watchdog_diagnostics.isAlive() && !is_alive )
  {
    if ( !serial_timed_out_ )
    {
      ROS_ERROR_STREAM("Kobuki : Timed out while waiting for serial data stream [" << name << "].");
      serial_timed_out_ = true;
    }
    else
    {
      serial_timed_out_ = false;
    }
  }

  watchdog_diagnostics.update(is_alive);
  battery_diagnostics.update(kobuki.batteryStatus());
  cliff_diagnostics.update(kobuki.getCoreSensorData().cliff, kobuki.getCliffData());
  bumper_diagnostics.update(kobuki.getCoreSensorData().bumper);
  wheel_diagnostics.update(kobuki.getCoreSensorData().wheel_drop);
  motor_diagnostics.update(kobuki.getCurrentData().current);
  state_diagnostics.update(kobuki.isEnabled());
  gyro_diagnostics.update(kobuki.getInertiaData().angle);
  dinput_diagnostics.update(kobuki.getGpInputData().digital_input);
  ainput_diagnostics.update(kobuki.getGpInputData().analog_input);
  updater.update();

  return true;
}

/**
 * Two groups of publishers, one required by turtlebot, the other for
 * kobuki esoterics.
 */
void KobukiRos::advertiseTopics(ros::NodeHandle& nh)
{
  /*********************
  ** Turtlebot Required
  **********************/
  joint_state_publisher = nh.advertise <sensor_msgs::JointState>("joint_states",100);

  /*********************
  ** Kobuki Esoterics
  **********************/
  version_info_publisher = nh.advertise < kobuki_msgs::VersionInfo > ("version_info",  100, true); // latched publisher
  controller_info_publisher = nh.advertise < kobuki_msgs::ControllerInfo > ("controller_info",  100, true); // latched publisher
  button_event_publisher = nh.advertise < kobuki_msgs::ButtonEvent > ("events/button", 100);
  bumper_event_publisher = nh.advertise < kobuki_msgs::BumperEvent > ("events/bumper", 100);
  cliff_event_publisher  = nh.advertise < kobuki_msgs::CliffEvent >  ("events/cliff",  100);
  wheel_event_publisher  = nh.advertise < kobuki_msgs::WheelDropEvent > ("events/wheel_drop", 100);
  power_event_publisher  = nh.advertise < kobuki_msgs::PowerSystemEvent > ("events/power_system", 100);
  input_event_publisher  = nh.advertise < kobuki_msgs::DigitalInputEvent > ("events/digital_input", 100);
  robot_event_publisher  = nh.advertise < kobuki_msgs::RobotStateEvent > ("events/robot_state", 100, true); // also latched
  sensor_state_publisher = nh.advertise < kobuki_msgs::SensorState > ("sensors/core", 100);
  dock_ir_publisher = nh.advertise < kobuki_msgs::DockInfraRed > ("sensors/dock_ir", 100);
  imu_data_publisher = nh.advertise < sensor_msgs::Imu > ("sensors/imu_data", 100);
  raw_imu_data_publisher = nh.advertise < sensor_msgs::Imu > ("sensors/imu_data_raw", 100);
  raw_data_command_publisher = nh.advertise< std_msgs::String > ("debug/raw_data_command", 100);
  raw_data_stream_publisher = nh.advertise< std_msgs::String > ("debug/raw_data_stream", 100);
  raw_control_command_publisher = nh.advertise< std_msgs::Int16MultiArray > ("debug/raw_control_command", 100);
}

/**
 * Two groups of subscribers, one required by turtlebot, the other for
 * kobuki esoterics.
 */
void KobukiRos::subscribeTopics(ros::NodeHandle& nh)
{
  velocity_command_subscriber = nh.subscribe(std::string("commands/velocity"), 10, &KobukiRos::subscribeVelocityCommand, this);
  led1_command_subscriber =  nh.subscribe(std::string("commands/led1"), 10, &KobukiRos::subscribeLed1Command, this);
  led2_command_subscriber =  nh.subscribe(std::string("commands/led2"), 10, &KobukiRos::subscribeLed2Command, this);
  digital_output_command_subscriber =  nh.subscribe(std::string("commands/digital_output"), 10, &KobukiRos::subscribeDigitalOutputCommand, this);
  external_power_command_subscriber =  nh.subscribe(std::string("commands/external_power"), 10, &KobukiRos::subscribeExternalPowerCommand, this);
  sound_command_subscriber =  nh.subscribe(std::string("commands/sound"), 10, &KobukiRos::subscribeSoundCommand, this);
  reset_odometry_subscriber = nh.subscribe("commands/reset_odometry", 10, &KobukiRos::subscribeResetOdometry, this);
  motor_power_subscriber = nh.subscribe("commands/motor_power", 10, &KobukiRos::subscribeMotorPower, this);
  controller_info_command_subscriber =  nh.subscribe(std::string("commands/controller_info"), 10, &KobukiRos::subscribeControllerInfoCommand, this);
}


} // namespace kobuki

