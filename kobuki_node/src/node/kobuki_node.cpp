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
#include <kobuki_comms/VersionInfo.h>
#include "kobuki_node/kobuki_node.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
 ** Implementation [KobukiNode]
 *****************************************************************************/

/**
 * @brief Default constructor.
 *
 * Make sure you call the init() method to fully define this node.
 */
KobukiNode::KobukiNode(std::string& node_name) :
    name(node_name),
    wheel_left_name("wheel_left"),
    wheel_right_name("wheel_right"),
    odom_frame("odom"),
    base_frame("base_footprint"),
    publish_tf(false),
    slot_version_info(&KobukiNode::publishVersionInfo, *this),
    slot_stream_data(&KobukiNode::processStreamData, *this),
    slot_button_event(&KobukiNode::publishButtonEvent, *this),
    slot_bumper_event(&KobukiNode::publishBumperEvent, *this),
    slot_debug(&KobukiNode::rosDebug, *this),
    slot_info(&KobukiNode::rosInfo, *this),
    slot_warn(&KobukiNode::rosWarn, *this),
    slot_error(&KobukiNode::rosError, *this)
{
  joint_states.name.push_back("left_wheel_joint");
  joint_states.name.push_back("right_wheel_joint");
  joint_states.name.push_back("front_wheel_joint"); // front_castor_joint in create tbot
  joint_states.name.push_back("rear_wheel_joint");  // back_castor_joint in create tbot
  joint_states.position.resize(4,0.0);
  joint_states.velocity.resize(4,0.0);
  joint_states.effort.resize(4,0.0);

  updater.setHardwareID("Kobuki");
  updater.add(battery_diagnostics);
}

/**
 * This will wait some time while kobuki internally closes its threads and destructs
 * itself.
 */
KobukiNode::~KobukiNode()
{
  ROS_INFO_STREAM("Kobuki : waiting for kobuki thread to finish [" << name << "].");
}

bool KobukiNode::init(ros::NodeHandle& nh)
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
  slot_button_event.connect(name + std::string("/button_event"));
  slot_bumper_event.connect(name + std::string("/bumper_event"));
  slot_debug.connect(name + std::string("/ros_debug"));
  slot_info.connect(name + std::string("/ros_info"));
  slot_warn.connect(name + std::string("/ros_warn"));
  slot_error.connect(name + std::string("/ros_error"));

  /*********************
   ** Parameters
   **********************/
  Parameters parameters;

  double tmp;
  nh.param("cmd_vel_timeout", tmp, 0.6);
  cmd_vel_timeout.fromSec(tmp);
  ROS_INFO_STREAM("Kobuki : Velocity commands timeout: " << tmp << " seconds [" << name << "].");

  nh.param("simulation", parameters.simulation, false);
  parameters.sigslots_namespace = name; // name is automatically picked up by device_nodelet parent.
  if (!nh.getParam("device_port", parameters.device_port))
  {
    ROS_ERROR_STREAM("Kobuki : no device port given on the parameter server (e.g. /dev/ttyUSB0)[" << name << "].");
    return false;
  }
  if (!nh.getParam("protocol_version", parameters.protocol_version))
  {
    ROS_ERROR_STREAM("Kobuki : no protocol version given on the parameter server ('2.0')[" << name << "].");
    return false;
  }

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
    if ( parameters.simulation ) {
      ROS_INFO("Kobuki : driver going into loopback (simulation) mode.");
    } else {
      ROS_INFO_STREAM("Kobuki : configured for connection on device_port " << parameters.device_port << " [" << name << "].");
      ROS_INFO_STREAM("Kobuki : configured for firmware protocol_version " << parameters.protocol_version << " [" << name << "].");
      ROS_INFO_STREAM("Kobuki : driver running in normal (non-simulation) mode" << " [" << name << "].");
    }
  }

  /*********************
   ** Frames
   **********************/
  if (!nh.getParam("odom_frame", odom_frame)) {
    ROS_WARN_STREAM("Kobuki : no param server setting for odom_frame, using default [" << odom_frame << "][" << name << "].");
  } else {
    ROS_INFO_STREAM("Kobuki : using odom_frame [" << odom_frame << "][" << name << "].");
  }

  if (!nh.getParam("base_frame", base_frame)) {
    ROS_WARN_STREAM("Kobuki : no param server setting for base_frame, using default [" << base_frame << "][" << name << "].");
  } else {
    ROS_INFO_STREAM("Kobuki : using base_frame [" << base_frame << "][" << name << "].");
  }

  if (!nh.getParam("publish_tf", publish_tf)) {
    ROS_WARN_STREAM("Kobuki : no param server setting for publish_tf, using default [" << publish_tf << "][" << name << "].");
  } else {
    ROS_INFO_STREAM("Kobuki : using publish_tf [" << publish_tf << "][" << name << "].");
  }

  odom_trans.header.frame_id = odom_frame;
  odom_trans.child_frame_id = base_frame;
  odom.header.frame_id = odom_frame;
  odom.child_frame_id = base_frame;

  // Pose covariance (required by robot_pose_ekf) TODO: publish realistic values
  // Odometry yaw covariance must be much bigger than the covariance provided
  // by the imu, as the later takes much better measures
  odom.pose.covariance[0]  = 0.1;
  odom.pose.covariance[7]  = 0.1;
  odom.pose.covariance[35] = 0.2;

  odom.pose.covariance[14] = DBL_MAX; // set a very large covariance on unused
  odom.pose.covariance[21] = DBL_MAX; // dimensions (z, pitch and roll); this
  odom.pose.covariance[28] = DBL_MAX; // is a requirement of robot_pose_ekf

  pose.setIdentity();

  /*********************
   ** Driver Init
   **********************/
  try
  {
    kobuki.init(parameters);
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
      case (ecl::NotFoundError):
      {
        ROS_ERROR_STREAM("Kobuki : could not find the device [" << parameters.device_port << "][" << name << "].");
        break;
      }
      default:
      {
        ROS_ERROR_STREAM("Kobuki : initialisation failed [" << name << "].");
        ROS_ERROR_STREAM(e.what());
        break;
      }
    }
    return false;
  }

//  ecl::SigSlotsManager<>::printStatistics();
//  ecl::SigSlotsManager<const std::string&>::printStatistics();
//  ecl::SigSlotsManager<const VersionInfo&>::printStatistics();
//  ecl::SigSlotsManager<const ButtonEvent&>::printStatistics();

  return true;
}

bool KobukiNode::spin()
{
  ros::Rate loop_rate(10); // 100ms - cmd_vel_timeout should be greater than this

  while (ros::ok())
  {
    if ((kobuki.isEnabled() == true) && (last_cmd_time.isZero() == false) &&
        ((ros::Time::now() - last_cmd_time) > cmd_vel_timeout)) {
      std_msgs::StringPtr msg;
      disable(msg);

      ROS_WARN("No cmd_vel messages received within the last %.2f seconds; disable driver",
                cmd_vel_timeout.toSec());
    }

    battery_diagnostics.update(kobuki.batteryStatus());
    updater.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return true;
}

/**
 * Two groups of publishers, one required by turtlebot, the other for
 * kobuki esoterics.
 */
void KobukiNode::advertiseTopics(ros::NodeHandle& nh)
{
  /*********************
  ** Turtlebot Required
  **********************/
  joint_state_publisher = nh.advertise <sensor_msgs::JointState>("joint_states",100);
  odom_publisher = nh.advertise<nav_msgs::Odometry>("odom", 50); // topic name and queue size

  /*********************
  ** Kobuki Esoterics
  **********************/
  version_info_publisher = nh.advertise < kobuki_comms::VersionInfo > ("version_info", 100, true); // latched publisher
  button_event_publisher = nh.advertise < kobuki_comms::ButtonEvent > ("events/buttons", 100);
  bumper_event_publisher = nh.advertise < kobuki_comms::BumperEvent > ("events/bumpers", 100);
  core_sensor_data_publisher = nh.advertise < kobuki_comms::CoreSensors > ("sensors/core", 100);
  cliff_sensor_publisher = nh.advertise < kobuki_comms::Cliff > ("sensors/cliff", 100);
  current_sensor_publisher = nh.advertise < kobuki_comms::Current > ("sensors/current", 100);
  gp_input_data_publisher = nh.advertise < kobuki_comms::GpInput > ("sensors/gp_inputs", 100);
  imu_data_publisher = nh.advertise < sensor_msgs::Imu > ("sensors/imu_data", 100);
}

/**
 * Two groups of subscribers, one required by turtlebot, the other for
 * kobuki esoterics.
 */
void KobukiNode::subscribeTopics(ros::NodeHandle& nh)
{
  velocity_command_subscriber = nh.subscribe(std::string("cmd_vel"), 10, &KobukiNode::subscribeVelocityCommand, this);
  led_command_subscriber =  nh.subscribe(std::string("led_command"), 10, &KobukiNode::subscribeLedCommand, this);
  // A group enable/disable channel to listen to (these should get remapped to /enable in most cases).
  enable_subscriber = nh.subscribe("enable", 10, &KobukiNode::enable, this); // 10 is queue size
  disable_subscriber = nh.subscribe("disable", 10, &KobukiNode::disable, this);
  reset_odometry_subscriber = nh.subscribe("reset_odometry", 10, &KobukiNode::subscribeResetOdometry, this);

}

void KobukiNode::publishTransform(const geometry_msgs::Quaternion &odom_quat)
{
  if (publish_tf == false)
    return;

  odom_trans.header.stamp = ros::Time::now();
  odom_trans.transform.translation.x = pose.x();
  odom_trans.transform.translation.y = pose.y();
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  odom_broadcaster.sendTransform(odom_trans);
}

void KobukiNode::publishOdom(const geometry_msgs::Quaternion &odom_quat,
                             const ecl::linear_algebra::Vector3d &pose_update_rates)
{
  odom.header.stamp = ros::Time::now();

  // Position
  odom.pose.pose.position.x = pose.x();
  odom.pose.pose.position.y = pose.y();
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // Velocity
  odom.twist.twist.linear.x = pose_update_rates[0];
  odom.twist.twist.linear.y = pose_update_rates[1];
  odom.twist.twist.angular.z = pose_update_rates[2];

  odom_publisher.publish(odom);
}


} // namespace kobuki


/*		slot_reserved0, Rei */

