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
 * @file src/node/slot_callbacks.cpp
 *
 * @brief All the slot callbacks for interrupts from the kobuki driver.
 *
 * @date Apr 10, 2012
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <kobuki_comms/VersionInfo.h>
#include <kobuki_driver/modules/gp_input.hpp>
#include "kobuki_node/kobuki_node.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

void KobukiNode::processStreamData() {
  publishWheelState();
  publishCoreSensors();
  publishInertia();
  publishCliffData();
  publishCurrentData();
  publishGpInputData();
}

void KobukiNode::publishWheelState()
{
  if (ros::ok())
  {
    ecl::Pose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;

    kobuki.updateOdometry(pose_update, pose_update_rates);
    kobuki.getWheelJointStates(joint_states.position[0],joint_states.velocity[0],   // left wheel
                               joint_states.position[1],joint_states.velocity[1] ); // right wheel

    joint_states.header.stamp = ros::Time::now();
    joint_state_publisher.publish(joint_states);

    pose *= pose_update;

    //since all ros tf odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.heading());

    publishTransform(odom_quat);
    publishOdom(odom_quat, pose_update_rates);
  }
}

void KobukiNode::publishCoreSensors()
{
  if (ros::ok())
  {
    CoreSensors::Data data;
    kobuki.getCoreSensorData(data);

    if (core_sensor_data_publisher.getNumSubscribers() > 0)
    {
      // convert data format
      kobuki_comms::CoreSensors ros_data;
      ros_data.header.stamp = ros::Time::now();
      ros_data.time_stamp = data.time_stamp; // firmware time stamp
      ros_data.bump = data.bump;
      ros_data.wheel_drop = data.wheel_drop;
      ros_data.cliff = data.cliff;
      ros_data.left_encoder = data.left_encoder;
      ros_data.right_encoder = data.right_encoder;
      ros_data.left_pwm = data.left_pwm;
      ros_data.right_pwm = data.right_pwm;
      ros_data.buttons = data.buttons;
      ros_data.charger = data.charger;
      ros_data.battery = data.battery;
      core_sensor_data_publisher.publish(ros_data);
    }
  }
}

void KobukiNode::publishInertia()
{
  if (ros::ok())
  {
    if (imu_data_publisher.getNumSubscribers() > 0)
    {
      sensor_msgs::Imu msg;
      msg.header.frame_id = "gyro_link";
      msg.header.stamp = ros::Time::now();

      msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, kobuki.getHeading());

      // set a very large covariance on unused dimensions (pitch and roll);
      // set yaw covariance as very low, to make it dominate over the odometry heading
      // 1: fill once, as its always the same;  2: cannot get better estimation?
      msg.orientation_covariance[0] = DBL_MAX;
      msg.orientation_covariance[4] = DBL_MAX;
      msg.orientation_covariance[8] = 0.005;

      // fill angular velocity; we ignore acceleration for now
      msg.angular_velocity.z = kobuki.getAngularVelocity();

      // angular velocity covariance; useless by now, but robot_pose_ekf's
      // roadmap claims that it will compute velocities in the future
      msg.angular_velocity_covariance[0] = DBL_MAX;
      msg.angular_velocity_covariance[4] = DBL_MAX;
      msg.angular_velocity_covariance[8] = 0.005;

      imu_data_publisher.publish(msg);
    }
  }
}

void KobukiNode::publishCliffData()
{
  if (ros::ok())
  {
    if (cliff_sensor_publisher.getNumSubscribers() > 0)
    {
      Cliff::Data data;
      kobuki.getCliffData(data);
      kobuki_comms::Cliff ros_data;
      ros_data.header.stamp = ros::Time::now();
      ros_data.bottom = data.bottom;
      cliff_sensor_publisher.publish(ros_data);
      //std::cout << __func__ << std::endl;
    }
  }
}

void KobukiNode::publishCurrentData()
{
  if (ros::ok())
  {
    if (current_sensor_publisher.getNumSubscribers() > 0)
    {
      Current::Data data;
      kobuki.getCurrentData(data);
      kobuki_comms::Current ros_data;
      ros_data.header.stamp = ros::Time::now();
      ros_data.current = data.current;
      current_sensor_publisher.publish(ros_data);
      //std::cout << __func__ << std::endl;
    }
  }
}

void KobukiNode::publishGpInputData()
{
  if (ros::ok())
  {
    if (gp_input_data_publisher.getNumSubscribers() > 0)
    {
      GpInput::Data data;
      kobuki.getGpInputData(data);
      kobuki_comms::GpInput ros_data;
      ros_data.header.stamp = ros::Time::now();
      ros_data.gp_input = data.gp_input;
      gp_input_data_publisher.publish(ros_data);
    }
  }
}

/*****************************************************************************
** Non Default Stream Packets
*****************************************************************************/
/**
 * @brief Publish fw, hw, sw version information.
 *
 * The driver will only gather this data when initialising so it is
 * important that this publisher is latched.
 */
void KobukiNode::publishVersionInfo(const VersionInfo &version_info)
{
  if (ros::ok())
  {
    kobuki_comms::VersionInfo msg;
    msg.firmware = version_info.firmware;
    msg.hardware = version_info.hardware;
    msg.software = version_info.software;
    version_info_publisher.publish(msg);
  }
}

/*****************************************************************************
** Events
*****************************************************************************/

/**
 * @brief Publish button events.
 */
void KobukiNode::publishButtonEvent(const ButtonEvent &event)
{
  if (ros::ok())
  {
    kobuki_comms::ButtonEvent msg;
    switch(event.state) {
      case(ButtonEvent::Pressed) : { msg.state = kobuki_comms::ButtonEvent::Pressed; break; }
      case(ButtonEvent::Released) : { msg.state = kobuki_comms::ButtonEvent::Released; break; }
      default: break;
    }
    switch(event.button) {
      case(ButtonEvent::F0) : { msg.button = kobuki_comms::ButtonEvent::F0; break; }
      case(ButtonEvent::F1) : { msg.button = kobuki_comms::ButtonEvent::F1; break; }
      case(ButtonEvent::F2) : { msg.button = kobuki_comms::ButtonEvent::F2; break; }
      default: break;
    }
    button_event_publisher.publish(msg);
  }
}

} // namespace kobuki
