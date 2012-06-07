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

#include <std_msgs/String.h>
#include <kobuki_comms/VersionInfo.h>
#include <kobuki_driver/packets/gp_input.hpp>
#include "kobuki_node/kobuki_node.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

void KobukiNode::processStreamData() {
  publishWheelState();
  publishSensorState();
  publishInertia();
}

/*****************************************************************************
** Publish Sensor Stream Workers
*****************************************************************************/

void KobukiNode::publishSensorState()
{
  if ( ros::ok() ) {
    if (sensor_state_publisher.getNumSubscribers() > 0) {
      kobuki_comms::SensorState state;
      CoreSensors::Data data = kobuki.getCoreSensorData();
      state.header.stamp = ros::Time::now();
      state.time_stamp = data.time_stamp; // firmware time stamp
      state.bumper = data.bumper;
      state.wheel_drop = data.wheel_drop;
      state.cliff = data.cliff;
      state.left_encoder = data.left_encoder;
      state.right_encoder = data.right_encoder;
      state.left_pwm = data.left_pwm;
      state.right_pwm = data.right_pwm;
      state.buttons = data.buttons;
      state.charger = data.charger;
      state.battery = data.battery;

      Cliff::Data cliff_data = kobuki.getCliffData();
      state.bottom = cliff_data.bottom;

      Current::Data current_data = kobuki.getCurrentData();
      state.current = current_data.current;

      GpInput::Data gp_input_data = kobuki.getGpInputData();
      state.digital_input = gp_input_data.digital_input;
      for ( unsigned int i = 0; i < gp_input_data.analog_input.size(); ++i ) {
        state.analog_input.push_back(gp_input_data.analog_input[i]);
      }

      sensor_state_publisher.publish(state);
    }
  }
}

void KobukiNode::publishWheelState()
{
  ecl::Pose2D<double> pose_update;
  ecl::linear_algebra::Vector3d pose_update_rates;
  kobuki.updateOdometry(pose_update, pose_update_rates);
  kobuki.getWheelJointStates(joint_states.position[0],joint_states.velocity[0],   // left wheel
                             joint_states.position[1],joint_states.velocity[1] ); // right wheel

  odometry.update(pose_update, pose_update_rates);

  if (ros::ok())
  {
    joint_states.header.stamp = ros::Time::now();
    joint_state_publisher.publish(joint_states);
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

void KobukiNode::publishButtonEvent(const ButtonEvent &event)
{
  if (ros::ok())
  {
    kobuki_comms::ButtonEvent msg;
    switch(event.state) {
      case(ButtonEvent::Pressed) : { msg.state = kobuki_comms::ButtonEvent::PRESSED; break; }
      case(ButtonEvent::Released) : { msg.state = kobuki_comms::ButtonEvent::RELEASED; break; }
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

void KobukiNode::publishBumperEvent(const BumperEvent &event)
{
  if (ros::ok())
  {
    kobuki_comms::BumperEvent msg;
    switch(event.state) {
      case(BumperEvent::Pressed) : { msg.state = kobuki_comms::BumperEvent::PRESSED; break; }
      case(BumperEvent::Released) : { msg.state = kobuki_comms::BumperEvent::RELEASED; break; }
      default: break;
    }
    switch(event.bumper) {
      case(BumperEvent::Left) : { msg.bumper = kobuki_comms::BumperEvent::LEFT; break; }
      case(BumperEvent::Centre) : { msg.bumper = kobuki_comms::BumperEvent::CENTRE; break; }
      case(BumperEvent::Right) : { msg.bumper = kobuki_comms::BumperEvent::RIGHT; break; }
      default: break;
    }
    bumper_event_publisher.publish(msg);
  }
}

void KobukiNode::publishWheelDropEvent(const WheelDropEvent &event)
{
  if (ros::ok())
  {
    kobuki_comms::WheelDropEvent msg;
    switch(event.state) {
      case(WheelDropEvent::Raised) : { msg.state = kobuki_comms::WheelDropEvent::RAISED; break; }
      case(WheelDropEvent::Dropped) : { msg.state = kobuki_comms::WheelDropEvent::DROPPED; break; }
      default: break;
    }
    switch(event.wheel_drop) {
      case(WheelDropEvent::Left) : { msg.wheel = kobuki_comms::WheelDropEvent::LEFT; break; }
      case(WheelDropEvent::Right) : { msg.wheel = kobuki_comms::WheelDropEvent::RIGHT; break; }
      default: break;
    }
    wheel_drop_event_publisher.publish(msg);
  }
}

void KobukiNode::publishCliffEvent(const CliffEvent &event)
{
  if (ros::ok())
  {
    kobuki_comms::CliffEvent msg;
    switch(event.state) {
      case(CliffEvent::Cliff) : { msg.state = kobuki_comms::CliffEvent::CLIFF; break; }
      case(CliffEvent::Floor) : { msg.state = kobuki_comms::CliffEvent::FLOOR; break; }
      default: break;
    }
    switch(event.cliff) {
      case(CliffEvent::Left) : { msg.cliff = kobuki_comms::CliffEvent::LEFT; break; }
      case(CliffEvent::Centre) : { msg.cliff = kobuki_comms::CliffEvent::CENTRE; break; }
      case(CliffEvent::Right) : { msg.cliff = kobuki_comms::CliffEvent::RIGHT; break; }
      default: break;
    }
    cliff_event_publisher.publish(msg);
  }
}

void KobukiNode::publishRawDataCommand(Command::Buffer &buffer)
{
  if ( raw_data_command_publisher.getNumSubscribers() > 0 ) { // do not do string processing if there is no-one listening.
    std::ostringstream ostream;
    Command::Buffer::Formatter format;
    ostream << format(buffer); // convert to an easily readable hex string.
    std_msgs::String s;
    s.data = ostream.str();
    if (ros::ok())
    {
      raw_data_command_publisher.publish(s);
    }
  }
}

} // namespace kobuki
