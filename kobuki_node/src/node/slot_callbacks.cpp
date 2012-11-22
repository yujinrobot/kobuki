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

#include "kobuki_node/kobuki_node.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

void KobukiNode::processStreamData() {
  publishWheelState();
  publishSensorState();
  publishDockIRData();
  publishInertia();
}

/*****************************************************************************
** Publish Sensor Stream Workers
*****************************************************************************/

void KobukiNode::publishSensorState()
{
  if ( ros::ok() ) {
    if (sensor_state_publisher.getNumSubscribers() > 0) {
      kobuki_msgs::SensorState state;
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
      state.over_current = data.over_current;

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

    if (bumper_as_pc_publisher.getNumSubscribers() > 0) {
      uint8_t bumper = kobuki.getCoreSensorData().bumper;
      bumper_pc.header.stamp = ros::Time::now();

      // Republish bumper readings as pointcloud so navistack can use them for poor-man navigation
      bumper_pc[1].x = (bumper & CoreSensors::Flags::CenterBumper) ? +  bumper_pc_radius : FLT_MAX;

      bumper_pc[0].x = (bumper & CoreSensors::Flags::LeftBumper)   ? + side_bump_x_coord : FLT_MAX;
      bumper_pc[2].x = (bumper & CoreSensors::Flags::RightBumper)  ? + side_bump_x_coord : FLT_MAX;

      bumper_pc[0].y = (bumper & CoreSensors::Flags::LeftBumper)   ? + side_bump_y_coord : FLT_MAX;
      bumper_pc[2].y = (bumper & CoreSensors::Flags::RightBumper)  ? - side_bump_y_coord : FLT_MAX;

      bumper_as_pc_publisher.publish(bumper_pc);
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

void KobukiNode::publishDockIRData()
{
  if (ros::ok())
  {
    if (dock_ir_publisher.getNumSubscribers() > 0)
    {
      kobuki_msgs::DockInfraRed msg;
      DockIR::Data data = kobuki.getDockIRData();
      msg.header.frame_id = "dock_ir_link";
      msg.header.stamp = ros::Time::now();

      msg.data.push_back( data.docking[0] );
      msg.data.push_back( data.docking[1] );
      msg.data.push_back( data.docking[2] );

      dock_ir_publisher.publish(msg);
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
    kobuki_msgs::VersionInfo msg;
    msg.firmware = version_info.firmware;
    msg.hardware = version_info.hardware;
    msg.software = version_info.software;

    msg.udid.resize(3);
    msg.udid[0] = version_info.udid0;
    msg.udid[1] = version_info.udid1;
    msg.udid[2] = version_info.udid2;

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
    kobuki_msgs::ButtonEvent msg;
    switch(event.state) {
      case(ButtonEvent::Pressed) : { msg.state = kobuki_msgs::ButtonEvent::PRESSED; break; }
      case(ButtonEvent::Released) : { msg.state = kobuki_msgs::ButtonEvent::RELEASED; break; }
      default: break;
    }
    switch(event.button) {
      case(ButtonEvent::Button0) : { msg.button = kobuki_msgs::ButtonEvent::Button0; break; }
      case(ButtonEvent::Button1) : { msg.button = kobuki_msgs::ButtonEvent::Button1; break; }
      case(ButtonEvent::Button2) : { msg.button = kobuki_msgs::ButtonEvent::Button2; break; }
      default: break;
    }
    button_event_publisher.publish(msg);
  }
}

void KobukiNode::publishBumperEvent(const BumperEvent &event)
{
  if (ros::ok())
  {
    kobuki_msgs::BumperEvent msg;
    switch(event.state) {
      case(BumperEvent::Pressed) : { msg.state = kobuki_msgs::BumperEvent::PRESSED; break; }
      case(BumperEvent::Released) : { msg.state = kobuki_msgs::BumperEvent::RELEASED; break; }
      default: break;
    }
    switch(event.bumper) {
      case(BumperEvent::Left) : { msg.bumper = kobuki_msgs::BumperEvent::LEFT; break; }
      case(BumperEvent::Center) : { msg.bumper = kobuki_msgs::BumperEvent::CENTER; break; }
      case(BumperEvent::Right) : { msg.bumper = kobuki_msgs::BumperEvent::RIGHT; break; }
      default: break;
    }
    bumper_event_publisher.publish(msg);
  }
}

void KobukiNode::publishCliffEvent(const CliffEvent &event)
{
  if (ros::ok())
  {
    kobuki_msgs::CliffEvent msg;
    switch(event.state) {
      case(CliffEvent::Floor) : { msg.state = kobuki_msgs::CliffEvent::FLOOR; break; }
      case(CliffEvent::Cliff) : { msg.state = kobuki_msgs::CliffEvent::CLIFF; break; }
      default: break;
    }
    switch(event.sensor) {
      case(CliffEvent::Left)   : { msg.sensor = kobuki_msgs::CliffEvent::LEFT;   break; }
      case(CliffEvent::Center) : { msg.sensor = kobuki_msgs::CliffEvent::CENTER; break; }
      case(CliffEvent::Right)  : { msg.sensor = kobuki_msgs::CliffEvent::RIGHT;  break; }
      default: break;
    }
    msg.bottom = event.bottom;
    cliff_event_publisher.publish(msg);
  }
}

void KobukiNode::publishWheelEvent(const WheelEvent &event)
{
  if (ros::ok())
  {
    kobuki_msgs::WheelDropEvent msg;
    switch(event.state) {
      case(WheelEvent::Dropped) : { msg.state = kobuki_msgs::WheelDropEvent::DROPPED; break; }
      case(WheelEvent::Raised)  : { msg.state = kobuki_msgs::WheelDropEvent::RAISED;  break; }
      default: break;
    }
    switch(event.wheel) {
      case(WheelEvent::Left)  : { msg.wheel = kobuki_msgs::WheelDropEvent::LEFT;  break; }
      case(WheelEvent::Right) : { msg.wheel = kobuki_msgs::WheelDropEvent::RIGHT; break; }
      default: break;
    }
    wheel_event_publisher.publish(msg);
  }
}

void KobukiNode::publishPowerEvent(const PowerEvent &event)
{
  if (ros::ok())
  {
    kobuki_msgs::PowerSystemEvent msg;
    switch(event.event) {
      case(PowerEvent::Unplugged) :
        { msg.event = kobuki_msgs::PowerSystemEvent::UNPLUGGED; break; }
      case(PowerEvent::PluggedToAdapter) :
        { msg.event = kobuki_msgs::PowerSystemEvent::PLUGGED_TO_ADAPTER;  break; }
      case(PowerEvent::PluggedToDockbase) :
        { msg.event = kobuki_msgs::PowerSystemEvent::PLUGGED_TO_DOCKBASE; break; }
      case(PowerEvent::ChargeCompleted)  :
        { msg.event = kobuki_msgs::PowerSystemEvent::CHARGE_COMPLETED;  break; }
      case(PowerEvent::BatteryLow) :
        { msg.event = kobuki_msgs::PowerSystemEvent::BATTERY_LOW; break; }
      case(PowerEvent::BatteryCritical) :
        { msg.event = kobuki_msgs::PowerSystemEvent::BATTERY_CRITICAL;  break; }
      default: break;
    }
    power_event_publisher.publish(msg);
  }
}

void KobukiNode::publishInputEvent(const InputEvent &event)
{
  if (ros::ok())
  {
    kobuki_msgs::DigitalInputEvent msg;
    for (unsigned int i = 0; i < msg.values.size(); i++)
      msg.values[i] = event.values[i];
    input_event_publisher.publish(msg);
  }
}

void KobukiNode::publishRobotEvent(const RobotEvent &event)
{
  if (ros::ok())
  {
    kobuki_msgs::RobotStateEvent msg;
    switch(event.state) {
      case(RobotEvent::Online)  : { msg.state = kobuki_msgs::RobotStateEvent::ONLINE;  break; }
      case(RobotEvent::Offline) : { msg.state = kobuki_msgs::RobotStateEvent::OFFLINE; break; }
      default: break;
    }

    robot_event_publisher.publish(msg);
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

void KobukiNode::publishRawDataStream(PacketFinder::BufferType &buffer)
{
  if ( raw_data_stream_publisher.getNumSubscribers() > 0 ) { // do not do string processing if there is no-one listening.
    std::ostringstream ostream;
    ostream << "[ " ;
    ostream << std::setfill('0') << std::uppercase;
    for (unsigned int i=0; i < buffer.size(); i++)
        ostream << std::hex << std::setw(2) << static_cast<unsigned int>(buffer[i]) << " " << std::dec;
    ostream << "]";
    std_msgs::String s;
    s.data = ostream.str();
    if (ros::ok())
    {
      raw_data_stream_publisher.publish(s);
    }
  }
}

} // namespace kobuki
