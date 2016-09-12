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
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "kobuki_node/kobuki_ros.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

void KobukiRos::processStreamData() {
  publishWheelState();
  publishSensorState();
  publishDockIRData();
  publishInertia();
  publishRawInertia();
}

/*****************************************************************************
** Publish Sensor Stream Workers
*****************************************************************************/

void KobukiRos::publishSensorState()
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
  }
}

void KobukiRos::publishWheelState()
{
  // Take latest encoders and gyro data
  ecl::LegacyPose2D<double> pose_update;
  ecl::linear_algebra::Vector3d pose_update_rates;
  kobuki.updateOdometry(pose_update, pose_update_rates);
  kobuki.getWheelJointStates(joint_states.position[0], joint_states.velocity[0],   // left wheel
                             joint_states.position[1], joint_states.velocity[1]);  // right wheel

  // Update and publish odometry and joint states
  odometry.update(pose_update, pose_update_rates, kobuki.getHeading(), kobuki.getAngularVelocity());

  if (ros::ok())
  {
    joint_states.header.stamp = ros::Time::now();
    joint_state_publisher.publish(joint_states);
  }
}

void KobukiRos::publishInertia()
{
  if (ros::ok())
  {
    if (imu_data_publisher.getNumSubscribers() > 0)
    {
      // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
      sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);

      msg->header.frame_id = "gyro_link";
      msg->header.stamp = ros::Time::now();

      msg->orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, kobuki.getHeading());

      // set a non-zero covariance on unused dimensions (pitch and roll); this is a requirement of robot_pose_ekf
      // set yaw covariance as very low, to make it dominate over the odometry heading when combined
      // 1: fill once, as its always the same;  2: using an invented value; cannot we get a realistic estimation?
      msg->orientation_covariance[0] = DBL_MAX;
      msg->orientation_covariance[4] = DBL_MAX;
      msg->orientation_covariance[8] = 0.05;

      // fill angular velocity; we ignore acceleration for now
      msg->angular_velocity.z = kobuki.getAngularVelocity();

      // angular velocity covariance; useless by now, but robot_pose_ekf's
      // roadmap claims that it will compute velocities in the future
      msg->angular_velocity_covariance[0] = DBL_MAX;
      msg->angular_velocity_covariance[4] = DBL_MAX;
      msg->angular_velocity_covariance[8] = 0.05;

      imu_data_publisher.publish(msg);
    }
  }
}

void KobukiRos::publishRawInertia()
{
  if ( ros::ok() && (raw_imu_data_publisher.getNumSubscribers() > 0) )
  {
    // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
    sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);
    ThreeAxisGyro::Data data = kobuki.getRawInertiaData();

    ros::Time now = ros::Time::now();
    ros::Duration interval(0.01); // Time interval between each sensor reading.
    const double digit_to_dps = 0.00875; // digit to deg/s ratio, comes from datasheet of 3d gyro[L3G4200D].
    unsigned int length = data.followed_data_length/3;
    for( unsigned int i=0; i<length; i++) {
      // Each sensor reading has id, that circulate 0 to 255.
      //msg->header.frame_id = std::string("gyro_link_" + boost::lexical_cast<std::string>((unsigned int)data.frame_id+i));
      msg->header.frame_id = "gyro_link";

      // Update rate of 3d gyro sensor is 100 Hz, but robot's update rate is 50 Hz.
      // So, here is some compensation.
      // See also https://github.com/yujinrobot/kobuki/issues/216
      msg->header.stamp = now - interval * (length-i-1);

      // Sensing axis of 3d gyro is not match with robot. It is rotated 90 degree counterclockwise about z-axis.
      msg->angular_velocity.x = angles::from_degrees( -digit_to_dps * (short)data.data[i*3+1] );
      msg->angular_velocity.y = angles::from_degrees(  digit_to_dps * (short)data.data[i*3+0] );
      msg->angular_velocity.z = angles::from_degrees(  digit_to_dps * (short)data.data[i*3+2] );

      raw_imu_data_publisher.publish(msg);
    }
  }
}

void KobukiRos::publishDockIRData()
{
  if (ros::ok())
  {
    if (dock_ir_publisher.getNumSubscribers() > 0)
    {
      DockIR::Data data = kobuki.getDockIRData();

      // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
      kobuki_msgs::DockInfraRedPtr msg(new kobuki_msgs::DockInfraRed);

      msg->header.frame_id = "dock_ir_link";
      msg->header.stamp = ros::Time::now();

      msg->data.push_back( data.docking[0] );
      msg->data.push_back( data.docking[1] );
      msg->data.push_back( data.docking[2] );

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
void KobukiRos::publishVersionInfo(const VersionInfo &version_info)
{
  if (ros::ok())
  {
    kobuki_msgs::VersionInfoPtr msg(new kobuki_msgs::VersionInfo);

    msg->firmware = VersionInfo::toString(version_info.firmware);
    msg->hardware = VersionInfo::toString(version_info.hardware);
    msg->software = VersionInfo::getSoftwareVersion();

    msg->udid.resize(3);
    msg->udid[0] = version_info.udid0;
    msg->udid[1] = version_info.udid1;
    msg->udid[2] = version_info.udid2;

    // Set available features mask depending on firmware and driver versions
    if (version_info.firmware > 65536)  // 1.0.0
    {
      msg->features |= kobuki_msgs::VersionInfo::SMOOTH_MOVE_START;
      msg->features |= kobuki_msgs::VersionInfo::GYROSCOPE_3D_DATA;
    }
    if (version_info.firmware > 65792)  // 1.1.0
    {
      // msg->features |= kobuki_msgs::VersionInfo::SOMETHING_JINCHA_FANCY;
    }
    // if (msg->firmware > ...

    version_info_publisher.publish(msg);
  }
}

void KobukiRos::publishControllerInfo()
{
  if (ros::ok())
  {
    kobuki_msgs::ControllerInfoPtr msg(new kobuki_msgs::ControllerInfo);
    ControllerInfo::Data data = kobuki.getControllerInfoData();

    msg->type = data.type;
    msg->p_gain = static_cast<float>(data.p_gain) * 0.001f;;
    msg->i_gain = static_cast<float>(data.i_gain) * 0.001f;;
    msg->d_gain = static_cast<float>(data.d_gain) * 0.001f;;

    controller_info_publisher.publish(msg);
  }
}

/*****************************************************************************
** Events
*****************************************************************************/

void KobukiRos::publishButtonEvent(const ButtonEvent &event)
{
  if (ros::ok())
  {
    kobuki_msgs::ButtonEventPtr msg(new kobuki_msgs::ButtonEvent);
    switch(event.state) {
      case(ButtonEvent::Pressed)  : { msg->state = kobuki_msgs::ButtonEvent::PRESSED;  break; }
      case(ButtonEvent::Released) : { msg->state = kobuki_msgs::ButtonEvent::RELEASED; break; }
      default: break;
    }
    switch(event.button) {
      case(ButtonEvent::Button0) : { msg->button = kobuki_msgs::ButtonEvent::Button0; break; }
      case(ButtonEvent::Button1) : { msg->button = kobuki_msgs::ButtonEvent::Button1; break; }
      case(ButtonEvent::Button2) : { msg->button = kobuki_msgs::ButtonEvent::Button2; break; }
      default: break;
    }
    button_event_publisher.publish(msg);
  }
}

void KobukiRos::publishBumperEvent(const BumperEvent &event)
{
  if (ros::ok())
  {
    kobuki_msgs::BumperEventPtr msg(new kobuki_msgs::BumperEvent);
    switch(event.state) {
      case(BumperEvent::Pressed)  : { msg->state = kobuki_msgs::BumperEvent::PRESSED;  break; }
      case(BumperEvent::Released) : { msg->state = kobuki_msgs::BumperEvent::RELEASED; break; }
      default: break;
    }
    switch(event.bumper) {
      case(BumperEvent::Left)   : { msg->bumper = kobuki_msgs::BumperEvent::LEFT;   break; }
      case(BumperEvent::Center) : { msg->bumper = kobuki_msgs::BumperEvent::CENTER; break; }
      case(BumperEvent::Right)  : { msg->bumper = kobuki_msgs::BumperEvent::RIGHT;  break; }
      default: break;
    }
    bumper_event_publisher.publish(msg);
  }
}

void KobukiRos::publishCliffEvent(const CliffEvent &event)
{
  if (ros::ok())
  {
    kobuki_msgs::CliffEventPtr msg(new kobuki_msgs::CliffEvent);
    switch(event.state) {
      case(CliffEvent::Floor) : { msg->state = kobuki_msgs::CliffEvent::FLOOR; break; }
      case(CliffEvent::Cliff) : { msg->state = kobuki_msgs::CliffEvent::CLIFF; break; }
      default: break;
    }
    switch(event.sensor) {
      case(CliffEvent::Left)   : { msg->sensor = kobuki_msgs::CliffEvent::LEFT;   break; }
      case(CliffEvent::Center) : { msg->sensor = kobuki_msgs::CliffEvent::CENTER; break; }
      case(CliffEvent::Right)  : { msg->sensor = kobuki_msgs::CliffEvent::RIGHT;  break; }
      default: break;
    }
    msg->bottom = event.bottom;
    cliff_event_publisher.publish(msg);
  }
}

void KobukiRos::publishWheelEvent(const WheelEvent &event)
{
  if (ros::ok())
  {
    kobuki_msgs::WheelDropEventPtr msg(new kobuki_msgs::WheelDropEvent);
    switch(event.state) {
      case(WheelEvent::Dropped) : { msg->state = kobuki_msgs::WheelDropEvent::DROPPED; break; }
      case(WheelEvent::Raised)  : { msg->state = kobuki_msgs::WheelDropEvent::RAISED;  break; }
      default: break;
    }
    switch(event.wheel) {
      case(WheelEvent::Left)  : { msg->wheel = kobuki_msgs::WheelDropEvent::LEFT;  break; }
      case(WheelEvent::Right) : { msg->wheel = kobuki_msgs::WheelDropEvent::RIGHT; break; }
      default: break;
    }
    wheel_event_publisher.publish(msg);
  }
}

void KobukiRos::publishPowerEvent(const PowerEvent &event)
{
  if (ros::ok())
  {
    kobuki_msgs::PowerSystemEventPtr msg(new kobuki_msgs::PowerSystemEvent);
    switch(event.event) {
      case(PowerEvent::Unplugged) :
        { msg->event = kobuki_msgs::PowerSystemEvent::UNPLUGGED; break; }
      case(PowerEvent::PluggedToAdapter) :
        { msg->event = kobuki_msgs::PowerSystemEvent::PLUGGED_TO_ADAPTER;  break; }
      case(PowerEvent::PluggedToDockbase) :
        { msg->event = kobuki_msgs::PowerSystemEvent::PLUGGED_TO_DOCKBASE; break; }
      case(PowerEvent::ChargeCompleted)  :
        { msg->event = kobuki_msgs::PowerSystemEvent::CHARGE_COMPLETED;  break; }
      case(PowerEvent::BatteryLow) :
        { msg->event = kobuki_msgs::PowerSystemEvent::BATTERY_LOW; break; }
      case(PowerEvent::BatteryCritical) :
        { msg->event = kobuki_msgs::PowerSystemEvent::BATTERY_CRITICAL;  break; }
      default: break;
    }
    power_event_publisher.publish(msg);
  }
}

void KobukiRos::publishInputEvent(const InputEvent &event)
{
  if (ros::ok())
  {
    kobuki_msgs::DigitalInputEventPtr msg(new kobuki_msgs::DigitalInputEvent);
    for (unsigned int i = 0; i < msg->values.size(); i++)
      msg->values[i] = event.values[i];
    input_event_publisher.publish(msg);
  }
}

void KobukiRos::publishRobotEvent(const RobotEvent &event)
{
  if (ros::ok())
  {
    kobuki_msgs::RobotStateEventPtr msg(new kobuki_msgs::RobotStateEvent);
    switch(event.state) {
      case(RobotEvent::Online)  : { msg->state = kobuki_msgs::RobotStateEvent::ONLINE;  break; }
      case(RobotEvent::Offline) : { msg->state = kobuki_msgs::RobotStateEvent::OFFLINE; break; }
      default: break;
    }

    robot_event_publisher.publish(msg);
  }
}

/**
 * @brief Prints the raw data stream to a publisher.
 *
 * This is a lazy publisher, it only publishes if someone is listening. It publishes the
 * hex byte values of the raw data commands. Useful for debugging command to protocol
 * byte packets to the firmware.
 *
 * The signal which calls this
 * function is sending a copy of the buffer (don't worry about mutexes). Be ideal if we used
 * const PacketFinder::BufferType here, but haven't updated PushPop to work with consts yet.
 *
 * @param buffer
 */
void KobukiRos::publishRawDataCommand(Command::Buffer &buffer)
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
/**
 * @brief Prints the raw data stream to a publisher.
 *
 * This is a lazy publisher, it only publishes if someone is listening. It publishes the
 * hex byte values of the raw data (incoming) stream. Useful for checking when bytes get
 * mangled.
 *
 * The signal which calls this
 * function is sending a copy of the buffer (don't worry about mutexes). Be ideal if we used
 * const PacketFinder::BufferType here, but haven't updated PushPop to work with consts yet.
 *
 * @param buffer
 */
void KobukiRos::publishRawDataStream(PacketFinder::BufferType &buffer)
{
  if ( raw_data_stream_publisher.getNumSubscribers() > 0 ) { // do not do string processing if there is no-one listening.
    /*std::cout << "size: [" << buffer.size() << "], asize: [" << buffer.asize() << "]" << std::endl;
    std::cout << "leader: " << buffer.leader << ", follower: " << buffer.follower  << std::endl;
    {
      std::ostringstream ostream;
      PacketFinder::BufferType::Formatter format;
      ostream << format(buffer); // convert to an easily readable hex string.
      //std::cout << ostream.str() << std::endl;
      std_msgs::String s;
      s.data = ostream.str();
      if (ros::ok())
      {
        raw_data_stream_publisher.publish(s);
      }
    }*/
    {
      std::ostringstream ostream;
      ostream << "{ " ;
      ostream << std::setfill('0') << std::uppercase;
      for (unsigned int i=0; i < buffer.size(); i++)
          ostream << std::hex << std::setw(2) << static_cast<unsigned int>(buffer[i]) << " " << std::dec;
      ostream << "}";
      //std::cout << ostream.str() << std::endl;
      std_msgs::StringPtr msg(new std_msgs::String);
      msg->data = ostream.str();
      if (ros::ok())
      {
        raw_data_stream_publisher.publish(msg);
      }
    }
  }
}

void KobukiRos::publishRawControlCommand(const std::vector<short> &velocity_commands)
{
  if ( raw_control_command_publisher.getNumSubscribers() > 0 ) {
    std_msgs::Int16MultiArrayPtr msg(new std_msgs::Int16MultiArray);
    msg->data = velocity_commands;
    if (ros::ok())
    {
      raw_control_command_publisher.publish(msg);
    }
  }
  return;
}

} // namespace kobuki
