/**
 * @file /kobuki_node/include/kobuki_node/sigslots.cpp
 *
 * @brief sigslot gathering
 *
 * Contains all SigSlots for the kobuki_node
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

void KobukiNode::publishWheelState()
{
  if (ros::ok())
  {

    // TODO really horrible; refactor
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

void KobukiNode::publishSensorData()
{
  if (ros::ok())
  {
    if (sensor_data_publisher.getNumSubscribers() > 0)
    {
      kobuki_comms::SensorData data;
      kobuki.getSensorData(data);
      data.header.stamp = ros::Time::now();
      sensor_data_publisher.publish(data);
      //std::cout << "publishSensorData()" << std::endl;
    }
  }
}

#if 0
void KobukiNode::publishRawDataSent( const Packet::BufferStencil &bytes )
{

  if ( ros::ok() )
  {
    if ( raw_data_sent_publisher.getNumSubscribers() > 0 )
    {
      ecl::Format<Packet::Buffer> format;
      ecl::StringStream sstream;
      sstream << format(bytes);
      std_msgs::String msg;
      msg.data = sstream.str();
      raw_data_sent_publisher.publish(msg);
    }
  }
}
#endif

#if 0
void KobukiNode::publishInvalidPacket( const Packet::BufferStencil &bytes )
{

  if ( ros::ok() )
  {
    if ( invalid_packet_publisher.getNumSubscribers() > 0 )
    {
      ecl::Format<Packet::Buffer> format;
      ecl::StringStream sstream;
      sstream << format(bytes);
      std_msgs::String msg;
      msg.data = sstream.str();
      invalid_packet_publisher.publish(msg);
    }
  }
}
#endif

void KobukiNode::publishIRData()
{
  if (ros::ok())
  {
    if (ir_data_publisher.getNumSubscribers() > 0)
    {
      kobuki_comms::IR data;
      kobuki.getIRData(data);
      data.header.stamp = ros::Time::now();
      ir_data_publisher.publish(data);
      //std::cout << __func__ << std::endl;
    }
  }
}

void KobukiNode::publishDockIRData()
{
  if (ros::ok())
  {
    if (sensor_data_publisher.getNumSubscribers() > 0)
    {
      kobuki_comms::SensorData data;
      kobuki.getSensorData(data);
      data.header.stamp = ros::Time::now();
      sensor_data_publisher.publish(data);
      //std::cout << __func__ << std::endl;
    }
  }
}

void KobukiNode::publishInertiaData()
{
  if (ros::ok())
  {
    // TODO This is useless by now... publish anyway
    if (inertia_data_publisher.getNumSubscribers() > 0)
    {
      kobuki_comms::Inertia data;
      kobuki.getInertiaData(data);
      data.header.stamp = ros::Time::now();
      inertia_data_publisher.publish(data);
      //std::cout << __func__ << std::endl;
    }

    // Convert internal kobuki_comms::Inertia data into a ros sensor_msgs::Imu message
    if (imu_data_publisher.getNumSubscribers() > 0)
    {
      kobuki_comms::Inertia data;
      kobuki.getInertiaData(data);

      sensor_msgs::Imu msg;
      msg.header.frame_id = "gyro_link";
      msg.header.seq = data.header.seq;
      msg.header.stamp = ros::Time::now();

      msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, data.angle);

      // set a very large covariance on unused dimensions (pitch and roll);
      // set yaw covariance as very low, to make it dominate over the odometry heading
      // TODO 1: fill once, as its always the same;  TODO 2: cannot get better estimation?
      msg.orientation_covariance[0] = DBL_MAX;
      msg.orientation_covariance[4] = DBL_MAX;
      msg.orientation_covariance[8] = 0.005;

      // ignore velocity and acceleration by now

      imu_data_publisher.publish(msg);
    }
  }
}

void KobukiNode::publishCliffData()
{
  if (ros::ok())
  {
    if (cliff_data_publisher.getNumSubscribers() > 0)
    {
      kobuki_comms::Cliff data;
      kobuki.getCliffData(data);
      data.header.stamp = ros::Time::now();
      cliff_data_publisher.publish(data);
      //std::cout << __func__ << std::endl;
    }
  }
}

void KobukiNode::publishCurrentData()
{
  if (ros::ok())
  {
    if (current_data_publisher.getNumSubscribers() > 0)
    {
      kobuki_comms::Current data;
      kobuki.getCurrentData(data);
      data.header.stamp = ros::Time::now();
      current_data_publisher.publish(data);
      //std::cout << __func__ << std::endl;
    }
  }
}

void KobukiNode::publishMagnetData()
{
  if (ros::ok())
  {
    if (magnet_data_publisher.getNumSubscribers() > 0)
    {
      kobuki_comms::Magnet data;
      kobuki.getMagnetData(data);
      data.header.stamp = ros::Time::now();
      magnet_data_publisher.publish(data);
      //std::cout << __func__ << std::endl;
    }
  }
}

void KobukiNode::publishHWData()
{
  if (ros::ok())
  {
    if (hw_data_publisher.getNumSubscribers() > 0)
    {
      kobuki_comms::HW data;
      kobuki.getHWData(data);
      data.header.stamp = ros::Time::now();
      hw_data_publisher.publish(data);
      //std::cout << __func__ << std::endl;
    }
  }
}

void KobukiNode::publishFWData()
{
  if (ros::ok())
  {
    if (fw_data_publisher.getNumSubscribers() > 0)
    {
      kobuki_comms::FW data;
      kobuki.getFWData(data);
      data.header.stamp = ros::Time::now();
      fw_data_publisher.publish(data);
      //std::cout << __func__ << std::endl;
    }
  }
}

void KobukiNode::publishTimeData()
{
  if (ros::ok())
  {
    if (time_data_publisher.getNumSubscribers() > 0)
    {
      kobuki_comms::Time data;
      kobuki.getTimeData(data);
      data.header.stamp = ros::Time::now();
      time_data_publisher.publish(data);
      //std::cout << __func__ << std::endl;
    }
  }
}

void KobukiNode::publishStGyroData()
{
  if (ros::ok())
  {
    if (st_gyro_data_publisher.getNumSubscribers() > 0)
    {
      kobuki_comms::StGyro data;
      kobuki.getStGyroData(data);
      data.header.stamp = ros::Time::now();
      st_gyro_data_publisher.publish(data);
      //std::cout << __func__ << std::endl;
    }
  }
}

void KobukiNode::publishEEPROMData()
{
  if (ros::ok())
  {
    if (eeprom_data_publisher.getNumSubscribers() > 0)
    {
      kobuki_comms::EEPROM data;
      kobuki.getEEPROMData(data);
      data.header.stamp = ros::Time::now();
      eeprom_data_publisher.publish(data);
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
      kobuki_comms::GpInput data;
      kobuki.getGpInputData(data);
      data.header.stamp = ros::Time::now();
      gp_input_data_publisher.publish(data);
      //std::cout << __func__ << std::endl;
    }
  }
}

void KobukiNode::subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg)
{
  if (kobuki.isEnabled())
  {
    // For now assuming this is in the robot frame, but probably this
    // should be global frame and require a transform
    //double vx = msg->linear.x;        // in (m/s)
    //double wz = msg->angular.z;       // in (rad/s)
    ROS_DEBUG_STREAM("subscribeVelocityCommand: [" << msg->linear.x << "],[" << msg->angular.z << "]");
    kobuki.setCommand(msg->linear.x, msg->angular.z);
  }
  return;
}

void KobukiNode::subscribeKobukiCommand(const kobuki_comms::CommandConstPtr msg)
{
  //if( kobuki.isEnabled() ) {
  kobuki.sendCommand(msg);
  //} else {
  //    ROS_WARN("Robot is not enabled.");
  //}
  return;
}

} // namespace kobuki
