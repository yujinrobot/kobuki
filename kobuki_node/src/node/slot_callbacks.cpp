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

void KobukiNode::publishCoreSensorData()
{
  if (ros::ok())
  {
    CoreSensors::Data data;
    kobuki.getCoreSensorData(data);

    if (core_sensor_data_publisher.getNumSubscribers() > 0)
    {
      // convert data format
      kobuki_comms::CoreSensors ros_data;
      ros_data.header0 = data.header_id;
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

    if (button_events_publisher.getNumSubscribers() > 0)
    {
      if (data.buttons != buttons_state)
      {
        kobuki_comms::Buttons buttons;
        // g++ complains if you use tertiary operators here - spitting the dummy because of uint8's? wierd.
        if ( data.buttons & kobuki_comms::CoreSensors::F0) {
          buttons.values.push_back(kobuki_comms::Buttons::PRESSED);
        } else {
          buttons.values.push_back(kobuki_comms::Buttons::RELEASED);
        }
        if ( data.buttons & kobuki_comms::CoreSensors::F1) {
          buttons.values.push_back(kobuki_comms::Buttons::PRESSED);
        } else {
          buttons.values.push_back(kobuki_comms::Buttons::RELEASED);
        }
        if ( data.buttons & kobuki_comms::CoreSensors::F2) {
          buttons.values.push_back(kobuki_comms::Buttons::PRESSED);
        } else {
          buttons.values.push_back(kobuki_comms::Buttons::RELEASED);
        }
        button_events_publisher.publish(buttons);

        buttons_state = data.buttons;
      }
    }
  }
}

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

void KobukiNode::publishInertiaData()
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

      // ignore velocity and acceleration for now - kobuki driver can give us rates though.
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
      CliffData::Data data;
      kobuki.getCliffData(data);
      kobuki_comms::Cliff ros_data;
      ros_data.header.stamp = ros::Time::now();
      ros_data.header_id = data.header_id;
      ros_data.bottom = data.bottom;
      cliff_data_publisher.publish(ros_data);
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
      CurrentData::Data data;
      kobuki.getCurrentData(data);
      kobuki_comms::Current ros_data;
      ros_data.header.stamp = ros::Time::now();
      ros_data.header_id = data.header_id;
      ros_data.current = data.current;
      current_data_publisher.publish(ros_data);
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
      HWData::Data data;
      kobuki.getHWData(data);
      kobuki_comms::HW ros_data;
      ros_data.header.stamp = ros::Time::now();
      ros_data.header_id = data.header_id;
      ros_data.mainboard_version = data.mainboard_version;
      hw_data_publisher.publish(ros_data);
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
      FWData::Data data;
      kobuki.getFWData(data);
      // convert data format
      kobuki_comms::FW ros_data;
      ros_data.header.stamp = ros::Time::now();
      ros_data.header_id = data.header_id;
      ros_data.fw_version = data.fw_version;
      fw_data_publisher.publish(ros_data);
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
    kobuki.setBaseControlCommand(msg->linear.x, msg->angular.z);

    last_cmd_time = ros::Time::now();
  }
  return;
}

void KobukiNode::subscribeLedCommand(const kobuki_comms::LedArrayConstPtr msg)
{
  if ( msg->values.size() != 2 ) {
    ROS_WARN_STREAM("Kobuki : led commands must specify values for both led's in the array.");
    return;
  }
  for ( unsigned int i = 0; i < msg->values.size(); ++i ) {
    LedNumber led = Led1;
    if ( i == 1 ) { led = Led2; }
    if ( msg->values[i] == kobuki_comms::LedArray::GREEN ) {
      kobuki.toggleLed(led, Green);
    } else if ( msg->values[i] == kobuki_comms::LedArray::ORANGE ) {
      kobuki.toggleLed(led, Orange);
    } else if ( msg->values[i] == kobuki_comms::LedArray::RED ) {
      kobuki.toggleLed(led, Red);
    } else {
      kobuki.toggleLed(led, Black);
    }
  }
  return;
}

} // namespace kobuki
