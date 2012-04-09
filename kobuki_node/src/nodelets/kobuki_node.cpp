/**
 * @file /cruizcore/src/nodelets/cruizcore_nodelet.cpp
 *
 * @brief Ros nodelet for the cruizcore driver.
 *
 * @date 20/08/2010
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <float.h>

#include <tf/tf.h>

#include <pluginlib/class_list_macros.h>
#include <ecl/streams/string_stream.hpp>
#include "kobuki_node/kobuki_node.hpp"


/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
 ** Implementation [KobukiNodelet]
 *****************************************************************************/
/**
 * @brief Default constructor.
 *
 * Make sure you call the init() method to fully define this node.
 */
KobukiNodelet::KobukiNodelet() :
    wheel_left_name("wheel_left"),
    wheel_right_name("wheel_right"),
    slot_wheel_state(&KobukiNodelet::publishWheelState, *this),
    slot_sensor_data(&KobukiNodelet::publishSensorData,*this),
    slot_ir(&KobukiNodelet::publishIRData, *this),
    slot_dock_ir(&KobukiNodelet::publishDockIRData, *this),
    slot_inertia(&KobukiNodelet::publishInertiaData, *this),
    slot_cliff(&KobukiNodelet::publishCliffData, *this),
    slot_current(&KobukiNodelet::publishCurrentData, *this),
    slot_magnet(&KobukiNodelet::publishMagnetData, *this),
    slot_hw(&KobukiNodelet::publishHWData, *this),
    slot_fw(&KobukiNodelet::publishFWData, *this),
    slot_time(&KobukiNodelet::publishTimeData, *this),
    slot_st_gyro(&KobukiNodelet::publishStGyroData, *this),
    slot_eeprom(&KobukiNodelet::publishEEPROMData, *this),
    slot_gp_input(&KobukiNodelet::publishGpInputData, *this),
    slot_debug(&KobukiNodelet::rosDebug, *this),
    slot_info(&KobukiNodelet::rosInfo, *this),
    slot_warn(&KobukiNodelet::rosWarn, *this),
    slot_error(&KobukiNodelet::rosError, *this),
    odom_frame("odom"),
    base_frame("base_footprint"),
    publish_tf(false)
{
  joint_states.name.push_back("left_wheel_joint");
  joint_states.name.push_back("right_wheel_joint");
  joint_states.name.push_back("front_wheel_joint"); // front_castor_joint in create tbot
  joint_states.name.push_back("rear_wheel_joint");  // back_castor_joint in create tbot
  joint_states.position.resize(4,0.0);
  joint_states.velocity.resize(4,0.0);
  joint_states.effort.resize(4,0.0);
}
/**
 :* @brief Destructs, but only after the thread has cleanly terminated.
 *
 * Ensures we stay alive long enough for the thread to cleanly terminate.
 */
KobukiNodelet::~KobukiNodelet()
{
  this->shutdown_requested = true;
  ROS_INFO_STREAM("Kobuki : waiting for kobuki thread to finish [" << name << "].");
  kobuki.close();
  //kobuki.join();
}

/*****************************************************************************
 ** CruizCoreNodelet [DeviceNodelet Setup]
 *****************************************************************************/
/**
 * @brief Initialises the cruizcore from the ros parameter server.
 *
 * To set up the cruizcore node you need to roslaunch it along with a parameter
 * configuration. That configuration is done from the parameter server here.
 * If configuration fails, it shuts down the ros process node.
 *
 * @param nh : private nodehandle to use for initialisation.
 */
bool KobukiNodelet::init(ros::NodeHandle& nh)
{

  /*********************
   ** Communications
   **********************/
  advertiseTopics(nh);
  subscribeTopics(nh);

  /*********************
   ** Sigslots
   **********************/
  slot_wheel_state.connect(name + std::string("/joint_state"));
  slot_sensor_data.connect(name + std::string("/sensor_data"));
  slot_ir.connect(name + std::string("/ir"));
  slot_dock_ir.connect(name + std::string("/dock_ir"));
  slot_inertia.connect(name + std::string("/inertia"));
  slot_cliff.connect(name + std::string("/cliff"));
  slot_current.connect(name + std::string("/current"));
  slot_magnet.connect(name + std::string("/magnet"));
  slot_hw.connect(name + std::string("/hw"));
  slot_fw.connect(name + std::string("/fw"));
  slot_time.connect(name + std::string("/time"));
  slot_st_gyro.connect(name + std::string("/st_gyro"));
  slot_eeprom.connect(name + std::string("/eeprom"));
  slot_gp_input.connect(name + std::string("/gp_input"));

  slot_debug.connect(name + std::string("/ros_debug"));
  slot_info.connect(name + std::string("/ros_info"));
  slot_warn.connect(name + std::string("/ros_warn"));
  slot_error.connect(name + std::string("/ros_error"));

  /*********************
   ** Parameters
   **********************/
  Parameters parameters;

  parameters.sigslots_namespace = name; // name is automatically picked up by device_nodelet parent.
  if (!nh.getParam("device_port", parameters.device_port))
  {
    ROS_ERROR_STREAM("Kobuki : no device port given on the parameter server (e.g. /dev/ttyUSB0)[" << name << "].");
    return false;
  }
  if (!nh.getParam("protocol_version", parameters.protocol_version))
  {
    ROS_ERROR_STREAM("Kobuki : no protocol version given on the parameter server ('2.0')[" << name << "].");
    std::cout << "protocol_version: " << parameters.protocol_version << std::endl;
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
    ROS_INFO_STREAM("Kobuki : configured for connection on device_port " << parameters.device_port << " [" << name << "].");
    ROS_INFO_STREAM("Kobuki : configured for firmware protocol_version " << parameters.protocol_version << " [" << name << "].");
  }

  /*********************
   ** Frames
   **********************/

  if (!nh.getParam("odom_frame", odom_frame))
    NODELET_WARN_STREAM("DiffDriveBase : no param server setting for odom_frame, using default [" << odom_frame << "][" << name << "].");
  else
    NODELET_INFO_STREAM("DiffDriveBase : using odom_frame [" << odom_frame << "][" << name << "].");

  if (!nh.getParam("base_frame", base_frame))
    NODELET_WARN_STREAM("DiffDriveBase : no param server setting for base_frame, using default [" << base_frame << "][" << name << "].");
  else
    NODELET_INFO_STREAM("DiffDriveBase : using base_frame [" << base_frame << "][" << name << "].");

  if (!nh.getParam("publish_tf", publish_tf))
    NODELET_WARN_STREAM("DiffDriveBase : no param server setting for publish_tf, using default [" << publish_tf << "][" << name << "].");
  else
    NODELET_INFO_STREAM("DiffDriveBase : using publish_tf [" << publish_tf << "][" << name << "].");

  odom_trans.header.frame_id = odom_frame;
  odom_trans.child_frame_id = base_frame;
  odom.header.frame_id = odom_frame;
  odom.child_frame_id = base_frame;

  // Pose covariance (required by robot_pose_ekf) TODO: publish realistic values
  odom.pose.covariance[0] = 0.1;
  odom.pose.covariance[7] = 0.1;
  odom.pose.covariance[35] = 0.2;

  odom.pose.covariance[14] = 10;//DBL_MAX; // set a very large covariance on unused
  odom.pose.covariance[21] = 10;//DBL_MAX; // dimensions (z, pitch and roll); this
  odom.pose.covariance[28] = 10;//DBL_MAX; // is a requirement of robot_pose_ekf

  pose.setIdentity();

  /*********************
   ** Published msgs
   **********************/

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

  return true;
}

/**
 * Two groups of publishers, one required by turtlebot, the other for
 * kobuki esoterics.
 */
void KobukiNodelet::advertiseTopics(ros::NodeHandle& nh)
{
  /*********************
  ** Turtlebot Required
  **********************/
  joint_state_publisher = nh.advertise <sensor_msgs::JointState>("joint_states",100);
  odom_publisher = nh.advertise<nav_msgs::Odometry>("odom", 50); // topic name and queue size

  /*********************
  ** Kobuki Esoterics
  **********************/

  wheel_left_state_publisher = nh.advertise < device_comms::JointState > (std::string("joint_state/") + wheel_left_name, 100);
  wheel_right_state_publisher = nh.advertise < device_comms::JointState > (std::string("joint_state/") + wheel_right_name, 100);
  sensor_data_publisher = nh.advertise < kobuki_comms::SensorData > ("sensor_data", 100);

  ir_data_publisher = nh.advertise < kobuki_comms::IR > ("ir_data", 100);
  dock_ir_data_publisher = nh.advertise < kobuki_comms::DockIR > ("dock_ir_data", 100);
  inertia_data_publisher = nh.advertise < kobuki_comms::Inertia > ("inertia_data", 100);
  imu_data_publisher = nh.advertise < sensor_msgs::Imu > ("imu_data", 100);
  cliff_data_publisher = nh.advertise < kobuki_comms::Cliff > ("cliff_data", 100);
  current_data_publisher = nh.advertise < kobuki_comms::Current > ("current_data", 100);
  magnet_data_publisher = nh.advertise < kobuki_comms::Magnet > ("merge_data", 100);
  hw_data_publisher = nh.advertise < kobuki_comms::HW > ("hw_data", 100);
  fw_data_publisher = nh.advertise < kobuki_comms::FW > ("fw_data", 100);
  time_data_publisher = nh.advertise < kobuki_comms::Time > ("time_data", 100);
  st_gyro_data_publisher = nh.advertise < kobuki_comms::StGyro > ("st_gyro_data", 100);  // TODO delete?
  eeprom_data_publisher = nh.advertise < kobuki_comms::EEPROM > ("eeprom_data", 100);
  gp_input_data_publisher = nh.advertise < kobuki_comms::GpInput > ("gp_input_data", 100);
}

/**
 * Two groups of subscribers, one required by turtlebot, the other for
 * kobuki esoterics.
 */
void KobukiNodelet::subscribeTopics(ros::NodeHandle& nh)
{
  wheel_left_command_subscriber = nh.subscribe(std::string("joint_command/") + wheel_left_name, 10,
                                               &KobukiNodelet::subscribeJointCommandLeft, this);
  wheel_right_command_subscriber = nh.subscribe(std::string("joint_command/") + wheel_right_name, 10,
                                                &KobukiNodelet::subscribeJointCommandRight, this);
  velocity_command_subscriber = nh.subscribe(std::string("cmd_vel"), 10, &KobukiNodelet::subscribeVelocityCommand,
                                             this);
  kobuki_command_subscriber = nh.subscribe(std::string("kobuki_command"), 10, &KobukiNodelet::subscribeKobukiCommand,
                                           this);
}

/*****************************************************************************
 ** Implementation [CruizCoreNodelet][Threads]
 *****************************************************************************/
/**
 * @brief The cruizcore node's io processing loop (run in the thread).
 *
 * This loop runs in a background thread - this is necessary as you may often
 * wish to have differen't cruizcore's running at different scan frequencies (e.g.
 * arm cruizcore at high frequency, peripheral head/scanner motors at low frequency).
 *
 * Note that intermittent sends will intersperse themselves within a scan.
 *
 * The type of commands sent to each motor (board) is identified by the feedback command held
 * in each motor.
 */


void KobukiNodelet::publishTransform(const geometry_msgs::Quaternion &odom_quat)
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

void KobukiNodelet::publishOdom(const geometry_msgs::Quaternion &odom_quat,
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

void KobukiNodelet::publishWheelState()
{
  //waitForInitialisation();
  if (ros::ok() && !shutdown_requested)
  {
//    if (wheel_left_state_publisher.getNumSubscribers() > 0)
  //  {
//      kobuki.pubtime("  wheel_left:ent");
      device_comms::JointState joint_state_l;
      joint_state_l.name = "wheel_left";
      joint_state_l.stamp = ros::Time::now();
      kobuki.getJointState(joint_state_l);
      wheel_left_state_publisher.publish(joint_state_l);
//      kobuki.pubtime("  wheel_left:pub");
/*    }
    if (wheel_right_state_publisher.getNumSubscribers() > 0)
    {*/
//      kobuki.pubtime("  wheel_right:ent");
      device_comms::JointState joint_state_r;
      joint_state_r.name = "wheel_right";
      joint_state_r.stamp = ros::Time::now();
      kobuki.getJointState(joint_state_r);
      wheel_right_state_publisher.publish(joint_state_r);
//      kobuki.pubtime("  wheel_right:pub");
//    }

    // TODO really horrible; refactor
    ecl::Pose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;

    kobuki.updateOdometry(joint_state_l.position, joint_state_l.velocity,
                          joint_state_r.position, joint_state_r.velocity,
                          pose_update, pose_update_rates);

    joint_states.header.stamp = ros::Time::now();
    joint_state_publisher.publish(joint_states);

    pose *= pose_update;

    //since all ros tf odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.heading());

    publishTransform(odom_quat);
    publishOdom(odom_quat, pose_update_rates);
  }
}

/*****************************************************************************
 ** CruizCoreNodelet Publications
 *****************************************************************************/
/**
 * @brief Publishes byte representations of raw data packets that are received.
 *
 * This is a callback for the signal sent by the cruizcore driver whenever
 * a raw packet is received.
 *
 * @param bytes : the raw byte array that is being recieved.
 */
void KobukiNodelet::publishSensorData()
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

void KobukiNodelet::subscribeJointCommandLeft(const device_comms::JointCommand cmd)
{
  //cmd.value;
  return;
}

void KobukiNodelet::subscribeJointCommandRight(const device_comms::JointCommand cmd)
{
  //cmd.value;
  return;
}

void KobukiNodelet::subscribeVelocityCommand(const geometry_msgs::TwistConstPtr &msg)
{
  if (kobuki.isEnabled())
  {
    // For now assuming this is in the robot frame, but probably this
    // should be global frame and require a transform
    //double vx = msg->linear.x;	// in (m/s)
    //double wz = msg->angular.z;	// in (rad/s)
    ROS_DEBUG_STREAM("subscribeVelocityCommand: [" << msg->linear.x << "],[" << msg->angular.z << "]");
    kobuki.setCommand(msg->linear.x, msg->angular.z);
  }
  else
  {
    ROS_WARN("Robot is not enabled.");
  }
  return;
}

void KobukiNodelet::subscribeKobukiCommand(const kobuki_comms::CommandConstPtr &msg)
{
  //if( kobuki.isEnabled() ) {
  kobuki.sendCommand(msg);
  //} else {
  //	ROS_WARN("Robot is not enabled.");
  //}
  return;
}

/**
 * @brief Publishes byte representations of raw data packets that are sent.
 *
 * This is a callback for the signal sent by the cruizcore driver whenever
 * a raw packet is sent.
 *
 * @param bytes : the raw byte array that is being sent.
 */
#if 0
void CruizCoreNodelet::publishRawDataSent( const Packet::BufferStencil &bytes )
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
/**
 * @brief Publishes byte representations of invalid packets that are received.
 *
 * This is a callback for the signal sent by the cruizcore driver whenever
 * an invalid packet is received.
 *
 * @param bytes : the raw byte array that is the invalid packet being recieved.
 */
#if 0
void CruizCoreNodelet::publishInvalidPacket( const Packet::BufferStencil &bytes )
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

void KobukiNodelet::publishIRData()
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

void KobukiNodelet::publishDockIRData()
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

void KobukiNodelet::publishInertiaData()
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
      msg.header.frame_id;// = odom_frame;
      msg.header.seq = data.header.seq;
      msg.header.stamp = ros::Time::now();

      // angle comes as hundredths of degree, convert to radians
      float yaw = (data.angle/18000.0)*M_PI;
      msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);

      // set a very large covariance on unused dimensions (pitch and roll);
      // set yaw covariance as very low, to make it dominate over the odometry heading
      // TODO 1: fill once, as its always the same;  TODO 2: cannot get better estimation?
//      for (unsigned i = 0; i < msg.orientation_covariance.size(); ++i)
        msg.orientation_covariance[8] = 0.01;//DBL_MAX;
        msg.orientation_covariance[4] = 0.01;//DBL_MAX;

      msg.orientation_covariance[0] = 0.01;

      // ignore velocity and acceleration by now

      imu_data_publisher.publish(msg);
    }


  }
}

void KobukiNodelet::publishCliffData()
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

void KobukiNodelet::publishCurrentData()
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

void KobukiNodelet::publishMagnetData()
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

void KobukiNodelet::publishHWData()
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

void KobukiNodelet::publishFWData()
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

void KobukiNodelet::publishTimeData()
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

void KobukiNodelet::publishStGyroData()
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

void KobukiNodelet::publishEEPROMData()
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

void KobukiNodelet::publishGpInputData()
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

/*		slot_reserved0, Rei*/
} // namespace k

/*****************************************************************************
 ** Nodelet Plugin Registration
 *****************************************************************************/

PLUGINLIB_DECLARE_CLASS(kobuki_node, KobukiNodelet, kobuki::KobukiNodelet, nodelet::Nodelet);

