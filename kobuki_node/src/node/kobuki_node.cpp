/**
 * @file /kobuki_node/src/node/kobuki_node.cpp
 *
 * @brief ...
 *
 * @date 10/04/2012
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
    slot_wheel_state(&KobukiNode::publishWheelState, *this),
    slot_sensor_data(&KobukiNode::publishSensorData,*this),
    slot_ir(&KobukiNode::publishIRData, *this),
    slot_dock_ir(&KobukiNode::publishDockIRData, *this),
    slot_inertia(&KobukiNode::publishInertiaData, *this),
    slot_cliff(&KobukiNode::publishCliffData, *this),
    slot_current(&KobukiNode::publishCurrentData, *this),
    slot_hw(&KobukiNode::publishHWData, *this),
    slot_fw(&KobukiNode::publishFWData, *this),
    slot_time(&KobukiNode::publishTimeData, *this),
    slot_eeprom(&KobukiNode::publishEEPROMData, *this),
    slot_gp_input(&KobukiNode::publishGpInputData, *this),
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
}

/**
 :* @brief Destructs, but only after the thread has cleanly terminated.
 *
 * Ensures we stay alive long enough for the thread to cleanly terminate.
 */
KobukiNode::~KobukiNode()
{
  ROS_INFO_STREAM("Kobuki : waiting for kobuki thread to finish [" << name << "].");
  kobuki.close();
  //kobuki.join();
}

bool KobukiNode::init(ros::NodeHandle& nh)
{

  /*********************
   ** Communications
   **********************/
  advertiseTopics(nh);
  subscribeTopics(nh);

  reset_odometry_server = nh.advertiseService("reset_odometry", &KobukiNode::serveResetOdometry, this);

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
  slot_hw.connect(name + std::string("/hw"));
  slot_fw.connect(name + std::string("/fw"));
  slot_time.connect(name + std::string("/time"));
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

  if (!nh.getParam("base_frame", base_frame))
    ROS_WARN_STREAM("Kobuki : no param server setting for base_frame, using default [" << base_frame << "][" << name << "].");
  else
    ROS_INFO_STREAM("Kobuki : using base_frame [" << base_frame << "][" << name << "].");

  if (!nh.getParam("publish_tf", publish_tf))
    ROS_WARN_STREAM("Kobuki : no param server setting for publish_tf, using default [" << publish_tf << "][" << name << "].");
  else
    ROS_INFO_STREAM("Kobuki : using publish_tf [" << publish_tf << "][" << name << "].");

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

bool KobukiNode::spin(ros::NodeHandle& nh)
{
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    if ((kobuki.isEnabled() == true) && (last_cmd_time.isZero() == false) &&
        ((ros::Time::now() - last_cmd_time) > cmd_vel_timeout)) {
      std_msgs::StringPtr msg;
      disable(msg);

      ROS_DEBUG("No cmd_vel messages received within the last %.2f seconds; disable driver",
                cmd_vel_timeout.toSec());
    }

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
  sensor_data_publisher = nh.advertise < kobuki_comms::SensorData > ("sensor_data", 100);
  ir_data_publisher = nh.advertise < kobuki_comms::IR > ("ir_data", 100);
  dock_ir_data_publisher = nh.advertise < kobuki_comms::DockIR > ("dock_ir_data", 100);
  imu_data_publisher = nh.advertise < sensor_msgs::Imu > ("imu_data", 100);
  cliff_data_publisher = nh.advertise < kobuki_comms::Cliff > ("cliff_data", 100);
  current_data_publisher = nh.advertise < kobuki_comms::Current > ("current_data", 100);
  hw_data_publisher = nh.advertise < kobuki_comms::HW > ("hw_data", 100);
  fw_data_publisher = nh.advertise < kobuki_comms::FW > ("fw_data", 100);
  time_data_publisher = nh.advertise < kobuki_comms::Time > ("time_data", 100);
  eeprom_data_publisher = nh.advertise < kobuki_comms::EEPROM > ("eeprom_data", 100);
  gp_input_data_publisher = nh.advertise < kobuki_comms::GpInput > ("gp_input_data", 100);
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


/*****************************************************************************
 ** Service Callbacks
 *****************************************************************************/
/**
 * @brief Reset the odometry variables.
 *
 * @param request : the stream type to send back.
 * @param response : the outgoing data stream (empty).
 * @return bool : always succeeds (true).
 */
bool KobukiNode::serveResetOdometry(std_srvs::Empty::Request &request,
                                    std_srvs::Empty::Response &response)
{
  ROS_INFO_STREAM("Mobile base : resetting the odometry [" << name << "].");
  pose.setIdentity();
  joint_states.position[0] = 0.0; // wheel_left
  joint_states.velocity[0] = 0.0;
  joint_states.position[1] = 0.0; // wheel_right
  joint_states.velocity[1] = 0.0;

  kobuki.resetOdometry();

  return true;
}

} // namespace kobuki


/*		slot_reserved0, Rei */

