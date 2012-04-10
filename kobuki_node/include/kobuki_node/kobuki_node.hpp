/**
 * @file /kobuki_node/include/kobuki_node/kobuki_node.hpp
 *
 * @brief Wraps the kobuki driver in a ROS node
 *
 * @date 10/04/2012
 **/

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef KOBUKI_NODE_HPP_
#define KOBUKI_NODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <kobuki_comms/SensorData.h>
#include <ecl/sigslots.hpp>
#include <kobuki_driver/kobuki.hpp>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{
class KobukiNode
{
public:
  /*********************
   ** C&D
   **********************/
  KobukiNode(std::string& node_name);
  ~KobukiNode();

  bool init(ros::NodeHandle& nh);
  bool spin(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  void advertiseTopics(ros::NodeHandle& nh);
  void subscribeTopics(ros::NodeHandle& nh);

  void publishTransform(const geometry_msgs::Quaternion &odom_quat);
  void publishOdom(const geometry_msgs::Quaternion &odom_quat, const ecl::linear_algebra::Vector3d &pose_update_rates);

  bool serveResetOdometry(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

  /*********************
   ** Variables
   **********************/
  std::string name; // name of the ROS node

  Kobuki kobuki;

  // Continuously published messages
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom;
  ecl::Pose2D<double> pose;

  const std::string wheel_left_name;
  const std::string wheel_right_name;

  std::string odom_frame;
  std::string base_frame;
  ros::Duration cmd_vel_timeout;
  ros::Time last_cmd_time;
  bool publish_tf;

  /*********************
   ** Ros Comms
   **********************/
  ros::ServiceServer reset_odometry_server;

  ros::Publisher ir_data_publisher, dock_ir_data_publisher, imu_data_publisher,
                 cliff_data_publisher, current_data_publisher, hw_data_publisher,
                 fw_data_publisher, time_data_publisher, eeprom_data_publisher,
                 gp_input_data_publisher, joint_state_publisher, odom_publisher,
                 sensor_data_publisher;
  ros::Subscriber velocity_command_subscriber, kobuki_command_subscriber, enable_subscriber, disable_subscriber;

  ecl::Slot<> slot_wheel_state, slot_sensor_data, slot_ir, slot_dock_ir,
              slot_inertia, slot_cliff, slot_current, slot_hw, slot_fw, slot_time,
              slot_eeprom, slot_gp_input;
  ecl::Slot<const std::string&> slot_debug, slot_info, slot_warn, slot_error;
  tf::TransformBroadcaster odom_broadcaster;
  sensor_msgs::JointState joint_states;

  /*********************
   ** SigSlots
   **********************/
  void publishWheelState();
  void publishSensorData();
  void publishIRData();
  void publishDockIRData();
  void publishInertiaData();
  void publishCliffData();
  void publishCurrentData();
  void publishHWData();
  void publishFWData();
  void publishTimeData();
  void publishEEPROMData();
  void publishGpInputData();
  void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr);
  void subscribeKobukiCommand(const kobuki_comms::CommandConstPtr);

  /*********************
   ** Ros Logging
   **********************/
  void rosDebug(const std::string &msg)
  {
    ROS_DEBUG_STREAM("Kobuki : " << msg);
  }
  void rosInfo(const std::string &msg)
  {
    ROS_INFO_STREAM("Kobuki : " << msg);
  }
  void rosWarn(const std::string &msg)
  {
    ROS_WARN_STREAM("Kobuki : " << msg);
  }
  void rosError(const std::string &msg)
  {
    ROS_ERROR_STREAM("Kobuki : " << msg);
  }

  void enable(const std_msgs::StringConstPtr msg)
  {
    kobuki.enable();
    ROS_INFO_STREAM("Kobuki : enabled.");
    last_cmd_time.fromSec(0);
  }

  void disable(const std_msgs::StringConstPtr msg)
  {
    kobuki.disable();
    ROS_INFO_STREAM("Kobuki : disabled.");
    last_cmd_time.fromSec(0);
  }
};

} // namespace kobuki

#endif /* KOBUKI_NODE_HPP_ */
