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
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <ecl/sigslots.hpp>
#include <kobuki_comms/Cliff.h>
#include <kobuki_comms/ButtonEvent.h>
#include <kobuki_comms/CoreSensors.h>
#include <kobuki_comms/Current.h>
#include <kobuki_comms/GpInput.h>
#include <kobuki_comms/LedArray.h>
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
  bool spin();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  void advertiseTopics(ros::NodeHandle& nh);
  void subscribeTopics(ros::NodeHandle& nh);
  void publishTransform(const geometry_msgs::Quaternion &odom_quat);
  void publishOdom(const geometry_msgs::Quaternion &odom_quat, const ecl::linear_algebra::Vector3d &pose_update_rates);

  /*********************
   ** Variables
   **********************/
  std::string name; // name of the ROS node
  Kobuki kobuki;
  uint8_t buttons_state;
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
  ros::Publisher imu_data_publisher,
                 cliff_sensor_publisher, current_sensor_publisher, version_info_publisher,
                 gp_input_data_publisher, joint_state_publisher, odom_publisher,
                 core_sensor_data_publisher, button_events_publisher;
  ros::Subscriber velocity_command_subscriber, led_command_subscriber, reset_odometry_subscriber;
  ros::Subscriber enable_subscriber, disable_subscriber; // may eventually disappear

  ecl::Slot<> slot_wheel_state, slot_core_sensors,
              slot_inertia, slot_cliff, slot_current, slot_version_info,
              slot_gp_input;
  ecl::Slot<const std::string&> slot_debug, slot_info, slot_warn, slot_error;
  tf::TransformBroadcaster odom_broadcaster;
  sensor_msgs::JointState joint_states;

  /*********************
   ** SigSlots
   **********************/
  void publishWheelState();
  void publishCoreSensorData();
  void publishInertiaData();
  void publishCliffData();
  void publishCurrentData();
  void publishVersionInfo();
  void publishGpInputData();
  void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr);
  void subscribeLedCommand(const kobuki_comms::LedArrayConstPtr);
  void subscribeResetOdometry(const std_msgs::EmptyConstPtr);

  /*********************
   ** Ros Logging
   **********************/
  void rosDebug(const std::string &msg) { ROS_DEBUG_STREAM("Kobuki : " << msg); }
  void rosInfo(const std::string &msg) { ROS_INFO_STREAM("Kobuki : " << msg); }
  void rosWarn(const std::string &msg) { ROS_WARN_STREAM("Kobuki : " << msg); }
  void rosError(const std::string &msg) { ROS_ERROR_STREAM("Kobuki : " << msg); }

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
