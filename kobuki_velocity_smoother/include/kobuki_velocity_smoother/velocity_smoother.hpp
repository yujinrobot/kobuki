/**
 * @file /include/kobuki_velocity_smoother/velocity_smoother_nodelet.hpp
 *
 * @brief Velocity smoother interface
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/devel/kobuki_velocity_smoother/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef KOBUKI_VELOCITY_SMOOTHER_HPP_
#define KOBUKI_VELOCITY_SMOOTHER_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <algorithm>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <ecl/threads/thread.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki_velocity_smoother {

/*****************************************************************************
** VelocitySmoother
*****************************************************************************/

class VelocitySmoother final : public rclcpp::Node
{
public:
  explicit VelocitySmoother(const rclcpp::NodeOptions & options);

  ~VelocitySmoother() override;
  VelocitySmoother(VelocitySmoother && c) = delete;
  VelocitySmoother & operator=(VelocitySmoother && c) = delete;
  VelocitySmoother(const VelocitySmoother & c) = delete;
  VelocitySmoother & operator=(const VelocitySmoother & c) = delete;

private:
  enum RobotFeedbackType
  {
    NONE,
    ODOMETRY,
    COMMANDS
  } robot_feedback;  /**< What source to use as robot velocity feedback */

  std::string name{"velocity_smoother"};
  bool quiet;        /**< Quieten some warnings that are unavoidable because of velocity multiplexing. **/
  double speed_lim_v, accel_lim_v, decel_lim_v;
  double speed_lim_w, accel_lim_w, decel_lim_w;
  double decel_factor;

  double frequency;

  std::mutex current_vel_mutex;
  geometry_msgs::msg::Twist  current_vel;
  std::mutex target_vel_mutex;
  geometry_msgs::msg::Twist   target_vel;
  double last_cmd_vel_linear_x;
  double last_cmd_vel_angular_z;

  bool                 shutdown_req; /**< Shutdown requested by nodelet; kill worker thread */
  std::atomic<bool>                 input_active;
  std::atomic<double>                cb_avg_time;
  std::mutex              last_velocity_mutex;
  rclcpp::Time            last_velocity_cb_time;
  std::vector<double> period_record; /**< Historic of latest periods between velocity commands */
  unsigned int             pr_next; /**< Next position to fill in the periods record buffer */

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;    /**< Current velocity from odometry */
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr current_vel_sub; /**< Current velocity from commands sent to the robot, not necessarily by this node */
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_in_vel_sub;  /**< Incoming raw velocity commands */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  smooth_vel_pub;  /**< Outgoing smoothed velocity commands */

  ecl::Thread spin_thread;

  void spin();
  void velocityCB(const geometry_msgs::msg::Twist::SharedPtr msg);
  void robotVelCB(const geometry_msgs::msg::Twist::SharedPtr msg);
  void odometryCB(const nav_msgs::msg::Odometry::SharedPtr msg);

  double sign(double x)  { return x < 0.0 ? -1.0 : +1.0; };

  double median(std::vector<double> & values) {
    // Return the median element of an doubles vector
    std::nth_element(values.begin(), values.begin() + values.size()/2, values.end());
    return values[values.size()/2];
  }
};

} // namespace kobuki_velocity_smoother

#endif /* KOBUKI_VELOCITY_SMOOTHER_HPP_ */
