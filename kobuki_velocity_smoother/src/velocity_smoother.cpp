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

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "kobuki_velocity_smoother/velocity_smoother.hpp"

/*****************************************************************************
 ** Preprocessing
 *****************************************************************************/

#define PERIOD_RECORD_SIZE    5

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki_velocity_smoother {

/*********************
** Implementation
**********************/

VelocitySmoother::VelocitySmoother(const rclcpp::NodeOptions & options) : rclcpp::Node("velocity_smoother_node", options)
, input_active(false)
, pr_next(0)
{
  double frequency = this->declare_parameter("frequency", 20.0);
  quiet = this->declare_parameter("quiet", false);
  double decel_factor = this->declare_parameter("decel_factor", 1.0);
  int feedback = this->declare_parameter("robot_feedback", static_cast<int>(NONE));

  if ((static_cast<int>(feedback) < NONE) || (static_cast<int>(feedback) > COMMANDS))
  {
    throw std::runtime_error("Invalid robot feedback type. Valid options are 0 (NONE, default), 1 (ODOMETRY) and 2 (COMMANDS)");
  }

  robot_feedback = static_cast<RobotFeedbackType>(feedback);

  // Mandatory parameters
  rclcpp::ParameterValue speed_v = this->declare_parameter("speed_lim_v");
  if (speed_v.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    throw std::runtime_error("speed_lim_v must be specified as a double");
  }
  speed_lim_v = speed_v.get<double>();

  rclcpp::ParameterValue speed_w = this->declare_parameter("speed_lim_w");
  if (speed_w.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    throw std::runtime_error("speed_lim_w must be specified as a double");
  }
  speed_lim_w = speed_w.get<double>();

  rclcpp::ParameterValue accel_v = this->declare_parameter("accel_lim_v");
  if (accel_v.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    throw std::runtime_error("accel_lim_v must be specified as a double");
  }
  accel_lim_v = accel_v.get<double>();

  rclcpp::ParameterValue accel_w = this->declare_parameter("accel_lim_w");
  if (accel_w.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    throw std::runtime_error("accel_lim_w must be specified as a double");
  }
  accel_lim_w = accel_w.get<double>();

  // Deceleration can be more aggressive, if necessary
  decel_lim_v = decel_factor*accel_lim_v;
  decel_lim_w = decel_factor*accel_lim_w;

  // Publishers and subscribers
  odometry_sub    = this->create_subscription<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(1), std::bind(&VelocitySmoother::odometryCB, this, std::placeholders::_1));
  current_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("robot_cmd_vel", rclcpp::QoS(1), std::bind(&VelocitySmoother::robotVelCB, this, std::placeholders::_1));
  raw_in_vel_sub  = this->create_subscription<geometry_msgs::msg::Twist>("raw_cmd_vel", rclcpp::QoS(1), std::bind(&VelocitySmoother::velocityCB, this, std::placeholders::_1));
  smooth_vel_pub  = this->create_publisher<geometry_msgs::msg::Twist>("smooth_cmd_vel", 1);

  period = 1.0 / frequency;
  timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<uint64_t>(period * 1000.0)), std::bind(&VelocitySmoother::timerCB, this));
}

VelocitySmoother::~VelocitySmoother()
{
}

void VelocitySmoother::velocityCB(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Estimate commands frequency; we do continuously as it can be very different depending on the
  // publisher type, and we don't want to impose extra constraints to keep this package flexible
  if (period_record.size() < PERIOD_RECORD_SIZE)
  {
    period_record.push_back((this->get_clock()->now() - last_velocity_cb_time).seconds());
  }
  else
  {
    period_record[pr_next] = (this->get_clock()->now() - last_velocity_cb_time).seconds();
  }

  pr_next++;
  pr_next %= period_record.size();
  last_velocity_cb_time = this->get_clock()->now();

  if (period_record.size() <= PERIOD_RECORD_SIZE/2)
  {
    // wait until we have some values; make a reasonable assumption (10 Hz) meanwhile
    cb_avg_time = 0.1;
  }
  else
  {
    // enough; recalculate with the latest input
    cb_avg_time = median(period_record);
  }

  input_active = true;

  // Bound speed with the maximum values
  target_vel.linear.x  =
    msg->linear.x  > 0.0 ? std::min(msg->linear.x,  speed_lim_v) : std::max(msg->linear.x,  -speed_lim_v);
  target_vel.angular.z =
    msg->angular.z > 0.0 ? std::min(msg->angular.z, speed_lim_w) : std::max(msg->angular.z, -speed_lim_w);
}

void VelocitySmoother::odometryCB(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (robot_feedback == ODOMETRY)
  {
    current_vel = msg->twist.twist;
  }

  // ignore otherwise
}

void VelocitySmoother::robotVelCB(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (robot_feedback == COMMANDS)
  {
    current_vel = *msg;
  }

  // ignore otherwise
}

void VelocitySmoother::timerCB()
{
  if ((input_active == true) && (cb_avg_time > 0.0) &&
      ((this->get_clock()->now() - last_velocity_cb_time).seconds() > std::min(3.0*cb_avg_time, 0.5)))
  {
    // Velocity input no active anymore; normally last command is a zero-velocity one, but reassure
    // this, just in case something went wrong with our input, or he just forgot good manners...
    // Issue #2, extra check in case cb_avg_time is very big, for example with several atomic commands
    // The cb_avg_time > 0 check is required to deal with low-rate simulated time, that can make that
    // several messages arrive with the same time and so lead to a zero median
    input_active = false;
    if (target_vel.linear.x != 0.0 || target_vel.angular.z != 0.0)
    {
      RCLCPP_WARN(get_logger(), "Velocity Smoother : input went inactive leaving us a non-zero target velocity (%d, %d), zeroing...[%s]",
                  target_vel.linear.x,
                  target_vel.angular.z,
                  name.c_str());
      target_vel = geometry_msgs::msg::Twist();
    }
  }

  // check if the feedback is off from what we expect
  // don't care about min / max velocities here, just for rough checking
  double period_buffer = 2.0;

  double v_deviation_lower_bound = last_cmd_vel_linear_x - decel_lim_v * period * period_buffer;
  double v_deviation_upper_bound = last_cmd_vel_linear_x + accel_lim_v * period * period_buffer;

  double w_deviation_lower_bound = last_cmd_vel_angular_z - decel_lim_w * period * period_buffer;
  double angular_max_deviation = last_cmd_vel_angular_z + accel_lim_w * period * period_buffer;

  bool v_different_from_feedback = current_vel.linear.x < v_deviation_lower_bound || current_vel.linear.x > v_deviation_upper_bound;
  bool w_different_from_feedback = current_vel.angular.z < w_deviation_lower_bound || current_vel.angular.z > angular_max_deviation;

  if ((robot_feedback != NONE) && (input_active == true) && (cb_avg_time > 0.0) &&
      (((this->get_clock()->now() - last_velocity_cb_time).seconds() > 5.0*cb_avg_time)     || // 5 missing msgs
          v_different_from_feedback || w_different_from_feedback))
  {
    // If the publisher has been inactive for a while, or if our current commanding differs a lot
    // from robot velocity feedback, we cannot trust the former; relay on robot's feedback instead
    // This might not work super well using the odometry if it has a high delay
    if ( !quiet ) {
      // this condition can be unavoidable due to preemption of current velocity control on
      // velocity multiplexer so be quiet if we're instructed to do so
      RCLCPP_WARN(get_logger(), "Velocity Smoother : using robot velocity feedback %s instead of last command: %f, %f, %f, [%s]",
                  std::string(robot_feedback == ODOMETRY ? "odometry" : "end commands").c_str(),
                  (this->get_clock()->now() - last_velocity_cb_time).seconds(),
                  current_vel.linear.x  - last_cmd_vel_linear_x,
                  current_vel.angular.z - last_cmd_vel_angular_z,
                  name.c_str());
    }
    last_cmd_vel_linear_x = current_vel.linear.x;
    last_cmd_vel_angular_z = current_vel.angular.z;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();

  if ((target_vel.linear.x  != last_cmd_vel_linear_x) ||
      (target_vel.angular.z != last_cmd_vel_angular_z))
  {
    // Try to reach target velocity ensuring that we don't exceed the acceleration limits
    cmd_vel->linear = target_vel.linear;
    cmd_vel->angular = target_vel.angular;

    double v_inc, w_inc, max_v_inc, max_w_inc;

    v_inc = target_vel.linear.x - last_cmd_vel_linear_x;
    if ((robot_feedback == ODOMETRY) && (current_vel.linear.x*target_vel.linear.x < 0.0))
    {
      // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
      max_v_inc = decel_lim_v*period;
    }
    else
    {
      max_v_inc = ((v_inc*target_vel.linear.x > 0.0) ? accel_lim_v : decel_lim_v)*period;
    }

    w_inc = target_vel.angular.z - last_cmd_vel_angular_z;
    if ((robot_feedback == ODOMETRY) && (current_vel.angular.z*target_vel.angular.z < 0.0))
    {
      // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
      max_w_inc = decel_lim_w*period;
    }
    else
    {
      max_w_inc = ((w_inc*target_vel.angular.z > 0.0) ? accel_lim_w : decel_lim_w)*period;
    }

    // Calculate and normalise vectors A (desired velocity increment) and B (maximum velocity increment),
    // where v acts as coordinate x and w as coordinate y; the sign of the angle from A to B determines
    // which velocity (v or w) must be overconstrained to keep the direction provided as command
    double MA = std::sqrt(    v_inc *     v_inc +     w_inc *     w_inc);
    double MB = std::sqrt(max_v_inc * max_v_inc + max_w_inc * max_w_inc);

    double Av = std::abs(v_inc) / MA;
    double Aw = std::abs(w_inc) / MA;
    double Bv = max_v_inc / MB;
    double Bw = max_w_inc / MB;
    double theta = std::atan2(Bw, Bv) - atan2(Aw, Av);

    if (theta < 0)
    {
      // overconstrain linear velocity
      max_v_inc = (max_w_inc*std::abs(v_inc))/std::abs(w_inc);
    }
    else
    {
      // overconstrain angular velocity
      max_w_inc = (max_v_inc*std::abs(w_inc))/std::abs(v_inc);
    }

    if (std::abs(v_inc) > max_v_inc)
    {
      // we must limit linear velocity
      cmd_vel->linear.x  = last_cmd_vel_linear_x  + sign(v_inc)*max_v_inc;
    }

    if (std::abs(w_inc) > max_w_inc)
    {
      // we must limit angular velocity
      cmd_vel->angular.z = last_cmd_vel_angular_z + sign(w_inc)*max_w_inc;
    }
    last_cmd_vel_linear_x = cmd_vel->linear.x;
    last_cmd_vel_angular_z = cmd_vel->angular.z;
    smooth_vel_pub->publish(std::move(cmd_vel));
  }
  else if (input_active == true)
  {
    // We already reached target velocity; just keep resending last command while input is active
    smooth_vel_pub->publish(std::move(cmd_vel));
  }
}

rcl_interfaces::msg::SetParametersResult VelocitySmoother::parameterUpdate(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const rclcpp::Parameter & parameter : parameters)
  {
    if (parameter.get_name() == "frequency")
    {
      result.successful = false;
      result.reason = "frequency cannot be changed on-the-fly";
      break;
    }
    else if (parameter.get_name() == "quiet")
    {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "quiet must be a boolean";
        break;
      }

      quiet = parameter.get_value<bool>();
    }
    else if (parameter.get_name() == "decel_factor")
    {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "decel_factor must be a double";
        break;
      }

      double decel_factor = parameter.get_value<double>();
      decel_lim_v = decel_factor*accel_lim_v;
      decel_lim_w = decel_factor*accel_lim_w;
    }
    else if (parameter.get_name() == "robot_feedback")
    {
      result.successful = false;
      result.reason = "robot_feedback cannot be changed on-the-fly";
      break;
    }
    else if (parameter.get_name() == "speed_lim_v")
    {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "speed_lim_v must be a double";
        break;
      }

      speed_lim_v = parameter.get_value<double>();
    }
    else if (parameter.get_name() == "speed_lim_w")
    {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "speed_lim_w must be a double";
        break;
      }

      speed_lim_w = parameter.get_value<double>();
    }
    else if (parameter.get_name() == "accel_lim_v")
    {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "accel_lim_v must be a double";
        break;
      }

      accel_lim_v = parameter.get_value<double>();
    }
    else if (parameter.get_name() == "accel_lim_w")
    {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "accel_lim_w must be a double";
        break;
      }

      accel_lim_w = parameter.get_value<double>();
    }
    else
    {
      result.successful = false;
      result.reason = "unknown parameter";
      break;
    }
  }

  return result;
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(kobuki_velocity_smoother::VelocitySmoother)
