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
 * @file /kobuki_safety_controller/include/kobuki_safety_controller/safety_controller.hpp
 *
 * @brief Kobuki-specific safety controller
 *
 * This controller uses Kobuki's bumper, cliff and wheel drop sensors to ensure safe operation.
 *
 * @author Marcus Liebhardt, Yujin Robot
 *
 * @date Nov 30, 2012
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef SAFETY_CONTROLLER_HPP_
#define SAFETY_CONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <ros/ros.h>
#include <ocl_controllers/default_controller.hpp>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/WheelDropEvent.h>

namespace kobuki
{

/**
 * @ brief Keeps track of safety-related events and commands Kobuki to move accordingly
 *
 * The SafetyController keeps track of bumper, cliff and wheel drop events. In case of the first two,
 * Kobuki is commanded to move back. In the latter case, Kobuki is stopped.
 *
 * This controller can be enabled/disabled.
 * The safety states (bumper pressed etc.) can be reseted, what can be dangerous!
 */
class SafetyController : public ocl::Controller
{
public:
  SafetyController(ros::NodeHandle& nh, std::string& name) :
    Controller(),
    nh_(nh),
    name_(name),
    bumper_pressed_(false),
    cliff_detected_(false),
    wheel_left_dropped_(false),
    wheel_right_dropped_(false){};
  ~SafetyController(){};

  /**
   * Set-up necessary publishers/subscribers and avariables
   * @return true, if successful
   */
  bool init()
  {
    enable_controller_subscriber_ = nh_.subscribe("enable", 10, &SafetyController::enableCB, this);
    disable_controller_subscriber_ = nh_.subscribe("disable", 10, &SafetyController::disableCB, this);
    bumper_event_subscriber_ = nh_.subscribe("events/bumper", 10, &SafetyController::bumperEventCB, this);
    cliff_event_subscriber_  = nh_.subscribe("events/cliff",  10, &SafetyController::cliffEventCB, this);
    wheel_event_subscriber_  = nh_.subscribe("events/wheel_drop", 10, &SafetyController::wheelEventCB, this);
    reset_safety_states_subscriber_ = nh_.subscribe("reset", 10, &SafetyController::resetSafetyStatesCB, this);
    velocity_command_publisher_ = nh_.advertise< geometry_msgs::Twist >("cmd_vel", 10);
    return true;
  };

  /**
   * @ brief Checks safety states and publishes velocity commands when necessary
   */
  void spin();

private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_;
  ros::Subscriber bumper_event_subscriber_, cliff_event_subscriber_, wheel_event_subscriber_;
  ros::Subscriber reset_safety_states_subscriber_;
  ros::Publisher controller_state_publisher_, velocity_command_publisher_;
  bool bumper_pressed_, cliff_detected_, wheel_left_dropped_, wheel_right_dropped_;
  geometry_msgs::TwistPtr msg_; // velocity command

  /**
   * @brief ROS logging output for enabling the controller
   * @param msg incoming topic message
   */
  void enableCB(const std_msgs::EmptyConstPtr msg);

  /**
   * @brief ROS logging output for disabling the controller
   * @param msg incoming topic message
   */
  void disableCB(const std_msgs::EmptyConstPtr msg);

  /**
   * @brief Keeps track of bumps
   * @param msg incoming topic message
   */
  void bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg);

  /**
   * @brief Keeps track of cliff detection
   * @param msg incoming topic message
   */
  void cliffEventCB(const kobuki_msgs::CliffEventConstPtr msg);

  /**
   * @brief Keeps track of the wheel states
   * @param msg incoming topic message
   */
  void wheelEventCB(const kobuki_msgs::WheelDropEventConstPtr msg);

  /**
   * @brief Callback for resetting safety variables
   *
   * Allows resetting bumper, cliff and wheel drop states.
   * DANGEROUS!
   *
   * @param msg incoming topic message
   */
  void resetSafetyStatesCB(const std_msgs::EmptyConstPtr msg);
};


void SafetyController::enableCB(const std_msgs::EmptyConstPtr msg)
{
  if (this->enable())
  {
    ROS_INFO_STREAM("Controller has been enabled. [" << name_ << "]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already enabled. [" << name_ <<"]");
  }
};

void SafetyController::disableCB(const std_msgs::EmptyConstPtr msg)
{
  if (this->disable())
  {
    ROS_INFO_STREAM("Controller has been disabled. [" << name_ <<"]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already disabled. [" << name_ <<"]");
  }
};

void SafetyController::cliffEventCB(const kobuki_msgs::CliffEventConstPtr msg)
{
  if (msg->state == kobuki_msgs::CliffEvent::CLIFF)
  {
    ROS_DEBUG_STREAM("Cliff detected. Moving backwards. [" << name_ << "]");
    cliff_detected_ = true;
  }
  else // kobuki_msgs::CliffEvent::FLOOR
  {
    ROS_DEBUG_STREAM("Not detecting any cliffs. Resuming normal operation. [" << name_ << "]");
    cliff_detected_ = false;
  }
};


void SafetyController::bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg)
{
  if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
  {
    ROS_DEBUG_STREAM("Bumper pressed. Moving backwards. [" << name_ << "]");
    bumper_pressed_ = true;
  }
  else // kobuki_msgs::BumperEvent::RELEASED
  {
    ROS_DEBUG_STREAM("No bumper pressed. Resuming normal operation. [" << name_ << "]");
    bumper_pressed_ = false;
  }
};

void SafetyController::wheelEventCB(const kobuki_msgs::WheelDropEventConstPtr msg)
{
  if (msg->state == kobuki_msgs::WheelDropEvent::DROPPED)
  {
    // need to keep track of both wheels separately
    if (msg->wheel == kobuki_msgs::WheelDropEvent::LEFT)
    {
      ROS_DEBUG_STREAM("Left wheel dropped. [" << name_ << "]");
      wheel_left_dropped_ = true;
    }
    else // kobuki_msgs::WheelDropEvent::RIGHT
    {
      ROS_DEBUG_STREAM("Right wheel dropped. [" << name_ << "]");
      wheel_right_dropped_ = true;
    }
  }
  else // kobuki_msgs::WheelDropEvent::RAISED
  {
    // need to keep track of both wheels separately
    if (msg->wheel == kobuki_msgs::WheelDropEvent::LEFT)
    {
      ROS_DEBUG_STREAM("Left wheel raised. [" << name_ << "]");
      wheel_left_dropped_ = false;
    }
    else // kobuki_msgs::WheelDropEvent::RIGHT
    {
      ROS_DEBUG_STREAM("Right wheel raised. [" << name_ << "]");
      wheel_right_dropped_ = false;
    }
    if (!wheel_left_dropped_ && !wheel_right_dropped_)
    {
      ROS_DEBUG_STREAM("Both wheels raised. Resuming normal operation. [" << name_ << "]");
    }
  }
};

void SafetyController::resetSafetyStatesCB(const std_msgs::EmptyConstPtr msg)
{
  wheel_left_dropped_ = false;
  wheel_right_dropped_ = false;
  bumper_pressed_ = false;
  cliff_detected_ = false;
  ROS_WARN_STREAM("All safety states have been reseted to false. [" << name_ << "]");
}

void SafetyController::spin()
{
  if (this->getState())
  {
    if (wheel_left_dropped_ || wheel_right_dropped_)
    {
      msg_.reset(new geometry_msgs::Twist());
      msg_->linear.x = 0.0;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = 0.0;
      velocity_command_publisher_.publish(msg_);
    }
    else if (bumper_pressed_ || cliff_detected_)
    {
      msg_.reset(new geometry_msgs::Twist());
      msg_->linear.x = -0.1;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = 0.0;
      velocity_command_publisher_.publish(msg_);
    }
  }
};

} // namespace kobuki

#endif /* SAFETY_CONTROLLER_HPP_ */
