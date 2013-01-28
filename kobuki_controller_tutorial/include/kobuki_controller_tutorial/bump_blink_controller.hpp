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
 * @file /kobuki_controller_tutorial/include/kobuki_controller_tutorial/bump_blink_controller.hpp
 *
 * @brief "Bump-Blink"-controller for the Kobuki controller tutorial
 *
 * A simple nodelet-based controller for Kobuki, which makes one of Kobuki's LEDs blink, when a bumper is pressed.
 *
 * @author Marcus Liebhardt, Yujin Robot
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef BUMP_BLINK_CONTROLLER_HPP_
#define BUMP_BLINK_CONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <yocs_controllers/default_controller.hpp>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/Led.h>

namespace kobuki
{

/**
 * @ brief A simple bump-blink-controller
 *
 * A simple nodelet-based controller for Kobuki, which makes one of Kobuki's LEDs blink, when a bumper is pressed.
 */
class BumpBlinkController : public yocs::Controller
{
public:
  BumpBlinkController(ros::NodeHandle& nh, std::string& name) : Controller(), nh_(nh), name_(name){};
  ~BumpBlinkController(){};

  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  bool init()
  {
    enable_controller_subscriber_ = nh_.subscribe("enable", 10, &BumpBlinkController::enableCB, this);
    disable_controller_subscriber_ = nh_.subscribe("disable", 10, &BumpBlinkController::disableCB, this);
    bumper_event_subscriber_ = nh_.subscribe("events/bumper", 10, &BumpBlinkController::bumperEventCB, this);
    // choose between led1 and led2
    blink_publisher_ = nh_.advertise< kobuki_msgs::Led >("commands/led1", 10);
    return true;
  };

private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_;
  ros::Subscriber bumper_event_subscriber_;
  ros::Publisher blink_publisher_;

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
   * @brief Turns on/off a LED, when a bumper is pressed/released
   * @param msg incoming topic message
   */
  void bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg);
};

void BumpBlinkController::enableCB(const std_msgs::EmptyConstPtr msg)
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

void BumpBlinkController::disableCB(const std_msgs::EmptyConstPtr msg)
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

void BumpBlinkController::bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg)
{
  if (this->getState()) // check, if the controller is active
  {
    // Preparing LED message
    kobuki_msgs::LedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::Led());

    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
      ROS_INFO_STREAM("Bumper pressed. Turning LED on. [" << name_ << "]");
      led_msg_ptr->value = kobuki_msgs::Led::GREEN;
      blink_publisher_.publish(led_msg_ptr);
    }
    else // kobuki_msgs::BumperEvent::RELEASED
    {
      ROS_INFO_STREAM("Bumper released. Turning LED off. [" << name_ << "]");
      led_msg_ptr->value = kobuki_msgs::Led::BLACK;
      blink_publisher_.publish(led_msg_ptr);
    }
  }
};

} // namespace kobuki
// %EndTag(FULLTEXT)%
#endif /* BUMP_BLINK_CONTROLLER_HPP_ */
