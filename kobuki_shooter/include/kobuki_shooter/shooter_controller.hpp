/*
 * Copyright (c) 2013, Yujin Robot.
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
** Ifdefs
*****************************************************************************/

#ifndef SHOOTER_CONTROLLER_HPP_
#define SHOOTER_CONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <string>
#include <ros/ros.h>
#include <yocs_controllers/default_controller.hpp>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/Sound.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/DigitalInputEvent.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/WheelDropEvent.h>

namespace kobuki
{

/**
 * @ brief Shoot things with Kobuki
 *
 * Use this shooter controller to set Kobuki loose in order to hit anything you want. Connect to buttons to Kobuki's
 * digital inputs to adjust direction and speed for Kobuki's attack.
 */
class ShooterController : public yocs::Controller
{
public:
  ShooterController(ros::NodeHandle& nh, std::string& name) :
    Controller(),
    nh_(nh),
    name_(name),
    wheel_left_dropped_(false),
    wheel_right_dropped_(false),
    wheel_dropped_(false),
    bumper_left_pressed_(false),
    bumper_center_pressed_(false),
    bumper_right_pressed_(false),
    bumper_pressed_(false),
    cliff_left_detected_(false),
    cliff_center_detected_(false),
    cliff_right_detected_(false),
    cliff_detected_(false),
    idle_(true),
    aiming_(false),
    charging_(false),
    shooting_(false),
    blink_freq_(2.0),
    last_blink_time_(0.0),
    led_1_(true),
    turn_left_(true){};

  ~ShooterController(){};

  /**
   * Set-up necessary publishers/subscribers and variables
   * @return true, if successful
   */
  bool init()
  {
    enable_controller_subscriber_ = nh_.subscribe("enable", 10, &ShooterController::enableCB, this);
    disable_controller_subscriber_ = nh_.subscribe("disable", 10, &ShooterController::disableCB, this);
    bumper_event_subscriber_ = nh_.subscribe("events/bumper", 10, &ShooterController::bumperEventCB, this);
    cliff_event_subscriber_  = nh_.subscribe("events/cliff",  10, &ShooterController::cliffEventCB, this);
    wheel_event_subscriber_  = nh_.subscribe("events/wheel_drop", 10, &ShooterController::wheelEventCB, this);
    button_events_subscriber_ = nh_.subscribe("events/digital_input", 10, &ShooterController::buttonEventCB, this);
    reset_safety_states_subscriber_ = nh_.subscribe("reset", 10, &ShooterController::resetSafetyStatesCB, this);
    led_1_command_publisher_ = nh_.advertise<kobuki_msgs::Led>("commands/led1", 1);
    led_2_command_publisher_ = nh_.advertise<kobuki_msgs::Led>("commands/led2", 1);
    sound_command_publisher_ = nh_.advertise<kobuki_msgs::Sound>("commands/sound", 1);
    velocity_command_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    nh_.param("resources_path", resources_path_, std::string("/"));
    if (resources_path_[resources_path_.length() - 1] != '/')
      resources_path_ += "/";

    shooting_cmd_vel_.reset(new geometry_msgs::Twist);
    return true;
  };

  /**
   * @ brief Checks safety states and publishes velocity commands when necessary
   */
  void spinOnce();

private:
  ros::NodeHandle nh_;
  std::string name_;
  std::string resources_path_;
  ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_;
  ros::Subscriber bumper_event_subscriber_, cliff_event_subscriber_, wheel_event_subscriber_;
  ros::Subscriber button_events_subscriber_, reset_safety_states_subscriber_;
  ros::Publisher led_1_command_publisher_, led_2_command_publisher_, sound_command_publisher_;
  ros::Publisher velocity_command_publisher_;
  bool wheel_left_dropped_, wheel_right_dropped_, wheel_dropped_;
  bool bumper_left_pressed_, bumper_center_pressed_, bumper_right_pressed_, bumper_pressed_;
  bool cliff_left_detected_, cliff_center_detected_, cliff_right_detected_, cliff_detected_;
  /**
   * Controller in idle mode, if true
   */
  bool idle_;
  /**
   * Controller in aiming mode, if true
   */
  bool aiming_;
  /**
   * Controller in charging mode, if true
   */
  bool charging_;
  /**
   * Controller in shooting mode, if true
   */
  bool shooting_;

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
   * @brief Keeps track of the wheel drop sensor states
   * @param msg incoming topic message
   */
  void wheelEventCB(const kobuki_msgs::WheelDropEventConstPtr msg);

  /**
   * @brief Keeps track of the button states
   * @param msg incoming topic message
   */
  void buttonEventCB(const kobuki_msgs::DigitalInputEventConstPtr msg);

  /**
   * @brief Callback for resetting all safety variables/states
   *
   * Allows resetting bumper, cliff and wheel drop states.
   *
   * @param msg incoming topic message
   */
  void resetSafetyStatesCB(const std_msgs::EmptyConstPtr msg);

  /**
   * Turns LED 1 & 2 on and off
   */
  void blink();
  double last_blink_time_;
  double blink_freq_;
  bool led_1_;

  /**
   * Play beeps at beep_freq_ rate
   */
  void beep();
  double last_beep_time_;
  double beep_freq_;

  /**
   * Increase Kobuki's shooting speed
   */
  void charge();
  /// Kobuki's shooting speed
  geometry_msgs::TwistPtr shooting_cmd_vel_;

  /**
   * Drives Kobuki at the given shooting velocity
   */
  void shoot();

  /**
   * Turns Kobuki a bit to the left or right
   * @param left if true, then Kobiki turns left, otherwise right
   */
  void turn(bool left);
  /// If true, then Kobuki will turn left, otherwise right
  bool turn_left_;

  /**
   * Drives Kobuki a bit backward, e.g. when bumpers are pressed or a cliff was detected.
   */
  void backOff();

  /**
   * Stops Kobuki.
   */
  void stop();
};


void ShooterController::enableCB(const std_msgs::EmptyConstPtr msg)
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

void ShooterController::disableCB(const std_msgs::EmptyConstPtr msg)
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

void ShooterController::cliffEventCB(const kobuki_msgs::CliffEventConstPtr msg)
{
  if (msg->state == kobuki_msgs::CliffEvent::CLIFF)
  {
    switch (msg->sensor)
    {
      case kobuki_msgs::CliffEvent::LEFT:
      {
        cliff_left_detected_ = true;
        break;
      }
      case kobuki_msgs::CliffEvent::CENTER:
      {
        cliff_center_detected_ = true;
        break;
      }
      case kobuki_msgs::CliffEvent::RIGHT:
      {
        cliff_right_detected_ = true;
        break;
      }
    }
  }
  else // kobuki_msgs::CliffEvent::FLOOR
  {
    switch (msg->sensor)
    {
      case kobuki_msgs::CliffEvent::LEFT:
      {
        cliff_left_detected_   = false;
        break;
      }
      case kobuki_msgs::CliffEvent::CENTER:
      {
        cliff_center_detected_ = false;
        break;
      }
      case kobuki_msgs::CliffEvent::RIGHT:
      {
        cliff_right_detected_  = false;
        break;
      }
    }
  }
  if (cliff_left_detected_ || cliff_center_detected_ || cliff_right_detected_)
  {
    cliff_detected_ = true;
    ROS_DEBUG_STREAM("Cliff detected. [" << name_ << "]");
    if (shooting_)
    {
      shooting_ = false;
      idle_ = true;
      stop();
      shooting_cmd_vel_->linear.x = 0.0;

      system(("rosrun kobuki_shooter play_sound.bash " + resources_path_ + "scary.wav").c_str());
    }
  }
  else
  {
    cliff_detected_ = false;
    ROS_DEBUG_STREAM("No cliff in sight. [" << name_ << "]");
  }
};

void ShooterController::bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg)
{
  if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
  {
    switch (msg->bumper)
    {
      case kobuki_msgs::BumperEvent::LEFT:
      {
        bumper_left_pressed_   = true;
        break;
      }
      case kobuki_msgs::BumperEvent::CENTER:
      {
        bumper_center_pressed_ = true;
        break;
      }
      case kobuki_msgs::BumperEvent::RIGHT:
      {
        bumper_right_pressed_  = true;
        break;
      }
    }
  }
  else // kobuki_msgs::BumperEvent::RELEASED
  {
    switch (msg->bumper)
    {
      case kobuki_msgs::BumperEvent::LEFT:
      {
        bumper_left_pressed_   = false;
        break;
      }
      case kobuki_msgs::BumperEvent::CENTER:
      {
        bumper_center_pressed_ = false;
        break;
      }
      case kobuki_msgs::BumperEvent::RIGHT:
      {
        bumper_right_pressed_  = false;
        break;
      }
    }
  }
  if (bumper_left_pressed_ || bumper_center_pressed_ || bumper_right_pressed_)
  {
    bumper_pressed_ = true;
    ROS_DEBUG_STREAM("One or more bumpers pressed. [" << name_ << "]");
    if (shooting_)
    {
      shooting_ = false;
      idle_ = true;
      stop();
      shooting_cmd_vel_->linear.x = 0.0;

      system(("rosrun kobuki_shooter play_sound.bash " + resources_path_ + "sick.wav").c_str());
    }
  }
  else
  {
    bumper_pressed_ = false;
    ROS_DEBUG_STREAM("All bumpers released. [" << name_ << "]");
  }
};

void ShooterController::wheelEventCB(const kobuki_msgs::WheelDropEventConstPtr msg)
{
  if (msg->state == kobuki_msgs::WheelDropEvent::DROPPED)
  {
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
    if (wheel_left_dropped_ || wheel_right_dropped_)
    {
      wheel_dropped_ = true;
      ROS_DEBUG_STREAM("One or both wheels dropped. " << name_ << "]");
      if (shooting_)
      {
        shooting_ = false;
        idle_ = true;
        stop();
        shooting_cmd_vel_->linear.x = 0.0;

        system(("rosrun kobuki_shooter play_sound.bash " + resources_path_ + "scary.wav").c_str());
      }
    }
    else
    {
      wheel_dropped_ = false;
      ROS_DEBUG_STREAM("Both wheels raised. " << name_ << "]");
    }
  }
};

void ShooterController::buttonEventCB(const kobuki_msgs::DigitalInputEventConstPtr msg)
{
  if (msg->values[0] == false)
  {
    if (idle_)
    {
      if (turn_left_)
      {
        ROS_INFO_STREAM("Aiming left. [" << name_ << "]");
        turn_left_ = false;
      }
      else
      {
        ROS_INFO_STREAM("Aiming right. [" << name_ << "]");
        turn_left_ = true;
      }
      idle_ = false;
      aiming_ = true;
    }
    else if (shooting_) // stop when button is pressed
    {
      shooting_ = false;
      idle_ = true;
      stop();
      shooting_cmd_vel_->linear.x = 0.0;
    }
  }
  else
  {
    if (aiming_)
    {
      stop();
      idle_ = true;
      aiming_ = false;
      ROS_INFO_STREAM("Target locked. [" << name_ << "]");
    }
  }

  if (msg->values[1] == false)
  {
    if (idle_ || aiming_)
    {
      idle_ = false;
      charging_ = true;
      ROS_INFO_STREAM("Charging ... [" << name_ << "]");
    }
    else if (shooting_) // stop when button is pressed
    {
      shooting_ = false;
      idle_ = true;
      stop();
      shooting_cmd_vel_->linear.x = 0.0;
    }
  }
  else
  {
    if (charging_)
    {
      charging_ = false;
      shooting_ = true;
      ROS_WARN_STREAM("Fire! [" << name_ << "]");

      system(("rosrun kobuki_shooter play_sound.bash " + resources_path_ + "watchout.wav").c_str());
    }
  }
};

void ShooterController::resetSafetyStatesCB(const std_msgs::EmptyConstPtr msg)
{
  stop();
  wheel_left_dropped_    = false;
  wheel_right_dropped_   = false;
  bumper_left_pressed_   = false;
  bumper_center_pressed_ = false;
  bumper_right_pressed_  = false;
  cliff_left_detected_   = false;
  cliff_center_detected_ = false;
  cliff_right_detected_  = false;
  wheel_dropped_         = false;
  bumper_pressed_        = false;
  cliff_detected_        = false;
  ROS_WARN_STREAM("All safety states have been reset to false. [" << name_ << "]");
  idle_ = true;
  aiming_ = false;
  charging_ = false;
  shooting_ = false;
  shooting_cmd_vel_->linear.x = 0.0;
  ROS_WARN_STREAM("Shooter controller now in idle mode. [" << name_ << "]");
}

void ShooterController::blink()
{
  double now = ros::Time::now().toSec();
  double delta_t = now - last_blink_time_;
  if (delta_t > (1 / blink_freq_))
  {
    last_blink_time_ = now;
    kobuki_msgs::LedPtr msg1(new kobuki_msgs::Led);
    kobuki_msgs::LedPtr msg2(new kobuki_msgs::Led);
    if (led_1_)
    {
      msg1->value = kobuki_msgs::Led::GREEN;
      led_1_command_publisher_.publish(msg1);
      msg2->value = kobuki_msgs::Led::BLACK;
      led_2_command_publisher_.publish(msg2);
      led_1_ = false;
    }
    else
    {
      msg1->value = kobuki_msgs::Led::BLACK;
      led_1_command_publisher_.publish(msg1);
      msg2->value = kobuki_msgs::Led::GREEN;
      led_2_command_publisher_.publish(msg2);
      led_1_ = true;
    }
  }
  return;
}

void ShooterController::beep()
{
  double now = ros::Time::now().toSec();
  double delta_t = now - last_beep_time_;
  if (delta_t > (1 / beep_freq_))
  {
    last_beep_time_ = now;
    kobuki_msgs::SoundPtr msg(new kobuki_msgs::Sound);
    msg->value = kobuki_msgs::Sound::BUTTON;
    sound_command_publisher_.publish(msg);
  }
  return;
}

void ShooterController::charge()
{
  if (shooting_cmd_vel_->linear.x == 0.0)
  {
    shooting_cmd_vel_->linear.x = 0.01;
  }
  else if (shooting_cmd_vel_->linear.x < 1.0)
  {
    shooting_cmd_vel_->linear.x = shooting_cmd_vel_->linear.x * 1.1;
  }
  ROS_INFO_STREAM("Shooting speed now at " << shooting_cmd_vel_->linear.x << " m/s. [" << name_ << "]");

  // play sound, which reflects charging level
  beep_freq_ = shooting_cmd_vel_->linear.x*20.0 + 0.4;
  beep();

  // TODO: play with LEDs, which reflects charging level
  return;
}

void ShooterController::shoot()
{
  velocity_command_publisher_.publish(shooting_cmd_vel_);
  ROS_WARN_STREAM("Goooooooo! [" << name_ << "]");
  return;
}

void ShooterController::turn(bool left)
{
  msg_.reset(new geometry_msgs::Twist());
  msg_->linear.x = 0.0;
  msg_->linear.y = 0.0;
  msg_->linear.z = 0.0;
  msg_->angular.x = 0.0;
  msg_->angular.y = 0.0;
  if (left)
  {
    msg_->angular.z = 1.0;
  }
  else
  {
    msg_->angular.z = -1.0;
  }
  velocity_command_publisher_.publish(msg_);
  return;
}

void ShooterController::backOff()
{
  msg_.reset(new geometry_msgs::Twist());
  msg_->linear.x = -0.1;
  msg_->linear.y = 0.0;
  msg_->linear.z = 0.0;
  msg_->angular.x = 0.0;
  msg_->angular.y = 0.0;
  msg_->angular.z = 0.0;
  velocity_command_publisher_.publish(msg_);
  return;
}

void ShooterController::stop()
{
  msg_.reset(new geometry_msgs::Twist());
  msg_->linear.x = 0.0;
  msg_->linear.y = 0.0;
  msg_->linear.z = 0.0;
  msg_->angular.x = 0.0;
  msg_->angular.y = 0.0;
  msg_->angular.z = 0.0;
  velocity_command_publisher_.publish(msg_);
  return;
}

void ShooterController::spinOnce()
{
  ROS_DEBUG_STREAM_THROTTLE(1.0, "Current controller state: idle = " << idle_ << ", aiming = " << aiming_
                            << ", charging = " << charging_ << ", shooting = " << shooting_);
  if (this->getState())
  {
    if (idle_)
    {
      blink();
    }
    else if (aiming_)
    {
      if (turn_left_)
      {
        turn(true);
      }
      else
      {
        turn(false);
      }
    }
    else if (charging_)
    {
      charge();
    }
    else if (shooting_)
    {
      shoot();
    }
  }
};

} // namespace kobuki

#endif /* SHOOTER_CONTROLLER_HPP_ */
