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
 * @file /kobuki_keyop/src/keyop_core.cpp
 *
 * @brief Creates a node for remote controlling parts of robot_core.
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <ecl/time.hpp>
#include <ecl/exceptions.hpp>
#include <std_srvs/Empty.h>
#include <kobuki_msgs/MotorPower.h>
#include "../include/keyop_core/keyop_core.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace keyop_core
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

/**
 * @brief Default constructor, needs initialisation.
 */
KeyOpCore::KeyOpCore() : last_zero_vel_sent(true), // avoid zero-vel messages from the beginning
                         accept_incoming(true),
                         power_status(false),
                         wait_for_connection_(true),
                         cmd(new geometry_msgs::Twist()),
                         cmd_stamped(new geometry_msgs::TwistStamped()),
                         linear_vel_step(0.1),
                         linear_vel_max(3.4),
                         angular_vel_step(0.02),
                         angular_vel_max(1.2),
                         quit_requested(false),
                         key_file_descriptor(0)
{
  tcgetattr(key_file_descriptor, &original_terminal_state); // get terminal properties
}

KeyOpCore::~KeyOpCore()
{
  tcsetattr(key_file_descriptor, TCSANOW, &original_terminal_state);
}

/**
 * @brief Initialises the node.
 */
bool KeyOpCore::init()
{
  ros::NodeHandle nh("~");

  name = nh.getUnresolvedNamespace();

  /*********************
   ** Parameters
   **********************/
  nh.getParam("linear_vel_step", linear_vel_step);
  nh.getParam("linear_vel_max", linear_vel_max);
  nh.getParam("angular_vel_step", angular_vel_step);
  nh.getParam("angular_vel_max", angular_vel_max);
  nh.getParam("wait_for_connection", wait_for_connection_);

  ROS_INFO_STREAM("KeyOpCore : using linear  vel step [" << linear_vel_step << "].");
  ROS_INFO_STREAM("KeyOpCore : using linear  vel max  [" << linear_vel_max << "].");
  ROS_INFO_STREAM("KeyOpCore : using angular vel step [" << angular_vel_step << "].");
  ROS_INFO_STREAM("KeyOpCore : using angular vel max  [" << angular_vel_max << "].");

  /*********************
   ** Subscribers
   **********************/
  keyinput_subscriber = nh.subscribe("teleop", 1, &KeyOpCore::remoteKeyInputReceived, this);

  /*********************
   ** Publishers
   **********************/
  velocity_publisher_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  motor_power_publisher_ = nh.advertise<kobuki_msgs::MotorPower>("motor_power", 1);

  /*********************
   ** Velocities
   **********************/
  cmd->linear.x = 0.0;
  cmd->linear.y = 0.0;
  cmd->linear.z = 0.0;
  cmd->angular.x = 0.0;
  cmd->angular.y = 0.0;
  cmd->angular.z = 0.0;

  /*********************
   ** Wait for connection
   **********************/
  if (!wait_for_connection_)
  {
    return true;
  }
  ecl::MilliSleep millisleep;
  int count = 0;
  bool connected = false;
  while (!connected)
  {
    if (motor_power_publisher_.getNumSubscribers() > 0)
    {
      connected = true;
      break;
    }
    if (count == 6)
    {
      connected = false;
      break;
    }
    else
    {
      ROS_WARN_STREAM("KeyOp: could not connect, trying again after 500ms...");
      try
      {
        millisleep(500);
      }
      catch (ecl::StandardException e)
      {
        ROS_ERROR_STREAM("Waiting has been interrupted.");
        ROS_DEBUG_STREAM(e.what());
        return false;
      }
      ++count;
    }
  }
  if (!connected)
  {
    ROS_ERROR("KeyOp: could not connect.");
    ROS_ERROR("KeyOp: check remappings for enable/disable topics).");
  }
  else
  {
    kobuki_msgs::MotorPower power_cmd;
    power_cmd.state = kobuki_msgs::MotorPower::ON;
    motor_power_publisher_.publish(power_cmd);
    ROS_INFO("KeyOp: connected.");
    power_status = true;
  }

  // start keyboard input thread
  thread.start(&KeyOpCore::keyboardInputLoop, *this);
  return true;
}

/*****************************************************************************
 ** Implementation [Spin]
 *****************************************************************************/

/**
 * @brief Worker thread loop; sends current velocity command at a fixed rate.
 *
 * It also process ros functions as well as aborting when requested.
 */
void KeyOpCore::spin()
{
  ros::Rate loop_rate(10);

  while (!quit_requested && ros::ok())
  {
    // Avoid spamming robot with continuous zero-velocity messages
    if ((cmd->linear.x  != 0.0) || (cmd->linear.y  != 0.0) || (cmd->linear.z  != 0.0) ||
        (cmd->angular.x != 0.0) || (cmd->angular.y != 0.0) || (cmd->angular.z != 0.0))
    {
      velocity_publisher_.publish(cmd);
      last_zero_vel_sent = false;
    }
    else if (last_zero_vel_sent == false)
    {
      velocity_publisher_.publish(cmd);
      last_zero_vel_sent = true;
    }
    accept_incoming = true;
    ros::spinOnce();
    loop_rate.sleep();
  }
  if (quit_requested)
  { // ros node is still ok, send a disable command
    disable();
  }
  else
  {
    // just in case we got here not via a keyboard quit request
    quit_requested = true;
    thread.cancel();
  }
  thread.join();
}

/*****************************************************************************
 ** Implementation [Keyboard]
 *****************************************************************************/

/**
 * @brief The worker thread function that accepts input keyboard commands.
 *
 * This is ok here - but later it might be a good idea to make a node which
 * posts keyboard events to a topic. Recycle common code if used by many!
 */
void KeyOpCore::keyboardInputLoop()
{
  struct termios raw;
  memcpy(&raw, &original_terminal_state, sizeof(struct termios));

  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(key_file_descriptor, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Forward/back arrows : linear velocity incr/decr.");
  puts("Right/left arrows : angular velocity incr/decr.");
  puts("Spacebar : reset linear/angular velocities.");
  puts("d : disable motors.");
  puts("e : enable motors.");
  puts("q : quit.");
  char c;
  while (!quit_requested)
  {
    if (read(key_file_descriptor, &c, 1) < 0)
    {
      perror("read char failed():");
      exit(-1);
    }
    processKeyboardInput(c);
  }
}

/**
 * @brief Callback function for remote keyboard inputs subscriber.
 */
void KeyOpCore::remoteKeyInputReceived(const kobuki_msgs::KeyboardInput& key)
{
  processKeyboardInput(key.pressedKey);
}

/**
 * @brief Process individual keyboard inputs.
 *
 * @param c keyboard input.
 */
void KeyOpCore::processKeyboardInput(char c)
{
  /*
   * Arrow keys are a bit special, they are escape characters - meaning they
   * trigger a sequence of keycodes. In this case, 'esc-[-Keycode_xxx'. We
   * ignore the esc-[ and just parse the last one. So long as we avoid using
   * the last one for its actual purpose (e.g. left arrow corresponds to
   * esc-[-D) we can keep the parsing simple.
   */
  switch (c)
  {
    case kobuki_msgs::KeyboardInput::KeyCode_Left:
    {
      incrementAngularVelocity();
      break;
    }
    case kobuki_msgs::KeyboardInput::KeyCode_Right:
    {
      decrementAngularVelocity();
      break;
    }
    case kobuki_msgs::KeyboardInput::KeyCode_Up:
    {
      incrementLinearVelocity();
      break;
    }
    case kobuki_msgs::KeyboardInput::KeyCode_Down:
    {
      decrementLinearVelocity();
      break;
    }
    case kobuki_msgs::KeyboardInput::KeyCode_Space:
    {
      resetVelocity();
      break;
    }
    case 'q':
    {
      quit_requested = true;
      break;
    }
    case 'd':
    {
      disable();
      break;
    }
    case 'e':
    {
      enable();
      break;
    }
    default:
    {
      break;
    }
  }
}

/*****************************************************************************
 ** Implementation [Commands]
 *****************************************************************************/
/**
 * @brief Disables commands to the navigation system.
 *
 * This does the following things:
 *
 * - Disables power to the navigation motors (via device_manager).
 * @param msg
 */
void KeyOpCore::disable()
{
  cmd->linear.x = 0.0;
  cmd->angular.z = 0.0;
  velocity_publisher_.publish(cmd);
  accept_incoming = false;

  if (power_status)
  {
    ROS_INFO("KeyOp: die, die, die (disabling power to the device's motor system).");
    kobuki_msgs::MotorPower power_cmd;
    power_cmd.state = kobuki_msgs::MotorPower::OFF;
    motor_power_publisher_.publish(power_cmd);
    power_status = false;
  }
  else
  {
    ROS_WARN("KeyOp: Motor system has already been powered down.");
  }
}

/**
 * @brief Reset/re-enable the navigation system.
 *
 * - resets the command velocities.
 * - reenable power if not enabled.
 */
void KeyOpCore::enable()
{
  accept_incoming = false;

  cmd->linear.x = 0.0;
  cmd->angular.z = 0.0;
  velocity_publisher_.publish(cmd);

  if (!power_status)
  {
    ROS_INFO("KeyOp: Enabling power to the device subsystem.");
    kobuki_msgs::MotorPower power_cmd;
    power_cmd.state = kobuki_msgs::MotorPower::ON;
    motor_power_publisher_.publish(power_cmd);
    power_status = true;
  }
  else
  {
    ROS_WARN("KeyOp: Device has already been powered up.");
  }
}

/**
 * @brief If not already maxxed, increment the command velocities..
 */
void KeyOpCore::incrementLinearVelocity()
{
  if (power_status)
  {
    if (cmd->linear.x <= linear_vel_max)
    {
      cmd->linear.x += linear_vel_step;
    }
    ROS_INFO_STREAM("KeyOp: linear  velocity incremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

/**
 * @brief If not already minned, decrement the linear velocities..
 */
void KeyOpCore::decrementLinearVelocity()
{
  if (power_status)
  {
    if (cmd->linear.x >= -linear_vel_max)
    {
      cmd->linear.x -= linear_vel_step;
    }
    ROS_INFO_STREAM("KeyOp: linear  velocity decremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

/**
 * @brief If not already maxxed, increment the angular velocities..
 */
void KeyOpCore::incrementAngularVelocity()
{
  if (power_status)
  {
    if (cmd->angular.z <= angular_vel_max)
    {
      cmd->angular.z += angular_vel_step;
    }
    ROS_INFO_STREAM("KeyOp: angular velocity incremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

/**
 * @brief If not already mined, decrement the angular velocities..
 */
void KeyOpCore::decrementAngularVelocity()
{
  if (power_status)
  {
    if (cmd->angular.z >= -angular_vel_max)
    {
      cmd->angular.z -= angular_vel_step;
    }
    ROS_INFO_STREAM("KeyOp: angular velocity decremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

void KeyOpCore::resetVelocity()
{
  if (power_status)
  {
    cmd->angular.z = 0.0;
    cmd->linear.x = 0.0;
    ROS_INFO_STREAM("KeyOp: reset linear/angular velocities.");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

} // namespace keyop_core
