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

/**
 * @file /kobuki_driver/src/tool/simple_keyop.cpp
 *
 * @brief Tools/utility program to control by keyboard.
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <csignal>
#include <termios.h> // for keyboard input
#include <ecl/time.hpp>
#include <ecl/threads.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/exceptions.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/geometry/pose2d.hpp>
#include "kobuki_driver/kobuki.hpp"

/*****************************************************************************
** Classes
*****************************************************************************/

/**
 * @brief Keyboard remote control for our robot core (mobile base).
 *
 */
class KobukiManager
{
public:
  /*********************
   ** C&D
   **********************/
  KobukiManager();
  ~KobukiManager();
  bool init();

  /*********************
   ** Runtime
   **********************/
  void spin();

  /*********************
   ** Callbacks
   **********************/
  void processStreamData();

  /*********************
   ** Accessor
   **********************/
  ecl::Pose2D<double> getPose();

private:
  double vx, wz;
  ecl::Pose2D<double> pose;
  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;

  double linear_vel_step, linear_vel_max;
  double angular_vel_step, angular_vel_max;
  std::string name;

  /*********************
   ** Commands
   **********************/
  void incrementLinearVelocity();
  void decrementLinearVelocity();
  void incrementAngularVelocity();
  void decrementAngularVelocity();
  void resetVelocity();

  /*********************
   ** Keylogging
   **********************/

  void keyboardInputLoop();
  void processKeyboardInput(char c);
  void restoreTerminal();
  bool quit_requested;
  int key_file_descriptor;
  struct termios original_terminal_state;
  ecl::Thread thread;
};

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

/**
 * @brief Default constructor, needs initialisation.
 */
KobukiManager::KobukiManager() :
                         linear_vel_step(0.05),
                         linear_vel_max(1.0),
                         angular_vel_step(0.33),
                         angular_vel_max(6.6),
                         quit_requested(false),
                         key_file_descriptor(0),
                         vx(0.0), wz(0.0),
                         slot_stream_data(&KobukiManager::processStreamData, *this)
{
  tcgetattr(key_file_descriptor, &original_terminal_state); // get terminal properties
}

KobukiManager::~KobukiManager()
{
  kobuki.setBaseControl(0,0); // linear_velocity, angular_velocity in (m/s), (rad/s)
  kobuki.disable();
  tcsetattr(key_file_descriptor, TCSANOW, &original_terminal_state);
}

/**
 * @brief Initialises the node.
 */
bool KobukiManager::init()
{
  /*********************
   ** Parameters
   **********************/
  std::cout << "KobukiManager : using linear  vel step [" << linear_vel_step << "]." << std::endl;
  std::cout << "KobukiManager : using linear  vel max  [" << linear_vel_max << "]." << std::endl;
  std::cout << "KobukiManager : using angular vel step [" << angular_vel_step << "]." << std::endl;
  std::cout << "KobukiManager : using angular vel max  [" << angular_vel_max << "]." << std::endl;

  /*********************
   ** Velocities
   **********************/
  vx = 0.0;
  wz = 0.0;

  /*********************
   ** Kobuki
   **********************/
  kobuki::Parameters parameters;
  parameters.sigslots_namespace = "/kobuki";
  parameters.device_port = "/dev/kobuki";
  parameters.enable_acceleration_limiter = true;

  kobuki.init(parameters);
  kobuki.enable();
  slot_stream_data.connect("/kobuki/stream_data");

  /*********************
   ** Wait for connection
   **********************/
  thread.start(&KobukiManager::keyboardInputLoop, *this);
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
void KobukiManager::spin()
{
/*
  {
    // just in case we got here not via a keyboard quit request
    quit_requested = true;
    thread.cancel();
  }
*/
  ecl::Sleep sleep(0.1);
  while (!quit_requested){
    sleep();
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
void KobukiManager::keyboardInputLoop()
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
 * @brief Process individual keyboard inputs.
 *
 * @param c keyboard input.
 */
void KobukiManager::processKeyboardInput(char c)
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
    case 68://kobuki_msgs::KeyboardInput::KeyCode_Left:
    {
      incrementAngularVelocity();
      break;
    }
    case 67://kobuki_msgs::KeyboardInput::KeyCode_Right:
    {
      decrementAngularVelocity();
      break;
    }
    case 65://kobuki_msgs::KeyboardInput::KeyCode_Up:
    {
      incrementLinearVelocity();
      break;
    }
    case 66://kobuki_msgs::KeyboardInput::KeyCode_Down:
    {
      decrementLinearVelocity();
      break;
    }
    case 32://kobuki_msgs::KeyboardInput::KeyCode_Space:
    {
      resetVelocity();
      break;
    }
    case 'q':
    {
      quit_requested = true;
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
 * @brief If not already maxxed, increment the command velocities..
 */
void KobukiManager::incrementLinearVelocity()
{
  if (vx <= linear_vel_max)
  {
    vx += linear_vel_step;
  }
//  ROS_INFO_STREAM("KeyOp: linear  velocity incremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
}

/**
 * @brief If not already minned, decrement the linear velocities..
 */
void KobukiManager::decrementLinearVelocity()
{
  if (vx >= -linear_vel_max)
  {
    vx -= linear_vel_step;
  }
//  ROS_INFO_STREAM("KeyOp: linear  velocity decremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
}

/**
 * @brief If not already maxxed, increment the angular velocities..
 */
void KobukiManager::incrementAngularVelocity()
{
  if (wz <= angular_vel_max)
  {
    wz += angular_vel_step;
  }
//  ROS_INFO_STREAM("KeyOp: angular velocity incremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
}

/**
 * @brief If not already mined, decrement the angular velocities..
 */
void KobukiManager::decrementAngularVelocity()
{
  if (wz >= -angular_vel_max)
  {
    wz -= angular_vel_step;
  }
//    ROS_INFO_STREAM("KeyOp: angular velocity decremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
}

void KobukiManager::resetVelocity()
{
  vx = 0.0;
  wz = 0.0;
//    ROS_INFO_STREAM("KeyOp: reset linear/angular velocities.");
}

void KobukiManager::processStreamData() {
  ecl::Pose2D<double> pose_update;
  ecl::linear_algebra::Vector3d pose_update_rates;
  kobuki.updateOdometry(pose_update, pose_update_rates);
  pose *= pose_update;
//  dx += pose_update.x();
//  dth += pose_update.heading();
  //std::cout << dx << ", " << dth << std::endl;
  //std::cout << kobuki.getHeading() << ", " << pose.heading() << std::endl;
  //std::cout << "[" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;

  kobuki.setBaseControl(vx, wz);
}

ecl::Pose2D<double> KobukiManager::getPose() {
  return pose;
}

/*****************************************************************************
** Signal Handler
*****************************************************************************/

bool shutdown_req = false;
void signalHandler(int signum) {
  shutdown_req = true;
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  signal(SIGINT, signalHandler);

  std::cout << "Simple Keyop : Utility for driving kobuki by keyboard." << std::endl;
  KobukiManager kobuki_manager;
  kobuki_manager.init();

  ecl::Sleep sleep(1);
  ecl::Pose2D<double> pose;
  try {
    while (!shutdown_req){
      sleep();
      pose = kobuki_manager.getPose();
      std::cout << "current pose: [" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
    }
  } catch ( ecl::StandardException &e ) {
    std::cout << e.what();
  }
  return 0;
}
