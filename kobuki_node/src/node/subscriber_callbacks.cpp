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
 * @file /kobuki_node/src/node/subscriber_callbacks.cpp
 *
 * @brief Subscriber callbacks for kobuki node.
 *
 * @date 14/04/2012
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "../../include/kobuki_node/kobuki_node.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void KobukiNode::subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg)
{
  if (kobuki.isEnabled())
  {
    // For now assuming this is in the robot frame, but probably this
    // should be global frame and require a transform
    //double vx = msg->linear.x;        // in (m/s)
    //double wz = msg->angular.z;       // in (rad/s)
    ROS_DEBUG_STREAM("Kobuki : velocity command received [" << msg->linear.x << "],[" << msg->angular.z << "]");
    kobuki.setBaseControl(msg->linear.x, msg->angular.z);
    odometry.resetTimeout();
  }
  return;
}

  
void KobukiNode::subscribeLed1Command(const kobuki_msgs::LedConstPtr msg)
{
  switch( msg->value ) {
  case kobuki_msgs::Led::GREEN:  kobuki.setLed(Led1, Green ); break;
  case kobuki_msgs::Led::ORANGE: kobuki.setLed(Led1, Orange ); break; 
  case kobuki_msgs::Led::RED:    kobuki.setLed(Led1, Red ); break;
  case kobuki_msgs::Led::BLACK:  kobuki.setLed(Led1, Black ); break;
  default: ROS_WARN_STREAM("Kobuki : led 1 command value invalid."); break;
  }
  return;
}

void KobukiNode::subscribeLed2Command(const kobuki_msgs::LedConstPtr msg)
{
  switch( msg->value ) {
  case kobuki_msgs::Led::GREEN:  kobuki.setLed(Led2, Green ); break;
  case kobuki_msgs::Led::ORANGE: kobuki.setLed(Led2, Orange ); break;
  case kobuki_msgs::Led::RED:    kobuki.setLed(Led2, Red ); break;
  case kobuki_msgs::Led::BLACK:  kobuki.setLed(Led2, Black ); break;
  default: ROS_WARN_STREAM("Kobuki : led 2 command value invalid."); break;
  }
  return;
}

void KobukiNode::subscribeDigitalOutputCommand(const kobuki_msgs::DigitalOutputConstPtr msg)
{
  DigitalOutput digital_output;
  for ( unsigned int i = 0; i < 4; ++i ) {
    digital_output.values[i] = msg->values[i];
    digital_output.mask[i] = msg->mask[i];
  }
  kobuki.setDigitalOutput(digital_output);
  return;
}

void KobukiNode::subscribeExternalPowerCommand(const kobuki_msgs::DigitalOutputConstPtr msg)
{
  DigitalOutput digital_output;
  for ( unsigned int i = 0; i < 4; ++i ) {
    digital_output.values[i] = msg->values[i];
    digital_output.mask[i] = msg->mask[i];
  }
  kobuki.setExternalPower(digital_output);
  return;
}

/**
 * @brief Play a predefined sound (single sound or sound sequence)
 */
void KobukiNode::subscribeSoundCommand(const kobuki_msgs::SoundConstPtr msg)
{
  if ( msg->value == kobuki_msgs::Sound::ON )
  {
    kobuki.playSoundSequence(On);
  }
  else if ( msg->value == kobuki_msgs::Sound::OFF )
  {
    kobuki.playSoundSequence(Off);
  }
  else if ( msg->value == kobuki_msgs::Sound::RECHARGE )
  {
    kobuki.playSoundSequence(Recharge);
  }
  else if ( msg->value == kobuki_msgs::Sound::BUTTON )
  {
    kobuki.playSoundSequence(Button);
  }
  else if ( msg->value == kobuki_msgs::Sound::ERROR )
  {
    kobuki.playSoundSequence(Error);
  }
  else if ( msg->value == kobuki_msgs::Sound::CLEANINGSTART )
  {
    kobuki.playSoundSequence(CleaningStart);
  }
  else if ( msg->value == kobuki_msgs::Sound::CLEANINGEND )
  {
    kobuki.playSoundSequence(CleaningEnd);
  }
  else
  {
    ROS_WARN_STREAM("Kobuki: Invalid sound command! There is no sound stored for value '" << msg->value << "'.");
  }
  return;
}

/**
 * @brief Reset the odometry variables.
 */
void KobukiNode::subscribeResetOdometry(const std_msgs::EmptyConstPtr /* msg */)
{
  ROS_INFO_STREAM("Mobile base : resetting the odometry [" << name << "].");
  joint_states.position[0] = 0.0; // wheel_left
  joint_states.velocity[0] = 0.0;
  joint_states.position[1] = 0.0; // wheel_right
  joint_states.velocity[1] = 0.0;
  odometry.resetOdometry();
  kobuki.resetOdometry();
  return;
}

void KobukiNode::enable(const std_msgs::StringConstPtr msg)
{
  kobuki.enable();
  ROS_INFO_STREAM("Kobuki : enabled.");
  odometry.resetTimeout();
}

void KobukiNode::disable(const std_msgs::StringConstPtr msg)
{
  kobuki.disable();
  ROS_INFO_STREAM("Kobuki : disabled.");
  odometry.resetTimeout();
}

} // namespace kobuki
