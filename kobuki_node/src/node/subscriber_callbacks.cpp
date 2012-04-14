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

namespace kobuki {

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
    ROS_DEBUG_STREAM("subscribeVelocityCommand: [" << msg->linear.x << "],[" << msg->angular.z << "]");
    kobuki.setBaseControlCommand(msg->linear.x, msg->angular.z);

    last_cmd_time = ros::Time::now();
  }
  return;
}

void KobukiNode::subscribeLedCommand(const kobuki_comms::LedArrayConstPtr msg)
{
  if ( msg->values.size() != 2 ) {
    ROS_WARN_STREAM("Kobuki : led commands must specify values for both led's in the array.");
    return;
  }
  for ( unsigned int i = 0; i < msg->values.size(); ++i ) {
    LedNumber led = Led1;
    if ( i == 1 ) { led = Led2; }
    if ( msg->values[i] == kobuki_comms::LedArray::GREEN ) {
      kobuki.toggleLed(led, Green);
    } else if ( msg->values[i] == kobuki_comms::LedArray::ORANGE ) {
      kobuki.toggleLed(led, Orange);
    } else if ( msg->values[i] == kobuki_comms::LedArray::RED ) {
      kobuki.toggleLed(led, Red);
    } else {
      kobuki.toggleLed(led, Black);
    }
  }
  return;
}

/**
 * @brief Reset the odometry variables.
 */
void KobukiNode::subscribeResetOdometry(const std_msgs::EmptyConstPtr /* msg */)
{
  ROS_INFO_STREAM("Mobile base : resetting the odometry [" << name << "].");
  pose.setIdentity();
  joint_states.position[0] = 0.0; // wheel_left
  joint_states.velocity[0] = 0.0;
  joint_states.position[1] = 0.0; // wheel_right
  joint_states.velocity[1] = 0.0;
  kobuki.resetOdometry();
  return;
}

void KobukiNode::enable(const std_msgs::StringConstPtr msg)
{
  kobuki.enable();
  ROS_INFO_STREAM("Kobuki : enabled.");
  last_cmd_time.fromSec(0);
}

void KobukiNode::disable(const std_msgs::StringConstPtr msg)
{
  kobuki.disable();
  ROS_INFO_STREAM("Kobuki : disabled.");
  last_cmd_time.fromSec(0);
}


} // namespace kobuki
