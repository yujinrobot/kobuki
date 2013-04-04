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
 * @file /include/kobuki_bumper2pc/kobuki_bumper2pc.hpp
 *
 * @brief Bumper/cliff to pointcloud nodelet class declaration.
 *
 * Publish bumpers and cliff sensors events as points in a pointcloud, so navistack can use them
 * for poor-man navigation. Implemented as a nodelet intended to run together with kobuki_node.
 *
 * @author Jorge Santos, Yujin Robot
 *
 **/

#ifndef _KOBUKI_BUMPER2PC_HPP_
#define _KOBUKI_BUMPER2PC_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <kobuki_msgs/SensorState.h>

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace kobuki_bumper2pc
{

/**
 * @brief Bumper2PcNodelet class declaration
 */
class Bumper2PcNodelet : public nodelet::Nodelet
{
public:
  Bumper2PcNodelet() : FAR_AWAY(100), prev_bumper(0), prev_cliff(0) { }
  ~Bumper2PcNodelet() { }

  virtual void onInit();

private:
  const double FAR_AWAY;  // somewhere out of reach from the robot

  uint8_t prev_bumper;
  uint8_t prev_cliff;

  double pointcloud_radius_;
  double pointcloud_side_x_;
  double pointcloud_side_y_;

  ros::Publisher  pointcloud_pub_;

  ros::Subscriber core_sensor_sub_;

  pcl::PointCloud<pcl::PointXYZ> pointcloud_;

  /**
   * @brief Core sensors state structure callback
   * @param msg incoming topic message
   */
  void coreSensorCB(const kobuki_msgs::SensorState::ConstPtr& msg);
};

} // namespace kobuki_bumper2pc

#endif // _KOBUKI_BUMPER2PC_HPP_
