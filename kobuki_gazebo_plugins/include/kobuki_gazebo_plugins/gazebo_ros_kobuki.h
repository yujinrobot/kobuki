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
 *
 * This work is based on the Gazebo ROS plugin for the iRobot Create by Nate Koenig.
 */

#ifndef GAZEBO_GAZEBO_ROS_KOBUKI_H__
#define GAZEBO_GAZEBO_ROS_KOBUKI_H__

#include <string>
#include <boost/thread.hpp>
//#include <boost/bind.hpp> // used for what?

#include <gazebo/Controller.hh>
#include <gazebo/Model.hh>
#include <gazebo/Geom.hh>
#include <gazebo/Time.hh>
#include <gazebo/RaySensor.hh>

#include <ros/ros.h>
//#include <geometry_msgs/TwistWithCovariance.h> // used for what?
//#include <geometry_msgs/PoseWithCovariance.h> // used for what?
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>


namespace gazebo
{
class GazeboRosKobuki : public Controller
{
public:
  GazeboRosKobuki(gazebo::Entity *parent);
  virtual ~GazeboRosKobuki();

  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void FiniChild();
  virtual void UpdateChild();

private:

  void UpdateSensors();
  void OnContact(const gazebo::Contact &contact);
  void OnCmdVel(const geometry_msgs::TwistConstPtr &msg);

  ros::NodeHandle *rosnode_;
  //ros::Service operating_mode_srv_;
  //ros::Service digital_output_srv_;

  ros::Publisher sensor_state_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher joint_state_pub_;

  ros::Subscriber cmd_vel_sub_;

  ParamT<std::string> *node_namespaceP_;
  ParamT<std::string> *left_wheel_joint_nameP_;
  ParamT<std::string> *right_wheel_joint_nameP_;
  ParamT<std::string> *front_castor_joint_nameP_;
  ParamT<std::string> *rear_castor_joint_nameP_;
  ParamT<std::string> *base_geom_nameP_;

  /// Separation between the wheels
  ParamT<float> *wheel_sepP_;

  /// Diameter of the wheels
  ParamT<float> *wheel_diamP_;

  ///Torque applied to the wheels
  ParamT<float> *torqueP_;

  Model *my_parent_;

  /// Speeds of the wheels
  float *wheel_speed_;

  // Simulation time of the last update
  Time prev_update_time_;
  Time last_cmd_vel_time_;

  float odom_pose_[3];
  float odom_vel_[3];

  bool set_joints_[4];
  Joint *joints_[4];
  Geom *base_geom_;

  RaySensor *left_cliff_sensor_;
  RaySensor *leftfront_cliff_sensor_;
  RaySensor *rightfront_cliff_sensor_;
  RaySensor *right_cliff_sensor_;
  RaySensor *wall_sensor_;

  tf::TransformBroadcaster transform_broadcaster_;
  sensor_msgs::JointState js_;

  turtlebot_node::TurtlebotSensorState sensor_state_;

  void spin();
  boost::thread *spinner_thread_;
};
}

#endif /* GAZEBO_GAZEBO_ROS_KOBUKI_H__ */
