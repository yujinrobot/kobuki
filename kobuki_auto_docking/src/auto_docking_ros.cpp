/**
 * @file /auto_docking/src/auto_docking_ros.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Nov 30, 2012
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "kobuki_auto_docking/auto_docking_ros.hpp"

namespace kobuki 
{

AutoDockingROS::AutoDockingROS()
  : shutdown_requested(false)
{;}

AutoDockingROS::~AutoDockingROS()
{
  shutdown_requested = true;
}

bool AutoDockingROS::init(ros::NodeHandle& nh)
{
  core_sensors_ = nh.subscribe("/mobile_base/sensors/core", 10, &AutoDockingROS::coreCb, this);
  dock_ir_ = nh.subscribe("/mobile_base/sensors/dock_ir", 10, &AutoDockingROS::irCb, this);
  odom_ = nh.subscribe("/odom", 10, &AutoDockingROS::odomCb, this);
  //odom_.subscribe("some_topic");
  //ir_.subscribe("some_topic", AutoDockingROS::ir_cb);
  //velocity_commander_.publish("some_topic");
  return dock_.init();
}

/*
 * optional, use if needed
 */
void AutoDockingROS::spin()
{
  //msg.data = dock_.target_direction_;
  //velocity_commander_.publish(msg);
  dock_.auto_dock();
}

void AutoDockingROS::odomCb(const nav_msgs::OdometryPtr msg)
{
  //odom_ = data; 
  ;
  //dock_.update(odom_);
}

void AutoDockingROS::coreCb(const kobuki_msgs::SensorStatePtr msg)
{
  ;
}

void AutoDockingROS::irCb(const kobuki_msgs::DockInfraRedPtr msg)
{
  ;
}

} //namespace kobuki

