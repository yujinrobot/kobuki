/**
 * @file /auto_docking/include/auto_docking/auto_docking_ros.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef AUTO_DOCKING_ROS_HPP_
#define AUTO_DOCKING_ROS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <kobuki_msgs/AutoDockingAction.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/DockInfraRed.h>

#include <sstream>
#include <vector>
#include <ecl/geometry/legacy_pose2d.hpp>
#include <ecl/linear_algebra.hpp>
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <kobuki_dock_drive/dock_drive.hpp>

namespace kobuki
{

typedef message_filters::sync_policies::ApproximateTime<
  nav_msgs::Odometry,
  kobuki_msgs::SensorState,
  kobuki_msgs::DockInfraRed
> SyncPolicy;

class AutoDockingROS
{
public:
//  AutoDockingROS();
  AutoDockingROS(std::string name);
//  AutoDockingROS(ros::NodeHandle& nh);
//  AutoDockingROS(ros::NodeHandle& nh, std::string name);
  ~AutoDockingROS();

  bool init(ros::NodeHandle& nh);
  void spin();

private:
  AutoDockingROS* self;
  DockDrive dock_;

  std::string name_;
  bool shutdown_requested_;

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kobuki_msgs::AutoDockingAction> as_;

  kobuki_msgs::AutoDockingGoal goal_;
  kobuki_msgs::AutoDockingFeedback feedback_;
  kobuki_msgs::AutoDockingResult result_;

  ros::Subscriber debug_;
  ros::Publisher velocity_commander_, motor_power_enabler_, debug_jabber_;

  boost::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry> > odom_sub_;
  boost::shared_ptr<message_filters::Subscriber<kobuki_msgs::DockInfraRed> > ir_sub_;
  boost::shared_ptr<message_filters::Subscriber<kobuki_msgs::SensorState> > core_sub_;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;

  void goalCb();
  void preemptCb();

  void syncCb(const nav_msgs::OdometryConstPtr& odom,
              const kobuki_msgs::SensorStateConstPtr& core,
              const kobuki_msgs::DockInfraRedConstPtr& ir);
  void debugCb(const std_msgs::StringConstPtr& msg);
};

} //namespace kobuki
#endif /* AUTO_DOCKING_ROS_HPP_ */

