/**
 * @file /auto_docking/include/auto_docking/auto_docking_ros.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Nov 30, 2012
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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/DockInfraRed.h>
#include <ecl/geometry/pose2d.hpp>
#include "auto_docking.hpp"

namespace kobuki 
{

typedef message_filters::sync_policies::ApproximateTime<kobuki_msgs::SensorState, kobuki_msgs::DockInfraRed> SyncPolicy;

class AutoDockingROS
{
public:
  AutoDockingROS();
  ~AutoDockingROS();

  bool init(ros::NodeHandle& nh);

  /*
   * optional, use if needed
   */
  void spin();

private:
  bool shutdown_requested;
  ros::NodeHandle nh_;
  AutoDocking dock_;
  ros::Subscriber odom_, dock_ir_, core_sensors_, do_dock_, cancel_dock_, debug_;
  ros::Publisher velocity_commander_, motor_power_enabler_;

//  message_filters::Subscriber<kobuki_msgs::DockInfraRed> ir_sub_;
//  message_filters::Subscriber<kobuki_msgs::SensorState> core_sub_;
  boost::shared_ptr<message_filters::Subscriber<kobuki_msgs::DockInfraRed> > ir_sub_;
  boost::shared_ptr<message_filters::Subscriber<kobuki_msgs::SensorState> > core_sub_;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
  //boost::shared_ptr<message_filters::TimeSynchronizer<kobuki_msgs::SensorState, kobuki_msgs::DockInfraRed> > sync_;

  //ecl::Pose2D odom_;
  void odomCb(const nav_msgs::OdometryPtr msg);
  void coreCb(const kobuki_msgs::SensorStatePtr msg);
  void irCb(const kobuki_msgs::DockInfraRedPtr msg);

  void doCb(const std_msgs::StringPtr msg);
  void cancelCb(const std_msgs::StringPtr msg);
  void debugCb(const std_msgs::StringPtr msg);

  void syncCb(const kobuki_msgs::SensorStateConstPtr& core, const kobuki_msgs::DockInfraRedConstPtr& ir);
};

} //namespace kobuki
#endif /* AUTO_DOCKING_ROS_HPP_ */

