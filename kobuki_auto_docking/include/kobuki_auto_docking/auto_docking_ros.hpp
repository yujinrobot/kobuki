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
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/DockInfraRed.h>

#include <ecl/geometry/pose2d.hpp>
#include "auto_docking.hpp"

namespace kobuki 
{

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
  AutoDocking dock_;
  ros::Subscriber odom_, dock_ir_, core_sensors_;
  ros::Publisher velocity_commander_;

  //ecl::Pose2D odom_;
  void odomCb(const nav_msgs::OdometryPtr msg);
  void coreCb(const kobuki_msgs::SensorStatePtr msg);
  void irCb(const kobuki_msgs::DockInfraRedPtr msg);

};

} //namespace kobuki
#endif /* AUTO_DOCKING_ROS_HPP_ */

