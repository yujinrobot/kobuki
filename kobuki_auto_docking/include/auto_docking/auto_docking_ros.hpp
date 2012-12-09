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

namespace kobuki {

class AutoDockingROS
{
public:
  AutoDockingROS();
  ~AutoDockingROS();

  bool init(ros::NodeHandle& nh)
  {
    core_sensors_ = nh.subscribe("/mobile_base/sensors/core", 10, &AutoDockingROS::coreCb, this);
    dock_ir_ = nh.subscribe("/mobile_base/sensors/dock_ir", 10, &AutoDockingROS::irCb, this);
    odom_ = nh.subscribe("/odom", 10, &AutoDockingROS::odomCb, this);
    //odom_.subscribe("some_topic");
    //ir_.subscribe("some_topic", AutoDockingROS::ir_cb);
    //velocity_commander_.publish("some_topic");
    return dock_.init();
  };

  /*
   * optional, use if needed
   */
  void spin()
  {
    //msg.data = dock_.target_direction_;
    //velocity_commander_.publish(msg);
    dock_.auto_dock();
  };

private:
  AutoDocking dock_;
  ros::Subscriber odom_, dock_ir_, core_sensors_;
  ros::Publisher velocity_commander_;

  //ecl::Pose2D odom_;

  void odomCb(const nav_msgs::OdometryPtr msg)
  {
    //std::cout << *msg << std::endl;
    //odom_ = data; 
    ;
    //dock_.update(odom_);
  };

  void coreCb(const kobuki_msgs::SensorStatePtr msg)
  {
    ;
  };

  void irCb(const kobuki_msgs::DockInfraRedPtr msg)
  {
    ;
  };

};

} //namespace kobuki
#endif /* AUTO_DOCKING_ROS_HPP_ */

