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
#include <kobuki_msgs/DockInfraRed.h>
#include "auto_docking.hpp"

class AutoDockingROS
{
public:
  AutoDockingROS();
  ~AutoDockingROS();

  bool init(ros::NodeHandle& nh)
  {
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
  ros::Subscriber odom_, dock_ir_, core_sensor_;
  ros::Publisher velocity_commander_;

  //ecl::Pose2D odom_;

  void odom_callback(const nav_msgs::OdometryPtr &msg)
  {
    //odom_ = data; 
    ;
    //dock_.update(odom_);
  };

  void ir_cb(const kobuki_msgs::DockInfraRedPtr &msg)
  {
    ;
  };

};
#endif /* AUTO_DOCKING_ROS_HPP_ */

