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
#include "auto_docking.hpp"

class AutoDockingROS
{
public:
  AutoDockingROS(){};
  ~AutoDockingROS(){};

  void init()
  {
    //odom_.subscribe("some_topic");
    //ir_.subscribe("some_topic", AutoDockingROS::ir_cb);
    //velocity_commander_.publish("some_topic");
  };

  /*
   * optional, use if needed
   */
  void spin()
  {
    msg.data = dock_.target_direction_;
    velocity_commander_.publish(msg);
    dock_.auto_dock();
  };

private:
  AutoDocking dock_;
  ros::Subscriber odom_, ir_;
  ros::Publisher velocity_commander_;

  magic_odom_variable odom_;

  void odom_callback(data)
  {
    odom_ = data;
  };

  void ir_cb(data)
  {
    if (data == x)
    {
      dock_.ir_left_ = true;
    }
  };

};
#endif /* AUTO_DOCKING_ROS_HPP_ */

