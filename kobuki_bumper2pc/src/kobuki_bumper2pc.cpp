/**
 * @file /src/kobuki_bumper2pc.cpp
 *
 * @brief Bumper to pointcloud nodelet class implementation.
 *
 * @author Jorge Santos, Yujin Robot
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <pluginlib/class_list_macros.h>

#include "../include/kobuki_bumper2pc/kobuki_bumper2pc.hpp"

namespace kobuki_bumper2pc
{

void Bumper2PcNodelet::cliffEventCB(const kobuki_msgs::CliffEvent::ConstPtr& msg)
{
  if (pointcloud_pub_.getNumSubscribers() == 0)
    return;

  // We replicate the sensors order of bumper/cliff event messages: LEFT = 0, CENTER = 1 and RIGHT = 2
  switch (msg->sensor)
  {
    case kobuki_msgs::CliffEvent::LEFT:
      pointcloud_[msg->sensor].x = (msg->state == kobuki_msgs::CliffEvent::CLIFF)? +pointcloud_side_x_: 99;
      pointcloud_[msg->sensor].y = (msg->state == kobuki_msgs::CliffEvent::CLIFF)? +pointcloud_side_y_: 99;
      break;
    case kobuki_msgs::CliffEvent::CENTER:       // y-coordinate is always 0
      pointcloud_[msg->sensor].x = (msg->state == kobuki_msgs::CliffEvent::CLIFF)? +pointcloud_radius_: 99;
      break;
    case kobuki_msgs::CliffEvent::RIGHT:
      pointcloud_[msg->sensor].x = (msg->state == kobuki_msgs::CliffEvent::CLIFF)? +pointcloud_side_x_: 99;
      pointcloud_[msg->sensor].y = (msg->state == kobuki_msgs::CliffEvent::CLIFF)? -pointcloud_side_y_: 99;
      break;
    default:
      // This cannot happen unless CliffEvent message changes
      ROS_WARN("Unknown sensor id (%d); ignoring", msg->sensor);
      break;
  }

  pointcloud_pub_.publish(pointcloud_);
}

void Bumper2PcNodelet::bumperEventCB(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  if (pointcloud_pub_.getNumSubscribers() == 0)
    return;

  // We replicate the sensors order of bumper/cliff event messages: LEFT = 0, CENTER = 1 and RIGHT = 2
  switch (msg->bumper)
  {
    case kobuki_msgs::BumperEvent::LEFT:
      pointcloud_[msg->bumper].x = (msg->state == kobuki_msgs::BumperEvent::PRESSED)? +pointcloud_side_x_: 99;
      pointcloud_[msg->bumper].y = (msg->state == kobuki_msgs::BumperEvent::PRESSED)? +pointcloud_side_y_: 99;
      break;
    case kobuki_msgs::BumperEvent::CENTER:       // y-coordinate is always 0
      pointcloud_[msg->bumper].x = (msg->state == kobuki_msgs::BumperEvent::PRESSED)? +pointcloud_radius_: 99;
      break;
    case kobuki_msgs::BumperEvent::RIGHT:
      pointcloud_[msg->bumper].x = (msg->state == kobuki_msgs::BumperEvent::PRESSED)? +pointcloud_side_x_: 99;
      pointcloud_[msg->bumper].y = (msg->state == kobuki_msgs::BumperEvent::PRESSED)? -pointcloud_side_y_: 99;
      break;
    default:
      // This cannot happen unless BumperEvent message changes
      ROS_WARN("Unknown sensor id (%d); ignoring", msg->bumper);
      break;
  }

  pointcloud_pub_.publish(pointcloud_);
}

void Bumper2PcNodelet::onInit()
{
  ros::NodeHandle nh = this->getPrivateNodeHandle();

  // Bumper/cliff pointcloud distance to base frame; should be something like the robot radius plus
  // costmap resolution plus an extra to cope with robot inertia. This is a bit tricky parameter: if
  // it's too low, costmap will ignore this pointcloud (the robot footprint runs over the hit obstacle),
  // but if it's too big, hit obstacles will be mapped too far from the robot and the navigation around
  // them will probably fail.
  nh.param("pointcloud_radius", pointcloud_radius_, 0.25);
  pointcloud_side_x_ = pointcloud_radius_*sin(0.34906585); // 20 degrees
  pointcloud_side_y_ = pointcloud_radius_*cos(0.34906585);

  pointcloud_.resize(3);
  pointcloud_.header.frame_id = "/base_link";

  // bumper/cliff "points" fix coordinates (the others depend on sensor activation/deactivation)
  pointcloud_[0].x = pointcloud_[1].y = pointcloud_[2].x = 0.0;   // +π/2, 0 and -π/2 from x-axis
  pointcloud_[0].z = pointcloud_[1].z = pointcloud_[2].z = 0.015; // z: elevation from base frame

  pointcloud_pub_  = nh.advertise < pcl::PointCloud <pcl::PointXYZ> > ("pointcloud", 10);

  cliff_event_sub_  = nh.subscribe("cliff_events",  10, &Bumper2PcNodelet::cliffEventCB,  this);
  bumper_event_sub_ = nh.subscribe("bumper_events", 10, &Bumper2PcNodelet::bumperEventCB, this);

  ROS_INFO("Bumper/cliff pointcloud configured at distance %f from base frame", pointcloud_radius_);
}

} // namespace kobuki_bumper2pc


PLUGINLIB_EXPORT_CLASS(kobuki_bumper2pc::Bumper2PcNodelet, nodelet::Nodelet);
