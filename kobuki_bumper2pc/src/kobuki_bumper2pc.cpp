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

void Bumper2PcNodelet::coreSensorCB(const kobuki_msgs::SensorState::ConstPtr& msg)
{
  if (pointcloud_pub_.getNumSubscribers() == 0)
    return;

  // We publish just one "no events" pc and stop spamming when bumper/cliff conditions disappear
  if (! msg->bumper && ! msg->cliff && ! prev_bumper && ! prev_cliff)
    return;

  prev_bumper = msg->bumper;
  prev_cliff  = msg->cliff;

  // We replicate the sensors order of bumper/cliff event messages: LEFT = 0, CENTER = 1 and RIGHT = 2
  if ((msg->bumper & kobuki_msgs::SensorState::BUMPER_LEFT) ||
      (msg->cliff  & kobuki_msgs::SensorState::CLIFF_LEFT))
  {
    pointcloud_[0].x = + pointcloud_side_x_;
    pointcloud_[0].y = + pointcloud_side_y_;
  }
  else
  {
    pointcloud_[0].x = + FAR_AWAY;
    pointcloud_[0].y = + FAR_AWAY;
  }

  if ((msg->bumper & kobuki_msgs::SensorState::BUMPER_CENTRE) ||
      (msg->cliff  & kobuki_msgs::SensorState::CLIFF_CENTRE))
  {
    pointcloud_[1].x = + pointcloud_radius_;
  }
  else
  {
    pointcloud_[1].x = + FAR_AWAY;
  }

  if ((msg->bumper & kobuki_msgs::SensorState::BUMPER_RIGHT) ||
      (msg->cliff  & kobuki_msgs::SensorState::CLIFF_RIGHT))
  {
    pointcloud_[2].x = + pointcloud_side_x_;
    pointcloud_[2].y = - pointcloud_side_y_;
  }
  else
  {
    pointcloud_[2].x = + FAR_AWAY;
    pointcloud_[2].y = - FAR_AWAY;
  }

  pointcloud_.header.stamp = msg->header.stamp;
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
  pointcloud_[0].z = pointcloud_[1].z = pointcloud_[2].z = 0.04;  // z: elevation from base frame

  pointcloud_pub_  = nh.advertise < pcl::PointCloud <pcl::PointXYZ> > ("pointcloud", 10);
  core_sensor_sub_ = nh.subscribe("core_sensors", 10, &Bumper2PcNodelet::coreSensorCB, this);

  ROS_INFO("Bumper/cliff pointcloud configured at distance %f from base frame", pointcloud_radius_);
}

} // namespace kobuki_bumper2pc


PLUGINLIB_EXPORT_CLASS(kobuki_bumper2pc::Bumper2PcNodelet, nodelet::Nodelet);
