/**
 * @file /kobuki_node/src/node/main.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 10/04/2012
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include "../../include/kobuki_node/kobuki_node.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kobuki");
  ros::NodeHandle nh_private("~");
  std::string node_name = ros::this_node::getName();
  kobuki::KobukiNode kobuki_node(node_name);
  if (kobuki_node.init(nh_private))
  {
    kobuki_node.spin(nh_private);
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't initialise kobuki_node.");
  }
  return(0);
}
