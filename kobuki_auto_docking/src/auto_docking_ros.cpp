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
  debug_jabber_ = nh.advertise<std_msgs::String>("/dock_drive/debug", 10);
  motor_power_enabler_ = nh.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 10);
  velocity_commander_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  odom_ = nh.subscribe("/odom", 10, &AutoDockingROS::odomCb, this);
  core_sensors_ = nh.subscribe("/mobile_base/sensors/core", 10, &AutoDockingROS::coreCb, this);
  dock_ir_ = nh.subscribe("/mobile_base/sensors/dock_ir", 10, &AutoDockingROS::irCb, this);

  //core_sub_.subscribe(nh_, "/mobile_base/sensors/core", 10);
  //ir_sub_.subscribe(nh_, "/mobile_base/sensors/dock_ir", 10);
  //sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), core_sub_, ir_sub_));
  odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/odom", 10));
  core_sub_.reset(new message_filters::Subscriber<kobuki_msgs::SensorState>(nh, "/mobile_base/sensors/core", 10));
  ir_sub_.reset(new message_filters::Subscriber<kobuki_msgs::DockInfraRed>(nh, "/mobile_base/sensors/dock_ir", 10));
  sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *odom_sub_, *core_sub_, *ir_sub_));
  sync_->registerCallback(boost::bind(&AutoDockingROS::syncCb, *this, _1, _2, _3));

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
  return;

  while(!shutdown_requested){;}

  //msg.data = dock_.target_direction_;
  //velocity_commander_.publish(msg);
  //dock_.auto_dock();
}

void AutoDockingROS::syncCb(const nav_msgs::OdometryConstPtr& odom,
                            const kobuki_msgs::SensorStateConstPtr& core,
                            const kobuki_msgs::DockInfraRedConstPtr& ir)
{
  static int n = 0;
  ROS_INFO_STREAM("syncCb: " << __func__ << "(" << n++ << ")");
  std::cout << "---1---" << std::endl << odom << std::endl;
  std::cout << "---2---" << std::endl << core << std::endl;
  std::cout << "---3---" << std::endl << ir << std::endl;

  //
  geometry_msgs::TwistPtr msg(new geometry_msgs::Twist);
  msg->linear.x = 0.0;
  msg->angular.z = 0.0;
  velocity_commander_.publish(msg);

  //
  std::ostringstream oss;
  oss << "blah blah blah";
  std_msgs::StringPtr debug_log(new std_msgs::String);
  debug_log->data = oss.str();
  debug_jabber_.publish(debug_log);
  return;

//  dock_.update(odom, ir, core);
}
void AutoDockingROS::odomCb(const nav_msgs::OdometryPtr msg)
{
  static int n = 0;
  ROS_INFO_STREAM("odomCb: " << __func__ << "(" << n++ << ")");
  //odom_ = data; 
  //dock_.update(odom_);
}

void AutoDockingROS::coreCb(const kobuki_msgs::SensorStatePtr msg)
{
  static int n = 0;
  ROS_INFO_STREAM("coreCb: " << __func__ << "(" << n++ << ")");
}

void AutoDockingROS::irCb(const kobuki_msgs::DockInfraRedPtr msg)
{
  static int n = 0;
  ROS_INFO_STREAM("irCb: " << __func__ << "(" << n++ << ")");
}

} //namespace kobuki

