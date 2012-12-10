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
  : shutdown_requested_(false)
{;}

AutoDockingROS::~AutoDockingROS()
{
  shutdown_requested_ = true;
}

bool AutoDockingROS::init(ros::NodeHandle& nh)
{
  debug_jabber_ = nh.advertise<std_msgs::String>("/dock_drive/debug", 10);
  motor_power_enabler_ = nh.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 10);
  velocity_commander_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  do_dock_ = nh.subscribe("/dock_drive/commands/do_dock", 10, &AutoDockingROS::doCb, this);
  cancel_dock_ = nh.subscribe("/dock_drive/commands/cancel_dock", 10, &AutoDockingROS::cancelCb, this);
  debug_ = nh.subscribe("/dock_drive/commands/debug", 10, &AutoDockingROS::debugCb, this);

  odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/odom", 10));
  core_sub_.reset(new message_filters::Subscriber<kobuki_msgs::SensorState>(nh, "/mobile_base/sensors/core", 10));
  ir_sub_.reset(new message_filters::Subscriber<kobuki_msgs::DockInfraRed>(nh, "/mobile_base/sensors/dock_ir", 10));
  sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *odom_sub_, *core_sub_, *ir_sub_));
  sync_->registerCallback(boost::bind(&AutoDockingROS::syncCb, *this, _1, _2, _3));

  return dock_.init();
}

void AutoDockingROS::spin()
{
  return;

  while(!shutdown_requested_){;}
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


  //update
  //dock_.update(odom, ir, core);


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
}

void AutoDockingROS::doCb(const std_msgs::StringConstPtr& msg)
{
  dock_.enable(msg->data);

  kobuki_msgs::MotorPowerPtr power_cmd(new kobuki_msgs::MotorPower);
  power_cmd->state = kobuki_msgs::MotorPower::ON;
  motor_power_enabler_.publish(power_cmd);
}

void AutoDockingROS::cancelCb(const std_msgs::StringConstPtr& msg)
{
  dock_.disable(msg->data);

  kobuki_msgs::MotorPowerPtr power_cmd(new kobuki_msgs::MotorPower);
  power_cmd->state = kobuki_msgs::MotorPower::OFF;
  motor_power_enabler_.publish(power_cmd);
}

void AutoDockingROS::debugCb(const std_msgs::StringConstPtr& msg)
{
  dock_.enable(msg->data);
}

} //namespace kobuki
