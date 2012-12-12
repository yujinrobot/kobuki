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

//AutoDockingROS::AutoDockingROS()
AutoDockingROS::AutoDockingROS(std::string name)
//AutoDockingROS::AutoDockingROS(ros::NodeHandle& nh)
//AutoDockingROS::AutoDockingROS(ros::NodeHandle& nh, std::string name)
  : name_(name)
  , shutdown_requested_(false)
  , as_(nh_, name_+"_action", false)
{
  self = this;

  as_.registerGoalCallback(boost::bind(&AutoDockingROS::goalCb, this));
  as_.registerPreemptCallback(boost::bind(&AutoDockingROS::preemptCb, this));
  as_.start();
}

AutoDockingROS::~AutoDockingROS()
{
  shutdown_requested_ = true;
}

bool AutoDockingROS::init(ros::NodeHandle& nh)
{
  motor_power_enabler_ = nh.advertise<kobuki_msgs::MotorPower>("motor_power", 10);
  velocity_commander_ = nh.advertise<geometry_msgs::Twist>("velocity", 10);
  debug_jabber_ = nh.advertise<std_msgs::String>("debug/feedback", 10);

  do_dock_ = nh.subscribe("commands/do_dock", 10, &AutoDockingROS::doCb, this);
  cancel_dock_ = nh.subscribe("commands/cancel_dock", 10, &AutoDockingROS::cancelCb, this);
  debug_ = nh.subscribe("debug/mode_shift", 10, &AutoDockingROS::debugCb, this);

  odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "odom", 10));
  core_sub_.reset(new message_filters::Subscriber<kobuki_msgs::SensorState>(nh, "core", 10));
  ir_sub_.reset(new message_filters::Subscriber<kobuki_msgs::DockInfraRed>(nh, "dock_ir", 10));
  sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *odom_sub_, *core_sub_, *ir_sub_));
  sync_->registerCallback(boost::bind(&AutoDockingROS::syncCb, this, _1, _2, _3));

  return dock_.init();
}

void AutoDockingROS::spin()
{
  return;

  while(!shutdown_requested_){;}
}

void AutoDockingROS::goalCb()
{
  if (dock_.isEnabled() || as_.isActive()) {
    goal_ = *(as_.acceptNewGoal());
    result_.text = "Rejected: dock_drive is already enabled.";
    as_.setAborted( result_, "Rejected: dock_drive is already enabled." );
    ROS_INFO_STREAM("[" << name_ << "] Goal received but rejected.");
  } else {
    goal_ = *(as_.acceptNewGoal());
    toggleMotor(true);
    dock_.enable();
    ROS_INFO_STREAM("[" << name_ << "] Goal received and accepted.");
  }
}

void AutoDockingROS::preemptCb()
{
  ROS_INFO_STREAM("[" << name_ << "] Preempt.");
  as_.setPreempted();
}

void AutoDockingROS::syncCb(const nav_msgs::OdometryConstPtr& odom,
                            const kobuki_msgs::SensorStateConstPtr& core,
                            const kobuki_msgs::DockInfraRedConstPtr& ir)
{
  //conversions
  KDL::Rotation rot;
  tf::quaternionMsgToKDL( odom->pose.pose.orientation, rot );

  double r, p, y;
  rot.GetRPY(r, p, y);

  ecl::Pose2D<double> pose;
  pose.x(odom->pose.pose.position.x);
  pose.y(odom->pose.pose.position.y);
  pose.heading(y);

  //process and run
  if (self->dock_.isEnabled()) {
    //update
    self->dock_.update(ir->data, core->bumper, core->charger, pose);

    //debug stream
    std_msgs::StringPtr debug_log(new std_msgs::String);
    debug_log->data = self->dock_.getDebugStream();
    debug_jabber_.publish(debug_log);

    //publish velocity
    if (self->dock_.canRun()) {
      geometry_msgs::TwistPtr cmd_vel(new geometry_msgs::Twist);
      cmd_vel->linear.x = self->dock_.getVX();
      cmd_vel->angular.z = self->dock_.getWZ();
      velocity_commander_.publish(cmd_vel);
    }
  }


  //action server mock up
  if( as_.isActive() ) {
    if ( dock_.getState() == dock_.DONE ) {
        result_.text = "Arrived on docking station successfully.";
        as_.setSucceeded(result_);
        ROS_INFO_STREAM( "[" << name_ << "]: Result sent. Arrived on docking station successfully.");
    } else {
      feedback_.state = dock_.getStateStr();
      feedback_.text = dock_.getDebugStr();
      as_.publishFeedback(feedback_);
      ROS_INFO_STREAM( "[" << name_ << "]: Feedback sent.");
    }
  }
  return;
}

void AutoDockingROS::doCb(const std_msgs::EmptyConstPtr& msg)
{
  toggleMotor(true);
  dock_.enable();
  ROS_DEBUG_STREAM("dock_drive : auto docking enabled.");
}

void AutoDockingROS::cancelCb(const std_msgs::EmptyConstPtr& msg)
{
  toggleMotor(false);
  dock_.disable();
  ROS_DEBUG_STREAM("dock_drive : auto docking disabled.");
}

void AutoDockingROS::debugCb(const std_msgs::StringConstPtr& msg)
{
  dock_.modeShift(msg->data);
}

void AutoDockingROS::toggleMotor(const bool& on_off)
{
  kobuki_msgs::MotorPowerPtr power_cmd(new kobuki_msgs::MotorPower);
  if (on_off) power_cmd->state = kobuki_msgs::MotorPower::ON;
  else        power_cmd->state = kobuki_msgs::MotorPower::OFF;
  motor_power_enabler_.publish(power_cmd);
}

} //namespace kobuki
