/**
 * @file /auto_docking/src/auto_docking_ros.cpp
 *
 * @brief File comment
 *
 * File comment
 *
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
  if (as_.isActive()) {
    result_.text = "Aborted: Shutdown requested.";
    as_.setAborted( result_, result_.text );
  }
  dock_.disable();
}

bool AutoDockingROS::init(ros::NodeHandle& nh)
{
  // Configure docking drive
  double min_abs_v, min_abs_w;
  if (nh.getParam("min_abs_v", min_abs_v) == true)
    dock_.setMinAbsV(min_abs_v);

  if (nh.getParam("min_abs_w", min_abs_w) == true)
    dock_.setMinAbsW(min_abs_w);

  // Publishers and subscribers
  velocity_commander_ = nh.advertise<geometry_msgs::Twist>("velocity", 10);
  debug_jabber_ = nh.advertise<std_msgs::String>("debug/feedback", 10);

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
  if (dock_.isEnabled()) {
    goal_ = *(as_.acceptNewGoal());
    result_.text = "Rejected: dock_drive is already enabled.";
    as_.setAborted( result_, result_.text );
    ROS_INFO_STREAM("[" << name_ << "] New goal received but rejected.");
  } else {
    dock_.enable();
    goal_ = *(as_.acceptNewGoal());
    ROS_INFO_STREAM("[" << name_ << "] New goal received and accepted.");
  }
}

void AutoDockingROS::preemptCb()
{
  //ROS_DEBUG_STREAM("[" << name_ << "] Preempt requested.");
  dock_.disable();
  if (as_.isNewGoalAvailable()) {
    result_.text = "Preempted: New goal received.";
    as_.setPreempted( result_, result_.text );
    ROS_INFO_STREAM("[" << name_ << "] " << result_.text );
  } else {
    result_.text = "Cancelled: Cancel requested.";
    as_.setPreempted( result_, result_.text );
    ROS_INFO_STREAM("[" << name_ << "] " << result_.text );
    dock_.disable();
  }
}

void AutoDockingROS::syncCb(const nav_msgs::OdometryConstPtr& odom,
                            const kobuki_msgs::SensorStateConstPtr& core,
                            const kobuki_msgs::DockInfraRedConstPtr& ir)
{
  //process and run
  if(self->dock_.isEnabled()) {
    //conversions
    KDL::Rotation rot;
    tf::quaternionMsgToKDL( odom->pose.pose.orientation, rot );

    double r, p, y;
    rot.GetRPY(r, p, y);

    ecl::LegacyPose2D<double> pose;
    pose.x(odom->pose.pose.position.x);
    pose.y(odom->pose.pose.position.y);
    pose.heading(y);

    //update
    self->dock_.update(ir->data, core->bumper, core->charger, pose);

    //publish debug stream
    std_msgs::StringPtr debug_log(new std_msgs::String);
    debug_log->data = self->dock_.getDebugStream();
    debug_jabber_.publish(debug_log);

    //publish command velocity
    if (self->dock_.canRun()) {
      geometry_msgs::TwistPtr cmd_vel(new geometry_msgs::Twist);
      cmd_vel->linear.x = self->dock_.getVX();
      cmd_vel->angular.z = self->dock_.getWZ();
      velocity_commander_.publish(cmd_vel);
    }
  }

  //action server execution
  if( as_.isActive() ) {
    if ( dock_.getState() == RobotDockingState::DONE ) {
      result_.text = "Arrived on docking station successfully.";
      as_.setSucceeded(result_);
      ROS_INFO_STREAM( "[" << name_ << "]: Arrived on docking station successfully.");
      ROS_DEBUG_STREAM( "[" << name_ << "]: Result sent.");
      dock_.disable();
    } else if ( !dock_.isEnabled() ) { //Action Server is activated, but DockDrive is not enabled, or disabled unexpectedly
      ROS_ERROR_STREAM("[" << name_ << "] Unintended Case: ActionService is active, but DockDrive is not enabled..");
      result_.text = "Aborted: dock_drive is disabled unexpectedly.";
      as_.setAborted( result_, "Aborted: dock_drive is disabled unexpectedly." );
      ROS_INFO_STREAM("[" << name_ << "] Goal aborted.");
      dock_.disable();
    } else {
      feedback_.state = dock_.getStateStr();
      feedback_.text = dock_.getDebugStr();
      as_.publishFeedback(feedback_);
      ROS_DEBUG_STREAM( "[" << name_ << "]: Feedback sent.");
    }
  }
  return;
}

void AutoDockingROS::debugCb(const std_msgs::StringConstPtr& msg)
{
  dock_.modeShift(msg->data);
}


} //namespace kobuki
