#ifndef CONTROLLER_APP_HPP_
#define CONTROLLER_APP_HPP_

#include <ros/ros.h>

#include <std_msgs/Empty.h> 		
#include <std_msgs/String.h>

#include <nav_msgs/Odometry.h>		
#include <geometry_msgs/Twist.h>


#include <ecl/Eigen3/Geometry>
#include <ecl/threads/thread.hpp>
#include <ecl/geometry.hpp>

#include <remote_mutex/RemoteMutex.h>

//#include <robot_core/mobile_base/reset_odometry.h>

namespace controller_app {

enum State
{

  FORWARD,
  BACKWARD,
  TURNRIGHT,
  TURNLEFT,
  STOP,
  IDLE

};


class ControllerApp
{
public:
	ControllerApp();
	~ControllerApp();

	void init();
	void spin();
	
private:
	
	bool isArrive();
	void setGoalPose(float dGoalPoseX, float dGoalPoseY, float dGoalPoseThetaRad);
	State getState();

	void forward();
	void backward();
	void turnLeft();
	void turnRight();
	void stop();

	
	bool m_bIsActivity;
	bool m_bIsSetGoalPose;
	
	float m_dRobotPoseThetaRad;
	float m_dRobotPoseX;
	float m_dRobotPoseY;

	float m_dGoalPoseThetaRad;
	float m_dGoalPoseX;
	float m_dGoalPoseY;

	geometry_msgs::Twist cmd_vel;
	std_msgs::String power_cmd;

	RemoteMutex* mutex_iclebo;
	ros::Subscriber sub_crtl_come_to;
	ros::Subscriber sub_crtl_go_to;
	ros::Subscriber sub_odom;

	ros::Publisher pub_complete;
	ros::Publisher pub_cmd_vel;
	ros::Publisher pub_power_enable;
	ros::Publisher pub_power_disable;

	ros::ServiceClient client_reset;
	
	void callCb( const std_msgs::EmptyConstPtr& msg );
	void dismissCb( const std_msgs::EmptyConstPtr& msg );
	void odomCb( const nav_msgs::OdometryConstPtr& msg );

	

};

} //namespace pano_app

#endif /* PANO_APP_HPP_ */
