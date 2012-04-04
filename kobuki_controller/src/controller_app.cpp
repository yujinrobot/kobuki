#include "../include/kobukibot_controller/controller_app.hpp"

namespace controller_app {

ControllerApp::ControllerApp()
{

}

ControllerApp::~ControllerApp()
{

}

void ControllerApp::init() 
{
	ros::NodeHandle nh;

	sub_crtl_go_to = nh.subscribe("/call", 1, &ControllerApp::callCb, this);
	sub_crtl_come_to = nh.subscribe("/dismiss", 1, &ControllerApp::dismissCb, this);
	sub_odom = nh.subscribe("/robot_core/odom_combined", 100, &ControllerApp::odomCb, this);

	pub_cmd_vel = nh.advertise<geometry_msgs::Twist>( "/robot_core/cmd_vel", 100 );
	pub_power_enable = nh.advertise<std_msgs::String>( "/robot_core/enable", 10 );
	pub_power_disable = nh.advertise<std_msgs::String>( "/robot_core/disable", 10 );

	// define mutex;
	mutex_iclebo = new RemoteMutex( "iclebo" );

	//client_reset = nh.service<::reset_odometry>

	m_bIsActivity = false;
	m_bIsSetGoalPose = false;

	m_dRobotPoseThetaRad = 0.0;
	m_dRobotPoseX= 0.0;
	m_dRobotPoseY= 0.0;
	
	m_dGoalPoseThetaRad= 0.0;
	m_dGoalPoseX= 1.0;
	m_dGoalPoseY= 0.0;

	cmd_vel.linear.x = 0.0f;
	cmd_vel.linear.y = 0.0f;
	cmd_vel.linear.z = 0.0f;
	cmd_vel.angular.x = 0.0f;
	cmd_vel.angular.y = 0.0f;
	cmd_vel.angular.z = 0.0f; 
	
	power_cmd.data = "/robot_core/iclebo_base";

	ROS_INFO("Controller Application initialized.");
}

void ControllerApp::spin()
{
	ros::Rate loop_rate(10);
	//stop();
	while(ros::ok()){	
		if(m_bIsActivity){	
			switch(getState() ){
				case FORWARD: forward(); break;
				case BACKWARD: backward(); break;
				case TURNLEFT: turnLeft(); break;
				case TURNRIGHT: turnRight(); break;

				case STOP: stop(); break;
			
				case IDLE: 
 					stop();
					m_bIsActivity =false;
					//RemoteMutex mutex_iclebo(std::string("iclebo"));
					pub_power_disable.publish( power_cmd );
					while( !mutex_iclebo->unlock() ) {
						ROS_WARN_THROTTLE(30,"Failed to unlock mutex of iclebo platform");
					}	

					break;
				default:  
					break;
			}

			/*
			float dDiffDist = hypot(m_dGoalPoseX-m_dRobotPoseX, m_dGoalPoseY-m_dRobotPoseY );
			float dDiffTheta = atan2(m_dGoalPoseY-m_dRobotPoseY, m_dGoalPoseX-m_dRobotPoseX) - m_dRobotPoseThetaRad;
			//ROS_INFO("X: %.3f	Y: %.3f	ThetaDeg: %.3f", m_dRobotPoseX, m_dRobotPoseY, ecl::radians_to_degrees(m_dRobotPoseThetaRad));
			//ROS_INFO("dDiffDist: %.3f	dDiffTheta: %.3f", dDiffDist,  ecl::radians_to_degrees(dDiffTheta));
			
			if (dDiffTheta > ecl::degrees_to_radians(3.0)){
				turnRight();
			}
			else if (dDiffTheta < ecl::degrees_to_radians(-3.0)){
				turnLeft();
			}
			else if(dDiffDist > 0.2){
				forward();
			}
			else{
				stop();
				m_bIsActivity =false;
				//RemoteMutex mutex_iclebo(std::string("iclebo"));
				pub_power_disable.publish( power_cmd );
				while( !mutex_iclebo->unlock() ) {
					ROS_WARN_THROTTLE(30,"Failed to unlock mutex of iclebo platform");
				}	
				ROS_INFO("arrive goal");
			}
			*/
		}
		ros::spinOnce();
		loop_rate.sleep();

	}
	pub_power_disable.publish( power_cmd );
}

State ControllerApp::getState()
{
	State nCurState = STOP;

	float dDiffDist = hypot(m_dGoalPoseX-m_dRobotPoseX, m_dGoalPoseY-m_dRobotPoseY );
	float dDiffTheta = atan2(m_dGoalPoseY-m_dRobotPoseY, m_dGoalPoseX-m_dRobotPoseX) - m_dRobotPoseThetaRad;

	if(dDiffTheta > 3.141592){
		dDiffTheta -= 2*3.141592;	
	}
	else if(dDiffTheta < -3.141592){
		dDiffTheta += 2*3.141592;		
	}
	

//	ROS_INFO("X: %.3f	Y: %.3f	ThetaDeg: %.3f", m_dRobotPoseX, m_dRobotPoseY, ecl::radians_to_degrees(m_dRobotPoseThetaRad));
//	ROS_INFO("dDiffDist: %.3f	dDiffTheta: %.3f", dDiffDist,  ecl::radians_to_degrees(dDiffTheta));

	if(isArrive()){
		nCurState = IDLE;
		ROS_INFO("arrive goal");
	}	
	else if (dDiffTheta > ecl::degrees_to_radians(3.0)){
		nCurState = TURNRIGHT;
	}
	else if (dDiffTheta < ecl::degrees_to_radians(-3.0)){
		nCurState = TURNLEFT;
	}
	else if(dDiffDist > 0.1){
		nCurState = FORWARD;
	}
	else{
		nCurState = STOP;
	}
	return nCurState;
}


bool  ControllerApp::isArrive()
{
	float dDiffDist = hypot(m_dGoalPoseX-m_dRobotPoseX, m_dGoalPoseY-m_dRobotPoseY );

	if(dDiffDist < 0.1){
		return true;
	}
	return false;
}

void ControllerApp::setGoalPose(float dGoalPoseX, float dGoalPoseY, float dGoalPoseThetaRad)
{
	m_dGoalPoseThetaRad = dGoalPoseThetaRad;
	m_dGoalPoseX = dGoalPoseX;
	m_dGoalPoseY = dGoalPoseY;
	m_bIsSetGoalPose = true;
}

void ControllerApp::forward()
{
	cmd_vel.linear.x = 0.0f;
	cmd_vel.linear.y = 0.0f;
	cmd_vel.linear.z = 0.0f;
	cmd_vel.angular.x = 0.0f;
	cmd_vel.angular.y = 0.0f;
	cmd_vel.angular.z = 0.0f; 

	ROS_INFO("forward");
	cmd_vel.linear.x = 0.2; 
	pub_cmd_vel.publish( cmd_vel );
}
void ControllerApp::backward()
{
	cmd_vel.linear.x = 0.0f;
	cmd_vel.linear.y = 0.0f;
	cmd_vel.linear.z = 0.0f;
	cmd_vel.angular.x = 0.0f;
	cmd_vel.angular.y = 0.0f;
	cmd_vel.angular.z = 0.0f; 

	ROS_INFO("backward");
	cmd_vel.linear.x = -0.2; 
	pub_cmd_vel.publish( cmd_vel );


}
void ControllerApp::turnLeft()
{
	cmd_vel.linear.x = 0.0f;
	cmd_vel.linear.y = 0.0f;
	cmd_vel.linear.z = 0.0f;
	cmd_vel.angular.x = 0.0f;
	cmd_vel.angular.y = 0.0f;
	cmd_vel.angular.z = 0.0f; 

	ROS_INFO("turnLeft");
	cmd_vel.angular.z = -0.33f; 
	pub_cmd_vel.publish( cmd_vel );
}
void ControllerApp::turnRight()
{
	cmd_vel.linear.x = 0.0f;
	cmd_vel.linear.y = 0.0f;
	cmd_vel.linear.z = 0.0f;
	cmd_vel.angular.x = 0.0f;
	cmd_vel.angular.y = 0.0f;
	cmd_vel.angular.z = 0.0f; 

	ROS_INFO("turnRight");
	cmd_vel.angular.z = 0.33f; 
	pub_cmd_vel.publish( cmd_vel );
}
void ControllerApp::stop()
{
	ROS_INFO("turnRight");
	cmd_vel.linear.x = 0.0f;
	cmd_vel.linear.y = 0.0f;
	cmd_vel.linear.z = 0.0f;
	cmd_vel.angular.x = 0.0f;
	cmd_vel.angular.y = 0.0f;
	cmd_vel.angular.z = 0.0f; 
	pub_cmd_vel.publish( cmd_vel );
}

//callback function

void ControllerApp::odomCb( const nav_msgs::OdometryConstPtr& msg )
{
	//ROS_INFO("Call odomCb");
	//static double heading_last = 0.0f;
	double heading = 0.0f;
	Eigen::AngleAxisf angle_axis(Eigen::Quaternionf(
		msg->pose.pose.orientation.w, 
		msg->pose.pose.orientation.x, 
		msg->pose.pose.orientation.y, 
		msg->pose.pose.orientation.z
	));
	Eigen::Vector3f axis = angle_axis.axis();

	if( axis(2) > 0.0 ) heading = angle_axis.angle();
	else if( axis(2) < 0.0 ) heading = -1.0 * angle_axis.angle();

	//angle += fabs(ecl::wrap_angle(heading - heading_last));
	//heading_last = heading;
	m_dRobotPoseThetaRad = heading;
	m_dRobotPoseX = msg->pose.pose.position.x;
	m_dRobotPoseY = msg->pose.pose.position.y;
}




void ControllerApp::callCb( const std_msgs::EmptyConstPtr& msg )
{
	if(m_bIsSetGoalPose){
		ROS_INFO("set Goal");
		m_bIsSetGoalPose = false;
	}	
	else{
		ROS_INFO("no set Goal");
		m_dGoalPoseThetaRad= 0.0;
		m_dGoalPoseX= 1.0;
		m_dGoalPoseY= 0.0;
	}
	ROS_INFO("Call callCb");
  //RemoteMutex mutex_iclebo(std::string("iclebo"));
	while( !mutex_iclebo->lock(ros::Duration(30.0)) ) { //if locked
		ROS_WARN_THROTTLE(30,"Failed to lock mutex of iclebo platform");
	}
	pub_power_enable.publish( power_cmd );
	m_bIsActivity = true;
}

void ControllerApp::dismissCb( const std_msgs::EmptyConstPtr& msg )
{
	
	setGoalPose(0,0,0);

	if(m_bIsSetGoalPose){
		ROS_INFO("set Goal");
		m_bIsSetGoalPose = false;
	}	
	else{
		ROS_INFO("no set Goal");
		m_dGoalPoseThetaRad= 0.0;
		m_dGoalPoseX= 1.0;
		m_dGoalPoseY= 0.0;
	}

	ROS_INFO("Call dismissCb");

	while( !mutex_iclebo->lock(ros::Duration(30.0)) ) { //if locked
		ROS_WARN_THROTTLE(30,"Failed to lock mutex of iclebo platform");
	}
	pub_power_enable.publish( power_cmd );
	m_bIsActivity = true;
}


} //namespace controller_app
