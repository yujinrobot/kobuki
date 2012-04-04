#include "../include/kobukibot_photo/photo_app.hpp"

namespace photo_app {

PhotoApp::PhotoApp()
{

}

PhotoApp::~PhotoApp()
{

}

void PhotoApp::init() 
{
	ros::NodeHandle nh(ros::this_node::getName());
	image_transport::ImageTransport it(nh);

	sub_odom = nh.subscribe("/robot_core/odom_combined", 100, &PhotoApp::odomCb, this);
  
	pub_cmd_vel = nh.advertise<geometry_msgs::Twist>( "/robot_core/cmd_vel", 100 );
	pub_power_enable = nh.advertise<std_msgs::String>( "/robot_core/enable", 10 );
	pub_power_disable = nh.advertise<std_msgs::String>( "/robot_core/disable", 10 );
	
	pub_tilt_command = nh.advertise<std_msgs::Float64>( "/robot_core/tilt_controller/command", 10 );

	sub_take_photo = nh.subscribe("/take_photo", 1, &PhotoApp::directNearestPoint, this);
	sub_set_path = nh.subscribe("/set_path", 1, &PhotoApp::setPath, this);
	sub_track_nearest = nh.subscribe("/track", 1, &PhotoApp::trackNearestPoint, this);
	sub_direct_nearest = nh.subscribe("/direct", 1, &PhotoApp::directNearestPoint, this);

	sub_get_image = it.subscribe("/robot_core/camera/rgb/image_color", 1, &PhotoApp::savePhoto, this);
	sub_get_depth_image = it.subscribe("/robot_core/camera/depth/image", 1, &PhotoApp::findNearestPoint, this);
	pub_photo = it.advertise("/photo", 1);

	m_nState = IDLE;
	m_nFrameNum = 0;
	
	min_ = 1.0;
	min_i_ = 0;
	min_j_ = 0;
	rows_ = 0;
	cols_ = 0;
	find_j_ = 0;
	
	mutex_iclebo = new RemoteMutex( "iclebo" );
	power_cmd.data = "/robot_core/iclebo_base";
	
	cmd_vel.linear.x = 0.0f;
	cmd_vel.linear.y = 0.0f;
	cmd_vel.linear.z = 0.0f;
	cmd_vel.angular.x = 0.0f;
	cmd_vel.angular.y = 0.0f;
	cmd_vel.angular.z = 0.0f; 
	
	ROS_INFO("Photo Application initialized.");
}

void PhotoApp::spin()
{
//	cvNamedWindow("view");
	ros::Rate loop_rate(10);
	while( ros::ok() ) {
		if(m_nState == IDLE){
			std_msgs::Float64 msg;
			msg.data = -0.07;
			pub_tilt_command.publish(msg);
		} else if(m_nState == SNAP) {
			std_msgs::Float64 msg;
			msg.data = -0.8;
			pub_tilt_command.publish(msg);
		} else if(m_nState == TRACK) {
			int i = min_i_;
			int j = min_j_;
			int rows = rows_;
			int cols = cols_;
			double z = 0.0f;
			if( cols/2 > j-5 && cols/2 < j+5 ) {
				z = 0.0;
				m_nState = STOP;
			}
			if( cols/2 < j-5 ) z =  0.165;
			if( cols/2 > j+5 ) z = -0.165;
			
			std::cout << "i: " << i << " j: " << j << " z: " << z << std::endl;
			cmd_vel.angular.z = z;
			pub_cmd_vel.publish( cmd_vel );
		} else if(m_nState == DIRECT) {
			int i = min_i_;
			int j = min_j_;
			int cols = cols_;
			double z = 0.0f;

			 if( angle <= ecl::degrees_to_radians( std::fabs( target_z_ ) ) ) {
			    //last_angle = angle;
			    if( target_z_ < 0.0f ) z = 0.165;
			    else z = -0.165; 
			    std::cout << "we have to control." << std::endl;
			    std::cout << "angle: " << angle << " last_angle: " << last_angle <<  " target_z_: " << target_z_ << std::endl;
			    
			} else {
				std::cout << "arrived." << std::endl;
			    std::cout << "angle: " << angle << " last_angle: " << last_angle <<  " target_z_: " << target_z_ << std::endl;
				z = 0.0;
				
				std_msgs::Float64 msg;
				msg.data = -0.8;
				pub_tilt_command.publish(msg);
		
				m_nState = STOP;
			}
			
			    
			/*
			if( cols/2 > find_j_-5 && cols/2 < find_j_+5 ) {
				z = 0.0;
				m_nState = STOP;
			}
			if( cols/2 < find_j_-5 ) z =  0.165;
			if( cols/2 > find_j_+5 ) z = -0.165;
			*/
			std::cout << "i: " << i << " j: " << j << " z: " << z << std::endl;
			
			
			cmd_vel.angular.z = z;
			pub_cmd_vel.publish( cmd_vel );
			
		} else if(m_nState == STOP) {
			pub_power_disable.publish( power_cmd );
			while( !mutex_iclebo->unlock() ) {
				ROS_WARN_THROTTLE(30,"Failed to unlock mutex of iclebo platform");
			}
			ros::Duration(3).sleep();
			m_nState = SNAP;
    	}
	  ros::spinOnce();
	  loop_rate.sleep();
	}
}
void PhotoApp::takePhoto( const std_msgs::EmptyConstPtr& msg )
{
	m_nState = SNAP;	
}
void PhotoApp::setPath( const std_msgs::EmptyConstPtr& msg )
{
}
void PhotoApp::savePhoto( const sensor_msgs::ImageConstPtr& msg )
{
	if(m_nState == SNAP ){
			
		char cData[256];
		sensor_msgs::CvBridge bridge;
		sprintf(cData,"Frame%04d.jpg",m_nFrameNum++);
		cvSaveImage(cData, bridge.imgMsgToCv(msg, "bgr8"));
		ROS_INFO("Photo snap %s.", cData);

		pub_photo.publish( msg );
		m_nState = IDLE;
	}
	else if(m_nState == IDLE){
	}
}

void PhotoApp::trackNearestPoint( const std_msgs::EmptyConstPtr& msg )
{
	//powering up iclebo.
	//RemoteMutex mutex_iclebo(std::string("iclebo"));
	while( !mutex_iclebo->lock(ros::Duration(30.0)) ) { //if locked
		ROS_WARN_THROTTLE(30,"Failed to lock mutex of iclebo platform");
	}
	pub_power_enable.publish( power_cmd );
	
	m_nState = TRACK;
	return;
}

void PhotoApp::directNearestPoint( const std_msgs::EmptyConstPtr& msg )
{
	//powering up iclebo.
	//RemoteMutex mutex_iclebo(std::string("iclebo"));
	while( !mutex_iclebo->lock(ros::Duration(30.0)) ) { //if locked
		ROS_WARN_THROTTLE(30,"Failed to lock mutex of iclebo platform");
	}
	pub_power_enable.publish( power_cmd );
	
//	find_j_ = min_j_;
	
	angle = 0.0f;
	last_angle = 0.0f;
	if( min_j_ == 0 || min_j_ == 639 ) target_z_ = 0.0;
	else target_z_ = ( (double)min_j_ - (double)cols_/2 ) / (double)cols_ * 48.0;
//	ecl::degrees_to_radians(48.0);
	std::cout << "min_j_ is " << min_j_ << std::endl;
	std::cout << "cols_ is " << cols_ << std::endl;
	std::cout << "target_z is " << target_z_ << std::endl;
	m_nState = DIRECT;
	
	return;
}

void PhotoApp::findNearestPoint( const sensor_msgs::ImageConstPtr& msg )
{
	if( msg->encoding.find("F") != std::string::npos )
	{
	    cv::Mat float_image_bridge = img_bridge_.imgMsgToCv(msg, "passthrough");
		cv::Mat_<float> float_image = float_image_bridge;
		float min_val = 1.0;
		int min_i = 0;
		int min_j = 0;
		for(int i = float_image.rows*0.1; i < float_image.rows*0.6; ++i)
		{
			for(int j = 0; j < float_image.cols; ++j)
			{
				if( min_val > float_image(i, j) ) { min_val = float_image(i, j); min_i = i; min_j = j; }
			}
		}
		//std::cout << "min_val : " << min_val << std::endl;
		min_ = min_val;
		min_i_ = min_i;
		min_j_ = min_j;
		
		rows_ = float_image.rows;
		cols_ = float_image.cols;
	}

	return;
}

void PhotoApp::odomCb( const nav_msgs::OdometryConstPtr& msg )
{
  static double heading_last = 0.0f;
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

  angle += fabs(ecl::wrap_angle(heading - heading_last));
  heading_last = heading;
  
  //std::cout << "." ;
}


} //namespace photo_app
